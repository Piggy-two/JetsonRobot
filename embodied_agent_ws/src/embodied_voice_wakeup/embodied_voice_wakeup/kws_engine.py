#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地唤醒词引擎 —— **不依赖 ROS，可脱离节点单独证伪**。

为什么要有这一层
----------------
`wakeup_node.py` 里剩下的是"开麦克风、喂数据、发话题"。真正会出错的是**音频的
换算**与**关键词的装载**，这两件事都可以在没有硬件、没有 ROS 的情况下钉死，
所以它们在**这里**，并有单测（这个仓库一贯的分法：`safety_gate` / `scan_query` /
`motion_plan` 都是这么切的）。

为什么是软件唤醒而不是用硬件那个
--------------------------------
2026-10-08 实测（`DEV_NOTES` 坑 33 / `#25`）：厂商环形麦的**硬件唤醒上报不可用** ——
断电重启后说 3 遍唤醒词，串口 **0 字节**；握手包 4 种 RTS/DTR 组合也全无应答。
录音/播放一直是好的（D-022 验过），所以**采集通路可用、上报通路不可用**。
⇒ 唤醒改在 Jetson 上做：**用已验证可用的采集通路 + 本地 KWS 模型**。
好处是这条路**我们完全控制得住**：可回归、可量召回率、不依赖模块状态。

采样率换算为什么要自己写
------------------------
环形麦是 **48 kHz 双声道**，而 KWS 模型要 **16 kHz 单声道**。
没有交给 PortAudio/PulseAudio 去重采样，是因为**那个转换过程在验收里是看不见的**：
一旦它悄悄变差，表现是"召回率下降"，而我们会去怀疑模型/发音。
自己按整数倍**取均值**（一个粗糙但确定的低通）落成纯函数，就能**单独测**：
48k 的直流喂进去必须还是那个直流，长度必须正好是 1/3。
"""

import math
import os

#: 模型要求的采样率（训练时的采样率，改成别的会让特征提取错位）。
TARGET_RATE = 16000


def resample_to_16k(samples, in_rate):
    """把任意采样率的单声道 float32 序列降到 16 kHz。

    只支持**整数倍降采样**（本机是 48 kHz → 16 kHz，正好 3 倍）：按 `f` 个样本取均值。
    不做任意比例重采样 —— 那需要滤波器设计，而这里根本用不到；
    真要支持时应当显式加进来，而不是让它悄悄以"线性插值"的名义混过去。

    ⚠️ 取均值是**粗糙的低通**，不是严格抗混叠。对 KWS 够用（16 kHz 的语音里
    8 kHz 以上本来就没什么能量），但**不要**拿它去处理音乐一类的宽带信号。

    返回的数组长度是 **3 的整数倍**（最后不足一组的多余样本被丢弃，最多丢 2 个样本
    = 0.04 ms，对唤醒检测无影响）。
    """
    if in_rate == TARGET_RATE:
        return samples
    if in_rate % TARGET_RATE != 0:
        raise ValueError(
            f'只支持整数倍降采样：{in_rate} → {TARGET_RATE} 不是整数倍')
    factor = in_rate // TARGET_RATE
    n = (len(samples) // factor) * factor
    return samples[:n].reshape(-1, factor).mean(axis=1)


def parse_keywords(path):
    """读关键词表，返回 [(显示名, 音素数), ...]。**纯函数，用于校验表本身。**

    格式（sherpa-onnx KWS）：每行 `<用空格分开的音素> @<显示名>`，例如

        x iǎo h uàn x iǎo h uàn @小幻小幻

    `@` 后面那截是**给我们自己看的**（发事件时用），模型只吃前面的音素。
    漏了 `@` 也接受，显示名退化成音素串 —— 让它能用，但**不假装它是对的**。
    """
    out = []
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line:
                continue
            if '@' in line:
                phon, name = line.split('@', 1)
                phon, name = phon.strip(), name.strip()
            else:
                phon, name = line, line
            if not phon:
                raise ValueError(f'{path}:{lineno} 只有显示名没有音素')
            out.append((name, phon.split()))
    if not out:
        raise ValueError(f'{path} 里一个关键词都没有')
    return out


class KwsEngine:
    """sherpa-onnx 关键词检测器的薄封装：**喂 16 kHz 音频，吐检出的词**。

    `sherpa_onnx` **延迟导入**：纯逻辑的单测不该因为加载一个 onnxruntime 而变慢，
    更不该在没装它的机器上直接 import 失败。
    """

    def __init__(self, model_dir, keywords_file, num_threads=2,
                 threshold=0.25, score=1.0, provider='cpu'):
        import sherpa_onnx                                  # 延迟导入，见类文档
        for f in ('tokens.txt', 'encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
                  'decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx',
                  'joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx'):
            p = os.path.join(model_dir, f)
            if not os.path.isfile(p):
                raise FileNotFoundError(f'唤醒模型缺文件：{p}')
        if not os.path.isfile(keywords_file):
            raise FileNotFoundError(f'关键词表不存在：{keywords_file}')

        self.keywords = parse_keywords(keywords_file)
        self.threshold = float(threshold)
        self.score = float(score)
        self.model_dir = model_dir
        # 自己先过一遍词表：表写坏了要在**启动时**报错，而不是等到"喊它没反应"
        self._kws = sherpa_onnx.KeywordSpotter(
            tokens=os.path.join(model_dir, 'tokens.txt'),
            encoder=os.path.join(model_dir, 'encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx'),
            decoder=os.path.join(model_dir, 'decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx'),
            joiner=os.path.join(model_dir, 'joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx'),
            keywords_file=keywords_file,
            num_threads=int(num_threads),
            keywords_threshold=self.threshold,
            keywords_score=self.score,
            provider=provider,
        )
        self._stream = self._kws.create_stream()
        self.detections = 0

    def feed(self, samples_16k):
        """喂一段 **16 kHz 单声道 float32**，返回这一段里检出的词（可能为空、可能多个）。"""
        if len(samples_16k) == 0:
            return []
        self._stream.accept_waveform(TARGET_RATE, samples_16k)
        hits = []
        # ⚠️ 每一拍都要 decode 干净：`is_ready` 会在检出后仍然为真，
        #    不循环就会把同一个词反复吐出来（变成"喊一次响很多下"）。
        while self._kws.is_ready(self._stream):
            self._kws.decode_stream(self._stream)
            word = self._kws.get_result(self._stream)
            if word:
                hits.append(word)
                # 检出后立刻复位：不复位则下一段音频会接着上一次的状态算，
                # 第二次以后就再也认不出来了（真机上第一次就撞上了这个）。
                self._kws.reset_stream(self._stream)
        self.detections += len(hits)
        return hits

    def describe(self):
        names = '、'.join(n for n, _ in self.keywords)
        return (f'唤醒词 {len(self.keywords)} 个（{names}）｜阈值 {self.threshold}'
                f'｜加分 {self.score}｜模型 {os.path.basename(self.model_dir)}')
