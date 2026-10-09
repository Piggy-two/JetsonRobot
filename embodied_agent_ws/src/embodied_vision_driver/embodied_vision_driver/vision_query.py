#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉查询的**判定逻辑（纯 Python，不依赖 ROS）**。

和 `advance_plan.py` / `obstacle_guard.py` 同构：这是**安全依据**（"能不能说没有"），
所以必须能脱开 ROS、脱开相机、脱开模型反复推演。节点只负责取帧、推理、转发。

🔒 本模块只有一条规则，而它是**不对称**的（2026-10-09 立，DEV_NOTES 坑 43）
-----------------------------------------------------------------------
    找到了        ⇒ **可信**。看到了就是看到了 —— 正面证据不因为画面差而作废。
                    （宁可把糊画面里的误检当成"可能有"，也不要漏掉一个真的。）
    没找到        ⇒ **只有在画面质量达标时才敢这么说**。
    糊 / 没帧 / 模型没就绪 / 类别不在表里 ⇒ **「不知道」**，而不是「没有」。

⚠️ **为什么这条最要紧**：那天我们想给感知铺第一步，结果是**0 个检出** ——
而真相是**相机出厂就是失焦的**。**"检测器什么都没有"与"画面里真没有东西"
看起来一模一样**，任何下游都会把前者读成后者。
把「我不知道」说成「没有」，上层就会据此得出"这里安全 / 没有目标"的结论 ——
这正是本项目的铁律一直在防的（D-028：**不知道 ≠ 安全**）。

⚠️ 而且是**一段窗口**，不是一帧（2026-10-09 实测补上的）
--------------------------------------------------------
同一幅**静止**画面、同一个模型：`suitcase` 只出现在 **68%** 的帧里
（8 秒 152 帧，每帧检出数 0~3）。⇒ 拿**一帧**的"没找到"当结论，
就会在东西明明在眼前时、**三次里有一次自信地说"没有"**。
所以"确认没有"必须基于**最近一段窗口内每一个可用的帧**：
任何一帧看到了 ⇒ 就是看到了；只有**窗口里每一帧都没看到、且每一帧都够清楚**
才敢说"没有"，并且要说清是**几帧**的结论。

⚠️ 还有一条同族的：**类别名不在模型的类别表里，也必须是「不知道」**。
问"杯子"而模型只认 `cup`，如果按"没找到"回答，它会**永远**说"没有杯子" ——
一个永远不会报错的错答案。
"""

from collections import namedtuple

#: 一次查询的结论。`valid=False` 时 `found` 无意义（调用方**先看 valid**）。
Decision = namedtuple('Decision', 'valid found detail')

#: 用于判定的**一段窗口**（不是一帧）。
#:   frames     —— 窗口里有几帧可用的
#:   best       —— 窗口里得分最高的那个匹配检出（None = 一帧都没看到）
#:   quality_min —— 窗口里**最低**的那一帧清晰度（否定结论用最差的那帧说话）
#:   best_below  —— 窗口里**看到了、但分数低于调用方门槛**的那个（None = 连不达标的都没有）。
#:                 ⚠️ 它不是为了"放宽"，是为了**别把话说得比事实强**：
#:                 2026-10-09 实机试出来的 —— 人站在 0.84，调用方要 0.9，
#:                 回答"最近 8 帧里都没有 person"**按定义没错、但人读到的是"没有人"**。
#:                 低于门槛**也必须说出来**。
Window = namedtuple('Window', 'frames best quality_min best_below')


def side_of(cx, width):
    """目标中心相对光心的横向位置：−1 = 最左、0 = 正中、+1 = 最右。

    ⚠️ 这只是**画面里的左右**，**不含距离、不含世界坐标、也没换算朝向**
    （单目相机给不出距离 —— 本机没有深度，D-017）。把它当"目标在左边 30°"用是错的。
    """
    if width <= 0:
        return 0.0
    return max(-1.0, min(1.0, 2.0 * (cx / float(width)) - 1.0))


def pick_best(detections, label, min_score):
    """在检出里挑出 `label` 的**最高分**那个；没有达标的返回 None。

    :param detections: 元素需有 `.label` / `.score` 的对象（ROS 消息或测试替身都行）
    :param min_score: 低于它**不算数**。⚠️ 这个阈值由**调用方**给 ——
                      nano 模型在杂物场景常给 0.2~0.4，"多少算数"是**策略**问题，
                      不是一个可以替调用方拍板的常数。
    """
    want = str(label).strip().lower()
    best = None
    for d in detections or []:
        if str(d.label).strip().lower() != want:
            continue
        if d.score < min_score:
            continue
        if best is None or d.score > best.score:
            best = d
    return best


def label_is_known(label, known_labels):
    """这个类别名在模型的类别表里吗（大小写不敏感）。"""
    want = str(label).strip().lower()
    return any(str(k).strip().lower() == want for k in (known_labels or []))


def decide(*, model_ready, label_known, frame_age_s, frame_max_age_s,
           window, quality_min, min_frames, min_score, label, known_hint=''):
    """把"能不能回答、怎么回答"收在**一处**。返回 `Decision`。

    :param frame_age_s: 距**最新**一帧的秒数；从没收到过帧时为 None
    :param window: `Window` —— 最近一段窗口的汇总（见上面的定义）

    ⚠️ 判定的**顺序**有意义：**"有没有资格回答"排在"答案是什么"前面**。
       反过来写（先看结果、再补一句质量警告）也能跑，但那样**不可信的答案
       会先被说出来**，警告只是附注 —— 而附注没人读。
    """
    if not model_ready:
        return Decision(False, False, '模型还没就绪（正在加载，或加载失败）—— 不知道')

    if not label_known:
        # ★ 同族的坑：问一个模型不认识的类别，按"没找到"回答会**永远**说"没有"
        return Decision(
            False, False,
            f'模型的类别表里没有 {label!r} —— 不知道（不是"没有"）。'
            f'{known_hint}')

    if frame_age_s is None:
        return Decision(False, False, '还没有收到任何图像 —— 不知道')

    if frame_age_s > frame_max_age_s:
        return Decision(
            False, False,
            f'图像陈旧：{frame_age_s:.1f}s 没有新帧（上限 {frame_max_age_s:g}s）—— 不知道。'
            f'（画面停住时"没找到"不能算数：那是没在看的安静，不是没东西）')

    if window.best is not None:
        # ★ 正面证据不因为画面差而作废：**任何一帧**看到了就算看到
        return Decision(True, True,
                        f'看到了 {label}（score={window.best.score:.2f}，'
                        f'最近 {window.frames} 帧里出现过）')

    # ---- 到这里是"一帧都没看到"：够不够格说"没有"？ ----
    if window.frames < min_frames:
        return Decision(
            False, False,
            f'最近只拿到 {window.frames} 帧（要 ≥ {min_frames} 帧才够确认"没有"）'
            f'—— 不知道，不是"没有"')

    if window.best_below is not None:
        # ★ 看到了、但不够调用方要的那个分 —— **必须说出来**，
        #   否则回答会读成"这里没有人"，而人就在那儿（差距只是门槛）。
        b = window.best_below
        return Decision(
            True, False,
            f'**看到了 {label}（score={b.score:.2f}），但低于你要的 {min_score:g}** —— '
            f'按你的标准算"没有"，但画面里确实有它（要么放宽门槛，要么换个判断办法）')

    if window.quality_min < quality_min:
        # ★★ 本模块存在的理由
        return Decision(
            False, False,
            f'窗口里有画面过糊的帧（最低清晰度 {window.quality_min:.0f} < {quality_min:g}）'
            f'—— **不能因此说"没有 {label}"**：糊的画面会漏掉真东西。'
            f'2026-10-09 的教训：相机失焦时所有检测都是 0 个，而它和'
            f'"真没东西"看起来一模一样（DEV_NOTES 坑 43）')

    return Decision(True, False,
                    f'最近 {window.frames} 帧里都没有 {label}'
                    f'（最低清晰度 {window.quality_min:.0f} ≥ {quality_min:g}，'
                    f'这个"没有"是可信的）')
