#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`embodied_voice_wakeup` —— **本项目自己的语音唤醒**（Phase 5，Driver 层）。

为什么要有它（而不是用厂商那条）
--------------------------------
厂商环形麦的**硬件唤醒上报不可用**：2026-10-08 实测，断电重启后对着它说 3 遍唤醒词，
控制串口 `/dev/ring_mic` **0 字节**；握手包 4 种 RTS/DTR 组合也全无应答
（见 `docs/DEV_NOTES.md` 坑 33 与 `#25`）。**采集通路一直是好的**（D-022 验过录音/播放），
坏的只是"模块把自己的判断报出来"这一条。所以唤醒改在 Jetson 上做：

        环形麦（48 kHz 单声道采集）  →  16 kHz  →  本地 KWS  →  唤醒事件

**这条路我们完全控制得住**：模型、阈值、关键词都在我们手里，可回归、可量召回率，
不依赖任何厂商节点的状态。另外它**绕开了厂商 `voice_control_move`** —— 那个节点挂在
**无限幅**的 `/controller/cmd_vel` 上（#26），是运动测试前要显式停掉的东西。

它不做什么
----------
  · **不识别命令**，只报"有人喊了唤醒词"。唤醒之后的指令走 D-006 的命令路由器；
  · **不发任何控制指令**，不碰 `/cmd_vel`，不做安全判定 —— 唤醒与安全无关；
  · 不碰厂商 `~/ros2_ws`。

发布
----
    `/embodied/voice/wakeup`（`std_msgs/String`，**只在检出时发**，内容 = 关键词的显示名）
    `/embodied/voice/diag`（`Float64MultiArray` = [采集帧数, 已处理音频秒数,
                           累计检出次数, 距上次检出的秒数, 当前阈值]）

⚠️ **召回率不是 100%。** 默认阈值下真机初测是**说 5 遍认出 2 遍**。所以：
   · 本节点报的是"**检出**"，不是"**用户没喊**"—— 沉默**不能**当作反证；
   · 阈值/关键词要用 `--calibrate` 那种方式**量出来**，别靠感觉（见 README）。
"""

import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from embodied_voice_wakeup.kws_engine import KwsEngine, resample_to_16k


class VoiceWakeup(Node):
    def _p(self, name, default):
        """**先查后声明**再取值。

        ⚠️ 不能直接 `declare_parameter(name, default)`：参数文件里给了这个参数时，
        它**已经被声明过**，再声明一次会抛 `ParameterAlreadyDeclaredException`
        —— 而报错信息只说"已经声明"，看不出是"配置文件里写过的那些参数"导致的。
        这样写的好处是**有没有参数文件都能跑**（没给就落到这里的默认值）。
        """
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def __init__(self):
        super().__init__('voice_wakeup')
        g = self._p
        model_dir = str(g('model_dir', ''))
        keywords_file = str(g('keywords_file', ''))
        audio_device = str(g('audio_device', ''))   # 空 = 系统默认（本机 = 环形麦，见 README）
        capture_rate = int(g('capture_rate', 48000))  # 环形麦原生；16 kHz 走重采样那条分支
        num_threads = int(g('num_threads', 2))
        threshold = float(g('keywords_threshold', 0.25))  # ⚠️ 越大越难触发：召回与误报在此换
        score = float(g('keywords_score', 1.0))
        detect_cooldown = float(g('detect_cooldown', 1.0))  # 秒。同一句避免连发
        wake_topic = str(g('wake_topic', '/embodied/voice/wakeup'))
        diag_topic = str(g('diag_topic', '/embodied/voice/diag'))
        diag_rate = float(g('diag_rate', 2.0))

        self._cooldown = detect_cooldown
        self._last_hit = None
        self._frames = 0
        self._samples_in = 0
        self._samples_16k = 0

        self.wake_pub = self.create_publisher(String, wake_topic, 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, diag_topic, 10)
        self.create_timer(1.0 / diag_rate, self._publish_diag)

        if not model_dir or not keywords_file:
            # 明确拒绝，不静默降级 —— 一个"起来了但永远不报"的唤醒节点比没有更坏
            raise RuntimeError(
                '必须给 model_dir 与 keywords_file。唤醒模型是**大文件，不进仓库**，'
                '需另行准备（见包 README 的 "模型从哪来"）。')

        self.engine = KwsEngine(
            model_dir=model_dir, keywords_file=keywords_file,
            num_threads=num_threads, threshold=threshold, score=score)
        self.get_logger().info(f'唤醒引擎就绪：{self.engine.describe()}')

        self._q = queue.Queue(maxsize=64)
        self._rate = capture_rate
        self._start_audio(audio_device or None)

        # 音频回调只做"搬数据"，重采样与推理放在这个线程里 ——
        # 唤醒词检测是一次几十毫秒的推理，**不能**放进音频回调，那会丢帧。
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(
            f'语音唤醒启动 | 采集 {self._rate} Hz → 16 kHz | '
            f'事件 -> {wake_topic}')

    def _start_audio(self, device):
        try:
            import sounddevice as sd
        except ImportError as exc:                              # noqa: BLE001
            raise RuntimeError(f'没有 sounddevice：{exc}') from exc
        try:
            self._stream = sd.InputStream(
                device=device, channels=1, samplerate=self._rate,
                dtype='float32', blocksize=self._rate // 10, callback=self._on_audio)
            self._stream.start()
        except Exception as exc:                                # noqa: BLE001
            # 打不开麦克风就**停下来**并说清是哪一段 —— 静默失败会变成"喊了没反应"
            raise RuntimeError(
                f'打不开音频输入 device={device!r} @ {self._rate} Hz：{exc}。'
                f'查：麦克风在不在、有没有别的进程独占声卡。') from exc

    def _on_audio(self, indata, frames, t, status):              # noqa: ARG002
        self._frames += 1
        self._samples_in += len(indata)
        try:
            self._q.put_nowait(indata[:, 0].copy())
        except queue.Full:
            pass                                                # 宁可丢也不堵音频线程

    def _loop(self):
        while self._running and rclpy.ok():
            try:
                blk = self._q.get(timeout=0.3)
            except queue.Empty:
                continue
            try:
                x16 = resample_to_16k(blk, self._rate)
                self._samples_16k += len(x16)
                for word in self.engine.feed(x16):
                    self._on_wake(word)
            except Exception as exc:                            # noqa: BLE001
                self.get_logger().error(f'唤醒检测出错：{exc}')

    def _on_wake(self, word):
        now = time.monotonic()
        if self._last_hit is not None and now - self._last_hit < self._cooldown:
            self.get_logger().debug(f'忽略冷却期内的「{word}」（同一句连发）')
            return
        self._last_hit = now
        msg = String()
        msg.data = word
        self.wake_pub.publish(msg)
        self.get_logger().info(
            f'\033[1;32m★ 唤醒：{word}\033[0m（累计 {self.engine.detections} 次）')

    def _publish_diag(self):
        age = -1.0 if self._last_hit is None else time.monotonic() - self._last_hit
        m = Float64MultiArray()
        m.data = [float(self._frames), self._samples_16k / 16000.0,
                  float(self.engine.detections), age, self.engine.threshold]
        self.diag_pub.publish(m)

    def destroy_node(self):
        self._running = False
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:                                       # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = VoiceWakeup()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                                    # noqa: BLE001
        print(f'\033[31m语音唤醒启动失败：{exc}\033[0m')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
