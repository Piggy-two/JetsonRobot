#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay Safety Runtime（第一版）。

项目的核心主张是 **`Safety > Control > Skill > Agent`，Safety 具有最终否决权**。
本节点是这条主张的**第一个落点**：把"停"这件事从各个下游节点手里收上来，
变成一处**本地、不经过 LLM、不依赖任何人配合**的动作。

本版实现两件事：

**一、本地安全指令通路（D-006 / D-024 决策 3）**
订阅厂商 ASR 的文本话题，**在本节点内本地匹配**安全词（`estop.py`，纯逻辑、可单测）。
识别侧复用厂商的**离线**链路（不联网），执行侧完全由本项目自己做。
⚠️ **一个字都不经过 LLM，也不经过厂商的 `voice_control_*` 节点。**

**二、独立的零速通道（`zero_channel_topic`，默认 `/cmd_vel`）**
锁存期间，本节点**自己**以固定频率直接向底盘发 `Twist()`（全零）。

    为什么不"命令 Motor Driver 停"就够了 —— D-020 实测：底盘**没有指令超时保护**，
    停止发布 ≠ 停车，它会一直保持最后一条速度。而"持续发零"这件事现在**只在
    Motor Driver 里**（D-025）。**如果 Motor Driver 自己挂了，就没人发零了。**
    所以急停通道必须**独立**：不依赖 Motor Driver 存活、不依赖 Control Skill 存活。
    另两个 `~/stop` 服务照发（best-effort），但**急停不靠它们成立**。

    🔒 **本节点唯一的发布语句就是 `publish(Twist())`** —— 它在结构上
    **不可能**发出任何非零速度。这不是"约定"，是代码里只有这一个出口。

⚠️ **本版不做**：避障、速度/区域限制、Agent 侧的 Skill 网关、对 Motor Driver
   存活性的监视（"Motor Driver 挂了就自动接管"）。这些是后续增量。

⚠️ 触发后的急停是**锁存**的：必须显式调 `~/release` 才能解除（`estop.py` 的 `EStopLatch`）。
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger

from embodied_safety_runtime.estop import (
    DEFAULT_SAFETY_PHRASES, EStopLatch, is_safety_command)


class SafetyRuntime(Node):
    def __init__(self):
        super().__init__('safety_runtime')

        self.declare_parameter('voice_topic', '/asr_node/voice_words')
        # ⚠️ 独立零速通道。**只发零**，见文件头说明。
        self.declare_parameter('zero_channel_topic', '/cmd_vel')
        self.declare_parameter('zero_rate', 10.0)
        self.declare_parameter('safety_phrases', DEFAULT_SAFETY_PHRASES)
        # best-effort：把下游的锁存也打开（急停不依赖它们）
        self.declare_parameter('motor_stop_service', '/motor_driver/stop')
        self.declare_parameter('control_stop_service', '/control_skills/stop')
        self.declare_parameter('status_topic', '/embodied/safety/status')
        self.declare_parameter('event_topic', '/embodied/safety/events')

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self.phrases = list(g('safety_phrases'))
        self.latch = EStopLatch()
        self._zero_frames = 0
        self._triggers = 0

        # 🔒 唯一的发布者，且只发 Twist()
        self.zero_pub = self.create_publisher(Twist, g('zero_channel_topic'), 10)
        self.status_pub = self.create_publisher(Float64MultiArray, g('status_topic'), 10)
        self.event_pub = self.create_publisher(String, g('event_topic'), 10)

        self.create_subscription(String, g('voice_topic'), self.on_voice, 10)

        self.create_service(Trigger, '~/estop', self.on_estop)
        self.create_service(Trigger, '~/release', self.on_release)

        self._motor_cli = self.create_client(Trigger, g('motor_stop_service'))
        self._ctrl_cli = self.create_client(Trigger, g('control_stop_service'))
        self._pending = []

        rate = float(g('zero_rate'))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Safety Runtime 启动 | 语音安全词 <- {g("voice_topic")} | '
            f'独立零速通道 -> {g("zero_channel_topic")} @ {rate:.0f} Hz（**只发零**）| '
            f'安全词 {len(self.phrases)} 条')
        self.get_logger().warn(
            '⚠️ 本版**不含**避障 / 限速 / 对下游存活的监视；急停是锁存的，'
            '必须显式调 ~/release 才能解除。')

    # ---------- 本地安全指令（不经 LLM） ----------

    def on_voice(self, msg):
        hit, phrase = is_safety_command(msg.data, self.phrases)
        if hit:
            self.get_logger().error(
                f'★ 本地识别到安全指令：{msg.data!r}（命中「{phrase}」）—— 立即急停')
            self._trigger(f'voice:{phrase}')
        else:
            self.get_logger().debug(f'语音文本（非安全词）：{msg.data!r}')

    def _trigger(self, reason):
        changed = self.latch.trigger(reason)
        self._triggers += 1
        # 立刻发一帧零，不等定时器那一拍
        self._publish_zero()
        if changed:
            self._emit(f'estop_triggered:{reason}')
            self.get_logger().error(f'急停已锁存（{reason}）—— 需显式 ~/release 才能解除')
        # best-effort：把下游的锁存也打开
        self._call(self._motor_cli, 'motor_driver/~/stop')
        self._call(self._ctrl_cli, 'control_skills/~/stop')

    def _call(self, cli, what):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{what} 不可用（不影响急停：零速通道是独立的）')
            return
        fut = cli.call_async(Trigger.Request())
        self._pending.append((what, fut))
        fut.add_done_callback(lambda f, w=what: self.get_logger().info(f'{w} 已调用'))

    def on_estop(self, _req, res):
        """手动触发（也给验收/测试用）。"""
        self._trigger('service')
        res.success = True
        res.message = f'急停已触发（原因：{self.latch.reason}）'
        return res

    def on_release(self, _req, res):
        changed = self.latch.release()
        if changed:
            self._emit('estop_released')
            self.get_logger().warn('急停已**显式解除**（注意：Motor Driver 若也被锁存过，'
                                   '它需要各自的 ~/resume）')
        res.success = True
        res.message = '已解除' if changed else '本来就是解除的'
        return res

    # ---------- 独立零速通道 ----------

    def tick(self):
        if self.latch.latched:
            self._publish_zero()
        self._publish_status()

    def _publish_zero(self):
        self.zero_pub.publish(Twist())      # 🔒 全节点唯一的发布语句，只发零
        self._zero_frames += 1

    def _publish_status(self):
        m = Float64MultiArray()
        m.data = [1.0 if self.latch.latched else 0.0,
                  float(self._zero_frames), float(self._triggers)]
        self.status_pub.publish(m)

    def _emit(self, name):
        e = String()
        e.data = name
        self.event_pub.publish(e)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补一发零：这是"安全节点"该有的姿态（虽然它只管锁存期间的车）。
        try:
            node.zero_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
