#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay 电机/底盘 Driver（架构分层：Driver 层）。

把厂商底盘收口成本项目**唯一**的一个执行器入口，并把五条硬约束**封在这里**，
而不是让上层每个 Skill 各自记着：

  1. **主动持续发 0**（D-020 / #18）—— 底盘**没有指令超时保护**，停止发布 ≠ 停车。
     本节点以固定频率**持续发布**，空闲时发零。
  2. **存活判据只用 `imu_raw` / `battery`**（D-021 / #21）—— 断线时 `/odom` 照发 28.5 Hz，
     进程也照活，只有这两个会停。绝不用 `/odom`，绝不用 `pgrep`。
  3. **限幅**（纵深防御）—— 厂商对 `/cmd_vel` 有钳制，但 `/controller/cmd_vel` 那条**完全没有**（#19）。
     自己再钳一次，不依赖厂商实现对不对。
  4. **锁存停车** —— `/stop` 服务一旦触发，必须显式 `/resume` 才能再动。
  5. **安全层否决**（**D-037**）—— 直接读 Safety Runtime 的状态，它锁存期间本节点自己发零。
     为什么不能只靠"Safety 也往 `/cmd_vel` 发零"：那两个是**并列发布者、没有仲裁**
     （#28 / `DEV_NOTES` 坑 23），实测下游 `stop` 调不通时那条话题上**非零占 66%**。
     ⚠️ **安全层不在跑 ⇒ 本节点拒绝运动**（本机没有物理急停，#22）。

话题方向（与相机 Driver 的"收口"是同一个模式，方向相反）：

```text
上层 Skill ──→ /embodied/motor/cmd_vel ──→[本节点]──→ /cmd_vel ──→ 厂商 odom_publisher ──→ 电机
```

⚠️ **本节点不含任何语义/规划，LLM 不得介入**（CLAUDE.md §0）。
⚠️ **默认 `dry_run=true`**：不发 `/cmd_vel`，只发一条干跑话题。本机**没有物理急停**（#22），
   所以"能真动"必须是**显式打开**的、且必须在有人看护的时段打开。

判断逻辑（钳制 / 超时 / 存活 / 锁存）全部在 `safety_gate.py` 里，纯 Python、可离线单测；
本文件只负责 ROS 接线。
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16, Float64MultiArray, String
from std_srvs.srv import Trigger

from embodied_motor_driver.safety_gate import LINK_LOST, MotorSafetyGate


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/embodied/motor/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('dry_run_topic', '/embodied/motor/cmd_vel_dryrun')
        self.declare_parameter('status_topic', '/embodied/motor/status')
        self.declare_parameter('event_topic', '/embodied/motor/events')
        # 存活判据（D-021）：只认这两个
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.declare_parameter('battery_topic', '/ros_robot_controller/battery')
        self.declare_parameter('publish_rate', 20.0)
        # 与厂商 /cmd_vel 的钳制一致（实机实测值，见 PROJECT_STATUS §7）
        self.declare_parameter('max_vx', 0.2)
        self.declare_parameter('max_vy', 0.2)
        self.declare_parameter('max_wz', 0.5)
        self.declare_parameter('cmd_timeout', 0.5)
        # ---- 安全层否决（D-037）----
        # 本节点**直接读安全层的状态**，锁存期间自己输出零 —— 否决不再依赖
        # "谁先谁后"，也**不再依赖任何服务调用是否成功**（#28 / DEV_NOTES 坑 23）。
        # ⚠️ 默认 True：安全层不在跑就**不许动**（本机没有物理急停，#22）。
        self.declare_parameter('require_safety', True)
        self.declare_parameter('safety_status_topic', '/embodied/safety/status')
        self.declare_parameter('safety_timeout', 1.0)
        # ⚠️ 必须比**最慢的**那路遥测周期大不少：battery 实测 0.95 Hz（周期 1.05 s），
        #    imu_raw 46.9 Hz。取 2.5 s 是给 battery 留了 2 倍余量 ——
        #    若只按 imu 的周期去定，imu 单独失效时会跟着 battery 的节拍反复抖动。
        self.declare_parameter('telemetry_timeout', 2.5)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.dry_run = bool(g('dry_run'))
        self.out_topic = g('dry_run_topic') if self.dry_run else g('output_topic')

        self.gate = MotorSafetyGate(
            max_vx=g('max_vx'), max_vy=g('max_vy'), max_wz=g('max_wz'),
            cmd_timeout=g('cmd_timeout'), telemetry_timeout=g('telemetry_timeout'),
            require_safety=bool(g('require_safety')),
            safety_timeout=float(g('safety_timeout')))

        # ---- 发布：注意是**持续**发布，不是"有指令才发"（D-020）----
        self.pub = self.create_publisher(Twist, self.out_topic, 1)
        self.status_pub = self.create_publisher(
            Float64MultiArray, g('status_topic'), 10)
        # 链路事件：只在**变化**时发，便于上层订阅而不必轮询 status
        self.event_pub = self.create_publisher(String, g('event_topic'), 10)

        # ---- 订阅 ----
        self.create_subscription(Twist, g('input_topic'), self.on_cmd, 1)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, 10)
        self.create_subscription(UInt16, g('battery_topic'), self.on_battery, 10)
        # 安全层状态（D-037）。只有 require_safety 时才订阅 —— 关掉它就没有这道闸。
        if g('require_safety'):
            self.create_subscription(Float64MultiArray, g('safety_status_topic'),
                                     self.on_safety_status, 10)

        # ---- 服务 ----
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_service(Trigger, '~/resume', self.on_resume)

        # ---- 主循环 ----
        rate = float(g('publish_rate'))
        self.create_timer(1.0 / rate, self.tick)

        self._last_state = None
        self._last_link = None
        self._last_needs_rearm = False
        self._last_clamp_log = 0.0

        self.get_logger().info(
            f"Motor Driver 启动 | {g('input_topic')} -> {self.out_topic} | "
            f"限幅 ±{g('max_vx')}/{g('max_vy')} m/s, ±{g('max_wz')} rad/s | "
            f"指令超时 {g('cmd_timeout')}s，遥测超时 {g('telemetry_timeout')}s | "
            f"{rate:.0f} Hz")
        if g('require_safety'):
            self.get_logger().info(
                f"安全层否决（D-037）：读 {g('safety_status_topic')}，"
                f"状态停更 > {g('safety_timeout')}s 或它锁存 → 本节点自己输出零。"
                "⚠️ 它不在跑时本节点**拒绝运动**。")
        else:
            self.get_logger().warn(
                "🔴 require_safety=false：**不做安全层否决** —— 本节点行为回到 D-025 那一版。"
                "只允许用于台架 / 离线实验，真机运动不得用这个配置。")
        if self.dry_run:
            self.get_logger().warn(
                f"⚠️ dry_run=true：**不会**向 {g('output_topic')} 发布任何东西，"
                f"只发到 {self.out_topic}（车不会动）。真要驱动底盘需显式设 dry_run:=false，"
                "且必须在有人看护、能直接断电的时段。")
        else:
            self.get_logger().warn(
                f"🔴 dry_run=false：正在向 **{self.out_topic}** 发布，底盘会真的动！"
                "本机没有物理急停（#22）。")

    # ---------- 回调 ----------

    def on_cmd(self, msg):
        before = (msg.linear.x, msg.linear.y, msg.angular.z)
        after = self.gate.on_cmd(*before, now=time.monotonic())
        if any(abs(a - b) > 1e-9 for a, b in zip(before, after)):
            now = time.monotonic()
            if now - self._last_clamp_log > 2.0:
                self._last_clamp_log = now
                self.get_logger().warn(
                    f"指令被限幅：({before[0]:+.3f}, {before[1]:+.3f}, {before[2]:+.3f}) "
                    f"-> ({after[0]:+.3f}, {after[1]:+.3f}, {after[2]:+.3f})")

    def on_imu(self, _msg):
        self.gate.on_imu(time.monotonic())

    def on_battery(self, _msg):
        self.gate.on_battery(time.monotonic())

    def on_safety_status(self, msg):
        """安全层的状态。**只要第一个字段**（是否锁存）—— 本节点不做安全层的判断。"""
        if len(msg.data) >= 1:
            self.gate.on_safety_status(msg.data[0] > 0.5, time.monotonic())

    # ---------- 服务 ----------

    def on_stop(self, _req, res):
        self.gate.stop()
        res.success = True
        res.message = '已锁存停车（需显式 resume 才能再动）'
        self.get_logger().warn('收到 stop：已锁存停车')
        return res

    def on_resume(self, _req, res):
        self.gate.resume()
        res.success = True
        res.message = '已解除锁存'
        self.get_logger().warn('收到 resume：已解除锁存')
        return res

    # ---------- 主循环 ----------

    def tick(self):
        (vx, vy, wz, state, age_cmd, age_imu, age_batt,
         link, needs_rearm, age_safety, safety_latched,
         safety_blocked) = self.gate.step(time.monotonic())

        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.pub.publish(t)

        m = Float64MultiArray()
        m.data = [
            self.gate.state_code(state), float(vx), float(vy), float(wz),
            -1.0 if age_cmd is None else age_cmd * 1000.0,
            -1.0 if age_imu is None else age_imu * 1000.0,
            -1.0 if age_batt is None else age_batt * 1000.0,
            1.0 if self.gate.latched else 0.0,
            1.0 if self.dry_run else 0.0,
            1.0 if needs_rearm else 0.0,
            # ---- 以下是 2026-10-07 追加的安全层字段（前十个位置不变）----
            -1.0 if age_safety is None else age_safety * 1000.0,
            1.0 if safety_latched else 0.0,
            1.0 if safety_blocked else 0.0,
        ]
        self.status_pub.publish(m)

        # ---- 链路事件：只在**变化**时发，便于上层/日志订阅，不用轮询 status ----
        if link != self._last_link:
            if link == LINK_LOST:
                self._emit_event('chassis_link_lost')
                self.get_logger().error(
                    '底盘链路失联：遥测停发。⚠️ 底盘此时可能仍保持最后一条速度（D-020），'
                    '而重新 bind 不能自恢复 —— 真恢复需要 '
                    '`sudo systemctl restart start_app_node.service`（D-021）。'
                    '本节点会持续发零，直到显式 /resume。')
            elif self._last_link == LINK_LOST:
                self._emit_event('chassis_link_recovered')
                self.get_logger().warn('底盘链路恢复。')
            self._last_link = link

        if needs_rearm != self._last_needs_rearm:
            if needs_rearm:
                self._emit_event('rearm_required')
                self.get_logger().error(
                    '失联发生在底盘"可能还在动"的时刻 → 已置位「需重新使能」。'
                    '在显式 /resume 之前，即使上层继续发指令也**一律输出零**。')
            else:
                self._emit_event('rearmed')
                self.get_logger().warn('「需重新使能」已清除。')
            self._last_needs_rearm = needs_rearm

        if state != self._last_state:
            if state == self.gate.STATE_OK:
                self.get_logger().info(f'状态 -> {state}')
            elif state == self.gate.STATE_SAFETY_BLOCKED:
                self.get_logger().error(
                    f'状态 -> {state}（输出零速度）：**安全层没有放行**'
                    f'（状态龄 {m.data[10]:.0f} ms，锁存 = {bool(safety_latched)}）。'
                    '本节点现在直接读安全层状态，不再依赖任何服务调用是否成功（D-037）。'
                    '⚠️ 安全层不在跑时本节点**拒绝运动** —— 台架实验要显式 require_safety:=false')
            else:
                self.get_logger().warn(
                    f'状态 -> {state}（输出零速度；'
                    f'imu 龄 {m.data[5]:.0f} ms / battery 龄 {m.data[6]:.0f} ms）')
            self._last_state = state

    def _emit_event(self, name):
        e = String()
        e.data = name
        self.event_pub.publish(e)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补一帧零速度：万一上层还在动，至少留最后一条是"停"。
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
