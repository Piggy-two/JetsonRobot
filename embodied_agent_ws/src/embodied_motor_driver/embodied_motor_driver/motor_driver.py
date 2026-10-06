#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay 电机/底盘 Driver（架构分层：Driver 层）。

把厂商底盘收口成本项目**唯一**的一个执行器入口，并把四条硬约束**封在这里**，
而不是让上层每个 Skill 各自记着：

  1. **主动持续发 0**（D-020 / #18）—— 底盘**没有指令超时保护**，停止发布 ≠ 停车。
     本节点以固定频率**持续发布**，空闲时发零。
  2. **存活判据只用 `imu_raw` / `battery`**（D-021 / #21）—— 断线时 `/odom` 照发 28.5 Hz，
     进程也照活，只有这两个会停。绝不用 `/odom`，绝不用 `pgrep`。
  3. **限幅**（纵深防御）—— 厂商对 `/cmd_vel` 有钳制，但 `/controller/cmd_vel` 那条**完全没有**（#19）。
     自己再钳一次，不依赖厂商实现对不对。
  4. **锁存停车** —— `/stop` 服务一旦触发，必须显式 `/resume` 才能再动。

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
from std_msgs.msg import UInt16, Float64MultiArray
from std_srvs.srv import Trigger

from embodied_motor_driver.safety_gate import MotorSafetyGate


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/embodied/motor/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('dry_run_topic', '/embodied/motor/cmd_vel_dryrun')
        self.declare_parameter('status_topic', '/embodied/motor/status')
        # 存活判据（D-021）：只认这两个
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.declare_parameter('battery_topic', '/ros_robot_controller/battery')
        self.declare_parameter('publish_rate', 20.0)
        # 与厂商 /cmd_vel 的钳制一致（实机实测值，见 PROJECT_STATUS §7）
        self.declare_parameter('max_vx', 0.2)
        self.declare_parameter('max_vy', 0.2)
        self.declare_parameter('max_wz', 0.5)
        self.declare_parameter('cmd_timeout', 0.5)
        # ⚠️ 必须比**最慢的**那路遥测周期大不少：battery 实测 0.95 Hz（周期 1.05 s），
        #    imu_raw 46.9 Hz。取 2.5 s 是给 battery 留了 2 倍余量 ——
        #    若只按 imu 的周期去定，imu 单独失效时会跟着 battery 的节拍反复抖动。
        self.declare_parameter('telemetry_timeout', 2.5)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.dry_run = bool(g('dry_run'))
        self.out_topic = g('dry_run_topic') if self.dry_run else g('output_topic')

        self.gate = MotorSafetyGate(
            max_vx=g('max_vx'), max_vy=g('max_vy'), max_wz=g('max_wz'),
            cmd_timeout=g('cmd_timeout'), telemetry_timeout=g('telemetry_timeout'))

        # ---- 发布：注意是**持续**发布，不是"有指令才发"（D-020）----
        self.pub = self.create_publisher(Twist, self.out_topic, 1)
        self.status_pub = self.create_publisher(
            Float64MultiArray, g('status_topic'), 10)

        # ---- 订阅 ----
        self.create_subscription(Twist, g('input_topic'), self.on_cmd, 1)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, 10)
        self.create_subscription(UInt16, g('battery_topic'), self.on_battery, 10)

        # ---- 服务 ----
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_service(Trigger, '~/resume', self.on_resume)

        # ---- 主循环 ----
        rate = float(g('publish_rate'))
        self.create_timer(1.0 / rate, self.tick)

        self._last_state = None
        self._last_clamp_log = 0.0

        self.get_logger().info(
            f"Motor Driver 启动 | {g('input_topic')} -> {self.out_topic} | "
            f"限幅 ±{g('max_vx')}/{g('max_vy')} m/s, ±{g('max_wz')} rad/s | "
            f"指令超时 {g('cmd_timeout')}s，遥测超时 {g('telemetry_timeout')}s | "
            f"{rate:.0f} Hz")
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
        vx, vy, wz, state, age_cmd, age_imu, age_batt = self.gate.step(time.monotonic())

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
        ]
        self.status_pub.publish(m)

        if state != self._last_state:
            if state == self.gate.STATE_OK:
                self.get_logger().info(f'状态 -> {state}')
            else:
                self.get_logger().warn(
                    f'状态 -> {state}（输出零速度；'
                    f'imu 龄 {m.data[5]:.0f} ms / battery 龄 {m.data[6]:.0f} ms）')
            self._last_state = state


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
