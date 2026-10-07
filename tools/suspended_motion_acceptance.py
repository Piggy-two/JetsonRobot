#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**架空**运动验收 —— 第一次让本项目自己的链去驱动底盘。

    ⚠️ 这是【验收工装】，不是运行时组件。
    ⚠️ **前提：四轮离地。** 轮子在空中转，车不会跑掉。

为什么要有这个（和 dry-run 验收的区别）
---------------------------------------
`tools/upper_layer_dryrun_acceptance.py` 验的是**链路**：速度发出来了、积分对得上。
但 Motor Driver 一直是 `dry_run=true` —— **它从没真的驱动过底盘**。
本项目所有"车会不会按我说的动"的结论，都还没有真机证据。

本工装补的就是这一段，而且刻意在**架空**条件下做：轮子真的转、电机真的通电，
但车不会位移，失败模式最温和。

判据：IMU 的**哪个轴**？—— 别照搬 D-020
------------------------------------------
底盘**没有轮速回读**话题 —— `/ros_robot_controller/set_motor` 是**发下去的**指令，
只能证明"运动学算对了"，**不能**证明"轮子真的转了"（自证不算证据）。
唯一独立的物理量是 IMU。但**在地面测和悬空测，信号在不同的轴上**：

| 场景 | 主要信号 | 为什么 |
|---|---|---|
| **在地上**（D-020 那次） | `gyro_z`（偏航）振幅 ±0.05~0.067 vs 静止 ±0.0015 | 振动经**地面**耦合上来 |
| **悬空**（本工装） | **`gyro_x`（俯仰）** | 轮子一转产生**反作用力矩**，机身前后俯仰 |

⚠️ 本工装第一版**照搬了 D-020 的 `gyro_z`**，结果在悬空条件下什么都测不到，
差点得出"轮子没转"的结论。**同一台机器、同一个量，换个工况信号就换了轴** ——
所以这里改成：**同时看三个陀螺轴，取最大跨度，并与本次实测的基线比倍率**。

判据因此是**相对的**（span > max(绝对下限, 1.5 × 基线)），不依赖任何拍出来的绝对阈值 ——
扶着车的手抖、吊绳的摆动都会进基线，比倍率比比绝对值稳。

⚠️ **本工装的读数不能单独定案**：它给的是物理旁证，最终仍以**人眼看到轮子转**为准
（2026-10-07 首测就是这么定案的）。

驱动路径（**走我们自己的链，不发 `/cmd_vel`**）
----------------------------------------------
    【1】【2】【3】 → /control_skills/* 服务（Control Skill 直调）
    【4】        → 文本 → Command Router → Skill Gateway → Control Skill → Motor Driver

⚠️ **两条 2026-10-07 之后新增的耦合**（D-036 / D-037）：跑本工装时
   ① Safety Runtime 必须**关掉避障守卫**（`enable_obstacle_guard:=false`）——
      它会拦住正在被命令前进的运动（判据见 `upper_layer_dryrun_acceptance.py` 的说明）；
   ② Motor Driver 现在**要求安全层在场**（`require_safety` 默认 true），所以
      Safety Runtime 必须真的在跑，否则 Motor Driver 拒绝运动（状态码 5）。

前置（全部已启动，且 Motor Driver 必须 `dry_run:=false`）：
    embodied_motor_driver (dry_run:=false) / embodied_control_skills /
    embodied_skill_gateway (allow_motion:=true) / embodied_command_router (allow_motion:=true) /
    embodied_safety_runtime

用法：
    python3 tools/suspended_motion_acceptance.py
退出码：0 = 全部断言通过；1 = 有失败。
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import Imu

from embodied_skills_interfaces.srv import MoveRelative

SET_MOTOR = '/ros_robot_controller/set_motor'
IMU = '/ros_robot_controller/imu_raw'
TEXT_TOPIC = '/embodied/command/text'

# 相对判据：跨度要超过这个绝对下限，**并且**超过基线的这个倍率
ABS_FLOOR = 0.006
BASELINE_RATIO = 1.5

# ⚠️ 基线本身超过这个值就**拒绝出结论**。
#    悬空的车很容易荡（手扶、吊绳），晃起来基线的跨度能到 0.4~0.5 ——
#    那时"比基线的 1.5 倍"这个门槛高到永远达不到，**跑出来的全是假失败**。
#    与其把阈值调松去迎合，不如**拒绝测**：先让人把车扶稳/挂稳，再重跑。
#    （这一条是被实际跑坏一次才加上去的，见 DEVELOPMENT_LOG。）
QUIET_BASELINE_MAX = 0.08


class SuspendedTester(Node):
    def __init__(self):
        super().__init__('suspended_motion_acceptance')
        self._lock = threading.Lock()
        self._reset()
        self.create_subscription(MotorsState, SET_MOTOR, self._on_motor, 10)
        self.create_subscription(Imu, IMU, self._on_imu, 10)
        self._status = None
        self.create_subscription(Float64MultiArray, '/embodied/motor/status',
                                 self._on_status, 10)
        self.text_pub = self.create_publisher(String, TEXT_TOPIC, 10)
        self.move_cli = self.create_client(MoveRelative, '/control_skills/move_relative')
        self.ctrl_stop = self.create_client(Trigger, '/control_skills/stop')
        self.motor_stop = self.create_client(Trigger, '/motor_driver/stop')
        self.motor_resume = self.create_client(Trigger, '/motor_driver/resume')
        self.baseline = 0.0

    def _reset(self):
        self._g = {'x': [], 'y': [], 'z': []}
        self._rps = None
        self._rps_max = 0.0
        self._motor_msgs = 0

    def _on_motor(self, m):
        with self._lock:
            self._rps = [round(d.rps, 4) for d in m.data]
            self._rps_max = max(self._rps_max, max(abs(r) for r in self._rps))

    def _on_imu(self, m):
        with self._lock:
            self._g['x'].append(m.angular_velocity.x)
            self._g['y'].append(m.angular_velocity.y)
            self._g['z'].append(m.angular_velocity.z)

    def _on_status(self, m):
        with self._lock:
            self._status = list(m.data)

    def wait_chassis_alive(self, timeout_s=15.0):
        """解锁并**等到**底盘真的被确认在线。

        ⚠️ 为什么不能只发一次 `~/resume` 就往下走：解锁是异步的 ——
        调用返回时状态可能还是 `stopped`（Control Skill 会据此拒绝运动），
        于是【1】报"链断了"，而其实只是**锁还没解开**。
        这里改成**轮询到状态真的变好为止**，把时序问题从根上消掉。
        """
        self.fire(self.motor_resume)
        end = time.monotonic() + timeout_s
        last = None
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                st = self._status
            if st:
                last = st[0]
                if st[0] in (0.0, 1.0):        # 0=ok, 1=no_cmd：链路在线
                    return True, st
        return False, last

    def snapshot(self):
        with self._lock:
            spans = {k: (max(v) - min(v)) if len(v) > 1 else 0.0
                     for k, v in self._g.items()}
            return (spans, max(spans.values()), self._rps, self._rps_max,
                    self._motor_msgs)

    def spinning(self, span_max):
        """相对判据：既要有绝对动静，也要明显超过本次实测的基线。"""
        return span_max > max(ABS_FLOOR, BASELINE_RATIO * self.baseline)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def phase(self, seconds):
        with self._lock:
            self._reset()
        self.spin_for(seconds)
        return self.snapshot()

    def fmt(self, spans, span_max, rps, rps_max):
        return (f'IMU 各轴 ' + ' '.join(f'{k}={v:.4f}' for k, v in spans.items())
                + f'｜最大 {span_max:.4f}（基线 {self.baseline:.4f}）'
                f'｜窗口内最大 |rps| {rps_max:.4f}｜最后轮速 {rps}')

    def call(self, cli, req, timeout=20.0):
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def fire(self, cli):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())


def main():
    rclpy.init()
    node = SuspendedTester()
    ok = True
    results = []

    def check(name, passed, detail):
        nonlocal ok
        ok = ok and passed
        results.append((name, passed, detail))
        print(f'  {"✅" if passed else "❌"} {name}：{detail}')

    try:
        print()
        print('=' * 74)
        print('  架空运动验收 —— 本项目的链第一次真的驱动底盘（四轮离地）')
        print('=' * 74)
        print()

        node.spin_for(1.5)
        # ⚠️ 上一轮结束时 `finally` 会把 Motor Driver 锁存（那是刻意的兜底），
        #    所以本轮开头先解锁 —— 而且要**等到状态真的变好**再往下走。
        alive, st = node.wait_chassis_alive()
        if not alive:
            print(f'  ⛔ **底盘未确认在线**（motor status = {st}）—— 无法测。')
            print('     查：Motor Driver 在跑吗？dry_run 是 false 吗？')
            print('     （状态码 0=ok 1=no_cmd 2=telemetry_lost 3=stopped 4=rearm_required）')
            return 3
        print(f'（底盘已确认在线：motor status[0] = {st[0]}）')

        print('【基线】2.5 s 不动 —— 这条基线是后面所有"在转吗"判据的参照')
        spans, smax, rps, rmax, n = node.phase(2.5)
        node.baseline = smax
        print(f'    {node.fmt(spans, smax, rps, rmax)}')
        if smax > QUIET_BASELINE_MAX:
            # ★ 拒绝出结论，而不是把门槛调松去迎合。
            print()
            print(f'  ⛔ **基线不安静**（{smax:.4f} > {QUIET_BASELINE_MAX}）——'
                  f'车在晃，本次数据无法判读。')
            print('     请把车**扶稳或挂稳**（手尽量别动），然后重跑。')
            print('     ⚠️ 阈值调松只会把"假失败"换成"假成功"，两种都是坏数据。')
            print()
            return 2
        print(f'    判据：跨度 > max({ABS_FLOOR}, {BASELINE_RATIO} × {smax:.4f} = '
              f'{BASELINE_RATIO * smax:.4f})')
        print()

        print('【1】经 Control Skill 前进 0.2 m —— 轮子应当真的转起来')
        req = MoveRelative.Request()
        req.x = 0.2
        req.y = 0.0
        with node._lock:
            node._reset()
        res = node.call(node.move_cli, req)
        spans, smax, rps, rmax, n = node.snapshot()
        print(f'    Control Skill：{getattr(res, "message", "无响应")}')
        print(f'    {node.fmt(spans, smax, rps, rmax)}')
        check('Control Skill 报告成功', bool(getattr(res, 'success', False)),
              f'success={getattr(res, "success", None)}')
        check('★ 电机确实转了（IMU 最大跨度）', node.spinning(smax),
              f'{smax:.4f}（基线 {node.baseline:.4f}）')
        check('轮速符号符合前进（motor1/2 正、motor3/4 负）',
              rmax > 0 and rps is not None and rps[0] > 0 and rps[1] > 0
              and rps[2] < 0 and rps[3] < 0, f'{rps}')
        node.spin_for(1.0)
        print()

        print('【2】★ 运动中 `~/stop` —— 停车在底盘侧真的生效吗')
        print('     （发一个 1.0 m 的长请求，0.8 s 后叫停）')
        req = MoveRelative.Request()
        req.x = 1.0
        req.y = 0.0
        with node._lock:
            node._reset()
        fut = node.move_cli.call_async(req)
        node.spin_for(0.8)
        spans_mid, smax_mid, _, rmax_mid, _ = node.snapshot()
        node.fire(node.ctrl_stop)
        node.spin_for(0.7)              # 让"停"生效（Motor Driver 的 cmd_timeout 是 0.5s）
        with node._lock:
            node._reset()               # ⚠️ **必须重置** —— 否则"叫停后"的窗口里
                                        #    还留着叫停**之前**的轮速，"归零"永远判不过
        node.spin_for(1.5)
        spans_after, smax_after, rps_after, rmax_after, _ = node.snapshot()
        print(f'    叫停前：最大跨度 {smax_mid:.4f}｜窗口内最大 |rps| {rmax_mid:.4f}')
        print(f'    叫停后：最大跨度 {smax_after:.4f}｜窗口内最大 |rps| {rmax_after:.4f}'
              f'｜最后轮速 {rps_after}')
        check('叫停前确实在转', node.spinning(smax_mid), f'{smax_mid:.4f}')
        # ⚠️ 判"停"要用**轮速指令归零**，不能用 IMU 跨度。
        #    悬空时 IMU 测的是"机身晃动"，运动激起的摆动在电机停了之后还会继续荡 ——
        #    用跨度判"停"会把"还在荡"误读成"还在转"（这一版就踩过）。
        #    IMU 能回答"有没有力矩在作用"，回答不了"停了没有"。
        check('★ 叫停后轮速指令归零', rmax_after < 1e-6, f'{rmax_after:.4f}')
        print(f'    （IMU 跨度 叫停前 {smax_mid:.4f} → 叫停后 {smax_after:.4f}；'
              f'悬空时它会继续荡，仅作参考）')
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        if fut.done():
            r = fut.result()
            print(f'    被中止的请求返回：success={r.success}｜{r.message}')
            check('被中止的请求**如实报失败**', r.success is False, r.message)
        print()

        print('【3】★ Motor Driver 的 `~/stop` 锁存 —— 另一道锁，也要在底盘侧生效')
        req = MoveRelative.Request()
        req.x = 1.0
        req.y = 0.0
        with node._lock:
            node._reset()
        node.move_cli.call_async(req)
        node.spin_for(0.8)
        _, smax_mid, _, rmax_mid, _ = node.snapshot()
        node.fire(node.motor_stop)
        node.spin_for(0.7)
        with node._lock:
            node._reset()               # 同上：窗口必须从"停稳之后"才开始
        node.spin_for(1.5)
        _, smax_after, rps_after3, rmax_after, _ = node.snapshot()
        print(f'    锁存前：跨度 {smax_mid:.4f}｜窗口内最大 |rps| {rmax_mid:.4f}')
        print(f'    锁存后：跨度 {smax_after:.4f}｜窗口内最大 |rps| {rmax_after:.4f}'
              f'｜最后轮速 {rps_after3}')
        # 同【2】：判"停"看轮速指令，不看 IMU 跨度。
        check('★ Motor 锁存后轮速指令归零', rmax_after < 1e-6, f'{rmax_after:.4f}')
        req2 = MoveRelative.Request()
        req2.x = 0.1
        req2.y = 0.0
        res2 = node.call(node.move_cli, req2, timeout=6.0)
        print(f'    锁存期间再发运动：{getattr(res2, "message", "无响应")}')
        check('锁存期间拒绝新运动', getattr(res2, 'success', True) is False,
              f'success={getattr(res2, "success", None)}')
        node.fire(node.ctrl_stop)
        node.fire(node.motor_resume)
        node.spin_for(1.5)
        print()

        print('【4】完整链路：文本命令 → 路由器 → 网关 → Control Skill → 轮子')
        msg = String()
        msg.data = '向前走 0.3 米'
        with node._lock:
            node._reset()
        node.text_pub.publish(msg)
        node.spin_for(4.0)
        spans, smax, rps, rmax, n = node.snapshot()
        print(f'    {node.fmt(spans, smax, rps, rmax)}')
        check('★ 一条中文命令真的让轮子转了', node.spinning(smax),
              f'{smax:.4f}（基线 {node.baseline:.4f}）')
        check('★ 且确实下达过轮速', rmax > 0.1, f'窗口内最大 |rps| {rmax:.4f}')
        node.spin_for(1.5)
        print()

        print('【收尾】确认系统已回到静止')
        _, smax, rps, rmax, _ = node.phase(2.0)
        check('收尾时静止', not node.spinning(smax) and rmax < 1e-6,
              f'跨度 {smax:.4f}｜最大 |rps| {rmax:.4f}')
        print()

        print('=' * 74)
        failed = [n for n, p, _ in results if not p]
        if failed:
            print(f'  ❌ {len(failed)}/{len(results)} 项未通过：')
            for n in failed:
                print(f'     - {n}')
        else:
            print(f'  ✅ 全部 {len(results)} 项通过')
        print()
        print('  ⚠️ 本工装给的是**物理旁证**。轮子是否真的转了，'
              '最终以**人眼观察**为准（首测即如此定案）。')
        print('  ⚠️ 它验证的是"链能驱动底盘"+ "停得住"，**不是**"走到位了"——')
        print('     位移精度仍需地面实测（那是另一件事）。')
        print('=' * 74)
        print()
        return 0 if ok else 1
    finally:
        try:
            node.fire(node.ctrl_stop)
            node.fire(node.motor_stop)
            node.spin_for(0.5)
        except Exception:                                        # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
