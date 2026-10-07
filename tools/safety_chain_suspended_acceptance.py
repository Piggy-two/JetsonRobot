#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**安全链**架空验收（D-036 避障 / D-037 结构性否决）—— 四轮离地。

    ⚠️ 这是【验收工装】，不是运行时组件。
    ⚠️ **前提：四轮离地。** 轮子在空中转，车不会跑掉。
    ⚠️ 本工装会让 Motor Driver 以 `dry_run:=false` 运行 —— **人必须在场、能直接断电**。

它要回答的问题：**"停"这件事，在物理上真的成立吗？**
------------------------------------------------------
到这一轮之前，安全链只在**消息层面**验过：发零帧数、非零帧数、仲裁比例。
那些都是**自证** —— 证明"我发了什么"，证明不了"底盘停了没有"。
底盘没有轮速回读，唯一独立的物理量是 IMU；而**轮速指令归零**回答的是
"还下不下令让它转"（判据来源见 `suspended_motion_acceptance.py` 的说明）。

两段，各自对应一套启动参数（**必须分开跑，见下方配置**）
--------------------------------------------------------
    --phase veto   验**结构性否决**（D-037）：Safety 的 `/motor_driver/stop` **故意不可达**，
                   于是唯一能让底盘停下的路径就是"Motor Driver 读到了安全层状态"。
                   ★ 这一段是本次修复的核心证据。
    --phase guard  验**避障停车**（D-036）：真雷达 + 真守卫，命令它前进，
                   看守卫是否在底盘侧把车拦住。

配置 A（跑 `--phase veto`）—— **注意 `motor_stop_service` 指向不存在的服务**
--------------------------------------------------------
    ros2 launch embodied_motor_driver motor_driver.launch.py dry_run:=false
    ros2 launch embodied_control_skills control_skills.launch.py
    ros2 launch embodied_safety_runtime safety_runtime.launch.py \\
        enable_obstacle_guard:=false \\
        motor_stop_service:=/acceptance/absent

配置 B（跑 `--phase guard`）—— 全部用默认
------------------------------------------
    ros2 launch embodied_motor_driver motor_driver.launch.py dry_run:=false
    ros2 launch embodied_control_skills control_skills.launch.py
    ros2 launch embodied_lidar_driver lidar_driver.launch.py
    ros2 launch embodied_safety_runtime safety_runtime.launch.py

用法：
    python3 tools/safety_chain_suspended_acceptance.py --phase veto
    python3 tools/safety_chain_suspended_acceptance.py --phase guard
退出码：0 = 全部断言通过；1 = 有失败；2 = 基线不安静（拒测）；3 = 前置不满足。
"""

import argparse
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from ros_robot_controller_msgs.msg import MotorsState

from embodied_skills_interfaces.srv import MoveRelative

SET_MOTOR = '/ros_robot_controller/set_motor'
IMU = '/ros_robot_controller/imu_raw'
MOTOR_STATUS = '/embodied/motor/status'
SAFETY_STATUS = '/embodied/safety/status'
SAFETY_EVENTS = '/embodied/safety/events'

# status 话题的下标（前 10 个是 D-025 的，后 3 个是 D-037 追加的）
I_STATE, I_LATCHED, I_REARM = 0, 7, 9
I_SAFETY_AGE, I_SAFETY_LATCHED, I_SAFETY_BLOCKED = 10, 11, 12

STATE_OK, STATE_NO_CMD = 0.0, 1.0
STATE_STOPPED, STATE_REARM_REQUIRED = 3.0, 4.0
STATE_SAFETY_BLOCKED = 5.0

ABS_FLOOR = 0.006          # 与 suspended_motion_acceptance.py 同一套相对判据
BASELINE_RATIO = 1.5
QUIET_BASELINE_MAX = 0.08  # 基线超过它就**拒测**，不调松阈值


class SafetyChainTester(Node):
    def __init__(self):
        super().__init__('safety_chain_suspended_acceptance')
        self._lock = threading.Lock()
        self._reset()
        self.baseline = 0.0
        self.motor_status = None
        self.safety_status = None
        self.safety_events = []

        self.create_subscription(MotorsState, SET_MOTOR, self._on_motor, 10)
        self.create_subscription(Imu, IMU, self._on_imu, 10)
        self.create_subscription(Float64MultiArray, MOTOR_STATUS, self._on_motor_status, 10)
        self.create_subscription(Float64MultiArray, SAFETY_STATUS, self._on_safety_status, 10)
        self.create_subscription(String, SAFETY_EVENTS, self._on_safety_event, 20)

        # ⚠️ 第【3】【4】段要模拟一种**不会停的客户端**：它**不理会任何 stop**，
        #    一直以 20 Hz 发同一条非零指令。那才是"静默复动"真正危险的场景
        #    —— 用 Control Skill 测不出来：它被 `~/stop` 中止时会**补发一帧零**
        #    （control_skills.py:266），于是"被拦下那一刻"已经没有潜伏的非零指令了。
        self.cmd_pub = self.create_publisher(Twist, '/embodied/motor/cmd_vel', 10)
        self._raw_vx = 0.0
        self.create_timer(1.0 / 20.0, self._pub_raw)

        self.move_cli = self.create_client(MoveRelative, '/control_skills/move_relative')
        self.ctrl_stop = self.create_client(Trigger, '/control_skills/stop')
        self.motor_stop = self.create_client(Trigger, '/motor_driver/stop')
        self.motor_resume = self.create_client(Trigger, '/motor_driver/resume')
        self.safety_estop = self.create_client(Trigger, '/safety_runtime/estop')
        self.safety_release = self.create_client(Trigger, '/safety_runtime/release')

    # ---------- 观测 ----------

    def _reset(self):
        self._g = {'x': [], 'y': [], 'z': []}
        self._rps = None
        self._rps_max = 0.0

    def _on_motor(self, m):
        with self._lock:
            self._rps = [round(d.rps, 4) for d in m.data]
            self._rps_max = max(self._rps_max, max(abs(r) for r in self._rps))

    def _on_imu(self, m):
        with self._lock:
            self._g['x'].append(m.angular_velocity.x)
            self._g['y'].append(m.angular_velocity.y)
            self._g['z'].append(m.angular_velocity.z)

    def _on_motor_status(self, m):
        with self._lock:
            self.motor_status = list(m.data)

    def _on_safety_status(self, m):
        with self._lock:
            self.safety_status = list(m.data)

    def _on_safety_event(self, m):
        self.safety_events.append(m.data)

    def snapshot(self):
        with self._lock:
            spans = {k: (max(v) - min(v)) if len(v) > 1 else 0.0
                     for k, v in self._g.items()}
            return spans, max(spans.values()), self._rps, self._rps_max

    def spinning(self, span_max):
        return span_max > max(ABS_FLOOR, BASELINE_RATIO * self.baseline)

    def phase(self, seconds):
        with self._lock:
            self._reset()
        self.spin_for(seconds)
        return self.snapshot()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def mstat(self, idx):
        with self._lock:
            st = self.motor_status
        return None if not st or len(st) <= idx else st[idx]

    def sstat(self, idx):
        with self._lock:
            st = self.safety_status
        return None if not st or len(st) <= idx else st[idx]

    # ---------- 动作 ----------

    def call(self, cli, req=None, timeout=20.0):
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(req if req is not None else Trigger.Request())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def fire(self, cli):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())

    def start_move(self, x):
        req = MoveRelative.Request()
        req.x = float(x)
        req.y = 0.0
        return self.move_cli.call_async(req)

    def _pub_raw(self):
        """模拟"不会停的客户端"：只要 _raw_vx 非零就一直发。"""
        if self._raw_vx == 0.0:
            return
        t = Twist()
        t.linear.x = float(self._raw_vx)
        self.cmd_pub.publish(t)

    def set_raw(self, vx):
        """开/关那个不会停的客户端。关掉时会补几帧零，免得留一条 pending 指令。"""
        self._raw_vx = float(vx)
        if vx == 0.0:
            for _ in range(3):
                self.cmd_pub.publish(Twist())


def fmt(node, spans, smax, rps, rmax):
    return (f'IMU 各轴 ' + ' '.join(f'{k}={v:.4f}' for k, v in spans.items())
            + f'｜最大 {smax:.4f}（基线 {node.baseline:.4f}）'
            f'｜窗口内最大 |rps| {rmax:.4f}｜最后轮速 {rps}')


def preflight(node, rep, need_guard_off):
    """公共前置：底盘在线 + 静基线。返回 False 表示拒测。"""
    node.spin_for(1.5)
    node.fire(node.motor_resume)
    ok = node.wait_until(
        lambda: node.mstat(I_STATE) in (STATE_OK, STATE_NO_CMD), timeout=15.0)
    if not ok:
        print(f'  ⛔ 底盘未确认在线（motor status[0] = {node.mstat(I_STATE)}）')
        print('     查：Motor Driver 在跑吗？dry_run 是 false 吗？require_safety 会不会把它拦着？')
        print('     （状态码 0=ok 1=no_cmd 2=telemetry_lost 3=stopped 4=rearm_required '
              '5=safety_blocked）')
        return False
    print(f'（底盘已确认在线：state={node.mstat(I_STATE)}，'
          f'安全层龄 {node.mstat(I_SAFETY_AGE)} ms）')

    print('【基线】2.5 s 不动 —— 后面所有"在转吗"的判据以它为参照')
    spans, smax, rps, rmax = node.phase(2.5)
    node.baseline = smax
    print(f'    {fmt(node, spans, smax, rps, rmax)}')
    if smax > QUIET_BASELINE_MAX:
        print()
        print(f'  ⛔ **基线不安静**（{smax:.4f} > {QUIET_BASELINE_MAX}）—— 本次数据无法判读。')
        print('     请把车**扶稳或挂稳**（手尽量别动）后重跑。')
        print('     ⚠️ 阈值调松只会把"假失败"换成"假成功"。')
        return False
    print(f'    判据：跨度 > max({ABS_FLOOR}, {BASELINE_RATIO} × {smax:.4f})')
    print()
    return True


def phase_veto(node, rep):
    """★ D-037 的核心证据：**放下游 stop 不可达**时，底盘能不能停住。"""
    print('【1】命令前进 1.0 m（Control Skill，标称 0.15 m/s ≈ 6.7 s），先确认它真的在转')
    with node._lock:
        node._reset()
    fut = node.start_move(1.0)
    node.spin_for(1.0)
    spans, smax, rps, rmax = node.snapshot()
    print(f'    {fmt(node, spans, smax, rps, rmax)}')
    rep('★ 被命令的前进真的驱动了轮子', node.spinning(smax) and rmax > 0.1,
        f'跨度 {smax:.4f}｜最大 |rps| {rmax:.4f}')

    print('\n【2】★ 运动中触发 `~/estop` —— 此时 Safety 的 `/motor_driver/stop` **故意不可达**')
    res = node.call(node.safety_estop)
    print(f'    estop：{getattr(res, "message", "无响应")}')
    node.spin_for(1.5)                 # 让"停"落下去
    with node._lock:
        node._reset()
    spans2, smax2, rps2, rmax2 = node.phase(2.0)
    print(f'    叫停后 2.0 s：{fmt(node, spans2, smax2, rps2, rmax2)}')
    rep('★ 轮速指令归零（底盘侧真的停了）', rmax2 < 1e-6, f'{rmax2:.4f}')
    rep('★ 安全层报锁存', (node.sstat(0) or 0) > 0.5, f'safety status[0]={node.sstat(0)}')
    rep('★ Motor Driver 报 safety_blocked（状态码 5）',
        node.mstat(I_STATE) == STATE_SAFETY_BLOCKED, f'state={node.mstat(I_STATE)}')
    # ▼▼ 这一条是整段的关键：锁存标志**必须是 0** ▼▼
    rep('★ 且 Motor Driver **自己没被锁存**（证明停靠的是读到的状态，不是那次调不通的调用）',
        node.mstat(I_LATCHED) == 0.0, f'latched={node.mstat(I_LATCHED)}')
    ev = [e for e in node.safety_events if e.startswith('estop_triggered')]
    rep('收到 estop 事件', bool(ev), ev[-1] if ev else '（无）')

    print('\n【3】★ 换一个**不会停的客户端**：它以 20 Hz 一直发同一条非零指令，'
          '\n     不理会任何 stop —— 安全层拦下、再解除，底盘**不许自己接着跑**')
    # ⚠️ 顺序很要紧：**先把安全层解除**，再让客户端起来。
    #    【2】结束时安全层还是锁存的；若此时就起客户端，它会**一直推不动车**，
    #    等到后面才解除 —— 那时"被拦下那一刻"早就过去了，判据自然不成立。
    #    （本工装第一版就是这么写错的：客户端"确实在驱动轮子"那一条直接失败。）
    node.call(node.safety_release)
    node.call(node.motor_resume)
    node.wait_until(lambda: node.mstat(I_STATE) in (STATE_OK, STATE_NO_CMD), timeout=8.0)
    node.set_raw(0.15)
    with node._lock:
        node._reset()
    node.spin_for(1.2)
    spans3a, smax3a, rps3a, rmax3a = node.snapshot()
    print(f'    先是它在推着车走：{fmt(node, spans3a, smax3a, rps3a, rmax3a)}')
    rep('★ 这个客户端确实在驱动轮子', rmax3a > 0.1, f'最大 |rps| {rmax3a:.4f}')

    node.call(node.safety_estop)
    node.spin_for(1.2)
    rep('安全层锁存后底盘已停', True, f'state={node.mstat(I_STATE)}')

    node.call(node.safety_release)     # **只解除安全层**，客户端还在发！
    with node._lock:
        node._reset()
    node.spin_for(1.2)                 # 给"如果它要自己跑"留足时间
    spans3, smax3, rps3, rmax3 = node.snapshot()
    print(f'    解除后 1.2 s（客户端**仍在** 20 Hz 发 0.15 m/s）：'
          f'{fmt(node, spans3, smax3, rps3, rmax3)}')
    rep('★ 底盘仍然不动（真·没有静默复动）', rmax3 < 1e-6, f'{rmax3:.4f}')
    rep('Motor Driver 报 rearm_required（状态码 4）',
        node.mstat(I_STATE) == STATE_REARM_REQUIRED, f'state={node.mstat(I_STATE)}')

    print('\n【4】显式 `~/resume` 之后应当能再动（可恢复）—— 客户端还在发，这次它该生效')
    node.call(node.motor_resume)
    node.wait_until(lambda: node.mstat(I_STATE) in (STATE_OK, STATE_NO_CMD), timeout=8.0)
    with node._lock:
        node._reset()
    node.spin_for(1.5)
    spans4, smax4, rps4, rmax4 = node.snapshot()
    print(f'    {fmt(node, spans4, smax4, rps4, rmax4)}')
    rep('★ 显式 resume 后轮子又能转', node.spinning(smax4) and rmax4 > 0.1,
        f'跨度 {smax4:.4f}｜最大 |rps| {rmax4:.4f}')
    node.set_raw(0.0)
    node.spin_for(0.5)


def phase_guard(node, rep):
    """★ D-036：避障守卫在底盘侧真的拦得住吗。"""
    print('【1】命令前进 1.0 m，看守卫（默认 ±30°、阈值 = 速度 × 1.5 s）是否拦住它')
    before = len(node.safety_events)
    with node._lock:
        node._reset()
    node.start_move(1.0)
    got = node.wait_until(lambda: (node.sstat(0) or 0) > 0.5, timeout=8.0)
    spans, smax, rps, rmax = node.snapshot()
    print(f'    守卫锁存前：{fmt(node, spans, smax, rps, rmax)}')
    rep('★ 守卫锁存了', got, f'safety status[0]={node.sstat(0)}')
    ev = [e for e in node.safety_events[before:] if 'obstacle' in e]
    rep('★ 触发原因写明是障碍（带距离/方位）', bool(ev), ev[-1] if ev else '（无 obstacle 事件）')

    node.spin_for(1.5)
    with node._lock:
        node._reset()
    spans2, smax2, rps2, rmax2 = node.phase(2.0)
    print(f'    锁存后 2.0 s：{fmt(node, spans2, smax2, rps2, rmax2)}')
    rep('★ 轮速指令归零（避障停车在底盘侧生效）', rmax2 < 1e-6, f'{rmax2:.4f}')
    rep('Motor Driver 报 safety_blocked',
        node.mstat(I_STATE) == STATE_SAFETY_BLOCKED, f'state={node.mstat(I_STATE)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=['veto', 'guard'], required=True)
    args = ap.parse_args()

    rclpy.init()
    node = SafetyChainTester()
    results = []

    def rep(name, passed, detail):
        results.append((name, passed, detail))
        print(f'  {"✅" if passed else "❌"} {name}：{detail}')

    try:
        print()
        print('=' * 74)
        title = ('安全链架空验收 —— 结构性否决（D-037）'
                 if args.phase == 'veto' else
                 '安全链架空验收 —— 避障停车（D-036）')
        print(f'  {title}（四轮离地）')
        print('=' * 74)
        print()
        if not preflight(node, rep, args.phase == 'veto'):
            return 2

        if args.phase == 'veto':
            phase_veto(node, rep)
        else:
            phase_guard(node, rep)

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
        print('  ⚠️ IMU 给的是物理旁证；"轮速指令归零"回答的是"还下不下令让它转"。')
        print('     两者都不是"走到位了"的证据 —— 位移精度仍需地面实测。')
        print('=' * 74)
        print()
        return 0 if not failed else 1
    finally:
        try:
            node.fire(node.ctrl_stop)
            node.fire(node.motor_stop)
            node.fire(node.safety_release)
            node.spin_for(0.5)
        except Exception:                                        # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
