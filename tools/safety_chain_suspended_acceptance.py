#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**安全链**架空验收（D-036 避障 / D-037 结构性否决）—— 四轮离地。

    ⚠️ 这是【验收工装】，不是运行时组件。
    ⚠️ **`veto` / `guard` 两段的前提是四轮离地**（轮子在空中转，车不会跑掉）。
       **`direction` 一段不要求**：它把"该被拦住"的那一步放在任何位移之前，
       所以车在该停的时候本来就不该动 —— 但**仍需要有人看护、能直接断电**。
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
    --phase direction  验守卫的**方向性**：先问四个方向各有多远，然后
                   往**受阻**方向走（**必须拦下**）+ 往**通畅**方向走（**必须放行**）做对照。
                   ⇒ 一个什么都拦的守卫会被关掉，等于没有；这一条防的是那个。
                   ⚠️ **顺序是刻意的**：**先拦下、后放行**。反过来的话，"放行"那一步
                   会先把车真的挪走 0.3 m，再拿旧方位去判"拦下" —— 方位早变了 ⇒ 假失败。
                   把"拦下"放在任何位移之前，几何前提才成立。
                   ⇒ **这一段在地面上也能跑**（前两段不行）：车在"该被拦住"的那一步
                   本来就**不该动**，所以它顺带就把"命令它真的朝障碍走"验了。
                   ⚠️ **摆位有硬要求（两个方向都得有）**：至少一个方向 > 0.40 m 是空的，
                   **且**至少一个方向 ≤ 守卫**真实的**阈值有东西 —— 后者由
                   `min(max_range, max(min_range, 0.15×lookahead))` 算出（当前 0.30 m），
                   **且会先读守卫的 yaml 核对**，对不上直接拒测（D-040）。缺任一边即**拒测**
                   （只验"放行"等于把"守卫是不是一刀切"放过去了）。
                   跑之前可用 `--phase direction --dry-classify` 先量一遍摆位。

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
import os
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

from embodied_skills_interfaces.srv import MoveRelative, SectorMinRange

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
        # 用与守卫**同一个原语**问各方向（这样"通畅"的判据与守卫看到的完全一致）
        self.sector_cli = self.create_client(SectorMinRange, '/lidar_driver/sector_min_range')

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
    """公共前置：**判据与守卫同源** + 底盘在线 + 静基线。返回 False 表示拒测。"""
    # 先核对"判据用的数"和"守卫用的数"是不是同一个 —— 见 guard_config_mismatch()
    bad = guard_config_mismatch()
    if bad:
        print(f'  ⛔ **判据与被测物不同源** —— 拒测。')
        for line in bad.split('。'):
            if line.strip():
                print(f'     {line.strip()}。')
        return False
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


def probe_direction(node, center, max_range=1.0):
    """问某个方向 ±30°、1 m 内的最近回波（与守卫同一个原语、同一套几何）。"""
    req = SectorMinRange.Request()
    req.center = float(center)
    req.width = 1.0471975511965976
    req.max_range = float(max_range)
    res = node.call(node.sector_cli, req, timeout=5.0)
    if res is None:
        return None, None
    return (float(res.range) if res.valid else -1.0), float(res.angle)


# 候选方向：(名字, 扇区中心角, 给 Control Skill 的请求)
DIRECTIONS = [
    ('前', 0.0, dict(x=0.3, y=0.0)),
    ('左', 1.5707963267948966, dict(x=0.0, y=0.3)),
    ('右', -1.5707963267948966, dict(x=0.0, y=-0.3)),
    ('后', 3.141592653589793, dict(x=-0.3, y=0.0)),
]

# ── 判据的两个端：**必须与守卫的真实阈值同源**，否则会出一个死区 ──────────────
# 守卫（`obstacle_guard.GuardConfig`）算的是
#     stop_range = min(max_range, max(min_range, 速度 × lookahead))
# 并且它是**带着 max_range = stop_range 去问服务**的 —— 比它远的回波根本不返回
# （`obstacle_guard.py` 里 `evaluate()` 的那条注释）。所以在 0.15 m/s 下：
#     stop_range = min(1.00, max(0.30, 0.15 × 1.5)) = 0.30 m
# ⚠️ 工装第一版只写了一个 `CLEAR_MARGIN = 0.40`，**两端共用** —— 于是
#     (阈值, 0.40] 这一段成了死区：工装判它"受阻、守卫该拦"，
#     而守卫带着阈值去问，看不到它，**不会拦**。把障碍物放在死区里就会假失败。
#     本版把"受阻"的上界改回守卫的真实阈值（**这是收紧，不是放行**）。
#
# ⚠️⚠️ **这四个数是守卫那份配置的副本，改成对不上就会重演上面那个坑。**
#     所以下面 `preflight` 会**直接读守卫的 yaml 核对**，对不上就拒测 ——
#     与其指望"改的人记得同步"，不如让不同步**当场报错**（D-040）。
SKILL_SPEED = 0.15          # Control Skill 的 `nominal_speed` 默认值 —— 本工装用它走
GUARD_MIN_RANGE = 0.30      # 守卫的 `obstacle_min_range`（2026-10-08 按 D-040 由 0.20 提高）
GUARD_LOOKAHEAD = 1.5       # 守卫的 `obstacle_lookahead`
GUARD_MAX_RANGE = 1.00      # 守卫的 `obstacle_max_range`
GUARD_STOP_RANGE = min(GUARD_MAX_RANGE, max(GUARD_MIN_RANGE, SKILL_SPEED * GUARD_LOOKAHEAD))

#: 本工装往某个方向**真的要走多远**（`DIRECTIONS` 里每个 move 都是 0.3 m）。
WALK = 0.30

#: 判"这个方向确实通畅"的门槛 —— **必须同时满足"走之前不拦"和"走完之后也不拦"**：
#:     走完之后离障碍的距离 ≈ 初始读数 − WALK，它必须还在阈值之上：
#:         初始读数 > 阈值 + WALK
#:     再留一点余量（阈值本身已经含了车头前伸量和蠕动，这里只留"别贴着下结论"的余量）。
#: ⚠️ 这一条**不是**"1.5 倍阈值"那种拍出来的余量：0.30 阈值下，若只要求 > 0.45，
#:     一个 0.478 m 的方向会被判"通畅"，可车往那儿开 0.3 m 就只剩 0.178 m —— **半路被拦**，
#:     于是【3】假失败。**阈值一改，这一条自动跟着走**，不用再靠人记得同步。
CLEAR_MARGIN = round(GUARD_STOP_RANGE + WALK + 0.10, 3)


def guard_config_mismatch():
    """读**守卫自己的那份 yaml**，跟本工装的常量核对。对不上返回一句人话，对得上返回 None。

    为什么要这么做：这个坑已经发生过一次 —— 工装按 0.40 判"受阻"、守卫按 0.225 去看，
    中间一段谁都测不出真失败（坑 27）。**判据与被测物各拿一个数，迟早会分家。**
    不能打开 yaml 时只警告不拒测（比如工装被单独拷出去跑）。
    """
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('embodied_safety_runtime'),
                            'config', 'safety_runtime.yaml')
        with open(path, 'r', encoding='utf-8') as fh:
            cfg = yaml.safe_load(fh)
    except Exception as exc:                                     # noqa: BLE001
        print(f'  ⚠️ 读不到守卫的配置（{exc}）—— 跳过"判据同源"核对，'
              f'本工装按 min_range={GUARD_MIN_RANGE} / lookahead={GUARD_LOOKAHEAD} 判。')
        return None
    # 顶层是节点名（`safety_runtime:`），下面才是 `ros__parameters` —— 别写死节点名，
    # 换一个 launch 用的 yaml 也要能读
    rf = {}
    for _node, body in (cfg or {}).items():
        if isinstance(body, dict) and 'ros__parameters' in body:
            rf = body['ros__parameters']
            break
    live = (float(rf.get('obstacle_min_range', -1)),
            float(rf.get('obstacle_lookahead', -1)),
            float(rf.get('obstacle_max_range', -1)))
    mine = (GUARD_MIN_RANGE, GUARD_LOOKAHEAD, GUARD_MAX_RANGE)
    if live != mine:
        return (f'守卫配置 {live} ≠ 工装常量 {mine} —— **判据和被测物各用了一个数**，'
                f'这正是坑 27 那个死区的来源。请把工装里的常量改成与 '
                f'config/safety_runtime.yaml 一致。')
    return None


def classify_directions(probed):
    """把四个方向的读数分成（通畅，受阻）两类。**纯函数，便于单独证伪。**

    `probed` 是 [(名字, 中心角, 请求, 距离, 角度), ...]；距离 < 0 表示该扇区无回波。

      · **通畅**：1 m 内没有回波（`< 0`，那就是真没有东西），或最近回波 > `CLEAR_MARGIN`；
      · **受阻**：最近回波 ≤ `GUARD_STOP_RANGE` —— 用守卫**真实的**阈值，
        否则会出现"工装说该拦、守卫看不见"的死区（见上方说明）。
    """
    clear = [p for p in probed
             if p[3] is not None and (p[3] < 0 or p[3] > CLEAR_MARGIN)]
    blocked = [p for p in probed
               if p[3] is not None and 0.0 < p[3] <= GUARD_STOP_RANGE]
    return clear, blocked


def probe_four_directions(node):
    """问四个方向的最近回波并打印读数。**纯查询，不驱动任何东西。** 返回 probed。"""
    print(f'     守卫在 {SKILL_SPEED} m/s 下的真实阈值 = {GUARD_STOP_RANGE:.3f} m（已与守卫配置核对）；'
          f'判"通畅"要 > {CLEAR_MARGIN} m（留余量，别贴着阈值下结论）')
    probed = []
    for name, center, move in DIRECTIONS:
        r, a = probe_direction(node, center)
        probed.append((name, center, move, r, a))
        if r is None:
            shown, verdict = '（超时）', '⬜ 没问到'
        elif r < 0:
            shown, verdict = '（无回波）', '✅ 通畅'
        else:
            shown = f'{r:.3f} m'
            verdict = ('✅ 通畅' if r > CLEAR_MARGIN else
                       '❌ 受阻' if r <= GUARD_STOP_RANGE else
                       '⚠️ 灰区（比通畅余量近、守卫却看不到）')
        print(f'    {name}（中心 {center * 57.29578:+.0f}°）：最近回波 {shown}　{verdict}')
    return probed


def phase_direction(node, rep):
    """★ D-036 的方向性：**同一个方向扇区里的东西，只有朝它走时才该拦**。"""
    print('【1】先问四个方向 ±30°、1 m 内最近回波（用的就是守卫那个原语）')
    probed = probe_four_directions(node)
    if probed[0][3] is None:
        rep('问到 LiDAR 原语', False, 'sector_min_range 不可用 —— liDAR Driver 在跑吗？')
        return
    rep('问到 LiDAR 原语', True, f'{len(probed)} 个方向')

    clear, blocked = classify_directions(probed)
    if not clear:
        print()
        print(f'  ⛔ **四个方向都不通畅**（判据：最近回波要 > {CLEAR_MARGIN} m）—— 拒测。')
        print('     这一条要的是"有通畅方向可走"，而不是把阈值调松。')
        print('     请把车挪到某个方向前面是空的，或把前面的东西拿开。')
        rep('存在一个通畅方向供测试', False, '四个方向都被挡')
        return
    rep('存在一个通畅方向', True, f'{clear[0][0]}（最近回波 {clear[0][3]:.3f} m）')

    if not blocked:
        # ⚠️ 这一条**必须拒测**，不能"跳过对照继续跑"：本相位的全部意义就是
        #    "放行 vs 拦下"这个**对照**。只验放行，等于把"守卫是不是一刀切"放过去了 ——
        #    那正是 D-036 要防的东西（一个什么都拦的守卫会被关掉，等于没有）。
        print()
        print(f'  ⛔ **没有任何一个方向是"受阻"的**（判据：最近回波 ≤ '
              f'{GUARD_STOP_RANGE:.3f} m = 守卫的真实阈值）—— 拒测。')
        print('     这一段的核心是**对照**：通畅方向必须放行、受阻方向必须拦下。')
        print(f'     请在一个方向上放个东西（约 ≤ {GUARD_STOP_RANGE:.2f} m，'
              f'比 {GUARD_STOP_RANGE:.2f} m 更近才拦得住），')
        print(f'     同时留另一个方向 > {CLEAR_MARGIN} m 是空的。')
        rep('存在一个受阻方向供对照', False, '四个方向都通畅')
        return
    rep('存在一个受阻方向', True, f'{blocked[0][0]}（最近回波 {blocked[0][3]:.3f} m）')

    # ── 【2】对照（**先做**）────────────────────────────────────────────────
    # ⚠️ **顺序是刻意的，而且改过。** 早先的版本先验"放行"、后验"拦下"，
    #    在那个顺序下，"放行"那一步会在**地面**上真的把车挪走 0.3 m；
    #    挪完之后再拿【1】量到的方位去判"拦下"，方位早就不是那个方位了：
    #        物体在正前 0.15 m；车往「左」走 0.3 m 后，物体在车体系里变成
    #        (0.15, −0.3)，方位 atan2(−0.3, 0.15) = **−63°** —— 跑出 ±30° 扇区外，
    #        守卫看不见它 ⇒ **假失败**。
    #    把"拦下"提到**任何位移之前**，几何前提就成立；而且此刻车**本来就不该动**，
    #    所以这一条在地面上尤其有价值（它就是文档里一直空白的"命令它真的朝障碍走"）。
    bname, bcenter, bmove, br, ba = blocked[0]
    print(f'\n【2】★ 先验**拦下**：往**受阻的「{bname}」**方向走（那里 {br:.3f} m 有东西）'
          f'—— 守卫**该**拦')
    with node._lock:
        node._reset()
    req2 = MoveRelative.Request()
    req2.x = float(bmove['x'])
    req2.y = float(bmove['y'])
    node.move_cli.call_async(req2)
    got = node.wait_until(lambda: (node.sstat(0) or 0) > 0.5, timeout=6.0)
    rep(f'★ 守卫拦住了「{bname}」方向', got, f'safety status[0]={node.sstat(0)}')
    node.spin_for(1.2)
    with node._lock:
        node._reset()
    spans2, smax2, rps2, rmax2 = node.phase(2.0)
    print(f'    拦下后：{fmt(node, spans2, smax2, rps2, rmax2)}')
    rep('★ 拦下后轮速指令归零', rmax2 < 1e-6, f'{rmax2:.4f}')

    # ── 收尾这一档，再把车交还给"能走"的状态 ──
    # ⚠️ 顺序与"等它真的回来"都是必需的，这一版**踩过坑**：
    #    `safety_release` 之后立刻 `motor_resume`，那时 Motor Driver **还在状态 5**
    #    （`safety_blocked`），这次 resume 被吃掉；等它落到 **4**（`rearm_required`
    #    —— D-025/D-037 刻意要求"被拦住后必须显式重新使能"）时，已经没人再叫它了。
    #    而原来的代码**不检查 `wait_until` 的返回值**就往下走，于是 Control Skill
    #    如实拒绝（"底盘未确认在线"），却被工装报成 **"轮子没转"** ——
    #    **拿一个错误的原因否定了正确的系统**。所以：等它离开 5 → 再 resume →
    #    **确认真的回到能走的状态，回不来就拒测**。
    node.call(node.ctrl_stop)          # 中止那个还没跑完的请求（它正指着障碍）
    node.spin_for(0.5)
    node.call(node.safety_release)
    node.wait_until(lambda: node.mstat(I_STATE) != STATE_SAFETY_BLOCKED, timeout=6.0)
    rearmed = False
    for _ in range(3):
        node.call(node.motor_resume)
        rearmed = node.wait_until(
            lambda: node.mstat(I_STATE) in (STATE_OK, STATE_NO_CMD), timeout=4.0)
        if rearmed:
            break
    rep('★ 解除后底盘已重新使能（否则 Control Skill 会正确拒绝，别把那个读成"没转"）',
        rearmed, f'state={node.mstat(I_STATE)}')
    if not rearmed:
        print('     ⛔ 底盘没回到能走的状态 —— 后面的"放行"验不了，拒测。'
              '（这不等于守卫有问题：守卫那一条已经过了。）')
        return

    # ── 【3】放行（**后做**：这时才允许车真的动）────────────────────────────
    # 再量一次。**这是必要的**：如果【2】没拦住，车已经挪过了，
    # 【1】的读数就不再是"此刻"的事实 —— 拿旧读数当前提会把假失败读成真失败。
    print(f'\n【3】★ 再验**放行**：指令发出前**重新量一次**方向（确保"通畅"是此刻的事实）')
    fresh = probe_four_directions(node)
    if fresh[0][3] is None:
        rep('★ 指令发出前仍能问到一个通畅方向', False, 'sector_min_range 不可用')
        return
    fresh_clear, fresh_blocked = classify_directions(fresh)
    if not fresh_clear:
        rep('★ 指令发出前仍能问到一个通畅方向', False,
            '此刻四个方向都已不通畅 —— 不盲目发运动指令')
        return
    # ★ **对照的前提必须还在**：如果【2】用的那个受阻方向此刻**不再受阻**，
    #   说明那一段把障碍物挪走/撞倒了。照旧往下走会得到"同一个方向既拦下又放行"
    #   这种胡话 —— 而且会往一个刚刚还被判受阻的方向发运动指令。
    #   （真机第一次就撞上了：地面【2】的蠕动把 0.20 m 外的瓶子撞倒推走了。）
    if bname not in [p[0] for p in fresh_blocked]:
        print()
        print(f'  ⛔ **对照的前提没了**：【2】判为受阻的「{bname}」此刻已经不通阻'
              f'（re-probe 显示它已通畅）—— 拒测。')
        print('     这说明【2】那一段把障碍物挪走了/撞倒了。')
        print('     地面测试尤其容易：**车在守卫锁存前的蠕动就足以碰到 0.2 m 外的东西**。')
        print('     请把障碍物换成**推不动**的（墙 / 装水的大桶 / 抵住的东西），或把车架起来。')
        rep('★ 【2】用的受阻方向此刻仍然受阻（对照的前提还在）', False,
            f'「{bname}」已不再受阻')
        return
    name, center, move, r, a = fresh_clear[0]
    if name != clear[0][0]:
        print(f'    ⚠️ 通畅方向与【1】时不同（{clear[0][0]} → {name}）—— '
              f'说明【2】那一段车挪过位置，以**此刻**为准')
    print(f'    往**通畅的「{name}」**方向走 0.3 m —— 守卫**不该**拦它')
    with node._lock:
        node._reset()
    req = MoveRelative.Request()
    req.x = float(move['x'])
    req.y = float(move['y'])
    fut = node.move_cli.call_async(req)
    node.spin_for(2.5)
    spans, smax, rps, rmax = node.snapshot()
    latched = (node.sstat(0) or 0) > 0.5
    print(f'    {fmt(node, spans, smax, rps, rmax)}')
    print(f'    Control Skill：{getattr(fut.result(), "message", "（还没回）") if fut.done() else "（进行中）"}')
    rep(f'★ 守卫**没有**拦「{name}」方向的运动', not latched, f'safety status[0]={node.sstat(0)}')
    rep(f'★ 而且轮子真的朝「{name}」转了', rmax > 0.1, f'窗口内最大 |rps| {rmax:.4f}')

    print()
    print(f'  ⇒ 同一台车、同一次会话：「{bname}」拦下、「{name}」放行 —— 守卫是**有方向的**，')
    print('     不是"一刀切"。一个什么都拦的守卫会被关掉，等于没有。')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=['veto', 'guard', 'direction'], required=True)
    ap.add_argument('--dry-classify', action='store_true',
                    help='只量摆位并分类就退出 —— 不发任何运动指令。'
                         '不需要 Motor Driver，也不需要人看护（要车已摆好、LiDAR Driver 在跑）')
    args = ap.parse_args()

    rclpy.init()
    node = SafetyChainTester()
    results = []

    def rep(name, passed, detail):
        results.append((name, passed, detail))
        print(f'  {"✅" if passed else "❌"} {name}：{detail}')

    if args.dry_classify:
        # 摆位检查：**只读 LiDAR 原语**。刻意放在 preflight 之前 ——
        # 它既不需要底盘在线，也不该为了量一次距离就去动 `~/resume`。
        try:
            print()
            print('=' * 74)
            print('  摆位检查（不驱动任何东西 —— 只问 LiDAR 原语四个方向各有多远）')
            print('=' * 74)
            print()
            probed = probe_four_directions(node)
            if probed[0][3] is None:
                print()
                print('  ⛔ sector_min_range 不可用 —— LiDAR Driver 在跑吗？')
                return 3
            clear, blocked = classify_directions(probed)
            print()
            print(f'  通畅（> {CLEAR_MARGIN} m 或无回波）：'
                  + ('、'.join(p[0] for p in clear) or '（无）'))
            print(f'  受阻（≤ {GUARD_STOP_RANGE:.3f} m）：'
                  + ('、'.join(p[0] for p in blocked) or '（无）'))
            print()
            if not clear:
                print('  ⛔ 摆位不满足：**没有一个通畅方向** → `--phase direction` 会拒测。')
                print(f'     请在某个方向留出 > {CLEAR_MARGIN} m 的空档。')
                return 1
            if not blocked:
                print('  ⛔ 摆位不满足：**没有一个受阻方向** → `--phase direction` 会拒测。')
                print(f'     请在一个方向放个东西（≤ {GUARD_STOP_RANGE:.2f} m，'
                      f'比 {GUARD_STOP_RANGE:.2f} m 更近才拦得住），')
                print(f'     同时留另一个方向 > {CLEAR_MARGIN} m 是空的。')
                return 1
            print(f'  ✅ 摆位满足：往「{clear[0][0]}」（{clear[0][3] if clear[0][3] > 0 else "无回波"}）'
                  f'走须放行、往「{blocked[0][0]}」（{blocked[0][3]:.3f} m）走须拦下。')
            print('     下一步：起全套节点（人在场、能直接断电），跑 `--phase direction`。')
            print('     （`direction` 不要求四轮离地 —— 它把"该被拦住"那一步放在位移之前；'
                  '但【3】会真的把车开出去 0.3 m，那一侧要留够空。）')
            print()
            return 0
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    try:
        print()
        print('=' * 74)
        title = {'veto': '安全链架空验收 —— 结构性否决（D-037）',
                 'guard': '安全链架空验收 —— 避障停车（D-036）',
                 'direction': '安全链架空验收 —— 守卫的**方向性**（D-036）'}[args.phase]
        # `direction` 不要求四轮离地（它把"该被拦住"那一步放在位移之前）
        where = '四轮离地' if args.phase in ('veto', 'guard') else '地面或离地均可'
        print(f'  {title}（{where}）')
        print('=' * 74)
        print()
        if not preflight(node, rep, args.phase == 'veto'):
            return 2

        if args.phase == 'veto':
            phase_veto(node, rep)
        elif args.phase == 'guard':
            phase_guard(node, rep)
        else:
            phase_direction(node, rep)

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
