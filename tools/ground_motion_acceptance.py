#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**地面运动验收** —— 四轮着地，把「走到位了没有」从推断变成读数。

    ⚠️ 这是【验收工装】，不是运行时组件。
    ⚠️ 前提：**四轮着地、正前方 ≥2.5 m 通畅、约 2.0 m 处有一面墙/大箱子**。
    ⚠️ 会让 Motor Driver 以 `dry_run:=false` 运行 —— **人必须在场、能直接断电**。
    ⚠️ **先跑 `python3 tools/vendor_cmd_channel_check.py`** —— 本工装的结论只有在
       「本项目是唯一在发指令的人」这个前提下才成立（**D-050**）：
       厂商 app 生态走的是另一条 Twist 入口 `/controller/cmd_vel`，
       **不受本项目任何一道防线约束**。那上面有人发速度 ⇒ 本次位移/转向读数作废。

它要回答的问题：**"走到位了"这件事，到这个项目里第一次有独立证据之前，是不知道的。**
----------------------------------------------------------------------------------
到这一轮为止，所有"走了多远"的说法都是**自证**：
`Control Skill` 的 `success` 是"速度按时长发完了"（D-026 明说**不是**走到位），
`/odom` 是**把订阅到的指令值当速度积分**（`odom_publisher.cal_odom_fun()`，纯死推算），
`advance_until_blocked` 的 `travelled` 自称"开环积分，未经独立校验"。
三者说的都是**同一件事**："指令发出去多少"。**没有一个是"车真的走了多少"。**

本工装给这件事配一个**独立基准**：**原始 `/scan` 量到正前方那面墙的距离变化**。
雷达不参与控制，也没人问它意见 —— 它只是看着墙。

    ⚠️ 刻意**不用**我们自己的 `embodied_lidar_driver` 的原语：那是我们自己写的查询层，
       拿它当"独立基准"就又是自证了。这里直接读 `/scan` 自己算。

四个相位（前两个只要 Motor Driver + Control Skill + Safety；后两个还要上层）
--------------------------------------------------------------------------------
    --phase distance     **位移精度**：同一个请求，问三个不同的人"走了多远"
                         （请求值 / `/odom` 死推算 / **雷达**），并把差值摊开。
                         ★ 这一段直接回答 D-026 那句"`/odom` 作为『走了多远』的参考够不够用"。
    --phase stop         **避障触发后的真实余量**：一直往前顶，直到守卫锁存，
                         量"从它喊停到车真停住"车又跑了多远。
                         ★ 这是 `obstacle_min_range` / `obstacle_lookahead` 标定的**输入**。
                         ⚠️ 它**不经过 Control Skill**（直接发 `/embodied/motor/cmd_vel`）——
                            标定的是"车刹得住吗"，与谁发的指令无关，所以还能扫速度。
    --phase chain        **完整链路**：一句话 → 路由器 → 网关 → Control Skill → 轮子，
                         再用雷达量它到底走了多远。
    --phase closedloop   **`advance_until_blocked` 真机**：闭环技能在真障碍前报 `BLOCKED`。
                         ⚠️ 悬空时它每一步都"通畅"，**那个分支从来没被走过**。

    --dry-measure        只读一次前方距离就退出（**不驱动任何东西**），用于确认摆位。

退出码：0 = 全部断言通过；1 = 有失败；2 = 环境不合格（拒测）；3 = 前置不满足。
"""

import argparse
import math
from collections import deque
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from geometry_msgs.msg import Twist
from embodied_skills_interfaces.srv import AdvanceUntilBlocked, MoveRelative, Rotate

SCAN = '/scan'
ODOM = '/odom'
CMD_VEL = '/cmd_vel'
CMD_VEL_ENTRY = '/embodied/motor/cmd_vel'
MOTOR_STATUS = '/embodied/motor/status'
SAFETY_STATUS = '/embodied/safety/status'
SAFETY_EVENTS = '/embodied/safety/events'
IMU = '/ros_robot_controller/imu_raw'
COMMAND_TEXT = '/embodied/command/text'

I_STATE, I_LATCHED, I_REARM = 0, 7, 9
STATE_OK, STATE_NO_CMD = 0.0, 1.0
STATE_SAFETY_BLOCKED = 5.0

FRONT_HALF_WIDTH = math.radians(30.0)
#: **位移基准**用的扇区半宽。刻意比 ±30° 窄：小房间里 ±30° 里往往还有更近的
#: 侧向物体（本机实测 −18° 处有个 1.30 m 的东西），它们会把基准顶掉 ——
#: 那时量到的"位移"其实是量到那个东西身上。±12° 盯住**正对面那一段面**。
#: 判据依赖"基准是一块大致垂直于行进方向的面"：Δ距离 ≈ 沿行进方向的位移。
REF_HALF_WIDTH = math.radians(10.0)
#: 每帧距离取**多帧中位数**：LD19 的距离噪声在 1.5 m 处就有厘米级，
#: 单帧差分会把它整个变成"位移误差"。
HIST_LEN = 7
#: 车体最前端在雷达前方多远（由 `base_link_mec.stl` 包围盒算出，见 DEV_NOTES 坑 29）。
#: 所有"离障碍还有多远"的余量都要减掉它 —— 雷达量的是雷达自己到障碍的距离。
BODY_FRONT_AHEAD = 0.1306
#: 守卫的阈值参数（`config/safety_runtime.yaml` 的副本）—— 用来**自己算**它会在哪儿喊停。
#: ⚠️ 改守卫配置就要同步改这里，否则标定读数会指向一个不存在的阈值。
GUARD_MIN_RANGE = 0.30      # 2026-10-08 按 D-040 由 0.20 提高
GUARD_LOOKAHEAD = 1.5
#: 摆位前提：前方参照物至少这么远，否则开不出 1 m 就没地方了。
MIN_REFERENCE_RANGE = 1.15


def angle_diff(a, b):
    """两个角的最短差（弧度，落在 (−π, π]）。"""
    d = (a - b) % (2.0 * math.pi)
    return d - 2.0 * math.pi if d > math.pi else d


class GroundTester(Node):
    def __init__(self):
        super().__init__('ground_motion_acceptance')
        self._lock = threading.Lock()
        self._range = None          # 正前方 ±30° 最近回波（米；None = 无回波）
        self._range_angle = None
        self._scan_time = None
        self._hist = deque(maxlen=HIST_LEN)      # (r30, a30, r12, a12, t)
        self._odom_x = self._odom_y = 0.0
        self._cmd_int = 0.0         # ∫|v| dt —— 实际**发出**的位移（来自 /cmd_vel）
        self._cmd_last = None
        self._cmd_active_at = None
        #: 陀螺 z 积分 —— **旋转的独立基准**（不受任何指令影响）
        self._gyro_int = 0.0
        self._gyro_dt = 0.0
        self._gyro_last = None
        self._gyro_wz = None
        self._motor = None
        self._safety = None
        self._safety_events = []
        self._safety_events_flag = 0

        self.create_subscription(LaserScan, SCAN, self._on_scan, 10)
        self.create_subscription(Odometry, ODOM, self._on_odom, 20)
        self.create_subscription(Twist, CMD_VEL, self._on_cmd, 50)
        self.create_subscription(Float64MultiArray, MOTOR_STATUS, self._on_motor, 10)
        self.create_subscription(Float64MultiArray, SAFETY_STATUS, self._on_safety, 10)
        self.create_subscription(Imu, IMU, self._on_imu, 50)
        self.create_subscription(String, SAFETY_EVENTS, self._on_safety_event, 20)

        self.raw_pub = self.create_publisher(Twist, CMD_VEL_ENTRY, 10)
        self.text_pub = self.create_publisher(String, COMMAND_TEXT, 10)
        self._raw_vx = 0.0
        self.create_timer(1.0 / 20.0, self._pub_raw)

        self.move_cli = self.create_client(MoveRelative, '/control_skills/move_relative')
        self.rotate_cli = self.create_client(Rotate, '/control_skills/rotate')
        self.ctrl_stop = self.create_client(Trigger, '/control_skills/stop')
        self.motor_stop = self.create_client(Trigger, '/motor_driver/stop')
        self.motor_resume = self.create_client(Trigger, '/motor_driver/resume')
        self.safety_release = self.create_client(Trigger, '/safety_runtime/release')
        self.advance_cli = self.create_client(
            AdvanceUntilBlocked, '/autonomous_skills/advance_until_blocked')

    # ---------- 观测 ----------

    def _on_scan(self, msg):
        r30 = r12 = None
        a30 = a12 = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = msg.angle_min + i * msg.angle_increment
            d = abs(angle_diff(a, 0.0))
            if d > FRONT_HALF_WIDTH:
                continue
            if r30 is None or r < r30:
                r30, a30 = float(r), float(a)
            if d <= REF_HALF_WIDTH and (r12 is None or r < r12):
                r12, a12 = float(r), float(a)
        now = time.monotonic()
        with self._lock:
            self._range, self._range_angle = r30, a30
            self._scan_time = now
            self._hist.append((r30, a30, r12, a12, now))

    def _on_odom(self, msg):
        with self._lock:
            self._odom_x = float(msg.pose.pose.position.x)
            self._odom_y = float(msg.pose.pose.position.y)

    def _on_cmd(self, msg):
        now = time.monotonic()
        v = math.hypot(msg.linear.x, msg.linear.y)
        with self._lock:
            if self._cmd_last is not None:
                dt = now - self._cmd_last
                if 0.0 < dt < 0.5:          # 断流就不积分（别把空档当成匀速）
                    self._cmd_int += v * dt
            self._cmd_last = now
            if v > 0.01:
                self._cmd_active_at = now    # "最近一次真的在下令运动"

    def _on_imu(self, msg):
        now = time.monotonic()
        wz = float(msg.angular_velocity.z)
        with self._lock:
            if self._gyro_last is not None:
                dt = now - self._gyro_last
                if 0.0 < dt < 0.5:      # 断流不积分（零阶保持会造假角度）
                    self._gyro_int += wz * dt
                    self._gyro_dt += dt
            self._gyro_last = now
            self._gyro_wz = wz

    def gyro_reset(self):
        with self._lock:
            self._gyro_int = 0.0
            self._gyro_dt = 0.0

    def gyro_snapshot(self):
        with self._lock:
            return self._gyro_int, self._gyro_dt, self._gyro_wz

    def _on_motor(self, msg):
        self._motor = list(msg.data)

    def _on_safety(self, msg):
        self._safety = list(msg.data)

    def _on_safety_event(self, msg):
        self._safety_events.append(msg.data)
        self._safety_events_flag = len(self._safety_events)

    def mstat(self, idx):
        return None if self._motor is None else self._motor[idx]

    def sstat(self, idx):
        return None if self._safety is None else self._safety[idx]

    def snapshot(self):
        with self._lock:
            return dict(rng=self._range, ang=self._range_angle,
                        odom=(self._odom_x, self._odom_y),
                        cmd_int=self._cmd_int)

    def front(self, kind='wide', fresh_within=1.0):
        """正前方最近回波。**扫描陈旧就返回 (None, None) —— 不知道 ≠ 很远。**

        `kind='wide'` 用 ±30°（与**守卫**和 `path_clear` 同一套几何）；
        `kind='ref'` 用 ±12°（**位移基准**，见 `REF_HALF_WIDTH` 的说明）。
        两者都取最近 `HIST_LEN` 帧的**中位数**，把单帧噪声压掉。
        """
        with self._lock:
            if self._scan_time is None or time.monotonic() - self._scan_time > fresh_within:
                return None, None
            idx = 0 if kind == 'wide' else 2
            vals = [(h[idx], h[idx + 1]) for h in self._hist if h[idx] is not None]
        if not vals:
            return None, None
        vals.sort(key=lambda v: v[0])
        return vals[len(vals) // 2]

    # ---------- 动作 ----------

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        return False

    def wait_motion_done(self, timeout, quiet=0.8):
        """等到"最近一次真的在下令运动"之后安静 `quiet` 秒 —— 比固定睡眠可靠。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            with self._lock:
                last = self._cmd_active_at
            if last is not None and time.monotonic() - last > quiet:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def call(self, cli, req=None, timeout=20.0):
        if not cli.service_is_ready():
            return None
        fut = cli.call_async(req if req is not None else Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def move_relative(self, x, y=0.0, timeout=20.0):
        """走一步；返回服务响应（None = 没走到）。"""
        req = MoveRelative.Request()
        req.x, req.y = float(x), float(y)
        fut = self.move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def _pub_raw(self):
        if self._raw_vx == 0.0:
            return
        t = Twist()
        t.linear.x = float(self._raw_vx)
        self.raw_pub.publish(t)

    def set_raw(self, vx):
        self._raw_vx = float(vx)
        if vx == 0.0:
            for _ in range(3):
                self.raw_pub.publish(Twist())


def rep(results, name, passed, detail):
    results.append((name, passed, detail))
    print(f'  {"✅" if passed else "❌"} {name}：{detail}')


def ensure_chassis_ready(node, timeout=20.0):
    """把底盘弄到"能接受运动指令"。

    ⚠️ **必须重试**：一次性 `~/resume` 是不够的 —— 本工装上一次运行结束时会
    `motor_stop`（状态 3），而下一次运行里 `~/resume` 可能在服务还没被发现时
    就发出去了，被**静默丢掉**（`service_is_ready()` 为假时 `call` 直接返回 None）。
    这正是"不重试就下结论"那一类，本工具第一次跑就撞上了。
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if node.mstat(I_STATE) in (STATE_OK, STATE_NO_CMD):
            return True
        if node.motor_resume.service_is_ready():
            node.call(node.motor_resume, timeout=4.0)
        node.spin_for(0.8)
    return False


def preflight(node, results, need_reference=True, min_reference=None):
    """底盘在线 + 静基线 + 前方有参照物。返回 False 表示拒测。

    `min_reference` **按相位给**：`distance` 要往前开 0.8 m；`stop` 只要够它加速到
    匀速再顶上去。拿同一个数卡所有相位，会让 `stop` 白拒测（第一次就是这么挂的）。
    """
    node.spin_for(1.5)
    if not ensure_chassis_ready(node):
        print(f'  ⛔ 底盘未确认在线（motor status[0] = {node.mstat(I_STATE)}）')
        print('     查：Motor Driver 在跑吗？dry_run 是 false 吗？require_safety 拦着吗？')
        return False
    print(f'（底盘已确认在线：state={node.mstat(I_STATE)}）')

    # 参照物给它几秒 —— 雷达刚起来 / 刚重启时，头几帧可能还没进缓冲
    rng = ang = None
    if need_reference:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rng, ang = node.front('ref')
            if rng is not None:
                break
            node.spin_for(0.5)
    else:
        rng, ang = node.front('ref')
    if need_reference:
        if rng is None:
            print(f'  ⛔ **正前方 ±{math.degrees(REF_HALF_WIDTH):.0f}° 内没有回波** '
                  f'—— 找不到**位移基准**。')
            print('     摆位要求：正前方约 1.5~2 m 处有**一块大致垂直于车**的面（墙/大箱子）。')
            return False
        floor_ = MIN_REFERENCE_RANGE if min_reference is None else min_reference
        if rng < floor_:
            print(f'  ⛔ **参照物太近**（正前方 {rng:.3f} m < {floor_:.2f} m）—— 拒测。')
            print('     这一相位需要这么多前方余量才量得准。')
            return False
        print(f'（前方参照面 {rng:.3f} m，方位 {math.degrees(ang):+.1f}°）')

    node.spin_for(1.0)
    r0, _ = node.front('ref')
    node.spin_for(1.0)
    r1, _ = node.front('ref')
    if need_reference and (r0 is None or r1 is None or abs(r0 - r1) > 0.02):
        print(f'  ⛔ **距离读数不稳**（{r0} → {r1}）—— 车在动或雷达在抖，本次数据无法判读。')
        print('     请把车扶稳后重跑。⚠️ 阈值调松只会把"假失败"换成"假成功"。')
        return False
    if r1 is not None:
        print(f'（静止时读数稳定：{r1:.3f} m）')
    return True


# ---------------------------------------------------------------- distance

#: (请求位移, 说明)。正负号用 REP-103：+x 前进。
LEGS = [
    (0.15, '前 0.15 m'),
    (0.25, '前 0.25 m'),
    (0.40, '前 0.40 m'),
    (-0.40, '后 0.40 m（回到原处）'),
]


def phase_distance(node, results):
    """★ 同一个"走了 0.3 m"，三个人给出三个数：请求 / 死推算 / 雷达。"""
    for req_x, label in LEGS:
        r0, a0 = node.front('ref')
        if r0 is None:
            rep(results, f'{label}：请求前能看到参照物', False, '前方无回波')
            return
        with node._lock:
            node._cmd_int = 0.0
            node._cmd_last = None
        o0 = node.snapshot()['odom']
        ev0 = node._safety_events_flag

        print(f'\n【{label}】请求 x={req_x:+.2f} m（雷达起始 {r0:.3f} m @ {math.degrees(a0):+.1f}°）')
        res = node.move_relative(req_x)
        node.spin_for(1.2)          # 等它真的停稳再量
        s = node.snapshot()
        r1, a1 = node.front('ref')
        if r1 is None:
            rep(results, f'{label}：请求后仍能看到参照物', False, '前方无回波（开过头了？）')
            return

        lidar = r0 - r1                      # 正前方，前进即距离变小
        odom = math.hypot(s['odom'][0] - o0[0], s['odom'][1] - o0[1])
        cmd = s['cmd_int']
        err = lidar - req_x
        new_events = node._safety_events[ev0:]

        print(f'    服务回话：{getattr(res, "message", "（无响应）")}'
              f'｜elapsed {getattr(res, "elapsed", float("nan")):.2f} s')
        print(f'    请求      {req_x:+.4f} m')
        print(f'    /odom     {odom:+.4f} m   ← **死推算**（把指令当速度积分）')
        print(f'    ∫|v|dt    {cmd:+.4f} m   ← 实际发出去的指令')
        print(f'    ★ 雷达    {lidar:+.4f} m   ← **独立基准**（{r0:.3f} → {r1:.3f}，'
              f'方位 {math.degrees(a1):+.1f}°）')
        print(f'    误差      {err:+.4f} m（雷达 − 请求，{abs(err) / abs(req_x) * 100:+.1f}%）')

        tol = max(0.02, 0.15 * abs(req_x))
        rep(results, f'{label}：轮子确实在转（真的发生了运动）', abs(lidar) > 0.05,
            f'雷达位移 {lidar:+.4f} m')
        rep(results, f'{label}：雷达位移与请求**同向**', (lidar * req_x) > 0,
            f'{lidar:+.4f} vs 请求 {req_x:+.4f}')
        rep(results, f'{label}：雷达位移与请求之差在 {tol:.3f} m 内', abs(err) <= tol,
            f'{err:+.4f} m')
        rep(results, f'{label}：这一程没有触发急停', not new_events,
            new_events[-1] if new_events else '（无事件）')
        if res is not None:
            print(f'    ⚠️ 服务自报 success={res.success} —— '
                  f'它的含义是"速度按时长发完了"，**不是**"走到位了"（D-026）')
    print('\n  ⇒ 三个数摊开之后："请求"和"发出去的指令"可以差；'
          '"发出去的指令"和"真的走了多远"**又是另一件事**。')
    print('     后者只有雷达知道 —— 这就是 D-026 要的"反馈源"。')
    print('     ⚠️ 边界：雷达量的是**朝那面墙的法向**靠近了多少。'
          '斜着走、原地漂移的那部分，这个基准看不到。')


# ---------------------------------------------------------------- stop

def phase_stop(node, results, speed, max_travel):
    """★ 一直往前顶到守卫锁存，量"喊停之后车又跑了多远"。"""
    r0, _ = node.front()
    if r0 is None:
        rep(results, '起点能看到参照物', False, '前方无回波')
        return
    print(f'\n【避障停车】以 {speed} m/s 朝参照物（{r0:.3f} m）直走，直到守卫锁存')
    print(f'    守卫阈值 = min({1.0}, max({GUARD_MIN_RANGE}, {speed} × {GUARD_LOOKAHEAD})) = '
          f'{min(1.0, max(GUARD_MIN_RANGE, speed * GUARD_LOOKAHEAD)):.3f} m（从**雷达**算；'
          f'车头还要再往前 {BODY_FRONT_AHEAD:.3f} m）')

    ev0 = node._safety_events_flag
    with node._lock:
        node._cmd_int = 0.0
        node._cmd_last = None
    node.call(node.safety_release)
    node.call(node.motor_resume)
    node.set_raw(speed)
    got = node.wait_until(lambda: (node.sstat(0) or 0) > 0.5, timeout=max_travel / speed + 5.0)

    node.set_raw(0.0)
    node.spin_for(2.0)                      # 等它真的停稳
    r_final, a_final = node.front()
    ev = node._safety_events[ev0:]

    rep(results, '守卫锁存了', got, ev[-1] if ev else '（无事件）')
    if not got or r_final is None:
        return

    # ⚠️ "锁存那一刻的距离"要从**事件文本**里取，不能读 `/embodied/safety/status`
    #    的那个字段 —— 锁存之后 Motor Driver 立刻输出零，守卫的速度输入变成 0，
    #    它下一拍就不做判定了，那个字段随即被新的一拍覆盖。事件是**当时写下**的。
    at_latch_range = None
    for e in reversed(ev):
        if 'obstacle:' in e:
            try:
                at_latch_range = float(e.split('obstacle:')[1].split('m')[0])
            except (IndexError, ValueError):
                pass
            break
    at_latch_thr = min(1.0, max(GUARD_MIN_RANGE, speed * GUARD_LOOKAHEAD))  # 守卫的阈值公式
    if at_latch_range is None:
        rep(results, '事件里带上了"锁存那一刻的距离"', False, f'{ev}')
        return

    creep = at_latch_range - r_final         # 从喊停到真停，车又走了多远
    clearance = r_final - BODY_FRONT_AHEAD   # 车头离障碍还有多少
    print(f'    锁存那一刻雷达读到   {at_latch_range:.4f} m（阈值 {at_latch_thr:.4f} m）')
    print(f'    停稳后雷达读到       {r_final:.4f} m（方位 {math.degrees(a_final):+.1f}°）')
    print(f'    ★ **喊停之后又跑了** {creep:.4f} m   ← 这就是"锁存前的蠕动 + 停车距离"')
    print(f'    ★ 车头离障碍还剩     {clearance:.4f} m   '
          f'（= 雷达读数 − 车体前伸量 {BODY_FRONT_AHEAD:.4f}）')
    print(f'    标定口径：min_range ≥ 车体前伸量 + 蠕动 + 余量 '
          f'= {BODY_FRONT_AHEAD:.3f} + {creep:.3f} + 余量 = {BODY_FRONT_AHEAD + creep:.3f} + 余量')

    rep(results, '★ 停稳后车头**没有**碰到障碍（余量 > 0）', clearance > 0.0,
        f'余量 {clearance:.4f} m')
    rep(results, '★ 蠕动量被量出来了（不是"大致没有"）', creep >= 0.0, f'{creep:.4f} m')


# ---------------------------------------------------------------- chain

def phase_chain(node, results, text, expect_m):
    """★ 一句话走完整条链，再用雷达问它到底走了多远。"""
    r0, a0 = node.front('ref')
    if r0 is None:
        rep(results, '起点能看到参照物', False, '前方无回波')
        return
    print(f'\n【完整链路】往 {COMMAND_TEXT} 发「{text}」（雷达起始 {r0:.3f} m）')
    with node._lock:
        node._cmd_int = 0.0
        node._cmd_last = None
    m = String()
    m.data = text
    node.text_pub.publish(m)
    # 先等"真的开始动"，再等"安静下来" —— 别用固定睡眠猜它跑完没有
    started = node.wait_until(lambda: node.snapshot()['cmd_int'] > 0.02, timeout=10.0)
    node.wait_motion_done(timeout=20.0)
    if not started:
        rep(results, '这条文本命令确实驱动了运动', False,
            '10 s 内没有任何指令 —— 路由器/网关在跑吗？allow_motion 开了吗？')
        return

    r1, a1 = node.front('ref')
    if r1 is None:
        rep(results, '请求后仍能看到参照物', False, '前方无回波')
        return
    lidar = r0 - r1
    cmd = node.snapshot()['cmd_int']
    print(f'    ★ 雷达    {lidar:+.4f} m（{r0:.3f}@{math.degrees(a0):+.1f}° → {r1:.3f}@{math.degrees(a1):+.1f}°）')
    print(f'    ∫|v|dt    {cmd:+.4f} m')
    print(f'    误差      {lidar - expect_m:+.4f} m（相对期望 {expect_m:+.2f} m）')
    rep(results, '★ 一句话真的驱动了车（雷达看到位移）', abs(lidar) > 0.05, f'{lidar:+.4f} m')
    rep(results, '★ 位移与命令解析出的距离相符', abs(lidar - expect_m) <= max(0.05, 0.2 * expect_m),
        f'{lidar:+.4f} vs {expect_m:+.4f}')


# ---------------------------------------------------------------- closedloop

def phase_closedloop(node, results, max_distance, clear_range, step):
    """★ 闭环技能：每步重新问 LiDAR，真障碍前必须报 BLOCKED。

    ⚠️ 这一段**直接调技能服务**，不经过 Skill 网关 —— 因为要验的是**技能自己的闭环**。
       网关那条路（含 `allow_motion` 与策略上限）由 `--phase chain` 与悬空验收覆盖。
    """
    # ⚠️ **两个几何要分开**：技能自己判"受阻"用的是 ±30°（`path_clear`），
    #    而**量位移**必须用 ±10° 那个基准面 —— 本机 ±30° 里有两个不同深度的面，
    #    车一动，最近的回波会从一块面跳到另一块面上，Δ距离就不再是位移
    #    （第一次就是这么读出错觉的：自报 0.500 m、±30° 读出 0.381 m，差 24%）。
    r0, a0 = node.front('ref')
    r0w, a0w = node.front('wide')
    if r0 is None:
        rep(results, '起点能看到参照物', False, '前方无回波')
        return
    # clear_range 要**按实测起始距离**定，不能拍：
    # 障碍在 2.0 m 而 clear_range 只给 0.5 m 的话，技能会先走满 max_distance、
    # 报 ARRIVED —— 那样**根本走不到 BLOCKED 那个分支**，等于没验。
    if clear_range <= 0.0:
        clear_range = max(0.25, r0 - 0.6 * max_distance)
        print(f'    （clear_range 未指定，按起始距离自动定：{clear_range:.3f} m —— '
              f'让它在走掉约 {r0 - clear_range:.2f} m 时撞上"受阻"）')
    print(f'\n【闭环技能】advance_until_blocked：最多走 {max_distance} m、'
          f'clear_range {clear_range:.3f} m、步长 {step} m')
    print(f'    前方参照物 {r0:.3f} m —— 它应当在 {clear_range:.3f} m 处判受阻并停下')
    req = AdvanceUntilBlocked.Request()
    req.max_distance, req.clear_range, req.step = max_distance, clear_range, step
    fut = node.advance_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=max_distance / 0.15 + 20.0)
    res = fut.result()
    node.spin_for(1.5)
    r1, a1 = node.front('ref')
    r1w, a1w = node.front('wide')
    if res is None or r1 is None:
        rep(results, '闭环技能有回话且能看到参照物', False, f'{res} / {r1}')
        return
    lidar = r0 - r1
    print(f'    state={res.state}｜success={res.success}｜{res.message}')
    print(f'    自报 travelled {res.travelled:.4f} m｜elapsed {res.elapsed:.2f} s')
    print(f'    ★ 雷达（基准 ±10°）{lidar:+.4f} m'
          f'（{r0:.3f}@{math.degrees(a0):+.1f}° → {r1:.3f}@{math.degrees(a1):+.1f}°）')
    print(f'    （技能自己看的 ±30°：{r0w if r0w else -1:.3f} → '
          f'{r1w if r1w else -1:.3f} m —— 它判"受阻"用的是这个）')
    rep(results, '★ 真障碍前如实报 BLOCKED（悬空时这一步永远走不到）',
        res.state == 'BLOCKED', f'state={res.state}')
    rep(results, '★ 停在 clear_range 附近，没有撞上去',
        (r1w if r1w is not None else r1) >= clear_range - 0.10,
        f'技能看到的距离 {(r1w if r1w is not None else r1):.3f} m ≥ {clear_range:.3f} − 0.10')
    rep(results, '★ 自报的 travelled 与雷达一致',
        abs(res.travelled - lidar) <= max(0.06, 0.25 * abs(lidar)),
        f'自报 {res.travelled:.4f} vs 雷达 {lidar:+.4f}')


# ---------------------------------------------------------------- rotate

def phase_rotate(node, results, angle):
    """★ 原地旋转：请求一个角度，用 **IMU 陀螺积分**量它真转了多少。

    为什么 IMU 在这里算独立基准：它**不参与控制**，陀螺读的是机身角速度本身 ——
    与"发了多久的 wz 指令"完全是两回事（后者就是自证）。
    ⚠️ 陀螺有**零偏**，不扣掉会把"没转"读成"一直在转"，所以先静置量一段基线。
    """
    print(f'\n【原地旋转】请求 {angle:+.4f} rad（{math.degrees(angle):+.1f}°，逆时针为正）')
    node.spin_for(1.5)
    samples = []
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        _, _, wz = node.gyro_snapshot()
        if wz is not None:
            samples.append(wz)
    if not samples:
        rep(results, 'IMU 有数据（旋转的独立基准）', False,
            f'收不到 {IMU}')
        return
    bias = sum(samples) / len(samples)
    print(f'    静止零偏 {bias:+.5f} rad/s（{len(samples)} 个采样）—— 下面扣掉它')

    node.gyro_reset()
    req = Rotate.Request()
    req.angle = float(angle)
    fut = node.rotate_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=abs(angle) / 0.4 + 15.0)
    res = fut.result()
    node.spin_for(1.5)
    raw, dt, _ = node.gyro_snapshot()
    if dt <= 0.1:
        rep(results, '陀螺积分有足够样本', False, f'dt={dt:.3f} s')
        return
    corrected = raw - bias * dt
    err = corrected - angle
    print(f'    服务回话：{getattr(res, "message", "（无响应）")}'
          f'｜elapsed {getattr(res, "elapsed", float("nan")):.2f} s')
    print(f'    请求            {angle:+.4f} rad')
    print(f'    陀螺积分（扣零偏）{corrected:+.4f} rad  ← **独立基准**（{dt:.2f} s）')
    print(f'    误差            {err:+.4f} rad（{math.degrees(err):+.1f}°，'
          f'{abs(err) / abs(angle) * 100:.1f}%）')
    rep(results, '★ 真的转了（陀螺看到角速度积分）', abs(corrected) > 0.05,
        f'{corrected:+.4f} rad')
    rep(results, '★ 转向与请求**同号**（逆时针为正）', (corrected * angle) > 0,
        f'{corrected:+.4f} vs 请求 {angle:+.4f}')
    rep(results, '★ 角度误差在 15% 或 0.15 rad 内',
        abs(err) <= max(0.15, 0.15 * abs(angle)), f'{err:+.4f} rad')


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase',
                    choices=['distance', 'stop', 'chain', 'closedloop', 'rotate'])
    ap.add_argument('--dry-measure', action='store_true',
                    help='只读一次前方距离就退出 —— 不驱动任何东西')
    ap.add_argument('--speed', type=float, default=0.15, help='--phase stop 用的速度 m/s')
    ap.add_argument('--max-travel', type=float, default=2.0, help='--phase stop 的最长行程 m')
    ap.add_argument('--text', default='向前走 0.3 米', help='--phase chain 发出去的话')
    ap.add_argument('--expect', type=float, default=0.3, help='--phase chain 期望的位移 m')
    ap.add_argument('--max-distance', type=float, default=0.5)
    ap.add_argument('--clear-range', type=float, default=0.0,
                    help='<=0 表示按实测起始距离自动定（默认）')
    ap.add_argument('--step', type=float, default=0.1)
    ap.add_argument('--angle', type=float, default=1.5707963267948966,
                    help='--phase rotate 请求的角度（弧度，逆时针为正）')
    args = ap.parse_args()
    if args.phase is None and not args.dry_measure:
        ap.error('要么给 --phase，要么给 --dry-measure')

    rclpy.init()
    node = GroundTester()
    results = []

    if args.dry_measure:
        try:
            print()
            node.spin_for(3.0)
            rw, aw = node.front('wide')
            rr, ar = node.front('ref')
            if rr is None:
                print(f'  ⛔ 正前方 ±{math.degrees(REF_HALF_WIDTH):.0f}° 内没有回波 '
                      f'—— 雷达看不到**位移基准**。')
                return 2
            print(f'  位移基准（±{math.degrees(REF_HALF_WIDTH):.0f}° 中位）：'
                  f'{rr:.4f} m（方位 {math.degrees(ar):+.2f}°）')
            if rw is not None and abs(rw - rr) > 0.05:
                print(f'  ⚠️ 但 ±30° 里有更近的东西：{rw:.4f} m（方位 {math.degrees(aw):+.2f}°）')
                print('     —— 位移基准**不受影响**（基准只看 ±10°），但守卫看的是 ±30°，'
                      '它会先看到那个。')
            print(f'  车头离基准面还有     {rr - BODY_FRONT_AHEAD:.4f} m '
                  f'（车体前伸量 {BODY_FRONT_AHEAD:.4f} m）')
            if rr < MIN_REFERENCE_RANGE:
                print(f'  ⛔ 太近了（< {MIN_REFERENCE_RANGE} m）—— 车要往前开 0.8 m，拒测。')
                return 1
            print(f'  ✅ 摆位合格（正前方有 ≥{MIN_REFERENCE_RANGE} m 的参照面、大致垂直于车）')
            return 0
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    try:
        print()
        print('=' * 74)
        title = {'distance': '地面运动验收 —— **位移精度**（独立基准 = 原始 /scan）',
                 'stop': '地面运动验收 —— **避障停车余量**（阈值标定的输入）',
                 'chain': '地面运动验收 —— **完整链路**（一句话 → 轮子）',
                 'closedloop': '地面运动验收 —— **闭环技能** advance_until_blocked',
                 'rotate': '地面运动验收 —— **原地旋转**（独立基准 = IMU 陀螺）'}[args.phase]
        print(f'  {title}')
        print('=' * 74)
        print()
        # 每个相位需要的前方余量不同 —— 见 preflight 的说明
        min_ref = {'distance': 1.15, 'stop': 0.60,
                   'chain': 0.60, 'closedloop': 0.35}[args.phase] if args.phase != 'rotate' else None
        if not preflight(node, results,
                         need_reference=args.phase != 'rotate',
                         min_reference=min_ref):
            return 2
        if args.phase == 'distance':
            phase_distance(node, results)
        elif args.phase == 'stop':
            phase_stop(node, results, args.speed, args.max_travel)
        elif args.phase == 'rotate':
            phase_rotate(node, results, args.angle)
        elif args.phase == 'chain':
            phase_chain(node, results, args.text, args.expect)
        else:
            phase_closedloop(node, results, args.max_distance, args.clear_range, args.step)

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
        print('  ⚠️ "雷达看到位移"回答的是"车真的动了"；位移**精度**是否够用，'
              '取决于用途 —— 数字都摊在上面，别替它下结论。')
        print('=' * 74)
        print()
        return 0 if not failed else 1
    finally:
        try:
            node.set_raw(0.0)
            node.call(node.ctrl_stop)
            node.call(node.motor_stop)
            node.call(node.safety_release)
            node.spin_for(0.5)
        except Exception:                                        # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
