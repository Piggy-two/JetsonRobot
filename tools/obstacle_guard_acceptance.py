#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障守卫的**干跑端到端验收**工装（D-036）。

    ⚠️ 这是【验收工装】，不是运行时组件。

它验什么
--------
Safety Runtime 的避障守卫有三路输入，本工装把**三路全部造假**，
于是可以在**车不动、雷达不接、Motor Driver 不跑**的前提下把判定链走完：

    ① 假 Motor Driver 状态（`/acceptance/motor_status`）→ 提供"被命令的运动方向与速度"
    ② 假雷达心跳（`/acceptance/lidar_front`）          → 提供"雷达还活着吗"
    ③ 假 LiDAR 原语服务（`/acceptance/sector_min_range`）→ 提供"那个方向有没有东西"

再看 Safety Runtime 的实际行为：锁不锁存、发不发零、事件里写的什么。

⚠️ **为什么必须造这三路**：真跑起来时，避障要判"往哪个方向走"，
   而静止的车没有方向 —— 只有真的命令它在动，才谈得上"要撞上了"。
   造假不是为了绕过什么，是为了**能把"正被命令以 0.2 m/s 左移"这种状态摆出来**。

⚠️ **假 Motor 状态与真 Motor Driver 的一处差异**：真的 Motor Driver 一旦被锁存，
   它 status 里的 vx/vy 会**变成 0**（那是它的输出）；本工装照旧报"被命令的速度"。
   于是场景 [7]（解除后障碍还在 → 立刻重新锁上）在这里可以被观测到，
   而在真机上要看下游那个 `/motor_driver/stop` 有没有调通 —— **能调通时车根本不动**
   （Motor Driver 自己锁着），调不通时走的就是本工装这条路径。

前置（本条命令启动 Safety Runtime，指向本工装的假输入）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_safety_runtime safety_runtime.launch.py \\
        motor_status_topic:=/acceptance/motor_status \\
        lidar_front_topic:=/acceptance/lidar_front \\
        sector_service:=/acceptance/sector_min_range \\
        motor_stop_service:=/acceptance/motor_stop \\
        control_stop_service:=/acceptance/control_stop \\
        voice_topic:=/acceptance/voice_words

用法：
    python3 tools/obstacle_guard_acceptance.py

退出码：0 = 全部断言通过；1 = 有断言失败。
"""

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from embodied_skills_interfaces.srv import SectorMinRange

MOTOR_STATUS = '/acceptance/motor_status'
LIDAR_FRONT = '/acceptance/lidar_front'
SECTOR_SRV = '/acceptance/sector_min_range'
# ⚠️ 下游两个停服务也要用**本工装自己的名字**，不能占用真的 /motor_driver/stop：
#    同一服务名上出现两个 server 时，客户端挑到哪个是不确定的 ——
#    那会变成"验收把真 Motor Driver 的锁存"这种事，很难查。
MOTOR_STOP = '/acceptance/motor_stop'
CTRL_STOP = '/acceptance/control_stop'
CMD_VEL = '/cmd_vel'
SAFETY_STATUS = '/embodied/safety/status'
SAFETY_EVENTS = '/embodied/safety/events'
RELEASE = '/safety_runtime/release'

MOTOR_HZ = 20.0
HEARTBEAT_HZ = 10.0

# 与 config/safety_runtime.yaml 的默认值一致
LOOKAHEAD = 1.5
MIN_SPEED = 0.02


class Rig(Node):
    """假输入 + 观测。所有造假都在这一个节点里。"""

    def __init__(self):
        super().__init__('obstacle_guard_acceptance')
        self._lock = threading.Lock()

        # ---- 被造假的三路 ----
        self._vx = 0.0
        self._vy = 0.0
        self._valid = False
        self._range = -1.0
        self._heartbeat = True
        self._queries = []          # 收到的 sector 查询（center, width, max_range）

        self.motor_pub = self.create_publisher(Float64MultiArray, MOTOR_STATUS, 10)
        self.front_pub = self.create_publisher(Float64MultiArray, LIDAR_FRONT, 10)
        self.create_service(SectorMinRange, SECTOR_SRV, self._on_sector)

        # ---- 下游锁存（best-effort 调用，本工装记下来证明"真的调了"）----
        self.motor_stops = 0
        self.ctrl_stops = 0
        self.create_service(Trigger, MOTOR_STOP, self._on_motor_stop)
        self.create_service(Trigger, CTRL_STOP, self._on_ctrl_stop)

        # ---- 观测 ----
        self.zero_frames = 0        # /cmd_vel 上的总帧数
        self.nonzero_frames = 0     # 其中**非零**的帧数 —— 必须恒为 0
        self.events = []
        self.status = None
        self.create_subscription(Twist, CMD_VEL, self._on_cmd_vel, 50)
        self.create_subscription(String, SAFETY_EVENTS, self._on_event, 20)
        self.create_subscription(Float64MultiArray, SAFETY_STATUS, self._on_status, 10)

        self.create_timer(1.0 / MOTOR_HZ, self._pub_motor)
        self.create_timer(1.0 / HEARTBEAT_HZ, self._pub_front)

        self.release_cli = self.create_client(Trigger, RELEASE)

    # ---------- 造假 ----------

    def set_state(self, vx=0.0, vy=0.0, valid=False, rng=-1.0):
        with self._lock:
            self._vx, self._vy, self._valid, self._range = vx, vy, valid, rng

    def set_heartbeat(self, alive):
        with self._lock:
            self._heartbeat = alive

    def _pub_motor(self):
        with self._lock:
            vx, vy = self._vx, self._vy
        m = Float64MultiArray()
        # 与 Motor Driver 的 status 布局一致（state, vx, vy, wz, ...）
        m.data = [0.0, vx, vy, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.motor_pub.publish(m)

    def _pub_front(self):
        with self._lock:
            alive = self._heartbeat
        if alive:
            m = Float64MultiArray()
            m.data = [-1.0, 0.0, 0.0, 0.0]
            self.front_pub.publish(m)

    def _on_sector(self, req, res):
        with self._lock:
            self._queries.append((req.center, req.width, req.max_range))
            valid, rng = self._valid, self._range
        # 真原语会按 max_range 过滤；这里照做，让"阈值"真的起作用
        if valid and req.max_range > 0.0 and rng > req.max_range:
            valid = False
        res.valid = bool(valid)
        res.range = float(rng) if valid else -1.0
        res.angle = 0.0
        res.points = 1 if valid else 0
        return res

    def _on_motor_stop(self, _req, res):
        self.motor_stops += 1
        res.success = True
        res.message = 'ok'
        return res

    def _on_ctrl_stop(self, _req, res):
        self.ctrl_stops += 1
        res.success = True
        res.message = 'ok'
        return res

    # ---------- 观测 ----------

    def _on_cmd_vel(self, msg):
        self.zero_frames += 1
        if (abs(msg.linear.x) > 1e-9 or abs(msg.linear.y) > 1e-9
                or abs(msg.angular.z) > 1e-9):
            self.nonzero_frames += 1

    def _on_event(self, msg):
        self.events.append(msg.data)

    def _on_status(self, msg):
        self.status = list(msg.data)

    # ---------- 便利方法 ----------

    @property
    def latched(self):
        return bool(self.status and self.status[0] > 0.5)

    def guard_field(self, i):
        """status 里追加的避障字段：4=使能 5=判停 6=阈值 7=回波 8=方位。"""
        return None if not self.status or len(self.status) <= i else self.status[i]

    def release(self):
        if not self.release_cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{RELEASE} 不可用 —— Safety Runtime 在跑吗？')
        fut = self.release_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        return fut.result()

    def reset(self):
        """把车摆成"没被命令动、前面也空"，再解除锁存。

        :return: 重置后**是否仍然锁存**。调用方应当断言它是 False ——
                 否则下一个场景的"已锁存"会变成假阳性（它本来就是锁着的）。

        ⚠️ 这里必须 `spin_for` 而不是 `time.sleep`：假 Motor 状态是靠定时器发的，
           睡觉期间不 spin 就没人发状态，而且服务应答也收不到。
        """
        self.set_state(vx=0.0, vy=0.0, valid=False, rng=-1.0)
        spin_for(self, 0.5)                  # 让守卫先看到"没在动"
        self.release()
        spin_for(self, 0.5)
        self.events.clear()                  # 事件按场景分开，免得取到上一场景的
        return self.latched


# ---------- 验收 ----------

class Report:
    def __init__(self):
        self.rows = []
        self.failed = 0

    def check(self, name, ok, detail=''):
        self.rows.append((ok, name, detail))
        if not ok:
            self.failed += 1
        print(f'  {"✅" if ok else "❌"} {name}' + (f'  —— {detail}' if detail else ''))

    def summary(self):
        total = len(self.rows)
        print(f'\n{"=" * 68}')
        print(f'  合计 {total - self.failed}/{total} 项通过')
        print(f'{"=" * 68}')
        return self.failed == 0


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for(node, pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def scenario_stationary_never_stops(rig, rep):
    print('\n[1] 停着不动（哪怕前面 5 cm 有东西）—— 不该锁存')
    rep.check('重置后未锁存（下面所有"未锁存"才有意义）', not rig.reset())
    rig.set_state(vx=0.0, vy=0.0, valid=True, rng=0.05)
    spin_for(rig, 1.2)
    rep.check('未被锁存', not rig.latched, f'latched={rig.latched}')
    rep.check('守卫报"不判停"', rig.guard_field(5) == 0.0,
              f'判停字段={rig.guard_field(5)}')


def scenario_below_min_speed(rig, rep):
    print(f'\n[2] 速度 {MIN_SPEED / 2:.3f} m/s（低于 min_speed）—— 视为没在平移，不该锁存')
    rig.set_state(vx=MIN_SPEED / 2, vy=0.0, valid=True, rng=0.01)
    spin_for(rig, 1.2)
    rep.check('未被锁存', not rig.latched, f'latched={rig.latched}')


def scenario_obstacle_in_path(rig, rep):
    print(f'\n[3] 被命令前进 {0.20:.2f} m/s，前方 0.18 m 有东西'
          f'（阈值 {0.20 * LOOKAHEAD:.2f} m）—— 该锁存')
    rep.check('重置后未锁存（确认是干净状态重新触发）', not rig.reset())
    rig.set_state(vx=0.20, vy=0.0, valid=True, rng=0.18)
    ok = wait_for(rig, lambda: rig.latched, timeout=3.0)
    rep.check('已锁存', ok)
    ev = [e for e in rig.events if e.startswith('estop_triggered:obstacle:')]
    rep.check('事件是 obstacle 且带距离/方位', bool(ev) and '0.18m' in ev[-1] and '+0deg' in ev[-1],
              ev[-1] if ev else '（没有 obstacle 事件）')
    # ⚠️ 那两个下游服务是**异步** best-effort：锁存事件先到、服务请求后处理，
    #    所以要多 spin 一会儿再读计数（一看到锁存就读会读成 0，那是时序不是缺陷）。
    spin_for(rig, 0.6)
    rep.check(f'{MOTOR_STOP}（假）被调到', rig.motor_stops > 0, f'{rig.motor_stops} 次')
    rep.check(f'{CTRL_STOP}（假）被调到', rig.ctrl_stops > 0, f'{rig.ctrl_stops} 次')
    rep.check('status 里报出阈值 ≈ 0.30 m',
              rig.guard_field(6) is not None and abs(rig.guard_field(6) - 0.30) < 0.02,
              f'{rig.guard_field(6)}')
    rep.check('status 里报出回波 0.18 m',
              rig.guard_field(7) is not None and abs(rig.guard_field(7) - 0.18) < 0.01,
              f'{rig.guard_field(7)}')


def scenario_beyond_threshold(rig, rep):
    print('\n[4] 同样 0.20 m/s，但东西在 0.45 m（在阈值 0.30 m 之外）—— 不该锁存')
    rep.check('重置后未锁存', not rig.reset())
    rig.set_state(vx=0.20, vy=0.0, valid=True, rng=0.45)
    spin_for(rig, 1.5)
    rep.check('未被锁存', not rig.latched, f'latched={rig.latched}')
    rep.check('守卫报"不判停"', rig.guard_field(5) == 0.0,
              f'判停字段={rig.guard_field(5)}')


def scenario_direction_is_travel(rig, rep):
    print('\n[5] 同样的 0.18 m，但命令是**左移**（vy=+0.20）—— 方向该报 +90°，且照样锁存')
    rep.check('重置后未锁存', not rig.reset())
    rig.set_state(vx=0.0, vy=0.20, valid=True, rng=0.18)
    ok = wait_for(rig, lambda: rig.latched, timeout=3.0)
    rep.check('已锁存（判的是"往哪走"，不是"哪边是前"）', ok)
    ev = [e for e in rig.events if e.startswith('estop_triggered:obstacle:')]
    rep.check('事件方位是 +90deg', bool(ev) and '+90deg' in ev[-1], ev[-1] if ev else '—')


def scenario_unknown_scan(rig, rep):
    print('\n[6] 雷达心跳断掉，而车正被命令前进 0.20 m/s —— 「不知道 ≠ 安全」，该锁存')
    rep.check('重置后未锁存（确认是干净状态重新触发）', not rig.reset())
    rig.set_state(vx=0.20, vy=0.0, valid=False, rng=-1.0)
    rig.set_heartbeat(False)
    ok = wait_for(rig, lambda: rig.latched, timeout=4.0)
    rep.check('已锁存', ok)
    ev = [e for e in rig.events if e.startswith('estop_triggered:obstacle:')]
    rep.check('事件是 obstacle:unknown_scan', bool(ev) and ev[-1].endswith('unknown_scan'),
              ev[-1] if ev else '（本场景没收到任何 obstacle 事件）')
    rig.set_heartbeat(True)


def scenario_release_holds(rig, rep):
    print('\n[7] 解除后（雷达已恢复、车仍被命令前进）—— 障碍还在，就该**立刻重新锁上**')
    rep.check('重置后未锁存', not rig.reset())
    rig.set_state(vx=0.20, vy=0.0, valid=True, rng=0.18)
    rep.check('重新锁存（前置）', wait_for(rig, lambda: rig.latched, timeout=3.0))

    res = rig.release()
    rep.check('解除服务返回成功', bool(res and res.success), res.message if res else '—')
    relatched = wait_for(rig, lambda: rig.latched, timeout=3.0)
    rep.check('随即又被锁存 —— 不是"解除了就没事了"', relatched)


def warmup(rig, rep):
    """预热：先确认假输入真的在送、Safety 真的在看（否则"我没在看"会伪装成"系统没动"）。

    ⚠️ 必须**等到状态话题真的来了**再判，不能定时长睡觉：
       本工装一启动就得面对一个"上一次跑完留下的局面" —— 上次退出后没人喂假 Motor 状态，
       看门狗的 2 s 早已过，节点此刻是**锁着**的。所以要先把那一把清掉再开测。

    :return: 是否可以继续往下测
    """
    print('\n[0] 预热：等 Safety 的状态话题、并清掉上一轮遗留的锁存')
    got = wait_for(rig, lambda: rig.status is not None, timeout=20.0)
    rep.check('收到 /embodied/safety/status', got)
    if not got:
        print('  ⚠️ 收不到状态话题 —— 检查 Safety Runtime 是否在跑、且参数指向本工装的假话题')
        return False
    rep.check('清掉遗留锁存（重置后未锁存）', not rig.reset())
    rep.check('假 Motor Driver 状态已被消化（守卫使能 = 1）',
              (rig.guard_field(4) or 0.0) > 0.5, f'守卫使能字段={rig.guard_field(4)}')
    rep.check('假 LiDAR 原语服务被问过', len(rig._queries) > 0, f'{len(rig._queries)} 次')
    if rig._queries:
        c, w, mr = rig._queries[-1]
        rep.check('查询用对了扇区宽度（±30°）',
                  abs(w - 1.0471975511965976) < 1e-9, f'width={w}')
        rep.check('静止时也照问（否则每段运动头一拍会缺答案）',
                  mr > 0.0, f'max_range={mr:.3f}')
    return True


def main():
    rclpy.init()
    rig = Rig()
    rep = Report()

    print('=' * 68)
    print('  避障守卫干跑验收（D-036）—— 三路输入全部造假，车不动、雷达不接')
    print('=' * 68)

    if not warmup(rig, rep):
        rep.summary()
        rig.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    scenario_stationary_never_stops(rig, rep)
    scenario_below_min_speed(rig, rep)
    scenario_obstacle_in_path(rig, rep)
    scenario_beyond_threshold(rig, rep)
    scenario_direction_is_travel(rig, rep)
    scenario_unknown_scan(rig, rep)
    scenario_release_holds(rig, rep)

    print('\n[8] 结构不变量：本节点**只会发零**（D-027）')
    rep.check('订阅到的 /cmd_vel 帧数 > 0（证明它真的发过）', rig.zero_frames > 0,
              f'{rig.zero_frames} 帧')
    rep.check('**非零帧数 = 0**', rig.nonzero_frames == 0, f'{rig.nonzero_frames} 帧')

    rig.reset()          # 收尾：别把节点留成锁存状态
    ok = rep.summary()
    rig.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
