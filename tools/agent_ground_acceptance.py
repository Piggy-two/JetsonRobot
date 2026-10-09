#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 层的**地面**验收：一句话 → 真模型规划 → 真的驱动轮子 → 真的停住。

    ⚠️ 这是【验收工装】。**它会命令真车运动** —— 必须有人在车边、手能直接断电
       （本机没有物理急停 #22）。
    ⚠️ **先跑 `python3 tools/vendor_cmd_channel_check.py`** —— 本工装的结论只有在
       「本项目是唯一在发指令的人」这个前提下才成立（**D-050**）：
       厂商 app 生态走的是另一条 Twist 入口 `/controller/cmd_vel`，
       **不受本项目任何一道防线约束**。

它回答的问题
------------
2026-10-08 已经验过"单步计划能驱动底盘"。本工装验的是**之后新加的两样**在真机上
是否成立 —— 它们到那时为止**只在干跑里验过**（干跑只能说"命令下达到了轮子"，
**不能说"车真的照它走了"**）：

  · **多步计划**（D-043）：一个计划里第 2、3 步会不会**真的接着走**；
  · **重规划**（D-044）：被挡住之后，Agent 会不会**没人再说话**就换一条走法，
    而且换出来的那条**真的执行了**。

独立基准（**不用我们自己的东西自证**）
--------------------------------------
位移用**原始 `/scan`**：量正前方参照面的距离变化。⚠️ 刻意不用
`/embodied/lidar/front`、也不用 `/odom` —— 那是我们自己的原语/死推算，
用它来证明"车走了"就是自证（D-026 实测过：`/odom` 回答的是"指令发了多少"）。
旋转用 **IMU 陀螺积分**（与 `ground_motion_acceptance.py` 同法）。

安全包络（**三层，全部在场**）
------------------------------
  1. **技能自己是闭环的**：`advance_until_blocked` 每迈一步都重新问雷达，
     前方近于 `clear_range` 就停 —— 它**不会**朝着障碍物一直开。
  2. **避障守卫在跑**（默认开）：被命令的方向上 < `obstacle_min_range`（0.30 m）即锁存。
     它是"技能失效"时的兜底，不是主力。
  3. **本工装自己的预算**：独立测得的位移超过 `--budget-m` 就**取消任务并报失败** ——
     不由着模型把车开远。
  外加：速度是 Control Skill 的标称 0.15 m/s，一次 `advance` 最多 0.5 m（策略上限）。

前置（**用一键起栈**，见 `embodied_bringup`）：
    DEEPSEEK_API_KEY=... ros2 launch embodied_bringup demo.launch.py \\
        dry_run:=false allow_motion:=true llm_enabled:=true max_replans:=1 \\
        llm_base_url:=https://api.deepseek.com/v1 llm_model:=deepseek-flash \\
        llm_api_key_env:=DEEPSEEK_API_KEY

    ⚠️ `max_replans:=1`：第一次真机跑，让"换一次走法"就收手。
    ⚠️ 跑之前确认 `/controller/cmd_vel` 上没有**厂商语音节点**（`voice_control_move`
       挂的是无限幅话题，#26 —— 厂商栈重启会把它带回来）。

用法：
    python3 tools/agent_ground_acceptance.py --text "往前走一小段"
退出码：0 = 全部断言通过；1 = 有失败；4 = **拒测**（前提不在，什么都没动）。
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import AgentTask

SUBMIT = '/agent_runtime/submit'
STATUS = '/embodied/agent/status'
EVENTS = '/embodied/skill/events'
IMU_TOPIC = '/ros_robot_controller/imu_raw'
RELEASE_SAFETY = '/safety_runtime/release'
RESUME_MOTOR = '/motor_driver/resume'
GATEWAY_CANCEL = '/skill_gateway/cancel'

TERMINAL = frozenset({'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST', 'BLOCKED',
                      'FAILED', 'CANCELLED', 'FINISHED'})

#: 正前方参照面的搜索半宽（度）。与 `ground_motion_acceptance.py` 同口径。
FRONT_HALF_WIDTH = 30.0


def front_min(scan):
    """原始 `/scan` 里正前方 ±30° 的最近回波（米）。无回波返回 None。"""
    if scan is None:
        return None
    best = None
    for i, r in enumerate(scan.ranges):
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        if abs((a + 180.0) % 360.0 - 180.0) <= FRONT_HALF_WIDTH:
            if scan.range_min <= r <= scan.range_max:
                best = r if best is None else min(best, r)
    return best


class Rig(Node):
    def __init__(self):
        super().__init__('agent_ground_acceptance')
        self._lock = threading.Lock()
        self.scan = None
        self.fronts = []          # [(t, 前向最近距离)] —— 独立基准的时间序列
        self.gyro_z = []          # [(t, wz)]
        self.events = []
        self.status = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 50)
        self.create_subscription(SkillEvent, EVENTS,
                                 lambda m: self.events.append((m.skill, m.state)), 50)
        self.create_subscription(Float64MultiArray, STATUS, self._on_status, 10)
        self.submit_cli = self.create_client(AgentTask, SUBMIT)
        self.cancel_cli = self.create_client(Trigger, GATEWAY_CANCEL)
        self.release_cli = self.create_client(Trigger, RELEASE_SAFETY)
        self.resume_cli = self.create_client(Trigger, RESUME_MOTOR)

    def _on_scan(self, m):
        with self._lock:
            self.scan = m
            f = front_min(m)
            if f is not None:
                self.fronts.append((time.monotonic(), f))

    def _on_imu(self, m):
        with self._lock:
            self.gyro_z.append((time.monotonic(), m.angular_velocity.z))

    def _on_status(self, m):
        self.status = list(m.data)

    # ---- 基准读数 ----

    def front_now(self):
        with self._lock:
            return front_min(self.scan)

    def front_window(self, t0, t1):
        """`[t0, t1]` 窗口内前向距离的中位数 —— 单帧会被噪声和抖动带偏。"""
        with self._lock:
            vals = sorted(f for t, f in self.fronts if t0 <= t <= t1)
        if not vals:
            return None
        return vals[len(vals) // 2]

    def yaw_delta(self, t0, t1):
        """`[t0, t1]` 窗口内陀螺 z 的积分（弧度）。**IMU 独立于我们的控制链。**"""
        with self._lock:
            pts = [(t, w) for t, w in self.gyro_z if t0 <= t <= t1]
        total = 0.0
        for (ta, wa), (tb, wb) in zip(pts, pts[1:]):
            total += (wa + wb) / 2.0 * (tb - ta)
        return total

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def dispatches(self):
        return [s for s, st in self.events if st == 'RUNNING']


def clear_lock(rig, log):
    """把安全层与 Motor Driver 的锁都解掉（上一轮遗留的锁会让这一轮直接失败）。"""
    for cli, path in ((rig.release_cli, RELEASE_SAFETY),
                      (rig.resume_cli, RESUME_MOTOR)):
        if cli.wait_for_service(timeout_sec=3.0):
            cli.call_async(Trigger.Request())
            log(f'已请求解除 {path}')
        else:
            log(f'⚠️ {path} 不可用 —— 没解除（若它本来就没锁，无妨）')
    rig.spin(1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--text', default='往前走一小段',
                    help='交给 Agent 的那句话（会**发往第三方 LLM 端点**）')
    ap.add_argument('--budget-m', type=float, default=1.0,
                    help='独立测得的位移上限（米）。超了就取消并报失败 —— 不让模型把车开远')
    ap.add_argument('--min-clearance', type=float, default=0.8,
                    help='开跑前正前方至少要有这么多余量，否则**拒测**')
    ap.add_argument('--expect-blocked', action='store_true',
                    help='★ **明确声明"这一跑要的就是被挡住"**（验重规划用）：'
                         '前方余量只需 ≥ 0.35 m（= 避障守卫阈值 0.30 m 之上，'
                         '保证不是"一开跑就已经在安全区里"）。'
                         '⚠️ 不声明时**不许**用低余量跑 —— 那会把"没条件"读成"功能坏了"')
    ap.add_argument('--timeout', type=float, default=90.0, help='整条任务的上限（秒）')
    args = ap.parse_args()

    rclpy.init()
    rig = Rig()
    results = []

    def rep(name, ok, detail):
        results.append((name, ok, detail))
        print(f'  {"✅" if ok else "❌"} {name}：{detail}')

    def log(msg):
        print(f'  · {msg}', flush=True)

    try:
        print()
        print('=' * 74)
        print('  Agent 地面验收 —— 一句话 → 真模型 → **真的驱动轮子**')
        print(f'  任务文本：{args.text!r}｜位移预算 {args.budget_m:g} m')
        print('  ⚠️ 车会动。人要能直接断电（本机没有物理急停 #22）')
        print('=' * 74)
        print()
        if not rig.submit_cli.wait_for_service(timeout_sec=10.0):
            print(f'  ⛔ 拒测：{SUBMIT} 不可用 —— 整栈在跑吗？')
            return 4
        rig.spin(2.0)

        # ---- 前置：量余量（**不动**）----
        f0 = None
        end = time.monotonic() + 15.0
        while f0 is None and time.monotonic() < end:
            rclpy.spin_once(rig, timeout_sec=0.2)
            f0 = rig.front_now()
        if f0 is None:
            print('  ⛔ 拒测：读不到 `/scan` —— 厂商雷达在跑吗？')
            return 4
        need = 0.35 if args.expect_blocked else args.min_clearance
        if f0 < need:
            print(f'  ⛔ 拒测：正前方只有 {f0:.3f} m（要求 ≥ {need:g} m'
                  f'{"，因为声明了 --expect-blocked" if args.expect_blocked else ""}）'
                  f'—— 余量不够，这一趟不该动。把车挪到前方开阔处再来。')
            return 4
        log(f'前置：正前方 {f0:.3f} m ✓（独立基准 = 原始 /scan）'
            + ('｜★ 已声明"要的就是被挡住"' if args.expect_blocked else ''))
        clear_lock(rig, log)

        # ---- 提交 ----
        rig.events.clear()
        t0 = time.monotonic()
        req = AgentTask.Request()
        req.text = args.text
        req.principal = 'operator.manual'
        fut = rig.submit_cli.call_async(req)
        while not fut.done() and time.monotonic() - t0 < 60:
            rclpy.spin_once(rig, timeout_sec=0.05)
        res = fut.result() if fut.done() else None
        rep('受理（真模型给出了计划）', bool(res and res.accepted),
            (res.message[:110] if res else '提交超时'))
        if not (res and res.accepted):
            return 0

        # ---- 边跑边看：预算用完就取消 ----
        # ⚠️ 退出的两个条件都要**显式**：① 见到终态**且**事件流安静够久
        #    （重规划在两次派发之间要问一次网络，实测 2~7 s，窗口短了会在
        #    "还在换走法"的时候就判"跑完了"）；② 到时限。
        print('  ── 实时（每 2 s 一行；位移 = 前向参照面距离的变化）')
        over_budget = False
        last_print = 0.0
        last_event_n = 0
        last_event_t = time.monotonic()
        while time.monotonic() - t0 < args.timeout:
            rclpy.spin_once(rig, timeout_sec=0.05)
            now = time.monotonic()
            if len(rig.events) != last_event_n:
                last_event_n = len(rig.events)
                last_event_t = now
            fa = rig.front_window(t0, now)
            traveled = (f0 - fa) if fa is not None else 0.0
            if now - last_print > 2.0:
                last_print = now
                print(f'      t={now - t0:5.1f}s｜前方 {fa if fa is None else round(fa, 3)} m'
                      f'｜已前移 {traveled:+.3f} m'
                      f'｜事件 {len(rig.events)}｜旋转 {rig.yaw_delta(t0, now):+.3f} rad',
                      flush=True)
            if traveled > args.budget_m:
                over_budget = True
                break
            has_terminal = any(st in TERMINAL for _s, st in rig.events)
            if has_terminal and (now - last_event_t) > 8.0:
                break
        t1 = time.monotonic()

        if over_budget:
            log(f'位移超过预算 {args.budget_m:g} m —— **去取消任务**')
            rig.cancel_cli.call_async(Trigger.Request())
            rig.spin(2.0)

        # ---- 判定 ----
        fa = rig.front_window(t1 - 3.0, t1)
        traveled = (f0 - fa) if fa is not None else None
        disp = rig.dispatches()
        term = [st for _s, st in rig.events if st in TERMINAL]
        yaw = rig.yaw_delta(t0, t1)

        print()
        rep('没超预算（车没被开远）', not over_budget,
            f'独立基准测得的位移 {traveled if traveled is None else round(traveled, 3)} m'
            f'｜预算 {args.budget_m:g} m')

        if args.expect_blocked:
            # ★ 受阻场景：**第 1 步就该不动** —— 那才是"被挡住"而不是"开过去了"。
            #   这一跑要验的是**换了走法**，不是位移。
            rep('★ 第 1 步被挡住、且**没撞上去**（受阻时不迈步是对的）',
                traveled is not None and traveled < 0.10,
                f'前方参照面 {f0:.3f} → {"?" if fa is None else round(fa, 3)} m'
                f'（前移 {traveled if traveled is None else round(traveled, 3)} m）')
            rep('★ **没人再说话，Agent 自己换了走法**（网关收到第 2 次派发）',
                len(disp) >= 2 and disp[1] != disp[0],
                f'派发 {len(disp)} 次：{" → ".join(disp) or "无"}')
            rep('新走法**真的执行了**（有它自己的终态）',
                len([d for d in disp]) >= 2 and len(term) >= 2,
                f'终态 {"｜".join(term) or "无"}')
        else:
            rep('★ **车真的动了**（独立基准，不是自报）',
                traveled is not None and traveled > 0.03,
                f'前方参照面 {f0:.3f} → {"?" if fa is None else round(fa, 3)} m'
                f'（前移 {traveled if traveled is None else round(traveled, 3)} m）')
        rep('派发链完整（每一步都过了网关）', bool(disp),
            ' → '.join(disp) if disp else '（一个都没派）')
        rep('有终态（没挂在 WAIT 里）', bool(term), '｜'.join(term) or '无')
        print(f'  · 旋转总计（IMU 陀螺积分）：{yaw:+.3f} rad')
        print(f'  · 事件序列：{" → ".join(f"{s}:{st}" for s, st in rig.events)}')
        print(f'  · agent/status={rig.status}')

        # ---- 收尾：必须停住 ----
        rig.spin(3.0)
        yaw_after = rig.yaw_delta(t1, time.monotonic())
        rep('★ 跑完之后**车停住了**（IMU 不再积分出旋转）', abs(yaw_after) < 0.05,
            f'最后 3 s 的旋转 {yaw_after:+.4f} rad')

        print()
        print('=' * 74)
        failed = [n for n, ok, _ in results if not ok]
        if failed:
            print(f'  ❌ {len(failed)}/{len(results)} 项未过：')
            for n in failed:
                print(f'     - {n}')
        else:
            print(f'  ✅ 全部 {len(results)} 项通过')
        print('  ⚠️ 判据是"**命令有没有变成真位移**"；位移的**精度**另见 '
              'ground_motion_acceptance.py 的 distance 相位')
        print('=' * 74)
        return 0 if not failed else 1
    finally:
        rig.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
