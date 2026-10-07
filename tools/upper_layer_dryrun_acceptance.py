#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上层（命令路由 + Skill 网关）的**干跑端到端验收**工装。

    ⚠️ 这是【验收工装】，不是运行时组件。
       不得被 Agent Runtime、任何 Skill 或 Safety Runtime 调用。

为什么能在"车不动"的前提下验证
------------------------------
Motor Driver 默认 `dry_run=true`：它**不向 `/cmd_vel` 发布任何东西**，
而是把速度原样发到干跑话题 `/embodied/motor/cmd_vel_dryrun`。

于是**把这条话题上的速度对时间积分，就得到"被命令走过"的位移/转角** ——
车一动不动，但"命令有没有被正确下达、有没有被正确拦下"是可量的。
（这正是 Phase 2 验证 Control Skill 时用的手法，见 PROJECT_STATUS §7。）

它验的是**链路**，不是物理：
    ✅ 命令 → 解析 → 网关准入 → Control Skill → Motor Driver 的速度，数值对不对
    ✅ 被拒绝的请求，下游**一个字节的速度都没收到**（积分恒等于 0）
    ❌ **不**证明车真的会走到那个位置（那是真机测试的事，需要解冻）

前置（全部已启动）：
    embodied_motor_driver    (dry_run=true)
    embodied_control_skills
    embodied_skill_gateway   (allow_motion:=true)
    embodied_command_router  (allow_motion:=true)
    embodied_safety_runtime

用法：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    python3 tools/upper_layer_dryrun_acceptance.py

退出码：0 = 全部断言通过；1 = 有断言失败。
"""

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import SkillInvoke

TEXT_TOPIC = '/embodied/command/text'
DRYRUN_TOPIC = '/embodied/motor/cmd_vel_dryrun'
SAFETY_EVENTS = '/embodied/safety/events'
ESTOP_RELEASE = '/safety_runtime/release'
SKILL_EVENTS = '/embodied/skill/events'
INVOKE = '/skill_gateway/invoke'

# 会结束一条命令的终态
TERMINAL = frozenset({'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST', 'BLOCKED',
                      'FAILED', 'CANCELLED', 'FINISHED'})


class DryRunIntegrator(Node):
    """订阅干跑速度话题并积分。**用单调钟**（本机墙上钟会被 NTP 步进，#24）。"""

    def __init__(self):
        super().__init__('upper_layer_acceptance')
        self._lock = threading.Lock()
        self._reset()
        self.create_subscription(Twist, DRYRUN_TOPIC, self._on_twist, 50)
        self.create_subscription(String, SAFETY_EVENTS, self._on_event, 20)
        self.create_subscription(SkillEvent, SKILL_EVENTS, self._on_skill_event, 50)
        self.pub = self.create_publisher(String, TEXT_TOPIC, 10)
        self._release_cli = self.create_client(Trigger, ESTOP_RELEASE)
        self._resume_cli = self.create_client(Trigger, '/motor_driver/resume')
        self._invoke_cli = self.create_client(SkillInvoke, INVOKE)
        self.safety_events = []
        self.terminal_by_task = {}     # task_id -> [SkillEvent, ...]
        self.all_skill_events = []

    def _on_skill_event(self, m):
        self.all_skill_events.append(m)
        if m.state in TERMINAL:
            self.terminal_by_task.setdefault(m.task_id, []).append(m)

    def wait_terminal(self, task_id, timeout_s):
        """等某个 task_id 的终态事件。返回事件列表（空 = 一直没来）。"""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if self.terminal_by_task.get(task_id):
                return self.terminal_by_task[task_id]
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.terminal_by_task.get(task_id, [])

    def invoke(self, principal, skill, args_json):
        """直接调网关（绕开路由器），用于验证**事件契约**本身。"""
        if not self._invoke_cli.wait_for_service(timeout_sec=3.0):
            return None
        req = SkillInvoke.Request()
        req.principal = principal
        req.skill = skill
        req.args_json = args_json
        req.request_id = 'acceptance'
        fut = self._invoke_cli.call_async(req)
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def _reset(self):
        self.int_x = 0.0
        self.int_y = 0.0
        self.int_w = 0.0
        self.nonzero_frames = 0
        self.frames = 0
        self._last = time.monotonic()

    def _on_twist(self, m):
        now = time.monotonic()
        with self._lock:
            dt = now - self._last
            self._last = now
            self.frames += 1
            if abs(m.linear.x) > 1e-9 or abs(m.linear.y) > 1e-9 \
                    or abs(m.angular.z) > 1e-9:
                self.nonzero_frames += 1
                self.int_x += m.linear.x * dt
                self.int_y += m.linear.y * dt
                self.int_w += m.angular.z * dt

    def _on_event(self, m):
        self.safety_events.append(m.data)

    def snapshot(self):
        with self._lock:
            return self.int_x, self.int_y, self.int_w, self.nonzero_frames, self.frames

    def phase(self, text, seconds):
        """重置积分 → 发一条文本 → 等 seconds 秒 → 返回本段积分。"""
        with self._lock:
            self._reset()
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.snapshot()

    def idle(self, seconds):
        with self._lock:
            self._reset()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.snapshot()

    def release_estop(self):
        """解除急停链路上**所有**被锁存的下游。

        ⚠️ 三道锁是**互相独立**的（D-027 决策 3）：解除 Safety Runtime 的急停
        **不会**自动解除 Motor Driver 的停车锁存 —— 它要各自的 `~/resume`。
        只解一处会让后面的阶段全部被 Control Skill 以"底盘未确认在线"拒绝，
        看起来像"链路坏了"，其实是锁没解干净。
        """
        ok = True
        for name, cli in (('safety_runtime/release', self._release_cli),
                          ('motor_driver/resume', self._resume_cli)):
            if not cli.service_is_ready():
                ok = False
                continue
            cli.call_async(Trigger.Request())
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return ok


def main():
    rclpy.init()
    node = DryRunIntegrator()
    ok = True
    results = []

    def check(name, passed, detail):
        nonlocal ok
        ok = ok and passed
        results.append((name, passed, detail))
        print(f'  {"✅" if passed else "❌"} {name}：{detail}')

    try:
        print()
        print('=' * 72)
        print('  上层干跑验收（车不动；速度来自 /embodied/motor/cmd_vel_dryrun）')
        print('=' * 72)
        print()

        # 预热：等订阅建立
        node.idle(1.5)

        print('【基线】空闲 2 s（应当没有任何非零速度）')
        x, y, w, nz, fr = node.idle(2.0)
        print(f'    收到 {fr} 帧，非零 {nz} 帧')
        check('空闲时无指令', nz == 0, f'非零帧数 = {nz}')
        print()

        print('【1】"向前走 0.5 米" —— 应被受理并走到 0.5 m')
        x, y, w, nz, fr = node.phase('向前走 0.5 米', 5.0)
        print(f'    ∫vx = {x:+.4f} m   ∫vy = {y:+.4f} m   ∫wz = {w:+.4f} rad'
              f'   （{fr} 帧，非零 {nz}）')
        check('位移积分 ≈ 0.5 m', abs(x - 0.5) < 0.02, f'∫vx = {x:+.4f} m')
        check('侧向与转角为零', abs(y) < 1e-6 and abs(w) < 1e-6,
              f'∫vy = {y:+.4f}, ∫wz = {w:+.4f}')
        print()

        print('【2】"向前走 3 米" —— 超过网关策略上限 0.5 m，应被拒且积分为 0')
        x, y, w, nz, fr = node.phase('向前走 3 米', 3.0)
        print(f'    ∫vx = {x:+.4f} m （{fr} 帧，非零 {nz}）')
        check('被拒绝且下游零速度', nz == 0, f'非零帧数 = {nz}，∫vx = {x:+.4f}')
        print()

        print('【3】"去桌子旁找杯子" —— 复杂任务，应明确拒绝')
        x, y, w, nz, fr = node.phase('去桌子旁找杯子', 2.5)
        check('不产生任何运动', nz == 0, f'非零帧数 = {nz}')
        print()

        print('【4】"向前走 0.5" —— 缺单位，应拒绝而不是猜')
        x, y, w, nz, fr = node.phase('向前走 0.5', 2.5)
        check('不产生任何运动', nz == 0, f'非零帧数 = {nz}')
        print()

        print('【5】"左转 90 度" —— 应被受理（∫wz ≈ +1.5708 rad）')
        x, y, w, nz, fr = node.phase('左转 90 度', 6.5)
        print(f'    ∫wz = {w:+.4f} rad   （{fr} 帧，非零 {nz}）')
        check('转角积分 ≈ +π/2', abs(w - 1.5707963) < 0.05, f'∫wz = {w:+.4f} rad')
        print()

        print('【6】"前进1米后退2厘米" —— 两句连写，应拒绝（单位必须紧跟数值）')
        x, y, w, nz, fr = node.phase('前进1米后退2厘米', 2.5)
        check('不产生任何运动', nz == 0, f'非零帧数 = {nz}')
        print()

        print('【7】"停下" —— 安全词，应触发 Safety Runtime 急停并锁存')
        before = len(node.safety_events)
        x, y, w, nz, fr = node.phase('停下', 2.5)
        got = [e for e in node.safety_events[before:] if e.startswith('estop_triggered')]
        check('Safety Runtime 收到急停', len(got) > 0,
              f'事件 {got if got else "（无）"}')
        print()

        print('【8】急停锁存期间"向前走 0.5 米" —— 应由 Control Skill 拒绝')
        x, y, w, nz, fr = node.phase('向前走 0.5 米', 3.0)
        check('锁存期间不产生运动', nz == 0, f'非零帧数 = {nz}')
        print()

        # 先把急停链路整条解开（**三道锁是独立的**，见 release_estop 的注释），
        # 否则后面阶段会被 Control Skill 以"底盘未确认在线"拒绝 —— 那看起来像
        # 链路坏了，其实只是锁没解干净。
        released = node.release_estop()
        print(f'  解除急停链路：{"✅ 已解（含 Motor Driver 的 ~/resume）" if released else "⚠️ 有服务不可用"}')
        print()

        # ★ 事件契约：这一节是补上的 —— 上面那些判据只看"有没有速度"，
        #   而"任务在表里到了终态、事件却没发出去"这种故障，它们是看不见的。
        #   那种情况下调用方会一直等到自己的超时（"幽灵任务"）。
        print('【9】事件契约 —— 被受理的任务必须**恰好**产生一个终态事件')
        res = node.invoke('operator.manual', 'control.move_relative',
                          '{"x": 0.2, "y": 0.0}')
        if res is None or not res.accepted:
            check('受理成功', False, f'网关未受理：{getattr(res, "message", "无响应")}')
        else:
            events = node.wait_terminal(res.task_id, 12.0)
            states = [e.state for e in events]
            check('受理后产生了终态事件', len(events) >= 1,
                  f'{res.task_id} 的终态 {states or "（一个都没来）"}')
            check('终态**恰好一个**', len(events) == 1,
                  f'收到 {len(events)} 个：{states}')
            if events:
                check('control-tier 的终态不谎报到达',
                      events[0].state in ('FINISHED', 'FAILED', 'CANCELLED'),
                      f'state = {events[0].state}')
                check('verified 恒为 false（无独立反馈）',
                      events[0].verified is False, f'verified = {events[0].verified}')
                check('进度报 -1 而不是假进度',
                      events[0].progress == -1.0, f'progress = {events[0].progress}')
        print()

        print('【10】被**拒绝**的请求不得产生任何事件')
        n_before = len(node.all_skill_events)
        node.invoke('agent.planner', 'control.move_relative', '{"x": 0.1, "y": 0.0}')
        node.invoke('router.deterministic', 'control.move_relative', '{"x": 9.0, "y": 0.0}')
        node.idle(1.5)
        check('拒绝路径不产生事件', len(node.all_skill_events) == n_before,
              f'新增 {len(node.all_skill_events) - n_before} 个事件')
        print()

        # 收尾：再解一次，确保不把系统留在锁存状态（幂等）
        node.release_estop()
        print()

        print('=' * 72)
        failed = [n for n, p, _ in results if not p]
        if failed:
            print(f'  ❌ {len(failed)}/{len(results)} 项未通过：')
            for n in failed:
                print(f'     - {n}')
        else:
            print(f'  ✅ 全部 {len(results)} 项通过')
        print()
        print('  ⚠️ 验收的是**链路**不是物理：车全程未动，'
              '真机位移需要解冻运动测试才能验。')
        print('=' * 72)
        print()
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
