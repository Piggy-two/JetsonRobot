#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**演示**：一句话交出去，看机器人自己走 —— 把过程翻成人话。

    ⚠️ 这是【演示工装】，不是运行时组件。它**不做判定**（判定在
       `agent_ground_acceptance.py`（真机）与 `llm_planner_acceptance.py`（接口）里），
       只把"发生了什么"讲清楚，并且**该说的风险照说**（干跑 / 转过方向 / 被安全层拦住）。

它讲的是这条故事
----------------
    你说一句话 → 规划（规则表或真模型）→ 每一步各自过网关 → 车动
    → 被挡住 → **没有人再说话，Agent 自己换了一条走法** → 走通

⚠️ **两条路讲的是同一个故事，但不是同一件事**：规则表是**查表**、真模型是**生成**；
两者**共用同一个校验口**（D-038），所以行为一样、可信度没有差别。
脚本会照实打出"这条来自谁"（受理答复里就带着）。

用法
----
前置：整栈在跑 —— `ros2 launch embodied_bringup demo.launch.py …`

    # 离线 / 不走真模型：起栈时给 rules_file 指向 rules_example.yaml，
    # 那条故事的名字是「往前走被挡就绕开」（三步脱困）
    python3 tools/demo_run.py --text "往前走被挡就绕开"

    # 真模型
    python3 tools/demo_run.py --text "绕开前面的障碍继续往前走"

⚠️ 车会不会动由**起栈时**的 `dry_run` 决定，本脚本只如实转述（开头就打在屏幕上）。
   ⚠️ 起栈时若给了 `rules_file`，**LLM 那一跳根本不会被问到**（D-038：规则表优先）——
   想演示真模型就要让规则表留空。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64MultiArray, String

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import AgentTask

SUBMIT = '/agent_runtime/submit'
STATUS = '/embodied/agent/status'
EVENTS = '/embodied/skill/events'
MOTOR_STATUS = '/embodied/motor/status'
SAFETY_EVENTS = '/embodied/safety/events'
IMU_TOPIC = '/ros_robot_controller/imu_raw'
SCAN_TOPIC = '/scan'

TERMINAL = frozenset({'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST', 'BLOCKED',
                      'FAILED', 'CANCELLED', 'FINISHED'})

#: 终态的**人话**。⚠️ 关键是别把 `BLOCKED` 读成"坏了" —— 对脱困计划来说
#: 它是**正常的**第 1 步结局（D-043 的推进规则就是为它定的）。
TERMINAL_WORDS = {
    'ARRIVED': '✓ 完成（达成了这一步自己的目标条件）',
    'TARGET_FOUND': '✓ 找到了目标',
    'BLOCKED': '■ 前方受阻 —— ⚠️ 对这个计划来说这是**正常结局**，不是失败',
    'TARGET_LOST': '■ 目标跟丢了',
    'FAILED': '✗ 失败（故障：雷达问不到、或调用没成功 —— 都不是"受阻"）',
    'CANCELLED': '■ 被取消',
    'FINISHED': '· 技能返回了（control-tier 的说法，**不等于**到位）',
}

#: IMU 静止时的积分漂移量级（rad / 3 s）。判"停住了没有"要用它当尺子，
#: 而不是要求恰好 0 —— 陀螺有零偏，**不动也会积出一点**。
YAW_DRIFT_3S = 0.05


class Demo(Node):
    def __init__(self):
        super().__init__('demo_run')
        self.events = []          # [(skill, state)] 按到达顺序
        self.estops = []          # [(t, 文本)]
        self.status = None
        self.motor = None
        self.fronts = []          # [(t, 前向最近距离)]
        self.gyro = []            # [(t, wz)]
        self.create_subscription(Float64MultiArray, STATUS, self._on_status, 10)
        self.create_subscription(Float64MultiArray, MOTOR_STATUS, self._on_motor, 10)
        self.create_subscription(String, SAFETY_EVENTS, self._on_estop, 20)
        self.create_subscription(SkillEvent, EVENTS, self._on_skill, 50)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 50)
        self.submit_cli = self.create_client(AgentTask, SUBMIT)

    # ---- 订阅 ----

    def _on_status(self, m):
        self.status = list(m.data)

    def _on_motor(self, m):
        self.motor = list(m.data)

    def _on_estop(self, m):
        self.estops.append((time.monotonic(), m.data))

    def _on_skill(self, m):
        self.events.append((m.skill, m.state))

    def _on_scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            a = math.degrees(m.angle_min + i * m.angle_increment)
            if abs((a + 180.0) % 360.0 - 180.0) <= 30.0 and m.range_min <= r <= m.range_max:
                best = r if best is None else min(best, r)
        if best is not None:
            self.fronts.append((time.monotonic(), best))

    def _on_imu(self, m):
        self.gyro.append((time.monotonic(), m.angular_velocity.z))

    # ---- 读数 ----

    def dry_run(self):
        """Motor Driver 状态里 **`dry_run` 是第 9 位（0 起下标 8）**。

        字段表（`PROJECT_STATUS.md` §7.2）：
        `[状态码, vx, vy, wz, 指令龄, imu龄, battery龄, 锁存, **dry_run**,
          需重新使能, 安全层状态龄, 安全层是否锁存, 安全层是否拦着]`

        ⚠️ 第一版写成了下标 9（= "需重新使能"），于是**干跑的车被报成"会动"**。
        是"车没动"这个物理事实把它揪出来的 —— 读错字段不会报错，只会**说反**。
        """
        if not self.motor or len(self.motor) <= 8:
            return None
        return bool(self.motor[8])

    def front_window(self, t0, t1):
        """前向距离的**中位数**（单帧会被抖动带偏）。"""
        vals = sorted(f for t, f in self.fronts if t0 <= t <= t1)
        return vals[len(vals) // 2] if vals else None

    def yaw(self, t0, t1):
        """IMU 陀螺 z 的积分（rad）—— **独立于我们自己的控制链**。"""
        pts = [(t, w) for t, w in self.gyro if t0 <= t <= t1]
        return sum((a[1] + b[1]) / 2 * (b[0] - a[0]) for a, b in zip(pts, pts[1:]))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def plan_step_count(message):
    """从受理答复里读出"共几步" —— 用它判断后面**有没有多出派发**（= 换了走法）。"""
    if '共 ' not in message:
        return 1
    token = message.split('共 ', 1)[1].split(' ', 1)[0]
    digits = ''.join(ch for ch in token if ch.isdigit())
    return int(digits) if digits else 1


def main():
    ap = argparse.ArgumentParser(description='演示：一句话 → 机器人自己走')
    ap.add_argument('--text', default='往前走被挡就绕开',
                    help='交给 Agent 的那句话（默认是规则表里那条三步脱困）')
    ap.add_argument('--settle', type=float, default=10.0,
                    help='事件流安静多久算讲完了（秒）。⚠️ 要大于一次模型调用的耗时')
    ap.add_argument('--timeout', type=float, default=120.0, help='整段的上限（秒）')
    args = ap.parse_args()

    rclpy.init()
    node = Demo()
    try:
        print()
        print('=' * 72)
        print('  一句话 → 机器人自己走')
        print('=' * 72)
        if not node.submit_cli.wait_for_service(timeout_sec=10.0):
            print(f'  ⛔ {SUBMIT} 不可用 —— 整栈在跑吗？')
            print('     ros2 launch embodied_bringup demo.launch.py')
            return 3
        node.spin(2.0)

        dry = node.dry_run()
        if dry is None:
            print('  ⚠️ 读不到 Motor Driver 状态 —— 下面"动没动"的判断不可靠')
        elif dry:
            print('  🔒 **干跑**：命令只到 `/embodied/motor/cmd_vel_dryrun`，**车不会动**')
            print('     （要真动：起栈时 `dry_run:=false`，车边有人、手能断电）')
        else:
            print('  ⚠️ **当前会真的驱动底盘**（dry_run=false）：车边要有人、手能断电')
        print()
        print(f'  🗣 你说：{args.text}')
        print()

        t0 = time.monotonic()
        f0 = node.front_window(t0 - 1.0, t0)

        # 摆位提示（**只提示，不拦**）：演示最好看的那一拍是"被挡住 → 自己换路"，
        # 而它只在"前方比计划用的受阻阈值更近"时才会发生。
        # ⚠️ 这里**不猜**计划里的阈值（规则表里是 0.5，真模型每次自己定），
        #    只把当前读数报出来 + 说清怎么摆能看到那一拍。
        if f0 is not None:
            print(f'  📏 摆位：正前方参照面 {f0:.2f} m（原始 `/scan`）')
            print('     要看到"**被挡住 → 自己换一条路**"那一拍，'
                  '前方就得**比计划里的受阻阈值更近**'
                  '—— 规则表示例用 0.5 m，**真模型每次自己定**（策略上限 1.0 m）——')
            print('     想看到它，把车挪到离障碍 0.5 m 以内；'
                  '想看到"一路走通"，就让它前方开阔。')
            print()

        req = AgentTask.Request()
        req.text = args.text
        req.principal = 'operator.manual'
        fut = node.submit_cli.call_async(req)
        end = time.monotonic() + 60
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        res = fut.result() if fut.done() else None

        if res is None:
            print('  ⛔ 提交没有回话（超时）')
            return 1
        if not res.accepted:
            print('  ⛔ 没有受理 —— 这也是**如实的答复**（不是崩了）：')
            print(f'     {res.message}')
            return 1

        #: ★ **"换了走法"的判据是 Agent 的唤醒计数**，不是"派发次数超过了计划的步数"。
        #:    唤醒计数**每次尝试结束才 +1**（`_settle_attempt` 记的那一笔），
        #:    所以"这次派发之前计数涨过"⇒ 上一次尝试已经结束 ⇒ 这是一条**新计划**。
        #:    ⚠️ 第一版用的是"派发次数 > 首条计划的步数" —— 于是**新计划自己的第 2、3 步
        #:    也全被贴上"换了走法"**（实测：5 次派发贴了 4 条）。那种叙述错得很难看，
        #:    因为它把"一个计划的内部步骤"说成了"一次次改主意"。
        wake_base = node.status[2] if node.status else 0.0
        last_wake = wake_base
        n_steps = plan_step_count(res.message)
        print('  ① 规划好了 ✅')
        print(f'     {res.message}')
        print()
        print('  ② 执行（每一步都各自过网关，六项检查一遍不少）：')

        seen = 0
        attempt = 1
        last_change = time.monotonic()
        last_n = 0
        while time.monotonic() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if len(node.events) != last_n:
                last_n = len(node.events)
                last_change = now
                skill, state = node.events[-1]
                if state == 'RUNNING':
                    seen += 1
                    wakes = node.status[2] if node.status else last_wake
                    extra = ''
                    if wakes > last_wake:
                        attempt += 1
                        extra = (f' —— ★ **没有人再说话，Agent 自己换了走法**'
                                 f'（第 {attempt} 次尝试）')
                        last_wake = wakes
                    print(f'     → 第 {seen} 次派发：{skill}{extra}')
                elif state in TERMINAL:
                    print(f'       {TERMINAL_WORDS.get(state, state)}〔{skill}〕')
            quiet = now - last_change
            if (last_n and quiet > args.settle
                    and any(st in TERMINAL for _s, st in node.events)):
                break
        t1 = time.monotonic()
        print()

        if node.estops:
            print('  ⚠️ 安全层介入过：')
            for _ts, txt in node.estops:
                print(f'     ■ {txt}')
            print('     （这是**安全功能在干活**，不是故障。要接着跑得显式 `~/release`）')
            print()

        print('  ③ 车到底动没动（独立基准：原始 `/scan` 与 IMU —— 不用我们自己的东西自证）')
        f1 = node.front_window(t1 - 2.0, t1)
        yaw = node.yaw(t0, t1)
        if f0 is not None and f1 is not None:
            print(f'     正前方参照面 {f0:.3f} → {f1:.3f} m（变化 {f0 - f1:+.3f} m）')
            if abs(yaw) > 0.2:
                print('     ⚠️ 中间**转过方向** ⇒ 上面这个数**已经不再是纯位移**'
                      '（转完就换了参照物）—— 以 IMU 的旋转为准')
        else:
            print('     读不到 `/scan`，位移无法独立核对')
        print(f'     IMU 旋转积分：{yaw:+.3f} rad')

        node.spin(3.0)
        yaw_after = node.yaw(t1, time.monotonic())
        if dry:
            print(f'     干跑：车本来就没动（最后 3 s 旋转 {yaw_after:+.4f} rad）')
        else:
            moved = abs(yaw_after) > YAW_DRIFT_3S
            print(f'     跑完之后停住了吗：最后 3 s 旋转 {yaw_after:+.4f} rad'
                  f'（静止漂移量级 ~{YAW_DRIFT_3S}）⇒ '
                  f'{"⚠️ 还在动！" if moved else "✓ 停住了"}')
        if node.status:
            print(f'     Agent 状态 [在途, 仍在等, **被唤醒次数**, 丢弃]={node.status}')
        print()
        print('=' * 72)
        print('  ⚠️ 这是**演示**，不是验收 —— 判定在 `agent_ground_acceptance.py`（真机）')
        print('     与 `llm_planner_acceptance.py`（接口与失败路径）')
        print('=' * 72)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
