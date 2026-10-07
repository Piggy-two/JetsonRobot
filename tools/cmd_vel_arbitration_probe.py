#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测量 `/cmd_vel` 上**两路发布者的交替关系** —— 即 #28 / DEV_NOTES 坑 23 那条结论（D-036 决策 7）。

    ⚠️ 这是【验收工装】，不是运行时组件。

为什么这样测就**不需要车动**
--------------------------
Motor Driver 默认 `dry_run=true`：它**不向 `/cmd_vel` 发布任何东西**，
而是把"本来会发出去的速度"原样发到干跑话题 `/embodied/motor/cmd_vel_dryrun`。

于是把 Safety 的零速通道**也指到那条干跑话题**上（`zero_channel_topic:=...dryrun`），
两路发布者就被放进了**同一条话题**，交替关系被**完整复现** ——
而真正的 `/cmd_vel` 上**一个发布者都没有**，底盘在物理上不可能收到指令。

它回答的问题只有一个：

> **Safety Runtime 锁存期间，那条话题上跑的到底是"全是零"，还是"零与非零交替"？**

⚠️ 本工装**自己带硬前置检查**：开始之前先断言 `/cmd_vel` 的发布者数为 **0**，
不是 0 就**直接退出**（不测）。这条检查不是装饰 —— 它是"本工装不可能驱动底盘"的**依据**。

两种工况（`--stop-service`）
----------------------------
    --stop-service /motor_driver/stop      # 下游 `stop` 调得通
    --stop-service /acceptance/absent      # 下游 `stop` **调不通**（危险情形：
                                            # Motor Driver 还活着、还在转发指令）

⚠️ **D-037 之后这两档都应该是 0%** —— 那正是这个工装存在的意义
--------------------------------------------------------------
D-037 把否决权改成了**结构性**的：Motor Driver 直接读 `/embodied/safety/status`，
安全层锁存期间它**自己**输出零，不再依赖"下游 `stop` 这次调通没有"。

| 工况 | 修复前（2026-10-07 上半天） | **修复后** |
|---|---|---|
| 下游 `stop` 调得通 | 锁存后 **0% 非零** | 锁存后 **0% 非零**（不变） |
| 下游 `stop` **调不通** | 锁存后 **66% 非零**（= 速率比），**不衰减** | 锁存后 **0% 非零** ← **这一格就是修复的证据** |

⇒ **跑这两档必须都得到 0%**。哪一档不是 0%，就说明否决权又退回"咨询性"了。

用法
----
    ① 起 Motor Driver（**默认 dry_run，不要加 dry_run:=false**；
       默认 `require_safety:=true` 正是被验的那条路径，不要关）：
         ros2 launch embodied_motor_driver motor_driver.launch.py

    ② 起 Safety Runtime，零速通道指到干跑话题、避障守卫关掉（本工装只问仲裁）：
         ros2 launch embodied_safety_runtime safety_runtime.launch.py \\
             zero_channel_topic:=/embodied/motor/cmd_vel_dryrun \\
             enable_obstacle_guard:=false \\
             motor_stop_service:=/motor_driver/stop

    ③ 跑：
         python3 tools/cmd_vel_arbitration_probe.py
         python3 tools/cmd_vel_arbitration_probe.py --stop-service /acceptance/absent

⚠️ **两次运行之间要显式 `~/resume`**：D-037 之后，只要"被拦下那一刻底盘可能还在动"，
   Motor Driver 恢复后会置位「需重新使能」——**这是刻意的**（不许静默复动），
   但会让第二次运行的前置基线变成 0% 非零，看起来像"没在转发"。
   工装会打印 `⚠️ 一直没等到非零帧`，先 `ros2 service call /motor_driver/resume` 再跑。

退出码：0 = 测出来了；1 = 前置条件不满足（拒测）。
"""

import argparse
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# 上层命令入口（Motor Driver 的输入）。**它不是底盘入口** ——
# 在 dry_run 下它只会被转发到干跑话题，永远到不了轮子。
CMD_IN = '/embodied/motor/cmd_vel'
# 两路发布者共同所在的那条话题（= Motor Driver 的干跑输出话题）
OBSERVE = '/embodied/motor/cmd_vel_dryrun'
REAL_CMD_VEL = '/cmd_vel'
MOTOR_STATUS = '/embodied/motor/status'
SAFETY_EVENTS = '/embodied/safety/events'
ESTOP = '/safety_runtime/estop'
RELEASE = '/safety_runtime/release'

CMD_HZ = 20.0
CMD_VX = 0.15          # 米/秒。仅用于让 Motor Driver 输出非零 —— 到不了轮子
PRE_SECONDS = 3.0      # 锁存前采样时长
POST_SECONDS = 4.0     # 锁存后采样时长

ZERO_EPS = 1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbitration_probe')
        self._lock = threading.Lock()
        self.frames = []            # (t_monotonic, is_zero)
        self.events = []
        self.motor_latched = None
        self.motor_state = None

        self.pub = self.create_publisher(Twist, CMD_IN, 10)
        self.create_subscription(Twist, OBSERVE, self._on_twist, 200)
        self.create_subscription(String, SAFETY_EVENTS, self._on_event, 20)
        self.create_subscription(Float64MultiArray, MOTOR_STATUS, self._on_status, 10)
        self.create_timer(1.0 / CMD_HZ, self._pub_cmd)

        self.estop_cli = self.create_client(Trigger, ESTOP)
        self.release_cli = self.create_client(Trigger, RELEASE)

    def _pub_cmd(self):
        t = Twist()
        t.linear.x = CMD_VX
        self.pub.publish(t)

    def _on_twist(self, msg):
        is_zero = (abs(msg.linear.x) < ZERO_EPS and abs(msg.linear.y) < ZERO_EPS
                   and abs(msg.angular.z) < ZERO_EPS)
        with self._lock:
            self.frames.append((time.monotonic(), is_zero))

    def _on_event(self, msg):
        self.events.append((time.monotonic(), msg.data))

    def _on_status(self, msg):
        if len(msg.data) >= 8:
            self.motor_state = float(msg.data[0])
            self.motor_latched = float(msg.data[7]) > 0.5

    def window(self, t0, t1):
        with self._lock:
            return [z for (t, z) in self.frames if t0 <= t <= t1]

    def call(self, cli, what):
        if not cli.wait_for_service(timeout_sec=3.0):
            print(f'  ⚠️ {what} 不可用')
            return None
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def count_publishers(node, topic):
    return len(node.get_publishers_info_by_topic(topic))


def report_window(name, frames):
    n = len(frames)
    if n == 0:
        print(f'  {name}：**没有帧** —— 检查 Motor Driver / Safety 是否在跑、'
              f'零速通道是否指到了 {OBSERVE}')
        return None
    zeros = sum(1 for z in frames if z)
    nonzero = n - zeros
    pct = 100.0 * nonzero / n
    print(f'  {name}：共 {n} 帧，其中**非零 {nonzero} 帧（{pct:.0f}%）**、零 {zeros} 帧')
    return pct


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stop-service', default='/motor_driver/stop',
                    help='Safety 调用的下游停服务。指向不存在的名字 = 模拟"下游 stop 调不通"')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()

    print('=' * 72)
    print('  /cmd_vel 两路发布者交替关系实测（#28 / 坑 23 / D-036 决策 7）')
    print('=' * 72)
    print(f'  上层命令入口 : {CMD_IN} @ {CMD_HZ:.0f} Hz，vx={CMD_VX} m/s（到不了轮子）')
    print(f'  观测话题     : {OBSERVE}（Motor Driver 干跑输出 + Safety 零速通道）')
    print(f'  下游停服务   : {args.stop_service}')
    print()

    print('[前置] 本工装的"不可能驱动底盘"依据 —— 真的 /cmd_vel 必须 0 个发布者')
    spin_for(node, 2.0)
    n_real = count_publishers(node, REAL_CMD_VEL)
    print(f'  {REAL_CMD_VEL} 发布者数 = {n_real}')
    if n_real != 0:
        print('  ❌ 有发布者在真的 /cmd_vel 上 —— 拒测（先确认 Motor Driver 是 dry_run=true）')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    n_obs = count_publishers(node, OBSERVE)
    print(f'  {OBSERVE} 发布者数 = {n_obs}（期望 2：Motor Driver + Safety）')
    if n_obs < 2:
        print('  ❌ 观测话题上不足两个发布者 —— 拒测（两个节点都要在跑，且零速通道指到这里）')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print()

    # ---- 预热：确认真的有帧在流 ----
    print('[0] 预热：确认话题上真的有帧')
    spin_for(node, 1.5)
    if not node.window(time.monotonic() - 1.5, time.monotonic()):
        print('  ❌ 1.5 s 内一帧都没有 —— 拒测')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print(f'  ✅ 有帧（未被命令时 Motor Driver 也在持续发零，D-020）')

    # ---- 锁存前基线 ----
    print(f'\n[1] 锁存前基线（{PRE_SECONDS:.0f} s）：上层正以 {CMD_VX} m/s 命令前进')
    wait_for_nonzero(node, 4.0)
    t_pre0 = time.monotonic()
    spin_for(node, PRE_SECONDS)
    t_estop = time.monotonic()
    pct_pre = report_window('锁存前', node.window(t_pre0, t_estop))

    # ---- 触发急停 ----
    print(f'\n[2] 触发 ~/estop（此时两路发布者都在发）')
    res = node.call(node.estop_cli, ESTOP)
    print(f'  estop 返回：{res.message if res else "（调用失败）"}')
    ev = node.events[-1][1] if node.events else '（无事件）'
    print(f'  事件：{ev}')

    # ---- 锁存后 ----
    print(f'\n[3] 锁存后（{POST_SECONDS:.0f} s）—— **这一段的非零比例就是"停没停住"**')
    t_post0 = time.monotonic()
    spin_for(node, POST_SECONDS)
    t_post1 = time.monotonic()
    pct_post = report_window('锁存后', node.window(t_post0, t_post1))

    # 后半段单独看：给下游 stop 一点落地时间
    t_mid = (t_post0 + t_post1) / 2.0
    pct_tail = report_window('锁存后·后半段',
                             node.window(t_mid, t_post1))

    print(f'\n[4] Motor Driver 侧的状态（锁存标志 / 状态码）')
    spin_for(node, 0.5)
    print(f'  status 里的锁存标志 = {node.motor_latched}，状态码 = {node.motor_state}'
          f'（0=ok 1=no_cmd 2=telemetry_lost 3=stopped 4=rearm_required）')

    print('\n' + '=' * 72)
    print('  怎么读这些数')
    print('=' * 72)
    print(f'  · 锁存前非零 {pct_pre:.0f}% 是基线（只有 Motor Driver 在发指令）。')
    print(f'  · 锁存后非零 {pct_post:.0f}%：')
    print('      = 0  → **这条路上停住了**（D-037 之后两档都该是 0）；')
    print('      > 0  → **零与非零在交替**，否决权退回"咨询性"了 —— 说明它又在依赖')
    print('              "下游 stop 这次调通没有"，而不是在读安全层的状态。')
    if args.stop_service != '/motor_driver/stop':
        print(f'  · 本次 `--stop-service {args.stop_service}` 是**故意调不通**的 ——')
        print('    这一档是**修复前唯一会露馅**的那格（当时 66%）；现在它也必须是 0%。')

    # 收尾
    node.call(node.release_cli, RELEASE)
    node.destroy_node()
    rclpy.shutdown()


def wait_for_nonzero(node, timeout):
    """等到话题上出现非零帧（证明 Motor Driver 真的在转发指令）。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        with node._lock:
            if any(not z for (_, z) in node.frames):
                return True
    print('  ⚠️ 一直没等到非零帧 —— Motor Driver 可能在 no_cmd / 遥测失联状态')
    return False


if __name__ == '__main__':
    main()
