#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay Control Skill 层（架构分层：Control Skill）。

把 Motor Driver 的"速度"包成**确定性的动作原语**——上层说"向前 0.5 米"，
而不是"以 0.15 m/s 走 3.33 秒"。这一层**不经过 LLM**（CLAUDE.md §0 / D-006）。

```text
Local Parser / Agent ──→[本节点]──→ /embodied/motor/cmd_vel ──→ Motor Driver ──→ /cmd_vel
```

提供的原语（`embodied_skills_interfaces/srv/`）：

| 服务 | 语义 |
|---|---|
| `~/move_relative` | 机体坐标系下平移 (x, y) 米，**不改变朝向**。x 前 / y 左（REP-103） |
| `~/rotate` | 原地旋转 angle 弧度，**逆时针为正** |
| `~/stop` | **立即中止**当前运动（不等待），并把在途的请求以失败结束 |

设计要点：

- **一次只跑一个动作。** 运动中再来请求 → 直接拒绝（busy），而不是排队。
  排队会让"车在动"这件事变得不可预测，而安全层需要知道**现在到底在不在动**。
- **服务回调用 `ReentrantCallbackGroup` + `MultiThreadedExecutor`**：
  本层的服务回调会**阻塞到运动结束**，所以运动进行中，`~/stop` **必须**能被立刻处理，
  而新来的请求**必须**能真的跑起来（才能看到"占用中"并返回 busy）。
  若把它们放进 `MutuallyExclusiveCallbackGroup`，两者都会**排队** ——
  `stop` 排在最需要它的那一刻之后，新请求则变成隐藏的队列。**这是实测踩出来的**：
  第一版就是 mutually-exclusive，结果 `stop` 超时、第二个请求等第一个跑完又照做了。
- **开环**：只按时间发速度。见 `motion_plan.py` 顶部对局限的说明 ——
  上层**不要**把它当成"走到位了"，它只是"按时间发完了"。
- **不自己实现限速兜底**：限幅在 Motor Driver（D-025）。这里只做**断言**，
  发现配置出来的速度会超限就**拒绝运动** —— 因为被静默钳掉会导致
  实际位移与时长都对不上，而调用方拿到的却还是 success。
- 运动结束/中止后**显式发一帧零**（虽然 Motor Driver 会在 `cmd_timeout` 后自己归零，
  但那要等 0.5 s）。本节点空闲时**不持续发** —— 持续发零是 Motor Driver 的职责。
"""
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from embodied_control_skills.motion_plan import (
    PlanError, plan_rotate, plan_translate, within_chassis_limits)
from embodied_skills_interfaces.srv import MoveRelative, Rotate

# Motor Driver 的状态码（见 embodied_motor_driver / D-025）
MOTOR_STATE_OK = 0.0
MOTOR_STATE_NO_CMD = 1.0
MOTOR_STATE_TELEMETRY_LOST = 2.0
MOTOR_STATE_STOPPED = 3.0
MOTOR_STATE_REARM_REQUIRED = 4.0
# 底盘"确认在线"允许的状态：ok / no_cmd（后者只是"当前没有新指令"，链路是在的）
MOTOR_STATES_CHASSIS_ALIVE = (MOTOR_STATE_OK, MOTOR_STATE_NO_CMD)


class ControlSkills(Node):
    def __init__(self):
        super().__init__('control_skills')

        self.declare_parameter('output_topic', '/embodied/motor/cmd_vel')
        self.declare_parameter('status_topic', '/embodied/motor/status')
        self.declare_parameter('publish_rate', 20.0)
        # 标称速度**低于**底盘限幅：留余量，别顶着上限跑
        self.declare_parameter('nominal_speed', 0.15)
        self.declare_parameter('nominal_rate', 0.40)
        # 本项目自己的护栏：单次位移/转角上限（与"速度限幅"是两件事）
        self.declare_parameter('max_distance', 1.0)
        self.declare_parameter('max_angle', 3.141592653589793)
        # 与 Motor Driver / 厂商一致，仅用于**断言**
        self.declare_parameter('max_vx', 0.2)
        self.declare_parameter('max_vy', 0.2)
        self.declare_parameter('max_wz', 0.5)
        self.declare_parameter('chassis_status_timeout', 1.0)
        self.declare_parameter('finish_slack', 0.5)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self.pub = self.create_publisher(Twist, g('output_topic'), 10)

        self._status_cb_group = MutuallyExclusiveCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        # ⚠️ 服务回调必须用 **Reentrant** 回调组，不能用 MutuallyExclusive：
        #    本层的服务回调会**阻塞到运动结束**。mutually-exclusive 组里，
        #    运动中来第二个请求不会被"拒绝"，而是**排在后面等** ——
        #    那就成了一个隐藏的队列，而本层明确不要排队（"车现在到底在不在动"必须可预测）。
        #    Reentrant 才让后来的请求能真的跑起来、看到 `_goal` 已占用、然后返回 busy；
        #    也才让 `~/stop` 在最需要它的时刻（运动进行中）能立刻被处理。
        #    共享状态由 `self._lock` 保护。
        self._srv_cb_group = ReentrantCallbackGroup()

        self.create_subscription(Float64MultiArray, g('status_topic'),
                                 self.on_status, 10,
                                 callback_group=self._status_cb_group)

        self.create_service(MoveRelative, '~/move_relative',
                            self.on_move_relative, callback_group=self._srv_cb_group)
        self.create_service(Rotate, '~/rotate',
                            self.on_rotate, callback_group=self._srv_cb_group)
        self.create_service(Trigger, '~/stop',
                            self.on_stop, callback_group=self._srv_cb_group)

        self._lock = threading.Lock()
        self._goal = None            # {'plan','deadline','event','success','message'}
        self._abort = False
        self._last_status = None     # (monotonic, state_code)

        rate = float(g('publish_rate'))
        self.create_timer(1.0 / rate, self.tick, callback_group=self._timer_cb_group)

        self.get_logger().info(
            f'Control Skill 启动 | 输出 -> {g("output_topic")} | {rate:.0f} Hz | '
            f'标称 {g("nominal_speed")} m/s、{g("nominal_rate")} rad/s | '
            f'单次上限 {g("max_distance")} m、{g("max_angle")} rad')

    # ---------- 底盘状态（前置条件） ----------

    def on_status(self, msg):
        if len(msg.data) >= 1:
            self._last_status = (time.monotonic(), float(msg.data[0]))

    def _chassis_reason(self):
        """返回 None 表示"底盘已确认在线"，否则返回拒绝原因。"""
        timeout = float(self.get_parameter('chassis_status_timeout').value)
        if self._last_status is None:
            return '未收到 Motor Driver 的状态（它没在跑？）—— 拒绝运动'
        age, code = self._last_status
        if time.monotonic() - age > timeout:
            return f'Motor Driver 状态已陈旧 {time.monotonic() - age:.1f}s —— 拒绝运动'
        if code not in MOTOR_STATES_CHASSIS_ALIVE:
            name = {2.0: 'telemetry_lost', 3.0: 'stopped', 4.0: 'rearm_required'}.get(code, str(code))
            return f'Motor Driver 状态为 {name}（底盘未确认在线）—— 拒绝运动'
        return None

    # ---------- 服务 ----------

    def on_move_relative(self, req, res):
        return self._start(req.x, req.y, None, res)

    def on_rotate(self, req, res):
        return self._start(None, None, req.angle, res)

    def on_stop(self, _req, res):
        """立即中止。**不等待** —— 它要在最坏的时刻也能立刻返回。"""
        with self._lock:
            self._abort = True
            active = self._goal is not None
        self.get_logger().warn(f'收到 stop：{"中止当前运动" if active else "当前无运动，仍已置位"}')
        res.success = True
        res.message = '已请求中止' if active else '当前无运动'
        return res

    def _start(self, x, y, angle, res):
        reason = self._chassis_reason()
        if reason is not None:
            res.success = False
            res.message = reason
            self.get_logger().warn(f'拒绝运动：{reason}')
            return res

        try:
            if angle is None:
                plan = plan_translate(
                    x, y,
                    float(self.get_parameter('nominal_speed').value),
                    float(self.get_parameter('max_distance').value))
                what = f'平移 ({x:+.3f}, {y:+.3f}) m'
            else:
                plan = plan_rotate(
                    angle,
                    float(self.get_parameter('nominal_rate').value),
                    float(self.get_parameter('max_angle').value))
                what = f'旋转 {angle:+.3f} rad'
        except PlanError as e:
            res.success = False
            res.message = f'请求不可接受：{e}'
            self.get_logger().warn(res.message)
            return res

        if plan is None:
            res.success = True
            res.message = '请求小于阈值，无需运动'
            res.elapsed = 0.0
            return res

        ok, why = within_chassis_limits(
            plan,
            float(self.get_parameter('max_vx').value),
            float(self.get_parameter('max_vy').value),
            float(self.get_parameter('max_wz').value))
        if not ok:
            res.success = False
            res.message = f'规划出的速度超出底盘限幅（配置不一致）：{why}'
            self.get_logger().error(res.message)
            return res

        with self._lock:
            if self._goal is not None:
                res.success = False
                res.message = '正有一个动作在执行，拒绝新请求（本层一次只跑一个动作）'
                self.get_logger().warn('busy：' + res.message)
                return res
            ev = threading.Event()
            goal = {'plan': plan, 'deadline': time.monotonic() + plan.duration,
                    'event': ev, 'success': False,
                    'message': '未完成（内部错误）', 'started': time.monotonic()}
            self._goal = goal
            self._abort = False

        self.get_logger().info(f'开始 {what}：v=({plan.vx:+.3f}, {plan.vy:+.3f}) '
                               f'wz={plan.wz:+.3f}，时长 {plan.duration:.2f}s')

        slack = float(self.get_parameter('finish_slack').value)
        if not ev.wait(timeout=plan.duration + slack):
            # 控制循环没有按时收尾 —— 这是**本节点的故障**，必须显式暴露，
            # 不能默默返回一个 success。
            # ⚠️ 这里**不去动 `self._goal`**：万一是循环卡住而不是死掉，
            #    清掉它会让下一个请求在旧运动还没停的时候就开始新运动。
            self.publish_zero()
            res.success = False
            res.message = '超时未收尾（控制循环异常），已发零'
            self.get_logger().error(res.message)
            return res

        # ⚠️ 读**本请求自己持有的那条 goal**，不要读 `self._goal`：
        #    控制循环收尾时已经把它置 None 了（曾经因此 TypeError 把整个节点搞崩）。
        res.success = bool(goal['success'])
        res.message = str(goal['message'])
        res.elapsed = time.monotonic() - goal['started']
        self.get_logger().info(f'结束 {what}：success={res.success} ({res.message}) '
                               f'耗时 {res.elapsed:.2f}s')
        return res

    # ---------- 控制循环 ----------

    def tick(self):
        goal = None
        finished = False
        with self._lock:
            g = self._goal
            if g is None:
                if self._abort:          # 空闲时收到 stop：也补一帧零
                    self._abort = False
                    finished = True
            elif self._abort or time.monotonic() >= g['deadline']:
                abort = self._abort
                self._goal = None
                self._abort = False
                g['success'] = not abort
                g['message'] = '被 stop 中止' if abort else '已完成（开环：按时间发出）'
                g['event'].set()
                finished = True
            else:
                goal = g

        if finished:
            # ⚠️ **正常结束也必须显式发零**，不能只是"停止发布"：
            #    Motor Driver 会保持最后一条速度直到 cmd_timeout（0.5 s），
            #    那意味着每次运动都会多冲一段（0.15 m/s × 0.5 s = 7.5 cm）—— D-020 的同一个坑。
            self.publish_zero()
            return
        if goal is None:
            # 空闲：**不持续发**。持续发零是 Motor Driver 的职责（D-025），
            # 这里再发一遍只会掩盖它自己的 no_cmd 状态。
            return
        self.pub.publish(self._twist(goal['plan']))

    def publish_zero(self):
        self.pub.publish(Twist())

    @staticmethod
    def _twist(plan):
        t = Twist()
        t.linear.x = float(plan.vx)
        t.linear.y = float(plan.vy)
        t.angular.z = float(plan.wz)
        return t


def main(args=None):
    rclpy.init(args=args)
    node = ControlSkills()
    # ⚠️ 必须多线程：服务回调会阻塞到运动结束，而控制循环、状态订阅、
    #    以及后来（可能被拒绝）的请求都要在这期间继续被处理。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()      # 退出前留最后一帧"停"
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
