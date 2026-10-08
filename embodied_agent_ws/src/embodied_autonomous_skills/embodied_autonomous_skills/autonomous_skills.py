#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot 的第一个 **Autonomous Skill**（架构分层：Autonomous Skill）。

```text
[本节点] advance_until_blocked
   │  每一步：
   ├─→ /lidar_driver/path_clear       （看：前方通不通）
   └─→ /control_skills/move_relative  （动：迈一步）
```

与 Control Skill 的区别就是**闭环**：Control Skill 是"按时间发速度，发完就返回"，
它不看不问；本技能**每一步都重新判定**，用 LiDAR 做反馈（D-026 说的"有反馈之后再谈"）。

⚠️ **本技能未在真机上验证过。** 车从未真的这样走过 —— 走通的是**链路与判定逻辑**：
LiDAR 是真的，但运动是 dry-run 的。本轮验收只做到"判定 → 停止 → 终态"可复现。

它受三层闸门约束（D-033）：网关的 `allow_motion` 是第一层。

为什么直接调下层服务而不是再经网关
----------------------------------
D-029 定的铁律是「**除网关外，任何人不得直接调用技能服务**」。本节点直接调
`/control_skills/move_relative` 与 `/lidar_driver/path_clear`，看起来违反它 ——
所以把边界说清楚：

> **那条铁律管的是"Agent / 命令侧不得绕过准入"。** 一个技能在**自己的实现内部**
> 组合下层原语（"Skill 组合"），是分层架构本来的样子 —— 否则每加一个组合技能
> 都要在网关里开一条新路径，网关会退化成一个大 switch。
>
> 真正要守住的不变式是「**没有任何东西能不经过一次受检的入口就到达执行器**」。
> 它仍然成立：本技能本身是**注册在册、过了六项检查**才被派发的，
> 它的参数（`max_distance` 等）在网关那边受策略上限约束；
> 而它调用的 Control Skill 自己还有一层底盘前置条件与限幅断言。

⚠️ 这条边界需要在 DECISIONS 里补一条（见本次提交的 D-034）。
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from embodied_skills_interfaces.srv import (
    AdvanceUntilBlocked, MoveRelative, PathClear, Rotate, TurnUntilClear)

from embodied_autonomous_skills.advance_plan import (
    AdvancePlanError, decide, summarize, validate)
from embodied_autonomous_skills import turn_plan
from embodied_skill_gateway import task_state as ts


class AutonomousSkills(Node):
    def __init__(self):
        super().__init__('autonomous_skills')

        self.declare_parameter('path_clear_service', '/lidar_driver/path_clear')
        self.declare_parameter('move_service', '/control_skills/move_relative')
        self.declare_parameter('rotate_service', '/control_skills/rotate')
        self.declare_parameter('stop_service', '/control_skills/stop')
        self.declare_parameter('service_wait_timeout', 3.0)
        self.declare_parameter('sub_call_timeout', 12.0)
        # 前方扇区全宽（弧度）。pi/3 = 前方 ±30°，与 LiDAR Primitive 的默认口径一致。
        self.declare_parameter('sector_width', 1.0471975511965976)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self._srv_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._cancel = False
        self._busy = False

        self._clear_cli = self.create_client(
            PathClear, g('path_clear_service'), callback_group=self._srv_group)
        self._move_cli = self.create_client(
            MoveRelative, g('move_service'), callback_group=self._srv_group)
        self._rotate_cli = self.create_client(
            Rotate, g('rotate_service'), callback_group=self._srv_group)
        self._stop_cli = self.create_client(
            Trigger, g('stop_service'), callback_group=self._srv_group)

        self.create_service(AdvanceUntilBlocked, '~/advance_until_blocked',
                            self.on_advance, callback_group=self._srv_group)
        self.create_service(TurnUntilClear, '~/turn_until_clear',
                            self.on_turn, callback_group=self._srv_group)
        self.create_service(Trigger, '~/cancel', self.on_cancel,
                            callback_group=self._srv_group)

        self.get_logger().info(
            f'Autonomous Skill 启动 | 服务 ~/advance_until_blocked、~/turn_until_clear、~/cancel | '
            f'看 <- {g("path_clear_service")} | '
            f'动 -> {g("move_service")} 与 {g("rotate_service")} | '
            f'扇区全宽 {g("sector_width"):.3f} rad')
        self.get_logger().warn(
            '⚠️ 本技能**未在真机上验证过**（车从未这样走过）。'
            '受网关 allow_motion 闸门约束（D-033）')

    # ---------- 服务 ----------

    def on_cancel(self, _req, res):
        """取消。**立刻置位并顺手停掉正在进行的子动作。**

        ⚠️ 不"只把等待标记为放弃"：那会导致"车还在动、上层以为结束了"（D-030）。
        """
        with self._lock:
            self._cancel = True
        if self._stop_cli.service_is_ready():
            self._stop_cli.call_async(Trigger.Request())
        self.get_logger().warn('收到取消：已置位并请求 Control Skill 停止')
        res.success = True
        res.message = '已请求取消'
        return res

    def on_advance(self, req, res):
        # ⚠️ 服务回调会**阻塞到走完**，所以回调组必须是 Reentrant、
        #    执行器必须多线程 —— 否则 `~/cancel` 会在最需要它的时刻排队（DEV_NOTES 坑 10）。
        with self._lock:
            if self._busy:
                res.success = False
                res.state = ts.FAILED
                res.message = '已有一个前进动作在执行（本层一次只跑一个，不排队）'
                self.get_logger().warn(res.message)
                return res
            self._busy = True
            self._cancel = False

        started = time.monotonic()
        try:
            try:
                validate(req.max_distance, req.clear_range, req.step)
            except AdvancePlanError as e:
                res.success = False
                res.state = ts.FAILED
                res.message = f'参数不可接受：{e}'
                self.get_logger().warn(res.message)
                return res

            state, message, travelled = self._walk(
                req.max_distance, req.clear_range, req.step)
        finally:
            with self._lock:
                self._busy = False

        res.success = (state == ts.ARRIVED)
        res.state = state
        res.message = message
        res.elapsed = time.monotonic() - started
        res.travelled = travelled
        self.get_logger().info(
            f'{state}：{message} | 自报前进 {travelled:.3f} m / 耗时 {res.elapsed:.2f}s')
        return res

    def on_turn(self, req, res):
        """`turn_until_clear`：原地逐步转，直到前方通畅或转满上限。"""
        with self._lock:
            if self._busy:
                res.success = False
                res.state = ts.FAILED
                res.message = '已有一个动作在执行（本层一次只跑一个，不排队）'
                self.get_logger().warn(res.message)
                return res
            self._busy = True
            self._cancel = False

        started = time.monotonic()
        try:
            try:
                turn_plan.validate(req.max_angle, req.clear_range,
                                   req.step_angle, req.direction)
            except turn_plan.TurnPlanError as e:
                res.success = False
                res.state = ts.FAILED
                res.message = f'参数不可接受：{e}'
                self.get_logger().warn(res.message)
                return res

            state, message, heading = self._turn(
                req.max_angle, req.clear_range, req.step_angle, req.direction)
        finally:
            with self._lock:
                self._busy = False

        res.success = (state == ts.ARRIVED)
        res.state = state
        res.message = message
        res.elapsed = time.monotonic() - started
        res.heading = heading
        self.get_logger().info(
            f'{state}：{message} | 耗时 {res.elapsed:.2f}s')
        return res

    # ---------- 主循环 ----------

    def _walk(self, max_distance, clear_range, step):
        """逐步前进。返回 (终态, 说明, 已走距离)。"""
        remaining = float(max_distance)
        travelled = 0.0
        width = float(self.get_parameter('sector_width').value)
        wait = float(self.get_parameter('service_wait_timeout').value)

        while remaining > 1e-6:
            if self._cancelled():
                return (ts.CANCELLED, summarize(travelled, '被取消'), travelled)

            if not self._clear_cli.wait_for_service(timeout_sec=wait):
                return (ts.FAILED, summarize(
                    travelled, '雷达服务不可用 —— 无法判断前方，因此不迈步'), travelled)

            req = PathClear.Request()
            req.width = width
            req.clear_range = float(clear_range)
            clear, range_m, _angle = self._call(self._clear_cli, req)
            if clear is None:
                # 查询本身没有结论（没返回）—— 同样是"不知道"，不是"受阻"。
                return (ts.FAILED, summarize(
                    travelled, '前方查询无返回 —— 不知道 ≠ 安全，因此不迈步'), travelled)

            d = decide(clear, range_m, clear_range, remaining, step)
            if d.action != 'advance':
                return (d.terminal, summarize(travelled, d.note), travelled)

            moved, why = self._step(d.distance)
            if not moved:
                return (ts.FAILED, summarize(travelled, f'迈步失败：{why}'), travelled)

            travelled += d.distance
            remaining -= d.distance

        return (ts.ARRIVED, summarize(travelled, '走满上限'), travelled)

    def _step(self, distance):
        """迈一步。返回 (是否成功, 原因)。"""
        if not self._move_cli.wait_for_service(
                timeout_sec=float(self.get_parameter('service_wait_timeout').value)):
            return False, 'Control Skill 不可用'
        if self._cancelled():
            return False, '已被取消'

        req = MoveRelative.Request()
        req.x = float(distance)
        req.y = 0.0
        ok, msg, _elapsed = self._call(self._move_cli, req)
        if ok is None:
            return False, '调用无返回'
        if not ok:
            # Control Skill 的 success=False 可能是"底盘未在线"或"正忙"——
            # **原样转述它的原因**，不要自己编一个。
            return False, msg or 'Control Skill 拒绝'
        return True, ''

    # ---------- 工具 ----------

    def _turn(self, max_angle, clear_range, step_angle, direction):
        """逐步转。返回 (终态, 说明, 朝向)。

        ⚠️ **每一步都重新问一次雷达** —— 这正是它算 Autonomous 而不是 Control 的原因。
        朝向是**开环累加**（只累加真的转成功的步），未经独立校验。
        """
        remaining = float(max_angle)
        heading = 0.0
        width = float(self.get_parameter('sector_width').value)
        wait = float(self.get_parameter('service_wait_timeout').value)

        while True:
            if self._cancelled():
                return (ts.CANCELLED, turn_plan.summarize(heading, '被取消'), heading)

            if not self._clear_cli.wait_for_service(timeout_sec=wait):
                return (ts.FAILED, turn_plan.summarize(
                    heading, '雷达服务不可用 —— 无法判断前方，因此不转'), heading)

            req = PathClear.Request()
            req.width = width
            req.clear_range = float(clear_range)
            clear, range_m, _angle = self._call(self._clear_cli, req)
            if clear is None:
                return (ts.FAILED, turn_plan.summarize(
                    heading, '前方查询无返回 —— 不知道 ≠ 受阻，因此不转'), heading)

            d = turn_plan.decide(clear, range_m, clear_range,
                                 remaining, step_angle, direction)
            if d.action != 'turn':
                return (d.terminal, turn_plan.summarize(heading, d.note), heading)

            turned, why = self._rotate(d.angle)
            if not turned:
                return (ts.FAILED, turn_plan.summarize(
                    heading, f'转身失败：{why}'), heading)

            heading += d.angle
            remaining -= abs(d.angle)

    def _rotate(self, angle):
        """转一步（带符号）。返回 (是否成功, 原因)。"""
        if not self._rotate_cli.wait_for_service(
                timeout_sec=float(self.get_parameter('service_wait_timeout').value)):
            return False, 'Control Skill 的 rotate 不可用'
        if self._cancelled():
            return False, '已被取消'

        req = Rotate.Request()
        req.angle = float(angle)
        ok, msg, _elapsed = self._call(self._rotate_cli, req)
        if ok is None:
            return False, '调用无返回'
        if not ok:
            # 原样转述 Control Skill 的原因，不要自己编一个（与 _step 同）。
            return False, msg or 'Control Skill 拒绝'
        return True, ''

    def _cancelled(self):
        with self._lock:
            return self._cancel

    def _call(self, client, request):
        """调一个服务并等结果。返回字段元组；超时返回全 None。

        ⚠️ 等待用**单调钟**（#24 / DEV_NOTES 坑 13：墙上钟会被 NTP 步进）。
        """
        timeout = float(self.get_parameter('sub_call_timeout').value)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return (None, None, None)
        try:
            r = future.result()
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'服务调用异常：{e!r}')
            return (None, None, None)
        if isinstance(r, PathClear.Response):
            return r.clear, r.range, r.angle
        # MoveRelative 与 Rotate 的回包形状相同（success / message / elapsed）
        if isinstance(r, (MoveRelative.Response, Rotate.Response)):
            return r.success, r.message, r.elapsed
        return (None, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSkills()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
