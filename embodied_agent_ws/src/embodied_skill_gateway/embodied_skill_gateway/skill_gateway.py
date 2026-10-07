#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot 上层 Skill 网关（架构分层：Agent 与 Skill 之间）。

D-005 的落点：「所有动作必须经过 Tool / Skill Safety Gateway」。plan.md §19 原图：

```text
Agent
  ↓  SkillInvoke（通用信封，唯一入口）
Skill Gateway
  ├── Schema Validation      ← checks.py
  ├── Permission             ← checks.py（D-003 的唯一执行点）
  ├── Range Check            ← checks.py（策略边界，比能力更严）
  ├── Timeout                ← task_table.py 的 deadline
  ├── Cancel                 ← 落到技能自己的 stop，不是"放弃等待"
  └── Result Validation      ← checks.py（control-tier 恒 verified=false）
  ↓  强类型 .srv（复用既有接口）
Skill（Control / Autonomous / Semantic）
```

🔒 **本节点是唯一被允许直接调用技能服务的进程。** Agent、路由器、验收工装
一律走 `~/invoke`。

⚠️ **它不是转发器。** 它必须能拒绝某些底层技能会接受的东西：`agent.planner` 调
control 技能（权限）、0.8 m 的位移（策略比底层 1.0 m 的可行域更严）、
把开环的 `success` 标成 `verified=false` 且不产出 `ARRIVED`（语义）。
这三条都有回归测试，见 `test/test_checks.py` 末尾。

线程模型
--------
`~/invoke` **必须立刻返回**（D-030 的异步受理），因此：

- 所有服务用 `ReentrantCallbackGroup` + `MultiThreadedExecutor`；
- 真正的派发在 `ThreadPoolExecutor` 的工作线程里做；
- 共享状态（任务表、客户端缓存）由 `self._lock` 保护。

⚠️ 若把 `~/invoke` 放进 `MutuallyExclusiveCallbackGroup`，它会和阻塞的派发
抢同一个执行槽 —— 那就成了"受理被自己的执行堵住"。同一个坑在 Control Skill
上踩过一次（DEV_NOTES 坑 10）。

三层闸门（互相独立，见 D-033）
------------------------------
    ① 本节点的 `allow_motion`（默认 **false**）—— 允不允许**派发**会动的技能
    ② Motor Driver 的 `dry_run`（默认 **true**）—— 允不允许**发到厂商 /cmd_vel**
    ③ Control Skill 的前置条件 —— 底盘状态必须新鲜且确认在线

打开①**不等于**车会动：①②③任意一层没打开，车都动不了。
"""

import json
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import (
    AdvanceUntilBlocked, MoveRelative, PathClear, Rotate, SectorMinRange,
    SkillCancel, SkillInvoke, SkillList, SkillResult)

from embodied_skill_gateway import checks, task_state
from embodied_skill_gateway.registry import Registry
from embodied_skill_gateway.task_table import TaskTable, TaskTableFull

# srv_type 字符串 → 消息类。**显式映射**，不做动态 import：
# 写错了要在 import 期就炸，而不是等到某次调用才发现。
_SRV_TYPES = {
    'embodied_skills_interfaces/MoveRelative': MoveRelative,
    'embodied_skills_interfaces/Rotate': Rotate,
    'embodied_skills_interfaces/SectorMinRange': SectorMinRange,
    'embodied_skills_interfaces/PathClear': PathClear,
    'embodied_skills_interfaces/AdvanceUntilBlocked': AdvanceUntilBlocked,
    'std_srvs/Trigger': Trigger,
}


def _now_msg(node):
    return node.get_clock().now().to_msg()


def _to_jsonable(value):
    """把消息字段转成能进 result_json 的普通值。"""
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if hasattr(value, 'get_fields_and_field_types'):
        return {k: _to_jsonable(getattr(value, k))
                for k in value.get_fields_and_field_types()}
    return str(value)


class SkillGateway(Node):
    def __init__(self):
        super().__init__('skill_gateway')

        self.declare_parameter('registry_file', '')
        self.declare_parameter('event_topic', '/embodied/skill/events')
        self.declare_parameter('max_tasks', 200)
        self.declare_parameter('sweep_rate', 10.0)
        self.declare_parameter('service_wait_timeout', 3.0)
        # ⚠️ 三层闸门的第一层。默认 false（D-033）。
        #    **打开它不等于车会动** —— 车会不会动取决于 Motor Driver 的 dry_run。
        self.declare_parameter('allow_motion', False)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        reg_file = str(g('registry_file'))
        if not reg_file:
            from ament_index_python.packages import get_package_share_directory
            import os
            reg_file = os.path.join(get_package_share_directory('embodied_skill_gateway'),
                                    'config', 'skill_registry.yaml')
        # 注册表加载失败**必须立刻炸**：一个技能名写错的网关比没有网关更糟，
        # 它会静默地拒绝一切，而看起来"在正常运行"。
        self.registry = Registry.from_yaml(reg_file)

        self.events = self.create_publisher(SkillEvent, g('event_topic'), 20)

        self._lock = threading.Lock()
        self._table = TaskTable(max_tasks=int(g('max_tasks')))
        # ⚠️ **不要**叫 `self._clients` —— `rclpy.Node` 内部有一个同名的 list，
        #    `create_client()` 会往它 append。覆盖它之后，第一次创建服务客户端就会
        #    `AttributeError: 'dict' object has no attribute 'append'` 把节点打崩，
        #    而且报错点离真正的原因（命名冲突）隔了好几层。
        #    同类会撞的名字还有 `_clock` / `_context`。
        self._srv_clients = {}
        self._seq = 0
        self._pool = ThreadPoolExecutor(max_workers=4,
                                        thread_name_prefix='skill_dispatch')

        self._srv_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        self.create_service(SkillInvoke, '~/invoke', self.on_invoke,
                            callback_group=self._srv_group)
        self.create_service(SkillCancel, '~/cancel', self.on_cancel,
                            callback_group=self._srv_group)
        self.create_service(SkillResult, '~/get_result', self.on_get_result,
                            callback_group=self._srv_group)
        self.create_service(SkillList, '~/list', self.on_list,
                            callback_group=self._srv_group)

        rate = float(g('sweep_rate'))
        self.create_timer(1.0 / rate, self.sweep, callback_group=self._timer_group)

        self.get_logger().info(
            f'Skill Gateway 启动 | 注册 {len(self.registry)} 个技能 '
            f'{list(self.registry.names())} | 事件 -> {g("event_topic")} | '
            f'{rate:.0f} Hz 扫超时')
        if not g('allow_motion'):
            self.get_logger().warn(
                '🔒 allow_motion=false：拒绝派发可能引起运动的技能（control.*）'
                '。这是刻意的默认值 —— 打开它请显式 allow_motion:=true')

    # ---------- 服务：invoke ----------

    def on_invoke(self, req, res):
        """受理。**立刻返回** —— 真正的派发在工作线程里。"""
        result = checks.admit(self.registry, req.principal, req.skill, req.args_json)
        if not result.accepted:
            res.accepted = False
            res.message = result.reason
            self.get_logger().warn(f'拒绝 {req.skill}（{req.principal}）：{result.reason}')
            return res

        spec = result.spec

        # ⚠️ 第一层闸门：不派发**会动**的技能（D-033）。
        #    判据是注册表里的 `causes_motion`，**不是**层级 —— Autonomous Skill
        #    也会让车动，只看 `tier == 'control'` 会把它们漏过去。
        if spec.causes_motion and not self.get_parameter('allow_motion').value:
            res.accepted = False
            res.message = (checks.format_reject(
                f'allow_motion=false —— 拒绝派发可能引起运动的技能 {spec.name}'
                f'（打开请显式 allow_motion:=true）'))
            self.get_logger().warn(res.message)
            return res

        with self._lock:
            # 每技能同时只允许一个在途任务（沿用 D-026 决策 2 的"不排队"原则）。
            # 不排队是为了让"车现在到底在不在动"永远可回答。
            if spec.name in self._table.active_skills():
                res.accepted = False
                res.message = checks.format_reject(
                    f'{spec.name} 已有在途任务 —— 本层一次只跑一个动作，不排队')
                self.get_logger().warn(res.message)
                return res

            self._seq += 1
            task_id = f'{self._seq:06d}-{uuid.uuid4().hex[:8]}'
            try:
                rec = self._table.admit(task_id, spec, req.principal,
                                        req.request_id, now=time.monotonic())
            except TaskTableFull as e:
                res.accepted = False
                res.message = checks.format_reject(str(e))
                self.get_logger().error(res.message)
                return res

        res.accepted = True
        res.task_id = task_id
        res.state = rec.state
        res.timeout_s = spec.timeout_s
        res.message = f'已受理（{spec.tier} 层，超时 {spec.timeout_s:g}s）'
        self.get_logger().info(
            f'受理 {task_id} | {spec.name} | {req.principal} | 参数 {result.args}')

        self._pool.submit(self._dispatch, task_id, spec, result.args)
        return res

    # ---------- 服务：cancel / get_result / list ----------

    def on_cancel(self, req, res):
        rec, changed = self._table.request_cancel(req.task_id, req.reason)
        if rec is None:
            res.success = False
            res.message = f'未知的 task_id {req.task_id}'
            return res
        if rec.terminal:
            res.success = False
            res.message = f'任务已处于终态 {rec.state}，无需取消'
            return res
        if not changed:
            res.success = True
            res.message = '取消请求此前已置位'
            return res
        self.get_logger().warn(f'取消 {req.task_id}（{req.reason}）—— 会去停技能本身')
        # ⚠️ 真正让它停下来，而不是"不再等它"（见 SkillCancel.srv 的注释）。
        self._stop_skill(rec, reason=req.reason)
        res.success = True
        res.message = '已请求技能停止'
        return res

    def on_get_result(self, req, res):
        rec = self._table.get(req.task_id)
        if rec is None:
            res.known = False
            res.message = '未知的 task_id'
            return res
        res.known = True
        res.finished = rec.terminal
        res.state = rec.state
        res.verified = rec.verified          # ⚠️ 今天恒为 false，见 checks.validate_result
        res.result_json = rec.result_json
        res.message = rec.message
        res.elapsed = rec.elapsed
        res.stamp = _now_msg(self)
        return res

    def on_list(self, _req, res):
        res.skills_json = self.registry.to_json()
        return res

    # ---------- 派发 ----------

    def _dispatch(self, task_id, spec, args):
        """工作线程入口。**任何异常都必须落成终态。**

        ⚠️ 没有终态的任务是"幽灵任务"：调用方会一直等一个永远不会来的事件，
        最后只能靠它自己的超时脱身 —— 而它此时已经无法判断机器人到底停没停。
        所以这里兜底：不管炸成什么样，任务都要被置成 FAILED。
        """
        rec = self._table.get(task_id)
        if rec is None:
            return
        try:
            self._run_dispatch(task_id, spec, args, rec)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'{task_id} 派发时异常：{e!r}')
            # 已经落成终态就别再落一次（那是非法迁移，只会多一条噪声日志）
            if not rec.terminal:
                self._finish(rec, task_state.FAILED, f'网关内部异常：{e!r}')

    def _run_dispatch(self, task_id, spec, args, rec):
        self._table.start(task_id)
        self._publish_event(rec, task_state.RUNNING, '已派发到技能服务')

        srv_cls = _SRV_TYPES.get(spec.srv_type)
        if srv_cls is None:
            self._finish(rec, task_state.FAILED,
                         f'注册表声明了未知的 srv_type {spec.srv_type}')
            return

        cli = self._client(spec.target, srv_cls)
        if not cli.wait_for_service(timeout_sec=float(
                self.get_parameter('service_wait_timeout').value)):
            self._finish(rec, task_state.FAILED,
                         f'技能服务 {spec.target} 不可用（底层节点没在跑？）')
            return

        request = srv_cls.Request()
        for name, value in args.items():
            setattr(request, name, value)

        started = time.monotonic()
        future = cli.call_async(request)
        deadline = started + spec.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            # ⚠️ 超时 = **取消 + 终态**，不是"悄悄不再等"。
            #    只放弃等待而不停技能，会出现"车还在动，上层以为结束了"。
            self._table.request_cancel(task_id, '超时')
            self._stop_skill(rec, reason='超时')
            self._finish(rec, task_state.CANCELLED,
                         f'超时 {spec.timeout_s:g}s 未返回，已请求技能停止')
            return

        try:
            response = future.result()
        except Exception as e:                                  # noqa: BLE001
            self._finish(rec, task_state.FAILED, f'调用异常：{e}')
            return

        elapsed = time.monotonic() - started
        payload = {k: _to_jsonable(getattr(response, k))
                   for k in response.get_fields_and_field_types()}
        success = bool(getattr(response, 'success', True))
        message = str(getattr(response, 'message', ''))
        if 'elapsed' in payload and isinstance(payload['elapsed'], (int, float)):
            elapsed = float(payload['elapsed'])

        # ⚠️ Result Validation：其中"verified 恒为 false"是**刻意的语义改写**
        #    （control-tier 的 success 只表示"速度按时长发完了"，D-026）。
        verified, problem = checks.validate_result(spec.tier, success, elapsed, message)
        if problem is not None:
            self._finish(rec, task_state.FAILED, f'结果不自洽：{problem}')
            return

        if spec.tier == 'task':
            # **task-tier 的终态由技能自己报**（只有它知道"是走到了"还是"被挡了"），
            # 网关的职责是**校验它报的那个词**合法（D-032）。
            state = str(getattr(response, 'state', ''))
            why = checks.validate_task_terminal(state)
            if why is not None:
                self._finish(rec, task_state.FAILED,
                             f'技能自报的终态不合法：{why}', payload)
                return
            self._finish(rec, state, message or '技能返回', payload, verified)
        elif success:
            self._finish(rec, task_state.FINISHED, message or '技能返回',
                         payload, verified)
        else:
            self._finish(rec, task_state.FAILED, message or '技能报告失败',
                         payload, verified)

    def _stop_skill(self, rec, reason=''):
        """调该技能的取消入口。**best-effort** —— 安全兜底始终是 Safety Runtime
        的独立零速通道（D-027），取消机制出问题不该让"停不下来"成立。"""
        spec = self.registry.get(rec.skill)
        if spec is None or not spec.cancel_target:
            return
        cli = self._client(spec.cancel_target, Trigger)
        if not cli.service_is_ready():
            self.get_logger().warn(
                f'{spec.cancel_target} 不可用 —— 未能请求技能停止（{reason}）')
            return
        cli.call_async(Trigger.Request())
        self.get_logger().info(f'已请求 {spec.cancel_target} 停止（{reason}）')

    def _client(self, target, srv_cls):
        with self._lock:
            cli = self._srv_clients.get(target)
            if cli is None:
                cli = self.create_client(srv_cls, target,
                                         callback_group=self._srv_group)
                self._srv_clients[target] = cli
            return cli

    def _finish(self, rec, state, message, result_json=None, verified=False):
        try:
            done = self._table.finish(
                rec.task_id, state, message=message,
                result_json=json.dumps(result_json or {}, ensure_ascii=False),
                verified=verified)
        except task_state.InvalidTransition as e:
            self.get_logger().error(f'状态迁移被拒（{rec.task_id}）：{e}')
            return

        # ⚠️ **先发事件，再打日志。顺序是刻意的。**
        #    事件是调用方在等的那个东西，日志只是给人看的。曾经这里把日志夹在
        #    「置终态」与「发事件」之间，那行日志一抛异常，任务在表里已经是终态、
        #    而事件从没发出去 —— 调用方只能一直等到自己的超时（"幽灵任务"）。
        #    副作用里，**唯一不能省的那个要排在最前面**。
        try:
            self._publish_event(done, state, message)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'{rec.task_id} 事件发布失败：{e!r}')

        text = f'{rec.task_id} -> {state}：{message}（verified={verified}）'
        # ⚠️ 不要把 `get_logger().info` 取出来存成变量再按级别调用 ——
        #    rclpy 会抛 `ValueError: Logger severity cannot be changed between calls`。
        if state == task_state.FINISHED:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    # ---------- 超时扫描 ----------

    def sweep(self):
        """把过了 deadline 的任务**取消并置终态**。"""
        for rec in self._table.expired():
            self._table.request_cancel(rec.task_id, '超时')
            self._stop_skill(rec, reason='超时')
            self._finish(rec, task_state.CANCELLED,
                         f'超过 {self.registry.require(rec.skill).timeout_s:g}s 未结束，'
                         f'已请求技能停止')

    # ---------- 事件 ----------

    def _publish_event(self, rec, state, detail):
        msg = SkillEvent()
        msg.task_id = rec.task_id
        msg.skill = rec.skill
        msg.state = state
        msg.detail = detail
        # ⚠️ 开环技能**必须**报 -1，不得报一个看起来很合理的假进度（D-026 同一条原则）。
        msg.progress = -1.0
        msg.verified = bool(rec.verified)
        msg.stamp = _now_msg(self)
        self.events.publish(msg)

    # ---------- 收尾 ----------

    def shutdown(self):
        """退出前把在途任务停掉。**best-effort**，不阻塞退出，也**不许抛异常**。

        退出路径上抛异常会把一次正常关闭变成 exit code 1 —— 那会让日志里
        出现一个看起来像故障的痕迹，掩盖真正的故障。
        """
        for rec in self._table.active():
            try:
                self._stop_skill(rec, reason='网关退出')
            except Exception as e:                               # noqa: BLE001
                self.get_logger().warn(f'退出时停止 {rec.task_id} 失败：{e!r}')
        self._pool.shutdown(wait=False)


def main(args=None):
    rclpy.init(args=args)
    node = SkillGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
