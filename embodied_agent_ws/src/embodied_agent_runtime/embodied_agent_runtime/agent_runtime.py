#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Agent Runtime（第一批骨架）。

```text
   /agent_runtime/submit（AgentTask：自然语言任务）
              ↓
        Planner（**规则表**，不是 LLM —— Phase 7 才换）
              ↓  只允许命名 task-tier 技能（D-003，代码级校验）
        Skill Gateway ~/invoke
              ↓
        task-tier 技能（今天只有 autonomous.advance_until_blocked）
              ↓  /embodied/skill/events
        Event Manager ──→ Executor（WAIT 推进）──→ Memory（有界）
```

**本版有什么、没有什么**（说清楚比含糊更省事）：

| 组件 | 状态 |
|---|---|
| Executor（WAIT 推进 / 超时 / 唤醒判定） | ✅ 已实现，**纯逻辑 + 单测** |
| Event Manager | ✅ 已实现（订阅事件，按 task_id 派发） |
| Memory（有界历史） | ✅ 已实现 |
| Planner | ⚠️ **stub**：按一张规则表查表，查不到就明确拒绝 |
| LLM 规划 / 重规划 | ❌ Phase 7 |

⚠️ **规则表默认是空的** ⇒ 任何自然语言任务都会被明确拒绝
（「需要 LLM 规划，当前未实现（Phase 7）」）。**这是刻意的**，不是缺陷。

D-004 的两条语义在这里落地
--------------------------
1. **WAIT 不阻塞**：受理后记一笔就返回事件循环，没有任何地方在"等"
   （理由见 `execution.py` 顶部）。
2. **只有任务级事件唤醒 Agent**：`FINISHED`（control-tier）**不唤醒** ——
   它只表示"技能返回了"，不等于"任务结束了"。

⚠️ 超时 = **取消 + 终态**，不是"不再等了"（D-030）。超时后本节点会去调网关的
`~/cancel`，让它**真的把技能停掉**。

⚠️ **未在真机上验证过**：车从未被这条链路驱动过。受 `allow_motion` 闸门约束（D-033）。
"""

import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import AgentTask, SkillCancel, SkillInvoke

from embodied_agent_runtime import execution, planner
from embodied_agent_runtime.memory import AgentMemory

# 网关的注册表是"有哪些技能"的唯一事实来源 —— 本节点直接读**同一份数据**，
# 而不是再抄一份技能名单（抄一份就会漂移）。D-029：Registry 是数据。
from embodied_skill_gateway.registry import Registry


class AgentRuntime(Node):
    def __init__(self):
        super().__init__('agent_runtime')

        self.declare_parameter('gateway_service', '/skill_gateway/invoke')
        self.declare_parameter('gateway_cancel_service', '/skill_gateway/cancel')
        self.declare_parameter('event_topic', '/embodied/skill/events')
        self.declare_parameter('status_topic', '/embodied/agent/status')
        self.declare_parameter('registry_file', '')
        # ⚠️ 默认空表 ⇒ 任何任务都被拒绝。**这是刻意的**（见模块 docstring）。
        #
        # ⚠️ 规则表是**文件**不是命令行字符串：`ros2 launch ... rules_json:='{...}'`
        #    会被 launch 当成 **YAML** 解析成一个 dict，参数系统直接拒收
        #    （`Allowed value types are bytes, bool, int, float, str, ... Got <class 'dict'>`）。
        #    规则表本来就是**数据**，跟注册表一样放文件里更合适。
        self.declare_parameter('rules_file', '')
        self.declare_parameter('task_capacity', 100)
        self.declare_parameter('memory_capacity', 200)
        self.declare_parameter('submit_timeout', 0.0)
        self.declare_parameter('service_wait_timeout', 3.0)
        self.declare_parameter('sweep_rate', 5.0)
        # ⚠️ 三层闸门的第一层（D-033）。默认 false —— 本节点能间接让车动。
        self.declare_parameter('allow_motion', False)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        reg_file = str(g('registry_file'))
        if not reg_file:
            import os
            from ament_index_python.packages import get_package_share_directory
            reg_file = os.path.join(get_package_share_directory('embodied_skill_gateway'),
                                    'config', 'skill_registry.yaml')
        self.registry = Registry.from_yaml(reg_file)

        self.rules = self._load_rules(str(g('rules_file')))

        self.memory = AgentMemory(capacity=int(g('memory_capacity')))
        self._executor = execution.Executor(capacity=int(g('task_capacity')))
        self._lock = threading.Lock()

        self._evt_group = ReentrantCallbackGroup()
        self._srv_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        self.status_pub = self.create_publisher(Float64MultiArray, g('status_topic'), 10)
        self.create_subscription(SkillEvent, g('event_topic'), self.on_event, 50,
                                 callback_group=self._evt_group)

        self._gateway_cli = self.create_client(
            SkillInvoke, g('gateway_service'), callback_group=self._srv_group)
        self._cancel_cli = self.create_client(
            SkillCancel, g('gateway_cancel_service'), callback_group=self._srv_group)

        self.create_service(AgentTask, '~/submit', self.on_submit,
                            callback_group=self._srv_group)
        self.create_timer(1.0 / float(g('sweep_rate')), self.sweep,
                          callback_group=self._timer_group)

        self.get_logger().info(
            f'Agent Runtime 启动 | 规则 {len(self.rules)} 条 | '
            f'事件 <- {g("event_topic")} | 网关 -> {g("gateway_service")}')
        if not self.rules:
            self.get_logger().warn(
                '⚠️ 规则表为空 ⇒ 任何自然语言任务都会被**明确拒绝**'
                '（需要 LLM 规划，Phase 7 未实现）。这是刻意的默认值')
        if not g('allow_motion'):
            self.get_logger().warn(
                '🔒 allow_motion=false：拒绝派发可能引起运动的技能（D-033）')

    # ---------- 规则表 ----------

    @staticmethod
    def _load_rules(path):
        """从 YAML 文件加载规则表。返回 `{文本: {'skill':..., 'args': {...}}}`。

        文件格式（`config/rules_example.yaml` 有完整例子）：

            rules:
              向前走一小段:
                skill: autonomous.advance_until_blocked
                args: {max_distance: 0.2, clear_range: 0.5, step: 0.1}

        ⚠️ 文件写错**必须炸**（而不是静默当成空表）—— 否则表现为
        "什么都规划不出来"，而真正的原因是格式写错了，排查方向会跑偏。
        """
        if not path:
            return {}
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f'规则表顶层必须是 mapping，得到 {type(data).__name__}')
        rules = data.get('rules', data)
        if not isinstance(rules, dict):
            raise RuntimeError('规则表的 rules 必须是 mapping（文本 → {skill, args}）')
        for text, rule in rules.items():
            if not isinstance(rule, dict) or 'skill' not in rule:
                raise RuntimeError(f'规则 {text!r} 必须是一个含 skill 的 mapping，'
                                   f'得到 {rule!r}')
        return rules

    # ---------- 入口 ----------

    def on_submit(self, req, res):
        """规划一个自然语言任务。**不做任何阻塞等待** —— 受理即返回。"""
        result = planner.plan(req.text, self.rules, self.registry)

        if not result.accepted:
            res.accepted = False
            res.message = result.reason
            self.memory.remember_submission(
                '', '', req.principal, False, result.reason, now=time.monotonic())
            self.get_logger().warn(f'拒绝任务 {req.text!r}：{result.reason}')
            return res

        spec = self.registry.get(result.skill)
        if spec.causes_motion and not self.get_parameter('allow_motion').value:
            res.accepted = False
            res.message = (f'allow_motion=false —— 拒绝派发可能引起运动的技能 '
                           f'{result.skill}（打开请显式 allow_motion:=true）')
            self.get_logger().warn(res.message)
            return res

        invoke = SkillInvoke.Request()
        invoke.principal = 'agent.planner'
        invoke.skill = result.skill
        invoke.args_json = json.dumps(result.args, ensure_ascii=False)
        invoke.request_id = req.request_id

        if not self._gateway_cli.wait_for_service(
                timeout_sec=float(self.get_parameter('service_wait_timeout').value)):
            res.accepted = False
            res.message = 'Skill 网关不可用 —— 任务无处可去'
            self.get_logger().error(res.message)
            return res

        future = self._gateway_cli.call_async(invoke)
        wait = float(self.get_parameter('service_wait_timeout').value)
        deadline = time.monotonic() + wait
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            res.accepted = False
            res.message = '网关受理超时'
            self.get_logger().error(res.message)
            return res

        reply = future.result()
        if not reply.accepted:
            res.accepted = False
            res.message = f'网关拒绝：{reply.message}'
            self.memory.remember_submission('', result.skill, req.principal,
                                            False, reply.message, now=time.monotonic())
            self.get_logger().warn(res.message)
            return res

        with self._lock:
            rec = self._executor.submit(reply.task_id, result.skill,
                                        'agent.planner', reply.timeout_s)
        if rec is None:
            # 记不上就**去取消** —— 否则会有一个任务在跑而没人跟踪它。
            res.accepted = False
            res.message = '在途任务表已满 —— 已请求取消这次派发'
            self.get_logger().error(res.message)
            self._request_cancel(reply.task_id, 'Agent 侧无法跟踪')
            return res

        self.memory.remember_submission(reply.task_id, result.skill, req.principal,
                                        True, '', now=time.monotonic())
        res.accepted = True
        res.task_id = reply.task_id
        res.message = f'已受理并派发 {result.skill}（处于 WAIT，事件到达时唤醒）'
        self.get_logger().info(
            f'受理 {reply.task_id}：{result.skill} {result.args} —— 进入 WAIT')

        # 把网关给的超时也纳入本节点的兜底扫描：**谁受理谁负责收尾**。
        rec.deadline = min(rec.deadline, time.monotonic() + reply.timeout_s + 5.0)
        return res

    # ---------- 事件 ----------

    def on_event(self, msg):
        with self._lock:
            verdict = self._executor.on_event(msg.task_id, msg.state)
        if verdict == execution.IGNORE:
            return
        if verdict == execution.RECORD:
            self.get_logger().debug(
                f'{msg.task_id} -> {msg.state}（非任务级终态，不唤醒）')
            return
        if verdict == execution.DUPLICATE:
            self.get_logger().warn(
                f'{msg.task_id} 又收到一个终态 {msg.state} —— 忽略（不重复唤醒）')
            return

        rec = self._executor.get(msg.task_id)
        self.memory.remember_wakeup(msg.task_id, msg.skill, msg.state, msg.detail,
                                    now=time.monotonic())
        self.get_logger().error(
            f'★ 唤醒 Agent：{msg.task_id}｜{msg.skill}｜{msg.state}｜{msg.detail}'
            f'｜verified={msg.verified}')
        self.get_logger().warn(
            '⚠️ 本版**没有重规划**（Phase 7 才接 LLM）—— 到这里为止：'
            'Agent 知道任务终止了、终止在什么状态')
        if rec is not None and not msg.verified:
            self.get_logger().warn(
                f'⚠️ {msg.state} 是**技能自报**的，未经独立反馈确认'
                f'（verified=false）—— 不要把它读成"已经到位"')

    # ---------- 超时兜底 ----------

    def sweep(self):
        """超时 = **取消 + 终态**（D-030），不是"不再等了"。"""
        with self._lock:
            overdue = self._executor.expired()
        for rec in overdue:
            self.get_logger().error(
                f'{rec.task_id} 超过 {rec.deadline - rec.submitted_at:.1f}s 仍无终态 '
                f'—— 请求取消技能（**不**当作已完成）')
            self._request_cancel(rec.task_id, 'Agent 侧超时')
            with self._lock:
                self._executor.on_event(rec.task_id, 'FAILED')
        self._publish_status()

    def _request_cancel(self, task_id, reason):
        if not self._cancel_cli.service_is_ready():
            self.get_logger().warn(
                f'网关 cancel 服务不可用 —— 未能请求取消 {task_id}（{reason}）')
            return
        req = SkillCancel.Request()
        req.task_id = task_id
        req.reason = reason
        self._cancel_cli.call_async(req)

    # ---------- 状态 ----------

    def _publish_status(self):
        s = self._executor.stats()
        snap = self.memory.snapshot()
        msg = Float64MultiArray()
        msg.data = [
            float(s['tracked']),
            float(s['waiting']),
            float(snap['wakeups']),
            float(snap['dropped']),
        ]
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AgentRuntime()
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
