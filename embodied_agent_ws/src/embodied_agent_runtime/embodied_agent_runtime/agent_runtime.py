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

from embodied_agent_runtime import execution, llm_client, llm_planner, planner
from embodied_agent_runtime.memory import AgentMemory

# 网关的注册表是"有哪些技能"的唯一事实来源 —— 本节点直接读**同一份数据**，
# 而不是再抄一份技能名单（抄一份就会漂移）。D-029：Registry 是数据。
from embodied_skill_gateway import task_state as ts
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

        # ---- 云端 LLM 规划（Phase 7 / D-038）----
        # ⚠️ **默认关闭**，而且是刻意的两条理由：
        #   ① 它会把**用户说的话发到第三方**（对外发送数据）；
        #   ② 与 dry_run / allow_motion / require_safety 一样，闸门默认在保守那侧。
        # 关掉时"复杂任务"仍然被**明确拒绝**，理由会说清是"没开"而不是"不会"。
        self.declare_parameter('llm_enabled', False)
        self.declare_parameter('llm_base_url', '')
        self.declare_parameter('llm_model', '')
        # ⚠️ 只写**变量名**，不写密钥本身 —— 密钥绝不进仓库（CLAUDE.md §6）。
        self.declare_parameter('llm_api_key_env', '')
        self.declare_parameter('llm_timeout', 8.0)
        self.declare_parameter('llm_max_tokens', 400)

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
        # 单飞闸门：护住"问 LLM"那一跳（见 llm_planner.plan_task 的说明）
        self._planning_lock = threading.Lock()

        self.llm, self._llm_why = self._build_llm(g)

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
                '⚠️ 规则表为空 ⇒ 只有 LLM 那一跳能规划；它没开时任何自然语言任务'
                '都会被**明确拒绝**。这是刻意的默认值')
        if not g('allow_motion'):
            self.get_logger().warn(
                '🔒 allow_motion=false：拒绝派发可能引起运动的技能（D-033）')
        if self.llm is not None:
            self.get_logger().info(
                f'LLM 规划已启用：{g("llm_base_url")} 模型 {g("llm_model")} | '
                f'上限 {self.llm.timeout_s:g}s | '
                f'菜单里 {len(llm_planner.skill_menu(self.registry))} 个 task-tier 技能 | '
                f'⚠️ 用户文本会**发往该端点**')
        else:
            self.get_logger().warn(
                f'LLM 规划这一跳没开：{self._llm_why} —— 规则表没命中的任务'
                '会被明确拒绝，且理由里会写清是"没开"而不是"不会"（D-038）')

    # ---------- 云端 LLM（Phase 7 / D-038）----------

    def _build_llm(self, g):
        """配好 LLM 客户端；任何一步不满足就返回 `(None, 为什么)`。

        ⚠️ **读不到密钥不是启动错误**，只是这一跳没开 —— 机器人本来就该在
        没有网络的房间里继续做它本地能做的事（`plan.md` §18 的降级要求）。
        但**理由必须留得下来**，好在每次拒绝时如实告诉人。
        """
        if not g('llm_enabled'):
            return None, llm_planner.REFUSE_LLM_DISABLED
        base_url = str(g('llm_base_url') or '').strip()
        model = str(g('llm_model') or '').strip()
        if not base_url or not model:
            return None, '没有配置 llm_base_url / llm_model'
        env_name = str(g('llm_api_key_env') or '').strip()
        if not env_name:
            return None, '没有配置 llm_api_key_env（密钥只从环境变量读，不进仓库）'
        key = llm_client.read_api_key(env_name)
        if not key:
            # ⚠️ **不回显变量值**，只说变量名 —— 它可能被写成了密钥本身
            return None, f'环境变量 {env_name} 里没有读到 API key'

        timeout_s = float(g('llm_timeout'))
        budget = float(g('submit_timeout'))
        if budget > 0:
            # `submit_timeout` 从此有了真实含义：**整个 submit 路径的预算**。
            # 它约束的是这条路上最慢的那一跳（就是这次网络调用）。
            timeout_s = min(timeout_s, budget)
        return llm_client.OpenAiCompatClient(
            base_url=base_url, model=model, api_key=key,
            timeout_s=timeout_s, max_tokens=int(g('llm_max_tokens'))), ''

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
            raise RuntimeError('规则表的 rules 必须是 mapping（文本 → 规则）')
        for text, rule in rules.items():
            # ⚠️ 校验**复用 planner 里那个判定函数**，不在这里另写一份 ——
            #    否则"加载时认为合法的写法"与"规划时认为合法的写法"会分家，
            #    而症状是启动时报错、或反过来启动不报错却永远规划不出来。
            steps, why = planner._steps_from_rule(rule)
            if steps is None:
                raise RuntimeError(f'规则 {text!r} 不合法：{why}（得到 {rule!r}）')
        return rules

    # ---------- 入口 ----------

    def on_submit(self, req, res):
        """规划一个自然语言任务。**受理即返回**（技能的真正执行是异步的）。

        ⚠️ 两跳：**规则表优先**（离线、确定、快），没命中才问 LLM（D-038）。
        问 LLM 会让**这一个回调**阻塞几秒 —— 单飞闸门保证同一时刻只有一次
        （见 `llm_planner.plan_task` 里那段说明）。
        """
        t0 = time.monotonic()
        plan = llm_planner.plan_task(
            req.text, self.rules, self.registry, self.llm, guard=self._planning_lock)
        result, source, raw = plan.result, plan.source, plan.raw
        elapsed = time.monotonic() - t0

        # 规划痕迹进日志：**哪一跳**、**耗时**、以及 LLM 的**原始回包**。
        # 模型说了什么必须留得下来 —— 拒绝理由面向人，原文面向排查。
        self.get_logger().info(
            f'规划 {req.text!r} → {source}｜{elapsed * 1000:.0f} ms｜'
            f'{"接受 " + self._plan_label(result.steps) if result.accepted else "拒绝 " + result.reason}')
        if raw:
            # 截断，但留足能看出它在胡说多少的长度
            self.get_logger().info(f'LLM 原始回包：{raw[:500]!r}')

        if not result.accepted:
            res.accepted = False
            res.message = result.reason
            self.memory.remember_submission(
                '', '', req.principal, False, result.reason, now=time.monotonic())
            self.get_logger().warn(f'拒绝任务 {req.text!r}：{result.reason}')
            return res

        # 每一步都要过 `allow_motion` 闸门：**只要任何一步会引起运动**，
        # 整条计划都要显式放行才派发（闸门③，D-033）。
        movers = [s.skill for s in result.steps
                  if getattr(self.registry.get(s.skill), 'causes_motion', False)]
        if movers and not self.get_parameter('allow_motion').value:
            res.accepted = False
            res.message = (f'allow_motion=false —— 拒绝派发可能引起运动的技能 '
                           f'{movers}（打开请显式 allow_motion:=true）')
            self.get_logger().warn(res.message)
            return res

        # 派发第 1 步。**计划 id 就用第 1 步的网关任务号**：
        # 这样单步计划与从前**完全一致**（同一个 id、同一条事件、同一套回归），
        # 多步计划多出来的只是"后续步骤也绑在同一个计划上"（D-043）。
        reply, why = self._dispatch_step(result.steps[0], req.request_id)
        if reply is None:
            res.accepted = False
            res.message = why
            self.get_logger().error(res.message)
            return res
        if not reply.accepted:
            res.accepted = False
            res.message = f'网关拒绝：{reply.message}'
            self.memory.remember_submission('', result.steps[0].skill, req.principal,
                                            False, reply.message, now=time.monotonic())
            self.get_logger().warn(res.message)
            return res

        with self._lock:
            rec = self._executor.submit_plan(reply.task_id, result.steps,
                                             'agent.planner', reply.timeout_s)
            if rec is not None:
                # 把网关任务号绑到第 1 步上 —— 不绑就认不出它的事件（见 execution.py）
                self._executor.bind_step(reply.task_id, 0, reply.task_id)
        if rec is None:
            # 记不上就**去取消** —— 否则会有一个任务在跑而没人跟踪它。
            res.accepted = False
            res.message = '在途任务表已满 —— 已请求取消这次派发'
            self.get_logger().error(res.message)
            self._request_cancel(reply.task_id, 'Agent 侧无法跟踪')
            return res

        self.memory.remember_submission(reply.task_id, self._plan_label(result.steps),
                                        req.principal, True, '',
                                        now=time.monotonic())
        res.accepted = True
        res.task_id = reply.task_id
        n = len(result.steps)
        res.message = (f'已受理并派发 {result.steps[0].skill}'
                       f'（{"单步" if n == 1 else f"共 {n} 步，第 1 步已派发"}；'
                       f'处于 WAIT，事件到达时唤醒）')
        self.get_logger().info(
            f'受理 {reply.task_id}：{self._plan_label(result.steps)} —— 进入 WAIT')

        # 把网关给的超时也纳入本节点的兜底扫描：**谁受理谁负责收尾**。
        rec.deadline = min(rec.deadline, time.monotonic() + reply.timeout_s + 5.0)
        return res

    # ---------- 派发 ----------

    @staticmethod
    def _plan_label(steps):
        """计划在人看的地方长什么样：`A → B → C`。单步就是 `A`。"""
        return ' → '.join(s.skill for s in steps)

    def _dispatch_step(self, step, request_id):
        """把**一步**交给网关。返回 `(reply, 失败原因)`；失败时 reply 为 None。

        ⚠️ 每一步都走**同一个** `~/invoke` —— 多步没有旁路，
        第 2 步的六项检查与第 1 步一模一样。
        """
        invoke = SkillInvoke.Request()
        invoke.principal = 'agent.planner'
        invoke.skill = step.skill
        invoke.args_json = json.dumps(step.args, ensure_ascii=False)
        invoke.request_id = request_id

        wait = float(self.get_parameter('service_wait_timeout').value)
        if not self._gateway_cli.wait_for_service(timeout_sec=wait):
            return None, 'Skill 网关不可用 —— 任务无处可去'

        future = self._gateway_cli.call_async(invoke)
        deadline = time.monotonic() + wait
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None, f'网关受理超时（{step.skill}）'
        return future.result(), ''

    # ---------- 事件 ----------

    def on_event(self, msg):
        with self._lock:
            outcome = self._executor.on_event(msg.task_id, msg.state)
        if outcome.action == execution.IGNORE:
            return
        if outcome.action == execution.RECORD:
            self.get_logger().debug(
                f'{msg.task_id} -> {msg.state}（{outcome.reason or "非任务级终态"}）')
            return
        if outcome.action == execution.DUPLICATE:
            self.get_logger().warn(
                f'{msg.task_id} 又收到一个终态 {msg.state} —— 忽略（不重复唤醒）')
            return

        # ★ 还有下一步：**去派发它**，此时**不唤醒** Agent（D-043）。
        if outcome.action == execution.DISPATCH_NEXT:
            self.get_logger().info(
                f'计划继续：{outcome.reason}｜下一步 第 {outcome.step_index} 步')
            # ⚠️ 用 **outcome.plan_id**，不是 msg.task_id ——
            #    后者是**网关任务号**，第 2 步之后就不是计划 id 了。
            self._dispatch_next(outcome.plan_id, outcome.step_index)
            return

        # ---- 到这里就是**计划级终态**：唤醒 Agent 一次 ----
        rec = self._executor.get(outcome.plan_id)
        plan = list(rec.steps) if rec is not None else []
        self.memory.remember_wakeup(outcome.plan_id, msg.skill, msg.state, msg.detail,
                                    now=time.monotonic())
        self.get_logger().error(
            f'★ 唤醒 Agent：{outcome.plan_id}｜{self._plan_label(plan) if plan else msg.skill}'
            f'｜{msg.state}｜{outcome.reason or msg.detail}｜verified={msg.verified}')
        self.get_logger().warn(
            '⚠️ 本版**没有重规划**（Phase 7 才接 LLM）—— 到这里为止：'
            'Agent 知道任务终止了、终止在什么状态')
        if rec is not None and not msg.verified:
            self.get_logger().warn(
                f'⚠️ {msg.state} 是**技能自报**的，未经独立反馈确认'
                f'（verified=false）—— 不要把它读成"已经到位"')

    def _dispatch_next(self, plan_id, step_index):
        """派发计划里的下一 步（`step_index` 是**人看的序号**，1 起）。"""
        rec = self._executor.get(plan_id)
        if rec is None:
            return
        idx = step_index - 1
        if not (0 <= idx < rec.total):
            self.get_logger().error(f'计划 {plan_id} 没有第 {step_index} 步')
            return
        step = rec.steps[idx]
        reply, why = self._dispatch_step(step, f'{plan_id}-s{step_index}')
        if reply is None or not reply.accepted:
            # ⚠️ 后续步骤派不出去 ⇒ **整条计划到此为止**，而且必须**唤醒 Agent**：
            #    不唤醒就会有一条计划永远挂在 WAIT 里（超时兜底会收它，但那是兜底）。
            reason = why or getattr(reply, 'message', '被网关拒绝')
            with self._lock:
                self._executor.cancel(plan_id)
            self.get_logger().error(
                f'★ 计划 {plan_id} 的第 {step_index} 步派发失败：{reason}'
                f' —— 计划中止并唤醒 Agent')
            self.memory.remember_wakeup(plan_id, step.skill, ts.FAILED, reason,
                                        now=time.monotonic())
            return
        with self._lock:
            self._executor.bind_step(plan_id, idx, reply.task_id)
        self.get_logger().info(
            f'计划 {plan_id}：第 {step_index} 步已派发 {step.skill}'
            f'（网关任务 {reply.task_id}）')

    # ---------- 超时兜底 ----------

    def sweep(self):
        """超时 = **取消 + 终态**（D-030），不是"不再等了"。"""
        with self._lock:
            overdue = self._executor.expired()
        for rec in overdue:
            with self._lock:
                # 要取消的是**当前那一步**在网关那边的任务，不是计划 id ——
                # 走到第 3 步时，计划 id 指的是第 1 步，取消它等于什么也没停。
                step_task = self._executor.current_step_task_id(rec.task_id)
            self.get_logger().error(
                f'{rec.task_id} 超过 {rec.deadline - rec.submitted_at:.1f}s 仍无终态 '
                f'（停在第 {rec.current + 1}/{rec.total} 步）'
                f'—— 请求取消技能（**不**当作已完成）')
            self._request_cancel(step_task, 'Agent 侧超时')
            with self._lock:
                self._executor.timeout(rec.task_id)
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
