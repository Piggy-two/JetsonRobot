#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hybrid Command Router（D-006 的落点）。

```text
    /embodied/command/text   （生产环境可改指 /asr_node/voice_words）
              ↓
        [本地解析，不经 LLM]
              ↓
    ┌─────────┼──────────────┬──────────────┐
    ↓         ↓              ↓              ↓
 安全词    确定性命令      复杂任务      厂商状态文本 / 解析不了
    ↓         ↓              ↓              ↓
/estop   Skill 网关     Agent Runtime   忽略 / 拒绝（不猜）
  服务      ↓             （今天几乎总是
        Control Skill     回"需要 LLM 规划"）
```

> **路由器只负责"这是哪一类"，不负责"能不能做"。** 复杂任务交给 Agent 层判断 ——
> 在路由器里替它下结论，等于把 Agent 的职责抄一份到路由器里，两份迟早不一致。
> 今天 Agent Runtime 的 Planner 是 stub（规则表默认为空），所以复杂任务
> **仍然会被拒绝**，但拒绝是**从 Agent 层发出的**，理由写清了是 Phase 7 未到。

三条设计要点
------------

**1. 安全词判定复用 `embodied_safety_runtime.estop`，绝不重写。**
   整句匹配是安全依据（子串匹配会让「停止追踪」误触发整机急停），必须只有一处。

**2. 一次只处理一条命令，忙时拒绝而不是排队。**
   与 D-026 决策 2 同一个理由：排队会让"机器人现在到底在不在执行"变得不可预测。

   ⚠️ 这一点**必须靠"回调立刻返回 + 活交给工作线程"来实现**，光加一个 busy 标志是不够的 ——
   如果在订阅回调里直接阻塞到命令结束，后来的消息会在 **executor 层**排队，
   busy 判断根本轮不到执行（`MutuallyExclusiveCallbackGroup` 一次只放行一个回调）。
   结果就是"看起来做了拒绝，实际还是在排队"。

**3. 复杂任务明确拒绝，不假装听懂。**
   Agent Task 需要 LLM 规划（Phase 7），今天没有。回一句
   「需要 Agent 规划，当前未实现」比"嗯嗯"一声更诚实，也更容易排查。

⚠️ **本版只吃文本。** 厂商语音链路当前是死的（#25：控制串口完全沉默），
所以默认订 `/embodied/command/text`。等语音修好后把 `text_topic` 指到
`/asr_node/voice_words` 即可，**不用改代码**。

⚠️ **不处理「取消任务」**（详见 `command_parser.py` 顶部）：
它在厂商安全词表里，Safety Runtime 已独立订阅同一话题并会**锁存整机急停**。
本节点再加一条"温和取消"会让同一个词有两个解释。真要做需要与 Safety Runtime
的默认词表一起决策，不能在路由器里单方面改语义。
"""

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import AgentTask, SkillInvoke

from embodied_command_router import command_parser as cp

# 会结束一条命令的终态（SkillEvent 的语义见该 msg 的注释）
TERMINAL_STATES = frozenset({
    'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST', 'BLOCKED', 'FAILED', 'CANCELLED',
    'FINISHED',
})

# 本节点作为调用方的身份 —— 网关按它判 Permission（D-003 的落点）
PRINCIPAL = 'router.deterministic'


class _PendingResult:
    """等待一条命令的终态。

    用一个小类而不是给 `threading.Event` 挂属性：后者靠的是"这个类碰巧没有
    `__slots__`"，属于对标准库实现细节的依赖，读代码的人也不知道那两个属性从哪来。
    """

    def __init__(self):
        self._event = threading.Event()
        self.state = ''
        self.detail = ''

    def set_terminal(self, state, detail):
        self.state = state
        self.detail = detail
        self._event.set()

    def wait(self, timeout):
        return self._event.wait(timeout)


class CommandRouter(Node):
    def __init__(self):
        super().__init__('command_router')

        self.declare_parameter('text_topic', '/embodied/command/text')
        self.declare_parameter('gateway_service', '/skill_gateway/invoke')
        self.declare_parameter('event_topic', '/embodied/skill/events')
        self.declare_parameter('estop_service', '/safety_runtime/estop')
        self.declare_parameter('agent_service', '/agent_runtime/submit')
        self.declare_parameter('result_timeout', 30.0)
        self.declare_parameter('service_wait_timeout', 3.0)
        # ⚠️ 三层闸门里属于本节点的那一层（D-033）。默认 false：
        #    **不转发可能引起运动的命令**。打开它不等于车会动 —— 还要看
        #    网关的 allow_motion 与 Motor Driver 的 dry_run。
        self.declare_parameter('allow_motion', False)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self._lock = threading.Lock()
        self._busy = False
        self._waits = {}          # task_id -> _PendingResult
        # 单线程池：命令**串行**执行，且回调不阻塞（见文件头要点 2）
        self._pool = ThreadPoolExecutor(max_workers=1,
                                        thread_name_prefix='cmd_router')

        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._evt_group = ReentrantCallbackGroup()

        self.create_subscription(String, g('text_topic'), self.on_text, 10,
                                 callback_group=self._sub_group)
        self.create_subscription(SkillEvent, g('event_topic'), self.on_event, 50,
                                 callback_group=self._evt_group)

        self._estop_cli = self.create_client(Trigger, g('estop_service'),
                                             callback_group=self._evt_group)
        self._gw_cli = self.create_client(SkillInvoke, g('gateway_service'),
                                          callback_group=self._evt_group)
        self._agent_cli = self.create_client(AgentTask, g('agent_service'),
                                             callback_group=self._evt_group)

        self.get_logger().info(
            f'命令路由器启动 | 文本 <- {g("text_topic")} | 网关 -> {g("gateway_service")} '
            f'| 急停 -> {g("estop_service")}')
        if not g('allow_motion'):
            self.get_logger().warn(
                '🔒 allow_motion=false：拒绝转发可能引起运动的命令。'
                '打开请显式 allow_motion:=true')

    # ---------- 入口（**立刻返回**） ----------

    def on_text(self, msg):
        text = msg.data
        parsed = cp.parse(text)

        if parsed.kind == cp.IGNORE:
            self.get_logger().debug(f'忽略 {text!r}（{parsed.note}）')
            return

        if parsed.kind == cp.SAFETY:
            self.get_logger().error(f'★ 安全词 {text!r} —— 转发给 Safety Runtime')
            self._forward_estop(text)
            return

        if parsed.kind == cp.AGENT_TASK:
            # 复杂任务**交给 Agent 层判断**，而不是在路由器里替它下结论。
            # 路由器只负责"这是哪一类"，"能不能做"是 Agent 的事（D-006 的分工）。
            self._forward_agent_task(text)
            return

        if parsed.kind == cp.UNPARSED:
            self.get_logger().warn(f'拒绝 {text!r}：{parsed.note}')
            return

        # DETERMINISTIC
        if not self.get_parameter('allow_motion').value:
            self.get_logger().warn(
                f'拒绝 {text!r}：本节点 allow_motion=false —— 不转发可能引起运动的命令')
            return

        with self._lock:
            if self._busy:
                # ⚠️ 不排队。排队会让"机器人现在到底在不在执行"变得不可预测。
                self.get_logger().warn(
                    f'拒绝 {text!r}：上一条命令还在执行（本层一次只处理一条，不排队）')
                return
            self._busy = True

        self.get_logger().info(
            f'解析 {text!r} → {parsed.skill} {parsed.args}（{parsed.note}）')
        self._pool.submit(self._run_command, parsed)

    # ---------- 安全词 ----------

    def _forward_estop(self, text):
        """转发给 Safety Runtime。

        ⚠️ 这是**转发**，不是安全兜底本身：Safety Runtime 拥有独立零速通道。
        当 `text_topic` 就是语音话题时，它自己也会收到同一条文本 ——
        重复触发急停是无害的（锁存本就幂等）。这里失败只记日志，不掩盖。
        """
        if not self._estop_cli.service_is_ready():
            self.get_logger().error(
                '⚠️ Safety Runtime 的急停服务不可用 —— 本次转发失败'
                '（若 text_topic 就是语音话题，它自己也会收到并处理）')
            return
        self._estop_cli.call_async(Trigger.Request())

    # ---------- 复杂任务 ----------

    def _forward_agent_task(self, text):
        """转发给 Agent Runtime。**本节点不判断"能不能做"** —— 那是 Agent 的事。

        ⚠️ Agent Runtime 不在跑时，**要如实说"没人在处理"**，而不是假装拒绝了 ——
        两者的含义完全不同：一个是"我不会"，一个是"没人听"。
        """
        if not self._agent_cli.service_is_ready():
            self.get_logger().error(
                f'Agent Runtime 不在跑 —— {text!r} 无人处理'
                f'（本节点只负责分类，复杂任务归 Agent 层）')
            return
        req = AgentTask.Request()
        req.text = text
        req.principal = 'router.agent_task'
        req.request_id = ''
        fut = self._agent_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_agent_reply(f, text))

    def _on_agent_reply(self, future, text):
        try:
            res = future.result()
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'调用 Agent Runtime 异常：{e!r}')
            return
        if res.accepted:
            self.get_logger().info(f'Agent 接受 {text!r}：{res.message}（{res.task_id}）')
        else:
            # **原样转述** Agent 给的理由（例如"需要 LLM 规划，Phase 7 未实现"）
            self.get_logger().warn(f'Agent 未接受 {text!r}：{res.message}')

    # ---------- 确定性命令（在工作线程里跑） ----------

    def _run_command(self, parsed):
        try:
            self._dispatch(parsed)
        finally:
            with self._lock:
                self._busy = False

    def _dispatch(self, parsed):
        wait_s = float(self.get_parameter('service_wait_timeout').value)
        if not self._gw_cli.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error('Skill 网关不可用 —— 命令无处可去')
            return

        req = SkillInvoke.Request()
        req.principal = PRINCIPAL
        req.skill = parsed.skill
        req.args_json = json.dumps(parsed.args, ensure_ascii=False)
        req.request_id = ''

        future = self._gw_cli.call_async(req)
        if not self._wait_future(future, wait_s):
            self.get_logger().error('网关受理超时')
            return
        try:
            res = future.result()
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'调用网关异常：{e}')
            return

        if not res.accepted:
            # 拒绝原因可能来自权限 / 范围 / allow_motion —— 原样转述，别自己解释
            self.get_logger().warn(f'网关拒绝：{res.message}')
            return

        self.get_logger().info(
            f'已受理 {res.task_id}（{res.state}，超时 {res.timeout_s:g}s），等终态…')

        pending = _PendingResult()
        with self._lock:
            self._waits[res.task_id] = pending
        try:
            timeout = float(self.get_parameter('result_timeout').value)
            if not pending.wait(timeout):
                self.get_logger().error(
                    f'{res.task_id} 在 {timeout:g}s 内没有终态事件 —— '
                    f'结果未知，**不要**当成已完成')
                return
            self.get_logger().info(
                f'{res.task_id} 终态 {pending.state}：{pending.detail}')
        finally:
            with self._lock:
                self._waits.pop(res.task_id, None)

    # ---------- 事件 ----------

    def on_event(self, msg):
        if msg.state not in TERMINAL_STATES:
            return
        with self._lock:
            pending = self._waits.get(msg.task_id)
        if pending is not None:
            pending.set_terminal(msg.state, msg.detail)

    # ---------- 小工具 ----------

    def _wait_future(self, future, timeout_s):
        """等一个 service future（单调钟计时，见 #24 / DEV_NOTES 坑 13）。"""
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def shutdown(self):
        self._pool.shutdown(wait=False)


def main(args=None):
    rclpy.init(args=args)
    node = CommandRouter()
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
