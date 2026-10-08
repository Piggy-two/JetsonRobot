#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 的 WAIT 推进逻辑（**纯 Python，不依赖 ROS**）。

D-004 要的是「Agent 发起 Skill 后进入 **WAIT**，由本地运行时执行；
**只有任务级事件才唤醒 Agent**」。本模块就是那个 WAIT —— 而且是**不阻塞**的那种。

为什么不阻塞
------------
把 WAIT 实现成"在回调里等结果"，会得到三个副作用，每一个都很糟：

  · 等待期间收不到别的指令 → Agent 变成**不可取消**；
  · 等待期间判不了超时 → 变成**不可超时**；
  · 等待占住一个执行槽 → 并发请求在这条线上排队，而本项目的原则是**不排队**
    （D-026 决策 2：排队会让"车现在到底在不在动"变得不可预测）。

所以 WAIT 是**一次纯函数状态推进**：受理时记一笔，事件来了推一下，超时了扫一遍。
没有任何地方在"等"。

"只有任务级事件才唤醒"是硬约束
------------------------------
`task_state.is_waking` 是唯一的判据（task-tier 终态集）。
⚠️ control-tier 的 `FINISHED` **不唤醒** —— 它只表示"技能返回了"。
若把它也算唤醒，Agent 会以为任务完成而继续规划，而实际上车可能只是"走了一小段"。

多步计划（2026-10-08，D-043）
------------------------------
一个**计划**是一串有序步骤。每一步**各自**经网关派发、各自有 task_id 与终态 ——
**多步没有开出任何旁路**：第 2 步的准入检查与第 1 步完全一样。

**Agent 只在计划结束时被唤醒一次**，不是每一步都唤醒。理由：
中途唤醒 Agent 会让它对着一个"只走了一半"的世界重新规划，
而计划本身就是它刚刚做过的规划 —— 那会变成"规划 → 走一步 → 再规划"，
与 D-004 的"事件驱动，而不是实时 LLM 控制器"背道而驰。

推进规则（`ABORT_STATES` 之外一律继续）
---------------------------------------
    · 某步报 `FAILED` / `CANCELLED` ⇒ **整条计划立刻中止**，计划终态 = 那个状态；
    · 其余终态（`ARRIVED` / `BLOCKED` / `TARGET_FOUND` / `TARGET_LOST`）⇒ **继续下一步**。

⚠️ 第二条是**脱困计划成立的前提**：`advance` 被挡住时报的是 `BLOCKED`，
而计划恰恰要在那之后接着转 —— 把 `BLOCKED` 当成中止，脱困就永远走不到第 2 步。
"""

import time
from collections import namedtuple

from embodied_skill_gateway import task_state as ts

# 一次事件处理的结果
WAKE = 'wake'              # 计划级终态 —— **唤醒 Agent**
DISPATCH_NEXT = 'next'     # 还有下一步，**去派发它**（不唤醒）
RECORD = 'record'          # 收到了，但不推进（非终态）
IGNORE = 'ignore'          # 不是我在等的任务
DUPLICATE = 'duplicate'    # 这个计划已经醒过一次了 —— **不重复唤醒**

#: 一票中止的终态。**只有这两个** —— 见模块顶部"推进规则"。
#: ⚠️ `BLOCKED` **不在**这里，这是刻意的（脱困计划靠它继续）。
ABORT_STATES = (ts.FAILED, ts.CANCELLED)

# 推进结果：做什么、**属于哪个计划**、下一步是第几步（1 起）、计划终态、说明。
#
# ⚠️ `plan_id` 必须带出来：事件里给的是**网关任务号**，而调用方要按**计划 id**
#    去取记录（派发下一步、读计划里剩下的步骤）。
#    少了它，调用方只好拿事件里的任务号去查 —— 那在第 2 步之后**查不到**，
#    表现是"计划推进判出来了、却静默不派发下一步"（真机端到端第一次就撞上了）。
Outcome = namedtuple('Outcome', 'action plan_id step_index terminal reason')


class PendingPlan:
    """一个在途**计划**。`steps[i]` 是第 i 步（0 起），`step_task_ids[i]` 是它在网关那边的任务号。"""

    __slots__ = ('task_id', 'principal', 'steps', 'step_task_ids', 'current',
                 'submitted_at', 'deadline', 'woken', 'woken_state',
                 'last_step_state', 'dispatched')

    def __init__(self, task_id, steps, principal, timeout_s, now):
        self.task_id = task_id
        self.principal = principal
        self.steps = list(steps)
        self.step_task_ids = [None] * len(self.steps)
        self.current = 0                 # 正在执行的步骤下标（0 起）
        self.dispatched = False          # 第 1 步发出去了没有
        self.submitted_at = now
        self.deadline = now + float(timeout_s)
        self.woken = False
        self.woken_state = ''
        self.last_step_state = ''

    @property
    def total(self):
        return len(self.steps)

    def index_of(self, step_task_id):
        """这个网关任务号属于本计划的第几步（0 起）；不属于则 -1。"""
        try:
            return self.step_task_ids.index(step_task_id)
        except ValueError:
            return -1

    def expires_at(self, now):
        return not self.woken and now >= self.deadline


class Executor:
    """在途计划的簿记。**有界**（理由同 memory：内存只有 7.4 GiB，#7）。"""

    def __init__(self, capacity=100):
        if capacity < 1:
            raise ValueError(f'capacity 必须 ≥ 1，得到 {capacity}')
        self.capacity = int(capacity)
        self._plans = {}          # task_id(计划) -> PendingPlan
        self._by_step = {}        # 网关任务号 -> 计划 task_id

    # ---------- 提交 ----------

    def submit_plan(self, task_id, steps, principal, timeout_s, now=None):
        """记下一个在途计划。返回 `PendingPlan`，或 None（表满 / 重复 task_id）。"""
        now = time.monotonic() if now is None else now
        if task_id in self._plans or not steps:
            return None
        self._evict()
        if len(self._plans) >= self.capacity:
            return None
        rec = PendingPlan(task_id, steps, principal, timeout_s, now)
        self._plans[task_id] = rec
        return rec

    def bind_step(self, task_id, step_index, step_task_id):
        """把网关返回的任务号绑到第 `step_index` 步上。

        ⚠️ 必须绑：事件里带的是**网关那边的 task_id**，不是计划的。
        不绑就会出现"事件来了却不认识它"——表现为任务永远 WAIT 到超时。
        """
        rec = self._plans.get(task_id)
        if rec is None or not (0 <= step_index < rec.total):
            return False
        rec.step_task_ids[step_index] = step_task_id
        rec.dispatched = True
        self._by_step[step_task_id] = task_id
        return True

    def _evict(self):
        """满了先淘汰**已经醒过的**最老记录。还在等的计划一条都不能丢。"""
        if len(self._plans) < self.capacity:
            return
        for task_id, rec in list(self._plans.items()):
            if rec.woken:
                for stid in rec.step_task_ids:
                    self._by_step.pop(stid, None)
                del self._plans[task_id]
                if len(self._plans) < self.capacity:
                    return

    # ---------- 事件 ----------

    def on_event(self, task_id, state, now=None):
        """收到一条 `SkillEvent`，推进一次。`task_id` 是**网关那边的**任务号。

        返回 `Outcome`。只有 `WAKE` 才唤醒 Agent。
        """
        plan_id = self._by_step.get(task_id)
        if plan_id is None:
            # 不是我在等的任务 —— 可能是别的调用方（路由器 / 人工工装）发起的。
            # **不要**因此报警：事件话题是共享的，看见别人的事件是正常的。
            return Outcome(IGNORE, '', None, '', '')
        rec = self._plans.get(plan_id)
        if rec is None or rec.woken:
            return Outcome(DUPLICATE if rec else IGNORE, plan_id, None, '', '')

        idx = rec.index_of(task_id)
        if idx < 0:
            return Outcome(IGNORE, '', None, '', '')
        if idx != rec.current:
            # 迟到的旧步骤事件（比如超时后又被停下来的那次）。**不要**拿它推进计划：
            # 那会让第 3 步的终态去驱动一个已经走到第 5 步的计划。
            return Outcome(RECORD, plan_id, None, '',
                           f'第 {idx + 1} 步不是当前步，忽略')

        if not ts.is_waking(state):
            rec.last_step_state = state
            return Outcome(RECORD, plan_id, idx + 1, '',
                           f'第 {idx + 1} 步 {state}（非终态）')

        rec.last_step_state = state

        if state in ABORT_STATES:
            rec.woken = True
            rec.woken_state = state
            return Outcome(WAKE, plan_id, idx + 1, state,
                           f'第 {idx + 1}/{rec.total} 步报 {state} ⇒ 整条计划中止')

        if idx + 1 < rec.total:
            rec.current = idx + 1
            return Outcome(DISPATCH_NEXT, plan_id, rec.current + 1, '',
                           f'第 {idx + 1}/{rec.total} 步报 {state} ⇒ 继续下一步')

        rec.woken = True
        rec.woken_state = state
        return Outcome(WAKE, plan_id, idx + 1, state,
                       f'最后一步（第 {idx + 1}/{rec.total} 步）报 {state} ⇒ 计划结束')

    # ---------- 超时 ----------

    def expired(self, now=None):
        """过了 deadline 还没醒的计划。调用方要**去把技能停掉**，而不是只放弃等待。"""
        now = time.monotonic() if now is None else now
        return [r for r in self._plans.values() if r.expires_at(now)]

    def timeout(self, plan_id):
        """把一个过期的计划**直接**标成终态（`FAILED`）。返回是否找到。

        ⚠️ 不能拿 `plan_id` 去喂 `on_event`：`on_event` 认的是**网关任务号**，
        而且它只接受"当前那一步"的事件 —— 一个走到第 3 步的计划拿第 1 步的号
        去推进，会**原地不动**（表现是"超时了却永远醒不过来"）。
        超时是**计划级**的事，所以它有自己的一条路。
        """
        rec = self._plans.get(plan_id)
        if rec is None or rec.woken:
            return False
        rec.woken = True
        rec.woken_state = ts.FAILED
        return True

    def current_step_task_id(self, plan_id):
        """计划**当前那一步**在网关那边的任务号（还没有就返回计划 id 本身）。"""
        rec = self._plans.get(plan_id)
        if rec is None:
            return plan_id
        stid = rec.step_task_ids[rec.current] if 0 <= rec.current < rec.total else None
        return stid or plan_id

    def cancel(self, task_id):
        """显式取消一个计划。返回是否找到。（调用方仍需去网关 `~/cancel` 停技能。）"""
        rec = self._plans.get(task_id)
        if rec is None or rec.woken:
            return False
        rec.woken = True
        rec.woken_state = ts.CANCELLED
        return True

    # ---------- 查询 ----------

    def get(self, task_id):
        return self._plans.get(task_id)

    def waiting(self):
        return [r for r in self._plans.values() if not r.woken]

    def __len__(self):
        return len(self._plans)

    def stats(self):
        return {'tracked': len(self._plans), 'waiting': len(self.waiting()),
                'capacity': self.capacity}
