#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务表（**纯 Python，不依赖 ROS**）—— 任务的生命周期与有界存储。

它是"受理之后这个任务怎么样了"的唯一事实来源。三条不变式：

  1. **每个任务恰好一个终态。** 由 `task_state.check_transition` 保证
     （终态没有出边，重复置终态会抛错）。缺失会让 Agent 永远 WAIT；
     重复会让它被唤醒两次。
  2. **有界。** 内存只有 7.4 GiB，厂商栈已占约 5.5 GiB（#7）。
     历史任务清不掉、越跑越多是不可接受的。满了先淘汰**最老的终态**记录；
     一个终态都没有时**拒绝受理新任务**，而不是无界增长。
  3. **时间用单调钟。** ⚠️ **绝不用 `time.time()`** —— 本机墙上钟会被 NTP 步进
     （#24 / DEV_NOTES 坑 13），实测已经把一个 10.000 Hz 的读数算成 16.85 Hz。
     超时判定用墙上钟会出现"任务还没超时就过期"或反之。
"""

import time

from embodied_skill_gateway import task_state


class TaskRecord:
    """一个任务的全部状态。"""

    __slots__ = ('task_id', 'skill', 'principal', 'request_id', 'tier', 'state',
                 'created', 'started', 'deadline', 'terminal_at', 'result_json',
                 'message', 'verified', 'cancel_requested', 'cancel_reason')

    def __init__(self, task_id, skill, principal, request_id, tier, state, now,
                 timeout_s):
        self.task_id = task_id
        self.skill = skill
        self.principal = principal
        self.request_id = request_id
        self.tier = tier
        self.state = state
        self.created = now
        self.started = now
        self.deadline = now + float(timeout_s)
        self.terminal_at = None
        self.result_json = ''
        self.message = ''
        self.verified = False
        self.cancel_requested = False
        self.cancel_reason = ''

    @property
    def terminal(self):
        return task_state.is_terminal(self.state)

    @property
    def elapsed(self):
        end = self.terminal_at if self.terminal_at is not None else time.monotonic()
        return end - self.created

    def expires_at(self, now):
        return (not self.terminal) and now >= self.deadline

    def __repr__(self):
        return f'TaskRecord({self.task_id}, {self.skill}, {self.state})'


class TaskTableFull(Exception):
    """表满且无终态记录可淘汰。**拒绝受理**，不是静默丢弃最老的活跃任务。"""


class TaskTable:
    def __init__(self, max_tasks=200):
        if max_tasks < 1:
            raise ValueError(f'max_tasks 必须为正，得到 {max_tasks}')
        self.max_tasks = int(max_tasks)
        self._tasks = {}       # task_id -> TaskRecord（保持插入顺序，便于淘汰最老的）

    # ---------- 生命周期 ----------

    def admit(self, task_id, spec, principal, request_id, now=None):
        """受理一个新任务。表满则抛 `TaskTableFull`。"""
        now = time.monotonic() if now is None else now
        self._make_room()
        if len(self._tasks) >= self.max_tasks:
            raise TaskTableFull(
                f'任务表已满（{len(self._tasks)}/{self.max_tasks}）且**没有终态记录可淘汰** '
                f'—— 拒绝受理，以免丢掉还在跑的任务')
        # 起始状态按层级定：
        #   · task-tier 走文档的状态机 STARTED → RUNNING → 终态（plan.md §21）
        #     —— STARTED 表示"已受理、尚未开始执行"，对长任务是有意义的区分。
        #   · control-tier 没有这个阶段：受理即开始跑（Control Skill 本来就是
        #     "发速度、等它跑完"），所以直接 RUNNING。
        initial = (task_state.STARTED if spec.tier == 'task'
                   else task_state.RUNNING)
        rec = TaskRecord(task_id, spec.name, principal, request_id, spec.tier,
                         initial, now, spec.timeout_s)
        self._tasks[task_id] = rec
        return rec

    def start(self, task_id, now=None):
        """把任务推进到 RUNNING（task-tier 是 STARTED → RUNNING；control 已在 RUNNING）。"""
        rec = self.get(task_id)
        if rec is None:
            return None
        if rec.state == task_state.STARTED:
            task_state.check_transition(rec.state, task_state.RUNNING)
            rec.state = task_state.RUNNING
            rec.started = time.monotonic() if now is None else now
        return rec

    def finish(self, task_id, state, message='', result_json='', verified=False,
               now=None):
        """置终态。**非法迁移会抛 `InvalidTransition`**（不许静默忽略）。"""
        rec = self.get(task_id)
        if rec is None:
            return None
        # 非法迁移一律抛错 —— 不变式 1（恰好一个终态）就靠这一行保证：
        # 终态没有出边，所以"再置一次终态"会被 `check_transition` 拒绝。
        # ⚠️ 不能静默忽略：重复终态会让 Agent 被唤醒两次，缺失会让它永远 WAIT。
        task_state.check_transition(rec.state, state)
        rec.state = state
        rec.message = str(message)
        rec.result_json = result_json if isinstance(result_json, str) else str(result_json)
        rec.verified = bool(verified)
        rec.terminal_at = time.monotonic() if now is None else now
        return rec

    def request_cancel(self, task_id, reason=''):
        """置位取消请求。**不做实际取消** —— 那是节点的职责（要调技能的 stop）。

        返回 (记录, 是否发生了状态变化)。已经是终态的任务不置位。
        """
        rec = self.get(task_id)
        if rec is None or rec.terminal:
            return rec, False
        changed = not rec.cancel_requested
        rec.cancel_requested = True
        rec.cancel_reason = str(reason)
        return rec, changed

    # ---------- 查询 ----------

    def get(self, task_id):
        return self._tasks.get(task_id)

    def expired(self, now=None):
        """已过 deadline 且尚未终态的任务。调用方**必须先 cancel 再置终态** ——
        只把等待标记为放弃、却不去停技能，会出现"车还在动，上层以为结束了"。"""
        now = time.monotonic() if now is None else now
        return [r for r in self._tasks.values() if r.expires_at(now)]

    def active(self):
        return [r for r in self._tasks.values() if not r.terminal]

    def active_skills(self):
        """当前有在途任务的技能名集合。用于"每技能一个在途任务"的约束。"""
        return {r.skill for r in self.active()}

    def __len__(self):
        return len(self._tasks)

    def stats(self):
        return {'total': len(self._tasks), 'active': len(self.active()),
                'max': self.max_tasks}

    # ---------- 有界 ----------

    def _make_room(self):
        """表满时淘汰**最老的终态**记录（FIFO）。活跃任务一个都不动。"""
        if len(self._tasks) < self.max_tasks:
            return
        for task_id, rec in list(self._tasks.items()):
            if rec.terminal:
                del self._tasks[task_id]
                if len(self._tasks) < self.max_tasks:
                    return
