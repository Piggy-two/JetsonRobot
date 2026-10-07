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
"""

import time

from embodied_skill_gateway import task_state as ts

# 一次事件处理的结果
WAKE = 'wake'          # 任务级终态 —— **唤醒 Agent**
RECORD = 'record'      # 收到了，但不唤醒（control-tier 终态 / 非终态）
IGNORE = 'ignore'      # 不是我在等的任务
DUPLICATE = 'duplicate'  # 这个任务已经醒过一次了 —— **不重复唤醒**


class PendingTask:
    __slots__ = ('task_id', 'skill', 'principal', 'submitted_at', 'deadline',
                 'last_state', 'woken', 'woken_state')

    def __init__(self, task_id, skill, principal, timeout_s, now):
        self.task_id = task_id
        self.skill = skill
        self.principal = principal
        self.submitted_at = now
        self.deadline = now + float(timeout_s)
        self.last_state = ''
        self.woken = False
        self.woken_state = ''

    @property
    def expired(self):
        return not self.woken and time.monotonic() >= self.deadline

    def expires_at(self, now):        # 便于测试注入时钟
        return not self.woken and now >= self.deadline


class Executor:
    """在途任务的簿记。**有界**（理由同 memory：内存只有 7.4 GiB，#7）。"""

    def __init__(self, capacity=100):
        if capacity < 1:
            raise ValueError(f'capacity 必须 ≥ 1，得到 {capacity}')
        self.capacity = int(capacity)
        self._tasks = {}          # task_id -> PendingTask（保持插入序）

    # ---------- 提交 ----------

    def submit(self, task_id, skill, principal, timeout_s, now=None):
        """记下一个在途任务。返回 `PendingTask`，或 None（表满 / 重复 task_id）。"""
        now = time.monotonic() if now is None else now
        if task_id in self._tasks:
            return None
        self._evict()
        if len(self._tasks) >= self.capacity:
            return None
        rec = PendingTask(task_id, skill, principal, timeout_s, now)
        self._tasks[task_id] = rec
        return rec

    def _evict(self):
        """满了先淘汰**已经醒过的**最老记录。还在等的任务一条都不能丢。"""
        if len(self._tasks) < self.capacity:
            return
        for task_id, rec in list(self._tasks.items()):
            if rec.woken:
                del self._tasks[task_id]
                if len(self._tasks) < self.capacity:
                    return

    # ---------- 事件 ----------

    def on_event(self, task_id, state, now=None):
        """收到一条 `SkillEvent`，推进一次。返回上面五个结果之一。"""
        rec = self._tasks.get(task_id)
        if rec is None:
            # 不是我在等的任务 —— 可能是别的调用方（路由器 / 人工工装）发起的。
            # **不要**因此报警：事件话题是共享的，看见别人的事件是正常的。
            return IGNORE

        rec.last_state = state
        if not ts.is_waking(state):
            # 非任务级终态（含 control-tier 的 FINISHED）—— **记下但不唤醒**。
            return RECORD
        if rec.woken:
            # 终态恰好一个（task_table 保证），真到这一步说明上游出了问题；
            # 无论如何**不能唤醒两次**：Agent 会以为有两个任务先后完成。
            return DUPLICATE
        rec.woken = True
        rec.woken_state = state
        return WAKE

    # ---------- 超时 ----------

    def expired(self, now=None):
        """过了 deadline 还没醒的任务。调用方要**去把技能停掉**，而不是只放弃等待。"""
        now = time.monotonic() if now is None else now
        return [r for r in self._tasks.values() if r.expires_at(now)]

    # ---------- 查询 ----------

    def get(self, task_id):
        return self._tasks.get(task_id)

    def waiting(self):
        return [r for r in self._tasks.values() if not r.woken]

    def __len__(self):
        return len(self._tasks)

    def stats(self):
        return {'tracked': len(self._tasks), 'waiting': len(self.waiting()),
                'capacity': self.capacity}
