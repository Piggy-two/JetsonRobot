#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 的有界记忆（**纯 Python，不依赖 ROS**）。

为什么第一版就强调"有界"
------------------------
这不是"以后再说"的优化：本机内存只有 **7.4 GiB，而厂商栈启动即占约 5.5 GiB**（#7）。
一个"每次任务记一笔、永不清理"的列表，跑一晚上就是一个 OOM。
所以 Memory 从第一天起就是**环形缓冲**，而且容量是可配的。

⚠️ 这里**不存**任务的状态机真值 —— 那是网关 `task_table` 的职责。
Memory 存的是 **Agent 自己的视角**：我提交过什么、我被唤醒过几次。
两份记录**故意是分开的**：Agent 对任务的理解可能有偏差，
而"真值在哪"必须只有一个地方（否则出问题时要查两处，且不知道信谁）。
"""

from collections import deque


class BoundedHistory:
    """容量固定的历史。满了就丢**最老的**（FIFO）。"""

    __slots__ = ('_items', '_dropped')

    def __init__(self, capacity=100, name='history'):
        if capacity < 1:
            raise ValueError(f'{name} 的容量必须 ≥ 1，得到 {capacity}')
        self._items = deque(maxlen=int(capacity))
        self._dropped = 0

    def append(self, item):
        """返回被挤出去的那一条（没有则 None）—— 便于上层感知"我正在丢东西"。"""
        dropped = None
        if len(self._items) == self._items.maxlen:
            dropped = self._items[0]
        self._items.append(item)
        if dropped is not None:
            self._dropped += 1
        return dropped

    def recent(self, n=None):
        items = list(self._items)
        return items if n is None else items[-n:]

    @property
    def dropped(self):
        """累计丢掉了多少条。**能被观测**很重要 —— 否则"静默丢历史"无从发现。"""
        return self._dropped

    @property
    def capacity(self):
        return self._items.maxlen

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        return iter(self._items)


class AgentMemory:
    """Agent 视角的两条历史：提交过的任务、被唤醒过的事件。

    ⚠️ **不存 "任务当前状态"** —— 那会变成 `task_table` 的第二份副本，
    两份状态迟早不一致，而且不会有人知道该信哪一份。
    """

    def __init__(self, capacity=100):
        self.submissions = BoundedHistory(capacity, 'submissions')
        self.wakeups = BoundedHistory(capacity, 'wakeups')

    def remember_submission(self, task_id, skill, principal, accepted, message='',
                            now=None):
        return self.submissions.append({
            'task_id': task_id,
            'skill': skill,
            'principal': principal,
            'accepted': bool(accepted),
            'message': message,
            'at': now,
        })

    def remember_wakeup(self, task_id, skill, state, detail='', now=None):
        return self.wakeups.append({
            'task_id': task_id,
            'skill': skill,
            'state': state,
            'detail': detail,
            'at': now,
        })

    def snapshot(self):
        return {
            'submissions': len(self.submissions),
            'wakeups': len(self.wakeups),
            'dropped': self.submissions.dropped + self.wakeups.dropped,
        }
