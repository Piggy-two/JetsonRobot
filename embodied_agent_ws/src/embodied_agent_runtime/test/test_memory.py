#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住记忆的**有界性**。这不是"以后再说"的优化 —— 本机内存只有 7.4 GiB（#7）。"""

import pytest

from embodied_agent_runtime.memory import AgentMemory, BoundedHistory


def test_history_never_exceeds_its_capacity():
    h = BoundedHistory(capacity=3)
    for i in range(100):
        h.append(i)
    assert len(h) == 3
    assert h.recent() == [97, 98, 99]


def test_dropping_is_observable():
    """**被丢弃的条数必须能被看到。** 否则"静默丢历史"无从发现 ——
    而 Agent 基于不完整的历史做判断，看起来会和正常一模一样。"""
    h = BoundedHistory(capacity=2)
    h.append(1)
    h.append(2)
    assert h.dropped == 0
    h.append(3)
    assert h.dropped == 1
    h.append(4)
    assert h.dropped == 2


def test_append_returns_what_it_evicted():
    h = BoundedHistory(capacity=1)
    h.append('a')
    assert h.append('b') == 'a'


def test_recent_n():
    h = BoundedHistory(capacity=10)
    for i in range(5):
        h.append(i)
    assert h.recent(2) == [3, 4]
    assert h.recent(99) == [0, 1, 2, 3, 4]


def test_capacity_must_be_positive():
    with pytest.raises(ValueError):
        BoundedHistory(capacity=0)


def test_iteration_order_is_oldest_to_newest():
    h = BoundedHistory(capacity=3)
    for c in 'abc':
        h.append(c)
    assert list(h) == ['a', 'b', 'c']


# ---------- AgentMemory ----------

def test_agent_memory_keeps_two_separate_histories():
    m = AgentMemory(capacity=5)
    m.remember_submission('t1', 'autonomous.advance_until_blocked', 'agent.planner',
                          True, '')

    m.remember_wakeup('t1', 'autonomous.advance_until_blocked', 'ARRIVED', '走满上限')

    assert len(m.submissions) == 1
    assert len(m.wakeups) == 1
    assert m.submissions.recent()[0]['task_id'] == 't1'
    assert m.wakeups.recent()[0]['state'] == 'ARRIVED'


def test_rejected_submissions_are_remembered_too():
    """被拒绝的任务也要留痕 —— 否则"为什么刚才那句话没反应"就查不到。"""
    m = AgentMemory(capacity=5)
    m.remember_submission('', '', 'router.agent_task', False, '需要 LLM 规划')
    rec = m.submissions.recent()[0]
    assert rec['accepted'] is False
    assert 'LLM' in rec['message']


def test_snapshot_reports_both_counts_and_drops():
    m = AgentMemory(capacity=1)
    m.remember_submission('t1', 's', 'p', True)
    m.remember_submission('t2', 's', 'p', True)
    snap = m.snapshot()
    assert snap['submissions'] == 1
    assert snap['dropped'] == 1
