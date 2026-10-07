#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 WAIT 的推进语义 —— 尤其是「**只有任务级事件唤醒 Agent**」（D-004）。"""

import pytest

from embodied_agent_runtime import execution as ex
from embodied_skill_gateway import task_state as ts


def _exec(**kw):
    return ex.Executor(**kw)


def _submit(e, task_id='t1', skill='autonomous.advance_until_blocked',
            timeout_s=10.0, now=100.0):
    return e.submit(task_id, skill, 'agent.planner', timeout_s, now=now)


# ---------- 提交 ----------

def test_submit_creates_a_pending_task_with_a_monotonic_deadline():
    e = _exec()
    rec = _submit(e, timeout_s=10.0, now=100.0)
    assert rec is not None
    assert rec.deadline == pytest.approx(110.0)
    assert rec.woken is False
    assert e.get('t1') is rec


def test_duplicate_task_id_is_refused():
    e = _exec()
    _submit(e)
    assert _submit(e) is None


# ==========================================================================
# ★ 「只有任务级事件唤醒 Agent」
# ==========================================================================

def test_task_tier_terminal_wakes_the_agent():
    e = _exec()
    _submit(e)
    assert e.on_event('t1', ts.ARRIVED) == ex.WAKE
    assert e.get('t1').woken is True


@pytest.mark.parametrize('state', [ts.BLOCKED, ts.FAILED, ts.CANCELLED,
                                   ts.TARGET_FOUND, ts.TARGET_LOST])
def test_every_task_terminal_wakes(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state) == ex.WAKE


def test_control_tier_finished_does_NOT_wake_the_agent():
    """★ 这是本文件最要紧的一条。

    `FINISHED` 只表示「技能返回了」，**不表示任务结束了**（D-032）。
    若它也算唤醒，Agent 会在一次开环动作返回时就以为任务完成，
    然后基于这个假前提继续规划 —— 而车可能只是走了一小段。
    """
    e = _exec()
    _submit(e, skill='control.move_relative')
    assert e.on_event('t1', ts.FINISHED) == ex.RECORD
    assert e.get('t1').woken is False


@pytest.mark.parametrize('state', [ts.STARTED, ts.RUNNING])
def test_intermediate_states_are_recorded_but_do_not_wake(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state) == ex.RECORD
    assert e.get('t1').woken is False
    assert e.get('t1').last_state == state


def test_events_for_other_tasks_are_ignored_not_alarmed():
    """事件话题是**共享的** —— 看见别人的任务事件是正常的，不该报警。

    （路由器、人工验收工装都会经过同一个网关，它们的事件也在这条话题上。）
    """
    e = _exec()
    _submit(e, task_id='mine')
    assert e.on_event('someone-elses-task', ts.ARRIVED) == ex.IGNORE


def test_duplicate_terminal_does_not_wake_twice():
    """终态恰好一个（`task_table` 保证）。真到这一步说明上游出了问题 ——
    但无论如何**不能唤醒两次**：Agent 会以为有两个任务先后完成了。"""
    e = _exec()
    _submit(e)
    assert e.on_event('t1', ts.ARRIVED) == ex.WAKE
    assert e.on_event('t1', ts.ARRIVED) == ex.DUPLICATE
    assert e.on_event('t1', ts.BLOCKED) == ex.DUPLICATE


# ---------- 超时 ----------

def test_expired_only_returns_tasks_past_deadline_and_not_yet_woken():
    e = _exec()
    _submit(e, task_id='t1', timeout_s=5.0, now=100.0)
    _submit(e, task_id='t2', timeout_s=50.0, now=100.0)
    assert e.expired(now=104.0) == []
    assert [r.task_id for r in e.expired(now=106.0)] == ['t1']


def test_already_woken_task_never_expires():
    e = _exec()
    _submit(e, timeout_s=1.0, now=100.0)
    e.on_event('t1', ts.ARRIVED)
    assert e.expired(now=1e9) == []


# ---------- 有界 ----------

def test_executor_evicts_woken_tasks_when_full():
    e = _exec(capacity=3)
    for i in range(6):
        _submit(e, task_id=f't{i}', now=100.0 + i)
        e.on_event(f't{i}', ts.ARRIVED)
    assert len(e) <= 3


def test_executor_never_drops_a_still_waiting_task():
    """丢掉一个还在等的任务 = Agent 永远等不到它 —— 比"表满拒绝新任务"糟得多。"""
    e = _exec(capacity=2)
    _submit(e, task_id='w1')
    _submit(e, task_id='w2')
    assert _submit(e, task_id='w3') is None      # 拒绝，而不是丢 w1
    assert e.get('w1') is not None
    assert e.get('w2') is not None


def test_waiting_and_stats():
    e = _exec(capacity=5)
    _submit(e, task_id='t1')
    _submit(e, task_id='t2')
    e.on_event('t1', ts.ARRIVED)
    assert [r.task_id for r in e.waiting()] == ['t2']
    s = e.stats()
    assert s['tracked'] == 2 and s['waiting'] == 1 and s['capacity'] == 5


def test_capacity_must_be_positive():
    with pytest.raises(ValueError):
        ex.Executor(capacity=0)
