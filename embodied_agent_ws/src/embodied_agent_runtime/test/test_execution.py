#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 WAIT 的推进语义 —— 尤其是「**只有任务级事件唤醒 Agent**」（D-004）与
**多步计划的推进规则**（D-043）。
"""

import pytest

from embodied_agent_runtime import execution as ex
from embodied_agent_runtime.planner import Step
from embodied_skill_gateway import task_state as ts

ADV = 'autonomous.advance_until_blocked'
TURN = 'autonomous.turn_until_clear'


def _exec(**kw):
    return ex.Executor(**kw)


def _submit(e, task_id='t1', skill=ADV, timeout_s=10.0, now=100.0, steps=None):
    """提交一个计划并把第 1 步绑到 `task_id` 上（单步时计划 id 与它相同）。"""
    steps = steps if steps is not None else [Step(skill, {})]
    rec = e.submit_plan(task_id, steps, 'agent.planner', timeout_s, now=now)
    if rec is not None:
        e.bind_step(task_id, 0, task_id)
    return rec


# ---------- 提交 ----------

def test_submit_creates_a_pending_task_with_a_monotonic_deadline():
    e = _exec()
    rec = _submit(e, timeout_s=10.0, now=100.0)
    assert rec is not None
    assert rec.deadline == pytest.approx(110.0)
    assert rec.woken is False
    assert rec.total == 1
    assert e.get('t1') is rec


def test_duplicate_task_id_is_refused():
    e = _exec()
    _submit(e)
    assert _submit(e) is None


def test_empty_plan_is_refused():
    """一个没有步骤的计划没有意义 —— 它永远不会产生终态，只会挂在那儿。"""
    e = _exec()
    assert e.submit_plan('t1', [], 'agent.planner', 10.0, now=100.0) is None


# ==========================================================================
# ★ 「只有任务级事件唤醒 Agent」
# ==========================================================================

def test_task_tier_terminal_wakes_the_agent():
    e = _exec()
    _submit(e)
    out = e.on_event('t1', ts.ARRIVED)
    assert out.action == ex.WAKE
    assert e.get('t1').woken is True


@pytest.mark.parametrize('state', [ts.BLOCKED, ts.FAILED, ts.CANCELLED,
                                   ts.TARGET_FOUND, ts.TARGET_LOST])
def test_every_task_terminal_ends_a_one_step_plan(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state).action == ex.WAKE


def test_control_tier_finished_does_NOT_wake_the_agent():
    """★ 这是本文件最要紧的一条。

    `FINISHED` 只表示「技能返回了」，**不表示任务结束了**（D-032）。
    若它也算唤醒，Agent 会在一次开环动作返回时就以为任务完成，
    然后基于这个假前提继续规划 —— 而车可能只是走了一小段。
    """
    e = _exec()
    _submit(e, skill='control.move_relative')
    assert e.on_event('t1', ts.FINISHED).action == ex.RECORD
    assert e.get('t1').woken is False


@pytest.mark.parametrize('state', [ts.STARTED, ts.RUNNING])
def test_intermediate_states_are_recorded_but_do_not_wake(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state).action == ex.RECORD
    assert e.get('t1').woken is False
    assert e.get('t1').last_step_state == state


def test_events_for_other_tasks_are_ignored_not_alarmed():
    """事件话题是**共享的** —— 看见别人的任务事件是正常的，不该报警。

    （路由器、人工验收工装都会经过同一个网关，它们的事件也在这条话题上。）
    """
    e = _exec()
    _submit(e, task_id='mine')
    assert e.on_event('someone-elses-task', ts.ARRIVED).action == ex.IGNORE


def test_duplicate_terminal_does_not_wake_twice():
    """终态恰好一个（`task_table` 保证）。真到这一步说明上游出了问题 ——
    但无论如何**不能唤醒两次**：Agent 会以为有两个任务先后完成。"""
    e = _exec()
    _submit(e)
    assert e.on_event('t1', ts.ARRIVED).action == ex.WAKE
    assert e.on_event('t1', ts.ARRIVED).action == ex.DUPLICATE
    assert e.on_event('t1', ts.BLOCKED).action == ex.DUPLICATE


# ==========================================================================
# ★ ★ 多步计划（D-043）
# ==========================================================================

def test_multi_step_continues_after_a_non_aborting_terminal():
    """第 1 步到位 ⇒ 去派发第 2 步，**且此时不唤醒 Agent**。

    中途唤醒会让 Agent 对着一个"只走了一半"的世界重新规划，
    而那正是它刚规划过的东西（D-043）。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    out = e.on_event('p', ts.ARRIVED)          # 第 1 步的网关任务号就是 'p'
    assert out.action == ex.DISPATCH_NEXT
    assert out.step_index == 2
    assert e.get('p').woken is False


@pytest.mark.parametrize('state', [ts.ARRIVED, ts.BLOCKED, ts.TARGET_FOUND, ts.TARGET_LOST])
def test_multi_step_continues_on_every_non_aborting_terminal(state):
    """★ **`BLOCKED` 必须继续** —— 脱困计划就是"前进被挡 → 转身 → 再前进"。

    把 `BLOCKED` 当中止，脱困就永远走不到第 2 步。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', state).action == ex.DISPATCH_NEXT


@pytest.mark.parametrize('state', [ts.FAILED, ts.CANCELLED])
def test_multi_step_aborts_immediately_on_failed_or_cancelled(state):
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    out = e.on_event('p', state)
    assert out.action == ex.WAKE
    assert out.terminal == state
    assert e.get('p').woken is True


def test_last_step_terminal_ends_the_plan():
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', ts.ARRIVED).action == ex.DISPATCH_NEXT
    # 第 2 步由调用方派发后绑回来
    assert e.bind_step('p', 1, 'p-s2') is True
    out = e.on_event('p-s2', ts.BLOCKED)
    assert out.action == ex.WAKE
    assert out.terminal == ts.BLOCKED
    assert '最后一步' in out.reason


def test_a_late_event_from_an_earlier_step_does_not_advance_the_plan():
    """迟到的旧步骤事件**不能**推进计划 —— 那会让"第 2 步的终态"去驱动一个
    已经走到第 3 步的计划。（超时后又被停下来的那次，就会迟到。）"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {}), Step(ADV, {})])
    e.on_event('p', ts.ARRIVED)                # → 派发第 2 步
    e.bind_step('p', 1, 'p-s2')
    e.on_event('p-s2', ts.ARRIVED)             # → 派发第 3 步
    e.bind_step('p', 2, 'p-s3')
    late = e.on_event('p-s2', ts.FAILED)       # 第 2 步的迟到终态
    assert late.action == ex.RECORD
    assert e.get('p').woken is False
    assert e.get('p').current == 2             # 还停在第 3 步上


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


def test_timeout_marks_the_plan_terminal_without_going_through_on_event():
    """⚠️ 超时不能用 `on_event(plan_id, ...)` 收尾：`on_event` 认的是**当前那一步**
    的网关任务号，一个走到第 3 步的计划拿第 1 步的号去推进，会**原地不动**
    —— 表现就是"超时了却永远醒不过来"。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    e.on_event('p', ts.ARRIVED)                # 计划走到第 2 步
    e.bind_step('p', 1, 'p-s2')
    assert e.get('p').woken is False
    assert e.timeout('p') is True
    assert e.get('p').woken is True
    assert e.get('p').woken_state == ts.FAILED
    assert e.timeout('p') is False             # 幂等


def test_current_step_task_id_follows_the_plan():
    """超时要取消的是**当前那一步**，不是计划 id（取消第 1 步等于什么也没停）。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.current_step_task_id('p') == 'p'
    e.on_event('p', ts.ARRIVED)
    e.bind_step('p', 1, 'p-s2')
    assert e.current_step_task_id('p') == 'p-s2'


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


def test_outcome_carries_the_plan_id_not_the_gateway_task_id():
    """★ 回归：**端到端第一次就栽在这里**。

    事件里给的是**网关任务号**，而调用方要按**计划 id** 去取记录
    （派发下一步、读计划里剩下的步骤）。第 2 步之后这两个号**就不同了** ——
    outcome 不带 `plan_id` 的话，调用方会拿第 2 步的任务号去查计划，
    查不到，于是**静默不派发下一步**：日志里"计划继续…下一步 第 3 步"打出来了，
    而第 3 步永远不会发生。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', ts.ARRIVED).plan_id == 'p'
    e.bind_step('p', 1, 'p-s2')
    out = e.on_event('p-s2', ts.ARRIVED)
    assert out.action == ex.WAKE
    assert out.plan_id == 'p'          # ← **不是** 'p-s2'
