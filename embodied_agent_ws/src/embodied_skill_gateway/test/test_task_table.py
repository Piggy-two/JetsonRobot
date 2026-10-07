#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住任务表的三条不变式：**恰好一个终态 / 有界 / 用单调钟判超时**。"""

import pytest

from embodied_skill_gateway import task_state as ts
from embodied_skill_gateway.registry import ParamSpec, SkillSpec
from embodied_skill_gateway.task_table import TaskTable, TaskTableFull


def _spec(name='control.move_relative', tier='control', timeout_s=15.0):
    return SkillSpec(name, tier=tier, target='/t', srv_type='s',
                     params=[ParamSpec('x')], timeout_s=timeout_s,
                     allowed_principals=['router.deterministic'])


def _table(**kw):
    return TaskTable(**kw)


# ---------- 生命周期 ----------

def test_admit_creates_a_running_task_with_a_deadline():
    t = _table()
    rec = t.admit('t1', _spec(timeout_s=15.0), 'router.deterministic', 'r1', now=100.0)
    assert rec.state == ts.RUNNING
    assert rec.deadline == pytest.approx(115.0)
    assert rec.terminal is False
    assert t.get('t1') is rec


def test_finish_sets_exactly_one_terminal():
    t = _table()
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    rec = t.finish('t1', ts.FINISHED, message='已完成（开环：按时间发出）', now=103.0)
    assert rec.state == ts.FINISHED
    assert rec.terminal is True
    assert rec.terminal_at == pytest.approx(103.0)


def test_setting_a_terminal_twice_raises():
    """缺失终态会让 Agent 永远 WAIT；重复终态会让它被唤醒两次。都不允许。"""
    t = _table()
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    t.finish('t1', ts.FINISHED, now=101.0)
    with pytest.raises(ts.InvalidTransition):
        t.finish('t1', ts.FAILED, now=102.0)


def test_task_tier_cannot_end_in_the_control_only_terminal():
    """task-tier 的任务**不得**以 control-only 的 `FINISHED` 收尾。

    这不是吹毛求疵：`FINISHED` **不唤醒 Agent**（见 task_state 的注释）。
    一个 task-tier 任务若以它收尾，Agent 既不会被唤醒，也永远等不到真正的事件 ——
    它会一直 WAIT 下去。
    """
    t = _table()
    t.admit('t1', _spec(tier='task'), 'agent.planner', 'r1', now=100.0)
    with pytest.raises(ts.InvalidTransition):
        t.finish('t1', ts.FINISHED, now=101.0)


def test_task_tier_may_finish_before_running_is_observed():
    """`STARTED → 终态` 是**刻意允许**的：任务可能在网关观察到 RUNNING 之前就完成了。

    强行要求先经过 RUNNING，等于要求网关去"看见"一个它可能压根没机会看见的中间态。
    """
    t = _table()
    t.admit('t1', _spec(tier='task'), 'agent.planner', 'r1', now=100.0)
    rec = t.finish('t1', ts.BLOCKED, message='前方受阻', now=100.2)
    assert rec.state == ts.BLOCKED


def test_task_tier_started_then_finished():
    t = _table()
    t.admit('t1', _spec(tier='task'), 'agent.planner', 'r1', now=100.0)
    t.start('t1', now=100.5)
    rec = t.finish('t1', ts.ARRIVED, now=110.0)
    assert rec.state == ts.ARRIVED


def test_unknown_task_id_operations_are_noops_returning_none():
    t = _table()
    assert t.get('nope') is None
    assert t.finish('nope', ts.FINISHED) is None
    assert t.start('nope') is None


def test_elapsed_uses_monotonic_not_wall_clock():
    t = _table()
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    t.finish('t1', ts.FINISHED, now=103.5)
    assert t.get('t1').elapsed == pytest.approx(3.5)


# ---------- 超时 ----------

def test_expired_only_returns_non_terminal_tasks_past_deadline():
    t = _table()
    t.admit('t1', _spec(timeout_s=5.0), 'router.deterministic', 'r1', now=100.0)
    t.admit('t2', _spec(timeout_s=50.0), 'router.deterministic', 'r2', now=100.0)
    t.finish('t2', ts.FINISHED, now=101.0)

    assert t.expired(now=104.0) == []
    overdue = t.expired(now=106.0)
    assert [r.task_id for r in overdue] == ['t1']


def test_terminal_task_never_expires():
    t = _table()
    t.admit('t1', _spec(timeout_s=1.0), 'router.deterministic', 'r1', now=100.0)
    t.finish('t1', ts.FINISHED, now=100.5)
    assert t.expired(now=1e9) == []


def test_request_cancel_is_only_a_request_not_an_actual_stop():
    """置位取消**不做实际取消** —— 真正停技能是节点的职责（要调技能的 stop）。

    只把等待标记为放弃、却不去停技能，会出现"车还在动，上层以为结束了"。
    所以这两件事必须在代码里也是分开的。
    """
    t = _table()
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    rec, changed = t.request_cancel('t1', '用户取消')
    assert changed is True
    assert rec.cancel_requested is True
    assert rec.state == ts.RUNNING          # ← 状态没变，还活着
    assert rec.terminal is False
    rec2, changed2 = t.request_cancel('t1', '再来一次')
    assert changed2 is False                # 幂等


def test_cancel_request_on_a_terminal_task_is_ignored():
    t = _table()
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    t.finish('t1', ts.FINISHED, now=101.0)
    rec, changed = t.request_cancel('t1', '太晚了')
    assert changed is False
    assert rec.cancel_requested is False


# ---------- 有界 ----------

def test_table_evicts_oldest_terminal_when_full():
    t = _table(max_tasks=3)
    for i in range(5):
        t.admit(f't{i}', _spec(), 'router.deterministic', f'r{i}', now=100.0 + i)
        t.finish(f't{i}', ts.FINISHED, now=100.0 + i + 0.1)
    assert len(t) <= 3


def test_table_never_evicts_an_active_task():
    """表满时的淘汰**只能动终态记录**。丢掉一个还在跑的任务，
    等于让它永远没有终态 —— Agent 会一直 WAIT。"""
    t = _table(max_tasks=2)
    t.admit('active1', _spec(), 'router.deterministic', 'r1', now=100.0)
    t.admit('active2', _spec(), 'router.deterministic', 'r2', now=100.0)
    # 两条都活跃，没有终态可淘汰 -> 拒绝受理，而不是丢掉 active1
    with pytest.raises(TaskTableFull):
        t.admit('active3', _spec(), 'router.deterministic', 'r3', now=100.0)
    assert t.get('active1') is not None
    assert t.get('active2') is not None


def test_active_skills_reports_skills_with_in_flight_tasks():
    """用于"每技能一个在途任务"的约束（沿用 D-026 决策 2 的不排队原则）。"""
    t = _table()
    t.admit('t1', _spec('control.move_relative'), 'router.deterministic', 'r1', now=100.0)
    t.admit('t2', _spec('control.rotate'), 'router.deterministic', 'r2', now=100.0)
    assert t.active_skills() == {'control.move_relative', 'control.rotate'}
    t.finish('t2', ts.FINISHED, now=101.0)
    assert t.active_skills() == {'control.move_relative'}


def test_stats_reports_bounded_expectations():
    t = _table(max_tasks=10)
    t.admit('t1', _spec(), 'router.deterministic', 'r1', now=100.0)
    s = t.stats()
    assert s['total'] == 1 and s['active'] == 1 and s['max'] == 10
