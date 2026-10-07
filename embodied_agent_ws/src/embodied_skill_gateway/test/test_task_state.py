#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住状态机的几条**不变式** —— 它们直接决定 Agent 会不会被唤醒、被唤醒几次。"""

import pytest

from embodied_skill_gateway import task_state as ts


def test_task_states_match_plan_md_section_21():
    """8 个任务状态必须与 plan.md §21 逐字一致 —— 这是文档定死的枚举。"""
    assert ts.TASK_STATES == (
        'STARTED', 'RUNNING', 'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST',
        'BLOCKED', 'FAILED', 'CANCELLED')


def test_waking_set_is_exactly_the_task_terminal_set():
    """唤醒集必须**恒等于**任务终态集（D-004）。

    这两者如果各自演化（比如有人给 RUNNING 也加了唤醒），Agent 会被中间态淹没；
    反过来漏掉一个终态，Agent 就永远 WAIT。所以钉死它们是同一个集合。
    """
    assert set(ts.TASK_WAKING) == set(ts.TASK_TERMINAL)
    assert ts.TASK_TERMINAL == ('ARRIVED', 'TARGET_FOUND', 'TARGET_LOST',
                                'BLOCKED', 'FAILED', 'CANCELLED')


@pytest.mark.parametrize('state', ['STARTED', 'RUNNING'])
def test_intermediate_states_do_not_wake_the_agent(state):
    assert ts.is_waking(state) is False


@pytest.mark.parametrize('state', ts.TASK_TERMINAL)
def test_every_task_terminal_wakes_the_agent(state):
    assert ts.is_waking(state) is True


def test_finished_is_not_a_task_state():
    """★ 这条是 D-032 的核心：`FINISHED` 属于 control-tier，**不是**那 8 个任务状态之一。

    它只表示"技能返回了"，绝不表示"到位了"。若它混进任务状态集，
    上层就会把一次开环动作的返回读成"到达目标"。
    """
    assert ts.FINISHED not in ts.TASK_STATES
    assert ts.FINISHED in ts.CONTROL_STATES
    assert ts.FINISHED not in ts.TASK_TERMINAL


def test_only_finished_is_the_control_only_terminal_that_does_not_wake():
    """control-tier 独有的终态 `FINISHED` **不**唤醒 Agent。

    ⚠️ 别误读成"所有 control 终态都不唤醒" —— `FAILED` / `CANCELLED` 是两个词汇表
    **共用**的（它们本来就是 task 终态），所以它们**仍然会唤醒**。两个词汇表
    只在 `FINISHED` 这一处**分叉**，这条测试就把分叉钉在这里。

    为什么 FINISHED 不唤醒：Agent 本来就不该调 control 技能（D-003 的 Permission 拦着），
    所以它的"正常结束"对 Agent 没有意义，只对路由器与日志有意义。
    """
    assert ts.is_waking(ts.FINISHED) is False
    assert ts.FINISHED in ts.CONTROL_TERMINAL
    assert ts.FINISHED not in ts.TASK_TERMINAL
    # 共用的两个仍然唤醒
    assert ts.is_waking(ts.FAILED) is True
    assert ts.is_waking(ts.CANCELLED) is True


@pytest.mark.parametrize('state', ts.ALL_TERMINAL)
def test_terminal_states_have_no_outgoing_edges(state):
    """终态没有出边 —— 这是"每个任务恰好一个终态"的实现手段。"""
    assert ts.can_transition(state, ts.RUNNING) is False
    for other in ts.ALL_STATES:
        assert ts.can_transition(state, other) is False


@pytest.mark.parametrize('state', ts.ALL_TERMINAL)
def test_setting_a_terminal_twice_raises(state):
    """重复置终态必须**抛错**，不能静默忽略。

    静默忽略的后果是 Agent 被唤醒两次 —— 它会以为有两个任务先后完成了。
    """
    with pytest.raises(ts.InvalidTransition):
        ts.check_transition(state, state)


def test_happy_path_task_tier_transitions():
    assert ts.check_transition(ts.STARTED, ts.RUNNING)
    assert ts.check_transition(ts.RUNNING, ts.ARRIVED)


def test_control_tier_starts_running_and_can_finish():
    """control-tier 没有 STARTED 阶段（受理即开始跑），但 STARTED 本身仍属于 task-tier。"""
    assert ts.check_transition(ts.RUNNING, ts.FINISHED)
    assert ts.STARTED in ts.TASK_STATES
    assert ts.STARTED not in ts.CONTROL_STATES


def test_illegal_transition_raises_with_readable_reason():
    with pytest.raises(ts.InvalidTransition) as e:
        ts.check_transition(ts.ARRIVED, ts.RUNNING)
    assert '终态' in str(e.value)


def test_unknown_state_raises():
    with pytest.raises(ts.InvalidTransition):
        ts.check_transition('NOT_A_STATE', ts.RUNNING)
    with pytest.raises(ts.InvalidTransition):
        ts.check_transition(ts.RUNNING, 'NOT_A_STATE')


def test_is_known_covers_both_vocabularies():
    assert ts.is_known(ts.STARTED)
    assert ts.is_known(ts.FINISHED)
    assert not ts.is_known('MEOW')


def test_to_dict_is_self_describing():
    d = ts.to_dict()
    assert set(d['task_states']) == set(ts.TASK_STATES)
    assert d['transitions'][ts.FINISHED] == []
