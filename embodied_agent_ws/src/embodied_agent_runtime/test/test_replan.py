#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住重规划的策略（D-044）—— 尤其是三条**边界**：
什么时候**不该**重试、重试**不能原地转圈**、以及重试**不能绕过闸门**。
"""

import pytest

from embodied_agent_runtime import replan
from embodied_agent_runtime.planner import Step
from embodied_skill_gateway import task_state as ts

ADV = 'autonomous.advance_until_blocked'
TURN = 'autonomous.turn_until_clear'


def _rec(attempts):
    class R:
        pass
    r = R()
    r.attempts = attempts
    return r


# ---------- 该不该重试 ----------

@pytest.mark.parametrize('state', [ts.BLOCKED, ts.TARGET_LOST])
def test_the_two_states_that_deserve_a_second_try(state):
    ok, why = replan.should_replan(state, 0, 2)
    assert ok is True
    assert state in why


@pytest.mark.parametrize('state', [ts.ARRIVED, ts.TARGET_FOUND])
def test_success_is_never_replanned(state):
    """成功的终态再规划一次 = **把做成的事重做一遍**。"""
    ok, why = replan.should_replan(state, 0, 2)
    assert ok is False
    assert replan.REPLAN_NOT_WORTH_IT in why


def test_cancelled_is_never_replanned():
    """★ 取消是**用户**说停。用户按了停，车不该自己想个办法再动起来。"""
    assert replan.should_replan(ts.CANCELLED, 0, 2)[0] is False


def test_failed_is_never_replanned():
    """★ 故障重复同样的调用大概率同样失败，而且会把真正的问题
    盖在一串重试底下 —— 那正是"不许绕过真正的问题"要防的。"""
    ok, why = replan.should_replan(ts.FAILED, 0, 2)
    assert ok is False
    assert replan.REPLAN_NOT_WORTH_IT in why


def test_budget_bounds_the_retries():
    assert replan.should_replan(ts.BLOCKED, 1, 2)[0] is True
    ok, why = replan.should_replan(ts.BLOCKED, 2, 2)
    assert ok is False
    assert replan.REPLAN_BUDGET_SPENT in why


def test_zero_budget_means_no_replanning_at_all():
    """`max_replans=0` 要**彻底关掉**重规划（而不是"允许 0 次却仍走一遍逻辑"）。"""
    assert replan.should_replan(ts.BLOCKED, 0, 0)[0] is False


def test_the_refusal_reason_says_which_terminal_it_was():
    """拒绝理由要带上终态 —— 否则日志里只有"不值得重试"，
    看不出是**因为成功**还是**因为故障**，而这两件事的处理完全不同。"""
    assert ts.ARRIVED in replan.should_replan(ts.ARRIVED, 0, 2)[1]
    assert ts.FAILED in replan.should_replan(ts.FAILED, 0, 2)[1]


# ---------- ★ 防死循环 ----------

def test_a_plan_identical_to_the_last_attempt_is_rejected():
    """★ 这是本模块存在的理由。

    拿同一段文本问同一个（确定性）规划器，**多半会得到一模一样的计划**；
    一模一样的计划会以一模一样的方式再失败一次。放过去就是无限循环，
    而且不报错、不崩，只是安静地烧真车的电。
    """
    rec = _rec([[Step(ADV, {})]])
    ok, why = replan.check_new_plan([Step(ADV, {})], rec)
    assert ok is False
    assert replan.REPLAN_SAME_PLAN in why


def test_parameter_order_does_not_make_a_plan_look_new():
    """参数只是**先后写反**了，车要做的还是同一件事。"""
    rec = _rec([[Step(ADV, {'max_distance': 0.2, 'step': 0.1})]])
    ok, _ = replan.check_new_plan([Step(ADV, {'step': 0.1, 'max_distance': 0.2})], rec)
    assert ok is False


def test_a_two_cycle_oscillation_is_also_caught():
    """★ 只看"上一次"挡不住 A→B→A→B —— 那种循环每次都与上一次不同，
    但同样是死循环，只是周期为 2。所以要跟**全部**历史比。"""
    rec = _rec([[Step(ADV, {})], [Step(TURN, {'direction': 'left'})]])
    ok, why = replan.check_new_plan([Step(ADV, {})], rec)
    assert ok is False
    assert '第 1 次尝试' in why          # 指出撞的是哪一次


def test_a_genuinely_different_plan_is_accepted():
    rec = _rec([[Step(ADV, {})]])
    ok, why = replan.check_new_plan([Step(TURN, {'direction': 'left'}),
                                     Step(ADV, {})], rec)
    assert ok is True
    assert why == ''


def test_the_same_skills_with_different_arguments_count_as_different():
    """反过来也要对：`turn 左转` 与 `turn 右转` 是**两件不同的事**，不能当成重复挡掉
    —— 挡掉的话，最该试的那次（往另一边绕）反而永远试不到。"""
    rec = _rec([[Step(TURN, {'direction': 'left'})]])
    ok, _ = replan.check_new_plan([Step(TURN, {'direction': 'right'})], rec)
    assert ok is True


def test_dropping_a_step_makes_it_a_different_plan():
    rec = _rec([[Step(ADV, {}), Step(TURN, {})]])
    assert replan.check_new_plan([Step(ADV, {})], rec)[0] is True


def test_an_empty_plan_is_refused():
    """空计划永远不产生终态 —— 接受它等于把任务挂死在 WAIT 里。"""
    ok, why = replan.check_new_plan([], _rec([[Step(ADV, {})]]))
    assert ok is False
    assert replan.REPLAN_PLANNER_REFUSED in why


# ---------- ★ 闸门不许被绕过 ----------

def test_a_motion_bearing_replan_still_needs_allow_motion():
    """★ 重规划**不是**绕过闸门的路子。

    第一版计划过闸门、第二版不过，那就等于"失败一次就能让车动起来"——
    而这正是 D-033 三层闸门要防的东西。
    """
    rec = _rec([[Step(ADV, {})]])
    ok, why = replan.check_new_plan([Step(TURN, {})], rec,
                                    movers=[TURN], allow_motion=False)
    assert ok is False
    assert replan.REPLAN_NOT_CLEARED in why


def test_a_motion_bearing_replan_passes_once_the_gate_is_open():
    rec = _rec([[Step(ADV, {})]])
    ok, why = replan.check_new_plan([Step(TURN, {})], rec,
                                    movers=[TURN], allow_motion=True)
    assert ok is True
    assert why == ''


def test_a_pure_observation_replan_needs_no_gate():
    """只读技能（不含运动）任何时候都能派 —— 闸门只管运动。"""
    rec = _rec([[Step(ADV, {})]])
    assert replan.check_new_plan([Step('autonomous.look_around', {})], rec,
                                 movers=[], allow_motion=False)[0] is True


def test_repetition_is_judged_before_the_gate():
    """★ 顺序有讲究：一条**与上次相同**的计划，就算闸门没开，
    报出来的也必须是"重复"，不是"没放行" —— 否则排查的人会一路去查闸门，
    而真正的问题是**它又要走同一条走不通的路**。"""
    rec = _rec([[Step(ADV, {})]])
    ok, why = replan.check_new_plan([Step(ADV, {})], rec,
                                    movers=[ADV], allow_motion=False)
    assert ok is False
    assert replan.REPLAN_SAME_PLAN in why
    assert replan.REPLAN_NOT_CLEARED not in why
