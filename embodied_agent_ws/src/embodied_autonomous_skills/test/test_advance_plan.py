#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 `advance_until_blocked` 的判定逻辑。

★ 最要紧的一组在文件末尾：「**不知道 ≠ 受阻**」（D-028）。
  把"雷达没数据"报成"前方有障碍"，会让排查方向**完全跑偏** ——
  人去挪障碍物，而其实该去查雷达链路。
"""

import pytest

from embodied_autonomous_skills.advance_plan import (
    AdvancePlanError, decide, summarize, validate)
from embodied_skill_gateway import task_state as ts


# ---------- 参数校验 ----------

@pytest.mark.parametrize('args', [
    (0.0, 0.5, 0.1),        # max_distance = 0
    (-1.0, 0.5, 0.1),       # max_distance < 0
    (1.0, 0.0, 0.1),        # clear_range = 0
    (1.0, 0.5, 0.0),        # step = 0
    (float('nan'), 0.5, 0.1),
    (float('inf'), 0.5, 0.1),
])
def test_invalid_parameters_are_rejected(args):
    with pytest.raises(AdvancePlanError):
        validate(*args)


def test_step_larger_than_max_distance_is_rejected():
    """`step > max_distance` 是**静默失真**：步长实际不会生效（只会走 max_distance），
    而调用方以为自己设的步长起作用了。宁可在入口拒绝。"""
    with pytest.raises(AdvancePlanError, match='静默失真'):
        validate(0.5, 0.3, 1.0)


def test_valid_parameters_pass():
    validate(1.0, 0.3, 0.1)          # 不抛
    validate(0.1, 0.3, 0.1)          # step == max_distance 允许


# ---------- 正常前进 ----------

def test_clear_path_advances_by_one_step():
    d = decide(clear=True, range_m=2.0, clear_range=0.5, remaining=1.0, step=0.25)
    assert d.action == 'advance'
    assert d.distance == pytest.approx(0.25)
    assert d.terminal is None


def test_last_step_is_clamped_to_remaining():
    """最后一步不能超过剩余量 —— 否则会多走一截（而走多了是要撞的）。"""
    d = decide(clear=True, range_m=2.0, clear_range=0.5, remaining=0.1, step=0.25)
    assert d.action == 'advance'
    assert d.distance == pytest.approx(0.1)


def test_exhausted_budget_reports_arrived():
    d = decide(clear=True, range_m=2.0, clear_range=0.5, remaining=0.0, step=0.25)
    assert d.action == 'stop'
    assert d.terminal == ts.ARRIVED


# ---------- 受阻 ----------

def test_blocked_reports_blocked_with_the_measured_distance():
    d = decide(clear=False, range_m=0.2, clear_range=0.5, remaining=1.0, step=0.25)
    assert d.action == 'stop'
    assert d.terminal == ts.BLOCKED
    assert '0.200' in d.note            # 把实测距离写进原因，便于判断是不是误报


# ==========================================================================
# ★ 「不知道」≠「受阻」
# ==========================================================================

def test_no_scan_data_is_reported_as_failed_not_blocked():
    """★ 扫描陈旧时 `path_clear` 返回 `clear=false, range=-1`。

    那是**雷达没数据**（"不知道"），不是**前方有障碍**。
    报成 BLOCKED 会让人去挪障碍物，而真正该修的是雷达链路。
    """
    d = decide(clear=False, range_m=-1, clear_range=0.5, remaining=1.0, step=0.25)
    assert d.action == 'stop'
    assert d.terminal == ts.FAILED, '「不知道」必须报 FAILED，不能报 BLOCKED'
    assert d.terminal != ts.BLOCKED
    assert '不可用' in d.note
    # 而且要在措辞里**明确否掉**"前方有障碍"这个误读
    assert '不代表' in d.note
    assert '障碍' in d.note


def test_missing_range_value_is_also_treated_as_unknown():
    d = decide(clear=False, range_m=None, clear_range=0.5, remaining=1.0, step=0.25)
    assert d.terminal == ts.FAILED


def test_no_echo_at_all_is_clear_not_unknown():
    """★ 同一个 `range == -1`，**含义完全相反**的另一种情形。

    `clear=true, range=-1` 表示扇区内**一个回波都没有** —— 那是**空旷**，可以走。
    只有 `clear=false, range=-1` 才是"没有数据"。

    分不清这两种，就会在空旷场地上寸步难行（或在没数据时贸然前进）。
    """
    d = decide(clear=True, range_m=-1, clear_range=0.5, remaining=1.0, step=0.25)
    assert d.action == 'advance'
    assert d.terminal is None


# ---------- 与状态机的衔接 ----------

@pytest.mark.parametrize('terminal', [ts.ARRIVED, ts.BLOCKED, ts.FAILED])
def test_all_reachable_terminals_are_task_tier(terminal):
    """本技能报的终态必须落在 **task-tier** 词汇表里。

    报 `FINISHED`（control-tier）会被网关的结果校验拒掉 ——
    那等于用任务的词汇表撒谎（D-032）。
    """
    assert terminal in ts.TASK_TERMINAL
    assert terminal != ts.FINISHED
    assert ts.is_waking(terminal) is True      # 这些终态应当唤醒 Agent（D-004）


def test_summarize_includes_the_distance():
    assert '0.750' in summarize(0.75, '走满上限')
