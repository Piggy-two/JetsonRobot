#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`turn_plan` 的离线单测（不依赖 ROS）。

这个模块答错的两个方向代价不对称：
  · 该转不转 → 卡死在障碍前（烦人，但安全）；
  · **不该停却停** → 上层以为"这里被围住了"，于是去挪障碍物，而真相可能是雷达坏了。
所以这里把「**不知道 ≠ 受阻**」钉死 —— 它和 `advance_plan` 是同一条铁律。
"""

import pytest

from embodied_autonomous_skills.turn_plan import (
    TurnPlanError, decide, summarize, validate)
from embodied_skill_gateway import task_state as ts


# ---------- 参数校验 ----------

def test_accepts_sane_arguments():
    validate(1.5708, 0.30, 0.15, 1.0)
    validate(1.5708, 0.30, 0.15, -1.0)


@pytest.mark.parametrize('kwargs', [
    dict(max_angle=0.0, clear_range=0.3, step_angle=0.1, direction=1.0),
    dict(max_angle=-1.0, clear_range=0.3, step_angle=0.1, direction=1.0),
    dict(max_angle=1.0, clear_range=0.0, step_angle=0.1, direction=1.0),
    dict(max_angle=1.0, clear_range=0.3, step_angle=0.0, direction=1.0),
    dict(max_angle=1.0, clear_range=0.3, step_angle=2.0, direction=1.0),   # 步长 > 上限
    dict(max_angle=float('nan'), clear_range=0.3, step_angle=0.1, direction=1.0),
])
def test_rejects_bad_numbers(kwargs):
    with pytest.raises(TurnPlanError):
        validate(**kwargs)


@pytest.mark.parametrize('direction', [0.0, 0.5, 2.0, -0.5, 180.0])
def test_direction_must_be_exactly_plus_or_minus_one(direction):
    """**不能接受"接近 +1 的任意正数"** —— 那会让一个笔误（把弧度当方向）
    静默生效，表现是"转错边"。"""
    with pytest.raises(TurnPlanError):
        validate(1.0, 0.3, 0.1, direction)


def test_步长等于上限是允许的():
    validate(0.3, 0.3, 0.3, 1.0)          # 一步转满，合法


# ---------- 通畅：达成目标 ----------

def test_already_clear_arrives_without_turning():
    d = decide(clear=True, range_m=-1.0, clear_range=0.3,
               remaining=1.5, step_angle=0.1, direction=1.0)
    assert d.action == 'stop'
    assert d.angle == 0.0
    assert d.terminal == ts.ARRIVED


def test_clear_with_a_distant_echo_says_how_far():
    """`clear=True` 且**有**回波 = 回波在阈值**之外** —— 消息要说清多远，
    不能只说"通畅"（那会掩盖"前面其实有东西、只是还远"）。"""
    d = decide(True, 0.85, 0.30, 1.5, 0.1, 1.0)
    assert d.terminal == ts.ARRIVED
    assert '0.850' in d.note and '0.300' in d.note


# ---------- ★ 不知道 ≠ 受阻（本模块最要紧的一条） ----------

def test_no_reading_is_failed_not_blocked():
    """`clear=False` 且 `range<0` = **雷达没有数据**，不是"前面有障碍物"。

    报成 BLOCKED 会让排查方向完全跑偏（去挪障碍物，而该查雷达）。
    """
    d = decide(clear=False, range_m=-1.0, clear_range=0.3,
               remaining=1.5, step_angle=0.1, direction=1.0)
    assert d.terminal == ts.FAILED
    assert d.terminal != ts.BLOCKED
    assert d.action == 'stop'


def test_blocked_only_when_there_really_is_an_echo():
    d = decide(False, 0.12, 0.30, 1.5, 0.1, 1.0)
    assert d.action == 'turn'            # 还有余量 → 继续转
    assert d.terminal is None


# ---------- ★ 判定顺序：先看路，再看预算 ----------

def test_cleared_on_the_last_step_is_arrived_not_blocked():
    """**最后一步转完刚好通畅**时，`remaining` 正好耗尽。

    如果先判预算就会报成 `BLOCKED`（"转满上限仍受阻"）—— 而它其实**已经达成目标**。
    ⇒ 顺序必须是"**先看有没有通，再看还剩多少**"。
    """
    d = decide(clear=True, range_m=-1.0, clear_range=0.3,
               remaining=0.0, step_angle=0.1, direction=1.0)
    assert d.terminal == ts.ARRIVED


def test_exhausted_and_still_blocked_is_blocked():
    d = decide(clear=False, range_m=0.18, clear_range=0.30,
               remaining=0.0, step_angle=0.1, direction=1.0)
    assert d.action == 'stop'
    assert d.terminal == ts.BLOCKED
    assert '0.180' in d.note


# ---------- 转向与限幅 ----------

@pytest.mark.parametrize('direction,expect_sign', [(1.0, 1), (-1.0, -1)])
def test_turn_follows_direction(direction, expect_sign):
    d = decide(False, 0.20, 0.30, 1.5, 0.1, direction)
    assert d.action == 'turn'
    assert abs(d.angle) == pytest.approx(0.1)
    assert d.angle * expect_sign > 0


def test_last_step_is_clamped_to_remaining():
    """单步不能超过剩余量 —— 否则会转过头（超出的部分没人再判一次）。"""
    d = decide(False, 0.20, 0.30, remaining=0.04, step_angle=0.1, direction=1.0)
    assert d.action == 'turn'
    assert d.angle == pytest.approx(0.04)


def test_summarize_carries_the_sign():
    assert '+1.571' in summarize(1.5708, '通畅')
    assert '-0.300' in summarize(-0.30, '受阻')
