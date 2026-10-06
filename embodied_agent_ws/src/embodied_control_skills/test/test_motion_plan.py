#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`motion_plan` 的离线单测（不需要 ROS，也不会让车动）。

核心要钉住的是**积分性质**：恒定速度 × 时长 == 请求的位移。
干跑验证会去量同一条性质（把实际发出的速度序列积分），
所以这里先把它在纯逻辑层钉死 —— 出问题时能立刻分清是"算错了"还是"发错了"。
"""
import math

import pytest

from embodied_control_skills.motion_plan import (
    PlanError, plan_rotate, plan_translate, within_chassis_limits)

SPEED = 0.15
RATE = 0.40
MAXD = 1.0
MAXA = math.pi


# ---------- 平移 ----------

def test_translate_forward_integrates_exactly():
    p = plan_translate(0.5, 0.0, SPEED, MAXD)
    assert p.vx == pytest.approx(SPEED)
    assert p.vy == pytest.approx(0.0)
    assert p.vx * p.duration == pytest.approx(0.5)      # ← 这才是语义所在


def test_translate_strafe_left_integrates_exactly():
    p = plan_translate(0.0, 0.3, SPEED, MAXD)
    assert p.vx == pytest.approx(0.0)
    assert p.vy == pytest.approx(SPEED)
    assert p.vy * p.duration == pytest.approx(0.3)


@pytest.mark.parametrize('dx,dy', [(0.3, -0.4), (-0.2, 0.2), (0.5, 0.5)])
def test_translate_diagonal_integrates_to_the_requested_vector(dx, dy):
    p = plan_translate(dx, dy, SPEED, MAXD)
    assert p.vx * p.duration == pytest.approx(dx, abs=1e-12)
    assert p.vy * p.duration == pytest.approx(dy, abs=1e-12)
    # 速度大小恒为标称速度（方向由单位向量承担）
    assert math.hypot(p.vx, p.vy) == pytest.approx(SPEED)


def test_translate_rejects_too_far():
    with pytest.raises(PlanError):
        plan_translate(2.0, 0.0, SPEED, MAXD)


def test_translate_boundary_is_allowed():
    assert plan_translate(MAXD, 0.0, SPEED, MAXD) is not None


def test_translate_tiny_is_noop():
    assert plan_translate(1e-9, 0.0, SPEED, MAXD) is None


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_translate_rejects_non_finite(bad):
    with pytest.raises(PlanError):
        plan_translate(bad, 0.0, SPEED, MAXD)
    with pytest.raises(PlanError):
        plan_translate(0.1, 0.0, bad, MAXD)


def test_translate_rejects_non_positive_speed():
    with pytest.raises(PlanError):
        plan_translate(0.1, 0.0, 0.0, MAXD)
    with pytest.raises(PlanError):
        plan_translate(0.1, 0.0, -0.1, MAXD)


# ---------- 旋转 ----------

def test_rotate_ccw_positive():
    p = plan_rotate(math.pi / 2, RATE, MAXA)
    assert p.wz == pytest.approx(RATE)
    assert p.wz * p.duration == pytest.approx(math.pi / 2)   # ← 逆时针为正
    assert p.vx == 0.0 and p.vy == 0.0


def test_rotate_cw_negative():
    p = plan_rotate(-math.pi / 2, RATE, MAXA)
    assert p.wz == pytest.approx(-RATE)
    assert p.wz * p.duration == pytest.approx(-math.pi / 2)


def test_rotate_rejects_too_far():
    with pytest.raises(PlanError):
        plan_rotate(4.0, RATE, MAXA)


def test_rotate_tiny_is_noop():
    assert plan_rotate(1e-9, RATE, MAXA) is None


# ---------- 与底盘限幅的一致性断言 ----------

def test_within_limits_accepts_normal_plan():
    p = plan_translate(0.5, 0.0, SPEED, MAXD)
    ok, why = within_chassis_limits(p, 0.2, 0.2, 0.5)
    assert ok and why == ''


def test_within_limits_flags_misconfiguration():
    """标称速度配到限幅之外时必须被拦下 —— 否则会被 Motor Driver 静默钳掉，
    实际位移与时长都对不上，而调用方拿到的还是 success。"""
    p = plan_translate(0.5, 0.0, 0.9, MAXD)
    ok, why = within_chassis_limits(p, 0.2, 0.2, 0.5)
    assert not ok and 'vx' in why


def test_within_limits_handles_noop():
    assert within_chassis_limits(None, 0.2, 0.2, 0.5) == (True, '')
