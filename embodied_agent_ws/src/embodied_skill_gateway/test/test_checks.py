#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""六项检查的单测。

★ 最重要的一组是文件末尾的「**反代理回归测试**」——
  网关存在的理由就是"拒绝某些底层技能会接受的东西"。
  如果哪天重构把这三条性质弄丢了，网关就退化成一个转发器，而这些测试会红。
"""

import json

import pytest

from embodied_skill_gateway.checks import (
    admit, validate_range, validate_result, validate_schema)
from embodied_skill_gateway.registry import (
    ParamSpec, Registry, SkillSpec)


def _move_spec(**kw):
    base = dict(
        tier='control', target='/control_skills/move_relative',
        srv_type='embodied_skills_interfaces/MoveRelative',
        params=[ParamSpec('x', minimum=-0.5, maximum=0.5),
                ParamSpec('y', minimum=-0.5, maximum=0.5)],
        timeout_s=15.0,
        allowed_principals=['router.deterministic', 'operator.manual'],
        max_norm={'params': ['x', 'y'], 'max': 0.5, 'note': '单次位移策略上限'})
    base.update(kw)
    return SkillSpec('control.move_relative', **base)


def _registry(spec=None):
    spec = spec or _move_spec()
    query = SkillSpec(
        'primitive.path_clear', tier='primitive', target='/lidar_driver/path_clear',
        srv_type='embodied_skills_interfaces/PathClear',
        params=[ParamSpec('width', minimum=0.0, maximum=6.28),
                ParamSpec('clear_range', minimum=0.0, maximum=25.0)],
        timeout_s=5.0,
        allowed_principals=['agent.planner', 'router.deterministic'])
    return Registry([spec, query])


ROUTER = 'router.deterministic'
AGENT = 'agent.planner'


# ---------- Schema ----------

def test_unknown_skill_is_rejected():
    r = admit(_registry(), ROUTER, 'control.fly', '{}')
    assert not r.accepted
    assert r.reason.startswith('REJECTED')


def test_unavailable_skill_is_rejected_with_its_reason():
    reg = _registry(_move_spec(available=False, unavailable_reason='底层未启动'))
    r = admit(reg, ROUTER, 'control.move_relative', '{"x": 0.1, "y": 0}')
    assert not r.accepted and '底层未启动' in r.reason


def test_missing_required_param_is_rejected():
    r = admit(_registry(), ROUTER, 'control.move_relative', '{"x": 0.1}')
    assert not r.accepted and '缺少必填参数' in r.reason


def test_extra_param_is_rejected_not_ignored():
    """多传参数必须报错。

    静默忽略的后果：调用方以为"我设了 max_speed"，实际它被丢掉，
    机器人按默认速度跑 —— 一个安静的、方向性的错误。
    """
    r = admit(_registry(), ROUTER, 'control.move_relative',
              '{"x": 0.1, "y": 0, "speed": 9}')
    assert not r.accepted and '未定义的参数' in r.reason


def test_bool_is_not_silently_accepted_as_a_number():
    r = admit(_registry(), ROUTER, 'control.move_relative', '{"x": true, "y": 0}')
    assert not r.accepted and '期望 float' in r.reason


def test_nan_and_infinity_are_rejected():
    """JSON 里 NaN/Infinity 是非法的，但 Python 的 json 默认**接受**它们 —— 必须显式挡掉。"""
    for bad in ('NaN', 'Infinity', '-Infinity'):
        r = admit(_registry(), ROUTER, 'control.move_relative',
                  '{"x": %s, "y": 0}' % bad)
        assert not r.accepted, bad


def test_malformed_json_is_rejected():
    r = admit(_registry(), ROUTER, 'control.move_relative', '{"x": ')
    assert not r.accepted and '合法 JSON' in r.reason


def test_json_that_is_not_an_object_is_rejected():
    r = admit(_registry(), ROUTER, 'control.move_relative', '[0.1, 0]')
    assert not r.accepted and 'JSON object' in r.reason


def test_schema_accepts_a_well_formed_request():
    assert validate_schema(_move_spec(), {'x': 0.2, 'y': -0.1}) is None


# ---------- Range ----------

def test_value_above_policy_maximum_is_rejected():
    why = validate_range(_move_spec(), {'x': 0.5, 'y': 0.0})
    assert why is None            # 恰好在上限：允许
    why = validate_range(_move_spec(), {'x': 0.51, 'y': 0.0})
    assert why is not None and '超过策略上限' in why


def test_value_below_policy_minimum_is_rejected():
    why = validate_range(_move_spec(), {'x': -0.51, 'y': 0.0})
    assert why is not None and '低于策略下限' in why


def test_max_norm_catches_the_combination_single_field_checks_miss():
    """★ 这是 `max_norm` 存在的理由。

    x=0.4 与 y=0.4 **各自**都在 ±0.5 内，但位移是 0.566 > 0.5。
    只看单字段的上下限，会放行一个比策略允许的更远的请求。
    """
    why = validate_range(_move_spec(), {'x': 0.4, 'y': 0.4})
    assert why is not None and '模长' in why


def test_max_norm_boundary_is_inclusive():
    assert validate_range(_move_spec(), {'x': 0.5, 'y': 0.0}) is None


# ---------- Result Validation ----------

def test_control_skill_result_is_never_verified():
    """★ D-026 / D-032：Control Skill 的 success 是"速度按时长发完了"。

    今天没有任何独立反馈能确认"到位了"，所以 verified **在物理上**只能是 False。
    这个函数将来才会按技能类型返回 True。
    """
    verified, why = validate_result('control', True, 3.3, '已完成（开环：按时间发出）')
    assert verified is False and why is None


def test_negative_elapsed_is_rejected():
    verified, why = validate_result('control', True, -1.0)
    assert why is not None and '为负' in why


def test_non_finite_elapsed_is_rejected():
    _, why = validate_result('control', True, float('nan'))
    assert why is not None


def test_failure_without_a_reason_is_rejected():
    """失败却不给原因，会让上层无法判断该重试还是该放弃 —— 这不是小事。"""
    _, why = validate_result('control', False, 1.0, '')
    assert why is not None and '没给原因' in why


# ---------- 检查顺序 ----------

def test_permission_is_checked_before_schema():
    """★ 顺序是刻意的：先授权、再校验。

    若先校验 schema，一个"没权限的调用方 + 参数写错"的请求会得到
    "你的参数写错了" —— 那会让人以为**把参数改对就能调了**。
    架构约束不该被一个参数笔误掩盖。
    """
    r = admit(_registry(), AGENT, 'control.move_relative', '{"x": "not a number"}')
    assert not r.accepted
    assert '无权调用' in r.reason
    assert '期望 float' not in r.reason


# ==========================================================================
# ★ 反代理回归测试 —— 网关必须拒绝某些底层技能会接受的东西
# ==========================================================================

def test_agent_planner_cannot_invoke_a_control_skill():
    """① Permission：`agent.planner` 调 control 技能被拒。

    底层 Control Skill 会痛快接受（它只管运动与前置条件，不问"谁在问"）。
    这里是 D-003「Agent 不得触达控制技能」**唯一**能真正执行的地方。
    """
    spec = _move_spec()
    # 参数完全合法 —— 拒它的理由只能是权限，不能是别的
    r = admit(_registry(spec), AGENT, 'control.move_relative', '{"x": 0.1, "y": 0.0}')
    assert not r.accepted
    assert '无权调用' in r.reason


def test_agent_planner_can_invoke_a_read_only_primitive():
    """Permission 是**按技能**的策略，不是"一律不给 Agent"的一刀切。

    只读查询不碰执行器，让 Agent 用它做感知是合理且必要的。
    """
    r = admit(_registry(), AGENT, 'primitive.path_clear',
              '{"width": 0.5, "clear_range": 1.0}')
    assert r.accepted


def test_gateway_range_is_stricter_than_the_underlying_skill():
    """② Range：0.8 m 会被网关照拒，而底层 Control Skill 的 `max_distance` 是 1.0 ——
    它会接受。

    这条差异**就是网关的价值**：策略（允不允许要）比能力（能不能做到）更严。
    """
    r = admit(_registry(), ROUTER, 'control.move_relative', '{"x": 0.8, "y": 0.0}')
    assert not r.accepted and 'REJECTED' in r.reason


def test_result_never_claims_arrival_for_control_tier():
    """③ 语义改写：control-tier 的结果**不得**产出 `ARRIVED` 这类任务终态（D-032）。

    把 success=true 映射成"到达了"是撒谎，而且会让 Agent 的后续规划
    建立在"以为到位了"的假前提上。
    """
    from embodied_skill_gateway import task_state as ts
    verified, _ = validate_result('control', True, 1.0, '已完成（开环）')
    assert verified is False
    assert ts.is_terminal(ts.FINISHED)
    assert not ts.is_waking(ts.FINISHED)


def test_anonymous_callers_are_rejected():
    """没有 principal 一律拒绝 —— 不做匿名放行。

    否则 D-003 的架构红线会在"第一次有人加新调用方"时静默失效。
    """
    for principal in ('', None, '   '):
        r = admit(_registry(), principal, 'control.move_relative', '{"x": 0.1, "y": 0}')
        assert not r.accepted
