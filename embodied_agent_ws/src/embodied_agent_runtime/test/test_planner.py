#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 Planner 的两件事：**默认什么都不做**，以及**架构红线**（D-003）。"""

import pytest

from embodied_agent_runtime import planner
from embodied_skill_gateway.registry import ParamSpec, Registry, SkillSpec


def _registry():
    # ⚠️ 这几个 ParamSpec 要**与真注册表同形**（`skill_registry.yaml` 里
    #    `advance_until_blocked` 声明了三个参数，且都是必填）。
    #    本测试原本只声明了 `max_distance` —— 那时 planner 不校验参数，看不出差别；
    #    现在两条规划路径共用 `accept_skill()`、参数校验也接上了，声明不全就会
    #    让"合法的规则"被误判成"多写了参数"。
    return Registry([
        SkillSpec('autonomous.advance_until_blocked', tier='task',
                  target='/autonomous_skills/advance_until_blocked', srv_type='s',
                  params=[ParamSpec('max_distance', minimum=0.01, maximum=0.5),
                          ParamSpec('clear_range', minimum=0.1, maximum=1.0),
                          ParamSpec('step', minimum=0.02, maximum=0.2)],
                  timeout_s=45.0,
                  allowed_principals=['agent.planner']),
        SkillSpec('control.move_relative', tier='control',
                  target='/control_skills/move_relative', srv_type='s',
                  params=[ParamSpec('x')], timeout_s=15.0,
                  allowed_principals=['router.deterministic']),
        SkillSpec('primitive.path_clear', tier='primitive',
                  target='/lidar_driver/path_clear', srv_type='s',
                  params=[ParamSpec('width')], timeout_s=5.0,
                  allowed_principals=['agent.planner']),
        SkillSpec('autonomous.broken', tier='task', target='/t', srv_type='s',
                  params=[], timeout_s=10.0, allowed_principals=['agent.planner'],
                  available=False, unavailable_reason='底层未启动'),
    ])


TASK_RULE = {'走一小段': {'skill': 'autonomous.advance_until_blocked',
                          'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}}}


# ---------- 默认：什么都不做 ----------

def test_empty_rule_table_refuses_without_naming_a_skill():
    """★ 规则表为空 ⟹ 规则这一跳拒绝，且**不编造**一个技能名。

    ⚠️ 措辞在 D-038 之后改过：以前写"需要 LLM 规划，当前未实现（Phase 7）"，
    现在 LLM 已经接上了，"没命中"只说明**规则表里没有这一条** ——
    能不能做由下一跳回答。理由必须与事实一致，否则会把人指到错的方向去。
    """
    r = planner.plan('去桌子旁边找杯子', {}, _registry())
    assert r.accepted is False
    assert '没有匹配' in r.reason
    assert r.skill is None


def test_unmatched_text_is_refused_with_the_same_honest_reason():
    r = planner.plan('把这个房间扫一遍', TASK_RULE, _registry())
    assert r.accepted is False
    assert '没有匹配' in r.reason


def test_empty_text_is_refused():
    r = planner.plan('', TASK_RULE, _registry())
    assert r.accepted is False


# ---------- 命中规则 ----------

def test_matching_rule_yields_a_task_tier_call():
    r = planner.plan('走一小段', TASK_RULE, _registry())
    assert r.accepted is True
    assert r.skill == 'autonomous.advance_until_blocked'
    assert r.args == {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}


def test_matching_ignores_punctuation_and_spaces():
    for text in ('走一小段', '走一小段。', ' 走一小段 ', '走一小段！'):
        assert planner.plan(text, TASK_RULE, _registry()).accepted is True, text


# ==========================================================================
# ★★ 架构红线：Agent 只能命名 task-tier 技能（D-003 / D-029）
# ==========================================================================

def test_rule_naming_a_control_skill_is_rejected():
    """★ 这一条是 D-003 从"文档里的一句话"变成"代码里的一次拒绝"。

    没有它，将来 LLM 顺手填一条 `control.move_relative` 的规则，
    就能让 Agent 绕过准入直接驱动控制层 —— 而且**没有任何地方会报错**。
    """
    rules = {'偷偷动一下': {'skill': 'control.move_relative', 'args': {'x': 0.5}}}
    r = planner.plan('偷偷动一下', rules, _registry())
    assert r.accepted is False
    assert '架构红线' in r.reason
    assert 'control' in r.reason


def test_rule_naming_a_primitive_is_also_rejected():
    """只读查询也不行 —— 红线是按**层级**划的，不是按"危不危险"划的。

    否则"哪些能碰"会变成一次次的个案判断，而个案判断会随人/随模型漂移。
    """
    rules = {'看看前面': {'skill': 'primitive.path_clear', 'args': {'width': 1.0}}}
    r = planner.plan('看看前面', rules, _registry())
    assert r.accepted is False
    assert '架构红线' in r.reason


def test_rule_naming_an_unregistered_skill_is_rejected():
    """规则表与注册表漂移必须**报错**，否则会表现为"技能调不通"，
    而真正的原因是名字写错 —— 排查方向会跑偏。"""
    rules = {'走一小段': {'skill': 'autonomous.does_not_exist', 'args': {}}}
    r = planner.plan('走一小段', rules, _registry())
    assert r.accepted is False
    assert '未注册' in r.reason


def test_rule_without_a_skill_field_is_rejected():
    rules = {'走一小段': {'args': {'max_distance': 0.2}}}
    r = planner.plan('走一小段', rules, _registry())
    assert r.accepted is False
    assert 'skill' in r.reason


def test_unavailable_skill_is_refused_with_its_reason():
    rules = {'走一小段': {'skill': 'autonomous.broken', 'args': {}}}
    r = planner.plan('走一小段', rules, _registry())
    assert r.accepted is False
    assert '不可用' in r.reason and '底层未启动' in r.reason


def test_planner_output_can_never_describe_a_control_action():
    """把上面几条合起来看：**planner 的输出类型里根本没有"控制命令"这一档**。

    它只能给一个**技能名**，而这个技能名必须注册在册且属于 task-tier。
    架构红线因此不是靠自觉，而是靠"这个函数产不出那种东西"。
    """
    reg = _registry()
    for text, rules in (('走一小段', TASK_RULE),
                        ('偷偷动一下', {'偷偷动一下': {'skill': 'control.move_relative'}})):
        r = planner.plan(text, rules, reg)
        if r.accepted:
            assert reg.require(r.skill).tier == 'task'
