#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住注册表的性质：**它是数据、可以在启动时失败得响**。"""

import json

import pytest

from embodied_skill_gateway.registry import (
    ParamSpec, Registry, RegistryError, SkillSpec)


def _spec(name='control.move_relative', **kw):
    base = dict(tier='control', target='/control_skills/move_relative',
                srv_type='embodied_skills_interfaces/MoveRelative',
                params=[ParamSpec('x', minimum=-0.5, maximum=0.5),
                        ParamSpec('y', minimum=-0.5, maximum=0.5)],
                timeout_s=15.0,
                allowed_principals=['router.deterministic'],
                max_norm={'params': ['x', 'y'], 'max': 0.5})
    base.update(kw)
    return SkillSpec(name, **base)


# ---------- 构造期的自校验 ----------

def test_unknown_tier_is_rejected():
    with pytest.raises(RegistryError, match='未知层级'):
        _spec(tier='kitchen')


def test_missing_target_is_rejected():
    with pytest.raises(RegistryError, match='缺少 target'):
        _spec(target='')


def test_non_positive_timeout_is_rejected():
    with pytest.raises(RegistryError, match='timeout_s'):
        _spec(timeout_s=0)


def test_empty_allowed_principals_is_rejected():
    """一个谁都调不了的技能是**配置错误**，不是"已停用" —— 停用要用 available: false。

    这条防的是一种很难查的现场故障：技能看起来注册了、却永远调不通，
    而错误信息只会说"无权调用"，让人以为是自己身份写错了。
    """
    with pytest.raises(RegistryError, match='allowed_principals'):
        _spec(allowed_principals=[])


def test_duplicate_param_names_are_rejected():
    with pytest.raises(RegistryError, match='重复'):
        SkillSpec('x', tier='control', target='/t', srv_type='s',
                  params=[ParamSpec('a'), ParamSpec('a')],
                  allowed_principals=['p'])


def test_param_with_min_above_max_is_rejected():
    with pytest.raises(RegistryError, match='下限'):
        ParamSpec('a', minimum=1.0, maximum=0.0)


def test_unknown_param_type_is_rejected():
    with pytest.raises(RegistryError, match='未知类型'):
        ParamSpec('a', type='complex')


def test_max_norm_referencing_unknown_param_is_rejected():
    with pytest.raises(RegistryError, match='不存在的参数'):
        _spec(max_norm={'params': ['x', 'z'], 'max': 0.5})


def test_max_norm_with_wrong_keys_is_rejected():
    with pytest.raises(RegistryError, match='max_norm'):
        _spec(max_norm={'parameters': ['x', 'y'], 'max': 0.5})


# ---------- 类型判定 ----------

def test_bool_is_not_accepted_as_a_number():
    """`bool` 是 `int` 的子类，不特判的话 `True` 会通过 float 参数的类型检查。

    那等于把 `move_relative(x=True)` 当成 `x=1.0` 执行 —— 一个上游 bug 会变成
    一次真的运动，而且看不出任何异常。
    """
    assert ParamSpec('a', type='float').accepts_python_type(True) is False
    assert ParamSpec('a', type='int').accepts_python_type(True) is False
    assert ParamSpec('a', type='bool').accepts_python_type(True) is True


def test_int_is_accepted_for_float_param():
    assert ParamSpec('a', type='float').accepts_python_type(3) is True


# ---------- Registry ----------

def test_registry_rejects_duplicate_skill_names():
    with pytest.raises(RegistryError, match='重复'):
        Registry([_spec(), _spec()])


def test_empty_registry_is_rejected():
    with pytest.raises(RegistryError, match='为空'):
        Registry([])


def test_from_dict_accepts_both_shapes():
    d = {'tier': 'control', 'target': '/t', 'srv_type': 's',
         'allowed_principals': ['p']}
    assert 'a' in Registry.from_dict({'skills': {'a': d}})
    assert 'a' in Registry.from_dict({'a': d})


def test_registry_require_raises_for_unknown_skill():
    reg = Registry([_spec()])
    with pytest.raises(RegistryError, match='未注册'):
        reg.require('control.fly')


def test_public_view_does_not_leak_the_ros_service_name():
    """`~/list` 的投影**刻意不含** target（底层 ROS 服务名）。

    知道名字不等于能调用；少给一个"看起来像入口"的东西，
    就能少一条绕过 ~/invoke 准入的路。
    """
    view = _spec().public_view()
    assert 'target' not in view
    assert 'srv_type' not in view
    assert json.dumps(view)   # 可序列化


# ---------- 与底层能力的一致性 ----------

def test_policy_limit_wider_than_the_skill_is_rejected_at_startup():
    """★ 网关的策略上限**不得宽于**底层技能的可行域。

    宽出来的部分是空的：那些请求会一路走到 Control Skill 再被它拒绝，
    拒绝原因来自下层，上层的"策略"就没起作用。让这种配置错误在**启动时**就炸掉。
    """
    reg = Registry([_spec(max_norm={'params': ['x', 'y'], 'max': 2.0})])
    with pytest.raises(RegistryError, match='超过 Control Skill'):
        reg.check_against_control_skills_limits(max_distance=1.0)


def test_policy_limit_equal_to_the_skill_is_allowed():
    reg = Registry([_spec(max_norm={'params': ['x', 'y'], 'max': 1.0})])
    reg.check_against_control_skills_limits(max_distance=1.0)   # 不抛


def test_move_relative_without_max_norm_is_rejected_at_startup():
    reg = Registry([_spec(max_norm=None)])
    with pytest.raises(RegistryError, match='max_norm'):
        reg.check_against_control_skills_limits()


def test_shipped_registry_yaml_loads_and_passes_self_check():
    """真实的那份注册表必须能加载并通过自校验 —— 防止 YAML 与代码漂移。"""
    import os
    path = os.path.join(os.path.dirname(__file__), '..', 'config',
                        'skill_registry.yaml')
    reg = Registry.from_yaml(path)
    assert 'control.move_relative' in reg
    assert 'agent.planner' not in reg.require('control.move_relative').allowed_principals
    assert 'agent.planner' in reg.require('primitive.path_clear').allowed_principals
