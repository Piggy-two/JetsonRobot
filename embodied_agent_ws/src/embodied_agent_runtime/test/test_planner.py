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
        SkillSpec('autonomous.turn_until_clear', tier='task',
                  target='/autonomous_skills/turn_until_clear', srv_type='s',
                  params=[ParamSpec('max_angle', minimum=0.1, maximum=3.141592653589793),
                          ParamSpec('clear_range', minimum=0.1, maximum=1.0),
                          ParamSpec('step_angle', minimum=0.05, maximum=1.5707963267948966),
                          ParamSpec('direction', minimum=-1.0, maximum=1.0)],
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
    assert r.steps == []


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
    assert len(r.steps) == 1                       # 单步规则 ⇒ 一个步骤
    assert r.steps[0].skill == 'autonomous.advance_until_blocked'
    assert r.steps[0].args == {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}


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
            assert all(reg.require(s.skill).tier == 'task' for s in r.steps)


# ==========================================================================
# ★ 多步计划（D-043）
# ==========================================================================

MULTI_RULE = {
    '脱困': {'steps': [
        {'skill': 'autonomous.advance_until_blocked',
         'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
        {'skill': 'autonomous.turn_until_clear',
         'args': {'max_angle': 1.0, 'clear_range': 0.5, 'step_angle': 0.3,
                  'direction': 1.0}},
        {'skill': 'autonomous.advance_until_blocked',
         'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
    ]}}


def test_multi_step_rule_yields_an_ordered_plan():
    r = planner.plan('脱困', MULTI_RULE, _registry())
    assert r.accepted is True
    assert [s.skill for s in r.steps] == [
        'autonomous.advance_until_blocked',
        'autonomous.turn_until_clear',
        'autonomous.advance_until_blocked']


def test_a_single_bad_step_rejects_the_whole_plan():
    """★ **不做半条**。

    如果第 1 步能做、第 2 步踩红线，那么"先做第 1 步、做到一半再报错"
    会把车留在**半执行完**的状态，而计划本身已经作废 —— 没人知道该继续还是退回去。
    ⇒ 动之前就知道整条合法。
    """
    bad = {'脱困': {'steps': [
        {'skill': 'autonomous.advance_until_blocked', 'args': {'max_distance': 0.2,
                                                             'clear_range': 0.5,
                                                             'step': 0.1}},
        {'skill': 'control.move_relative', 'args': {'x': 0.1}},      # ← 第 2 步踩红线
    ]}}
    r = planner.plan('脱困', bad, _registry())
    assert r.accepted is False
    assert r.steps == []                       # 一个步骤都不给
    assert '第 2 步' in r.reason               # 且指出是**哪一步**
    assert '整条计划被拒' in r.reason


def test_writing_both_skill_and_steps_is_refused():
    """两种写法同时出现 ⇒ 写的人自己也没想清要走几步。"""
    r = planner.plan('脱困', {'脱困': {'skill': 'autonomous.advance_until_blocked',
                                       'steps': [{'skill': 'x'}]}}, _registry())
    assert r.accepted is False
    assert '同时写了' in r.reason


def test_steps_must_be_a_list():
    r = planner.plan('脱困', {'脱困': {'steps': 'advance'}}, _registry())
    assert r.accepted is False
    assert '列表' in r.reason


def test_rule_with_neither_skill_nor_steps_is_refused():
    r = planner.plan('脱困', {'脱困': {'nonsense': 1}}, _registry())
    assert r.accepted is False
    assert '既没有' in r.reason


# ==========================================================================
# ★ 步骤上的条件（D-047）：`when: {prev: <终态>}`
# ==========================================================================

def _plan(steps):
    return planner.accept_plan(steps, _registry(), '测试')


def test_a_condition_on_a_later_step_is_accepted():
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
               {'skill': 'autonomous.turn_until_clear',
                'args': {'max_angle': 1.0, 'clear_range': 0.5,
                         'step_angle': 0.3, 'direction': 1.0},
                'when': {'prev': 'TARGET_FOUND'}}])
    assert r.accepted is True
    assert r.steps[1].when == {'prev': 'TARGET_FOUND'}
    assert r.steps[0].when is None


def test_a_condition_on_the_FIRST_step_is_refused():
    """★ 第 1 步前面没有上一步，条件无从判定 —— 必须**明确拒绝**。

    如果放行（或者当没写），用户会以为加了限制，而实际什么都没发生。
    """
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1},
                'when': {'prev': 'TARGET_FOUND'}}])
    assert r.accepted is False
    assert '第 1 步不能带 when' in r.reason


def test_an_unknown_key_inside_when_is_refused():
    """★ 写了 `if:` 之类 —— 用户以为加了限制，实际没有。必须报错。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
               {'skill': 'autonomous.turn_until_clear',
                'args': {'max_angle': 1.0, 'clear_range': 0.5,
                         'step_angle': 0.3, 'direction': 1.0},
                'when': {'if': 'TARGET_FOUND'}}])
    assert r.accepted is False
    assert '不认识的键' in r.reason


def test_an_unknown_state_inside_when_is_refused():
    """★ 写了个不存在的状态名（比如 `SUCCESS`）⇒ 这一步**永远不会执行**，
    而且不会报错 —— 所以必须在规划期就拒掉。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
               {'skill': 'autonomous.turn_until_clear',
                'args': {'max_angle': 1.0, 'clear_range': 0.5,
                         'step_angle': 0.3, 'direction': 1.0},
                'when': {'prev': 'SUCCESS'}}])
    assert r.accepted is False
    assert '不是 task-tier 终态' in r.reason


def test_a_non_dict_when_is_refused():
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}},
               {'skill': 'autonomous.turn_until_clear',
                'args': {'max_angle': 1.0, 'clear_range': 0.5,
                         'step_angle': 0.3, 'direction': 1.0},
                'when': 'TARGET_FOUND'}])
    assert r.accepted is False
    assert '必须是一个映射' in r.reason


@pytest.mark.parametrize('state,expected', [
    ('TARGET_FOUND', True), ('BLOCKED', False), (None, False), ('', False)])
def test_when_ok(state, expected):
    step = planner.Step('x', {}, {'prev': 'TARGET_FOUND'})
    assert planner.when_ok(step, state) is expected


def test_when_ok_without_a_condition_is_always_true():
    assert planner.when_ok(planner.Step('x', {}), 'anything') is True


def test_a_condition_only_difference_makes_it_a_DIFFERENT_plan():
    """★ 这条与 D-044 的防死循环有关：两条只在**条件**上不同的计划是不同的走法。

    把它们当成"同一条"会误挡一次真正的换路；反过来当成"不同"则受预算约束 ——
    后者是安全的一侧。
    """
    a = [planner.Step('x', {}, None)]
    b = [planner.Step('x', {}, {'prev': 'TARGET_FOUND'})]
    assert planner.same_plan(a, b) is False


def test_render_plan_shows_the_condition():
    """★ 重规划时把"试过什么"给模型看 —— 条件不写出来，模型就不知道上次带没带。"""
    text = planner.render_plan([planner.Step('x', {'a': 1}, {'prev': 'TARGET_FOUND'})])
    assert '仅当上一步是 TARGET_FOUND' in text


# ==========================================================================
# ★ 参数绑定（D-048）：把上一步结果里的字段接到这一步的参数上
# ==========================================================================

ADV_ARGS = {'max_distance': 0.2, 'clear_range': 0.5, 'step': 0.1}
TURN_ARGS = {'max_angle': 1.0, 'clear_range': 0.5, 'step_angle': 0.3}


def test_a_binding_is_accepted_and_survives_validation():
    """★ 绑定的值在规划期**还不知道**，所以用占位值过形状检查，
    **绑定本身**必须原样留在计划里（派发时才换真值）。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS, direction={'from': 'prev', 'field': 'side',
                                                   'as': 'opposite_sign'})}])
    assert r.accepted is True
    assert r.steps[1].args['direction'] == {'from': 'prev', 'field': 'side',
                                            'as': 'opposite_sign'}


def test_a_binding_on_the_first_step_is_refused():
    """★ 第 1 步前面没有上一步 —— 没有东西可接。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked',
                'args': dict(ADV_ARGS,
                             max_distance={'from': 'prev', 'field': 'travelled'})}])
    assert r.accepted is False
    assert '第 1 步不能有参数绑定' in r.reason


def test_a_binding_to_a_nonexistent_param_is_refused():
    """绑定接到一个**不存在的参数名**上 —— 占位值给不出来，必须当场拒。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS, nonsense={'from': 'prev', 'field': 'side'})}])
    assert r.accepted is False
    assert '不存在的参数' in r.reason


def test_an_unknown_transform_is_refused():
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS, direction={'from': 'prev', 'field': 'side',
                                                   'as': 'flip'})}])
    assert r.accepted is False
    assert 'as 不认识' in r.reason


def test_binding_to_anything_but_the_previous_step_is_refused():
    """指向"更早的第 3 步"之类 —— 会让"第几步"变成要人肉追的算术题。"""
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS, direction={'from': 'step1', 'field': 'side'})}])
    assert r.accepted is False
    assert '只支持 from' in r.reason


def test_a_binding_with_a_missing_field_is_refused():
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS, direction={'from': 'prev'})}])
    assert r.accepted is False
    assert '缺 field' in r.reason


@pytest.mark.parametrize('raw,expected', [
    ({'side': -0.7}, 'value'),
])
def test_resolve_value_passes_through(raw, expected):
    assert expected == 'value'
    v, why = planner.resolve_binding({'from': 'prev', 'field': 'side'}, raw)
    assert why == '' and v == -0.7


def test_resolve_sign_and_opposite_sign_are_BOTH_available_and_different():
    """★★ 这个测试钉的是那个**符号陷阱**。

    `side` 是画面里的左右（左 = 负），`direction` 是 +1 = 逆时针 = 左转。
    ⇒「朝看到的那一边转」要 `opposite_sign`。两种变换都得在，而且**必须不一样** ——
    哪天有人"顺手统一"成一个，这条会炸。
    """
    prev = {'side': -0.7}          # 目标在画面**左**边
    sign, _ = planner.resolve_binding({'from': 'prev', 'field': 'side',
                                       'as': 'sign'}, prev)
    opp, _ = planner.resolve_binding({'from': 'prev', 'field': 'side',
                                      'as': 'opposite_sign'}, prev)
    assert sign == -1.0 and opp == +1.0     # 左转 = +1 → 用 opposite_sign
    assert sign != opp


def test_resolve_a_missing_field_lists_what_is_actually_there():
    """★ 只说"没有这个字段"，排查的人得自己去翻技能定义 —— 把实际字段列出来。"""
    v, why = planner.resolve_binding({'from': 'prev', 'field': 'nope'}, {'side': 1.0})
    assert v is None
    assert 'nope' in why and 'side' in why


def test_resolve_a_non_numeric_field_for_a_sign_is_refused_not_guessed():
    """★ 要对一个不是数的字段取符号 ⇒ **拒**，不猜（猜出来的方向没人能解释）。"""
    v, why = planner.resolve_binding({'from': 'prev', 'field': 'state',
                                      'as': 'sign'}, {'state': 'TARGET_FOUND'})
    assert v is None and '不是数' in why


def test_resolve_without_a_result_object_is_refused():
    v, why = planner.resolve_binding({'from': 'prev', 'field': 'side'}, None)
    assert v is None and '没有给出可用的结果' in why


# ---- 绑定与条件一起用（仓库示例规则里那条）--------------------------------

def test_condition_and_binding_together():
    """★ 「看到就往那边转」= 条件（**只在看到时**）+ 绑定（**往哪边转由 side 决定**）。"""
    from embodied_agent_runtime import llm_planner
    menu = llm_planner.skill_menu(_registry())
    assert menu, '菜单不该是空的（本测试顺带确认注册表可用）'
    r = _plan([{'skill': 'autonomous.advance_until_blocked', 'args': ADV_ARGS},
               {'skill': 'autonomous.turn_until_clear',
                'args': dict(TURN_ARGS,
                             direction={'from': 'prev', 'field': 'side',
                                        'as': 'opposite_sign'}),
                'when': {'prev': 'TARGET_FOUND'}}])
    assert r.accepted is True
    assert r.steps[1].when == {'prev': 'TARGET_FOUND'}
    assert r.steps[1].args['direction']['as'] == 'opposite_sign'
