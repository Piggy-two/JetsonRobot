#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`llm_planner` 的离线单测 —— **不碰网络、不需要密钥**。

这一层最大的风险不是"选错技能"，而是**某一类错误悄悄地不被拦**。
所以下面的用例不只是"路径对不对"，每一条都在问同一个问题：

    **这一种失败，有没有变成一句说清原因的中文拒绝？**
    **有没有任何一种情况能绕过架构红线？**

最后那条不变量（`test_no_path_can_accept_a_non_task_skill`）是把前面所有用例
合起来看的：**无论模型返回什么，被接受的计划都只能是 task-tier。**
"""

import pytest

from embodied_agent_runtime import planner
from embodied_agent_runtime.llm_client import LlmTransportError, LlmUnavailable
from embodied_agent_runtime.llm_planner import (
    REFUSE_LLM_DISABLED, SOURCE_LLM, SOURCE_NONE, SOURCE_RULES,
    ReplyError, build_messages, parse_reply, plan_task, plan_with_llm, skill_menu)
from embodied_skill_gateway.registry import ParamSpec, Registry, SkillSpec


def _registry():
    return Registry([
        SkillSpec('autonomous.advance_until_blocked', tier='task',
                  target='/autonomous_skills/advance_until_blocked', srv_type='s',
                  description='一步一步往前走，每一步重新问雷达，直到前方受阻',
                  params=[ParamSpec('max_distance', minimum=0.01, maximum=0.5),
                          ParamSpec('clear_range', minimum=0.1, maximum=1.0),
                          ParamSpec('step', minimum=0.02, maximum=0.2)],
                  timeout_s=45.0, allowed_principals=['agent.planner']),
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
                  params=[ParamSpec('x', minimum=-0.5, maximum=0.5),
                          ParamSpec('y', minimum=-0.5, maximum=0.5)],
                  timeout_s=15.0, allowed_principals=['router.deterministic']),
        SkillSpec('autonomous.broken', tier='task', target='/t', srv_type='s',
                  params=[], timeout_s=10.0, allowed_principals=['agent.planner'],
                  available=False, unavailable_reason='底层未启动'),
    ])


GOOD_ARGS = {'max_distance': 0.3, 'clear_range': 0.5, 'step': 0.1}
GOOD_REPLY = '{"skill": "autonomous.advance_until_blocked", "args": %s}' % (
    '{"max_distance": 0.3, "clear_range": 0.5, "step": 0.1}')


class FakeClient:
    """假 LLM：回一段定好的话，或抛一个定好的异常。**记录被调用过几次。**"""

    def __init__(self, reply=None, error=None, timeout_s=8.0):
        self._reply, self._error, self.timeout_s = reply, error, timeout_s
        self.calls = []

    def complete(self, messages):
        self.calls.append(messages)
        if self._error is not None:
            raise self._error
        return self._reply


# ==========================================================================
# 菜单：只放"这一层能碰的"
# ==========================================================================

def test_menu_exposes_only_available_task_skills():
    menu = skill_menu(_registry())
    # 菜单里**只有可用的 task-tier**：两个都在（`control.*`/`primitive.*`
    # 与不可用的 `autonomous.broken` 都不该出现）
    assert [m['skill'] for m in menu] == ['autonomous.advance_until_blocked',
                                          'autonomous.turn_until_clear']


def test_menu_carries_description_units_and_limits():
    """没有单位与范围的菜单，模型只能猜；猜出来的参数会被参数校验拒掉。"""
    p = skill_menu(_registry())[0]['params']
    assert {x['name'] for x in p} == {'max_distance', 'clear_range', 'step'}
    assert all('min' in x and 'max' in x and 'unit' in x for x in p)


def test_menu_never_leaks_a_callable_target():
    """`public_view()` 刻意不含 `target` —— 知道名字不等于能直接调服务（D-031）。"""
    text = str(skill_menu(_registry()))
    assert '/autonomous_skills/' not in text


def test_prompt_states_the_red_line_and_lists_the_menu():
    msgs = build_messages('去桌子旁边找杯子', skill_menu(_registry()))
    joined = '\n'.join(m['content'] for m in msgs)
    assert 'autonomous.advance_until_blocked' in joined
    assert 'control.*' in joined and 'primitive.*' in joined
    assert '去桌子旁边找杯子' in joined


# ==========================================================================
# 回包解析：**解析要宽，校验才严**
# ==========================================================================

def test_parse_plain_json():
    # 一步与多步是**同一个形状**（steps 是长度为 1 的列表）——
    # 少一个分支，也少一处"单步走这条、多步走那条"从而严格度分家的机会。
    assert parse_reply(GOOD_REPLY) == (
        'steps', [{'skill': 'autonomous.advance_until_blocked', 'args': GOOD_ARGS}])


def test_parse_tolerates_code_fences():
    assert parse_reply('```json\n' + GOOD_REPLY + '\n```')[0] == 'steps'


def test_parse_refuse():
    assert parse_reply('{"refuse": "机器人还没有找杯子的能力"}') == (
        'refuse', '机器人还没有找杯子的能力')


def test_parse_rejects_non_json():
    with pytest.raises(ReplyError):
        parse_reply('好的，我这就去！')


def test_parse_rejects_non_object():
    with pytest.raises(ReplyError):
        parse_reply('[1, 2, 3]')


def test_parse_rejects_object_without_skill_or_refuse():
    with pytest.raises(ReplyError):
        parse_reply('{"plan": "walk"}')


def test_parse_rejects_non_object_args():
    with pytest.raises(ReplyError):
        parse_reply('{"skill": "autonomous.advance_until_blocked", "args": [0.3]}')


def test_reply_error_carries_the_raw_text_for_the_log():
    """模型胡说的原文必须留得下来 —— 拒绝理由给人看，原文给排查用。"""
    with pytest.raises(ReplyError) as e:
        parse_reply('我不知道该说什么')
    assert e.value.raw == '我不知道该说什么'


# ==========================================================================
# ★★ 红线：LLM 与规则表走**同一个**校验口
# ==========================================================================

def test_llm_choosing_a_control_skill_hits_the_architecture_red_line():
    """★★ 这一条就是"代码里执行的架构红线"（D-003 / D-029）对 LLM 的版本。

    模型说"我让控制层转一下"—— Agent 层必须当场拒绝，
    而不是把 `control.move_relative` 丢给网关（那样 `agent.planner` 会被权限挡下，
    但那已经是**准入**层的拒绝；这一层要自己先拦住，因为**提示词是给模型的**，
    而模型可以无视它）。
    """
    c = FakeClient('{"skill": "control.move_relative", "args": {"x": 0.3, "y": 0}}')
    r, _ = plan_with_llm('往前挪一点', _registry(), c)
    assert r.accepted is False
    assert '架构红线' in r.reason
    assert 'control' in r.reason


def test_llm_choosing_an_unregistered_skill_is_rejected():
    c = FakeClient('{"skill": "semantic.search_object", "args": {}}')
    r, _ = plan_with_llm('找杯子', _registry(), c)
    assert r.accepted is False
    assert '未注册' in r.reason


def test_llm_choosing_an_unavailable_skill_is_rejected_with_its_reason():
    c = FakeClient('{"skill": "autonomous.broken", "args": {}}')
    r, _ = plan_with_llm('走一段', _registry(), c)
    assert r.accepted is False
    assert '不可用' in r.reason and '底层未启动' in r.reason


def test_llm_good_choice_is_accepted():
    c = FakeClient(GOOD_REPLY)
    r, raw = plan_with_llm('往前走一小段', _registry(), c)
    assert r.accepted is True
    assert len(r.steps) == 1
    assert r.steps[0].skill == 'autonomous.advance_until_blocked'
    assert r.steps[0].args == GOOD_ARGS
    assert raw == GOOD_REPLY
    assert len(c.calls) == 1


# ---------- 参数校验复用网关那份代码 ----------

def test_missing_required_arg_is_rejected():
    c = FakeClient('{"skill": "autonomous.advance_until_blocked", "args": {"max_distance": 0.3}}')
    r, _ = plan_with_llm('走一段', _registry(), c)
    assert r.accepted is False
    assert '缺少必填参数' in r.reason


def test_extra_arg_is_rejected_not_silently_ignored():
    args = dict(GOOD_ARGS, speed=1.0)
    c = FakeClient('{"skill": "autonomous.advance_until_blocked", "args": %s}'
                   % __import__('json').dumps(args))
    r, _ = plan_with_llm('走一段', _registry(), c)
    assert r.accepted is False
    assert '未定义的参数' in r.reason


def test_out_of_range_arg_is_rejected():
    c = FakeClient('{"skill": "autonomous.advance_until_blocked", "args": '
                   '{"max_distance": 9.9, "clear_range": 0.5, "step": 0.1}}')
    r, _ = plan_with_llm('走一段', _registry(), c)
    assert r.accepted is False
    assert '上限' in r.reason


def test_bool_where_a_number_is_expected_is_rejected():
    c = FakeClient('{"skill": "autonomous.advance_until_blocked", "args": '
                   '{"max_distance": true, "clear_range": 0.5, "step": 0.1}}')
    r, _ = plan_with_llm('走一段', _registry(), c)
    assert r.accepted is False


# ==========================================================================
# 组合：规则优先；规则**自己不合法**时不许甩给 LLM
# ==========================================================================

TASK_RULE = {'走一小段': {'skill': 'autonomous.advance_until_blocked', 'args': GOOD_ARGS}}


def test_rule_hit_wins_and_the_llm_is_never_asked():
    """规则表是**确定、离线、快**的那一跳；命中就不该花一次网络往返。"""
    c = FakeClient(GOOD_REPLY)
    out = plan_task('走一小段', TASK_RULE, _registry(), c)
    assert out.result.accepted is True
    assert out.source == SOURCE_RULES
    assert c.calls == []


def test_rule_hit_but_invalid_does_not_fall_through_to_the_llm():
    """★★ 规则表里写了 `control.*` —— 那是**配置错误**，必须停在规则表这一跳。

    甩给 LLM"顺手修一下"会把错误盖住，而红线恰恰是需要被人看见的那类错误。
    """
    bad = {'偷偷动一下': {'skill': 'control.move_relative', 'args': {'x': 0.3, 'y': 0}}}
    c = FakeClient(GOOD_REPLY)
    out = plan_task('偷偷动一下', bad, _registry(), c)
    assert out.result.accepted is False
    assert '架构红线' in out.result.reason
    assert out.source == SOURCE_RULES
    assert c.calls == []                       # ★ 一次都没问


def test_empty_text_does_not_burn_an_llm_call():
    c = FakeClient(GOOD_REPLY)
    out = plan_task('   ', TASK_RULE, _registry(), c)
    assert out.result.accepted is False
    assert c.calls == []


# ---------- 没开 / 不可用 / 调用失败：三种理由分开说 ----------

def test_llm_disabled_says_so_instead_of_pretending_not_to_understand():
    """★ 措辞必须与事实一致：**没开** 不是 **不会**。"""
    out = plan_task('去桌子旁边找杯子', {}, _registry(), None)
    assert out.result.accepted is False
    assert REFUSE_LLM_DISABLED in out.result.reason
    assert out.source == SOURCE_NONE


def test_llm_unavailable_is_distinguished_from_a_failed_call():
    c = FakeClient(error=LlmUnavailable('没有读到 API key（检查那个环境变量）'))
    out = plan_task('找杯子', {}, _registry(), c)
    assert out.result.accepted is False
    assert '不可用' in out.result.reason and 'API key' in out.result.reason
    assert out.source == SOURCE_NONE


def test_transport_failure_reports_the_timeout_ceiling():
    c = FakeClient(error=LlmTransportError('超时（8s）'), timeout_s=8.0)
    out = plan_task('找杯子', {}, _registry(), c)
    assert out.result.accepted is False
    assert '调用失败' in out.result.reason and '8' in out.result.reason
    assert out.source == SOURCE_LLM


def test_unparsable_reply_is_reported_and_the_raw_text_is_kept():
    c = FakeClient('我觉得可以先往前走一点点')
    out = plan_task('找杯子', {}, _registry(), c)
    assert out.result.accepted is False
    assert '无法解析' in out.result.reason
    assert out.raw == '我觉得可以先往前走一点点'      # ★ 供日志审计


def test_llm_refusal_is_relayed_verbatim():
    c = FakeClient('{"refuse": "机器人现在没有找杯子的能力"}')
    out = plan_task('找杯子', {}, _registry(), c)
    assert out.result.accepted is False
    assert '机器人现在没有找杯子的能力' in out.result.reason


# ==========================================================================
# 单飞闸门：占不到就**拒绝**，不排队
# ==========================================================================

def test_busy_guard_refuses_instead_of_queueing():
    """★ 并发 submit 各带一次几秒的 HTTP 会把线程池占满 ——
    那时 `on_event`（唤醒）与 `sweep`（超时兜底）就排不上队了。
    拒绝而不是排队，与 D-026 决策 2 同一条理由。"""
    import threading

    guard = threading.Lock()
    guard.acquire()                       # 假装"上一次规划还在跑"
    try:
        out = plan_task('找杯子', {}, _registry(), FakeClient(GOOD_REPLY), guard=guard)
    finally:
        guard.release()
    assert out.result.accepted is False
    assert '不排队' in out.result.reason
    assert out.source == SOURCE_NONE


def test_busy_guard_is_not_touched_when_a_rule_hits():
    """⚠️ 规则命中是 O(1) 纯查表，**不该**被一次网络调用连累。"""
    import threading

    guard = threading.Lock()
    guard.acquire()
    try:
        out = plan_task('走一小段', TASK_RULE, _registry(), FakeClient(GOOD_REPLY),
                        guard=guard)
    finally:
        guard.release()
    assert out.result.accepted is True
    assert out.source == SOURCE_RULES


def test_guard_is_released_after_a_successful_plan():
    import threading

    guard = threading.Lock()
    assert plan_task('找杯子', {}, _registry(), FakeClient(GOOD_REPLY),
                     guard=guard).result.accepted is True
    assert guard.acquire(blocking=False) is True       # 已经放开了
    guard.release()


def test_guard_is_released_even_when_the_call_fails():
    import threading

    guard = threading.Lock()
    plan_task('找杯子', {}, _registry(),
              FakeClient(error=LlmTransportError('超时（8s）')), guard=guard)
    assert guard.acquire(blocking=False) is True
    guard.release()


# ==========================================================================
# ★★ 总不变量
# ==========================================================================

def test_no_path_can_accept_a_non_task_skill():
    """把上面所有"可能的模型回话"合起来看：

    **无论模型返回什么，被接受的计划都只能是 task-tier 技能。**
    这条不变量才是"架构红线不是靠自觉"的真正含义 ——
    它不是某一条分支的检查，而是**这个组合函数产不出那种东西**。
    """
    reg = _registry()
    replies = [
        GOOD_REPLY,
        '```json\n' + GOOD_REPLY + '\n```',
        '{"skill": "control.move_relative", "args": {"x": 0.3, "y": 0}}',
        '{"skill": "control.rotate", "args": {"angle": 1.0}}',
        '{"skill": "autonomous.broken", "args": {}}',
        '{"skill": "does.not.exist", "args": {}}',
        '{"refuse": "做不到"}',
        '完全不按格式说',
        '',
    ]
    for reply in replies:
        out = plan_task('随便一句话', {}, reg, FakeClient(reply))
        if out.result.accepted:
            assert all(reg.require(s.skill).tier == 'task'
                       for s in out.result.steps), reply


def test_accepted_plan_is_always_dispatchable_by_the_gateway_principal():
    """被接受的技能必须真的允许 `agent.planner` 调 —— 否则会在网关那层才被拒。"""
    out = plan_task('往前走一小段', {}, _registry(), FakeClient(GOOD_REPLY))
    assert out.result.accepted is True
    assert all('agent.planner' in _registry().require(s.skill).allowed_principals
               for s in out.result.steps)


# ==========================================================================
# ★ 多步计划（D-043）
# ==========================================================================

MULTI_REPLY = ('{"steps": ['
               '{"skill": "autonomous.advance_until_blocked", '
               '"args": {"max_distance": 0.2, "clear_range": 0.5, "step": 0.1}}, '
               '{"skill": "autonomous.turn_until_clear", '
               '"args": {"max_angle": 1.0, "clear_range": 0.5, "step_angle": 0.3, '
               '"direction": 1.0}}]}')


def test_parse_multi_step_reply():
    kind, steps = parse_reply(MULTI_REPLY)
    assert kind == 'steps'
    assert [s['skill'] for s in steps] == ['autonomous.advance_until_blocked',
                                           'autonomous.turn_until_clear']


def test_empty_steps_is_a_parse_error():
    """空计划没有意义 —— 要么给步骤，要么用 refuse 明确拒绝。"""
    with pytest.raises(ReplyError, match='空数组'):
        parse_reply('{"steps": []}')


def test_steps_must_be_an_array():
    with pytest.raises(ReplyError, match='必须是数组'):
        parse_reply('{"steps": {"skill": "x"}}')


def test_a_step_without_a_skill_name_is_a_parse_error():
    with pytest.raises(ReplyError, match='第 1 步'):
        parse_reply('{"steps": [{"args": {}}]}')


def test_llm_multi_step_plan_is_accepted_whole():
    r, _raw = plan_with_llm('往前走，被挡就转个方向', _registry(), FakeClient(MULTI_REPLY))
    assert r.accepted is True
    assert len(r.steps) == 2


def test_llm_multi_step_with_a_red_line_anywhere_is_rejected_whole():
    """★ 多步**不是**绕过红线的办法：任何一步踩线，整条被拒，且**一个步骤都不执行**。

    这条要钉死 —— 否则模型很容易想"第 1 步合法、把越权的塞到第 2 步"。
    """
    reply = ('{"steps": ['
             '{"skill": "autonomous.advance_until_blocked", '
             '"args": {"max_distance": 0.2, "clear_range": 0.5, "step": 0.1}}, '
             '{"skill": "control.move_relative", "args": {"x": 0.1}}]}')
    r, _raw = plan_with_llm('随便', _registry(), FakeClient(reply))
    assert r.accepted is False
    assert r.steps == []
    assert '第 2 步' in r.reason
    assert '架构红线' in r.reason
