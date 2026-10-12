#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`llm_replan_style_probe` 的离线单测 —— **不碰网络、不碰 ROS、不动车**。

这份单测只覆盖两件事，而它们恰好是探针的**全部判断力**：

1. **那次重试算哪一种**（`classify_retry`）—— 探针测出来的"分布"完全由它决定，
   它错一个标签，整份结论就是错的。
2. **没接上就拒测**（`main` 的守卫）—— 一次"没有密钥却发出去"的调用
   既浪费又难看；而"没有密钥却说'模型不会'"更是**把两种完全不同的失败混成一句话**
   （D-038 的老毛病）。

    python3 -m pytest tools/test_llm_replan_style_probe.py -q
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import llm_replan_style_probe as probe    # noqa: E402


class Step:
    """计划里的一步。只要 `skill` / `args` / `when` 三个字段 —— 与真步骤同形。"""

    def __init__(self, skill, args=None, when=None):
        self.skill, self.args, self.when = skill, args or {}, when

    def __repr__(self):
        return f'Step({self.skill}, {self.args})'


ADV = 'autonomous.advance_until_blocked'
TURN = 'autonomous.turn_until_clear'


def adv(d=0.5, step=0.3):
    return Step(ADV, {'max_distance': d, 'step': step})


def turn(direction):
    return Step(TURN, {'direction': direction})


# --------------------------------------------------------------------------
# shape：技能序列 + 参数的**符号**
# --------------------------------------------------------------------------

def test_shape_keeps_skill_order():
    assert probe.skill_sequence([adv(), turn(1.0)]) == (ADV, TURN)
    assert probe.skill_sequence([adv(), turn(1.0)]) != \
        probe.skill_sequence([turn(1.0), adv()])


def test_shape_ignores_magnitude_but_keeps_sign():
    """★ 这就是 C 判据的全部内容：**幅度不算，方向算**。"""
    assert probe.shape([adv(0.5)]) == probe.shape([adv(0.15)])
    assert probe.shape([turn(1.0)]) != probe.shape([turn(-1.0)])


def test_shape_treats_zero_as_its_own_sign():
    """⚠️ 不许用 `copysign`：那是 `copysign(1, 0.0) == +1`，
    会把"原地不动"和"往前"混成一类 —— 而这两件事在重规划里正相反。"""
    assert probe.shape([adv(0.0)]) != probe.shape([adv(1.0)])
    assert probe.shape([adv(0.0)]) == probe.shape([adv(0.0)])


def test_shape_is_a_list_of_plain_pairs():
    got = probe.shape([turn(-1.0)])
    assert got == ((TURN, (('direction', -1),)),)


# --------------------------------------------------------------------------
# classify_retry：探针的判词
# --------------------------------------------------------------------------

def test_exact_repeat_is_labelled_first():
    assert probe.classify_retry([adv(0.5)], [adv(0.5)]).startswith('①')


def test_smaller_params_is_the_fake_switch():
    """D-045 实测到的那个模式 —— 这也正是这条取舍要挡的东西。"""
    got = probe.classify_retry([adv(0.5, 0.3)], [adv(0.15, 0.1)])
    assert got.startswith('②')


def test_mirror_is_not_a_repeat():
    """★ 换另一边绕：技能序列一样、幅度一样，**只有方向符号变了**。

    D-044 说那是"**是**换了走法"。B 判据会把它误伤；C 不会。
    """
    plan_a = [turn(1.0), adv(0.3)]
    plan_b = [turn(-1.0), adv(0.3)]
    assert probe.classify_retry(plan_a, plan_b).startswith('③')


def test_prepending_a_skill_is_a_real_switch():
    got = probe.classify_retry([adv(0.3)], [turn(1.0), adv(0.3)])
    assert got.startswith('④')


def test_a_wholly_different_plan_is_the_same_label():
    """⚠️ 刻意与上一条**同一个标签**：这两种在这次取舍里的代价完全一样
    （都该放行），细分只会让分布表更难读。第一版分过，被单测挡下了。"""
    assert probe.classify_retry([adv(0.3)], [turn(1.0)]).startswith('④')


def test_refusal_has_its_own_label():
    """⚠️ "模型如实说做不了"**不是**重试的一种 —— 它得单独看得见，
    否则会被读成"它又给了一条一样的"。"""
    assert '拒绝' in probe.classify_retry([adv(0.3)], [])


def test_labels_are_mutually_exclusive_on_the_known_cases():
    """一个标签一个代表 —— 五个用例必须给出**五个不同**的标签。"""
    cases = [
        ([adv(0.5)], [adv(0.5)]),                          # ①
        ([adv(0.5)], [adv(0.15)]),                         # ②
        ([turn(1.0), adv(0.3)], [turn(-1.0), adv(0.3)]),   # ③
        ([adv(0.3)], [turn(1.0)]),                         # ④
        ([adv(0.3)], []),                                  # 拒绝
    ]
    tags = [probe.classify_retry(a, b)[:1] for a, b in cases]
    assert len(set(tags)) == len(tags), f'标签撞了：{tags}'


# --------------------------------------------------------------------------
# verdict_of：把分布翻成"这一次取舍的代价"
# --------------------------------------------------------------------------

def test_verdict_names_all_three_rules():
    labels = {probe.classify_retry([adv(0.5)], [adv(0.15)]): 7,
              probe.classify_retry([turn(1.0)], [turn(-1.0)]): 3}
    text = probe.verdict_of(labels)
    for tag in ('A 现状', 'B 只看序列', 'C 序列+符号'):
        assert tag in text
    assert '7' in text and '3' in text          # 计数要真的出现在里面


def test_verdict_does_not_divide_by_zero():
    assert probe.verdict_of({}) == '（没有样本）'


# --------------------------------------------------------------------------
# main 的守卫：没接上就拒测，而且说得清是"没接上"
# --------------------------------------------------------------------------

def _argv(**over):
    args = ['probe', '--base-url', 'https://example.invalid/v1', '--model', 'x',
            '--api-key-env', 'PROBE_TEST_KEY_NOT_SET']
    for k, v in over.items():
        args += [f'--{k.replace("_", "-")}', v]
    return args


def test_refuses_without_endpoint_or_model(monkeypatch, capsys):
    monkeypatch.setattr(sys, 'argv', ['probe'])
    assert probe.main() == 4
    assert '必须显式给出' in capsys.readouterr().out


def test_refuses_without_a_key_and_says_so(monkeypatch, capsys):
    monkeypatch.delenv('PROBE_TEST_KEY_NOT_SET', raising=False)
    monkeypatch.setattr(sys, 'argv', _argv())
    assert probe.main() == 4
    out = capsys.readouterr().out
    assert '没有密钥' in out
    # ⚠️ 不许出现"DEEPSEEK_API_KEY 的值是 …"这类回显
    assert 'sk-' not in out


def test_refuses_on_a_missing_registry(monkeypatch, capsys):
    monkeypatch.setenv('PROBE_TEST_KEY_NOT_SET', 'not-a-real-key')
    monkeypatch.setattr(sys, 'argv', _argv(registry='/nope/none.yaml'))
    assert probe.main() == 4
    assert '读不到技能注册表' in capsys.readouterr().out


# --------------------------------------------------------------------------
# 种子计划必须**过得了真的注册表**
# --------------------------------------------------------------------------

def test_seed_plan_is_accepted_by_the_real_registry():
    """★ 这条是**从一次真实拒测里长出来的**（2026-10-12）。

    第一版种子只给了 `max_distance`，探针一跑就拒测：
    "缺少必填参数 ['clear_range', 'step']" —— 而那要等到**真的跑起来**才知道。
    ⇒ 把种子钉在真注册表上，这类错在离线就该现形。

    ⚠️ 它同时钉住另一件事：**种子是一次"真的会失败"的尝试** ——
    如果哪天这个技能被改名或参数表变了，这里会先响。
    """
    from embodied_agent_runtime import planner
    from embodied_skill_gateway.registry import Registry

    reg = Registry.from_yaml(str(probe.DEFAULT_REGISTRY))
    res = planner.accept_plan(probe.SEED_PLAN, reg, 'test')
    assert res.accepted, f'种子计划过不了注册表：{res.reason}'
    assert res.steps, '种子计划解析出来是空的'
    assert res.steps[0].skill == 'autonomous.advance_until_blocked'


def test_default_scenario_is_not_empty():
    assert probe.DEFAULT_TEXT.strip()


# --------------------------------------------------------------------------
# token 预算必须与运行时**同源**
# --------------------------------------------------------------------------

_RUNTIME_CFG = (Path(probe.__file__).resolve().parents[1] / 'embodied_agent_ws'
                / 'src' / 'embodied_agent_runtime' / 'config' / 'agent_runtime.yaml')


def test_probe_max_tokens_matches_the_runtime_default():
    """★ 探针的 token 预算必须与运行时的 `llm_max_tokens` **一致**。

    ⚠️ 不一致的话，这里量到的**截断率**就不是运行时那个截断率 ——
    而"10 次里 2 次被截断"正是 D-052 把默认值提到 8192 的**唯一依据**。
    判据两边各写一个数，迟早分家（**D-040 / 坑 27** 的老毛病）。
    """
    import yaml

    params = yaml.safe_load(_RUNTIME_CFG.read_text())['agent_runtime']['ros__parameters']
    runtime = params['llm_max_tokens']
    probe_default = probe.build_parser().get_default('max_tokens')
    assert probe_default == runtime, (
        f'探针默认 --max-tokens={probe_default} 与运行时 llm_max_tokens={runtime} 不一致 '
        f'—— 那会让探针量到的截断率不是运行时那个')


def test_defaults_are_readable_from_the_parser():
    """⚠️ 默认值要能被单测读到，所以参数表必须留一个 `build_parser()` 出口。"""
    ap = probe.build_parser()
    assert ap.get_default('phase') in ('ask', 'chain')
    assert ap.get_default('api_key_env') == 'DEEPSEEK_API_KEY'
