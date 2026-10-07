#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 的规划（**纯 Python，不依赖 ROS**）—— 第一版是**规则表**，不是 LLM。

`plan.md` §11 的核心循环是 `Understand → Plan → Select Skill → Execute → Observe → Re-plan`。
本模块只负责 `Plan` 那一格，而且刻意做得**极其保守**：

> **默认什么都不做。** 规则表为空时，任何自然语言任务都得到
> 「需要 LLM 规划，当前未实现（Phase 7）」—— **这是刻意的，不是缺陷。**

规则表是 Phase 7 那张 Rule Engine 的占位：它让"查表 → 调技能"这条**链路**今天就能被
端到端跑通与验证，而不必先有一个 LLM。等 LLM 接进来时，它替换的是**这张表的内容**，
不是这条链路。

🔒 架构红线（D-003 / D-029）：这一层**只能**命名 **task-tier** 技能
--------------------------------------------------------------------
Agent 不得触达 Control Skill。这条在别处只是文档；在这里它是**代码级校验**：

    规则表里写了 `control.move_relative` → **拒绝**，并且说明原因。

为什么值得专门写一段代码：规则表是**数据**，将来是 LLM 填的。
如果这层不校验，一条"顺手"的规则就能让 Agent 绕过 D-003，
而那时候**没有任何地方会报错** —— 它只会安静地工作，直到出事。
"""

from collections import namedtuple

REFUSE_NO_RULE = '没有匹配的规则 —— 需要 LLM 规划，当前未实现（Phase 7）'

# 规划结果：是否接受、要调哪个技能、参数、以及**拒绝原因**（人要看到的就是这句）
PlanResult = namedtuple('PlanResult', 'accepted skill args reason')

_PUNCT = ' \t\r\n，。！？、；：""\'\'（）()[]{}.,!?;:'


def normalize(text):
    """归一化，用于和规则表的键做**整句**匹配。

    ⚠️ 与 `estop.normalize` 长得像但用途不同：这个是**任务文本查表**用的，
    不承担任何安全语义。安全词判定仍然只有 `estop.py` 那一处（不许有第二份）。
    """
    if text is None:
        return ''
    s = str(text).strip().lower()
    for ch in _PUNCT:
        s = s.replace(ch, '')
    return s


def plan(text, rules, registry):
    """把一句自然语言任务映射成一个**技能调用**。

    :param rules: `{归一化后的文本: {'skill': '...', 'args': {...}}}`
    :param registry: 网关的 `Registry` —— 用它校验技能名与层级，
                     保证"能命名的技能"与"注册在册的技能"**是同一份事实**
    :return: `PlanResult`
    """
    s = normalize(text)
    if not s:
        return PlanResult(False, None, {}, '空文本，没有要规划的东西')

    rule = (rules or {}).get(s)
    if rule is None:
        return PlanResult(False, None, {}, REFUSE_NO_RULE)

    skill = rule.get('skill')
    if not skill:
        return PlanResult(False, None, {}, f'规则表里这条没有写 skill：{rule!r}')

    spec = registry.get(skill)
    if spec is None:
        # 规则表与注册表**漂移**了。这必须报错：否则会表现为"技能调不通"，
        # 而真正的原因是名字写错，排查方向会跑偏。
        return PlanResult(False, None, {},
                          f'规则表指向了未注册的技能 {skill!r}'
                          f'（已注册：{list(registry.names())}）')

    if spec.tier != 'task':
        # ★★ 架构红线（D-003）：Agent 只能命名 task-tier 技能。
        return PlanResult(
            False, None, {},
            f'规则表试图让 Agent 调用 {skill}（tier={spec.tier}）—— '
            f'架构红线：Agent 只能命名 task-tier（Autonomous / Semantic）技能，'
            f'不得触达 {spec.tier} 层（D-003 / D-029）')

    if not spec.available:
        why = f'：{spec.unavailable_reason}' if spec.unavailable_reason else ''
        return PlanResult(False, None, {}, f'技能 {skill} 当前不可用{why}')

    return PlanResult(True, skill, dict(rule.get('args') or {}), '')
