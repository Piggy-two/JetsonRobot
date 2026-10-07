#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 的规划（**纯 Python，不依赖 ROS**）—— 本模块是其中的 **Rule Engine**。

`plan.md` §18 定的形状是 `Rule Engine → Local Small LLM → Cloud LLM`。
本模块是**第一跳**：一张**整句精确匹配**的规则表，离线、确定、快。
命中不了才轮到 LLM（`llm_planner.py`，D-038）。

🔒 架构红线（D-003 / D-029）：这一层**只能**命名 **task-tier** 技能
--------------------------------------------------------------------
Agent 不得触达 Control Skill。这条在别处只是文档；在这里它是**代码级校验**：

    规则表里写了 `control.move_relative` → **拒绝**，并且说明原因。

为什么值得专门写一段代码：**规则表是数据，而且现在 LLM 也会往这条路上递东西**。
如果这层不校验，一条"顺手"的规则（或模型的一次幻觉）就能让 Agent 绕过 D-003，
而那时候**没有任何地方会报错** —— 它只会安静地工作，直到出事。

⚠️ 因此校验**只写一处**：`accept_skill(...)`。规则表与 LLM 两条路都从它过，
`source` 只影响措辞（「规则表试图…」/「LLM 试图…」），不影响判据。
**两条路不可能有不一样的严格度** —— 这是刻意的结构，不是巧合。
"""

from collections import namedtuple

from embodied_skill_gateway import checks

#: 规则表没命中。⚠️ 措辞里**不再**写"Phase 7 未实现" —— LLM 已经接上了，
#: 现在"没命中"只说明规则表里没有这一条；能不能做由 LLM 那一跳回答（D-038）。
REFUSE_NO_RULE = '规则表里没有匹配的条目'

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


def accept_skill(skill, args, registry, source):
    """**两条规划路径共用的唯一校验口。**

    顺序是刻意的：先"这个技能认不认得"，再"它是不是这一层能碰的"，
    然后"它现在能不能用"，最后才是参数。红线排在参数之前 ——
    一个越界的 `control.*` 请求应该被当成**越权**拒绝，而不是被当成**参数写错了**。

    :param source: 只用于措辞（例如 `'规则表'` / `'LLM'`）。**不参与任何判断。**
    """
    if not skill:
        return PlanResult(False, None, {}, f'{source}没有给出技能名')

    spec = registry.get(skill)
    if spec is None:
        # 与注册表**漂移**了。这必须报错：否则会表现为"技能调不通"，
        # 而真正的原因是名字写错（或模型编了一个），排查方向会跑偏。
        return PlanResult(False, None, {},
                          f'{source}指向了未注册的技能 {skill!r}'
                          f'（已注册：{list(registry.names())}）')

    if spec.tier != 'task':
        # ★★ 架构红线（D-003 / D-029）：Agent 只能命名 task-tier 技能。
        return PlanResult(
            False, None, {},
            f'{source}试图让 Agent 调用 {skill}（tier={spec.tier}）—— '
            f'架构红线：Agent 只能命名 task-tier（Autonomous / Semantic）技能，'
            f'不得触达 {spec.tier} 层（D-003 / D-029）')

    if not spec.available:
        why = f'：{spec.unavailable_reason}' if spec.unavailable_reason else ''
        return PlanResult(False, None, {}, f'技能 {skill} 当前不可用{why}')

    # 参数校验**复用网关那份代码**（`checks.py`），不另写一份：
    # 网关回头还会再查一遍（纵深防御），这里查是为了**当场**给出可读的理由，
    # 而不是让用户等到一次网关往返之后才被告知参数写错了。
    args = dict(args or {})
    why = checks.validate_schema(spec, args) or checks.validate_range(spec, args)
    if why:
        return PlanResult(False, None, {}, f'{source}给的参数不被接受：{why}')

    return PlanResult(True, skill, args, '')


def plan(text, rules, registry):
    """规则表那一跳：把一句自然语言任务映射成一个**技能调用**。

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

    if not isinstance(rule, dict) or not rule.get('skill'):
        return PlanResult(False, None, {}, f'规则表里这条没有写 skill：{rule!r}')

    return accept_skill(rule.get('skill'), rule.get('args'), registry, '规则表')
