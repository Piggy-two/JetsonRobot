#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Agent 的规划（**纯 Python，不依赖 ROS**）—— 本模块是其中的 **Rule Engine**。

`plan.md` §18 定的形状是 `Rule Engine → Local Small LLM → Cloud LLM`。
本模块是**第一跳**：一张**整句精确匹配**的规则表，离线、确定、快。
命中不了才轮到 LLM（`llm_planner.py`，D-038）。

**多步计划**（2026-10-08，D-043）
----------------------------------
D-038 当时刻意只做单步，理由是「注册表里只有一个 task-tier 技能，多步无从编排」。
现在有了第二个（`turn_until_clear`），多步才有意义 —— 最小的脱困行为就是
`advance_until_blocked → turn_until_clear → advance_until_blocked`。

一个**计划**是一串**有序步骤**，每一步都是「技能名 + 参数」。

🔒 **每一步都过同一个校验口**（`accept_skill`）
------------------------------------------------
多步**没有**开出任何新的旁路：计划里的每一步都被逐条校验，
而且**整个计划要么全合法、要么整条被拒**。见 `accept_plan` 里为什么不做"半条"。

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

#: 一个步骤：技能名 + 参数。`args` 永远是 dict（哪怕是空的）。
Step = namedtuple('Step', 'skill args')

# 规划结果：是否接受、**步骤列表**、以及拒绝原因（人要看到的就是这句）。
# ⚠️ 失败时 `steps` 为空 —— 部分计划没有意义（见 `accept_plan`）。
PlanResult = namedtuple('PlanResult', 'accepted steps reason')

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
    """**两条规划路径共用的唯一校验口**（对**一个**步骤）。

    顺序是刻意的：先"这个技能认不认得"，再"它是不是这一层能碰的"，
    然后"它现在能不能用"，最后才是参数。红线排在参数之前 ——
    一个越界的 `control.*` 请求应该被当成**越权**拒绝，而不是被当成**参数写错了**。

    :param source: 只用于措辞（例如 `'规则表'` / `'LLM'`）。**不参与任何判断。**
    """
    if not skill:
        return PlanResult(False, [], f'{source}没有给出技能名')

    spec = registry.get(skill)
    if spec is None:
        # 与注册表**漂移**了。这必须报错：否则会表现为"技能调不通"，
        # 而真正的原因是名字写错（或模型编了一个），排查方向会跑偏。
        return PlanResult(False, [], f'{source}指向了未注册的技能 {skill!r}'
                          f'（已注册：{list(registry.names())}）')

    if spec.tier != 'task':
        # ★★ 架构红线（D-003 / D-029）：Agent 只能命名 task-tier 技能。
        return PlanResult(
            False, [],
            f'{source}试图让 Agent 调用 {skill}（tier={spec.tier}）—— '
            f'架构红线：Agent 只能命名 task-tier（Autonomous / Semantic）技能，'
            f'不得触达 {spec.tier} 层（D-003 / D-029）')

    if not spec.available:
        why = f'：{spec.unavailable_reason}' if spec.unavailable_reason else ''
        return PlanResult(False, [], f'技能 {skill} 当前不可用{why}')

    # 参数校验**复用网关那份代码**（`checks.py`），不另写一份：
    # 网关回头还会再查一遍（纵深防御），这里查是为了**当场**给出可读的理由，
    # 而不是让用户等到一次网关往返之后才被告知参数写错了。
    args = dict(args or {})
    why = checks.validate_schema(spec, args) or checks.validate_range(spec, args)
    if why:
        return PlanResult(False, [], f'{source}给的参数不被接受：{why}')

    return PlanResult(True, [Step(skill, args)], '')


def accept_plan(steps, registry, source):
    """逐条校验一个计划的**每一步**。任何一步不合法 ⇒ **整条计划被拒**。

    ⚠️ **为什么不做"半条"**：如果第 1 步能做、第 2 步踩红线，那么
    "先做第 1 步、做到一半再报错" 会把车留在一个**半执行完的状态**，
    而计划本身已经作废 —— 没人知道该继续还是该退回去。
    ⇒ **先验完再动手**：动之前就知道这条计划整体合法。

    这也是 `_load_rules` 那种"写错必须炸"的同一条原则，只是搬到了运行期。

    :param steps: `[{'skill': ..., 'args': {...}}, ...]`
    """
    if not steps:
        return PlanResult(False, [], f'{source}没有给出任何步骤')

    out = []
    for i, raw in enumerate(steps, 1):
        if not isinstance(raw, dict):
            return PlanResult(False, [], f'{source}第 {i} 步不是一条技能条目：{raw!r}')
        r = accept_skill(raw.get('skill'), raw.get('args'), registry, f'{source}第 {i} 步')
        if not r.accepted:
            # 把"是第几步"带上 —— 否则模型/规则表写错时看不出错在哪一步
            return PlanResult(False, [], f'{r.reason}　⟹　整条计划被拒（不做半条）')
        out.append(r.steps[0])
    return PlanResult(True, out, '')


def _steps_from_rule(rule):
    """规则表里一条规则 → 步骤列表。**两种写法都支持**：

        skill: autonomous.xxx          # 单步（原来的写法，保持兼容）
        args: {...}

        steps:                         # 多步
          - {skill: ..., args: {...}}
          - {skill: ..., args: {...}}

    ⚠️ 两种**不许同时写** —— 那说明写的人自己也没想清要走几步。
    """
    if not isinstance(rule, dict):
        return None, f'规则表里这条不是一个映射：{rule!r}'
    has_steps, has_skill = 'steps' in rule, 'skill' in rule
    if has_steps and has_skill:
        return None, '规则表里这条同时写了 skill 和 steps —— 到底走几步不明确，拒绝'
    if has_steps:
        if not isinstance(rule['steps'], list):
            return None, f'steps 必须是一个列表：{rule["steps"]!r}'
        return rule['steps'], ''
    if has_skill:
        return [{'skill': rule.get('skill'), 'args': rule.get('args')}], ''
    return None, f'规则表里这条既没有 skill 也没有 steps：{rule!r}'


def plan_key(steps):
    """把一个计划压成可比较的键：`((技能, 参数元组), ...)`。

    参数按**键排序**再转元组 —— 否则 `{a:1,b:2}` 与 `{b:2,a:1}` 会被当成两个计划，
    而它们要机器人做的事**一模一样**。
    """
    out = []
    for s in steps or []:
        try:
            args = tuple(sorted((str(k), repr(v)) for k, v in (s.args or {}).items()))
        except AttributeError:
            args = ()
        out.append((getattr(s, 'skill', str(s)), args))
    return tuple(out)


def same_plan(a, b):
    """两个计划是不是**同一件事**（技能序列与参数都一样）。

    ⚠️ 这条不是优化，是**防死循环的结构件**：
    重规划若拿同一段文本问同一个规划器，**很可能得到一模一样的计划** ——
    那就会"计划 → 失败 → 再规划 → 同一个计划"无限转下去。
    ⇒ 调用方必须用它在派发前挡一道，见 `agent_runtime` 的 `_maybe_replan`。
    """
    return plan_key(a) == plan_key(b)


def render_plan(steps):
    """把一个计划画成**给模型看**的一行：`技能(参数=值) → 技能(参数=值)`。

    ⚠️ 与 `agent_runtime._plan_label` 长得像但**给的人不同**：那个是日志标签
    （只列技能名，够人扫一眼）；这个要进**提示词**，所以带上参数 ——
    不带参数的话，"换个走法"在模型眼里就没有可换的东西（它只会原样重来一遍）。

    参数按**名字排序**，好让两次渲染可逐字比较（模型会照着上一行的样子回）。
    """
    out = []
    for s in steps or []:
        try:
            items = sorted((str(k), v) for k, v in (s.args or {}).items())
        except AttributeError:
            items = []
        args = ', '.join(f'{k}={v!r}' for k, v in items)
        out.append(f'{getattr(s, "skill", s)}({args})')
    return ' → '.join(out)


def plan(text, rules, registry):
    """规则表那一跳：把一句自然语言任务映射成一个**计划**。

    :param rules: `{归一化后的文本: 规则}`，规则见 `_steps_from_rule`
    :param registry: 网关的 `Registry` —— 用它校验技能名与层级，
                     保证"能命名的技能"与"注册在册的技能"**是同一份事实**
    :return: `PlanResult`
    """
    s = normalize(text)
    if not s:
        return PlanResult(False, [], '空文本，没有要规划的东西')

    rule = (rules or {}).get(s)
    if rule is None:
        return PlanResult(False, [], REFUSE_NO_RULE)

    steps, why = _steps_from_rule(rule)
    if steps is None:
        return PlanResult(False, [], why)
    return accept_plan(steps, registry, '规则表')
