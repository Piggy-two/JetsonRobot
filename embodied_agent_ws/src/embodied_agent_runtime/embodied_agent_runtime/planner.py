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
from embodied_skill_gateway import task_state as ts

#: 规则表没命中。⚠️ 措辞里**不再**写"Phase 7 未实现" —— LLM 已经接上了，
#: 现在"没命中"只说明规则表里没有这一条；能不能做由 LLM 那一跳回答（D-038）。
REFUSE_NO_RULE = '规则表里没有匹配的条目'

#: 一个步骤：技能名 + 参数。`args` 永远是 dict（哪怕是空的）。
Step = namedtuple('Step', 'skill args when', defaults=(None,))
#: 步骤上的**条件**（可省）。目前只支持一种：`when: {prev: <task-tier 终态>}`
#: —— "**只有上一步以这个终态结束时，才执行我**"。
#:
#: ⚠️ 为什么条件只说"上一步的终态"，而不是一个更花哨的表达式语言：
#:    · 它是**执行器手上唯一确凿的观察**（`last_step_state`）——
#:      别的（画面里有什么、车在哪）都要去查别的东西，而"查什么、怎么查"本身
#:      又是一层设计；
#:    · 这一条就够表达"看看有没有人，**有**就往前走"这类话了，而那是今天
#:      最缺的一句（规划层此前**明令禁止**写条件，见 `llm_planner.SYSTEM_PROMPT`）。
#: ⚠️ 第 1 步**不许**带条件 —— 它前面没有东西，条件无从判定（在 `accept_plan` 里拒）。
WHEN_KEY = 'prev'

#: 参数**绑定**：把"上一步结果里的某个字段"接到这一步的某个参数上。
#: 写法：`"参数名": {"from": "prev", "field": "side", "as": "opposite_sign"}`
#:
#: ⚠️ **为什么 `as` 要显式写**（而不是偷偷替人算）：这里有一个真实的符号陷阱 ——
#:   `semantic.look_for` 给的 `side` 是**画面里的左右**（左 = 负），
#:   而 `turn_until_clear` 的 `direction` 是 **+1 = 逆时针 = 左转**。
#:   所以"朝着看到的那一边转"要的是 **`opposite_sign`**；
#:   直接写 `sign` 会**转向相反的一侧**，而且**看起来完全正常**（车照转、日志全绿）。
#:   ⇒ 变换必须**写在纸面上**，不能藏在实现里 —— 写错了至少能被人一眼看见。
#:   （画面左右与机体左右一致，这件事由 D-023 的相机朝向实机校验背书：
#:     把物体放在车前方偏右，运动质心 513 次全在画面**右**侧、0 次在左。）
BINDING_FROM = 'prev'
#: 支持的变换。`value` = 原样传（比如把上一步的 `travelled` 当这一步的 `max_distance`）。
BINDING_OPS = ('value', 'sign', 'opposite_sign')

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


def is_binding(value):
    """这个参数值是不是一个**绑定**（而不是一个普通字面量）。"""
    return isinstance(value, dict) and 'from' in value


def check_binding(raw, index, source):
    """校验一个参数绑定的**形状**。返回错误说明（`''` = 合法）。

    ⚠️ 这里**只验形状，不验取值** —— 值要等上一步跑完才知道。
    取值那道关在**派发时**由网关照常把守（绑定**不绕过**准入）。
    """
    if raw.get('from') != BINDING_FROM:
        return (f'{source}第 {index} 步的绑定只支持 from: {BINDING_FROM!r}'
                f'（"上一步"）—— 得到 {raw.get("from")!r}。'
                f'指向更早的步骤会让"第几步"变成一个要人肉追的算术题')
    field = raw.get('field')
    if not isinstance(field, str) or not field.strip():
        return f'{source}第 {index} 步的绑定缺 field（要接上一步结果里的哪个字段）'
    op = raw.get('as', 'value')
    if op not in BINDING_OPS:
        return (f'{source}第 {index} 步的绑定 as 不认识：{op!r}（支持 {list(BINDING_OPS)}）')
    extra = sorted(k for k in raw if k not in ('from', 'field', 'as'))
    if extra:
        return (f'{source}第 {index} 步的绑定里有不认识的键 {extra} —— '
                f'不认识的键必须报错，不能当没写')
    return ''


def resolve_binding(binding, prev_result):
    """把绑定解成一个具体值。返回 `(值, 错误说明)`。

    :param prev_result: 上一步技能的 `result_json` 解出来的对象（dict）
    """
    if not isinstance(prev_result, dict):
        return None, f'上一步没有给出可用的结果（得到 {type(prev_result).__name__}）'
    field = binding['field']
    if field not in prev_result:
        # ★ 把**它实际给了什么**列出来 —— 与"类别不在表里"同一条纪律：
        #   只说"没有这个字段"，排查的人得自己去翻技能定义
        return None, (f'上一步的结果里没有 {field!r} —— 它给的是 '
                      f'{sorted(prev_result)}')
    raw = prev_result[field]
    op = binding.get('as', 'value')
    if op == 'value':
        return raw, ''
    try:
        num = float(raw)
    except (TypeError, ValueError):
        return None, (f'上一步的 {field} 不是数（{raw!r}），'
                      f'而 as={op!r} 要对它取符号 —— 不做猜测')
    if op == 'sign':
        return (1.0 if num > 0 else (-1.0 if num < 0 else 0.0)), ''
    return (-1.0 if num > 0 else (1.0 if num < 0 else 0.0)), ''      # opposite_sign


def _placeholder_for(spec, name):
    """给**绑定的**参数填一个占位值，好让规划期把"名字对不对、必填项齐不齐"验掉。

    ⚠️ 它**只用来过形状检查**：绑定的真实值要等上一步跑完才知道，
    所以取值校验**推迟到派发时**，由网关照常做 —— 绑定**不绕过**准入。
    """
    p = spec.param(name)
    if p is None:
        return None
    if p.type in ('float', 'int'):
        lo, hi = p.minimum, p.maximum
        if lo is not None and lo > 0:
            return lo
        if hi is not None and hi < 0:
            return hi
        return 0.0 if p.type == 'float' else 0
    if p.type == 'bool':
        return False
    return 'x'


def when_ok(step, prev_state):
    """这一步的**条件满不满足**。没有条件 = 满足。

    ⚠️ 这是条件语义的**唯一**一处实现 —— `accept_plan` 校验形状、执行器判定取值，
    两边都从这里读，免得"合法写法"与"实际判法"分家（那种分家的症状是
    **配置通过校验却永远不执行**）。
    """
    cond = getattr(step, 'when', None)
    if not cond:
        return True
    return cond.get(WHEN_KEY) == prev_state


def check_when(raw, index, source):
    """校验一步的 `when`。返回 `(when, 错误说明)`；没有条件时返回 `(None, '')`。

    ⚠️ 三条都要**明确拒绝**而不是忽略，理由是它们都会**静默改变行为**：
      · 第 1 步带条件 —— 它前面没有东西，条件永远无从判定；
      · 不认识的键（比如写了 `if:` / `unless:`）—— 用户以为加了限制，实际没有；
      · 不认识的状态名（比如写 `SUCCESS`）—— 条件永远不成立，那一步**永远不跑**。
    """
    if raw is None:
        if index == 1:
            return None, ''
        return None, ''
    if not isinstance(raw, dict):
        return None, f'{source}第 {index} 步的 when 必须是一个映射，得到 {raw!r}'
    if index == 1:
        return None, (f'{source}第 1 步不能带 when —— 它前面没有上一步，'
                      f'条件无从判定（要"无条件先做点什么"就把它放第 1 步）')
    extra = sorted(k for k in raw if k != WHEN_KEY)
    if extra:
        return None, (f'{source}第 {index} 步的 when 里有不认识的键 {extra} —— '
                      f'只支持 {WHEN_KEY!r}（"上一步的终态"）；'
                      f'不认识的键必须报错，不能当没写')
    want = raw.get(WHEN_KEY)
    if want not in ts.TASK_TERMINAL:
        return None, (f'{source}第 {index} 步的 when.{WHEN_KEY} 不是 task-tier 终态：'
                      f'{want!r}（合法值：{list(ts.TASK_TERMINAL)}）—— '
                      f'写错的话这一步**永远不会执行**，而且不会报错')
    return {WHEN_KEY: want}, ''


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
        args = raw.get('args')
        bound = {}
        if isinstance(args, dict):
            for k, v in args.items():
                if not is_binding(v):
                    continue
                why = check_binding(v, i, source)
                if why:
                    return PlanResult(False, [], f'{why}　⟹　整条计划被拒（不做半条）')
                if i == 1:
                    return PlanResult(False, [], (
                        f'{source}第 1 步不能有参数绑定 —— 它前面没有上一步，'
                        f'没有东西可接（要"先做点什么再看"就把它放第 1 步）'
                        f'　⟹　整条计划被拒（不做半条）'))
                bound[k] = v
        if bound:
            # 占位值只为了让下面的参数校验跑得过（名字/必填/类型形状）；
            # 真值在派发时替换，并由**网关**照常校验。
            spec = registry.get(raw.get('skill'))
            args = dict(args)
            for k, v in bound.items():
                ph = _placeholder_for(spec, k) if spec is not None else None
                if ph is None:
                    return PlanResult(False, [], (
                        f'{source}第 {i} 步把绑定接到了不存在的参数 {k!r} 上'
                        f'　⟹　整条计划被拒（不做半条）'))
                args[k] = ph
        r = accept_skill(raw.get('skill'), args, registry, f'{source}第 {i} 步')
        if not r.accepted:
            # 把"是第几步"带上 —— 否则模型/规则表写错时看不出错在哪一步
            return PlanResult(False, [], f'{r.reason}　⟹　整条计划被拒（不做半条）')
        when, why = check_when(raw.get('when'), i, source)
        if why:
            return PlanResult(False, [], f'{why}　⟹　整条计划被拒（不做半条）')
        st = r.steps[0]
        real = dict(st.args)
        real.update(bound)          # 把占位值换回**绑定本身**
        out.append(Step(st.skill, real, when))
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
        w = getattr(s, 'when', None)
        cond = tuple(sorted((str(k), repr(v)) for k, v in w.items())) if w else ()
        out.append((getattr(s, 'skill', str(s)), args, cond))
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
        w = getattr(s, 'when', None)
        tail = f' 〔仅当上一步是 {w[WHEN_KEY]}〕' if w else ''
        out.append(f'{getattr(s, "skill", s)}({args}){tail}')
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
