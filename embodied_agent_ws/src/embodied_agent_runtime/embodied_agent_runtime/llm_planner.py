#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""规则的**下一跳**：让云端 LLM 选一个技能（**纯 Python，不依赖 ROS**）。

`plan.md` §18 的形状是 `Rule Engine → Local Small LLM → Cloud LLM`。
本模块是**第二跳**：规则表没命中时才轮到它（`planner.py` 是第一跳）。

它只做**单步**：一次选 **一个** task-tier 技能。这是刻意的 —— 注册表现在只有
一个 task-tier 技能，多步计划既无技能可编排、也无从验证（D-038）。

三条不变量
----------
1. **规则表可能命中但自己不合法**（比如写了个 `control.*`）。
   那时**绝不再问 LLM** —— 那是配置错误，让模型"顺手修一下"只会把错误盖住。
   判据：只有 `REFUSE_NO_RULE` 这一种拒绝才往下走。
2. **LLM 提的东西和规则表提的东西走同一个校验口**（`planner.accept_skill`）。
   红线不因为"是谁提的"而松一格。
3. **LLM 不可用 ≠ 我听不懂。** 没开、没密钥、超时、回包烂 —— 各是各的话，
   因为人要做的事不一样。任何情况下都**不静默降级**成一句含糊的拒绝。
"""

import json
from collections import namedtuple

from embodied_agent_runtime import planner
from embodied_agent_runtime.llm_client import LlmTransportError, LlmUnavailable

#: 组合结果：**结果** + 是哪一跳产生的（进日志） + LLM 的原始回包（可审计，没问就是空）
Composed = namedtuple('Composed', 'result source raw')

SOURCE_RULES = 'rules'
SOURCE_LLM = 'llm'
SOURCE_NONE = 'none'

REFUSE_LLM_DISABLED = 'LLM 规划未启用（llm_enabled=false）'
REFUSE_LLM_NO_CLIENT = 'LLM 客户端没有配置'
#: 单飞闸门被占住（见 `plan_task` 的 `guard`）
REFUSE_LLM_BUSY = ('正在规划上一个任务 —— 本层**不排队**，请稍后再试'
                   '（与 D-026 决策 2 同一条理由）')


class ReplyError(Exception):
    """回包**内容**不合约定（不是传输问题）。

    **自带原始回包**：模型胡说的原文必须留得下来 ——
    拒绝的理由面向人，但排查时人要看的是模型到底说了什么。
    """

    def __init__(self, message, raw=''):
        super().__init__(message)
        self.raw = raw


SYSTEM_PROMPT = """你是这台机器人的**任务规划模块**。

你能做的事：从下面那份技能清单里选技能并给出参数 —— 可以是**一步**，
也可以是**一串按执行顺序排列的步骤**（多步计划）。

**为什么要多步**：有些任务只有组合才能完成，例如
「往前走，被挡住就换个方向再往前走」= 三个步骤：
  advance_until_blocked → turn_until_clear → advance_until_blocked

🔒 绝对不许选清单以外的名字，尤其不许选控制层或原语层的技能
（`control.*` / `primitive.*`）—— 那些是机器人自己的动作原语，不归你调用。
**多步不是绕过它的办法**：每一步都会被**单独**检查，与第一步的检查完全一样，
而且**只要有任何一步不合法，整条计划都会被拒**（不会"先做合法的前几步"）。

判断原则：
- **一步能做完就只给一步** —— 不要为了显得聪明而硬凑成多步。
- 需要组合时，按**实际执行顺序**列出，参数按清单里给的单位与取值范围给。
- 步骤之间**不要写条件、不要写「如果…就…」**：运行时就是按顺序执行，
  某一步失败（FAILED/CANCELLED）会**自动中止**整条计划，其余终态则继续下一步。
- 做不到 → **如实拒绝**并说明缺什么能力。宁可说"做不到"，也不要硬凑。

只输出**一个 JSON 对象**，不要解释、不要 Markdown 代码块：
  计划：{"steps": [{"skill": "<技能名>", "args": {<参数名>: <数值>}}, ...]}
  拒绝：{"refuse": "<一句话理由>"}
"""


def skill_menu(registry):
    """把注册表投影成 LLM 的**菜单**。

    只收 **task-tier 且 available** 的技能 —— 与 `accept_skill` 的判据一致：
    菜单里没有的东西，模型"想选也选不到"，这比事后拒绝更省事，也更不容易出错。
    （事后拒绝仍然保留 —— 模型可以不看菜单。）

    用的是 `SkillSpec.public_view()`：它本来就是为此设计的（D-031），
    而且**不含 `target` / `srv_type`** —— 知道名字不等于能直接调服务。
    """
    menu = []
    for name in registry.names():
        spec = registry.get(name)
        if spec is None or spec.tier != 'task' or not spec.available:
            continue
        view = spec.public_view()
        menu.append({
            'skill': name,
            'description': view['description'],
            'params': view['params'],
            'moves_robot': view['causes_motion'],
        })
    return menu


def build_messages(text, menu):
    """拼一轮对话。菜单以 JSON 附在系统消息里 —— 它就是"能选的东西"的**全部**。"""
    return [
        {'role': 'system', 'content': SYSTEM_PROMPT},
        {'role': 'system',
         'content': '可用技能清单（这是全部，不许自己造）：\n'
                    + json.dumps(menu, ensure_ascii=False, indent=2)},
        {'role': 'user', 'content': str(text)},
    ]


def _strip_code_fence(raw):
    """模型很爱套一层 ```json 围栏。解析要**宽**，校验才严。"""
    s = str(raw).strip()
    if not s.startswith('```'):
        return s
    lines = s.splitlines()
    if len(lines) >= 2 and lines[-1].strip().startswith('```'):
        lines = lines[1:-1]
    else:
        lines = lines[1:]
    return '\n'.join(lines).strip()


def parse_reply(raw):
    """把回包读成 `('steps', [{'skill':…, 'args':…}, …])` 或 `('refuse', 理由)`。

    **一步与多步用同一个形状**（`steps` 是长度为 1 的列表）—— 少一个分支，
    也少一处"单步走这条路、多步走那条路"从而严格度分家的机会。

    ⚠️ 仍然接受旧的 `{"skill": …}` 写法：**解析要宽，校验才严**
    （模型不一定听劝，而"它没按格式来"不该表现为"整件事做不了"）。

    :raises ReplyError: 不是 JSON / 不是对象 / 三种键都没有 / 类型不对
                        （**异常里带着原始回包**）
    """
    try:
        return _parse_reply(raw)
    except ReplyError as exc:
        exc.raw = str(raw)                # 在**失败点**就带上原文，别指望调用方还记得
        raise


def _parse_reply(raw):
    try:
        obj = json.loads(_strip_code_fence(raw))
    except (ValueError, TypeError) as exc:
        raise ReplyError(f'不是合法 JSON（{exc}）') from exc
    if not isinstance(obj, dict):
        raise ReplyError(f'顶层不是 JSON object，是 {type(obj).__name__}')

    if 'refuse' in obj:
        why = obj['refuse']
        if not isinstance(why, str) or not why.strip():
            raise ReplyError('refuse 字段是空的')
        return 'refuse', why.strip()

    if 'steps' in obj:
        steps = obj['steps']
        if not isinstance(steps, list):
            raise ReplyError(f'steps 必须是数组，是 {type(steps).__name__}')
        if not steps:
            raise ReplyError('steps 是空数组 —— 要么给至少一步，要么用 refuse 明确拒绝')
        out = []
        for i, st in enumerate(steps, 1):
            if not isinstance(st, dict):
                raise ReplyError(f'第 {i} 步不是 object，是 {type(st).__name__}')
            skill = st.get('skill')
            if not isinstance(skill, str) or not skill.strip():
                raise ReplyError(f'第 {i} 步的 skill 不是非空字符串')
            args = st.get('args', {})
            if not isinstance(args, dict):
                raise ReplyError(f'第 {i} 步的 args 必须是 object，是 {type(args).__name__}')
            out.append({'skill': skill.strip(), 'args': args})
        return 'steps', out

    if 'skill' in obj:                     # 旧的单步写法，仍然接受
        skill = obj['skill']
        if not isinstance(skill, str) or not skill.strip():
            raise ReplyError('skill 字段不是非空字符串')
        args = obj.get('args', {})
        if not isinstance(args, dict):
            raise ReplyError(f'args 必须是 object，是 {type(args).__name__}')
        return 'steps', [{'skill': skill.strip(), 'args': args}]

    raise ReplyError(f'既没有 steps / skill 也没有 refuse（收到：{sorted(obj)}）')


def plan_with_llm(text, registry, client):
    """问一次 LLM，把它提的东西**交给与规则表同一个校验口**。

    :return: `(PlanResult, raw_reply)`
    :raises ReplyError: 回包不合约定（调用方负责记原文并转成拒绝）
    :raises LlmUnavailable / LlmTransportError: 这一跳没跑成
    """
    raw = client.complete(build_messages(text, skill_menu(registry)))
    try:
        kind, payload = parse_reply(raw)
    except ReplyError as exc:
        exc.raw = raw                     # 原文带上，别让它烂在这里
        raise
    if kind == 'refuse':
        return planner.PlanResult(False, [], f'LLM 判断做不了：{payload}'), raw
    steps = payload
    # ★ 与规则表**同一个**校验口（`accept_plan` 内部逐步调 `accept_skill`）——
    #   红线、注册表、参数校验，**每一步**都一次不少；任何一步不合法则整条被拒。
    return planner.accept_plan(steps, registry, 'LLM'), raw


def plan_task(text, rules, registry, client, guard=None):
    """**组合那两跳**：规则表优先，没命中才问 LLM。

    :param client: 配好的 `OpenAiCompatClient`，或 `None`（= 这一跳没开）。
    :param guard: 一个 `threading.Lock`，或 `None`。**单飞闸门**：只有在**真的要去
        问 LLM** 的时候才尝试占用它；占不到就当场拒绝。

        ⚠️ 为什么必须有：节点是 4 线程的 `MultiThreadedExecutor`，而 `~/submit`
        的回调是**同步**的。几次并发 submit 各带一次几秒的 HTTP，
        就会把线程池占满 —— 那时 `on_event`（唤醒）与 `sweep`（超时兜底）
        排不上队，而这两个恰恰是**兜底**的东西。
        拒绝而不是排队，与 D-026 决策 2 同一条理由（排队会让调用方变成
        不可取消、不可超时的单线程）。

        ⚠️ 它**只护住 LLM 那一跳**：规则命中是 O(1) 的纯查表，
        不该被一次网络调用连累 —— 所以 `guard` 在规则命中时**根本不碰**。
    :return: `Composed(result, source, raw)`
    """
    result = planner.plan(text, rules, registry)
    if result.accepted or result.reason != planner.REFUSE_NO_RULE:
        # 命中且合法 → 用规则；
        # 命中但**自己不合法**（例如写了 control.*）→ 就停在这儿。
        # ⚠️ 这一条很重要：把配置错误交给 LLM"修一下"，等于把错误盖住，
        #    而红线恰恰是需要被人看见的那类错误。
        return Composed(result, SOURCE_RULES, '')

    if client is None:
        why = f'{result.reason}，且 {REFUSE_LLM_DISABLED}'
        return Composed(planner.PlanResult(False, [], why), SOURCE_NONE, '')

    held = False
    if guard is not None:
        held = guard.acquire(blocking=False)
        if not held:
            return Composed(planner.PlanResult(False, [], REFUSE_LLM_BUSY),
                            SOURCE_NONE, '')
    try:
        try:
            result, raw = plan_with_llm(text, registry, client)
        except LlmUnavailable as exc:
            why = f'{result.reason}，且 LLM 不可用：{exc}'
            return Composed(planner.PlanResult(False, [], why), SOURCE_NONE, '')
        except LlmTransportError as exc:
            why = (f'{result.reason}；LLM 调用失败'
                   f'（上限 {client.timeout_s:g}s）：{exc}')
            return Composed(planner.PlanResult(False, [], why), SOURCE_LLM, '')
        except ReplyError as exc:
            why = f'{result.reason}；LLM 回包无法解析：{exc}（原始回包已记入日志）'
            return Composed(planner.PlanResult(False, [], why), SOURCE_LLM,
                            exc.raw)
        return Composed(result, SOURCE_LLM, raw)
    finally:
        if held:
            guard.release()
