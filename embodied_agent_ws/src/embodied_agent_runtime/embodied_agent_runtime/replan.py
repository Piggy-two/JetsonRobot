#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""重规划的策略（**纯 Python，不依赖 ROS**）—— 什么时候值得"换个走法再来一次"。

D-044。`plan.md` §18 的闭环是

    计划 → 执行 → 观察 → **重规划** → 执行 …

上面那一横前的三段早就有了（`planner` / `execution` / `agent_runtime`）；
本模块补的是**"重规划"该不该发生**这一问 —— 它是**策略**，所以是纯函数，
可以脱离 ROS 与真机反复推演。真正去问规划器、去派发，在 `agent_runtime` 里。

⚠️ **为什么必须先立一条结构约束**（不是模型自觉）
------------------------------------------------
重规划最省事的写法是：任务失败 ⇒ 拿**同一段文本**再问一次规划器。
而规划器是**确定性的**（规则表那一跳完全确定；LLM 那一跳温度也低）——
所以它**多半会给出和上次一模一样的计划**。那一模一样的计划会以
一模一样的方式再失败一次，于是

    计划 → 失败 → 重规划 → 同一个计划 → 失败 → …

**转得飞快、每次看起来都很合理，而且不报任何错。** 这是本项目里最危险的一类
bug：它不崩、不告警，只是安静地绕圈，而且烧的是**真车的电和真人的耐心**。

⇒ 所以"**新计划必须与试过的都不同**"是一条**结构性检查**（`repeats_earlier`），
写在派发**之前**，不靠模型自觉、也不靠提示词请求它"别重复"。

⚠️ **有界**：`max_replans` 兜底。就算每次都给出**不同**的计划，
"A → B → C → D → …" 也可能永远收敛不了。两条检查互补：
`repeats_earlier` 管**原地转圈**，`max_replans` 管**一路越走越远**。
"""

from embodied_skill_gateway import task_state as ts

from embodied_agent_runtime import planner

#: 值得重试的终态。**只有这两个**，理由是逐条排除的：
#:
#:   BLOCKED      路被挡住了 —— 这**正是**"换个走法"的定义。最该重试的一个。
#:   TARGET_LOST  目标跟丢了 —— 退回去重找是常规操作。
#:
#: ⚠️ 其余四个**都不重试**，而且理由各不相同：
#:
#:   ARRIVED / TARGET_FOUND  是**成功**。再规划一次等于把做成的事重做一遍。
#:   CANCELLED               是**用户**说停。用户按了停，车不该自己想个办法动起来。
#:   FAILED                  是**故障**（派发失败 / 内部错误）。故障重复同样的调用
#:                           大概率同样失败，而且会把真正的问题盖在一串重试底下
#:                           —— 那正是 CLAUDE.md §3"不许绕过真正的问题"要防的。
REPLANNABLE = (ts.BLOCKED, ts.TARGET_LOST)

#: 重规划请求被拒 / 不成立的理由（面向人，且必须能区分"没开"和"不会"）
REPLAN_NOT_WORTH_IT = '这个终态不值得重试'
REPLAN_BUDGET_SPENT = '重规划预算用完'
REPLAN_NO_TEXT = '没有留下原始任务文本，无从再问一次'
REPLAN_PLANNER_REFUSED = '规划器没给出新计划'
REPLAN_SAME_PLAN = '新计划与试过的一模一样 —— 再派发一次会得到同样的结果'
REPLAN_NOT_CLEARED = 'allow_motion 没开，重规划涉及运动 —— 不派发'


def should_replan(state, replans_done, max_replans):
    """值不值得再规划一次。返回 `(bool, 理由)`。

    ⚠️ 理由**在两边都要给**：拒绝时它是解释，同意时它是日志 ——
    "为什么又试了一次"和"为什么不再试了"同样要能回答。
    """
    if state not in REPLANNABLE:
        return False, f'{REPLAN_NOT_WORTH_IT}（{state or "无终态"}）'
    if replans_done >= int(max_replans):
        return False, (f'{REPLAN_BUDGET_SPENT}（已试 {replans_done} 次，'
                       f'上限 {int(max_replans)}）')
    return True, f'{state} ⇒ 值得换个走法再试'


def repeats_earlier(steps, attempts):
    """新计划是不是**和试过的某一个**一模一样。返回那个旧计划的序号（0 起）或 -1。

    ⚠️ 比的是**全部**历史尝试，不是只看上一次。只看上一次挡不住 A→B→A→B 的
    来回摇摆 —— 那种"每次都与上一次不同"的循环同样是死循环，只是周期为 2。
    """
    for i, old in enumerate(attempts or []):
        if planner.same_plan(steps, old):
            return i
    return -1


def check_new_plan(steps, rec, movers=(), allow_motion=False):
    """把一条**新规划出来的**计划过一遍重规划该过的关。返回 `(ok, 理由)`。

    顺序对应三个不同的问题，**不能换**：
      ① 有没有东西可派？  —— 空计划没意义
      ② 和试过的一样吗？  —— **防死循环**，最要紧的一条
      ③ 是不是又要动？    —— 闸门（D-033），运动必须显式放行

    ② 排在 ③ 前面是刻意的：一条**与上次相同**的计划，就算闸门放行了也不该派 ——
    所以"重复"要在"要不要放行"之前就被判掉，否则日志里会看到
    "闸门拒绝"这种**跑偏的原因**（真正的问题是重复，不是没放行）。

    :param movers: 这条计划里**会引起运动**的技能名
    :param allow_motion: 闸门③的开关（`agent_runtime` 的参数）
    """
    if not steps:
        return False, f'{REPLAN_PLANNER_REFUSED}（空计划）'
    idx = repeats_earlier(steps, getattr(rec, 'attempts', None))
    if idx >= 0:
        return False, f'{REPLAN_SAME_PLAN}（与第 {idx + 1} 次尝试相同）'
    if movers and not allow_motion:
        return False, f'{REPLAN_NOT_CLEARED}（{list(movers)}）'
    return True, ''
