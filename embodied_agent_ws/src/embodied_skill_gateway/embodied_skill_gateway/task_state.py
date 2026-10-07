#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务状态机（**纯 Python，不依赖 ROS**）。

单独成文件的理由和 `safety_gate.py` / `motion_plan.py` / `estop.py` 一样：
这是**安全依据**（"Agent 什么时候被唤醒"直接决定它下一步规划什么），
必须能离线证伪，不能靠"跑起来看着像对的"。

两个词汇表（**别混用**）
------------------------

**A) task-tier 技能（Autonomous / Semantic）** —— `plan.md` §21 定的 8 个状态：

    STARTED / RUNNING / ARRIVED / TARGET_FOUND / TARGET_LOST / BLOCKED / FAILED / CANCELLED

后 6 个是终态，也是**唯一会唤醒 Agent 的状态**（D-004）。
`STARTED` / `RUNNING` **不唤醒** —— 靠每个中间态唤醒会淹没 Agent，
靠轮询又会浪费算力；D-004 要的正是"事件驱动，而不是实时 LLM 控制器"。

**B) control-tier 技能（Control Skill）** —— 见 `FINISHED` 那一段。

为什么 Control Skill **不进那 8 个状态**（D-032）
--------------------------------------------------
Control Skill 的 `success` 含义是「**速度按时长发完了**」，不是「走到位了」（D-026）。
8 个状态里**没有任何一个能诚实表达它**：

  · 映射成 `ARRIVED`   → 撒谎。Agent 会以为到位了，后续规划建立在假前提上。
  · 映射成 `TARGET_FOUND` → 更离谱。
  · 其余终态都是失败语义 → 也不对。

硬塞只能编造。所以 control-tier 另走一条 dispatch 路径，终态用
`FINISHED` / `FAILED` / `CANCELLED`，其中：

> ⚠️ **`FINISHED` 不是那 8 个任务状态之一**，它的含义**只有**「技能返回了」，
>    绝不表示"到位了 / 达成目标了"。

⚠️ **两个词汇表只在 `FINISHED` 这一处分叉。**
`FAILED` / `CANCELLED` 是**共用**的（它们本来就是任务终态，"失败"和"被取消"
在两个层级上含义一致），所以它们**仍然会唤醒 Agent**。
别把"control-tier 不唤醒"误推广成"control 的所有终态都不唤醒"——
只有 `FINISHED` 不唤醒。

不变式
------
1. **终态没有出边** —— 到了终态就不能再变。
2. **每个任务恰好一个终态** —— 缺失会让 Agent 永远 WAIT；重复会让它被唤醒两次。
3. **唤醒集 == task-tier 终态集** —— 这两者必须恒等，不能各自演化。
"""

# ---------- task-tier：plan.md §21 的 8 个状态 ----------
STARTED = 'STARTED'
RUNNING = 'RUNNING'
ARRIVED = 'ARRIVED'
TARGET_FOUND = 'TARGET_FOUND'
TARGET_LOST = 'TARGET_LOST'
BLOCKED = 'BLOCKED'
FAILED = 'FAILED'
CANCELLED = 'CANCELLED'

TASK_STATES = (STARTED, RUNNING, ARRIVED, TARGET_FOUND, TARGET_LOST,
               BLOCKED, FAILED, CANCELLED)

# task-tier 的终态（plan.md §21 / ARCHITECTURE.md §8 的迁移箭头终点）
TASK_TERMINAL = (ARRIVED, TARGET_FOUND, TARGET_LOST, BLOCKED, FAILED, CANCELLED)

# D-004：唤醒 Agent 的**只有**任务级终态
TASK_WAKING = TASK_TERMINAL

# ---------- control-tier：表达"技能返回了"，不表达"到达了" ----------
FINISHED = 'FINISHED'

CONTROL_STATES = (RUNNING, FINISHED, FAILED, CANCELLED)
CONTROL_TERMINAL = (FINISHED, FAILED, CANCELLED)

# 全部终态（两个词汇表的并集；FAILED / CANCELLED 两边共用）
ALL_TERMINAL = tuple(dict.fromkeys(TASK_TERMINAL + CONTROL_TERMINAL))

# 合法迁移表。终态一律 **没有出边** —— 这是"恰好一个终态"的实现手段。
_TRANSITIONS = {
    # task-tier
    STARTED: frozenset({RUNNING}) | frozenset(TASK_TERMINAL),
    RUNNING: frozenset(TASK_TERMINAL) | frozenset(CONTROL_TERMINAL),
    # control-tier 的起点也是 RUNNING（受理即开始跑，没有"已受理但未启动"的阶段）
    # 终态：全部无出边
    ARRIVED: frozenset(),
    TARGET_FOUND: frozenset(),
    TARGET_LOST: frozenset(),
    BLOCKED: frozenset(),
    FAILED: frozenset(),
    CANCELLED: frozenset(),
    FINISHED: frozenset(),
}

ALL_STATES = tuple(_TRANSITIONS.keys())


class InvalidTransition(Exception):
    """非法的状态迁移。**必须抛错，不能静默忽略** —— 静默忽略会让状态机失去意义。"""


def is_terminal(state):
    """是不是终态。"""
    return state in ALL_TERMINAL


def is_waking(state):
    """这个状态会不会唤醒 Agent（D-004）。

    ⚠️ 只有 **task-tier 终态** 会。`FINISHED`（control-tier）**不会** ——
    它只表示"技能返回了"，而 Agent 本来就不该调 control 技能（D-003 的 Permission 拦着）。
    """
    return state in TASK_WAKING


def is_known(state):
    return state in ALL_STATES


def can_transition(old, new):
    """`old` → `new` 是否合法。"""
    if old not in _TRANSITIONS or new not in _TRANSITIONS:
        return False
    return new in _TRANSITIONS[old]


def check_transition(old, new):
    """不合法就抛 `InvalidTransition`（带可读原因）。"""
    if not is_known(old):
        raise InvalidTransition(f'未知的旧状态 {old!r}')
    if not is_known(new):
        raise InvalidTransition(f'未知的新状态 {new!r}')
    if not can_transition(old, new):
        if is_terminal(old):
            raise InvalidTransition(
                f'{old} 是终态，没有出边（不能变成 {new}）—— 每个任务恰好一个终态')
        raise InvalidTransition(f'不允许 {old} → {new}')
    return True


def to_dict():
    """给 `~/list` 或诊断用的一份自描述。"""
    return {
        'task_states': list(TASK_STATES),
        'task_terminal': list(TASK_TERMINAL),
        'task_waking': list(TASK_WAKING),
        'control_states': list(CONTROL_STATES),
        'control_terminal': list(CONTROL_TERMINAL),
        'all_terminal': list(ALL_TERMINAL),
        'transitions': {k: sorted(v) for k, v in _TRANSITIONS.items()},
    }
