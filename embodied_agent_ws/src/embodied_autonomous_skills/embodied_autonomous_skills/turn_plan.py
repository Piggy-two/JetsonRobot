#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`turn_until_clear` 的**判定逻辑（纯 Python，不依赖 ROS）**。

和 `advance_plan.py` 同构：这是**安全依据**（"这一步转不转"），必须能离线证伪。
节点负责循环、服务调用、累加角度；判断只在这里。

⚠️ 最要紧的一条（与 `advance_plan.py` 同）：**「不知道」不是「受阻」**
------------------------------------------------------------------
`path_clear` 的 `range = -1` 有两种完全不同的含义：

| 情况 | `clear` | 真实含义 |
|---|---|---|
| 扇区内一个回波都没有 | `true` | **空旷** —— 目标已达成 |
| 扫描陈旧 / 从未收到 | `false` | **我不知道** —— fail-safe 报 false |

把第二种报成 `BLOCKED`，上层会看到「这里被围住了」，而真相是**雷达坏了**；
排查方向会完全跑偏（去挪障碍物，而该查雷达）。所以：**受阻报 `BLOCKED`，不知道报 `FAILED`。**

⚠️ 本条技能里 `ARRIVED` 的含义（**很容易被读错**）
-----------------------------------------------------
`ARRIVED` 通常被读成"到达了某个位置"。**反应式技能没有"位置"可言** ——
这里的含义是「**达成了本技能自己定义的目标条件**」，也就是**前方通畅了**。

⚠️ 把它读成"位移到位"是错的。当前**没有任何独立反馈**能证实朝向或位移
（`rotate` 是开环的，自报的 `heading` 也是开环累加），所以：
**不要拿本技能的 `ARRIVED` 去推出"现在朝向多少度"。**
"""

from collections import namedtuple

from embodied_skill_gateway import task_state as ts

EPS = 1e-6

#: 只接受这两个转向值。**不收"接近 +1 的任意正数"** —— 那会让一个笔误
#: （比如把弧度 0.5 当成方向）静默生效，而表现是"转错边"。
VALID_DIRECTIONS = (1.0, -1.0)

Decision = namedtuple('Decision', 'action angle terminal note')


class TurnPlanError(Exception):
    """参数不可接受。**拒绝执行**，不要"猜一个差不多的"。"""


def _check(value, name, positive=True):
    if value != value or value in (float('inf'), float('-inf')):
        raise TurnPlanError(f'{name} 不是有限数（{value}）')
    if positive and value <= 0:
        raise TurnPlanError(f'{name} 必须为正，得到 {value}')


def validate(max_angle, clear_range, step_angle, direction):
    """四个参数是否可接受。不接受就抛 `TurnPlanError`。"""
    _check(max_angle, 'max_angle')
    _check(clear_range, 'clear_range')
    _check(step_angle, 'step_angle')
    if step_angle > max_angle + EPS:
        # 与 AdvancePlan 同一条理由：一步就超过总上限，调用方以为步长生效了 —— 静默失真。
        raise TurnPlanError(
            f'step_angle {step_angle} 大于 max_angle {max_angle} —— 步长实际不会生效，'
            f'拒绝这种静默失真')
    _check(direction, 'direction', positive=False)
    if direction not in VALID_DIRECTIONS:
        raise TurnPlanError(
            f'direction 只接受 +1（逆时针/左）或 -1（顺时针/右），得到 {direction}')


def decide(clear, range_m, clear_range, remaining, step_angle, direction):
    """根据一次前方查询决定"再转一步"还是"停，以及为什么停"。

    :param clear: `path_clear` 返回的 clear
    :param range_m: `path_clear` 返回的 range（米；无回波或无数据时为 -1）
    :param clear_range: 本次用的"多近算受阻"阈值（米），只用于写清楚停止原因
    :param remaining: 本次还剩多少弧度可转
    :param step_angle: 单步转角（正数）
    :param direction: +1 / -1
    :return: `Decision`（`angle` 是**带符号**的下一步转角）
    """
    # ★ **先看有没有通** —— 顺序不能反。
    #    如果先判 remaining，那么"最后一步转完刚好通畅"会被报成 BLOCKED
    #    （转满上限），而它其实已经达成了目标。先看路，再看预算。
    if clear:
        if range_m is not None and range_m >= 0:
            # 有回波但**在阈值之外** —— 说清是"多远"，别只说"通畅"
            note = f'前方最近回波 {range_m:.3f} m，远于阈值 {clear_range:.3f} m'
        else:
            note = f'前方 {clear_range:.3f} m 内一个回波都没有，通畅'
        return Decision('stop', 0.0, ts.ARRIVED, note)

    if remaining <= EPS:
        return Decision('stop', 0.0, ts.BLOCKED,
                        f'转满上限后前方 {range_m:.3f} m 内仍有回波'
                        if (range_m is not None and range_m >= 0)
                        else '转满上限后前方仍然不通')

    # `clear=False` 时 `range<0` 只可能是"没有数据"（见模块顶部那张表）。
    if range_m is None or range_m < 0:
        return Decision(
            'stop', 0.0, ts.FAILED,
            '前方距离**不可用**（雷达无数据或扫描陈旧）—— 不知道 ≠ 受阻，因此不转。'
            '⚠️ 这**不代表**前面有障碍物：该查的是雷达链路，不是去挪障碍物')

    return Decision('turn', direction * min(step_angle, remaining), None, '')


def summarize(heading, reason):
    """给事件/日志用的一句话。`heading` 以弧度、带符号（= 各步转角之和）。"""
    return f'朝向 {heading:+.3f} rad｜{reason}'
