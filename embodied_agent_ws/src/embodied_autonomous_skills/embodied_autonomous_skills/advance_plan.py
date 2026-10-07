#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`advance_until_blocked` 的**判定逻辑（纯 Python，不依赖 ROS）**。

单独成文件的理由和 `motion_plan.py` / `safety_gate.py` 一样：这是**安全依据**
（"这一步迈不迈"），必须能离线证伪。

它做的事很小：**根据一次前方查询，决定"迈下一步"还是"停，以及为什么停"**。
循环、服务调用、积分都在节点里；判断只在这里。

⚠️ 最要紧的一条：**「不知道」不是「受阻」（D-028）**
------------------------------------------------------
`path_clear` 的返回里，`range = -1` 有**两种完全不同的含义**：

| 情况 | `clear` | `range` | 真实含义 |
|---|---|---|---|
| 扇区内一个回波都没有 | `true` | `-1` | **空旷**，可以走 |
| 扫描陈旧 / 从未收到 | **`false`** | `-1` | **我不知道**，`clear=false` 只是 fail-safe |

（这条写在该 `.srv` 的注释与 `scan_query.py` 的实现里：**"不知道 ≠ 安全"**。）

如果把第二种也报成 `BLOCKED`，上层会看到「前方受阻」——而**真实情况是雷达坏了**。
那会让排查方向完全跑偏（去挪障碍物，而其实该去查雷达）。
所以本模块把两者分开：**受阻报 `BLOCKED`，不知道报 `FAILED` 并写明原因**。
"""

from collections import namedtuple

# 终态常量从**网关**取（D-029：状态机的 canonical 模块在 `embodied_skill_gateway`），
# 保证全项目只有一处定义，不会各写各的字面量。
from embodied_skill_gateway import task_state as ts

# 小于这个值就当"走满了"，避免浮点残差导致多迈一步
EPS = 1e-6

# 判定结果：要做什么（advance / stop）、走多远、以及终止原因
Decision = namedtuple('Decision', 'action distance terminal note')


class AdvancePlanError(Exception):
    """参数不可接受。**拒绝执行**，不要"猜一个差不多的"。"""


def _check(value, name, positive=True):
    if value != value or value in (float('inf'), float('-inf')):
        raise AdvancePlanError(f'{name} 不是有限数（{value}）')
    if positive and value <= 0:
        raise AdvancePlanError(f'{name} 必须为正，得到 {value}')


def validate(max_distance, clear_range, step):
    """三个参数是否可接受。不接受就抛 `AdvancePlanError`。

    ⚠️ `step > max_distance` 也是错的：那意味着"一步就超过总上限"，
    实际只会走 `max_distance`，但调用方以为自己设的步长生效了 —— 属于**静默失真**。
    """
    _check(max_distance, 'max_distance')
    _check(clear_range, 'clear_range')
    _check(step, 'step')
    if step > max_distance + EPS:
        raise AdvancePlanError(
            f'step {step} 大于 max_distance {max_distance} —— 步长实际不会生效，'
            f'拒绝这种静默失真')


def decide(clear, range_m, clear_range, remaining, step):
    """根据一次前方查询决定下一步。

    :param clear: `path_clear` 返回的 clear
    :param range_m: `path_clear` 返回的 range（米；无回波或无数据时为 -1）
    :param clear_range: 本次用的"多近算受阻"阈值（米），只用于写清楚停止原因
    :param remaining: 本次还剩多少米可走
    :param step: 单步距离
    :return: `Decision`
    """
    if remaining <= EPS:
        return Decision('stop', 0.0, ts.ARRIVED, '已走满计划的距离上限')

    if not clear:
        if range_m is None or range_m < 0:
            # ★ 「不知道」——**不是**「受阻」。见模块顶部那张表。
            return Decision(
                'stop', 0.0, ts.FAILED,
                '前方距离**不可用**（雷达无数据或扫描陈旧）—— 不知道 ≠ 安全，因此不迈步。'
                '⚠️ 这**不代表**前方有障碍物：该查的是雷达链路，不是去挪障碍物')
        return Decision(
            'stop', 0.0, ts.BLOCKED,
            f'前方 {range_m:.3f} m 内有回波，近于阈值 {clear_range:.3f} m')

    # 通畅：迈一步，但不要超过剩余量
    return Decision('advance', min(step, remaining), None, '')


def summarize(travelled, reason):
    """给事件/日志用的一句话。"""
    return f'前进 {travelled:.3f} m（{reason}）'
