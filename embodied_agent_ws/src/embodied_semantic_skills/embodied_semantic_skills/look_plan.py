#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`look_for` 的语义 —— **纯 Python，不依赖 ROS、不依赖相机、不依赖模型**。

本模块只做一件事：把"这份回答能不能信"映射成 **task-tier 终态**。
它之所以单独成文件、还要有单测，是因为**这三行就是本技能的语义**：

    TARGET_FOUND  看到了                  ← 正面证据
    TARGET_LOST   **确认没有**（画面质量达标才敢这么说）
    FAILED        **不知道**（糊 / 没帧 / 模型没就绪 / 类别不在表里）

⚠️ **最容易写反的一格**：把"不知道"报成 `TARGET_LOST`。
那会让上层得出"这里没有目标"的结论 —— 而真相可能只是**画面糊了**。
2026-10-09 我们就栽在这上面：相机失焦，所有检测都是 0 个，
而它和"真没东西"看起来一模一样（`DEV_NOTES` 坑 43）。
⇒ 与 D-028 同一条铁律：**不知道 ≠ 没有**（那里是"不知道 ≠ 安全"）。

⚠️ 为什么"不知道"用 `FAILED` 而不用 `BLOCKED`：本项目已有先例 ——
LiDAR 原语在**扫描陈旧**时报 `FAILED`、"前方有东西"才报 `BLOCKED`（D-028）。
`FAILED` 在这里的含义是"**这一问没有得到可信的回答**"，
所以 `message` **必须**把原因说清，否则人会去查错方向。
"""

from embodied_skill_gateway import task_state as ts


def state_for(*, valid, found):
    """`(valid, found)` → task-tier 终态。

    :param valid: 视觉那边"这份回答算不算数"（`FindInView.valid`）。
                  ⚠️ 它是**先决条件**：`valid=False` 时 `found` 无意义。
    """
    if not valid:
        # ★ 不知道 —— **绝不能**落到 TARGET_LOST
        return ts.FAILED
    return ts.TARGET_FOUND if found else ts.TARGET_LOST


def success_for(state):
    """与其它 task-tier 技能同规矩：`success` 只等于"达成了本技能的目标条件"。

    ⚠️ 所以"确认没有"是 `TARGET_LOST` —— **`success=False` 但这不是故障**。
    把它读成"技能坏了"是本项目反复提醒的那类误读（D-032）。
    """
    return state == ts.TARGET_FOUND


def bounded_score(value, lo=0.0, hi=1.0):
    """把置信度夹回 `[0, 1]`。

    为什么要有这一步：`FindInView` 的 `score` 是 `float32`，模型给的分经过
    网络/消息往返之后可能变成 `1.0000001` 这种值。而**下游会拿它跟阈值比** ——
    差一个 ULP 就能让"刚好达标"变成"没达标"（与 D-039 那次"两个数各说各话"同族：
    跨接口的数值，**边界要有人负责**）。
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return lo
    return max(lo, min(hi, v))
