#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`look_for` 的语义 —— **纯 Python，不依赖 ROS、不依赖相机、不依赖模型**。

本模块只做两件事：把"这份回答能不能信"映射成 **task-tier 终态**（`state_for`），
以及校验"**问的是不是一句完整的话**"（`check_asked_side`）。
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


def check_asked_side(side):
    """`look_on_side` **必须**真的问一个侧。返回错误说明（`''` = 合法）。

    ⚠️ **空值不是"不限"，是"没问"** —— 这是本模块新加的一格，理由和 `state_for`
    那三行同源：**别让一个错误的问题得到一个看起来正常的答案**。
    调用方（或模型）漏填 `side` 时，它想问的是"**左半幅**有没有人"，
    如果空值被当成"整幅"，它会拿到一个**关于整个画面**的回答，
    而且**两个结果都长得对**（有 / 没有）—— 不会报错、不会崩、也不会有人发现。
    ⇒ 这就是本项目反复遇到的那一族：**一个永远不会报错的错答案**。

    ⇒ 要"整幅"就调 `semantic.look_for`，那是**另一个问题**（`TARGET_LOST` 的含义也不同）。

    ⚠️ **这一格是第二道防线**（2026-10-09 实机确认）：经**网关**的调用**根本到不了这里** ——
    网关自己就拒空串（`REJECTED: 参数 side 是空字符串`），完全不给则报
    `缺少必填参数 ['side']`。所以这条检查真正护住的是**直接调 `~/look_on_side` 的人**
    （调试脚本、以后的别的调用方）。两道都留着：**前面那道是策略，这道是这一层自己的完整性**。
    """
    # ⚠️ 判据是"**是个非空的字符串**"，不是 `str(side).strip()` ——
    #    后者会把 `None` 变成 `'None'`，于是"没给"被判成了"给了"。
    #    （这种错在真机接线里走不到，但一个**看着像在验、其实不验**的检查
    #      比没有检查更坏：它会让人以为这一格已经被守住了。）
    if isinstance(side, str) and side.strip():
        return ''
    return ('没给 side —— 这是"某一侧"的查询，**空值不等于"不限"**'
            '（`TARGET_LOST` 在两处的含义不同；要查整幅请用 semantic.look_for）')


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
