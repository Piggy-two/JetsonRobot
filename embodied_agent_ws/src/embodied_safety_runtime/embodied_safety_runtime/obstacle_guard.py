#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障守卫的**判定逻辑（纯 Python，不依赖 ROS）**。

D-027 明确把"避障"列为**本版不做**；本模块填的就是这个缺口。
它只回答一个问题：

> **此刻，按当前被命令的运动方向，前方 `stop_range` 之内有没有东西？**

## 为什么阈值是"时间"而不是"距离"

D-028 留了一条硬指标：**避障阈值必须在真实部署环境实测标定**，因为实测近场回波占比
在两次摆位之间从 **3.6% 变到 67%**（同一个雷达、同一台车）——
"任何拍出来的固定距离都会被场地否决"。

所以这里**不用固定距离**，用**预留时间**（time-to-collision 式）：

    stop_range = clamp(速度 × 预留时间, 下限, 上限)

速度越快、需要越早刹，阈值自动跟着变大；停着不动时阈值为 0（**不触发**）。
关键性质：**阈值是可调量而不是场地常量** —— 它由"这车刹得住吗"决定，
标定一次就与场地无关（场地变的是"前面恰好有没有东西"，不是"车需要多远刹住"）。

## 三条"不知道就停"的方向

本模块**只在能确定"正被命令往某个方向走"时才判**，其余情况按下面的规则：

| 情况 | 判定 | 为什么 |
|---|---|---|
| 没有速度信息（从没见过 Motor Driver 状态） | **不触发** | 启动时本来就没人命令运动。与 `watchdog.py`「首次见到之前永不判失联」同一条理由：**假警报会淹掉真警报** |
| 有速度信息但速度 < `min_speed`（基本是停着） | **不触发** | 静止、原地转身都不是"要往前走"，此时报警只会在墙边误锁 |
| 速度够大，雷达不新鲜，但**才刚开始问、还没等到第一份回答**（`scan_pending`） | **不触发** | 「**我还没看**」不等于「**我看不见**」。静默期的"没有数据"什么也证明不了 —— 这条是本项目踩过的坑（`DEV_NOTES` 坑 15 的更正），不设它的话，**每次"从静止开始动"的头一拍都会误锁存** |
| 速度够大，雷达不新鲜（问过且已超时） | **触发**（`unknown_scan`） | **不知道 ≠ 安全**（D-028）。瞎着往前走是最危险的一种"不知道" |
| 速度够大，方向扇区内有回波且 ≤ `stop_range` | **触发**（`obstacle`） | 本功能的正题 |

`scan_pending` 由调用方按"还没拿到过任何回答、且距首次发问不超过一个回答超时"给出 ——
即它是个**有界的**宽限：超时之后仍然没有回答，就照「不知道」处理。

⚠️ **本模块不判断"要不要绕开"** —— 它只会停。绕行属 Phase 3 的局部规划器
（`docs/ARCHITECTURE.md` §9.1 的 `LiDAR → Costmap → Detection → Planner → Velocity`）。
"""

import math
from dataclasses import dataclass

# 触发原因（会被拼进事件名 `estop_triggered:...`，所以是纯 ASCII）
REASON_OBSTACLE = 'obstacle'
REASON_UNKNOWN_SCAN = 'obstacle:unknown_scan'


@dataclass(frozen=True)
class GuardConfig:
    """避障守卫的参数。

    ⚠️ `lookahead` / `min_range` / `width` 三个量的默认值是**待实测标定的临时值**，
    不是"拍出来的正确值"：标定方法见 `docs/DEV_NOTES.md`（本功能那一节）。
    """

    enabled: bool = True
    #: 预留时间（秒）—— 阈值 = 速度 × 它。**这是唯一该按本车刹停能力标定的量。**
    lookahead: float = 1.5
    #: 阈值下限（米）—— 低速时兜底，反映"本车自己的 footprint + 反应距离"。
    #: ⚠️ 这个数**必须从雷达量出来的距离里扣掉车体前伸量**（车头在雷达前方 0.131 m），
    #: 再留出锁存前的蠕动（实测 0.050~0.055 m）。0.20 时只余 1.9 cm（实测 3.9 cm），
    #: 2026-10-08 按 D-040 提到 **0.30**。⚠️ **改它必须同步改 `config/safety_runtime.yaml`** ——
    #: 运行时以那份 yaml 为准。
    min_range: float = 0.30
    #: 阈值上限（米）—— 只是合理性封顶，正常速度下不应触及
    max_range: float = 1.00
    #: 运动方向扇区的全宽（弧度）。默认 ±30°，与 LiDAR 原语的"前方扇区"同宽
    width: float = 1.0471975511965976
    #: 低于此速度视为"没有在平移"（米/秒）
    min_speed: float = 0.02

    def __post_init__(self):
        if self.lookahead <= 0.0:
            raise ValueError(f'lookahead 必须为正: {self.lookahead}')
        if self.min_range < 0.0:
            raise ValueError(f'min_range 不能为负: {self.min_range}')
        if self.max_range <= 0.0:
            raise ValueError(f'max_range 必须为正: {self.max_range}')
        if self.max_range < self.min_range:
            raise ValueError(
                f'max_range({self.max_range}) 不能小于 min_range({self.min_range})')
        if not (0.0 < self.width <= 2.0 * math.pi):
            raise ValueError(f'width 必须落在 (0, 2π]: {self.width}')
        if self.min_speed < 0.0:
            raise ValueError(f'min_speed 不能为负: {self.min_speed}')


@dataclass(frozen=True)
class GuardInput:
    """一次判定的输入（全部来自节点侧，本模块不碰 ROS）。"""

    #: 是否**曾经**拿到过 Motor Driver 状态（决定"知不知道速度"）
    speed_known: bool
    #: Motor Driver 状态里的输出速度（机体坐标系；锁存/超时时它自己就是 0）
    vx: float
    vy: float
    #: 雷达心跳是否新鲜（由 `/embodied/lidar/front` 的到达时刻判定）
    scan_fresh: bool
    #: 最近一次 `sector_min_range` 在运动方向扇区内的结果
    scan_valid: bool
    scan_range: float
    #: 「才刚开始问、还没等到第一份回答」—— 与"问过但没有回答"不同，见模块文档
    scan_pending: bool = False


@dataclass(frozen=True)
class GuardDecision:
    stop: bool
    #: '' | 'obstacle' | 'obstacle:unknown_scan'
    reason: str
    #: 本次判定用的阈值（米）；未激活时为 0
    stop_range: float
    #: 运动方向（弧度，REP-103）；未激活时为 0
    bearing: float
    #: 平移速度（米/秒）
    speed: float
    #: 扇区内最近回波（米）；无回波或未知时为 −1
    range: float

    @property
    def reason_string(self):
        """拼进 `estop_triggered:<这里>` 的字符串。"""
        if self.reason == REASON_OBSTACLE:
            return f'{REASON_OBSTACLE}:{self.range:.2f}m@{math.degrees(self.bearing):+.0f}deg'
        return self.reason


def angle_diff(a, b):
    """两个角的最短差（弧度，落在 (−π, π]）。"""
    d = (a - b) % (2.0 * math.pi)
    return d - 2.0 * math.pi if d > math.pi else d


#: 回答与当前问题之间允许的方向差。扇区是 ±30°，几度以内显然还是"同一个问题"；
#: 容差取 0 会让速度微调时**每一拍**都判定作废、重新发问 —— 宽限一耗尽就变成假锁存。
REPLY_DIRECTION_TOL = math.radians(5.0)


def reply_covers_question(reply_center, reply_max_range, reply_valid,
                          bearing, stop_range, tol=REPLY_DIRECTION_TOL):
    """手头这份回答，**答的是不是现在这个问题**？两件事都要对上：

      · **方向** —— 这份回答是问 `reply_center` 得来的，而此刻的运动方向是 `bearing`；
      · **范围** —— 若这份回答说"扇区里什么都没看到"（`reply_valid=False`），
        它只在**当时问的范围 ≥ 当前阈值**时才作数：拿一句"0.20 m 内没有东西"
        去担保 0.225 m 的阈值是不成立的。
        有回波时不受此限 —— 真看见一个 0.15 m 的东西，比阈值近就是比阈值近。

    ⚠️ 不配对的后果**两个方向都有害**：旧方向近、新方向空 ⇒ **假锁存**；
    旧方向远、新方向有障碍 ⇒ **漏判一拍**。真实案例见 `DEV_NOTES` 坑 28。
    """
    if abs(angle_diff(reply_center, bearing)) > tol:
        return False
    if reply_valid:
        return True
    return reply_max_range >= stop_range - 1e-9


def evaluate(cfg, gi):
    """按 `cfg` 判定 `gi` 这一刻要不要停车。**纯函数，可离线证伪。**

    `GuardDecision.range` 一律是"**最近一次新鲜扇区读数**"（没有就是 −1），
    与"这次要不要停"无关 —— 这样即便守卫处在"不判定"的分支里，
    上层仍能从状态里看出**它到底有没有在读世界**（诊断"守卫是不是瞎了"时最需要这个）。
    """
    #: 最近一次新鲜读数（不新鲜时**不**透出旧值：那会让人以为它是当前的）
    observed = gi.scan_range if (gi.scan_fresh and gi.scan_valid) else -1.0

    if not cfg.enabled:
        return GuardDecision(False, '', 0.0, 0.0, 0.0, -1.0)

    # 从没见过速度 → 启动阶段，没有"被命令的运动"可言（见模块文档的表）
    if not gi.speed_known:
        return GuardDecision(False, '', 0.0, 0.0, 0.0, -1.0)

    speed = math.hypot(gi.vx, gi.vy)
    bearing = math.atan2(gi.vy, gi.vx) if speed > 0.0 else 0.0

    if speed < cfg.min_speed:
        # 停着（或原地转身）时不做避障判定 —— 否则停在墙边会被反复锁存
        return GuardDecision(False, '', 0.0, bearing, speed, observed)

    stop_range = min(cfg.max_range, max(cfg.min_range, speed * cfg.lookahead))

    if not gi.scan_fresh:
        if gi.scan_pending:
            # 「我还没看」≠「我看不见」：第一次问出去、回答还在路上。
            # 不宽限的话，**每段运动的第一拍都会以"不知道"锁存**（这不是理论，
            # 真雷达验证时第一次就撞上了）。宽限是**有界的**：超时后照样按"不知道"停。
            return GuardDecision(False, '', stop_range, bearing, speed, observed)
        # 不知道 ≠ 安全：瞎着往前走，停。
        return GuardDecision(True, REASON_UNKNOWN_SCAN, stop_range, bearing, speed, observed)

    # 注意：请求时已经把 max_range 设成了 stop_range，这里**再比一次**是刻意的
    # —— 让"太近"的判据同时活在本模块里，可以不依赖服务端语义单独单测。
    if gi.scan_valid and 0.0 < gi.scan_range <= stop_range:
        return GuardDecision(True, REASON_OBSTACLE, stop_range, bearing, speed, observed)

    return GuardDecision(False, '', stop_range, bearing, speed, observed)
