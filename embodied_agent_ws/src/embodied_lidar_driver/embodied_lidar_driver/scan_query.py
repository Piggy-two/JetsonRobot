#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR 扫描的**几何查询逻辑（纯 Python，不依赖 ROS）**。

抽出来的理由和别处一样：这些是安全层与 Skill 要依赖的判断，必须能离线证伪。
尤其下面这条，**不单测几乎必然会写错**：

⚠️ **扇区会跨越 `angle_min`/`angle_max` 的接缝。**

「正前方 ±30°」在 LD19 的数据里**不是一个连续的下标区间** ——
LD19 的 `/scan` 是 `angle_min=0`、逆时针增大、`angle_max=2π`，
所以"正前方"实际是 **330°~360°（下标靠后）** 加上 **0°~30°（下标靠前）** 两段。
任何"按角度找到起点、然后切一段数组"的写法都会**静默漏掉一半扇区**，
而漏掉的那一半恰好是最该看见的那一半。

所以本模块**一律按角度差判定**（`in_sector` 用最短角差），不按下标切片。
"""

import math

# 角度比较的容差（弧度）。约 6e-5 度 —— 远小于 LD19 的角分辨率 0.71°。
ANGLE_EPS = 1e-6


def wrap_to_pi(angle):
    """把角度归一化到 (-π, π]。"""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def angular_diff(a, b):
    """a − b 的**最短有符号角差**，落在 (-π, π]。"""
    return wrap_to_pi(a - b)


def in_sector(angle, center, width):
    """`angle` 是否落在以 `center` 为中心、总宽 `width` 的扇区内。

    宽度为 2π 或更大时视为全覆盖。
    """
    if width >= 2.0 * math.pi:
        return True
    return abs(angular_diff(angle, center)) <= (width / 2.0) + ANGLE_EPS


def valid_range(r, max_range=None):
    """一个回波值是否可用。

    ⚠️ 三种都要挡掉，而且**理由各不相同**：
      · 非有限值（NaN / inf）—— 厂商驱动用它们表示"这一束没有回波"（实测 3~7% 的点）；
      · ≤ 0 —— 物理上不可能的回波，通常是自反射或数据异常；
      · 超过 `max_range` —— 由调用方按用途决定"多远算无关"。
    """
    if r is None:
        return False
    if not math.isfinite(r):
        return False
    if r <= 0.0:
        return False
    if max_range is not None and max_range > 0.0 and r > max_range:
        return False
    return True


def sector_min_range(ranges, angle_min, angle_increment, center, width, max_range=None):
    """求扇区内最近的有效回波。

    :param ranges: 每个角度上的距离序列（`LaserScan.ranges`）
    :param angle_min: 第 0 个点的角度（弧度）
    :param angle_increment: 相邻点的角度增量（弧度）
    :param center: 扇区中心角（弧度，REP-103：0=前方、逆时针为正）
    :param width: 扇区总宽（弧度）
    :param max_range: 超过此距离的回波忽略；None 或 <=0 表示不设上限
    :return: (valid, range, angle, points)
             `angle` 用**扫描自身的角度约定**返回（即 `angle_min + i*increment`），
             这样调用方拿到的数与 `/scan` 里的角度是同一套，不必换算。
    """
    best_r = None
    best_a = 0.0
    n = 0
    if ranges is None or angle_increment is None or angle_increment == 0.0:
        return False, 0.0, 0.0, 0

    for i, r in enumerate(ranges):
        if not valid_range(r, max_range):
            continue
        a = angle_min + i * angle_increment
        if not in_sector(a, center, width):
            continue
        n += 1
        if best_r is None or r < best_r:
            best_r = r
            best_a = a

    if best_r is None:
        return False, 0.0, 0.0, 0
    return True, float(best_r), float(best_a), n


def is_path_clear(ranges, angle_min, angle_increment, center, width,
                  clear_range, max_range=None):
    """扇区内 `clear_range` 距离之内没有回波即为通畅。

    :return: (clear, range, angle)；扇区内无有效回波时 range = -1。
    """
    valid, r, a, _ = sector_min_range(
        ranges, angle_min, angle_increment, center, width,
        max_range=None if max_range is None else max_range)
    if not valid:
        # ⚠️ "没有回波"被当成"通畅"是**有意**的（空旷时本来就该通畅），
        #    但调用方必须知道这是**假设**而非证据 —— 返回 range=-1 就是为了让它可判。
        return True, -1.0, 0.0
    return (r > clear_range), r, a
