#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`scan_query` 的离线单测（不需要 ROS、不需要 LiDAR）。

重点是**扇区跨接缝**那组 —— 那是本模块最容易静默写错的地方，
而且错法是"漏掉一半扇区、恰好漏掉最该看见的那一半"。
"""
import math

import pytest

from embodied_lidar_driver.scan_query import (
    angular_diff, in_sector, is_path_clear, sector_min_range, valid_range,
    wrap_to_pi)

INF = float('inf')
NAN = float('nan')


# ---------- 角度工具 ----------

@pytest.mark.parametrize('a,expect', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
    (3 * math.pi / 2, -math.pi / 2), (2 * math.pi + 0.1, 0.1),
    (-2 * math.pi - 0.1, -0.1),
])
def test_wrap_to_pi(a, expect):
    assert wrap_to_pi(a) == pytest.approx(expect)


def test_angular_diff_takes_the_short_way():
    # 350° 与 10° 只差 20°，不是 340°
    assert angular_diff(math.radians(350), math.radians(10)) == pytest.approx(math.radians(-20))
    assert angular_diff(math.radians(10), math.radians(350)) == pytest.approx(math.radians(20))


def test_in_sector_across_the_seam():
    """正前方 ±30° 必须同时包住 350° 和 10° 两侧。"""
    c, w = 0.0, math.radians(60)
    assert in_sector(math.radians(0), c, w)
    assert in_sector(math.radians(10), c, w)
    assert in_sector(math.radians(350), c, w)      # ← 接缝另一侧
    assert in_sector(math.radians(29.9), c, w)
    assert not in_sector(math.radians(40), c, w)
    assert not in_sector(math.radians(320), c, w)


def test_in_sector_full_width():
    assert in_sector(1.234, 0.0, 2 * math.pi)
    assert in_sector(1.234, 0.0, 10.0)


# ---------- 有效回波判定 ----------

@pytest.mark.parametrize('r,ok', [
    (1.0, True), (0.02, True), (25.0, True),
    (NAN, False), (INF, False), (-INF, False), (0.0, False), (-1.0, False), (None, False),
])
def test_valid_range(r, ok):
    assert valid_range(r) is ok


def test_valid_range_max_filter():
    assert valid_range(3.0, max_range=5.0) is True
    assert valid_range(6.0, max_range=5.0) is False
    assert valid_range(6.0, max_range=None) is True
    assert valid_range(6.0, max_range=0.0) is True       # <=0 表示不设上限


# ---------- 扇区最近回波 ----------

def test_min_range_basic():
    # 角度 0, 1, 2, 3 rad，距离 5, 3, 9, 1
    valid, r, a, n = sector_min_range([5, 3, 9, 1], 0.0, 1.0, center=0.0, width=2.0)
    assert valid and r == pytest.approx(3.0) and n == 2          # 只含 0 和 1 rad
    assert a == pytest.approx(1.0)


def test_min_range_ignores_nan_and_inf():
    valid, r, a, n = sector_min_range([NAN, INF, 2.5, NAN], 0.0, 1.0, 0.0, 10.0)
    assert valid and r == pytest.approx(2.5) and n == 1 and a == pytest.approx(2.0)


def test_min_range_respects_max_range():
    # 最近的是 4.0，但被 max_range=3 滤掉；剩下 2.0？不，这里只有 4.0 与 9.0
    valid, r, a, n = sector_min_range([4.0, 9.0], 0.0, 1.0, 0.0, 10.0, max_range=5.0)
    assert valid and r == pytest.approx(4.0)
    valid2, r2, a2, n2 = sector_min_range([4.0, 9.0], 0.0, 1.0, 0.0, 10.0, max_range=3.0)
    assert not valid2 and n2 == 0


def test_min_range_empty_sector_is_invalid():
    valid, r, a, n = sector_min_range([1.0, 1.0], 0.0, 0.1, center=3.0, width=0.1)
    assert not valid and n == 0


def test_min_range_empty_input():
    assert sector_min_range([], 0.0, 0.01, 0.0, 1.0) == (False, 0.0, 0.0, 0)
    assert sector_min_range(None, 0.0, 0.01, 0.0, 1.0) == (False, 0.0, 0.0, 0)


def test_min_range_zero_increment_is_rejected():
    """increment=0 会让所有点落在同一角度上 —— 数据本身有问题，不该硬算。"""
    assert sector_min_range([1.0, 2.0], 0.0, 0.0, 0.0, 1.0) == (False, 0.0, 0.0, 0)


def test_min_range_across_the_seam():
    """LD19 的真实排布：angle_min=0、逆时针、angle_max=2π。

    "正前方 ±90°" 应当同时看到 0°、90° 和 270°(=-90°) 三处。
    这里把最近的回波放在 270°（接缝靠后那一侧），验证它确实被算进来。
    """
    inc = math.pi / 2
    # 角度 0°, 90°, 180°, 270°；最近的 1.5 m 放在 **270°**（接缝靠后那一侧）
    ranges = [4.0, INF, 9.9, 1.5]
    valid, r, a, n = sector_min_range(ranges, 0.0, inc, center=0.0, width=math.pi)
    assert valid and r == pytest.approx(1.5) and n == 2      # 0° 与 270° 都在扇区内
    assert a == pytest.approx(3 * math.pi / 2)


def test_min_range_returns_angle_in_scan_convention():
    """返回的角度要用扫描自身的约定（0~2π），不要换成 (-π,π] —— 否则调用方要自己换算。"""
    inc = 0.1
    ranges = [INF] * 10 + [2.0]
    valid, r, a, n = sector_min_range(ranges, 0.0, inc, center=0.0, width=2 * math.pi)
    assert valid and a == pytest.approx(1.0)                 # 下标 10 × 0.1


# ---------- 通畅判定 ----------

def test_path_clear_true_when_obstacle_is_beyond_threshold():
    clear, r, a = is_path_clear([5.0], 0.0, 1.0, 0.0, 2.0, clear_range=1.0)
    assert clear and r == pytest.approx(5.0)


def test_path_not_clear_when_obstacle_is_within_threshold():
    clear, r, a = is_path_clear([0.4], 0.0, 1.0, 0.0, 2.0, clear_range=1.0)
    assert not clear and r == pytest.approx(0.4)


def test_path_clear_when_no_echo_reports_minus_one():
    """没有回波算通畅 —— 但必须能看出来这是**假设**而不是证据（range=-1）。"""
    clear, r, a = is_path_clear([NAN, INF], 0.0, 1.0, 0.0, 2.0, clear_range=1.0)
    assert clear is True and r == -1.0
