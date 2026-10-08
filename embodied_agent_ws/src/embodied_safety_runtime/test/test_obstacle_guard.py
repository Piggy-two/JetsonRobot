#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`obstacle_guard` 的离线单测。

这个守卫答错的两个方向代价不对称：
  · 该停没停 → 撞上去；
  · 不该停却停 → 机器人被锁住（烦人，但安全）。
所以这里把"该停的必须停"钉死，同时也钉住**不该停的三种情况不许乱停**
（启动阶段 / 停着没动 / 障碍在阈值之外）—— 因为一个老是误锁的避障会被人关掉，
关掉之后就等于没有。

⚠️ 注意本模块的判据是**运动方向**上的扇区，不是"前方"。侧移（麦轮）时
方向是 ±90°，这里专门有用例。
"""
import math

import pytest

from embodied_safety_runtime.obstacle_guard import (
    REASON_OBSTACLE, REASON_UNKNOWN_SCAN, GuardConfig, GuardInput, evaluate,
    reply_covers_question)


def gi(vx=0.0, vy=0.0, known=True, fresh=True, valid=False, rng=-1.0, pending=False):
    return GuardInput(speed_known=known, vx=vx, vy=vy, scan_fresh=fresh,
                      scan_valid=valid, scan_range=rng, scan_pending=pending)


# ---------- 不该停的三种情况 ----------

def test_disabled_never_stops():
    cfg = GuardConfig(enabled=False)
    d = evaluate(cfg, gi(vx=0.2, valid=True, rng=0.05))
    assert d.stop is False
    assert d.reason == ''


def test_never_seen_speed_does_not_stop():
    """启动阶段没拿到过 Motor Driver 状态 → 不做判定（假警报会淹掉真警报）。"""
    d = evaluate(GuardConfig(), gi(known=False, valid=True, rng=0.05))
    assert d.stop is False


def test_stationary_does_not_stop_even_with_obstacle():
    """停着不动时，前面 5 cm 有东西也不该锁存 —— 否则停在墙边就再也起不来。"""
    d = evaluate(GuardConfig(), gi(vx=0.0, vy=0.0, valid=True, rng=0.05))
    assert d.stop is False


def test_speed_below_min_is_treated_as_stationary():
    cfg = GuardConfig(min_speed=0.02)
    d = evaluate(cfg, gi(vx=0.019, valid=True, rng=0.01))
    assert d.stop is False


def test_obstacle_beyond_stop_range_does_not_stop():
    cfg = GuardConfig(lookahead=1.5, min_range=0.20)
    # 0.20 m/s × 1.5 s = 0.30 m 阈值；0.35 m 的东西不该触发
    d = evaluate(cfg, gi(vx=0.20, valid=True, rng=0.35))
    assert d.stop is False
    assert d.stop_range == pytest.approx(0.30)
    assert d.range == pytest.approx(0.35)


def test_empty_sector_with_fresh_scan_does_not_stop():
    d = evaluate(GuardConfig(), gi(vx=0.20, fresh=True, valid=False, rng=-1.0))
    assert d.stop is False
    assert d.range == -1.0


# ---------- 该停的两种情况 ----------

def test_obstacle_within_stop_range_stops():
    cfg = GuardConfig(lookahead=1.5, min_range=0.20)
    d = evaluate(cfg, gi(vx=0.20, fresh=True, valid=True, rng=0.18))
    assert d.stop is True
    assert d.reason == REASON_OBSTACLE
    assert d.reason_string.startswith('obstacle:0.18m@')


def test_unknown_scan_stops_while_moving():
    """**不知道 ≠ 安全**：雷达不新鲜时，正被命令往前走就必须停。"""
    d = evaluate(GuardConfig(), gi(vx=0.20, fresh=False))
    assert d.stop is True
    assert d.reason == REASON_UNKNOWN_SCAN
    assert d.reason_string == 'obstacle:unknown_scan'


def test_unknown_scan_does_not_stop_while_stationary():
    """雷达坏了但车没被命令动 —— 不必锁存（锁存了也是"停着"，只多一次假警报）。"""
    d = evaluate(GuardConfig(), gi(vx=0.0, fresh=False))
    assert d.stop is False


def test_pending_first_reply_does_not_stop():
    """⚠️「我还没看」≠「我看不见」：**刚问出去、回答还没回来**时不能锁存。

    这条不是理论 —— 真雷达上第一次验证就撞上了：喂一条 0.03 m/s 的指令，
    守卫在**同一拍**里发问并判定，`_reply` 还是 None，于是报 unknown_scan 锁存。
    没有这条宽限的话，每段运动的头一拍都会误锁。
    """
    d = evaluate(GuardConfig(), gi(vx=0.20, fresh=False, pending=True))
    assert d.stop is False


def test_pending_does_not_excuse_a_fresh_reading():
    """宽限只管"还没有回答"这一种；有新鲜读数时它不该改变任何判定。"""
    d = evaluate(GuardConfig(), gi(vx=0.20, fresh=True, valid=True, rng=0.10, pending=True))
    assert d.stop is True


def test_pending_does_not_apply_when_stationary():
    """停着本来就不判 —— 别让宽限把"该判"的分支也顺带放过。"""
    d = evaluate(GuardConfig(), gi(vx=0.0, fresh=False, pending=True))
    assert d.stop is False


# ---------- 阈值随速度自适应（TTC 而不是固定距离）----------

@pytest.mark.parametrize('speed,expected', [
    (0.05, 0.20),    # 0.075 < 下限 → 取 0.20
    (0.10, 0.20),    # 0.150 < 下限 → 取 0.20
    (0.20, 0.30),    # 0.300
    (0.30, 0.45),    # 0.450
])
def test_stop_range_scales_with_speed(speed, expected):
    cfg = GuardConfig(lookahead=1.5, min_range=0.20, max_range=1.00)
    d = evaluate(cfg, gi(vx=speed))
    assert d.stop_range == pytest.approx(expected)


def test_stop_range_is_capped():
    cfg = GuardConfig(lookahead=1.5, min_range=0.20, max_range=0.50)
    d = evaluate(cfg, gi(vx=2.0, valid=True, rng=0.45))
    assert d.stop_range == pytest.approx(0.50)
    assert d.stop is True          # 0.45 ≤ 0.50


def test_same_obstacle_stops_when_fast_but_not_when_slow():
    """同一个障碍：跑起来该停，慢下来就不该停 —— 这就是"按时间不按距离"的意思。"""
    cfg = GuardConfig(lookahead=1.5, min_range=0.05)
    assert evaluate(cfg, gi(vx=0.20, valid=True, rng=0.28)).stop is True    # 阈值 0.30
    assert evaluate(cfg, gi(vx=0.05, valid=True, rng=0.28)).stop is False   # 阈值 0.075


# ---------- 方向：判的是"往哪走"，不是"哪边是前" ----------

@pytest.mark.parametrize('vx,vy,deg', [
    (0.2, 0.0, 0.0),
    (0.0, 0.2, 90.0),        # 左移
    (0.0, -0.2, -90.0),      # 右移
    (-0.2, 0.0, 180.0),      # 后退
    (0.2, 0.2, 45.0),        # 斜向
])
def test_bearing_follows_travel_direction(vx, vy, deg):
    d = evaluate(GuardConfig(), gi(vx=vx, vy=vy))
    assert math.degrees(d.bearing) == pytest.approx(deg)
    assert d.speed == pytest.approx(math.hypot(vx, vy))


def test_reason_string_carries_bearing():
    d = evaluate(GuardConfig(), gi(vx=0.0, vy=0.2, valid=True, rng=0.10))
    assert d.reason_string.endswith('@+90deg')


# ---------- 边界 ----------

def test_exact_boundary_counts_as_blocked():
    """正好等于阈值算"太近"（撞上去的风险，往保守那侧取）。"""
    cfg = GuardConfig(lookahead=1.0, min_range=0.10, max_range=1.0)
    d = evaluate(cfg, gi(vx=0.20, valid=True, rng=0.20))
    assert d.stop is True


def test_zero_range_is_not_treated_as_obstacle():
    """range ≤ 0 不是"零距离有东西"，而是无效读数（−1），不能拿它当障碍。"""
    cfg = GuardConfig()
    d = evaluate(cfg, gi(vx=0.20, fresh=True, valid=False, rng=-1.0))
    assert d.stop is False


# ---------- 可观测性：不判定的分支里也要如实透出"最近读到什么" ----------

def test_stationary_still_reports_the_reading():
    """停着不判停，但 `range` 要如实报出读数 —— 否则没法诊断"守卫是不是瞎了"。"""
    d = evaluate(GuardConfig(), gi(vx=0.0, valid=True, rng=0.15))
    assert d.stop is False
    assert d.range == pytest.approx(0.15)


def test_stale_reading_is_not_reported_as_current():
    """不新鲜时**不能**透出旧值（那会让人以为它是当前的）。"""
    d = evaluate(GuardConfig(), gi(vx=0.0, fresh=False, valid=True, rng=0.15))
    assert d.range == -1.0


# ---------- 回答必须和问题配对（真机验收找出来的缺陷） ----------
#
# 真实案例（2026-10-08，地面、`--phase direction`）：正前方 0.15 m 放了瓶子、
# 左方 0.83 m 空旷。先验"往受阻的前方走"，守卫正确锁存；随后改成"往通畅的左方走"，
# 守卫却**又锁存了**，日志写 `obstacle:0.15m@+90deg` —— 那个 0.15 是**前方**的距离，
# 被当成了**左方**扇区的读数。原因就是回答没有和"问的是哪个方向"配对。

def test_reply_for_another_direction_is_not_accepted():
    """★ 回归：拿前方的读数去判左方 —— 必须判为"这不是这个问题的答案"。"""
    front, left = 0.0, math.pi / 2.0
    assert reply_covers_question(reply_center=front, reply_max_range=0.225,
                                 reply_valid=True, bearing=left, stop_range=0.225) is False


def test_reply_for_same_direction_is_accepted():
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.225,
                                 reply_valid=True, bearing=0.0, stop_range=0.225) is True


def test_small_direction_drift_still_counts_as_the_same_question():
    """扇区是 ±30°，几度以内显然是同一个问题；容差取 0 会让速度微调时每拍都作废。"""
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.225,
                                 reply_valid=True,
                                 bearing=math.radians(4.0), stop_range=0.225) is True
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.225,
                                 reply_valid=True,
                                 bearing=math.radians(6.0), stop_range=0.225) is False


def test_no_echo_only_counts_if_it_searched_far_enough():
    """"0.20 m 内没有东西"不能担保 0.225 m 的阈值 —— 没回波时要有范围覆盖。"""
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.20,
                                 reply_valid=False, bearing=0.0, stop_range=0.225) is False
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.225,
                                 reply_valid=False, bearing=0.0, stop_range=0.225) is True


def test_a_real_echo_counts_even_from_a_narrower_query():
    """真看到一个 0.15 m 的东西，比阈值近就是比阈值近 —— 范围窄不影响这条。"""
    assert reply_covers_question(reply_center=0.0, reply_max_range=0.10,
                                 reply_valid=True, bearing=0.0, stop_range=0.225) is True


def test_direction_match_wraps_around_the_pi_seam():
    """正后方在 atan2 上可能落在 ±π 两侧，别让接缝把它判成"方向变了"。"""
    assert reply_covers_question(reply_center=math.pi, reply_max_range=0.225,
                                 reply_valid=True,
                                 bearing=-math.pi + 0.01, stop_range=0.225) is True


# ---------- 阈值下限的取值：有实测依据，且不许与运行时配置分家 ----------

def test_default_min_range_is_the_measured_value():
    """`min_range` 是**实测标定**出来的，不是拍的 —— 改它要有新的地面数据。

    依据 D-040：阈值从**雷达**算，而撞上去的是**车头**（车体前伸量 0.131 m），
    锁存前还会再跑 0.050~0.055 m。0.20 时只余 1.9 cm（实测 3.9 cm），
    2026-10-08 提到 0.30。
    """
    assert GuardConfig().min_range == pytest.approx(0.30)


def test_shipped_config_matches_the_dataclass_default():
    """**运行时以那份 yaml 为准** —— 它和代码默认值分家，就会出现"代码说 0.30、
    实际跑 0.20"这种谁也看不出来的状况（与坑 27 同族：两个数伺候同一件事）。"""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    import os
    path = os.path.join(get_package_share_directory('embodied_safety_runtime'),
                        'config', 'safety_runtime.yaml')
    with open(path, 'r', encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)
    rf = next(body['ros__parameters'] for body in cfg.values()
              if isinstance(body, dict) and 'ros__parameters' in body)
    d = GuardConfig()
    assert float(rf['obstacle_min_range']) == pytest.approx(d.min_range)
    assert float(rf['obstacle_lookahead']) == pytest.approx(d.lookahead)
    assert float(rf['obstacle_max_range']) == pytest.approx(d.max_range)


# ---------- 参数卫生 ----------

@pytest.mark.parametrize('kwargs', [
    {'lookahead': 0.0},
    {'lookahead': -1.0},
    {'min_range': -0.1},
    {'max_range': 0.0},
    {'max_range': 0.1, 'min_range': 0.2},   # 上限 < 下限
    {'width': 0.0},
    {'width': 2.0 * math.pi + 0.1},
    {'min_speed': -0.01},
])
def test_rejects_bad_config(kwargs):
    with pytest.raises(ValueError):
        GuardConfig(**kwargs)


def test_full_circle_width_is_allowed():
    cfg = GuardConfig(width=2.0 * math.pi)
    assert evaluate(cfg, gi(vx=0.2)).stop is False
