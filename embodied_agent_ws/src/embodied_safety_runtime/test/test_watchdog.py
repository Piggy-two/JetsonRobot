#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`StalenessWatchdog` 的离线单测。

这个看门狗负责回答一个**没人会来告诉你**的问题："Motor Driver 是不是已经死了"。
它答错的两个方向代价不对称：
  · 漏报 → 底盘保持最后速度一直跑（危险）；
  · 误报 → 机器人被莫名其妙锁住急停（烦人，但安全）。
所以单测要把"该报的时候一定报"钉死；同时也要钉住**启动阶段不报**，
因为假警报多了真警报就没人看了。
"""
import pytest

from embodied_safety_runtime.watchdog import StalenessWatchdog


def test_never_seen_does_not_expire():
    """⚠️ 关键：从未见过 ≠ 失联。否则每次启动都会先来一次假警报。"""
    wd = StalenessWatchdog(timeout=2.0)
    assert wd.ever_seen is False
    assert wd.age(100.0) is None
    assert wd.expired(100.0) is False


def test_fresh_signal_does_not_expire():
    wd = StalenessWatchdog(timeout=2.0)
    wd.on_signal(10.0)
    assert wd.ever_seen is True
    assert wd.age(10.5) == pytest.approx(0.5)
    assert wd.expired(10.5) is False


def test_expires_after_timeout():
    wd = StalenessWatchdog(timeout=2.0)
    wd.on_signal(10.0)
    assert wd.expired(11.9) is False      # 还没到
    assert wd.expired(12.1) is True       # 过了


def test_boundary_is_not_expired():
    """正好等于阈值不算过期（">" 不是 ">="），与 D-025 里链路判定的口径一致。"""
    wd = StalenessWatchdog(timeout=2.0)
    wd.on_signal(0.0)
    assert wd.expired(2.0) is False


def test_signal_refreshes_and_clears_expiry():
    wd = StalenessWatchdog(timeout=2.0)
    wd.on_signal(0.0)
    assert wd.expired(5.0) is True
    wd.on_signal(5.0)                      # 又活了
    assert wd.expired(5.1) is False


def test_reset_goes_back_to_never_seen():
    wd = StalenessWatchdog(timeout=2.0)
    wd.on_signal(0.0)
    wd.reset()
    assert wd.ever_seen is False
    assert wd.expired(100.0) is False


def test_rejects_non_positive_timeout():
    with pytest.raises(ValueError):
        StalenessWatchdog(timeout=0.0)
    with pytest.raises(ValueError):
        StalenessWatchdog(timeout=-1.0)


def test_chronological_order_of_checks_does_not_matter():
    """看门狗只用 (now - last)，不看绝对的 now —— 调用方传单调钟即可，且换基准不影响判定。"""
    a = StalenessWatchdog(timeout=1.0)
    b = StalenessWatchdog(timeout=1.0)
    a.on_signal(0.0)
    b.on_signal(1000.0)
    assert a.expired(1.5) == b.expired(1001.5) is True
