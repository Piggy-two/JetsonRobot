#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`MotorSafetyGate` 的离线单测（**不需要 ROS，也不需要底盘**）。

这些规则是安全依据，必须能离线证伪：本机没有物理急停（#22），
底盘又没有指令超时保护（D-020），跑起来"看着像对的"不算数。
"""
import pytest

from embodied_motor_driver.safety_gate import MotorSafetyGate


def make(**kw):
    """默认参数的安全门 + 一个已经"活"起来的底盘（否则什么都动不了）。"""
    g = MotorSafetyGate(**kw)
    g.on_imu(0.0)
    g.on_battery(0.0)
    return g


# ---------- 限幅 ----------

def test_clamps_to_limits():
    g = make(max_vx=0.2, max_vy=0.2, max_wz=0.5)
    assert g.on_cmd(0.9, -0.9, 3.0, 0.1) == pytest.approx((0.2, -0.2, 0.5))


def test_within_limits_untouched():
    g = make()
    assert g.on_cmd(0.11, -0.07, 0.3, 0.1) == pytest.approx((0.11, -0.07, 0.3))


def test_nan_becomes_zero():
    """NaN 绝不喂给电机：宁可当 0。"""
    g = make()
    vx, vy, wz = g.on_cmd(float('nan'), 0.1, float('nan'), 0.0)
    assert (vx, wz) == (0.0, 0.0)
    assert vy == pytest.approx(0.1)


# ---------- D-020：必须**每个周期都发**，且超时归零 ----------

def test_step_always_returns_a_velocity():
    """核心不变量：step() 永不返回"什么都不发"。底盘保持最后一条指令，不发 != 停。"""
    g = MotorSafetyGate()          # 连遥测都没有
    out = g.step(0.0)
    assert len(out) == 7
    assert out[:3] == (0.0, 0.0, 0.0)


def test_cmd_timeout_zeroes_output():
    # telemetry_timeout 放大，把这几个断言**隔离**到"指令超时"这一条上
    g = make(cmd_timeout=0.5, telemetry_timeout=10.0)
    g.on_cmd(0.2, 0.0, 0.0, now=1.0)
    vx, vy, wz, state, *_ = g.step(1.2)
    assert (vx, vy, wz) == pytest.approx((0.2, 0.0, 0.0))
    assert state == MotorSafetyGate.STATE_OK
    vx, vy, wz, state, *_ = g.step(1.6)                             # 龄 0.6 > 0.5
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert state == MotorSafetyGate.STATE_NO_CMD


# ---------- D-021：存活判据 ----------

def test_no_telemetry_means_lost_and_zero():
    g = MotorSafetyGate()
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    vx, _, _, state, *_ = g.step(0.1)
    assert vx == 0.0 and state == MotorSafetyGate.STATE_TELEMETRY_LOST


def test_stale_telemetry_means_lost():
    g = make(telemetry_timeout=1.0)
    g.on_cmd(0.2, 0.0, 0.0, now=1.0)
    _, _, _, state, *_ = g.step(3.0)        # 遥测停在 0.0，已 3 s
    assert state == MotorSafetyGate.STATE_TELEMETRY_LOST


def test_telemetry_alive_if_either_signal_is_fresh():
    """imu 停了但 battery 还在 -> 仍算底盘在线（取较新的那个）。"""
    g = MotorSafetyGate(telemetry_timeout=1.0)
    g.on_battery(5.0)
    g.on_cmd(0.2, 0.0, 0.0, now=5.0)
    assert g.step(5.2)[3] == MotorSafetyGate.STATE_OK


def test_odom_is_not_a_liveness_signal():
    """回归测试：本类**没有**任何接受 /odom 作为存活输入的方法。

    断线时 /odom 照发 28.5 Hz（D-021），把它当存活判据会让机器人在失联的底盘上继续下令。
    """
    assert not hasattr(MotorSafetyGate, 'on_odom')


def test_telemetry_loss_invalidates_stored_command():
    """存活恢复后**不得重放**失联前的旧指令——那会让车毫无预兆地窜一下。"""
    g = make(cmd_timeout=5.0, telemetry_timeout=1.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    g.step(3.0)                                  # 遥测早就超时 -> 作废指令
    g.on_imu(3.1)                                # 底盘"回来了"
    g.on_battery(3.1)
    vx, _, _, state, age_cmd, _, _ = g.step(3.2)
    assert vx == 0.0
    assert state == MotorSafetyGate.STATE_NO_CMD
    assert age_cmd is None                       # 指令确实被作废了，不是靠超时兜住的


# ---------- 锁存停车 ----------

def test_latched_stop_overrides_fresh_command():
    g = make()
    g.on_cmd(0.2, 0.0, 0.0, now=1.0)
    g.stop()
    vx, _, _, state, *_ = g.step(1.1)
    assert vx == 0.0 and state == MotorSafetyGate.STATE_STOPPED
    assert g.latched


def test_resume_allows_motion_again():
    g = make(telemetry_timeout=10.0)
    g.stop()
    g.step(1.0)
    g.resume()
    g.on_cmd(0.2, 0.0, 0.0, now=1.1)
    vx, vy, wz, state, *_ = g.step(1.2)
    assert (vx, vy, wz) == pytest.approx((0.2, 0.0, 0.0))
    assert state == MotorSafetyGate.STATE_OK


def test_stop_outranks_telemetry_loss():
    g = MotorSafetyGate()
    g.stop()
    _, _, _, state, *_ = g.step(0.0)
    assert state == MotorSafetyGate.STATE_STOPPED


def test_state_codes_are_distinct():
    codes = MotorSafetyGate.STATE_CODES.values()
    assert len(set(codes)) == len(list(codes))
