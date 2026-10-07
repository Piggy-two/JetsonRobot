#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`MotorSafetyGate` 的离线单测（**不需要 ROS，也不需要底盘**）。

这些规则是安全依据，必须能离线证伪：本机没有物理急停（#22），
底盘又没有指令超时保护（D-020），跑起来"看着像对的"不算数。
"""
import pytest

from embodied_motor_driver.safety_gate import (
    LINK_ALIVE, LINK_LOST, LINK_STARTUP, MotorSafetyGate)


def make(**kw):
    """默认参数的安全门 + 一个已经"活"起来的底盘（否则什么都动不了）。

    ⚠️ 这里显式 `require_safety=False`：本文件下面绝大多数用例验的是**别的规则**
    （限幅 / 指令超时 / 存活判据 / 链路失联），安全层那一条会**盖住它们全部**
    （它在优先级最高处）。安全层本身的规则由本文件末尾专门的用例覆盖。
    """
    kw.setdefault('require_safety', False)
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
    g = MotorSafetyGate(require_safety=False)   # 连遥测都没有
    out = g.step(0.0)
    assert len(out) == 12
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
    g = MotorSafetyGate(require_safety=False)
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
    g = MotorSafetyGate(telemetry_timeout=1.0, require_safety=False)
    g.on_battery(5.0)
    g.on_cmd(0.2, 0.0, 0.0, now=5.0)
    assert g.step(5.2)[3] == MotorSafetyGate.STATE_OK


def test_odom_is_not_a_liveness_signal():
    """回归测试：本类**没有**任何接受 /odom 作为存活输入的方法。

    断线时 /odom 照发 28.5 Hz（D-021），把它当存活判据会让机器人在失联的底盘上继续下令。
    """
    assert not hasattr(MotorSafetyGate, 'on_odom')


def test_telemetry_loss_invalidates_stored_command():
    """存活恢复后**不得重放**失联前的旧指令——那会让车毫无预兆地窜一下。

    注意这里同时会触发「需重新使能」（失联时有一条非零指令），那是**更强**的保护，
    由 `test_rearm_blocks_motion_even_while_upstream_keeps_commanding` 单独覆盖。
    """
    g = make(cmd_timeout=5.0, telemetry_timeout=1.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    g.step(0.5)                                  # 先让链路进入 alive（否则一直是 startup，不算失联）
    g.step(3.0)                                  # 遥测超时 -> 失联 + 作废指令
    g.on_imu(3.1)                                # 底盘"回来了"
    g.on_battery(3.1)
    vx, _, _, state, age_cmd, *_ = g.step(3.2)
    assert vx == 0.0
    assert age_cmd is None                       # 指令确实被作废了，不是靠超时兜住的
    assert state == MotorSafetyGate.STATE_REARM_REQUIRED


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
    g = MotorSafetyGate(require_safety=False)
    g.stop()
    _, _, _, state, *_ = g.step(0.0)
    assert state == MotorSafetyGate.STATE_STOPPED


def test_state_codes_are_distinct():
    codes = MotorSafetyGate.STATE_CODES.values()
    assert len(set(codes)) == len(list(codes))


def test_step_returns_twelve_fields():
    """返回值是**位置约定**（status 话题直接照它填），所以字段数也要钉住。"""
    assert len(MotorSafetyGate().step(0.0)) == 12


# ---------- 链路状态机与「失联后需重新使能」（D-021 的安全窗口） ----------

def test_startup_is_not_reported_as_link_lost():
    """刚启动、从未收到过遥测时**不算失联** —— 否则每次开机都先来一次假警报。"""
    g = MotorSafetyGate(require_safety=False)
    out = g.step(10.0)
    assert out[7] == LINK_STARTUP
    assert out[3] == MotorSafetyGate.STATE_TELEMETRY_LOST   # 仍然输出零（安全）


def test_link_goes_lost_then_requires_rearm_if_it_was_moving():
    g = MotorSafetyGate(cmd_timeout=2.0, telemetry_timeout=1.0, require_safety=False)
    g.on_imu(0.0)
    g.on_battery(0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    assert g.step(0.5)[7] == LINK_ALIVE

    assert g.step(1.2)[7] == LINK_LOST                  # 遥测超时

    g.on_imu(1.3)
    g.on_battery(1.3)                                   # 链路回来了
    out = g.step(1.4)
    assert out[7] == LINK_ALIVE
    assert out[8] is True                               # 失联时它在动 -> 要求重新使能
    assert out[3] == MotorSafetyGate.STATE_REARM_REQUIRED
    assert out[:3] == (0.0, 0.0, 0.0)


def test_no_rearm_needed_if_it_was_idle_when_link_dropped():
    """失联前本来就是静止/零指令 —— 恢复后没有理由多要一次确认。"""
    g = MotorSafetyGate(cmd_timeout=2.0, telemetry_timeout=1.0, require_safety=False)
    g.on_imu(0.0)
    g.on_battery(0.0)
    g.on_cmd(0.0, 0.0, 0.0, now=0.0)
    g.step(0.5)
    assert g.step(1.2)[7] == LINK_LOST

    g.on_imu(1.3)
    g.on_battery(1.3)
    out = g.step(1.4)
    assert out[7] == LINK_ALIVE
    assert out[8] is False


def test_rearm_blocks_motion_even_while_upstream_keeps_commanding():
    """这条是重点：危险不是"旧指令被重放"，而是**上层一直在发**。

    链路一恢复，那条指令会立刻生效 -> 机器人毫无预兆地继续跑，
    而操作者正以为它是停着的、甚至可能正在搬它。
    """
    g = MotorSafetyGate(cmd_timeout=5.0, telemetry_timeout=1.0, require_safety=False)
    g.on_imu(0.0)
    g.on_battery(0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    g.step(0.5)
    g.step(1.2)                                         # 失联
    g.on_imu(1.3)
    g.on_battery(1.3)
    g.on_cmd(0.2, 0.0, 0.0, now=1.35)                   # 上层**一直在发**
    out = g.step(1.4)
    assert out[:3] == (0.0, 0.0, 0.0)
    assert out[3] == MotorSafetyGate.STATE_REARM_REQUIRED


def test_resume_clears_rearm_and_motion_may_continue():
    g = MotorSafetyGate(cmd_timeout=5.0, telemetry_timeout=1.0, require_safety=False)
    g.on_imu(0.0)
    g.on_battery(0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    g.step(0.5)
    g.step(1.2)
    g.on_imu(1.3)
    g.on_battery(1.3)
    g.step(1.4)
    assert g.needs_rearm

    g.resume()
    g.on_cmd(0.2, 0.0, 0.0, now=1.5)
    out = g.step(1.6)
    assert out[8] is False
    assert out[:3] == pytest.approx((0.2, 0.0, 0.0))


def test_stop_latch_cleared_by_resume_too():
    """resume() 同时清掉锁存与重新使能 —— 对操作者它们是同一个问题。"""
    g = make()
    g.stop()
    g.step(0.1)
    assert g.latched
    g.resume()
    assert not g.latched and not g.needs_rearm


# ---------- 安全层否决（D-037）：把否决权从「咨询性」变成「结构性」 ----------

def test_safety_never_seen_blocks_motion():
    """⚠️ 核心决策：安全层不在跑 = 不存在否决权 ⇒ **不许动**（不是"放行"）。"""
    g = make(require_safety=True)          # 从未收到过安全层状态
    g.on_cmd(0.2, 0.0, 0.0, now=0.1)
    vx, _, _, state, *_ = g.step(0.2)
    assert state == MotorSafetyGate.STATE_SAFETY_BLOCKED
    assert vx == 0.0
    assert g.safety_age(0.2) is None


def test_safety_latched_blocks_even_with_a_fresh_command():
    g = make(require_safety=True, safety_timeout=5.0, cmd_timeout=5.0)
    g.on_safety_status(True, 0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.1)
    vx, _, _, state, *_ = g.step(0.2)
    assert state == MotorSafetyGate.STATE_SAFETY_BLOCKED
    assert vx == 0.0


def test_safety_stale_blocks_motion():
    """安全层**停更**与"从未见过"同样按拦处理 —— 半死不活的安全层不算安全层。"""
    g = make(require_safety=True, safety_timeout=1.0, cmd_timeout=5.0)
    g.on_safety_status(False, 0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    assert g.step(0.5)[3] == MotorSafetyGate.STATE_OK
    assert g.step(1.5)[3] == MotorSafetyGate.STATE_SAFETY_BLOCKED    # 龄 1.5 > 1.0
    assert g.step(1.5)[0] == 0.0


def test_safety_boundary_is_still_fresh():
    """正好等于阈值算新鲜（与 D-025 的 `>` 口径一致）。"""
    g = make(require_safety=True, safety_timeout=1.0, cmd_timeout=5.0)
    g.on_safety_status(False, 0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.0)
    assert g.step(1.0)[3] == MotorSafetyGate.STATE_OK
    assert g.step(1.01)[3] == MotorSafetyGate.STATE_SAFETY_BLOCKED


def test_require_safety_false_never_blocks():
    """台架实验的显式放行口：关掉它，本节点行为回到 D-025 那一版。"""
    g = make(require_safety=False, cmd_timeout=5.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.1)
    vx, _, _, state, *_ = g.step(0.2)
    assert state == MotorSafetyGate.STATE_OK
    assert vx == pytest.approx(0.2)
    assert g.safety_blocked is False


def test_safety_outranks_the_local_latch_in_the_reported_state():
    """`Safety > Control`：两个都拦着时，报出的是安全层那个原因。"""
    g = make(require_safety=True, safety_timeout=5.0)
    g.on_safety_status(True, 0.0)
    g.stop()
    assert g.step(0.2)[3] == MotorSafetyGate.STATE_SAFETY_BLOCKED
    g.on_safety_status(False, 0.3)
    assert g.step(0.3)[3] == MotorSafetyGate.STATE_STOPPED


def test_safety_loss_while_commanding_requires_rearm_on_return():
    """被安全层拦下时"底盘可能还在动" ⇒ 它回来后**不能自己接着跑**（D-025 的同一理由）。"""
    g = make(require_safety=True, safety_timeout=0.5, cmd_timeout=5.0)
    g.on_safety_status(False, 0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.1)
    assert g.step(0.2)[3] == MotorSafetyGate.STATE_OK

    vx, _, _, state, *_ = g.step(1.0)          # 安全层停更 1.0 s > 0.5
    assert state == MotorSafetyGate.STATE_SAFETY_BLOCKED
    assert vx == 0.0

    g.on_safety_status(False, 1.1)             # 安全层回来了，且**没有**锁存
    vx, _, _, state, *_ = g.step(1.1)
    assert state == MotorSafetyGate.STATE_REARM_REQUIRED
    assert vx == 0.0


def test_resume_clears_the_safety_induced_rearm():
    g = make(require_safety=True, safety_timeout=0.5, cmd_timeout=5.0)
    g.on_safety_status(False, 0.0)
    g.on_cmd(0.2, 0.0, 0.0, now=0.1)
    g.step(1.0)
    g.on_safety_status(False, 1.1)
    assert g.step(1.1)[3] == MotorSafetyGate.STATE_REARM_REQUIRED
    g.resume()
    vx, _, _, state, *_ = g.step(1.2)
    assert state == MotorSafetyGate.STATE_OK
    assert vx == pytest.approx(0.2)


def test_safety_block_without_a_command_returns_without_rearm():
    """本来就没在动 —— 不该多要一次确认（否则每次启动都会人烦一次）。"""
    g = make(require_safety=True, safety_timeout=0.5, cmd_timeout=5.0)
    g.on_safety_status(False, 0.0)
    assert g.step(1.0)[3] == MotorSafetyGate.STATE_SAFETY_BLOCKED
    g.on_safety_status(False, 1.1)
    g.on_cmd(0.2, 0.0, 0.0, now=1.15)
    vx, _, _, state, *_ = g.step(1.2)
    assert state == MotorSafetyGate.STATE_OK
    assert vx == pytest.approx(0.2)


def test_safety_properties_reflect_the_last_message():
    g = make(require_safety=True)
    g.on_safety_status(True, 3.0)
    assert g.safety_latched is True
    assert g.safety_age(3.4) == pytest.approx(0.4)
    g.on_safety_status(False, 4.0)
    assert g.safety_latched is False
