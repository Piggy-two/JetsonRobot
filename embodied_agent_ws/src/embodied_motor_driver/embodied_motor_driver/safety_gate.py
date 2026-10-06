#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motor Driver 的安全整形逻辑（**纯 Python，不依赖 ROS**）。

为什么单独成文件：本项目**没有物理急停**（#22 / D-021），底盘又**没有指令超时保护**
（#20 / D-020）。也就是说"该不该动、该不该停"这些判断是**唯一的安全依据**，
不能让它们散在 ROS 回调里靠"跑起来看着像对的"。抽出来就能**离线单测**，
每条规则都能被证伪。

本类只回答一个问题：
    给定「上层最新指令」「底盘存活信号的到达时刻」「当前时刻」，
    本周期**应该**往 `/cmd_vel` 发什么？

四条规则（优先级从高到低）：

  1. **锁存停车（latched stop）** —— 一旦被要求停，必须先显式 `resume()` 才能再动。
     抗"下游还没来得及处理就被下一条指令覆盖"。
  2. **底盘存活**（D-021）—— 存活判据**只能用 `imu_raw` / `battery` 的到达时刻**，
     绝不能用 `/odom`（断线时它照发 28.5 Hz）也不能用 `pgrep`。
     存活信号超时 → 输出零，**并且作废已存的指令**（防止存活恢复后把旧指令重放出去，
     那会让车毫无预兆地窜一下）。
  3. **指令超时**（D-020）—— 上层停发指令 ≠ 停车。超过 `cmd_timeout` 未收到新指令即输出零。
  4. **限幅** —— 发布前再钳一次。厂商 `odom_publisher` 对 `/cmd_vel` 也钳
     （±0.2 m/s / ±0.5 rad/s），但那只是**一道**；且 `/controller/cmd_vel` 那条**完全没钳**（#19）。
     自己再钳一次是纵深防御，不依赖厂商实现对不对。

⚠️ 一个贯穿全类的性质：**`step()` 永远返回一个速度**，绝不返回"什么都不发"。
这正是 D-020 的教训——底盘保持最后一条指令，**停止发布 ≠ 停车**，
所以我们必须**每个周期都主动发**（哪怕是零）。

时间一律用**单调钟**（调用方传 `time.monotonic()`）：系统实时钟在这台机器上会被
NTP 跳跃（#24 实测跳了 338.6 s），拿它算超时会瞬间失效。
"""


class MotorSafetyGate:
    # 状态码（同时用于 status 话题，故取数值）
    STATE_OK = 'ok'
    STATE_NO_CMD = 'no_cmd'
    STATE_TELEMETRY_LOST = 'telemetry_lost'
    STATE_STOPPED = 'stopped'

    STATE_CODES = {
        STATE_OK: 0.0,
        STATE_NO_CMD: 1.0,
        STATE_TELEMETRY_LOST: 2.0,
        STATE_STOPPED: 3.0,
    }

    def __init__(self, max_vx=0.2, max_vy=0.2, max_wz=0.5,
                 cmd_timeout=0.5, telemetry_timeout=2.5):
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_wz = float(max_wz)
        self.cmd_timeout = float(cmd_timeout)
        self.telemetry_timeout = float(telemetry_timeout)

        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = None
        self._imu_time = None
        self._battery_time = None
        self._latched = False

    # ---------- 输入 ----------

    def on_cmd(self, vx, vy, wz, now):
        """收到上层指令。返回**钳制后**的 (vx, vy, wz)，便于上层发现被限幅。"""
        vx = self._clamp(vx, self.max_vx)
        vy = self._clamp(vy, self.max_vy)
        wz = self._clamp(wz, self.max_wz)
        self._cmd = (vx, vy, wz)
        self._cmd_time = now
        return self._cmd

    def on_imu(self, now):
        """底盘 IMU 遥测到达（存活判据之一）。"""
        self._imu_time = now

    def on_battery(self, now):
        """底盘电池遥测到达（存活判据之一）。"""
        self._battery_time = now

    # ---------- 锁存停车 ----------

    def stop(self):
        """锁存停车：此后一直输出零，直到显式 resume()。"""
        self._latched = True

    def resume(self):
        """解除锁存。**必须显式调用**，这是刻意的。"""
        self._latched = False

    @property
    def latched(self):
        return self._latched

    # ---------- 求值 ----------

    def step(self, now):
        """本周期该发什么。

        :return: (vx, vy, wz, state, age_cmd, age_imu, age_batt)
                 age_* 为秒；未收到过该信号时为 None。
        """
        age_cmd = None if self._cmd_time is None else (now - self._cmd_time)
        age_imu = None if self._imu_time is None else (now - self._imu_time)
        age_batt = None if self._battery_time is None else (now - self._battery_time)

        if self._latched:
            return (0.0, 0.0, 0.0, self.STATE_STOPPED,
                    age_cmd, age_imu, age_batt)

        # 存活：取 imu / battery 里**较新的那个**。
        # 任一在阈值内就算底盘在；两个都没到过、或最新的也超时，即判定失联。
        ages = [a for a in (age_imu, age_batt) if a is not None]
        if not ages or min(ages) > self.telemetry_timeout:
            # ⚠️ 顺手作废已存指令：否则存活恢复的瞬间会把一条旧指令重放出去，
            #    车会毫无预兆地窜一下（而这是"恢复"时刻，人最容易放松警惕）。
            self._cmd_time = None
            return (0.0, 0.0, 0.0, self.STATE_TELEMETRY_LOST,
                    age_cmd, age_imu, age_batt)

        if age_cmd is None or age_cmd > self.cmd_timeout:
            return (0.0, 0.0, 0.0, self.STATE_NO_CMD,
                    age_cmd, age_imu, age_batt)

        vx, vy, wz = self._cmd
        return (vx, vy, wz, self.STATE_OK, age_cmd, age_imu, age_batt)

    # ---------- 内部 ----------

    @staticmethod
    def _clamp(value, limit):
        limit = abs(limit)
        if value != value:          # NaN —— 宁可当 0，也不要把它喂给电机
            return 0.0
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return float(value)

    def state_code(self, state):
        return self.STATE_CODES.get(state, 9.0)
