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

规则（优先级从高到低）：

  1. **锁存停车** —— 一旦被要求停，必须先显式 `resume()` 才能再动。
  2. **失联后需重新使能** —— 见下面「链路状态机」。
  3. **底盘存活**（D-021）—— 存活判据**只能用 `imu_raw` / `battery` 的到达时刻**，
     绝不能用 `/odom`（断线时它照发 28.5 Hz）也不能用 `pgrep`。
  4. **指令超时**（D-020）—— 上层停发指令 ≠ 停车。超过 `cmd_timeout` 未收到新指令即输出零。
  5. **限幅** —— 发布前再钳一次（纵深防御，`/controller/cmd_vel` 那条**完全没钳**，#19）。

**链路状态机（startup / alive / lost）与「失联后需重新使能」**

D-021 实测：USB 断线后**重新 bind 不自恢复**，必须重启厂商服务；而这段窗口里
**底盘保持着最后一条速度指令**（D-020）—— 所以「检测到失联」和「重新可控」之间存在
一个**秒级安全窗口**。

本类只负责这个窗口的**策略侧**（重启系统服务需要 root，Driver 自己做不到）：

    startup ──首次收到遥测──→ alive ──遥测超时──→ lost ──遥测恢复──→ alive
                                  ↑                                    │
                                  └────── 若失联时底盘"可能还在动" ─────┘
                                          则置位 needs_rearm，必须显式 rearm()

**为什么失联恢复后要重新使能：** 危险不在于"指令会被重放"（存指令在失联时已被作废），
而在于**上层可能一直在以 20 Hz 发布同一条指令**。链路一恢复，那条指令立刻生效 →
机器人会在链路回来的一瞬间**毫无预兆地继续跑**，而操作者此刻正以为它是"停着的"，
甚至可能正在搬它。**所以必须由人（或上层）显式确认一次。**

只在"**失联时底盘可能还在动**"时才要求重新使能 —— 即失联那一刻存在一条**未超时且非零**
的指令。失联前本来就是静止/无指令，恢复后没有理由多要一次确认。

⚠️ 一个贯穿全类的性质：**`step()` 永远返回一个速度**，绝不返回"什么都不发"。
这正是 D-020 的教训——底盘保持最后一条指令，**停止发布 ≠ 停车**，
所以我们必须**每个周期都主动发**（哪怕是零）。

时间一律用**单调钟**（调用方传 `time.monotonic()`）：系统实时钟在这台机器上会被
NTP 跳跃（#24 实测跳了 338.6 s），拿它算超时会瞬间失效。
"""

# 链路状态
LINK_STARTUP = 'startup'    # 从未收到过遥测（刚启动）
LINK_ALIVE = 'alive'
LINK_LOST = 'lost'


class MotorSafetyGate:
    # 状态码（同时用于 status 话题，故取数值）
    STATE_OK = 'ok'
    STATE_NO_CMD = 'no_cmd'
    STATE_TELEMETRY_LOST = 'telemetry_lost'
    STATE_STOPPED = 'stopped'
    STATE_REARM_REQUIRED = 'rearm_required'

    STATE_CODES = {
        STATE_OK: 0.0,
        STATE_NO_CMD: 1.0,
        STATE_TELEMETRY_LOST: 2.0,
        STATE_STOPPED: 3.0,
        STATE_REARM_REQUIRED: 4.0,
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

        self._link = LINK_STARTUP
        self._needs_rearm = False
        self._left_moving = False   # 失联发生的那一刻，底盘是否"可能还在动"

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

    # ---------- 锁存停车 / 重新使能 ----------

    def stop(self):
        """锁存停车：此后一直输出零，直到显式 resume()。"""
        self._latched = True

    def resume(self):
        """解除锁存停车 **并** 清掉"失联后需重新使能"。

        两者合成一个动作是**刻意**的：对操作者来说它们回答的是同一个问题——
        "我已经看过了，可以继续"。日志里会分别说明清掉了什么。
        """
        self._latched = False
        self._needs_rearm = False

    @property
    def latched(self):
        return self._latched

    # ---------- 链路状态 ----------

    @property
    def link(self):
        return self._link

    @property
    def needs_rearm(self):
        return self._needs_rearm

    def _update_link(self, now, age_cmd, age_imu, age_batt):
        """更新链路状态机。返回 True 表示遥测当前是新鲜的。"""
        ages = [a for a in (age_imu, age_batt) if a is not None]
        # 存活：取 imu / battery 里**较新的那个**。
        # 任一在阈值内就算底盘在；两个都没到过、或最新的也超时，即判定失联。
        fresh = bool(ages) and min(ages) <= self.telemetry_timeout

        if fresh:
            if self._link == LINK_LOST:
                self._needs_rearm = self._left_moving
                self._left_moving = False
            self._link = LINK_ALIVE
            return True

        if self._link == LINK_ALIVE:
            # 刚刚失联：记住那一刻底盘"是否可能还在动"。
            # 判据 = 存在一条**未超时且非零**的指令（为零则底盘本来就是停的）。
            cmd_active = (age_cmd is not None and age_cmd <= self.cmd_timeout
                          and any(abs(c) > 1e-9 for c in self._cmd))
            self._left_moving = cmd_active
            self._link = LINK_LOST
        # startup 且从未收到过遥测：不算"失联"（刚启动时本来就没有），
        # 否则每次启动都会先报一次假警报。
        return False

    # ---------- 求值 ----------

    def step(self, now):
        """本周期该发什么。

        :return: (vx, vy, wz, state, age_cmd, age_imu, age_batt, link, needs_rearm)
                 age_* 为秒；未收到过该信号时为 None。
                 link ∈ {'startup', 'alive', 'lost'}；needs_rearm 为 bool。
        """
        age_cmd = None if self._cmd_time is None else (now - self._cmd_time)
        age_imu = None if self._imu_time is None else (now - self._imu_time)
        age_batt = None if self._battery_time is None else (now - self._battery_time)

        fresh = self._update_link(now, age_cmd, age_imu, age_batt)

        def out(state, vx=0.0, vy=0.0, wz=0.0):
            return (vx, vy, wz, state, age_cmd, age_imu, age_batt,
                    self._link, self._needs_rearm)

        if self._latched:
            return out(self.STATE_STOPPED)

        if self._needs_rearm:
            # 失联期间底盘保着最后一条速度（D-020），恢复后不能自己接着跑 ——
            # 必须等显式 rearm()。
            return out(self.STATE_REARM_REQUIRED)

        if not fresh:
            # ⚠️ 顺手作废已存指令：否则存活恢复的瞬间会把一条旧指令重放出去。
            self._cmd_time = None
            return out(self.STATE_TELEMETRY_LOST)

        if age_cmd is None or age_cmd > self.cmd_timeout:
            return out(self.STATE_NO_CMD)

        vx, vy, wz = self._cmd
        return out(self.STATE_OK, vx, vy, wz)

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
