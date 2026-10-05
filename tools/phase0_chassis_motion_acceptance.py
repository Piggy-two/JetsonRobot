#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 0 底盘运动验收脚本（bench acceptance harness，非运行时组件）。

用途：在【人工在场 + 人工看护】条件下，验证底盘运动链路与安全边界，
并把结果填进 `docs/PROJECT_STATUS.md` §7 接口清单。

定位说明（重要）：
  本脚本是**验收工装**，不是机器人的控制路径。按 D-002 / D-016，运行时只有
  Control Skill 允许发布 `/cmd_vel`，且必须经 Safety Gateway。本脚本仅用于
  Phase 0 人工监督下的实机验收，**不得**被 Agent Runtime 或任何 Skill 调用。

安全设计：
  1. 入口只用 `/cmd_vel` —— `odom_publisher` 的 `app_cmd_vel_callback` 对它硬限幅
     （linear.x/y ±0.2 m/s，angular.z ±0.5 rad/s）。**绝不**用 `/controller/cmd_vel`
     （`cmd_vel_callback` 路径，无任何限幅，且 5 个厂商 app 挂在其上）。
  2. 脚本自身再加一层更低的硬限速 HARD_MAX_*，双重保险。
  3. 任何退出路径（正常结束 / 异常 / SIGINT / SIGTERM / 看门狗）都显式发布 0 速度，
     并保持 2s。**原因：底盘固件没有任何指令超时保护**（见 D-020）——
     不发 0，电机会一直保持最后一条速度指令。
  4. 总运行时长看门狗。

证据强度说明：
  `/odom` 的 twist 由 `odom_publisher` 用【指令值】直接积分得到（纯死推算），
  只能证明指令链路通，**不能**证明轮子真的转了。`/odom_raw` 的 wz 同理。
  轮子是否真正转动，以 (a) 现场目视 与 (b) IMU 陀螺 z（`/ros_robot_controller/imu_raw`，
  真实物理量，本机实测零偏 +0.009 rad/s、std 0.0008）为准。

用法：
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 tools/phase0_chassis_motion_acceptance.py --yes
    python3 tools/phase0_chassis_motion_acceptance.py --yes --precheck-only
"""
import argparse
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import Imu

HARD_MAX_VX = 0.15      # m/s   —— 脚本层硬限速（厂商层为 0.2）
HARD_MAX_WZ = 0.40      # rad/s —— 脚本层硬限速（厂商层为 0.5）
OVERRIDE_MAX_VX = 0.50  # 仅限 --allow-overlimit 时的放行上限（用于验证厂商限幅）
PUB_HZ = 20.0
WATCHDOG_S = 180.0
STEP_D = 2.0

STEPS = [
    ('T1 前进  +0.10 m/s',   0.10, 0.0, 0.0),
    ('T2 后退  -0.10 m/s',  -0.10, 0.0, 0.0),
    ('T3 左移  +0.10 m/s',   0.00, 0.10, 0.0),
    ('T4 右移  -0.10 m/s',   0.00, -0.10, 0.0),
    ('T5 左转  +0.30 rad/s', 0.00, 0.0, 0.30),
    ('T6 右转  -0.30 rad/s', 0.00, 0.0, -0.30),
]


class ChassisTester(Node):
    def __init__(self):
        super().__init__('phase0_chassis_acceptance')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Odometry, '/odom', self._on_odom, 1)
        self.create_subscription(Odometry, '/odom_raw', self._on_odom_raw, 1)
        self.create_subscription(MotorsState, '/ros_robot_controller/set_motor',
                                 self._on_motor, 10)
        self.create_subscription(Imu, '/ros_robot_controller/imu_raw',
                                 self._on_imu, 1)
        self.odom = None
        self.odom_raw = None
        self.imu = None
        self.motor_rps = None
        self.motor_msgs = 0
        self.motor_window = 0
        self.allow_over = False

    def _on_odom(self, m):
        self.odom = m

    def _on_odom_raw(self, m):
        self.odom_raw = m

    def _on_motor(self, m):
        self.motor_rps = [(d.id, round(d.rps, 4)) for d in m.data]
        self.motor_msgs += 1
        self.motor_window += 1

    def _on_imu(self, m):
        self.imu = m

    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        cap_v = OVERRIDE_MAX_VX if self.allow_over else HARD_MAX_VX
        if abs(vx) > cap_v + 1e-9 or abs(vy) > cap_v + 1e-9 \
                or abs(wz) > HARD_MAX_WZ + 1e-9:
            raise ValueError(f'speed exceeds script hard limit: {vx},{vy},{wz}')
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def spin_once(self, timeout=0.01):
        rclpy.spin_once(self, timeout_sec=timeout)

    def stop(self, seconds=1.0):
        """显式发 0 并保持 —— 底盘无 failsafe，这是唯一的停车手段。"""
        end = time.time() + seconds
        while time.time() < end:
            self.publish(0.0, 0.0, 0.0)
            self.spin_once()
            time.sleep(1.0 / PUB_HZ)


def _tw(msg):
    if msg is None:
        return None
    t = msg.twist.twist
    return (t.linear.x, t.linear.y, t.angular.z)


def _avg(seq, i):
    return sum(s[i] for s in seq) / len(seq) if seq else float('nan')


def _mean(seq):
    return sum(seq) / len(seq) if seq else float('nan')


def _span(seq):
    return (max(seq) - min(seq)) / 2.0 if seq else float('nan')


def hold(node, vx, vy, wz, duration):
    """持续发布 (vx,vy,wz) 共 duration 秒，返回后段实测均值。"""
    node.motor_window = 0
    od, raw, gyro = [], [], []
    t_end = time.time() + duration
    while time.time() < t_end:
        node.publish(vx, vy, wz)
        node.spin_once()
        s = _tw(node.odom)
        if s:
            od.append(s)
        if node.odom_raw is not None:
            raw.append(node.odom_raw.twist.twist.angular.z)
        if node.imu is not None:
            gyro.append(node.imu.angular_velocity.z)
        time.sleep(1.0 / PUB_HZ)
    cut = len(od) // 3
    return {
        'odom': (_avg(od[cut:], 0), _avg(od[cut:], 1), _avg(od[cut:], 2)),
        'odom_raw_wz': _mean(raw[len(raw) // 3:]),
        'imu_wz': _mean(gyro[len(gyro) // 3:]),
        'imu_span': _span(gyro[len(gyro) // 3:]),
        'imu_n': len(gyro),
        'nmsg': node.motor_window,
        'rps': node.motor_rps,
    }


def silence(node, duration, drain=1.5):
    """停止发布，观察底盘是否自行停车（验证是否存在指令超时保护）。

    先空转 drain 秒排空订阅队列积压（此期间同样不发布），再清零计数器正式开始测量。
    返回的 imu_span（振荡幅度）可区分"电机在转（振动宽）"与"已停（振动窄）"。
    """
    t_drain = time.time() + drain
    while time.time() < t_drain:
        node.spin_once(timeout=0.02)
    node.motor_window = 0
    od, gyro = [], []
    t_end = time.time() + duration
    while time.time() < t_end:
        node.spin_once(timeout=0.02)
        s = _tw(node.odom)
        if s:
            od.append(s)
        if node.imu is not None:
            gyro.append(node.imu.angular_velocity.z)
    return {
        'odom': (_avg(od[len(od) // 2:], 0), _avg(od[len(od) // 2:], 1),
                 _avg(od[len(od) // 2:], 2)),
        'imu_wz': _mean(gyro),
        'imu_span': _span(gyro),
        'nmsg': node.motor_window,
    }


def report(label, cmd, r):
    vx, vy, wz = cmd
    print(f'    指令      : vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}')
    print(f'    /odom     : vx={r["odom"][0]:+.4f} vy={r["odom"][1]:+.4f} '
          f'wz={r["odom"][2]:+.4f}   (死推算)')
    print(f'    /odom_raw : wz={r["odom_raw_wz"]:+.4f}   (原始，无 EKF)')
    print(f'    IMU 陀螺 z: {r["imu_wz"]:+.4f} ± {r["imu_span"]:.4f} rad/s '
          f'(n={r["imu_n"]})  ← 真实物理量')
    print(f'    set_motor : {r["nmsg"]} 条   轮速(rps)={r["rps"]}')


_stop_now = threading.Event()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yes', action='store_true', help='确认已有人看护、场地安全')
    ap.add_argument('--skip-clamp', action='store_true')
    ap.add_argument('--skip-timeout', action='store_true')
    ap.add_argument('--precheck-only', action='store_true')
    args = ap.parse_args()
    if not args.yes:
        print('拒绝运行：必须显式传入 --yes 确认有人看护。')
        return 2

    rclpy.init()
    node = ChassisTester()

    def _sig(signum, frame):
        _stop_now.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    def watchdog():
        if not _stop_now.wait(WATCHDOG_S):
            print(f'\n!! 看门狗超时 {WATCHDOG_S}s，强制停车 !!')
            _stop_now.set()
    threading.Thread(target=watchdog, daemon=True).start()

    t0 = time.time()
    n = 0
    try:
        print('=== 预检 ===')
        for _ in range(200):
            node.spin_once()
            if node.odom is not None and node.imu is not None:
                break
        print(f'  /odom={node.odom is not None}  imu={node.imu is not None}')
        node.stop(1.5)
        print(f'  静置 set_motor={node.motor_msgs} 条（>0 说明零命令链路通）\n')

        if args.precheck_only:
            print('--precheck-only：退出。')
            return 0

        for label, vx, vy, wz in STEPS:
            if _stop_now.is_set():
                break
            print(f'--- {label} ---')
            report(label, (vx, vy, wz), hold(node, vx, vy, wz, STEP_D))
            n += 1
            node.stop(1.0)
            print('    已停车\n')

        if not args.skip_clamp and not _stop_now.is_set():
            print('--- T7 限幅验证：指令 vx=+0.30（高于厂商上限 0.2）---')
            node.allow_over = True
            r = hold(node, 0.30, 0.0, 0.0, 1.5)
            node.allow_over = False
            report('T7', (0.30, 0.0, 0.0), r)
            got = r['odom'][0]
            if abs(got - 0.20) < 0.03:
                print('    => 厂商限幅生效：指令 0.30 被钳到 ≈0.20 ✅')
            elif abs(got - 0.30) < 0.03:
                print('    => 未限幅！指令原样透传 ⚠️')
            else:
                print(f'    => 结果不明确（{got:+.4f}）')
            n += 1
            node.stop(1.0)
            print('    已停车\n')

        if not args.skip_timeout and not _stop_now.is_set():
            print('--- T8 指令超时验证：发 1.5s 速度后【完全停止发布】 ---')
            hold(node, 0.10, 0.0, 0.0, 1.5)
            r = silence(node, 4.0)
            print(f'    静默 4s 期间 set_motor={r["nmsg"]} 条（0=无新指令）')
            print(f'    静默末段 /odom: vx={r["odom"][0]:+.4f} wz={r["odom"][2]:+.4f}'
                  f'   (死推算，保持最后指令属预期)')
            print(f'    静默末段 IMU 陀螺 z: {r["imu_wz"]:+.4f} ± {r["imu_span"]:.4f} rad/s')
            if r['imu_span'] > 0.02:
                print('    => 振荡幅度宽 → 电机【仍在转】：固件无 failsafe ⚠️ '
                      '必须始终显式发 0')
            else:
                print('    => 振荡幅度窄 → 电机已停：存在超时保护')
            n += 1
            node.stop(1.0)
            print('    已停车\n')

    except Exception as e:
        print(f'\n!! 异常：{type(e).__name__}: {e} —— 立即停车 !!')
    finally:
        print('=== 收尾：发 2s 零速度 ===')
        try:
            node.stop(2.0)
        except Exception as e:
            print(f'  收尾发 0 失败: {e}')
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print(f'\n总耗时 {time.time() - t0:.1f}s，共 {n} 步')
    return 0


if __name__ == '__main__':
    sys.exit(main())
