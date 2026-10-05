#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_rotation_probe.py —— 用 LiDAR 测量机器人【原地旋转角速度】的验收工装。

    ⚠️ 这是 Phase 0 的【验收/验证工装】，不是运行时组件。
       不得被 Agent Runtime、任何 Skill 或 Safety Runtime 调用。

为什么需要它
------------
验证「机器人到底转没转」时，常见的三个证据源都不可靠：

  · `/odom`      —— `odom_publisher` 直接对【订阅到的指令值】积分，纯死推算。
                    指令发出去它就有读数，哪怕轮子没动、哪怕串口已断。
  · `IMU`        —— 是独立物理量，可信；但它与桥节点 `ros_robot_controller`
                    【同生共死】：桥节点一挂/一被冻结，imu_raw 立刻消失。
                    想测「指令断了板子还转不转」时，它恰好是最先瞎掉的那个。
  · 目视轮子      —— 旋转中的麦轮看不出转没转，实测两次都「没看清楚」。

LiDAR 走【独立的 USB 口（1-2.1）与独立进程】，桥节点死了它照常出数据，
所以它可以作为「机器人是否在动」的独立仪器。

原理
----
LiDAR 固连在机器人上。机器人原地转 θ，扫描图相对传感器就整体平移 θ。
对相邻（或首末）两帧 `ranges` 数组做互相关，找到使逐点差最小的索引位移 k，
则转角 = k × `angle_increment`。

丢帧免疫
--------
本工装【不】用「相邻帧位移 ÷ 固定扫描周期」求速率 —— 订阅端一旦丢帧，
prev 就不是真正的相邻帧，数值会被悄悄缩放（实测踩过这个坑）。
改为两种互相独立的方法求【总转角】，只依赖被选中的帧本身：

  1. 直接法：整段的首帧 vs 末帧，一次互相关
  2. 链式法：按 ~1 s 步长逐段互相关，再累加

两法差值即自检。同时 LiDAR 每帧点数会浮动（实测 503/504/505），
故只使用【出现次数最多的长度】的帧做互相关（互相关要求两帧等长）。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    # 测 6 秒，输出总转角与平均角速度
    python3 tools/lidar_rotation_probe.py --duration 6

    # 判定「是否静止」（转速绝对值 < 0.01 rad/s 视为静止）
    python3 tools/lidar_rotation_probe.py --duration 6 --expect-stationary

    # 判定「是否在按给定速度转」（±30% 容差）
    python3 tools/lidar_rotation_probe.py --duration 6 --expect-rate 0.15

退出码：0 = 判定通过 / 仅测量；1 = --expect-* 判定失败。

限制
----
  · 需要环境有足够结构（墙、家具）。空旷场地上各方向距离相近，互相关无峰值。
  · 整段转角必须 < π（本工装靠互相关，超过就会混叠）。6 s @ 0.15 rad/s ≈ 0.9 rad，安全。
    转速高时请缩短 --duration。
  · 只测【原地旋转】。纯平移（vx/vy）不改变扫描图的整体角度，本工装测不到。
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

FAR_SHIFT = 150      # 直接法索引搜索范围（150 点 ≈ 1.87 rad，真值 < π 即不混叠）
NEAR_SHIFT = 40      # 链式法每段搜索范围
CHAIN_GAP = 1.0      # 链式法每段目标时长 (s)
STATIC_EPS = 0.01    # |转速| < 该值视为静止 (rad/s)


def best_shift(a, b, rng):
    """在 ±rng 内找使 |a[i+k] - b[i]| 最小的索引位移 k。

    返回 (k, 该位移下的平均绝对差)。找不到足够重叠时返回 (0, inf)。
    """
    best_k, best_c = 0, float('inf')
    n = len(a)
    for k in range(-rng, rng + 1):
        if k >= 0:
            x, y = a[k:], b[:n - k] if k else b
        else:
            x, y = a[:k], b[-k:]
        if x.shape != y.shape or x.size < 100:
            continue
        ok = np.isfinite(x) & np.isfinite(y)
        if ok.sum() < 100:
            continue
        c = float(np.mean(np.abs(x[ok] - y[ok])))
        if c < best_c:
            best_c, best_k = c, k
    return best_k, best_c


class LidarRotationProbe(Node):
    def __init__(self):
        super().__init__('lidar_rotation_probe')
        self.create_subscription(LaserScan, '/scan', self._on_scan, 50)
        self.scans = []      # [(时间戳, ranges)]
        self.inc = None

    def _on_scan(self, m):
        self.inc = m.angle_increment
        r = np.asarray(m.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > 0.05), r, np.nan)
        self.scans.append((time.time(), r))

    def collect(self, seconds):
        self.scans = []
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _modal(self):
        """取出现次数最多的帧长，返回 (长度, 该长度的帧下标)。LiDAR 每帧点数会浮动。"""
        if len(self.scans) < 2:
            return None, []
        lens = [r.shape[0] for _, r in self.scans]
        L = max(set(lens), key=lens.count)
        return L, [i for i, n in enumerate(lens) if n == L]

    def measure(self):
        """返回 dict：总转角 / 平均转速 / 直接法 / 链式法 / 用时 / 帧数 / 帧长。"""
        L, idxs = self._modal()
        if L is None or len(idxs) < 2:
            return None
        elapsed = self.scans[idxs[-1]][0] - self.scans[idxs[0]][0]
        if elapsed <= 0.05:
            return None
        k_direct, _ = best_shift(self.scans[idxs[0]][1], self.scans[idxs[-1]][1], FAR_SHIFT)
        direct = k_direct * self.inc
        chain, n_chain, p = 0.0, 0, 0
        while p < len(idxs) - 1:
            j = p + 1
            while j < len(idxs) - 1 and \
                    self.scans[idxs[j]][0] - self.scans[idxs[p]][0] < CHAIN_GAP:
                j += 1
            if self.scans[idxs[j]][0] <= self.scans[idxs[p]][0]:
                p = j
                continue
            k, _ = best_shift(self.scans[idxs[p]][1], self.scans[idxs[j]][1], NEAR_SHIFT)
            chain += k * self.inc
            n_chain += 1
            p = j
        total = direct if n_chain == 0 else (direct + chain) / 2.0
        return {'total': total, 'direct': direct, 'chain': chain,
                'elapsed': elapsed, 'n_scans': len(self.scans), 'frame_len': L,
                'rate': total / elapsed}


def main():
    ap = argparse.ArgumentParser(
        description='用 LiDAR 测量机器人原地旋转角速度（Phase 0 验收工装）')
    ap.add_argument('--duration', type=float, default=6.0, help='测量时长 s（默认 6）')
    ap.add_argument('--expect-stationary', action='store_true', help='判定是否静止')
    ap.add_argument('--expect-rate', type=float, default=None,
                    help='期望角速度 rad/s，±30%% 内判通过')
    ap.add_argument('--warmup', type=float, default=1.0, help='预热时长 s')
    args = ap.parse_args()

    rclpy.init()
    node = LidarRotationProbe()
    try:
        node.collect(args.warmup)
        if node.inc is None:
            print('❌ 拿不到 /scan —— LiDAR 未运行或话题名不对', file=sys.stderr)
            return 1
        node.collect(args.duration)
        r = node.measure()
        if r is None:
            print('❌ 有效帧不足，无法测量', file=sys.stderr)
            return 1

        print(f'  LiDAR angle_increment = {node.inc:.6f} rad '
              f'({2 * np.pi / node.inc:.0f} 点/圈)')
        print(f'  收到 {r["n_scans"]} 帧（等长 {r["frame_len"]}）/ {r["elapsed"]:.2f} s')
        print(f'  总转角 = {r["total"]:+.4f} rad '
              f'(直接法 {r["direct"]:+.4f} / 链式法 {r["chain"]:+.4f})')
        print(f'  ★ 平均角速度 = {r["rate"]:+.4f} rad/s')

        agree = abs(r['direct'] - r['chain'])
        # 两法自检：绝对值与相对值都要超标才算存疑。
        # 只判相对值会在「静止」时误报 —— 两值都≈0 时 1 个索引点(0.0125 rad)
        # 的相对差会被放大到 100%，但绝对量毫无意义。
        if agree > 0.05 and agree > 0.25 * max(abs(r['direct']), abs(r['chain'])):
            print(f'  ⚠️ 两法差值 {agree:.4f} rad 偏大，结果存疑')

        if args.expect_stationary:
            ok = abs(r['rate']) < STATIC_EPS
            print(f'  {"✅ 判定：静止" if ok else "❌ 判定：在动"} '
                  f'(阈值 |ω| < {STATIC_EPS} rad/s)')
            return 0 if ok else 1
        if args.expect_rate is not None:
            ok = abs(r['rate'] - args.expect_rate) <= 0.3 * abs(args.expect_rate)
            print(f'  {"✅ 判定：符合" if ok else "❌ 判定：不符"} '
                  f'(期望 {args.expect_rate:+.4f} rad/s，±30%)')
            return 0 if ok else 1
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
