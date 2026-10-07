#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_environment_probe.py —— 测量 LiDAR 的【距离-角度剖面】，判定近场回波来自
外部环境还是机器人自身结构。

    ⚠️ 这是 Phase 0 的【验收/诊断工装】，不是运行时组件。
       不得被 Agent Runtime、任何 Skill 或 Safety Runtime 调用。

为什么需要它
------------
2026-10-06 首次量 LiDAR Primitive 时观察到一个环境条件（见 PROJECT_STATUS §7）：
当前摆位下 **67% 的回波在 0.6 m 以内**，且分成两块形态 ——

  · 前方 0°~60° 一段随角度【平滑变远】、呈 `d/cosθ` 形态（像是正前方约 0.25 m
    处有一个平整面）；
  · 198°~359° 一圈恒在 0.10~0.28 m。

后果是 `path_clear(±30°, 1 m)` **恒为 false**，这个原语在当前摆位下给不出
有意义的"通畅"判定。但**成因未定**：可能是外部环境（旁边的墙 / 桌子 / 人），
也可能是**雷达看到了机器人自身的结构**（底盘、轮子、支架）。

判定方法
--------
判据不是"距离多近"，而是【回波跟不跟着车走】——**自身结构与雷达固连，
车挪到哪儿它都在同样的角度上；环境回波则随摆位而变。**

  · 换个摆位（或**把车原地转 90°**）再测一次，两次对比：
      - 回波**跟着车走** → 机器人自身结构（避障时须按角度掩膜剔除）；
      - 回波**留在原地** → 外部环境（换个场地就没了）。

⚠️ **不需要特意找空旷场地** —— 要的是"**两次摆位不同**"，不是"某一次足够开阔"。
（初版把关"挪到开阔处或吊起来"当成了必要条件，其实只是当时最方便的做法。）

本工装只做【静态剖面测量】，把形态量化下来；车动不动由人决定，
`--json` 存一份便于两次对比。**转车 90° 的判别与场地大小无关，是最省事的做法。**

2026-10-07 实测结论（本次判定的实际用例）：
原地 67% 近场回波 + 198°~359° 一圈 0.10~0.28 m；换摆位后 → **3.6%、贴身环消失**
→ **外部环境**（详见 `docs/DECISIONS.md` D-028 补充与 `docs/DEV_NOTES.md` §1.4）。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash

    # 默认 25 帧 / 10° 一格
    python3 tools/lidar_environment_probe.py

    # 更细的分格 + 存一份 JSON 便于两次对比
    python3 tools/lidar_environment_probe.py --bin-deg 5 --json /tmp/open_area.json

退出码：0 = 测到数据；1 = 拿不到 /scan 或有效帧不足。

限制
----
  · 只反映【当前摆位】。LiDAR 装在车上，车一转身剖面全变。
  · 中位距离用的是各分格的原始点中位数，不做时间滤波；静止场景下已足够稳。
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

MIN_VALID_RANGE = 0.05   # 小于此值视为无效（设备量程下限 0.02，留余量）
NEAR_FIELD = 0.60        # 「近场」阈值，与 2026-10-06 观察口径一致 (m)
RING_FIELD = 0.35        # 「紧贴雷达」阈值，用于圈出贴身回波 (m)
PLOT_MAX = 5.0           # 柱状图满格距离 (m)


class LidarEnvironmentProbe(Node):
    def __init__(self, topic):
        super().__init__('lidar_environment_probe')
        self.create_subscription(LaserScan, topic, self._on_scan, 50)
        self.frames = []     # [(时间戳, angle_min, angle_increment, ranges)]
        self.topic = topic

    def _on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r) & (r > MIN_VALID_RANGE), r, np.nan)
        # ⚠️ 用时【单调钟】。本机墙上钟会被 NTP 步进（见已知问题 #24）——
        # 用 time.time() 算时长会得到毫无意义的速率（实测被算成 16.85 Hz，
        # 而 /scan 真实是 10.000 Hz）。
        self.frames.append((time.monotonic(), m.angle_min, m.angle_increment, r))

    def collect(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def wrap_pi(a):
    """把角度折到 [-π, π)。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(
        description='LiDAR 距离-角度剖面测量（Phase 0 诊断工装）')
    ap.add_argument('--topic', default='/scan', help='点云话题（默认 /scan）')
    ap.add_argument('--frames', type=int, default=25, help='采样帧数（默认 25）')
    ap.add_argument('--timeout', type=float, default=15.0, help='最长等待秒数')
    ap.add_argument('--bin-deg', type=float, default=10.0, help='角度分格宽度（默认 10°）')
    ap.add_argument('--json', default=None, help='把结果写一份 JSON 到该路径')
    args = ap.parse_args()

    rclpy.init()
    node = LidarEnvironmentProbe(args.topic)
    try:
        t0 = time.time()
        while len(node.frames) < args.frames and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.frames:
            print(f'❌ 拿不到 {args.topic} —— LiDAR 未运行或话题名不对', file=sys.stderr)
            return 1
        if len(node.frames) < 3:
            print(f'❌ 有效帧不足（{len(node.frames)} 帧）', file=sys.stderr)
            return 1

        frames = node.frames
        ts = np.array([f[0] for f in frames])
        span = float(ts[-1] - ts[0])
        dt = float(np.median(np.diff(ts))) if len(ts) > 1 else float('nan')
        lens = [f[3].shape[0] for f in frames]
        inc = frames[-1][2]

        # ---- 逐分格汇总（把所有帧的点汇进同一张直方图） ----
        nbin = int(round(2 * math.pi / math.radians(args.bin_deg)))
        bin_w = 2 * math.pi / nbin
        all_r, all_a = [], []
        for _, amin, ainc, r in frames:
            a = amin + np.arange(r.shape[0]) * ainc
            all_r.append(r)
            all_a.append(np.asarray(a))
        R = np.concatenate(all_r)
        A = np.concatenate(all_a)
        ok = np.isfinite(R)

        idx = np.floor(((A % (2 * math.pi)) / bin_w)).astype(int) % nbin
        med = np.full(nbin, np.nan)
        mn = np.full(nbin, np.nan)
        cnt = np.zeros(nbin, dtype=int)
        for b in range(nbin):
            sel = ok & (idx == b)
            cnt[b] = int(sel.sum())
            if cnt[b]:
                med[b] = float(np.median(R[sel]))
                mn[b] = float(np.min(R[sel]))

        # ---- 每帧的「前方 ±30° 最近回波」，跨帧取中位 = path_clear 判据 ----
        per_frame_front = []
        for _, amin, ainc, r in frames:
            a = np.asarray(amin + np.arange(r.shape[0]) * ainc)
            sel = np.isfinite(r) & (np.abs(wrap_pi(a)) <= math.radians(30))
            if sel.any():
                per_frame_front.append(float(np.min(r[sel])))
        front = float(np.median(per_frame_front)) if per_frame_front else float('nan')

        # ---- 前方 d·cosθ 检验：若回波来自一个正前方的平整面，则 d·cosθ ≈ 常数 ----
        dc = []
        for b in range(nbin):
            c = (b + 0.5) * bin_w
            if abs(wrap_pi(c)) <= math.radians(60) and np.isfinite(med[b]):
                dc.append(med[b] * math.cos(wrap_pi(c)))
        flat_fit = float(np.median(dc)) if dc else float('nan')
        flat_spread = float(np.std(dc)) if dc else float('nan')

        # ---- 贴身环：中位距离 < RING_FIELD 的分格 ----
        ring = [b for b in range(nbin) if np.isfinite(med[b]) and med[b] < RING_FIELD]

        # ---- 输出 ----
        print()
        print('=' * 68)
        print(f'  LiDAR 环境剖面  ——  {args.topic}')
        print('=' * 68)
        print(f'  帧数 {len(frames)} / 采集 {span:.2f} s '
              f'（帧间隔中位 {dt * 1000:.1f} ms → {1.0 / dt:.3f} Hz；'
              f'/scan 标称 10.000 Hz）')
        print(f'  angle_increment {inc:.6f} rad = {2 * math.pi / inc:.1f} 点/圈')
        print(f'  点数/帧：中位 {int(np.median(lens))}（{min(lens)}~{max(lens)}）')
        print(f'  有效回波比例 {100.0 * ok.sum() / R.size:.1f}%')
        near = ok & (R < NEAR_FIELD)
        print(f'  近场占比（< {NEAR_FIELD} m）  {100.0 * near.sum() / ok.sum():.1f}%'
              f'   ← 2026-10-06 实测为 67%')
        print()

        print(f'  {"角度区间":<12}{"中位距离":>10}{"最小距离":>10}{"点数":>8}   柱状（满格 '
              f'{PLOT_MAX:.0f} m）')
        print('  ' + '-' * 64)
        for b in range(nbin):
            lo = math.degrees(b * bin_w)
            hi = math.degrees((b + 1) * bin_w)
            if np.isfinite(med[b]):
                bar = '█' * max(1, int(round(min(med[b], PLOT_MAX) / PLOT_MAX * 24)))
                pad = ' ' * (24 - len(bar))
                print(f'  {lo:5.0f}°-{hi:3.0f}° {med[b]:10.3f} {mn[b]:10.3f} '
                      f'{cnt[b]:8d}   {bar}{pad}{med[b]:.2f}')
            else:
                print(f'  {lo:5.0f}°-{hi:3.0f}° {"--":>10} {"--":>10} {cnt[b]:8d}   (无回波)')

        print()
        print('  判定')
        print('  ' + '-' * 64)
        clr = 'False' if not (math.isfinite(front) and front > 1.0) else 'True'
        print(f'  · 前方 ±30° 每帧最近回波（跨帧中位）= {front:.3f} m')
        print(f'    → path_clear(±30°, 1 m) = {clr}')
        if clr == 'False':
            print(f'      ⚠️ 前方 1 m 内确有回波。**若现场本来就有东西，这是正确答案**；')
            print(f'         只有当你确认前方空旷、它仍报 false 时，才是原语或摆位有问题。')
        if ring:
            lo = math.degrees(min(ring) * bin_w)
            hi = math.degrees((max(ring) + 1) * bin_w)
            print(f'  · 贴身回波（中位 < {RING_FIELD} m）：{len(ring)}/{nbin} 格，'
                  f'跨度 {lo:.0f}° ~ {hi:.0f}°')
            # 圈是否连续（跨越 0° 接缝也算）
            contiguous = all(((b - ring[0]) % nbin) == i for i, b in enumerate(ring))
            print(f'    连续性：{"连续（一圈）" if contiguous else "断续（分几段）"}')
        else:
            print(f'  · 贴身回波（中位 < {RING_FIELD} m）：无')
        print(f'  · 前方 ±60° 的 d·cosθ 拟合：中位 {flat_fit:.3f} m，'
              f'标准差 {flat_spread:.3f} m')
        print(f'    {"→ 近似常数，符合「正前方一个平整面」" if flat_spread < 0.06 else "→ 起伏大，不像单一平整面"}')

        print()
        print('  下一步（判定近场回波是"自身的"还是"环境的"）：')
        print('    判据不是"距离多近"，而是【回波跟不跟着车走】：')
        print('      · 换个摆位（或把车原地转 90°）再测一次，用 --json 存档做对比；')
        print('      · 回波【跟着车走】→ 机器人自身结构（避障时须按角度掩膜剔除）；')
        print('      · 回波【留在原地】→ 外部环境（换个场地就没了）。')
        print('    ⚠️ 不需要特意找空旷场地 —— 要的是"两次摆位不同"，不是"某一次足够开阔"。')
        print()

        if args.json:
            with open(args.json, 'w') as f:
                json.dump({
                    'topic': args.topic,
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'frames': len(frames), 'span_s': span,
                    'n_points_per_frame_median': int(np.median(lens)),
                    'valid_ratio': float(ok.sum() / R.size),
                    'near_ratio_lt_060': float(near.sum() / ok.sum()),
                    'front_pm30_per_frame_min_median': front,
                    'bin_deg': args.bin_deg,
                    'bin_median': [None if not np.isfinite(v) else round(float(v), 4) for v in med],
                    'bin_min': [None if not np.isfinite(v) else round(float(v), 4) for v in mn],
                    'bin_count': cnt.tolist(),
                    'flat_fit_d': flat_fit, 'flat_fit_std': flat_spread,
                    'ring_bins_lt_035': ring,
                }, f, ensure_ascii=False, indent=2)
            print(f'  已写入 {args.json}')
            print()
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
