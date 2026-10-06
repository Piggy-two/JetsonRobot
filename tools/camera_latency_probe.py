#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机「端到端延迟」与「时间戳偏移」验收工装。

⚠️ 这是【验收/验证仪器】，不是运行时组件。
   不得被 Agent Runtime 或任何 Skill 调用。它会在运行期间改动相机的一个 V4L2 控制项
   （默认 `brightness`），虽然结束时会还原，但只应在人工看护下、非联调时段使用。

为什么需要它
------------
「相机比真实世界晚 0.72 秒」和「图像很新、但它的时间戳写早了 0.72 秒」是两件完全不同的事，
对控制的影响也不同（前者要调管线，后者只需重新打时间戳）。
只测「收到的时刻 − header.stamp」是分辨不出来的：在这个测量里两个未知量合成同一个数。

本工装造一个【世界端的时间基准】来把两者分开：
  `v4l2` 的 `brightness` 是 ISP 数字偏移，**逐帧立即生效**（不像自动曝光要收敛）。
  在 t_cmd 改变亮度 → 世界在 t_cmd 变了 → 含新内容的那一帧**真实曝光时刻 ≈ t_cmd**。
  于是分别读它的「收到时刻」和「header.stamp」，得到两个独立的数：

    端到端延迟   = 收到时刻 − t_cmd      ← 控制真正关心的量
    时间戳偏移   = header.stamp − t_cmd  ← ≈0 则时间戳诚实；明显为负则时间戳陈旧

同时订阅 raw 与 compressed 两条话题：compressed 体积小 20 倍，
两者之差可分离出「订阅端解析大图的开销」，避免把自己的开销算成相机的延迟。

用法示例
--------
    python3 tools/camera_latency_probe.py                    # 默认 image_raw + compressed
    python3 tools/camera_latency_probe.py --topic-only raw   # 只测 raw
    python3 tools/camera_latency_probe.py --device /dev/video0 --ctrl brightness

判读
----
  · 端到端延迟 约一帧到数帧        → 相机管线健康
    ⚠️ 该量**随负载变化**，不要假设是常数：2026-10-05 实测约 30 ms，2026-10-06 约 120 ms。
  · 时间戳偏移 明显为负            → header.stamp 陈旧，融合前必须重新打时间戳。
    ⚠️ **偏移量不是常数**：2026-10-05 为 −0.74 s，2026-10-06 为 −338.6 s（见
    PROJECT_STATUS #24）。**不要按固定值去补**，只能用自己的实时钟重建。
  · 标定 Overlay Driver 的 `pipeline_latency`：
        --topic /embodied/camera/image --repeat 6 --settle 2
      读「汇总」里「戳-下发」的均值 D，则
        新值 = 当前值 + D − T/2      （T = 该话题单帧周期）
    单次事件读数带 ±T/2 的帧栅格量化抖动，**必须取多次均值**，且**必须保持 --jitter 非 0**
    （相位被锁住时量化误差会退化成固定偏置，均值跟着偏）。
"""
import argparse
import random
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from sensor_msgs.msg import Image, CompressedImage

RAW_TOPIC = '/depth_cam/rgb0/image_raw'
CMP_TOPIC = '/depth_cam/rgb0/image_compressed'
DEV = '/dev/video0'
CTRL = 'brightness'
# 亮度档位：默认取该控件的两端 + 还原值
LEVELS = (127, -128, 0)


def sparse_brightness(data):
    """稀疏采样求平均亮度 —— 避免整帧解析拖慢订阅端。"""
    return statistics.mean(data[::997]) if len(data) else 0.0


def compressed_brightness(data):
    """解 JPEG 后求平均亮度。

    ⚠️ 不能直接对 JPEG 字节流稀疏采样当亮度用 —— 那是熵编码数据，
    与画面亮度无稳定关系（会得到"亮度几乎没变"的假结论）。
    cv2 不可用时退回稀疏字节均值，仅能当作粗糙的阶跃指示。
    """
    try:
        import cv2
        import numpy as np
        arr = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8),
                           cv2.IMREAD_GRAYSCALE)
        if arr is not None:
            return float(arr[::16, ::16].mean())
    except Exception:
        pass
    return sparse_brightness(bytes(data))


def set_ctrl(dev, ctrl, value):
    """下发控制项，返回下发时刻（这是本工装的"世界端"基准）。"""
    t = time.time()
    subprocess.Popen(['v4l2-ctl', '-d', dev, '--set-ctrl', f'{ctrl}={value}'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return t


def get_ctrl(dev, ctrl):
    try:
        out = subprocess.run(['v4l2-ctl', '-d', dev, '--get-ctrl', ctrl],
                             capture_output=True, text=True, timeout=5).stdout
        return int(out.split(':')[-1].strip())
    except Exception:
        return None


class Probe(Node):
    def __init__(self, topics):
        super().__init__('camera_latency_probe')
        self.recs = {t: [] for t in topics}
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        for t in topics:
            if 'compressed' in t:
                self.create_subscription(
                    CompressedImage, t,
                    lambda m, k=t: self.recs[k].append(
                        (time.time(),
                         m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         compressed_brightness(m.data))), q)
            else:
                self.create_subscription(
                    Image, t,
                    lambda m, k=t: self.recs[k].append(
                        (time.time(),
                         m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         sparse_brightness(m.data))), q)

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)


def analyse(tag, recs, events, frame_budget):
    print(f'\n  【{tag}】共 {len(recs)} 帧')
    if len(recs) < 10:
        print('    样本不足，跳过')
        return
    # 单次事件的判读受「帧栅格量化」影响，抖动可达 ±半个帧周期；
    # 只有多次事件取均值才定得住（见文件头「判读」）。
    d_arr, d_stamp = [], []
    for label, t_cmd in events:
        before = [b for t, s, b in recs if t_cmd - 1.0 < t < t_cmd - 0.05]
        after = [b for t, s, b in recs if t_cmd + 0.02 < t < t_cmd + 1.0]
        if not before or not after:
            print(f'    {label}: 样本不足')
            continue
        b0, b1 = statistics.median(before), statistics.median(after)
        if abs(b1 - b0) < 5:                      # 变化太小，判不出跳变
            print(f'    {label}: 亮度几乎没变 ({b0:.1f} -> {b1:.1f})，跳过')
            continue
        step = None
        for t, s, b in recs:
            if t > t_cmd and abs(b - b0) > 0.4 * abs(b1 - b0):
                step = (t, s, b)
                break
        if step is None:
            print(f'    {label}: 未检出跳变 ({b0:.1f} -> {b1:.1f})')
            continue
        t_r, t_s, _ = step
        d_arr.append((t_r - t_cmd) * 1000)
        d_stamp.append((t_s - t_cmd) * 1000)
        print(f'    {label}: 亮度 {b0:.1f} -> {b1:.1f}')
        print(f'      端到端延迟 (收到 - 下发) = {(t_r - t_cmd) * 1000:+.0f} ms')
        print(f'      该帧时间戳 - 下发时刻    = {(t_s - t_cmd) * 1000:+.0f} ms')
    if len(d_arr) >= 3:
        print(f'    —— 汇总（{len(d_arr)} 次事件）——')
        print(f'      端到端延迟: 均值 {statistics.mean(d_arr):+.0f} ms, '
              f'标准差 {statistics.pstdev(d_arr):.0f} ms')
        print(f'      戳-下发   : 均值 {statistics.mean(d_stamp):+.0f} ms, '
              f'标准差 {statistics.pstdev(d_stamp):.0f} ms')
    off = [(s - t) * 1000 for t, s, _ in recs]
    print(f'    稳态时间戳偏移: 均值 {statistics.mean(off):+.1f} ms, '
          f'抖动 {max(off) - min(off):.1f} ms')
    if frame_budget:
        print(f'    → 相对本话题单帧周期（约 {frame_budget:.0f} ms）判断延迟是否≈一帧')


def main():
    ap = argparse.ArgumentParser(
        description='相机端到端延迟与时间戳偏移验收工装（详见文件头注释）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default=DEV, help=f'V4L2 设备（默认 {DEV}）')
    ap.add_argument('--ctrl', default=CTRL, help=f'用作"世界端探针"的控制项（默认 {CTRL}）')
    ap.add_argument('--topic-only', choices=['raw', 'compressed', 'both'], default='both',
                    help='只测其中一条内置话题（默认两条都测，用于分离订阅端开销）')
    ap.add_argument('--topic', action='append', default=None, metavar='TOPIC',
                    help='直接指定要测的话题（可重复）。给了本项则忽略 --topic-only。'
                         '话题名含 compressed 的按 CompressedImage 订阅，其余按 Image。'
                         '用途：标定 Overlay Driver 输出话题（如 /embodied/camera/image）的'
                         '「重新打时间戳」是否落在世界事件时刻上。')
    ap.add_argument('--settle', type=float, default=4.0, help='每档位保持秒数（默认 4）')
    ap.add_argument('--repeat', type=int, default=0,
                    help='在两档之间额外往复 N 轮（每轮 2 次事件）。单次事件受帧栅格量化影响'
                         '有 ±半帧抖动，标定 pipeline_latency 时应取 --repeat 5~8 后看「汇总」均值')
    ap.add_argument('--jitter', type=float, default=1.0,
                    help='每个事件后额外随机等待 0~JITTER 秒（默认 1.0）。⚠️ 不要设 0：'
                         '事件时刻相对帧栅格的相位若固定，量化误差就不随机，「汇总」均值会偏 '
                         '（实测固定相位下 14 次事件全挤在同一个数上）')
    ap.add_argument('--no-restore', action='store_true', help='结束时不要还原控制项（默认会还原）')
    args, _ = ap.parse_known_args()

    topics = []
    if args.topic:
        topics = list(args.topic)
    else:
        if args.topic_only in ('raw', 'both'):
            topics.append(RAW_TOPIC)
        if args.topic_only in ('compressed', 'both'):
            topics.append(CMP_TOPIC)

    original = get_ctrl(args.device, args.ctrl)
    if original is None:
        print(f'❌ 读不到 {args.device} 的 {args.ctrl}，设备不存在或无权限', file=sys.stderr)
        return 2
    print(f'设备 {args.device} · 控制项 {args.ctrl} · 原始值 {original}')
    print(f'订阅 {topics}')

    rclpy.init()
    node = Probe(topics)
    events = []
    try:
        node.spin_for(3.0)                       # 预热
        # 事件序列：先跳到低档，再回高档；--repeat 追加 N 轮往复
        seq = [LEVELS[1], LEVELS[2]] + [LEVELS[1], LEVELS[2]] * max(0, args.repeat)
        for lv in seq:
            events.append((f'{args.ctrl} -> {lv}', set_ctrl(args.device, args.ctrl, lv)))
            # 随机抖动：打散事件时刻相对帧栅格的相位，让量化误差服从 U(0,T)，
            # 否则单帧周期的量化误差会变成一个固定偏置，混进「汇总」均值里。
            node.spin_for(args.settle + random.uniform(0.0, args.jitter))
    finally:
        if not args.no_restore:
            set_ctrl(args.device, args.ctrl, original)
            time.sleep(0.5)
            print(f'\n已还原 {args.ctrl} = {get_ctrl(args.device, args.ctrl)}（原 {original}）')

    print('\n' + '=' * 68)
    for t in topics:
        recs = node.recs[t]
        # 用中位帧间隔估单帧周期，供"延迟是否≈一帧"参照
        ts = sorted(x[0] for x in recs)
        budget = statistics.median([b - a for a, b in zip(ts, ts[1:])]) * 1000 if len(ts) > 2 else None
        analyse(t, recs, events, budget)

    print('\n判读：端到端延迟≈一帧 → 管线健康；时间戳偏移明显为负且稳定 → 时间戳陈旧，'
          '融合前必须重新打时间戳。')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
