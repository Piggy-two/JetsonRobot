#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**手动对焦助手**：盯着中心区域，把清晰度打成分，实时刷在一行里。

    ⚠️ 这是【工装】，不是运行时组件。它**只订阅图像**，不发布任何东西。

为什么要有它
------------
这台车的相机是**手动调焦**的（拧镜头），而对焦好不好**人眼在预览窗口里看不准** ——
尤其是画面小、流还延迟的时候。把"清不清晰"变成一个**一直在变的数字**，
拧的时候就有依据了：**往数字大的方向拧，越拧越大就对了**。

判据用的是 **Laplacian 方差**（对焦评估的经典做法）：图像越清晰，边缘越锐，
二阶导的方差越大。⚠️ 它**只在"画面里有纹理"时才有意义** ——
对着白墙/天花板，数字天然很低，那是**没东西可聚焦**，不是相机的错。
所以：**拧之前先在镜头前放个带字/带图案的东西**。

⚠️ 它同时会告诉你**是不是已经到最好**：记着本次运行见过的最大值。拧过头了会掉下来，
那就往回拧一点 —— 这比盯着数字本身好使。

用法：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 tools/focus_assist.py                 # 一直跑，Ctrl-C 停
    python3 tools/focus_assist.py --seconds 120   # 跑两分钟自己停
    python3 tools/focus_assist.py --roi 0.4       # 只看中心 40%（默认 50%）

⚠️ 只看**中心区域**：广角镜头的边缘本来就软，拿整幅算会把焦点判歪。
"""

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_bgr(msg):
    """把图像转成 BGR。**转换方式必须与厂商节点一致**（已交叉对照验过）。"""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'yuv422_yuy2':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUY2)
    if msg.encoding == 'bgr8':
        return buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'不认识的 encoding：{msg.encoding}')


class FocusAssist(Node):
    def __init__(self, topic, roi):
        super().__init__('focus_assist')
        self.roi = roi
        self.img = None
        self.frames = 0
        self._warned = False
        self.create_subscription(Image, topic, self._on_image, 5)

    def _on_image(self, m):
        try:
            self.img = to_bgr(m)
            self.frames += 1
        except ValueError as exc:
            if not self._warned:
                print(f'\n⚠️ {exc} —— 换一条话题试试', flush=True)
                self._warned = True

    def score(self):
        """中心 ROI 的清晰度：Laplacian 方差（越大越清晰）。"""
        if self.img is None:
            return None
        h, w = self.img.shape[:2]
        rh, rw = int(h * self.roi), int(w * self.roi)
        y0, x0 = (h - rh) // 2, (w - rw) // 2
        gray = cv2.cvtColor(self.img[y0:y0 + rh, x0:x0 + rw], cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def bar(value, top):
    """一行进度条 —— 人拧镜头时看的是"有没有变长"，不是小数第三位。"""
    n = 0 if top <= 0 else int(round(24 * min(value / top, 1.0)))
    return '█' * n + '░' * (24 - n)


def main():
    ap = argparse.ArgumentParser(description='手动对焦助手（只读图像）')
    ap.add_argument('--topic', default='/depth_cam/rgb0/image_raw')
    ap.add_argument('--roi', type=float, default=0.5,
                    help='中心取样区域占整幅的比例（默认 0.5）。广角边缘本就软，别取满')
    ap.add_argument('--seconds', type=float, default=0.0, help='跑多少秒后自己停（0=一直跑）')
    ap.add_argument('--hz', type=float, default=3.0, help='刷新率（默认 3 次/秒）')
    args = ap.parse_args()

    rclpy.init()
    node = FocusAssist(args.topic, args.roi)
    print()
    print('=' * 68)
    print('  手动对焦助手 —— 往**数字变大**的方向拧')
    print(f'  话题：{args.topic}｜取样：中心 {args.roi * 100:.0f}%')
    print('  ⚠️ 先在镜头前放个**带字/带图案**的东西：对着白墙数字天然很低，')
    print('     那是"没东西可聚焦"，不是相机的问题。')
    print('=' * 68)
    print()

    best, best_at = 0.0, 0.0
    t0 = time.monotonic()
    last = 0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last < 1.0 / args.hz:
                continue
            last = now
            s = node.score()
            if s is not None:
                if s > best:
                    best, best_at = s, now - t0
                # 刻度用"本次见过的最好值"作参照，于是进度条只在**变好**时才变长
                mark = '★ 新高' if abs(s - best) < 1e-9 and best > 0 else ''
                print(f'\r  清晰度 {s:7.1f}  {bar(s, max(best, 1.0))}  '
                      f'本次最佳 {best:7.1f}（{best_at:5.1f}s） {mark:<6}',
                      end='', flush=True)
            elif node.frames == 0 and now - t0 > 5.0:
                print(f'\r  等不到图像（{args.topic} 上没有帧）—— 相机在跑吗？'
                      f'{" " * 30}', end='', flush=True)
            if args.seconds and now - t0 >= args.seconds:
                break
        print()
        print(f'\n  结束：本次最佳 {best:.1f}。⚠️ 这个数是**相对**的 —— '
              f'它只说明"这次比那次清楚"，不是绝对的质量指标。')
        return 0
    except KeyboardInterrupt:
        print(f'\n\n  中断。本次最佳 {best:.1f}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
