#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**跑任何运动验收之前**先跑它：确认没有别人在驱动这台车。

    ⚠️ 这是【前置检查 / 验收工装】，不是运行时组件。它**自己不发任何指令**。

它为什么存在
------------
厂商底盘有**三条** Twist 入口，全部汇进 `odom_publisher` 的**同一个回调**，
而那个回调里**没有任何闸门**（源码：`ros2_ws/src/driver/controller/controller/
odom_publisher_node.py`，行号见下表）：

| 话题 | 谁在用 | 限幅 | 进回调的方式 |
|---|---|---|---|
| `/cmd_vel` | **本项目的栈**（Motor Driver / Safety 零速流） | ✅ ±0.2 m/s、±0.5 rad/s | 行 138 → `app_cmd_vel_callback` → 行 198 |
| `/controller/cmd_vel` | **厂商整个 app 生态** | ❌ **无** | 行 136 → **直接** `cmd_vel_callback` |
| `/app/cmd_vel` | 厂商手机 app 那一侧 | ✅ | 行 137 → `acker_cmd_vel_callback` → 行 214 |

⇒ **`/controller/cmd_vel` 是一条绕过本项目四道防线的并行指令通道**：
Motor Driver 管不着它、Skill 网关/Agent 管不着它、**Safety Runtime 的零速流也管不着它**
（那只发 `/cmd_vel`），连厂商自己的限幅都绕开了。
2026-10-09 实测：厂商的 `self_driving` / `line_following` / `object_tracking` /
`joystick_control` / `lidar_app` **五个 app 当时都挂着**（但**没在发** —— 8 秒里一条都没有）。

**它不阻止任何事**（那条路是厂商代码里写死的，我们改不了）。它做的是**把"我以为"变成读数**：
运动验收的结论只有在"**全程没有第二个人在驱动底盘**"的前提下才成立，
而在此之前那个前提**从来没有被检查过**。

⚠️ **"没收到消息" ≠ "没人能发"** —— 本工装把这两件事分开报：
发布者数目来自 ROS 图（**谁有能力发**），收到的帧来自订阅（**此刻谁在发**）。
编队里最重要的一条判据是**非零帧数**，因为发零**不会让车动**。

用法
----
    python3 tools/vendor_cmd_channel_check.py                # 看 10 秒
    python3 tools/vendor_cmd_channel_check.py --seconds 120  # 陪整场运动验收
    python3 tools/vendor_cmd_channel_check.py --seconds 0    # 一直看到 Ctrl-C

自检（**报警那条路必须被打响过**，否则它不算证据）
--------------------------------------------------
一个从没响过的警报，和一个坏掉的警报**看起来一样**。所以本工装留了一个
`--topics` 入口，把三条通道换成**无害的话题**，然后往上面发一条非零速度，
看它是不是真的报警。用的话题**没有任何消费者**，车不可能动：

    # 一个终端：
    python3 tools/vendor_cmd_channel_check.py --seconds 8 \
        --topics /acceptance/fake_cmd

    # 另一个终端（发一条非零；这条话题没人订阅，车不会动）：
    ros2 topic pub -r 5 /acceptance/fake_cmd geometry_msgs/msg/Twist \
        '{linear: {x: 0.1}}'

    ⇒ 期望：报告 `⚠️ … N 帧非零`、**退出码 1**。

⚠️ `--topics` **只用于自检**。指向真正能驱动车的话题 = 把警报线拆掉。

退出码
------
    0 = **干净**：三条通道上都没有非零帧（此刻没有别人在驱动底盘）
    1 = **不干净**：厂商那两条通道上出现了**非零**帧
        ⇒ **这次运动验收的结论不能采信**（有第二个司机，测出来的位移/转向不一定是我们的）
    4 = **拒测**：ROS 图读不到（rclpy 没起来 / 环境没 source）
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

# ⚠️ 通道清单、"非零"的判定、读图的重试逻辑 —— **全部来自 `vendor_channels`**。
#    那是单一事实源，本工装只是它的一个 CLI 外壳。理由：会动车的验收工装有 6 个，
#    它们与本工装共用同一份判据，而判据散成七份迟早会分家
#    —— **D-040 / 坑 27** 就是这么长出过一个死区的。
import vendor_channels as vc


class Rig(Node):
    """本工装的节点。判据一律委托给 `vendor_channels`（这里不写第二份）。"""

    def __init__(self, channels=vc.CHANNELS):
        super().__init__('vendor_cmd_channel_check')
        self.watch = vc.Watch(self, channels)
        self.channels = channels
        self.stats = self.watch.stats

    def publishers(self, topic, attempts=5, gap=0.2):
        """见 `vendor_channels.publishers`（含"单次查询会偏低"那一条）。"""
        return vc.publishers(self, topic, attempts, gap)


def main():
    ap = argparse.ArgumentParser(description='运动验收前置检查：有没有别人在驱动底盘')
    ap.add_argument('--seconds', type=float, default=10.0,
                    help='观察多少秒；0 = 一直看到 Ctrl-C（默认 10）')
    ap.add_argument('--topics', nargs='+', default=None,
                    help='⚠️ **只用于自检**：把三条通道换成别的话题 '
                         '（验证"警报真的会响"）。指向能驱动车的话题 = 拆掉警报线')
    args = ap.parse_args()

    if args.topics:
        channels = tuple((t, False, '⚠️ 自检用（非本项目通道）') for t in args.topics)
    else:
        channels = vc.CHANNELS

    rclpy.init()
    rig = Rig(channels)
    print()
    print('=' * 78)
    if args.topics:
        print('  🧪 **自检模式**：通道已被 --topics 换成无害话题 —— 这不是一次真检查')
    else:
        print('  ⚠️ 前置检查：此刻有没有**别人**在驱动这台车')
    print('=' * 78)
    print()
    print('  三条 Twist 入口都汇进 odom_publisher 的同一个回调，回调里没有闸门。')
    print('  本项目只走 /cmd_vel（有限幅）；厂商 app 生态走 /controller/cmd_vel（无限幅）。')
    print()

    # ---- 先读图：谁能发（这一步不需要等） ----
    graph = {}
    for topic, ours, why in channels:
        info = rig.publishers(topic)
        if info is None:
            print(f'  ⛔ 拒测：读不到 {topic} 的发布者信息 —— 环境没 source 好吗？')
            rig.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return vc.EXIT_REFUSE
        graph[topic] = info

    print('  ── 谁**有能力**发（ROS 图；与"此刻在不在发"无关）──')
    for topic, ours, why in channels:
        count, names, unnamed = graph[topic]
        mark = '本项目' if ours else '⚠️ 别人的'
        tail = f'，另有 {unnamed} 个**读不到名字**' if unnamed else ''
        print(f'    {topic:<22} **至少 {count} 个**发布者（{mark}）：'
              f'{", ".join(names) if names else "——"}{tail}')
    print()
    print('  ⚠️ "至少"是刻意的：图发现是**渐进**的，**单次**查询会**偏低** ——')
    print('     2026-10-09 实测：单次查询报 4 个，重试取并集后才与 CLI 的 5 个一致。')
    print('     ⇒ 这里报的仍是**下界**。要**权威清单**用：')
    print('         ros2 topic info /controller/cmd_vel -v')

    # ---- 再听：此刻谁在发 ----
    seconds = args.seconds
    print()
    if seconds > 0:
        print(f'  ── 听 {seconds:g} 秒：此刻到底有没有人在发 ──')
    else:
        print('  ── 一直听着（Ctrl-C 结束）──')
    deadline = time.monotonic() + seconds if seconds > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            rclpy.spin_once(rig, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    for topic, ours, why in channels:
        # ⚠️ 措辞来自 `vendor_channels.describe` —— 本工装不写第二份
        #    （"只有已知的底盘入口才敢说有人在驱动底盘"那条就在里面）。
        print(f'    {topic:<22} {vc.describe(topic, rig.stats[topic], ours)}')
    bad = vc.second_drivers(rig.stats, channels)

    print()
    print('=' * 78)
    if bad:
        where = '自检话题' if args.topics else '厂商通道'
        print(f'  ❌ **不干净**：{where}上有非零速度：')
        for t in bad:
            print(f'       - {t}')
        if args.topics:
            print('  ⇒ 🧪 **自检通过**：非零帧被如实抓到、退出码为 1 —— 报警这条路是通的。')
        else:
            print('  ⇒ **这次运动验收的结论不能采信** —— 有第二个司机，')
            print('     测出来的位移/转向不一定是我们的。先让那些 app 停下来再测。')
        rc = vc.EXIT_SECOND_DRIVER
    else:
        print('  ✅ **干净**：三条通道上都没有非零帧 —— 此刻没有别人在驱动底盘。')
        print('  ⚠️ 这只说明**此刻**。它证明不了下一秒 —— 厂商那些 app 还挂着，')
        print('     任何一个被使能（摇杆 / app / 服务）都会绕过本项目全部四道防线。')
        print('  ⚠️ 而**这一点已经不再是靠人记得了**：6 个会动车的验收工装现在')
        print('     各自执行同一道闸（`vendor_channels.install`），并且全程盯着。')
        rc = vc.EXIT_CLEAN
    print('=' * 78)

    rig.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
