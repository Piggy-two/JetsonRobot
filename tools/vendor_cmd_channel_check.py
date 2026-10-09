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
from geometry_msgs.msg import Twist
from rclpy.node import Node

#: 三条入口。`ours` 标出哪条是本项目的 —— 其余两条出现非零就要报警。
CHANNELS = (
    ('/cmd_vel', True, '本项目的栈（Motor Driver / Safety 零速流）'),
    ('/controller/cmd_vel', False, '厂商的 app 生态（⚠️ 无限幅、无闸门）'),
    ('/app/cmd_vel', False, '厂商手机 app 那一侧'),
)

#: 已知的**底盘入口**。只有这些话题上出现非零，"有人在驱动底盘"才是**有依据**的结论；
#: 自检（`--topics`）指向别的话题时，工装只报读数、**不下这个结论**。
KNOWN_CHASSIS = frozenset(t for t, _, _ in CHANNELS)

#: 判定"非零"的阈值。⚠️ 与其它工装一致：浮点噪声不算数。
EPS = 1e-9


class Rig(Node):
    def __init__(self, channels=CHANNELS):
        super().__init__('vendor_cmd_channel_check')
        self.channels = channels
        self.stats = {t: {'frames': 0, 'nonzero': 0, 'peak': 0.0}
                      for t, _, _ in channels}
        for topic, _, _ in channels:
            self.create_subscription(
                Twist, topic,
                lambda m, t=topic: self._on_twist(t, m), 50)

    def _on_twist(self, topic, msg):
        s = self.stats[topic]
        s['frames'] += 1
        mag = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z))
        if mag > EPS:
            s['nonzero'] += 1
            s['peak'] = max(s['peak'], mag)

    def publishers(self, topic, attempts=5, gap=0.2):
        """ROS 图里**有能力发**的人（与"此刻在发"无关）。

        ⚠️ 返回 `(最多看到几个, 名字集合, 读不到名字的个数)` —— **计数是下界**。

        为什么不能只查一次：2026-10-09 实测，同一条话题上
        `ros2 topic info -v` 报 **5 个**发布者，而本方法的**单次**查询只报 **4 个**、
        其中 3 个连名字都读不到。图发现是**逐渐**的，一次查询会**偏低** ——
        而"报少了"正是本工装存在的理由所要防的那种错（**没看到 ≠ 没有**）。
        ⇒ 重试取并集；即便如此也只保证"**至少**这么多"，所以结论里**不许**写"就这些"。

        名字读不到时**如实说读不到**，绝不拿占位符当节点名印出去。
        """
        seen = {}
        unnamed = 0          # ⚠️ **单次最多**读到几个占位符，不是各次累加 ——
        count = 0            #    累加会把"3 个读不到"报成"15 个读不到"。
        for _ in range(max(1, attempts)):
            try:
                info = list(self.get_publishers_info_by_topic(topic))
            except Exception:                          # noqa: BLE001
                return None
            count = max(count, len(info))
            blind = 0
            for i in info:
                name = _endpoint_name(i)
                if name is None:
                    blind += 1
                else:
                    seen[name] = True
            unnamed = max(unnamed, blind)
            time.sleep(gap)
        return count, sorted(seen), unnamed


def _endpoint_name(info):
    """一个发布者的名字；**读不到时返回 `None`**（不是编一个名字出来）。

    ⚠️ ROS 有时只给占位符（`_NODE_NAME_UNKNOWN_`）—— 那时**如实说读不到**，
    不要把一个占位符当成节点名印出去（那会让人以为真有个叫这名字的节点）。
    """
    name = str(getattr(info, 'node_name', '') or '')
    ns = str(getattr(info, 'node_namespace', '') or '')
    if not name or 'UNKNOWN' in name.upper() or 'UNKNOWN' in ns.upper():
        return None
    return f'{ns.rstrip("/")}/{name}'.lstrip('/')


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
        channels = CHANNELS

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
            return 4
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
    bad = []
    for topic, ours, why in channels:
        s = rig.stats[topic]
        if s['frames'] == 0:
            verdict = '**一秒都没发**'
        elif s['nonzero'] == 0:
            verdict = f'发了 {s["frames"]} 帧，**全是零**（不会让车动）'
        else:
            # ⚠️ 只有**已知的底盘入口**才敢说"有人在驱动底盘"。
            #    自检模式指向的是一条我们不了解的话题 —— 那时**只报读数，不下结论**
            #    （下了就是无依据的结论，而这正是本工装要防的东西）。
            tail = ('—— 有人在真的驱动底盘'
                    if topic in KNOWN_CHASSIS
                    else '—— ⚠️ 这条话题上出现了非零帧（本工装**不知道它通向哪里**）')
            verdict = (f'⚠️ 发了 {s["frames"]} 帧，其中 **{s["nonzero"]} 帧非零**'
                       f'（峰值 {s["peak"]:.3f}）{tail}')
            if not ours:
                bad.append(topic)
        print(f'    {topic:<22} {verdict}')

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
        rc = 1
    else:
        print('  ✅ **干净**：三条通道上都没有非零帧 —— 此刻没有别人在驱动底盘。')
        print('  ⚠️ 这只说明**此刻**。它证明不了下一秒 —— 厂商那些 app 还挂着，')
        print('     任何一个被使能（摇杆 / app / 服务）都会绕过本项目全部四道防线。')
        rc = 0
    print('=' * 78)

    rig.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
