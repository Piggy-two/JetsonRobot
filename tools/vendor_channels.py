#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D-050 那条并行指令通道 —— **通道清单、判据、闸门，只在这里写一次**。

它为什么是一个模块，而不是几行复制粘贴
--------------------------------------
会**命令真车运动**的验收工装一共有 **6 个**：

    agent_ground_acceptance.py            ground_motion_acceptance.py
    suspended_motion_acceptance.py        safety_chain_suspended_acceptance.py
    phase0_chassis_motion_acceptance.py   demo_run.py

D-050 把前置检查写进了其中 **2 个的注释**里 —— 而**注释不会拦任何人**，
另外 **4 个连注释都没有**。把同一段判据抄进 7 个文件（6 个工装 + 1 个检查工装），
迟早会分家；这个项目已经吃过一次这个亏：**判据与被测物各用一个常量，
中间长出一个死区**（**D-040 / 坑 27**，`CLEAR_MARGIN` 那一次）。

所以：通道表、`非零` 的定义、结论的措辞，**只有这一份**。

它做的是"拦"，不是"报告"
------------------------
`vendor_cmd_channel_check.py` 是**人来跑的前置检查**（读图 + 听流量 + 出报告）。
本模块是**工装自己执行的闸门**，两个动作：

1. **起飞前**（`install`）：固定 3.0 s 观察窗，出现非零帧 ⇒ **拒测**（还没开始动，
   这是最安全的拦截点）。⚠️ **不给关掉它的开关** —— 能关掉的闸门等于没有闸门。
2. **全程**（`attach` / `finalize`）：观察与本工装同生共死，结束时若期间出现过非零帧
   ⇒ **把这次验收的结论标成作废**（退出码 3）。

⚠️ 一条必须说清的边界：全程观察**与工装自己的 spin 共用**（订阅挂在工装那个节点上）。
工装不 spin 的间隙观察不到。运动期间工装一直在 spin，所以**测量窗**是覆盖的；
真正裸奔的是运动之间的短间隙 —— 与其为此在验收工装里塞一个后台线程
（那会在实车运动时引入 rclpy 关停竞态，代价比它防的东西大），
不如把这条边界**写在纸面上**。

⚠️ 本模块**自己不发布任何东西** —— 它只订阅。所以它不可能成为第二个司机。
"""

import argparse
import atexit
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#: 三条 Twist 入口。`ours` 标出哪条是本项目的 —— 其余两条出现非零就要报警。
CHANNELS = (
    ('/cmd_vel', True, '本项目的栈（Motor Driver / Safety 零速流）'),
    ('/controller/cmd_vel', False, '厂商的 app 生态（⚠️ 无限幅、无闸门）'),
    ('/app/cmd_vel', False, '厂商手机 app 那一侧'),
)

#: 已知的**底盘入口**。只有这些话题上出现非零，"有人在驱动底盘"才是**有依据**的结论；
#: 自检（`--vendor-watch-topics`）指向别的话题时，只报读数、**不下这个结论**。
KNOWN_CHASSIS = frozenset(t for t, _, _ in CHANNELS)

#: 判定"非零"的阈值。⚠️ 与其它工装一致：浮点噪声不算数。
EPS = 1e-9

#: 起飞前的观察窗。**固定值** —— 故意不做成参数（见模块文档）。
PREFLIGHT_SECONDS = 3.0

#: 退出码。0 / 1 / 4 与 `vendor_cmd_channel_check.py` 一致。
EXIT_CLEAN = 0
EXIT_SECOND_DRIVER = 1      # 出现过非零帧（自检模式下用它证明"警报真的会响"）
EXIT_VOID = 3               # ⚠️ **本工装的结论作废**：验收期间出现过第二个司机
EXIT_REFUSE = 4             # 拒测（起飞前就不干净 / ROS 图读不到）


# --------------------------------------------------------------------------
# 判据（纯函数 —— 可离线证伪的那一部分）
# --------------------------------------------------------------------------

def twist_magnitude(msg):
    """一条 Twist 的"大小"。发零**不会让车动**，所以判据只认这个数。"""
    return max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z))


def new_stats(channels=CHANNELS):
    """每条通道的计数器。`nonzero` 是唯一有分量的一格。"""
    return {t: {'frames': 0, 'nonzero': 0, 'peak': 0.0} for t, _, _ in channels}


def note(stats, topic, msg):
    """记一帧。**只加不减** —— 之后谁来读都是同一份账。"""
    s = stats[topic]
    s['frames'] += 1
    mag = twist_magnitude(msg)
    if mag > EPS:
        s['nonzero'] += 1
        s['peak'] = max(s['peak'], mag)


def second_drivers(stats, channels=CHANNELS):
    """**非本项目**的通道上出现过非零帧的话题名（升序）。

    ⚠️ `/cmd_vel` 上出现非零**不算** —— 那正是本工装自己在让车动，
    验收期间它必然非零。判据是"**别人**在发"，不是"有人在发"。
    """
    return sorted(t for t, ours, _ in channels
                  if not ours and stats[t]['nonzero'] > 0)


def endpoint_name(info):
    """一个发布者的名字；**读不到时返回 `None`**（不是编一个名字出来）。

    ⚠️ ROS 有时只给占位符（`_NODE_NAME_UNKNOWN_`）—— 那时**如实说读不到**，
    不要把一个占位符当成节点名印出去（那会让人以为真有个叫这名字的节点）。
    """
    name = str(getattr(info, 'node_name', '') or '')
    ns = str(getattr(info, 'node_namespace', '') or '')
    if not name or 'UNKNOWN' in name.upper() or 'UNKNOWN' in ns.upper():
        return None
    return f'{ns.rstrip("/")}/{name}'.lstrip('/')


def describe(topic, stat, ours, known=KNOWN_CHASSIS):
    """把一条通道的读数翻成一句话。

    ⚠️ **结论的分量要匹配证据的分量**：只有**已知的底盘入口**才敢说
    "有人在真的驱动底盘"。自检指向的是一条我们不了解的话题 —— 那时只报读数。
    """
    if stat['frames'] == 0:
        return '**一秒都没发**'
    if stat['nonzero'] == 0:
        return f'发了 {stat["frames"]} 帧，**全是零**（不会让车动）'
    tail = ('—— 有人在真的驱动底盘'
            if topic in known
            else '—— ⚠️ 这条话题上出现了非零帧（**不知道它通向哪里**）')
    return (f'⚠️ 发了 {stat["frames"]} 帧，其中 **{stat["nonzero"]} 帧非零**'
            f'（峰值 {stat["peak"]:.3f}）{tail}')


def publishers(node, topic, attempts=5, gap=0.2):
    """ROS 图里**有能力发**的人（与"此刻在发"无关）。

    ⚠️ 返回 `(最多看到几个, 名字集合, 读不到名字的个数)` —— **计数是下界**。

    为什么不能只查一次：2026-10-09 实测，同一条话题上 `ros2 topic info -v`
    报 **5 个**发布者，而**单次**查询只报 **4 个**、其中 3 个连名字都读不到。
    图发现是**逐渐**的，一次查询会**偏低** —— 而"报少了"正是本模块存在的理由
    所要防的那种错（**没看到 ≠ 没有**，坑 24 → 坑 43 → 坑 51，同一个教训第三次）。
    ⇒ 重试取并集；即便如此也只保证"**至少**这么多"，所以结论里**不许**写"就这些"。
    """
    seen = {}
    unnamed = 0          # ⚠️ **单次最多**读到几个占位符，不是各次累加 ——
    count = 0            #    累加会把"3 个读不到"报成"15 个读不到"。
    for _ in range(max(1, attempts)):
        try:
            info = list(node.get_publishers_info_by_topic(topic))
        except Exception:                                          # noqa: BLE001
            return None
        count = max(count, len(info))
        blind = 0
        for i in info:
            name = endpoint_name(i)
            if name is None:
                blind += 1
            else:
                seen[name] = True
        unnamed = max(unnamed, blind)
        time.sleep(gap)
    return count, sorted(seen), unnamed


# --------------------------------------------------------------------------
# 观察（订阅挂到**别人的节点**上）
# --------------------------------------------------------------------------

class Watch:
    """把三条通道的订阅挂到某个已有节点上。

    ⚠️ 刻意**不**另起节点、不另起线程：验收工装已经在 spin 了，
    运动期间的回调一定会被喂到。另起线程会在实车运动时引入 rclpy 关停竞态。
    """

    def __init__(self, node, channels=CHANNELS):
        self.channels = channels
        self.stats = new_stats(channels)
        self._subs = [
            node.create_subscription(
                Twist, topic,
                lambda m, t=topic: note(self.stats, t, m), 50)
            for topic, _, _ in channels
        ]

    @property
    def dirty(self):
        return second_drivers(self.stats, self.channels)


#: 当前挂着的那个观察者。**刻意是个模块级单例** —— 为的是让验收工装里
#: 只需要三行接线（`add_arguments` / `install` / `attach` + `finalize`），
#: 而 `finalize` 又能被 `atexit` 那条兜底路径找到（漏了 `finalize` 也藏不住）。
_watch = None


def attach(node, channels=CHANNELS):
    """开始**全程**观察。返回观察者本身（多数工装不需要接返回值）。

    同时注册一条 `atexit` 兜底：工装若从别的路径退出（早返回 / 抛异常），
    只要期间出现过非零帧，**照样打那张作废横幅** —— 不让它静默漏掉。
    """
    global _watch
    _watch = Watch(node, channels)
    atexit.register(_atexit_report)
    return _watch


def _atexit_report():
    """兜底：`finalize` 没被走到时，至少把话说出来（此时改不了退出码）。"""
    if _watch is not None and _watch.dirty:
        print(_void_banner(_watch), file=sys.stderr)


def finalize(rc):
    """收工。全程干净 ⇒ 原样返回；出现过第二个司机 ⇒ **把结论作废**（退出码 3）。

    ⚠️ 作废时**不看工装自己判了什么**：它可能老老实实报了 ✅。
    那些 ✅ 是**在错误前提下算出来的**，留着比没有更坏。
    """
    global _watch
    if _watch is None:
        return rc
    dirty = _watch.dirty
    print()
    if not dirty:
        print(f'  ✅ 全程观察干净：{len(_watch.channels)} 条 Twist 入口上都没有非零'
              f'帧 —— 本项目是唯一在发指令的人。')
        _watch = None
        return rc
    print(_void_banner(_watch))
    _watch = None
    return EXIT_VOID


def _void_banner(watch):
    lines = ['=' * 78,
             '  ❌ **本次结论作废** —— 验收期间**有第二个司机**',
             '=' * 78, '',
             '  下面这些通道在本次运行期间出现了**非零**速度：']
    for t in watch.dirty:
        s = watch.stats[t]
        why = dict((k, w) for k, _, w in watch.channels).get(t, '')
        lines.append(f'       - {t:<22} {s["nonzero"]} 帧非零（峰值 '
                     f'{s["peak"]:.3f}）{why}')
    lines += ['',
              '  ⇒ 测出来的位移/转向**不一定是本项目的**（D-050）。',
              '  ⇒ 上面那些 ✅ 是在**错误的前提下**算出来的，别采信。',
              '  ⇒ 先让厂商那些 app 停下来，再重跑本工装。',
              '=' * 78]
    return '\n'.join(lines)


# --------------------------------------------------------------------------
# 起飞前的闸门
# --------------------------------------------------------------------------

def add_arguments(ap):
    """把本模块的参数加到一个已有的 `ArgumentParser` 上。

    ⚠️ **正常模式没有"关掉它"的开关** —— 只有一个 `--vendor-watch-seconds`，
    而且它**只在自检模式下有意义**（自检模式根本不进运动流程，所以关不掉任何东西）。
    传了它却不带 `--vendor-watch-topics` ⇒ **当场报错**，免得有人以为改动了真闸门。
    """
    ap.add_argument(
        '--vendor-watch-topics', nargs='+', default=None,
        help='⚠️ **只用于自检**：把观察通道换成这些话题（**必须是无害话题**），'
             '验证"拦得住"这条路真的通。用了它本工装就**只验闸门**、'
             '到此为止 —— 绝不进入运动流程')
    ap.add_argument(
        '--vendor-watch-seconds', type=float, default=None,
        help='⚠️ **只在自检模式有效**；正常模式的观察窗是固定的 '
             f'{PREFLIGHT_SECONDS:g} s，没有开关可以关掉它')


def install(args):
    """起飞前检查。**返回 `None` = 可以继续**；返回一个整数 = 就地结束、拿它当退出码。

    三个返回值：

    * `None` —— 干净，继续跑正经验收。
    * `EXIT_REFUSE`(4) —— 起飞前就不干净（或自检模式下**故意**验到了非零）。

    ⚠️ 自检模式（`--vendor-watch-topics`）**一定**在这里结束，无论干不干净：
    否则"自检"就成了一个**绕过真闸门的开关** —— 传个假话题就能把警报线拆掉。
    """
    if args.vendor_watch_seconds is not None and not args.vendor_watch_topics:
        print('⛔ `--vendor-watch-seconds` 只对自检模式（`--vendor-watch-topics`）有效。')
        print('   正常模式的观察窗是固定的 —— 那是有意的，不给"关掉闸门"的开关。')
        return EXIT_REFUSE

    self_check = bool(args.vendor_watch_topics)
    if self_check:
        channels = tuple((t, False, '⚠️ 自检用（非本项目通道）')
                         for t in args.vendor_watch_topics)
        seconds = (args.vendor_watch_seconds
                   if args.vendor_watch_seconds is not None else PREFLIGHT_SECONDS)
    else:
        channels = CHANNELS
        seconds = PREFLIGHT_SECONDS

    stats, graph = _listen(seconds, channels)

    print()
    print('=' * 78)
    if self_check:
        print('  🧪 **自检模式**：通道已被 --vendor-watch-topics 换成别的话题')
        print('     —— 这不是一次真检查，而且本工装**到此为止**（不会去动车）')
    else:
        print(f'  ⚠️ 前置检查（**本工装自己做的**，D-050）：起飞前 {seconds:g} 秒里')
        print('     有没有**别人**在驱动这台车')
    print('=' * 78)
    print()
    _print_graph(graph, channels)
    for topic, ours, why in channels:
        print(f'    {topic:<22} {describe(topic, stats[topic], ours)}')

    bad = second_drivers(stats, channels)
    print()
    if bad:
        print('=' * 78)
        if self_check:
            print('  🧪 **自检通过**：非零帧被如实抓到、退出码为 1 —— 报警这条路是通的。')
        else:
            print('  ⛔ **拒测**：起飞前厂商通道上已经有非零速度：')
            for t in bad:
                print(f'       - {t}')
            print()
            print('  ⇒ 这次验收的结论**不能采信** —— 有第二个司机，')
            print('     测出来的位移/转向不一定是本项目的（D-050）。')
            print('  ⇒ 先让厂商那些 app 停下来（关掉 app / 重启厂商栈），再重跑。')
        print('=' * 78)
        return EXIT_SECOND_DRIVER if self_check else EXIT_REFUSE

    print('=' * 78)
    if self_check:
        print('  ✅ **自检的"干净"这一侧**：这几条话题上都没有非零帧。')
        print('     ⚠️ 它只证明了**没收到非零帧**，不证明别的 —— 要验拦得住，')
        print('        得往话题上发一条非零（见本工装的头部说明）。')
    else:
        print(f'  ✅ **干净**：起飞前 {seconds:g} 秒没有别人在驱动底盘 —— 继续。')
        print('     ⚠️ 这只是**起飞前**。全程另有并发观察，收工时会再报一次；')
        print('        而飞机停在原地的那几秒，观察与本工装自己的 spin 共用。')
    print('=' * 78)
    return EXIT_CLEAN if self_check else None


def _print_graph(graph, channels):
    print('  ── 谁**有能力**发（ROS 图；与"此刻在不在发"无关）──')
    for topic, ours, why in channels:
        info = graph.get(topic)
        if info is None:
            print(f'    {topic:<22} ⚠️ **读不到** —— 这条通道上"谁能发"此刻不可知')
            continue
        count, names, unnamed = info
        mark = '本项目' if ours else '⚠️ 别人的'
        tail = f'，另有 {unnamed} 个**读不到名字**' if unnamed else ''
        print(f'    {topic:<22} **至少 {count} 个**发布者（{mark}）：'
              f'{", ".join(names) if names else "——"}{tail}')
    print()
    print('  ⚠️ "至少"是刻意的：图发现是**渐进**的，**单次**查询会**偏低** ——')
    print('     2026-10-09 实测：单次查询报 4 个，重试取并集后才与 CLI 的 5 个一致。')
    print('     ⇒ 这里报的仍是**下界**。要**权威清单**用：')
    print('       ros2 topic info /controller/cmd_vel -v')
    print()
    print('  ⚠️ 本模块**只订阅、从不发布** —— 它不可能成为第二个司机。')


def _listen(seconds, channels):
    """起一个**只属于自己的**节点听 `seconds` 秒，返回 `(stats, graph)`。

    ⚠️ 这一步通常在宿主工装 `rclpy.init()` **之前**跑，所以这里自己管上下文，
    而且只在**是自己起的**时候才关掉它（别把宿主已经 init 的上下文拆了）。
    """
    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    node = None
    try:
        node = Node('vendor_channel_gate')
        watch = Watch(node, channels)
        graph = {t: publishers(node, t) for t, _, _ in channels}
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:                                          # noqa: BLE001
            pass
        if owned:
            try:
                rclpy.shutdown()
            except Exception:                                      # noqa: BLE001
                pass
    return watch.stats, graph
