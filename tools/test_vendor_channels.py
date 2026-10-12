#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`vendor_channels` 的离线单测 —— **不需要 ROS 图、不需要车、不需要节点**。

这一份刻意覆盖两类东西：

1. **判据本身**（"非零"、谁的账算谁的、措辞的分量）。
2. **闸门的接线**（`Watch` 到底挂了哪三条话题、`finalize` 在什么条件下把结论作废）。

第 2 类值得单独测：那条路**本来要靠"真的出现第二个司机"才能验**，
而那是只能在实车上撞见的条件 —— 用一个假的节点对象把订阅的回调抓出来手动喂进去，
就能在不碰硬件的前提下把"作废"这条路走通。

    python3 -m pytest tools/test_vendor_channels.py -q
"""

import argparse
import sys
from pathlib import Path

import pytest
from geometry_msgs.msg import Twist

sys.path.insert(0, str(Path(__file__).resolve().parent))

import vendor_channels as vc          # noqa: E402


# --------------------------------------------------------------------------
# 假的节点：只记录订阅，并把回调交出来供测试手动喂
# --------------------------------------------------------------------------

class FakeNode:
    def __init__(self):
        self.subs = []           # [(topic, msg_type, callback)]

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs.append((topic, msg_type, callback))

        class _Sub:                                # 只是让调用点有个东西可拿
            pass

        return _Sub()

    def feed_msg(self, topic, msg):
        """往某条话题上"发"一帧 —— 只喂给本进程注册的回调，不出进程、不动车。"""
        for t, _, cb in self.subs:
            if t == topic:
                cb(msg)


def twist(x=0.0, y=0.0, wz=0.0):
    m = Twist()
    m.linear.x = x
    m.linear.y = y
    m.angular.z = wz
    return m


@pytest.fixture(autouse=True)
def _clean_module_state():
    """每个用例前后都把模块级那个观察者清掉 —— 用例之间不许互相污染。"""
    vc._watch = None
    yield
    vc._watch = None


# --------------------------------------------------------------------------
# 1. 通道清单
# --------------------------------------------------------------------------

def test_exactly_one_channel_is_ours():
    """⚠️ "谁的账算谁的"全靠这个标记 —— 多标一个，闸门就瞎一只眼。"""
    ours = [t for t, o, _ in vc.CHANNELS if o]
    assert ours == ['/cmd_vel']


def test_known_chassis_matches_channel_list():
    """两张表必须是同一份 —— 分家就会出现"警告说 A、判据看 B"。"""
    assert vc.KNOWN_CHASSIS == {t for t, _, _ in vc.CHANNELS}


def test_vendor_channels_are_present():
    assert '/controller/cmd_vel' in vc.KNOWN_CHASSIS
    assert '/app/cmd_vel' in vc.KNOWN_CHASSIS


# --------------------------------------------------------------------------
# 2. 判据
# --------------------------------------------------------------------------

def test_magnitude_takes_the_largest_of_three():
    assert vc.twist_magnitude(twist(x=0.1, y=-0.3, wz=0.2)) == pytest.approx(0.3)
    assert vc.twist_magnitude(twist(wz=-0.5)) == pytest.approx(0.5)


def test_float_noise_is_not_nonzero():
    """发零**不会让车动** —— 浮点噪声不许把一次干净的验收判成作废。"""
    st = vc.new_stats()
    vc.note(st, '/controller/cmd_vel', twist(x=1e-12, wz=-1e-13))
    assert st['/controller/cmd_vel']['frames'] == 1
    assert st['/controller/cmd_vel']['nonzero'] == 0
    assert vc.second_drivers(st) == []


def test_note_counts_frames_nonzero_and_peak():
    st = vc.new_stats()
    for m in (twist(x=0.0), twist(x=0.2), twist(wz=-0.35), twist(y=0.1)):
        vc.note(st, '/controller/cmd_vel', m)
    s = st['/controller/cmd_vel']
    assert (s['frames'], s['nonzero']) == (4, 3)
    assert s['peak'] == pytest.approx(0.35)


def test_our_own_channel_is_never_a_second_driver():
    """⚠️ 验收期间 `/cmd_vel` 上**本来**就会是非零 —— 那正是本工装自己在开车。"""
    st = vc.new_stats()
    vc.note(st, '/cmd_vel', twist(x=0.2))
    vc.note(st, '/cmd_vel', twist(x=0.2))
    assert vc.second_drivers(st) == []


def test_vendor_channel_nonzero_is_a_second_driver():
    st = vc.new_stats()
    vc.note(st, '/cmd_vel', twist(x=0.2))                 # 我们自己
    vc.note(st, '/controller/cmd_vel', twist(wz=0.1))     # 别人
    assert vc.second_drivers(st) == ['/controller/cmd_vel']


def test_zero_only_flow_on_vendor_channel_is_not_a_second_driver():
    st = vc.new_stats()
    for _ in range(10):
        vc.note(st, '/controller/cmd_vel', twist())
    assert vc.second_drivers(st) == []


def test_second_drivers_is_sorted_and_covers_both_vendor_channels():
    st = vc.new_stats()
    vc.note(st, '/controller/cmd_vel', twist(x=0.1))
    vc.note(st, '/app/cmd_vel', twist(x=0.1))
    assert vc.second_drivers(st) == ['/app/cmd_vel', '/controller/cmd_vel']


# --------------------------------------------------------------------------
# 3. 措辞：结论的分量要匹配证据的分量
# --------------------------------------------------------------------------

class _Info:
    def __init__(self, name, ns=''):
        self.node_name = name
        self.node_namespace = ns


def test_endpoint_name_reads_normally():
    assert vc.endpoint_name(_Info('self_driving', '/')) == 'self_driving'
    assert vc.endpoint_name(_Info('line_following', '/apps')) == 'apps/line_following'


def test_endpoint_name_refuses_to_invent_a_name():
    """⚠️ 读不到名字时**返回 None** —— 绝不把占位符当节点名印出去。"""
    assert vc.endpoint_name(_Info('_NODE_NAME_UNKNOWN_')) is None
    assert vc.endpoint_name(_Info('')) is None
    assert vc.endpoint_name(_Info('x', '/_NODE_NAME_UNKNOWN_')) is None


def test_describe_silent_and_zero_are_different_words():
    st = vc.new_stats()
    silent = vc.describe('/controller/cmd_vel', st['/controller/cmd_vel'], False)
    vc.note(st, '/controller/cmd_vel', twist())
    zeros = vc.describe('/controller/cmd_vel', st['/controller/cmd_vel'], False)
    assert silent != zeros
    assert '一秒都没发' in silent
    assert '全是零' in zeros


def test_describe_claims_a_second_driver_only_on_known_chassis():
    """🔴 **铁律**：只有已知的底盘入口才敢说"有人在真的驱动底盘"。"""
    st = vc.new_stats()
    vc.note(st, '/controller/cmd_vel', twist(x=0.3))
    line = vc.describe('/controller/cmd_vel', st['/controller/cmd_vel'], False)
    assert '有人在真的驱动底盘' in line
    assert '0.300' in line

    # 自检指向的是一条我们不了解的话题 —— **只报读数，不下结论**
    st2 = {'/acceptance/fake_cmd': {'frames': 0, 'nonzero': 0, 'peak': 0.0}}
    vc.note(st2, '/acceptance/fake_cmd', twist(x=0.3))
    line2 = vc.describe('/acceptance/fake_cmd', st2['/acceptance/fake_cmd'], False)
    assert '有人在真的驱动底盘' not in line2
    assert '不知道它通向哪里' in line2


# --------------------------------------------------------------------------
# 4. 闸门的接线（用假节点把回调抓出来喂 —— 不碰硬件）
# --------------------------------------------------------------------------

def test_watch_subscribes_to_exactly_the_three_channels():
    node = FakeNode()
    vc.Watch(node)
    assert [t for t, _, _ in node.subs] == [t for t, _, _ in vc.CHANNELS]
    assert all(mt is Twist for _, mt, _ in node.subs)


def test_watch_feeds_the_same_stats_object():
    node = FakeNode()
    w = vc.Watch(node)
    node.feed_msg('/controller/cmd_vel', twist(x=0.4))
    assert w.stats['/controller/cmd_vel']['nonzero'] == 1
    assert w.dirty == ['/controller/cmd_vel']


def test_finalize_passes_a_clean_run_through_untouched():
    vc.attach(FakeNode())
    assert vc.finalize(0) == 0
    vc.attach(FakeNode())
    assert vc.finalize(1) == 1


def test_finalize_keeps_a_nonzero_rc_nonzero_when_clean():
    vc.attach(FakeNode())
    assert vc.finalize(7) == 7


def test_finalize_voids_the_conclusion_when_a_vendor_app_speaks():
    """★ 本用例就是那条**在实车上很难撞见**的路：验收期间冒出第二个司机。

    ⚠️ 工装自己老老实实报了 0（全部通过）—— 作废时**不看它**：
    那些 ✅ 是在错误的前提下算出来的，留着比没有更坏。
    """
    node = FakeNode()
    vc.attach(node)
    node.feed_msg('/cmd_vel', twist(x=0.15))              # 我们自己，不算
    node.feed_msg('/controller/cmd_vel', twist(wz=0.2))   # 第二个司机
    assert vc.finalize(0) == vc.EXIT_VOID


def test_finalize_voids_even_a_failing_run():
    node = FakeNode()
    vc.attach(node)
    node.feed_msg('/app/cmd_vel', twist(x=0.1))
    assert vc.finalize(1) == vc.EXIT_VOID


def test_finalize_is_idempotent_and_settles():
    """作废只报一次；报过之后 `_watch` 必须清掉（否则 atexit 会重复嚷嚷）。"""
    node = FakeNode()
    vc.attach(node)
    node.feed_msg('/controller/cmd_vel', twist(x=0.1))
    assert vc.finalize(0) == vc.EXIT_VOID
    assert vc._watch is None
    assert vc.finalize(0) == 0          # 没有观察者时，原样放行


def test_atexit_net_speaks_up_when_finalize_was_skipped(capsys):
    """★ 兜底那条路：工装从**别的**路径退出（早返回 / 抛异常），漏掉了 `finalize`。

    ⚠️ 这条路**在实车上没法验** —— 要验就得真的让第二个司机说话，
    而那正是要防的事。所以在这里直接叫兜底函数，证明它**真的会说话**：
    一个从没响过的警报和坏掉的警报看起来一样（**坑 50**）。
    """
    node = FakeNode()
    vc.attach(node)
    node.feed_msg('/controller/cmd_vel', twist(x=0.3))
    assert vc._watch is not None and vc._watch.dirty      # 确实有脏东西

    vc._atexit_report()
    err = capsys.readouterr().err
    assert '本次结论作废' in err
    assert '/controller/cmd_vel' in err


def test_atexit_net_stays_quiet_when_clean(capsys):
    vc.attach(FakeNode())
    vc._atexit_report()
    assert capsys.readouterr().err == ''


def test_attach_twice_replaces_the_observer():
    first = FakeNode()
    vc.attach(first)
    second = FakeNode()
    vc.attach(second)
    first.feed_msg('/controller/cmd_vel', twist(x=0.9))     # 旧节点的话不再算数
    assert vc.finalize(0) == 0


# --------------------------------------------------------------------------
# 5. 参数：**不给"关掉闸门"的开关**
# --------------------------------------------------------------------------

def _parse(argv):
    ap = argparse.ArgumentParser()
    vc.add_arguments(ap)
    return ap.parse_args(argv)


def test_cli_defaults_have_no_way_to_disable_the_gate():
    args = _parse([])
    assert args.vendor_watch_topics is None
    assert args.vendor_watch_seconds is None      # ⇒ 观察窗是那个固定的 3.0 s


def test_self_check_topics_are_accepted():
    args = _parse(['--vendor-watch-topics', '/acceptance/fake_cmd'])
    assert args.vendor_watch_topics == ['/acceptance/fake_cmd']


def test_preflight_window_is_a_fixed_constant():
    """⚠️ 它是一个**常量**，不是参数 —— 能关掉的闸门等于没有闸门。"""
    assert vc.PREFLIGHT_SECONDS > 0


def test_exit_codes_are_distinct_and_match_the_check_tool():
    assert vc.EXIT_CLEAN == 0
    assert vc.EXIT_SECOND_DRIVER == 1
    assert vc.EXIT_REFUSE == 4
    # ⚠️ 0 / 1 / 4 是 `vendor_cmd_channel_check.py` 的口径，**不许漂**。
    assert {vc.EXIT_CLEAN, vc.EXIT_SECOND_DRIVER, vc.EXIT_REFUSE} == {0, 1, 4}


#: 会动车的工装**已经在用**的退出码。⚠️ 退出码是这几个程序**共用的命名空间** ——
#: 本模块的任何一个码只要落进这里，两个不同的意思就会长得一模一样。
#: 来源：2026-10-12 逐个 `grep` 出来的（`demo_run` 3 = submit 服务不可用、
#: `safety_chain_suspended_acceptance` 2/3、`suspended_motion_acceptance` 2/3、
#: `phase0_chassis_motion_acceptance` 2、`ground_motion_acceptance` 2、
#: `agent_ground_acceptance` 4、`vendor_cmd_channel_check` 0/1/4）。
_CODES_ALREADY_TAKEN_BY_RIGS = frozenset({0, 1, 2, 3, 4})


def test_void_code_does_not_collide_with_any_rigs_own_exit_code():
    """★ 这条是**从一次真实事故里长出来的**（2026-10-12）。

    "结论作废"第一版取 3 —— 而 3 早已被三个工装用着表示"**依赖没起来**"，
    含义**正好相反**。当场就撞上了：底盘锁存没解除，工装如实返回 3，
    而那个 3 **看起来就像"结论被作废了"**。

    ⇒ 本模块的码，除 0/1/4 那三个（与检查工装刻意的对齐）之外，
    **一个都不许落在工装已用的集合里**。
    """
    assert vc.EXIT_VOID not in _CODES_ALREADY_TAKEN_BY_RIGS, (
        f'EXIT_VOID={vc.EXIT_VOID} 与某个工装自己的退出码撞了 —— '
        f'那会让"依赖没起来"和"结论作废"长得一模一样')
    assert len({vc.EXIT_CLEAN, vc.EXIT_SECOND_DRIVER, vc.EXIT_REFUSE,
                vc.EXIT_VOID}) == 4
