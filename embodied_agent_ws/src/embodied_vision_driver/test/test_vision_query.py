#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住视觉查询的判定 —— 尤其是那条**不对称规则**：
**"找到了"可信；"没找到"只有在**一段窗口**都达标时才可信；其余一律是「不知道」。**
"""

import pytest

from embodied_vision_driver import vision_query as vq


class Det:
    """检出替身（真实消息是 `VisionDetection`，这里只带用得到的字段）。"""

    def __init__(self, label, score, side=0.0):
        self.label, self.score, self.side = label, score, side


def call(**kw):
    """把必填参数填上默认值，测试只写它关心的那几个。

    默认窗口：10 帧、都清楚、**什么都没看到**（一个"底气十足的否定"）。
    """
    base = dict(model_ready=True, label_known=True, frame_age_s=0.05,
                frame_max_age_s=0.5, label='person', known_hint='',
                quality_min=100.0, min_frames=3, min_score=0.3,
                window=vq.Window(frames=10, best=None, quality_min=500.0,
                                 best_below=None))
    base.update(kw)
    return vq.decide(**base)


def window(best=None, frames=10, quality_min=500.0, best_below=None):
    return vq.Window(frames=frames, best=best, quality_min=quality_min,
                     best_below=best_below)


# ==========================================================================
# ★★ 核心：不对称规则
# ==========================================================================

def test_found_is_trusted_even_when_the_image_is_blurry():
    """★ 正面证据不因为画面差而作废 —— 宁可把糊画面里的误检当成"可能有"，
    也不要因为在糊画面上就把它丢掉（漏掉一个真的比多看一眼更贵）。"""
    d = call(window=window(best=Det('person', 0.8), quality_min=5.0))
    assert (d.valid, d.found) == (True, True)


def test_a_single_sighting_wins_over_a_window_full_of_misses():
    """★★ **同一幅静止画面里，模型是时有时无的。**

    实测（2026-10-09）：8 秒 152 帧，`suitcase` 只出现在 **68%** 的帧
    （每帧检出数 0~3）。⇒ 判定必须看**窗口**：只要窗口里**任何一帧**看到了，
    就是看到了 —— 否则东西明明在眼前，却会三次里有一次被说成"没有"。
    """
    d = call(window=window(best=Det('person', 0.4), frames=20, quality_min=400.0))
    assert (d.valid, d.found) == (True, True)
    assert '看到' in d.detail


def test_not_found_on_a_blurry_window_is_NOT_an_answer():
    """★★ 本模块存在的理由。

    "检测器什么都没有"与"画面里真没有东西"看起来一模一样 —— 把前者读成后者，
    上层会据此得出"这里没有目标"的结论。2026-10-09 就是这么栽的：
    相机失焦，所有检测都是 0 个，而当时没人觉得这有什么不对（DEV_NOTES 坑 43）。
    """
    d = call(window=window(best=None, frames=10, quality_min=38.0), quality_min=100.0)
    assert d.valid is False, '糊画面上的"没找到"必须是「不知道」'
    assert d.found is False
    assert '过糊' in d.detail and '不能因此说' in d.detail
    assert 'person' in d.detail


def test_a_window_with_one_blurry_frame_is_not_enough_to_deny():
    """★ 否定用**窗口里最差的那一帧**说话：中间糊过一帧，"没有"就不够硬。

    （这正是"清不清晰"与"有没有"之间的关系：糊的那一帧可能刚好漏掉了它。）
    """
    d = call(window=window(best=None, frames=10, quality_min=40.0), quality_min=100.0)
    assert d.valid is False and '过糊' in d.detail


def test_not_found_on_a_good_window_is_a_real_answer():
    """反向对照：窗口达标时说"没有"是**真结论**，不能被前面那条规则连累。"""
    d = call(window=window(best=None, frames=10, quality_min=800.0))
    assert (d.valid, d.found) == (True, False)
    assert '没有 person' in d.detail
    assert '10 帧' in d.detail, '要说清这个"没有"是几帧的结论'


def test_too_few_frames_is_unknown_not_a_denial():
    """★ 窗口里帧太少 ⇒ 不知道。

    一帧的"没找到"什么都证明不了 —— 它可能是模型刚好打了个盹，
    也可能是画面刚好没刷新。**没有足够的观察，就不能下否定的结论。**
    """
    d = call(window=window(best=None, frames=1, quality_min=800.0), min_frames=3)
    assert d.valid is False
    assert '1 帧' in d.detail and '才够确认' in d.detail


def test_the_frame_floor_is_not_off_by_one():
    assert call(window=window(frames=3), min_frames=3).valid is True
    assert call(window=window(frames=2), min_frames=3).valid is False


# ==========================================================================
# ★ 同族：类别名不在表里，也必须是「不知道」
# ==========================================================================

def test_an_unknown_label_is_unknown_not_absent():
    """★ 问"杯子"而模型只认 `cup` ⇒ 按"没找到"回答会**永远**说"没有杯子" ——
    一个永远不会报错的错答案。所以它必须是「不知道」。"""
    d = call(label_known=False, label='杯子', known_hint='能查的是：person, cup')
    assert d.valid is False
    assert '类别表里没有' in d.detail and '不知道' in d.detail
    assert '不是"没有"' in d.detail


# ==========================================================================
# 其余"没资格回答"的情形
# ==========================================================================

def test_model_not_ready_is_unknown():
    d = call(model_ready=False)
    assert (d.valid, d.found) == (False, False)
    assert '还没就绪' in d.detail


def test_never_received_a_frame_is_unknown():
    d = call(frame_age_s=None)
    assert d.valid is False
    assert '还没有收到任何图像' in d.detail


def test_a_stale_frame_is_unknown_even_if_we_think_we_saw_something():
    """★ "画面停住时没找到"不能算数：那是**没在看的安静**，不是没东西。

    同理，停更时的"找到了"也不该当成此刻的答案 —— 这一问问的是**现在**。
    """
    d = call(frame_age_s=3.0, frame_max_age_s=0.5, window=window(best=Det('person', 0.9)))
    assert d.valid is False
    assert '陈旧' in d.detail


def test_the_age_limit_is_not_off_by_one():
    """正好等于上限算不算老 —— 钉住边界，免得以后有人把 `>` 写成 `>=` 而没人发现。"""
    assert call(frame_age_s=0.5, frame_max_age_s=0.5).valid is True
    assert call(frame_age_s=0.51, frame_max_age_s=0.5).valid is False


# ==========================================================================
# 挑最好的那个（窗口内）
# ==========================================================================

def test_pick_best_takes_the_highest_score():
    dets = [Det('person', 0.3), Det('person', 0.7), Det('chair', 0.9)]
    assert vq.pick_best(dets, 'person', 0.25).score == 0.7


def test_pick_best_ignores_anything_below_the_threshold():
    """阈值由调用方给 —— 这里只验"它真的被用上了"。"""
    dets = [Det('person', 0.2)]
    assert vq.pick_best(dets, 'person', 0.25) is None
    assert vq.pick_best(dets, 'person', 0.15).score == 0.2


def test_pick_best_matches_the_label_without_regard_to_case():
    """模型给的是小写，人手输入时可能大写 —— 别让大小写变成一个"查不到"。"""
    assert vq.pick_best([Det('Person', 0.5)], 'person', 0.1).score == 0.5


def test_pick_best_on_nothing_is_none_not_an_exception():
    assert vq.pick_best([], 'person', 0.1) is None
    assert vq.pick_best(None, 'person', 0.1) is None


def test_pick_best_does_not_match_a_partial_name():
    """`person` 不该匹配到 `personal` 之类 —— 精确匹配，不做近似。"""
    assert vq.pick_best([Det('personal', 0.9)], 'person', 0.1) is None


# ==========================================================================
# 左右
# ==========================================================================

@pytest.mark.parametrize('cx,expected', [(0, -1.0), (320, 0.0), (640, 1.0),
                                         (160, -0.5), (480, 0.5)])
def test_side_of(cx, expected):
    assert vq.side_of(cx, 640) == pytest.approx(expected)


def test_side_of_clamps_outside_the_frame():
    """框跑到画面外时不许外推 —— 上层拿它当"哪一侧"用，±1 就是它的边界。"""
    assert vq.side_of(-100, 640) == -1.0
    assert vq.side_of(999, 640) == 1.0


def test_side_of_a_degenerate_width_is_zero_not_a_crash():
    assert vq.side_of(10, 0) == 0.0


def test_label_is_known_is_case_insensitive():
    assert vq.label_is_known('Person', ['person', 'cup']) is True
    assert vq.label_is_known('bottle', ['person', 'cup']) is False


# ==========================================================================
# ★ 低于门槛的"看到了"也必须说出来（2026-10-09 实机试出来的）
# ==========================================================================

def test_a_sighting_below_the_callers_threshold_is_reported_not_denied():
    """★★ 人站在 1 m 处、模型给 0.84，而调用方要 0.9。

    按定义"没有 ≥0.9 的 person"是对的，但回答"最近 8 帧里都没有 person"
    **读起来是"这里没有人"** —— 而人就在那儿。
    ⇒ 低于门槛**也要说出来**，让调用方自己决定是放宽门槛还是换个办法。
    """
    d = call(min_score=0.9,
             window=window(best=None, best_below=Det('person', 0.84)))
    assert d.valid is True          # 我们**确实看过**了
    assert d.found is False         # 按调用方的标准，不算数
    assert '看到了 person' in d.detail
    assert '0.84' in d.detail and '低于你要的 0.9' in d.detail


def test_a_qualifying_sighting_wins_over_a_sub_threshold_one():
    """够分的那个优先 —— "低于门槛"只是兜底措辞，不该盖过真正的答案。"""
    d = call(min_score=0.3,
             window=window(best=Det('person', 0.85), best_below=None))
    assert (d.valid, d.found) == (True, True)
    assert '看到了 person' in d.detail


def test_too_few_frames_also_reports_a_blind_camera():
    """★ 主因是"帧太少"，但**画面本身不可用**这条线索必须一起给出来。

    2026-10-09 实测撞到过：相机全黑（清晰度 0）**加上** GPU 被另一个模型抢着
    （我们只有 ~5.6 Hz）⇒ 真因（画面不可用）被症状（帧太少）盖住，
    排查的人会去查算力，而该看的是相机。
    """
    d = call(window=window(best=None, frames=2, quality_min=0.0),
             min_frames=3, quality_min=100.0)
    assert d.valid is False
    assert '帧' in d.detail and '清晰度' in d.detail and '先看相机' in d.detail


def test_few_frames_with_a_fine_picture_does_not_add_the_hint():
    """反向对照：画面是好的、只是帧少 —— 别乱加"先看相机"这种误导。"""
    d = call(window=window(best=None, frames=2, quality_min=800.0), min_frames=3)
    assert '先看相机' not in d.detail


# ==========================================================================
# ★★ 只看某一侧（`semantic.look_on_side` 的底）
# ==========================================================================
#
# ⚠️ 这一节里最要紧的一条是最后一条：**写错 side 必须是"不知道"，不是"没有"**。
#    因为"当成不限"恰好会得到一个**看起来完全正常**的答案（整个画面都看），
#    而调用方以为自己问的是"左边"。

def test_on_side_unlimited_accepts_everything():
    """空 = 不限定 ⇒ 老的 `look_for` 行为一点不变。"""
    for v in (-1.0, -0.5, 0.0, 0.5, 1.0):
        assert vq.on_side(v, '')


@pytest.mark.parametrize('value,want,expected', [
    (-1.0, 'left', True), (-0.5, 'left', True),
    (0.0, 'left', False), (0.2, 'left', False), (1.0, 'left', False),
    (1.0, 'right', True), (0.5, 'right', True),
    (0.0, 'right', False), (-0.2, 'right', False), (-1.0, 'right', False),
])
def test_on_side(value, want, expected):
    """画面三等分的外侧两段。⚠️ 中间那 1/3 两边都**不算** ——
    这是有意的：没有一个"中间有多宽"的约定会被场地接受（与 D-028 同一条）。"""
    assert vq.on_side(value, want) is expected


def test_the_boundary_belongs_to_the_side():
    """边界值（恰好 1/3）算**在内** —— "≥ 1/3"这种边界必须钉死，
    否则"刚好在边上"这种帧会在两次调用之间摇摆（与 D-039 同族）。"""
    assert vq.on_side(-vq.SIDE_THRESHOLD, 'left') is True
    assert vq.on_side(vq.SIDE_THRESHOLD, 'right') is True
    assert vq.on_side(vq.SIDE_THRESHOLD, 'left') is False


def test_pick_best_on_a_side_only_considers_that_side():
    """★ 最高分的那个在**另一侧**时，也不能被选中。
    这是"只看左边"能成立的唯一依据 —— 否则它会拿右边的高分当真，
    而返回的 `side` 与调用方问的那一侧自相矛盾。"""
    dets = [Det('person', 0.9, side=0.8),      # 右边，分最高
            Det('person', 0.4, side=-0.8)]     # 左边
    assert vq.pick_best(dets, 'person', 0.1, 'left').score == 0.4
    assert vq.pick_best(dets, 'person', 0.1, 'right').score == 0.9


def test_pick_best_on_a_side_with_nothing_there_is_none():
    """那一侧什么都没有 ⇒ None ⇒ 上层会说"那一侧没有"（这是**有依据**的否定，
    前提是窗口/清晰度也达标 —— 见 `decide`）。"""
    dets = [Det('person', 0.9, side=0.8)]
    assert vq.pick_best(dets, 'person', 0.1, 'left') is None


def test_the_middle_of_the_frame_belongs_to_neither_side():
    """正中间：两边都查不到。⚠️ 这不是缺陷 —— 是**刻意不提供 `center`** 的后果，
    调用方要"中间"就自己用 `side` 的绝对值判（那是调用方的策略）。"""
    dets = [Det('person', 0.9, side=0.1)]
    assert vq.pick_best(dets, 'person', 0.1, 'left') is None
    assert vq.pick_best(dets, 'person', 0.1, 'right') is None
    assert vq.pick_best(dets, 'person', 0.1).score == 0.9      # 不限 ⇒ 找得到


@pytest.mark.parametrize('side', ['', 'left', 'right'])
def test_check_side_accepts_the_three_legal_values(side):
    assert vq.check_side(side) == ''


@pytest.mark.parametrize('side', ['Left', 'LEFT', 'left ', 'middle', 'center',
                                  '左边', 'both', 'up'])
def test_check_side_rejects_anything_else(side):
    """★ **必须报错，不能当成"不限"**。

    当成"不限"的话，"只看左边"会静默地变成"整个画面都看" ——
    而结果**看起来完全正常**，没人会发现问的不是同一件事。
    （大小写和尾空格也不放过：这个字符串是人手敲的，也是 LLM 写的。）
    """
    assert vq.check_side(side) != ''


def test_a_bad_side_is_unknown_NOT_a_denial():
    """★★ 本节存在的理由。

    写错 side 时最省事的做法是"忽略它、照整幅答" —— 那会得到
    "最近 10 帧里都没有 person"，读起来是"这儿没有人"，
    而调用方问的是"**左边**有没有人"。
    ⇒ 必须是 `valid=False`（不知道）⇒ 上层映射成 `FAILED`，**不是** `TARGET_LOST`。
    """
    d = call(want_side='middle')
    assert d.valid is False
    assert d.found is False
    assert '不知道' in d.detail and '不是"没有"' in d.detail
    assert 'middle' in d.detail          # 得把写错的那个值原样报出来


def test_a_bad_side_is_caught_before_anything_else():
    """它排在判定链的最前面：**连"我有没有资格回答"这一步都过不去**，
    所以后面的窗口/清晰度/帧数一概不影响结论（也就不会先说出一个答案、
    再附注一句"其实这个答案不算数"）。"""
    d = call(want_side='center',
             window=window(best=Det('person', 0.9, side=0.9)))
    assert d.valid is False           # 哪怕画面里**真的**看到了
    assert d.found is False


def test_a_good_side_says_which_half_in_words():
    """人话里必须带上"哪半幅" —— 否则上层读到"最近 10 帧里都没有 person"
    会以为问的是整幅画面。"""
    d = call(want_side='left')
    assert d.valid is True and d.found is False
    assert '左半幅' in d.detail


def test_a_side_sighting_says_which_half_in_words():
    d = call(want_side='right', window=window(best=Det('person', 0.8, side=0.7)))
    assert (d.valid, d.found) == (True, True)
    assert '右半幅' in d.detail


def test_without_a_side_nothing_is_said_about_halves():
    """反向对照：不限时**不该**冒出"半幅"字样 —— 那会让人以为限定过。"""
    assert '半幅' not in call().detail
    assert '半幅' not in call(
        window=window(best=Det('person', 0.8))).detail


def test_a_blurry_picture_also_says_which_half():
    """★★ 这条是**真机跑出来的**（2026-10-09，车在暗处、清晰度 0）。

    当时问的是"左半幅"，回来的却是 ——
    「窗口里有画面过糊的帧……**不能因此说"没有 person"**」，**没有一个字提到"左"**。
    读的人（或 Agent）会以为在说整幅画面。
    ⇒ 限定过的查询，**每一句**否定（哪怕是"我不敢说没有"这种）都要带限定。
    """
    d = call(want_side='left', window=window(best=None, quality_min=0.0),
             quality_min=100.0)
    assert d.valid is False
    assert '左半幅' in d.detail


def test_a_sub_threshold_sighting_also_says_which_half():
    """同上：那句"看到了、只是低于门槛"也带着限定（它正是在说**那一侧**看到了）。"""
    d = call(want_side='right', min_score=0.9,
             window=window(best=None, quality_min=500.0,
                           best_below=Det('person', 0.84, side=0.8)))
    assert '右半幅' in d.detail
