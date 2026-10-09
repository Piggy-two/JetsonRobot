#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住视觉查询的判定 —— 尤其是那条**不对称规则**：
**"找到了"可信；"没找到"只有在画面质量达标时才可信；其余一律是「不知道」。**
"""

import pytest

from embodied_vision_driver import vision_query as vq


class Det:
    """检出替身（真实消息是 `VisionDetection`，这里只要 `.label` / `.score`）。"""

    def __init__(self, label, score):
        self.label, self.score = label, score


def call(**kw):
    """把必填参数填上默认值，测试只写它关心的那几个。"""
    base = dict(model_ready=True, label_known=True, frame_age_s=0.05,
                frame_max_age_s=0.5, image_quality=500.0, quality_min=100.0,
                best=None, label='person')
    base.update(kw)
    return vq.decide(**base)


# ==========================================================================
# ★★ 核心：不对称规则
# ==========================================================================

def test_found_is_trusted_even_when_the_image_is_blurry():
    """★ 正面证据不因为画面差而作废 —— 宁可把糊画面里的误检当成"可能有"，
    也不要因为在糊画面上就把它丢掉（漏掉一个真的比多看一眼更贵）。"""
    d = call(best=Det('person', 0.8), image_quality=5.0)
    assert (d.valid, d.found) == (True, True)


def test_not_found_on_a_blurry_image_is_NOT_an_answer():
    """★★ 本模块存在的理由。

    "检测器什么都没有"与"画面里真没有东西"看起来一模一样 —— 把前者读成后者，
    上层会据此得出"这里没有目标"的结论。2026-10-09 就是这么栽的：
    相机失焦，所有检测都是 0 个，而当时没人觉得这有什么不对（DEV_NOTES 坑 43）。
    """
    d = call(best=None, image_quality=38.0, quality_min=100.0)
    assert d.valid is False, '糊画面上的"没找到"必须是「不知道」'
    assert d.found is False
    assert '过糊' in d.detail and '不能因此说' in d.detail
    assert 'person' in d.detail


def test_not_found_on_a_good_image_is_a_real_answer():
    """反向对照：画面达标时说"没有"是**真结论**，不能被前面那条规则连累。"""
    d = call(best=None, image_quality=800.0, quality_min=100.0)
    assert (d.valid, d.found) == (True, False)
    assert '没有 person' in d.detail


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
    d = call(frame_age_s=3.0, frame_max_age_s=0.5, best=Det('person', 0.9))
    assert d.valid is False
    assert '陈旧' in d.detail


def test_the_age_limit_is_not_off_by_one():
    """正好等于上限算不算老 —— 钉住边界，免得以后有人把 `>` 写成 `>=` 而没人发现。"""
    assert call(frame_age_s=0.5, frame_max_age_s=0.5).valid is True
    assert call(frame_age_s=0.51, frame_max_age_s=0.5).valid is False


# ==========================================================================
# 挑最好的那个
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
