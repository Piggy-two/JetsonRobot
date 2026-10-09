#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 `look_for` 的语义 —— **它是三行映射，而每一行都有代价**。

⚠️ 这里最要紧的一条是"**不知道不能报成没有**"：这条一旦松了，
上层就会在**画面糊了**的时候得出"这里没有目标"的结论，
而那种错误**不会报错、也不会崩**，只会让人相信一个假的否定。
"""

import pytest

from embodied_skill_gateway import task_state as ts

from embodied_semantic_skills import look_plan


# ==========================================================================
# ★★ 状态映射
# ==========================================================================

def test_found_maps_to_target_found():
    assert look_plan.state_for(valid=True, found=True) == ts.TARGET_FOUND


def test_a_confident_negative_maps_to_target_lost():
    """画面质量达标 ⇒ 这个"没有"是真的 ⇒ TARGET_LOST（"目标不在"）。"""
    assert look_plan.state_for(valid=True, found=False) == ts.TARGET_LOST


def test_unknown_maps_to_failed_and_NEVER_to_target_lost():
    """★★ 本模块存在的理由。

    `valid=False` 有四种来源：画面过糊 / 没有新鲜帧 / 模型没就绪 / 类别不在表里。
    这四种**都**是"我不知道"，而不是"没有目标"。
    把它们报成 `TARGET_LOST`，上层就会据此行动 —— 而真相可能是镜头糊了。
    """
    assert look_plan.state_for(valid=False, found=False) == ts.FAILED
    assert look_plan.state_for(valid=False, found=False) != ts.TARGET_LOST


def test_valid_false_with_found_true_is_still_failed():
    """`valid=False` 是**先决条件**：`found` 无论是什么都无意义。

    （驱动当前不会产出这种组合；但**接口允许**，所以语义要定死，
    免得哪天组合一出现，两个模块各自"讲道理"讲出两种结果。）
    """
    assert look_plan.state_for(valid=False, found=True) == ts.FAILED


# ==========================================================================
# success 的含义
# ==========================================================================

def test_success_only_for_found():
    assert look_plan.success_for(ts.TARGET_FOUND) is True


@pytest.mark.parametrize('state', [ts.TARGET_LOST, ts.FAILED, ts.CANCELLED,
                                   ts.BLOCKED, ts.ARRIVED])
def test_success_is_false_for_everything_else(state):
    """⚠️ **`TARGET_LOST` 的 `success=False` 不是"技能坏了"** ——
    它的含义是"达成了本技能的目标条件吗？没有"。把它读成故障是本项目反复提醒的误读（D-032）。"""
    assert look_plan.success_for(state) is False


# ==========================================================================
# 边界上的置信度
# ==========================================================================

@pytest.mark.parametrize('given,expected', [
    (0.5, 0.5), (0.0, 0.0), (1.0, 1.0),
    (1.0000001, 1.0),          # float32 往返之后可能超一点点
    (-0.0000001, 0.0),
    (2.0, 1.0), (-1.0, 0.0),
])
def test_bounded_score(given, expected):
    assert look_plan.bounded_score(given) == pytest.approx(expected)


def test_bounded_score_survives_junk():
    """拿不到数就当 0 —— 一个"没找到"不该因为一个畸形字段而变成崩溃。"""
    assert look_plan.bounded_score(None) == 0.0
    assert look_plan.bounded_score('nonsense') == 0.0


# ==========================================================================
# ★ "问的是不是一句完整的话" —— 空 side 不是"不限"，是"没问"
# ==========================================================================

@pytest.mark.parametrize('side', ['left', 'right', ' left ', 'LEFT'])
def test_a_named_side_passes(side):
    """非空就算"问了一个侧" —— 值合不合法是**下一层**的事（`vision_query.check_side`），
    这里只管"有没有问"，不复制一份词表（D-034）。"""
    assert look_plan.check_asked_side(side) == ''


@pytest.mark.parametrize('side', ['', '   ', None])
def test_an_empty_side_is_a_missing_question(side):
    """★★ 空值**不是**"不限"，是"没问"。

    如果空值被当成"整幅"，调用方想问"**左半幅**有没有人"，
    却会拿到一个**关于整个画面**的回答 —— 而且**两个结果都长得对**：
    不会报错、不会崩、也不会有人发现（两处的 `TARGET_LOST` 含义本来就不同）。
    这就是本项目反复遇到的那一族：**一个永远不会报错的错答案**。
    ⇒ 要整幅就调 `semantic.look_for`。
    """
    why = look_plan.check_asked_side(side)
    assert why != ''
    assert '没给 side' in why
    assert 'look_for' in why          # 拒绝必须**指出正确的做法**，否则读数的人只学会绕圈
