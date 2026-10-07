#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令解析的单测。

⚠️ 本文件 import 了 `embodied_safety_runtime.estop`（解析器复用它做安全词判定），
所以跑测试时那个包也要能被 import —— 从本包目录跑即可（两包同在一个工作区）。
"""

import math

import pytest

from embodied_command_router import command_parser as cp


def _assert_det(text, skill, expect, tol=1e-6):
    r = cp.parse(text)
    assert r.kind == cp.DETERMINISTIC, f'{text!r} 应被解析为确定性命令，实得 {r}'
    assert r.skill == skill, f'{text!r} 应走 {skill}，实得 {r.skill}'
    for k, v in expect.items():
        assert r.args[k] == pytest.approx(v, abs=tol), f'{text!r} 的 {k} 不对：{r.args}'


# ---------- 分类 ----------

@pytest.mark.parametrize('text', ['停', '停下', '急停', '别动', 'stop', 'emergency stop'])
def test_safety_words_are_classified_as_safety(text):
    assert cp.parse(text).kind == cp.SAFETY


def test_safety_classification_reuses_the_shared_whole_sentence_matching():
    """★ 必须复用 `estop.py` 的整句匹配 —— `停止追踪` **不是**安全词。

    子串匹配会让厂商词表里的「停止追踪」「停止分拣」全部误触发整机急停。
    这个取舍只在 estop.py 里写一次，路由器不许自己再实现一遍。
    """
    assert cp.parse('停止追踪').kind != cp.SAFETY
    assert cp.parse('停止分拣').kind != cp.SAFETY


@pytest.mark.parametrize('text', ['唤醒成功(wake-up-success)', '唤醒成功', '睡眠'])
def test_vendor_status_text_is_ignored_not_executed(text):
    assert cp.parse(text).kind == cp.IGNORE


@pytest.mark.parametrize('text', ['去桌子旁边找杯子', '帮我拿一下水杯', '然后回到原点'])
def test_task_like_text_is_classified_as_agent_task(text):
    assert cp.parse(text).kind == cp.AGENT_TASK


# ---------- 平移 ----------

@pytest.mark.parametrize('text', ['向前走0.5米', '前进0.5米', '向前0.5m', '往前0.5米'])
def test_forward_variants(text):
    _assert_det(text, 'control.move_relative', {'x': 0.5, 'y': 0.0})


@pytest.mark.parametrize('text', ['后退0.3米', '向后0.3米'])
def test_backward_variants(text):
    _assert_det(text, 'control.move_relative', {'x': -0.3, 'y': 0.0})


@pytest.mark.parametrize('text', ['左移0.2米', '向左移动0.2米', '往左0.2米'])
def test_strafe_left_variants(text):
    _assert_det(text, 'control.move_relative', {'x': 0.0, 'y': 0.2})


@pytest.mark.parametrize('text', ['右移0.2米', '向右移动0.2米'])
def test_strafe_right_variants(text):
    _assert_det(text, 'control.move_relative', {'x': 0.0, 'y': -0.2})


def test_centimeters_are_converted_not_taken_literally():
    """`50厘米` 是 0.5 米。把 50 当米执行是一次 100 倍的错误运动。"""
    _assert_det('向前50厘米', 'control.move_relative', {'x': 0.5, 'y': 0.0})


# ---------- 旋转 ----------

def test_left_turn_is_counter_clockwise_positive():
    _assert_det('左转90度', 'control.rotate', {'angle': math.pi / 2})


def test_right_turn_is_clockwise_negative():
    _assert_det('右转30度', 'control.rotate', {'angle': -math.pi / 6})


def test_degree_sign_is_accepted():
    _assert_det('右转30°', 'control.rotate', {'angle': -math.pi / 6})


def test_radians_are_accepted_verbatim():
    _assert_det('左转1.5708弧度', 'control.rotate', {'angle': 1.5708})


def test_strafe_and_turn_are_never_confused():
    """★ 「左移」与「左转」必须分得清清楚楚 —— 一个平移、一个旋转。

    这两个词只差一个字，而执行结果完全不同（一个横着走、一个原地转）。
    """
    assert cp.parse('左移0.2米').skill == 'control.move_relative'
    assert cp.parse('左转0.2度').skill == 'control.rotate'
    assert cp.parse('向左移动0.3米').args == {'x': 0.0, 'y': 0.3}
    assert cp.parse('向左转30度').args == {'angle': pytest.approx(math.pi / 6)}


# ---------- 中文数字 ----------

@pytest.mark.parametrize('text,expect', [
    ('向前零点五米', 0.5),
    ('向前一米', 1.0),
    ('向前十米', 10.0),
    ('向前十五米', 15.0),
    ('向前二十五厘米', 0.25),
])
def test_chinese_numerals(text, expect):
    r = cp.parse(text)
    assert r.kind == cp.DETERMINISTIC, r
    assert abs(r.args['x']) == pytest.approx(expect)


@pytest.mark.parametrize('text,expect', [
    ('左转三十度', math.pi / 6),
    ('左转九十度', math.pi / 2),
    ('左转一百八十度', math.pi),
])
def test_chinese_numerals_in_angles(text, expect):
    _assert_det(text, 'control.rotate', {'angle': expect}, tol=1e-5)


# ---------- 拒绝（不猜） ----------

@pytest.mark.parametrize('text', ['左转90', '向前0.5', '右转30'])
def test_missing_unit_is_rejected_not_guessed(text):
    """★ 缺单位必须**拒绝**。

    `左转3` 按度是 3°、按弧度是 172° —— 差 57 倍；`向前50` 按米是 50 m、
    按厘米是 0.5 m —— 差 100 倍。给默认值就是把"猜错"变成一次**无声的、
    方向性的**错误运动；拒绝对用户的代价只是再说一遍。
    """
    r = cp.parse(text)
    assert r.kind == cp.UNPARSED
    assert '单位' in r.note


@pytest.mark.parametrize('text', ['左转90斤', '向前0.5光年', '右转3秒'])
def test_unknown_units_are_rejected(text):
    assert cp.parse(text).kind == cp.UNPARSED


def test_rotation_without_a_direction_is_rejected():
    r = cp.parse('原地转90度')
    assert r.kind == cp.UNPARSED
    assert '方向' in r.note


def test_strafe_without_direction_is_rejected():
    assert cp.parse('移动0.5米').kind != cp.DETERMINISTIC


def test_text_without_a_number_is_rejected():
    r = cp.parse('向前走')
    assert r.kind == cp.UNPARSED
    assert '数值' in r.note


def test_unit_must_be_adjacent_to_the_number():
    """`前进1米，后退2厘米` 是两句连写，不能被解析成一个命令。"""
    r = cp.parse('前进1米后退2厘米')
    assert r.kind == cp.UNPARSED


def test_zero_magnitude_is_rejected():
    assert cp.parse('向前0米').kind == cp.UNPARSED


def test_empty_text_is_ignored():
    assert cp.parse('').kind == cp.IGNORE
    assert cp.parse(None).kind == cp.IGNORE


def test_leading_dot_number_is_not_mangled():
    """⚠️ 这是不能用 `estop.normalize` 做解析归一化的原因 —— 它会删掉小数点，
    `0.5` 会变成 `05`（也就是 5）。"""
    _assert_det('向前0.5米', 'control.move_relative', {'x': 0.5, 'y': 0.0})


# ---------- 解析结果的说明文字 ----------

def test_note_explains_what_was_understood():
    """解析成功时 note 要写清"我理解成什么" —— 用户与日志都要能核对。"""
    r = cp.parse('向前50厘米')
    assert '0.5' in r.note or '0.500' in r.note
    r2 = cp.parse('左转90度')
    assert '90.0' in r2.note
