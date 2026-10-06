#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`estop` 的离线单测（不需要 ROS、不需要底盘、不经过任何语音硬件）。

安全词判定的两种错法都要钉住：**该停的必须停**，**不该停的不能停**。
后者常被忽略，但它是安全通路可信度的来源 ——
一个动不动就整机停住的按钮，很快就会被当成故障而不是保护。
"""
import pytest

from embodied_safety_runtime.estop import (
    DEFAULT_SAFETY_PHRASES, EStopLatch, is_safety_command, normalize)


# ---------- 该停的必须停 ----------

@pytest.mark.parametrize('text', [
    '停下', '停', '停下来', '停止', '急停', '紧急停止', '别动', '不要动', '取消任务',
    'stop', 'Stop', 'STOP', 'halt', 'freeze', 'emergency stop', 'cancel task',
])
def test_recognises_safety_commands(text):
    hit, phrase = is_safety_command(text)
    assert hit, f'{text!r} 应该被判为安全指令'
    assert phrase


def test_tolerates_spacing_and_punctuation():
    for text in [' 停 下 ', '停下。', '停 下！', 'stop!', 'Stop.', ' stop ']:
        assert is_safety_command(text)[0], f'{text!r} 应被识别'


def test_chinese_with_inserted_spaces():
    """中文 ASR 有时会插空格 —— 归一化要吸收掉。"""
    assert is_safety_command('停   下')[0]


# ---------- 不该停的不能停 ----------

@pytest.mark.parametrize('text', [
    '停止追踪',            # ← 厂商 BNF 里的功能词，只停"追踪"这个功能
    '停止分拣',
    'stop tracking',
    'stop color recognition',
    'stop color sorting',
    'stop gripping',
])
def test_functional_stop_words_do_not_trigger_estop(text):
    """⚠️ 这条是**刻意**的取舍，不是遗漏。

    若改用子串匹配，`停` 会让这一整类"只停某个功能"的短语全部触发整机急停 ——
    用户说"停止追踪"机器人停住，再说一次还是停住，安全通路立刻失去可信度。
    详见 `estop.py` 里 `is_safety_command` 的说明。
    """
    assert not is_safety_command(text)[0], f'{text!r} 不该触发整机急停'


@pytest.mark.parametrize('text', [
    '唤醒成功(wake-up-success)',   # 厂商会往同一条话题发这些状态文本
    '休眠(Sleep)',
    '失败5次(Fail-5-times)',
    '失败10次(Fail-10-times)',
    '',
    None,
    'go forward',
    'turn left',
])
def test_non_safety_text_does_not_trigger(text):
    assert not is_safety_command(text)[0]


# ---------- 词表可配置 ----------

def test_custom_phrases_replace_the_defaults():
    assert not is_safety_command('趴下', ['放下'])[0]
    assert is_safety_command('趴下', ['趴下'])[0]


def test_default_phrase_list_is_not_empty():
    assert len(DEFAULT_SAFETY_PHRASES) > 0


def test_normalize_strips_punct_and_lowercases():
    assert normalize('  Stop， 追踪！ ') == 'stop追踪'


# ---------- 锁存 ----------

def test_latch_triggers_and_requires_explicit_release():
    l = EStopLatch()
    assert not l.latched
    assert l.trigger('voice:停') is True        # 状态发生变化
    assert l.latched and l.reason == 'voice:停'
    # 再触发一次不算"变化"（便于只对进入记一次日志），但仍然保持锁存
    assert l.trigger('voice:停') is False
    assert l.latched
    assert l.release() is True
    assert not l.latched and l.reason == ''
    assert l.release() is False                 # 本来就是解除的


def test_latch_reason_is_overwritten_by_latest_trigger():
    l = EStopLatch()
    l.trigger('a')
    l.trigger('b')
    assert l.latched and l.reason == 'b'
