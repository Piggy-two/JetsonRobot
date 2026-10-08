#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`kws_engine` 的离线单测 —— **不加载模型、不开麦克风**。

这里钉的是两件"错了会表现为'唤醒不灵'、但看日志完全看不出来"的事：
  · **采样率换算**：48 kHz → 16 kHz 做错了，模型收到的就是一段被拉长/压扁的音频，
    表现是召回率下降，而人会去怀疑发音或模型；
  · **关键词表**：写坏了应当**启动即报错**，而不是等到"喊它没反应"。
"""

import numpy as np
import pytest

from embodied_voice_wakeup.kws_engine import (
    TARGET_RATE, parse_keywords, resample_to_16k)


# ---------- 采样率换算 ----------

def test_16k_passes_through_untouched():
    """已经是 16 kHz 就不要动它 —— 任何"顺手再过一道"都可能引入误差。"""
    x = np.array([0.1, -0.2, 0.3], dtype=np.float32)
    assert resample_to_16k(x, TARGET_RATE) is x


def test_48k_becomes_exactly_one_third():
    x = np.arange(48000, dtype=np.float32)
    y = resample_to_16k(x, 48000)
    assert len(y) == 16000


def test_dc_level_is_preserved():
    """常量信号必须原样出来 —— 取均值而不是抽样，就是为了这一条。

    如果实现是"每 3 个取第 1 个"，直流也对；但换成有噪声的信号就会露馅
    （见下一条）。这条是基础正确性。
    """
    x = np.full(30000, 0.25, dtype=np.float32)
    y = resample_to_16k(x, 48000)
    assert np.allclose(y, 0.25)


def test_averaging_beats_picking_for_noisy_signal():
    """**这一条是"取均值"与"抽样"的分水岭。**

    构造一个 48 kHz 的方波：抽样会在某个相位上永远取到高电平或低电平，
    而取均值会把每一组的三点摊平。用"输出里有多少个不同的值"来区分：
    抽样只有 2 个值，取均值会多得多。
    """
    x = np.tile(np.array([1.0, 0.0, 0.0], dtype=np.float32), 4000)
    y = resample_to_16k(x, 48000)
    assert len(np.unique(np.round(y, 6))) == 1        # 每 3 个正好是 (1,0,0) → 均值 1/3
    assert np.allclose(y, 1.0 / 3.0)


def test_trailing_samples_are_dropped_not_kept():
    """长度不是整数倍时丢掉尾巴，而不是让它变成半个样本。"""
    x = np.arange(48001, dtype=np.float32)             # 48000 的 1 倍 + 1
    y = resample_to_16k(x, 48000)
    assert len(y) == 16000


def test_non_integer_factor_is_refused():
    """44.1 kHz 不是 16 kHz 的整数倍 —— **明确拒绝**，不要悄悄线性插值。"""
    with pytest.raises(ValueError):
        resample_to_16k(np.zeros(44100, dtype=np.float32), 44100)


def test_empty_input_is_fine():
    y = resample_to_16k(np.zeros(0, dtype=np.float32), 48000)
    assert len(y) == 0


# ---------- 关键词表 ----------

def test_parses_phonemes_and_display_name(tmp_path):
    p = tmp_path / 'kw.txt'
    p.write_text('x iǎo h uàn x iǎo h uàn @小幻小幻\n', encoding='utf-8')
    kws = parse_keywords(str(p))
    assert kws == [('小幻小幻', ['x', 'iǎo', 'h', 'uàn', 'x', 'iǎo', 'h', 'uàn'])]


def test_missing_display_name_is_accepted_but_not_pretended(tmp_path):
    """漏了 `@显示名` 让它能用，但显示名退化成音素串 —— 不要假装它是对的。"""
    p = tmp_path / 'kw.txt'
    p.write_text('x iǎo\n', encoding='utf-8')
    (name, phon), = parse_keywords(str(p))
    assert name == 'x iǎo'
    assert phon == ['x', 'iǎo']


def test_blank_lines_and_blank_file(tmp_path):
    p = tmp_path / 'kw.txt'
    p.write_text('\n\n  \n', encoding='utf-8')
    with pytest.raises(ValueError):
        parse_keywords(str(p))                    # 一个词都没有 ⇒ 启动就该炸


def test_display_name_without_phonemes_is_refused(tmp_path):
    p = tmp_path / 'kw.txt'
    p.write_text('@只有名字\n', encoding='utf-8')
    with pytest.raises(ValueError):
        parse_keywords(str(p))
