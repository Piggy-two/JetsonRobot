#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`vision_acceptance` 的离线单测 —— **不碰相机、不碰网关、不动车**。

这一份钉两件事，而它们都来自 2026-10-12 那次真实发现：

1. **`position_of` 的判读**：它是 [5b] 唯一的判据，读错一个字段名就会
   让"位置写反"这一格**静默通过**。
2. **正面格不许被删掉**：这套工装原先 `look_on_side` 的三个结论**全是
   `TARGET_LOST` / `FAILED`** —— 于是**一个永远返回 `TARGET_LOST` 的实现
   能整套跑绿**。用例补上了，就得有东西看着它别被删回去。

    python3 -m pytest tools/test_vision_acceptance.py -q
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import vision_acceptance as va          # noqa: E402

SRC = Path(va.__file__).read_text()


# --------------------------------------------------------------------------
# position_of：读的是网关 result_json 里的 `position`
# --------------------------------------------------------------------------

def test_reads_the_position_field():
    assert va.position_of('{"position": -0.8, "score": 0.4}') == pytest.approx(-0.8)
    assert va.position_of('{"position": 0.9}') == pytest.approx(0.9)


def test_zero_is_a_real_value_not_a_miss():
    """⚠️ `position=0.0` 是合法读数（正中间），**不许**被当成"没读到"。"""
    assert va.position_of('{"position": 0.0}') == 0.0


@pytest.mark.parametrize('raw', [
    '', '{}', '不是 JSON', None,
    '{"side": -0.8}',              # ⚠️ `look_for` 用的名字，**不许**串用
    '{"position": null}',
    '{"position": "left"}',
    '{"position": true}',          # bool 是 int 的子类 —— 别让它蒙混过去
])
def test_missing_or_wrong_shaped_position_yields_none(raw):
    """取不到就 `None` —— **绝不编一个数出来**（编了 [5b] 就凭空通过）。"""
    assert va.position_of(raw) is None


# --------------------------------------------------------------------------
# 结构性检查：`look_on_side` 的**正面格**必须还在
# --------------------------------------------------------------------------

def test_look_on_side_has_a_positive_case():
    """★ 这条是**从一次真实发现里长出来的**（2026-10-12）。

    本工装原先 `look_on_side` 的用例**全部**断言 `TARGET_LOST`/`FAILED` ——
    也就是说**一个"永远返回 `TARGET_LOST`"的实现能整套跑绿**。
    缺的那格不是"没跑过"，是**工装里压根没有这条用例**。

    ⇒ 用一条结构性检查看着它：正面格（带 `side=` 去问、且断言 `TARGET_FOUND`）在，
    且它与反面格**成对**。
    """
    assert 'side=here' in SRC, '正面格不见了：`look_on_side` 又只剩反面用例了'
    assert 'TARGET_FOUND' in SRC.split('side=here', 1)[1].split('side=other')[0], \
        '正面格问了 `side=here`，却没有断言 TARGET_FOUND'
    assert 'side=other' in SRC, '反面格不见了'


def test_both_halves_are_skipped_together():
    """⚠️ 用例构造不出来时，三格必须**一起跳过** ——
    只跑一半会给出"看起来通过"的错结论。"""
    for tag in ('[5a]', '[5b]', '[5c]'):
        assert f"skip('{tag}" in SRC, f'{tag} 没有跟着一起跳过'


# --------------------------------------------------------------------------
# 判据本身：那条结构不变量与 side 余量
# --------------------------------------------------------------------------

def test_side_margin_matches_the_rule_it_claims_to_respect():
    """⚠️ 余量 0.5 比规则的 1/3 更严是**刻意的**：贴着中线的实例算"不算数"，
    而不是硬测出一个与 `side` 无关的结论（见 `SIDE_MARGIN` 的说明）。"""
    assert va.SIDE_MARGIN >= 1 / 3


def test_case_list_in_the_docstring_matches_the_labels_used():
    """文档里的用例清单与实际打出来的标签**不许分家**。"""
    for tag in ('[1]', '[2]', '[3]', '[4]', '[5a]', '[5b]', '[5c]', '[6]'):
        assert f'rep(f\'{tag}' in SRC or f"rep('{tag}" in SRC or f'skip({tag!r}' in SRC \
            or f"skip('{tag}" in SRC, f'{tag} 在清单里，但代码里打不出这个标签'
