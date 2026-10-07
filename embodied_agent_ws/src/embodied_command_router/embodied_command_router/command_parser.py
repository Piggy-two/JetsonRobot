#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令分类与本地解析（**纯 Python，不依赖 ROS**）。

`plan.md` §10 / `README.md` §2.3 把命令分成三类，规则已经定死，本模块照做：

| 类别 | 例子 | 去向 | 经 LLM |
|---|---|---|---|
| Safety Command | 停 / 急停 / 别动 | Safety Runtime → 停车 | ❌ |
| Deterministic Command | 向前走 0.5 米 / 右转 30° | Control Skill（经网关） | ❌ |
| Agent Task | 去桌子旁找杯子 | Agent Runtime → LLM Planner | （将来 ✅） |

本模块负责**判断是哪一类**，以及**把确定性命令解析成结构化调用**。

两个刻意的设计取舍
------------------

**1. 安全词判定复用 `estop.py`，绝不重写。**
   那份实现用的是**整句匹配**（`is_safety_command`），理由写在它的 docstring 里：
   子串匹配会让厂商词表里的 `停止追踪` / `停止分拣` 全部误触发整机急停。
   那是安全依据，必须只有一处。

**2. 数字没带单位就拒绝，不猜。**
   `左转 3` 究竟指 3° 还是 3 rad？**差 57 倍**。`向前 50` 是 50 米还是 50 厘米？
   差 100 倍。给一个默认值会让"猜错"变成一次**无声的、方向性的**错误运动 ——
   而拒绝对用户的代价只是再说一遍。所以：

      · 平移：只认 米 / m / 公尺 / 厘米 / cm
      · 旋转：只认 度 / ° / deg / 弧度 / rad

   这条与本项目一贯的"**宁可拒绝，不猜**"一致。

一个**没有**做的事（以及为什么）
--------------------------------
本模块**不**处理「取消任务」。原因：厂商的安全词表 `DEFAULT_SAFETY_PHRASES` 里
**已经含**「取消任务」，而 Safety Runtime **独立订阅同一条语音话题**并在本节点内匹配 ——
所以用户说「取消任务」时，**整机急停会被锁存**（要 `~/release` 才解除），
这与本模块做什么无关。

若本模块再加一条"温和取消"分支，会造成**同一个词有两个不同解释**：
上层以为只是取消了当前动作，实际机器人已经急停锁存。那比不做更糟。
真要区分，需要与 Safety Runtime 的默认词表**一起**决策（属后续工作），
不能在路由器里单方面改语义。
"""

import re

from embodied_safety_runtime.estop import is_safety_command

# 命令类别
SAFETY = 'safety'
DETERMINISTIC = 'deterministic'
AGENT_TASK = 'agent_task'
IGNORE = 'ignore'          # 厂商的状态提示音文本，不是用户的命令
UNPARSED = 'unparsed'      # 听到了，但解析不出来 —— **拒绝，不猜**


class ParsedCommand:
    """解析结果。`skill` / `args` 只在 `DETERMINISTIC` 时非空。"""

    __slots__ = ('kind', 'skill', 'args', 'note')

    def __init__(self, kind, skill=None, args=None, note=''):
        self.kind = kind
        self.skill = skill
        self.args = args or {}
        self.note = note

    def __repr__(self):
        return f'ParsedCommand({self.kind}, {self.skill}, {self.args}, {self.note!r})'


# ---------- 归一化 ----------
# ⚠️ 不能直接用 `estop.normalize`：它会**删掉 `.`**，`0.5` 会变成 `05`。
#    安全词匹配继续用它的（那是安全依据），解析用这一份。
_STRIP = ' \t\r\n，。！？、；：""\'\'（）()[]{}!?;:'

_ROTATE_MARK = ('转', 'turn')
_LEFT_WORDS = ('向左', '左转', '逆时针', '逆', 'left', 'counter')
_RIGHT_WORDS = ('向右', '右转', '顺时针', '顺', 'right', 'clock')
_FORWARD_WORDS = ('向前', '前进', '往前', '前走', '前移', 'forward', 'ahead')
_BACK_WORDS = ('向后', '后退', '往后', '后移', 'backward', 'back')
_STRAFE_LEFT_WORDS = ('向左', '左移', '往左', '左平移', '左走')
_STRAFE_RIGHT_WORDS = ('向右', '右移', '往右', '右平移', '右走')

# 单位 → (换算系数, 说明)。**只认这些**，别的一律拒绝。
_LINEAR_UNITS = (('厘米', 0.01), ('cm', 0.01), ('米', 1.0), ('m', 1.0), ('公尺', 1.0))
_ANGLE_UNITS = (('弧度', 1.0), ('rad', 1.0), ('度', 0.017453292519943295),
                ('°', 0.017453292519943295), ('deg', 0.017453292519943295))

# 厂商状态提示音，不是用户命令 —— 记它们会让日志淹没在噪声里
_IGNORE_MARKS = ('唤醒成功', 'wake-up-success', '唤醒失败', '睡眠', 'sleep')

# 「这看起来是个复杂任务」的弱启发式。⚠️ 只是**决定拒绝时说什么**，
# 因为复杂任务今天也走不通（Phase 7 才有 LLM）。所以判错的代价很低。
_TASK_HINTS = ('去', '找', '帮忙', '然后', '如果', '拿', '送', '看看', '巡逻',
               '巡检', '跟随', '搜索', '回到')


def normalize(text):
    """解析用的归一化：去空白与标点、英文转小写。**保留数字、`.` 与 `°`。**"""
    if text is None:
        return ''
    s = str(text).strip().lower()
    for ch in _STRIP:
        s = s.replace(ch, '')
    return s


# ---------- 数字 ----------

_CN_DIGITS = {'零': 0, '〇': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
              '五': 5, '六': 6, '七': 7, '八': 8, '九': 9}
_CN_UNITS = {'十': 10, '百': 100, '千': 1000}

_ARABIC_RE = re.compile(r'(\d+(?:\.\d+)?)')
_CN_RE = re.compile(r'([零〇一二三四五六七八九十百千两]+(?:\.[0-9]+)?'
                    r'(?:点[零〇一二三四五六七八九]+)?)')


def _cn_int_to_int(s):
    """中文数字整数部分 → int（支持到 9999）。解析不了返回 None。"""
    total = section = number = 0
    for ch in s:
        if ch in _CN_DIGITS:
            number = _CN_DIGITS[ch]
        elif ch in _CN_UNITS:
            unit = _CN_UNITS[ch]
            if number == 0:
                number = 1        # 「十」单用 = 10
            section += number * unit
            number = 0
        else:
            return None
    return total + section + number


def _cn_to_float(s):
    """中文数字 → float，支持「点」小数。解析不了返回 None。"""
    if '点' in s:
        head, _, tail = s.partition('点')
        whole = _cn_int_to_int(head) if head else 0
        if whole is None:
            return None
        frac_digits = ''
        for ch in tail:
            if ch not in _CN_DIGITS:
                return None
            frac_digits += str(_CN_DIGITS[ch])
        return whole + (float('0.' + frac_digits) if frac_digits else 0.0)
    v = _cn_int_to_int(s)
    return None if v is None else float(v)


def find_number(text):
    """在文本里找第一个数值。返回 (值, 起始下标, 终止下标)，找不到返回 (None, -1, -1)。

    阿拉伯数字优先（`0.5米` 比中文更常见），中文数字作后备（`三十度`）。
    """
    m = _ARABIC_RE.search(text)
    if m:
        return float(m.group(1)), m.start(), m.end()
    m = _CN_RE.search(text)
    if m:
        v = _cn_to_float(m.group(1))
        if v is not None:
            return v, m.start(), m.end()
    return None, -1, -1


def find_unit(text, units):
    """在文本里找单位。返回 (换算系数, 单位字面, 起始下标)，找不到返回 (None, '', -1)。"""
    for literal, factor in units:
        idx = text.find(literal)
        if idx >= 0:
            return factor, literal, idx
    return None, '', -1


# ---------- 分类 ----------

def classify(text):
    """粗分类：先安全词，再确定性命令的形状，最后才是复杂任务。"""
    s = normalize(text)
    if not s:
        return IGNORE

    # ① 安全词：**复用 estop.py 的整句匹配**，不重写。
    hit, phrase = is_safety_command(text)
    if hit:
        return SAFETY

    # ② 厂商的状态提示音
    if any(mark in s for mark in _IGNORE_MARKS):
        return IGNORE

    # ③ 看起来是确定性命令的形状吗（有方向词或「转」）
    if any(w in s for w in _ROTATE_MARK) or \
            any(w in s for w in _FORWARD_WORDS + _BACK_WORDS +
                _STRAFE_LEFT_WORDS + _STRAFE_RIGHT_WORDS):
        return DETERMINISTIC

    # ④ 复杂任务 vs 听不懂 —— 只影响**拒绝时说什么**（两者今天都不执行）
    if any(w in s for w in _TASK_HINTS):
        return AGENT_TASK
    return UNPARSED


# ---------- 解析 ----------

def parse(text):
    """文本 → `ParsedCommand`。"""
    kind = classify(text)
    if kind != DETERMINISTIC:
        return ParsedCommand(kind, note={'safety': '安全词，交给 Safety Runtime',
                                         'ignore': '厂商状态文本，忽略',
                                         'agent_task': '复杂任务，需要 Agent 规划（Phase 7 未实现）',
                                         'unparsed': '无法本地解析'
                                         }.get(kind, ''))

    s = normalize(text)
    is_rotation = any(w in s for w in _ROTATE_MARK)

    # ---------- 方向 ----------
    if is_rotation:
        left = any(w in s for w in _LEFT_WORDS)
        right = any(w in s for w in _RIGHT_WORDS)
        if left == right:      # 都没说，或同时命中 —— 不猜
            return ParsedCommand(UNPARSED, note='是旋转指令，但没能确定方向（左/右）')
        sign = 1.0 if left else -1.0
        skill, units, what = 'control.rotate', _ANGLE_UNITS, '转角'
    else:
        fwd = any(w in s for w in _FORWARD_WORDS)
        back = any(w in s for w in _BACK_WORDS)
        left = any(w in s for w in _STRAFE_LEFT_WORDS)
        right = any(w in s for w in _STRAFE_RIGHT_WORDS)
        # ⚠️ 「向左」既是平移词也是旋转词，但旋转在上面已经按「转」字分流走了，
        #    所以到这里 left 只可能是**平移**的向左。
        picked = [x for x in (fwd, back, left, right) if x]
        if len(picked) != 1:
            return ParsedCommand(UNPARSED, note='是平移指令，但方向不唯一或没识别到')
        if fwd:
            vec, skill = (1.0, 0.0), 'control.move_relative'
        elif back:
            vec, skill = (-1.0, 0.0), 'control.move_relative'
        elif left:
            vec, skill = (0.0, 1.0), 'control.move_relative'
        else:
            vec, skill = (0.0, -1.0), 'control.move_relative'
        units, what = _LINEAR_UNITS, '位移'

    # ---------- 数值 ----------
    value, n_start, n_end = find_number(s)
    if value is None:
        return ParsedCommand(UNPARSED, note=f'是{what}指令，但没找到数值')

    # ---------- 单位 ----------
    factor, literal, u_start = find_unit(s, units)
    if factor is None:
        if is_rotation:
            hint = '度（如「右转30度」）或弧度（如「右转0.52弧度」）'
        else:
            hint = '米（如「向前0.5米」）或厘米（如「向前50厘米」）'
        return ParsedCommand(
            UNPARSED,
            note=f'{what}缺少单位，拒绝猜 —— 请说明是{hint}。'
                 f'⚠️ 同一个数字按不同单位可以差 57 倍，猜错是一次无声的错误运动')

    # 单位必须**紧跟在数值后面**（中间只允许空格）。
    # 否则「前进1米，后退2厘米」这种两句连写会张冠李戴。
    between = s[n_end:u_start]
    if between.strip():
        return ParsedCommand(UNPARSED,
                             note=f'数值与单位之间夹了别的内容（{between!r}），拒绝解析')

    magnitude = abs(value) * factor
    if magnitude <= 0:
        return ParsedCommand(UNPARSED, note='数值为 0 —— 没有要执行的动作')

    if is_rotation:
        angle = sign * magnitude
        return ParsedCommand(DETERMINISTIC, skill,
                             {'angle': round(angle, 6)},
                             f'解析为旋转 {angle:+.4f} rad（{magnitude / 0.017453292519943295:.1f}°）')
    x = vec[0] * magnitude
    y = vec[1] * magnitude
    return ParsedCommand(DETERMINISTIC, skill,
                         {'x': round(x, 6), 'y': round(y, 6)},
                         f'解析为平移 ({x:+.3f}, {y:+.3f}) m')
