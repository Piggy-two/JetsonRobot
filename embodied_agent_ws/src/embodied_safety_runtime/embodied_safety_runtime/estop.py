#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地安全指令的**判定逻辑（纯 Python，不依赖 ROS）**。

D-006 要求：「停 / 急停 / 取消任务」这类安全指令走**本地解析、绕过 LLM**。
D-024 决策 3 进一步定下：执行侧必须是**本项目自己**的 Safety 入口。

本模块只管两件事，都可以离线证伪：
  1. 一句话**是不是**安全指令（`is_safety_command`）
  2. 急停**锁存**的状态机（`EStopLatch`）
"""

# 默认的安全词表。**都是"整句停下"语义**，不含"停止追踪"这类只停某个功能的短语。
DEFAULT_SAFETY_PHRASES = [
    # 中文（厂商 BNF 词表里是 `停下`；其余是常见同义说法）
    '停', '停下', '停下来', '停止', '急停', '紧急停止', '别动', '不要动', '别走', '取消任务',
    # 英文（厂商 BNF 里是 `stop`）
    'stop', 'halt', 'freeze', "don't move", 'dont move', 'cancel task', 'emergency stop',
]

# 归一化时丢掉的标点
_PUNCT = ' \t\r\n，。！？、；：""\'\'（）()[]{}.,!?;:'


def normalize(text):
    """归一化：去空格与标点、英文转小写。

    为什么要**去掉空格**：中文 ASR 有时会插入空格，而英文短语本身含空格，
    两种语言要能用同一套比较方式。
    """
    if text is None:
        return ''
    s = str(text).strip().lower()
    for ch in _PUNCT:
        s = s.replace(ch, '')
    return s


def is_safety_command(text, phrases=None):
    """判断一句 ASR 文本是不是安全指令。

    :return: (是否安全指令, 命中的词)

    ⚠️ **用整句匹配，不用子串匹配** —— 这是一个刻意的取舍，两种错法都想过：

    - 子串匹配（"只要包含"）：**安全词表里加一个 `停`，就会让厂商词表里的
      `停止追踪` / `停止分拣` 全部误触发急停**。用户说"停止追踪"机器人整机停住，
      再说一次还是停住 —— 这会迅速把安全通路变成一个不可信的按钮，
      而不可信的安全通路比没有更危险。
    - 整句匹配（本实现）：**可能漏掉没想到的说法**。这是真实代价，用两条缓解：
      ① 词表可配置（`phrases` 参数），并能加入现场常用说法；
      ② 厂商的 BNF 语法本来就把可识别词限制在一个固定集合里，
         ASR **不会**产出任意句子，所以"没想到的说法"实际很有限。
    """
    if phrases is None:
        phrases = DEFAULT_SAFETY_PHRASES
    s = normalize(text)
    if not s:
        return False, None
    for p in phrases:
        if s == normalize(p):
            return True, p
    return False, None


class EStopLatch:
    """急停锁存。

    "锁存"是重点：**触发之后必须显式解除**，不能因为下一句话/下一条指令就自动恢复。
    理由和 D-025 的停车锁存一样 —— 解除急停这个动作本身必须是一次明确的、
    有人（或上层）负责的决定。
    """

    def __init__(self):
        self._latched = False
        self._reason = ''

    @property
    def latched(self):
        return self._latched

    @property
    def reason(self):
        return self._reason

    def trigger(self, reason):
        """触发急停。**返回是否发生了状态变化**（便于只对"进入"记一次日志）。"""
        changed = not self._latched
        self._latched = True
        self._reason = str(reason)
        return changed

    def release(self, reason=''):
        """显式解除。返回是否发生了状态变化。"""
        changed = self._latched
        self._latched = False
        self._reason = ''
        return changed
