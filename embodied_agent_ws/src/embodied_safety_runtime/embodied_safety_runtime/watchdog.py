#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**停更看门狗**（纯 Python，不依赖 ROS）。

用途：监视某个**应该周期性出现的信号**（这里的第一个用途是 Motor Driver 的状态话题），
一旦它停更就说明那个进程没了 —— 而"它没了"恰恰意味着**没人再发零了**：
底盘没有指令超时保护（D-020），"持续发零"目前只在 Motor Driver 里（D-025）。
所以这是 Safety Runtime 必须自己盯着的一件事。

⚠️ **首次见到之前永不判失联。** 否则每次启动、或把监视目标指向一个新建的话题时，
都会先报一次假警报 —— 假警报会把真警报淹掉。
这条与 D-025 链路状态机里"startup 不算 lost"是同一条原则。
"""


class StalenessWatchdog:
    def __init__(self, timeout):
        if timeout <= 0:
            raise ValueError(f'timeout 必须为正，得到 {timeout}')
        self.timeout = float(timeout)
        self._last = None

    @property
    def ever_seen(self):
        return self._last is not None

    def on_signal(self, now):
        """收到一次信号。"""
        self._last = now

    def age(self, now):
        """距上一次信号的秒数；从未见过则为 None。"""
        return None if self._last is None else (now - self._last)

    def expired(self, now):
        """是否已停更到该报警的程度。

        ⚠️ **从未见过时返回 False**（不是 True）—— 见模块说明。
        """
        if self._last is None:
            return False
        return (now - self._last) > self.timeout

    def reset(self):
        """回到"从未见过"的状态（例如监视目标被换掉时）。"""
        self._last = None
