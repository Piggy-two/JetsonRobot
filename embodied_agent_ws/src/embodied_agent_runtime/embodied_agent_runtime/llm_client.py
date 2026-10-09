#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云端 LLM 的**最小客户端**（纯 Python，不依赖 ROS）。

为什么是手写的、只有几十行
----------------------------
① **零新依赖**：只用标准库 `urllib`。机器人上多一个 pip 包就多一份要跟着
   JetPack / 内核版本走的东西，而这个调用点简单到不值得。
② **传输可注入**：`transport` 是个普通可调用对象。单测里塞一个假的，
   于是**整条规划链可以完全不碰网络、也不需要密钥**地跑完 —— 这是本模块
   能被离线证伪的前提。

密钥**只从环境变量读**（变量名可配），**绝不进仓库、绝不进日志**
（CLAUDE.md §6）。本模块不打印、不回显它。

⚠️ 本模块**不做任何语义判断** —— 它不认识技能、不校验计划。它只负责
"把消息发出去、把回话取回来，并把失败**分类**"。
"回话里那个技能能不能调"是 `planner.accept_skill()` 的事。
"""

import json
import os
import urllib.error
import urllib.request


class LlmUnavailable(Exception):
    """**这一跳根本没法开始**：没配置、没开、或找不到密钥。

    ⚠️ 与 `LlmTransportError` 分开，是因为**人要做的事不一样**：
    这个要去看配置/环境变量；那个要去看网络与对端。
    """


class LlmTransportError(Exception):
    """配置是好的，但这次调用失败了：网络、超时、HTTP 错误码、回包不成形。"""


class LlmTruncated(LlmTransportError):
    """**回包被 `max_tokens` 截断了** —— 对端答了，是我们给的空间不够。

    ⚠️ 为什么单独立一类（它是 `LlmTransportError` 的子类，老的 except 不会漏接）：
    它和"调用失败"**要做的事完全相反** ——
      · 调用失败 → 去看网络、对端、超时（`timeout_s`）；
      · 被截断   → 去调大 `max_tokens`（网络是好的，对端也是好的）。
    混成一句"LLM 调用失败"，会把排查方向整个指错，而且**它看起来像偶发故障**：
    同一个提示词这次成、下次不成（实测三次里坏一次），于是最容易被读成
    "网络不稳" —— 而真正的原因是**推理模型的思维链也计入 `max_tokens`**，
    消耗量抖动极大（同一提示词实测 215 ~ 2708 tokens）。
    """


def read_api_key(env_name):
    """从环境变量读密钥。**读不到就返回 None**，不抛 —— 让调用方决定怎么表述。"""
    if not env_name:
        return None
    val = os.environ.get(str(env_name))
    return val.strip() if val and val.strip() else None


def _urllib_transport(url, headers, payload, timeout_s):
    """默认传输：POST 一段 JSON，取回一段 JSON。**失败一律抛 `LlmTransportError`。**

    ⚠️ 异常分类要小心：`HTTPError` 是 `URLError` 的子类，`URLError` 又是 `OSError`
    的子类，而 socket 超时在 Python 3.10 就是内建的 `TimeoutError`（也是 `OSError`）。
    所以**先接 HTTPError（要拿状态码），再统一接 OSError**，别把顺序写反。
    """
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers=headers, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            body = resp.read()
    except urllib.error.HTTPError as exc:
        # 把回包正文带一点出来 —— 端点报错时那句话通常在正文里，比状态码有用得多
        try:
            detail = exc.read().decode('utf-8', 'replace')[:300]
        except Exception:                                        # noqa: BLE001
            detail = ''
        raise LlmTransportError(f'HTTP {exc.code}{"：" + detail if detail else ""}') from exc
    except TimeoutError as exc:
        raise LlmTransportError(f'超时（{timeout_s:g}s）') from exc
    except OSError as exc:
        raise LlmTransportError(f'连不上：{exc}') from exc

    try:
        return json.loads(body.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LlmTransportError(f'回包不是合法 JSON：{exc}') from exc


class OpenAiCompatClient:
    """OpenAI 兼容的 `/chat/completions` 客户端（DeepSeek 等就是这个形状）。

    :param transport: `(url, headers, payload, timeout_s) -> dict`。
                      默认走 `urllib`；单测传假的。
    """

    def __init__(self, base_url, model, api_key,
                 timeout_s=8.0, max_tokens=400, temperature=0.0, transport=None):
        self.base_url = str(base_url or '').rstrip('/')
        self.model = str(model or '')
        self._api_key = api_key
        self.timeout_s = float(timeout_s)
        self.max_tokens = int(max_tokens)
        self.temperature = float(temperature)
        self._transport = transport or _urllib_transport

    @property
    def url(self):
        return f'{self.base_url}/chat/completions'

    def complete(self, messages):
        """发一轮对话，返回助手那条消息的**纯文本**内容。

        :raises LlmUnavailable: 没配置端点/模型，或没有密钥
        :raises LlmTransportError: 本次调用失败（超时/网络/HTTP/回包不成形）
        """
        if not self.base_url or not self.model:
            raise LlmUnavailable('没有配置 LLM 端点或模型')
        if not self._api_key:
            raise LlmUnavailable('没有读到 API key（检查那个环境变量）')

        payload = {
            'model': self.model,
            'messages': list(messages),
            'temperature': self.temperature,
            'max_tokens': self.max_tokens,
            'stream': False,
        }
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self._api_key}',
        }
        reply = self._transport(self.url, headers, payload, self.timeout_s)

        if not isinstance(reply, dict):
            raise LlmTransportError(f'回包顶层不是对象：{type(reply).__name__}')
        if reply.get('error'):
            raise LlmTransportError(f'端点返回错误：{reply["error"]}')
        try:
            choice = reply['choices'][0]
            content = choice['message']['content']
        except (KeyError, IndexError, TypeError) as exc:
            raise LlmTransportError(f'回包结构不认识（缺 choices[0].message.content）：{exc}') from exc

        # ⚠️ **先看 `finish_reason`，再看内容** —— 顺序不能反。
        #    截断的两种形态都要当成截断：
        #      ① content 为空（思维链把预算吃光了，一个字都没轮上）；
        #      ② content 有东西但是**半截 JSON**（`{"skill": "…` 就断了）。
        #    只看"内容空不空"会漏掉 ②，而 ② 更坏：半截 JSON 会被下游报成
        #    "回包无法解析"，把"预算不够"说成"模型不会说话"。
        reason = choice.get('finish_reason')
        used = (reply.get('usage') or {}).get('completion_tokens_details') or {}
        thought = used.get('reasoning_tokens')
        extra = f'（本次推理消耗 {thought} tokens）' if thought else ''
        if reason == 'length':
            raise LlmTruncated(
                f'回包被截断：max_tokens={self.max_tokens} 用完了{extra}。'
                f'推理模型的思维链**也计入**这笔预算，用量抖动很大 —— '
                f'调大 max_tokens，或换一个不做思维链的模型')

        if not isinstance(content, str) or not content.strip():
            # 到这里 finish_reason 不是 length：对端确实没给内容，但**不是被我们截的**
            raise LlmTransportError(
                f'回包里的内容是空的（finish_reason={reason!r}）')
        return content
