#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`llm_client` 的离线单测 —— **一次都不碰网络**。

要钉住的是**失败分类**：这一跳是"根本没配好"还是"这次调用失败了"，
对应**人要做的事完全不同**（去改配置 vs 去看网络/对端）。
把两者混成一句"调用失败"，会把排查方向指错。
"""

import json

import pytest

from embodied_agent_runtime import llm_client
from embodied_agent_runtime.llm_client import (
    LlmTransportError, LlmUnavailable, OpenAiCompatClient, read_api_key)


def client(**kw):
    kw.setdefault('base_url', 'https://example.invalid/v1')
    kw.setdefault('model', 'some-model')
    kw.setdefault('api_key', 'k')
    return OpenAiCompatClient(**kw)


def envelope(content):
    return {'choices': [{'message': {'role': 'assistant', 'content': content}}]}


# ---------- 密钥：只从环境变量读 ----------

def test_read_api_key_from_env(monkeypatch):
    monkeypatch.setenv('JETSONROBOT_TEST_KEY', '  secret  ')
    assert read_api_key('JETSONROBOT_TEST_KEY') == 'secret'


def test_read_api_key_absent_is_none(monkeypatch):
    monkeypatch.delenv('JETSONROBOT_TEST_KEY', raising=False)
    assert read_api_key('JETSONROBOT_TEST_KEY') is None
    assert read_api_key('') is None


# ---------- "根本没配好" → LlmUnavailable ----------

def test_missing_key_is_unavailable():
    c = OpenAiCompatClient('https://x.invalid/v1', 'm', None,
                           transport=lambda *a: pytest.fail('不该发出请求'))
    with pytest.raises(LlmUnavailable):
        c.complete([{'role': 'user', 'content': 'hi'}])


def test_missing_endpoint_is_unavailable():
    c = OpenAiCompatClient('', 'm', 'k', transport=lambda *a: pytest.fail('不该发出请求'))
    with pytest.raises(LlmUnavailable):
        c.complete([{'role': 'user', 'content': 'hi'}])


def test_missing_model_is_unavailable():
    c = OpenAiCompatClient('https://x.invalid/v1', '', 'k',
                           transport=lambda *a: pytest.fail('不该发出请求'))
    with pytest.raises(LlmUnavailable):
        c.complete([{'role': 'user', 'content': 'hi'}])


# ---------- 正常一轮 ----------

def test_returns_assistant_content_and_sends_expected_payload():
    seen = {}

    def transport(url, headers, payload, timeout_s):
        seen.update(url=url, headers=headers, payload=payload, timeout=timeout_s)
        return envelope('{"skill": "x"}')

    out = client(timeout_s=3.5, max_tokens=123, temperature=0.0,
                 transport=transport).complete([{'role': 'user', 'content': 'hi'}])
    assert out == '{"skill": "x"}'
    assert seen['url'] == 'https://example.invalid/v1/chat/completions'
    assert seen['headers']['Authorization'] == 'Bearer k'
    assert seen['payload']['model'] == 'some-model'
    assert seen['payload']['temperature'] == 0.0      # ★ 规划要确定性
    assert seen['payload']['max_tokens'] == 123
    assert seen['payload']['stream'] is False
    assert seen['timeout'] == 3.5


def test_base_url_trailing_slash_is_normalised():
    c = client(base_url='https://example.invalid/v1/')
    assert c.url == 'https://example.invalid/v1/chat/completions'


# ---------- "这次失败了" → LlmTransportError ----------

def test_endpoint_error_field_is_transport_error():
    c = client(transport=lambda *a: {'error': {'message': 'rate limited'}})
    with pytest.raises(LlmTransportError) as e:
        c.complete([{'role': 'user', 'content': 'hi'}])
    assert 'rate limited' in str(e.value)


def test_unexpected_envelope_is_transport_error():
    c = client(transport=lambda *a: {'nope': 1})
    with pytest.raises(LlmTransportError):
        c.complete([{'role': 'user', 'content': 'hi'}])


def test_empty_content_is_transport_error():
    c = client(transport=lambda *a: envelope('   '))
    with pytest.raises(LlmTransportError):
        c.complete([{'role': 'user', 'content': 'hi'}])


def test_non_dict_reply_is_transport_error():
    c = client(transport=lambda *a: ['not', 'a', 'dict'])
    with pytest.raises(LlmTransportError):
        c.complete([{'role': 'user', 'content': 'hi'}])


def test_transport_exception_propagates_as_transport_error():
    def boom(*_a):
        raise LlmTransportError('超时（8s）')

    with pytest.raises(LlmTransportError):
        client(transport=boom).complete([{'role': 'user', 'content': 'hi'}])


# ---------- 默认传输：不发请求，只验它把失败**归类** ----------

def test_default_transport_is_the_urllib_one():
    c = client()
    assert c._transport is llm_client._urllib_transport     # noqa: SLF001


def test_default_transport_wraps_connection_errors(monkeypatch):
    """真打一个**必然失败**的地址（保留域名，不出网），确认异常被归类成传输错误。"""
    import urllib.error

    def raise_url_error(*_a, **_kw):
        raise urllib.error.URLError('nodename nor servname provided')

    monkeypatch.setattr(llm_client.urllib.request, 'urlopen', raise_url_error)
    with pytest.raises(LlmTransportError) as e:
        client(base_url='https://nonexistent.invalid/v1', timeout_s=0.5).complete(
            [{'role': 'user', 'content': 'hi'}])
    assert '连不上' in str(e.value)


def test_default_transport_reports_http_status(monkeypatch):
    import io
    import urllib.error

    def raise_http_error(req, timeout=None):
        raise urllib.error.HTTPError(
            req.full_url, 500, 'Server Error', {}, io.BytesIO(b'{"error":"boom"}'))

    monkeypatch.setattr(llm_client.urllib.request, 'urlopen', raise_http_error)
    with pytest.raises(LlmTransportError) as e:
        client(timeout_s=0.5).complete([{'role': 'user', 'content': 'hi'}])
    assert 'HTTP 500' in str(e.value) and 'boom' in str(e.value)


def test_default_transport_rejects_non_json_body(monkeypatch):
    class FakeResp:
        def __enter__(self):
            return self

        def __exit__(self, *_a):
            return False

        def read(self):
            return b'<html>not json</html>'

    monkeypatch.setattr(llm_client.urllib.request, 'urlopen',
                        lambda *a, **kw: FakeResp())
    with pytest.raises(LlmTransportError) as e:
        client(timeout_s=0.5).complete([{'role': 'user', 'content': 'hi'}])
    assert '不是合法 JSON' in str(e.value)


def test_payload_is_json_serialisable():
    """默认传输会 `json.dumps(payload)` —— 拼出来的东西必须真的能序列化。"""
    seen = {}

    def transport(url, headers, payload, timeout_s):
        json.dumps(payload)
        seen['ok'] = True
        return envelope('{}')

    client(transport=transport).complete([{'role': 'user', 'content': '中文任务'}])
    assert seen.get('ok') is True
