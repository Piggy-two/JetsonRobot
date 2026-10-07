#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一个**假的 OpenAI 兼容端点**，专门用来验 Planner 的 LLM 那一跳。

    ⚠️ 这是【验收工装】，不是运行时组件。永远不要把它指给真机器人用。

为什么需要它
------------
`agent_runtime` 的 LLM 规划（D-038）有**一大半逻辑是"模型不听话时怎么办"**：
它选了控制层技能、它编了个不存在的技能、它回的话根本不是 JSON、
它干脆不回。这些情况**无法靠真模型稳定复现**，而它们恰恰是最该被验的。

这个假端点把"模型会说什么"变成一张**固定表**，于是：

    ✅ 整条链可以在**不联网、不需要密钥**、不需要真模型的情况下跑完
    ✅ 每种"模型不听话"都能被**指名道姓地**复现
    ✅ 它睡多久由 `--delay` 说了算 —— 超时路径也能测

它回话的挑选方式：把请求里**最后一条 user 消息**拿来做子串匹配，
命中表里哪条就回哪条；都不命中就回 `--default`。

用法
----
    python3 tools/fake_llm_endpoint.py --port 8765            # 起假端点
    python3 tools/fake_llm_endpoint.py --delay 30            # 故意不回（测超时）
    python3 tools/fake_llm_endpoint.py --reply-file my.json  # 用自己的回话表

然后再起 Agent Runtime 指向它（**密钥随便填一个**，本端点不校验）：

    export FAKE_LLM_KEY=whatever
    ros2 launch embodied_agent_runtime agent_runtime.launch.py \\
        llm_enabled:=true \\
        llm_base_url:=http://127.0.0.1:8765/v1 \\
        llm_model:=fake-model \\
        llm_api_key_env:=FAKE_LLM_KEY \\
        llm_timeout:=3.0
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

#: 默认回话表：**键是用户消息里的子串**，值是模型"回"的内容。
#: 每一条都对应一个**必须被正确处理**的情形。
DEFAULT_TABLE = {
    # —— 正常：选一个合法的 task-tier 技能
    '往前走一小段': '{"skill": "autonomous.advance_until_blocked", '
                    '"args": {"max_distance": 0.2, "clear_range": 0.25, "step": 0.1}}',
    # —— 越权：模型去碰控制层（**架构红线必须当场拦住**）
    '偷偷动一下': '{"skill": "control.move_relative", "args": {"x": 0.3, "y": 0.0}}',
    # —— 编造：技能根本不存在
    '编个技能': '{"skill": "semantic.search_object", "args": {}}',
    # —— 参数越界
    '走很远': '{"skill": "autonomous.advance_until_blocked", '
              '"args": {"max_distance": 99.0, "clear_range": 0.25, "step": 0.1}}',
    # —— 参数名写错（模型很爱加个自认为合理的字段）
    '快点走': '{"skill": "autonomous.advance_until_blocked", '
              '"args": {"max_distance": 0.2, "clear_range": 0.25, "step": 0.1, "speed": 1.0}}',
    # —— 不回 JSON，用大白话
    '随便说说': '我觉得可以先往前走一点点，再看看情况。',
    # —— 套了 Markdown 围栏（**解析要宽**，这种必须能读出来）
    '带围栏': '```json\n{"skill": "autonomous.advance_until_blocked", '
              '"args": {"max_distance": 0.2, "clear_range": 0.25, "step": 0.1}}\n```',
    # —— 模型主动承认做不到（应当**原样转述**它的理由）
    '找杯子': '{"refuse": "机器人现在没有识别物体的能力（Phase 4 未开始）"}',
}

DEFAULT_REPLY = '{"refuse": "这个任务我看不懂，拒绝执行"}'


class Handler(BaseHTTPRequestHandler):
    table = DEFAULT_TABLE
    default = DEFAULT_REPLY
    delay = 0.0
    model_expected = None

    def log_message(self, fmt, *args):      # noqa: A002
        pass                                 # 用自己的打印，不用默认的

    def _reply_for(self, text):
        for key, reply in self.table.items():
            if key in text:
                return key, reply
        return '(默认)', self.default

    def do_POST(self):                       # noqa: N802
        length = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(length) if length else b'{}'
        try:
            body = json.loads(raw.decode('utf-8'))
        except Exception as exc:             # noqa: BLE001
            self._send(400, {'error': {'message': f'请求不是合法 JSON：{exc}'}})
            return

        messages = body.get('messages') or []
        text = ''
        for m in reversed(messages):
            if m.get('role') == 'user':
                text = str(m.get('content', ''))
                break
        hit, reply = self._reply_for(text)

        # ⚠️ **只打印到达的事实，绝不回显 Authorization** —— 那条里是密钥
        print(f'[假端点] 收到 {body.get("model")!r}｜user={text!r} → 回「{hit}」',
              flush=True)
        print(f'[假端点]   带 Authorization 头：{bool(self.headers.get("Authorization"))}'
              f'｜messages {len(messages)} 条', flush=True)

        if self.delay:
            print(f'[假端点]   故意睡 {self.delay:g}s（测超时路径）', flush=True)
            time.sleep(self.delay)

        self._send(200, {'choices': [{'index': 0, 'finish_reason': 'stop',
                                      'message': {'role': 'assistant',
                                                  'content': reply}}]})

    def _send(self, code, obj):
        payload = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main():
    ap = argparse.ArgumentParser(description='假的 OpenAI 兼容端点（验收用）')
    ap.add_argument('--host', default='127.0.0.1',
                    help='⚠️ 默认只听本机回环。别绑 0.0.0.0 —— 它不校验任何凭据')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--delay', type=float, default=0.0,
                    help='收到请求后先睡这么久再回 —— 用来测客户端的超时路径')
    ap.add_argument('--default', default=DEFAULT_REPLY,
                    help='没有命中任何一条时的回话')
    ap.add_argument('--reply-file', default='',
                    help='自定义回话表（JSON：{子串: 回话}），合并进默认表')
    args = ap.parse_args()

    if args.reply_file:
        with open(args.reply_file, encoding='utf-8') as fh:
            Handler.table = dict(DEFAULT_TABLE, **json.load(fh))
    Handler.default = args.default
    Handler.delay = args.delay

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'[假端点] 监听 http://{args.host}:{args.port}/v1/chat/completions '
          f'（延迟 {args.delay:g}s，{len(Handler.table)} 条回话）', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        print('\n[假端点] 已停止', flush=True)


if __name__ == '__main__':
    main()
