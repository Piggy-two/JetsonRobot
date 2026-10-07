#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LLM 规划（D-038）的端到端验收 —— 对着**假端点**跑，不联网、不需要密钥。

    ⚠️ 这是【验收工装】，不是运行时组件。

它验的是"**模型不听话时怎么办**"，不是"模型聪不聪明"
------------------------------------------------------
真模型无法稳定复现"它选了控制层技能""它编了个不存在的技能""它回的话不是 JSON"
—— 而这几件事恰恰是**最危险、最该被验**的。所以假端点把模型的话变成一张固定表
（`tools/fake_llm_endpoint.py`），每种情形都能**指名道姓地**复现。

三段，各自一套启动参数：

    --phase plan   假端点正常回话：八种"模型可能说的话"，每种都该有**说清原因**的结局
    --phase dead   假端点**没起来**：必须报"连不上/调用失败"，**不是**"听不懂"
    --phase slow   假端点**故意不回**：必须超时拒绝，且**节点仍然活着**（status 照发）
                   ⚠️ 这一段要让假端点 `--delay` 大于 Agent 的 `llm_timeout`
                   （例如端点 --delay 30、节点 llm_timeout:=3.0）

前置（Motor Driver 用默认 `dry_run=true` —— **车不会动**）：
    python3 tools/fake_llm_endpoint.py --port 8765 &
    ros2 launch embodied_motor_driver       motor_driver.launch.py
    ros2 launch embodied_lidar_driver       lidar_driver.launch.py
    ros2 launch embodied_control_skills     control_skills.launch.py
    ros2 launch embodied_autonomous_skills  autonomous_skills.launch.py
    ros2 launch embodied_skill_gateway      skill_gateway.launch.py allow_motion:=true
    ros2 launch embodied_safety_runtime     safety_runtime.launch.py enable_obstacle_guard:=false
    export FAKE_LLM_KEY=whatever
    ros2 launch embodied_agent_runtime agent_runtime.launch.py \\
        allow_motion:=true llm_enabled:=true \\
        llm_base_url:=http://127.0.0.1:8765/v1 llm_model:=fake-model \\
        llm_api_key_env:=FAKE_LLM_KEY llm_timeout:=3.0

    ⚠️ **rules_file 留空**（默认）：所有任务都会走到 LLM 那一跳，这正是要验的路。

用法：
    python3 tools/llm_planner_acceptance.py --phase plan
    python3 tools/llm_planner_acceptance.py --phase dead
    python3 tools/llm_planner_acceptance.py --phase slow
退出码：0 = 全部断言通过；1 = 有失败。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from embodied_skills_interfaces.srv import AgentTask

SUBMIT = '/agent_runtime/submit'
STATUS = '/embodied/agent/status'
GATEWAY_CANCEL = '/skill_gateway/cancel'

#: (任务文本, 期望, 理由里必须出现的片段)
#:   期望 'refuse' = 必须被拒绝；'accept' = 必须被受理
CASES = [
    ('往前走一小段', 'accept', ''),                    # 合法选择
    ('带围栏', 'accept', ''),                          # 套了 ``` 围栏也要能读出来
    ('偷偷动一下', 'refuse', '架构红线'),               # ★ 模型去碰控制层
    ('编个技能', 'refuse', '未注册'),                   # 模型编了个不存在的技能
    ('走很远', 'refuse', '上限'),                       # 参数越界
    ('快点走', 'refuse', '未定义的参数'),               # 参数名写错，不许静默忽略
    ('随便说说', 'refuse', '无法解析'),                 # 回的不是 JSON
    ('找杯子', 'refuse', 'Phase 4'),                    # 模型自己说做不到 → 原样转述
    ('这句话表里没有', 'refuse', '看不懂'),             # 走了假端点的默认回话
]


class SubmitClient(Node):
    def __init__(self):
        super().__init__('llm_planner_acceptance')
        self.cli = self.create_client(AgentTask, SUBMIT)
        self.cancel_cli = self.create_client(Trigger, GATEWAY_CANCEL)
        self.status = None
        self.status_frames = 0
        self.create_subscription(Float64MultiArray, STATUS, self._on_status, 10)

    def _on_status(self, m):
        self.status = list(m.data)
        self.status_frames += 1

    def submit(self, text, principal='operator.manual', timeout=20.0):
        req = AgentTask.Request()
        req.text = text
        req.principal = principal
        fut = self.cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=['plan', 'dead', 'slow'], required=True)
    ap.add_argument('--settle', type=float, default=2.5,
                    help='受理之后等技能跑完的时间（秒）')
    args = ap.parse_args()

    rclpy.init()
    node = SubmitClient()
    results = []

    def rep(name, ok, detail):
        results.append((name, ok, detail))
        print(f'  {"✅" if ok else "❌"} {name}：{detail}')

    try:
        print()
        print('=' * 74)
        print(f'  LLM 规划端到端验收（D-038）—— phase={args.phase}')
        print('=' * 74)
        print()

        if not node.cli.wait_for_service(timeout_sec=10.0):
            print(f'  ⛔ {SUBMIT} 不可用 —— Agent Runtime 在跑吗？')
            return 3
        node.spin_for(1.0)

        if args.phase == 'plan':
            for text, expect, fragment in CASES:
                res = node.submit(text)
                if res is None:
                    rep(text, False, '提交超时（没有回话）')
                    continue
                got = 'accept' if res.accepted else 'refuse'
                ok = got == expect and (not fragment or fragment in res.message)
                detail = f'{got}｜{res.message[:90]}'
                rep(text, ok, detail)
                if res.accepted:
                    print(f'      task_id={res.task_id}')
                    node.spin_for(args.settle)
                    node.cancel_cli.call_async(Trigger.Request())
                    node.spin_for(0.5)

        elif args.phase == 'dead':
            res = node.submit('找杯子')
            assert res is not None
            rep('假端点没起来时**明确说调用失败**', not res.accepted, res.message[:110])
            rep('★ 且理由里**不是**"听不懂"（不静默降级）',
                '调用失败' in res.message or '不可用' in res.message, res.message[:110])
            rep('理由里给出上限，便于判断该改哪个参数',
                '上限' in res.message or '超时' in res.message, res.message[:110])

        else:  # slow
            print('（假端点会故意不回；下面确认这条路上的**其余部分没被卡死**）')
            # ⚠️ 只发**一次**提交，然后**在它还没返回的时候**看状态话题 ——
            #    发两次会撞上单飞闸门，那测的就成了另一件事。
            before = node.status_frames
            fut = node.cli.call_async(_req('找杯子'))
            node.spin_for(1.5)                      # llm_timeout 内，提交应仍在飞
            mid = node.status_frames
            end = time.monotonic() + 15.0
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            res = fut.result() if fut.done() else None

            rep('超时被当成**拒绝**（不是含糊通过）',
                res is not None and not res.accepted,
                (res.message[:110] if res else '提交本身超时了'))
            rep('★ 阻塞期间 /embodied/agent/status 仍在发布（节点没被卡死）',
                mid > before, f'帧数 {before} → {mid}（1.5 s 内）')
            rep('★ 超时返回之后仍在发布', node.status_frames > mid,
                f'总帧数 {node.status_frames}')
        print()
        print('=' * 74)
        failed = [n for n, ok, _ in results if not ok]
        if failed:
            print(f'  ❌ {len(failed)}/{len(results)} 项未通过：')
            for n in failed:
                print(f'     - {n}')
        else:
            print(f'  ✅ 全部 {len(results)} 项通过')
        print('=' * 74)
        print()
        return 0 if not failed else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _req(text):
    r = AgentTask.Request()
    r.text = text
    r.principal = 'operator.manual'
    return r


if __name__ == '__main__':
    sys.exit(main())
