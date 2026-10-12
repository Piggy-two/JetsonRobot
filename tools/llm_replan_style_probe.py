#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**真模型在"上一条计划失败了"之后，到底会换出什么样的走法？**

它为什么存在
------------
D-044 给重规划上了一道**结构性**闸：新计划必须与**试过的任何一个**都不一样
（`replan.repeats_earlier` → `planner.same_plan`），比的是**技能 + 全部参数**。
故意留的余地是 `turn 左` vs `turn 右` —— 那**是**换了走法。

D-045 接上真模型之后，观察到一个它挡不住的模式：
**真模型常把"同一个技能、参数改小"当成换了走法**（实测 `advance(0.3/0.5/0.1)`
→ `advance(0.15/0.2/0.05)`）。按定义那**不算重复**，于是**由预算兜底**
（`max_replans` 默认 2）—— 也就是"试两次都一样，然后收工"。

于是有一个**产品取舍**摆在面前：要不要把判据收紧。三个候选：

| | 判据 | 挡住"参数改小" | 保住"换另一边绕" |
|---|---|---|---|
| **A** | 技能 + **全部参数**（现状） | ❌ 挡不住 | ✅ |
| **B** | 只看**技能序列** | ✅ | ❌ **误伤** |
| **C** | 技能序列 + **参数的符号** | ✅ | ✅ |

⚠️ 但 C 也不是免费的：它同样会拒掉"**同方向、幅度改小**"的重试
（`advance` 被挡在 0.30 时改成 `advance(0.15)` 反而可能走通）。
⇒ **三条各有各的误判面 —— 真正的判据是频率。** 本工装就是去量那个频率的。

它怎么保证量的是**同一件事**
----------------------------
它**不自己拼提示词**，而是直接调用运行时重规划走的那一跳
`llm_planner.plan_with_llm(text, registry, client, tried, states)`
—— 同一段 `RETRY_PROMPT`、同一份技能菜单、同一个校验口。自己拼一份出来，
量到的就是另一个问题了。

两种跑法
--------
    --phase ask    固定一段失败历史，问 N 次。看**分布**：同一条历史，
                   模型会给出哪几种走法？（这是纯粹的"提议风格"）
    --phase chain  模拟一整条链：把模型自己给出的计划**当作"又失败了"**接回历史，
                   跑到预算上限。看**链**会不会撞上"重复被拒"或把预算烧光。

    ⚠️ chain 里"又失败了"是**模拟**的（每次都记 `BLOCKED`）—— 那是**最坏情况**
       （永远不成），也正是"假换走法"最容易现形的区间。真实场景是混合结局。

用法
----
    python3 tools/llm_replan_style_probe.py \
        --base-url https://api.deepseek.com/v1 --model deepseek-flash \
        --api-key-env DEEPSEEK_API_KEY --n 12

⚠️ **它会把任务文本发往第三方端点**（这正是被验的东西）。所以密钥只从**环境变量**读、
**不回显、不落盘**；端点与模型都必须显式给出，没有默认值 —— 不给就**什么也不发**。
⚠️ 它**不碰 ROS、不发布任何消息、不动车**。诊断打印里也不会出现任何密钥。
⚠️ 真模型是**非确定性**的：本工装只报**分布与计数**，不判分。

状态（诚实标注）
----------------
    离线：**已验** —— 判定逻辑 16 项单测；"没密钥/没端点就拒测"实跑过（退出码 4）。
    真端点：**尚未跑** —— 2026-10-12 环境里没有 `DEEPSEEK_API_KEY`。
            ⇒ 本工装产出的**任何分布**目前都不存在，别引用一个没跑过的结论。
"""

import argparse
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'embodied_agent_ws' / 'src' / 'embodied_skill_gateway'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'embodied_agent_ws' / 'src' / 'embodied_agent_runtime'))

from embodied_agent_runtime import llm_client, llm_planner, planner, replan  # noqa: E402
from embodied_skill_gateway.registry import Registry                   # noqa: E402

DEFAULT_REGISTRY = (Path(__file__).resolve().parents[1] / 'embodied_agent_ws'
                    / 'src' / 'embodied_skill_gateway' / 'config'
                    / 'skill_registry.yaml')

#: 默认场景。与真机上用过的那句同族（"绕开障碍继续往前"，D-045 记录里它**自己**
#: 给出了三步脱困计划并收敛）。换一句话就是换一个实验，所以做成参数。
DEFAULT_TEXT = '绕开障碍继续往前'


# --------------------------------------------------------------------------
# 判定（纯函数 —— 可离线证伪的那一部分）
# --------------------------------------------------------------------------

def shape(plan):
    """把一条计划压成"技能序列 + 每个数值参数的**符号**"。

    ⚠️ 符号用 `(v>0)-(v<0)` 而不是 `math.copysign`：布尔要真的变成 0/±1，
    而 `copysign(1, 0.0)` 是 +1 —— 那会把"原地不动"和"往前"混成一类。
    """
    out = []
    for s in plan or []:
        args = getattr(s, 'args', None) or {}
        signs = tuple(sorted(
            (str(k), 0 if not isinstance(v, (int, float))
             else (v > 0) - (v < 0))
            for k, v in args.items()))
        out.append((str(getattr(s, 'skill', s)), signs))
    return tuple(out)


def skill_sequence(plan):
    return tuple(str(getattr(s, 'skill', s)) for s in plan or [])


def classify_retry(prev, new):
    """这一次重试**算哪种**。返回一个短标签，用于计数。

    ⚠️ 四个标签**互斥**，判据从"最像重复"往"最像换了走法"排 ——
    顺序不能换：一条一字不差的计划，同时也满足"同序列同符号"。

    ⚠️ 刻意**不**再把"换了条路"细分成"在原来的起手上接别的"vs"整个换"——
    第一版那么分过，而那两种在**这次取舍里代价完全一样**（都是该放行的真换走法），
    细分只会让分布表更难读（而且那条判据自己就站不住：`adv` → `turn→adv`
    会落进"整个换"）。**分类只切在决策真正要看的那条线上。**
    """
    if not new:
        return '拒绝（模型如实说做不了）'
    if planner.same_plan(new, prev):
        return '① 一字不差的重来'
    if shape(new) == shape(prev):
        return '② 假换走法（同序列、同方向，只改了幅度）'
    if skill_sequence(new) == skill_sequence(prev):
        return '③ 换方向/换条件（同技能序列，符号变了）'
    return '④ 换了走法（技能序列真的变了）'


def verdict_of(labels):
    """把"模型会怎么走"翻成本次取舍里的**代价**。"""
    n = sum(labels.values())
    if not n:
        return '（没有样本）'
    fake = labels.get('② 假换走法（同序列、同方向，只改了幅度）', 0)
    mirror = labels.get('③ 换方向/换条件（同技能序列，符号变了）', 0)
    exact = labels.get('① 一字不差的重来', 0)
    lines = [
        f'  A 现状   ：挡不住 ②（{fake} 次要靠预算兜底）、也挡不住 ③（{mirror} 次 —— 那是**该放行**的）',
        f'  B 只看序列：会拒掉 ③ 的 {mirror} 次（**误伤"换另一边绕"**），'
        f'并拒掉 ② 的 {fake} 次',
        f'  C 序列+符号：挡 ② 的 {fake} 次 ✓、放 ③ 的 {mirror} 次 ✓',
    ]
    if exact:
        lines.append(f'  另：一字不差的重来 {exact} 次 —— 三条都挡得住（防死循环的原始目标）')
    return '\n'.join(lines)


# --------------------------------------------------------------------------
# 跑
# --------------------------------------------------------------------------

def _one_try(text, reg, client, tried, states, log=print):
    """问一次（复用的就是运行时那一跳）。返回 (steps, state, raw) 或 (None, 原因, '')。"""
    try:
        result, raw = llm_planner.plan_with_llm(text, reg, client, tried, states)
    except llm_planner.ReplyError as exc:
        return None, f'回包不合约定：{exc}', getattr(exc, 'raw', '')
    except llm_client.LlmUnavailable as exc:
        return None, f'**那一跳没跑成**（{exc}）—— 这不是"模型不会"，是没接上', ''
    except llm_client.LlmTransportError as exc:
        return None, f'**传输失败**（{type(exc).__name__}：{exc}）', ''
    if not result.ok:
        return None, f'被拒：{result.reason}', raw
    return result.steps, '', raw


def phase_ask(args, reg, client):
    """同一条失败历史，问 N 次 —— 看**分布**。"""
    seed = planner.accept_plan(
        [{'skill': 'autonomous.advance_until_blocked', 'args': {'max_distance': 0.5}}],
        reg, 'probe')
    if not seed.ok:
        print(f'  ⛔ 种子计划自己就不合法（{seed.reason}）—— 探针的前提没了，拒测。')
        return Counter()
    first = seed.steps
    tried, states = [first], ['BLOCKED']

    print(f'  固定历史：{planner.render_plan(first)} —— 结果：BLOCKED')
    print(f'  问模型 {args.n} 次"换个走法"，看它换出什么：')
    print()
    labels = Counter()
    for i in range(1, args.n + 1):
        steps, err, _ = _one_try(args.text, reg, client, tried, states)
        if steps is None:
            labels[err.split('（')[0]] += 1
            print(f'    {i:>3}. ⚠️ {err}')
            continue
        tag = classify_retry(first, steps)
        labels[tag] += 1
        print(f'    {i:>3}. {tag}\n         {planner.render_plan(steps)}')
    return labels


def phase_chain(args, reg, client):
    """模拟一整条链：把模型自己给的计划当作"又失败了"接回历史，直到预算用完。"""
    print(f'  模拟 {args.chains} 条链，每条最多 {args.max_replans + 1} 步'
          f'（每次失败都记 BLOCKED —— **最坏情况**）')
    print()
    labels = Counter()
    for c in range(1, args.chains + 1):
        tried, states = [], []
        print(f'  ── 第 {c} 条链 ──')
        for step in range(args.max_replans + 1):
            steps, err, _ = _one_try(args.text, reg, client, tried, states)
            if steps is None:
                labels[err.split('（')[0]] += 1
                print(f'     {step + 1}. ⚠️ {err}')
                break
            if tried:
                tag = classify_retry(tried[-1], steps)
                labels[tag] += 1
                print(f'     {step + 1}. {tag}')
            else:
                print(f'     {step + 1}. （首次规划）')
            print(f'         {planner.render_plan(steps)}')
            # 🔒 真闸门也跑一遍 —— 这一条正是运行时**会拒掉**的那种
            if tried:
                rec = type('R', (), {'attempts': list(tried)})()
                ok, why = replan.check_new_plan(steps, rec, allow_motion=True)
                print(f'         ⇒ 现有判据 A：{"放行" if ok else "**拒绝**"}'
                      f'{"（" + why + "）" if why else ""}')
            tried.append(steps)
            states.append('BLOCKED')
        print()
    return labels


def report(labels, title):
    print()
    print('=' * 78)
    print(f'  {title}')
    print('=' * 78)
    total = sum(labels.values())
    for tag, n in labels.most_common():
        print(f'    {n:>3}/{total}  ({n / total * 100:5.1f}%)  {tag}')
    print()
    print(verdict_of(labels))
    print('=' * 78)


def main():
    ap = argparse.ArgumentParser(description='真模型的重试风格探针（只读、不碰 ROS、不动车）')
    ap.add_argument('--phase', choices=['ask', 'chain'], default='ask')
    ap.add_argument('--text', default=DEFAULT_TEXT, help='交给模型的任务那句话')
    ap.add_argument('--n', type=int, default=12, help='--phase ask：问几次')
    ap.add_argument('--chains', type=int, default=6, help='--phase chain：跑几条链')
    ap.add_argument('--max-replans', type=int, default=2, help='链长（与运行时的预算同义）')
    ap.add_argument('--registry', default=str(DEFAULT_REGISTRY))
    ap.add_argument('--base-url', default='', help='**必须显式给**，无默认值')
    ap.add_argument('--model', default='', help='**必须显式给**，无默认值')
    ap.add_argument('--api-key-env', default='DEEPSEEK_API_KEY',
                    help='从哪个环境变量读密钥（**不回显**）')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--max-tokens', type=int, default=4096,
                    help='⚠️ 推理模型的思维链也计入额度，给小了会被截断（D-045）')
    args = ap.parse_args()

    if not args.base_url or not args.model:
        print('⛔ 拒测：`--base-url` 与 `--model` 都必须显式给出 —— 不给就什么也不发。')
        return 4
    key = llm_client.read_api_key(args.api_key_env)
    if not key:
        print(f'⛔ 拒测：环境变量 {args.api_key_env} 里没有密钥。')
        print('   （本工装不回显、不落盘、不去找它 —— 请在**能被 source 到**的地方设置它。）')
        return 4
    if not Path(args.registry).is_file():
        print(f'⛔ 拒测：读不到技能注册表 {args.registry}')
        return 4

    reg = Registry.from_yaml(str(args.registry))
    client = llm_client.OpenAiCompatClient(
        base_url=args.base_url, model=args.model, api_key=key,
        timeout_s=args.timeout, max_tokens=args.max_tokens)

    print()
    print('=' * 78)
    print('  真模型的**重试风格**探针 —— D-045 那条待定的取舍')
    print('=' * 78)
    print(f'  端点 {args.base_url}｜模型 {args.model}｜密钥来自 {args.api_key_env}（**不回显**）')
    print(f'  任务：「{args.text}」')
    print()

    labels = phase_ask(args, reg, client) if args.phase == 'ask' \
        else phase_chain(args, reg, client)
    report(labels, '分布' + ('（同一条历史问多次）' if args.phase == 'ask'
                             else '（每条链里每一次重试）'))
    print('  ⚠️ 这是**分布**，不是判分 —— 真模型非确定性，样本量在这里。')
    print('  ⚠️ 结论只对这条提示词与这个模型成立；换一句话就是换一个实验。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
