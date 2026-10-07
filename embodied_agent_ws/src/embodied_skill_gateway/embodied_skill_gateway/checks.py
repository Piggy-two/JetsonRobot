#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Skill 网关的**六项检查**（**纯 Python，不依赖 ROS**）。

D-005 的原文是「Tool / Skill 请求必须经过 Safety Gateway（Schema 校验 / 权限 /
范围检查 / 超时 / 取消 / 结果校验）」，plan.md §19 给了同一张图，例子是：

    LLM: move_relative(100m)
    系统: REJECTED — distance exceeds safety limit

⚠️ **网关不能退化成单纯代理。** 判据很硬：

> **它必须拒绝某些底层技能会接受的东西。**

具体有**三类**非透传行为，缺一条就说明它只是个转发器：

  1. **Permission** —— `agent.planner` 调 `control.move_relative` 被拒，
     而底层 Control Skill 会痛快地接受（它只管运动和前置条件，不管"谁在问"）。
     这是 D-003「Agent 不得触达控制技能」**唯一**能真正执行的地方。
  2. **更严的 Range** —— 注册表给 `move_relative` 的位移上限**小于**
     Control Skill 自己的 `max_distance`。策略是"允不允许要"，与"能不能做到"是两件事。
  3. **语义改写** —— Control Skill 的 `success` 被**强制**标成 `verified=false`，
     且 control-tier **不得**产出 `ARRIVED` 这类任务终态（D-032）。

这三条都写成了单测（"拒绝底层会接受的东西"），它们是**反代理的回归测试**。
"""

import json
import math

from embodied_skill_gateway import task_state as ts

# 拒绝原因统一前缀，便于上层与日志一眼分辨"被策略拦下"与"技能自己失败"
REJECT = 'REJECTED'


class AdmitResult:
    """一次准入判定的结果。"""

    __slots__ = ('accepted', 'spec', 'args', 'reason')

    def __init__(self, accepted, spec=None, args=None, reason=''):
        self.accepted = accepted
        self.spec = spec
        self.args = args or {}
        self.reason = reason

    def __repr__(self):
        return (f'AdmitResult(accepted={self.accepted}, '
                f'skill={getattr(self.spec, "name", None)}, reason={self.reason!r})')


# ---------- 1. Schema Validation ----------

def validate_schema(spec, args):
    """参数结构是否合法。返回原因字符串，或 None 表示通过。

    校验的六件事（**全部**是强类型服务替我们做不了的，见 `SkillInvoke.srv` 的注释）：
      ① 必填参数在不在
      ② 有没有**多余**的参数（打错字要报错，不能静默忽略）
      ③ 每个参数类型对不对（含 bool 不能当 int 用）
      ④ 数值是不是有限数（NaN / inf 一律拒绝）
      ⑤ 字符串不能为空
      ⑥ 参数个数与注册表一致（不允许多传少传）
    """
    if not isinstance(args, dict):
        return f'参数必须是 JSON object，得到 {type(args).__name__}'

    known = {p.name: p for p in spec.params}

    missing = [n for n, p in known.items() if p.required and n not in args]
    if missing:
        return f'缺少必填参数 {missing}'

    extra = [k for k in args if k not in known]
    if extra:
        return (f'出现未定义的参数 {extra}（本技能只接受 {sorted(known)}）'
                f'—— 参数名打错必须报错，不能静默忽略')

    for name, value in args.items():
        p = known[name]
        if not p.accepts_python_type(value):
            return f'参数 {name} 期望 {p.type}，得到 {type(value).__name__}'
        if p.type in ('float', 'int'):
            if not math.isfinite(float(value)):
                return f'参数 {name} 不是有限数（{value}）'
        if p.type == 'string' and not str(value).strip():
            return f'参数 {name} 是空字符串'
    return None


# ---------- 2. Permission ----------

def validate_permission(spec, principal):
    """调用者有没有权限。返回原因字符串，或 None 表示通过。

    ⚠️ **未登记的 principal 一律拒绝**，没有"默认放行"这条路。
    默认放行会让 D-003 的架构红线在第一次有人加新调用方时就静默失效。
    """
    if not principal or not str(principal).strip():
        return '未提供 principal —— 拒绝（不做匿名放行）'
    if principal not in spec.allowed_principals:
        return (f'{principal} 无权调用 {spec.name}'
                f'（允许的调用方：{sorted(spec.allowed_principals)}）')
    return None


# ---------- 3. Range Check ----------

def validate_range(spec, args):
    """参数是否在**策略**边界内。返回原因字符串，或 None 表示通过。

    ⚠️ 与 `motion_plan.py` 的 `max_distance` 分工不同：
        motion_plan 管「**能不能做到**」；这里管「**允不允许要**」。
    """
    for name, value in args.items():
        p = spec.param(name)
        if p is None or p.type not in ('float', 'int'):
            continue
        v = float(value)
        if p.minimum is not None and v < p.minimum - 1e-9:
            return (f'参数 {name} = {v:g}{p.unit} 低于策略下限 '
                    f'{p.minimum:g}{p.unit}')
        if p.maximum is not None and v > p.maximum + 1e-9:
            reason = (f'参数 {name} = {v:g}{p.unit} 超过策略上限 '
                      f'{p.maximum:g}{p.unit}')
            if p.unit:
                reason += f'（该值会被记为 {v}{p.unit}）'
            return reason

    norm = spec.max_norm
    if norm is not None:
        vals = [float(args[n]) for n in norm['params'] if n in args]
        if len(vals) == len(norm['params']):
            mag = math.sqrt(sum(v * v for v in vals))
            if mag > norm['max'] + 1e-9:
                detail = '、'.join(f'{n}={float(args[n]):g}' for n in norm['params'])
                note = f'（{norm["note"]}）' if norm['note'] else ''
                # ⚠️ 这正是 D-005 举的例子：move_relative(100m) → REJECTED
                return (f'{detail} 的模长 {mag:.3f} 超过策略上限 '
                        f'{norm["max"]:g} {note}')
    return None


# ---------- 6. Result Validation ----------

def validate_result(tier, success, elapsed, message=''):
    """技能返回的结果自洽吗。返回 (verified, 原因或 None)。

    ⚠️ **`verified` 恒为 False，这是刻意的语义改写，不是遗漏（D-026 / D-032）**：

        Control Skill 的 `success` 含义是「**速度按时长发完了**」，
        **不是**「走到位了」。当前没有任何独立反馈（里程计闭环 / 视觉）能确认目标达成，
        所以"验证通过"这件事**今天在物理上就无法成立**。

        上层**不得**把 verified=false 读成"已经到位"。将来有了独立反馈，
        这个函数会按技能类型返回 True —— 那时它才有真实含义。
    """
    if not isinstance(success, bool):
        return False, f'技能返回的 success 不是布尔值（{type(success).__name__}）'
    if not isinstance(elapsed, (int, float)) or not math.isfinite(float(elapsed)):
        return False, f'耗时不是有限数（{elapsed!r}）'
    if float(elapsed) < 0:
        return False, f'耗时为负（{elapsed}）'
    if not success and not str(message).strip():
        return False, '技能报告失败却没给原因 —— 空原因会让上层无法判断该重试还是该放弃'
    return False, None


def validate_task_terminal(state):
    """**task-tier** 技能自报的终态是否合法。返回原因字符串，或 None 表示通过。

    这是 task-tier 的 Result Validation。与 control-tier 的关键差别：
    control-tier 的终态由**网关**决定（`success` → `FINISHED`），
    而 task-tier 的终态由**技能自己报**（`ARRIVED` / `BLOCKED` / …）——
    因为只有技能知道"是走到了"还是"被挡了"。

    于是这里必须**校验它报的那个词**：

      · 必须是已知的、且是**终态**（报个 `RUNNING` 回来是无效的）；
      · **不得是 `FINISHED`** —— 那是 control-tier 的词汇。
        task-tier 用它，Agent 会因为 `FINISHED` **不唤醒**而永远 WAIT，
        而且"技能返回了"被伪装成"任务结束了"（D-032）。
    """
    if not state:
        return 'task-tier 技能必须自报终态（ARRIVED / BLOCKED / FAILED / …）'
    if not ts.is_known(state):
        return f'未知的状态名 {state!r}'
    if not ts.is_terminal(state):
        return f'{state} 不是终态 —— task-tier 返回时必须已到终态'
    if state == ts.FINISHED:
        return (f'{ts.FINISHED} 是 control-tier 的终态，task-tier 任务不得用它 '
                f'（那等于用任务的词汇表撒谎，且不会唤醒 Agent，D-032）')
    return None


def format_reject(reason):
    """把原因包装成统一的 `REJECTED: ...`。"""
    return f'{REJECT}: {reason}'


# ---------- 总入口 ----------

def admit(registry, principal, skill, args_json):
    """完整准入判定。返回 `AdmitResult`。

    顺序是刻意的：**先授权、再校验**。

    先判 Permission 而不是先判 Schema，是因为"你不该调这个技能"与
    "你的参数写错了"是两件不同的事 —— 前者是架构约束，不该被一个参数笔误
    掩盖成后者（那会让人以为"把参数改对就能调了"）。
    """
    spec = registry.get(skill)
    if spec is None:
        return AdmitResult(False, reason=format_reject(
            f'未注册的技能 {skill!r}（已注册：{list(registry.names())}）'))

    if not spec.available:
        why = f'：{spec.unavailable_reason}' if spec.unavailable_reason else ''
        return AdmitResult(False, spec=spec,
                           reason=format_reject(f'技能 {skill} 当前不可用{why}'))

    why = validate_permission(spec, principal)
    if why is not None:
        return AdmitResult(False, spec=spec, reason=format_reject(why))

    try:
        args = json.loads(args_json) if isinstance(args_json, str) else args_json
    except (ValueError, TypeError) as e:
        return AdmitResult(False, spec=spec,
                           reason=format_reject(f'args_json 不是合法 JSON：{e}'))

    why = validate_schema(spec, args)
    if why is not None:
        return AdmitResult(False, spec=spec, reason=format_reject(why))

    why = validate_range(spec, args)
    if why is not None:
        return AdmitResult(False, spec=spec, args=args, reason=format_reject(why))

    return AdmitResult(True, spec=spec, args=args)
