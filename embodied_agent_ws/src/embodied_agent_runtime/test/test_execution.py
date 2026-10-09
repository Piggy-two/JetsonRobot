#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""钉住 WAIT 的推进语义 —— 尤其是「**只有任务级事件唤醒 Agent**」（D-004）、
**多步计划的推进规则**（D-043）与**重规划的记账**（D-044）。

⚠️ 2026-10-09 把动作 `WAKE` 改名成 `PLAN_ENDED`：加了重规划之后，
"这个**计划**结束了"与"这个**任务**结束了（⇒ 该唤醒 Agent）"**不再是同一件事** ——
计划失败可能只是换个走法再来一次。旧名字会让人（和我）在节点里写出
"计划一结束就唤醒"的代码，而那正好把重规划**绕过**了。改名是重规划逼出来的。
"""

import pytest

from embodied_agent_runtime import execution as ex
from embodied_agent_runtime.planner import Step
from embodied_skill_gateway import task_state as ts

ADV = 'autonomous.advance_until_blocked'
TURN = 'autonomous.turn_until_clear'


def _exec(**kw):
    return ex.Executor(**kw)


def _submit(e, task_id='t1', skill=ADV, timeout_s=10.0, now=100.0, steps=None):
    """提交一个计划并把第 1 步绑到 `task_id` 上（单步时计划 id 与它相同）。"""
    steps = steps if steps is not None else [Step(skill, {})]
    rec = e.submit_plan(task_id, steps, 'agent.planner', timeout_s, now=now)
    if rec is not None:
        e.bind_step(task_id, 0, task_id)
    return rec


# ---------- 提交 ----------

def test_submit_creates_a_pending_task_with_a_monotonic_deadline():
    e = _exec()
    rec = _submit(e, timeout_s=10.0, now=100.0)
    assert rec is not None
    assert rec.deadline == pytest.approx(110.0)
    assert rec.woken is False
    assert rec.total == 1
    assert e.get('t1') is rec


def test_duplicate_task_id_is_refused():
    e = _exec()
    _submit(e)
    assert _submit(e) is None


def test_empty_plan_is_refused():
    """一个没有步骤的计划没有意义 —— 它永远不会产生终态，只会挂在那儿。"""
    e = _exec()
    assert e.submit_plan('t1', [], 'agent.planner', 10.0, now=100.0) is None


# ==========================================================================
# ★ 「只有任务级事件唤醒 Agent」
# ==========================================================================

def test_task_tier_terminal_ends_the_attempt_but_not_the_task():
    """★ 任务级终态 ⇒ **这次尝试**结束。⚠️ 它**不**自己把任务收尾 ——
    收尾是调用方的决定（可能还要重规划），见 `finish`。

    （D-004 的原话是"只有任务级事件唤醒 Agent"。加了重规划之后要补一句：
    任务级终态**让 Agent 拿到话轮**，而这一轮里它可能选择"换个走法再来一次"，
    那时候任务并没有结束。把这两件事塞进一个标志里，重规划就做不成。）"""
    e = _exec()
    _submit(e)
    out = e.on_event('t1', ts.ARRIVED)
    assert out.action == ex.PLAN_ENDED
    assert e.get('t1').ended is True
    assert e.get('t1').woken is False        # ← 任务还没收尾


@pytest.mark.parametrize('state', [ts.BLOCKED, ts.FAILED, ts.CANCELLED,
                                   ts.TARGET_FOUND, ts.TARGET_LOST])
def test_every_task_terminal_ends_a_one_step_plan(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state).action == ex.PLAN_ENDED


def test_control_tier_finished_does_NOT_wake_the_agent():
    """★ 这是本文件最要紧的一条。

    `FINISHED` 只表示「技能返回了」，**不表示任务结束了**（D-032）。
    若它也算唤醒，Agent 会在一次开环动作返回时就以为任务完成，
    然后基于这个假前提继续规划 —— 而车可能只是走了一小段。
    """
    e = _exec()
    _submit(e, skill='control.move_relative')
    assert e.on_event('t1', ts.FINISHED).action == ex.RECORD
    assert e.get('t1').woken is False


@pytest.mark.parametrize('state', [ts.STARTED, ts.RUNNING])
def test_intermediate_states_are_recorded_but_do_not_wake(state):
    e = _exec()
    _submit(e)
    assert e.on_event('t1', state).action == ex.RECORD
    assert e.get('t1').woken is False
    assert e.get('t1').last_step_state == state


def test_events_for_other_tasks_are_ignored_not_alarmed():
    """事件话题是**共享的** —— 看见别人的任务事件是正常的，不该报警。

    （路由器、人工验收工装都会经过同一个网关，它们的事件也在这条话题上。）
    """
    e = _exec()
    _submit(e, task_id='mine')
    assert e.on_event('someone-elses-task', ts.ARRIVED).action == ex.IGNORE


def test_duplicate_terminal_does_not_wake_twice():
    """终态恰好一个（`task_table` 保证）。真到这一步说明上游出了问题 ——
    但无论如何**不能唤醒两次**：Agent 会以为有两个任务先后完成。"""
    e = _exec()
    _submit(e)
    assert e.on_event('t1', ts.ARRIVED).action == ex.PLAN_ENDED
    assert e.on_event('t1', ts.ARRIVED).action == ex.DUPLICATE
    assert e.on_event('t1', ts.BLOCKED).action == ex.DUPLICATE


# ==========================================================================
# ★ ★ 多步计划（D-043）
# ==========================================================================

def test_multi_step_continues_after_a_non_aborting_terminal():
    """第 1 步到位 ⇒ 去派发第 2 步，**且此时不唤醒 Agent**。

    中途唤醒会让 Agent 对着一个"只走了一半"的世界重新规划，
    而那正是它刚规划过的东西（D-043）。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    out = e.on_event('p', ts.ARRIVED)          # 第 1 步的网关任务号就是 'p'
    assert out.action == ex.DISPATCH_NEXT
    assert out.step_index == 2
    assert e.get('p').woken is False


@pytest.mark.parametrize('state', [ts.ARRIVED, ts.BLOCKED, ts.TARGET_FOUND, ts.TARGET_LOST])
def test_multi_step_continues_on_every_non_aborting_terminal(state):
    """★ **`BLOCKED` 必须继续** —— 脱困计划就是"前进被挡 → 转身 → 再前进"。

    把 `BLOCKED` 当中止，脱困就永远走不到第 2 步。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', state).action == ex.DISPATCH_NEXT


@pytest.mark.parametrize('state', [ts.FAILED, ts.CANCELLED])
def test_multi_step_aborts_immediately_on_failed_or_cancelled(state):
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    out = e.on_event('p', state)
    assert out.action == ex.PLAN_ENDED
    assert out.terminal == state
    assert e.get('p').ended is True


def test_last_step_terminal_ends_the_plan():
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', ts.ARRIVED).action == ex.DISPATCH_NEXT
    # 第 2 步由调用方派发后绑回来
    assert e.bind_step('p', 1, 'p-s2') is True
    out = e.on_event('p-s2', ts.BLOCKED)
    assert out.action == ex.PLAN_ENDED
    assert out.terminal == ts.BLOCKED
    assert '最后一步' in out.reason


def test_a_late_event_from_an_earlier_step_does_not_advance_the_plan():
    """迟到的旧步骤事件**不能**推进计划 —— 那会让"第 2 步的终态"去驱动一个
    已经走到第 3 步的计划。（超时后又被停下来的那次，就会迟到。）"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {}), Step(ADV, {})])
    e.on_event('p', ts.ARRIVED)                # → 派发第 2 步
    e.bind_step('p', 1, 'p-s2')
    e.on_event('p-s2', ts.ARRIVED)             # → 派发第 3 步
    e.bind_step('p', 2, 'p-s3')
    late = e.on_event('p-s2', ts.FAILED)       # 第 2 步的迟到终态
    assert late.action == ex.RECORD
    assert e.get('p').woken is False
    assert e.get('p').current == 2             # 还停在第 3 步上


# ---------- 超时 ----------

def test_expired_only_returns_tasks_past_deadline_and_not_yet_woken():
    e = _exec()
    _submit(e, task_id='t1', timeout_s=5.0, now=100.0)
    _submit(e, task_id='t2', timeout_s=50.0, now=100.0)
    assert e.expired(now=104.0) == []
    assert [r.task_id for r in e.expired(now=106.0)] == ['t1']


def test_already_woken_task_never_expires():
    e = _exec()
    _submit(e, timeout_s=1.0, now=100.0)
    e.on_event('t1', ts.ARRIVED)
    assert e.expired(now=1e9) == []


def test_timeout_marks_the_plan_terminal_without_going_through_on_event():
    """⚠️ 超时不能用 `on_event(plan_id, ...)` 收尾：`on_event` 认的是**当前那一步**
    的网关任务号，一个走到第 3 步的计划拿第 1 步的号去推进，会**原地不动**
    —— 表现就是"超时了却永远醒不过来"。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    e.on_event('p', ts.ARRIVED)                # 计划走到第 2 步
    e.bind_step('p', 1, 'p-s2')
    assert e.get('p').woken is False
    assert e.timeout('p') is True
    assert e.get('p').ended is True
    assert e.get('p').ended_state == ts.FAILED
    assert e.timeout('p') is False             # 幂等


def test_current_step_task_id_follows_the_plan():
    """超时要取消的是**当前那一步**，不是计划 id（取消第 1 步等于什么也没停）。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.current_step_task_id('p') == 'p'
    e.on_event('p', ts.ARRIVED)
    e.bind_step('p', 1, 'p-s2')
    assert e.current_step_task_id('p') == 'p-s2'


# ---------- 有界 ----------

def test_executor_evicts_woken_tasks_when_full():
    e = _exec(capacity=3)
    for i in range(6):
        _submit(e, task_id=f't{i}', now=100.0 + i)
        e.on_event(f't{i}', ts.ARRIVED)
    assert len(e) <= 3


def test_executor_never_drops_a_still_waiting_task():
    """丢掉一个还在等的任务 = Agent 永远等不到它 —— 比"表满拒绝新任务"糟得多。"""
    e = _exec(capacity=2)
    _submit(e, task_id='w1')
    _submit(e, task_id='w2')
    assert _submit(e, task_id='w3') is None      # 拒绝，而不是丢 w1
    assert e.get('w1') is not None
    assert e.get('w2') is not None


def test_waiting_and_stats():
    e = _exec(capacity=5)
    _submit(e, task_id='t1')
    _submit(e, task_id='t2')
    e.on_event('t1', ts.ARRIVED)
    assert [r.task_id for r in e.waiting()] == ['t2']
    s = e.stats()
    assert s['tracked'] == 2 and s['waiting'] == 1 and s['capacity'] == 5


def test_capacity_must_be_positive():
    with pytest.raises(ValueError):
        ex.Executor(capacity=0)


def test_outcome_carries_the_plan_id_not_the_gateway_task_id():
    """★ 回归：**端到端第一次就栽在这里**。

    事件里给的是**网关任务号**，而调用方要按**计划 id** 去取记录
    （派发下一步、读计划里剩下的步骤）。第 2 步之后这两个号**就不同了** ——
    outcome 不带 `plan_id` 的话，调用方会拿第 2 步的任务号去查计划，
    查不到，于是**静默不派发下一步**：日志里"计划继续…下一步 第 3 步"打出来了，
    而第 3 步永远不会发生。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    assert e.on_event('p', ts.ARRIVED).plan_id == 'p'
    e.bind_step('p', 1, 'p-s2')
    out = e.on_event('p-s2', ts.ARRIVED)
    assert out.action == ex.PLAN_ENDED
    assert out.plan_id == 'p'          # ← **不是** 'p-s2'


# ==========================================================================
# ★ ★ 重规划的记账（D-044）
# ==========================================================================

def test_first_attempt_is_not_a_replan():
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    rec = e.get('p')
    assert rec.replans == 0
    assert len(rec.attempts) == 1


def test_start_attempt_keeps_the_plan_id_but_replaces_the_steps():
    """★ 重规划**不换计划 id**：任务还是那个任务，换的只是"这次怎么走"。

    换 id 的话，调用方得同时记住"任务号"和"计划号"两个东西，
    而用户在日志里看到的是**两个不同的任务** —— 明明只是一个任务换了走法。
    """
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    e.on_event('p', ts.BLOCKED)                       # 第 1 步被挡 → 值得重规划
    rec = e.start_attempt('p', [Step(TURN, {'direction': 'left'})], 10.0, now=105.0)
    assert rec is not None
    assert rec.task_id == 'p'                         # ← 计划 id 不变
    assert rec.replans == 1
    assert len(rec.attempts) == 2
    assert [s.skill for s in rec.steps] == [TURN]


def test_start_attempt_restarts_the_walk_from_step_zero():
    """新尝试从**第 1 步**开始走 —— 沿用它上次走到的下标会让新计划的第 1 步被跳过。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {}), Step(TURN, {})])
    e.on_event('p', ts.ARRIVED)                       # 走到第 2 步
    e.bind_step('p', 1, 'p-s2')
    rec = e.start_attempt('p', [Step(ADV, {}), Step(TURN, {}), Step(ADV, {})], 10.0, now=105.0)
    assert rec.current == 0
    assert rec.dispatched is False
    assert rec.step_task_ids == [None, None, None]
    assert rec.total == 3


def test_start_attempt_gets_a_fresh_deadline():
    """★ 新尝试必须**重新计时**。旧截止时刻可能早就过了 —— 沿用它的话，
    刚规划出来的尝试会在下一次 sweep 里**立刻**被判超时，表现为
    "重规划了但一步都没走"。"""
    e = _exec()
    _submit(e, task_id='p', timeout_s=5.0, now=100.0)
    assert e.get('p').deadline == pytest.approx(105.0)
    rec = e.start_attempt('p', [Step(TURN, {})], 5.0, now=104.0)
    assert rec.deadline == pytest.approx(109.0)       # 从 104 起算，不是 100


def test_a_late_event_from_the_abandoned_attempt_cannot_advance_the_new_one():
    """★ 旧尝试的步骤**可能还在跑**（比如它报 BLOCKED 之后才真正停稳）。
    它的事件若还能推进新尝试，新计划的第 1 步就会被一次**来自旧计划的终态**
    推着往前走 —— 而那个终态描述的是一件已经放弃的事。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    e.on_event('p', ts.BLOCKED)
    e.start_attempt('p', [Step(TURN, {})], 10.0, now=105.0)
    e.bind_step('p', 0, 'p-try2')
    late = e.on_event('p', ts.ARRIVED)                # 第 1 次尝试的迟到终态
    assert late.action == ex.IGNORE
    assert e.get('p').current == 0
    assert e.get('p').woken is False


def test_start_attempt_is_allowed_after_the_attempt_ended():
    """★ 尝试结束**不等于**任务结束 —— 这正是不许用 `woken` 兼任两者的理由。
    少了这条，重规划会被自己刚收到的终态挡死，而且**不报错**。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    e.on_event('p', ts.BLOCKED)
    assert e.get('p').ended is True
    assert e.start_attempt('p', [Step(TURN, {})], 10.0, now=105.0) is not None


def test_start_attempt_is_refused_after_the_task_is_finished():
    """任务**收尾之后**（`finish`）不许再开新尝试 —— 那会让一个已经交代过的任务
    重新活过来，而调用方那边早就把它当完成了。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    e.on_event('p', ts.FAILED)
    assert e.finish('p') is True
    assert e.get('p').woken is True
    assert e.start_attempt('p', [Step(TURN, {})], 10.0, now=105.0) is None
    assert e.get('p').replans == 0


def test_start_attempt_on_an_unknown_plan_is_none_not_an_exception():
    """查不到 ≠ 异常：调用方可能正好撞上容量淘汰，那时候该做的是唤醒，不是崩。"""
    e = _exec()
    assert e.start_attempt('nope', [Step(TURN, {})], 10.0, now=105.0) is None


def test_start_attempt_refuses_an_empty_plan():
    """空计划永远不产生终态 —— 接受它等于把任务挂死在那儿。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    assert e.start_attempt('p', [], 10.0, now=105.0) is None
    assert e.get('p').replans == 0


def test_the_original_text_is_kept_for_the_next_question():
    """★ 重规划要**拿原话再问一次规划器** —— 所以文本必须留在记录里。
    存"归一化后的键"不够：LLM 那一跳要的是人原本说的那句话。"""
    e = _exec()
    e.submit_plan('p', [Step(ADV, {})], 'agent.planner', 10.0, now=100.0,
                  text='往前走，被挡就绕开')
    assert e.get('p').text == '往前走，被挡就绕开'


def test_finish_closes_the_task_with_the_state_that_ended_it():
    """★ `finish` 是"计划结束"与"任务结束"拆开之后**必需**的一步。
    少了它，记录会永远停在 `ended` 上：既不超时、也不被淘汰 —— 就漏在表里了。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    e.on_event('p', ts.BLOCKED)
    assert e.finish('p') is True
    assert e.get('p').woken_state == ts.BLOCKED     # 沿用结束它的那个终态
    assert e.get('p').pending is False
    assert e.waiting() == []
    assert e.finish('p') is False                   # 幂等


def test_an_ended_but_unfinished_attempt_no_longer_expires():
    """已结束的尝试**不再**报超时：它的 deadline 停在过去，而它只是在等节点
    决定"重规划还是收尾"。不排除掉的话，每次 sweep 都会把同一个计划再报一遍超时,
    调用方就会一遍遍地去取消一个早就没在跑的任务。"""
    e = _exec()
    _submit(e, task_id='p', timeout_s=5.0, now=100.0)
    e.on_event('p', ts.BLOCKED)
    assert e.expired(now=1e9) == []


def test_the_executor_does_not_decide_whether_a_timeout_deserves_a_retry():
    """★ 执行器**只记账，不定政策**。

    超时之后它照样允许开新尝试 —— 因为"该不该重试"是**策略**，属于
    `replan.should_replan`（而它今天对超时标的 `FAILED` 说不：技能可能还在跑，
    这时候再派一条计划就可能两条技能同时抢底盘）。

    钉在这里的是**边界**：这个模块不去猜政策。哪天有人把政策挪进来，
    这条测试会先炸 —— 而那正是提醒他"政策只有一处"的地方。
    """
    e = _exec()
    _submit(e, task_id='p', timeout_s=5.0, now=100.0)
    assert e.timeout('p') is True
    rec = e.start_attempt('p', [Step(TURN, {})], 5.0, now=200.0)
    assert rec is not None
    assert rec.ended is False
    assert rec.expires_at(200.0) is False
    assert e.expired(now=201.0) == []


def test_cancel_is_terminal_and_not_replannable():
    """取消是**用户**说"停"，不是"这条路走不通" —— 所以它一次到位，
    不允许再被重规划唤起（否则用户按了停，车又自己想了个办法动起来）。"""
    e = _exec()
    _submit(e, task_id='p', steps=[Step(ADV, {})])
    assert e.cancel('p') is True
    assert e.get('p').woken is True
    assert e.get('p').woken_state == ts.CANCELLED
    assert e.start_attempt('p', [Step(TURN, {})], 10.0, now=105.0) is None


def test_eviction_does_not_drop_an_attempt_awaiting_the_replan_decision():
    """刚结束、还在等"重规划还是收尾"的记录**不能**被淘汰 ——
    淘汰掉的话 `start_attempt` / `finish` 都找不到它，那笔账就永久丢了。"""
    e = _exec(capacity=2)
    _submit(e, task_id='a', steps=[Step(ADV, {})])
    e.on_event('a', ts.BLOCKED)              # a 结束尝试、等决定
    _submit(e, task_id='b', steps=[Step(ADV, {})])
    _submit(e, task_id='c', steps=[Step(ADV, {})])   # 满了
    assert e.get('a') is not None
    assert e.start_attempt('a', [Step(TURN, {})], 10.0, now=105.0) is not None
