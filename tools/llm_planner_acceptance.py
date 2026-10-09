#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LLM 规划（D-038）的端到端验收 —— 对着**假端点**跑，不联网、不需要密钥。

    ⚠️ 这是【验收工装】，不是运行时组件。

它验的是"**模型不听话时怎么办**"，不是"模型聪不聪明"
------------------------------------------------------
真模型无法稳定复现"它选了控制层技能""它编了个不存在的技能""它回的话不是 JSON"
—— 而这几件事恰恰是**最危险、最该被验**的。所以假端点把模型的话变成一张固定表
（`tools/fake_llm_endpoint.py`），每种情形都能**指名道姓地**复现。

四段，各自一套启动参数：

    --phase plan   假端点正常回话：八种"模型可能说的话"，每种都该有**说清原因**的结局
    --phase dead   假端点**没起来**：必须报"连不上/调用失败"，**不是**"听不懂"
    --phase slow   假端点**故意不回**：必须超时拒绝，且**节点仍然活着**（status 照发）
                   ⚠️ 这一段要让假端点 `--delay` 大于 Agent 的 `llm_timeout`
                   （例如端点 --delay 30、节点 llm_timeout:=3.0）
    --phase replan ★ 重规划（D-044）：**没人再说话，Agent 自己换了一条走法**
                   ⚠️ 它要**前方近处有回波**（`clear_range` 以内），否则 `BLOCKED`
                   不会出现、重规划也就不会触发 —— 那种情况下工装**拒测**（退出码 4），
                   而不是照跑出个假结论。跑之前先量一下 `/embodied/lidar/front`。

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

⚠️ `--phase replan` 另外两条（都是**刻意**的）：
    · 建议 `max_replans:=1` —— 让"重规划一次"这件事在日志里清清爽爽。
      （默认 2 也能过：第 2 次尝试若再失败，重试表回的还是它 ⇒ 被"必须不一样"挡住。）
    · **假端点要能认出重规划**：它从 `RETRY_PROMPT` 推导判据，所以**必须先
      source 本工作区**，否则启动时会打印"⚠️ 认不出重规划"，那一段就验不了。

用法：
    python3 tools/llm_planner_acceptance.py --phase plan
    python3 tools/llm_planner_acceptance.py --phase dead
    python3 tools/llm_planner_acceptance.py --phase slow
    python3 tools/llm_planner_acceptance.py --phase replan
退出码：0 = 全部断言通过；1 = 有失败；4 = **拒测**（前提不在，什么都没验）。
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from embodied_skills_interfaces.msg import SkillEvent
from embodied_skills_interfaces.srv import AgentTask

SUBMIT = '/agent_runtime/submit'
STATUS = '/embodied/agent/status'
GATEWAY_CANCEL = '/skill_gateway/cancel'
SKILL_EVENTS = '/embodied/skill/events'
DRYRUN_TOPIC = '/embodied/motor/cmd_vel_dryrun'
FRONT_TOPIC = '/embodied/lidar/front'

#: 技能侧的终态（8 状态机里"会结束一次尝试"的那几个 + control-tier 的 FINISHED）。
TERMINAL = frozenset({'ARRIVED', 'TARGET_FOUND', 'TARGET_LOST', 'BLOCKED',
                      'FAILED', 'CANCELLED', 'FINISHED'})

#: ★ 重规划那一相用的任务文本。**文本本身就对假端点有意义**：
#: `往前走但换条路` 在"首次规划"与"带着历史再问"时会拿到**不同**的计划；
#: `再来一次也一样` 两次都拿到**同一条**（用来验防死循环那道闸）。
REPLAN_TEXT = '往前走但换条路'
REPLAN_SAME_TEXT = '再来一次也一样'
#: 这两条计划里 `advance_until_blocked` 用的受阻阈值 —— 前置判据要跟它同源。
REPLAN_CLEAR_RANGE = 0.25
#: 那一相要**恰好两次**派发：第 1 次被挡（BLOCKED）→ 换条路；第 2 次是换出来的那条。
#: 之所以能断言"恰好"，是因为换出来的那条若再失败，重试表回的还是它
#: ⇒ 会被"必须与试过的不一样"挡住（这正是两道闸互补的地方：有界 + 不重复）。

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
        # ---- 重规划那一相要用的观察面 ----
        self.skill_events = []          # [(skill, state), ...] 按到达顺序
        self.wz_seen = 0.0              # 干跑话题上见过的最大 |wz|
        self.front = None               # (range, valid)
        self.create_subscription(SkillEvent, SKILL_EVENTS, self._on_skill_event, 50)
        self.create_subscription(Twist, DRYRUN_TOPIC, self._on_twist, 50)
        self.create_subscription(Float64MultiArray, FRONT_TOPIC, self._on_front, 10)

    def _on_status(self, m):
        self.status = list(m.data)
        self.status_frames += 1

    def _on_skill_event(self, m):
        self.skill_events.append((m.skill, m.state))

    def _on_twist(self, m):
        self.wz_seen = max(self.wz_seen, abs(m.angular.z))

    def _on_front(self, m):
        # [range, angle, valid, 点数]（见 lidar_driver）：**-1 = 没有回波**
        if len(m.data) >= 3:
            self.front = (float(m.data[0]), bool(m.data[2]))

    # ---- 重规划那一相的判据 ----

    def clear_log(self):
        self.skill_events.clear()
        self.wz_seen = 0.0

    def dispatches(self):
        """网关**派发**出去的技能，按到达顺序（`RUNNING` 是派发那一刻发的）。

        ⚠️ 本话题上**只有 task-tier** 事件：control 技能是 Autonomous 技能
        直接用服务调的，**不经过网关**，所以它们不会混进来（SkillEvent 的注释里
        写死了"发布者只有网关一个"）。
        """
        return [(s, st) for s, st in self.skill_events if st == 'RUNNING']

    def terminals(self):
        return [(s, st) for s, st in self.skill_events if st in TERMINAL]

    def wait_for(self, predicate, timeout_s, poll=0.05):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=poll)
        return predicate()

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


def run_replan(node, args, rep):
    """★ 重规划（D-044）：**没人再说话，Agent 自己换了一条走法**。

    分两段，第二段是**反例** —— 少了它，"重规划能用"就只是"它又派了一次"
    而已，分不清派出去的是**新计划**还是**同一条失败过的计划**。

    返回 4 = **拒测**（前提不在，这一段没跑）；返回 0 = 跑完了 ——
    过没过由调用处的汇总说了算（每一次 `rep` 都已经记进 `results`）。
    ⚠️ **「拒测」与「没通过」是两回事**：前者是"这个摆位验不了"，
    后者是"验了、结论不对"。两者都非零退出，但含义完全不同。
    """
    # ---- 0）前置：前方真要有近处回波 ----
    # ⚠️ 重规划的触发器是 `BLOCKED` / `TARGET_LOST`（见 replan.REPLANNABLE），
    #    而当前只有 `advance_until_blocked` 能确定性地产生 `BLOCKED` ——
    #    它要求前方扇区内有东西近于 `clear_range`。摆位不对，这一段**验不了**：
    #    那时若照跑，得到的是"计划一路 ARRIVED、什么也没发生"，看起来像**功能坏了**，
    #    而真相是**前提没有**。所以这里**拒测**，不猜、不迁就。
    if not node.wait_for(lambda: node.front is not None, 10.0):
        print(f'  ⛔ 拒测：读不到 {FRONT_TOPIC} —— LiDAR Driver 在跑吗？')
        return 4
    rng, valid = node.front
    if (not valid) or rng < 0 or rng >= REPLAN_CLEAR_RANGE:
        print(f'  ⛔ 拒测：前方最近回波 {"无" if not valid or rng < 0 else f"{rng:.3f} m"}，'
              f'而计划用的 clear_range={REPLAN_CLEAR_RANGE} m —— 这个摆位不会出现 '
              f'BLOCKED，重规划也就不会被触发。请把障碍物摆到正前方 '
              f'{REPLAN_CLEAR_RANGE} m 以内再来。')
        return 4
    print(f'  · 前置：前方最近回波 {rng:.3f} m < clear_range {REPLAN_CLEAR_RANGE} m ✓'
          f'（`advance_until_blocked` 会立刻报 BLOCKED）')
    print()

    # ================= 一）换了一条走法 =================
    print(f'  ① 提交「{REPLAN_TEXT}」—— 它第一次拿到的计划会被挡住，')
    print(     '     重规划时假端点会给出**另一个技能**（判据是历史提示词）')
    wake_before = node.status[2] if node.status else 0
    node.clear_log()
    res = node.submit(REPLAN_TEXT)
    rep('受理', bool(res and res.accepted), (res.message[:80] if res else '提交超时'))
    if not (res and res.accepted):
        return 0                       # 失败已记在 results 里，交回汇总

    t0 = time.monotonic()
    node.wait_for(lambda: len(node.terminals()) >= 2, args.replan_wait)
    waited = time.monotonic() - t0
    disp, term = node.dispatches(), node.terminals()

    rep('第 1 次尝试被派发到网关', disp[:1] == [('autonomous.advance_until_blocked', 'RUNNING')],
        f'{disp[:1]}')
    rep('第 1 次尝试的终态是 BLOCKED',
        term[:1] == [('autonomous.advance_until_blocked', 'BLOCKED')], f'{term[:1]}')
    rep('★ 没有第二次提交，网关却又收到一次派发（= 重规划真的发生了）',
        len(disp) >= 2, f'派发 {len(disp)} 次：{[s for s, _ in disp]}')
    rep('★ 换出来的技能与第一次**不同**（换了条走法，不是又派了一遍）',
        len(disp) >= 2 and disp[1][0] != disp[0][0],
        f'{disp[0][0] if disp else "?"} → {disp[1][0] if len(disp) > 1 else "（没有第二次）"}')
    rep('★ 恰好两次派发（第 2 次再失败时重试表还是它 ⇒ 被"必须不一样"挡下）',
        len(disp) == 2, f'派发 {len(disp)} 次')
    rep('第 2 次尝试也有终态（没有挂在 WAIT 里）',
        len(term) >= 2, f'{term[1:2]}｜{waited:.1f}s 内')
    rep('★ 第 2 次尝试**真的下达到了轮子**（干跑话题上出现非零 wz）',
        node.wz_seen > 1e-6, f'|wz| 峰值 {node.wz_seen:.3f} rad/s')
    # ⚠️ **不能一看到终态事件就读唤醒计数**：唤醒是 Agent 收到那条事件之后才记的，
    #    而状态话题是 5 Hz 发的 —— 抢读会拿到"上一条事件"时的旧值，
    #    表现是"功能明明做了、工装却说没做"（工装自己的竞态，不是产品的问题）。
    node.wait_for(lambda: node.status is not None
                  and node.status[2] - wake_before >= 2, 10.0)
    wake_after = node.status[2] if node.status else 0
    rep('★ 每一次尝试结束都唤醒 Agent 一次（两次尝试 ≥ 2 次唤醒）',
        wake_after - wake_before >= 2, f'唤醒 {wake_before:g} → {wake_after:g}')
    rep('任务已收尾（在途/在等归零）',
        node.wait_for(lambda: node.status and node.status[1] == 0, 15.0),
        f'status={node.status}')
    print('  ① 期间的技能事件：')
    for s, st in node.skill_events:
        print(f'      {st:<12} {s}')
    print()

    # ================= 二）反例：回来的还是同一条 =================
    print(f'  ② 反例：「{REPLAN_SAME_TEXT}」—— 重规划回的是**一模一样**的计划，')
    print(     '     必须被"新计划必须与试过的不一样"当场挡住，**不派发**')
    node.clear_log()
    res = node.submit(REPLAN_SAME_TEXT)
    rep('反例受理', bool(res and res.accepted), (res.message[:80] if res else '提交超时'))
    if not (res and res.accepted):
        return 0                       # 同上
    node.wait_for(lambda: len(node.terminals()) >= 1, args.replan_wait)
    rep('反例的第 1 次尝试报 BLOCKED',
        node.terminals()[:1] == [('autonomous.advance_until_blocked', 'BLOCKED')],
        f'{node.terminals()[:1]}')
    # 再干等一段：重规划至少要问一次网络（假端点**立刻**回），
    # 真被放行的话这段时间里一定看得见第二次派发。
    node.spin_for(8.0)
    rep('★ 重规划拿到同一条计划 ⇒ 拒绝派发（防死循环那道闸生效）',
        len(node.dispatches()) == 1, f'派发 {len(node.dispatches())} 次')
    rep('反例任务也收尾了（不是把记录漏在表里）',
        node.wait_for(lambda: node.status and node.status[1] == 0, 15.0),
        f'status={node.status}')
    print()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=['plan', 'dead', 'slow', 'replan'], required=True)
    ap.add_argument('--settle', type=float, default=2.5,
                    help='受理之后等技能跑完的时间（秒）')
    ap.add_argument('--replan-wait', type=float, default=60.0,
                    help='replan 相里等"两次尝试跑完"的上限（秒）；'
                         '第 2 次尝试要转好几步，留足')
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

        elif args.phase == 'replan':
            # ⚠️ 它自己已经逐条报过了（rep）；这里只把**拒测**与失败透传出去，
            #    后面那段汇总照旧打印 —— 拒测不是"有断言没过"，两者要分得开。
            rc = run_replan(node, args, rep)
            if rc != 0:
                return rc

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
