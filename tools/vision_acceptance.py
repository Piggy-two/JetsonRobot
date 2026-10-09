#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉链路的**端到端验收** —— 经网关派发 `semantic.look_for` / `look_on_side`，验它的**语义**。

    ⚠️ 这是【验收工装】。它**只读、不动车**：全程不发任何速度指令。

为什么验的是"语义"而不是"准不准"
----------------------------------
检测得准不准是**模型**的事，本工装管不着；**能不能信它给的结论**才是这条链的责任
（D-046 / DEV_NOTES 坑 43/45/46）。所以几个相位分别钉住几句不同的话：

    [1] 画面里真有的东西            → `TARGET_FOUND`
    [2] 同一个东西、门槛抬到够不着   → `TARGET_LOST`，**但 detail 必须说"看到了、只是低于门槛"**
    [3] 模型类别表里**没有**的类别   → **`FAILED`**（"不知道"），**绝不能是 `TARGET_LOST`**
    [4] 画面里确认没有的东西        → `TARGET_LOST`，且 detail 要说清依据（几帧 / 清晰度）
    [5] 东西在**另一侧**时问这一侧   → `TARGET_LOST`，且 detail 必须点明"哪半幅"
    [6] `side` 写成不认识的词        → **`FAILED`**（"不知道"），**绝不能是 `TARGET_LOST`**

⚠️ [2] 与 [3] 是两种**完全不同**的"没有"，把它们混起来是本项目花了一整天才分开的事：
    [2] 是"我看见了，但按你的标准不算数"（该做的是放宽门槛）；
    [3] 是"我根本不知道这是什么"（该做的是换个类别名）。
    两者都报成"没有"的时候，Agent 会得出"这里没有目标"并真的走开。

⚠️ [6] 要防的是**最隐蔽**的一种错：把不认识的 `side` **当成"不限"**。
    那样得到的答案（"整幅里没有"）看起来完全正常，而调用方问的是"左半幅" ——
    这跟 [3] 是同一族：**一个永远不会报错的错答案**。

⚠️ [5] 有个前提：那个类别必须**只出现在一侧**。两侧都有实例时本工装**跳过它**
    （⏭，不算通过也不算失败）—— 硬测会测出一个与 `side` 无关的结论。

外加一条**结构不变量**：本链路的技能是 `causes_motion: false`，
所以全程 `/cmd_vel` 与干跑话题上**非零帧数必须恒为 0**（"它永远不会命令运动"）。

前置（**不需要动车**，但需要相机在出图）：
    ros2 launch embodied_bringup demo.launch.py            # 九节点（含视觉）
    # 或：分别起 skill_gateway / vision_driver / semantic_skills

用法：
    python3 tools/vision_acceptance.py
退出码：0 = 全部通过；1 = 有失败；4 = **拒测**（相机没出图 / 模型没就绪 / 网关不在）。
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from embodied_skills_interfaces.msg import VisionDetections
from embodied_skills_interfaces.srv import SkillInvoke, SkillResult

INVOKE = '/skill_gateway/invoke'
GET_RESULT = '/skill_gateway/get_result'
DETECTIONS = '/embodied/vision/detections'
DIAG = '/embodied/vision/diag'
MOTOR_DRYRUN = '/embodied/motor/cmd_vel_dryrun'
CMD_VEL = '/cmd_vel'

SKILL = 'semantic.look_for'
SKILL_SIDE = 'semantic.look_on_side'
PRINCIPAL = 'agent.planner'

#: 用哪几个类别名探"模型不认识" —— 挑 COCO 里**几乎不可能出现在室内**的，
#: 于是它们要么是"模型不认识"（[3] 要验的），要么是"确认没有"（[4]）。
#: ⚠️ 换了模型这张表就不适用了 —— 那时 [3] 会失败并**说清**是为什么（见下）。
FOREIGN_LABEL = '杯子'          # 中文名，通用 COCO 模型必然不认识
ABSENT_LABEL = 'zebra'         # COCO 里的一类，室内画面里必然没有
BAD_SIDE = 'middle'            # 不认识的 side 值（[6] 用）

#: 挑用例时"离中线够远"的余量。
#: ⚠️ 这**不是**规则的副本 —— 规则只有一处（`vision_query.SIDE_THRESHOLD` = 1/3）。
#:    这里只要一个"这个目标稳稳落在某一侧"的判据，取 0.5 是**故意比规则更严**：
#:    更严只会让本工装挑用例时更保守，**不会**让两份判据分家（D-034 的老教训）。
SIDE_MARGIN = 0.5


class Rig(Node):
    """订阅服务端与**所有可能被命令运动的地方**，用来钉那条结构不变量。"""

    def __init__(self):
        super().__init__('vision_acceptance')
        self.diag = None
        self.labels = {}
        #: 类别名 → 它**稳稳落在**哪几侧（{'left'} / {'right'} / {'left','right'}）。
        #: 只收 |side| ≥ `SIDE_MARGIN` 的实例 —— 见那个常数的说明。
        self.halves = {}
        self.nonzero_motion = 0
        self.motion_frames = 0
        self.create_subscription(Float64MultiArray, DIAG, self._on_diag, 10)
        self.create_subscription(VisionDetections, DETECTIONS, self._on_dets, 20)
        for t in (MOTOR_DRYRUN, CMD_VEL):
            self.create_subscription(Twist, t, self._on_twist, 50)
        self.invoke_cli = self.create_client(SkillInvoke, INVOKE)
        self.result_cli = self.create_client(SkillResult, GET_RESULT)

    def _on_diag(self, m):
        self.diag = list(m.data)

    def _on_dets(self, m):
        for o in m.objects:
            self.labels[o.label] = max(self.labels.get(o.label, 0.0), float(o.score))
            # ⚠️ "贴着中线"也要**记下来**（'near'）：一个 side=+0.35 的实例
            #    在本工装的 0.5 余量下算"近中线"，但在**规则的 1/3** 下算**右侧**。
            #    不记它，[5] 就可能挑到一个"其实也出现在别处"的类别，
            #    然后因为一个与 `side` 无关的原因失败。
            half = ('left' if o.side <= -SIDE_MARGIN
                    else 'right' if o.side >= SIDE_MARGIN else 'near')
            self.halves.setdefault(o.label, set()).add(half)

    def _on_twist(self, m):
        self.motion_frames += 1
        if abs(m.linear.x) > 1e-9 or abs(m.linear.y) > 1e-9 or abs(m.angular.z) > 1e-9:
            self.nonzero_motion += 1

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---- 经网关问一次，并取终态 ----

    def ask(self, label, min_score, side='', timeout=25.0):
        """经网关派发一次查询并等终态。

        :param side: 非空 ⇒ 走 `semantic.look_on_side`（另一个技能）；
                     空 ⇒ 走 `semantic.look_for`（整幅）。
        """
        args = {'label': label, 'min_score': float(min_score)}
        if side:
            args['side'] = side
        req = SkillInvoke.Request()
        req.principal = PRINCIPAL
        req.skill = SKILL_SIDE if side else SKILL
        req.args_json = json.dumps(args, ensure_ascii=False)
        req.request_id = 'vision-acceptance'
        fut = self.invoke_cli.call_async(req)
        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        r = fut.result() if fut.done() else None
        if r is None or not r.accepted:
            return None, None, (r.message if r else '网关没有回话')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            q = SkillResult.Request()
            q.task_id = r.task_id
            f2 = self.result_cli.call_async(q)
            e2 = time.monotonic() + 5.0
            while not f2.done() and time.monotonic() < e2:
                rclpy.spin_once(self, timeout_sec=0.05)
            rr = f2.result() if f2.done() else None
            if rr is not None and rr.finished:
                return rr.state, rr.result_json, rr.message
            self.spin(0.3)
        return None, None, '等不到终态'


def main():
    rclpy.init()
    rig = Rig()
    results = []
    skipped = []

    def rep(name, ok, detail):
        results.append((name, ok, detail))
        print(f'  {"✅" if ok else "❌"} {name}：{detail}')

    def skip(name, detail):
        """**跳过 ≠ 通过**。用例构造不出来时说清楚，不当成绩。"""
        skipped.append(name)
        print(f'  ⏭ {name}：{detail}')

    try:
        print()
        print('=' * 76)
        print('  视觉链路验收（D-046）—— 只读、不动车')
        print('=' * 76)
        print()
        if not rig.invoke_cli.wait_for_service(timeout_sec=10.0):
            print(f'  ⛔ 拒测：{INVOKE} 不可用 —— Skill Gateway 在跑吗？')
            return 4
        rig.result_cli.wait_for_service(timeout_sec=5.0)
        rig.spin(4.0)                      # 让订阅匹配上，再开始断言（"我没收到"≠"没发生"）

        # ---- 前置：模型与画面 ----
        if rig.diag is None:
            rig.spin(5.0)                  # 再给一会儿：节点可能刚起来
        if rig.diag is None:
            print(f'  ⛔ 拒测：读不到 {DIAG} —— Vision Driver 在跑吗？')
            return 4
        rate, quality, _, ready, age_ms = rig.diag[:5]
        if ready < 0.5:
            print(f'  ⛔ 拒测：模型没就绪（Vision Driver 日志里有原因）')
            return 4
        if age_ms < 0 or age_ms > 500:
            print(f'  ⛔ 拒测：没有新鲜的图像（最近一帧龄 {age_ms:.0f} ms）—— 相机在出图吗？')
            return 4
        print(f'  · 前置：模型就绪｜推理 {rate:.1f} Hz｜清晰度 {quality:.0f}｜帧龄 {age_ms:.0f} ms')
        print()

        # ---- [1]/[2]：拿一个"现在画面里真的有"的类别 ----
        rig.labels.clear()
        rig.spin(2.0)
        if not rig.labels:
            print('  ⛔ 拒测：两秒内**画面里没有任何检出** —— 本工装需要一个"真的看得到"'
                  '的目标才能验"找到了"与"看到了但不够分"这两句。'
                  '把车对着任何一个能检出的东西（人或物）再来。')
            return 4
        label, score = max(rig.labels.items(), key=lambda kv: kv[1])
        print(f'  · 画面里最强的那个类别：{label!r}（score≈{score:.2f}）')
        print()

        st, js, msg = rig.ask(label, 0.0)
        rep('[1] 画面里真有的东西 ⇒ TARGET_FOUND', st == 'TARGET_FOUND',
            f'{st}｜{msg}')

        st, js, msg = rig.ask(label, min(1.0, round(score + 0.05, 3)))
        rep('[2] 门槛抬到够不着 ⇒ TARGET_LOST，**且必须说"看到了、只是低于门槛"**',
            st == 'TARGET_LOST' and '低于你要的' in (msg or ''),
            f'{st}｜{msg}')

        # ---- [3]：模型不认识的类别 ----
        st, js, msg = rig.ask(FOREIGN_LABEL, 0.3)
        rep(f'[3] 模型不认识的类别（{FOREIGN_LABEL}） ⇒ **FAILED**（不知道），绝不能是 TARGET_LOST',
            st == 'FAILED' and '不知道' in (msg or ''),
            f'{st}｜{msg}')

        # ---- [4]：确认没有 ----
        st, js, msg = rig.ask(ABSENT_LABEL, 0.3)
        rep(f'[4] 画面里确认没有（{ABSENT_LABEL}） ⇒ TARGET_LOST，且说清依据',
            st == 'TARGET_LOST' and '帧' in (msg or ''),
            f'{st}｜{msg}')

        # ---- [5]：东西在另一侧时，问这一侧 ----
        # ⚠️ 用例要求那个类别**只出现在一侧、且从不贴近中线**（见 `SIDE_MARGIN`）——
        #    否则测出来的是一个与 `side` 无关的结论，宁可跳过。
        halves = rig.halves.get(label, set())
        if halves not in ({'left'}, {'right'}):
            seen = '、'.join(sorted(halves)) or '（一帧都没记录到）'
            skip('[5] 东西在**另一侧**时问这一侧 ⇒ TARGET_LOST',
                 f'挑中的 {label!r} 出现过的位置是 {seen}，不是"只在一侧"'
                 f'（本项要一个**只在一侧**的目标；把车对着单个东西再来）')
        else:
            here = next(iter(halves))
            other = 'right' if here == 'left' else 'left'
            st, js, msg = rig.ask(label, 0.0, side=other)
            rep(f'[5] {label!r} 在{"左" if here == "left" else "右"}边，'
                f'却问 **{other}** 边 ⇒ TARGET_LOST，且必须点明"哪半幅"',
                st == 'TARGET_LOST' and '半幅' in (msg or ''),
                f'{st}｜{msg}')

        # ---- [6]：side 写成不认识的词 ----
        st, js, msg = rig.ask(label, 0.0, side=BAD_SIDE)
        rep(f'[6] side 写成不认识的 {BAD_SIDE!r} ⇒ **FAILED**（不知道），'
            f'绝不能是 TARGET_LOST',
            st == 'FAILED' and 'side' in (msg or '') and '不知道' in (msg or ''),
            f'{st}｜{msg}')

        # ---- 结构不变量 ----
        rep('★ 全程**没有发出过任何速度指令**（该链路 causes_motion: false）',
            rig.nonzero_motion == 0,
            f'{rig.motion_frames} 帧里非零 {rig.nonzero_motion} 帧')

        print()
        print('=' * 76)
        failed = [n for n, ok, _ in results if not ok]
        if failed:
            print(f'  ❌ {len(failed)}/{len(results)} 项未过：')
            for n in failed:
                print(f'     - {n}')
        else:
            print(f'  ✅ 全部 {len(results)} 项通过')
        if skipped:
            print(f'  ⏭ 另有 {len(skipped)} 项**跳过**（用例没能构造出来，**不算通过**）：')
            for n in skipped:
                print(f'     - {n}')
        print('  ⚠️ 它验的是"**能不能信这个结论**"，不是"模型认得准不准"')
        print('=' * 76)
        return 0 if not failed else 1
    finally:
        rig.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
