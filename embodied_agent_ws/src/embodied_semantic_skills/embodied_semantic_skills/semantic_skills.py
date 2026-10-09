#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Semantic Skill 层（task-tier）—— `look_for` 与 `look_on_side`。

| 模块 | 角色 |
|---|---|
| 本节点 | 调 Vision Driver 问一句，再把答复**翻译成终态**（不做判定） |
| `look_plan.py` | **翻译规则**：`(valid, found)` → 终态（纯函数，有单测） |

两个技能共用**同一条**提问通道（`_ask`）和**同一张**状态映射表
（`look_plan.state_for`）：

| 技能 | 问的是什么 | `TARGET_LOST` 的含义 |
|---|---|---|
| `look_for` | 整幅画面里有没有 <label> | **整幅**都没有 |
| `look_on_side` | **画面某一侧**有没有 <label> | **那一侧**没有（另一侧不算数） |

⚠️ 为什么是两个技能而不是给 `look_for` 加一个可选参数：**"哪一侧"改变了问题的语义** ——
    同上一个参数，`TARGET_LOST` 的**含义**就变了（整幅没有 ↔ 那一侧没有）。
    同一个名字担两种含义正是本项目一贯拒绝的（`LookOnSide.srv` 里 `position`
    刻意不叫 `side`，是同一个理由）。分成两条注册表项，
    LLM 的菜单里也就**各有各的说法**，不必去猜一个参数词表的约定。

🔒 **状态映射**（见 `look_plan.py`）：
    TARGET_FOUND = 看到了；TARGET_LOST = **确认没有**；FAILED = **不知道**。
把"不知道"说成"没有"，上层就会得出"这里没有目标"的结论 —— 而真相可能只是画面糊了。

⚠️ **本节点自己不碰相机、不碰模型**：它只调 `~/find_in_view`。
   于是"什么算可信"的规则**只有一处**（`vision_query.py`）——
   在这里再判一次就会有两份规则，而两份规则**迟早会分家**（D-034 的老教训）。
   ⇒ `side` 的**取值校验**（只认 'left' / 'right'）也**不在**这里做：
     注册表只能查字符串"非空"，说不清枚举；所以它由下游 `vision_query.check_side`
     挡住，并且**以 FAILED（不知道）的形式回到上层** —— 不是静默当成"不限"。

⚠️ 本层技能**只读、不动**：网关注册表里 `causes_motion: false`，
   所以它们**不受** `allow_motion` 闸门约束，也不需要人看护。
"""

import time
from collections import namedtuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from embodied_skill_gateway import task_state as ts
from embodied_skills_interfaces.srv import FindInView, LookFor, LookOnSide

from embodied_semantic_skills import look_plan

#: 一次提问的答复（已翻译成终态）。`position` = 画面里的左右位置（−1…+1）。
Answer = namedtuple('Answer', 'state message score position quality')


class SemanticSkills(Node):

    def __init__(self):
        super().__init__('semantic_skills')

        self.declare_parameter('find_service', '/vision_driver/find_in_view')
        #: 调下游原语的上限（秒）。⚠️ 图像是本地话题、判定是纯计算，
        #: 正常是毫秒级；留 2 s 是给"节点刚起来还在发现服务"这种情况的余量。
        self.declare_parameter('sub_call_timeout', 2.0)

        # ★ **必须用 ReentrantCallbackGroup**：本节点在**服务回调里同步等**下游服务的答复。
        #   默认的回调组是**互斥**的 —— 于是"正在跑的那个回调"会把同一组里的
        #   服务响应回调一起挡住，future 永远不会完成：**死锁到超时**。
        #   症状正好是"下游没有回话"，很容易被读成"下游挂了"。
        #   `autonomous_skills` 早就是这么做的（它也在回调里同步调 Control Skill）。
        self._group = ReentrantCallbackGroup()
        self.cli = self.create_client(FindInView,
                                      str(self.get_parameter('find_service').value),
                                      callback_group=self._group)
        self.create_service(LookFor, '~/look_for', self.on_look_for,
                            callback_group=self._group)
        self.create_service(LookOnSide, '~/look_on_side', self.on_look_on_side,
                            callback_group=self._group)

        self.get_logger().info(
            f'Semantic Skill 启动 | 服务 ~/look_for、~/look_on_side ｜ 看 <- '
            f'{self.get_parameter("find_service").value}')
        self.get_logger().info(
            '🔒 状态映射：看到了=TARGET_FOUND｜**确认没有**=TARGET_LOST｜'
            '**不知道**=FAILED（画面糊/没帧/模型没就绪/类别不在表里/side 写错）'
            '—— "不知道"绝不会被报成"没有"')

    # ---------- 服务 ----------

    def on_look_for(self, req, res):
        """整幅画面里有没有 <label>。"""
        t0 = time.monotonic()
        ans = self._ask(label=str(req.label).strip(),
                        min_score=float(req.min_score))
        res.side = ans.position
        return self._fill(res, ans, t0)

    def on_look_on_side(self, req, res):
        """**画面某一侧**有没有 <label>。

        ⚠️ `side` 的取值在这里**不判**（理由见模块头）：写错时下游会说
        "不认识的 side … —— 不知道（不是"没有"）"，本技能把它原样转成 `FAILED`。
        于是"只看左边"**不会**静默地变成"整个画面都看"。
        """
        t0 = time.monotonic()
        side = str(req.side).strip()
        # ⚠️ **空值不是"不限"，是"没问"**（见 `look_plan.check_asked_side`）：
        #    漏填 side 而静默按整幅回答，会得到一个**看起来完全正常**的答案，
        #    而调用方问的是另一件事。要整幅就调 `look_for`。
        bad = look_plan.check_asked_side(side)
        if bad:
            ans = Answer(ts.FAILED, f'{bad} —— 不知道（不是"没有"）',
                         0.0, 0.0, -1.0)
        else:
            ans = self._ask(label=str(req.label).strip(),
                            min_score=float(req.min_score), side=side)
        res.position = ans.position
        return self._fill(res, ans, t0)

    # ---------- 共用的提问通道 ----------

    def _ask(self, *, label, min_score, side=''):
        """调 Vision Driver 问一句，把答复翻成终态。返回 `Answer`。

        ⚠️ **每一步没跑成都是 `FAILED`（不知道），绝不是 `TARGET_LOST`（没有）**：
        没给类别名 / 视觉服务不在 / 超时没回话 —— 这三件事都**不构成**
        "这里没有那个东西"的证据。
        """
        if not label:
            return Answer(ts.FAILED,
                          '没有给类别名 —— 不知道要查什么（这不算"没有"）',
                          0.0, 0.0, -1.0)

        if not self.cli.service_is_ready() and \
                not self.cli.wait_for_service(timeout_sec=float(
                    self.get_parameter('sub_call_timeout').value)):
            # ★ 视觉得那一跳根本没跑成 ⇒ 不知道。**不是**"没有"。
            msg = ('视觉服务不可用（' +
                   str(self.get_parameter('find_service').value) +
                   '）—— 不知道，不是"没有"。先看 Vision Driver 在不在跑')
            self.get_logger().warn(msg)
            return Answer(ts.FAILED, msg, 0.0, 0.0, -1.0)

        ask = FindInView.Request()
        ask.label = label
        ask.min_score = min_score
        ask.side = side                     # '' = 不限（`look_for` 走的就是这条）
        reply = self._call(ask)

        if reply is None:
            msg = '视觉服务没有回话（超时）—— 不知道，不是"没有"'
            self.get_logger().warn(msg)
            return Answer(ts.FAILED, msg, 0.0, 0.0, -1.0)

        state = look_plan.state_for(valid=bool(reply.valid), found=bool(reply.found))
        # ⚠️ 人话**原样转述**驱动给的那句，不在这里另编一句 ——
        #    另编就会有两份措辞，而两份措辞迟早分家（D-034）。
        #    驱动那边的措辞里已经带了"（左半幅）"这种限定，正是上层要读的。
        return Answer(
            state, str(reply.detail),
            look_plan.bounded_score(reply.score) if reply.found else 0.0,
            float(reply.side) if reply.found else 0.0,
            float(reply.image_quality))

    def _fill(self, res, ans, t0):
        """把 `Answer` 填进响应里两个技能**共有**的那几个字段。

        ⚠️ 共有的只是这几个；`side` / `position` 名字不同（含义不同），
        由各自的处理函数填 —— 这里刻意**不碰**（见 `LookOnSide.srv` 的说明）。
        """
        res.success = look_plan.success_for(ans.state)
        res.state = ans.state
        res.message = ans.message
        res.elapsed = time.monotonic() - t0
        res.score = ans.score
        res.image_quality = ans.quality
        return res

    def _call(self, request):
        """调下游原语并等结果；超时/异常返回 None。

        ⚠️ 等待用**单调钟**（#24 / DEV_NOTES 坑 13：墙上钟会被 NTP 步进）。
        ⚠️ 本节点用 `MultiThreadedExecutor`，所以阻塞在回调里不会把节点卡死
        （与 `autonomous_skills` 同一个做法）。
        """
        timeout = float(self.get_parameter('sub_call_timeout').value)
        future = self.cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'找东西的服务调用异常：{exc!r}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSkills()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
