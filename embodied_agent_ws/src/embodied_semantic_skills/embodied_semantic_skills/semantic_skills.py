#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Semantic Skill 层（task-tier）—— 第一个技能 `look_for`。

| 模块 | 角色 |
|---|---|
| 本节点 | 调 Vision Driver 问一句，再把答复**翻译成终态**（不做判定） |
| `look_plan.py` | **翻译规则**：`(valid, found)` → 终态（纯函数，有单测） |

🔒 **本技能的全部价值在状态映射**（见 `look_plan.py`）：
    TARGET_FOUND = 看到了；TARGET_LOST = **确认没有**；FAILED = **不知道**。
把"不知道"说成"没有"，上层就会得出"这里没有目标"的结论 —— 而真相可能只是画面糊了。

⚠️ **本节点自己不碰相机、不碰模型**：它只调 `~/find_in_view`。
   于是"什么算可信"的规则**只有一处**（`vision_query.py`）——
   在这里再判一次就会有两份规则，而两份规则**迟早会分家**（D-034 的老教训）。

⚠️ 本技能**只读、不动**：网关注册表里 `causes_motion: false`，
   所以它**不受** `allow_motion` 闸门约束，也不需要人看护。
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from embodied_skill_gateway import task_state as ts
from embodied_skills_interfaces.srv import FindInView, LookFor

from embodied_semantic_skills import look_plan


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

        self.get_logger().info(
            f'Semantic Skill 启动 | 服务 ~/look_for ｜ 看 <- '
            f'{self.get_parameter("find_service").value}')
        self.get_logger().info(
            '🔒 状态映射：看到了=TARGET_FOUND｜**确认没有**=TARGET_LOST｜'
            '**不知道**=FAILED（画面糊/没帧/模型没就绪/类别不在表里）'
            '—— "不知道"绝不会被报成"没有"')

    # ---------- 查询 ----------

    def on_look_for(self, req, res):
        t0 = time.monotonic()
        label = str(req.label).strip()

        if not label:
            res.success = False
            res.state = ts.FAILED
            res.message = '没有给类别名 —— 不知道要查什么（这不算"没有"）'
            res.elapsed = time.monotonic() - t0
            res.image_quality = -1.0
            return res

        if not self.cli.service_is_ready() and \
                not self.cli.wait_for_service(timeout_sec=float(
                    self.get_parameter('sub_call_timeout').value)):
            # ★ 视觉得那一跳根本没跑成 ⇒ 不知道。**不是**"没有"。
            res.success = False
            res.state = ts.FAILED
            res.message = ('视觉服务不可用（' +
                           str(self.get_parameter('find_service').value) +
                           '）—— 不知道，不是"没有"。先看 Vision Driver 在不在跑')
            res.elapsed = time.monotonic() - t0
            res.image_quality = -1.0
            self.get_logger().warn(res.message)
            return res

        ask = FindInView.Request()
        ask.label = label
        ask.min_score = float(req.min_score)
        reply = self._call(ask)

        if reply is None:
            res.success = False
            res.state = ts.FAILED
            res.message = '视觉服务没有回话（超时）—— 不知道，不是"没有"'
            res.elapsed = time.monotonic() - t0
            res.image_quality = -1.0
            self.get_logger().warn(res.message)
            return res

        state = look_plan.state_for(valid=bool(reply.valid), found=bool(reply.found))
        res.success = look_plan.success_for(state)
        res.state = state
        # ⚠️ 人话**原样转述**驱动给的那句，不在这里另编一句 ——
        #    另编就会有两份措辞，而两份措辞迟早分家（D-034）。
        res.message = str(reply.detail)
        res.elapsed = time.monotonic() - t0
        res.image_quality = float(reply.image_quality)
        res.score = look_plan.bounded_score(reply.score) if reply.found else 0.0
        res.side = float(reply.side) if reply.found else 0.0
        self.get_logger().info(f'look_for({label}) → {state}｜{res.message}')
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
