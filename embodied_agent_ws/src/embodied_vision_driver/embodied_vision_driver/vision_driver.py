#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vision Driver —— 把"画面里有没有某个东西、在哪一侧"变成**一句可回答的话**。

| 模块 | 角色 |
|---|---|
| 本节点 | 取帧 → 质量打分 → 推理 → 转发（**不做判断**） |
| `vision_query.py` | **判定**：能不能回答、怎么回答（纯函数，可离线证伪） |

🔒 **为什么"不能回答"要单独说**（本节点最要紧的一条）
---------------------------------------------------
`vision_query.decide()` 里那条**不对称规则**：找到了可信；**"没找到"只有在
画面质量达标时才敢说**；糊画面 / 没帧 / 模型没就绪 / 类别不在表里 ⇒ **「不知道」**。
理由见 `vision_query.py` 顶部与 `DEV_NOTES` 坑 43（相机失焦 ⇒ 所有检测都是 0 个，
而它和"真没东西"看起来一模一样）。

⚠️ 本节点**不写模型、不训练、也不改厂商任何文件**：模型是从厂商 SDK 里**借**的
（`~/ros2_ws/.../yolo_detect/models/26/yolo26n.engine`，80 类通用 COCO）。
借而不改的理由与 §0.2 的 Overlay 规则一致：厂商目录只读。

⚠️ 为什么**不用**厂商那个 `yolo_node`（虽然它能换 engine 参数）：它有两个
**静默失效点**（DEV_NOTES 坑 44）—— 初始化后要等一个**硬编码的全局服务
`/yolo/start`** 才检测，而类别表是**启动参数**、换模型忘了换它就全是错名字。
自己加载没有这两个坑，话题名与语义也归我们。

依赖：`ultralytics`（厂商环境里已有）。⚠️ 它在**载入模型**时要十几秒
（TensorRT 反序列化），所以模型在**后台线程**里加载：节点**立刻可用**并诚实回答
"模型还没就绪"，而不是卡住不响应、或者假装没有。
"""

import collections
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from embodied_skills_interfaces.msg import VisionDetection, VisionDetections
from embodied_skills_interfaces.srv import FindInView

from embodied_vision_driver import vision_query as vq

#: 厂商 SDK 里那个**通用**模型的默认位置（80 类 COCO，默认没被厂商加载）
DEFAULT_MODEL_DIR = ('/home/ubuntu/ros2_ws/src/example/example/'
                     'yolo_detect/models/26')
DEFAULT_MODEL_FILE = 'yolo26n.engine'


def to_bgr(msg):
    """把图像转成 BGR。转换方式与 `tools/focus_assist.py` / 厂商节点一致（已交叉对照）。"""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'yuv422_yuy2':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 2), cv2.COLOR_YUV2BGR_YUY2)
    if msg.encoding == 'bgr8':
        return buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'不认识的 encoding：{msg.encoding}')


def sharpness(bgr, roi):
    """中心取样区的清晰度（Laplacian 方差）—— 与 `tools/focus_assist.py` 同一口径。

    ⚠️ 它是**相对**指标：画面里本来就没纹理（白墙）时天然很低，
    那是"没东西可聚焦"，不是相机坏了。所以下游用它时要有阈值，而不是要求它很高。
    """
    h, w = bgr.shape[:2]
    rh, rw = int(h * roi), int(w * roi)
    y0, x0 = (h - rh) // 2, (w - rw) // 2
    gray = cv2.cvtColor(bgr[y0:y0 + rh, x0:x0 + rw], cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


class VisionDriver(Node):

    def __init__(self):
        super().__init__('vision_driver')
        g = lambda n: self.get_parameter(n).value        # noqa: E731

        self.declare_parameter('image_topic', '/depth_cam/rgb0/image_raw')
        self.declare_parameter('model_dir', DEFAULT_MODEL_DIR)
        self.declare_parameter('model_file', DEFAULT_MODEL_FILE)
        #: 发到**话题**上的门槛。⚠️ 服务那条路上门槛由**调用方**给
        #: （"多少分算数"是策略，不是本节点能替人拍板的常数）。
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('quality_roi', 0.5)
        #: 清晰度门槛（中心区 Laplacian 方差）。**按本机实测标定**：
        #: 失焦时整帧 ≈ 38、中心 ≈ 40；调好焦后整帧 ≈ 1759、中心 ≈ 3050（2026-10-09）。
        #: 取 100 是保守的**下限**（判"糊"用），不是"好"的标准。
        self.declare_parameter('quality_min', 100.0)
        self.declare_parameter('frame_max_age', 0.5)
        #: ★ 判定用的**窗口**（秒）。否定结论必须基于一段窗口，不能基于一帧 ——
        #:   实测（2026-10-09）：**同一幅静止画面**里 `suitcase` 只出现在 **68%** 的帧，
        #:   拿一帧的"没找到"当结论 ⇒ 东西明明在眼前却**三次里有一次说"没有"**。
        #: ⚠️ **按真实推理帧率定，不是按标称帧率定**：2026-10-09 实测 ——
        #:   厂商那个 yolo 也在抢 GPU 时，我们单次推理均值 **179 ms**（最快 17 ms，
        #:   抖动极大）⇒ 只有 **~5.6 Hz**；0.5 s 的窗口里就只剩 ~2.8 帧，
        #:   而 `min_frames=3` ⇒ **否定的判定会时不时变成"不知道"**。
        #:   （那是**正确**的行为，但会让技能时灵时不灵。）⇒ 窗口放到 1.0 s。
        #:   ⚠️ 这个 engine 是**固定形状 640×640** 的，`imgsz` 调小会直接报错 ——
        #:   想靠降分辨率提速，得重新导出模型。
        self.declare_parameter('confirm_window', 1.0)
        #: 窗口里至少要有几帧才够确认"没有"。太少 ⇒ 不知道（不是"没有"）。
        self.declare_parameter('min_frames', 3)
        self.declare_parameter('detections_topic', '/embodied/vision/detections')
        self.declare_parameter('diag_topic', '/embodied/vision/diag')

        self._lock = threading.Lock()
        self._image = None            # 最新一帧（BGR）——队列长度 1，旧的直接丢
        self._image_at = None         # ★ 收到它的**本机时刻**（不是图像 header 的时间戳）
        self._frames = 0
        self._drops = 0
        self._model = None
        self._ready = False
        self._load_error = ''
        self._infer_rate = 0.0
        self._last_result = None      # (detections, quality, 收到该帧的时刻)
        #: 最近一段时间的推理结果 (t, detections, quality) —— 服务判定要用**窗口**
        self._history = collections.deque(maxlen=200)
        self._warned_encoding = False

        self.pub = self.create_publisher(VisionDetections, g('detections_topic'), 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, g('diag_topic'), 10)
        self.create_subscription(Image, g('image_topic'), self._on_image, 1)
        self.create_service(FindInView, '~/find_in_view', self._on_find)

        threading.Thread(target=self._load_model, daemon=True).start()
        threading.Thread(target=self._work, daemon=True).start()
        self.create_timer(1.0, self._publish_diag)

        self.get_logger().info(
            f'Vision Driver 启动 | {g("image_topic")} → {g("detections_topic")}｜'
            f'服务 ~/find_in_view｜模型 {g("model_dir")}/{g("model_file")}')
        self.get_logger().warn(
            '⚠️ 类别表来自**模型自带**：问一个它不认识的类别，本节点会回答'
            '「不知道」而**不是**「没有」（否则会永远说"没有那个东西"，且不报错）')

    # ---------- 模型 ----------

    def _load_model(self):
        """在**后台**加载模型。失败也要留下来 —— 上层要能区分"没就绪"和"不会"。"""
        import os
        path = os.path.join(str(self.get_parameter('model_dir').value),
                            str(self.get_parameter('model_file').value))
        try:
            from ultralytics import YOLO
            self._model = YOLO(path, task='detect', verbose=False)
            self._ready = True
            names = list(self._model.names.values())
            self.get_logger().info(
                f'模型就绪：{path}｜{len(names)} 个类别（如 {names[:6]} …）')
        except Exception as exc:                     # noqa: BLE001
            self._load_error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(
                f'模型加载失败（{path}）：{self._load_error} —— '
                f'本节点会一直回答「不知道」，**不会**回答「没有」')

    # ---------- 取帧 ----------

    def _on_image(self, msg):
        try:
            bgr = to_bgr(msg)
        except ValueError as exc:
            if not self._warned_encoding:
                self._warned_encoding = True
                self.get_logger().warn(f'跳过这些帧：{exc}（只报一次）')
            return
        with self._lock:
            if self._image is not None:
                self._drops += 1                     # 推理跟不上就丢旧的，绝不排队
            self._image = bgr
            self._image_at = time.monotonic()
            self._frames += 1

    # ---------- 推理 ----------

    def _work(self):
        last_t = None
        while rclpy.ok():
            if not self._ready:
                time.sleep(0.1)
                continue
            with self._lock:
                bgr, at = self._image, self._image_at
                self._image = None                       # 取走这一帧
            if bgr is None:
                time.sleep(0.01)
                continue
            try:
                quality = sharpness(bgr, float(self.get_parameter('quality_roi').value))
                conf = float(self.get_parameter('confidence').value)
                res = self._model(bgr, verbose=False, conf=conf)[0]
                h, w = bgr.shape[:2]
                dets = []
                for b in res.boxes:
                    x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                    dets.append(VisionDetection(
                        label=str(self._model.names[int(b.cls[0])]),
                        score=float(b.conf[0]),
                        x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2),
                        side=float(vq.side_of((x1 + x2) / 2.0, w))))
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().error(f'推理失败：{type(exc).__name__}: {exc}')
                time.sleep(0.2)
                continue
            now = time.monotonic()
            if last_t is not None and now > last_t:
                # 指数平滑：单帧的抖动不该让人以为帧率在跳
                self._infer_rate = 0.7 * self._infer_rate + 0.3 / (now - last_t)
            last_t = now
            with self._lock:
                self._last_result = (dets, quality, at, now)
                self._history.append((now, dets, quality))
                cutoff = now - float(self.get_parameter('confirm_window').value) - 1.0
                while self._history and self._history[0][0] < cutoff:
                    self._history.popleft()
            self._publish(dets, quality)

    def _publish(self, dets, quality):
        m = VisionDetections()
        # ⚠️ 用**本节点收到这一帧的时刻**打戳，不用图像 `header.stamp`
        #    （#23 / #24：厂商图像戳陈旧且陈旧量不是常数，实测过一次 338.6 秒）
        m.stamp = self.get_clock().now().to_msg()
        m.image_quality = float(quality)
        m.objects = dets
        self.pub.publish(m)

    def _publish_diag(self):
        with self._lock:
            age = None if self._image_at is None else time.monotonic() - self._image_at
            n = len(self._last_result[0]) if self._last_result else 0
            quality = float(self._last_result[1]) if self._last_result else -1.0
            frames, drops = self._frames, self._drops
        m = Float64MultiArray()
        # 字段顺序**定死在这里**（与 LiDAR/相机 Driver 同一条纪律：下标是接口）
        m.data = [float(self._infer_rate), quality, float(n),
                  1.0 if self._ready else 0.0,
                  -1.0 if age is None else age * 1000.0,
                  float(frames), float(drops)]
        self.diag_pub.publish(m)

    # ---------- 查询 ----------

    def _on_find(self, req, res):
        now = time.monotonic()
        with self._lock:
            age = None if self._image_at is None else now - self._image_at
            # ★ 窗口 = 最近 `confirm_window` 秒里**每一次推理的结果**（不是某一帧）
            span = float(self.get_parameter('confirm_window').value)
            recent = [h for h in self._history if now - h[0] <= span]
        best, qmin, below = None, None, None
        for _t, dets_i, q_i in recent:
            b = vq.pick_best(dets_i, req.label, req.min_score)
            if b is not None and (best is None or b.score > best.score):
                best = b
            # ⚠️ 同时记住"看到了但不够分"的那个：低于门槛也不能假装没看见
            b_any = vq.pick_best(dets_i, req.label, 0.0)
            if b_any is not None and b_any.score < req.min_score \
                    and (below is None or b_any.score > below.score):
                below = b_any
            qmin = q_i if qmin is None else min(qmin, q_i)
        window = vq.Window(frames=len(recent), best=best,
                           quality_min=-1.0 if qmin is None else float(qmin),
                           best_below=below)
        quality = float(qmin) if qmin is not None else -1.0

        known = list(self._model.names.values()) if self._ready else []
        hint = ('能查的是：' + ', '.join(known[:12]) + ' …') if known else ''

        if not self._ready and self._load_error:
            res.valid, res.found = False, False
            res.detail = f'模型加载失败：{self._load_error} —— 不知道'
            res.image_quality = quality
            return res

        d = vq.decide(
            model_ready=self._ready,
            label_known=vq.label_is_known(req.label, known),
            frame_age_s=age,
            frame_max_age_s=float(self.get_parameter('frame_max_age').value),
            window=window,
            quality_min=float(self.get_parameter('quality_min').value),
            min_frames=int(self.get_parameter('min_frames').value),
            min_score=float(req.min_score),
            label=req.label, known_hint=hint)

        res.valid, res.found, res.detail = d.valid, d.found, d.detail
        res.image_quality = quality
        if d.found:
            b = window.best
            res.score = float(b.score)
            res.x1, res.y1, res.x2, res.y2 = int(b.x1), int(b.y1), int(b.x2), int(b.y2)
            res.side = float(b.side)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = VisionDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
