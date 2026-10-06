#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay 相机 Driver（架构分层：Driver 层）。

把厂商 usb_cam 的相机源收口成本项目可用的形式，只做两件确定性的事：

  1. 重新打时间戳
     厂商 `/depth_cam/rgb0/image_raw` 的 header.stamp 比「真实采集时刻」早约
     0.72 s（实测数据见 docs/DECISIONS.md D-022 / 已知问题 #23）。图像内容本身
     是新鲜的（端到端延迟仅约一帧），陈旧的只是时间戳。
     任何把图像与 LiDAR/IMU/里程计做时间对齐的融合，都会因此错位 0.72 s。

  2. 补发 TF 帧 camera_link0 -> camera
     厂商图像的 frame_id 是 `camera`，但 URDF 里只定义了 `camera_link0`，
     `camera` 不在 TF 树（已知问题 #16）。任何 tf2 查询都会失败。

设计取舍（为什么这么做，详见 docs/DEV_NOTES.md）：
  · 时间戳给出三档 `stamp_source`，默认 `corrected`：
      - original  ：原样透传（用于对比/复现问题）
      - receipt   ：用本节点收到该帧的时刻（误差 = 传输延迟，几十 ms，恒为正）
      - corrected ：receipt - pipeline_latency（默认一帧，最接近真实曝光时刻）
    我们**不发明**无法验证的修正量：pipeline_latency 是参数，且诊断话题会同时
    报出「原始时间戳的陈旧量」，便于日后实测校准。
  · TF 的旋转默认取 REP-103 光学坐标系（z 前 / x 右 / y 下）。
    ⚠️ 厂商自己发布过的四元数是 x=上/y=右/z=前，**不是** REP-103，故未沿用。
    `camera_link0` 的实际朝向由机械安装决定，URDF 的 rpy=0 表达不了，
    **必须实机目视确认一次**（见 launch 文件注释里的校验方法）。
  · 订阅用 sensor_data QoS（BEST_EFFORT）：与厂商发布端兼容，且低延迟。
    队列只留最新帧（depth=1），避免积压把延迟算进时间戳。

本节点不含任何语义/规划逻辑，LLM 不得介入（CLAUDE.md §0）。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import StaticTransformBroadcaster


def rpy_to_quaternion(roll, pitch, yaw):
    """ZYX 内旋 RPY -> 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/depth_cam/rgb0/image_raw')
        self.declare_parameter('output_topic', '/embodied/camera/image')
        self.declare_parameter('diag_topic', '/embodied/camera/diag')
        self.declare_parameter('output_frame', 'camera')
        self.declare_parameter('parent_frame', 'camera_link0')
        # original | receipt | corrected
        self.declare_parameter('stamp_source', 'corrected')
        # 秒；2026-10-06 实测标定值（标定方法见 config/camera_driver.yaml）。
        # ⚠️ 随负载变化，不是物理常数：同一条管线 2026-10-05 实测约 30 ms。
        self.declare_parameter('pipeline_latency', 0.110)
        self.declare_parameter('publish_static_tf', True)
        # REP-103 光学：z 前 / x 右 / y 下 -> rpy = (-pi/2, 0, -pi/2)
        self.declare_parameter('tf_roll', -math.pi / 2.0)
        self.declare_parameter('tf_pitch', 0.0)
        self.declare_parameter('tf_yaw', -math.pi / 2.0)
        self.declare_parameter('diag_period', 5.0)

        self.stamp_source = self.get_parameter('stamp_source').value
        if self.stamp_source not in ('original', 'receipt', 'corrected'):
            self.get_logger().warn(
                f"stamp_source='{self.stamp_source}' 不合法，回退为 corrected")
            self.stamp_source = 'corrected'
        self.latency = float(self.get_parameter('pipeline_latency').value)
        self.out_frame = self.get_parameter('output_frame').value
        self.in_frame = None

        # ---- 发布者 ----
        out_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(
            Image, self.get_parameter('output_topic').value, out_qos)
        self.diag_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('diag_topic').value, 10)

        # ---- 静态 TF ----
        if self.get_parameter('publish_static_tf').value:
            self._publish_static_tf()

        # ---- 订阅者（队列只留最新帧，避免积压计入延迟）----
        in_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self.on_image, in_qos)

        # ---- 诊断统计 ----
        self.n = 0
        self.n_first = 0
        self.t_first_recv = None
        self.sum_staleness = 0.0    # 原始时间戳相对"收到时刻"的陈旧量
        self.sum_latency_used = 0.0
        self.max_staleness = 0.0
        period = float(self.get_parameter('diag_period').value)
        self.create_timer(period, self.report)

        self.get_logger().info(
            f"相机 Driver 启动 | {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} | "
            f"stamp_source={self.stamp_source} latency={self.latency * 1000:.0f}ms | "
            f"TF {self.get_parameter('parent_frame').value} -> {self.out_frame}")

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent_frame').value
        t.child_frame_id = self.out_frame
        qx, qy, qz, qw = rpy_to_quaternion(
            float(self.get_parameter('tf_roll').value),
            float(self.get_parameter('tf_pitch').value),
            float(self.get_parameter('tf_yaw').value))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(t)
        self.get_logger().info(
            f"静态 TF: {t.header.frame_id} -> {t.child_frame_id} "
            f"rpy=({float(self.get_parameter('tf_roll').value):.4f}, "
            f"{float(self.get_parameter('tf_pitch').value):.4f}, "
            f"{float(self.get_parameter('tf_yaw').value):.4f})")

    def _restamp(self, msg):
        """按 stamp_source 计算输出时间戳，返回 (stamp, staleness, used_latency)。"""
        now = self.get_clock().now()
        orig = msg.header.stamp
        staleness = (now - Duration(seconds=orig.sec, nanoseconds=orig.nanosec))
        staleness = staleness.nanoseconds * 1e-9   # 正数 = 原始戳比"现在"旧

        if self.stamp_source == 'original':
            return orig, staleness, 0.0
        if self.stamp_source == 'receipt':
            return now.to_msg(), staleness, 0.0
        # corrected
        corrected = now - Duration(seconds=self.latency)
        return corrected.to_msg(), staleness, self.latency

    def on_image(self, msg):
        stamp, staleness, used = self._restamp(msg)

        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = self.out_frame
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self.pub.publish(out)

        # 统计
        if self.in_frame is None:
            self.in_frame = msg.header.frame_id
            self.get_logger().info(f"输入 frame_id='{msg.header.frame_id}'")
        self.n += 1
        if self.t_first_recv is None:
            self.t_first_recv = self.get_clock().now()
            self.n_first = 0
        self.sum_staleness += staleness
        self.sum_latency_used += used
        self.max_staleness = max(self.max_staleness, staleness)

    def report(self):
        if self.n == 0:
            self.get_logger().warn(
                f"未收到任何图像：{self.get_parameter('input_topic').value} 无数据。"
                "确认厂商相机节点已启动（bringup 或 depth_camera.launch.py）。")
            return
        now = self.get_clock().now()
        dt = (now - self.t_first_recv).nanoseconds * 1e-9
        rate = (self.n - self.n_first) / dt if dt > 0 else 0.0
        mean_stale = self.sum_staleness / self.n * 1000.0
        mean_used = self.sum_latency_used / self.n * 1000.0

        m = Float64MultiArray()
        m.data = [float(self.n), rate, mean_stale, mean_used,
                  self.max_staleness * 1000.0]
        self.diag_pub.publish(m)

        # data = [帧数, 实测帧率Hz, 原始戳平均陈旧量ms, 平均修正量ms, 最大陈旧量ms]
        self.get_logger().info(
            f"帧 {self.n} | {rate:.1f} Hz | 原始戳陈旧 均值 {mean_stale:.0f} ms "
            f"最大 {self.max_staleness * 1000:.0f} ms | 施加修正 {mean_used:.0f} ms "
            f"| stamp_source={self.stamp_source}")
        if self.stamp_source == 'original' and mean_stale > 200.0:
            self.get_logger().warn(
                'stamp_source=original：正在透传陈旧时间戳（约 '
                f'{mean_stale:.0f} ms），下游时间对齐会错位。')


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
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
