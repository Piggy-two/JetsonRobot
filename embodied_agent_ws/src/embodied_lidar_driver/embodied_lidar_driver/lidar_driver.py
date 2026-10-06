#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay LiDAR Driver / Primitive 层。

厂商的 `/scan` 本身是**对的**（实测 10.00 Hz、360°、`frame_id=lidar_frame`、TF 已就位，
见 PROJECT_STATUS §7），所以本节点**不做"收口"** —— 它不做时间戳重打、不改 TF。
它做的事是**在原始扫描之上提供可被安全层与 Skill 调用的查询原语**：

```text
/scan ──→[本节点]──→ /embodied/lidar/front      （前方扇区最近距离，每帧更新）
                 └──→ ~/sector_min_range        （任意扇区的最近回波）
                 └──→ ~/path_clear              （某方向是否通畅）
```

**为什么不只是"转发一下"**：上层真正要问的不是"这一圈点云长什么样"，
而是「**我这个方向、这个距离内，有没有东西**」。把它做成原语，
调用方才不必各自去下标切片 —— 而**扇区跨 0/2π 接缝**那个坑（见 `scan_query.py`）
正是各自实现时最容易静默写错的地方。

⚠️ **数据陈旧时绝不报"通畅"。** 本节点把两种情况分得很开：

    · 扫描**新鲜**、扇区内没有回波  → `clear=true, range=-1`（**假设**空旷，已如实标注）
    · 扫描**陈旧/还没来**          → `clear=false, range=-1`（**不知道 ≠ 安全**）

    把后者报成 true，就是在"传感器挂了"的时候告诉上层"前面没东西"——
    这是本文件里最危险的一个错误，所以单列出来。

⚠️ 本节点**不**发任何控制指令，也不做避障决策（那属 Safety Runtime / Autonomous Skill）。
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from embodied_lidar_driver.scan_query import is_path_clear, sector_min_range
from embodied_skills_interfaces.srv import PathClear, SectorMinRange


class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_driver')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_topic', '/embodied/lidar/front')
        # 前方扇区：±30°，只看 3 m 以内（再远对近距避障没有意义）
        self.declare_parameter('front_width', 1.0471975511965976)
        self.declare_parameter('front_max_range', 3.0)
        # 多久没收到扫描即视为"没有可用数据"
        self.declare_parameter('scan_timeout', 1.0)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self._scan = None
        self._scan_time = None

        self.front_pub = self.create_publisher(Float64MultiArray, g('front_topic'), 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.create_service(SectorMinRange, '~/sector_min_range', self.on_sector)
        self.create_service(PathClear, '~/path_clear', self.on_path_clear)

        self.get_logger().info(
            f'LiDAR Driver 启动 | {g("scan_topic")} -> {g("front_topic")} '
            f'（前方 ±{g("front_width") / 2 * 180 / 3.141592653589793:.0f}°，'
            f'≤{g("front_max_range")} m）| 服务 ~/sector_min_range、~/path_clear')

    # ---------- 输入 ----------

    def on_scan(self, msg):
        self._scan = msg
        self._scan_time = time.monotonic()
        m = Float64MultiArray()
        valid, r, a, n = sector_min_range(
            msg.ranges, msg.angle_min, msg.angle_increment,
            center=0.0,
            width=float(self.get_parameter('front_width').value),
            max_range=float(self.get_parameter('front_max_range').value))
        m.data = [r if valid else -1.0, a, 1.0 if valid else 0.0, float(n)]
        self.front_pub.publish(m)

    # ---------- 数据新鲜度 ----------

    def _fresh_scan(self):
        """返回新鲜的扫描；**没有新鲜数据时返回 None**（调用方必须据此保守回答）。"""
        if self._scan is None:
            return None
        timeout = float(self.get_parameter('scan_timeout').value)
        if time.monotonic() - self._scan_time > timeout:
            return None
        return self._scan

    # ---------- 服务 ----------

    def on_sector(self, req, res):
        s = self._fresh_scan()
        if s is None:
            res.valid = False
            res.range = -1.0
            res.angle = 0.0
            res.points = 0
            self.get_logger().warn('sector_min_range：没有新鲜扫描 —— 返回 invalid')
            return res
        valid, r, a, n = sector_min_range(
            s.ranges, s.angle_min, s.angle_increment,
            req.center, req.width, req.max_range)
        res.valid, res.range, res.angle, res.points = valid, r, a, n
        return res

    def on_path_clear(self, req, res):
        s = self._fresh_scan()
        if s is None:
            # ⚠️ **不知道 ≠ 安全**：没有数据时绝不能报 clear=true。
            res.clear = False
            res.range = -1.0
            res.angle = 0.0
            self.get_logger().warn('path_clear：没有新鲜扫描 —— 报 not-clear（不知道 ≠ 安全）')
            return res
        clear, r, a = is_path_clear(
            s.ranges, s.angle_min, s.angle_increment,
            center=0.0, width=req.width, clear_range=req.clear_range)
        res.clear, res.range, res.angle = clear, r, a
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriver()
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
