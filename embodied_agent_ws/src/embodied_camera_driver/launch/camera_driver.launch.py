"""启动 JetsonRobot Overlay 相机 Driver。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_camera_driver camera_driver.launch.py

前提：厂商相机节点已在跑（bringup 或 peripherals/depth_camera.launch.py），
即 `/depth_cam/rgb0/image_raw` 有数据。

✅ TF 朝向已于 2026-10-06 实机校验通过（无镜像、无滚转），参数不需要修正。
   若日后改动机械安装，按下面的方法重做 —— 不要用 `tf2_echo`：
   它只会把我们自己发布的静态变换原样打印出来，**证明不了任何事**。

   校验方法：把物体放在机器人【前方偏右、低于镜头】处来回晃动，然后
     · 从 `/depth_cam/rgb0/camera_info` 取光心 (cx, cy)；
     · 对 `/embodied/camera/image` 的相邻帧做差分，取变化象素质心 (u, v)；
     · 判据：u > cx → 物体在机器人右侧（x>0）；v > cy → 在下方（y>0）。
   若符号不符，用 tf_roll/tf_pitch/tf_yaw 参数修正，**不要改代码**。
   ⚠️ 该方法只能排除「装反 / 滚转 90°」，**测不了 pitch（俯仰）**。
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_camera_driver'),
                       'config', 'camera_driver.yaml')
    return LaunchDescription([
        Node(
            package='embodied_camera_driver',
            executable='camera_driver',
            name='camera_driver',
            output='screen',
            parameters=[cfg],
        ),
    ])
