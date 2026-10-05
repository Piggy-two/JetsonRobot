"""启动 JetsonRobot Overlay 相机 Driver。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_camera_driver camera_driver.launch.py

前提：厂商相机节点已在跑（bringup 或 peripherals/depth_camera.launch.py），
即 `/depth_cam/rgb0/image_raw` 有数据。

⚠️ 首次使用必须实机校验一次 TF 朝向（URDF 表达不了这个，只能看）：
    ros2 run tf2_ros tf2_echo camera_link0 camera
  然后目视判断：
    在相机视野里放一个"偏右下"的物体，
    其坐标应满足 x>0（前方）、y>0（偏右）、z>0（偏下）。
  若符号不符，用 tf_roll/tf_pitch/tf_yaw 参数修正，不要改代码。
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
