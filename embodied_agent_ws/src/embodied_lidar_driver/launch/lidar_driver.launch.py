"""启动 JetsonRobot Overlay LiDAR Primitive。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_lidar_driver lidar_driver.launch.py

⚠️ **前置条件：厂商雷达节点已在跑**（`/scan` 有数据）。默认 `LIDAR_TYPE=LD19`
   时由厂商 `bringup` 的 `ldlidar_LD19.launch.py` 提供。

查询原语：

    # 正前方 ±30°、3 m 内最近的回波
    ros2 service call /lidar_driver/sector_min_range \
        embodied_skills_interfaces/srv/SectorMinRange \
        "{center: 0.0, width: 1.0472, max_range: 3.0}"

    # 正前方 ±30°、1 m 内是否通畅
    ros2 service call /lidar_driver/path_clear \
        embodied_skills_interfaces/srv/PathClear \
        "{width: 1.0472, clear_range: 1.0}"

    # 持续信号（每帧扫描更新一次，约 10 Hz）
    #   data = [最近距离m, 该回波角度rad, 是否有效 1/0, 扇区内有效点数]
    #   无有效回波时 距离 = -1
    ros2 topic echo /embodied/lidar/front

⚠️ 两条语义边界，别误读：
  ① `clear=true` 只说明**这个扇形、这个距离内没有回波**，不保证路径整体安全
     （旁边可能有东西、地上可能有坑、回波可能被吸音材质吃掉）。
  ② **没有新鲜扫描时 `path_clear` 报 not-clear** —— 不知道不等于安全。
     可通过 `ros2 topic hz /scan` 与 `scan_timeout` 判断是哪种情况。
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_lidar_driver'),
                       'config', 'lidar_driver.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='厂商雷达话题。测试时可指向假话题以喂合成扫描'),
        Node(
            package='embodied_lidar_driver',
            executable='lidar_driver',
            name='lidar_driver',
            output='screen',
            parameters=[cfg, {'scan_topic': LaunchConfiguration('scan_topic')}],
        ),
    ])
