"""启动 JetsonRobot Autonomous Skill 层（第一个 task-tier 技能）。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_autonomous_skills autonomous_skills.launch.py

⚠️ **前置条件**：
  · `embodied_lidar_driver`（前方查询）；
  · `embodied_control_skills` + `embodied_motor_driver`（迈步）；
  · 要真的会动，网关必须 `allow_motion:=true`（D-033 第一层闸门）。

手工调用（**车会动** —— 请先确认三层闸门与看护条件）：
    ros2 service call /autonomous_skills/advance_until_blocked \\
      embodied_skills_interfaces/srv/AdvanceUntilBlocked \\
      "{max_distance: 0.3, clear_range: 0.5, step: 0.1}"

取消（会同时停掉正在进行的子动作）：
    ros2 service call /autonomous_skills/cancel std_srvs/srv/Trigger "{}"

⚠️ **本技能未在真机上验证过。** 走通的是判定逻辑与链路（LiDAR 是真的、运动是 dry-run 的）。
⚠️ `clear_range` **必须按部署环境实测标定**（D-028）：同一台车两次摆位，
   近场回波占比在 3.6% 与 67% 之间摆动。
⚠️ 「不知道 ≠ 受阻」：雷达没数据时报 **FAILED**（不是 BLOCKED）——
   那是在说"我不知道前方怎么样"，不是"前方有障碍"。
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_autonomous_skills'),
                       'config', 'autonomous_skills.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'path_clear_service', default_value='/lidar_driver/path_clear',
            description='前方通不通的查询入口（只读）。测试时可指向假话题'),
        Node(
            package='embodied_autonomous_skills',
            executable='autonomous_skills',
            name='autonomous_skills',
            output='screen',
            parameters=[cfg, {
                'path_clear_service': LaunchConfiguration('path_clear_service'),
            }],
        ),
    ])
