"""启动 JetsonRobot Skill 网关（D-005 的落点）。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_skill_gateway skill_gateway.launch.py

⚠️ **前置条件**：网关照常启动，但**它自己不带任何技能** —— 调用的技能服务
   （`/control_skills/*`、`/lidar_driver/*`）必须由各自的节点提供，否则
   受理后会停在"服务不可用"并判失败。

看有什么技能可用（只读投影，不含底层 ROS 服务名）：
    ros2 service call /skill_gateway/list embodied_skills_interfaces/srv/SkillList "{}"

手工调用一次（走完整准入路径）：
    ros2 service call /skill_gateway/invoke embodied_skills_interfaces/srv/SkillInvoke \\
      "{principal: 'router.deterministic', skill: 'control.move_relative', \\
        args_json: '{\\"x\\": 0.5, \\"y\\": 0.0}', request_id: 'manual'}"

⚠️ **默认 `allow_motion=false`**：上面这条会被**拒绝**（控制类技能需要显式打开）。
   这是刻意的 —— 见 D-033 的三层闸门。要真跑通请加 `allow_motion:=true`。

⚠️ 受理是**异步**的：`~/invoke` 立刻返回 `task_id`，结果要看
   `/embodied/skill/events` 或调 `~/get_result`。这不是实现偷懒，
   而是 D-030 定的形态 —— 阻塞等待会让调用方变成不可取消、不可超时的单线程。
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_skill_gateway'),
                       'config', 'skill_gateway.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='三层闸门①：是否允许派发可能引起运动的技能（control.*）。'
                        '⚠️ 打开它不等于车会动 —— 还要看 Motor Driver 的 dry_run'),
        DeclareLaunchArgument(
            'registry_file', default_value='',
            description='注册表 YAML 路径。留空 = 包 share 目录下的 config/skill_registry.yaml'),
        Node(
            package='embodied_skill_gateway',
            executable='skill_gateway',
            name='skill_gateway',
            output='screen',
            parameters=[cfg, {
                'allow_motion': LaunchConfiguration('allow_motion'),
                'registry_file': LaunchConfiguration('registry_file'),
            }],
        ),
    ])
