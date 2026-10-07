"""启动 JetsonRobot Agent Runtime（第一批骨架）。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_agent_runtime agent_runtime.launch.py

⚠️ **前置条件**：`embodied_skill_gateway` 必须在跑（本节点经它派发，不直接调技能）。

提交一个自然语言任务（**默认会被拒绝** —— 见下）：
    ros2 service call /agent_runtime/submit embodied_skills_interfaces/srv/AgentTask \\
      "{text: '去桌子旁边找杯子', principal: 'router.agent_task', request_id: ''}"

预期回答：`accepted=false`，原因是「需要 LLM 规划，当前未实现（Phase 7）」。
**这是刻意的默认值** —— 规则表默认为空（`rules_file` 为空）。

要跑通链路，用一个规则表**文件**（格式见 `config/rules_example.yaml`）：

    ros2 launch embodied_agent_runtime agent_runtime.launch.py \\
      allow_motion:=true \\
      rules_file:=$(ros2 pkg prefix embodied_agent_runtime)/share/embodied_agent_runtime/config/rules_example.yaml

⚠️ **为什么是文件而不是命令行里的 JSON**：`ros2 launch ... rules_json:='{...}'`
   会被 launch 当成 **YAML** 解析成 dict，参数系统直接拒收
   （`Allowed value types are ... Got <class 'dict'>`）。规则表本来就是数据。

⚠️ 打开 `allow_motion` **不等于**车会动：还要网关也 `allow_motion:=true`，
   且 Motor Driver 不处于 `dry_run`（D-033 的三层闸门）。

看状态（`[在途任务数, 仍在等, 被唤醒次数, 被丢弃的历史条数]`）：
    ros2 topic echo /embodied/agent/status
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_agent_runtime'),
                       'config', 'agent_runtime.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'rules_file', default_value='',
            description='规则表 YAML 的路径。⚠️ 留空 ⇒ 一切都拒绝。'
                        '格式见 config/rules_example.yaml。'
                        '🔒 只能写 task-tier 技能，写 control.* 会被拒（D-003）'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='三层闸门：是否允许派发可能引起运动的技能。'
                        '⚠️ 打开它不等于车会动 —— 还要看网关与 Motor Driver'),
        Node(
            package='embodied_agent_runtime',
            executable='agent_runtime',
            name='agent_runtime',
            output='screen',
            parameters=[cfg, {
                'rules_file': LaunchConfiguration('rules_file'),
                'allow_motion': LaunchConfiguration('allow_motion'),
            }],
        ),
    ])
