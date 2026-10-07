"""启动 JetsonRobot 命令路由器（D-006：安全词 / 确定性命令 / 复杂任务三分类）。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_command_router command_router.launch.py

⚠️ **前置条件**：
  · `embodied_skill_gateway` 必须在跑（确定性命令要经它走准入）；
  · 要执行运动，网关与本节点都需 `allow_motion:=true`，且 Motor Driver 需在跑。

喂一条命令（本版只吃文本 —— 语音链路当前是死的，#25）：
    ros2 topic pub --once /embodied/command/text std_msgs/msg/String "{data: '向前走 0.5 米'}"
    ros2 topic pub --once /embodied/command/text std_msgs/msg/String "{data: '左转 90 度'}"
    ros2 topic pub --once /embodied/command/text std_msgs/msg/String "{data: '停下'}"
    ros2 topic pub --once /embodied/command/text std_msgs/msg/String "{data: '去桌子旁找杯子'}"

预期：前两条 → 经网关到 Control Skill；`停下` → Safety Runtime 锁存急停；
      最后一条 → 转给 **Agent Runtime**，由它回答"需要 LLM 规划，Phase 7 未实现"
      （路由器**不替 Agent 判断"能不能做"**）。
      `向前走 0.5`（缺单位）→ 拒绝，**不猜**。

⚠️ **一次只处理一条命令**，忙时**拒绝**而不是排队（与 D-026 决策 2 同一理由）。
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_command_router'),
                       'config', 'command_router.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'text_topic', default_value='/embodied/command/text',
            description='文本输入话题。语音修好后可指向 /asr_node/voice_words'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='三层闸门：是否允许转发可能引起运动的命令。'
                        '⚠️ 打开它不等于车会动 —— 还要看网关与 Motor Driver 的开关'),
        Node(
            package='embodied_command_router',
            executable='command_router',
            name='command_router',
            output='screen',
            parameters=[cfg, {
                'text_topic': LaunchConfiguration('text_topic'),
                'allow_motion': LaunchConfiguration('allow_motion'),
            }],
        ),
    ])
