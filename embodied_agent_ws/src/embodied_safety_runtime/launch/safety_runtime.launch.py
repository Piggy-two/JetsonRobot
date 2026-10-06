"""启动 JetsonRobot Overlay Safety Runtime（第一版）。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_safety_runtime safety_runtime.launch.py

它做什么：
  ① **本地安全指令通路**（D-006 / D-024）：订阅厂商离线 ASR 的文本，
     **在本节点内**匹配安全词 → 立即急停。**不经过 LLM，不经过厂商 voice_control_* 节点。**
  ② **独立的零速通道**：锁存期间自己直接向 `/cmd_vel` @10 Hz 发 `Twist()`（全零）。
     ⚠️ 这一条**不依赖 Motor Driver 存活** —— 因为"持续发零"目前只在 Motor Driver 里，
     它自己挂了就没人发零了，而底盘**没有指令超时保护**（D-020）。

手工触发 / 解除（验收与测试用）：
    ros2 service call /safety_runtime/estop   std_srvs/srv/Trigger "{}"
    ros2 service call /safety_runtime/release std_srvs/srv/Trigger "{}"
    # 也可以直接喂一条文本，走真实的本地解析路径：
    ros2 topic pub --once /asr_node/voice_words std_msgs/msg/String "{data: 'stop'}"

看状态与事件：
    ros2 topic echo /embodied/safety/status   # [是否锁存, 已发零帧数, 触发次数]
    ros2 topic echo /embodied/safety/events   # estop_triggered:<原因> / estop_released

⚠️ 急停是**锁存**的：触发后必须显式 `~/release` 才能解除 —— 解除这个动作本身
   必须是一次明确的、有人负责的决定。
⚠️ 解除本节点的锁存**不会**自动解除 Motor Driver / Control Skill 各自的锁存，
   它们各有自己的 `~/resume`。三道锁是**独立**的，这是刻意的。

本版**不含**：避障、速度/区域限制、Agent 侧 Skill 网关、对 Motor Driver 存活性的监视
（"它挂了就自动接管"）。
"""  # noqa: D205
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_safety_runtime'),
                       'config', 'safety_runtime.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'voice_topic', default_value='/asr_node/voice_words',
            description='厂商离线 ASR 的文本输出。测试时可指向假话题以喂合成文本'),
        Node(
            package='embodied_safety_runtime',
            executable='safety_runtime',
            name='safety_runtime',
            output='screen',
            parameters=[cfg, {'voice_topic': LaunchConfiguration('voice_topic')}],
        ),
    ])
