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
  ③ **对 Motor Driver 的停更看门狗**：它自己挂了既不报错也没人转告，
     而本节点不可能"等别人告诉我该停"。所以自己盯着 `/embodied/motor/status`，
     **停更 > `motor_watchdog`（默认 2 s）即自动急停并接管发零**。
     ⚠️ 首次见到之前永不判失联（否则每次启动先来一次假警报，把真警报淹掉）。
  ④ **避障守卫**（`obstacle_guard.py`，默认开启）：按"**被命令的运动方向**"问
     LiDAR 原语 `~/sector_min_range`，阈值 = `速度 × obstacle_lookahead`（TTC 式，
     不是固定距离）。雷达/原语**不新鲜**时按「不知道 ≠ 安全」停车。
     ⚠️ 它依赖 Motor Driver 的状态（方向与速度从那里来）；关掉 `watch_motor`
     会让它**显式禁用并报错**，不会静默退化成"不避障"。

前置：避障要问 LiDAR 原语，所以需要 `embodied_lidar_driver` 在跑；
      要看它拦车，还需要 `embodied_motor_driver` 在跑（状态里有方向与速度）。

手工触发 / 解除（验收与测试用）：
    ros2 service call /safety_runtime/estop   std_srvs/srv/Trigger "{}"
    ros2 service call /safety_runtime/release std_srvs/srv/Trigger "{}"
    # 也可以直接喂一条文本，走真实的本地解析路径：
    ros2 topic pub --once /asr_node/voice_words std_msgs/msg/String "{data: 'stop'}"

看状态与事件：
    ros2 topic echo /embodied/safety/status
      # [是否锁存, 已发零帧数, 触发次数, motor 状态龄 ms,
      #  守卫是否使能, 此刻是否判停, 阈值 m, 扇区内最近回波 m, 运动方向 rad]
    ros2 topic echo /embodied/safety/events   # estop_triggered:<原因> / estop_released
      #   原因可为：voice:<词> / service / motor_driver_lost
      #             / obstacle:<距离>m<方位>deg / obstacle:unknown_scan

⚠️ 急停是**锁存**的：触发后必须显式 `~/release` 才能解除 —— 解除这个动作本身
   必须是一次明确的、有人负责的决定。避障触发用的也是同一把锁。
⚠️ 解除本节点的锁存**不会**自动解除 Motor Driver / Control Skill 各自的锁存，
   它们各有自己的 `~/resume`。三道锁是**独立**的，这是刻意的。

本版仍**不含**：限速、速度/区域限制、**绕行**（绕行属 Phase 3 的局部规划器）、
原地转身时的机体扫掠判断。
"""
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
        DeclareLaunchArgument(
            'motor_status_topic', default_value='/embodied/motor/status',
            description='看门狗监视的话题（避障也从这里取方向与速度）。'
                        '测试时可指向假话题以模拟 Motor Driver 挂掉'),
        DeclareLaunchArgument(
            'lidar_front_topic', default_value='/embodied/lidar/front',
            description='避障的**新鲜度心跳**话题。测试时可指向假话题以模拟雷达失联'),
        DeclareLaunchArgument(
            'sector_service', default_value='/lidar_driver/sector_min_range',
            description='避障问方向的 LiDAR 原语服务。测试时可指向假服务'),
        DeclareLaunchArgument(
            'enable_obstacle_guard', default_value='true',
            description='避障守卫开关。做其它验收（如上层 dry-run）时应显式设 false，'
                        '否则 LiDAR 原语没在跑时前进指令会被判「不知道」而锁存'),
        DeclareLaunchArgument(
            'motor_stop_service', default_value='/motor_driver/stop',
            description='best-effort 打开下游锁存用的服务。验收工装应指向它自己的假服务，'
                        '避免抢真的 Motor Driver 的服务名'),
        DeclareLaunchArgument(
            'control_stop_service', default_value='/control_skills/stop',
            description='同上，Control Skill 的停服务'),
        DeclareLaunchArgument(
            'watch_motor', default_value='true',
            description='对 Motor Driver 的停更看门狗。⚠️ 它同时是**避障的方向来源**：'
                        '关掉它，避障会显式禁用并报错（不会静默退化成"不避障"）'),
        DeclareLaunchArgument(
            'zero_channel_topic', default_value='/cmd_vel',
            description='独立零速通道的发布话题。⚠️ 验收工装可以把它指向 Motor Driver 的'
                        '**干跑话题**，这样两路发布者被放在同一条话题上、'
                        '而真的 /cmd_vel 上一个发布者都没有（`tools/cmd_vel_arbitration_probe.py`）'),
        Node(
            package='embodied_safety_runtime',
            executable='safety_runtime',
            name='safety_runtime',
            output='screen',
            parameters=[cfg, {
                'voice_topic': LaunchConfiguration('voice_topic'),
                'motor_status_topic': LaunchConfiguration('motor_status_topic'),
                'lidar_front_topic': LaunchConfiguration('lidar_front_topic'),
                'sector_service': LaunchConfiguration('sector_service'),
                'enable_obstacle_guard': LaunchConfiguration('enable_obstacle_guard'),
                'motor_stop_service': LaunchConfiguration('motor_stop_service'),
                'control_stop_service': LaunchConfiguration('control_stop_service'),
                'watch_motor': LaunchConfiguration('watch_motor'),
                'zero_channel_topic': LaunchConfiguration('zero_channel_topic'),
            }],
        ),
    ])
