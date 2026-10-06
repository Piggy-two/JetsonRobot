"""启动 JetsonRobot Overlay 电机/底盘 Driver。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_motor_driver motor_driver.launch.py

⚠️ **默认 dry_run:=true，车不会动**：节点不会向 `/cmd_vel` 发布任何东西，
   只会把同样的速度发到 `/embodied/motor/cmd_vel_dryrun`，供离线验证逻辑。

要让底盘**真的动**（必须有人看护、能直接断电 —— 本机没有物理急停，见 #22）：

    ros2 launch embodied_motor_driver motor_driver.launch.py dry_run:=false

停下来的两种方式（互相独立）：
    ros2 service call /motor_driver/stop   std_srvs/srv/Trigger "{}"   # 锁存，需 resume
    ros2 service call /motor_driver/resume std_srvs/srv/Trigger "{}"
    # 或者直接杀掉本节点 —— 但⚠️ 底盘没有指令超时保护，最后一条速度会被保持，
    # 所以杀节点**不是**停车手段；停车必须靠本节点持续发零（D-020）。

看状态（状态码 / 速度 / 各信号龄）：
    ros2 topic echo /embodied/motor/status
      data = [状态码, vx, vy, wz, 指令龄ms, imu龄ms, battery龄ms, 是否锁存, 是否dry_run]
      状态码 0=ok 1=no_cmd 2=telemetry_lost 3=stopped；龄为 -1 表示从未收到
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_motor_driver'),
                       'config', 'motor_driver.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='true（默认）= 不驱动底盘，只发干跑话题；'
                        'false = 真的向 /cmd_vel 发布，必须有人看护且能直接断电'),
        Node(
            package='embodied_motor_driver',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[cfg, {'dry_run': LaunchConfiguration('dry_run')}],
        ),
    ])
