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
      data = [状态码, vx, vy, wz, 指令龄ms, imu龄ms, battery龄ms,
              是否锁存, 是否dry_run, 是否需重新使能]
      状态码 0=ok 1=no_cmd 2=telemetry_lost 3=stopped 4=rearm_required；龄为 -1 表示从未收到

看链路事件（只在变化时发）：
    ros2 topic echo /embodied/motor/events
      chassis_link_lost / chassis_link_recovered / rearm_required / rearmed

⚠️ **底盘链路失联时的安全窗口**（D-021 实测）：
    USB 断线后 `/odom` 照发 28.5 Hz 而 `imu_raw` / `battery` 停发；
    **重新 bind 不自恢复**，真恢复需要 `sudo systemctl restart start_app_node.service`。
    而这段窗口里**底盘保持着最后一条速度指令**（D-020）——「停不下来」是真实风险。
    本节点的处置：
      ① 遥测超时即判失联，**持续发零**；
      ② 若失联那刻底盘"可能还在动"（存在未超时且非零的指令），
         恢复后**置位「需重新使能」**，在显式 `~/resume` 之前**即使上层继续发指令也一律输出零**
         —— 防止链路一恢复就"毫无预兆地接着跑"；
      ③ **重启厂商服务需要 root，本节点做不到**，所以真恢复仍须人工介入。
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
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... imu_topic:=x` 覆盖 ——
        #    未声明的会被**静默忽略**（本轮就踩过：以为改了话题，其实还在订阅真遥测）。
        DeclareLaunchArgument(
            'imu_topic', default_value='/ros_robot_controller/imu_raw',
            description='存活判据之一（D-021）。测试时可指向假话题以制造链路失联'),
        DeclareLaunchArgument(
            'battery_topic', default_value='/ros_robot_controller/battery',
            description='存活判据之二（D-021）。只认这两个，绝不认 /odom'),
        Node(
            package='embodied_motor_driver',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[cfg, {
                'dry_run': LaunchConfiguration('dry_run'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'battery_topic': LaunchConfiguration('battery_topic'),
            }],
        ),
    ])
