"""启动 JetsonRobot Overlay Control Skill 层。

用法（先 source 厂商工作区，再 source 本 Overlay）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/JetsonRobot/embodied_agent_ws/install/setup.bash
    ros2 launch embodied_control_skills control_skills.launch.py

⚠️ **前置条件：Motor Driver 必须已经在跑**（本节点靠它的状态话题判断"底盘是否确认在线"）。
   没在跑的话，所有运动请求都会被拒绝 —— 这是**刻意的**，不是 bug。

    ros2 launch embodied_motor_driver motor_driver.launch.py     # 默认 dry_run=true，车不会动

调用原语：

    # 向前 0.5 米（机体坐标系：x 前 / y 左）
    ros2 service call /control_skills/move_relative \
        embodied_skills_interfaces/srv/MoveRelative "{x: 0.5, y: 0.0}"

    # 原地左转 90°（逆时针为正，单位弧度）
    ros2 service call /control_skills/rotate \
        embodied_skills_interfaces/srv/Rotate "{angle: 1.5708}"

    # 立即中止当前运动（单独的回调组，运动中也能立刻进来）
    ros2 service call /control_skills/stop std_srvs/srv/Trigger "{}"

⚠️ **这一层是开环的**：它只是"按时间发速度"，返回 success 的含义是
   **"速度按时长发完了"**，**不是"真的走到位了"**。轮子打滑、地面、负载、电压
   都会让实际位移偏离。闭环需要里程计/激光/视觉反馈，是后续的事。见 `motion_plan.py`。

一次只跑一个动作：运动中再来请求会被**拒绝**（busy），不排队 ——
排队会让"车现在到底在不在动"变得不可预测。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_control_skills'),
                       'config', 'control_skills.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖 ——
        #    未声明的会被静默忽略（见 DEV_NOTES 坑 9）。
        DeclareLaunchArgument(
            'output_topic', default_value='/embodied/motor/cmd_vel',
            description='Motor Driver 的执行器入口。⚠️ 不要指向 /cmd_vel —— 那样就绕过了 Driver 层'),
        Node(
            package='embodied_control_skills',
            executable='control_skills',
            name='control_skills',
            output='screen',
            parameters=[cfg, {'output_topic': LaunchConfiguration('output_topic')}],
        ),
    ])
