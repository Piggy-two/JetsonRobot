import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('embodied_voice_wakeup')
    cfg = os.path.join(share, 'config', 'voice_wakeup.yaml')
    default_keywords = os.path.join(share, 'config', 'keywords.txt')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_dir', default_value='',
            description='唤醒模型目录（**大文件，不进仓库**，见包 README）。留空 ⇒ 拒绝启动'),
        DeclareLaunchArgument(
            'keywords_file', default_value=default_keywords,
            description='关键词表。格式：`<音素...> @<显示名>`，见 config/keywords.txt'),
        DeclareLaunchArgument(
            'keywords_threshold', default_value='0.25',
            description='**召回与误报在它上面换**：越小越容易触发。别靠感觉调，要量'),
        DeclareLaunchArgument(
            'audio_device', default_value='',
            description='音频输入设备；留空 = 系统默认（本机 = 环形麦）'),
        Node(
            package='embodied_voice_wakeup',
            executable='wakeup_node',
            name='voice_wakeup',
            output='screen',
            parameters=[cfg, {
                'model_dir': LaunchConfiguration('model_dir'),
                'keywords_file': LaunchConfiguration('keywords_file'),
                'keywords_threshold': LaunchConfiguration('keywords_threshold'),
                'audio_device': LaunchConfiguration('audio_device'),
            }],
        ),
    ])
