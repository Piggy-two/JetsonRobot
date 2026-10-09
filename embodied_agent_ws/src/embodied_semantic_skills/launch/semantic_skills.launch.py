#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拉起 Semantic Skill 层。

    ros2 launch embodied_semantic_skills semantic_skills.launch.py

⚠️ 前置：**Vision Driver 在跑**（它才有 `~/find_in_view`）。
   视觉不在时本技能**不报错** —— 它会返回 `FAILED` 并说清"视觉服务不可用 —— 不知道，
   不是'没有'"。这是刻意的：**没在看 ≠ 没有**。

⚠️ 本技能**只读、不动**（网关注册表里 `causes_motion: false`）：
   不需要人看护，也不受 `allow_motion` 闸门约束。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_semantic_skills'),
                       'config', 'semantic_skills.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'find_service', default_value='/vision_driver/find_in_view',
            description='问谁（Vision Driver 的查询服务）'),
        Node(
            package='embodied_semantic_skills',
            executable='semantic_skills',
            name='semantic_skills',
            output='screen',
            parameters=[cfg, {'find_service': LaunchConfiguration('find_service')}],
        ),
        LogInfo(msg=[
            '\n── Semantic Skill ──\n',
            '  look_for：看一眼视野里有没有某个东西。\n',
            '  🔒 看到了 = TARGET_FOUND｜**确认没有** = TARGET_LOST｜'
            '**不知道** = FAILED\n',
            '     （画面糊 / 没帧 / 模型没就绪 / 类别不在表里 ⇒ 一律 FAILED，'
            '绝不会被报成"没有"）\n',
        ]),
    ])
