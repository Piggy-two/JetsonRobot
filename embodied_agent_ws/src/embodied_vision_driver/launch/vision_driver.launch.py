#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拉起 Vision Driver。

    ros2 launch embodied_vision_driver vision_driver.launch.py

⚠️ 前置：**相机在出图**（厂商 `usb_cam` 在跑，或把 `image_topic` 指向我们自己的
   `embodied_camera_driver` 输出）。没有图像时本节点**不报错** —— 它会一直回答
   「不知道」（"还没收到任何图像"），这是刻意的：**没在看 ≠ 没有**。

⚠️ 模型在**后台线程**加载（TensorRT 反序列化要十几秒）：节点立刻可用，
   加载完成前一律回答「不知道（模型还没就绪）」。

⚠️ 还有一件事**本节点管不了、但会让它整体失效**：**镜头对焦**。
   2026-10-09 实测相机出厂是失焦的，那时所有检测都是 0 个，而它和"真没东西"
   看起来一模一样（DEV_NOTES 坑 43）。调焦用 `tools/focus_assist.py`，
   看 `/embodied/vision/diag` 的第 2 位（清晰度）也能立刻看出来。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('embodied_vision_driver'),
                       'config', 'vision_driver.yaml')
    return LaunchDescription([
        # ⚠️ 参数必须**声明**才能用 `ros2 launch ... x:=y` 覆盖（DEV_NOTES 坑 9）
        DeclareLaunchArgument(
            'image_topic', default_value='/depth_cam/rgb0/image_raw',
            description='输入图像。默认厂商相机原始话题；'
                        '也可指向 /embodied/camera/image（我们自己的相机 Driver）'),
        DeclareLaunchArgument(
            'model_dir', default_value=(
                '/home/ubuntu/ros2_ws/src/example/example/yolo_detect/models/26'),
            description='模型目录。⚠️ 默认从**厂商 SDK 借**通用模型（只读，不改厂商文件）'),
        DeclareLaunchArgument('model_file', default_value='yolo26n.engine',
                              description='模型文件。⚠️ 换模型必须换类别表 —— '
                                          '本节点从模型**自己**读类别，不需要人配'),
        DeclareLaunchArgument('confidence', default_value='0.25',
                              description='发到话题上的置信度门槛'),
        DeclareLaunchArgument('quality_min', default_value='100.0',
                              description='清晰度门槛（判"糊"用，本机实测标定：'
                                          '失焦≈40、调好≈3050）'),
        DeclareLaunchArgument('frame_max_age', default_value='0.5',
                              description='图像最大年龄（秒），超了就说「不知道」'),
        Node(
            package='embodied_vision_driver',
            executable='vision_driver',
            name='vision_driver',
            output='screen',
            parameters=[cfg, {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_dir': LaunchConfiguration('model_dir'),
                'model_file': LaunchConfiguration('model_file'),
                'confidence': LaunchConfiguration('confidence'),
                'quality_min': LaunchConfiguration('quality_min'),
                'frame_max_age': LaunchConfiguration('frame_max_age'),
            }],
        ),
        LogInfo(msg=[
            '\n── Vision Driver ──\n',
            '  它只回答"画面里有没有某个东西、放在哪一侧"。\n',
            '  🔒 找不到 ≠ 没东西：画面糊 / 没帧 / 模型没就绪 / 类别不在表里\n',
            '     ⇒ 一律回答「不知道」（理由见 vision_query.py 顶部）。\n',
        ]),
    ])
