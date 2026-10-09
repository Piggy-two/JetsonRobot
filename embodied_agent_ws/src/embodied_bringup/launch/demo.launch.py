#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**一键拉起本项目的整栈**（不含厂商 `~/ros2_ws` 那一套）。

    ros2 launch embodied_bringup demo.launch.py [参数...]

为什么要有它
------------
在此之前，"起一次栈"是**手敲六条 `ros2 launch`**（验收文档里到处是那六行）。
参数多、彼此有耦合（D-036 / D-037），**起错一个就表现得像"功能坏了"**：

| 起错的地方 | 症状 | 真相 |
|---|---|---|
| `allow_motion` 没开 | 任务被拒："拒绝派发可能引起运动的技能" | 闸门在正常工作 |
| 规则表没留空 | LLM 那一跳**根本不会被问到** | 验的不是你想验的东西 |
| `require_safety` 还开着而 Safety 没起 | Motor Driver 报状态 5，车不动 | D-037 的否决在生效 |
| 避障守卫还开着 | 一往下令就往障碍方向锁存 | D-036 的守卫在生效 |

⇒ 把这些**收口成一条命令**，并把"这一栈现在是什么安全姿态"**打在启动日志里**。

⚠️ **它不改变任何安全默认值**：门还是各节点自己那几道，本文件只是把它们
按"能跑起来"的组合摆好，并**显式打印出来**。要真动，必须有人**显式**说
`dry_run:=false`（CLAUDE.md §8：运动测试必须保留急停、限速和人工看护）。

前置（本文件**不**拉厂商栈）
----------------------------
`/scan` 来自厂商 `ldlidar` 节点，遥测来自 `/ros_robot_controller` ——
它们在厂商 bringup 里（`start_app_node.service`）。**先确认厂商栈在跑**，
否则 LiDAR Driver 会报"扫描陈旧/无数据"，避障与 Autonomous 技能都会保守拒绝。

用法
----
看（**车不会动**，干跑）—— 推荐先用这条确认链路：
    DEEPSEEK_API_KEY=... ros2 launch embodied_bringup demo.launch.py \\
        llm_enabled:=true allow_motion:=true \\
        llm_base_url:=https://api.deepseek.com/v1 llm_model:=deepseek-flash \\
        llm_api_key_env:=DEEPSEEK_API_KEY

真动（**人工看护 + 手能直接断电**；本机没有物理急停 #22）：
    DEEPSEEK_API_KEY=... ros2 launch embodied_bringup demo.launch.py \\
        dry_run:=false allow_motion:=true llm_enabled:=true ...（同上）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, filename, args, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', filename)),
        launch_arguments=args.items(), condition=condition)


def generate_launch_description():
    args = [
        # ---- 安全姿态（三个默认值都是**保守**那一侧）----
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Motor Driver 干跑。true（默认）= 车**不可能**动；'
                        'false = 真的驱动底盘 —— 必须有人看护且能直接断电'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='三层闸门（D-033）：网关与 Agent 是否允许派发会引起运动的技能。'
                        '⚠️ 它**独立于** dry_run：两个都要显式打开，车才可能动'),
        DeclareLaunchArgument(
            'obstacle_guard', default_value='true',
            description='Safety 的避障守卫（D-036）。默认开 —— 它是**安全功能**。'
                        '⚠️ 但做 Agent/运动学验收时要关掉：守卫会拦住"正被命令前进"的验收本身'),
        DeclareLaunchArgument(
            'require_safety', default_value='true',
            description='Motor Driver 读安全层状态（D-037）。默认开；'
                        '**单独跑 Motor Driver 做台架实验**时才设 false'),
        # ---- 规划那一跳 ----
        DeclareLaunchArgument(
            'rules_file', default_value='',
            description='规则表 YAML。⚠️ **留空（默认）⇒ 一切都走 LLM 那一跳**；'
                        '要验 LLM 就必须留空（D-038）'),
        DeclareLaunchArgument(
            'llm_enabled', default_value='false',
            description='LLM 规划（D-038）。默认关：它会把**用户说的话发到第三方**'),
        DeclareLaunchArgument('llm_base_url', default_value='',
                              description='OpenAI 兼容端点，如 https://api.deepseek.com/v1'),
        DeclareLaunchArgument('llm_model', default_value='',
                              description='模型名（如 deepseek-flash）'),
        DeclareLaunchArgument('llm_api_key_env', default_value='',
                              description='**密钥所在的环境变量名**（不是密钥本身）'),
        DeclareLaunchArgument(
            'llm_timeout', default_value='30.0',
            description='单次 LLM 调用的硬上限（秒）。⚠️ 比 Agent 自己的默认（8s）宽：'
                        '**推理模型要想几秒**，太紧会把正常调用判成超时'),
        DeclareLaunchArgument(
            'max_replans', default_value='2',
            description='重规划预算（D-044）。0 = 关掉（计划失败就此收手）'),
        # ---- 感知那一路 ----
        DeclareLaunchArgument(
            'with_vision', default_value='true',
            description='是否连**视觉**一起起（Vision Driver + Semantic Skill）。'
                        '🔒 这一路**只读、不动**（`causes_motion: false`），'
                        '所以默认开着 —— 演示要的就是"看得见"。'
                        '关掉它（`false`）用于：不想占 GPU、或做与视觉无关的验收'),
        DeclareLaunchArgument(
            'vision_model_dir', default_value=(
                '/home/ubuntu/ros2_ws/src/example/example/yolo_detect/models/26'),
            description='视觉模型目录。⚠️ 默认从**厂商 SDK 借**通用模型（只读，不改厂商文件）'),
        DeclareLaunchArgument('vision_model_file', default_value='yolo26n.engine',
                              description='模型文件名（80 类通用 COCO）'),
    ]

    includes = [
        # ⚠️ 顺序有意义：**安全层先起**。Motor Driver 起来后 1.0 s 内看不到
        #    安全层状态就会拒绝运动（D-037），倒过来起会有一段"车动不了"的窗口。
        _include('embodied_safety_runtime', 'safety_runtime.launch.py', {
            'enable_obstacle_guard': LaunchConfiguration('obstacle_guard'),
        }),
        _include('embodied_motor_driver', 'motor_driver.launch.py', {
            'dry_run': LaunchConfiguration('dry_run'),
            'require_safety': LaunchConfiguration('require_safety'),
        }),
        _include('embodied_lidar_driver', 'lidar_driver.launch.py', {}),
        _include('embodied_control_skills', 'control_skills.launch.py', {}),
        _include('embodied_autonomous_skills', 'autonomous_skills.launch.py', {}),
        _include('embodied_skill_gateway', 'skill_gateway.launch.py', {
            'allow_motion': LaunchConfiguration('allow_motion'),
        }),
        _include('embodied_agent_runtime', 'agent_runtime.launch.py', {
            'allow_motion': LaunchConfiguration('allow_motion'),
            'rules_file': LaunchConfiguration('rules_file'),
            'llm_enabled': LaunchConfiguration('llm_enabled'),
            'llm_base_url': LaunchConfiguration('llm_base_url'),
            'llm_model': LaunchConfiguration('llm_model'),
            'llm_api_key_env': LaunchConfiguration('llm_api_key_env'),
            'llm_timeout': LaunchConfiguration('llm_timeout'),
            'max_replans': LaunchConfiguration('max_replans'),
        }),
        # 感知那一路。⚠️ 排在最后：它**谁也不依赖**（相机在厂商栈里、
        # 技能服务自己会重试发现），但让它最后起，日志读起来更像"一层一层搭上去"。
        _include('embodied_vision_driver', 'vision_driver.launch.py', {
            'model_dir': LaunchConfiguration('vision_model_dir'),
            'model_file': LaunchConfiguration('vision_model_file'),
        }, condition=IfCondition(LaunchConfiguration('with_vision'))),
        _include('embodied_semantic_skills', 'semantic_skills.launch.py', {},
                 condition=IfCondition(LaunchConfiguration('with_vision'))),
    ]

    # 把"这一栈现在什么姿态"打在启动日志里 —— 出问题时第一眼要能看见，
    # 而不是去六个终端里翻各自的启动参数。
    banner = LogInfo(msg=[
        '\n──────── 本项目整栈启动（embodied_bringup）────────\n',
        '  Motor Driver dry_run = ', LaunchConfiguration('dry_run'),
        '   ← false 意味着**车真的会动**：人要能直接断电，本机没有物理急停\n',
        '  运动闸门 allow_motion = ', LaunchConfiguration('allow_motion'), '\n',
        '  避障守卫 obstacle_guard = ', LaunchConfiguration('obstacle_guard'),
        '   （关掉它只为验收；平时开着才是对的）\n',
        '  规划那一跳 llm_enabled = ', LaunchConfiguration('llm_enabled'),
        '   规则表 rules_file = ', LaunchConfiguration('rules_file'),
        '（空 ⇒ 全走 LLM）\n',
        '  感知 with_vision = ', LaunchConfiguration('with_vision'),
        '   ← 只读、不动；关掉它就没有"看得见"的能力\n',
        '───────────────────────────────────────────────',
    ])

    return LaunchDescription(args + [banner] + includes)
