# Jetson Hybrid Embodied Agent
## 基于 Jetson Orin 的混合式语音具身智能机器人平台方案

---

## 1. 项目定位

本项目目标是在 Jetson Orin 上构建一套 **本地自治 + LLM Agent + 分层 Skill + 实时安全控制** 的具身智能机器人系统。

机器人通过语音接受自然语言任务，由 LLM 负责高层任务理解与规划；导航、避障、寻迹、目标跟随等能力作为本地 Skill 独立闭环执行；Linux Kernel、Driver 与硬件接口提供最底层的确定性能力；Safety Runtime 独立于大模型运行。

项目的核心思想不是“Jetson 上跑一个 Agent 去控制小车”，而是：

> **整台机器人本身就是一个 Embodied Agent。**

其中：
- **Jetson** 是机器人的计算与认知平台；
- **LLM** 是高级认知与任务规划模块；
- **Robot Runtime** 是承载 Context、Tools、State、Memory、Execution、Feedback 和 Safety 的 Physical AI Harness；
- **Skill Runtime** 将硬件能力逐层封装成可被 Agent 调用的机器人技能；
- **Linux Kernel / Driver** 是真实硬件能力的基础支撑；
- **Motor / Camera / LiDAR / Mic** 构成机器人的身体与感知器官。

---

# 2. 核心架构思想

整个系统采用：

> **Hierarchical Skill Architecture + Hybrid Agent Architecture + Event-driven Execution**

即：

```text
LLM / Agent
    ↓
Semantic Skill
    ↓
Autonomous Skill
    ↓
Control Primitive
    ↓
Linux Driver
    ↓
Hardware
```

越往上：
- 语义更强
- 抽象程度更高
- 更依赖 Agent / LLM
- 时间尺度更慢

越往下：
- 更确定
- 更实时
- 更靠近硬件
- 不依赖 LLM

---

# 3. 机器人本身就是一个 Agent

一个完整 Agent 应该具备：

```text
Perception
+
Reasoning
+
Action
+
Environment Feedback
```

本项目中：

```text
               Environment
                    │
                    ▼
               Perception
        Camera / LiDAR / Mic
                    │
                    ▼
               Cognition
        LLM / Planner / Memory
                    │
                    ▼
                 Skills
    Navigation / Follow / Search / Patrol
                    │
                    ▼
                 Action
          Motor / Speaker / Motion
                    │
                    └──────────────→ Environment
```

因此，Agent 不应该只被理解为一个运行在 Jetson 上的 Python 程序。

更准确的定义是：

> **机器人整体是 Agent，Jetson 提供认知与计算能力，LLM 提供高层推理能力，Skill Runtime 和 Linux 系统负责将智能决策转化为真实世界动作。**

---

# 4. Jetson 的角色

Jetson Orin 是整台机器人上的核心计算平台，主要承担：

- 语音识别 ASR
- Agent Runtime
- 本地轻量 LLM（可选）
- 云端 LLM 调用
- TensorRT 视觉推理
- ROS2
- SLAM / Navigation
- Skill Runtime
- Safety / Monitor
- 系统状态管理

Jetson 不只是“跑模型”，而是整个 Robot Agent 的中央计算节点。

---

# 5. LLM 的角色

LLM 不负责实时控制，而负责：

- 自然语言理解
- 复杂任务拆解
- 多步骤规划
- Skill 选择
- 条件判断
- 失败后的任务重规划
- 最终自然语言总结

例如：

用户说：

> 去桌子旁边看看有没有水杯，没有的话再去沙发附近看看。

LLM 负责生成高层计划：

```text
1. search_object("cup", "desk")
2. if NOT_FOUND:
       search_object("cup", "sofa")
3. report_result()
```

LLM 不负责：

```text
左轮 100 RPM
右轮 90 RPM
向左偏 2°
前方 0.4m 有障碍
```

这些全部由本地 Skill 与 Controller 完成。

---

# 6. Physical AI Harness

Robot Runtime 可以被理解成围绕 LLM 构建的一套 **Physical AI Harness**。

它负责给 LLM 提供：

```text
Context
Tools
State
Memory
Execution Environment
Feedback
Safety Constraints
```

整体链路：

```text
LLM
 ↓
Agent Runtime
 ↓
Tool / Skill Registry
 ↓
Skill Manager
 ↓
ROS2 / Autonomous Runtime
 ↓
Control Runtime
 ↓
Linux Kernel / Driver
 ↓
Hardware
```

真实世界反馈：

```text
Camera / LiDAR / Odom / Motor
            ↓
      Robot Runtime
            ↓
          Event
            ↓
          Agent
```

因此 Harness 的职责是：

> **把 LLM 的抽象决策变成受控、可验证、可反馈的物理行为。**

---

# 7. 分层 Skill 架构

## Level 0：Hardware Capability

最底层是真实硬件能力：

```text
Motor
Camera
LiDAR
Microphone
Speaker
IMU
```

由 Linux Kernel 和 Driver 提供访问，例如：

```text
/dev/video0
/dev/ttyUSB0
CAN
I2C
SPI
PWM
USB
```

这一层不直接暴露给 LLM。

---

## Level 1：Driver / Primitive

提供最基础的软件原语：

```text
set_motor_speed()
read_camera_frame()
read_lidar_scan()
read_imu()
play_audio()
```

特点：
- 确定性
- 高频
- 靠近硬件
- 不依赖 Agent

---

## Level 2：Control Skill

将基础原语组合成简单控制能力：

```text
move_forward()
move_backward()
rotate()
move_relative()
stop()
```

内部可能包含：
- PID
- 轮速控制
- 里程计
- 编码器反馈
- 麦轮运动学

---

## Level 3：Autonomous Skill

真正的机器人自主能力：

```text
navigate_to()
follow_person()
follow_line()
avoid_obstacle()
patrol_route()
return_home()
```

整个过程在 Jetson 本地闭环运行，不需要 LLM 高频参与。

---

## Level 4：Semantic Skill

进一步把机器人能力封装成有语义的任务能力：

```text
search_object("cup")
inspect_area("office")
find_person("Tom")
check_room("meeting_room")
```

例如：

```text
search_object("cup", "desk")
```

内部：

```text
navigate_to("desk")
 ↓
capture_image()
 ↓
detect_object("cup")
 ↓
rotate_scan()
 ↓
return FOUND / NOT_FOUND
```

---

## Level 5：Agent Task

最高层是自然语言任务：

> 去实验室看看有没有人，如果没人就回来。

Agent 只需要组合 Semantic Skill：

```text
inspect_area("lab")
 ↓
if person_found:
    report()
else:
    return_home()
```

---

# 8. 三层时间尺度

| 层级 | 时间尺度 | 主要职责 | LLM 参与 |
|---|---|---|---|
| Agent Layer | 秒级 / 事件驱动 | 理解、规划、重规划 | 是 |
| Skill / Autonomy Layer | 10~30 Hz 或异步任务 | 导航、跟随、搜索、巡检 | 否 |
| Control / Safety Layer | 20~100 Hz | 电机、PID、急停、避障 | 禁止 |

例如：

```text
用户：去门口
```

Agent：

```text
navigate_to("door")
```

之后几十秒：

```text
SLAM
Localization
Path Planning
Obstacle Avoidance
Motion Control
```

全部由本地系统执行。

Agent 只等待：

```text
ARRIVED
BLOCKED
FAILED
CANCELLED
```

---

# 9. Voice Interaction

```text
Microphone
   ↓
Wake Word
   ↓
VAD
   ↓
ASR
   ↓
Text
   ↓
Command Router
```

任务完成后：

```text
Agent Response
   ↓
TTS
   ↓
Speaker
```

形成：

```text
人说话
 ↓
机器人理解
 ↓
机器人行动
 ↓
机器人反馈
 ↓
机器人说话
```

---

# 10. Hybrid Command Router

## A. Safety Command

例如：

```text
停
急停
取消任务
别动
```

直接：

```text
Voice
 ↓
Local Parser
 ↓
Safety Runtime
 ↓
Motor Stop
```

不经过 LLM。

## B. Deterministic Command

例如：

```text
向前走 0.5 米
右转 30°
回到原地
```

本地解析并直接调用 Control Skill。

## C. Agent Task

例如：

> 去桌子旁边找杯子，没有就去沙发旁边找。

流程：

```text
ASR
 ↓
Agent Runtime
 ↓
LLM Planner
 ↓
Semantic Skill
```

---

# 11. Agent Runtime

建议包含：

```text
agent_runtime/
├── planner
├── executor
├── tool_registry
├── skill_registry
├── memory
├── task_manager
├── context_manager
└── event_manager
```

核心循环：

```text
Understand
 ↓
Plan
 ↓
Select Skill
 ↓
Execute
 ↓
Observe Event
 ↓
Evaluate
 ↓
Re-plan
```

---

# 12. Skill Manager

Agent 不直接访问：

```text
Motor
GPIO
PWM
ROS Topic
```

而调用：

```text
navigate_to()
follow_person()
search_object()
inspect_area()
```

Skill Manager 负责：
- Skill 注册
- 参数校验
- 执行
- 状态管理
- Cancel
- Timeout
- Event 上报

---

# 13. 自动避障

```text
LiDAR
 ↓
Local Costmap
 ↓
Obstacle Detection
 ↓
Local Planner
 ↓
Velocity Command
```

普通障碍自动绕行；只有长时间无法通过时才：

```text
BLOCKED
 ↓
Agent
 ↓
Re-plan
```

---

# 14. SLAM 与自主导航

```text
LiDAR
 ↓
SLAM
 ↓
Map
 ↓
Localization
 ↓
Global Planner
 ↓
Local Planner
 ↓
Controller
```

地图增加语义标签：

```text
desk
door
sofa
home
office
lab
```

Agent 只需要：

```text
navigate_to("desk")
```

---

# 15. 目标跟随

```text
follow_person()
```

内部：

```text
Camera
 ↓
YOLO
 ↓
Tracker
 ↓
Target Position
 ↓
Follower Controller
 ↓
Obstacle Avoidance
 ↓
Motor
```

Agent 只处理启动、停止以及 TARGET_LOST 后的任务决策。

---

# 16. 寻迹

```text
follow_line()
```

内部：

```text
Camera
 ↓
ROI
 ↓
Line Detection
 ↓
Deviation
 ↓
PID
 ↓
Mecanum Control
```

可进一步封装：

```text
patrol_route()
```

---

# 17. Vision Skill

```text
Camera
 ↓
GStreamer
 ↓
CUDA
 ↓
TensorRT
 ↓
YOLO
 ↓
Tracker
```

对上层暴露：

```text
detect_object()
detect_person()
capture_image()
get_target_position()
describe_scene()
```

---

# 18. LLM 混合架构

```text
Task Router
 ├── Rule Engine
 ├── Local Small LLM
 └── Cloud LLM
```

### Rule Engine
负责：

```text
stop
move
turn
status
```

### Local LLM
后期加入，用于：
- 简单语言理解
- 简单 Tool Calling
- 离线模式

### Cloud LLM
负责：
- 复杂任务规划
- 条件任务
- 多步骤推理
- 异常处理
- 复杂语言理解

断网后机器人仍可以：

```text
移动
导航
避障
寻迹
跟随
视觉检测
停止
```

只是高级 Agent 能力降级。

---

# 19. Tool / Skill Safety Gateway

```text
Agent
 ↓
Tool / Skill Request
 ↓
Safety Gateway
 ├── Schema Validation
 ├── Permission
 ├── Range Check
 ├── Timeout
 ├── Cancel
 └── Result Validation
 ↓
Skill Manager
```

例如：

```text
LLM:
move_relative(100m)
```

系统：

```text
REJECTED
distance exceeds safety limit
```

---

# 20. Safety Runtime

Safety Runtime 独立于 LLM：

```text
Emergency Stop
Obstacle Stop
Speed Limit
Watchdog
Command Timeout
Motor Timeout
Low Battery Protection
```

执行优先级：

```text
Safety
 >
Control
 >
Skill
 >
Agent
```

即：

> **Safety 具有最终否决权。**

---

# 21. Event-driven Agent

Skill 状态：

```text
STARTED
RUNNING
ARRIVED
TARGET_FOUND
TARGET_LOST
BLOCKED
FAILED
CANCELLED
```

例如：

```text
Agent
 ↓
navigate_to("desk")
 ↓
WAIT

[Robot Local Runtime]

ARRIVED
 ↓
Wake Agent
 ↓
search_object("cup")
```

这就是：

> **Event-driven Agent，而不是 Real-time LLM Controller。**

---

# 22. 推荐进程架构

```text
voice_service
agent_service
skill_service
vision_service
navigation_service
robot_service
monitor_service
```

建议：

```text
C++
├── robot_service
├── navigation_service
├── safety
└── 部分 vision runtime

Python
├── voice_service
├── agent_service
└── LLM orchestration
```

---

# 23. 最终 Demo

## Demo 1：安全控制

用户：

> 停！

本地 Safety Runtime 立即停止。

## Demo 2：简单运动

用户：

> 向前走半米。

本地解析并调用：

```text
move_relative(0.5)
```

## Demo 3：自主导航

用户：

> 去门口。

Agent：

```text
navigate_to("door")
```

机器人本地完成规划、避障、控制和到达。

## Demo 4：目标跟随

用户：

> 跟着前面那个人。

Agent：

```text
follow_person()
```

后续完全本地闭环。

## Demo 5：复杂 Agent Task

用户：

> 去桌子旁边找杯子，没有就去沙发附近找，找到以后告诉我。

执行：

```text
Plan
 ↓
search_object("cup", "desk")
 ↓
NOT_FOUND
 ↓
search_object("cup", "sofa")
 ↓
FOUND
 ↓
Report
```

---

# 24. 启动测试与接口验收

在开发新的 Agent、Skill 或视觉模型前，先对现有厂商 ROS2 运行栈做启动测试和接口验收。目标是确认“已安装的软件包”与“当前小车上真实可用的硬件能力”一致；仅发现包名、进程或 Workspace 不视为设备已验收。

## 24.1 验收原则

- 保持厂商 `ros2_ws` 与 `third_party` Workspace 不变，将其作为系统 SDK 使用。
- 新项目放在独立的 `embodied_agent_ws` Overlay Workspace 中，避免直接修改厂商包。
- 按厂商既有 source 顺序加载 ROS2 环境，记录 `ROS_DISTRO`、`AMENT_PREFIX_PATH`、`PYTHONPATH` 和启动脚本来源。
- 单设备测试通过后再进行多传感器联调；任何运动测试均保留急停、限速和人工看护。
- 将每项测试的启动命令、节点名、topic、service、action、参数文件、设备路径和结果写入接口清单，作为后续 Skill 的唯一对接依据。

## 24.2 启动前基线检查

先确认 Jetson 与 ROS2 环境正常，特别是根分区空间、性能模式和现有节点状态：

```bash
jtop
tegrastats
df -h /
echo $ROS_DISTRO
which ros2
env | grep -E 'ROS|AMENT|COLCON'
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

当前基线中的根分区可用空间较紧张，应在部署 PyTorch、模型权重、TensorRT Engine、ROS bag 或 Docker 镜像前先确认是否可安全扩容或清理；不要在未确认分区布局前修改分区。

## 24.3 底盘与安全测试

验证厂商底盘控制链路，包括 `ros_robot_controller`、`controller`、`kinematics`、`servo_controller` 及其实际启动文件。

验收内容：

- 确认控制器节点、里程计、TF、速度指令 topic 与急停接口。
- 在车轮悬空或留有安全距离的条件下，依次测试 `stop()`、低速前进、后退、左右平移、原地旋转。
- 验证速度上限、命令超时、通信中断后的停车行为，以及遥控/手柄与软件指令的优先级。
- 记录底盘坐标系、麦克纳姆运动学约定、`cmd_vel` 类型和速度单位，禁止由 LLM 直接发布底盘控制命令。

通过标准：低速运动方向正确；停止命令可重复、可立即生效；里程计与 TF 持续发布；异常或超时不会保持运动。

## 24.4 LiDAR 与避障测试

确认实际使用的雷达型号、串口/USB 设备路径、波特率和对应驱动。当前环境已发现 `ydlidar_ros2_driver`、`sllidar_ros2`、`sclidar_ros2`、`ldlidar_stl_ros2`、`Aurora930` 等候选包，但需以实机 launch 文件和 ROS graph 为准。

验收内容：

- 启动单一雷达驱动，确认 `LaserScan` 数据频率、角度范围、量程和 frame_id。
- 在 RViz 中检查点云/扫描方向、安装朝向和 TF 变换。
- 放置静态障碍物，验证 local costmap 或避障模块能接收扫描并触发减速/停止。
- 记录驱动包、launch、参数文件、串口权限与设备重连行为。

通过标准：扫描稳定无大面积丢点；坐标方向正确；障碍物能被安全控制或导航层识别。

## 24.5 相机与视觉测试

验证 Orbbec 相机链路及 ROS2 图像接口。当前已发现 `orbbec_camera`、`orbbec_camera_msgs`、`cv_bridge`、`image_geometry`、`apriltag_msgs`，后续 YOLO/TensorRT 节点在此接口之上接入。

验收内容：

- 确认 RGB、深度、CameraInfo 和 TF topic；记录分辨率、帧率、编码格式及 frame_id。
- 在 RViz 或图像查看工具中检查画面、深度图、时间戳和帧率稳定性。
- 如使用深度相机，验证 RGB 与深度对齐、相机内参及外参。
- 运行最小视觉节点，验证 `cv_bridge` 收图；再验证 ONNX/TensorRT 推理节点可订阅图像并发布检测结果。

通过标准：连续采集稳定，无明显掉帧或时间戳异常；视觉节点可获得正确图像；检测结果坐标与相机画面一致。

## 24.6 语音与麦克风测试

验证麦克风、离线 ASR 与扬声器/TTS 的完整输入输出链路。当前环境发现 `xf_mic_asr_offline` 及其消息包，但仍需确认实际麦克风阵列、USB 音频设备、采样率和启动方式。

验收内容：

- 列出音频输入输出设备，确认麦克风阵列和扬声器设备编号。
- 启动 ASR 节点，检查音频流、识别文本 topic 或 service，以及静音、噪声环境下的表现。
- 测试关键词/停止命令的本地解析；该路径必须绕过 LLM 并能触发 Safety Runtime。
- 测试 TTS 播报，确认音量、延迟和与底盘运动状态的互不阻塞。

通过标准：可稳定获得语音文本；“停/急停/取消任务”等安全指令能进入本地安全链路；TTS 能清晰播报状态与结果。

## 24.7 SLAM、导航与系统联调

底盘和雷达单测通过后，再测试 `slam`、`gmapping`、`navigation`、`rf2o_laser_odometry`、`teb_local_planner` 等现有能力。

验收顺序：

```text
底盘与急停
  ↓
LiDAR 与 TF
  ↓
里程计与定位
  ↓
建图 / 保存地图
  ↓
导航到测试点
  ↓
静态障碍物避障
  ↓
相机、语音与导航联调
```

通过标准：可复现建图、定位和到达测试点；遇障时优先由本地控制/导航层处理；长时间无法通行时才向上层返回 `BLOCKED`，由 Agent 决定重规划或询问用户。

## 24.8 接口清单交付物

启动测试完成后，形成一份版本化的硬件与 ROS2 接口清单，至少包含：

| 模块 | 已确认驱动/包 | 启动入口 | 输入接口 | 输出接口 | TF / 设备路径 | 验收结果 |
|---|---|---|---|---|---|---|
| 底盘 | 实机确认后填写 | launch / service | `cmd_vel` 或等效接口 | odom / status | base_link / 控制器链路 | 通过 / 待测 |
| LiDAR | 实机确认后填写 | launch | 串口 / USB | `scan` | laser frame / 设备路径 | 通过 / 待测 |
| 相机 | Orbbec 实机确认后填写 | launch | RGB / Depth | image / CameraInfo | camera frame | 通过 / 待测 |
| 语音 | 麦克风与 ASR 实机确认后填写 | launch | audio | text / intent | 音频设备编号 | 通过 / 待测 |
| 导航 | 实机确认后填写 | launch | goal / map | status / event | map / odom / base_link | 通过 / 待测 |

只有接口清单中标记为“通过”的能力，才允许封装为 Driver / Primitive、Control Skill 或上层 Semantic Skill。

# 25. 项目开发阶段

## Phase 0：环境与硬件启动验收
完成 Jetson 基线检查、厂商 ROS2 环境加载、底盘、LiDAR、Camera、Mic、Speaker 的单设备启动测试，并产出接口清单。

## Phase 1：Driver / Primitive
完成 Camera Driver、Motor Driver、LiDAR Driver，以及 set_velocity()、capture_frame()、read_scan()。

## Phase 2：Robot Control
完成 move_forward()、rotate()、move_relative()、stop()。

## Phase 3：Autonomous Skills
完成 SLAM、Navigation、Obstacle Avoidance、follow_person()、follow_line()。

## Phase 4：Semantic Skills
完成 search_object()、inspect_area()、patrol_route()、return_home()。

## Phase 5：Voice System
完成 Wake Word、VAD、ASR、TTS。

## Phase 6：Agent Runtime
完成 Planner、Executor、Skill Registry、Event Manager、Memory、Safety Gateway。

## Phase 7：Hybrid LLM
接入 Rule Engine + Cloud LLM，并预留 Local Small LLM。

---

# 26. 第一版不做的功能

为了保证主线清晰，第一版暂时不加入：

```text
机械臂
多机器人
强化学习导航
复杂 RAG
大型本地 VLM
自动充电
```

---

# 27. 项目核心亮点

## 1. Robot-as-Agent
不是“Agent 控制机器人”，而是：

> **机器人整体就是 Agent。**

## 2. Physical AI Harness
通过 Context、Tools、Skills、State、Memory、Execution、Feedback、Safety 将 LLM 与真实物理世界连接。

## 3. Hierarchical Skill Architecture

```text
Hardware
 ↓
Driver
 ↓
Primitive
 ↓
Control Skill
 ↓
Autonomous Skill
 ↓
Semantic Skill
 ↓
Agent Task
```

让复杂智能建立在确定性的底层能力之上。

## 4. Hybrid Intelligence

```text
Rule Engine
+
Local AI
+
Local Autonomy
+
Cloud LLM
```

而不是所有决策都依赖云端。

## 5. Event-driven Execution
Agent 只处理任务级事件，不进入实时控制环。

## 6. Safe Physical Agent
LLM 无法直接控制电机，所有动作都经过 Skill Manager、Safety Gateway 和 Safety Runtime。

---

# 28. 最终项目定义

> **基于 Jetson Orin 构建一个 Robot-as-Agent 的混合式语音具身智能平台，将整台机器人抽象为具备 Perception、Reasoning、Skill、Action 和 Feedback 的完整 Agent；以 LLM 作为高级认知核心，以 Robot Runtime 作为 Physical AI Harness，并通过分层 Skill 架构将 Linux Kernel / Driver 提供的底层硬件能力逐级封装为 Control Skill、Autonomous Skill 和 Semantic Skill，实现自然语言任务到真实机器人行为的安全闭环。**

---

# 29. 一句话版本

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**
