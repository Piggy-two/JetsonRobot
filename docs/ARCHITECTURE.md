# ARCHITECTURE.md — 系统架构

> 本文件描述 JetsonRobot 的系统整体架构。**架构发生变化时必须持续更新本文件。**
>
> 状态标记：✅ 已实机确认 ｜ 📋 设计中（尚未实现） ｜ ❓ 待实机确认

---

## 1. 总体架构

整个系统采用 **Hierarchical Skill Architecture + Hybrid Agent Architecture + Event-driven Execution**。

```text
                    ┌─────────────────────────────┐
                    │        Environment          │
                    └──────────▲──────────────────┘
                               │
        ┌──────────────────────┴──────────────────────┐
        │              Perception                     │
        │      Camera / LiDAR / Mic / IMU             │
        └──────────────────────┬──────────────────────┘
                               ▼
        ┌─────────────────────────────────────────────┐
        │              Cognition                      │
        │        LLM / Planner / Memory               │
        └──────────────────────┬──────────────────────┘
                               ▼
        ┌─────────────────────────────────────────────┐
        │                Skills                       │
        │  Navigation / Follow / Search / Patrol      │
        └──────────────────────┬──────────────────────┘
                               ▼
        ┌─────────────────────────────────────────────┐
        │                Action                       │
        │      Motor / Speaker / Motion               │
        └──────────────────────┬──────────────────────┘
                               │
                               └──────────► Environment
```

**核心定义**：机器人整体是 Agent，不是"Agent 控制机器人"。

---

## 2. 分层 Skill 架构

| Level | 名称 | 内容 | 状态 |
|---|---|---|---|
| Level 0 | Hardware Capability | Motor / Camera / LiDAR / Microphone / Speaker / IMU，通过 `/dev/video0`、`/dev/ttyUSB0`、CAN、I2C、SPI、PWM、USB 访问。**不直接暴露给 LLM** | ✅ 硬件在生产商栈中可用 |
| Level 1 | Driver / Primitive | `set_motor_speed()` / `read_camera_frame()` / `read_lidar_scan()` / `read_imu()` / `play_audio()`。确定性、高频、靠近硬件、不依赖 Agent | 📋 |
| Level 2 | Control Skill | `move_forward()` / `move_backward()` / `rotate()` / `move_relative()` / `stop()`。内部含 PID、轮速控制、里程计、编码器反馈、麦轮运动学 | 📋 |
| Level 3 | Autonomous Skill | `navigate_to()` / `follow_person()` / `follow_line()` / `avoid_obstacle()` / `patrol_route()` / `return_home()`。Jetson 本地闭环，不需要 LLM 高频参与 | 📋 |
| Level 4 | Semantic Skill | `search_object("cup")` / `inspect_area("office")` / `find_person("Tom")` / `check_room("meeting_room")` | 📋 |
| Level 5 | Agent Task | 自然语言任务。Agent 只组合 Semantic Skill | 📋 |

**越往上**：语义更强、抽象程度更高、更依赖 Agent/LLM、时间尺度更慢。
**越往下**：更确定、更实时、更靠近硬件、不依赖 LLM。

---

## 3. 三层时间尺度

| 层级 | 时间尺度 | 主要职责 | LLM 参与 |
|---|---|---|---|
| Agent Layer | 秒级 / 事件驱动 | 理解、规划、重规划 | ✅ 是 |
| Skill / Autonomy Layer | 10~30 Hz 或异步任务 | 导航、跟随、搜索、巡检 | ❌ 否 |
| Control / Safety Layer | 20~100 Hz | 电机、PID、急停、避障 | 🚫 **禁止** |

示例 —— 用户说"去门口"：

```text
Agent 层：  navigate_to("door")            ← LLM 只做这一件事
                    ↓
此后几十秒全部由本地执行：
  SLAM → Localization → Path Planning → Obstacle Avoidance → Motion Control
                    ↓
Agent 只等待事件： ARRIVED / BLOCKED / FAILED / CANCELLED
```

---

## 4. Jetson 上运行的模块

Jetson Orin（L4T R36.4.3 / JetPack 6.x）是中央计算节点，承担：

| 模块 | 说明 | 状态 |
|---|---|---|
| 语音识别 ASR | `xf_mic_asr_offline`（厂商离线 ASR） | ✅ 包已存在，❓ 待实机验收 |
| Agent Runtime | Planner / Executor / Skill Registry / Event Manager / Memory / Safety Gateway | 📋 |
| 本地轻量 LLM | 后期加入，用于简单语言理解 / Tool Calling / 离线模式 | 📋 第一版不做 |
| 云端 LLM 调用 | 复杂任务规划、多步骤推理、异常处理 | 📋 |
| TensorRT 视觉推理 | GStreamer → CUDA → TensorRT → YOLO → Tracker | 📋 |
| ROS2 | Humble | ✅ |
| SLAM / Navigation | `slam` / `gmapping` / `navigation` / `teb_local_planner` | ✅ 包存在，❓ 待实机验收 |
| Skill Runtime | Skill 注册 / 参数校验 / 执行 / 状态管理 / Cancel / Timeout / Event 上报 | 📋 |
| Safety / Monitor | 急停 / 避障停止 / 限速 / Watchdog / 命令超时 / 电机超时 / 低电量保护 | 📋 |
| 系统状态管理 | jtop / tegrastats / 磁盘与性能模式 | ✅ |

---

## 5. 模块关系

### 5.1 自上而下的控制流

```text
LLM
 ↓
Agent Runtime            （Context / Tools / State / Memory / Execution / Safety）
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

### 5.2 自下而上的反馈流

```text
Camera / LiDAR / Odom / Motor
            ↓
      Robot Runtime
            ↓
          Event
            ↓
          Agent
```

**Robot Runtime 的职责**：

> 把 LLM 的抽象决策变成受控、可验证、可反馈的物理行为。

它是围绕 LLM 构建的 **Physical AI Harness**，向 LLM 提供：

```text
Context / Tools / State / Memory / Execution Environment / Feedback / Safety Constraints
```

### 5.3 各层关系总览

| 从 | 到 | 关系 |
|---|---|---|
| Agent | Skill | **只能**调用 Semantic Skill，不直接访问 Motor / GPIO / PWM / ROS Topic |
| Skill Manager | ROS2 | 通过 ROS2 topic / service / action 驱动底层 |
| ROS2 | Driver | 调用 Driver / Primitive |
| Driver | Linux Kernel | 通过 `/dev/*`、CAN、I2C、SPI、PWM、USB |
| Linux Kernel | Hardware | 真实电气与机械动作 |
| Safety Runtime | 所有层 | **否决权**，独立于 LLM |

---

## 6. Safety 架构

### 6.1 Tool / Skill Safety Gateway

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

示例：

```text
LLM:  move_relative(100m)
系统: REJECTED — distance exceeds safety limit
```

### 6.2 Safety Runtime（独立于 LLM）

```text
Emergency Stop / Obstacle Stop / Speed Limit
Watchdog / Command Timeout / Motor Timeout / Low Battery Protection
```

优先级：

```text
Safety > Control > Skill > Agent
```

> **Safety 具有最终否决权。**

---

## 7. Hybrid Command Router

| 类型 | 示例 | 路径 | 经 LLM |
|---|---|---|---|
| A. Safety Command | 停 / 急停 / 取消任务 / 别动 | Voice → Local Parser → Safety Runtime → Motor Stop | ❌ |
| B. Deterministic Command | 向前走 0.5 米 / 右转 30° / 回到原地 | Voice → Local Parser → Control Skill | ❌ |
| C. Agent Task | 去桌子旁找杯子，没有就去沙发旁找 | ASR → Agent Runtime → LLM Planner → Semantic Skill | ✅ |

### 7.1 语音链路

```text
Microphone → Wake Word → VAD → ASR → Text → Command Router
```

```text
Agent Response → TTS → Speaker
```

### 7.2 LLM 混合架构

```text
Task Router
 ├── Rule Engine       → stop / move / turn / status
 ├── Local Small LLM   → 简单语言理解 / Tool Calling / 离线模式（后期）
 └── Cloud LLM         → 复杂规划 / 条件任务 / 多步推理 / 异常处理
```

**断网降级**：机器人仍可移动、导航、避障、寻迹、跟随、视觉检测、停止，仅高级 Agent 能力降级。

---

## 8. Event-driven Agent

Skill 状态机：

```text
STARTED → RUNNING → ARRIVED / TARGET_FOUND / TARGET_LOST
                  → BLOCKED / FAILED / CANCELLED
```

执行模型：

```text
Agent → navigate_to("desk") → WAIT
                ↓
       [Robot Local Runtime 独立运行]
                ↓
             ARRIVED
                ↓
           Wake Agent
                ↓
       search_object("cup")
```

> 这是 **Event-driven Agent，而不是 Real-time LLM Controller**。

---

## 9. 关键能力的数据流

### 9.1 自动避障

```text
LiDAR → Local Costmap → Obstacle Detection → Local Planner → Velocity Command
```

普通障碍自动绕行。只有长时间无法通过时才 `BLOCKED → Agent → Re-plan`。

### 9.2 SLAM 与自主导航

```text
LiDAR → SLAM → Map → Localization → Global Planner → Local Planner → Controller
```

地图带语义标签：`desk` / `door` / `sofa` / `home` / `office` / `lab`。
Agent 只需 `navigate_to("desk")`。

### 9.3 目标跟随

```text
Camera → YOLO → Tracker → Target Position → Follower Controller
       → Obstacle Avoidance → Motor
```

Agent 只处理启动、停止以及 `TARGET_LOST` 后的任务决策。

### 9.4 寻迹

```text
Camera → ROI → Line Detection → Deviation → PID → Mecanum Control
```

可进一步封装为 `patrol_route()`。

### 9.5 视觉 Skill

```text
Camera → GStreamer → CUDA → TensorRT → YOLO → Tracker
```

对上层暴露：`detect_object()` / `detect_person()` / `capture_image()` / `get_target_position()` / `describe_scene()`。

---

## 10. 进程架构（设计）

```text
voice_service / agent_service / skill_service / vision_service
navigation_service / robot_service / monitor_service
```

| 语言 | 模块 | 理由 |
|---|---|---|
| C++ | `robot_service` / `navigation_service` / `safety` / 部分 vision runtime | 实时性、确定性、靠近硬件 |
| Python | `voice_service` / `agent_service` / LLM orchestration | 生态、LLM SDK、迭代速度 |

---

## 11. Agent Runtime 目录（计划）

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
Understand → Plan → Select Skill → Execute → Observe Event → Evaluate → Re-plan
```

> ⚠️ 上述目录**尚未创建**，仅为设计。实现时以实际代码为准并更新本文件。

---

## 12. 工作空间布局（实机）

```text
~/ros2_ws/            ✅ 厂商 ROS2 工作空间（系统 SDK，只读使用，不改动）
    src/{app,bringup,calibration,driver,example,interfaces,
         large_models,multi,navigation,openclaw_controller,
         peripherals,simulations,slam,xf_mic_asr_offline, ...}
~/third_party/        ✅ 6.4G，OpenCV / YDLidar-SDK / orbbec_ws / rtabmap_ws /
                         sherpa-onnx / yolo / aurora_ws / gmapping_ws ...
~/JetsonRobot/        ← 本仓库（文档 + 未来的 overlay 代码）

embodied_agent_ws/    📋 计划：本项目 Overlay Workspace，避免修改厂商包
```

**加载顺序**（以厂商既有顺序为准，不要随意改动）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```
