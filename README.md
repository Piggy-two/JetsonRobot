# JetsonRobot — 基于 Jetson Orin 的混合式语音具身智能机器人平台

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**

---

## 1. 项目目标

在 Jetson Orin 上构建一套 **本地自治 + LLM Agent + 分层 Skill + 实时安全控制** 的具身智能机器人系统。

机器人通过语音接受自然语言任务，由 LLM 负责高层任务理解与规划；导航、避障、寻迹、目标跟随等能力作为本地 Skill 独立闭环执行；Linux Kernel、Driver 与硬件接口提供最底层的确定性能力；Safety Runtime 独立于大模型运行。

核心思想不是"Jetson 上跑一个 Agent 去控制小车"，而是：

> **整台机器人本身就是一个 Embodied Agent。**

| 组成 | 角色 |
|---|---|
| Jetson Orin | 机器人的计算与认知平台 |
| LLM | 高级认知与任务规划模块 |
| Robot Runtime | 承载 Context / Tools / State / Memory / Execution / Feedback / Safety 的 Physical AI Harness |
| Skill Runtime | 将硬件能力逐层封装成可被 Agent 调用的机器人技能 |
| Linux Kernel / Driver | 真实硬件能力的基础支撑 |
| Motor / Camera / LiDAR / Mic | 机器人的身体与感知器官 |

---

## 2. 核心功能

### 2.1 分层 Skill 架构

```text
LLM / Agent
    ↓
Semantic Skill          语义任务：search_object / inspect_area / find_person
    ↓
Autonomous Skill        自主能力：navigate_to / follow_person / follow_line / avoid_obstacle
    ↓
Control Skill           控制能力：move_forward / rotate / move_relative / stop
    ↓
Driver / Primitive      软件原语：set_motor_speed / read_camera_frame / read_lidar_scan
    ↓
Linux Driver
    ↓
Hardware                Motor / Camera / LiDAR / Mic / Speaker / IMU
```

### 2.2 三层时间尺度

| 层级 | 时间尺度 | 主要职责 | LLM 参与 |
|---|---|---|---|
| Agent Layer | 秒级 / 事件驱动 | 理解、规划、重规划 | 是 |
| Skill / Autonomy Layer | 10~30 Hz 或异步任务 | 导航、跟随、搜索、巡检 | 否 |
| Control / Safety Layer | 20~100 Hz | 电机、PID、急停、避障 | **禁止** |

### 2.3 Hybrid Command Router

| 命令类型 | 示例 | 处理路径 | 经过 LLM |
|---|---|---|---|
| Safety Command | "停" / "急停" / "别动" | Local Parser → Safety Runtime → Motor Stop | **否** |
| Deterministic Command | "向前走 0.5 米" / "右转 30°" | Local Parser → Control Skill | **否** |
| Agent Task | "去桌子旁边找杯子，没有就去沙发附近找" | Agent Runtime → LLM Planner → Semantic Skill | 是 |

### 2.4 离线降级能力

断网后机器人仍可执行：移动、导航、避障、寻迹、跟随、视觉检测、停止 —— 仅高级 Agent 能力降级。

### 2.5 安全设计

`Safety > Control > Skill > Agent`，**Safety 具有最终否决权**。

LLM 无法直接控制电机；所有动作必须经过 Tool / Skill Safety Gateway（Schema Validation / Permission / Range Check / Timeout / Cancel / Result Validation）与 Skill Manager。

---

## 3. 硬件与软件环境（当前实机确认）

| 项目 | 实测结果 |
|---|---|
| 计算平台 | NVIDIA Jetson Orin（L4T **R36.4.3**，JetPack 6.x） |
| OS | Linux `5.15.148-tegra` (aarch64) |
| ROS2 | **Humble**（`/opt/ros/humble`） |
| 底盘 | 麦克纳姆轮（厂商 ROS2 栈，含 `ros_robot_controller` / `controller` / `kinematics` / `servo_controller`）。基线实测命令链：`/cmd_vel` → `odom_publisher` → `/ros_robot_controller/set_motor`；`/odom` 30 Hz |
| 相机 | Orbbec（`orbbec_camera` / `orbbec_camera_msgs`）。基线实测：仅 `/depth_cam/rgb0/image_raw` 且**无数据流**，待验收 |
| 雷达 | 候选：`ydlidar_ros2_driver` / `sllidar_ros2` / `sclidar_ros2` / `ldlidar_stl_ros2` / `Aurora930`。基线实测：ROS 图中**无 `/scan`**，**未就绪**，需先查设备连接 |
| 语音 | `xf_mic_asr_offline`（离线 ASR） |
| 厂商工作空间 | `~/ros2_ws`（6 类 src 子包）、`~/third_party`（OpenCV / YDLidar-SDK / orbbec / rtabmap / sherpa-onnx / yolo 等） |

> ⚠️ 厂商 `ros2_ws` 与 `third_party` 作为**系统 SDK** 使用，保持不改动。本项目代码放在独立的 Overlay Workspace（计划名 `embodied_agent_ws`）。

---

## 4. 快速启动方法

### 4.1 环境加载

```bash
# 厂商既有 source 顺序（以实机为准，不要随意改动顺序）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo $ROS_DISTRO        # 期望 humble
which ros2
```

### 4.2 基线检查

```bash
jtop                                       # Jetson 状态
df -h /                                    # 磁盘（当前根分区紧张，见下）
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### 4.3 启动与验收

单设备启动测试与接口验收流程见 [`docs/plan.md`](docs/plan.md) 第 24 章，以及 [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md) 的当前进度。

**验收顺序**：底盘与急停 → LiDAR 与 TF → 里程计与定位 → 建图 / 保存地图 → 导航到测试点 → 静态障碍物避障 → 相机、语音与导航联调。

> ⚠️ 任何运动测试均必须保留急停、限速和人工看护。

### 4.4 磁盘容量状态

根分区 **116G，已用 57G，可用 55G（52%）** —— 2026-09-23 已在线扩容（65G → 116G），**磁盘阻塞解除**。

部署 PyTorch、模型权重、TensorRT Engine、ROS bag 或 Docker 镜像前无需再扩容，但需注意：

- 磁盘仍有约 **119G 未纳入 GPT**（`last-lba=250069646` 限制）。需要时再改 GPT 几何，属**高风险操作**：动手前必须 `sgdisk -b` + `sfdisk -d` 备份，并保留 p1 的 `PARTUUID`（`root=PARTUUID` 写在 `/boot/extlinux/extlinux.conf`，GUID 变更将导致无法启动）。
- 内存仅 **7.4Gi（8GB 版 Orin）**，`/swapfile` 8G 是实际安全余量，不要随意缩小。
- `/opt/ota_package` 是**引导链 OTA capsule 载荷**（非残留），`~/.ollama/models` 是 Phase 7 本地小模型候选资产 —— 均不得当作缓存清理。

---

## 5. 当前完成度

**项目阶段：Phase 0 — 环境与硬件启动验收（进行中）**

| 阶段 | 内容 | 状态 |
|---|---|---|
| **Phase 0** | 环境与硬件启动验收（底盘 / LiDAR / 相机 / 麦克风 / 扬声器 + 接口清单） | 🚧 **进行中** |
| Phase 1 | Driver / Primitive（Camera / Motor / LiDAR Driver） | ⬜ 未开始 |
| Phase 2 | Robot Control（`move_forward` / `rotate` / `move_relative` / `stop`） | ⬜ 未开始 |
| Phase 3 | Autonomous Skills（SLAM / Navigation / 避障 / `follow_person` / `follow_line`） | ⬜ 未开始 |
| Phase 4 | Semantic Skills（`search_object` / `inspect_area` / `patrol_route` / `return_home`） | ⬜ 未开始 |
| Phase 5 | Voice System（Wake Word / VAD / ASR / TTS） | ⬜ 未开始 |
| Phase 6 | Agent Runtime（Planner / Executor / Skill Registry / Event Manager / Memory / Safety Gateway） | ⬜ 未开始 |
| Phase 7 | Hybrid LLM（Rule Engine + Cloud LLM + 预留 Local Small LLM） | ⬜ 未开始 |

> **仓库现状**：目前仅包含设计方案（`docs/plan.md`），**尚无业务代码**。代码将在 Phase 0 验收通过后按上表顺序引入。

**第一版明确不做**：机械臂、多机器人、强化学习导航、复杂 RAG、大型本地 VLM、自动充电。

---

## 6. 文档索引

| 文件 | 内容 |
|---|---|
| [`docs/plan.md`](docs/plan.md) | 完整架构方案（原始设计文档，29 章） |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 系统架构、模块关系、数据流与控制流 |
| [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md) | 当前进度、已知问题、下一步计划（**恢复项目状态的入口**） |
| [`docs/DEVELOPMENT_LOG.md`](docs/DEVELOPMENT_LOG.md) | 按日期的重要开发记录 |
| [`docs/DECISIONS.md`](docs/DECISIONS.md) | 重要技术决策及原因 |
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 长期开发规则 |

---

## 7. 一句话版本

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**
