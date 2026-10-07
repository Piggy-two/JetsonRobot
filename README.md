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
| 底盘 | 麦克纳姆轮（厂商 ROS2 栈，含 `ros_robot_controller` / `controller` / `kinematics` / `servo_controller`）。命令链 `/cmd_vel` → `odom_publisher` → `/ros_robot_controller/set_motor`；`/odom` 30 Hz。✅ **2026-10-05 六向运动实测通过**（方向正确、轮速与解算逐位吻合），入口 `/cmd_vel` 厂商限幅 **±0.2 m/s / ±0.5 rad/s**。⚠️ **底盘无指令超时保护，停车必须显式发 0**（D-020）；同日**通信中断模拟**实测：指令断裂时底盘仍以 **105% 速率继续旋转**，断线后 `/odom` 照发而 `imu_raw`/`battery` 停发、**不自恢复**，且**本机无物理急停**（D-021） |
| 相机 | **单目** USB 摄像头（UVC `32e6:9005`，YUYV 640×480，`/dev/video0`）。全系统**无深度相机**，视觉基线见 `docs/DECISIONS.md` D-017；已按 D-019 走厂商 `usb_cam` 分支，`/depth_cam/rgb0/image_raw` 已出图（1 个发布者）。✅ **2026-10-05 硬件验收通过**：**端到端延迟 ≈ 一帧（20~45 ms）**（用 `v4l2` 的 `brightness` 当"传感器端打光"探针实测，工装 `tools/camera_latency_probe.py`）。⚠️ **图像 `header.stamp` 比真实采集时刻早 0.72 s**（内容新鲜、时间戳陈旧，须在自建 Driver 层重新打时间戳，#23 / **D-022**）；**实际速率约 22.6 Hz 且随负载变化**（设备只支持 30/25/20/15/10/5 六档，属主机侧丢帧）；`frame_id=camera` **不在 TF 树**（#16，补救须落在 Overlay）。**2026-10-06 第五轮**：Driver 的 `pipeline_latency` 由估计的 45 ms **实测标定为 110 ms**；并查清厂商戳陈旧量**不恒定**（本轮 **338.6 s**，随每次开机的时钟跳变，#24）——**只有 `usb_cam` 一个节点受影响**（`/scan`、`/odom`、`/tf` 均正常）。**第六轮**：TF 朝向**实机校验通过**（补的 `camera_link0 → camera` 无镜像、无滚转）；Driver 已无遗留项 |
| 雷达 | **LD19**（`ldlidar_stl_ros2`，230400，`/dev/lidar`）。✅ **`/scan` 实测通过**：10.00 Hz、360°、502~505 点/帧、有效回波 93.5~97.0%、`frame_id=lidar_frame`，TF 已就位（见 D-018 与 `docs/PROJECT_STATUS.md` §7） |
| 语音 | ✅ **2026-10-05 硬件验收通过**（`docs/DECISIONS.md` **D-022**）：控制串口 `/dev/ttyCH341USB1`（USB 路径 `1-2.4.1`）可用；声卡 0（`0c76:161f`，`1-2.4.2`，与串口同一个 Hub）**录音 + 播放均通过**（用户确认听到提示音，采集侧削顶 0%）；`/dev/ring_mic` 缺失的根因是**过期 udev 路径**（规则写 `1-2.3.1`，实际 `1-2.4.1`），**已修正**，厂商 `mic_init.launch.py` 的 `awake_node.py` / `asr_node.py` / `voice_control` 正常启动。**2026-10-06 第七轮实测**：麦克风**确认可用**（声学回路：播 440 Hz，录音中该分量涨三个数量级）；链路查清是**全本地**（唤醒在环形麦硬件做、识别走讯飞**离线** SDK + 本地 BNF 语法，**不经云端不经 LLM**），**安全词「停下」/`stop` 就在命令词表里**。⚠️ **但「唤醒」实测不通**：人声与**四个本地 TTS 合成的候选唤醒词**都唤不醒（**#25**）；且执行侧有重大缺陷——「停下」**没有对应处理分支**（现在能停是掉进默认零 Twist **碰巧成立**），且走**无限幅**的 `/controller/cmd_vel`（**#26**）。配置 `MIC_TYPE=xf`；`ASR_MODE=online` ⚠️ **该变量只被厂商 `large_models` 读取，麦克风栈不读它**，故**不再是"断网不可用"的理由** |
| 厂商工作空间 | `~/ros2_ws`（6 类 src 子包）、`~/third_party`（OpenCV / YDLidar-SDK / orbbec / rtabmap / sherpa-onnx / yolo 等） |

> ⚠️ 厂商 `ros2_ws` 与 `third_party` 作为**系统 SDK** 使用，保持不改动。本项目代码放在独立的 Overlay Workspace（计划名 `embodied_agent_ws`）。

---

## 4. 快速启动方法

### 4.1 环境加载

```bash
# 厂商既有 source 顺序（以实机为准，不要随意改动顺序）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 本项目 Overlay（必须【在厂商之后】source）
source ~/JetsonRobot/embodied_agent_ws/install/setup.bash

echo $ROS_DISTRO        # 期望 humble
which ros2
```

> ⚠️ 上面这些命令**必须在 `bash` 里执行**。zsh 没有 `BASH_SOURCE`，ROS 的
> `setup.bash` 会定位失败（现象与原因见 `docs/DEV_NOTES.md` 坑 5）。

若业务代码有改动，先在 `embodied_agent_ws/` 下 `colcon build --symlink-install`。

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

> 实际推进：**LiDAR 与 TF ✅ 已通过**（2026-09-28，`/scan` 10.00 Hz / 360°）；**底盘运动 ✅ 已通过**（2026-10-05，六向方向正确 + 轮速与解算逐位吻合，限速 ±0.2 m/s）；**通信中断 ✅ 已模拟完成**（2026-10-05，D-020 / D-021）；**相机 ✅ 硬件通过**（2026-10-05：端到端延迟 ≈ 一帧（20~45 ms），但 `header.stamp` 早 0.72 s 待收口，**D-022**）；**语音盒 ✅ 音频硬件通过**（2026-10-05：控制串口 / 录音 / 播放三项，**D-022**）；**Overlay Workspace ✅ 已建立 + 首个业务代码节点 ✅ 已落地**（2026-10-05 第四轮：`embodied_agent_ws/`，内含相机 Driver `embodied_camera_driver`，在同一节点内收口**重新打时间戳 #23** 与**补发 TF `camera_link0 → camera` #16**，**D-023**——项目**自此有业务代码**）。**2026-10-06 第五轮**：相机 Driver 的 `pipeline_latency` 标定为 **110 ms**（D-023 补充）；厂商图像戳的陈旧量被查清**并非恒定**（本轮 338.6 s，随每次开机的时钟前拨而变，**#24**）。**2026-10-06 第六轮**：**相机 TF 朝向实机校验通过**（运动质心 513 次检出全在画面右侧、0 次在左）—— **相机 Driver 至此无遗留项**。**2026-10-06 第七轮（语音软件侧验收，未完成）**：麦克风**确认可用**（声学回路实测）；语音链路查清是**全本地、不经云端不经 LLM**，**安全词「停下」/`stop` 在命令词表里**；⚠️ **但「唤醒」实测不通**（人声 + 四个本地 TTS 合成的候选唤醒词都唤不醒，**#25**），且执行侧「停下」**没有对应分支、走无限幅话题**（**#26**）。**剩余**：**语音唤醒卡点定位**（需人工批准停厂商 ASR 三节点）、**急停链路**（用户已明确暂缓）。遥控优先级一项用户已表示当前无需求。
>
> 🔴 已知硬约束（Phase 1+ 设计时不得违反）：**底盘无指令超时保护，停车必须显式持续发 0**；**存活判据只能用 `imu_raw`/`battery`**（`/odom` 断线照发）；**本机没有物理急停**；**图像 `header.stamp` 不可信，且陈旧量不是常数**（2026-10-06 实测 338.6 s，随每次开机的时钟前拨而变，#24；须用**本节点自己的实时钟**重新打时间戳）。另：**相机帧率既非 30 fps 也非恒定**，且**时间戳修正量 `pipeline_latency` 也随负载变化**（2026-10-05 约 30 ms → 2026-10-06 为 110 ms），Phase 4 定帧率/延迟预算时须实测并做降级。

> ⚠️ 任何运动测试均必须保留急停、限速和人工看护。**由于本机无物理急停，测试时人必须能直接断电。**

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
| **Phase 0** | 环境与硬件启动验收（底盘 / LiDAR / 相机 / 麦克风 / 扬声器 + 接口清单） | 🚧 **进行中**（LiDAR ✅ / 底盘运动 ✅ / 通信中断 ✅ / **相机 ✅ 已完全收口** / **语音音频硬件 ✅**；⚠️ **语音唤醒实测不通**、急停链路用户已暂缓） |
| **Phase 1** | Driver / Primitive（Camera / Motor / LiDAR Driver） | 🚧 **已开始** —— **相机 Driver ✅ 无遗留**（`embodied_camera_driver`，D-023）；**电机 Driver ✅ 已落地、干跑 + 链路失联验证通过**（`embodied_motor_driver`，**D-025**，含**失联检测/告警/恢复后拒绝静默复动**；**默认 `dry_run=true` 不驱动底盘**；真机运动测试待人工看护时做）；**LiDAR Primitive ✅ 已落地**（`embodied_lidar_driver`，**D-028** —— 厂商 `/scan` 本身是对的，故做**查询原语**而非收口：扇区最近距离 / 通畅判定；对真实雷达独立重算交叉验证**逐位一致**）。**Phase 1 三个 Driver 齐了** |
| Phase 2 | Robot Control（`move_forward` / `rotate` / `move_relative` / `stop`） | 🚧 **已开始** —— `embodied_control_skills` 落地 `move_relative` / `rotate` / `stop`（**D-026**，Service 接口 + 项目自己的 `.srv`），**干跑验证 16/16 通过**（用**速度积分**量位移，车不动）。⚠️ **第一版是开环**：`success` = "速度按时长发完了"，**不是走到位**。**2026-10-07 起控制动作多走一跳 `embodied_skill_gateway`**（D-029/D-031：六项检查，**没有旁路**） |
| Phase 3 | Autonomous Skills（SLAM / Navigation / 避障 / `follow_person` / `follow_line`） | ⬜ 未开始 |
| Phase 4 | Semantic Skills（`search_object` / `inspect_area` / `patrol_route` / `return_home`） | ⬜ 未开始 |
| Phase 5 | Voice System（Wake Word / VAD / ASR / TTS） | ⬜ 未开始 |
| Phase 6 | Agent Runtime（Planner / Executor / Skill Registry / Event Manager / Memory / Safety Gateway） | 🚧 **Safety 部分提前开始**（**D-027**：`embodied_safety_runtime` 第一版——**本地安全指令通路**（不经过 LLM、不经过厂商节点）+ **独立零速通道**（不依赖 Motor Driver 存活）+ **Motor Driver 停更看门狗**（它挂了自动接管、未恢复时拒绝解除）。因为**本机没有物理急停**，#22 这条约束今天就压着，不必等 Agent 层建好）。**2026-10-07 又落地两块**：**`embodied_skill_gateway`（D-029~D-032）** 与 **`embodied_command_router`（D-006）** —— 架构里第一次有了"Agent 那一侧"：命令 → 解析 → **网关六项检查** → 技能，**离线 143 项单测 + dry-run 端到端 16/16 全过**。⚠️ **但车全程没动**，进「未经验证的代码」清单。**其余（Planner / Executor / Event Manager / Memory）属第二批** |
| Phase 7 | Hybrid LLM（Rule Engine + Cloud LLM + 预留 Local Small LLM） | ⬜ 未开始 |

> **仓库现状**：包含设计方案（`docs/plan.md`）、Phase 0 验收工装（`tools/`）、以及 **Overlay 业务代码工作区 `embodied_agent_ws/`**（首个节点为相机 Driver `embodied_camera_driver`，2026-10-05 落地，**D-023**；第二个是电机 Driver `embodied_motor_driver`，2026-10-06 落地，**D-025**；随后是 Control Skill `embodied_control_skills` 与接口包 `embodied_skills_interfaces`，**D-026**；再后是 Safety Runtime `embodied_safety_runtime`，**D-027**；以及 LiDAR Primitive `embodied_lidar_driver`，**D-028**；**2026-10-07 再落地上层两块**——Skill 网关 `embodied_skill_gateway`（**D-029~D-032**，含数据驱动注册表与 8 状态机）与命令路由器 `embodied_command_router`（**D-006**））。**共 8 个包**。后续业务代码按上表顺序在该工作区内引入。

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
| [`docs/DEV_NOTES.md`](docs/DEV_NOTES.md) | **开发思路、踩坑与解法** —— 方法论（如何给测不准的量造独立基准）与可复现的教训 |
| [`embodied_agent_ws/`](embodied_agent_ws/) | **本项目 Overlay Workspace**（业务代码在此；`build/ install/ log/` 不进 Git） |
| [`CLAUDE.md`](CLAUDE.md) | Claude Code 长期开发规则 |
| [`tools/`](tools/) | Phase 0 验收工装：`phase0_chassis_motion_acceptance.py`（底盘六向运动验收）、`lidar_rotation_probe.py`（用 LiDAR 独立测原地旋转角速度，丢帧免疫）、`camera_latency_probe.py`（用 `v4l2` 控制项当"世界端探针"，分离相机**端到端延迟**与**时间戳偏移**；`--topic` 可测任意话题（含 Overlay 输出）、`--repeat` + `--jitter` 做**多事件取均值**以标定 `pipeline_latency`）、`lidar_environment_probe.py`（把 LiDAR 的距离-角度剖面量化成表 + 柱状图，用于判定近场回波是**机器人自身结构**还是**外部环境** —— 判据是"换个摆位再测一次，看回波跟不跟着车走"）、`mic_serial_probe.py`（探测讯飞环形麦**控制串口** `/dev/ring_mic` 是否还活着：握手应答 + 纯监听模式。厂商 `awake_node` 的握手等待**没有超时**，它一卡住外面完全看不出来，这个工装把它变成明确读数 —— ⚠️ **收到 0 字节 ≠ 波特率不对**）、`upper_layer_dryrun_acceptance.py`（**上层干跑端到端验收**：把 `/embodied/motor/cmd_vel_dryrun` 上的速度**对时间积分**，于是"车一动不动"也能量出"被命令走了多远"—— 16 项判据含**事件契约**与"被拒请求不得产生任何事件"）。**验收用，不是运行时组件** |

---

## 7. 一句话版本

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**
