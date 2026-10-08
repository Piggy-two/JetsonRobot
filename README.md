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

> 实际推进：**LiDAR 与 TF ✅ 已通过**（2026-09-28，`/scan` 10.00 Hz / 360°）；**底盘运动 ✅ 已通过**（2026-10-05，六向方向正确 + 轮速与解算逐位吻合，限速 ±0.2 m/s）；**通信中断 ✅ 已模拟完成**（2026-10-05，D-020 / D-021）；**相机 ✅ 硬件通过**（2026-10-05：端到端延迟 ≈ 一帧（20~45 ms），但 `header.stamp` 早 0.72 s 待收口，**D-022**）；**语音盒 ✅ 音频硬件通过**（2026-10-05：控制串口 / 录音 / 播放三项，**D-022**）；**Overlay Workspace ✅ 已建立 + 首个业务代码节点 ✅ 已落地**（2026-10-05 第四轮：`embodied_agent_ws/`，内含相机 Driver `embodied_camera_driver`，在同一节点内收口**重新打时间戳 #23** 与**补发 TF `camera_link0 → camera` #16**，**D-023**——项目**自此有业务代码**）。**2026-10-06 第五轮**：相机 Driver 的 `pipeline_latency` 标定为 **110 ms**（D-023 补充）；厂商图像戳的陈旧量被查清**并非恒定**（本轮 338.6 s，随每次开机的时钟前拨而变，**#24**）。**2026-10-06 第六轮**：**相机 TF 朝向实机校验通过**（运动质心 513 次检出全在画面右侧、0 次在左）—— **相机 Driver 至此无遗留项**。**2026-10-06 第七轮（语音软件侧验收，未完成）**：麦克风**确认可用**（声学回路实测）；语音链路查清是**全本地、不经云端不经 LLM**，**安全词「停下」/`stop` 在命令词表里**；⚠️ **但「唤醒」实测不通**（人声 + 四个本地 TTS 合成的候选唤醒词都唤不醒，**#25**），且执行侧「停下」**没有对应分支、走无限幅话题**（**#26**）。**剩余**：**语音唤醒卡点定位**（需人工批准停厂商 ASR 三节点）、**急停链路**（用户已明确暂缓）。遥控优先级一项用户已表示当前无需求。**2026-10-07 第七轮**：**避障接进 Safety Runtime**（**D-036**，只停不绕行）—— 离线单测 37 项 + 干跑验收 32/32 + **真雷达链路验证**（状态读出 **0.154 m**、静止**不误锁**、假速度触发 `obstacle:0.16m@+0deg`）。⚠️ 同轮从代码里读出一条硬约束：**`/cmd_vel` 上没有仲裁**（#28）—— **同日第八轮已把它量出来**（车不动）：下游 `stop` 调得通 → 锁存后 **121 帧全零**；**调不通 → 非零 66%（= 速率比 2/3）且不衰减**。⇒ 否决权当时是"咨询性"的：对方配合才作数。**同日第九轮已修（D-037）**：Motor Driver 改为**直接读安全层状态**、锁存期间自己输出零，**安全层不在跑则拒绝运动**（状态码 5 = `safety_blocked`）；复测同一档 **66% → 0%**。⚠️ 代价：Motor Driver **多了一个依赖**（要有 Safety 在跑），现有"只跑 Motor Driver"的验收工装要一起起 Safety 且关掉避障守卫。**同日第十轮架空实测（四轮离地）把物理效果也验了**：`veto` **11/11** + `guard` **4/4** —— 下游 `stop` **故意不可达**时运动中急停**底盘真的停了**（Motor Driver 自己的锁存标志 = 0，证明停靠的是"读到的状态"）、解除后**不静默复动**、避障以真距离 `0.16m@+0deg` 锁存并停住。⚠️ 边界：验的是"停得住"，**不是"走到位了"**；架空时轮子**不承重**。**2026-10-07 第十二轮：Phase 7 第一跳落地（D-038）** —— Planner 接上**云端 LLM**：规则表没命中时由 LLM **选一个** task-tier 技能（或明确拒绝）。**单步是刻意的**（注册表现在只有一个 task-tier 技能）。三条取舍：**规则命中但自己不合法时绝不问 LLM**、**两条路共用一个校验口**（严格度一致是结构事实）、**失败分类绝不静默降级**（"没开"和"不会"必须分开说）。**默认关闭**（会把用户的话发往第三方）。**验证**：离线单测 90 项 + 端到端 **15/15**（对假端点，**不联网**）+ 回归 23/23。⚠️ **未接真端点**。**2026-10-08 第二轮：安全链的第四段验收 `--phase direction` 在地面上跑通 8/8** —— 正前方放**推不走**的物体（0.191 m），**命令它朝障碍走 → 守卫拦下、轮速指令归零（0.0000）、底盘重新使能**；同一会话改成**往通畅的左方走 0.3 m**（0.991 m）→ **守卫不拦、轮子真的转了**（0.5968 rps、IMU 基线 **59 倍**）。**这一趟找出了守卫自己的一个真缺陷（D-039）**：守卫**只存回答、不存"这回答是问哪个方向的"**，方向一变就把旧方向的读数当新方向的用 —— 修前同一序列会打出 `obstacle:0.15m@+90deg`（那个 0.15 其实是**前方**的距离）而**假锁存**；⚠️ 它与坑 25 不同：那次是"压根没有回答"，这次是"**有一个别人问题的答案**"，所有"没有数据"的防线一条都不会响。**另有一条物理结论（坑 29）**：阈值是**从雷达算的**，而撞上去的是**车头** —— 由 `base_link_mec.stl` 包围盒算出**车体最前端在雷达前方 13.1 cm**，故 0.225 m 的阈值给车头只剩 **9.4 cm**，再扣 **2.2 cm** 蠕动 ⇒ **当前 `min_range = 0.20 m` 护不住车体自己**（地面验收里正是这样把 0.20 m 外的瓶子撞倒了）。⇒ **标定阈值时必须把车体前伸量算进去**，这是下一步第一件事。**2026-10-08 第三轮：四轮着地的整套验收跑完（五个相位全过，`tools/ground_motion_acceptance.py`）** —— **「走到位了没有」第一次有独立证据**（基准 = 原始 `/scan` 量参照面距离变化；旋转用 **IMU 陀螺**）。**位移 16/16**：请求 0.15/0.25/0.40/−0.40 m ⇒ 雷达测得 **0.1480/0.2410/0.3890/−0.4010 m**（误差 **−0.2~−1.1 cm**）—— ⇒ **开环的 `success` 其实挺准，而 `/odom` 反而偏得更多**（与雷达差 0.5~2 cm）；**旋转 3/3**（请求 90°、IMU 得 **85.7°**）⇒ 平动与转动**都系统性偏短 1~6%**（开环启动延迟）。**停车 6/6（两档速度，标定见 D-040）**：0.15 m/s 档**喊停后又跑 5.0 cm、车头只剩 3.9 cm**；0.20 m/s 档 5.5 cm / 11.4 cm，**蠕动与速度基本无关**。**链路 2/2**（雷达实测 0.306 / 0.485 m）；**闭环 3/3**（`BLOCKED` 分支在真障碍前如实报出、**自报 0.2000 m 与雷达逐位吻合**）；**Agent Runtime 真机端到端**（8 状态机跳 RUNNING → BLOCKED，雷达确认 0.100 m）。⇒ **「未经验证的代码」清单里四行同时转正**。**同日第四轮：把 D-040 落地** —— `obstacle_min_range` **0.20 → 0.30**（含车体前伸量 0.131 m + 锁存前蠕动 0.055 m），**0.15 m/s 档车头余量 3.9 → 9.3 cm**；真机回归 `--phase direction` **8/8** + `--phase stop` **3/3 × 两档**。⚠️ **副作用**：本机全部可达速度上 `速度 × lookahead ≤ 0.30` ⇒ **`lookahead` 当前惰性**，阈值实际是固定的 0.30 m（TTC 形式保留，提高限速/换更快底盘会重新起作用）。✅ 同时**堵掉"判据与被测物各用一个数"的复发路径**：方向性工装现在**读守卫的 yaml 核对**，对不上当场拒测（正反两向都验过）。
>
> 🔴 已知硬约束（Phase 1+ 设计时不得违反）：**底盘无指令超时保护，停车必须显式持续发 0**；**存活判据只能用 `imu_raw`/`battery`**（`/odom` 断线照发）；**本机没有物理急停**；**`/cmd_vel` 上没有仲裁**（Safety 的零速流与 Motor Driver 的指令流是**并列发布者**，厂商端"收到一条转一条" —— "我发了零"**不等于**"零说了算"。**2026-10-07 已实测**：下游 `stop` 调得通时锁存后 **121 帧全零**；**调不通时非零 66%（= 速率比 2/3）且不衰减**，即"抖着走"。⇒ 否决权当时是"咨询性"的。**2026-10-07 已修复（D-037）**：Motor Driver **直接读安全层状态**、锁存期间自己输出零，**安全层不在跑则拒绝运动**；复测同一档 **66% → 0%**。#28 / DEV_NOTES 坑 23）；**图像 `header.stamp` 不可信，且陈旧量不是常数**（2026-10-06 实测 338.6 s，随每次开机的时钟前拨而变，#24；须用**本节点自己的实时钟**重新打时间戳）。另：**相机帧率既非 30 fps 也非恒定**，且**时间戳修正量 `pipeline_latency` 也随负载变化**（2026-10-05 约 30 ms → 2026-10-06 为 110 ms），Phase 4 定帧率/延迟预算时须实测并做降级。

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
| **Phase 0** | 环境与硬件启动验收（底盘 / LiDAR / 相机 / 麦克风 / 扬声器 + 接口清单） | ✅ **已完成（2026-10-08，带两条例外）**（LiDAR ✅ / 底盘运动 ✅ / 通信中断 ✅ / **相机 ✅ 已完全收口** / **语音音频硬件 ✅**；⚠️ **语音唤醒实测不通**、急停链路用户已暂缓） |
| **Phase 1** | Driver / Primitive（Camera / Motor / LiDAR Driver） | 🚧 **已开始** —— **相机 Driver ✅ 无遗留**（`embodied_camera_driver`，D-023）；**电机 Driver ✅ 已落地、干跑 + 链路失联 + 安全层否决验证通过**（`embodied_motor_driver`，**D-025** + **D-037**，含**失联检测/告警/恢复后拒绝静默复动**、**直接读安全层状态并在它锁存时自己输出零**（**安全层不在跑则拒绝运动**）；**默认 `dry_run=true` 不驱动底盘**；真机运动测试待人工看护时做）；**LiDAR Primitive ✅ 已落地**（`embodied_lidar_driver`，**D-028** —— 厂商 `/scan` 本身是对的，故做**查询原语**而非收口：扇区最近距离 / 通畅判定；对真实雷达独立重算交叉验证**逐位一致**）。**Phase 1 三个 Driver 齐了** |
| Phase 2 | Robot Control（`move_forward` / `rotate` / `move_relative` / `stop`） | 🚧 **已开始** —— `embodied_control_skills` 落地 `move_relative` / `rotate` / `stop`（**D-026**，Service 接口 + 项目自己的 `.srv`），**干跑验证 16/16 通过**（用**速度积分**量位移，车不动）。⚠️ **第一版是开环**：`success` = "速度按时长发完了"，**不是走到位**。**2026-10-07 起控制动作多走一跳 `embodied_skill_gateway`**（D-029/D-031：六项检查，**没有旁路**） |
| Phase 3 | Autonomous Skills（SLAM / Navigation / 避障 / `follow_person` / `follow_line`） | 🚧 **已开始，两个 task-tier 技能**（**D-034** + **D-042**）：`advance_until_blocked`（前进到被挡，**有地面证据**）与 **`turn_until_clear`**（转到通畅，2026-10-08，**只干跑验过**）—— 都是**闭环**，**8 状态机已跑起来**。⚠️ `navigate_to` 类能力仍缺 **SLAM / 导航栈**（本机有包但从未验收） |
| Phase 4 | Semantic Skills（`search_object` / `inspect_area` / `patrol_route` / `return_home`） | ⬜ 未开始 |
| Phase 5 | Voice System（Wake Word / VAD / ASR / TTS） | ⬜ 未开始 |
| Phase 6 | Agent Runtime（Planner / Executor / Skill Registry / Event Manager / Memory / Safety Gateway） | 🚧 **Safety 部分提前开始**（**D-027**：`embodied_safety_runtime` 第一版——**本地安全指令通路**（不经过 LLM、不经过厂商节点）+ **独立零速通道**（不依赖 Motor Driver 存活）+ **Motor Driver 停更看门狗**（它挂了自动接管、未恢复时拒绝解除）。因为**本机没有物理急停**，#22 这条约束今天就压着，不必等 Agent 层建好）。**2026-10-07 又落地两块**：**`embodied_skill_gateway`（D-029~D-032）** 与 **`embodied_command_router`（D-006）** —— 架构里第一次有了"Agent 那一侧"：命令 → 解析 → **网关六项检查** → 技能，**离线 143 项单测 + dry-run 端到端 16/16 全过**。⚠️ **但车全程没动**，进「未经验证的代码」清单。**2026-10-07 第四轮把最后一块补上**：**`embodied_autonomous_skills`（D-034，第一个 task-tier 技能）** 与 **`embodied_agent_runtime`（D-035：Executor / Event Manager / Memory）** —— **8 状态机第一次真的跑起来**，命令 → 网关 → task-tier 技能 → 事件 → **唤醒 Agent** 这条链端到端跑通（**201 项离线单测 + dry-run 23/23**）。⚠️ **Planner 仍是 stub**（规则表默认空 ⇒ 一切拒绝），**LLM 规划与重规划属 Phase 7**。⚠️ **车仍全程没动**。**2026-10-07 第七轮补上避障**（**D-036**，⚠️ 它要求 Safety 在跑、且做运动学验收时要关掉它）：`embodied_safety_runtime` 新增**避障守卫** —— 按「**被命令的运动方向**」问 LiDAR 原语，阈值 = `速度 × 预留时间`（**TTC 式，不是固定距离**，直接回应 D-028 的"任何固定阈值都会被场地否决"），**触发即锁存**。**只做"停"不做绕行**。⚠️ 顺带查出一条硬约束：**`/cmd_vel` 上 Safety 与 Motor Driver 没有仲裁**（#28）—— "我发了零"不等于"零说了算"。**已实测**（车不动）：下游 `stop` 调得通 → 锁存后 **121 帧全零**；**调不通 → 非零 66% 且不衰减**。**同日第九轮修掉（D-037）**：Motor Driver 直接读安全层状态、锁存期间自己输出零，复测同一档 **66% → 0%**；⚠️ 代价是它**要求 Safety 在跑**，台架要 `require_safety:=false` |
| Phase 7 | Hybrid LLM（Rule Engine + Cloud LLM + 预留 Local Small LLM） | 🚧 **已开始**（**D-038** + **D-043 多步计划**）：**Rule Engine ✅**（规则表）+ **Cloud LLM ✅ 已接**（OpenAI 兼容 / **默认关闭** / **单步**计划 / 红线两处强制 / 失败分类不静默降级）。**未做**：**重规划**、多步与条件计划、**Local Small LLM**、真端点联调 |

> **仓库现状**：包含设计方案（`docs/plan.md`）、Phase 0 验收工装（`tools/`）、以及 **Overlay 业务代码工作区 `embodied_agent_ws/`**（首个节点为相机 Driver `embodied_camera_driver`，2026-10-05 落地，**D-023**；第二个是电机 Driver `embodied_motor_driver`，2026-10-06 落地，**D-025**；随后是 Control Skill `embodied_control_skills` 与接口包 `embodied_skills_interfaces`，**D-026**；再后是 Safety Runtime `embodied_safety_runtime`，**D-027**；以及 LiDAR Primitive `embodied_lidar_driver`，**D-028**；**2026-10-07 再落地上层两块**——Skill 网关 `embodied_skill_gateway`（**D-029~D-032**，含数据驱动注册表与 8 状态机）与命令路由器 `embodied_command_router`（**D-006**）；**同日第四轮再落地两块**——Autonomous Skill `embodied_autonomous_skills`（**D-034**，第一个 task-tier 技能）与 Agent Runtime `embodied_agent_runtime`（**D-035**，Executor / Event Manager / Memory）；**同日第七轮给 Safety Runtime 补上避障**（`obstacle_guard.py`，**D-036**））。**2026-10-08 同日**：**厂商语音唤醒判死**（独占串口后唤醒词说 5 遍 = 0 字节、握手 4 种组合 = 0 字节、**拔插断电重启后再试 3 遍 = 0 字节**），**厂商三个语音节点已停用**（D-041；⚠️ 随厂商 bringup 自启，重启会回来）；**语音整条链路搁置**（用户决定）。**同日再落地 `embodied_voice_wakeup`（本地语音唤醒，Phase 5）** —— 厂商环形麦的**硬件唤醒上报实测不可用**（断电重启后说 3 遍唤醒词仍 0 字节），故改由本项目自己做：`环形麦 → 16k → 本地 KWS → /embodied/voice/wakeup`。⚠️ **真机召回只有 48%，尚不可用**（详见该包 README）。**2026-10-08 再落地第二个 task-tier 技能 `autonomous.turn_until_clear`**（原地逐步转直到前方通畅，`embodied_autonomous_skills` 内，**D-042**）——与 `advance_until_blocked` 合起来构成最小的「前进到被挡 → 转到通畅」脱困行为，也是**多步规划**的第一块拼图。**2026-10-08 再落地「多步计划」**（**D-043**）：一个计划是**有序步骤串**，**每一步各自经网关**（无旁路）、**任一步不合法 ⇒ 整条被拒**、`BLOCKED` 继续而 `FAILED` 中止、**Agent 只在计划结束时醒一次** —— 真机干跑验证了三步脱困（`ARRIVED → BLOCKED → ARRIVED`）。**共 11 个包**。后续业务代码按上表顺序在该工作区内引入。

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
| [`tools/`](tools/) | Phase 0 验收工装：`phase0_chassis_motion_acceptance.py`（底盘六向运动验收）、`lidar_rotation_probe.py`（用 LiDAR 独立测原地旋转角速度，丢帧免疫）、`camera_latency_probe.py`（用 `v4l2` 控制项当"世界端探针"，分离相机**端到端延迟**与**时间戳偏移**；`--topic` 可测任意话题（含 Overlay 输出）、`--repeat` + `--jitter` 做**多事件取均值**以标定 `pipeline_latency`）、`lidar_environment_probe.py`（把 LiDAR 的距离-角度剖面量化成表 + 柱状图，用于判定近场回波是**机器人自身结构**还是**外部环境** —— 判据是"换个摆位再测一次，看回波跟不跟着车走"）、`mic_serial_probe.py`（探测讯飞环形麦**控制串口** `/dev/ring_mic` 是否还活着：握手应答 + 纯监听模式。厂商 `awake_node` 的握手等待**没有超时**，它一卡住外面完全看不出来，这个工装把它变成明确读数 —— ⚠️ **收到 0 字节 ≠ 波特率不对**；更要紧的是 **静默期的 0 字节什么也证明不了**：这条通道"有语音才说话"，得**先去激它**。**`--decode` 模式**按 `aa55` 帧协议解码，把串口上到底在说什么变成一句句话：`aa550300fb` =「唤醒成功」、`aa550200fb` =「休眠」、`aa5500XXfb` = 固定命令表第 XX 条 —— **实测发现这块模块自己就做识别**，直接把命令编号发出来）、`upper_layer_dryrun_acceptance.py`（**上层干跑端到端验收**：把 `/embodied/motor/cmd_vel_dryrun` 上的速度**对时间积分**，于是"车一动不动"也能量出"被命令走了多远"—— **23 项判据**含**事件契约**、"被拒请求不得产生任何事件"、以及 **task-tier 端到端唤醒链**。⚠️ 它有**流量预热**：先确认干跑话题真的在送帧，否则「我没在看」会伪装成「系统没动」—— 这个工装自己踩过，见 DEV_NOTES 坑 19）。`llm_planner_acceptance.py` + `fake_llm_endpoint.py`（**LLM 规划的端到端验收 + 一个假的 OpenAI 兼容端点**：把"模型会说什么"变成一张**固定表**，于是「它选了控制层技能」「它编了个不存在的技能」「它回的话不是 JSON」「它干脆不回」这些**无法靠真模型稳定复现、却最该被验**的情形都能指名道姓地跑一遍 —— 而且**不联网、不需要密钥**。三段：`plan` 九种回话、`dead` 端点没起来、`slow` 端点不回。实测 **15/15**）。`safety_chain_suspended_acceptance.py`（**安全链架空验收**：四轮离地，把"停"从消息层推到**底盘层**。两段 —— `--phase veto` 验**结构性否决**：把 Safety 的 `/motor_driver/stop` **故意设成不可达**，运动中急停仍要**真的停住**，且 Motor Driver **自己的锁存标志必须是 0**（证明停靠的是"读到的状态"而不是那次调不通的调用）；再用一个**不会停的客户端**验"解除后不静默复动"。`--phase guard` 验**避障停车**；`--phase direction` 验守卫的**方向性**（往通畅方向走**必须放行**、往受阻方向走**必须拦下**，做对照 —— 防的是"什么都拦的守卫会被关掉，等于没有"）。判据沿用 IMU 跨度 + 轮速指令，基线不安静则拒测。实测 **veto 11/11 + guard 4/4**；`direction` 2026-10-07 因**四个方向都被挡**（0.164/0.198/0.180/0.288 m）**拒测** —— ⚠️ **随后复查判据，发现工装自己有个会让这次白跑的缺陷**：它把"受阻"的上界也写成 `CLEAR_MARGIN = 0.40`，而守卫的真实阈值是 `min(1.0, max(0.20, 0.15×1.5)) = 0.225 m`，且它是**带着 `max_range = 0.225` 去问服务**的（更远的回波根本不返回）⇒ **(0.225, 0.40] 是死区**：工装判"该拦"、守卫**看不见**，障碍物放 0.30 m 就会**假失败**。**已修**：判据与守卫的真实阈值**同源**，`CLEAR_MARGIN` 只负责"通畅"那一端；无回波改判**通畅**；缺"受阻方向"从"跳过对照"改成**拒测**；并加 **`--dry-classify`（只量摆位、不发任何运动指令）** —— 摆位不合格时直接说清缺哪一边（见 DEV_NOTES 坑 27）。**2026-10-08 又改两处并跑通 `direction` 8/8**：① **把"拦下"提到任何位移之前** —— 早先的顺序里"放行"那步会先把车真的开走 0.3 m，再拿旧方位判"拦下"（实测：正前 0.15 m 的物体，车左移 0.3 m 后方位变成 −63°，跑出 ±30° 扇区外 ⇒ **假失败**）；**⇒ 这一段因此在地面上也能跑**，而且顺带把文档里一直空白的"命令它真的朝障碍走"验了。② **对照的前提必须还在**：若"拦下"用的那个方向此刻已不通阻（障碍物被车撞走了），**拒测**而不是照跑 —— 真机第一次就打出过"同一个方向既拦下又放行"的胡话。⚠️ 地面跑的硬前提：**障碍物必须推不走**（墙/重物）—— 车在守卫锁存前的蠕动实测 **2.2 cm**，足以把 0.20 m 外的瓶子撞倒推走。**8/8 这一趟还额外找出了守卫自己的一个真缺陷（D-039）**，见下）。`cmd_vel_arbitration_probe.py`（**`/cmd_vel` 两路发布者交替关系实测**：Motor Driver 保持 `dry_run=true`，把 Safety 的零速通道**指到同一条干跑话题**，交替关系完整复现而**真 `/cmd_vel` 发布者数 = 0** —— 工装**自己先断言这一点、不是 0 就拒测**，这就是"它不可能驱动底盘"的依据。读出：下游 `stop` **能调通**时锁存后 **121 帧全零**；**调不通**时**非零 66%（= 速率比 20/(20+10)）且不衰减**，即"抖着走"而非停车。⚠️ **D-037 之后这两档都必须是 0%** —— "调不通"那一档从 66% 变 0%，就是"否决权已结构化"的证据）。`obstacle_guard_acceptance.py`（**避障守卫干跑验收**：把守卫的**三路输入全部造假** —— 假 Motor 状态、假雷达心跳、假 `sector_min_range` 服务 —— 于是**车不动、雷达不接**也能把判定链走完，**32 项判据**含"停着不误锁""阈值外不停""左移按 +90° 判""心跳断掉报 `unknown_scan`""解除后障碍还在就重新锁上"，以及结构不变量 **`/cmd_vel` 非零帧数恒为 0**）。`suspended_motion_acceptance.py`（**架空运动验收**：四轮离地、`dry_run:=false`，第一次让**本项目自己的链**真的驱动底盘。⚠️ 判据分两条 —— **IMU 跨度**回答"有没有力矩在作用"、**轮速指令**回答"停了没有"；照搬地面工况的 `gyro_z` 会在悬空时得出相反结论（DEV_NOTES 坑 20/21）。基线不安静时**拒绝出结论**，不调松阈值）。`ground_motion_acceptance.py`（**地面运动验收**：四轮着地，**把「走到位了没有」从推断变成读数**。核心是给每一个"走了多远/转了多少"的说法配一个**独立基准** —— 位移用**原始 `/scan`** 量正前方参照面的距离变化（⚠️ **刻意不用我们自己的 `embodied_lidar_driver` 原语**：那是我们自己写的查询层，拿它当基准就又是自证），旋转用 **IMU 陀螺积分**（它不参与控制）。五个相位：`distance`（位移精度，把**请求值 / `/odom` 死推算 / 雷达**三个数摊开比 —— 直接回答 D-026 那句"`/odom` 够不够用"）、`stop`（**守卫喊停之后车又跑了多远** → 阈值标定的输入，**不经过 Control Skill** 所以能扫速度）、`chain`（一句话走完整条链再量）、`closedloop`（`BLOCKED` 分支）、`rotate`。另有 `--dry-measure` **只读一次前方距离就退出**，用于确认摆位。⚠️ 位移基准的扇区**刻意收窄到 ±10°**：±30° 里若有两个不同深度的面，车一动最近回波就在两块面之间跳，Δ距离就不再是位移（DEV_NOTES 坑 30）。实测 **距离 16/16 + 旋转 3/3 + 停车 6/6 + 链路 2/2 + 闭环 3/3**）。**验收用，不是运行时组件** |

---

## 7. 一句话版本

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**
