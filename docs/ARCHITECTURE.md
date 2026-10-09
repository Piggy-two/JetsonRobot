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
| Level 0 | Hardware Capability | Motor / Camera / LiDAR / Microphone / Speaker / IMU，通过 `/dev/rrc`（底盘）、`/dev/lidar`（雷达）、`/dev/video0`（单目相机）、USB 声卡、CAN / I2C / SPI / PWM / USB 访问。**不直接暴露给 LLM** | 🟡 设备均已**实测存在**（2026-09-28，见 `PROJECT_STATUS.md` §7 与 D-017 / D-018）→ **LiDAR 已通过验收**（`/scan` 10.00 Hz / 360°）、**相机已出图**；底盘运动仍未测 |
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
| **Skill Gateway + Registry** | `embodied_skill_gateway`（Overlay）：D-005 的六项检查 + **数据驱动的注册表** + 任务表与 8 状态机。🔒 **唯一被允许直接调用技能服务的进程** | ✅ 已实现（2026-10-07，D-029/D-030/D-031/D-032） |
| **Hybrid Command Router** | `embodied_command_router`（Overlay）：安全词 / 确定性命令 / 复杂任务三分类 + 中文命令解析。安全词判定**复用** `estop.py` 的整句匹配 | ✅ 已实现（2026-10-07，D-006） |
| **Agent Runtime** | `embodied_agent_runtime`（Overlay）：**Executor**（WAIT 推进 / 超时 / 唤醒判定）+ **Event Manager** + **Memory**（有界）+ **Planner 两跳**（**D-038**：规则表优先，没命中才交给**云端 LLM** 选 task-tier 技能；**默认关闭**）+ **多步计划**（**D-043**：逐步过网关，Agent 只在计划结束时醒一次）+ **重规划**（**D-044**：`BLOCKED` / `TARGET_LOST` ⇒ 换一条走法，**必须不同且有界**）。🔒 红线在代码里执行，两条路**共用一个校验口** | ✅ 实现完成（D-035 / D-038 / D-043 / D-044）；✅ **地面端到端通过**（单步，2026-10-08）+ ✅ **干跑端到端 29/29**（2026-10-09）；✅ **已接真端点**（2026-10-09，DeepSeek / **D-045**）—— 真模型下四类任务结构断言全过、有一次重规划**收敛到 `ARRIVED`**；⚠️ 效果是**观察**不是"通过" |
| **Autonomous Skill** | `embodied_autonomous_skills`（Overlay）：第一个 task-tier 技能 `advance_until_blocked` —— **闭环**（每步重新问 LiDAR） | ⚠️ 已实现（2026-10-07，D-034）；**未真机验证**，受 `allow_motion` 闸门约束 |
| **Vision Driver** | `embodied_vision_driver`（Overlay）：把"画面里有没有某个东西、在哪一侧"变成一句**可以信的回答**。判定在纯 Python 的 `vision_query.py`，节点只取帧 / 推理 / 转发。🔒 **核心是那条不对称规则**：找到了可信；**"没找到"只有在画面质量达标时才敢说**；糊 / 没帧 / 模型没就绪 / 类别不在表里 ⇒ **「不知道」而不是「没有」**（**D-046**） | ✅ **实机通过**（2026-10-09）：detections **22.3 Hz**、服务语义三连（找到 / 确认真没有 / 不知道）、★ 只抬高清晰度门槛 ⇒ 从"确认真没有"降级为"不知道"。⚠️ 单目、`side` 不含距离（D-017）；⚠️ **对焦是人工的**，对焦不对时表现仍是"什么都没检出"（#29 / 坑 43）；⚠️ 还没有任何 `semantic.*` 技能接上它 |
| 本地轻量 LLM | 后期加入，用于简单语言理解 / Tool Calling / 离线模式 | 📋 第一版不做 |
| 云端 LLM 调用 | 复杂任务规划、多步骤推理、异常处理 | 🚧 **接口已接（D-038）**：OpenAI 兼容端点、**默认关闭**、支持**多步计划**（D-043）与**重规划**（D-044，提示词里带上"试过什么、各自什么结果"）；失败分类充分（超时/连不上/回包烂/**被截断**各是各的话），**绝不静默降级**。✅ **已接真端点**（2026-10-09）：DeepSeek 的**推理模型**——⚠️ 思维链也计入 `max_tokens`，默认已提到 4096（**D-045**） |
| TensorRT 视觉推理 | GStreamer → CUDA → TensorRT → YOLO → Tracker | 📋 |
| 相机 Driver | `embodied_camera_driver`（Overlay，Driver 层）：收口厂商相机源——**重新打时间戳**（原始戳早 0.72 s） + **补发 TF 帧** `camera_link0 → camera`。只做确定性数据整形，不含语义/规划 | ✅ 已实现（2026-10-05） |
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
Skill Gateway            ← 唯一准入点（D-005 的六项检查）
 │                         **它查 Skill Registry（数据），而不是排在 Registry 后面**
 ├─── Skill Registry（数据：有哪些技能 / 参数与策略边界 / 权限 / 超时）
 ↓
Skill Manager            ← 第一版与 Gateway 同进程，是它内部的"派发"职责（D-029 决策 4）
 ↓
ROS2 / Autonomous Runtime
 ↓
Control Runtime
 ↓
Linux Kernel / Driver
 ↓
Hardware
```

> 📌 **2026-10-07 更正（D-029）**：本节原来画的是 `Agent Runtime → Tool / Skill Registry →
> Skill Manager`，把 **Registry 画成了一个并列的跳**。实际上 Registry 是 **Gateway 查的数据** ——
> "有哪些技能、参数边界是多少、谁能调"是对着它查的，它本身不承担任何控制流。
>
> 📌 **已落地的部分（2026-10-07）**：`embodied_skill_gateway`（Skill Gateway + 注册表 + 任务表）、
> `embodied_command_router`（Hybrid Command Router，§7）、
> `embodied_agent_runtime`（**Executor / Event Manager / Memory** + **Planner 两跳**：
> 规则表优先，没命中才问**云端 LLM**；**默认关闭**，**D-038**）+ **多步计划**（**D-043**）
> + **重规划**（**D-044**，§18 的闭环四段齐了）、
> `embodied_autonomous_skills`（第一个 task-tier 技能）。
> `embodied_bringup`（**不是组件**：一条命令起上面这些节点，并把安全姿态打在启动日志里）。
> **Skill Manager 的独立进程形态仍是设计**（第一版与 Gateway 同进程，D-029 决策 4）。
> 详见 **D-029 / D-030 / D-031 / D-032 / D-034 / D-035 / D-038 / D-043 / D-044**。

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
Safety Gateway            ← 2026-10-07 起为 embodied_skill_gateway（D-005 的落点）
 ├── Schema Validation     ← checks.py：字段缺失/多余/类型/NaN/inf（**通用信封**，D-031）
 ├── Permission            ← checks.py：按 principal 判定（**D-003 的唯一执行点**，D-029）
 ├── Range Check           ← checks.py：**策略**边界，刻意比底层能力更严
 ├── Timeout               ← task_table.py：deadline 用**单调钟**（#24）
 ├── Cancel                ← 落到技能自己的 stop，**不是"不再等它"**（D-030）
 └── Result Validation     ← checks.py：control-tier 恒 verified=false，且不得产出 ARRIVED（D-032）
 ↓
Skill Manager              ← 第一版与 Gateway 同进程（D-029 决策 4）
```

**六项检查各自"在哪一层被真正执行"**（这一列表比框图更重要）：

| 检查 | 真正的执行点 |
|---|---|
| Schema Validation | `checks.validate_schema` —— 通用信封让它**有事可做**；强类型服务会让它退化成空检查 |
| Permission | `checks.validate_permission` —— 注册表的 `allowed_principals` |
| Range Check | `checks.validate_range` —— 含**跨字段**约束 `max_norm`（`(0.5, 0.5)` 的模长是 0.707，只看单字段放不住） |
| Timeout | `task_table` 的 deadline + `skill_gateway.sweep()` |
| Cancel | `skill_gateway._stop_skill()` —— **真的去停技能** |
| Result Validation | `checks.validate_result` —— 含"`verified` 恒 false"这条**语义改写** |

**反代理性质（网关存在的理由）**：它**必须能拒绝某些底层技能会接受的东西**。
三类非透传行为 —— 权限（`agent.planner` 调 control 技能被拒）、更严的范围（0.8 m 被拒，
而底层 `max_distance` 是 1.0）、语义改写（把开环的 `success` 标成 `verified=false`
且不产出 `ARRIVED`）—— 每一条都有**回归测试**（`test/test_checks.py` 末尾）与**活体实测记录**。

示例：

```text
LLM:  move_relative(100m)
系统: REJECTED: 参数 x = 100m 超过策略上限 0.5m（该值会被记为 100.0m）

实测（2026-10-07，车未动）：
  agent.planner 调 control.move_relative → REJECTED: agent.planner 无权调用 …
  参数 x = 0.8m                          → REJECTED: 参数 x = 0.8m 超过策略上限 0.5m
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

**已实现的部分**（`embodied_safety_runtime`，D-027 + D-036）：

| 能力 | 状态 | 数据来源 |
|---|---|---|
| **Emergency Stop** | ✅ 本地安全词通路（订阅 ASR 文本，**不经 LLM、不经厂商节点**）+ `~/estop` 服务 | `/asr_node/voice_words` |
| **独立零速通道** | ✅ 锁存期间自己向 `/cmd_vel` @10 Hz 发零；**唯一发布语句是 `publish(Twist())`** | —— |
| **Watchdog（Motor Driver 停更）** | ✅ 停更 > 2 s 自动接管发零；未恢复时拒绝 `~/release` | `/embodied/motor/status` |
| **Obstacle Stop** | ✅ **D-036**：按"被命令的运动方向"，阈值 = `速度 × 预留时间`；雷达不新鲜时**停**（不知道 ≠ 安全）。⚠️ 只停不绕行，阈值待标定 | `~/sector_min_range` + `/embodied/lidar/front`（心跳） |
| **Command Timeout / Motor Timeout** | ✅ 在 **Motor Driver** 里（D-025），不在本节点 | —— |
| **Speed Limit** | ✅ 在 **Motor Driver** 里（#19 的二次限幅） | —— |
| **Low Battery Protection** | ⬜ 未做 | —— |

```text
本地安全词 ─┐
~/estop    ─┼─→ EStopLatch ─┬─→ /cmd_vel 零速流（独立通道）
看门狗失联 ─┤               ├─→ 下游 ~/stop（best-effort，把它们的锁存也打开）
避障守卫   ─┘               └─→ /embodied/safety/status + /events
```

> ⚠️ **零速流与 Motor Driver 的指令流之间没有仲裁**（见 `DEV_NOTES` 坑 23）：
> 两者是**并列发布者**，厂商端"收到一条转一条"。真正让车停住的是**下游锁存**
> （`/motor_driver/stop`）；零速流是 **Motor Driver 已经死了**时的后备。
> 这也是避障停车选择**锁存**而不是"自动解除"的原因（D-036 决策 1）。
> ⚠️ 该交互**尚未实测** —— D-027 的验收刻意在 Motor Driver 不跑时进行。

> ⚠️ **实测约束（2026-10-05，D-020）**：上面这一整套 **Watchdog / Command Timeout / Motor Timeout 全部要由本项目自己实现** —— 厂商底盘**没有任何指令超时保护**：停止发布后电机会保持最后一条速度指令继续转，实测 IMU 振荡幅度 ±0.067 rad/s（对比发 0 时 ±0.0015）。
>
> 因此 **Motor Stop 的动作定义为「主动、持续向 `/cmd_vel` 发布零速度」**，而不是「停止发指令」；且 Safety Runtime 必须拥有**独立于 Control Skill 的发布通道**，否则上游一旦卡死，停车指令也发不出去。

> ⚠️ **实测约束（2026-10-05，D-021）—— Watchdog 的「输入信号」本身有陷阱**：
> - **存活判据只能用 `/ros_robot_controller/imu_raw` 与 `/ros_robot_controller/battery`。** 断线实测中 `/odom` **照常发布 28.5 Hz**（它是纯死推算），拿它做 watchdog 输入等于**监视自己**；桥节点进程状态同样不可靠（三轮实测三种行为）。
> - **厂商栈不会自恢复**：USB 断线后仅重新 bind 无效，必须重启整个 `start_app_node.service`（秒级全栈中断）。→ **本项目必须自己实现串口重连**，在此之前「断线→重新可控」之间存在**秒级安全窗口**，且窗口内底盘保持最后速度（D-020）。
> - **本机没有物理急停**（厂商按键脚本里的 `sudo halt` 被注释掉）→ 不存在可以兜底的硬件层。**唯一的停车手段是软件持续发 0，或直接断电。**

---

## 7. Hybrid Command Router

| 类型 | 示例 | 路径 | 经 LLM |
|---|---|---|---|
| A. Safety Command | 停 / 急停 / 取消任务 / 别动 | Voice → Local Parser → Safety Runtime → Motor Stop | ❌ |
| B. Deterministic Command | 向前走 0.5 米 / 右转 30° / 回到原地 | Voice → Local Parser → **Skill Gateway → Control Skill** | ❌ |
| C. Agent Task | 去桌子旁找杯子，没有就去沙发旁找 | ASR → Agent Runtime → LLM Planner → Semantic Skill | ✅ |

> 📌 **已落地（2026-10-07）**：`embodied_command_router` 实现上面这张表的分流；
> B 类**多走一跳 Skill Gateway**（D-029/D-031：所有动作必须过六项检查，没有旁路）。
> C 类**转给 `embodied_agent_runtime`**，由它回答"能不能做"—— 路由器只负责分类（D-035）。
> 而 Agent Runtime **默认**下规则表是空的、LLM 那一跳是关的，所以 C 类**默认仍然会被拒绝**，
> 但拒绝是**从 Agent 层发出的**，理由写清了是"没开"（而不是"不会"）。
> ⚠️ **C 类仍然走不到今天**：`semantic.*` 技能一个都还没有 —— 规划器就算给出计划，
> 也会在网关那里因"未注册的技能"被拒（找不到杯子这件事，本机还没有任何能力支撑）。
>
> ⚠️ **`停止追踪` 不是安全词**：判定用**整句匹配**（`embodied_safety_runtime/estop.py`），
> 子串匹配会让厂商词表里的 `停止追踪` / `停止分拣` 全部误触发整机急停。
>
> ⚠️ **`取消任务` 当前会锁存整机急停**：它在厂商安全词表里，而 Safety Runtime
> **独立订阅同一条语音话题**并匹配 —— 这与命令路由器做什么无关。若要区分"温和取消"
> 与"急停"，必须与 Safety Runtime 的默认词表**一起**决策（见本文件 §待补充决策）。

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

> ⚠️ **视觉输入是单目**（2026-09-28 实测确认，全系统无深度相机，见 **D-017**）：
> - `get_target_position()` **不能**沿用"深度对齐"方案，需改为单目方案（像素几何 + 已知目标尺寸 / 标定 + LiDAR 辅助），实现时追加决策记录。
> - `avoid_obstacle()` 的测距来源**只能依赖 LiDAR**（D-018），不得设计成依赖深度图。
> - 建图**不能**走 RGBD SLAM，只能走 2D LiDAR SLAM。
> - **Phase 0 的相机接入走厂商 `usb_cam` 分支**（改 `.typerc` 一行，见 **D-019**），其话题经 remap 后与旧深度分支同名（`/depth_cam/rgb0/image_raw`），下游 app 无需改动；若 Phase 1 改为 Overlay 自建 Driver，需先更新 D-019。
> - ⚠️ 图像 `frame_id=camera` **不在 TF 树中**（树里是 `camera_link0`，见 `PROJECT_STATUS.md` #16）—— 做视觉 Skill 前必须补齐或显式指定相机帧，**不得假设图像帧可直接做 TF 变换**。

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
~/JetsonRobot/        ← 本仓库
    embodied_agent_ws/    ✅ 本项目 Overlay Workspace（在仓库内，随 git 版本管理）
        src/              ← 本项目自己的包（**进 Git**）
            embodied_camera_driver/   相机 Driver：时间戳重打 + TF 补发
        build/ install/ log/          构建产物（**不进 Git**，已 gitignore）
    vendor_reference/     📖 ~/ros2_ws/src 的本地只读副本，775M，仅供查阅（不进 Git）
    docs/ tools/          ✅ 文档与 Phase 0 验收工装
```

> ⚠️ Overlay 放在**仓库内**而非 `~/embodied_agent_ws`，是为了让代码进 Git、
> `git status` 能反映真实开发状态（CLAUDE.md §1/§4）。
> 历史上仓库根曾有一个 `src/src/` 的厂商副本，其存在迫使 `.gitignore` 写了
> 一条**裸的 `src/`** 规则；该规则会连带忽略工作区自己的源码，故已改名
> `vendor_reference/` 并把规则改为锚定形式（详见 `docs/DEV_NOTES.md` 坑 4）。

**加载顺序**（以厂商既有顺序为准，不要随意改动；注意 Overlay 必须**在厂商之后** source）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/JetsonRobot/embodied_agent_ws/install/setup.bash   # ← 本项目 Overlay
```

> ⚠️ 上述命令必须在 **bash** 里执行。zsh 没有 `BASH_SOURCE`，ROS 的 `setup.bash`
> 会定位失败（详见 `docs/DEV_NOTES.md` 坑 5）。
