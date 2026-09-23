# DECISIONS.md — 重要技术决策记录

> 记录**为什么**这样设计，防止后续忘记设计原因而做出冲突的改动。
>
> 格式：决策 → 原因 → 影响 / 约束。新增决策请追加，不要删除历史决策；若决策被推翻，标注"已废弃 + 替代决策"。

---

## D-001：机器人整体是 Agent，而不是"Agent 控制机器人"

**决策**：把整台机器人抽象为一个具备 Perception / Reasoning / Action / Environment Feedback 的完整 Agent。Jetson 提供认知与计算能力，LLM 提供高层推理，Skill Runtime 与 Linux 负责把决策变成真实动作。

**原因**：
- "Jetson 上跑一个 Agent 去控制小车"会把智能局限在一个 Python 进程里，而真实的感知、执行、反馈分散在整个系统。
- Agent 的完整定义要求闭环（感知 → 推理 → 动作 → 环境反馈），只有整机才能构成这个闭环。

**影响 / 约束**：
- 不能用"Agent 进程"的视角去划分模块边界。
- 编写文档与代码时，"Agent" 指整机；指进程时明确写 `agent_service` / `agent_runtime`。

---

## D-002：LLM 不参与实时控制

**决策**：LLM 只负责自然语言理解、任务拆解、多步骤规划、Skill 选择、条件判断、失败重规划、结果总结。**禁止** LLM 参与电机 / PID / 急停 / 避障等实时控制。

**原因**：
- LLM 推理延迟（秒级）与控制环要求（20~100 Hz）相差数个数量级。
- LLM 输出不确定，无法提供实时控制的确定性与可验证性。
- 断网时控制能力必须仍然可用。

**影响 / 约束**：
- LLM 只输出 `navigate_to("desk")` 这类语义调用，**绝不输出** `左轮 100 RPM / 右轮 90 RPM / 向左偏 2°`。
- 任何进入控制环的代码不得依赖大模型。
- 这是**架构红线**，不接受为"方便"而打破。

---

## D-003：采用分层 Skill 架构（6 层）

**决策**：`Hardware → Driver → Primitive → Control Skill → Autonomous Skill → Semantic Skill → Agent Task`。

**原因**：
- 让复杂智能建立在确定性的底层能力之上：越往下越确定、越实时、越不依赖 LLM；越往上语义越强、时间尺度越慢。
- 每层可独立测试与验收，避免"整机联调才能发现问题"。

**影响 / 约束**：
- Agent **只能**调用 Semantic Skill，不直接访问 Motor / GPIO / PWM / ROS Topic。
- 新增能力时必须明确它属于哪一层。
- 只有验收通过的能力才允许向上封装（见 D-009）。

---

## D-004：采用 Event-driven Agent，而非 Real-time LLM Controller

**决策**：Agent 发起 Skill 后进入 `WAIT`，由本地运行时执行；只有任务级事件（`ARRIVED` / `BLOCKED` / `FAILED` / `CANCELLED` / `TARGET_LOST` 等）才唤醒 Agent。

**原因**：
- 让 LLM 持续轮询状态会浪费算力且引入延迟。
- 导航 / 跟随等过程需要几十秒本地闭环，期间 LLM 无事可做。
- 事件粒度让 Agent 的重规划时机明确、可测试。

**影响 / 约束**：
- Skill 必须有明确的状态机与事件上报机制。
- Skill Manager 负责 Cancel / Timeout / Event 上报。

---

## D-005：Safety Runtime 独立于 LLM，且拥有最终否决权

**决策**：Safety Runtime 独立运行，包含急停、避障停止、限速、Watchdog、命令超时、电机超时、低电量保护。优先级 `Safety > Control > Skill > Agent`。

**原因**：
- 安全不能依赖一个可能超时、断网、幻觉的组件。
- 断网 / LLM 故障时安全能力必须绝对可用。

**影响 / 约束**：
- Safety 路径**不经过 LLM**。
- Tool / Skill 请求必须经过 Safety Gateway（Schema 校验 / 权限 / 范围检查 / 超时 / 取消 / 结果校验）。
- 例如 `move_relative(100m)` 必须被 `REJECTED: distance exceeds safety limit`。

---

## D-006：安全指令走本地解析，绕过 LLM

**决策**：`停 / 急停 / 取消任务 / 别动` 等安全指令走 `Voice → Local Parser → Safety Runtime → Motor Stop`，不经过 LLM。确定性指令（"向前走 0.5 米"）同理走本地解析 → Control Skill。

**原因**：
- 安全指令要求**最低延迟**与**最高可靠性**，走 LLM 会引入秒级延迟与失败风险。
- 大部分日常指令其实是确定性的，本地解析即可完成，无需消耗 LLM。

**影响 / 约束**：
- 必须维护一份本地关键词/规则表，且能被 ASR 直接触发。
- 只有真正的复杂任务（条件、多步、模糊语义）才进入 Agent Runtime。

---

## D-007：视觉推理使用 TensorRT

**决策**：视觉链路为 `Camera → GStreamer → CUDA → TensorRT → YOLO → Tracker`，对上层暴露 `detect_object()` / `detect_person()` / `capture_image()` / `get_target_position()` / `describe_scene()`。

**原因**：
- Jetson Orin 上 TensorRT 能充分利用 GPU/NVDLA，显著降低推理延迟，满足跟随、寻迹等 10~30 Hz 的实时要求。
- 目标是本地实时闭环，不能依赖云端推理。

**影响 / 约束**：
- `*.engine` / `*.plan` 是**平台相关的大二进制产物**，必须由构建脚本从 ONNX 生成，**不进 Git**（见 D-010）。
- 需记录 TensorRT / CUDA / JetPack 版本，因为 engine 与版本强绑定。

---

## D-008：C++ 与 Python 分工

**决策**：

| 语言 | 模块 | 理由 |
|---|---|---|
| C++ | `robot_service` / `navigation_service` / `safety` / 部分 vision runtime | 实时性、确定性、靠近硬件 |
| Python | `voice_service` / `agent_service` / LLM orchestration | 生态、LLM SDK、迭代速度 |

**原因**：
- 控制与安全环对延迟和确定性敏感，适合 C++。
- LLM / ASR 生态以 Python 为主，且迭代频繁，适合 Python。
- 强行统一语言会在某一侧付出明显代价。

**影响 / 约束**：跨语言接口需要明确的 IPC / ROS2 接口约定（见 D-011）。

---

## D-009：厂商 ROS2 栈作为系统 SDK，项目代码放 Overlay Workspace

**决策**：保持厂商 `~/ros2_ws` 与 `~/third_party` 不变，作为系统 SDK 使用；本项目代码放在独立的 Overlay Workspace（计划名 `embodied_agent_ws`）。

**原因**：
- 厂商栈包含大量已验证的驱动与配置（`ros_robot_controller` / `kinematics` / `orbbec_camera` / `xf_mic_asr_offline` 等），直接修改风险高、难以回滚。
- Overlay 方式可以随时丢弃本项目代码而不影响底层能力。

**影响 / 约束**：
- **不修改厂商包**；确需修补时以 Overlay 覆盖的方式实现，并在此记录。
- 环境加载沿用厂商既有 source 顺序，改动顺序需记录原因。

---

## D-010：模型权重与 engine 不进 Git，暂不引入 Git LFS

**决策**：`.gitignore` 排除 `*.engine` / `*.plan` / `*.onnx` / `*.pt` / `*.pth` 等模型产物。**暂不使用 Git LFS。**

**原因**：
- 这些文件体积大且是**可重新生成**的构建产物（ONNX → engine）。
- 当前根分区仅剩 3.0G，LFS 会增加管理复杂度与存储压力。
- 尚无必须版本化的模型资产。

**影响 / 约束**：
- 模型通过**构建脚本 + 版本化配置**复现，而不是提交二进制。
- 若未来确有必须版本化的权重，**先与用户讨论 Git LFS**，不要直接提交大文件。

---

## D-011：Agent 与底层的接口走 Skill Manager + ROS2，不用直连

**决策**：Agent → Skill Manager → ROS2（topic / service / action）→ Driver。Agent 不直接发布 `cmd_vel` 或访问 `/dev/*`。

**原因**：
- Skill Manager 统一承担注册、参数校验、执行、状态管理、Cancel、Timeout、事件上报。
- 集中一处做校验与安全拦截，避免每个调用点各自实现。

**影响 / 约束**：
- 新增底层能力时，先在 Skill Manager 注册，而不是让 Agent 直接调用 ROS2。
- 具体 IPC 形式（topic / service / action）在实现对应 Skill 时确定，并在此追加决策。

---

## D-012：第一版明确不做机械臂 / 多机器人 / 强化学习导航 / 复杂 RAG / 大型本地 VLM / 自动充电

**决策**：为保证主线清晰，第一版不包含上述功能。

**原因**：这些功能会显著分散精力，且都依赖尚未完成的底层能力。

**影响 / 约束**：评审新需求时，若属于上述范围，默认推迟到第一版之后。

---

## D-013：文档与代码在同一个 commit 内更新

**决策**：架构、进度、开发日志、决策记录的更新与对应代码改动放在同一个 commit。

**原因**：文档滞后会直接导致下一次会话基于过时信息做决策，破坏"可恢复的长期开发闭环"。

**影响 / 约束**：文档同步视为任务完成的一部分，不是可选收尾。

---

## D-014：磁盘不足时，优先"扩容 + 只回收可再生缓存"，不删引导链与固件资产

**决策**：根分区空间不足的解法顺序为 ① 先侦察分区布局 ② 扩容 ③ 只回收**可再生**缓存（日志、pip cache、ROS 日志、snap 旧 revision）。以下资产**默认不删**：

| 资产 | 保留原因 |
|---|---|
| `/opt/ota_package`（238M） | `TEGRA_BL_*.Cap` / `BOOTAA64.efi` 是 **A/B 引导链 OTA capsule 载荷**，删除会丢失固件 capsule 更新能力，离线难以重建 |
| `/usr/src/linux-headers-5.15.0-168*`（137M） | 需 `apt purge`，会连带安装新头文件包并升级元包；在接近满的分区上触发 apt 事务风险大于 137M 收益 |
| `~/.ollama/models`（3.6G） | Phase 7 本地小模型的候选资产，删后需重新 pull（依赖网络） |
| `~/.vscode-server`（3.9G） | 连上即按需重新下载，由用户按需清理更合适 |
| `/swapfile`（8G） | 内存仅 7.4Gi，这是实际的安全余量 |

**原因**：
- 这台机器是**嵌入式设备**，引导链 / 固件 / 离线安装资产一旦删除，恢复成本远高于其占用空间。
- "看起来像残留"的目录（`ota_package`、内核头）实际承担功能职责，**必须先核查内容再判断**，不能按名字归类为垃圾。
- 清理上限只有几个 G，而扩容一次可得 52.8G —— 性价比与风险都更优。

**影响 / 约束**：
- 清理前必须逐项核对内容与归属（`du` / `dpkg -S` / `apt-get --dry-run`），并说明"为什么可以删"。
- 本机可安全清理的路径：`/var/log/{syslog,kern.log}`（截断）、`~/.cache/pip`、`~/.ros/log`、`snap remove --revision=<旧 rev>`。注意 `~/.ros/rtabmap.db` 是 SLAM 数据库，**不在清理范围**。

---

## D-015：分区扩容必须保留 PARTUUID，用 sfdisk 回灌而非重新创建分区

**决策**：扩容操作固定为 `sgdisk -b` + `sfdisk -d` 备份 → 只改 p1 的 `size` → `sfdisk --no-reread` 回灌 → `partx -u <part>` → `resize2fs`。

**明确不用的做法**：
- `growpart`：本机（Ubuntu 22.04 aarch64）未安装，需额外装 `cloud-guest-utils`。
- `sgdisk -d 1` + `sgdisk -n 1:…`：**会重新生成分区 GUID**，除非额外指定 `-u`。

**原因**：
- 根分区以 **PARTUUID** 引用：`/boot/extlinux/extlinux.conf` 中 `root=PARTUUID=7e601f05-7305-42c0-afad-85b790a82e91`（由 cbootargs 传入）。分区 GUID 一变，系统无法启动。
- `sfdisk -d` 的 dump 含每个分区的 `uuid=`，原样回灌可完整保留所有 GUID 与分区属性（类型、名称、ESP 标志）。
- 在线扩容对**最后一个分区**成立：ext4 支持在线增长，`partx -u` 通过 `BLKPG_RESIZE_PARTITION` ioctl 更新内核，无需重启。

**影响 / 约束**：
- 任何分区操作前必须备份（`sgdisk -b <bin>` + `sfdisk -d > <txt>`），并留档到 `/root`。
- 操作后**强制复核**：`lsblk -no PARTUUID` 与 `extlinux.conf` 中的 `root=PARTUUID` 必须逐字符一致；`sgdisk -v` 无错误。
- 回滚：`sgdisk -l <bin>` 恢复分区表；文件系统回缩需离线 `e2fsck` + `resize2fs`（正常不应发生）。
- 涉及磁盘 / 分区的操作**必须先经用户确认**，不与其它改动混在同一个 commit。

---

## D-016：Control Skill 接入点为 `/cmd_vel`，不与厂商 app 争 `/controller/cmd_vel`

**决策**：本项目的控制类 Skill 通过发布 `geometry_msgs/msg/Twist` 到 **`/cmd_vel`** 驱动底盘。**不**发布到 `/controller/cmd_vel`，**更不**直接发布 `/ros_robot_controller/set_motor`。

**实测依据**（2026-09-23 Phase 0 基线）：

| 话题 | 发布者 | 订阅者 | 结论 |
|---|---|---|---|
| `/cmd_vel` | **0** | 1（`odom_publisher`） | ✅ 干净，专用外部入口 |
| `/controller/cmd_vel` | **5**（`lidar_app` / `line_following` / `object_tracking` / `self_driving` / … 厂商 app） | 1（`odom_publisher`） | ❌ 多方争用，不可作为接入点 |
| `/ros_robot_controller/set_motor` | 1（`odom_publisher`） | 1（`ros_robot_controller`） | Primitive 层，Control Skill 不得直接调用 |

**原因**：
- 厂商的 4~5 个 app 都挂在 `/controller/cmd_vel` 上，且各自带 `enter` / `heartbeat` 服务，**被激活即开始发运动指令**。项目若也接这一条，等于与 app 抢方向盘，且无法解释"车为什么动了"。
- `/cmd_vel` 是 ROS 生态公认的底盘入口（nav2 / teleop 同约定），发布者当前为 0 —— 独占且可预期。
- `/ros_robot_controller` 是硬件桥（`set_motor` / servo / buzzer / led / oled），属于 Primitive 层。直接调用它会绕过运动学层与 Safety，违反 D-003 / D-005。

**影响 / 约束**：
- 运动测试前必须确认厂商 app 未激活；这是 Phase 0 底盘验收的**前置条件**。
- 麦轮运动学约定、单位、速度上限、命令超时由 `odom_publisher` 决定，需在实测中记录后补充进接口清单。
- 若将来必须与某个厂商 app 共存，需先明确**互斥与优先级**，不得默认并行发布。

---

## 待补充的决策（尚未确定）

| 议题 | 说明 |
|---|---|
| **底盘选型理由** | 实机为麦克纳姆轮厂商底盘，但 `plan.md` 未记录选型原因。需用户补充：为什么选麦轮（全向移动 / 场地限制 / 成本 / 厂商方案） |
| **LiDAR 型号** | 存在 5 个候选驱动包，需实机确认后记录选型理由 |
| **相机型号** | Orbbec 具体型号与深度对齐方案待确认 |
| **跨语言 IPC 具体形式** | D-011 仅确定分层，topic / service / action 的具体选择待实现时确定 |
| **本地小模型选型** | Phase 7 议题，待定 |
| **Cloud LLM 选型** | 待定；涉及成本、延迟、隐私与离线降级策略 |
