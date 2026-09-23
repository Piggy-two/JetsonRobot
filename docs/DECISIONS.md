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

## 待补充的决策（尚未确定）

| 议题 | 说明 |
|---|---|
| **底盘选型理由** | 实机为麦克纳姆轮厂商底盘，但 `plan.md` 未记录选型原因。需用户补充：为什么选麦轮（全向移动 / 场地限制 / 成本 / 厂商方案） |
| **LiDAR 型号** | 存在 5 个候选驱动包，需实机确认后记录选型理由 |
| **相机型号** | Orbbec 具体型号与深度对齐方案待确认 |
| **跨语言 IPC 具体形式** | D-011 仅确定分层，topic / service / action 的具体选择待实现时确定 |
| **本地小模型选型** | Phase 7 议题，待定 |
| **Cloud LLM 选型** | 待定；涉及成本、延迟、隐私与离线降级策略 |
