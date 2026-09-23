# DEVELOPMENT_LOG.md — 开发日志

> 按日期记录**重要**开发过程：做了什么 / 为什么这么做 / 测试结果 / 遇到的问题 / 最终结果。
>
> **不记录无意义的小修改。** 只记录对项目走向有影响的开发过程。

---

## 2026-09-23 — 建立工程维护机制与项目文档结构

### 做了什么

为 JetsonRobot 建立长期开发闭环（需求 → 改代码 → 测试 → 分析 → 更新文档 → git diff → commit → push），本阶段**不改动任何业务代码**（实际上仓库当时也没有业务代码）。

产出：

1. `README.md` —— 项目对外介绍、目标、核心功能、快速启动、当前完成度
2. `CLAUDE.md` —— Claude Code 长期开发规则（10 节）
3. `docs/ARCHITECTURE.md` —— 系统架构、模块关系、数据流 / 控制流
4. `docs/PROJECT_STATUS.md` —— 当前阶段、已完成、已知问题、下一步（状态恢复入口）
5. `docs/DEVELOPMENT_LOG.md` —— 本文件
6. `docs/DECISIONS.md` —— 技术决策及原因
7. `.gitignore` —— 按 Jetson / ROS2 / Python / TensorRT 技术栈定制

### 为什么这么做

- 项目此前只有一份 29 章的 `docs/plan.md` 设计方案，缺少**工程侧**的文档结构，导致新会话无法快速恢复上下文。
- 建立 `PROJECT_STATUS.md` 作为状态入口、`DECISIONS.md` 作为设计原因记录，避免后续重复讨论或遗忘设计动机。
- 先立规则再写代码，避免代码先于规范导致架构漂移。

### 实机环境摸底结果

| 项目 | 结果 |
|---|---|
| 平台 | NVIDIA Jetson Orin，L4T **R36.4.3**（JetPack 6.x），内核 `5.15.148-tegra` |
| ROS2 | **Humble**，`/opt/ros/humble/bin/ros2` |
| 厂商工作空间 | `~/ros2_ws`（592M）= `app`/`bringup`/`calibration`/`driver`/`example`/`interfaces`/`large_models`/`multi`/`navigation`/`openclaw_controller`/`peripherals`/`simulations`/`slam`/`xf_mic_asr_offline` 等 |
| 第三方 | `~/third_party`（6.4G）= `aurora_ws`/`gmapping_ws`/`orbbec_ws`/`rtabmap_ws`/`sherpa-onnx`/`YDLidar-SDK`/`yolo`/`opencv` 等 |
| 大模型目录 | `~/large_models`（282M，含 `agent_demo.py`/`llm_demo.py`/`asr_demo.py` 等厂商 demo） |
| 数据集 | `~/my_data`（`JPEGImages`/`Annotations`/`data.yaml`，YOLO 格式） |
| **磁盘** | 🔴 **根分区 96% 已用：58G / 64G，仅剩 3.0G** |

### 测试结果

**Git 状态处理（关键）：**

发现本地与远程历史分叉：

```text
local  main      94b40df  author: Piggy-two
origin/main      ac3b350  author: JetAuto
```

但两者 **tree hash 完全一致**（均为 `29f6b76861a464f813d1bca8f8c36eda507cfba4`），即**内容完全相同、仅作者与时间不同** —— 同一份内容被提交了两次。

处理方式（经用户确认）：`git reset --soft origin/main`

- 本地 `main` 指向 `ac3b350`，与远程一致，分叉消除
- 内容 100% 保留（tree 相同），**远程历史完全未改动**
- 保留本地恢复点：tag `backup-local-94b40df`
- **未使用 force push，未使用 `reset --hard`**

**网络连通性测试：**

| 测试 | 结果 |
|---|---|
| `ssh -T git@github.com`（22 端口） | ❌ `connect to host github.com port 22: Connection timed out` |
| `ssh -T -p 443 git@ssh.github.com` | ⚠️ 可达，但 `Permission denied (publickey)` |
| `git ls-remote https://github.com/...` | ✅ 成功 |

**结论**：22 端口被网络封锁；443 端口网络可达但 SSH 公钥未注册到 GitHub 账号。已在 `~/.ssh/config` 配置 `HostName ssh.github.com` / `Port 443`（原配置备份为 `~/.ssh/config.bak.*`）。

### 遇到的问题

1. **历史分叉** —— 见上，已安全解决。
2. **SSH 22 端口封锁** —— 已用 443 端口规避。
3. **SSH 公钥未注册** —— 阻塞 `git push`，需用户在 GitHub 账号添加 `~/.ssh/id_ed25519.pub`。
4. **磁盘空间仅剩 3.0G** —— 未解决，已记录为最高优先级已知问题。

### 最终结果

- 工程维护机制（文档 + 规则 + .gitignore）建立完成
- Git 历史分叉已安全消除，本地与远程一致
- **`git push` 尚未验证成功**（等待公钥注册）
- 未改动任何业务代码，未改动厂商 `~/ros2_ws` 与 `~/third_party`

---

## 2026-09-22 — 完成整体架构设计方案

### 做了什么

产出 `docs/plan.md`（29 章），定义完整技术路线：

- 项目定位：Robot-as-Agent（整台机器人是 Agent，而非 Agent 控制机器人）
- 核心架构：Hierarchical Skill Architecture + Hybrid Agent Architecture + Event-driven Execution
- 6 层 Skill 架构（Hardware → Driver → Primitive → Control → Autonomous → Semantic → Agent Task）
- 三层时间尺度（Agent 秒级 / Skill 10~30Hz / Control 20~100Hz）
- Hybrid Command Router（Safety / Deterministic / Agent Task 三类分流）
- Safety Gateway + Safety Runtime（Safety 具有最终否决权）
- Event-driven Agent（Skill 状态机 + 事件唤醒）
- 硬件启动验收流程（第 24 章）与 8 个开发阶段（第 25 章）

### 为什么这么做

在写代码前先确定分层边界与安全边界，特别是"LLM 不参与实时控制"这一条，避免后期因架构不清导致重构。

### 测试结果

纯设计文档，无代码测试。

### 遇到的问题

无。

### 最终结果

设计方案完成并提交（remote `ac3b350`）。
