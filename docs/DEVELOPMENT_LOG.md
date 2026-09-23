# DEVELOPMENT_LOG.md — 开发日志

> 按日期记录**重要**开发过程：做了什么 / 为什么这么做 / 测试结果 / 遇到的问题 / 最终结果。
>
> **不记录无意义的小修改。** 只记录对项目走向有影响的开发过程。

---

## 2026-09-23（续二）— Phase 0 基线：厂商整机栈 ROS 图实测

### 做了什么

对**正在运行**的厂商整机栈（`bringup`）做**只读**基线测量 —— 未发布任何命令、未改动厂商文件、未动车：

- 环境加载链、启动方式、资源占用
- 节点 / 话题 / 服务 / action 清单与消息类型
- 底盘命令链（谁发 `cmd_vel`、谁转发、谁落到硬件）
- 关键频率实测（`/odom`）
- 安全相关：厂商 app 的争用点、遥控链路、当前是否静止

### 为什么这么做

"下一步计划"第 1 项即 Phase 0 基线检查，且接口清单需要**实测值**而非包名（CLAUDE.md：仅发现包名不视为验收）。基线必须在**不动车**的前提下完成，因此全程只读。

### 实测结果

**启动与环境加载**

| 项 | 实测 |
|---|---|
| 启动方式 | `ros2 launch bringup bringup.launch.py`（PID 706，2026-09-23 20:35:27 启动）—— **非 systemd、非 autostart**，为手工启动 |
| 环境链 | `~/.zshrc:1` → `source $HOME/ros2_ws/.zshrc` → `source $HOME/ros2_ws/.robotrc` |
| 平台 | L4T R36.4.3（JetPack 6.x）、aarch64、`5.15.148-tegra` |
| 资源 | RAM **5472/7620MB**；CPU 7~17%@729MHz（低频，负载很低）；tj 54℃；VDD_IN 4.9W |

**规模**：18 个节点、50+ 话题，其中含 `rosbridge_websocket` / `web_video_server` / `rosapi`（厂商 web 端在运行）

**底盘命令链（本次最重要的发现）**

```text
/cmd_vel (发布者 0，订阅者 1)  ─┐
/app/cmd_vel                   ├→ odom_publisher → /ros_robot_controller/set_motor → ros_robot_controller → 电机
/controller/cmd_vel (发布者 5) ─┘        ↑                                                    ↓
                                   /odom_raw → ekf_node → /odom (30Hz)     battery/button/imu_raw/joy/sbus
```

- `/ros_robot_controller` **不订阅任何 `cmd_vel`**，只认 `/ros_robot_controller/set_motor`（`ros_robot_controller_msgs/MotorsState`）→ 它是 Primitive 层硬件桥
- `odom_publisher` 是**唯一**同时"订阅 `cmd_vel` + 发布 `set_motor`"的节点 → 运动学层

**频率实测**：`/odom` = **30.009 Hz**（min 0.033s / max 0.034s / std 0.00018s）→ 稳定

**遥控与物理输入链路**（安全验收抓手）：`/ros_robot_controller/{sbus, joy, button}` 确认存在（SBUS 遥控 / 手柄 / 物理按键）

**当前安全状态**：被动监听 `/controller/cmd_vel` 8s **无任何消息**；`/client_count` = 0（无 web 客户端）→ **底盘此刻静止、无人远程操控**

**未就绪项**

- **LiDAR**：ROS 图中**无 `/scan`**；`/lidar_app` 订阅列表为空（只发布 `/controller/cmd_vel` 与云台 servo）；`aurora930_node` 进程不存在 → LiDAR 数据根本没进 ROS 图
- **相机**：仅 `/depth_cam/rgb0/image_raw`，有 publisher 但 6s 采样无消息，且无 CameraInfo / depth 话题

### 遇到的问题

1. **厂商 app 全部具备驱动底盘的能力**（🔴 安全）—— `/controller/cmd_vel` 的 5 个发布者含 `lidar_app` / `line_following` / `object_tracking` / `self_driving`，每个都带 `enter` / `heartbeat` 服务，激活即发运动指令。**这是底盘验收必须先处理的前置风险**（已知问题 #10）。
2. **LiDAR 完全未就绪** —— 与先前 syslog 中 `aurora930_node ... wait device insert...` 互相印证（该进程当前不在运行，日志疑为历史记录）。已记为已知问题 #4。
3. **相机有话题无数据** —— 已记为已知问题 #11。
4. **厂商栈含机械臂/夹爪控制器** —— `/arm_controller`、`/gripper_controller` 提供 `follow_joint_trajectory`。按 D-012 第一版不做机械臂，仅记录其存在。
5. **内存实测** —— 厂商栈一启动即占 ~5.5G，坐实 8GB 的约束（已知问题 #7）。

### 最终结果

- **Phase 0 基线检查完成**（ROS 图 + 环境加载顺序 + 资源实测）
- 接口清单填入底盘 / 相机 / LiDAR 实测值；**无一项标"通过"**（运动与传感器均未验收）
- 新增决策 **D-016**：控制接入点选 `/cmd_vel`，不与厂商 app 争 `/controller/cmd_vel`
- 全程只读：未发布命令、未改厂商文件、未动车

---

## 2026-09-23（续）— 解除磁盘阻塞：根分区在线扩容 65G → 116G

### 做了什么

1. **分区布局侦察**（只读，不修改任何分区）
2. **零风险缓存清理** ~2.35G
3. **在线扩容**根分区与文件系统：65G → 116G，可用 2.9G → 55G
4. 两项"看似可删"的资产经核查后**主动放弃删除**（见下"遇到的问题"）
5. GPT 备份留档：`/root/gpt-nvme0n1-2026-09-23.bin` + `.txt`

### 为什么这么做

磁盘是 Phase 0~7 全部部署（PyTorch / 模型权重 / engine / rosbag / Docker）的硬前提。

侦察发现关键事实：**这块 256GB NVMe 只用了 65G，后面 185G 空闲从未被使用** —— 所以"清理 3G"只是权宜，**扩容才是根本解法**：

```text
nvme0n1 = 256GB (500118192 扇区)，GPT 声明的 last-lba = 250069646（≈119 GiB）
├─ p2~p13   A/B 内核槽 + recovery + ESP       0.64 GiB
├─ p14/p15  UDA / reserved                    0.92 GiB
├─ p1  APP  ext4 = /  扇区 3050048→139405311  65 GiB（已用 57.9G）
├─ 空闲    扇区 139405312→250069646           52.8 GiB  ← 可直接用
└─ GPT 之外 扇区 250069647→500118192          119 GiB   ← 被 GPT last-lba 挡住，未解锁
```

p1 是磁盘上**最后一个分区**，空闲空间紧邻其后 → 可在线扩容，无需重启、无需迁移数据。

### 测试结果

**清理（2.35G，逐项可查）**

| 项 | 回收 | 验证 |
|---|---|---|
| `/var/log/syslog` 391M + `kern.log` 75M | 466M | 截断后均为 4.0K |
| snap 旧版本 4 个（gnome-42-2204 rev245 / gnome-46-2404 rev147 / mesa-2404 rev1166 / cups rev1208） | 1.34G | `snap list --all` 已无这些 revision，`.snap` 文件消失 |
| `~/.cache/pip` | 373M | 目录已删除 |
| `~/.ros/log` | 169M | 目录已删除（`rtabmap.db` 保留） |

**扩容（关键校验，全部通过）**

```bash
sudo sfdisk --no-reread /dev/nvme0n1 < /tmp/gpt-new.txt   # 只改 p1 size，uuid 原样保留
sudo partx -u /dev/nvme0n1p1                              # 内核分区表更新
sudo resize2fs /dev/nvme0n1p1                             # 在线扩展 ext4
```

| 校验项 | 结果 |
|---|---|
| `sfdisk -d` 中 p1 | `size 136355264 → 247019599`，`uuid=7E601F05-…` **未变** |
| `lsblk -no PARTUUID /dev/nvme0n1p1` | `7e601f05-7305-42c0-afad-85b790a82e91` —— 与 `extlinux.conf` 的 `root=PARTUUID` **逐字符一致** |
| `/sys/block/nvme0n1/nvme0n1p1/size` | `247019599`（内核已跟上） |
| `resize2fs` | `30877449 (4k) blocks long` |
| `sgdisk -v /dev/nvme0n1` | **No problems found** |
| `dmesg \| grep ext4` | 无 error / warn |
| 读写实测 | 64MB 写入 803 MB/s，读写正常 |
| `df -h /` | **116G，已用 57G，可用 55G，52%** |

### 遇到的问题

1. **`snap remove <name> --revision=<rev>` 才是正确语法** —— 写成 `--revision=<rev> <name>` 会报 `snap "cups 1208" is not installed`。
2. **`/usr/src/linux-headers-5.15.0-168{,-generic}`（137M）放弃删除** —— 原以为是 x86 头文件，实为 Ubuntu **arm64 generic** 内核头（对 tegra 内核同样无用）。但 `apt-get purge --dry-run` 显示会**连带安装 `linux-headers-5.15.0-194` 新包并升级 `linux-headers-generic` 元包**（系统有 1008 个待升级包）。为 137M 在 96% 分区上触发 apt 事务不划算。
3. **`/opt/ota_package/t23x`（238M）放弃删除** —— 原以为是 OTA 残留，实为 `TEGRA_BL_*.Cap` + `xusb_t234_prod.bin` + `BOOTAA64.efi`，即 **A/B 引导链 OTA capsule 载荷**。删除会丢失固件 capsule 更新能力，离线难以重建，不值得。
4. **磁盘仍有 ~119G 未纳入 GPT** —— 本次按"不碰 GPT 几何"执行，`last-lba` 保持 250069646。需要时再单独执行（改 `last-lba` + p1 size 回灌即可），当前 55G 充足。
5. **内存仅 7.4Gi（8GB 版 Orin）** —— 一并确认的约束：PyTorch + 视觉 + 本地 LLM 并行余量有限，`/swapfile` 8G 是实际安全余量（本次未缩）。

### 最终结果

- 根分区 **65G → 116G**，可用 **2.9G → 55G**，占用率 **96% → 52%**，**磁盘阻塞解除**
- PARTUUID 保留，**未修改 `extlinux.conf` / fstab / 任何引导配置**，未重启
- 未改动厂商 `~/ros2_ws`、`~/third_party`、`~/large_models`
- GPT 备份留档在 `/root/gpt-nvme0n1-2026-09-23.{bin,txt}`

**附带确认**：本次会话中 `git push` **实测成功**（`ac3b350..1a102ba main -> main`），说明 `~/.ssh/config` 的 `ssh.github.com:443` 与 GitHub 公钥注册均已生效 —— 上一个条目中"`git push` 尚未验证成功"的遗留问题就此关闭。

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
