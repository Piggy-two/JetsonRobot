# PROJECT_STATUS.md — 项目当前状态

> **这是 Claude Code 下一次会话快速恢复项目状态的主要文件。**
> 每次开发任务结束前必须更新本文件。
>
> **最后更新：2026-09-23**

---

## 1. 一句话状态

**项目处于 Phase 0 起步阶段：设计文档已完成，工程维护机制已建立，磁盘阻塞已解除，尚无任何业务代码，硬件接口验收尚未开始。**

---

## 2. 当前阶段

### Phase 0：环境与硬件启动验收 🚧 进行中

目标：确认"已安装的软件包"与"当前小车上真实可用的硬件能力"一致。

> 仅发现包名、进程或 Workspace **不视为设备已验收**。

| 验收项 | 状态 | 说明 |
|---|---|---|
| 基线环境（ROS2 / Jetson / 磁盘） | 🟢 已完成 | ROS2 Humble、Jetson Orin（8GB，内存 7.4Gi）、磁盘已扩容至 116G；仅剩"厂商环境加载顺序"待记录（已知问题 #6） |
| 底盘与安全（`ros_robot_controller` / `controller` / `kinematics` / `servo_controller`） | 🟡 进行中 | 命令链与 `/odom`(30.0 Hz) 已实测确认（见 §7）；**运动未测**（需急停 + 看护）；遥控链路 `/sbus`、`/joy`、`/button` 已确认存在 |
| LiDAR 与避障 | 🔴 未就绪 | ROS 图中**无 `/scan`**，`/lidar_app` 订阅为空，`aurora930_node` 未运行 → 需查物理连接后再确认驱动 |
| 相机与视觉（Orbbec） | 🟡 部分完成 | 仅 `/depth_cam/rgb0/image_raw` 一个话题，有 publisher 但采样无数据；无 CameraInfo / depth 话题 |
| 语音与麦克风（`xf_mic_asr_offline`） | ⬜ 未开始 | |
| SLAM / 导航 / 系统联调 | ⬜ 未开始 | 依赖上述全部通过 |
| **接口清单交付物** | 🟡 进行中 | 底盘 / 相机 / LiDAR 已填入实测值，语音与导航待补 |

---

## 3. 已完成

| 项 | 说明 | 日期 |
|---|---|---|
| 架构设计方案 | `docs/plan.md`，29 章完整设计（Robot-as-Agent / 分层 Skill / Safety / Event-driven） | 2026-09-22 |
| 实机环境基线摸底 | 确认 Jetson Orin L4T R36.4.3、ROS2 Humble、厂商 `~/ros2_ws` 与 `~/third_party` 存在 | 2026-09-23 |
| Git 仓库与远程连接 | `origin` = `git@github.com:Piggy-two/JetsonRobot.git`，SSH over 443 已配置 | 2026-09-23 |
| 工程维护机制 | `README.md` / `CLAUDE.md` / `docs/` 四份长期文档 / `.gitignore` | 2026-09-23 |
| 磁盘阻塞解除 | 根分区**在线扩容 65G → 116G**（保留 PARTUUID，未改引导配置、未重启）+ 清理可再生缓存 2.35G；可用 2.9G → 55G | 2026-09-23 |
| GitHub Push 打通 | `git push` 实测成功（`ac3b350..1a102ba`），SSH over 443 + 公钥注册均已生效 | 2026-09-23 |
| Phase 0 基线 ROS 图实测 | 厂商 `bringup` 整机栈在运行时的 18 节点 / 50+ 话题、底盘命令链、`/odom` 30Hz、遥控链路、app 争用点全部摸清（见 `DEVELOPMENT_LOG.md`） | 2026-09-23 |

---

## 4. 正在进行

- **无正在进行的代码开发。** 当前处于"文档与基础设施就绪、等待硬件验收"的节点。

---

## 5. 已知问题

| # | 问题 | 影响 | 处理 |
|---|---|---|---|
| 1 | ~~根分区仅剩 3.0G（96% 已用）~~ **已解决** | ✅ 已解除 | 2026-09-23 在线扩容至 116G（可用 55G，52%），PARTUUID 保留；详见 `DEVELOPMENT_LOG.md`。剩余 ~119G 未纳入 GPT，**非阻塞** |
| 2 | **GitHub SSH 22 端口被网络封锁** | ✅ 已规避 | 已在 `~/.ssh/config` 配置走 `ssh.github.com:443`，实测可用 |
| 3 | ~~SSH 公钥未注册到 GitHub~~ **已解决** | ✅ 已解除 | 2026-09-23 实测 `git push` 成功（`ac3b350..1a102ba`） |
| 4 | **LiDAR 未就绪（型号亦未确认）** | 🔴 阻塞 LiDAR / SLAM / 导航 | 基线实测：ROS 图中**无 `/scan`**、`/lidar_app` 订阅为空、`aurora930_node` 未运行。**先查物理连接/供电/USB，再谈驱动选型**。候选：`ydlidar_ros2_driver` / `sllidar_ros2` / `sclidar_ros2` / `ldlidar_stl_ros2` / `Aurora930` |
| 5 | **仓库尚无代码** | ⬜ 非缺陷 | 按 Phase 顺序引入，不要提前创建空模块 |
| 6 | 厂商栈加载顺序未记录 | 🟡 待补 | 需记录 `ROS_DISTRO` / `AMENT_PREFIX_PATH` / `PYTHONPATH` 与启动脚本来源 |
| 7 | **内存仅 7.4Gi（8GB 版 Orin）** | 🟡 设计约束 | **实测：厂商 bringup 栈一启动即占用 ~5.5G（5472/7620MB）**，留给本项目的余量很小 → Phase 6/7 本地小模型与视觉并发必须按此预算设计；`/swapfile` 8G 是实际安全余量，**不要缩** |
| 8 | syslog 中 `aurora930_node` 反复 `wait device insert...` | 🟡 Phase 0 线索 | 提示 LiDAR 侧可能存在 Aurora930 驱动（待确认是当前状态还是历史日志）；实机验收时以 launch 文件与 ROS graph 为准 |
| 9 | `apt` 有 1008 个待升级包 | ⬜ 非缺陷 | 暂不升级（升级前需确认不影响厂商 SDK 与内核）；涉及 `linux-headers-generic` 元包，勿单独 purge |
| 10 | **厂商 app 可随时接管底盘** | 🔴 安全 | 实测 `/controller/cmd_vel` 有 **5 个发布者**（`lidar_app` / `line_following` / `object_tracking` / `self_driving` 等），各自带 `enter` / `heartbeat` 服务，被激活即开始发运动指令。**运动测试前必须确认这些 app 未激活，并保留急停与人工看护** |
| 11 | 相机有 publisher 但无数据流 | 🟡 待验收 | `/depth_cam/rgb0/image_raw` 存在但采样无消息，且无 CameraInfo / depth 话题；Phase 0 相机验收时确认设备与 launch 配置 |
| 12 | 厂商栈含机械臂/夹爪控制器 | ⬜ 非本项目范围 | `/arm_controller`、`/gripper_controller` 提供 `follow_joint_trajectory`；按 `DECISIONS.md` D-012 第一版不做机械臂，仅记录其存在 |

---

## 6. 下一步计划

**优先级从高到低：**

1. **底盘与急停验收**（最高优先，**必须**车轮悬空或留安全距离 + 人工看护）—— 前置条件：先确认 5 个厂商 app 处于**未激活**状态（`/lidar_app/exit`、`/line_following/*`、`/object_tracking/*`、`/self_driving/*` 的 `enter`/`heartbeat` 服务）。测试 `stop()`、低速前进/后退/平移/原地旋转；验证速度上限、命令超时、通信中断停车、**遥控优先级**（`/ros_robot_controller/sbus` / `joy` / `button` 已确认存在）。从 `/cmd_vel` 发布测试（见 D-016）。
2. **LiDAR 物理排查 → 验收** —— 当前**无 `/scan`**：先查设备连接/供电/USB 与型号，再确认驱动；验收 `LaserScan` 频率/角度/量程/frame_id 与 TF。
3. **相机验收** —— 当前只有 `/depth_cam/rgb0/image_raw` 且**无数据流**：先查设备与 launch 配置，再验 RGB/Depth/CameraInfo/TF 与 `cv_bridge` 最小收图。
4. **语音验收** —— 音频设备、ASR 文本输出、"停/急停/取消任务"本地解析链路（必须绕过 LLM）。
5. **产出接口清单** —— Phase 0 的交付物（见上，底盘/相机/LiDAR 已有实测值）。
6. 只有接口清单标记"通过"的能力，才允许封装为 Skill。

> 已完成：磁盘阻塞解除、SSH 公钥注册、**Phase 0 基线检查**（ROS 图 / 环境加载顺序 / 资源实测）均于 2026-09-23 完成。若空间再度紧张，按 `DECISIONS.md` D-014 的顺序处理，并参考 D-015 的 PARTUUID 约束。

---

## 7. 接口清单（Phase 0 交付物 · 填写中）

> 2026-09-23 只读基线实测填写。**"验收结果"列只有实机测试通过后才允许标"通过"** —— 目前无一项标通过。

| 模块 | 已确认驱动/包 | 启动入口 | 输入接口 | 输出接口 | TF / 设备路径 | 验收结果 |
|---|---|---|---|---|---|---|
| 底盘 | ✅ `ros_robot_controller`（硬件桥）+ `controller`/`odom_publisher`（运动学）+ `servo_controller` | `ros2 launch bringup bringup.launch.py`（实测在运行） | ✅ `/cmd_vel` 或 `/controller/cmd_vel`（`geometry_msgs/Twist`）→ `odom_publisher` → `/ros_robot_controller/set_motor`（`MotorsState`） | ✅ `/odom_raw` → `ekf_node` → `/odom`（**实测 30.0 Hz**，抖动 <1ms）；`/ros_robot_controller/{battery,button,imu_raw,joy,sbus}` | ⬜ TF 帧待确认 | 🟡 链路已实测；**运动未测** |
| LiDAR | ❌ 未出现在 ROS 图 | — | — | ❌ 无 `/scan` | ⬜ | 🔴 未就绪 |
| 相机 | `orbbec_camera`（bringup 内加载） | 同上 | ⬜ | 🟡 仅 `/depth_cam/rgb0/image_raw`，**无数据流** | ⬜ | 🟡 部分 |
| 语音 | `xf_mic_asr_offline`（待确认） | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ 未测 |
| 导航 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ 未测 |

**实测命令链（2026-09-23）**：

```text
/cmd_vel (发布者 0，订阅者 1)  ─┐
/app/cmd_vel                   ├─→ odom_publisher ─→ /ros_robot_controller/set_motor ─→ ros_robot_controller ─→ 电机
/controller/cmd_vel (发布者 5) ─┘        ↑                                                    ↓
                                   /odom_raw ─→ ekf_node ─→ /odom (30Hz)      battery / button / imu_raw / joy / sbus
```

> ⚠️ `/ros_robot_controller` **不订阅任何 `cmd_vel`** —— 它只认 `/ros_robot_controller/set_motor`。厂商的 5 个 app（`lidar_app` / `line_following` / `object_tracking` / `self_driving` 等）都挂在 `/controller/cmd_vel` 上，**激活即抢方向盘**。

---

## 8. 阶段总览

| 阶段 | 内容 | 状态 |
|---|---|---|
| **Phase 0** | 环境与硬件启动验收 + 接口清单 | 🚧 **进行中** |
| Phase 1 | Driver / Primitive（Camera / Motor / LiDAR Driver） | ⬜ 未开始 |
| Phase 2 | Robot Control（`move_forward` / `rotate` / `move_relative` / `stop`） | ⬜ 未开始 |
| Phase 3 | Autonomous Skills（SLAM / Navigation / 避障 / `follow_person` / `follow_line`） | ⬜ 未开始 |
| Phase 4 | Semantic Skills（`search_object` / `inspect_area` / `patrol_route` / `return_home`） | ⬜ 未开始 |
| Phase 5 | Voice System（Wake Word / VAD / ASR / TTS） | ⬜ 未开始 |
| Phase 6 | Agent Runtime（Planner / Executor / Skill Registry / Event Manager / Memory / Safety Gateway） | ⬜ 未开始 |
| Phase 7 | Hybrid LLM（Rule Engine + Cloud LLM + 预留 Local Small LLM） | ⬜ 未开始 |

---

## 9. 新会话恢复检查清单

```bash
# 1. 读文档
cat README.md docs/PROJECT_STATUS.md

# 2. 看仓库状态
git status && git log --oneline -10

# 3. 确认环境
echo $ROS_DISTRO && df -h /
```

然后从本文件「第 6 节 下一步计划」继续。
