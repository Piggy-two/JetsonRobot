# PROJECT_STATUS.md — 项目当前状态

> **这是 Claude Code 下一次会话快速恢复项目状态的主要文件。**
> 每次开发任务结束前必须更新本文件。
>
> **最后更新：2026-09-28**

---

## 1. 一句话状态

**项目处于 Phase 0 硬件验收阶段：设计文档与工程维护机制已建立、磁盘阻塞已解除，仍无业务代码。2026-09-28 完成一次全量硬件接口侦察 —— 四个 USB 口逐口识别完毕，LiDAR 型号实测确认为 LD19 且设备健康，相机定性为单目，视觉基线（D-017）与 LiDAR 选型（D-018）已定。LiDAR 的 `/scan` 待重启厂商栈后验证；运动仍**未测**。**

---

## 2. 当前阶段

### Phase 0：环境与硬件启动验收 🚧 进行中

目标：确认"已安装的软件包"与"当前小车上真实可用的硬件能力"一致。

> 仅发现包名、进程或 Workspace **不视为设备已验收**。

| 验收项 | 状态 | 说明 |
|---|---|---|
| 基线环境（ROS2 / Jetson / 磁盘 / 环境加载链） | 🟢 已完成 | ROS2 Humble、Jetson Orin（8GB，内存 7.4Gi）、磁盘已扩容至 116G；环境加载链与机型配置位置已记录（`.zshrc` → `.robotrc` → `.typerc`，原已知问题 #6 已关闭） |
| 底盘与安全（`ros_robot_controller` / `controller` / `kinematics` / `servo_controller`） | 🟡 进行中 | 命令链与 `/odom`(30.0 Hz) 已实测确认（见 §7）；**运动未测**（需急停 + 看护）；遥控链路 `/sbus`、`/joy`、`/button` 已确认存在 |
| LiDAR 与避障 | 🟡 根因已定位并修复，**待验证** | 型号实测确认为 **LD19**，设备健康（CRC 99.9% 通过、360° 完整扫描、4992 点/秒，见 D-018）。原「无 `/scan`」根因：udev 把 `/dev/lidar` 指向了底盘串口 → 2026-09-28 已修正。**等 bringup 重启后验 `/scan`** |
| 相机与视觉 | 🟡 部分完成 | 实测为**单目** UVC 摄像头（icSpring `32e6:9005`，YUYV 640×480@30，`/dev/video0`），全系统仅此一个摄像头。厂商 `.typerc` 写的 `DEPTH_CAMERA_TYPE=aurora` 与实际不符，`aurora930_node` 已退出 → `/depth_cam/rgb0/image_raw` 当前 **0 个发布者**。视觉基线见 **D-017** |
| 语音与麦克风（`xf_mic_asr_offline`） | ⬜ 未开始 | 已知配置 `MIC_TYPE=xf` / `ASR_MODE=online`（⚠️ 在线 ASR，断网不可用）。USB 声卡 0（`0c76:161f`）已被内核识别，但 `/dev/ring_mic` 未建立（`xf_mic.rules` 匹配的口 `1-2.3.1` 在本机不存在） |
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
| **硬件接口全量侦察** | 四个 USB 口逐口识别（含 Hub 拓扑）、LiDAR 型号确认为 LD19 并实测健康度、相机定性为单目、"LiDAR 无 `/scan`" 根因定位并修复 udev、`.typerc` 机型配置错配定位（见 `DEVELOPMENT_LOG.md`；决策 D-017 / D-018） | 2026-09-28 |

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
| 4 | **LiDAR 无 `/scan`（原「未就绪」）** | 🟡 根因已定位并修复，待验证 | **2026-09-28 定位**：雷达（LD19，物理口 `1-2.1`）本身健康，但 `/etc/udev/rules.d/lidar.rules` 启用的规则匹配的是 `1-2.2`（= 底盘串口 `ttyACM0`），导致 `/dev/lidar` 指向底盘、`ldlidar` 节点打不开该口 → 节点未启动 → ROS 图中无 `/scan`。**已改规则（备份 `lidar.rules.bak-20260928`）**，`/dev/lidar` 现指向 `ttyCH341USB0`。因未重启厂商栈，**`/scan` 仍需重启后验证**。型号与实测依据见 **D-018** |
| 5 | **仓库尚无代码** | ⬜ 非缺陷 | 按 Phase 顺序引入，不要提前创建空模块 |
| 6 | ~~厂商栈加载顺序未记录~~ **已解决** | ✅ 已解除 | 加载链已记录：`~/.zshrc` → `~/ros2_ws/.zshrc` → `~/ros2_ws/.robotrc` → `~/ros2_ws/.typerc`。2026-09-28 另取得**运行中进程的真实环境**（`/proc/<pid>/environ`，含完整 `AMENT_PREFIX_PATH` / `LD_LIBRARY_PATH` / `PYTHONPATH`）。**机型配置在 `.typerc`**（`LIDAR_TYPE` / `DEPTH_CAMERA_TYPE` / `MACHINE_TYPE` / `MIC_TYPE` / `ASR_MODE`），见已知问题 #13 |
| 7 | **内存仅 7.4Gi（8GB 版 Orin）** | 🟡 设计约束 | **实测：厂商 bringup 栈一启动即占用 ~5.5G（5472/7620MB）**，留给本项目的余量很小 → Phase 6/7 本地小模型与视觉并发必须按此预算设计；`/swapfile` 8G 是实际安全余量，**不要缩** |
| 8 | ~~syslog 中 `aurora930_node` 反复 `wait device insert...`~~ **已解释** | ✅ 已澄清 | **2026-09-28**：Aurora930 是**深度相机**（Deptrum，VID `3251`）驱动，**不是** LiDAR 驱动（此前记错）。本机无该设备，故节点反复等待后于 `21:12:20` 报 `No deptrum device connected! It's going to quit...` 并退出。根因是 `.typerc` 机型配置写成 `DEPTH_CAMERA_TYPE=aurora`，见已知问题 #13 与 **D-017** |
| 9 | `apt` 有 1008 个待升级包 | ⬜ 非缺陷 | 暂不升级（升级前需确认不影响厂商 SDK 与内核）；涉及 `linux-headers-generic` 元包，勿单独 purge |
| 10 | **厂商 app 可随时接管底盘** | 🔴 安全 | 实测 `/controller/cmd_vel` 有 **5 个发布者**（`lidar_app` / `line_following` / `object_tracking` / `self_driving` 等），各自带 `enter` / `heartbeat` 服务，被激活即开始发运动指令。**运动测试前必须确认这些 app 未激活，并保留急停与人工看护** |
| 11 | ~~相机有 publisher 但无数据流~~ **已定性为配置错配** | 🟡 待处理 | **2026-09-28 实测**：该话题现为 **0 个发布者**，只有 `yolo` 在订阅 —— 不是"有 publisher 无数据"，而是**话题挂在了一个不存在的深度相机上**。实机为单目 UVC 摄像头。处理方式见 #13 与 **D-017** |
| 12 | 厂商栈含机械臂/夹爪控制器 | ⬜ 非本项目范围 | `/arm_controller`、`/gripper_controller` 提供 `follow_joint_trajectory`；按 `DECISIONS.md` D-012 第一版不做机械臂，仅记录其存在 |
| 13 | **`.typerc` 机型配置与实机硬件不符** | 🟡 阻塞相机验收 | `~/ros2_ws/.typerc:9` 的 `DEPTH_CAMERA_TYPE=aurora` 期望深相机，但实机是单目 USB 摄像头。逐项核对：`LIDAR_TYPE=LD19` ✅ 与实机一致；`DEPTH_CAMERA_TYPE=aurora` ❌ 不符。厂商 `usb_cam` 分支的 `usb_cam_param.yaml`（`/dev/video0` + `yuyv` + 640×480）**与实机摄像头完全吻合**，`usb_cam` 包亦已随系统安装 → 理论上改机型配置即可让厂商栈驱动真实相机。**但改它属改动厂商配置（D-009），需先讨论再动**；本项目相机 Driver 按 D-017 在 Overlay 自建 |
| 14 | **副 Hub 上的 CH340（`ttyCH341USB1`）身份未确认** | 🟡 Phase 0 收尾项 | USB 口 4（`1-2.4`）下挂一层 Hub，`1-2.4.1` 是一个**静默无数据**的 CH340（`/dev/ttyCH341USB1`），`1-2.4.2` 是 USB 声卡。因 `xf_mic.rules` 期望的 `/dev/ring_mic`（口 `1-2.3.1`）在本机不存在，疑为**讯飞环形麦的串口控制口**（与声卡同处一个复合设备）；也可能属云台/舵机控制器。**需实测确认**，直接影响 Phase 5 语音与舵机验收 |

---

## 6. 下一步计划

**优先级从高到低：**

1. **底盘与急停验收**（最高优先，**必须**车轮悬空或留安全距离 + 人工看护）—— 前置条件：先确认 5 个厂商 app 处于**未激活**状态（`/lidar_app/exit`、`/line_following/*`、`/object_tracking/*`、`/self_driving/*` 的 `enter`/`heartbeat` 服务）。测试 `stop()`、低速前进/后退/平移/原地旋转；验证速度上限、命令超时、通信中断停车、**遥控优先级**（`/ros_robot_controller/sbus` / `joy` / `button` 已确认存在）。从 `/cmd_vel` 发布测试（见 D-016）。
2. **LiDAR `/scan` 验证**（当前最接近"通过"的一项）—— 根因已修复（`/dev/lidar` 已指向真实雷达口），**只剩重启厂商栈验证**：重启 `bringup` 后确认 `/scan` 有数据，并记录 `LaserScan` 的频率 / 角度范围 / 量程 / `frame_id` 与 TF。雷达侧是只读的，风险低；但重启会中断当前整机栈，**需挑时间**。
3. **相机：先决策再验收** —— 实机是单目（D-017），有两条路需先选：
   - (a) 改 `~/ros2_ws/.typerc` 的 `DEPTH_CAMERA_TYPE`，让厂商栈走 `usb_cam` 分支（其 `usb_cam_param.yaml` 与实机摄像头完全吻合，见 #13）—— 但属**改动厂商配置**（D-009），需先讨论；
   - (b) 本项目按 D-017 在 Overlay 自建单目 Driver。

   选定后再验采集、`camera_info`、TF 与 `cv_bridge` 最小收图。
4. **确认 `ttyCH341USB1` 身份**（#14）—— 直接关系到 Phase 5 语音（`/dev/ring_mic` 缺失）与舵机验收，**成本很低，值得先做**。
5. **语音验收** —— 音频设备、ASR 文本输出、"停/急停/取消任务"本地解析链路（必须绕过 LLM）。⚠️ 注意 `ASR_MODE=online`，需确认断网时的降级行为（D-006 要求安全指令不依赖网络）。
6. **产出接口清单** —— Phase 0 的交付物（见上，底盘/相机/LiDAR 已有实测值）。
7. 只有接口清单标记"通过"的能力，才允许封装为 Skill。

> 已完成：磁盘阻塞解除、SSH 公钥注册、**Phase 0 基线检查**（2026-09-23）、**硬件接口全量侦察**（2026-09-28）。若空间再度紧张，按 `DECISIONS.md` D-014 的顺序处理，并参考 D-015 的 PARTUUID 约束。

---

## 7. 接口清单（Phase 0 交付物 · 填写中）

> 2026-09-23 只读基线实测填写，2026-09-28 补齐 LiDAR / 相机 / 语音的设备级实测值。**"验收结果"列只有实机测试通过后才允许标"通过"** —— 目前仍无一项标通过。

| 模块 | 已确认驱动/包 | 启动入口 | 输入接口 | 输出接口 | TF / 设备路径 | 验收结果 |
|---|---|---|---|---|---|---|
| 底盘 | ✅ `ros_robot_controller`（硬件桥）+ `controller`/`odom_publisher`（运动学）+ `servo_controller` | `ros2 launch bringup bringup.launch.py`（实测在运行） | ✅ `/cmd_vel` 或 `/controller/cmd_vel`（`geometry_msgs/Twist`）→ `odom_publisher` → `/ros_robot_controller/set_motor`（`MotorsState`） | ✅ `/odom_raw` → `ekf_node` → `/odom`（**实测 30.0 Hz**，抖动 <1ms）；`/ros_robot_controller/{battery,button,imu_raw,joy,sbus}` | ⬜ TF 帧待确认 | 🟡 链路已实测；**运动未测** |
| LiDAR | ✅ **LD19**（`ldlidar_stl_ros2`，230400） | `peripherals/launch/include/ldlidar_LD19.launch.py`（由 `LIDAR_TYPE=LD19` 选择） | ✅ `/dev/lidar` → `ttyCH341USB0`（Hub 口 `1-2.1`） | ❌ 当前无 `/scan`（**节点未启动**）；重启后应为 `sensor_msgs/LaserScan` | 设备 `/dev/lidar`；参数 `frame_id` 默认 `base_laser` | 🟡 型号与数据质量已实测（D-018）；**驱动未运行，待重启验证** |
| 相机 | 单目 UVC（内核 `uvcvideo`）；厂商分支指向的 `aurora930` 不存在 | 厂商 `peripherals/launch/depth_camera.launch.py`（`DEPTH_CAMERA_TYPE=aurora` → **已失效**） | ⬜ | ❌ 无数据：`/depth_cam/rgb0/image_raw` **0 个发布者** | 设备 `/dev/video0`（YUYV 640×480@30）；`/dev/video1` = metadata 节点 | 🟡 设备已确认；**输出未验收**（基线见 D-017） |
| 语音 | `xf_mic_asr_offline`（未运行） | ⬜ | ⬜ | ⬜ | 声卡 0 = USB Audio `0c76:161f`；`/dev/ring_mic` **未建立**（#14） | ⬜ 未测 |
| 导航 | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ 未测 |

**实测命令链（2026-09-23）**：

```text
/cmd_vel (发布者 0，订阅者 1)  ─┐
/app/cmd_vel                   ├─→ odom_publisher ─→ /ros_robot_controller/set_motor ─→ ros_robot_controller ─→ 电机
/controller/cmd_vel (发布者 5) ─┘        ↑                                                    ↓
                                   /odom_raw ─→ ekf_node ─→ /odom (30Hz)      battery / button / imu_raw / joy / sbus
```

> ⚠️ `/ros_robot_controller` **不订阅任何 `cmd_vel`** —— 它只认 `/ros_robot_controller/set_motor`。厂商的 5 个 app（`lidar_app` / `line_following` / `object_tracking` / `self_driving` 等）都挂在 `/controller/cmd_vel` 上，**激活即抢方向盘**。

**实测 USB 拓扑（2026-09-28）**：

```text
Jetson 根 Hub (bus 1)
└─ 端口 2 ─→ Realtek 4 口 Hub  ← 机器人扩展板的 4 个 USB 口，4 口全满
   ├─ 口 1  1-2.1    CH340  1a86:7523   → /dev/ttyCH341USB0 = /dev/lidar  ★ LD19 激光雷达
   ├─ 口 2  1-2.2    CH9102 1a86:55d4   → /dev/ttyACM0      = /dev/rrc    ★ 底盘控制板
   ├─ 口 3  1-2.3    icSpring 32e6:9005 → /dev/video0,1                  ★ 单目摄像头
   └─ 口 4  1-2.4    QinHeng Hub 1a86:8091  ← 又套一层 Hub
      ├─ 1-2.4.1    CH340  1a86:7523   → /dev/ttyCH341USB1   静默，身份未确认（#14）
      └─ 1-2.4.2    JMTek  0c76:161f   → 声卡 0 + event1      ★ USB 声卡 / 麦克风

另有 1-3 = Realtek 13d3:3549 蓝牙（板载 WiFi/BT 模块，非外部口）
USB3 侧（bus 2）的 4 口 Hub 上无任何设备
```

> ⚠️ 两个 CH340（`1a86:7523`）**都没有序列号**，无法按 ID 区分，只能按物理路径匹配。`/dev/lidar` 的 udev 规则绑定的是 `1-2.1:1.0`，**更换 USB 插口会失效**（见 D-018）。
> ⚠️ `/dev/lidar` 原先被 udev 错误地指向 `ttyACM0`（底盘串口），已于 2026-09-28 修正（见已知问题 #4）。

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
