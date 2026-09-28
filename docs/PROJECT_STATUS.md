# PROJECT_STATUS.md — 项目当前状态

> **这是 Claude Code 下一次会话快速恢复项目状态的主要文件。**
> 每次开发任务结束前必须更新本文件。
>
> **最后更新：2026-09-28**

---

## 1. 一句话状态

**项目处于 Phase 0 硬件验收阶段：设计文档与工程维护机制已建立、磁盘阻塞已解除，仍无业务代码。2026-09-28 完成两轮硬件验收 —— 第一轮为全量接口侦察（四个 USB 口逐口识别、LiDAR 型号实测确认为 LD19、相机定性为单目，决策 D-017 / D-018）；第二轮修掉 LiDAR 的 udev 错配并重启厂商栈验证，`/scan` **实测通过**（10.00 Hz、360°、`frame_id=lidar_frame`、TF `base_link→lidar_frame` 已存在），相机按 D-019 切到厂商 `usb_cam` 分支后 `/depth_cam/rgb0/image_raw` 已出图。Phase 0 现在只剩**底盘与急停的实机运动验收**（需车轮悬空或安全距离 + 人工看护）这块硬骨头 —— 运动仍**未测**。**

---

## 2. 当前阶段

### Phase 0：环境与硬件启动验收 🚧 进行中

目标：确认"已安装的软件包"与"当前小车上真实可用的硬件能力"一致。

> 仅发现包名、进程或 Workspace **不视为设备已验收**。

| 验收项 | 状态 | 说明 |
|---|---|---|
| 基线环境（ROS2 / Jetson / 磁盘 / 环境加载链） | 🟢 已完成 | ROS2 Humble、Jetson Orin（8GB，内存 7.4Gi）、磁盘已扩容至 116G；环境加载链与机型配置位置已记录（`.zshrc` → `.robotrc` → `.typerc`，原已知问题 #6 已关闭） |
| 底盘与安全（`ros_robot_controller` / `controller` / `kinematics` / `servo_controller`） | 🟡 进行中 | 命令链与 `/odom`(30.0 Hz) 已实测确认（见 §7）；**运动未测**（需急停 + 看护）；遥控链路 `/sbus`、`/joy`、`/button` 已确认存在 |
| LiDAR 与避障 | 🟢 **已完成（通过）** | 型号实测确认为 **LD19**，设备健康（CRC 99.9%、4992 点/秒，见 D-018）。原「无 `/scan`」根因是 udev 把 `/dev/lidar` 指向了底盘串口，**已修复并重启验证通过**：`/scan` **10.00 Hz**、360°、502~505 点/帧、有效回波 93.5~97.0%、`frame_id=lidar_frame`、TF `base_link→lidar_frame`（静态 `[0.011, 0, 0.136]`）存在。实测记录见 §7 |
| 相机与视觉 | 🟡 已出图，细节待收口 | 实测为**单目** UVC 摄像头（icSpring `32e6:9005`，YUYV 640×480@30，`/dev/video0`），全系统仅此一个摄像头（视觉基线 **D-017**）。已按 **D-019** 把 `.typerc` 切到厂商 `usb_cam` 分支 → `/depth_cam/rgb0/image_raw` 由 0 个发布者变为 **1 个**。⚠️ 话题速率仅 ~10.3–10.9 Hz（设备侧协商 30 fps，#15）；图像 `frame_id=camera` 不在 TF 树中（#16） |
| 语音与麦克风（`xf_mic_asr_offline`） | ⬜ 未开始 | 已知配置 `MIC_TYPE=xf` / `ASR_MODE=online`（⚠️ 在线 ASR，断网不可用）。USB 声卡 0（`0c76:161f`）已被内核识别，但 `/dev/ring_mic` 未建立 —— 根因已定位：`xf_mic.rules` 期望麦克风串口在 `1-2.3.1`，而该口现在是**相机**（#14） |
| SLAM / 导航 / 系统联调 | ⬜ 未开始 | 依赖上述全部通过 |
| **接口清单交付物** | 🟡 进行中 | 底盘 / 相机 / LiDAR 已填入实测值；**LiDAR 已可标"通过"**，语音与导航待补 |

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
| **LiDAR `/scan` 验收通过 + 相机接入厂商 `usb_cam`** | 重启厂商栈（`start_app_node.service`）验证：`/scan` 10.00 Hz / 360° / `frame_id=lidar_frame`、TF 已存在、有效回波 93.5~97.0%；相机按 D-019 改 `.typerc` 后出图（remap 后话题名与旧分支一致，下游 app 无需改动）；期间 `/controller/cmd_vel` **全程静默**，坐实厂商 4 个 app 重启后处于**未激活**状态（#10 更新） | 2026-09-28 |

---

## 4. 正在进行

- **无正在进行的代码开发。** 当前处于"文档与基础设施就绪、硬件验收收尾"的节点：LiDAR 已通过，剩余**相机收口**（#15/#16）、**语音前置**（`/dev/ring_mic`，#14）与**底盘运动验收**。

---

## 5. 已知问题

| # | 问题 | 影响 | 处理 |
|---|---|---|---|
| 1 | ~~根分区仅剩 3.0G（96% 已用）~~ **已解决** | ✅ 已解除 | 2026-09-23 在线扩容至 116G（可用 55G，52%），PARTUUID 保留；详见 `DEVELOPMENT_LOG.md`。剩余 ~119G 未纳入 GPT，**非阻塞** |
| 2 | **GitHub SSH 22 端口被网络封锁** | ✅ 已规避 | 已在 `~/.ssh/config` 配置走 `ssh.github.com:443`，实测可用 |
| 3 | ~~SSH 公钥未注册到 GitHub~~ **已解决** | ✅ 已解除 | 2026-09-23 实测 `git push` 成功（`ac3b350..1a102ba`） |
| 4 | ~~LiDAR 无 `/scan`~~ **已解决** | ✅ 已解除 | **2026-09-28 定位并修复**：雷达（LD19，物理口 `1-2.1`）本身健康，但 `/etc/udev/rules.d/lidar.rules` 启用的规则匹配的是 `1-2.2`（= 底盘串口 `ttyACM0`），`/dev/lidar` 指向底盘 → `ldlidar` 节点打开该口后 3 秒报 `ldlidar communication is abnormal` 并以 **exit code 1 退出**（旧启动日志实证）。改规则（备份 `lidar.rules.bak-20260928`）后重启厂商栈，日志变为 `ldlidar communication is normal` + `Publish topic message`，**`/scan` 10.00 Hz 实测通过**。型号依据见 **D-018**，验收数据见 §7 |
| 5 | **仓库尚无代码** | ⬜ 非缺陷 | 按 Phase 顺序引入，不要提前创建空模块 |
| 6 | ~~厂商栈加载顺序未记录~~ **已解决** | ✅ 已解除 | 加载链已记录：`~/.zshrc` → `~/ros2_ws/.zshrc` → `~/ros2_ws/.robotrc` → `~/ros2_ws/.typerc`。2026-09-28 另取得**运行中进程的真实环境**（`/proc/<pid>/environ`，含完整 `AMENT_PREFIX_PATH` / `LD_LIBRARY_PATH` / `PYTHONPATH`）。**机型配置在 `.typerc`**（`LIDAR_TYPE` / `DEPTH_CAMERA_TYPE` / `MACHINE_TYPE` / `MIC_TYPE` / `ASR_MODE`），见已知问题 #13 |
| 7 | **内存仅 7.4Gi（8GB 版 Orin）** | 🟡 设计约束 | **实测：厂商 bringup 栈一启动即占用 ~5.5G（5472/7620MB）**，留给本项目的余量很小 → Phase 6/7 本地小模型与视觉并发必须按此预算设计；`/swapfile` 8G 是实际安全余量，**不要缩** |
| 8 | ~~syslog 中 `aurora930_node` 反复 `wait device insert...`~~ **已解释** | ✅ 已澄清 | **2026-09-28**：Aurora930 是**深度相机**（Deptrum，VID `3251`）驱动，**不是** LiDAR 驱动（此前记错）。本机无该设备，故节点反复等待后于 `21:12:20` 报 `No deptrum device connected! It's going to quit...` 并退出。根因是 `.typerc` 机型配置写成 `DEPTH_CAMERA_TYPE=aurora`，见已知问题 #13 与 **D-017** |
| 9 | `apt` 有 1008 个待升级包 | ⬜ 非缺陷 | 暂不升级（升级前需确认不影响厂商 SDK 与内核）；涉及 `linux-headers-generic` 元包，勿单独 purge |
| 10 | **厂商 app 可随时接管底盘** | 🔴 安全 | 实测 `/controller/cmd_vel` 有 **5 个发布者**（`lidar_app` / `line_following` / `object_tracking` / `self_driving` / `joystick_control`），各自带 `enter` / `heartbeat` / `set_running` 服务，**被激活即开始发运动指令**。**2026-09-28 补充实测**：整栈重启后这些节点只是**注册了发布者但不发消息**（`/controller/cmd_vel` 连续采样全程静默）—— 即**默认关闭、需显式 `set_running` 才动**。这降低了重启风险，但**不改变结论**：运动测试前必须确认它们未激活，并保留急停与人工看护 |
| 11 | ~~相机有 publisher 但无数据流~~ **已解决** | ✅ 已解除 | **2026-09-28 定性**：不是"有 publisher 无数据"，而是**话题挂在一个不存在的深度相机上**（`DEPTH_CAMERA_TYPE=aurora`）。按 **D-019** 切到厂商 `usb_cam` 分支后，`/depth_cam/rgb0/image_raw` 由 0 个发布者变为 **1 个**，图像已实测出流。遗留细节另立 #15 / #16 |
| 12 | 厂商栈含机械臂/夹爪控制器 | ⬜ 非本项目范围 | `/arm_controller`、`/gripper_controller` 提供 `follow_joint_trajectory`；按 `DECISIONS.md` D-012 第一版不做机械臂，仅记录其存在 |
| 13 | ~~`.typerc` 机型配置与实机硬件不符~~ **已解决** | ✅ 已解除 | `~/ros2_ws/.typerc:9` 原为 `DEPTH_CAMERA_TYPE=aurora`（期望深相机），与实机的单目 USB 摄像头不符。逐项核对：`LIDAR_TYPE=LD19` ✅ 一致；`DEPTH_CAMERA_TYPE` ❌ 不符。厂商 `usb_cam` 分支参数（`/dev/video0` + `yuyv` + 640×480）与实机**逐项吻合**。**2026-09-28 按用户决策改为 `usb_cam`**（备份 `.typerc.bak-20260928`）并重启验证出图，决策与影响见 **D-019** |
| 14 | **副 Hub 上的 CH340（`ttyCH341USB1`）已定位，功能为高置信推断** | 🟡 待人工验证 | **2026-09-28 实测**：`udevadm info` → DEVPATH `…/usb1/1-2/1-2.4/1-2.4.1/1-2.4.1:1.0/tty/ttyCH341USB1`，`1a86:7523`（QinHeng CH340），驱动 `usb_ch341`，接口类 `ff/01/02`，**无厂商字符串、无序列号** → 与雷达**同型号芯片**，描述符层面无法区分。**该口 `1-2.4.1` 不在任何厂商规则的候选路径里**（`/etc/lidar.rules` 认领 `1-2.1`；`/etc/xf_mic.rules` 认领 `1-2.3.1`；厂商源码规则认领 `1-2.3.1.1`/`1-2.1.1.1`/`1-2.3.4`/`1-2.1.4`）。**结构上高度吻合讯飞环形麦**：`1-2.4` 是一层 QinHeng Hub，其口 1 是 CH340、口 2 是 USB 声卡（`0c76:161f`，Audio+HID）—— 与厂商规则期望的「Hub 下挂 CH340 + 声卡」复合设备**同形**，只是插在**口 4 而非规则写死的口 3**（口 3 现被相机占用）。**无任何进程持有该口**。**决定性验证（需人工在场）**：① 拔掉 `1-2.4` 看 `ttyCH341USB1` 与声卡 0 是否同时消失；② 把规则改指 `1-2.4.1:1.0` 后跑厂商 `xf_mic_asr_offline/startup_test.launch.py` 自检 |
| 15 | **相机话题速率远低于设备侧速率（~11 Hz vs 30 fps）** | 🟡 Phase 4 前必须收口 | 设备侧 `v4l2-ctl --get-parm` 协商 **640×480 YUYV @ 30.000 fps**，`usb_cam` 日志亦报 `at 30 FPS`；但 `/depth_cam/rgb0/image_raw` **四次独立实测全部落在 9.8 ~ 11.5 Hz**（10.75 / 10.27 / 9.79 Hz，以及最小间隔 0.081 s）。候选原因：① 发布环本就 ~11 Hz；② **Python 订阅侧丢帧**（614 KB/帧 × 30 fps ≈ 18 MB/s）；③ 同一 USB2 Hub 上等时音频与两个 CH340 分走带宽。**第三次实测的间隔分布高度均匀**（min 0.081 s / max 0.096 s / std dev 0.004 s，而非 33/67/100 ms 的丢帧特征）→ 更像①；但 `camera_info`（极小消息）曾出现 **0.027 s 间隔（≈37 Hz）**，与①矛盾。**证据不一致，故不下结论** —— 需用 C++ 订阅端或 `usb_cam` 自身帧计数确认，Phase 4 定帧率预算前必须定论 |
| 16 | **图像 `frame_id=camera` 不在 TF 树中** | 🟡 Phase 4 前置 | `usb_cam_param.yaml` 的 `frame_id: "camera"`，而 TF 树中的相机帧是 `camera_link0`（`base_link → camera_link0`），另有 `depth_cam` 分支留下的**孤立**静态 TF `ascamera_camera_link_0 → depth_cam_color_frame`。→ **图像消息的相机帧无法在 TF 中解析**（厂商遗留不一致）。做 D-007 的 `get_target_position()` 时必须显式补齐/指定相机帧，**不得假设图像帧可直接做 TF 变换** |
| 17 | **厂商 `99-usb-cam.rules` 指向不存在的脚本** | ⬜ 非阻塞 | `/etc/udev/rules.d/99-usb-cam.rules:3` 的 `RUN+="/home/ubuntu/.dtb/.link_yuyv_camera.sh"` 指向**不存在的脚本**（`~/.dtb/` 目录不存在）→ 规则空转，**系统里没有稳定的相机符号链接**（对比雷达有 `/dev/lidar`）。当前 `/dev/video0` 只是枚举顺序的结果，**多摄像头或重插后不保证稳定**。本项目若需固定相机路径，应在 Overlay 自建 udev 规则（不改厂商文件） |

---

## 6. 下一步计划

**优先级从高到低：**

1. **底盘与急停验收**（**当前 Phase 0 唯一剩下的硬骨头**，**必须**车轮悬空或留安全距离 + 人工看护）—— 前置条件已明确：5 个厂商 app **默认关闭**，需显式 `set_running` 才发运动指令（#10 已实测坐实），因此开测前只需确认它们未激活。测试 `stop()`、低速前进/后退/平移/原地旋转；验证速度上限、命令超时、通信中断停车、**遥控优先级**（`/ros_robot_controller/sbus` / `joy` / `button` 已确认存在）。从 `/cmd_vel` 发布测试（见 D-016）。
2. **相机链路收口**（#15 速率 / #16 frame_id）—— 数据已通（D-019），但要先把「话题实际 ~10.6 Hz 而设备 30 fps」的原因定下来，并补齐相机 TF 帧。**收口后再决定 D-019 留的口子：继续用厂商 `usb_cam`，还是在 Overlay 自建 Driver。**
3. **确认 `ttyCH341USB1` = 讯飞环形麦并解决 Hub 口冲突**（#14）—— 麦克风串口与相机在抢 Hub 口 3；这步不解决，Phase 5 语音无法开工。需人工在场（拔插验证）或改 udev 规则后跑厂商麦克风自检。
4. **恢复语音自检链路** —— 让 `/dev/ring_mic` 真正建立，使厂商 `startup_check` 能自动跑 `xf_mic_asr_offline/startup_test.launch.py`。
5. **语音验收** —— 音频设备、ASR 文本输出、"停/急停/取消任务"本地解析链路（必须绕过 LLM）。⚠️ 注意 `ASR_MODE=online`，需确认断网时的降级行为（D-006 要求安全指令不依赖网络）。
6. **产出接口清单** —— Phase 0 的交付物（见上，底盘/相机/LiDAR 已有实测值）。
7. 只有接口清单标记"通过"的能力，才允许封装为 Skill。

> 已完成：磁盘阻塞解除、SSH 公钥注册、**Phase 0 基线检查**（2026-09-23）、**硬件接口全量侦察**（2026-09-28）。若空间再度紧张，按 `DECISIONS.md` D-014 的顺序处理，并参考 D-015 的 PARTUUID 约束。

---

## 7. 接口清单（Phase 0 交付物 · 填写中）

> 2026-09-23 只读基线实测填写，2026-09-28 补齐 LiDAR / 相机 / 语音的设备级实测值。**"验收结果"列只有实机测试通过后才允许标"通过"** —— 目前 **LiDAR 一项已标"通过"**（2026-09-28 重启厂商栈实测），其余仍未通过。

| 模块 | 已确认驱动/包 | 启动入口 | 输入接口 | 输出接口 | TF / 设备路径 | 验收结果 |
|---|---|---|---|---|---|---|
| 底盘 | ✅ `ros_robot_controller`（硬件桥）+ `controller`/`odom_publisher`（运动学）+ `servo_controller` | `ros2 launch bringup bringup.launch.py`（实测在运行） | ✅ `/cmd_vel` 或 `/controller/cmd_vel`（`geometry_msgs/Twist`）→ `odom_publisher` → `/ros_robot_controller/set_motor`（`MotorsState`） | ✅ `/odom_raw` → `ekf_node` → `/odom`（**实测 30.0 Hz**，抖动 <1ms）；`/ros_robot_controller/{battery,button,imu_raw,joy,sbus}` | ⬜ TF 帧待确认 | 🟡 链路已实测；**运动未测** |
| LiDAR | ✅ **LD19**（`ldlidar_stl_ros2`，230400） | `peripherals/launch/include/ldlidar_LD19.launch.py`（由 `LIDAR_TYPE=LD19` 选择） | ✅ `/dev/lidar` → `ttyCH341USB0`（Hub 口 `1-2.1`） | ✅ `/scan`（`sensor_msgs/LaserScan`），**实测 10.00 Hz** | 设备 `/dev/lidar`；`frame_id=lidar_frame`；TF `base_link → lidar_frame` 静态 `[0.011, 0, 0.136]` | ✅ **通过**（2026-09-28 实测，数据见下） |
| 相机 | 单目 UVC（内核 `uvcvideo`）+ 厂商 `usb_cam` 分支 | 厂商 `peripherals/launch/depth_camera.launch.py`（`DEPTH_CAMERA_TYPE=usb_cam` → `usb_cam_node_exe`，见 D-019） | ✅ `/dev/video0`（YUYV 640×480@30，设备侧协商） | ✅ `/depth_cam/rgb0/image_raw`（1 个发布者，`encoding=yuv422_yuy2`）+ `/depth_cam/rgb0/camera_info` | 设备 `/dev/video0`；图像 `frame_id=camera`（**不在 TF 树中**，#16） | 🟡 已出图；速率与 TF 待收口（#15 / #16） |
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
      ├─ 1-2.4.1    CH340  1a86:7523   → /dev/ttyCH341USB1   无人持有；疑为讯飞环形麦串口（#14）
      └─ 1-2.4.2    JMTek  0c76:161f   → 声卡 0 + event1      ★ USB 声卡 / 麦克风

另有 1-3 = Realtek 13d3:3549 蓝牙（板载 WiFi/BT 模块，非外部口）
USB3 侧（bus 2）的 4 口 Hub 上无任何设备
```

> ⚠️ 两个 CH340（`1a86:7523`）**都没有序列号**，无法按 ID 区分，只能按物理路径匹配。`/dev/lidar` 的 udev 规则绑定的是 `1-2.1:1.0`，**更换 USB 插口会失效**（见 D-018）。
> ⚠️ `/dev/lidar` 原先被 udev 错误地指向 `ttyACM0`（底盘串口），已于 2026-09-28 修正（见已知问题 #4）。
> ⚠️ 厂商 udev 规则**按物理口写死**且候选路径互不相同（麦克风期望 `1-2.3.1`、雷达候选 `1-2.2`/`1-2.3.4`/`1-2.1.4`）—— **插错一个口，整条功能失效且不报错**。换口或加设备前先核对本节拓扑与规则。

**实测 `/scan`（2026-09-28 · LiDAR 验收通过）**：

| 项 | 实测值 |
|---|---|
| 节点 / 话题 | `LD19` → `/scan`（`sensor_msgs/LaserScan`） |
| 频率 | **10.00 Hz**（`scan_time` 0.1000 s；33 条消息标准差 0.00007 s） |
| 角度 | `angle_min=0.0`、`angle_max=6.2832`（**360.0°**）、`angle_increment` 0.7143°~0.7186° |
| 每帧点数 | **502 ~ 505**（随每圈实际点数微变） |
| 量程 | 设备声明 `[0.02, 25.0] m`；**实测回波 167 mm ~ 5265 mm** |
| 有效点比例 | **93.5% ~ 97.0%**（每帧 14~33 点无回波 = NaN，属正常） |
| `frame_id` | `lidar_frame` |
| TF | ✅ 静态 `base_link → lidar_frame` = 平移 `[0.011, 0, 0.136]`，旋转为单位四元数 |
| 驱动日志 | `ldlidar communication is normal.` + `Publish topic message:ldlidar scan data.` |

> 📌 **记录方式说明**：`ros2 topic echo` 对长数组会**省略**为一行 `- '...'`（本机实测 502 个点只打印出 129 行），**不能**用它判断点数或丢点。上表的点数 / 有效率来自 `rclpy` 订阅端直接读 `msg.ranges`。

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
