# DEVELOPMENT_LOG.md — 开发日志

> 按日期记录**重要**开发过程：做了什么 / 为什么这么做 / 测试结果 / 遇到的问题 / 最终结果。
>
> **不记录无意义的小修改。** 只记录对项目走向有影响的开发过程。

---

## 2026-09-28（续）— LiDAR `/scan` 验收通过 + 相机接入厂商 `usb_cam` 分支

### 做了什么

按用户决策，把上一次侦察留下的两件事**合并到同一次重启**里做完：

1. **确认 `ttyCH341USB1` 身份** —— 跑 `udevadm info -q property / -a`，取 VID:PID、接口类与 USB 物理路径
2. **改厂商机型配置** —— `~/ros2_ws/.typerc:9` 的 `DEPTH_CAMERA_TYPE` 由 `aurora` 改为 `usb_cam`（改前备份 `.typerc.bak-20260928`）
3. **首次重启厂商整机栈** —— `sudo systemctl restart start_app_node.service`（此前查明 bringup 由该 systemd 服务拉起，`Type=simple` / `Restart=always` / `KillMode=mixed`）
4. **验证 `/scan`** —— 频率、角度范围、量程、点数、有效率、`frame_id`、TF
5. **验证相机** —— 出图、编码、设备侧协商速率、下游 app 兼容性、资源占用
6. **全程监控 `/controller/cmd_vel`** —— 重启前后各采样，确认没有任何运动指令上线

### 为什么这么做

- **LiDAR 优先级最高**：按 D-017，LiDAR 是避障与建图的**唯一**外部测距来源；udev 根因修好后只差"重启验证"这一步（根因见上一则日志与已知问题 #4）。
- **相机按 D-019 用厂商既有分支**：厂商 `usb_cam_param.yaml` 与实机摄像头逐项吻合，改一行配置即可，属"用既有能力"而非"造新能力"，代价最小。
- **两件事合并成一次重启**：每重启一次整机栈都是对运行中系统的一次中断与风险，能合并就合并。
- **重启前先确认底盘安全**：重启前实测 `/controller/cmd_vel` 无消息，且查明 4 个厂商 app 需显式调用 `set_running` 才发运动指令 —— 即重启只会回到"默认关闭"状态。

### 实测结果

**① `/scan` 验收通过**（这是 Phase 0 至今**第一项**可标"通过"的能力）

| 项 | 实测值 |
|---|---|
| 驱动日志 | `ldlidar node start is success` → `ldlidar communication is normal.` → `Publish topic message:ldlidar scan data.` |
| 节点 / 话题 | `LD19` → `/scan` |
| 频率 | **10.00 Hz**（`scan_time` 0.1000 s，33 条消息标准差 0.00007 s） |
| 角度 | 0.0° ~ 360.0°，`angle_increment` 0.7143°~0.7186° |
| 每帧点数 | 502 ~ 505 |
| 量程 | 声明 `[0.02, 25.0] m`；实测回波 167 mm ~ 5265 mm |
| 有效点 | **93.5% ~ 97.0%**（每帧 14~33 点 NaN = 无回波，正常） |
| `frame_id` | `lidar_frame` |
| TF | ✅ `base_link → lidar_frame` = `[0.011, 0, 0.136]`，单位四元数 |

**对照**：修复前的启动日志里，同一个节点在 `/dev/lidar` 指向底盘串口时是 `ldlidar communication is abnormal.` + `process has died [exit code 1]` —— **一条日志就把根因坐实了**。

**② 相机接入厂商 `usb_cam` 分支**

| 项 | 实测值 |
|---|---|
| 节点日志 | `Starting 'usb_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS` |
| 设备侧协商 | `v4l2-ctl --get-parm` → **640×480 YUYV @ 30.000 fps**（`Size Image` 614400 B） |
| 话题 | `/depth_cam/rgb0/image_raw`：**0 个发布者 → 1 个**（`usb_cam`），`encoding=yuv422_yuy2` |
| 话题速率 | ⚠️ 实测 **10.75 / 10.27 Hz**（两次），`camera_info` 10.87 Hz —— **未达设备侧 30 fps**（见「遇到的问题」） |
| 下游兼容 | 话题名经 remap 后与 aurora 分支**同名**，`yolo` 重启后继续订阅同一话题，**app 侧零改动** |
| 资源 | `yolo` 随即开跑：**RSS 1010 MB（13.2% 内存）、23.5% CPU**；整机 load 1.34，可用内存 2.8 G |

**③ 安全：重启全程底盘静默**

重启前后多次采样 `/controller/cmd_vel`（`ros2 topic hz` / `echo --once`）**均无任何消息**。5 个发布者节点（`lidar_app` / `line_following` / `object_tracking` / `self_driving` / `joystick_control`）重启后**只注册发布者、不发消息**，且各自带 `set_running` / `enter` / `heartbeat` 服务 —— 坐实**默认关闭、需显式激活**。

**④ `ttyCH341USB1` 定位**（详见已知问题 #14）

`1a86:7523` CH340，路径 `…/usb1/1-2/1-2.4/1-2.4.1/1-2.4.1:1.0/tty/ttyCH341USB1`，接口类 `ff/01/02`，**无厂商字符串、无序列号**（与雷达同型号芯片，描述符无法区分）。该口**不在任何厂商 udev 规则的候选路径中**；它与同层 Hub 上的 USB 声卡（`0c76:161f`）构成"Hub + CH340 + 声卡"的复合形态，**与 `xf_mic.rules` 期望的讯飞环形麦同形**，只是插在口 4 而非规则写死的口 3（口 3 现被相机占用）。

### 遇到的问题

1. **`ros2 topic echo` 会把长数组省略成 `- '...'`** —— 捕获的 `/scan` 只有 129 行数据，一度让人以为"雷达每帧只出 129 个点"。改用 `rclpy` 直接读 `msg.ranges` 才拿到真值 **502~505**。**教训与上次的 `dd`、CRC 同类：测量工具本身会骗人，关键数值必须交叉验证。**
2. **相机话题速率只有设备侧的 1/3（~10.6 Hz vs 30 fps）—— 原因未定**。候选：① 发布环本就 ~10.6 Hz；② Python 订阅侧丢帧（614 KB/帧 × 30 fps ≈ 18 MB/s）；③ 同 Hub 上等时音频与两个 CH340 分走带宽。`camera_info`（极小消息）曾出现 0.027 s 间隔（≈37 Hz），提示发布器会突发 → 倾向 ②。**未下结论，列为已知问题 #15**，需用 C++ 订阅端或 `usb_cam` 自身计数确认。
3. **图像 `frame_id=camera` 不在 TF 树中**（树里是 `camera_link0`），另有孤立静态 TF `ascamera_camera_link_0 → depth_cam_color_frame` → 列为 #16。
4. **`99-usb-cam.rules` 指向不存在的脚本** `~/.dtb/.link_yuyv_camera.sh` → 规则空转，系统里**没有稳定的相机符号链接** → 列为 #17。
5. **麦克风与相机抢同一个 Hub 口**：`xf_mic.rules` 期望麦克风串口在 `1-2.3.1`，而该口现在是相机 → 这是 Phase 5 语音的前置障碍（#14）。
6. `usb_cam` 启动时报 `unknown control 'white_balance_temperature_auto'` 与 `white_balance_temperature: Permission denied`（参数文件里的白平衡设置在**这台相机上不存在/只读**）→ 白平衡不会被应用，可能影响 `line_following` 的颜色阈值，留待相机收口时处理。

### 最终结果

- **LiDAR `/scan` 实测通过**，接口清单中 LiDAR 项首次标记"通过"，Phase 0 的硬骨头只剩**底盘与急停运动验收**。
- 相机由"0 个发布者"变为**已出图**，厂商 app 与 `yolo` 无需改动。
- 已知问题：**#4 / #11 / #13 关闭**，#10 / #14 更新，**新增 #15（相机速率）/ #16（相机 TF 帧）/ #17（udev 悬空脚本）**。
- 新增决策 **D-019**（相机接入路线），D-017 中"相机 Driver 一律在 Overlay 自建"的表述按 D-019 修订。
- 系统侧改动两处，均已备份、可回滚：`/etc/udev/rules.d/lidar.rules`（上一次）与 `~/ros2_ws/.typerc`（本次）。

---

## 2026-09-28 — 硬件接口全量侦察：USB 拓扑 / LiDAR 型号确认 / 相机定性为单目

### 做了什么

用户提出「设备上四个 USB 口都接了东西，能否检测到」，由此对**全部外部硬件接口**做了一次系统侦察：

1. **四个 USB 口逐口识别** —— 含 Hub 拓扑（发现口 4 下还套了一层 Hub）
2. **LiDAR 型号实测确认** + 数据质量验证（协议、CRC、点频、角度覆盖）
3. **相机定性** —— 确认全系统只有**一个单目**摄像头，无任何深度设备
4. **定位并修复**「LiDAR 无 `/scan`」的根因（系统 udev 规则错配）
5. **定位**厂商机型配置 `.typerc` 与实际硬件的错配
6. 取得**运行中进程的真实环境变量**（`/proc/<pid>/environ`），坐实厂商栈的实际加载配置

侦察期间厂商 `bringup` 整机栈**处于运行状态**（18 个节点在线）。全程**只读**：未发布任何命令、未动车、未改动厂商 `ros2_ws` / `third_party`。**唯一的写操作是修正一个系统 udev 规则**（修改前已备份）。

### 为什么这么做

- Phase 0 的交付物是**接口清单**，而「下一步计划」第 2、3 项（LiDAR / 相机）此前一直卡在"先查设备连接"这一步，无法推进。
- 已知问题 #4 记的是「LiDAR 未就绪（型号亦未确认），先查物理连接」——**连接状态本身就没查过**，属于必须补上的侦察。
- 这类侦察必须在**不动车**的前提下完成，因此全程只读。

### 实测结果

**USB 拓扑**（`lsusb -t` + `/sys/bus/usb/devices` + `udevadm`）

```text
Jetson 根 Hub (bus 1)
└─ 端口 2 ─→ Realtek 4 口 Hub  ← 机器人扩展板的 4 个 USB 口，4 口全满
   ├─ 口 1  1-2.1    CH340  1a86:7523   → /dev/ttyCH341USB0 = /dev/lidar  ★ LD19 激光雷达
   ├─ 口 2  1-2.2    CH9102 1a86:55d4   → /dev/ttyACM0      = /dev/rrc    ★ 底盘控制板
   ├─ 口 3  1-2.3    icSpring 32e6:9005 → /dev/video0,1                  ★ 单目摄像头
   └─ 口 4  1-2.4    QinHeng Hub 1a86:8091  ← 又套一层 Hub
      ├─ 1-2.4.1    CH340  1a86:7523   → /dev/ttyCH341USB1   静默，身份未确认
      └─ 1-2.4.2    JMTek  0c76:161f   → 声卡 0 + event1      ★ USB 声卡 / 麦克风

另有 1-3 = Realtek 13d3:3549 蓝牙（板载 WiFi/BT 模块，非外部口）
USB3 侧（bus 2）的 4 口 Hub 上无任何设备
```

> 口 2 被 `ros_robot_controller`（PID 2884）与 PID 689 以 `F....` 方式持有 → 底盘链路确认走 `ttyACM0`。

**LiDAR：型号与健康度**（230400 波特率被动读取，**未启动任何驱动**）

| 项 | 实测 | 结论 |
|---|---|---|
| 协议 | `0x54 0x2C` 帧头，包长**恒为 47 字节** | LD19/LD06 协议 |
| CRC | 多项式 `0x4D`，**1248/1249 包通过（99.9%）** | 报文真实有效，非噪声 |
| 字节守恒 | 416.5 包/秒 × 47 B = **19576 B/s**，端口实测 **19579 B/s** | 偏差 0.01% → 每一字节都被包结构解释 |
| 点频 | **4992 点/秒** | 规格 4500 点/秒（10 Hz × 450） |
| 角度覆盖 | 起始角 2.5° → 356.3°，488 个不同起始角 | **完整 360°** |
| 测距 | 14976 点**全部有效**（166~2453 mm，中位 262 mm） | 零无效点 |

→ **雷达本身完全健康。** 型号确认为 **LD19**（与厂商 `LIDAR_TYPE=LD19` 一致）。

踩坑记录：首次用 `dd bs=4700 count=1` 采样只得到 64 字节，误以为速率极低。实为 **`dd` 在指定 `count` 时每次只做一次 `read()`**，取到的是瞬时缓冲量，**不是速率测量**。改用 `select` 连续读取 6 秒后才得到真实速率。另：不做 CRC 校验的朴素 `54 2C` 同步会混入假包（曾出现 63020 mm 的超量程值），**CRC 过滤后无效点归零**。

**相机：定性为单目**（`v4l2-ctl` + UVC 描述符 + 全总线枚举）

| 项 | 实测 |
|---|---|
| 摄像头数量 | **全系统仅 1 个**：UVC `32e6:9005`「icspring camera」@ `1-2.3` |
| 节点 | `/dev/video0` = Video Capture（YUYV 640×480@30）；`/dev/video1` = **Metadata Capture**（`UVCH` UVC 载荷头元数据，**不是深度流**） |
| 格式 | UVC 描述符中**仅 1 种格式**（`guidFormat 32595559…` = YUYV），无任何深度/IR 的 GUID；控制项无 ToF/IR 相关 |
| 深度设备 | ❌ 无 Deptrum（`3251`）、无 Orbbec（`2bc5`）；CSI 侧 `tegra-camrtc` 亦无 sensor 注册 |

**运行中厂商栈的真实配置**（`/proc/<bringup_pid>/environ`）

```text
LIDAR_TYPE=LD19          DEPTH_CAMERA_TYPE=aurora      MACHINE_TYPE=ROSOrin_Mecanum
need_compile=False       ROS_DISTRO=humble             ROS_DOMAIN_ID=0
```

定义位置：`~/ros2_ws/.zshrc` → `.robotrc` → **`.typerc`**（机型配置文件，逐项含 `LIDAR_TYPE` / `DEPTH_CAMERA_TYPE` / `MIC_TYPE=xf` / `ASR_MODE=online`）。

### 遇到的问题

1. **`/dev/lidar` 被 udev 指向了底盘串口**（🔴 本次最重要发现，也是已知问题 #4 的真正根因）

   ```text
   /etc/udev/rules.d/lidar.rules 中启用的规则:   KERNELS=="1-2.2:1.0"  → SYMLINK+="lidar"
   而 1-2.2 是底盘 CH9102 → ttyACM0 (=/dev/rrc)
   真正的雷达在 1-2.1  → ttyCH341USB0
   ```

   链条：雷达健康但 `ldlidar` 节点要打开 `/dev/lidar`（=`ttyACM0`，已被底盘进程独占）→ **节点起不来** → ROS 图中无 `/scan`。这解释了 2026-09-23 基线里「无 `/scan`」与 syslog 里 `wait device insert...` 的全部现象，**根因不在硬件**。

   另注：厂商自带的 `usb_ch341` 驱动把 CH340 命名为 `/dev/ttyCH341USB*`（而非内核标准的 `ttyUSB*`），这也是「按 `ttyUSB0` 找不到雷达」的表面原因。

   **处置**：改 `lidar.rules` 为 `KERNELS=="1-2.1:1.0"`（旧行保留为注释并写明原因），备份为 `lidar.rules.bak-20260928`。`udevadm test` **dry-run 先验证**（雷达设备 → `LINK 'lidar'`；底盘设备 → `Removing/updating old device symlink '/dev/lidar', which is no longer belonging to this device`），再**只对雷达那一个设备**触发 uevent（该口无人占用）。

   ```text
   修正前: /dev/lidar -> ttyACM0        (底盘串口，错)
   修正后: /dev/lidar -> ttyCH341USB0   (LD19 雷达)
   /dev/rrc -> ttyACM0 未变；底盘进程 PID 689 / 2884 未受影响
   ```

2. **Aurora930 是深度相机，不是 LiDAR** —— 此前基线把 `aurora930_node` 当成雷达驱动来排查（已知问题 #8），方向错了。它是 Deptrum 的**深度相机**（VID `3251`）驱动，本机无此设备，故节点 `21:12:20` 报 `No deptrum device connected! It's going to quit...` 后退出。

3. **`.typerc` 机型配置与实机不符** —— `DEPTH_CAMERA_TYPE=aurora` 期望深相机，实机是单目。旁证：厂商 `usb_cam` 分支的 `usb_cam_param.yaml`（`/dev/video0` + `yuyv` + 640×480）**与实机摄像头完全吻合**，且 `usb_cam` 包**已随系统安装在 `/opt/ros/humble`** —— 说明本机本该走 `usb_cam` 分支，`.typerc` 的机型填错了。**未改动**（属厂商配置，D-009）。

4. **`ttyCH341USB1` 身份未确认** —— 口 4 副 Hub 下有一个静默 CH340。`xf_mic.rules` 期望的 `/dev/ring_mic`（口 `1-2.3.1`）在本机**不存在**，故疑其为讯飞环形麦的串口控制口（与同处 `1-2.4` 的 USB 声卡构成一个复合设备），也可能属云台/舵机控制器。**待实测**（新记已知问题 #14）。

5. **两个 CH340 均无序列号** —— `1a86:7523` 的 `serial` 字段为空，无法按 ID 区分雷达与另一路 CH340，**只能按物理路径匹配**。这意味着 `/dev/lidar` 规则绑定 `1-2.1:1.0`，**更换 USB 插口即失效**，是已知的脆弱点（已写入 D-018）。

### 最终结果

- 四个 USB 口**全部识别**，并建立完整 USB 拓扑图（已写入接口清单）
- **LiDAR 型号确认为 LD19**，且实测证明**设备健康**（CRC 99.9%、360° 完整、4992 点/秒）
- **相机定性为单目**，全系统无任何深度设备
- 新增决策 **D-017**（视觉基线为单目，不假设深度）与 **D-018**（LiDAR 型号与 `/dev/lidar` 接入约定）
- 已知问题：#4 / #6 / #8 / #11 **更新**（#6、#8、#11 定性关闭），新增 **#13**（`.typerc` 配置错配）、**#14**（`ttyCH341USB1` 身份）
- **`/scan` 尚未验证** —— 因未重启厂商栈（用户选择"改 udev 但不重启"），驱动未运行；`/scan` 验收需重启后完成
- 未发布任何运动命令、未动车、未改动厂商 `ros2_ws` / `third_party`；系统 udev 改动已备份

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
