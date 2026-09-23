# PROJECT_STATUS.md — 项目当前状态

> **这是 Claude Code 下一次会话快速恢复项目状态的主要文件。**
> 每次开发任务结束前必须更新本文件。
>
> **最后更新：2026-09-23**

---

## 1. 一句话状态

**项目处于 Phase 0 起步阶段：设计文档已完成，工程维护机制已建立，尚无任何业务代码，硬件接口验收尚未开始。**

---

## 2. 当前阶段

### Phase 0：环境与硬件启动验收 🚧 进行中

目标：确认"已安装的软件包"与"当前小车上真实可用的硬件能力"一致。

> 仅发现包名、进程或 Workspace **不视为设备已验收**。

| 验收项 | 状态 | 说明 |
|---|---|---|
| 基线环境（ROS2 / Jetson / 磁盘） | 🟡 部分完成 | ROS2 Humble 确认；磁盘告警 |
| 底盘与安全（`ros_robot_controller` / `controller` / `kinematics` / `servo_controller`） | ⬜ 未开始 | 需实机测试 |
| LiDAR 与避障 | ⬜ 未开始 | 驱动型号待确认 |
| 相机与视觉（Orbbec） | ⬜ 未开始 | |
| 语音与麦克风（`xf_mic_asr_offline`） | ⬜ 未开始 | |
| SLAM / 导航 / 系统联调 | ⬜ 未开始 | 依赖上述全部通过 |
| **接口清单交付物** | ⬜ 未开始 | Phase 0 的最终产物 |

---

## 3. 已完成

| 项 | 说明 | 日期 |
|---|---|---|
| 架构设计方案 | `docs/plan.md`，29 章完整设计（Robot-as-Agent / 分层 Skill / Safety / Event-driven） | 2026-09-22 |
| 实机环境基线摸底 | 确认 Jetson Orin L4T R36.4.3、ROS2 Humble、厂商 `~/ros2_ws` 与 `~/third_party` 存在 | 2026-09-23 |
| Git 仓库与远程连接 | `origin` = `git@github.com:Piggy-two/JetsonRobot.git`，SSH over 443 已配置 | 2026-09-23 |
| 工程维护机制 | `README.md` / `CLAUDE.md` / `docs/` 四份长期文档 / `.gitignore` | 2026-09-23 |

---

## 4. 正在进行

- **无正在进行的代码开发。** 当前处于"文档与基础设施就绪、等待硬件验收"的节点。

---

## 5. 已知问题

| # | 问题 | 影响 | 处理 |
|---|---|---|---|
| 1 | **根分区仅剩 3.0G（96% 已用，58G/64G）** | 🔴 阻塞 | 部署 PyTorch / 模型权重 / TensorRT Engine / ROS bag / Docker 前必须先清理或扩容。**未确认分区布局前不要改分区** |
| 2 | **GitHub SSH 22 端口被网络封锁** | 🟡 已规避 | 已在 `~/.ssh/config` 配置走 `ssh.github.com:443`；SSH 公钥仍需注册到 GitHub 账号才能 push |
| 3 | **SSH 公钥未注册到 GitHub** | 🟡 待用户操作 | 公钥 `~/.ssh/id_ed25519.pub`（尾 `...eLXLE`）需加到 GitHub → Settings → SSH keys |
| 4 | **LiDAR 驱动型号未确认** | 🟡 待验收 | 候选：`ydlidar_ros2_driver` / `sllidar_ros2` / `sclidar_ros2` / `ldlidar_stl_ros2` / `Aurora930`。必须以实机 launch 与 ROS graph 为准 |
| 5 | **仓库尚无代码** | ⬜ 非缺陷 | 按 Phase 顺序引入，不要提前创建空模块 |
| 6 | 厂商栈加载顺序未记录 | 🟡 待补 | 需记录 `ROS_DISTRO` / `AMENT_PREFIX_PATH` / `PYTHONPATH` 与启动脚本来源 |

---

## 6. 下一步计划

**优先级从高到低：**

1. **解除磁盘阻塞** —— 确认根分区布局，清理或扩容到可用空间。这是所有后续部署的前置条件。
2. **完成 SSH 公钥注册** —— 使 `git push` 可用。
3. **执行 Phase 0 基线检查** —— `jtop` / `tegrastats` / `df -h /` / `ros2 node|topic|service|action list`，并记录厂商环境加载顺序。
4. **底盘与安全验收** —— 在车轮悬空或留安全距离条件下测试 `stop()`、低速前进/后退/平移/原地旋转；验证速度上限、命令超时、通信中断停车、遥控优先级；记录坐标系、麦轮运动学约定、`cmd_vel` 类型与单位。
5. **LiDAR 验收** —— 确认型号、设备路径、波特率；验证 `LaserScan` 频率/角度/量程/frame_id 与 TF。
6. **相机验收** —— Orbbec RGB/Depth/CameraInfo/TF；`cv_bridge` 收图最小验证。
7. **语音验收** —— 音频设备、ASR 文本输出、"停/急停/取消任务"本地解析链路（必须绕过 LLM）。
8. **产出接口清单** —— Phase 0 的交付物（见下）。
9. 只有接口清单标记"通过"的能力，才允许封装为 Skill。

---

## 7. 接口清单（Phase 0 交付物 · 尚未填写）

| 模块 | 已确认驱动/包 | 启动入口 | 输入接口 | 输出接口 | TF / 设备路径 | 验收结果 |
|---|---|---|---|---|---|---|
| 底盘 | 待实机确认 | launch / service | `cmd_vel` 或等效 | odom / status | base_link | 待测 |
| LiDAR | 待实机确认 | launch | 串口 / USB | `scan` | laser frame | 待测 |
| 相机 | Orbbec 待确认 | launch | RGB / Depth | image / CameraInfo | camera frame | 待测 |
| 语音 | 麦克风与 ASR 待确认 | launch | audio | text / intent | 音频设备号 | 待测 |
| 导航 | 待实机确认 | launch | goal / map | status / event | map / odom / base_link | 待测 |

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
