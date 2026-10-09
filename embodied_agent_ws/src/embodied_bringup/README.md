# embodied_bringup — 一键拉起本项目的整栈

> ⚠️ **它不是运行时组件，也不含任何安全逻辑。** 它唯一做的事是：把九个节点按
> "能跑起来"的组合摆好，并把**当前的安全姿态打印在启动日志里**。
> 真正决定安危的仍然是各节点自己的闸门默认值。

## 为什么有这个包

在此之前，"起一次栈"要**手敲六条 `ros2 launch`**（各验收文档里都是那六行）。
参数之间有耦合（D-036 / D-037），**起错一个就表现得像功能坏了**：

| 起错的地方 | 症状 | 真相 |
|---|---|---|
| `allow_motion` 没开 | 任务被拒："拒绝派发可能引起运动的技能" | 闸门在正常工作 |
| `rules_file` 没留空 | LLM 那一跳**根本不会被问到** | 验的不是你想验的东西 |
| Safety 没起而 `require_safety` 还是默认 | Motor Driver 报状态 5，车不动 | D-037 的否决在生效 |
| 避障守卫还开着 | 一往下令就锁存 | D-036 的守卫在生效 |

## 用法

```bash
# 先确认厂商栈在跑（/scan 与遥测都来自它，本包不拉厂商栈）
systemctl status start_app_node.service

# ① 看：车不会动（干跑），**推荐先用这条确认链路**
DEEPSEEK_API_KEY=... ros2 launch embodied_bringup demo.launch.py \
    llm_enabled:=true allow_motion:=true \
    llm_base_url:=https://api.deepseek.com/v1 llm_model:=deepseek-flash \
    llm_api_key_env:=DEEPSEEK_API_KEY

# ② 真动：⚠️ 人工看护 + 手能直接断电（本机没有物理急停 #22）
DEEPSEEK_API_KEY=... ros2 launch embodied_bringup demo.launch.py \
    dry_run:=false allow_motion:=true llm_enabled:=true ...（其余同上）
```

密钥**不要写进这里**：`llm_api_key_env` 只收**变量名**；密钥本身放仓库之外
（例如 `~/.config/jetsonrobot/deepseek.env`，600 权限），用前 `source` 一下。

## 三个默认值都在**保守**那一侧

| 参数 | 默认 | 为什么 |
|---|---|---|
| `dry_run` | `true` | 本机没有物理急停（#22）—— "车会动"必须由人**显式**打开 |
| `allow_motion` | `false` | 三层闸门（D-033）。⚠️ 它**独立于** `dry_run`：两个都要开，车才可能动 |
| `obstacle_guard` | `true` | 它是**安全功能**（D-036）。只在做验收时才关（守卫会拦住"正被命令前进"的验收本身） |
| `llm_enabled` | `false` | 打开它意味着**用户的话会发往第三方** |
| `with_vision` | `true` | 视觉那一路（Vision Driver + Semantic Skill）。🔒 它**只读、不动**（`causes_motion: false`），所以默认开着；不想占 GPU 或做与视觉无关的验收时关掉 |
| `rules_file` | 空 | 与 D-038 一致：空表 ⇒ 一切都走 LLM 那一跳 |

## 一个已知的**起动窗口**（实测，约 3 秒）

即使把 Safety 排在前面，进程起来仍有先后：Motor Driver 在**看不到**安全层状态时
会报 `状态 -> safety_blocked`（D-037 的否决在生效）。实测它在 Safety 开始发状态后
**自动恢复**（约 **3 秒**后转回 `no_cmd`，状态码 1），**不需要人工干预**。

⇒ **含义**：栈刚起的那两三秒里，车是**动不了**的（这正是 D-037 想要的方向）。
把任务提交放在那之后即可，别把这几秒的拒绝当成故障。

## 与验收工装的关系

它只**起栈**，不**判定**。判定在 `tools/` 下。**完整演示步骤（含"摆位决定能看到哪一拍"）见 [`docs/DEMO.md`](../../../docs/DEMO.md)**：


| 工装 | 验什么 | 要不要人 |
|---|---|---|
| `llm_planner_acceptance.py` | 接口与失败路径（假端点） | 不要 |
| `real_llm_probe.py` | 真模型的**效果**（只记不判分） | 不要 |
| `upper_layer_dryrun_acceptance.py` | 上层链路 + 干跑积分 | 不要 |
| `ground_motion_acceptance.py` | 走到位了没有（**地面**） | **要** |
| `suspended_motion_acceptance.py` | 通电驱动 + 停得住（**架空**） | **要** |
| `agent_ground_acceptance.py` | 一句话 → 真模型 → **真车运动**（独立基准：`/scan` + IMU） | **要** |
