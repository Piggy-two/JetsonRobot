# embodied_semantic_skills — Semantic Skill 层（task-tier）

> 第一个技能：**`look_for`** —— 看一眼视野里有没有某个类别的东西。

## 🔒 本技能的全部价值在**状态映射**

| 视觉那边的答复 | 本技能报的终态 | 含义 |
|---|---|---|
| 看到了 | `TARGET_FOUND` | 正面证据 |
| **确认没有**（画面质量达标） | `TARGET_LOST` | 一个**可信的否定** |
| **不知道**（画面糊 / 没新鲜帧 / 模型没就绪 / 类别不在表里） | **`FAILED`** | ⚠️ **不是"没有"** |

**为什么这一格最容易写反、也最贵**：把"不知道"报成"没有"，上层就会得出
"这里没有目标"的结论 —— 而真相可能只是**镜头糊了**。2026-10-09 我们就是这么栽的：
相机失焦时所有检测都是 0 个，而它和"真没东西"**看起来一模一样**
（`docs/DEV_NOTES.md` 坑 43）。⇒ 与 D-028 同一条铁律：**不知道 ≠ 没有**。

映射规则在 `look_plan.py`（**纯 Python**，有单测）。

## 用

```bash
# 前置：Vision Driver 在跑（它才有 ~/find_in_view）
ros2 launch embodied_vision_driver vision_driver.launch.py
ros2 launch embodied_semantic_skills semantic_skills.launch.py

ros2 service call /semantic_skills/look_for embodied_skills_interfaces/srv/LookFor \
  "{label: person, min_score: 0.3}"
```

⚠️ 视觉不在时本技能**不报错**：它返回 `FAILED` 并说清"视觉服务不可用 —— 不知道，
不是'没有'"。**没在看 ≠ 没有**，同上。

⚠️ 本技能**只读、不动**：网关注册表里 `causes_motion: false`，
不受 `allow_motion` 闸门约束，也不需要人看护。

## 它自己**不判**任何事

本节点不碰相机、不碰模型，只调 `~/find_in_view` 然后把答复**翻译成终态**。
于是"什么算可信"的规则**只有一处**（`vision_query.py`）——
在这里再判一次就会有两份规则，而两份规则**迟早分家**（D-034 的老教训）。

⚠️ **单目相机**：`side` 只是画面里的左右，**不含距离**（D-017）。
所以"确认看到人"**不等于**"人离我多远"。

## 进 Agent 的技能清单

它已在网关的注册表里（`semantic.look_for`，`tier: task`，`causes_motion: false`），
所以 **LLM 那一跳能选到它** —— 于是"看看前面有没有人"这类话可以被**规划**出来。

## 测试

```bash
cd src/embodied_semantic_skills && python3 -m pytest test/ -q
```
