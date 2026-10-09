# embodied_semantic_skills — Semantic Skill 层（task-tier）

> 两个技能：
> **`look_for`** —— 看一眼视野里有没有某个类别的东西；
> **`look_on_side`** —— 看**画面的某一侧**有没有。

## 🔒 全部价值在**状态映射**（两个技能共用同一张表）

| 视觉那边的答复 | 本层报的终态 | 含义 |
|---|---|---|
| 看到了 | `TARGET_FOUND` | 正面证据 |
| **确认没有**（画面质量达标） | `TARGET_LOST` | 一个**可信的否定** |
| **不知道**（画面糊 / 没新鲜帧 / 模型没就绪 / 类别不在表里 / `side` 写错） | **`FAILED`** | ⚠️ **不是"没有"** |

**为什么这一格最容易写反、也最贵**：把"不知道"报成"没有"，上层就会得出
"这里没有目标"的结论 —— 而真相可能只是**镜头糊了**。2026-10-09 我们就是这么栽的：
相机失焦时所有检测都是 0 个，而它和"真没东西"**看起来一模一样**
（`docs/DEV_NOTES.md` 坑 43）。⇒ 与 D-028 同一条铁律：**不知道 ≠ 没有**。

映射规则在 `look_plan.py`（**纯 Python**，有单测）——**两个技能走的是同一个函数**，
不是两份实现。

## 两个技能的分工

| 技能 | 问的是 | `TARGET_LOST` 的含义 |
|---|---|---|
| `look_for` | 整幅画面里有没有 `<label>` | **整幅**都没有 |
| `look_on_side` | **`side` 那一侧**有没有 `<label>`（`'left'` / `'right'`） | **那一侧**没有（另一侧不算数） |

**为什么"左边有没有人"需要第二个技能**：本项目的 Agent **只能对终态做反应**
（条件就是 `when: {prev: <终态>}`）—— 它**没有"比较数值"这种东西**。
而 `look_for` 给的是 `side` 这个**数**。⇒ 那个问题在今天**根本问不出来**。
`look_on_side` 做的事就是把**数值问题**变成**状态问题**。

⚠️ 也正因为"哪一侧"会**改变 `TARGET_LOST` 的含义**，它是**另一条注册表项**
而不是 `look_for` 的一个可选参数 —— 同一个名字担两种含义是本项目一贯拒绝的
（`LookOnSide.srv` 里 `position` 刻意不叫 `side`，是同一个理由）。

## 用

```bash
# 前置：Vision Driver 在跑（它才有 ~/find_in_view）
ros2 launch embodied_vision_driver vision_driver.launch.py
ros2 launch embodied_semantic_skills semantic_skills.launch.py

ros2 service call /semantic_skills/look_for embodied_skills_interfaces/srv/LookFor \
  "{label: person, min_score: 0.3}"

ros2 service call /semantic_skills/look_on_side embodied_skills_interfaces/srv/LookOnSide \
  "{label: person, side: left, min_score: 0.3}"
```

⚠️ 视觉不在时本层**不报错**：它返回 `FAILED` 并说清"视觉服务不可用 —— 不知道，
不是'没有'"。**没在看 ≠ 没有**，同上。

⚠️ `side` 只认 `'left'` / `'right'`，**写别的会如实回 `FAILED`**（"不知道"），
**不会**静默当成"不限" —— 那会让"只看左边"变成"整个画面都看"，
而结果**看起来完全正常**（与"类别名不在表里"同族：一个永远不会报错的错答案）。

⚠️ **空值也一样被拒**：空**不是**"不限"，是"**没问**"（2026-10-09 实测三条路）——
经**网关** ⇒ 空串被网关自己拒（`REJECTED: 参数 side 是空字符串`）、压根不给 ⇒
`缺少必填参数 ['side']`；**直接**调 `~/look_on_side` ⇒ `FAILED`（`look_plan.check_asked_side`）。
要查整幅就调 `look_for` —— 那**是另一个问题**（两处的 `TARGET_LOST` 含义不同）。

⚠️ 本层技能**只读、不动**：网关注册表里 `causes_motion: false`，
不受 `allow_motion` 闸门约束，也不需要人看护。

## 它自己**不判**任何事

本节点不碰相机、不碰模型，只调 `~/find_in_view` 然后把答复**翻译成终态**。
于是"什么算可信"的规则**只有一处**（`vision_query.py`）——
在这里再判一次就会有两份规则，而两份规则**迟早分家**（D-034 的老教训）。
`side` 的取值校验**也**不在这一层（注册表只能查"是非空字符串"），
它由 `vision_query.check_side` 挡住 —— 同样是**只有一处**。

⚠️ **单目相机**：`side` / `position` 只是画面里的左右，**不含距离**（D-017）。
所以"确认看到人"**不等于**"人离我多远"。

## 进 Agent 的技能清单

两条都在网关的注册表里（`semantic.look_for`、`semantic.look_on_side`，
`tier: task`，`causes_motion: false`），所以 **LLM 那一跳能选到它们** ——
于是"看看前面有没有人""左边有没有人"这类话可以被**规划**出来。

⚠️ LLM 的菜单只带参数**名字与类型**、**带不了枚举** —— 所以
`side` 只认 `'left'` / `'right'` 这件事写在**注册表的 description 里**，
模型才看得见。加新技能时同理：**模型不知道的约定，等于不存在**。

## 测试

```bash
cd src/embodied_semantic_skills && python3 -m pytest test/ -q
cd src/embodied_vision_driver   && python3 -m pytest test/ -q   # side 判据在那边
```
