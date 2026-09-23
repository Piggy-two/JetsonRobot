# CLAUDE.md — JetsonRobot 长期开发规则

本文件是 Claude Code 在本项目中的**长期开发规则**。每次会话开始前必须遵守。

> 项目：基于 Jetson Orin 的混合式语音具身智能机器人平台
> 仓库：`git@github.com:Piggy-two/JetsonRobot.git`
> 当前阶段：Phase 0（环境与硬件启动验收）——仓库目前**只有设计方案，没有业务代码**

---

## 0. 项目一句话

> **LLM 负责思考，Skill 负责行动，Linux 负责落地，机器人整体就是 Agent。**

**LLM 绝不参与实时控制。** 任何进入控制环（电机 / PID / 急停 / 避障）的代码都不得依赖大模型。

---

## 1. 每次开始开发前（强制）

按顺序执行，理解状态后再动代码：

1. 阅读 `README.md`
2. 阅读 `docs/PROJECT_STATUS.md` ← **恢复项目状态的主要入口**
3. 按需阅读 `docs/ARCHITECTURE.md` 和 `docs/DECISIONS.md`
4. 执行 `git status` 和 `git log --oneline -10`
5. 确认"当前阶段 / 已完成 / 进行中 / 已知问题"后再开始

**不允许在未读 `PROJECT_STATUS.md` 的情况下直接修改代码。**

---

## 2. 修改代码前

- **先阅读相关代码**，不允许凭空猜测现有实现
- 不允许假设不存在的模块 —— 本仓库当前**没有** `agent_runtime/`、`skill_service/` 等目录，需要时再创建
- **优先修改现有模块**，不随意重复造文件
- 保持项目架构清晰，遵守分层：`Hardware → Driver → Primitive → Control Skill → Autonomous Skill → Semantic Skill → Agent Task`
- 越往下越确定、越实时、越不依赖 LLM；越往上语义越强、时间尺度越慢
- 新代码放在 Overlay Workspace（计划名 `embodied_agent_ws`），**不要修改厂商 `~/ros2_ws` 与 `~/third_party`**

---

## 3. 每完成一个明确开发任务

1. 编译或运行对应测试（ROS2 用 `colcon build`，Python 用对应测试；**无测试的改动必须说明如何手动验证**）
2. 检查错误与警告，不要忽略
3. 检查 `git diff`，确认没有误改、没有夹带无关文件
4. 总结本次修改

**不允许为了通过测试而绕过真正的问题。**

---

## 4. 修改影响文档时（强制同步）

| 如果修改影响… | 必须更新 |
|---|---|
| 架构、模块关系、数据流 | `docs/ARCHITECTURE.md` |
| 当前进度、已完成 / 进行中 / 下一步 | `docs/PROJECT_STATUS.md` |
| 重要开发过程（做了什么 / 为什么 / 测试结果 / 问题） | `docs/DEVELOPMENT_LOG.md` |
| 技术路线、设计选择及其原因 | `docs/DECISIONS.md` |

文档与代码在**同一个 commit** 内更新。文档滞后视为任务未完成。

---

## 5. Git 工作流

完成一个**可验证**的开发任务后：

```bash
git status          # 确认改动范围
git diff            # 逐行检查
# → 运行测试 / 验证
git add <具体文件>   # 避免 git add -A 夹带无关文件
git commit -m "..."
git push
```

**Commit message 规范**（Conventional Commits）：

```text
feat: add voice command pipeline
feat: add navigation skill
fix: recover RTSP stream independently
perf: optimize TensorRT inference scheduling
docs: update robot architecture
refactor: extract pid controller from chassis node
test: add lidar scan validation
chore: update gitignore for TensorRT engines
```

要求：一次 commit 只做一件事；message 说明**做了什么**，必要时在 body 说明**为什么**。

---

## 6. 禁止事项

- ❌ `git push --force`（**任何情况下都不允许**）
- ❌ `git reset --hard`
- ❌ 擅自删除用户已有代码
- ❌ 提交 API Key / Token / 密码 / `.env`
- ❌ 提交大模型权重、TensorRT engine（`*.engine` / `*.plan`）、大型数据集、视频、ROS bag、运行日志
- ❌ 为了通过测试而绕过真正的问题
- ❌ 让 LLM 直接发布底盘控制命令

### 大文件处理

模型权重 / engine **不要**直接 `git add`。

如确需版本管理，**先向用户说明并讨论 Git LFS**，不要擅自提交大文件。

当前实机上的大目录（**均不在本仓库内，不要复制进来**）：
- `~/large_models`（282M）
- `~/third_party`（6.4G）
- `~/ros2_ws`（592M，含 `build/` `install/` `log/`）

---

## 7. 工作区已有他人修改时

如果工作区本来就存在**不是你产生的修改**：

1. **不要覆盖或删除**
2. 先识别修改来源（`git status` / `git diff` / `git stash list` / `git log`）
3. 向用户确认后再继续
4. 必要时用 `git stash` 暂存，而不是丢弃

---

## 8. 硬件与安全约束（本项目特有）

- 厂商 `~/ros2_ws` 与 `~/third_party` 只读使用，作为系统 SDK
- **任何运动测试必须保留急停、限速和人工看护**
- 根分区当前 **96% 已用（仅剩 3.0G）**，部署大文件前必须先确认可安全清理或扩容
- 只有 `docs/PROJECT_STATUS.md` 接口清单中标记为"通过"的能力，才允许封装为 Skill
- `Safety > Control > Skill > Agent` —— Safety Runtime 具有最终否决权

---

## 9. 每次开发任务结束时（强制输出）

```
## 本次完成内容
<做了什么>

## 修改文件
<file list>

## 测试结果
<命令 + 结果；未测试需说明原因>

## Commit
<hash + message>

## GitHub Push
<成功 / 失败 + 原因>

## 下一步建议
<建议的下一步>
```

---

## 10. 网络环境注意事项（本机实测）

| 项目 | 状态 |
|---|---|
| GitHub SSH 22 端口 | ❌ 被网络封锁（connection timed out） |
| GitHub SSH 443 端口 | ✅ 可达，已在 `~/.ssh/config` 配置 `HostName ssh.github.com` / `Port 443` |
| GitHub HTTPS | ✅ 可达 |
| SSH 公钥注册 | ⚠️ `~/.ssh/id_ed25519` 需注册到 GitHub 账号后才能 push |

`git push` 失败时优先检查此项，不要尝试 force push。
