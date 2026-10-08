# embodied_voice_wakeup —— 本项目自己的语音唤醒

> Phase 5 / Driver 层。**不碰厂商 `~/ros2_ws`，不用厂商唤醒节点。**

## 一句话

用**已验证可用的录音通路** + **本地 KWS 模型**，在 Jetson 上自己做唤醒：
`环形麦(48k) → 16k → KWS → /embodied/voice/wakeup`。

## 为什么不用厂商那条

厂商环形麦的**硬件唤醒上报不可用**（2026-10-08 实测，见 `docs/DEV_NOTES.md` 坑 33 / `#25`）：

| 测试 | 结果 |
|---|---|
| `awake_node` 跑着时说话 | 零事件（它只认 `a5 01 04` + JSON，而模块说的是 `aa55` 那套） |
| 独占串口后说话（最早一次） | 收到 2 帧 `aa550300fb`，之后 3.5 分钟归零 |
| 独占串口 + 唤醒词 ×5 | **0 字节 / 210 秒** |
| 握手包 × 4 种 RTS/DTR 组合 | **0 字节** |
| **断电重启后再试** | **0 字节** |

**录音/播放一直是好的**（D-022 验过），坏的只是"模块把自己的判断报出来"这一条。
所以唤醒改在 Jetson 上做 —— 模型、阈值、关键词都在我们手里，**可回归、可量召回率**。

顺带一个好处：这条路**绕开了 `voice_control_move`**，那个节点挂在**无限幅**的
`/controller/cmd_vel` 上（#26），是运动测试前要显式停掉的东西。

## 模型从哪来

⚠️ **模型是大文件，不进 Git**（CLAUDE.md §6）。放在 `~/large_models/kws/` 下。

默认模型：`sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01`（int8 全套 **~5 MB**）。
这台机器上 **GitHub 不通**（`github.com` 连不上），从 **ModelScope** 取：

```bash
F=sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01
D=$HOME/large_models/kws/$F && mkdir -p "$D" && cd "$D"
for f in encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx \
         decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx \
         joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx \
         tokens.txt keywords.txt ; do
  curl -sLO "https://www.modelscope.cn/models/pkufool/$F/resolve/master/$f"
done
```

## 怎么跑

```bash
# 必须先 source 厂商，再 source 本 Overlay
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/JetsonRobot/embodied_agent_ws/install/setup.bash

ros2 launch embodied_voice_wakeup voice_wakeup.launch.py \
    model_dir:=$HOME/large_models/kws/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01
```

话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/embodied/voice/wakeup` | `std_msgs/String` | **只在检出时发**，内容 = 关键词显示名 |
| `/embodied/voice/diag` | `Float64MultiArray` | `[采集帧数, 已处理音频秒, 累计检出, 距上次检出秒, 阈值]` |

## ⚠️ 召回率：**实测约 48%**（这是这个包当前最大的问题）

2026-10-08 实测（录音 90 s，用本机的 ASR 模型转录出**真值**，不靠人报数）：

```
真值：唤醒词连续说了 27 遍（22–52 s），之后是 3 句不相干的话
KWS 检出 13 遍            ⇒ 召回 13/27 = 48%，误报 0
```

**这不是调参能解决的** —— 在**同一段录音上离线扫过**：

| 扫的量 | 范围 | 最好 |
|---|---|---|
| `keywords_threshold` | 0.05 / 0.25 / 0.90 | 0.05–0.25 **完全无差别**；0.90 全灭 |
| `keywords_score` | 1 → 12 | **3 最好（48%）**，再高反而变差（8 → 7%，12 → 4%） |
| `max_active_paths` | 4 / 8 / 16 | 都是 46~48% |
| 关键词加变体 | 「小幻小幻」+「小坏小坏」 | 26% → **48%**（ASR 把你的发音听成「小坏」） |

**而同一台机器上的 ASR 模型把这 27 遍基本全听对了**（转录里是一长串「小坏小坏」没有断）。
⇒ **问题在 KWS 模型与这个人的发音之间，不在硬件、不在参数。**

**这意味着：沉默不能当作"用户没喊"的证据。** 本节点报的是"检出"，不是"没喊"。
下游如果用唤醒做门控，必须容忍漏检（例如允许超时后仍然接受命令，而不是把漏检当拒绝）。

## 可能的出路（还没做）

1. **换一个模型本身就训练过的词**（如「小爱同学」「你好问问」）—— 成本低，一次 90 s 录音就能验；
2. **改用 ASR 做唤醒**（"在识别文本里找唤醒词"）—— 本机 xlarge 模型实测 **RTF 0.55**
   （2 线程、约 2 倍实时），召回接近 100%，但对这块 **8GB / 已被厂商栈占掉 5.5G** 的 Orin 是实实在在的负担（`#7`）；
3. 找一个**更小的中文流式 ASR**（本机没有；ModelScope 上按已知命名没找到）。

## 测试

```bash
cd ~/JetsonRobot/embodied_agent_ws/src/embodied_voice_wakeup
python3 -m pytest test/ -q      # 11 项，不加载模型、不开麦克风
```
