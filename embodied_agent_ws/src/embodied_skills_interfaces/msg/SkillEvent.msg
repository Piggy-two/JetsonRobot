# 任务级事件 —— D-004 的「只有任务级事件才唤醒 Agent」的载体（D-030）。
#
# 为什么用 .msg 而不是沿用例行的 `std_msgs/String` + 语法约定：
#   `/embodied/motor/events` 与 `/embodied/safety/events` 是**给人看日志**的；
#   本话题是**给代码消费**的结构化事件（Agent 的 Event Manager 要按 task_id 匹配、
#   按 state 判定是否唤醒）。让下游去 split 字符串做逻辑判断，正是本项目一直避免的事。
#
# 🔒 **发布者只有 Skill Gateway 一个。** 技能自己不得直接发本话题 ——
#    这样可以保证状态序列不会因技能实现不同而错乱。

string task_id        # 与 SkillInvoke 返回的 task_id 对应
string skill          # 注册表里的技能名

# 状态名。**两个词汇表，别混用：**
#
# A) task-tier 技能（Autonomous / Semantic）用 plan.md §21 的 8 个状态：
#      STARTED / RUNNING / ARRIVED / TARGET_FOUND / TARGET_LOST / BLOCKED / FAILED / CANCELLED
#    其中后 6 个是**终态**，也是**唯一会唤醒 Agent 的状态**（D-004）。
#    STARTED / RUNNING **不唤醒** —— 唤醒靠轮询会浪费算力，靠每个中间态会淹没 Agent。
#
# B) control-tier 技能（Control Skill）**不进那 8 个状态**，用：
#      RUNNING / FINISHED / FAILED / CANCELLED
#    ⚠️ `FINISHED` **不是**上面 8 个任务状态之一，它的含义只有
#       「**技能返回了**」，**绝不**表示"到位了 / 达成目标了"。
#       理由（D-032）：Control Skill 的 success 含义是"速度按时长发完了"（D-026），
#       8 个状态里没有任何一个能诚实表达它 —— 把 success=true 映射成 ARRIVED 就是撒谎，
#       而且会让 Agent 的后续规划建立在"以为到位了"的假前提上。

string state

string detail         # 人类可读补充说明（中文）。**下游不得解析它做逻辑判断。**

float64 progress      # 0~1 的进度；**-1 = 该技能无法给出有意义的进度**。
                      #   开环技能**必须**报 -1，不得报一个看起来很合理的假进度（与 D-026 同一条原则）。

bool verified         # false = 这个状态是**技能自报**的，未经独立反馈确认。
                      #   当前**所有**技能的终态都是 verified=false（没有任何独立里程/视觉反馈）。

builtin_interfaces/Time stamp
