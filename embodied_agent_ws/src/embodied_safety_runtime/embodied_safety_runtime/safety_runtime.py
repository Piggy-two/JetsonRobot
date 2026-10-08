#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetsonRobot Overlay Safety Runtime。

项目的核心主张是 **`Safety > Control > Skill > Agent`，Safety 具有最终否决权**。
本节点是这条主张的**第一个落点**：把"停"这件事从各个下游节点手里收上来，
变成一处**本地、不经过 LLM、不依赖任何人配合**的动作。

本版实现四件事：

**一、本地安全指令通路（D-006 / D-024 决策 3）**
订阅厂商 ASR 的文本话题，**在本节点内本地匹配**安全词（`estop.py`，纯逻辑、可单测）。
识别侧复用厂商的**离线**链路（不联网），执行侧完全由本项目自己做。
⚠️ **一个字都不经过 LLM，也不经过厂商的 `voice_control_*` 节点。**

**二、独立的零速通道（`zero_channel_topic`，默认 `/cmd_vel`）**
锁存期间，本节点**自己**以固定频率直接向底盘发 `Twist()`（全零）。

    为什么不"命令 Motor Driver 停"就够了 —— D-020 实测：底盘**没有指令超时保护**，
    停止发布 ≠ 停车，它会一直保持最后一条速度。而"持续发零"这件事现在**只在
    Motor Driver 里**（D-025）。**如果 Motor Driver 自己挂了，就没人发零了。**
    所以急停通道必须**独立**：不依赖 Motor Driver 存活、不依赖 Control Skill 存活。
    另两个 `~/stop` 服务照发（best-effort），但**急停不靠它们成立**。

    🔒 **本节点唯一的发布语句就是 `publish(Twist())`** —— 它在结构上
    **不可能**发出任何非零速度。这不是"约定"，是代码里只有这一个出口。

**三、对 Motor Driver 的停更看门狗（`motor_status_topic` / `motor_watchdog`）**
第二件事只解决了"**已经**知道要停"的情况。还有一种是"**根本没人知道要停**"：
Motor Driver 进程自己死了 —— 它既不报错、也没人转告。这时底盘会保持最后速度一直跑。

所以本节点监视 Motor Driver 的状态话题：**它停更超过 `motor_watchdog` 秒即自动急停**，
由本节点接管发零。这就是"独立通道"存在的意义真正兑现的时刻。

⚠️ **首次见到之前永不判失联**（`watchdog.py`）—— 否则每次启动都会先来一次假警报。

**四、避障守卫（`obstacle_guard.py`，默认开启）**

D-027 曾把"避障"列为本版不做；这一块补的就是它。判定逻辑全在纯 Python 的
`obstacle_guard.py` 里（可离线单测），本节点只做三件接线：

    ① **读 Motor Driver 状态里那三个速度**（`/embodied/motor/status` 的 vx/vy）
       —— 那就是"此刻被命令的运动方向与快慢"，因此**方向判定用的是真实指令**，
          而不是"哪里是前方"。侧移、后退都按各自方向判。

    ② **问 LiDAR 原语**（`~/sector_min_range`，D-028 的已验收接口，复用它的
          跨 0/2π 接缝几何）—— 只问"那个方向、那个距离内有没有东西"。

    ③ **判数据新不新鲜**。这里有个坑，值得单独写下来：

       ⚠️ `sector_min_range` 在「**扫描陈旧**」与「**扇区真的空**」两种情况下
          返回的值**一模一样**（都是 `valid=false, range=-1`，见 `lidar_driver.py`）。
          直接拿它判"前面没东西"，就会把"我瞎了"读成"路是通的" ——
          正好是 D-028 那条「**不知道 ≠ 安全**」要防的错误。

       所以新鲜度**不由被监视的答案自己声明**，而是另取一路心跳：
       订阅 `/embodied/lidar/front`（LiDAR 原语每收到一帧扫描发一次），**只看到达时刻**。
       心跳停了即判"不知道"，此时若正被命令运动 → 停。

阈值不用固定距离，而是 `速度 × 预留时间`（TTC 式）—— 理由见 `obstacle_guard.py`
（D-028 实测近场回波占比在两次摆位间 3.6% → 67%，**任何固定距离都会被场地否决**）。

⚠️ **触发后同样是锁存**：与语音急停、看门狗失联共用同一个 `EStopLatch`，
必须显式调 `~/release`。

⚠️ **本版仍不做**：限速、速度/区域限制、**绕行**（绕行属 Phase 3 的局部规划器）、
   原地转身时的机体扫掠判断。

⚠️ 触发后的急停是**锁存**的：必须显式调 `~/release` 才能解除（`estop.py` 的 `EStopLatch`）。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger

from embodied_skills_interfaces.srv import SectorMinRange

from embodied_safety_runtime.estop import (
    DEFAULT_SAFETY_PHRASES, EStopLatch, is_safety_command)
from embodied_safety_runtime.obstacle_guard import (
    GuardConfig, GuardInput, evaluate, reply_covers_question)
from embodied_safety_runtime.watchdog import StalenessWatchdog


class SafetyRuntime(Node):
    def __init__(self):
        super().__init__('safety_runtime')

        self.declare_parameter('voice_topic', '/asr_node/voice_words')
        # ⚠️ 独立零速通道。**只发零**，见文件头说明。
        self.declare_parameter('zero_channel_topic', '/cmd_vel')
        self.declare_parameter('zero_rate', 10.0)
        self.declare_parameter('safety_phrases', DEFAULT_SAFETY_PHRASES)
        # 对 Motor Driver 的停更看门狗：**它挂了就没人发零了**，必须自动接管
        self.declare_parameter('watch_motor', True)
        self.declare_parameter('motor_status_topic', '/embodied/motor/status')
        self.declare_parameter('motor_watchdog', 2.0)
        # best-effort：把下游的锁存也打开（急停不依赖它们）
        self.declare_parameter('motor_stop_service', '/motor_driver/stop')
        self.declare_parameter('control_stop_service', '/control_skills/stop')
        self.declare_parameter('status_topic', '/embodied/safety/status')
        self.declare_parameter('event_topic', '/embodied/safety/events')

        # ---- 避障守卫（obstacle_guard.py）----
        self.declare_parameter('enable_obstacle_guard', True)
        # 阈值 = 速度 × lookahead（TTC 式）。⚠️ lookahead / min_range / width
        # 是**待实测标定的临时值**，标定方法见 obstacle_guard.py 与 DEV_NOTES。
        self.declare_parameter('obstacle_lookahead', 1.5)
        self.declare_parameter('obstacle_min_range', 0.20)
        self.declare_parameter('obstacle_max_range', 1.00)
        self.declare_parameter('obstacle_width', 1.0471975511965976)
        self.declare_parameter('obstacle_min_speed', 0.02)
        # 新鲜度心跳（LiDAR 原语每帧扫描发一次）+ 心跳超时
        self.declare_parameter('lidar_front_topic', '/embodied/lidar/front')
        self.declare_parameter('obstacle_scan_timeout', 0.5)
        self.declare_parameter('sector_service', '/lidar_driver/sector_min_range')
        # 回答本身的新鲜度：服务没答上来（节点不在/卡住）也算"不知道"
        self.declare_parameter('obstacle_reply_timeout', 0.5)

        g = lambda n: self.get_parameter(n).value          # noqa: E731

        self.phrases = list(g('safety_phrases'))
        self.latch = EStopLatch()
        self._zero_frames = 0
        self._triggers = 0

        # ---- 避障守卫的状态 ----
        # 速度来源是 Motor Driver 状态里那三个数（**它自己的输出**，锁存/超时时就是 0）
        self._speed_known = False
        self._vx = 0.0
        self._vy = 0.0
        # 雷达心跳 / 服务回答
        self._heartbeat_time = None
        self._reply = None                 # (valid, range)
        self._reply_time = None
        #: 手头这份回答**是问哪个方向 / 什么范围**得到的 —— 回答必须和问题配对，
        #: 否则会把别的方向的距离当成这个方向的（见 `reply_covers_question`）
        self._reply_center = None
        self._reply_max_range = None
        self._asked_center = None
        self._asked_max_range = None
        self._sector_fut = None
        self._asking_since = None          # 开始问却还没拿到回答的起点（有界宽限）
        self._guard = None                 # 最近一次 GuardDecision
        self._scan_timeout = float(g('obstacle_scan_timeout'))
        self._reply_timeout = float(g('obstacle_reply_timeout'))

        # ⚠️ 守卫要判"往哪个方向走"，而方向只能从 Motor Driver 状态里拿。
        #    关掉 watch_motor 就等于把方向来源断了 —— 那**不能**静默退化成"不避障"，
        #    必须显式禁用并报错（静默失效正是这个项目一直在防的东西）。
        self._guard_enabled = bool(g('enable_obstacle_guard')) and bool(g('watch_motor'))
        self._guard_cfg = GuardConfig(
            enabled=self._guard_enabled,
            lookahead=float(g('obstacle_lookahead')),
            min_range=float(g('obstacle_min_range')),
            max_range=float(g('obstacle_max_range')),
            width=float(g('obstacle_width')),
            min_speed=float(g('obstacle_min_speed')))

        # 🔒 唯一的发布者，且只发 Twist()
        self.zero_pub = self.create_publisher(Twist, g('zero_channel_topic'), 10)
        self.status_pub = self.create_publisher(Float64MultiArray, g('status_topic'), 10)
        self.event_pub = self.create_publisher(String, g('event_topic'), 10)

        self.create_subscription(String, g('voice_topic'), self.on_voice, 10)
        if g('watch_motor'):
            self.motor_wd = StalenessWatchdog(float(g('motor_watchdog')))
            self.create_subscription(Float64MultiArray, g('motor_status_topic'),
                                     self.on_motor_status, 10)
        else:
            self.motor_wd = None
            self.get_logger().warn('watch_motor=false：**不会**在 Motor Driver 挂掉时自动接管')

        self.create_service(Trigger, '~/estop', self.on_estop)
        self.create_service(Trigger, '~/release', self.on_release)

        # 避障：心跳只看到达时刻（不读内容），方向性答案来自 LiDAR 原语的服务
        if self._guard_enabled:
            self.create_subscription(
                Float64MultiArray, g('lidar_front_topic'), self.on_lidar_front, 10)
            self._sector_cli = self.create_client(SectorMinRange, g('sector_service'))
        else:
            self._sector_cli = None

        self._motor_cli = self.create_client(Trigger, g('motor_stop_service'))
        self._ctrl_cli = self.create_client(Trigger, g('control_stop_service'))
        self._pending = []

        rate = float(g('zero_rate'))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Safety Runtime 启动 | 语音安全词 <- {g("voice_topic")} | '
            f'独立零速通道 -> {g("zero_channel_topic")} @ {rate:.0f} Hz（**只发零**）| '
            f'安全词 {len(self.phrases)} 条')
        if g('watch_motor'):
            self.get_logger().info(
                f'看门狗：监视 {g("motor_status_topic")}，停更 > {g("motor_watchdog")}s '
                '即自动接管发零')
        if self._guard_enabled:
            self.get_logger().info(
                f'避障守卫：方向取自 {g("motor_status_topic")} 的速度，'
                f'问 {g("sector_service")}，心跳看 {g("lidar_front_topic")} | '
                f'阈值 = 速度 × {g("obstacle_lookahead")}s '
                f'(下限 {g("obstacle_min_range")} / 上限 {g("obstacle_max_range")} m)，'
                f'扇区 ±{float(g("obstacle_width")) / 2 * 57.29577951308232:.0f}°')
            self.get_logger().warn(
                '⚠️ 避障阈值里的 lookahead / min_range / width 是**临时值，尚未实测标定** '
                '（D-028 明确要求按部署环境实测）。')
        elif g('enable_obstacle_guard'):
            self.get_logger().error(
                'watch_motor=false ⇒ 拿不到"被命令的运动方向" ⇒ **避障守卫已被禁用**。'
                '这不是"没有避障需求"，是它无法工作 —— 要避障就必须开着 watch_motor。')
        else:
            self.get_logger().warn('enable_obstacle_guard=false：**本节点不做避障判定**')
        self.get_logger().warn(
            '⚠️ 本版仍**不含**限速 / 区域限制 / 绕行；急停是锁存的，'
            '必须显式调 ~/release 才能解除。')

    # ---------- 本地安全指令（不经 LLM） ----------

    def on_voice(self, msg):
        hit, phrase = is_safety_command(msg.data, self.phrases)
        if hit:
            self.get_logger().error(
                f'★ 本地识别到安全指令：{msg.data!r}（命中「{phrase}」）—— 立即急停')
            self._trigger(f'voice:{phrase}')
        else:
            self.get_logger().debug(f'语音文本（非安全词）：{msg.data!r}')

    def on_motor_status(self, msg):
        """看门狗只要"它还在说话"；避障还要它说的**速度**。

        取 `data[1]` / `data[2]` 是 Motor Driver 的**输出**速度（见
        `embodied_motor_driver/motor_driver.py` 的 status 布局）—— 它锁存/超时时
        自己就是 0，所以这里不需要另判状态码：**"输出是零"本身就等于"没有被命令运动"**。
        """
        self.motor_wd.on_signal(time.monotonic())
        if len(msg.data) >= 3:
            self._vx = float(msg.data[1])
            self._vy = float(msg.data[2])
            self._speed_known = True

    def on_lidar_front(self, _msg):
        """**心跳**：只记到达时刻，内容一律不看（内容由 `sector_service` 提供）。

        为什么要单独一路心跳：`sector_min_range` 把"扫描陈旧"和"扇区真的空"
        返回成**同一个值**（`valid=false, range=-1`）。新鲜度不能由被监视的答案
        自己声明 —— 那样"传感器挂了"会被读成"前面没东西"（D-028 的「不知道 ≠ 安全」）。
        """
        self._heartbeat_time = time.monotonic()

    # ---------- 避障守卫 ----------

    def _update_obstacle(self, now):
        """收上一次的回答、必要时发新的一次，然后算出这一刻的 GuardDecision。"""
        if not self._guard_enabled:
            return

        # ① 收上一次的异步回答
        if self._sector_fut is not None and self._sector_fut.done():
            try:
                res = self._sector_fut.result()
                self._reply = (bool(res.valid), float(res.range))
                self._reply_time = now
                # 连同"这个问题是什么"一起存 —— 回答和问题分家就会出事
                self._reply_center = self._asked_center
                self._reply_max_range = self._asked_max_range
                self._asking_since = None          # 拿到了，宽限期结束（见下）
            except Exception as exc:                      # noqa: BLE001
                self._reply = None
                self.get_logger().warn(f'sector_min_range 调用失败：{exc}')
            self._sector_fut = None

        # ② 阈值要用**当前**速度算，所以每次判定前先算一遍
        speed = (self._vx ** 2 + self._vy ** 2) ** 0.5 if self._speed_known else 0.0
        stop_range = min(self._guard_cfg.max_range,
                         max(self._guard_cfg.min_range,
                             speed * self._guard_cfg.lookahead))

        # ②′ **手头这份回答，如果不是"现在这个问题"的答案，就丢掉**
        #
        # ⚠️ 早先没有这一步，于是方向一变（例如从"前进"改成"左移"）那一刻，
        #    守卫会拿**上一个方向**的距离去和**这个方向**的阈值比 —— 真实案例：
        #    前方 0.15 m 有瓶子、左方 0.83 m 空旷，改成左移后守卫读到
        #    "左方 0.15 m" 并**假锁存**（`DEV_NOTES` 坑 28 / **D-039**）。
        #    反过来的情形更要命：旧方向远、新方向有障碍 ⇒ **漏判一拍**。
        #    丢掉的这一份不会被当成"没有回答"而下重手 —— 下面 ③ 会按当前方向
        #    立刻重问，④ 走的是既有的**有界宽限**（坑 25 那套），所以既不假锁存、
        #    也不会因为宽限而长期不判定。
        if self._reply is not None:
            bearing_now = math.atan2(self._vy, self._vx)
            if not reply_covers_question(
                    self._reply_center, self._reply_max_range, self._reply[0],
                    bearing_now, stop_range):
                self._reply = None
                self._reply_time = None
                self._reply_center = None
                self._reply_max_range = None

        # ③ 发新请求（一次只留一个在飞，避免 10 Hz 叠请求把原语压垮）
        #
        # ⚠️ **静止时也照问**（方向按前方、距离按下限）。看着像多余，其实是必需的：
        #    判定要求"回答新鲜"，而回答只能"问"出来。如果只在动起来之后才问，
        #    那么**每段运动的头一拍**都没有回答 → 被判成"不知道" → 一给指令就锁存。
        #    一直问，就能保证动起来那一刻手上已经有一份≤一拍的答案。
        if self._sector_fut is None and self._speed_known:
            if self._sector_cli.service_is_ready():
                req = SectorMinRange.Request()
                req.center = math.atan2(self._vy, self._vx)
                req.width = self._guard_cfg.width
                req.max_range = stop_range
                # 记下"问的是什么"，回答回来时一起存（②′ 要用它判断配对）
                self._asked_center = float(req.center)
                self._asked_max_range = float(req.max_range)
                self._sector_fut = self._sector_cli.call_async(req)
                if self._asking_since is None:
                    # 记下"从什么时候开始问却还没拿到回答" —— 这是上面那条**有界宽限**
                    # 的计时起点。没有它，每段运动的头一拍都会被判「不知道」而锁存。
                    self._asking_since = now
            # 服务不在 —— 不发请求，下面的"回答陈旧"会把它变成"不知道"

        # ④ 组装判定输入
        heartbeat_fresh = (self._heartbeat_time is not None
                           and now - self._heartbeat_time <= self._scan_timeout)
        reply_fresh = (self._reply is not None
                       and self._reply_time is not None
                       and now - self._reply_time <= self._reply_timeout)
        valid, rng = self._reply if self._reply is not None else (False, -1.0)

        self._guard = evaluate(self._guard_cfg, GuardInput(
            speed_known=self._speed_known, vx=self._vx, vy=self._vy,
            # ⚠️ 两个新鲜度都要：心跳答"雷达还在说话吗"，回答答"原语还答得上来吗"
            scan_fresh=heartbeat_fresh and reply_fresh,
            scan_valid=valid, scan_range=rng,
            # 「才刚开始问、还没等到第一份回答」—— 有界宽限，理由见 obstacle_guard.py
            scan_pending=(self._reply is None and self._asking_since is not None
                          and now - self._asking_since <= self._reply_timeout)))

    def _trigger(self, reason):
        changed = self.latch.trigger(reason)
        self._triggers += 1
        # 立刻发一帧零，不等定时器那一拍
        self._publish_zero()
        if changed:
            self._emit(f'estop_triggered:{reason}')
            self.get_logger().error(f'急停已锁存（{reason}）—— 需显式 ~/release 才能解除')
        # best-effort：把下游的锁存也打开
        self._call(self._motor_cli, 'motor_driver/~/stop')
        self._call(self._ctrl_cli, 'control_skills/~/stop')

    def _call(self, cli, what):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{what} 不可用（不影响急停：零速通道是独立的）')
            return
        fut = cli.call_async(Trigger.Request())
        self._pending.append((what, fut))
        fut.add_done_callback(lambda f, w=what: self.get_logger().info(f'{w} 已调用'))

    def on_estop(self, _req, res):
        """手动触发（也给验收/测试用）。"""
        self._trigger('service')
        res.success = True
        res.message = f'急停已触发（原因：{self.latch.reason}）'
        return res

    def on_release(self, _req, res):
        # ⚠️ Motor Driver 还没活过来的话，**不允许解除** ——
        #    解除就等于本节点停止发零，而那时没有别人在发零，底盘会保持最后速度跑下去。
        #    与其"解除了又立刻重新锁存"（看起来像 bug），不如明确拒绝并说清原因。
        if self.motor_wd is not None:
            now = time.monotonic()
            if self.motor_wd.expired(now):
                age = self.motor_wd.age(now)
                res.success = False
                res.message = (f'拒绝解除：Motor Driver 状态仍停更 {age:.1f}s —— '
                               '解除会让底盘无人发零（D-020）。先让它恢复。')
                self.get_logger().error(res.message)
                return res

        changed = self.latch.release()
        if changed:
            self._emit('estop_released')
            self.get_logger().warn('急停已**显式解除**（注意：Motor Driver 若也被锁存过，'
                                   '它需要各自的 ~/resume）')
        res.success = True
        res.message = '已解除' if changed else '本来就是解除的'
        return res

    # ---------- 独立零速通道 ----------

    def tick(self):
        now = time.monotonic()

        # 避障判定要在看门狗之前更新：它决定了这一拍要不要停。
        self._update_obstacle(now)

        # 先看门狗再发零：Motor Driver 挂了就自动接管，别等"有人告诉我"。
        if (self.motor_wd is not None and not self.latch.latched
                and self.motor_wd.expired(now)):
            age = self.motor_wd.age(now)
            self.get_logger().error(
                f'★ Motor Driver 状态已停更 {age:.1f}s —— 判定它已退出，**自动接管发零**。'
                '（底盘没有指令超时保护（D-020），没人发零它会保持最后速度一直跑。）')
            self._trigger('motor_driver_lost')
        elif (not self.latch.latched and self._guard is not None and self._guard.stop):
            self.get_logger().error(
                f'★ 避障守卫触发：{self._guard.reason_string} | '
                f'被命令速度 {self._guard.speed:.2f} m/s @ '
                f'{math.degrees(self._guard.bearing):+.0f}°，阈值 {self._guard.stop_range:.2f} m'
                + ('（雷达/原语**不新鲜** —— 不知道 ≠ 安全，宁可停）'
                   if self._guard.range < 0.0 else f'，扇区内最近回波 {self._guard.range:.2f} m'))
            self._trigger(self._guard.reason_string)

        if self.latch.latched:
            self._publish_zero()
        self._publish_status(now)

    def _publish_zero(self):
        self.zero_pub.publish(Twist())      # 🔒 全节点唯一的发布语句，只发零
        self._zero_frames += 1

    def _publish_status(self, now):
        age = None if self.motor_wd is None else self.motor_wd.age(now)
        g = self._guard
        m = Float64MultiArray()
        m.data = [1.0 if self.latch.latched else 0.0,
                  float(self._zero_frames), float(self._triggers),
                  -1.0 if age is None else age * 1000.0,
                  # ---- 以下是 2026-10-07 追加的避障字段（前四个位置不变）----
                  1.0 if self._guard_enabled else 0.0,
                  1.0 if (g is not None and g.stop) else 0.0,
                  -1.0 if g is None else float(g.stop_range),
                  -1.0 if g is None else float(g.range),
                  -1.0 if g is None else float(g.bearing)]
        self.status_pub.publish(m)

    def _emit(self, name):
        e = String()
        e.data = name
        self.event_pub.publish(e)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补一发零：这是"安全节点"该有的姿态（虽然它只管锁存期间的车）。
        try:
            node.zero_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
