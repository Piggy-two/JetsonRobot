#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mic_serial_probe.py —— 探测讯飞环形麦的**控制串口**（`/dev/ring_mic`）是否还活着。

    ⚠️ 这是 Phase 0/5 的【诊断工装】，不是运行时组件。
       不得被 Agent Runtime、任何 Skill 或 Safety Runtime 调用。

为什么需要它
------------
2026-10-07 定位「语音唤醒不通」（#25）时发现：厂商 `awake_node.py` 的诊断能力**接近于零** ——

  · 它的 `CircleMic.send()` 是 `while True:` 等对端应答，**没有超时**。对端不回，
    它就永远卡在那里；而外面看到的是**进程正常存活、日志一个字没有**（状态 `do_select`）。
  · 它的 `enable_setting` 分支在 journal 里**什么都不打印**，除非真的走到了那两行 `print`。

于是"硬件到底会不会说话"这件事，在厂商代码里**根本读不出来**。本工装把它变成一个明确读数。

两种模式
--------
默认（握手探测）：
    发一次厂商的握手包，试全部 4 种 (RTS, DTR) 组合 —— CH340 的 DTR/RTS 有时是设备的复位/使能线。

--listen N（纯监听）：
    在 N 秒内只听着，每 2 秒补发一次握手，把**收到的每一个字节**带时间戳打出来。
    用于配合人声/合成音频，看设备会不会主动上报唤醒事件。

怎么读结果（这是本工装最要紧的部分）
------------------------------------
  · **收到 0 字节 ≠ 波特率不对。** 波特率错会收到**乱码**（跳变照样被采到，只是解错），
    不会收到"什么都没有"。**0 字节说明这条线收方向上没有电平活动** ——
    是"对端没在发 / 没接上"，不是"我解读错了"。（见 DEV_NOTES 坑 15）
  · CTS/DSR/CD/RI 全 False 是**弱证据**：很多 CH340 根本不驱动这几根线，别据此下结论。

用法
----
    # 握手探测（四种 RTS/DTR 组合，各听 1.5 s）
    python3 tools/mic_serial_probe.py

    # 纯监听 120 秒 —— 这期间说唤醒词，看有没有任何反应
    python3 tools/mic_serial_probe.py --listen 120

⚠️ 前提：**测之前请先停掉 `awake_node`** ——
   `sudo systemctl stop start_app_node.service`，测完 `sudo systemctl start start_app_node.service`。

   ⚠️ 注意「停掉」不是为了"打开端口"：实测 **pyserial 默认不加排他锁，两个进程能同时打开
   同一个 tty**（本工装对着正在跑的 `awake_node` 也能开成功）。要停是为了**读数干净** ——
   两个读者并存时，设备发来的字节会被**随机分给其中一个**，谁都不完整。
   这条同样适用于判读：**"能打开"从来不代表"没人在用"**。

退出码：0 = 测完（不代表连通）；1 = 端口打不开。
"""

import argparse
import sys
import time

import serial

HANDSHAKE = bytes([0xA5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00,
                   0xA5, 0x00, 0x00, 0x00, 0xB0])

# ---------- 第二套协议：`aa 55 ...`（2026-10-07 实测发现）----------
#
# ⚠️ 这块麦克风的控制串口**并不说 `a5 01 04` 那套协议**。对着它说话，
#    它吐的是 **5 字节的 `aa 55` 帧** —— 而 `awake_node.py`（`MIC_TYPE=xf` 选中的那个）
#    只认 `a5 01 04` + 一段 JSON，所以**把这些帧全部当成噪声丢掉了**。
#
# 帧表与命令表**抄自厂商源码** `xf_mic_asr_offline/scripts/wonder_echo_pro_node.py:52-102`
# （那个节点是 `mic_init.launch.py` 在 `MIC_TYPE != 'xf'` 时才会启动的分支）。
# 这里只抄一张小表用于**解码**，不改厂商任何东西。
_WONDER_FRAMES = {
    'aa550300fb': '唤醒成功(wake-up-success)',
    'aa550200fb': '休眠(Sleep)',
}

# 命令帧模板 `aa550001fb`，第 7~8 个字符（1-based）换成 1 起的命令序号。
_WONDER_CMDS_ZH = [
    '拔个萝卜', '拿给我', '开启颜色识别', '关闭颜色识别', '开启颜色分拣', '关闭颜色分拣',
    '追踪红色', '追踪绿色', '追踪蓝色', '停止追踪', '夹取红色', '夹取绿色', '夹取蓝色',
    '夹取球体', '夹取圆柱体', '夹取立方体', '关闭夹取', '开启垃圾分类', '关闭垃圾分类',
    '前进', '后退', '左转', '右转', '停下', '漂移', '过来',
    "去'A'点", "去'B'点", "去'C'点", '回原点', '导航搬运',
]
_WONDER_CMDS_EN = [
    'pick a carrot', 'pass me please', 'start color recognition', 'stop color recognition',
    'start color sorting', 'stop color sorting', 'track red object', 'track green object',
    'track blue object', 'stop tracking', 'gripping red', 'gripping green', 'gripping blue',
    'gripping the sphere', 'gripping the cylinder', 'gripping the cuboid', 'stop gripping',
    'sort waste', 'stop sort waste', 'go forward', 'go backward', 'turn left', 'turn right',
    'stop', 'drift', 'come here', 'go to A point', 'go to B point', 'go to C point',
    'go back to the start', 'navigate and transport',
]


def _build_wonder_cmds():
    """序号 → (中文, 英文)。**两张表按序号对齐**，所以两边都要留着 ——
    只留一边会让人以为模块说的就是那一种语言（实际取决于模块侧的配置）。"""
    out = {}
    for i, (zh, en) in enumerate(zip(_WONDER_CMDS_ZH, _WONDER_CMDS_EN), start=1):
        out['aa5500%02xfb' % i] = (zh, en)
    return out


_WONDER_CMDS = _build_wonder_cmds()


def decode_wonder(frame_hex):
    """把 5 字节 `aa55` 帧解码成一句话。认识就返回含义，不认识就返回 None。"""
    if frame_hex in _WONDER_FRAMES:
        return _WONDER_FRAMES[frame_hex]
    if frame_hex in _WONDER_CMDS:
        zh, en = _WONDER_CMDS[frame_hex]
        return f'命令 #{int(frame_hex[6:8], 16)}：「{zh}」/「{en}」'
    return None


def wonder_frames(buf):
    """从字节流里切出所有 `aa 55` 开头的 5 字节帧，返回 (帧列表, 剩余缓冲)。"""
    frames = []
    while len(buf) >= 5:
        if buf[0] == 0xAA and buf[1] == 0x55:
            frames.append(buf[:5])
            buf = buf[5:]
        else:
            # 不是帧头就丢掉一个字节继续找（流里可能有噪声/半截帧）
            buf = buf[1:]
    return frames, buf


def open_port(port):
    s = serial.Serial(None, 115200, serial.EIGHTBITS, serial.PARITY_NONE,
                      serial.STOPBITS_ONE, timeout=0.05)
    s.setPort(port)
    s.open()
    return s


def handshake_probe(port):
    """四种 (RTS, DTR) 组合各发一次握手，返回是否收到 a5 01 ff。"""
    print(f'端口 {port}')
    print(f'  发送的握手包: {" ".join(f"{b:02x}" for b in HANDSHAKE)}')
    print()
    ok = False
    for rts in (False, True):
        for dtr in (False, True):
            s = open_port(port)
            s.rts = rts
            s.dtr = dtr
            time.sleep(0.15)
            s.reset_input_buffer()
            s.write(HANDSHAKE)
            got = b''
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                chunk = s.read(256)
                if chunk:
                    got += chunk
            s.close()
            head = ' '.join(f'{b:02x}' for b in got[:24])
            if b'\xa5\x01\xff' in got:
                mark, ok = '  ← ★ 收到 a5 01 ff 握手应答！', True
            elif got:
                mark = '  ← 有数据但不是握手应答'
            else:
                mark = '  (无任何回应)'
            print(f'  RTS={str(rts):<5} DTR={str(dtr):<5} 收到 {len(got):3d} 字节'
                  f'{"  [" + head + "]" if got else ""}{mark}')
    print()
    if ok:
        print('✅ 硬件会对握手应答 —— 厂商那条通路是通的，卡住另有原因')
    else:
        print('❌ 四种组合都收不到 a5 01 ff')
        print('   → 控制串口对握手完全无应答；厂商 awake_node 的 enable_setting 通路在本机不可用。')
    return 0


def modem_lines(port):
    s = open_port(port)
    print('调制解调器控制线（⚠️ 弱证据，CH340 常不驱动这几根线）:')
    print(f'  CTS={s.cts}  DSR={s.dsr}  CD={s.cd}  RI={s.ri}')
    print()
    s.close()


def listen(port, duration):
    """纯监听：只问"设备会不会主动吐字节"，与握手是否被应答无关。"""
    s = open_port(port)
    s.reset_input_buffer()
    t0 = time.monotonic()
    last_hs = 0.0
    total = 0
    reads = 0
    print(f'监听 {port} 共 {duration:.0f} 秒，每 2 秒补发一次握手……')
    print('（现在请对麦克风说唤醒词）')
    print()
    while time.monotonic() - t0 < duration:
        now = time.monotonic()
        if now - last_hs >= 2.0:
            s.write(HANDSHAKE)
            last_hs = now
        chunk = s.read(512)
        if chunk:
            total += len(chunk)
            reads += 1
            hexs = ' '.join(f'{b:02x}' for b in chunk[:32])
            print(f'  [+{now - t0:6.2f}s] 收到 {len(chunk):3d} 字节: {hexs}', flush=True)
    s.close()
    print()
    print(f'总计收到 {total} 字节，{reads} 次读数。')
    if total == 0:
        print('❌ 整段时间这条串口【完全沉默】—— 设备侧一个字节都没发过来。')
        print('   唤醒事件走的就是这条线 ⇒ 唤醒在结构上不可能产生。')
        print('   ⚠️ 零字节 ≠ 波特率不对（波特率错会收到乱码）。')
    else:
        print('✅ 串口有数据 —— TX 线是活的，可以继续看协议解析。')
    return 0


def decode_stream(port, duration):
    """按 `aa 55` 帧协议解码（2026-10-07 实测发现的那套）。

    这是本工装目前**最有信息量**的模式：它把"串口上到底在说什么"变成一句句话。
    """
    s = open_port(port)
    s.reset_input_buffer()
    t0 = time.monotonic()
    buf = b''
    counts = {}
    unknown = []
    print(f'解码监听 {port} 共 {duration:.0f} 秒（按 aa55 帧协议）……')
    print('（现在请对麦克风说话）')
    print()
    while time.monotonic() - t0 < duration:
        chunk = s.read(256)
        if not chunk:
            continue
        buf += chunk
        frames, buf = wonder_frames(buf)
        for f in frames:
            hexs = f.hex()
            meaning = decode_wonder(hexs)
            counts[hexs] = counts.get(hexs, 0) + 1
            ts = f'[+{time.monotonic() - t0:6.2f}s]'
            if meaning:
                print(f'  {ts} ★ {hexs}  →  {meaning}', flush=True)
            else:
                print(f'  {ts} ? {hexs}  →  （表里没有这个帧）', flush=True)
                unknown.append(hexs)
    s.close()
    print()
    print(f'总帧数 {sum(counts.values())}：')
    for hexs, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        meaning = decode_wonder(hexs) or '（未知帧）'
        print(f'  {n:4d} × {hexs}  →  {meaning}')
    if not counts:
        print('  （一个 aa55 帧都没收到 —— 说话了吗？）')
    print()
    print('⚠️ 判读：')
    print('  · 若 **只在你说话时** 出现 `aa550300fb` → 它是对语音的反应（VAD 或唤醒）')
    print('  · 要区分「VAD」与「真的匹配上唤醒词」：**说一句不是唤醒词的话**看它响不响')
    print('  · 若出现 `aa5500XXfb` 且 XX 对应某个命令 → **模块自己做了识别**，')
    print('    直接把命令编号发出来（那就不需要 Jetson 侧跑 ASR）')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description='探测讯飞环形麦控制串口是否活着（诊断工装）')
    ap.add_argument('--port', default='/dev/ring_mic', help='控制串口（默认 /dev/ring_mic）')
    ap.add_argument('--listen', type=float, default=None,
                    help='纯监听模式，给定秒数（默认做握手探测）')
    ap.add_argument('--decode', type=float, default=None,
                    help='**按 aa55 帧协议解码**监听给定秒数 —— 信息量最大的模式')
    ap.add_argument('--no-modem-lines', action='store_true', help='跳过控制线读数')
    args = ap.parse_args()

    try:
        open_port(args.port).close()
    except Exception as e:
        print(f'❌ 打不开 {args.port}: {e}', file=sys.stderr)
        print('   是不是 awake_node 还占着它？先 sudo systemctl stop start_app_node.service',
              file=sys.stderr)
        return 1

    if not args.no_modem_lines:
        modem_lines(args.port)

    if args.decode is not None:
        return decode_stream(args.port, args.decode)
    if args.listen is not None:
        return listen(args.port, args.listen)
    return handshake_probe(args.port)


if __name__ == '__main__':
    sys.exit(main())
