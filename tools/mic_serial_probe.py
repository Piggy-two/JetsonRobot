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


def main():
    ap = argparse.ArgumentParser(
        description='探测讯飞环形麦控制串口是否活着（诊断工装）')
    ap.add_argument('--port', default='/dev/ring_mic', help='控制串口（默认 /dev/ring_mic）')
    ap.add_argument('--listen', type=float, default=None,
                    help='纯监听模式，给定秒数（默认做握手探测）')
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

    if args.listen is not None:
        return listen(args.port, args.listen)
    return handshake_probe(args.port)


if __name__ == '__main__':
    sys.exit(main())
