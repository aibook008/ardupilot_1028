#!/usr/bin/env python3
"""DroneCAN (UAVCAN v0) 传输 CRC 计算 + 多帧拆包 —— Himark / 新 V1.0 倾转舵机控制器

用途：
  1. 计算一帧业务数据（不含 CRC 两字节与 tail byte）的 UAVCAN v0 transfer CRC，
     与抓包中 tail byte 前的两个 CRC 字节比对；
  2. 解码业务数据字段，确认解析没串位；
  3. --build 按 libcanard 发送逻辑拆出每条 CAN 帧的 8 字节（含 CRC 与 tail byte），
     用于下发前组包或与 candump 抓包逐帧比对。

支持的预设（--type）：
  tilt_fb      V1.0 倾转舵机控制器反馈  DTID=2026 (0x7EA) prio=20  25 字节  docx §2
  tilt_cmd     V1.0 倾转舵机控制器命令  DTID=2025 (0x7E9) prio=20  12 字节  docx §3
  himark_info  旧 Himark ServoInfo      DTID=2019 (0x7E3)          12 字节（位打包）
  himark_cmd   旧 Himark ServoCmd       DTID=2018 (0x7E2)          变长 ≤17×10bit

CRC 算法与 modules/DroneCAN/libcanard/canard.c 一致：
  - crcAddByte:      CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, 不反射)
  - crcAddSignature: 64 位数据类型签名按 LSB-first 拆 8 字节喂入 (canard.c:1862)
  - crcAdd:          payload 原序喂入 (canard.c:1871)
  - 多帧拆包:        首帧 CRC低+CRC高+5字节载荷, 之后每帧 7 字节, tail byte 规则 (canard.c:1235-1271)
接收侧同源校验流程见 canard.c:557。

用法：
  python3 himark_servo_crc.py                          # 内置：旧 Himark ServoInfo 算例
  python3 himark_servo_crc.py --type tilt_fb           # V1.0 反馈算例（文档示例 buffer）
  python3 himark_servo_crc.py --type tilt_cmd          # V1.0 命令算例（文档示例 buffer）
  python3 himark_servo_crc.py --type tilt_fb --build --tid 3 --node 100
                                                       # 组出实际 CAN 帧（含 CAN ID）
  python3 himark_servo_crc.py --type tilt_fb F0 D8 ... # 校验抓包 payload（hex 字节序列）
  python3 himark_servo_crc.py --sig 0x123456789ABCDEF0 <bytes...>   # 任意签名/任意消息

背景文档：docs/himarkservo.md、docs/UAVCAN通讯协议V1.0_倾转舵机控制器.docx、docs/df200-uavcan.md

已知文档问题（2026-09-22 版 docx）：
  - §3.4 命令算例：按文档给的 buffer+签名算出 0x99B5，文档写 0x2E4D，且该 buffer 前 8 字节
    与反馈示例完全相同（疑复制粘贴），单比特翻转/常见 CRC 变体均无法凑出 0x2E4D；
  - §2.3 反馈算例：正文说 25 字节，示例 buffer 只给了 24 字节（少 1 字节）。
    联调时以本脚本计算值为准，或抓真实帧反算确认。
"""
import argparse
import struct
import sys

# ---------------------------------------------------------------- CRC 核心（libcanard 同款）

def crc_add_byte(crc, b):
    """等价 canard.c crcAddByte: XOR byte<<8 后 LSB-first 移位 8 轮 (poly 0x1021)"""
    crc ^= (b << 8) & 0xFFFF
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    return crc


def crc_add_signature(crc, sig):
    """等价 canard.c crcAddSignature (canard.c:1862): 签名 LSB-first 逐字节"""
    for shift in range(0, 64, 8):
        crc = crc_add_byte(crc, (sig >> shift) & 0xFF)
    return crc


def crc_add(crc, data):
    """等价 canard.c crcAdd (canard.c:1871)"""
    for b in data:
        crc = crc_add_byte(crc, b)
    return crc


def transfer_crc(sig, payload):
    """UAVCAN v0 transfer CRC = crcAdd(crcAddSignature(0xFFFF, sig), payload)"""
    return crc_add(crc_add_signature(0xFFFF, sig), payload)


# ---------------------------------------------------------------- 多帧拆包（canard.c:1235-1271 同款）

def build_frames(payload, crc, tid):
    """按 libcanard 发送逻辑把 payload(+CRC) 拆成 CAN 帧数据字节。
    返回 [(帧数据字节列表), ...]；首帧 = CRC低+CRC高+5字节，之后每帧 7 字节载荷，
    末帧不补零（数据+tail 实际长度，canard.c:1254 的 DLC 处理结果）。
    tail byte: bit7=S, bit6=E, bit5=T(交替,首帧=0), bit4-0=transfer id。
    """
    stream = [crc & 0xFF, (crc >> 8) & 0xFF] + list(payload)
    frames = []
    toggle = 0
    idx = 0
    first = True
    while idx < len(stream):
        chunk = stream[idx:idx + 7]
        idx += len(chunk)
        tail = 0x80 if first else 0          # S 位
        if idx >= len(stream):
            tail |= 0x40                     # E 位
        tail |= (toggle << 5) | (tid & 0x1F)  # T 位 + transfer id
        frames.append(chunk + [tail])
        first = False
        toggle ^= 1
    return frames


# ---------------------------------------------------------------- 字段解码

def _signed16(v):
    return v - (1 << 16) if v >= (1 << 15) else v


def decode_servoinfo(buf):
    """旧 Himark ServoInfo：LSB-first 位流打包，96 bit 固定 12 字节"""
    if len(buf) != 12:
        print(f"  (ServoInfo 需 12 字节，当前 {len(buf)}，跳过解码)")
        return
    bits = []
    for byte in buf:
        bits += [(byte >> i) & 1 for i in range(8)]
    pos = 0

    def take(n):
        nonlocal pos
        v = 0
        for k in range(n):
            v |= bits[pos + k] << k
        pos += n
        return v

    f = [
        ("servo_id", take(5)),
        ("pwm_input(us)", take(12)),
        ("pos_cmd(0.01°)", _signed16(take(16))),
        ("pos_sensor(0.01°)", _signed16(take(16))),
        ("voltage(V)", take(12) * 0.01),
        ("current(A)", take(10) * 0.01),
        ("pcb_temp(°C)", take(10) * 0.2 - 40),
        ("motor_temp(°C)", take(10) * 0.2 - 40),
        ("error_status", take(5)),
    ]
    assert pos == 96
    return f


def decode_tilt_cmd(buf):
    """V1.0 倾转舵机控制命令（docx §3.5）：字节对齐小端，12 字节"""
    if len(buf) < 8:
        print(f"  (命令需 12 字节，当前 {len(buf)}，跳过解码)")
        return
    pos = struct.unpack_from("<4h", bytes(buf), 0)
    f = [(f"ch{i}_pos_cmd raw={pos[i]} (x0.01={pos[i] * 0.01})", "") for i in range(4)]
    if len(buf) >= 12:
        lock_names = {0x55: "闭锁", 0xAA: "开锁"}
        for i in range(4):
            v = buf[8 + i]
            f.append((f"ch{i}_lock=0x{v:02X}", lock_names.get(v, "仅执行位置指令")))
    return f


_TILT_STATUS_BITS = ["CAN2故障", "CAN1故障", "DSP故障", "正常工作", "制动解锁", "rsv5", "rsv6", "rsv7"]
_TILT_FAULT_BITS = ["电位计", "旋变", "开环", "停转", "内部电源", "制动器", "过流", "过/欠压"]


def decode_tilt_fb(buf):
    """V1.0 倾转舵机控制器反馈（docx §2.4-2.6）：字节对齐小端，25 字节"""
    f = []
    n = min(len(buf), 25)
    if n >= 8:
        for i, v in enumerate(struct.unpack_from("<4h", bytes(buf), 0)):
            f.append((f"ch{i}_pos(mm)", v * 0.01))
    if n >= 9:
        f.append(("supply_voltage(V)", buf[8] * 0.2))
    if n >= 17:
        for i, v in enumerate(struct.unpack_from("<4h", bytes(buf), 9)):
            f.append((f"ch{i}_current(A)", v * 0.01))
    if n >= 21:
        for i in range(min(4, n - 17)):
            v = buf[17 + i]
            bits = ", ".join(name for b, name in enumerate(_TILT_STATUS_BITS) if v >> b & 1) or "无"
            f.append((f"ch{i}_status=0x{v:02X}", bits))
    if n >= 25:
        for i in range(min(4, n - 21)):
            v = buf[21 + i]
            bits = ", ".join(name for b, name in enumerate(_TILT_FAULT_BITS) if v >> b & 1) or "无"
            f.append((f"ch{i}_fault=0x{v:02X}", bits))
    if n == 24:
        f.append(("(缺 1 字节)", "文档示例只有 24B，ch3_fault 缺失"))
    return f


# ---------------------------------------------------------------- 预设

def _hex_list(s):
    return [int(x for x in [0])] if False else None  # placeholder, unused


PRESETS = {
    "tilt_fb": dict(
        title="V1.0 倾转舵机控制器反馈 (docx §2)",
        dtid=2026, prio=20, sig=0x3A8120AAB9EF661A, expect_len=25,
        example=[0xF0, 0xD8, 0x00, 0x00, 0x94, 0x11, 0x50, 0x46,
                 0x78, 0x1E, 0x00, 0x28, 0x00, 0x32, 0x00, 0x3C,
                 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00],
        example_note="文档示例只有 24 字节(§2.2/2.3 称 25 字节)，少 1 字节；CRC 按 24B 算 = 0xF7AF",
        decode=decode_tilt_fb,
    ),
    "tilt_cmd": dict(
        title="V1.0 倾转舵机控制器命令 (docx §3)",
        dtid=2025, prio=20, sig=0x389765D2EAC0E400, expect_len=12,
        example=[0xF0, 0xD8, 0x00, 0x00, 0x94, 0x11, 0x50, 0x46,
                 0x55, 0xAA, 0x55, 0xAA],
        example_note="文档算例 CRC 写 0x2E4D，但按文档自带 buffer+签名算 = 0x99B5，对不上；"
                     "且该 buffer 前 8 字节与反馈示例相同，疑复制粘贴错误",
        decode=decode_tilt_cmd,
    ),
    "himark_info": dict(
        title="旧 Himark ServoInfo (docs/himarkservo.md)",
        dtid=2019, prio=None, sig=0xCA8F4B8F97D23B57, expect_len=12,
        example=[0x02, 0x39, 0x72, 0x01, 0xF0, 0x81, 0xED, 0x90, 0x00, 0x9C, 0xE4, 0x00],
        example_note="真实抓包算例，CRC = 0xA0A8",
        decode=decode_servoinfo,
    ),
    "himark_cmd": dict(
        title="旧 Himark ServoCmd (docs/himarkservo.md)",
        dtid=2018, prio=None, sig=0x5D09E48551CE9194, expect_len=None,
        example=None,
        example_note="变长 uint10[<=17]，无内置算例",
        decode=None,
    ),
}


def main(argv):
    ap = argparse.ArgumentParser(description="DroneCAN v0 transfer CRC + 多帧拆包验证")
    ap.add_argument("bytes_", nargs="*", metavar="XX", help="payload 字节(hex)，缺省用内置算例")
    ap.add_argument("--type", default="himark_info", choices=sorted(PRESETS))
    ap.add_argument("--sig", help="覆盖数据类型签名(hex)")
    ap.add_argument("--tid", type=lambda x: int(x, 0), default=0, help="transfer id 0-31 (tail byte, --build 用)")
    ap.add_argument("--node", type=lambda x: int(x, 0), default=100, help="节点 ID（打印 CAN ID 用）")
    ap.add_argument("--build", action="store_true", help="拆出每条 CAN 帧（含 CRC 字节与 tail byte）")
    args = ap.parse_args(argv)

    p = PRESETS[args.type]
    sig = int(args.sig, 16) if args.sig else p["sig"]
    data = [int(x.replace("0x", "").replace("0X", "").replace(",", ""), 16) for x in args.bytes_] or p["example"]
    if not data:
        ap.error(f"--type {args.type} 无内置算例，请传 payload 字节")

    # 自检：CRC-16/CCITT-FALSE 标准检验值
    assert crc_add(0xFFFF, b"123456789") == 0x29B1, "CRC 实现有误!"

    print(f"类型      : {p['title']}")
    print(f"签名      : 0x{sig:016X}")
    if p["prio"] is not None:
        can_id = (p["prio"] << 24) | (p["dtid"] << 8) | args.node
        print(f"CAN ID    : 0x{can_id:08X} (prio={p['prio']}, DTID={p['dtid']}, node={args.node})")
    print(f"payload   : ({len(data)} 字节) " + " ".join(f"{b:02X}" for b in data))
    if p["expect_len"] and len(data) != p["expect_len"]:
        print(f"⚠ 长度 {len(data)} ≠ 预期 {p['expect_len']} 字节")

    crc = transfer_crc(sig, data)
    print(f"CRC       : 0x{crc:04X}   (on-wire 低字节在前: {crc & 0xFF:02X} {crc >> 8:02X})")

    if p["example"] and data == p["example"] and p.get("example_note"):
        print(f"※ {p['example_note']}")

    if p["decode"]:
        fields = p["decode"](data)
        if fields:
            print("字段解码:")
            for k, v in fields:
                print(f"  {k:28s} {v}")

    if args.build:
        frames = build_frames(data, crc, args.tid)
        print(f"\n多帧拆包 (--build, transfer id={args.tid & 0x1F})，共 {len(frames)} 帧:")
        for i, fr in enumerate(frames):
            tail = fr[-1]
            s, e, t, tid = (tail >> 7) & 1, (tail >> 6) & 1, (tail >> 5) & 1, tail & 0x1F
            print(f"  帧{i + 1}: DLC={len(fr):d}  " + " ".join(f"{b:02X}" for b in fr)
                  + f"   [S={s} E={e} T={t} tid={tid}]")


if __name__ == "__main__":
    main(sys.argv[1:])
