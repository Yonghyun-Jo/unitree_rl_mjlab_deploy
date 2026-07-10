"""
pico_wire.py — PICO 캡처 UDP 프레임 바이너리 레이아웃 (노트북·com1 공유 단일 소스).

목표: 한 프레임을 MTU 미만(~806B) 고정 바이너리로 → 단일 UDP 데이터그램 → 손실=스킵.
float32(little-endian) 고정 struct. 노트북 발행자와 com1 수신자가 이 모듈을 함께 import.

패킷 레이아웃 (모두 little-endian):
  [헤더 16B]  magic(2s='PC') ver(u8=1) flags(u8) seq(u32) t_capture_ns(u64)
  [headset 28B]   7×f32  (x,y,z,qx,qy,qz,qw)
  [L ctrl 45B]    pose(7×f32) trigger(f32) grip(f32) axis(2×f32) buttons(u8)
  [R ctrl 45B]    〃
  [body 672B]     24관절 × 7×f32   (body_available=0 이면 전부 0)
  총 = 16+28+45+45+672 = 806 bytes

flags 비트:  bit0=body_available, bit1=streaming(valid headset)
buttons 비트: bit0=primary, bit1=secondary, bit2=menu, bit3=axis_click
"""
import struct

MAGIC = b"PC"
VERSION = 1

_HEADER_FMT = "<2sBBIQ"          # magic, ver, flags, seq, t_ns
HEADER_SIZE = struct.calcsize(_HEADER_FMT)     # 16
_HEADSET_FMT = "<7f"             # 28
_CTRL_FMT = "<11fB"              # pose7, trigger, grip, axis2, buttons  = 45
_BODY_FMT = "<168f"             # 24*7 = 672
_CTRL_SIZE = struct.calcsize(_CTRL_FMT)         # 45
PACKET_SIZE = HEADER_SIZE + 28 + _CTRL_SIZE * 2 + 672   # 806


def _pack_ctrl(c):
    b = ((1 if c["primary"] else 0) | (2 if c["secondary"] else 0)
         | (4 if c["menu"] else 0) | (8 if c["axis_click"] else 0))
    p = c["pose"]
    ax = c["axis"]
    return struct.pack(_CTRL_FMT, p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                       float(c["trigger"]), float(c["grip"]), float(ax[0]), float(ax[1]), b)


def _unpack_ctrl(buf, off):
    v = struct.unpack_from(_CTRL_FMT, buf, off)
    ctrl = {
        "pose": list(v[0:7]),
        "trigger": v[7], "grip": v[8],
        "axis": [v[9], v[10]],
        "primary": bool(v[11] & 1), "secondary": bool(v[11] & 2),
        "menu": bool(v[11] & 4), "axis_click": bool(v[11] & 8),
    }
    return ctrl, off + _CTRL_SIZE


def pack_frame(seq, t_ns, streaming, headset, lc, rc, body_avail, body_joints):
    """dict/list 입력 → bytes(806). body_joints: 24×7 리스트 또는 None."""
    flags = (1 if body_avail else 0) | (2 if streaming else 0)
    out = bytearray()
    out += struct.pack(_HEADER_FMT, MAGIC, VERSION, flags, seq & 0xFFFFFFFF, int(t_ns) & 0xFFFFFFFFFFFFFFFF)
    out += struct.pack(_HEADSET_FMT, headset[0], headset[1], headset[2], headset[3], headset[4], headset[5], headset[6])
    out += _pack_ctrl(lc)
    out += _pack_ctrl(rc)
    if body_avail and body_joints is not None:
        flat = [v for j in body_joints for v in j[:7]]
    else:
        flat = [0.0] * 168
    out += struct.pack(_BODY_FMT, *flat)
    return bytes(out)


def unpack_frame(buf):
    """bytes → dict (JSON 프레임과 동일 shape). 잘못된/짧은 패킷은 None."""
    if len(buf) < PACKET_SIZE:
        return None
    magic, ver, flags, seq, t_ns = struct.unpack_from(_HEADER_FMT, buf, 0)
    if magic != MAGIC or ver != VERSION:
        return None
    off = HEADER_SIZE
    headset = list(struct.unpack_from(_HEADSET_FMT, buf, off)); off += 28
    lc, off = _unpack_ctrl(buf, off)
    rc, off = _unpack_ctrl(buf, off)
    body_flat = struct.unpack_from(_BODY_FMT, buf, off); off += 672
    body_avail = bool(flags & 1)
    streaming = bool(flags & 2)
    joints = [list(body_flat[i * 7:(i + 1) * 7]) for i in range(24)] if body_avail else None
    return {
        "seq": seq,
        "t_capture_ns": t_ns,
        "streaming": streaming,
        "headset": headset,
        "controllers": {"left": lc, "right": rc},
        "body": {"available": body_avail, "joints": joints},
    }
