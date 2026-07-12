"""E-stop 채널 계약 — /dev/shm/g1_estop.

Python 생산자(브릿지 SafetyMonitor)가 이 packed 12B struct를 하트비트로 write하면,
C++(EstopChannel.h `fsm_estop_poll`)가 읽어 flag!=0 또는 하트비트 stale 시 강제 Passive(damping).
⚠ 이 레이아웃을 EstopChannel.h 구조체와 반드시 동기화.

레이아웃(little-endian, packed):  magic(i) seq(I) flag(i) = 12 bytes
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_estop"
MAGIC = 0x6703             # 0x6701 gui, 0x6702 vr, 0x6703 estop
FMT = "<iIi"              # magic seq flag = 12 bytes


def write(seq: int, flag: int) -> None:
    """원자적 publish. flag=0 -> run, !=0 -> 강제 Passive. seq는 매 호출 증가시켜 하트비트로 쓴다
    (정지 시 C++가 ~0.5s 후 dead-writer로 판단해 fail-safe damping)."""
    buf = struct.pack(FMT, MAGIC, int(seq), int(flag))
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish (no torn reads)


def clear() -> None:
    """정상 종료 시 disarm: 파일 제거 -> C++ 파일없음=미무장(정상 FSM)."""
    try:
        os.remove(SHM_PATH)
    except FileNotFoundError:
        pass
