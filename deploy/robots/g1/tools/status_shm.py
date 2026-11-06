"""g1_ctrl 상태 칸(/dev/shm/g1_masked_status, magic 0x6706) 읽기 — 로봇 위 중계(arcade_relay.py)가 import 한다.

g1_ctrl 의 정책 스레드가 틱(50 Hz)마다 한 번 쓴다(include/MaskedStatus.h). ⚠ 칸 배치는 그 C++ 구조체와 같은 커밋으로만
바꾼다 — tests/test_status_shm_layout.py 가 구조체를 정규식으로 읽어 FMT·FIELDS 와 대조한다.

read(path) → magic 을 뺀 칸 전부의 dict(g1_arcade docs/protocol_relay_v1.md §3 의 status), 아니면 None:
  파일 없음 · 크기가 80 B 가 아님 · magic 이 0x6706 이 아님 · 안정된 프레임을 64 번 안에 못 얻음(쓰는 쪽이 쓰는 중에 죽음 등).
낡음(정책이 멈춤)은 여기서 판정하지 않는다 — 받는 쪽이 tick 이 늘지 않는 것으로 본다(active_mode 0 = 정책이 떠났다).

찢긴 읽기 막기(seqlock, 쓰는 쪽은 MaskedStatus.h 머리 주석): seq(4 B) 읽기 → 프레임(80 B) 읽기 → seq 다시 읽기. 셋이 같은
짝수(프레임 속 seq 칸 포함)일 때만 받는다. 셋은 각각 pread 한 번이다 — 커널은 각 복사 전후로 쪽 참조를 잡고 놓는(완전 순서
원자 연산) 길을 지나므로 세 읽기가 프로그램 순서대로 보인다고 본다(x86 은 TSO 라 원래 그렇다 · arm64 는 이 근거의 추정, 실측
없음). 칸은 모두 4 B 정렬이라 한 칸이 반쯤 바뀐 값으로 읽히지는 않는다.

python 3.8 문법만 쓴다(로봇 기본 python) — `list[int]` 주석·`X | None` 금지(tests/test_status_shm_layout.py 가 본다).
"""
from __future__ import annotations

import os
import struct
from typing import Optional

SHM_PATH = "/dev/shm/g1_masked_status"
MAGIC = 0x6706          # 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay 와 다르다
# < little-endian, packed. 이름 순서 = MaskedStatus.h 칸 순서.
FMT = "<iIIiiiiiiiIiiIifffff"
FIELDS = ("magic", "seq", "tick", "active_mode", "last_req_mode", "last_req_result", "refusal_code", "refused_mode",
          "m5_preset", "m5_arrived", "m5_arrive_seq", "safety", "fsm_state", "gui_seq_applied", "relay_stale",
          "z_fk", "tilt_deg", "vx_cmd", "vy_cmd", "wz_cmd")
SIZE = 80
MAX_TRIES = 64
_SEQ_OFF = 4

# 값의 뜻(MaskedStatus.h 와 같다) — 받는 쪽이 문장을 고를 때
REQ_NONE, REQ_ACCEPTED, REQ_REFUSED = 0, 1, 2
REFUSAL = {0: "none", 1: "unknown mode", 2: "slot lacks mode", 3: "exit blocked", 4: "enter window",
           5: "channel not allowed", 6: "posture refused"}
SAFETY = {0: "ok", 1: "warn", 2: "crit"}

assert struct.calcsize(FMT) == SIZE and len(FIELDS) == len(FMT) - 1


def _seq(fd: int) -> Optional[int]:
    b = os.pread(fd, 4, _SEQ_OFF)
    return struct.unpack("<I", b)[0] if len(b) == 4 else None


def read(path: str = SHM_PATH) -> Optional[dict]:
    """magic 을 뺀 칸 전부(이름 = FIELDS[1:]) 또는 None. 한 번 부를 때 pread 3 번(안정되면)."""
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
    except OSError:
        return None
    try:
        if os.fstat(fd).st_size != SIZE:
            return None
        for _ in range(MAX_TRIES):
            s0 = _seq(fd)
            if s0 is None:
                return None
            if s0 & 1:                       # 쓰는 중
                continue
            buf = os.pread(fd, SIZE, 0)
            s1 = _seq(fd)
            if len(buf) != SIZE:
                return None
            vals = struct.unpack(FMT, buf)
            if vals[0] != MAGIC:
                return None
            if s1 != s0 or vals[1] != s0:    # 읽는 사이에 쓰였다
                continue
            return dict(zip(FIELDS[1:], vals[1:]))
        return None
    except OSError:
        return None
    finally:
        os.close(fd)


if __name__ == "__main__":
    # 사람이 보는 용도: python3 status_shm.py [경로] — 10 Hz 로 한 줄씩(Ctrl+C 로 끝)
    import sys
    import time
    p = sys.argv[1] if len(sys.argv) > 1 else SHM_PATH
    try:
        while True:
            print(read(p), flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
