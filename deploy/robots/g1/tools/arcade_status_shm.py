"""아케이드 상태 칸(/dev/shm/g1_arcade_status, magic 0x6708) 읽기 — 로봇 위 중계가 import 한다.

g1_ctrl 의 정책 스레드가 틱(50 Hz)마다 한 번 쓴다(include/ArcadeStatus.h). ⚠ 칸 배치는 그 C++ 구조체와 같은 커밋으로만
바꾼다 — tests/test_arcade_shm_layout.py 가 구조체를 정규식으로 읽어 FMT·FIELDS 와 대조한다.
role · skill · notice 는 core 표(role_names() · skill_names() · notice_text(code))의 인덱스/코드다. 문자열은 칸에 없다.

read(path) → magic 을 뺀 칸 전부의 dict, 아니면 None:
  파일 없음 · 크기가 88 B 가 아님 · magic 이 0x6708 이 아님 · 안정된 프레임을 64 번 안에 못 얻음.
낡음(정책이 멈춤)은 여기서 판정하지 않는다 — 받는 쪽이 tick 이 늘지 않는 것으로 본다.
(상태 칸 g1_masked_status 0x6706 읽기는 status_shm.py — 별개의 칸이다.)

찢긴 읽기 막기(seqlock, 쓰는 쪽은 ArcadeStatus.h 머리 주석): seq(4 B) 읽기 → 프레임(88 B) 읽기 → seq 다시 읽기. 셋이 같은
짝수(프레임 속 seq 칸 포함)일 때만 받는다. 셋은 각각 pread 한 번이다 — 커널은 각 복사 전후로 쪽 참조를 잡고 놓는(완전 순서
원자 연산) 길을 지나므로 세 읽기가 프로그램 순서대로 보인다고 본다(x86 은 TSO 라 원래 그렇다 · arm64 는 이 근거의 추정, 실측
없음). 칸은 모두 4 B 정렬이라 한 칸이 반쯤 바뀐 값으로 읽히지는 않는다.

python 3.8 문법만 쓴다(로봇 기본 python) — `list[int]` 주석·`X | None` 금지(tests/test_arcade_shm_layout.py 가 본다).
"""
from __future__ import annotations

import os
import struct
from typing import Optional

SHM_PATH = "/dev/shm/g1_arcade_status"
MAGIC = 0x6708          # 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay · 0x6706 상태 칸 · 0x6707 의도 칸 과 다르다
# < little-endian, packed. 이름 순서 = ArcadeStatus.h 칸 순서.
FMT = "<iIIiiiiiiiiiiiiiIfffff"
FIELDS = ("magic", "seq", "tick", "role", "skill", "mode", "preset", "route_step", "notice", "safety_class", "busy",
          "qd_warn", "qd_crit", "intent_stale", "rearmed", "fsm_state", "tick_us", "z_fk", "tilt_deg", "cmd_vx", "cmd_vy",
          "cmd_wz")
SIZE = 88
MAX_TRIES = 64
_SEQ_OFF = 4

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
    # 사람이 보는 용도: python3 arcade_status_shm.py [경로] — 10 Hz 로 한 줄씩(Ctrl+C 로 끝)
    import sys
    import time
    p = sys.argv[1] if len(sys.argv) > 1 else SHM_PATH
    try:
        while True:
            print(read(p), flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
