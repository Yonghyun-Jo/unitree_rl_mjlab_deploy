"""아케이드 의도 칸(/dev/shm/g1_arcade_intent, magic 0x6707) — 로봇 위 중계가 50 Hz 로 쓰고 g1_ctrl 이 읽는다.

브라우저는 «의도»(이동 축 + 눌린 이벤트)만 보낸다. C++ 쪽 = include/ArcadeIntent.h `struct ArcadeIntentFrame`.
⚠ 칸 배치는 그 구조체와 같은 커밋으로만 바꾼다 — tests/test_arcade_shm_layout.py 가 정규식으로 대조한다.

write_intent(fwd, side, turn, run, events, event_age_ms) — 호출마다 seq(하트비트)를 +1, events 가 0 이 아니면 event_seq 도 +1.
  os.replace 로 통째 교체(찢긴 읽기 없음 · gui_shm.py 와 같은 방식 — 임시 파일 = <dst>.<pid>.tmp, 쓰는 쪽마다 따로). 값이 같아도
  매번 쓴다(하트비트).
  events = 마지막 이벤트 비트(EV_*, event_seq 와 짝) · event_age_ms = 쓰는 쪽이 아는 그 이벤트의 나이(읽는 쪽이 500 ms 넘으면 버린다).
  «끈적한 이벤트»: 0 이 아닌 events 를 주면 새 이벤트(event_seq +1)다. 이후 events=0 으로 부르면 마지막 비트와 event_seq 를 그대로
  다시 싣고 age 만 지금 시각으로 갱신한다(읽는 쪽이 이벤트 프레임을 놓쳐도 소실되지 않게 · 중복 적용은 event_seq 가 막는다).
  age = 넘긴 event_age_ms + 그 이벤트를 쓴 뒤 흐른 시간(단조 시계 _mono).
read(path) → magic 을 뺀 칸의 dict, 아니면 None(시험·점검용).

python 3.8 문법만 쓴다(로봇 기본 python).
"""
from __future__ import annotations

import math
import os
import struct
import time
from typing import Optional

SHM_PATH = "/dev/shm/g1_arcade_intent"
MAGIC = 0x6707          # 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay · 0x6706 상태 칸 · 0x6708 아케이드 상태 와 다르다
FMT = "<iIfffiIII"
FIELDS = ("magic", "seq", "fwd", "side", "turn", "run", "events", "event_seq", "event_age_ms")
SIZE = 36
EV_JUMP, EV_CRAWL, EV_GETUP, EV_LIE, EV_BOX = 1, 2, 4, 8, 16   # = core arcade::Event

assert struct.calcsize(FMT) == SIZE and len(FIELDS) == len(FMT) - 1

_state = {"seeded": False, "seq": 0, "event_seq": 0, "events": 0, "event_t": 0.0}
_mono = time.monotonic          # 시험이 바꿔 끼운다


def _reset() -> None:
    """쓰기 상태를 처음으로(시험용) — 다음 write_intent 가 다시 시드한다."""
    _state.update(seeded=False, seq=0, event_seq=0, events=0, event_t=0.0)


def _clamp(x: float) -> float:
    x = float(x)
    if not math.isfinite(x):          # NaN · ±inf = 0 (±1 로 바꾸지 않는다)
        return 0.0
    return max(-1.0, min(1.0, x))


def tmp_path(dst: str) -> str:
    """원자 교체용 임시 파일 = <dst>.<pid>.tmp — 쓰는 쪽이 둘이면(중계 · 시험 재생기가 겹치면) 서로의 임시 파일을 덮지 않게(검토 I6)."""
    return "%s.%d.tmp" % (dst, os.getpid())


def write_intent(fwd: float, side: float, turn: float, run: int = 0, events: int = 0,
                 event_age_ms: int = 0, path: Optional[str] = None) -> None:
    """프레임 하나를 원자 교체로 낸다. path 기본 = SHM_PATH(호출 때 읽는다 — 시험이 바꿔 끼운다)."""
    if not _state["seeded"]:                     # 쓰는 쪽을 다시 띄워도 이전 seq 와 겹치지 않게 시각으로 시드
        _state["seeded"] = True
        _state["seq"] = time.time_ns() & 0xFFFFFFFF
        _state["event_seq"] = (time.time_ns() >> 7) & 0xFFFFFFFF
        _state["events"], _state["event_t"] = 0, 0.0
    _state["seq"] = (_state["seq"] + 1) & 0xFFFFFFFF
    now = _mono()
    if int(events) != 0:                         # 새 이벤트 — 끈적하게 다음 이벤트까지 싣는다
        _state["event_seq"] = (_state["event_seq"] + 1) & 0xFFFFFFFF
        _state["events"] = int(events) & 0xFFFFFFFF
        _state["event_t"] = now - max(0, int(event_age_ms)) / 1000.0
    age = int(round(max(0.0, now - _state["event_t"]) * 1000.0)) if _state["events"] else 0
    age = min(max(0, age), 0xFFFFFFFF)           # 포화(넘치면 struct.error 대신 최대값 = 낡음)
    buf = struct.pack(FMT, MAGIC, _state["seq"], _clamp(fwd), _clamp(side), _clamp(turn),
                      1 if run else 0, _state["events"], _state["event_seq"], age)
    dst = path if path is not None else SHM_PATH
    tmp = tmp_path(dst)
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, dst)


def read(path: str = SHM_PATH) -> Optional[dict]:
    """magic 을 뺀 칸 전부(이름 = FIELDS[1:]) 또는 None(없음 · 크기 · magic 다름)."""
    try:
        with open(path, "rb") as f:
            buf = f.read(SIZE + 1)
    except OSError:
        return None
    if len(buf) != SIZE:
        return None
    vals = struct.unpack(FMT, buf)
    if vals[0] != MAGIC:
        return None
    return dict(zip(FIELDS[1:], vals[1:]))
