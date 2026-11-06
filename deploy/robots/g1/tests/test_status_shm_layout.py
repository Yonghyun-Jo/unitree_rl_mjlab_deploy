#!/usr/bin/env python3
"""/dev/shm/g1_masked_status(g1_ctrl → 중계) 레이아웃이 C++ `struct MaskedStatus`(include/MaskedStatus.h)와 python
`status_shm.FMT`·`FIELDS` 사이에서 안 갈렸는지 + read() 의 거름(파일 없음·크기·magic·seqlock 재시도 64 회) + C++ 로 쓴 칸을
python 이 그대로 읽는지(g++ 가 있으면).
한쪽만 칸을 더하면 둘 다 «돌아가는데» 중계가 엉뚱한 칸을 브라우저에 보낸다(거부 사유·모드가 쓰레기).
실행:  python3 deploy/robots/g1/tests/test_status_shm_layout.py   (pytest 로도)
"""
from __future__ import annotations

import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(G1 / "tools"))
_CH = {"int32_t": "i", "uint32_t": "I", "float": "f"}
# 브리프(g1_arcade stage2a task-2)의 칸 순서 그대로 — 구조체를 고칠 때 이 표도 같이 바꾼다(프로토콜 v1 이 이 이름을 쓴다).
_BRIEF = [("magic", "i"), ("seq", "I"), ("tick", "I"), ("active_mode", "i"), ("last_req_mode", "i"),
          ("last_req_result", "i"), ("refusal_code", "i"), ("refused_mode", "i"), ("m5_preset", "i"),
          ("m5_arrived", "i"), ("m5_arrive_seq", "I"), ("safety", "i"), ("fsm_state", "i"),
          ("gui_seq_applied", "I"), ("relay_stale", "i"), ("z_fk", "f"), ("tilt_deg", "f"),
          ("vx_cmd", "f"), ("vy_cmd", "f"), ("wz_cmd", "f")]


def _cpp_src():
    return (G1 / "include/MaskedStatus.h").read_text(encoding="utf-8")


def _cpp_fields():
    m = re.search(r"struct MaskedStatus\s*\{(.*?)\n\};", _cpp_src(), re.S)
    assert m, "MaskedStatus.h 에서 struct MaskedStatus 를 못 찾았다"
    fmt, names = "", []
    for line in m.group(1).splitlines():
        code = line.split("//")[0]
        if not code.strip():
            continue
        mm = re.match(r"\s*(int32_t|uint32_t|float)\s+([\w\s,]+);\s*$", code)
        # 모르는 타입(int16_t·double·배열…)을 조용히 건너뛰면 FMT 가 «맞아 보이는» 채로 C++ 만 칸이 밀린다.
        assert mm, f"struct MaskedStatus 의 이 줄을 해석 못 한다 — 타입을 int32_t·uint32_t·float 로: {line.strip()!r}"
        ns = [n.strip() for n in mm.group(2).split(",") if n.strip()]
        fmt += _CH[mm.group(1)] * len(ns)
        names += ns
    return "<" + fmt, names


def _pack(**over):
    import status_shm
    v = dict(magic=status_shm.MAGIC, seq=4, tick=123, active_mode=5, last_req_mode=6, last_req_result=2,
             refusal_code=4, refused_mode=6, m5_preset=4, m5_arrived=1, m5_arrive_seq=7, safety=1, fsm_state=6,
             gui_seq_applied=0xFFFFFFF0, relay_stale=1, z_fk=0.75, tilt_deg=2.5, vx_cmd=0.25, vy_cmd=-0.5, wz_cmd=1.0)
    v.update(over)
    return struct.pack(status_shm.FMT, *[v[k] for k in status_shm.FIELDS]), v


def test_layout_matches():
    import status_shm
    fmt, names = _cpp_fields()
    assert status_shm.FMT == fmt, f"python {status_shm.FMT} != C++ {fmt}"
    assert list(status_shm.FIELDS) == names, f"칸 이름 순서가 다르다: {status_shm.FIELDS} != {names}"
    assert struct.calcsize(status_shm.FMT) == status_shm.SIZE == 80


def test_brief_field_order():
    """구조체가 브리프(=프로토콜 v1 status 칸)의 이름·형·순서 그대로인가."""
    fmt, names = _cpp_fields()
    assert names == [n for n, _ in _BRIEF], names
    assert fmt == "<" + "".join(t for _, t in _BRIEF), fmt


def test_magic_matches_and_differs():
    """0x6706 — 다른 공유 메모리 채널(gui v1 0x6701 · vr 0x6702 · estop 0x6703 · gui v2 0x6704 · relay 0x6705)과 겹치지 않는다."""
    import gui_shm
    import status_shm
    m = re.search(r"MASKED_STATUS_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", _cpp_src())
    assert m and int(m.group(1), 16) == status_shm.MAGIC == 0x6706
    assert status_shm.MAGIC not in (0x6701, 0x6702, 0x6703, 0x6704, 0x6705)
    assert status_shm.MAGIC not in (gui_shm.MAGIC, gui_shm.MAGIC_GUI_RELAY)
    estop_h = (G1.parent.parent / "include/FSM/EstopChannel.h").read_text(encoding="utf-8")
    assert status_shm.MAGIC != int(re.search(r"ESTOP_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", estop_h).group(1), 16)
    vr_py = (G1 / "teleop/vr_shm.py").read_text(encoding="utf-8")
    assert status_shm.MAGIC != int(re.search(r"^MAGIC\s*=\s*0x([0-9A-Fa-f]+)", vr_py, re.M).group(1), 16)
    assert re.search(r'DEFAULT_PATH\s*=\s*"([^"]+)"', _cpp_src()).group(1) == status_shm.SHM_PATH == "/dev/shm/g1_masked_status"


def test_read_roundtrip():
    """magic 을 뺀 모든 칸이 같은 이름·값으로 나온다(프로토콜 v1 §3 의 status)."""
    import status_shm
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, "st")
        buf, v = _pack()
        Path(p).write_bytes(buf)
        r = status_shm.read(p)
        assert r is not None
        assert list(r.keys()) == list(status_shm.FIELDS[1:]) and "magic" not in r
        for k in status_shm.FIELDS[1:]:
            assert r[k] == v[k], (k, r[k], v[k])
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_read_none_cases():
    """파일 없음 · 크기 다름 · magic 다름 · 쓰는 중(홀수 seq)에 멈춤 → None."""
    import status_shm
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, "st")
        assert status_shm.read(p) is None                                     # 없음
        buf, _ = _pack()
        Path(p).write_bytes(buf[:79]); assert status_shm.read(p) is None      # 짧음
        Path(p).write_bytes(buf + b"\0"); assert status_shm.read(p) is None   # 김
        Path(p).write_bytes(_pack(magic=0x6704)[0]); assert status_shm.read(p) is None   # 남의 칸
        Path(p).write_bytes(_pack(seq=7)[0]); assert status_shm.read(p) is None          # 쓰는 중에 죽은 쓰는 쪽
        Path(p).write_bytes(_pack(seq=8)[0]); assert status_shm.read(p) is not None      # 대조: 짝수면 읽힌다
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _with_fake_pread(frames, fn):
    """os.pread 를 가로채 seq 읽기(4 B @4)·프레임 읽기(80 B @0)에 차례로 frames 의 값을 돌려준다. 호출 기록을 돌려준다."""
    import status_shm
    calls = []
    it = iter(frames)
    real = status_shm.os.pread

    def fake(fd, n, off):
        calls.append((n, off))
        b = next(it)
        return b[4:8] if (n, off) == (4, 4) else b

    status_shm.os.pread = fake
    try:
        return fn(), calls
    finally:
        status_shm.os.pread = real


def test_read_retries_until_stable():
    """seqlock: 홀수 seq · 앞뒤가 다른 seq · 프레임 속 seq 가 다른 경우는 다시 읽는다. 안정되면 받는다."""
    import status_shm
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, "st")
        good, v = _pack(seq=10)
        Path(p).write_bytes(good)
        odd = _pack(seq=9)[0]
        s12 = _pack(seq=12)[0]
        # 시도 1: seq 홀수 → 바로 다시 · 시도 2: 앞 10, 프레임 10, 뒤 12(쓰는 중 바뀜) → 다시 · 시도 3: 앞 12, 프레임 속 10 → 다시
        # 시도 4: 10 / 10 / 10 → 받음
        frames = [odd, good, good, s12, s12, good, s12, good, good, good]
        r, calls = _with_fake_pread(frames, lambda: status_shm.read(p))
        assert r is not None and r["seq"] == 10 and r["tick"] == v["tick"]
        assert len(calls) == 1 + 3 + 3 + 3, calls
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_read_gives_up_after_64():
    import status_shm
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, "st")
        Path(p).write_bytes(_pack(seq=10)[0])
        odd = _pack(seq=11)[0]
        r, calls = _with_fake_pread([odd] * 1000, lambda: status_shm.read(p))
        assert r is None
        assert status_shm.MAX_TRIES == 64 and len(calls) == 64, len(calls)   # seq 읽기 64 번에서 멈춘다
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_py38_runtime_syntax():
    """로봇은 python 3.8 — 3.9+ 런타임 문법(`list[int]` 주석·`X | None`)은 «문법은 통과하고 실행에서 죽는다»(memory
    robot-telemetry-already-built-not-deployed). ast(feature_version 3.8) 로 읽고, 형 주석마다 그 두 모양을 찾는다."""
    import ast
    src = (G1 / "tools/status_shm.py").read_text(encoding="utf-8")
    tree = ast.parse(src, feature_version=(3, 8))
    anns = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            a = node.args
            args = a.posonlyargs + a.args + a.kwonlyargs + [x for x in (a.vararg, a.kwarg) if x]
            anns += [x.annotation for x in args if x.annotation is not None]
            if node.returns is not None:
                anns.append(node.returns)
        elif isinstance(node, ast.AnnAssign):
            anns.append(node.annotation)
    assert anns, "형 주석을 하나도 못 찾았다 — 검사가 움직이나(read 에 -> Optional[dict] 가 있다)"
    for ann in anns:
        for sub in ast.walk(ann):
            assert not (isinstance(sub, ast.BinOp) and isinstance(sub.op, ast.BitOr)), \
                f"3.10+ 합집합 형 표기(X | Y): {ast.dump(ann)}"
            assert not (isinstance(sub, ast.Subscript) and isinstance(sub.value, ast.Name)
                        and sub.value.id in ("list", "dict", "tuple", "set", "frozenset", "type")), \
                f"3.9+ 형 표기(list[...] 등): {ast.dump(ann)}"


def test_cpp_writer_python_reader():
    """C++ MaskedStatusWriter 가 쓴 칸을 status_shm.read 가 그대로 읽는다(바이트 배치·magic·seqlock 짝수 끝 상태)."""
    import status_shm
    gxx = shutil.which("g++")
    if not gxx:
        print("  skip test_cpp_writer_python_reader (g++ 없음)")
        return
    d = tempfile.mkdtemp()
    try:
        prog = os.path.join(d, "w.cpp")
        Path(prog).write_text(r'''
#include "MaskedStatus.h"
int main(int, char** argv) {
    g1::masked_status::MaskedStatusWriter w;
    if (!w.open(argv[1])) return 2;
    MaskedStatus s{};
    s.tick = 77; s.active_mode = 6; s.last_req_mode = 6; s.last_req_result = 2; s.refusal_code = 4; s.refused_mode = 6;
    s.m5_preset = 4; s.m5_arrived = 1; s.m5_arrive_seq = 3; s.safety = 2; s.fsm_state = 6; s.gui_seq_applied = 4000000000u;
    s.relay_stale = 1; s.z_fk = 0.4375f; s.tilt_deg = 67.5f; s.vx_cmd = 0.25f; s.vy_cmd = -0.125f; s.wz_cmd = 0.5f;
    s.magic = 0; s.seq = 999;      // 쓰는 쪽이 magic·seq 를 스스로 채운다(넘긴 값 무시)
    w.write(s); w.write(s);
    return 0;
}
''', encoding="utf-8")
        exe = os.path.join(d, "w")
        subprocess.run([gxx, "-std=gnu++17", "-O2", "-I", str(G1 / "include"), prog, "-o", exe], check=True)
        p = os.path.join(d, "st")
        subprocess.run([exe, p], check=True)
        r = status_shm.read(p)
        want = dict(seq=6, tick=77, active_mode=6, last_req_mode=6, last_req_result=2, refusal_code=4, refused_mode=6,
                    m5_preset=4, m5_arrived=1, m5_arrive_seq=3, safety=2, fsm_state=6, gui_seq_applied=4000000000,
                    relay_stale=1, z_fk=0.4375, tilt_deg=67.5, vx_cmd=0.25, vy_cmd=-0.125, wz_cmd=0.5)
        assert r == want, r                       # seq 6 = open 의 첫 줄(2) + 두 번 쓰기(4, 6)
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn(); print(f"  ok   {name}")
            except Exception as e:                      # noqa: BLE001 — struct.error·import 실패도 FAIL 한 줄로
                fails += 1; print(f"  FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if fails else 0)
