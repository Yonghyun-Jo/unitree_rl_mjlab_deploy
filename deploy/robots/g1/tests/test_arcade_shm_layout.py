#!/usr/bin/env python3
"""아케이드 의도 칸(0x6707)·아케이드 상태 칸(0x6708)의 레이아웃이 C++ 구조체(include/ArcadeIntent.h `struct ArcadeIntentFrame` ·
include/ArcadeStatus.h `struct ArcadeStatus`)와 python FMT·FIELDS(tools/arcade_intent_shm.py · tools/arcade_status_shm.py) 사이에서
안 갈렸는지 + magic 이 다른 채널과 겹치지 않는지 + 크기 + 읽기·쓰기 왕복 + 두 python 파일이 3.8 문법인지.
실행:  python3 deploy/robots/g1/tests/test_arcade_shm_layout.py   (pytest 로도)
"""
from __future__ import annotations

import ast
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
# 브리프의 칸 순서 그대로 — 구조체를 고칠 때 이 표도 같이 바꾼다.
_INTENT_BRIEF = [("magic", "i"), ("seq", "I"), ("fwd", "f"), ("side", "f"), ("turn", "f"), ("run", "i"), ("events", "I"),
                 ("event_seq", "I"), ("event_age_ms", "I")]
_STATUS_BRIEF = [("magic", "i"), ("seq", "I"), ("tick", "I"), ("role", "i"), ("skill", "i"), ("mode", "i"), ("preset", "i"),
                 ("route_step", "i"), ("notice", "i"), ("safety_class", "i"), ("busy", "i"), ("qd_warn", "i"), ("qd_crit", "i"),
                 ("intent_stale", "i"), ("rearmed", "i"), ("fsm_state", "i"), ("tick_us", "I"), ("z_fk", "f"),
                 ("tilt_deg", "f"), ("cmd_vx", "f"), ("cmd_vy", "f"), ("cmd_wz", "f")]


def _src(name):
    return (G1 / "include" / name).read_text(encoding="utf-8")


def _cpp_fields(header, struct_name):
    m = re.search(r"struct " + struct_name + r"\s*\{(.*?)\n\};", _src(header), re.S)
    assert m, f"{header} 에서 struct {struct_name} 를 못 찾았다"
    fmt, names = "", []
    for line in m.group(1).splitlines():
        code = line.split("//")[0]
        if not code.strip():
            continue
        mm = re.match(r"\s*(int32_t|uint32_t|float)\s+([\w\s,]+);\s*$", code)
        # 모르는 타입을 조용히 건너뛰면 FMT 가 «맞아 보이는» 채로 C++ 만 칸이 밀린다.
        assert mm, f"struct {struct_name} 의 이 줄을 해석 못 한다 — 타입을 int32_t·uint32_t·float 로: {line.strip()!r}"
        ns = [n.strip() for n in mm.group(2).split(",") if n.strip()]
        fmt += _CH[mm.group(1)] * len(ns)
        names += ns
    return "<" + fmt, names


def test_intent_layout_matches():
    import arcade_intent_shm as m
    fmt, names = _cpp_fields("ArcadeIntent.h", "ArcadeIntentFrame")
    assert m.FMT == fmt, f"python {m.FMT} != C++ {fmt}"
    assert list(m.FIELDS) == names, f"칸 이름 순서가 다르다: {m.FIELDS} != {names}"
    assert struct.calcsize(m.FMT) == m.SIZE == 36
    assert names == [n for n, _ in _INTENT_BRIEF] and fmt == "<" + "".join(t for _, t in _INTENT_BRIEF)


def test_status_layout_matches():
    import arcade_status_shm as m
    fmt, names = _cpp_fields("ArcadeStatus.h", "ArcadeStatus")
    assert m.FMT == fmt, f"python {m.FMT} != C++ {fmt}"
    assert list(m.FIELDS) == names, f"칸 이름 순서가 다르다: {m.FIELDS} != {names}"
    assert struct.calcsize(m.FMT) == m.SIZE == 88
    assert names == [n for n, _ in _STATUS_BRIEF] and fmt == "<" + "".join(t for _, t in _STATUS_BRIEF)


def test_magics_match_and_differ():
    """0x6707(의도)·0x6708(상태) — C++ 상수 = python 상수 · 기존 채널 0x6701~0x6706 과 · 서로 겹치지 않는다."""
    import arcade_intent_shm as i
    import arcade_status_shm as s
    import gui_shm
    import status_shm
    mi = re.search(r"ARCADE_INTENT_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", _src("ArcadeIntent.h"))
    ms = re.search(r"ARCADE_STATUS_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", _src("ArcadeStatus.h"))
    assert mi and int(mi.group(1), 16) == i.MAGIC == 0x6707
    assert ms and int(ms.group(1), 16) == s.MAGIC == 0x6708
    old = {0x6701, 0x6702, 0x6703, 0x6704, 0x6705, 0x6706}
    assert not ({i.MAGIC, s.MAGIC} & old) and i.MAGIC != s.MAGIC
    assert not ({i.MAGIC, s.MAGIC} & {gui_shm.MAGIC, gui_shm.MAGIC_GUI_RELAY, status_shm.MAGIC})
    estop_h = (G1.parent.parent / "include/FSM/EstopChannel.h").read_text(encoding="utf-8")
    estop = int(re.search(r"ESTOP_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", estop_h).group(1), 16)
    vr_py = (G1 / "teleop/vr_shm.py").read_text(encoding="utf-8")
    vr = int(re.search(r"^MAGIC\s*=\s*0x([0-9A-Fa-f]+)", vr_py, re.M).group(1), 16)
    assert i.MAGIC not in (estop, vr) and s.MAGIC not in (estop, vr)
    assert re.search(r'ARCADE_INTENT_PATH\s*=\s*"([^"]+)"', _src("ArcadeIntent.h")).group(1) == i.SHM_PATH == "/dev/shm/g1_arcade_intent"
    assert re.search(r'DEFAULT_PATH\s*=\s*"([^"]+)"', _src("ArcadeStatus.h")).group(1) == s.SHM_PATH == "/dev/shm/g1_arcade_status"


def test_event_bits_match_cpp():
    import arcade_intent_shm as i
    src = _src("ArcadeIntent.h")
    for name, val in (("JUMP", i.EV_JUMP), ("CRAWL", i.EV_CRAWL), ("GETUP", i.EV_GETUP), ("LIE", i.EV_LIE), ("BOX", i.EV_BOX)):
        assert int(re.search(r"INTENT_EV_" + name + r"\s*=\s*(\d+)", src).group(1)) == val, name
    assert [i.EV_JUMP, i.EV_CRAWL, i.EV_GETUP, i.EV_LIE, i.EV_BOX] == [1, 2, 4, 8, 16]


def test_intent_write_read_roundtrip():
    import arcade_intent_shm as m
    d = tempfile.mkdtemp(prefix="g1_arcade_intent_test_")
    try:
        p = os.path.join(d, "intent")
        assert m.read(p) is None                                          # 없음
        m._state["seq"] = m._state["event_seq"] = 0
        m.write_intent(0.5, -0.25, 2.0, 1, 0, 0, path=p)                  # 2.0 → clamp 1.0
        a = m.read(p)
        assert a["fwd"] == 0.5 and a["side"] == -0.25 and a["turn"] == 1.0 and a["run"] == 1 and a["events"] == 0
        assert "magic" not in a and list(a) == list(m.FIELDS[1:])
        m.write_intent(0.5, -0.25, 1.0, 1, 0, 0, path=p)
        b = m.read(p)
        assert b["seq"] == (a["seq"] + 1) & 0xFFFFFFFF and b["event_seq"] == a["event_seq"], "값이 같아도 seq 는 +1 · 이벤트 없으면 event_seq 그대로"
        m.write_intent(0, 0, 0, 0, m.EV_JUMP | m.EV_BOX, 30, path=p)
        c = m.read(p)
        assert c["events"] == 17 and c["event_age_ms"] == 30 and c["event_seq"] == (a["event_seq"] + 1) & 0xFFFFFFFF
        assert c["seq"] == (b["seq"] + 1) & 0xFFFFFFFF
        assert not os.path.exists(p + ".tmp"), "os.replace 로 교체하면 .tmp 가 남지 않는다"
        m._state["seq"] = 0xFFFFFFFF                                      # uint32 감김
        m.write_intent(0, 0, 0, path=p)
        assert m.read(p)["seq"] == 0
        m.write_intent(float("nan"), 0, 0, path=p)
        assert m.read(p)["fwd"] == 0.0
        Path(p).write_bytes(b"\0" * 35); assert m.read(p) is None         # 짧음
        Path(p).write_bytes(b"\0" * 37); assert m.read(p) is None         # 김
        Path(p).write_bytes(struct.pack(m.FMT, 0x6705, 1, 0, 0, 0, 0, 0, 0, 0)); assert m.read(p) is None   # 남의 magic
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _pack_status(**over):
    import arcade_status_shm as m
    v = dict(zip(m.FIELDS, [m.MAGIC, 4, 123] + list(range(1, 14)) + [987, 0.75, 2.5, 0.25, -0.5, 1.0]))
    v.update(over)
    return struct.pack(m.FMT, *[v[k] for k in m.FIELDS]), v


def test_status_read_roundtrip_and_none_cases():
    import arcade_status_shm as m
    d = tempfile.mkdtemp(prefix="g1_arcade_status_test_")
    try:
        p = os.path.join(d, "st")
        assert m.read(p) is None
        buf, v = _pack_status()
        Path(p).write_bytes(buf)
        r = m.read(p)
        assert r is not None and list(r) == list(m.FIELDS[1:]) and "magic" not in r
        for k in m.FIELDS[1:]:
            assert r[k] == v[k], (k, r[k], v[k])
        Path(p).write_bytes(buf[:87]); assert m.read(p) is None
        Path(p).write_bytes(buf + b"\0"); assert m.read(p) is None
        Path(p).write_bytes(_pack_status(magic=0x6706)[0]); assert m.read(p) is None       # 상태 칸 g1_masked_status 의 magic
        Path(p).write_bytes(_pack_status(seq=7)[0]); assert m.read(p) is None              # 쓰는 중
        Path(p).write_bytes(_pack_status(seq=8)[0]); assert m.read(p) is not None
        assert m.MAX_TRIES == 64
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_cpp_writer_python_reader():
    """C++ ArcadeStatusWriter 가 쓴 칸을 arcade_status_shm.read 가 그대로 읽는다(g++ 가 있으면)."""
    import arcade_status_shm as m
    gxx = shutil.which("g++")
    if not gxx:
        print("  skip test_cpp_writer_python_reader (g++ 없음)")
        return
    d = tempfile.mkdtemp(prefix="g1_arcade_status_test_")
    try:
        prog = os.path.join(d, "w.cpp")
        Path(prog).write_text(r'''
#include "ArcadeStatus.h"
int main(int, char** argv) {
    g1::arcade_status::ArcadeStatusWriter w;
    if (!w.open(argv[1])) return 2;
    ArcadeStatus s{};
    s.tick = 77; s.role = 1; s.skill = 2; s.mode = 3; s.preset = 4; s.route_step = 5; s.notice = 6; s.safety_class = 1;
    s.busy = 1; s.qd_warn = 1; s.qd_crit = 0; s.intent_stale = 1; s.rearmed = 1; s.fsm_state = 6; s.tick_us = 4000000000u;
    s.z_fk = 0.4375f; s.tilt_deg = 67.5f; s.cmd_vx = 0.25f; s.cmd_vy = -0.125f; s.cmd_wz = 0.5f;
    s.magic = 0; s.seq = 999;
    w.write(s); w.write(s);
    return 0;
}
''', encoding="utf-8")
        exe = os.path.join(d, "w")
        subprocess.run([gxx, "-std=gnu++17", "-O2", "-I", str(G1 / "include"), prog, "-o", exe], check=True)
        p = os.path.join(d, "st")
        subprocess.run([exe, p], check=True)
        want = dict(seq=6, tick=77, role=1, skill=2, mode=3, preset=4, route_step=5, notice=6, safety_class=1, busy=1,
                    qd_warn=1, qd_crit=0, intent_stale=1, rearmed=1, fsm_state=6, tick_us=4000000000,
                    z_fk=0.4375, tilt_deg=67.5, cmd_vx=0.25, cmd_vy=-0.125, cmd_wz=0.5)
        assert m.read(p) == want, m.read(p)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_cpp_reads_python_intent():
    """arcade_intent_shm.write_intent 가 쓴 파일을 C++ read_intent_file · poll_intent 가 같은 값으로 읽는다(g++ 가 있으면)."""
    import arcade_intent_shm as m
    gxx = shutil.which("g++")
    if not gxx:
        print("  skip test_cpp_reads_python_intent (g++ 없음)")
        return
    d = tempfile.mkdtemp(prefix="g1_arcade_intent_test_")
    try:
        prog = os.path.join(d, "r.cpp")
        Path(prog).write_text(r'''
#include "ArcadeIntent.h"
#include <cstdio>
int main(int, char** argv) {
    ArcadeIntentFrame f{};
    if (!read_intent_file(argv[1], f)) return 2;
    std::printf("%d %u %.4f %.4f %.4f %d %u %u %u\n", f.magic, f.seq, f.fwd, f.side, f.turn, f.run, f.events, f.event_seq, f.event_age_ms);
    return 0;
}
''', encoding="utf-8")
        exe = os.path.join(d, "r")
        subprocess.run([gxx, "-std=gnu++17", "-O2", "-I", str(G1 / "include"), prog, "-o", exe], check=True)
        p = os.path.join(d, "intent")
        m._state["seq"] = m._state["event_seq"] = 0
        m.write_intent(0.5, -0.25, 0.75, 1, m.EV_CRAWL, 40, path=p)
        r = m.read(p)
        out = subprocess.run([exe, p], check=True, capture_output=True, text=True).stdout.split()
        assert out == [str(0x6707), str(r["seq"]), "0.5000", "-0.2500", "0.7500", "1", "2", str(r["event_seq"]), "40"], out
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_py38_runtime_syntax():
    """로봇은 python 3.8 — 3.9+ 런타임 문법(`list[int]` 주석 · `X | None`)은 «문법은 통과하고 실행에서 죽는다».
    두 파일 모두 ast(feature_version 3.8) 로 읽고 형 주석마다 그 두 모양을 찾는다."""
    for name in ("arcade_intent_shm.py", "arcade_status_shm.py"):
        src = (G1 / "tools" / name).read_text(encoding="utf-8")
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
        assert anns, f"{name}: 형 주석을 하나도 못 찾았다 — 검사가 움직이나"
        for ann in anns:
            for sub in ast.walk(ann):
                assert not (isinstance(sub, ast.BinOp) and isinstance(sub.op, ast.BitOr)), \
                    f"{name}: 3.10+ 합집합 형 표기(X | Y): {ast.dump(ann)}"
                assert not (isinstance(sub, ast.Subscript) and isinstance(sub.value, ast.Name)
                            and sub.value.id in ("list", "dict", "tuple", "set", "frozenset", "type")), \
                    f"{name}: 3.9+ 형 표기(list[...] 등): {ast.dump(ann)}"


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn(); print(f"  ok   {name}")
            except Exception as e:                      # noqa: BLE001 — struct.error·import 실패도 FAIL 한 줄로
                fails += 1; print(f"  FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if fails else 0)
