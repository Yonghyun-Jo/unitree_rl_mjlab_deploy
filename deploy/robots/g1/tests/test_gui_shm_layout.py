#!/usr/bin/env python3
"""/dev/shm/g1_masked_gui 레이아웃이 C++ `struct GuiCtrl` 과 python `gui_shm.FMT` 사이에서 안 갈렸는지.
한쪽만 필드를 더하면 컴파일도 실행도 되는데 C++ 이 엉뚱한 칸을 읽는다(모드·자세 버튼이 쓰레기).
실행:  python3 deploy/robots/g1/tests/test_gui_shm_layout.py
"""
from __future__ import annotations

import re
import shutil
import struct
import sys
import tempfile
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(G1 / "tools"))
_CH = {"int32_t": "i", "uint32_t": "I", "float": "f"}


def _cpp_src() -> str:
    return (G1 / "src/State_Mimic.cpp").read_text(encoding="utf-8")


def _cpp_fmt() -> str:
    m = re.search(r"struct GuiCtrl\s*\{(.*?)\n\};", _cpp_src(), re.S)
    assert m, "State_Mimic.cpp 에서 struct GuiCtrl 을 못 찾았다"
    out = ""
    for line in m.group(1).splitlines():
        mm = re.match(r"\s*(int32_t|uint32_t|float)\s+([\w\s,]+);", line.split("//")[0])
        if mm:
            out += _CH[mm.group(1)] * len([n for n in mm.group(2).split(",") if n.strip()])
    return "<" + out


def test_layout_matches():
    import gui_shm
    assert gui_shm.FMT == _cpp_fmt(), f"python {gui_shm.FMT} != C++ {_cpp_fmt()}"
    assert struct.calcsize(gui_shm.FMT) == 12 * 4


def test_magic_matches_and_is_new():
    import gui_shm
    m = re.search(r"GUI_CTRL_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", _cpp_src())
    assert m and int(m.group(1), 16) == gui_shm.MAGIC == 0x6703
    assert gui_shm.MAGIC != 0x6701, "v1 magic 을 다시 쓰면 옛 GUI 가 새 제어기에 붙는다"


def test_write_is_one_shot(tmp_path=None):
    """mode_req·clip_req 는 한 번 보내면 되돌아간다 — 다음 쓰기(속도 변경)가 모드를 다시 요청하지 않는다."""
    import gui_shm
    # 진짜 /dev/shm/g1_masked_gui 는 절대 건드리지 않는다(돌고 있는 제어기가 읽는 파일이다).
    own = tmp_path is None
    d = Path(tempfile.mkdtemp(prefix="g1_gui_shm_test_")) if own else Path(tmp_path)
    real = gui_shm.SHM_PATH
    try:
        gui_shm.SHM_PATH = str(d / "g1_masked_gui_test")
        sent = []
        st = dict(seq=0, vx=0.0, vy=0.0, wz=0.0, period_steps=43, height_scale=1.0, turn_k=0.3,
                  mode_req=5, clip_req=2)
        gui_shm.write(st)
        sent.append(struct.unpack(gui_shm.FMT, Path(gui_shm.SHM_PATH).read_bytes()))
        gui_shm.write(st)
        sent.append(struct.unpack(gui_shm.FMT, Path(gui_shm.SHM_PATH).read_bytes()))
        assert sent[0][2] == 5 and sent[1][2] == 0, "mode_req 는 1회성"
        assert sent[0][11] == 2 and sent[1][11] == -1, "clip_req 는 1회성"
    finally:
        gui_shm.SHM_PATH = real
        if own:
            shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn(); print(f"  ok   {name}")
            except AssertionError as e:
                fails += 1; print(f"  FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
