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
        code = line.split("//")[0]
        if not code.strip():
            continue
        mm = re.match(r"\s*(int32_t|uint32_t|float)\s+([\w\s,]+);\s*$", code)
        # 모르는 타입(int16_t·double·배열…)을 조용히 건너뛰면 FMT 가 «맞아 보이는» 채로 C++ 만 칸이 밀린다.
        assert mm, f"struct GuiCtrl 의 이 줄을 해석 못 한다 — 테스트 파서를 넓히거나 타입을 고칠 것: {line.strip()!r}"
        out += _CH[mm.group(1)] * len([n for n in mm.group(2).split(",") if n.strip()])
    return "<" + out


def test_layout_matches():
    import gui_shm
    assert gui_shm.FMT == _cpp_fmt(), f"python {gui_shm.FMT} != C++ {_cpp_fmt()}"
    assert struct.calcsize(gui_shm.FMT) == 12 * 4


def test_magic_matches_and_is_new():
    import gui_shm
    m = re.search(r"GUI_CTRL_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", _cpp_src())
    assert m and int(m.group(1), 16) == gui_shm.MAGIC == 0x6704
    assert gui_shm.MAGIC != 0x6701, "v1 magic 을 다시 쓰면 옛 GUI 가 새 제어기에 붙는다"


def test_magic_differs_from_other_channels():
    """채널마다 magic 이 다르다 — 경로가 한 번 엇갈려도 남의 파일을 제 것으로 읽지 않게.
    (E-stop 리더는 «≥12 B + 자기 magic» 이면 받는다 — GUI 가 같은 magic 이면 mode_req 를 E-stop flag 로 읽는다.)"""
    import gui_shm
    estop_h = (G1.parent.parent / "include/FSM/EstopChannel.h").read_text(encoding="utf-8")
    m = re.search(r"ESTOP_MAGIC\s*=\s*0x([0-9A-Fa-f]+)", estop_h)
    assert m, "deploy/include/FSM/EstopChannel.h 에서 ESTOP_MAGIC 을 못 찾았다"
    estop = int(m.group(1), 16)
    vr_py = (G1 / "teleop/vr_shm.py").read_text(encoding="utf-8")
    mv = re.search(r"^MAGIC\s*=\s*0x([0-9A-Fa-f]+)", vr_py, re.M)
    assert mv, "teleop/vr_shm.py 에서 MAGIC 을 못 찾았다"
    vr = int(mv.group(1), 16)
    assert gui_shm.MAGIC != estop, f"GUI magic {gui_shm.MAGIC:#x} == E-stop magic {estop:#x}"
    assert gui_shm.MAGIC != vr and gui_shm.MAGIC != 0x6702, f"GUI magic {gui_shm.MAGIC:#x} == VR magic"


def test_write_is_one_shot(tmp_path=None):
    """mode_req·clip_req·m5_preset 은 한 번 보내면 되돌아간다 — 다음 쓰기(속도 변경)가 모드·클립·자세를 다시 싣지 않는다."""
    import gui_shm
    # 진짜 /dev/shm/g1_masked_gui 는 절대 건드리지 않는다(돌고 있는 제어기가 읽는 파일이다).
    own = tmp_path is None
    d = Path(tempfile.mkdtemp(prefix="g1_gui_shm_test_")) if own else Path(tmp_path)
    real = gui_shm.SHM_PATH
    try:
        gui_shm.SHM_PATH = str(d / "g1_masked_gui_test")
        sent = []
        st = dict(seq=0, vx=0.0, vy=0.0, wz=0.0, period_steps=43, height_scale=1.0, turn_k=0.3,
                  mode_req=5, clip_req=2, m5_preset=7, m5_press_seq=3)
        gui_shm.write(st)
        sent.append(struct.unpack(gui_shm.FMT, Path(gui_shm.SHM_PATH).read_bytes()))
        gui_shm.write(st)
        sent.append(struct.unpack(gui_shm.FMT, Path(gui_shm.SHM_PATH).read_bytes()))
        assert sent[0][2] == 5 and sent[1][2] == 0, "mode_req 는 1회성"
        assert sent[0][11] == 2 and sent[1][11] == -1, "clip_req 는 1회성"
        assert sent[0][9] == 7 and sent[1][9] == 0, "m5_preset 은 1회성 (자세 없는 다음 쓰기가 누름을 다시 싣지 않는다)"
        assert sent[0][10] == sent[1][10] == 3, "m5_press_seq 는 그대로 (누를 때만 GUI 가 올린다)"
    finally:
        gui_shm.SHM_PATH = real
        if own:
            shutil.rmtree(d, ignore_errors=True)


def test_write_wraps_uint32_counters():
    """seq·m5_press_seq 는 uint32 칸 — GUI 가 시각으로 시작하는 번호(ms)도, 감긴 번호도 예외 없이 나가야 한다."""
    import gui_shm
    d = Path(tempfile.mkdtemp(prefix="g1_gui_shm_test_"))
    real = gui_shm.SHM_PATH
    try:
        gui_shm.SHM_PATH = str(d / "g1_masked_gui_test")
        st = dict(seq=0xFFFFFFFF, vx=0.0, vy=0.0, wz=0.0, period_steps=43, height_scale=1.0, turn_k=0.3,
                  m5_preset=1, m5_press_seq=(1 << 32) + 5)
        gui_shm.write(st)                                    # struct 'I' 는 넘치면 struct.error
        r = struct.unpack(gui_shm.FMT, Path(gui_shm.SHM_PATH).read_bytes())
        assert r[1] == 0 and st["seq"] == 0, f"seq 가 32 bit 에서 감기지 않았다: {r[1]}"
        assert r[10] == 5, f"m5_press_seq 가 32 bit 로 잘리지 않았다: {r[10]}"
        st["m5_press_seq"] = int(1790057409840) & 0xFFFFFFFF   # masked_gui 의 시각 시작값과 같은 꼴
        gui_shm.write(st)
    finally:
        gui_shm.SHM_PATH = real
        shutil.rmtree(d, ignore_errors=True)


def test_replay_requests_mode_on_change():
    """replay_cmd(세 번째 쓰는 쪽)는 첫 표본과 모드가 바뀔 때 mode_req 를 싣는다 — v2 엔 cmd_mode 칸이 없어서
    안 실으면 재생본의 모드 전환이 조용히 전부 사라진다. 바뀐 뒤 잠깐(MODE_REQ_HOLD_S)은 재전송, 그 뒤엔 없음."""
    import replay_cmd
    H = replay_cmd.MODE_REQ_HOLD_S
    rows = [(0.00, 1, 0.0, 0.0, 0.0),        # 첫 표본 → 요청
            (0.02, 1, 0.0, 0.0, 0.0),        # 변화 없음 → 안 보냄
            (1.00, 1, 0.5, 0.0, 0.0),        # 속도만 → 보내되 요청 없음
            (2.00, 2, 0.5, 0.0, 0.0),        # 모드 변경 → 요청
            (2.00 + H / 2, 2, 0.6, 0.0, 0.0),  # 창 안의 다음 쓰기 → 재전송
            (2.00 + 2 * H, 2, 0.8, 0.0, 0.0),  # 창 밖 → 요청 없음
            (3.00, 2, 0.8, 0.0, 0.0)]        # 변화 없음 → 안 보냄
    lv = t_mode = None
    out = []
    for t, mode, x, y, w in rows:
        upd, lv, t_mode = replay_cmd.plan_write(lv, t, mode, x, y, w, 0.02, t_mode)
        out.append(None if upd is None else upd.get("mode_req", 0))
    assert out == [1, None, 0, 2, 2, 0, None], out


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn(); print(f"  ok   {name}")
            except Exception as e:                      # noqa: BLE001 — struct.error·import 실패도 FAIL 한 줄로
                fails += 1; print(f"  FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if fails else 0)
