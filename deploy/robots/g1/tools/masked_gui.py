#!/usr/bin/env python3
"""Browser control GUI for the masked-locomotion deploy (mjlab-style, via viser).

Mirrors the mjlab play GUI (modes from the mode table + mode5 postures + clip select +
base_vel + foot-trajectory generator) AND the C++ keyboard teleop (WASD/QE/space) — both via
browser widgets and browser keyboard hotkeys.
Instead of driving a sim it writes a small packed struct to shared memory
(/dev/shm/g1_masked_gui); the C++ controller (State_Mimic.cpp g_poll_gui) reads it each step
and overrides mode / base_vel / foot-gen params. Run it ALONGSIDE the running g1_ctrl, ON
THE SAME MACHINE (shared /dev/shm) — sim2sim OR real robot (tethered control PC):

    ~/.local/bin/uv run --with viser python deploy/robots/g1/tools/masked_gui.py
    # open the printed viser URL (http://localhost:8080). The browser can be a remote
    # laptop pointing at <controlPC>:8080; only THIS python must share the controller's host.

If the GUI is not running, keyboard (terminal) + joystick still drive the robot.

⚠ Struct layout MUST match State_Mimic.cpp `struct GuiCtrl` (#pragma pack(1)).
⚠ Deploy velocity caps (vx +2.5/-1.5, vy 0.8, wz 2.0) still clamp base_vel in C++.
"""
from __future__ import annotations

import os
import re
import time

import viser

import gui_shm  # shared /dev/shm struct contract (also used by pico_control_bridge.py)
from mode5_presets_gen import PRESETS   # 생성 파일 — mode5 자세 버튼 표 (index, name, key, n)
from mode_table_gen import MODES        # 생성 파일 — 모드 표 (id, name, key, safety). 번호를 여기 박지 않는다

KB_STEP = 0.1
BTNS_PER_ROW = 3           # 버튼 그룹 한 줄에 몇 개 (viser 는 한 줄로만 펴서 폭을 넘으면 잘린다)
VXCAP, VXCAP_BWD = gui_shm.VXCAP, gui_shm.VXCAP_BWD   # deploy caps (match C++). vx 는 비대칭
VYCAP, WCAP = gui_shm.VYCAP, gui_shm.WCAP

state = dict(seq=0, cmd_mode=1, vx=0.0, vy=0.0, wz=0.0,
             period_steps=43, height_scale=1.0, turn_k=0.3,
             mode_req=0, m5_preset=0, clip_req=-1,   # 1회성 요청은 gui_shm.write 가 되돌린다
             # 자세 누름 번호는 시각으로 시작한다 — GUI 를 다시 띄워도 제어기가 마지막으로 본 번호와 겹쳐
             # 첫 누름이 무시되지 않게(0 부터면 겹칠 수 있다). 구조체 칸이 uint32 라 32 bit 로 자른다.
             m5_press_seq=int(time.time() * 1000) & 0xFFFFFFFF)

# 클립 칸 = State_Mimic.cpp g_build_clips 의 이름·순서(primary · light · demo6 중 «설정에 실린 것만»).
# 원천 = config/config.yaml 의 Mimic_Masked 블록 motion_file · motion_file_light · motion_file_demo6
# (G1_POLICY_SLOT 은 policy_dir 만 바꾸고 클립 줄은 그대로 읽는다 — 제어기와 같은 원천이다).
# light 가 없는 설정이면 demo6 이 1 번이 된다. 읽지 못하면 옛 고정 표로 — 제어기가 고른 칸 이름을 로그로 답한다
# («[clip] i/n «이름»»).
_CLIP_KEYS = (("motion_file", "primary"), ("motion_file_light", "light"), ("motion_file_demo6", "demo6"))
# 0번 칸은 파일이 아니라 제어기가 짓는 합성 «stand»(기본 자세 유지) — g_build_clips 가 맨 앞에 넣는다.
# 🔴 순서·개수가 제어기와 어긋나면 엉뚱한 클립이 재생된다(제어기는 번호만 받는다).
_STAND_LABEL = "0 stand (정지)"
_CLIPS_FALLBACK = (_STAND_LABEL, "1 primary", "2 light", "3 demo6")


def _short_clip(stem: str) -> str:
    """클립 파일명을 버튼에 들어갈 짧은 이름으로: g1_fight1_subject1_colmov2 → fight1s1.
    «g1_» 접두 · «_subject<N>» → «s<N>» · 꼬리의 리타겟/트림 표시(colmov2 · upper16s · stand10s)를 뗀다.
    떼고 나서 두 클립이 같은 이름이 되면 그 둘은 원래 이름을 쓴다(_clip_labels 가 본다) — 버튼이 짧아도
    «어느 클립인지» 가 흐려지면 안 된다."""
    s = re.sub(r"^g1_", "", stem)
    s = re.sub(r"_subject(\d+)", r"s\1", s)
    tail = re.compile(r"^(colmov?\d*|[a-z]*\d+s)$")          # colmo·colmov2 · upper16s·stand10s (트림 길이 표시)
    parts = [p for p in s.split("_") if p]
    while len(parts) > 1 and tail.match(parts[-1]):
        parts.pop()
    return "_".join(parts) or stem


def _clip_labels(config_yaml: str) -> tuple:
    """config.yaml 의 Mimic_Masked 상태 블록(들여쓰기 2)에서 클립 줄을 찾아 "i 짧은이름" 라벨을 만든다.
    PyYAML 없이 줄 단위로 읽는다(이 GUI 는 viser 만 얹은 uv 환경에서 돈다).
    반환 = (버튼 라벨들, "i = 원래 파일명" 한 줄) — 버튼은 짧게, 원래 이름은 버튼 밑 한 줄에 남긴다."""
    try:
        lines = open(config_yaml, encoding="utf-8").read().splitlines()
    except OSError:
        return _CLIPS_FALLBACK, ""
    found, inside = {}, False
    for ln in lines:
        if re.match(r"^  Mimic_Masked:", ln):
            inside = True
            continue
        if inside and ln.strip() and not ln.startswith("   "):     # 다음 상태 블록(들여쓰기 ≤ 2)
            break
        m = re.match(r"^    (motion_file(?:_light|_demo6)?):\s*(\S+)", ln) if inside else None
        if m:
            found[m.group(1)] = os.path.splitext(os.path.basename(m.group(2)))[0]
    stems = [found[k] for k, _n in _CLIP_KEYS if k in found]
    if not stems:
        return _CLIPS_FALLBACK, ""
    short = [_short_clip(s) for s in stems]
    for i, s in enumerate(short):                              # 짧게 하다 겹치면 그 칸은 원래 이름으로
        if short.count(s) > 1:
            short[i] = stems[i]
    labels = (_STAND_LABEL,) + tuple(f"{i + 1} {s}" for i, s in enumerate(short))
    detail = "0 = 기본 자세 유지(제어기가 만든다) · " + " · ".join(
        f"{i + 1} = {stem}" for i, stem in enumerate(stems))
    return labels, detail


CLIPS, CLIPS_DETAIL = _clip_labels(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "config.yaml"))


def _clamp(x, lo, hi):
    return gui_shm.clamp(x, lo, hi)


def _write() -> None:
    gui_shm.write(state)


def main() -> None:
    srv = viser.ViserServer()
    g = srv.gui
    _suppress = [False]   # guard: programmatic slider updates (set_vel) must not re-write

    status = g.add_markdown("")

    def refresh() -> None:
        names = {i: name for i, name, _key, _s in MODES}
        status.content = (f"**mode {state['cmd_mode']}** ({names[state['cmd_mode']]})  ·  "
                          f"base_vel [{state['vx']:.2f}, {state['vy']:.2f}, {state['wz']:.2f}]")

    # 🔴 viser 의 button_group 은 한 줄로만 편다 — 패널 폭을 넘으면 뒤쪽 버튼이 «잘린 채로» 안 보인다
    #    (2026-09-23: 자세 «드러누움», 클립 뒷칸. 스크롤 막대도 안 나와서 없는 버튼처럼 보인다).
    #    그래서 per_row 개씩 끊어 여러 그룹으로 낸다 — 항목이 늘어도 줄이 저절로 는다.
    #    이름표는 첫 줄만 단다(viser 가 이름표를 왼쪽 칸에 두므로, 빈 이름표 = 버튼이 그만큼 넓어진다).
    def button_rows(label: str, options, on_click, per_row: int = BTNS_PER_ROW) -> None:
        for r in range(0, len(options), per_row):
            g.add_button_group(label if r == 0 else "", options[r:r + per_row]).on_click(on_click)

    # ---- sliders (also updated by the hotkeys below) ----
    with g.add_folder("Mode"):
        mode_btns = g.add_button_group("cmd_mode", tuple(f"{i}: {name}" for i, name, _key, _s in MODES))

        @mode_btns.on_click
        def _(ev) -> None:
            set_mode(int(ev.target.value.split(":")[0]))

    with g.add_folder("mode5 posture"):
        g.add_markdown("mode5 에서만 — 다른 모드에선 제어기가 거부한다")
        g.add_markdown("키가 있는 자세만 내놓는다(config/mode5_keys.yaml). 표의 나머지 자세는 검증 뒤에 연다")
        # 🔴 키 없는 표 자세(베어 크롤·한쪽 낮춘 지지 L/R)는 운용자에게 안 내놓는다 — 검증 전(spec §4.5 = 6).
        #    m5_preset 은 그대로 «표 index + 1» — 거른다고 번호를 당기지 않는다.
        preset_label = {f"{name} ({key}) n={n}": index
                        for index, name, key, n in PRESETS if key}

        def _on_preset(ev) -> None:
            state["m5_preset"] = preset_label[ev.target.value] + 1   # shm: 0 = 없음, 1.. = PRESETS[index] + 1
            state["m5_press_seq"] = (state.get("m5_press_seq", 0) + 1) & 0xFFFFFFFF  # 같은 자세 재입력 = 새 목표
            _write(); refresh()

        button_rows("m5_preset", tuple(preset_label), _on_preset)   # 자세 9개까지 늘어도 줄이 는다

    with g.add_folder("Clip (mode4 playback)"):
        g.add_markdown("mode4 는 «0 stand»(정지 자세)로 들어간다 — 클립은 들어간 뒤에 고른다. "
                       "재생 중 교체는 직립일 때만(아니면 거부 한 줄), «0 stand» 로 돌아오는 것은 언제나 된다")

        def _on_clip(ev) -> None:
            state["clip_req"] = int(ev.target.value.split(" ")[0])
            _write(); refresh()

        # 버튼은 짧은 이름(g1_fight1_subject1_colmov2 → «1 fight1s1»), 원래 파일명은 밑에 한 줄로 남긴다.
        button_rows("Select clip", CLIPS, _on_clip)
        if CLIPS_DETAIL:
            g.add_markdown(CLIPS_DETAIL)

    with g.add_folder("base_vel (yaw-local)"):
        vx = g.add_slider("vx", -VXCAP_BWD, VXCAP, 0.01, 0.0)   # 전진 2.5 / 후진 1.5
        vy = g.add_slider("vy", -VYCAP, VYCAP, 0.01, 0.0)
        wz = g.add_slider("wz (turn)", -WCAP, WCAP, 0.01, 0.0)

        def _v(_=None) -> None:
            if _suppress[0]:   # programmatic slider update from set_vel -> don't re-write
                return
            state["vx"], state["vy"], state["wz"] = vx.value, vy.value, wz.value
            _write(); refresh()
        for w in (vx, vy, wz):
            w.on_update(_v)

        zero = g.add_button("base_vel = 0  (space)")

        @zero.on_click
        def _(_) -> None:
            set_vel(0.0, 0.0, 0.0)

    with g.add_folder("Foot-Z Generator"):
        period = g.add_slider("stride / step period", 20, 120, 1, 43)
        hscale = g.add_slider("foot height (×auto)", 0.5, 3.0, 0.05, 1.0)
        turnk = g.add_slider("turn_k (|wz|→step)", 0.0, 0.8, 0.05, 0.3)

        apply_btn = g.add_button("Apply foot-gen")

        @apply_btn.on_click
        def _(_) -> None:
            state["period_steps"] = int(period.value)
            state["height_scale"] = float(hscale.value)
            state["turn_k"] = float(turnk.value)
            _write(); refresh()

        reset_btn = g.add_button("Reset to auto (data default)")

        @reset_btn.on_click
        def _(_) -> None:
            period.value, hscale.value, turnk.value = 43, 1.0, 0.3
            state["period_steps"], state["height_scale"], state["turn_k"] = 43, 1.0, 0.3
            _write(); refresh()

    g.add_markdown(
        "**Keyboard (focus the browser):** W/S = vx±, A/D = vy±, Q/E = wz±, "
        "Space = stop · " + "/".join(key for _i, _n, key, _s in MODES) + " = mode (표의 키)")

    # ---- shared setters (sliders + hotkeys both call these) ----
    def set_vel(nx, ny, nw) -> None:
        nx = _clamp(nx, -VXCAP_BWD, VXCAP); ny = _clamp(ny, -VYCAP, VYCAP); nw = _clamp(nw, -WCAP, WCAP)
        state["vx"], state["vy"], state["wz"] = nx, ny, nw
        _suppress[0] = True                         # slider .value sets below won't re-write
        vx.value, vy.value, wz.value = nx, ny, nw   # reflect in sliders (visual)
        _suppress[0] = False
        _write(); refresh()                         # single authoritative publish

    def set_mode(m) -> None:
        state["cmd_mode"] = m     # 이 GUI 의 표시용 (제어기가 거부했을 수도 있다 — 터미널 로그가 답한다)
        state["mode_req"] = m     # 1회성 요청 — gui_shm.write 가 보낸 뒤 0 으로 되돌린다
        _write(); refresh()

    # ---- browser keyboard hotkeys (mirror the C++ keyboard teleop) ----
    def cmd(label, key, fn):
        h = g.add_command(label, hotkey=key)
        h.on_trigger(lambda _=None: fn())

    cmd("forward (vx+)",  "W", lambda: set_vel(state["vx"] + KB_STEP, state["vy"], state["wz"]))
    cmd("back (vx-)",     "S", lambda: set_vel(state["vx"] - KB_STEP, state["vy"], state["wz"]))
    cmd("strafe L (vy+)", "A", lambda: set_vel(state["vx"], state["vy"] + KB_STEP, state["wz"]))
    cmd("strafe R (vy-)", "D", lambda: set_vel(state["vx"], state["vy"] - KB_STEP, state["wz"]))
    cmd("yaw CCW (wz+)",  "Q", lambda: set_vel(state["vx"], state["vy"], state["wz"] + KB_STEP))
    cmd("yaw CW (wz-)",   "E", lambda: set_vel(state["vx"], state["vy"], state["wz"] - KB_STEP))
    cmd("stop",           "space", lambda: set_vel(0.0, 0.0, 0.0))
    for i, name, key, _s in MODES:                 # 모드 단축키 = 표의 key 열 (번호를 여기 박지 않는다)
        cmd(f"mode {i} ({name})", key, lambda i=i: set_mode(i))

    _write(); refresh()
    print(f"[masked_gui] writing {gui_shm.SHM_PATH}; open the viser URL above. Ctrl-C to quit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
