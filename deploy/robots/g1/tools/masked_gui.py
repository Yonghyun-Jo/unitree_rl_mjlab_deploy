#!/usr/bin/env python3
"""Browser control GUI for the masked-locomotion deploy (mjlab-style, via viser).

Mirrors the mjlab play GUI (mode 1/2/3 + base_vel + foot-trajectory generator) AND the C++
keyboard teleop (WASD/QE/space) — both via browser widgets and browser keyboard hotkeys.
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

import time

import viser

import gui_shm  # shared /dev/shm struct contract (also used by pico_control_bridge.py)

KB_STEP = 0.1
VXCAP, VXCAP_BWD = gui_shm.VXCAP, gui_shm.VXCAP_BWD   # deploy caps (match C++). vx 는 비대칭
VYCAP, WCAP = gui_shm.VYCAP, gui_shm.WCAP

state = dict(seq=0, cmd_mode=1, vx=0.0, vy=0.0, wz=0.0,
             period_steps=43, height_scale=1.0, turn_k=0.3)


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
        names = {1: "full-auto", 2: "upper-teleop", 3: "full-teleop"}
        status.content = (f"**mode {state['cmd_mode']}** ({names[state['cmd_mode']]})  ·  "
                          f"base_vel [{state['vx']:.2f}, {state['vy']:.2f}, {state['wz']:.2f}]")

    # ---- sliders (also updated by the hotkeys below) ----
    with g.add_folder("Mode"):
        mode_btns = g.add_button_group("cmd_mode", ("1: full-auto", "2: upper-teleop", "3: full-teleop"))

        @mode_btns.on_click
        def _(ev) -> None:
            set_mode(int(ev.target.value[0]))

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
        "Space = stop · 1/2/3 = mode")

    # ---- shared setters (sliders + hotkeys both call these) ----
    def set_vel(nx, ny, nw) -> None:
        nx = _clamp(nx, -VXCAP_BWD, VXCAP); ny = _clamp(ny, -VYCAP, VYCAP); nw = _clamp(nw, -WCAP, WCAP)
        state["vx"], state["vy"], state["wz"] = nx, ny, nw
        _suppress[0] = True                         # slider .value sets below won't re-write
        vx.value, vy.value, wz.value = nx, ny, nw   # reflect in sliders (visual)
        _suppress[0] = False
        _write(); refresh()                         # single authoritative publish

    def set_mode(m) -> None:
        state["cmd_mode"] = m
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
    cmd("mode 1",         "1", lambda: set_mode(1))
    cmd("mode 2",         "2", lambda: set_mode(2))
    cmd("mode 3",         "3", lambda: set_mode(3))

    _write(); refresh()
    print(f"[masked_gui] writing {gui_shm.SHM_PATH}; open the viser URL above. Ctrl-C to quit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
