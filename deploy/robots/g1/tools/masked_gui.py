#!/usr/bin/env python3
"""Browser control GUI for the masked-locomotion deploy (mjlab-style, via viser).

Mirrors the mjlab play GUI (mode 1/2/3 + base_vel + foot-trajectory generator), but instead
of driving a sim it writes a small packed struct to shared memory (/dev/shm/g1_masked_gui).
The C++ controller (State_Mimic.cpp g_poll_gui) reads it each step and overrides mode /
base_vel / foot-gen params. Run it ALONGSIDE the running g1_ctrl (sim2sim or real):

    ~/.local/bin/uv run --with viser python deploy/robots/g1/tools/masked_gui.py
    # then open the printed viser URL (http://localhost:8080)

If the GUI is not running, keyboard (WASD/QE, 1/2/3) + joystick still drive the robot.

⚠ The struct layout MUST match State_Mimic.cpp `struct GuiCtrl` (#pragma pack(1)).
⚠ Deploy velocity caps (KB_MAXV=1.0, KB_MAXW=0.6) still clamp base_vel in C++.
"""
from __future__ import annotations

import struct
import os

import viser

SHM_PATH = "/dev/shm/g1_masked_gui"
MAGIC = 0x6701
# <  little-endian, packed.  i I i f f f i f f  = magic seq mode vx vy wz period hscale turnk
FMT = "<iIifffiff"

# current GUI state (full struct written every change)
state = dict(seq=0, cmd_mode=1, vx=0.0, vy=0.0, wz=0.0,
             period_steps=43, height_scale=1.0, turn_k=0.3)


def _write() -> None:
    state["seq"] += 1
    buf = struct.pack(FMT, MAGIC, state["seq"], state["cmd_mode"],
                      state["vx"], state["vy"], state["wz"],
                      state["period_steps"], state["height_scale"], state["turn_k"])
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish


def main() -> None:
    srv = viser.ViserServer()
    g = srv.gui

    status = g.add_markdown("**mode 1** (full-auto)  base_vel [0.00, 0.00, 0.00]")

    def refresh() -> None:
        names = {1: "full-auto", 2: "upper-teleop", 3: "full-teleop"}
        status.content = (f"**mode {state['cmd_mode']}** ({names[state['cmd_mode']]})  "
                          f"base_vel [{state['vx']:.2f}, {state['vy']:.2f}, {state['wz']:.2f}]")

    with g.add_folder("Mode"):
        mode = g.add_button_group("cmd_mode", ("1: full-auto", "2: upper-teleop", "3: full-teleop"))

        @mode.on_click
        def _(ev) -> None:
            state["cmd_mode"] = int(ev.target.value[0])
            _write(); refresh()

    with g.add_folder("base_vel (yaw-local)"):
        vx = g.add_slider("vx", -1.0, 1.0, 0.01, 0.0)   # deploy cap KB_MAXV=1.0
        vy = g.add_slider("vy", -1.0, 1.0, 0.01, 0.0)
        wz = g.add_slider("wz (turn)", -0.6, 0.6, 0.01, 0.0)  # deploy cap KB_MAXW=0.6

        def _v(_=None) -> None:
            state["vx"], state["vy"], state["wz"] = vx.value, vy.value, wz.value
            _write(); refresh()
        for w in (vx, vy, wz):
            w.on_update(_v)

        zero = g.add_button("base_vel = 0")

        @zero.on_click
        def _(_) -> None:
            vx.value = vy.value = wz.value = 0.0
            _v()

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

    _write()  # publish initial state
    print(f"[masked_gui] writing {SHM_PATH}; open the viser URL above. Ctrl-C to quit.")
    import time
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
