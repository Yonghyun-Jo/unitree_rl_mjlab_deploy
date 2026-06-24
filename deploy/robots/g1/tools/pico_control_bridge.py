#!/usr/bin/env python3
"""PICO VR control bridge (variant A: base_vel + mode only).

Reads the XRoboToolkit controller (thumbsticks + buttons) and writes the masked-loco control
struct to /dev/shm/g1_masked_gui — the SAME channel as masked_gui.py, so g1_ctrl needs no
change (it already reads it in g_poll_gui). Run ON THE SAME HOST as g1_ctrl (shared /dev/shm).

This is the lightweight control path (thumbstick → base_vel, buttons → mode). The full-body
mode3 teleop (PICO body tracking → GMR retarget → joint reference) is the SEPARATE reference
bridge (variant B), which needs GMR.

── Control mapping ──────────────────────────────────────────────────────────────────────
  Left stick  Y → vx (forward/back)      Left stick X → vy (strafe)
  Right stick X → wz (yaw)
  X button → mode 1 (full-auto walk)   Y → mode 2 (upper-teleop)   A → mode 3 (full-teleop)
  B button → stop (base_vel = 0)
Scaled to deploy caps (VCAP=1.0 m/s, WCAP=0.6 rad/s); g1_ctrl clamps again.

── Provenance (reference_code) ──
  axis→velocity layout: TWIST2/deploy_real/xrobot_teleop_to_robot_w_hand.py
    _update_velocity_commands (left stick xy, right stick x yaw; y/x sign convention).
  xrt SDK API (init / get_left_axis / get_X_button …): GMR/.../xrobot_utils.py.
  Adapted: TWIST2's idle/teleop/pause state machine + Redis whole-body obs are NOT used here
  (variant A only commands base_vel + our explicit mode 1/2/3 over the GuiCtrl shm channel).

Requires: xrobotoolkit_sdk + PICO PC-Service running (NOT GMR).
    uv run --with xrobotoolkit_sdk python deploy/robots/g1/tools/pico_control_bridge.py
"""
from __future__ import annotations

import time

import gui_shm  # same dir (tools/) — added to sys.path when run as a script

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    raise SystemExit("xrobotoolkit_sdk not found. Install it and start the PICO PC-Service.")

DEADZONE = 0.08   # ignore centered-stick drift
RATE_HZ = 50      # match the controller step rate


def _dz(v: float) -> float:
    return 0.0 if abs(v) < DEADZONE else v


def main() -> None:
    xrt.init()
    state = dict(seq=0, cmd_mode=1, vx=0.0, vy=0.0, wz=0.0,
                 period_steps=43, height_scale=1.0, turn_k=0.3)
    prev = dict(X=False, Y=False, A=False, B=False)
    print(f"[pico_bridge] writing {gui_shm.SHM_PATH}  "
          f"(L-stick=move, R-stick=turn, X/Y/A=mode1/2/3, B=stop). Ctrl-C to quit.")
    dt = 1.0 / RATE_HZ
    while True:
        la = xrt.get_left_axis()    # [x, y] in [-1,1]
        ra = xrt.get_right_axis()
        # base_vel — TWIST2 mapping, scaled to deploy caps (left stick xy, right stick x yaw).
        vx =  _dz(la[1]) * gui_shm.VCAP
        vy = -_dz(la[0]) * gui_shm.VCAP
        wz = -_dz(ra[0]) * gui_shm.WCAP
        # buttons -> mode (edge-triggered: switch once per press)
        X, Y, A, B = xrt.get_X_button(), xrt.get_Y_button(), xrt.get_A_button(), xrt.get_B_button()
        mode = state["cmd_mode"]
        if X and not prev["X"]: mode = 1
        if Y and not prev["Y"]: mode = 2
        if A and not prev["A"]: mode = 3
        stop = bool(B and not prev["B"])
        prev.update(X=X, Y=Y, A=A, B=B)
        if stop:
            vx = vy = wz = 0.0
        # publish only on a meaningful change (avoid spamming the channel when idle)
        changed = (mode != state["cmd_mode"] or stop
                   or abs(vx - state["vx"]) > 1e-3
                   or abs(vy - state["vy"]) > 1e-3
                   or abs(wz - state["wz"]) > 1e-3)
        state["cmd_mode"] = mode
        state["vx"], state["vy"], state["wz"] = vx, vy, wz
        if changed:
            gui_shm.write(state)
        time.sleep(dt)


if __name__ == "__main__":
    main()
