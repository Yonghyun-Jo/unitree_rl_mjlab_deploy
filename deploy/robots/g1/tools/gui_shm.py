"""Shared control-channel contract for the masked-loco deploy.

Both the viser GUI (masked_gui.py) and the PICO bridge (pico_control_bridge.py) write this
packed struct to /dev/shm/g1_masked_gui; the C++ controller reads it (State_Mimic.cpp
`struct GuiCtrl`, #pragma pack(1), g_poll_gui). ⚠ KEEP THIS LAYOUT IN SYNC WITH THAT C++
STRUCT — this file is the single Python source of the contract.
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_masked_gui"
MAGIC = 0x6701
# <  little-endian, packed.  i I i f f f i f f
#   magic  seq  cmd_mode  vx  vy  wz  period_steps  height_scale  turn_k
FMT = "<iIifffiff"

# Deploy velocity caps = TRAINING base_vel range (match C++ KB_MAXVX/VY/W). Clamped again in C++.
#   vx p99=3.10 → 3.0 (running);  vy |.|p99=1.88 (lateral sparse) → 1.5;  wz → 0.6 (conservative).
VXCAP, VYCAP, WCAP = 3.0, 1.5, 0.6


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def write(state: dict) -> None:
    """Atomically publish the control struct. `state` has keys: seq, cmd_mode, vx, vy, wz,
    period_steps, height_scale, turn_k. Increments state['seq']."""
    state["seq"] += 1
    buf = struct.pack(FMT, MAGIC, state["seq"], state["cmd_mode"],
                      state["vx"], state["vy"], state["wz"],
                      state["period_steps"], state["height_scale"], state["turn_k"])
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish (no torn reads)
