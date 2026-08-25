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

# Deploy velocity caps = TRAINING base_vel range. C++ State_Mimic.cpp 의
#   VX_MAX_FWD / VX_MAX_BWD / KB_MAXVY / KB_MAXW 와 같아야 한다 (C++ 가 한 번 더 clamp 한다).
# 출처: mjlab_g1_motion/tasks/stage4_mode1_env_cfg.py:39
#   CMD_BASE_VEL = vx(-1.5, 2.5) · vy(-0.8, 0.8) · wz(-2.0, 2.0)
# ⚠ vx 비대칭. 종전 값(3.0/1.5/2.0)은 «클립 속도 p99» 근거였는데 mode1 은 클립이 아니라
#   CMD_BASE_VEL 로 학습하므로 낡은 근거였다.
VXCAP, VXCAP_BWD, VYCAP, WCAP = 2.5, 1.5, 0.8, 2.0


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
