"""Shared control-channel contract for the masked-loco deploy.

Both the viser GUI (masked_gui.py) and the PICO bridge (pico_control_bridge.py) write this
packed struct to /dev/shm/g1_masked_gui; the C++ controller reads it (State_Mimic.cpp
`struct GuiCtrl`, #pragma pack(1), g_poll_gui). ⚠ KEEP THIS LAYOUT IN SYNC WITH THAT C++
STRUCT — this file is the single Python source of the contract
(tests/test_gui_shm_layout.py 가 C++ 구조체와 바이트 배치를 대조한다).
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_masked_gui"
MAGIC = 0x6703          # v2 (2026-09-22). v1 = 0x6701 — C++ 가 거부한다(옛 GUI 가 새 제어기를 조종하지 않게).
# <  little-endian, packed.
#   magic seq mode_req vx vy wz period_steps height_scale turn_k m5_preset m5_press_seq clip_req
#   mode_req  = 0 이면 «모드 요청 없음». 1회성 — 보낸 뒤 0 으로 되돌린다(속도만 바꿨는데 모드가 다시 요청되지 않게).
#   m5_preset = 0 없음, 1..N = mode5_presets_gen.PRESETS[index] + 1.  m5_press_seq = 누를 때마다 +1.
#   clip_req  = −1 그대로, 0.. 고를 클립 (재생 중이면 C++ 가 거부). 1회성.
FMT = "<iIifffiffiIi"

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
    """Atomically publish the control struct. 필수 키: seq, vx, vy, wz, period_steps, height_scale, turn_k.
    선택 키: mode_req(0), m5_preset(0), m5_press_seq(0), clip_req(−1). Increments state['seq']."""
    state["seq"] += 1
    buf = struct.pack(FMT, MAGIC, state["seq"], int(state.get("mode_req", 0)),
                      state["vx"], state["vy"], state["wz"],
                      state["period_steps"], state["height_scale"], state["turn_k"],
                      int(state.get("m5_preset", 0)), int(state.get("m5_press_seq", 0)),
                      int(state.get("clip_req", -1)))
    state["mode_req"] = 0
    state["clip_req"] = -1
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish (no torn reads)
