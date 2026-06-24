"""VR reference channel contract — /dev/shm/g1_vr_ref (variant B).

A com1-local producer (vr_replay.py for a recorded clip, or zmq_to_vr_bridge.py from
PICO+GMR) writes this packed struct; the C++ controller reads it (State_Mimic.cpp `struct
VrRef`, #pragma pack(1), g_poll_vr) and feeds it into the MotionLoader so the masked obs
read the VR reference. ⚠ KEEP THIS LAYOUT IN SYNC WITH THAT C++ STRUCT.

Layout (little-endian, packed):
  magic(i) seq(I) valid(i) cmd_mode(i) | base_vel[3] root_quat[4:wxyz] dof_pos[29] dof_vel[29]
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_vr_ref"
MAGIC = 0x6702
FMT = "<iIii" + "f" * (3 + 4 + 29 + 29)   # 4 ints + 65 floats = 276 bytes


def write(seq: int, valid: int, cmd_mode: int, base_vel, root_quat, dof_pos, dof_vel) -> None:
    """Atomically publish the VR reference. base_vel[3], root_quat[4:wxyz], dof_pos[29],
    dof_vel[29]. valid=0 -> C++ reverts to the clip."""
    assert len(base_vel) == 3 and len(root_quat) == 4 and len(dof_pos) == 29 and len(dof_vel) == 29
    buf = struct.pack(FMT, MAGIC, int(seq), int(valid), int(cmd_mode),
                      *base_vel, *root_quat, *dof_pos, *dof_vel)
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish
