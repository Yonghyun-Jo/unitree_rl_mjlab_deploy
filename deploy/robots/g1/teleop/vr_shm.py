"""VR reference channel contract — /dev/shm/g1_vr_ref (variant B).

A com1-local producer (vr_replay.py for a recorded clip, or zmq_to_vr_bridge.py from
PICO+GMR) writes this packed struct; the C++ controller reads it (State_Mimic.cpp `struct
VrRef`, #pragma pack(1), g_poll_vr) and feeds it into the MotionLoader so the masked obs
read the VR reference. ⚠ KEEP THIS LAYOUT IN SYNC WITH THAT C++ STRUCT.

Layout (little-endian, packed):
  magic(i) seq(I) valid(i) cmd_mode(i) | base_vel[3] root_quat[4:wxyz] dof_pos[29] dof_vel[29]
  [+ v2 확장] foot_z[2]        ← 레퍼런스 발 world-z [z_L, z_R]

# 왜 foot_z 가 필요한가 (2026-08-20 추가)
mode3(전신 텔레옵)은 학습 cfg(stage4_mode3_env_cfg)에 FOOT_GEN 이 «없다» → mjlab 의
calc_ref_foot_height 가 생성기가 아니라 «레퍼런스 발 world-z» 를 obs 로 먹인다.
그래서 배포에서도 VR 레퍼런스의 발 높이를 같이 실어 보내야 학습과 같은 obs 가 된다.
안 실으면 C++ 이 stance(두 발 접지) 상수로 폴백한다 — 발을 들고 있어도 «접지» 라고
말하게 되어 학습과 어긋난다.

# 구버전 호환
foot_z 를 생략하면(=None) 확장 전 276 바이트로 쓴다. C++ 은 «파일 길이» 로 신/구를
가르므로, 구버전 publisher 여도 정상 수신되고 발-z 만 없는 것으로 처리된다.
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_vr_ref"
MAGIC = 0x6702
FMT_V1 = "<iIii" + "f" * (3 + 4 + 29 + 29)      # 4 ints + 65 floats = 276 bytes (확장 전)
FMT = FMT_V1 + "ff"                              # + foot_z[2]        = 284 bytes
SIZE_V1 = struct.calcsize(FMT_V1)
SIZE = struct.calcsize(FMT)


def write(seq: int, valid: int, cmd_mode: int, base_vel, root_quat, dof_pos, dof_vel,
          foot_z=None) -> None:
    """Atomically publish the VR reference. base_vel[3], root_quat[4:wxyz], dof_pos[29],
    dof_vel[29]. valid=0 -> C++ reverts to the clip.

    foot_z: [z_L, z_R] 레퍼런스 발 world-z (m). None 이면 확장 전 레이아웃으로 써서
    C++ 이 «발-z 없음» 으로 처리하게 한다 (0 을 보내면 «발이 바닥 아래» 라는 거짓 obs 가
    되므로, 모를 때는 반드시 None 이어야 한다)."""
    assert len(base_vel) == 3 and len(root_quat) == 4 and len(dof_pos) == 29 and len(dof_vel) == 29
    if foot_z is None:
        buf = struct.pack(FMT_V1, MAGIC, int(seq), int(valid), int(cmd_mode),
                          *base_vel, *root_quat, *dof_pos, *dof_vel)
    else:
        assert len(foot_z) == 2
        buf = struct.pack(FMT, MAGIC, int(seq), int(valid), int(cmd_mode),
                          *base_vel, *root_quat, *dof_pos, *dof_vel, *foot_z)
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish
