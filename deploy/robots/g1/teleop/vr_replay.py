#!/usr/bin/env python3
"""Phase 3.5 de-risk: replay a recorded motion clip as the VR reference.

Writes each frame's (joint_pos, joint_vel, pelvis quat) to /dev/shm/g1_vr_ref at the control
rate, so the C++ obs-swap (g_poll_vr -> MotionLoader VR override) can be validated WITHOUT
PICO / ZMQ / GMR. Run g1_ctrl in mode 2 (upper-teleop) and this should make the policy track
the clip's ARMS (upper body) while walking via base_vel — confirming the VR plumbing + the
coordinate convention BEFORE wiring the live PICO pipeline.

Because masked_footz_v0 was trained on these clips, replaying a manifest clip is in-distribution.

Usage (on com1, alongside unitree_mujoco + `g1_ctrl --network=lo`):
    python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2
    # e.g. a manifest motion: .../mjlab_g1_motion/motions/walk/walk3_subject3.npz
npz needs: joint_pos[T,29], joint_vel[T,29], body_quat_w[T,B,4] (pelvis = body 0, wxyz).
        body_pos_w[T,30,3] 이 있으면 레퍼런스 발 world-z 도 같이 실어 보낸다 (mode3 필수 —
        아래 FOOT_BODY_IDX 주석 참조).
"""
from __future__ import annotations

import argparse
import time

import numpy as np

import vr_shm  # same dir (teleop/)

# npz body_pos_w 의 body 축 = MuJoCo g1.xml body 순서(world 제외, 30개).
#   6 = left_ankle_roll_link, 12 = right_ankle_roll_link
# 학습(mjlab_g1_motion FOOT_BODIES)이 쓰는 두 body 와 같다. C++ 쪽
# State_Mimic.h::MotionLoader_::NPZ_FOOT_IDX 와 반드시 같은 값이어야 한다
# (tests/test_ref_foot_idx.py 가 이름으로 대조한다).
NPZ_NUM_BODIES = 30
FOOT_BODY_IDX = (6, 12)   # [L, R]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("npz", help="motion .npz (joint_pos, joint_vel, body_quat_w)")
    p.add_argument("--mode", type=int, default=2, help="cmd_mode (2=upper-teleop, 3=full)")
    p.add_argument("--base-vel", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   help="vx vy wz commanded while replaying (legs walk via this)")
    p.add_argument("--fps", type=float, default=50.0)
    p.add_argument("--loop", action="store_true", help="loop the clip")
    args = p.parse_args()

    d = np.load(args.npz)
    jp = np.asarray(d["joint_pos"], dtype=np.float32)        # [T,29]
    jv = np.asarray(d["joint_vel"], dtype=np.float32)        # [T,29]
    quat = np.asarray(d["body_quat_w"][:, 0], dtype=np.float32)  # pelvis [T,4] wxyz
    # 레퍼런스 발 world-z [T,2]. mode3 은 학습에 FOOT_GEN 이 없어 이 값이 foot_z obs 원천이다.
    # 없으면 None 으로 두어 C++ 이 stance 폴백임을 «알고» 처리하게 한다 (0 을 보내면 거짓 obs).
    foot_z = None
    if "body_pos_w" in d and d["body_pos_w"].shape[1] == NPZ_NUM_BODIES:
        foot_z = np.asarray(d["body_pos_w"][:, FOOT_BODY_IDX, 2], dtype=np.float32)  # [T,2]
    else:
        print("[vr_replay] ⚠ body_pos_w 없음/모양 다름 -> 발-z 미전송 "
              "(mode3 은 stance 상수로 폴백 = 학습과 불일치)")
    T = len(jp)
    assert jp.shape[1] == 29 and jv.shape[1] == 29 and quat.shape[1] == 4, "unexpected dims"
    dt = 1.0 / args.fps
    print(f"[vr_replay] {args.npz}: {T} frames, mode={args.mode}, base_vel={args.base_vel}, "
          f"loop={args.loop}. Ctrl-C to stop.")
    seq = 0
    try:
        i = 0
        while True:
            f = i % T
            vr_shm.write(seq, 1, args.mode, args.base_vel,
                         quat[f].tolist(), jp[f].tolist(), jv[f].tolist(),
                         foot_z=None if foot_z is None else foot_z[f].tolist())
            seq += 1
            i += 1
            if not args.loop and i >= T:
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        # valid=0 -> C++ reverts to the clip (don't leave a stale VR override).
        vr_shm.write(seq, 0, args.mode, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0] * 29, [0.0] * 29)
        print("\n[vr_replay] stopped (valid=0 -> back to clip).")


if __name__ == "__main__":
    main()
