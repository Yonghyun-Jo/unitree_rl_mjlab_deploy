#!/usr/bin/env python3
"""Live PICO VR -> gmt_multihead policy bridge (com1, gmr env).

Replaces vr_replay.py's clip source with the LIVE PICO stream:

  노트북 pico_publisher.py --ZMQ--> [this: bind :5556]
    -> GMR("xrobot","unitree_g1").retarget(body[24])  -> qpos[36]
       root_quat = qpos[3:7] (wxyz),  dof_pos = qpos[7:36] (== robot_spec JOINT_ORDER, no remap)
    -> dof_vel  = finite-diff(dof_pos) + EMA  (policy uses q̇_ref; NOT published by laptop)
    -> base_vel = controller thumbstick,  cmd_mode = buttons
    -> vr_shm.write(...) -> /dev/shm/g1_vr_ref  -> g1_ctrl (gmt_multihead) -> unitree_mujoco

cmd_mode masks in C++: mode2=상체(팔·waist), mode3=전신(다리 포함). ONE bridge, full VrRef.

⚠ RUN with the gmr env's python (VIRTUAL_ENV in the profile hijacks conda run):
    /home/piene/miniconda3/envs/gmr/bin/python \
        /home/piene/unitree_rl_mjlab/deploy/robots/g1/teleop/vr_teleop_bridge.py --mode 2
  (run alongside: unitree_mujoco  +  g1_ctrl --network=lo  [f->stand, m->policy])
"""
from __future__ import annotations

import argparse
import json
import signal
import time

import numpy as np
import zmq
from scipy.spatial.transform import Rotation

import vr_shm  # same teleop/ dir
from general_motion_retargeting import GeneralMotionRetargeting
from general_motion_retargeting.rot_utils import quat_mul_np


def _sigterm(*_):
    raise KeyboardInterrupt  # SIGTERM/timeout -> clean finally (valid=0 revert)


# SMPL 24-joint order the PICO body stream uses (GMR xrobot_utils.body_joint_names).
BODY_JOINT_NAMES = [
    "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
    "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
    "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
    "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand",
]

IDENTITY_QUAT = [1.0, 0.0, 0.0, 0.0]
ZERO29 = [0.0] * 29

# Unity -> right-hand transform that GMR's XRobotStreamer.get_processed_body_data applies
# (xrobot_utils.py:175-190). The PICO stream carries RAW get_body_joints_pose per joint:
#   [x, y, z, qx, qy, qz, qw]  (position + quaternion, SCALAR-LAST xyzw).
# The bridge must (1) reorder quat -> wxyz, (2) apply this transform, else GMR retargets garbage.
_R_UNITY2RH = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
_ROT_QUAT = Rotation.from_matrix(_R_UNITY2RH).as_quat(scalar_first=True)  # wxyz


def _msg_body_to_human(joints, transform: bool = True) -> dict:
    """PICO body.joints [24][7] (raw [x,y,z,qx,qy,qz,qw]) -> GMR human_data {name:[pos,quat_wxyz]}.

    transform=True replicates get_processed_body_data (reorder xyzw->wxyz + Unity->RH). Set False
    only if the publisher already sends processed (wxyz + RH) data.
    """
    human = {}
    for i, name in enumerate(BODY_JOINT_NAMES):
        j = joints[i]
        if transform:
            qx, qy, qz, qw = j[3], j[4], j[5], j[6]
            quat = quat_mul_np(_ROT_QUAT, np.array([qw, qx, qy, qz]), scalar_first=True)
            pos = np.array(j[0:3]) @ _R_UNITY2RH.T
            human[name] = [pos.tolist(), quat.tolist()]
        else:
            human[name] = [list(j[0:3]), list(j[3:7])]
    return human


def _drain_latest(sub):
    """Return the freshest multipart message (drop stale), or None if nothing pending."""
    latest = None
    while True:
        try:
            latest = sub.recv_multipart(zmq.NOBLOCK)
        except zmq.Again:
            return latest


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--mode", type=int, default=2, choices=[1, 2, 3],
                    help="default cmd_mode (2=upper-teleop, 3=full). buttons override at runtime.")
    ap.add_argument("--ema", type=float, default=0.5, help="dof_vel EMA alpha (0..1, higher=less smoothing)")
    ap.add_argument("--vx", type=float, default=1.5, help="base_vel vx cap (m/s) at full stick")
    ap.add_argument("--vy", type=float, default=0.8, help="base_vel vy cap")
    ap.add_argument("--wz", type=float, default=1.5, help="base_vel wz cap (rad/s)")
    ap.add_argument("--no-body-transform", action="store_true",
                    help="skip Unity->RH + quat reorder (only if publisher already sends processed body)")
    ap.add_argument("--grip-enable", action="store_true",
                    help="only teleop while a grip is held (valid=0 otherwise). default: always on.")
    ap.add_argument("--height", type=float, default=None, help="operator height (m) for GMR scaling")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, _sigterm)

    print("[bridge] loading GMR (xrobot -> unitree_g1) ...")
    gmr = GeneralMotionRetargeting("xrobot", "unitree_g1", actual_human_height=args.height)
    print("[bridge] GMR ready.")

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 4)                 # small queue; _drain_latest keeps freshest
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, 2000)
    sub.bind(f"tcp://*:{args.port}")
    print(f"[bridge] bind tcp://*:{args.port}  default mode={args.mode}  grip_enable={args.grip_enable}")

    prev_dof = None
    dof_vel = np.zeros(29, dtype=np.float32)
    cmd_mode = args.mode
    seq_out = 0
    last_t = None
    # status
    n = 0
    t_report = time.monotonic()
    proc = 0

    def revert():
        nonlocal seq_out
        vr_shm.write(seq_out, 0, cmd_mode, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)
        seq_out += 1

    try:
        while True:
            msg = _drain_latest(sub)
            if msg is None:
                try:
                    msg = sub.recv_multipart()          # block (with RCVTIMEO)
                except zmq.Again:
                    print("[bridge] no data (publisher off? PICO Send ON?)")
                    prev_dof = None
                    continue
            _, payload = msg
            f = json.loads(payload.decode())

            if not f.get("streaming") or not f.get("body", {}).get("available"):
                revert()                                # body 없음 -> clip 복귀
                prev_dof = None
                continue

            # --- controllers -> base_vel, cmd_mode, enable ---
            L, R = f["controllers"]["left"], f["controllers"]["right"]
            base_vel = [L["axis"][1] * args.vx, -L["axis"][0] * args.vy, -R["axis"][0] * args.wz]
            if L["primary"] or R["primary"]:
                cmd_mode = 2
            if L["secondary"] or R["secondary"]:
                cmd_mode = 3
            if L["menu"] or R["menu"]:
                cmd_mode = 1
            enabled = (not args.grip_enable) or (L["grip"] > 0.5 or R["grip"] > 0.5)

            # --- body[24] -> GMR human_data dict -> qpos ---
            j = f["body"]["joints"]                     # [24][7] = raw [x,y,z, qx,qy,qz,qw]
            human = _msg_body_to_human(j, transform=not args.no_body_transform)
            qpos = gmr.retarget(human, offset_to_ground=True)   # [36]
            root_quat = [float(x) for x in qpos[3:7]]           # wxyz
            dof_pos = np.asarray(qpos[7:36], dtype=np.float32)  # 29, == deploy JOINT_ORDER

            # --- dof_vel = finite-diff + EMA (policy's q̇_ref) ---
            now = time.monotonic()
            if prev_dof is not None and last_t is not None:
                dt = max(now - last_t, 1e-3)
                raw = (dof_pos - prev_dof) / dt
                dof_vel = args.ema * raw + (1.0 - args.ema) * dof_vel
            prev_dof = dof_pos.copy()
            last_t = now

            vr_shm.write(seq_out, 1 if enabled else 0, cmd_mode, base_vel, root_quat,
                         dof_pos.tolist(), dof_vel.tolist())
            seq_out += 1
            n += 1
            proc += 1
            if now - t_report >= 1.0:
                hz = proc / (now - t_report)
                print(f"[bridge] {hz:5.1f}Hz  mode={cmd_mode}  valid={1 if enabled else 0}  "
                      f"base_vel=({base_vel[0]:+.2f},{base_vel[1]:+.2f},{base_vel[2]:+.2f})  "
                      f"arm(L_sho_pitch idx15)={dof_pos[15]:+.2f}")
                proc = 0
                t_report = now
    except KeyboardInterrupt:
        pass
    finally:
        revert()                                        # valid=0 -> C++ reverts to clip
        sub.close(0)
        print(f"\n[bridge] stopped (valid=0). total {n} frames retargeted.")


if __name__ == "__main__":
    main()
