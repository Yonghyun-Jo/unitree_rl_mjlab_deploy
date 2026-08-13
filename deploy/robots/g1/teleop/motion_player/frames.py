"""참조 프레임 생성 — 순수 함수만. 파일도 shm도 시간도 건드리지 않는다.

여기서 만든 RefFrame 을 publisher 가 vr_shm.write() 로 내보낸다.
순수하게 유지하는 이유: 로봇/시뮬 없이 램프 연속성·속도 스케일·좌표변환을 테스트하기 위해서다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .clips import ClipData, DeployProfile

# 배포 base_vel 캡 — C++ KB_MAXVX / KB_MAXVY / KB_MAXW 와 동일해야 한다.
CAP_VX = 3.0
CAP_VY = 1.5
CAP_WZ = 2.0

_IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class RefFrame:
    cmd_mode: int
    valid: int
    base_vel: tuple[float, float, float]
    root_quat: tuple[float, float, float, float]     # wxyz
    dof_pos: np.ndarray                              # [29]
    dof_vel: np.ndarray                              # [29]


def smoothstep(s: float) -> float:
    """3s^2 - 2s^3. 양 끝에서 1차 도함수가 0이라 진입/이탈이 부드럽다."""
    s = min(1.0, max(0.0, float(s)))
    return s * s * (3.0 - 2.0 * s)


def quat_slerp(a, b, s: float) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    s = min(1.0, max(0.0, float(s)))
    dot = float(np.dot(a, b))
    if dot < 0.0:                 # 최단 경로
        b, dot = -b, -dot
    if dot > 0.9995:              # 거의 같으면 선형보간 후 정규화
        out = a + s * (b - a)
        return out / np.linalg.norm(out)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_t = math.sin(theta)
    return (math.sin((1.0 - s) * theta) / sin_t) * a + (math.sin(s * theta) / sin_t) * b


def yaw_local_base_vel(quat_wxyz, lin_vel_w, ang_vel_w) -> tuple[float, float, float]:
    """pelvis 세계좌표 속도 -> yaw-local [vx, vy, wz].

    학습 base_vel 분포가 "clip pelvis velocity" 라 이 값이 in-distribution 이다
    (deploy/robots/g1/src/State_Mimic.cpp:187-188 주석).
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    vwx, vwy = float(lin_vel_w[0]), float(lin_vel_w[1])
    return (c * vwx + s * vwy, -s * vwx + c * vwy, float(ang_vel_w[2]))


def clamp_base_vel(bv) -> tuple[float, float, float]:
    return (min(CAP_VX, max(-CAP_VX, float(bv[0]))),
            min(CAP_VY, max(-CAP_VY, float(bv[1]))),
            min(CAP_WZ, max(-CAP_WZ, float(bv[2]))))


def _resolve_base_vel(clip: ClipData, f_idx: int, mode: int,
                      kind: str, manual_bv) -> tuple[float, float, float]:
    # mode3 은 C++ MaskedLocoController 가 base_vel 을 0 으로 덮는다(cmd_mode>=3).
    # 혼란을 막기 위해 우리도 0 을 보낸다.
    if mode >= 3:
        return (0.0, 0.0, 0.0)
    if kind == "clip":
        return clamp_base_vel(yaw_local_base_vel(clip.root_quat[f_idx],
                                                 clip.pelvis_lin_vel_w[f_idx],
                                                 clip.pelvis_ang_vel_w[f_idx]))
    if kind == "manual":
        return clamp_base_vel(manual_bv)
    return (0.0, 0.0, 0.0)


def ramp_frame(clip: ClipData, profile: DeployProfile, f_anchor: int, s: float,
               mode: int, base_vel, direction: str, a_scale: float = 1.0) -> RefFrame:
    """standby <-> clip[f_anchor] 사이를 smoothstep 으로 잇는다.

    direction="in":  s=0 -> standby, s=1 -> clip[f_anchor]
    direction="out": s=0 -> clip[f_anchor], s=1 -> standby
    a_scale: 램프인 도중 중단 시, 이미 도달한 진행도에서 되돌리기 위한 배율.
    """
    a = smoothstep(s)
    if direction == "out":
        a = 1.0 - a
    a = a_scale * a
    target_p = clip.joint_pos[f_anchor].astype(np.float64)
    target_v = clip.joint_vel[f_anchor].astype(np.float64)
    standby = profile.standby.astype(np.float64)
    dof_pos = standby + a * (target_p - standby)
    dof_vel = a * target_v
    quat = quat_slerp(_IDENTITY_QUAT, clip.root_quat[f_anchor].astype(np.float64), a)
    return RefFrame(cmd_mode=mode, valid=1,
                    base_vel=clamp_base_vel(base_vel) if mode < 3 else (0.0, 0.0, 0.0),
                    root_quat=tuple(float(v) for v in quat),
                    dof_pos=dof_pos.astype(np.float32),
                    dof_vel=dof_vel.astype(np.float32))


def play_frame(clip: ClipData, f_idx: int, speed: float, mode: int,
               base_vel_kind: str, manual_bv) -> RefFrame:
    """재생 중 한 프레임. ⚠ speed 를 바꾸면 dof_vel 도 같은 배율로 곱한다."""
    f_idx = int(min(max(f_idx, 0), clip.n_frames - 1))
    bv = _resolve_base_vel(clip, f_idx, mode, base_vel_kind, manual_bv)
    return RefFrame(cmd_mode=mode, valid=1, base_vel=bv,
                    root_quat=tuple(float(v) for v in clip.root_quat[f_idx]),
                    dof_pos=clip.joint_pos[f_idx].astype(np.float32),
                    dof_vel=(clip.joint_vel[f_idx] * float(speed)).astype(np.float32))
