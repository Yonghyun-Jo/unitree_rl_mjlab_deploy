"""클립 npz 로딩 + 재생 전 프리플라이트 검사.

프리플라이트는 "이 구간을 이 모드로 재생해도 되는가"를 판정한다. 하나라도 걸리면
재생하지 않는다 — 실로봇에서 되돌릴 수 없는 것은 낙상뿐이다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .resolver import ClipInfo

# 램프인 속도 상한과 바닥값. 실측 entry_err 0.40~0.64 rad -> 1.5~1.9 s.
# C++ 모드전환 크로스페이드(switch_blend_steps: 50 = 1.0 s)보다 항상 길어야 한다.
RAMP_RATE_RAD_S = 0.35
RAMP_IN_MIN_S = 1.5


@dataclass
class DeployProfile:
    standby: np.ndarray                  # [29] deploy.yaml default_joint_pos
    pos_min: np.ndarray | None           # [29] 또는 None(검사 생략)
    pos_max: np.ndarray | None


@dataclass
class ClipData:
    name: str
    fps: float
    n_frames: int
    duration: float
    joint_pos: np.ndarray                # [T,29]
    joint_vel: np.ndarray                # [T,29]
    root_quat: np.ndarray                # [T,4] pelvis wxyz
    pelvis_lin_vel_w: np.ndarray         # [T,3]
    pelvis_ang_vel_w: np.ndarray         # [T,3]


@dataclass
class PreflightResult:
    ok: bool
    reasons: list[str] = field(default_factory=list)
    f_start: int = 0
    f_end: int = 0
    entry_err: float = 0.0
    entry_joint: int = -1
    ramp_in_s: float = RAMP_IN_MIN_S


def load_deploy_profile(policy_dir: Path) -> DeployProfile:
    """deploy.yaml 에서 standby pose 와 관절한계를 읽는다. 하드코딩 금지."""
    doc = yaml.safe_load((Path(policy_dir) / "params" / "deploy.yaml").read_text())
    standby = np.asarray(doc["default_joint_pos"], dtype=np.float32)
    lo = hi = None
    safety = doc.get("safety") or {}
    if isinstance(safety.get("pos_min"), list) and isinstance(safety.get("pos_max"), list):
        lo = np.asarray(safety["pos_min"], dtype=np.float32)
        hi = np.asarray(safety["pos_max"], dtype=np.float32)
    else:
        clip = ((doc.get("actions") or {}).get("JointPositionAction") or {}).get("clip")
        if isinstance(clip, list) and len(clip) == len(standby):
            arr = np.asarray(clip, dtype=np.float32)
            lo, hi = arr[:, 0], arr[:, 1]
    if lo is None or lo.shape != standby.shape:
        lo = hi = None
    return DeployProfile(standby=standby, pos_min=lo, pos_max=hi)


def load_clip(info: ClipInfo) -> ClipData:
    """npz -> ClipData. fps 는 파일에서 읽는다(50 하드코딩 금지). pelvis = body 0."""
    d = np.load(info.path)
    jp = np.asarray(d["joint_pos"], dtype=np.float32)
    jv = np.asarray(d["joint_vel"], dtype=np.float32)
    quat = np.asarray(d["body_quat_w"][:, 0], dtype=np.float32)      # pelvis wxyz
    lin = np.asarray(d["body_lin_vel_w"][:, 0], dtype=np.float32)
    ang = np.asarray(d["body_ang_vel_w"][:, 0], dtype=np.float32)
    fps = float(np.asarray(d["fps"]).reshape(-1)[0]) if "fps" in d else 50.0
    n = int(len(jp))
    return ClipData(name=info.name, fps=fps, n_frames=n, duration=n / fps,
                    joint_pos=jp, joint_vel=jv, root_quat=quat,
                    pelvis_lin_vel_w=lin, pelvis_ang_vel_w=ang)


def preflight(clip: ClipData, span: tuple[float, float], mode: int,
              allowed_modes: tuple[int, ...], profile: DeployProfile) -> PreflightResult:
    """재생 전 전수 검사. 통과해야만 송출한다."""
    reasons: list[str] = []

    if mode == 1:
        reasons.append("mode1 재생 불가 — 참조가 상·하체 전부 마스킹돼 정책에 도달하지 않는다 "
                       "(masked_joint_command/masked_root_ori_b 가 0). mode2 또는 mode3 을 쓰세요.")
    elif mode not in (2, 3):
        reasons.append(f"mode{mode} 는 VR 채널로 보낼 수 없다 (g_poll_vr 이 1~3만 수용)")
    elif mode not in allowed_modes:
        reasons.append(f"mode{mode} 는 이 클립/정책에서 유효하지 않다 (유효: {sorted(allowed_modes)})")

    t0, t1 = float(span[0]), float(span[1])
    span_ok = (0.0 <= t0 < t1 <= clip.duration)
    if not span_ok:
        reasons.append(f"구간이 잘못됨: [{t0:.2f}, {t1:.2f}] — 클립 길이 {clip.duration:.2f}s")
        return PreflightResult(ok=False, reasons=reasons)

    f_start = int(round(t0 * clip.fps))
    f_end = min(int(round(t1 * clip.fps)), clip.n_frames)
    if f_end - f_start < 2:
        reasons.append(f"구간이 너무 짧음: {f_end - f_start} 프레임")
        return PreflightResult(ok=False, reasons=reasons)

    seg_p = clip.joint_pos[f_start:f_end]
    seg_v = clip.joint_vel[f_start:f_end]
    if not (np.isfinite(seg_p).all() and np.isfinite(seg_v).all()
            and np.isfinite(clip.root_quat[f_start:f_end]).all()):
        reasons.append("구간에 NaN/Inf 가 있다")

    if profile.pos_min is not None:
        under = seg_p < profile.pos_min
        over = seg_p > profile.pos_max
        if under.any() or over.any():
            j = int(np.argmax((under | over).any(axis=0)))
            reasons.append(f"관절한계 위반: joint[{j}] "
                           f"(범위 [{profile.pos_min[j]:.3f}, {profile.pos_max[j]:.3f}])")

    diff = np.abs(clip.joint_pos[f_start] - profile.standby)
    entry_joint = int(np.argmax(diff))
    entry_err = float(diff[entry_joint])
    ramp_in_s = max(RAMP_IN_MIN_S, entry_err / RAMP_RATE_RAD_S)

    return PreflightResult(ok=not reasons, reasons=reasons, f_start=f_start, f_end=f_end,
                           entry_err=entry_err, entry_joint=entry_joint, ramp_in_s=ramp_in_s)
