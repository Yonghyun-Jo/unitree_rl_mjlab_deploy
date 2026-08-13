"""test_frames.py — 참조 프레임 생성 순수함수 검증 (로봇/파일 불필요)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import clips, frames  # noqa: E402


def _clip(T=100, fps=50.0, pos=0.8, vel=1.2):
    quat = np.zeros((T, 4), dtype=np.float32)
    quat[:, 0] = 1.0
    return clips.ClipData(
        name="c", fps=fps, n_frames=T, duration=T / fps,
        joint_pos=np.full((T, 29), pos, dtype=np.float32),
        joint_vel=np.full((T, 29), vel, dtype=np.float32),
        root_quat=quat,
        pelvis_lin_vel_w=np.zeros((T, 3), dtype=np.float32),
        pelvis_ang_vel_w=np.zeros((T, 3), dtype=np.float32))


def _profile(standby=0.0):
    return clips.DeployProfile(standby=np.full(29, standby, dtype=np.float32),
                               pos_min=None, pos_max=None)


def test_smoothstep_endpoints():
    assert frames.smoothstep(0.0) == 0.0
    assert frames.smoothstep(1.0) == 1.0
    assert abs(frames.smoothstep(0.5) - 0.5) < 1e-9
    assert frames.smoothstep(-3.0) == 0.0 and frames.smoothstep(9.0) == 1.0  # clamp
    print("  ok smoothstep")


def test_ramp_in_endpoints_are_continuous():
    """s=0 이면 정확히 standby, s=1 이면 정확히 clip[f0]. 경계에서 점프가 없어야 한다."""
    c, p = _clip(pos=0.8, vel=1.2), _profile(standby=0.0)
    a = frames.ramp_frame(c, p, f_anchor=0, s=0.0, mode=3, base_vel=(0, 0, 0), direction="in")
    b = frames.ramp_frame(c, p, f_anchor=0, s=1.0, mode=3, base_vel=(0, 0, 0), direction="in")
    assert np.allclose(a.dof_pos, 0.0), a.dof_pos[:3]
    assert np.allclose(a.dof_vel, 0.0), a.dof_vel[:3]
    assert np.allclose(b.dof_pos, 0.8), b.dof_pos[:3]
    assert np.allclose(b.dof_vel, 1.2), b.dof_vel[:3]
    print("  ok ramp_in_endpoints")


def test_ramp_out_endpoints():
    c, p = _clip(pos=0.8, vel=1.2), _profile(standby=0.0)
    a = frames.ramp_frame(c, p, f_anchor=5, s=0.0, mode=3, base_vel=(0, 0, 0), direction="out")
    b = frames.ramp_frame(c, p, f_anchor=5, s=1.0, mode=3, base_vel=(0, 0, 0), direction="out")
    assert np.allclose(a.dof_pos, 0.8) and np.allclose(a.dof_vel, 1.2)
    assert np.allclose(b.dof_pos, 0.0) and np.allclose(b.dof_vel, 0.0)
    print("  ok ramp_out_endpoints")


def test_ramp_is_monotonic():
    c, p = _clip(pos=1.0), _profile(standby=0.0)
    prev = -1.0
    for i in range(21):
        f = frames.ramp_frame(c, p, 0, i / 20.0, 3, (0, 0, 0), "in")
        cur = float(f.dof_pos[0])
        assert cur >= prev - 1e-9, (i, cur, prev)
        prev = cur
    print("  ok ramp_monotonic")


def test_speed_scales_velocity():
    """속도를 늦추면 qd_ref 도 같은 배율로 늦춰져야 한다."""
    c = _clip(vel=2.0)
    full = frames.play_frame(c, 10, speed=1.0, mode=3, base_vel_kind="zero", manual_bv=(0, 0, 0))
    half = frames.play_frame(c, 10, speed=0.5, mode=3, base_vel_kind="zero", manual_bv=(0, 0, 0))
    assert np.allclose(full.dof_vel, 2.0)
    assert np.allclose(half.dof_vel, 1.0), half.dof_vel[:3]
    assert np.allclose(full.dof_pos, half.dof_pos)      # 위치는 프레임이 같으면 같다
    print("  ok speed_scales_velocity")


def test_yaw_local_base_vel_identity():
    """yaw=0 이면 세계좌표 그대로."""
    vx, vy, wz = frames.yaw_local_base_vel((1.0, 0.0, 0.0, 0.0), (1.0, 2.0, 0.0), (0.0, 0.0, 0.5))
    assert abs(vx - 1.0) < 1e-6 and abs(vy - 2.0) < 1e-6 and abs(wz - 0.5) < 1e-6
    print("  ok base_vel_identity")


def test_yaw_local_base_vel_90deg():
    """yaw=+90deg 로봇이 세계 +x 로 1 m/s 이동 -> 몸 기준 vy = -1."""
    h = math.sqrt(0.5)
    q = (h, 0.0, 0.0, h)                    # wxyz, yaw = +pi/2
    vx, vy, wz = frames.yaw_local_base_vel(q, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert abs(vx - 0.0) < 1e-6, vx
    assert abs(vy + 1.0) < 1e-6, vy
    print("  ok base_vel_90deg")


def test_clamp_base_vel():
    assert frames.clamp_base_vel((99.0, -99.0, 99.0)) == (3.0, -1.5, 2.0)
    print("  ok clamp_base_vel")


def test_mode3_zeroes_base_vel():
    """mode3 은 C++ 가 어차피 0으로 덮는다. 혼란 방지를 위해 우리도 0을 보낸다."""
    c = _clip()
    f = frames.play_frame(c, 0, speed=1.0, mode=3, base_vel_kind="manual", manual_bv=(1.0, 1.0, 1.0))
    assert f.base_vel == (0.0, 0.0, 0.0), f.base_vel
    f2 = frames.play_frame(c, 0, speed=1.0, mode=2, base_vel_kind="manual", manual_bv=(1.0, 1.0, 1.0))
    assert f2.base_vel == (1.0, 1.0, 1.0), f2.base_vel
    print("  ok mode3_zeroes_base_vel")


def test_quat_slerp_endpoints():
    a = np.array([1.0, 0.0, 0.0, 0.0])
    b = np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
    assert np.allclose(frames.quat_slerp(a, b, 0.0), a)
    assert np.allclose(frames.quat_slerp(a, b, 1.0), b)
    mid = frames.quat_slerp(a, b, 0.5)
    assert abs(np.linalg.norm(mid) - 1.0) < 1e-6, mid
    print("  ok quat_slerp")


def test_quat_slerp_antipodal():
    """dot < 0: 부호가 반대인 quaternion (같은 회전)에서 최단경로를 취해야 한다."""
    h = math.sqrt(0.5)
    # 90도 회전 about z axis: [sqrt(0.5), 0, 0, sqrt(0.5)]
    # 부호 반대: [-sqrt(0.5), 0, 0, -sqrt(0.5)]
    a = np.array([1.0, 0.0, 0.0, 0.0])
    b = np.array([-h, 0.0, 0.0, -h])  # 부호가 반대 (dot < 0)

    # s=1.0 에서 결과가 unit norm
    result_end = frames.quat_slerp(a, b, 1.0)
    assert abs(np.linalg.norm(result_end) - 1.0) < 1e-6, result_end

    # s=1.0 에서 결과가 최단경로: b 또는 +b (부호 정규화 후)와 같아야 함
    assert np.allclose(result_end, b) or np.allclose(result_end, -b), result_end

    # s=0.5 에서 중간점: 최단경로면 w가 양수이고 0.9 이상 (45도에 가까움)
    # 긴 경로를 취하면 w가 음수이거나 매우 작음
    mid = frames.quat_slerp(a, b, 0.5)
    assert mid[0] > 0.9, f"mid w={mid[0]}, took long path instead of short"
    assert abs(np.linalg.norm(mid) - 1.0) < 1e-6, mid
    print("  ok quat_slerp_antipodal")


def test_quat_slerp_near_parallel():
    """dot > 0.9995: 거의 같은 두 quaternion에서 선형보간 + 정규화를 거친다."""
    # identity
    a = np.array([1.0, 0.0, 0.0, 0.0])
    # 아주 작은 회전 about z: ~0.001 rad => z ≈ sin(0.0005) ≈ 0.0005, w ≈ cos(0.0005) ≈ 0.99999988
    small_angle = 0.001
    b = np.array([
        math.cos(small_angle / 2.0),
        0.0,
        0.0,
        math.sin(small_angle / 2.0)
    ])
    # dot(a, b) ≈ cos(small_angle/2) ≈ 0.9999997... (> 0.9995 ✓)

    # s=0.5 에서 결과는 unit norm (정규화 있음)
    mid = frames.quat_slerp(a, b, 0.5)
    assert abs(np.linalg.norm(mid) - 1.0) < 1e-9, f"norm={np.linalg.norm(mid)}"

    # s=0.5 에서 결과는 a, b 사이에 있어야 함 (기하학적으로 interpolated)
    # a와 b가 아주 가까우므로 선형보간의 중간점도 거기 가까움
    assert np.allclose(mid, (a + b) / 2.0, atol=1e-6), mid
    print("  ok quat_slerp_near_parallel")


def main() -> int:
    test_smoothstep_endpoints()
    test_ramp_in_endpoints_are_continuous()
    test_ramp_out_endpoints()
    test_ramp_is_monotonic()
    test_speed_scales_velocity()
    test_yaw_local_base_vel_identity()
    test_yaw_local_base_vel_90deg()
    test_clamp_base_vel()
    test_mode3_zeroes_base_vel()
    test_quat_slerp_endpoints()
    test_quat_slerp_antipodal()
    test_quat_slerp_near_parallel()
    print("test_frames: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
