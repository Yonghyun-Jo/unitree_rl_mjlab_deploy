"""test_clips.py — 클립 로딩/프리플라이트 자체 검증 (합성 npz, 로봇 불필요)."""
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import clips, resolver  # noqa: E402

STANDBY = np.zeros(29, dtype=np.float32)


def _make_npz(path: Path, T=100, fps=50.0, offset=0.5, nan_at=None, nan_pelvis_at=None):
    jp = np.full((T, 29), offset, dtype=np.float32)
    jv = np.full((T, 29), 0.1, dtype=np.float32)
    if nan_at is not None:
        jp[nan_at, 3] = np.nan
    quat = np.zeros((T, 30, 4), dtype=np.float32)
    quat[:, :, 0] = 1.0
    lin = np.full((T, 30, 3), 0.2, dtype=np.float32)
    ang = np.full((T, 30, 3), 0.3, dtype=np.float32)
    if nan_pelvis_at is not None:
        lin[nan_pelvis_at, 0, 0] = np.nan    # pelvis(body 0) 선속도에 NaN 주입
    np.savez(path, joint_pos=jp, joint_vel=jv,
             body_pos_w=np.zeros((T, 30, 3), dtype=np.float32),
             body_quat_w=quat,
             body_lin_vel_w=lin,
             body_ang_vel_w=ang,
             fps=np.array([fps]))


def _write_deploy_yaml(policy_dir: Path, *, standby, safety=None, clip=None):
    """load_deploy_profile 이 읽을 임시 deploy.yaml 을 만든다 (실제 배포 yaml 은 건드리지 않음)."""
    params_dir = policy_dir / "params"
    params_dir.mkdir(parents=True, exist_ok=True)
    doc: dict = {"default_joint_pos": standby}
    if safety is not None:
        doc["safety"] = safety
    if clip is not None:
        doc["actions"] = {"JointPositionAction": {"clip": clip}}
    (params_dir / "deploy.yaml").write_text(yaml.safe_dump(doc))


def _profile(lo=-2.0, hi=2.0):
    return clips.DeployProfile(standby=STANDBY.copy(),
                               pos_min=np.full(29, lo, dtype=np.float32),
                               pos_max=np.full(29, hi, dtype=np.float32))


def _load(d: Path, **kw) -> clips.ClipData:
    p = d / "walk1_subject1.npz"
    _make_npz(p, **kw)
    return clips.load_clip(resolver.ClipInfo(name="walk1_subject1", path=p, modes=(1, 2, 3)))


def test_load_clip_reads_fps_and_pelvis():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=120, fps=30.0)
        assert c.fps == 30.0, c.fps          # 50 하드코딩 금지
        assert c.n_frames == 120
        assert abs(c.duration - 4.0) < 1e-6, c.duration
        assert c.joint_pos.shape == (120, 29)
        assert c.pelvis_lin_vel_w.shape == (120, 3)   # body 0 만 뽑아 옴
        assert c.root_quat.shape == (120, 4)
    print("  ok load_clip")


def test_preflight_ok_and_ramp_time():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.7)       # entry_err = 0.7 rad
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert r.ok, r.reasons
        assert r.f_start == 0 and r.f_end == 50, (r.f_start, r.f_end)
        assert abs(r.entry_err - 0.7) < 1e-5, r.entry_err
        assert abs(r.ramp_in_s - 2.0) < 1e-5, r.ramp_in_s   # 0.7/0.35 = 2.0 > 1.5
    print("  ok preflight_ok")


def test_preflight_ramp_floor():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.1)       # 0.1/0.35 = 0.29 -> 바닥값 1.5 로
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert abs(r.ramp_in_s - 1.5) < 1e-9, r.ramp_in_s
    print("  ok ramp_floor")


def test_preflight_rejects_mode1():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d))
        r = clips.preflight(c, (0.0, 1.0), mode=1, allowed_modes=(1, 2, 3), profile=_profile())
        assert not r.ok
        assert any("mode1" in s for s in r.reasons), r.reasons
    print("  ok reject_mode1")


def test_preflight_rejects_mode_not_allowed():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d))
        r = clips.preflight(c, (0.0, 1.0), mode=2, allowed_modes=(3,), profile=_profile())
        assert not r.ok
        assert any("mode2" in s for s in r.reasons), r.reasons
    print("  ok reject_mode_not_allowed")


def test_preflight_rejects_bad_span():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=100, fps=50.0)      # duration = 2.0 s
        for span in [(-1.0, 1.0), (0.0, 5.0), (1.5, 1.5), (1.5, 0.5)]:
            r = clips.preflight(c, span, mode=3, allowed_modes=(1, 2, 3), profile=_profile())
            assert not r.ok, span
            assert any("구간" in s for s in r.reasons), (span, r.reasons)
    print("  ok reject_bad_span")


def test_preflight_rejects_nan_in_span():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=100, nan_at=60)
        r = clips.preflight(c, (0.0, 0.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert r.ok, r.reasons                       # 0~25 프레임에는 NaN 없음
        r = clips.preflight(c, (1.0, 1.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert not r.ok                              # 50~75 프레임에 NaN 있음
        assert any("NaN" in s for s in r.reasons), r.reasons
    print("  ok reject_nan")


def test_preflight_rejects_nan_in_pelvis_vel():
    """joint_pos/joint_vel/root_quat 뿐 아니라 pelvis_lin_vel_w 의 NaN 도 잡아야 한다.

    이 값은 Task 3 의 yaw_local_base_vel() 을 거쳐 base_vel 로 /dev/shm/g1_vr_ref 에 실려
    실로봇(mode2)까지 그대로 간다 — joint 배열만 검사하면 구멍이 뚫린다.
    """
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=100, nan_pelvis_at=60)
        r = clips.preflight(c, (0.0, 0.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert r.ok, r.reasons                       # 0~25 프레임에는 NaN 없음
        r = clips.preflight(c, (1.0, 1.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert not r.ok                              # 50~75 프레임에 pelvis_lin_vel_w NaN 있음
        assert any("NaN" in s for s in r.reasons), r.reasons
    print("  ok reject_nan_pelvis_vel")


def test_preflight_rejects_joint_limit():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.5)
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3),
                            profile=_profile(lo=-0.2, hi=0.2))
        assert not r.ok
        assert any("관절한계" in s for s in r.reasons), r.reasons
    print("  ok reject_joint_limit")


def test_load_deploy_profile_clip_fallback():
    """safety 블록이 없을 때 actions.JointPositionAction.clip 으로 폴백하는지."""
    with tempfile.TemporaryDirectory() as d:
        policy_dir = Path(d) / "slot"
        standby = [0.0] * 29
        clip_pairs = [[-1.0, 1.0]] * 29
        _write_deploy_yaml(policy_dir, standby=standby, clip=clip_pairs)
        p = clips.load_deploy_profile(policy_dir)
        assert p.standby.shape == (29,), p.standby.shape
        assert p.pos_min is not None and p.pos_max is not None
        assert np.allclose(p.pos_min, -1.0) and np.allclose(p.pos_max, 1.0)
    print("  ok load_deploy_profile_clip_fallback")


def test_load_deploy_profile_no_limits():
    """safety 도 actions.JointPositionAction.clip 도 없으면 pos_min/pos_max 는 None (검사 생략)."""
    with tempfile.TemporaryDirectory() as d:
        policy_dir = Path(d) / "slot"
        standby = [0.0] * 29
        _write_deploy_yaml(policy_dir, standby=standby)
        p = clips.load_deploy_profile(policy_dir)
        assert p.standby.shape == (29,), p.standby.shape
        assert p.pos_min is None, p.pos_min
        assert p.pos_max is None, p.pos_max
    print("  ok load_deploy_profile_no_limits")


def test_preflight_skips_joint_limit_when_profile_has_none():
    """pos_min/pos_max 가 None 인 profile 로는 관절한계 검사를 건너뛰고(크래시 없이) 통과해야 한다."""
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=50.0)      # 실제 한계라면 무조건 걸릴 값
        profile = clips.DeployProfile(standby=STANDBY.copy(), pos_min=None, pos_max=None)
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3), profile=profile)
        assert r.ok, r.reasons
        assert not any("관절한계" in s for s in r.reasons), r.reasons
    print("  ok preflight_skips_joint_limit_none")


def test_load_deploy_profile_from_real_yaml():
    """실제 배포 yaml 을 읽어 standby/한계가 29개로 나오는지."""
    here = Path(__file__).resolve().parents[3]   # -> deploy/robots/g1
    slot = here / "config" / "policy" / "mimic_masked" / "gmt_multihead_cwc_scratch"
    if not (slot / "params" / "deploy.yaml").is_file():
        print("  skip load_deploy_profile (슬롯 없음)")
        return
    p = clips.load_deploy_profile(slot)
    assert p.standby.shape == (29,), p.standby.shape
    assert abs(float(p.standby[3]) - 0.669) < 1e-6, p.standby[3]   # KNEES_BENT 무릎
    assert p.pos_min is not None and p.pos_min.shape == (29,)
    print("  ok load_deploy_profile")


def main() -> int:
    test_load_clip_reads_fps_and_pelvis()
    test_preflight_ok_and_ramp_time()
    test_preflight_ramp_floor()
    test_preflight_rejects_mode1()
    test_preflight_rejects_mode_not_allowed()
    test_preflight_rejects_bad_span()
    test_preflight_rejects_nan_in_span()
    test_preflight_rejects_nan_in_pelvis_vel()
    test_preflight_rejects_joint_limit()
    test_load_deploy_profile_clip_fallback()
    test_load_deploy_profile_no_limits()
    test_preflight_skips_joint_limit_when_profile_has_none()
    test_load_deploy_profile_from_real_yaml()
    print("test_clips: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
