"""test_publisher.py — 송출 시퀀스/중단경로 검증 (shm 대신 가짜 writer)."""
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import vr_shm  # noqa: E402
from motion_player import clips, publisher  # noqa: E402


def _spec(speed=1.0, mode=3, ramp_in=1.0, ramp_out=1.0, T=200, fps=50.0):
    quat = np.zeros((T, 4), dtype=np.float32)
    quat[:, 0] = 1.0
    clip = clips.ClipData(name="c", fps=fps, n_frames=T, duration=T / fps,
                          joint_pos=np.full((T, 29), 0.8, dtype=np.float32),
                          joint_vel=np.full((T, 29), 1.0, dtype=np.float32),
                          root_quat=quat,
                          pelvis_lin_vel_w=np.zeros((T, 3), dtype=np.float32),
                          pelvis_ang_vel_w=np.zeros((T, 3), dtype=np.float32))
    profile = clips.DeployProfile(standby=np.zeros(29, dtype=np.float32),
                                  pos_min=None, pos_max=None)
    return publisher.PlaybackSpec(clip=clip, profile=profile, f_start=0, f_end=100,
                                  mode=mode, speed=speed, base_vel_kind="zero",
                                  manual_bv=(0.0, 0.0, 0.0),
                                  ramp_in_s=ramp_in, ramp_out_s=ramp_out)


def test_plan_starts_at_standby_and_ends_at_standby():
    seq = list(publisher.plan_frames(_spec()))
    assert np.allclose(seq[0][1].dof_pos, 0.0), seq[0][1].dof_pos[:3]
    assert seq[0][1].cmd_mode == 3 and seq[0][1].valid == 1
    # 마지막 재생 프레임(RELEASE 직전)은 standby 로 돌아와 있어야 한다
    play_frames = [f for _, f in seq if f.valid == 1 and f.cmd_mode == 3]
    assert np.allclose(play_frames[-1].dof_pos, 0.0, atol=1e-5), play_frames[-1].dof_pos[:3]
    print("  ok starts_and_ends_at_standby")


def test_plan_release_switches_to_mode1_then_invalidates():
    seq = [f for _, f in publisher.plan_frames(_spec())]
    modes = [f.cmd_mode for f in seq]
    assert 1 in modes, "RELEASE 에서 mode1 패킷이 없다 — C++ 크로스페이드가 안 걸린다"
    first_m1 = modes.index(1)
    assert all(m == 3 for m in modes[:first_m1]), modes[:first_m1][:5]
    assert seq[-1].valid == 0, "마지막 패킷은 valid=0 이어야 한다"
    assert all(f.valid == 1 for f in seq[:-1])
    print("  ok release_sequence")


def test_plan_reaches_clip_pose_during_play():
    seq = [f for _, f in publisher.plan_frames(_spec())]
    assert any(np.allclose(f.dof_pos, 0.8) for f in seq), "재생 구간에서 클립 자세에 도달 못함"
    print("  ok reaches_clip_pose")


def test_plan_frame_spacing_is_control_period():
    seq = list(publisher.plan_frames(_spec(fps=50.0)))
    dts = [seq[i + 1][0] - seq[i][0] for i in range(len(seq) - 1)]
    assert all(abs(d - 0.02) < 1e-9 for d in dts), sorted(set(round(d, 6) for d in dts))
    print("  ok frame_spacing")


def test_speed_half_doubles_wall_clock():
    """속도 0.5배 -> PLAY 구간 벽시계가 정확히 2배.

    ⚠ (cmd_mode==3, valid==1) 프레임 수를 세면 안 된다 — RAMP_IN/RAMP_OUT 프레임도 같은
    태그를 달고 있고 speed 와 무관하게 길이가 고정이라, PLAY 가 정확히 2배(100->200)여도
    총합은 202->302 = 1.495x 로 희석된다. 시간 차이로 재야 성질이 그대로 드러난다.
    """
    full = list(publisher.plan_frames(_spec(speed=1.0)))
    half = list(publisher.plan_frames(_spec(speed=0.5)))
    play_wall_full = 100 / 50.0 / 1.0     # (f_end-f_start)/fps/speed
    play_wall_half = 100 / 50.0 / 0.5
    assert abs((half[-1][0] - full[-1][0]) - (play_wall_half - play_wall_full)) < 1e-6, \
        (full[-1][0], half[-1][0])
    assert len(half) - len(full) == 100, (len(full), len(half))
    print("  ok speed_half_doubles_wall_clock")


def test_abort_shortens_and_still_ends_at_standby():
    seq = list(publisher.plan_frames(_spec(ramp_in=1.0, ramp_out=1.0), abort_at=1.5))
    assert seq[-1][1].valid == 0
    play_frames = [f for _, f in seq if f.valid == 1 and f.cmd_mode == 3]
    assert np.allclose(play_frames[-1].dof_pos, 0.0, atol=1e-5), play_frames[-1].dof_pos[:3]
    normal = list(publisher.plan_frames(_spec(ramp_in=1.0, ramp_out=1.0)))
    assert seq[-1][0] < normal[-1][0], (seq[-1][0], normal[-1][0])
    print("  ok abort_path")


def test_publisher_writes_bytes_matching_vr_shm_contract():
    """송출 패킷이 vr_shm 계약(276B, magic 0x6702)과 바이트 일치하는지."""
    captured = []

    def fake_write(seq, valid, cmd_mode, base_vel, root_quat, dof_pos, dof_vel):
        captured.append(struct.pack(vr_shm.FMT, vr_shm.MAGIC, int(seq), int(valid),
                                    int(cmd_mode), *base_vel, *root_quat, *dof_pos, *dof_vel))

    p = publisher.Publisher(writer=fake_write, sleeper=lambda _t: None, clock=_FakeClock())
    result = p.run(_spec(ramp_in=0.1, ramp_out=0.1))
    assert result == "completed", result
    assert captured, "아무것도 송출하지 않았다"
    for buf in captured:
        assert len(buf) == 276, len(buf)
        magic, _seq, _v, _m = struct.unpack("<iIii", buf[:16])
        assert magic == 0x6702, hex(magic)
    seqs = [struct.unpack("<iIii", b[:16])[1] for b in captured]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), "seq 가 단조증가하지 않는다"
    print("  ok shm_contract")


class _FakeClock:
    """벽시계 대신 쓰는 가짜 시계 — 호출할 때마다 20 ms 씩 흐른다."""
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        v = self.t
        self.t += 0.02
        return v


def test_publisher_should_abort_hook():
    calls = {"n": 0}

    def should_abort():
        calls["n"] += 1
        return calls["n"] > 10

    p = publisher.Publisher(writer=lambda *a, **k: None, sleeper=lambda _t: None,
                            clock=_FakeClock())
    result = p.run(_spec(ramp_in=0.1, ramp_out=0.1), should_abort=should_abort)
    assert result == "aborted", result
    print("  ok should_abort_hook")


class _JumpClock:
    """벽시계 대신 쓰는 가짜 시계 — 호출할 때마다 step 초씩 흐른다(드리프트 재현용)."""
    def __init__(self, step: float):
        self.t = 0.0
        self.step = step

    def __call__(self):
        v = self.t
        self.t += self.step
        return v


def test_abort_survives_severe_clock_drift():
    """should_abort 판정에 심한 시계 드리프트가 겹쳐도 RAMP_OUT/RELEASE 가 통째로

    사라지면 안 된다. run() 이 재계획 시 벽시계를 다시 읽으면(now = clock()-start),
    느린 hook·GC stall·스케줄러 정체로 now 가 재계획된 시퀀스 전체 길이를 넘어버릴 수
    있고, 그러면 tt>=now 필터가 빈 리스트를 만든다 — 로봇이 재생 도중 자세로 그대로
    멈춘다. t_rel(그 프레임 자신의 예정 시각)로 재계획하면 구조적으로 항상 비지 않는다.
    """
    calls = {"n": 0}

    def should_abort():
        calls["n"] += 1
        return calls["n"] > 5

    emitted = []
    p = publisher.Publisher(writer=lambda *a, **k: None, sleeper=lambda _t: None,
                            clock=_JumpClock(step=0.5))
    result = p.run(_spec(ramp_in=0.1, ramp_out=0.1),
                   on_tick=lambda t, f: emitted.append(f), should_abort=should_abort)
    assert result == "aborted", result
    assert emitted, "아무것도 송출하지 않았다"
    assert emitted[-1].valid == 0, "드리프트 상황에서도 마지막 패킷은 valid=0 이어야 한다"
    assert any(f.cmd_mode == 1 for f in emitted), \
        "드리프트 상황에서도 mode1 패킷이 최소 1개는 나가야 한다"
    print("  ok abort_survives_severe_clock_drift")


def test_emit_drops_repeats_but_never_drops_a_state_transition():
    """lag > CONTROL_DT 인 동안에도 (cmd_mode, valid) 전환 프레임은 절대 드롭되면 안 된다.

    같은 상태가 반복되는 중간 프레임은 시간축을 지키려 버려도 되지만, 이전에 실제로
    송출한 상태와 다른 프레임(RELEASE 의 mode1 전환, 마지막 valid=0)은 lag 와 무관하게
    항상 내보내야 한다 — g_poll_vr 이 valid=0 앞에서 cmd_mode 대입을 건너뛰므로 mode1
    전환은 오직 (cmd_mode=1, valid=1) 홀드 패킷으로만 전달된다.
    """
    spec = _spec(ramp_in=0.1, ramp_out=0.1)
    planned = [f for _, f in publisher.plan_frames(spec)]
    planned_states = {(f.cmd_mode, f.valid) for f in planned}

    emitted = []
    p = publisher.Publisher(writer=lambda *a, **k: None, sleeper=lambda _t: None,
                            clock=_JumpClock(step=0.03))
    result = p.run(spec, on_tick=lambda t, f: emitted.append(f))
    assert result == "completed", result
    assert len(emitted) < len(planned), (len(emitted), len(planned))
    emitted_states = {(f.cmd_mode, f.valid) for f in emitted}
    assert emitted_states == planned_states, (planned_states, emitted_states)
    assert emitted[-1].valid == 0, "마지막 송출 패킷은 valid=0 이어야 한다"
    print("  ok emit_drops_repeats_but_never_drops_a_state_transition")


def main() -> int:
    test_plan_starts_at_standby_and_ends_at_standby()
    test_plan_release_switches_to_mode1_then_invalidates()
    test_plan_reaches_clip_pose_during_play()
    test_plan_frame_spacing_is_control_period()
    test_speed_half_doubles_wall_clock()
    test_abort_shortens_and_still_ends_at_standby()
    test_publisher_writes_bytes_matching_vr_shm_contract()
    test_publisher_should_abort_hook()
    test_abort_survives_severe_clock_drift()
    test_emit_drops_repeats_but_never_drops_a_state_transition()
    print("test_publisher: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
