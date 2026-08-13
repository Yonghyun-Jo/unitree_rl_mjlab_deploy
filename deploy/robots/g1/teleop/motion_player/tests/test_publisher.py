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
    assert seq[0][1].cmd_mode == 1 and seq[0][1].valid == 1     # LEAD_IN(CRITICAL-2)
    # 마지막 재생 프레임(RELEASE 직전)은 standby 로 돌아와 있어야 한다
    play_frames = [f for _, f in seq if f.valid == 1 and f.cmd_mode == 3]
    assert np.allclose(play_frames[-1].dof_pos, 0.0, atol=1e-5), play_frames[-1].dof_pos[:3]
    print("  ok starts_and_ends_at_standby")


def test_plan_leads_in_with_mode1_before_playback_mode():
    """CRITICAL-2: 컨트롤러가 이미 mode2/3 이어도 재앵커/크로스페이드가 걸리도록, 재생은
    항상 cmd_mode=1 홀드로 시작해서 재생모드(2|3)로 넘어가야 한다."""
    seq = [f for _, f in publisher.plan_frames(_spec())]
    modes = [f.cmd_mode for f in seq]
    valids = [f.valid for f in seq]
    assert modes[0] == 1 and valids[0] == 1, (modes[0], valids[0])
    first_playback = next(i for i, m in enumerate(modes) if m in (2, 3))
    assert all(m == 1 for m in modes[:first_playback]), modes[:first_playback]
    assert modes[-1] == 1 and valids[-1] == 0
    assert modes[-2] == 1 and valids[-2] == 1
    print("  ok leads_in_with_mode1")


def test_plan_release_switches_to_mode1_then_invalidates():
    seq = [f for _, f in publisher.plan_frames(_spec())]
    modes = [f.cmd_mode for f in seq]
    # LEAD_IN(선두) 과 RELEASE(말미) 는 mode1, 그 사이(재생 구간)는 전부 mode3 이어야 한다.
    first_pb = next(i for i, m in enumerate(modes) if m == 3)
    last_pb = len(modes) - 1 - next(i for i, m in enumerate(reversed(modes)) if m == 3)
    assert all(m == 1 for m in modes[:first_pb]), "LEAD_IN 은 전부 mode1 이어야 한다"
    assert all(m == 3 for m in modes[first_pb:last_pb + 1]), "재생 구간은 mode3 이어야 한다"
    assert all(m == 1 for m in modes[last_pb + 1:]), "RELEASE 는 전부 mode1 이어야 한다"
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


def test_velocity_ref_continuous_across_ramp_boundaries_at_half_speed():
    """IMPORTANT-3: RAMP_IN->PLAY, PLAY->RAMP_OUT 경계에서 dof_vel 이 계단으로 튀면 안 된다.

    ramp_frame 은 a*clip_qd 를, play_frame 은 clip_qd*speed 를 내보내는데 speed != 1.0 이면
    경계(a=1)에서 a*clip_qd != clip_qd*speed 로 어긋난다 — ramp_frame 도 speed 를 반영해야
    한다. speed=1.0 에서는 버그가 안 보이므로 0.5 로 검증한다.
    """
    spec = _spec(speed=0.5, mode=3, ramp_in=0.2, ramp_out=0.2, T=200, fps=50.0)
    playback = [f for _, f in publisher.plan_frames(spec) if f.cmd_mode == spec.mode and f.valid == 1]

    n_in = max(1, int(round(spec.ramp_in_s / publisher.CONTROL_DT)))     # RAMP_IN = n_in+1 프레임
    n_clip = spec.f_end - spec.f_start
    play_wall_s = (n_clip / spec.clip.fps) / spec.speed
    n_play = max(1, int(round(play_wall_s / publisher.CONTROL_DT)))

    last_ramp_in = playback[n_in]
    first_play = playback[n_in + 1]
    last_play = playback[n_in + n_play]
    first_ramp_out = playback[n_in + n_play + 1]

    assert np.allclose(last_ramp_in.dof_vel, first_play.dof_vel, atol=1e-6), \
        (last_ramp_in.dof_vel[:3], first_play.dof_vel[:3])
    assert np.allclose(last_play.dof_vel, first_ramp_out.dof_vel, atol=1e-6), \
        (last_play.dof_vel[:3], first_ramp_out.dof_vel[:3])
    print("  ok velocity_ref_continuous_at_half_speed")


def test_abort_shortens_and_still_ends_at_standby():
    # LEAD_IN(LEAD_IN_HOLD_S) + RAMP_IN(1.0s) 을 넘겨 PLAY 구간 0.2s 지점에서 중단시켜야
    # 한다 — LEAD_IN 안에서 중단하면 아직 clip 을 한 번도 참조하지 않은 상태라
    # play_frames(cmd_mode==3) 가 통째로 비어 아래 [-1] 참조가 IndexError 로 죽는다.
    abort_at = publisher.LEAD_IN_HOLD_S + 1.0 + 0.2
    seq = list(publisher.plan_frames(_spec(ramp_in=1.0, ramp_out=1.0), abort_at=abort_at))
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
        # LEAD_IN_HOLD_S(=2.0s -> 100 프레임)을 넘어 RAMP_IN/PLAY 구간에서 중단시켜야
        # 한다 — LEAD_IN 안에서 중단하면 이미 standby/mode1 이라 plan_frames 가 즉시
        # valid=0 한 프레임만 내고 끝나버려 이 훅이 원래 겨냥하던 재계획 경로를 안 탄다.
        return calls["n"] > 120

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
        # LEAD_IN_HOLD_S(=2.0s -> 100 프레임)을 넘어 RAMP_IN/PLAY 구간에서 중단시켜야
        # 드리프트 재계획(RAMP_OUT/RELEASE)이 실제로 걸린다 — LEAD_IN 안에서 중단하면
        # 이미 mode1 이라 재계획이 valid=0 한 프레임으로 끝나버리고, emitted 도 전부
        # cmd_mode==1 이라 아래 mode1 검증이 (버그가 있어도) 트리비얼하게 통과해버린다.
        return calls["n"] > 130

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


def test_abort_during_ramp_in_reverses_instead_of_completing():
    """RAMP_IN 도중 중단하면 클립 자세까지 완주한 뒤 되돌아오면 안 된다.

    조작자가 abort 를 누른 건 뭔가 이상해서다 — 그런데 로봇이 클립 자세로 완주까지
    한 뒤에야 되돌아온다면 "멈추라고 눌렀는데 계속 나아갔다"가 되어 그 컨트롤을
    신뢰할 수 없게 만든다(즉각 정지가 필요하면 별도의 하드 E-stop 이 있다 — 이건
    "더 나아가지 않고 되돌아오기"만 보장하면 된다).
    """
    T = 200
    fps = 50.0
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
    spec = publisher.PlaybackSpec(clip=clip, profile=profile, f_start=0, f_end=100,
                                  mode=3, speed=1.0, base_vel_kind="zero",
                                  manual_bv=(0.0, 0.0, 0.0), ramp_in_s=1.0, ramp_out_s=1.0)

    seq = list(publisher.plan_frames(spec, abort_at=0.4))

    # (a) 클립 자세(joint_pos[f_start])에 절대 도달하면 안 된다 — 오늘 동작(완주 후
    #     되돌아옴)이라면 반드시 도달하므로 이 assert 가 구식 동작을 가려낸다.
    assert not any(np.allclose(f.dof_pos, clip.joint_pos[spec.f_start], atol=1e-5)
                   for _, f in seq), "RAMP_IN 중단 후에도 클립 자세에 도달했다"

    # RAMP_IN/RAMP_OUT 경계 = abort_at 시각 그 자체(RAMP_OUT 은 정확히 t=abort_at 에서
    # 시작한다 — plan_frames 의 t 는 단조증가이므로 이 경계로 명확히 나뉜다). dof_pos
    # 곡선 모양(증가->감소)으로 경계를 추정하면 바로 이 결함(첫 RAMP_OUT 프레임이
    # 계단으로 인해 마지막 RAMP_IN 프레임보다 값이 더 큰 경우) 때문에 오탐한다.
    before = [f for t, f in seq if t < 0.4 - 1e-9]
    after = [f for t, f in seq if t >= 0.4 - 1e-9]
    last_ramp_in = before[-1]
    first_ramp_out = after[0]

    # (b) RAMP_OUT 의 첫 프레임은 RAMP_IN 의 마지막 프레임과 연속이어야 한다(계단 없음).
    assert np.allclose(first_ramp_out.dof_pos, last_ramp_in.dof_pos, atol=1e-6), \
        (last_ramp_in.dof_pos[:3], first_ramp_out.dof_pos[:3])

    # (c) 시퀀스는 여전히 cmd_mode=1 프레임 다음에 valid=0 프레임으로 끝나야 한다.
    modes = [f.cmd_mode for _, f in seq]
    assert 1 in modes, "RELEASE 의 mode1 패킷이 없다"
    assert seq[-1][1].cmd_mode == 1 and seq[-1][1].valid == 0
    assert seq[-2][1].cmd_mode == 1 and seq[-2][1].valid == 1
    print("  ok abort_during_ramp_in_reverses_instead_of_completing")


def main() -> int:
    test_plan_starts_at_standby_and_ends_at_standby()
    test_plan_leads_in_with_mode1_before_playback_mode()
    test_plan_release_switches_to_mode1_then_invalidates()
    test_plan_reaches_clip_pose_during_play()
    test_plan_frame_spacing_is_control_period()
    test_speed_half_doubles_wall_clock()
    test_velocity_ref_continuous_across_ramp_boundaries_at_half_speed()
    test_abort_shortens_and_still_ends_at_standby()
    test_publisher_writes_bytes_matching_vr_shm_contract()
    test_publisher_should_abort_hook()
    test_abort_survives_severe_clock_drift()
    test_emit_drops_repeats_but_never_drops_a_state_transition()
    test_abort_during_ramp_in_reverses_instead_of_completing()
    print("test_publisher: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
