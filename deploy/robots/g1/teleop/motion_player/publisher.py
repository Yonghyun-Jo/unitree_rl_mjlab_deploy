"""Publisher — 램프인/재생/램프아웃/해제 상태기계와 50 Hz 송출.

시퀀스 계산(plan_frames, 순수)과 실행(Publisher.run, 시간·shm)을 나눈다.
로봇 없이 전체 시퀀스를 검증하기 위해서다.

⚠ 종료 프로토콜을 지키는 것이 이 모듈의 존재 이유다:
  - C++ clear_vr() 은 q_ref 를 standby 로 계단 스냅한다 -> RAMP_OUT 이 먼저 끝나야 valid=0.
  - C++ 재앵커/크로스페이드는 모드 전환에만 발동한다 -> 끝에 cmd_mode=1 패킷을 반드시 보낸다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterator

import numpy as np

from .clips import ClipData, DeployProfile
from .frames import RefFrame, play_frame, ramp_frame, smoothstep

LEAD_IN_HOLD_S = 0.3
RAMP_OUT_S = 1.5
ABORT_RAMP_OUT_S = 0.8
RELEASE_HOLD_S = 0.5
CONTROL_DT = 0.02          # 50 Hz — 정책 step_dt 와 동일


@dataclass
class PlaybackSpec:
    clip: ClipData
    profile: DeployProfile
    f_start: int
    f_end: int
    mode: int
    speed: float
    base_vel_kind: str          # "clip" | "zero" | "manual"
    manual_bv: tuple[float, float, float]
    ramp_in_s: float
    ramp_out_s: float = RAMP_OUT_S


def _standby_frame(spec: PlaybackSpec, cmd_mode: int, valid: int) -> RefFrame:
    return RefFrame(cmd_mode=cmd_mode, valid=valid, base_vel=(0.0, 0.0, 0.0),
                    root_quat=(1.0, 0.0, 0.0, 0.0),
                    dof_pos=spec.profile.standby.astype(np.float32),
                    dof_vel=np.zeros(29, dtype=np.float32))


def plan_frames(spec: PlaybackSpec, abort_at: float | None = None
                ) -> Iterator[tuple[float, RefFrame]]:
    """(경과 벽시계 초, 프레임) 시퀀스. 시간을 읽지 않는 순수 생성기.

    abort_at 이 주어지면 그 시각에 재생을 끊고 짧은 램프아웃으로 넘어간다.
    """
    t = 0.0

    # --- LEAD_IN: cmd_mode=1 홀드 ---
    # 컨트롤러가 이미 mode2/3 였다면(조작자 키·GUI·이전 생산자) g_cmd_mode 가 안 바뀌어
    # C++ 재앵커(init_quat)와 크로스페이드가 아예 안 걸린 채로 RAMP_IN 이 시작해버린다.
    # 재생 시작 전 반드시 mode1 을 한 번 거쳐 전환을 강제한다(spec §2.7: "매 재생을
    # mode1 → 재생모드 → mode1 로 왕복").
    n_lead = max(1, int(round(LEAD_IN_HOLD_S / CONTROL_DT)))
    for _ in range(n_lead):
        if abort_at is not None and t >= abort_at:
            # 아직 아무것도 움직이지 않았다(이미 standby/mode1) — 곧장 종료한다.
            yield t, _standby_frame(spec, cmd_mode=1, valid=0)
            return
        yield t, _standby_frame(spec, cmd_mode=1, valid=1)
        t += CONTROL_DT

    # --- RAMP_IN: standby -> clip[f_start] ---
    n_in = max(1, int(round(spec.ramp_in_s / CONTROL_DT)))
    s_in = 1.0                      # 램프인이 실제로 도달한 진행도 (1.0 = 완주)
    last_s = 0.0                    # 마지막으로 실제 송출한 프레임의 진행도
    for i in range(n_in + 1):
        if abort_at is not None and t >= abort_at:
            s_in = last_s
            break
        yield t, ramp_frame(spec.clip, spec.profile, spec.f_start, i / n_in,
                            spec.mode, (0.0, 0.0, 0.0), "in",
                            f_entry=spec.f_start, speed=spec.speed)
        last_s = i / n_in
        t += CONTROL_DT

    if s_in < 1.0:
        # 램프인 도중 중단 — 클립 자세까지 완주시키지 않고 이미 도달한 진행도에서
        # 그대로 되돌린다("멈추라고 눌렀는데 계속 나아갔다"를 막는다). PLAY 는 건너뛴다.
        aborted = True
        f_last = spec.f_start
        a_scale = smoothstep(s_in)
    else:
        # --- PLAY: 클립 시간축을 speed 로 훑는다 ---
        play_start = t
        n_clip = spec.f_end - spec.f_start
        play_wall_s = (n_clip / spec.clip.fps) / max(1e-6, spec.speed)
        n_play = max(1, int(round(play_wall_s / CONTROL_DT)))
        f_last = spec.f_start
        for i in range(n_play):
            if abort_at is not None and t >= abort_at:
                break
            clip_elapsed = (i * CONTROL_DT) * spec.speed
            f_last = min(spec.f_end - 1, spec.f_start + int(round(clip_elapsed * spec.clip.fps)))
            yield t, play_frame(spec.clip, f_last, spec.speed, spec.mode,
                                spec.base_vel_kind, spec.manual_bv, f_entry=spec.f_start)
            t += CONTROL_DT
        aborted = abort_at is not None and t < play_start + play_wall_s - 1e-9
        a_scale = 1.0

    # --- RAMP_OUT: clip[f_last] -> standby (a_scale<1 이면 램프인이 실제로 도달한
    #     진행도에서 이어서 되돌아온다 — 클립 자세로 완주했다 되돌아오는 계단을 막는다) ---
    out_s = ABORT_RAMP_OUT_S if aborted else spec.ramp_out_s
    n_out = max(1, int(round(out_s / CONTROL_DT)))
    for i in range(n_out + 1):
        yield t, ramp_frame(spec.clip, spec.profile, f_last, i / n_out,
                            spec.mode, (0.0, 0.0, 0.0), "out",
                            f_entry=spec.f_start, speed=spec.speed, a_scale=a_scale)
        t += CONTROL_DT

    # --- RELEASE: mode1 로 전환(재앵커/크로스페이드 유발) -> 유지 -> valid=0 ---
    for _ in range(max(1, int(round(RELEASE_HOLD_S / CONTROL_DT)))):
        yield t, _standby_frame(spec, cmd_mode=1, valid=1)
        t += CONTROL_DT
    yield t, _standby_frame(spec, cmd_mode=1, valid=0)


class Publisher:
    """plan_frames 를 실제 시간축에 태워 shm 으로 내보낸다."""

    def __init__(self, writer: Callable | None = None,
                 sleeper: Callable[[float], None] | None = None,
                 clock: Callable[[], float] | None = None):
        if writer is None:
            import vr_shm                        # teleop/ 에 있음 (sys.path 로 들어옴)
            writer = vr_shm.write
        self._write = writer
        self._sleep = sleeper if sleeper is not None else time.sleep
        self._clock = clock if clock is not None else time.perf_counter
        self._seq = 0
        self._last_state: tuple[int, int] | None = None

    def run(self, spec: PlaybackSpec,
            on_tick: Callable[[float, RefFrame], None] | None = None,
            should_abort: Callable[[], bool] | None = None) -> str:
        """재생. 반환값 "completed" | "aborted".

        타이밍은 sleep 누적이 아니라 절대시각 데드라인이다. 밀리면 프레임을 떨어뜨리고
        시간축을 지킨다 (참조가 느려지는 것보다 낫다).
        """
        start = self._clock()
        aborted = False
        gen = plan_frames(spec)
        pending: list[tuple[float, RefFrame]] = []
        for t_rel, frame in gen:
            if not aborted and should_abort is not None and should_abort():
                aborted = True
                # abort_at 은 반드시 t_rel(이 프레임 자신의 "예정" 시각) 이어야 한다 —
                # 벽시계를 다시 읽으면(now = clock()-start) should_abort 판정 시점의
                # 드리프트(느린 hook, GC stall, 스케줄러 정체 — 바로 아래 _emit 의 드롭
                # 규칙이 상정하는 그 상황)로 now 가 재계획된 시퀀스 전체 길이를 넘어버릴
                # 수 있고, 그러면 tt>=now 필터가 빈 리스트가 되어 RAMP_OUT/RELEASE 가
                # 통째로 사라진다(로봇이 재생 도중 자세로 그대로 멈춤). t_rel 은
                # plan_frames 자신의 스케줄 위에 있는 값이라 tt>=t_rel 결과가 구조적으로
                # 항상 비지 않는다 — "개선"한답시고 다시 벽시계로 되돌리지 말 것.
                pending = [(tt, ff) for tt, ff in plan_frames(spec, abort_at=t_rel)
                          if tt >= t_rel]
                break
            self._emit(t_rel, frame, start, on_tick)
        for t_rel, frame in pending:
            self._emit(t_rel, frame, start, on_tick)
        return "aborted" if aborted else "completed"

    def _emit(self, t_rel: float, frame: RefFrame, start: float, on_tick) -> None:
        deadline = start + t_rel
        lag = self._clock() - deadline
        state = (frame.cmd_mode, frame.valid)
        # (cmd_mode, valid) 가 마지막으로 실제 송출한 상태와 다르면(=상태 전환) lag 와
        # 무관하게 반드시 내보낸다. C++ g_poll_vr 은 valid=0 패킷에서 cmd_mode 대입 전에
        # 반환하므로, mode1 전환은 오직 (cmd_mode=1, valid=1) 홀드 패킷으로만 전달된다 —
        # 이 패킷들이 통째로 드롭되면 컨트롤러가 override 만 풀리고 mode1 로 못 넘어간다.
        is_transition = state != self._last_state
        if lag < -1e-6:
            self._sleep(-lag)
        elif lag > CONTROL_DT and not is_transition:
            return                        # 반복되는 중간 프레임만 버려서 시간축을 지킨다
        self._seq += 1
        self._write(self._seq, frame.valid, frame.cmd_mode, list(frame.base_vel),
                    list(frame.root_quat), frame.dof_pos.tolist(), frame.dof_vel.tolist())
        self._last_state = state
        if on_tick is not None:
            on_tick(t_rel, frame)
