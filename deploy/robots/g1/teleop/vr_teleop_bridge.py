#!/usr/bin/env python3
"""Live PICO VR -> gmt_multihead policy bridge (com1, gmr env).

Replaces vr_replay.py's clip source with the LIVE PICO stream:

  노트북 pico_publisher_udp.py --UDP--> [this: bind :5556]   (--transport zmq 로 구 publisher 롤백)
    -> GMR("xrobot","unitree_g1").retarget(body[24])  -> qpos[36]
       root_quat = qpos[3:7] (wxyz),  dof_pos = qpos[7:36] (== robot_spec JOINT_ORDER, no remap)
    -> dof_vel  = finite-diff(dof_pos) + EMA  (policy uses q̇_ref; NOT published by laptop)
    -> base_vel = controller thumbstick,  cmd_mode = buttons
    -> vr_shm.write(...) -> /dev/shm/g1_vr_ref  -> g1_ctrl (gmt_multihead) -> unitree_mujoco

cmd_mode masks in C++: mode2=상체(팔·waist), mode3=전신(다리 포함). ONE bridge, full VrRef.

⚠ Runs on the LAPTOP / a Linux PC near com1 (NOT the robot onboard — GMR stays off the
  control host). Set up the env ONCE, then run from the repo root:
    bash deploy/robots/g1/teleop/setup_teleop.sh          # venv + GMR + g1 meshes
    .venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py   # 기본 mode1(안전) 시작, 버튼 X/Y/B로 승격
  (on com1 the existing `gmr` conda env also works.)
  (run alongside: unitree_mujoco  +  g1_ctrl --network=lo  [f->stand, m->policy])
"""
from __future__ import annotations

import argparse
import collections
import json
import signal
import threading
import time

import numpy as np
try:
    import zmq            # zmq transport 전용. 로컬/UDP만 쓰면 미설치여도 됨.
except ImportError:
    zmq = None
from scipy.spatial.transform import Rotation

import vr_shm  # same teleop/ dir
import estop_shm            # /dev/shm/g1_estop 하트비트 (same teleop/ dir)
from teleop_safety import (  # watchdog(stale/rate) + E-stop(A latch), same teleop/ dir
    SafetyMonitor,
    estop_flag,
    resolve_estop_arming,
)
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


class ZmqReceiver:
    """롤백/비교용 ZMQ SUB 래퍼. UdpReceiver와 동일 인터페이스(latest/age_ms/close).

    com1이 bind, 노트북 PUB이 connect(기존 토폴로지). latest()는 NOBLOCK drain으로
    최신 JSON 프레임 dict 반환(없으면 None) — pace는 호출측 loop sleep이 담당(UDP와 통일).
    프레임 shape를 pico_wire(UDP)와 맞추기 위해 누락된 컨트롤러 버튼 키를 기본값으로 채운다
    (구 JSON publisher가 right.primary/menu를 안 보내면 SafetyMonitor 접근이 KeyError 나므로)."""

    def __init__(self, port):
        if zmq is None:
            raise SystemExit("pyzmq 미설치: --transport zmq 쓰려면 `pip install pyzmq`. (local/udp는 불필요)")
        ctx = zmq.Context.instance()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.RCVHWM, 4)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.bind(f"tcp://*:{port}")
        self._last_rx = None

    def latest(self):
        msg = _drain_latest(self.sub)          # NOBLOCK drain → 최신 1개 (or None)
        if msg is None:
            return None
        self._last_rx = time.monotonic()
        f = json.loads(msg[1].decode())
        ctrls = f.setdefault("controllers", {})
        for side in ("left", "right"):
            c = ctrls.setdefault(side, {})
            for k in ("primary", "secondary", "menu", "axis_click"):
                c.setdefault(k, False)
            c.setdefault("grip", 0.0)
            c.setdefault("axis", [0.0, 0.0])
        return f

    def age_ms(self):
        if self._last_rx is None:
            return float("inf")
        return (time.monotonic() - self._last_rx) * 1000.0

    def close(self):
        self.sub.close(0)


def _nlerp(q0, q1, a):
    """Normalized-lerp between wxyz quats by fraction a, with hemisphere alignment."""
    q0 = np.asarray(q0, dtype=np.float32)
    q1 = np.asarray(q1, dtype=np.float32)
    if float(np.dot(q0, q1)) < 0.0:
        q1 = -q1                                       # shortest arc (avoid ±q flip)
    q = q0 + a * (q1 - q0)
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-8 else q0


class _OneEuro:
    """Vector One-Euro filter (Casiez 2012) on a fixed-dt timeline.

    Adaptive low-pass for interactive/mocap signals: heavy smoothing when slow
    (kills jitter spikes), low lag when fast. Returns (x_hat, dx_hat) where dx_hat
    is the smoothed velocity (reused for gap extrapolation).
    """

    def __init__(self, dt, min_cutoff=1.0, beta=0.7, d_cutoff=1.0):
        self.dt = float(dt)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = None
        self.dx_prev = None

    def reset(self):
        self.x_prev = None
        self.dx_prev = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / self.dt)

    def __call__(self, x):
        x = np.asarray(x, dtype=np.float32)
        if self.x_prev is None:
            self.x_prev = x.copy()
            self.dx_prev = np.zeros_like(x)
            return x, np.zeros_like(x)
        dx = (x - self.x_prev) / self.dt
        a_d = self._alpha(self.d_cutoff)                        # scalar
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)   # per-element
        a = self._alpha(cutoff)                                 # per-element
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat, dx_hat


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--transport", choices=["udp", "zmq", "local"], default="udp",
                    help="pose 스트림 transport. udp(기본, 손실=스킵; pico_wire/udp_receiver) / "
                         "zmq(TCP·구 publisher 롤백용) / "
                         "local(온보드 co-located: xrt 직접읽기, 네트워크 없음). "
                         "기본이 udp인 근거는 2026-08-13 동일 링크 A/B 실측: 최대 무수신 "
                         "1173ms(zmq) → 164ms(udp), 워치독 문턱 200ms 초과 3.0초 → 0초.")
    ap.add_argument("--mode", type=int, default=1, choices=[1, 2, 3],
                    help="시작 cmd_mode (기본 1=안전 full-auto). 실행 중 버튼이 override: X→1/Y→2/B→3.")
    ap.add_argument("--ema", type=float, default=0.5,
                    help="dof_vel EMA alpha (0..1, higher=less smoothing). "
                         "⚠ --no-smooth 일 때만 유효 — 기본(--smooth)에서는 출력 스레드가 One-Euro dx_hat 을 "
                         "쓰므로 이 값은 무시된다(EMA 코드에 도달하지 않음).")
    # 풀스틱 캡. 배포 하드캡(State_Mimic.cpp KB_MAXVX/VY/W = 3.0/1.5/2.0, tools/gui_shm.py 와 동일)
    # 안쪽으로 vx/vy 는 여유를 두고, wz 만 하드캡과 같다. ⚠ 하드캡보다 크게 줘도 C++ 에서 잘린다.
    ap.add_argument("--vx", type=float, default=2.5, help="base_vel vx cap (m/s) at full stick")
    ap.add_argument("--vy", type=float, default=0.8, help="base_vel vy cap")   # 학습 봉투 vy_range=(-0.8, 0.8)
    ap.add_argument("--wz", type=float, default=2.0, help="base_vel wz cap (rad/s)")
    ap.add_argument("--no-body-transform", action="store_true",
                    help="skip Unity->RH + quat reorder (only if publisher already sends processed body)")
    ap.add_argument("--grip-enable", action="store_true",
                    help="deadman: grip 눌러야 텔레옵 활성. 놓으면 안전 mode1 복귀. 실제/deploy 권장.")
    ap.add_argument("--arm-estop", action=argparse.BooleanOptionalAction, default=None,
                    help="하드 E-stop 채널(/dev/shm/g1_estop) 무장. 기본=auto(transport=local일 때만 무장). "
                         "노트북 PICO로 실로봇을 돌릴 때(udp/zmq + 제어PC에서 bridge·g1_ctrl 동거) "
                         "--arm-estop 으로 명시 무장 → A버튼 래치·브릿지 死가 g1_ctrl 강제 Passive로 이어짐.")
    ap.add_argument("--estop-on-watchdog", action=argparse.BooleanOptionalAction, default=None,
                    help="워치독(통신 스톨/저수신)도 하드 Passive로 올릴지. 기본=auto(local=True, 네트워크=False). "
                         "네트워크에서 켜면 WiFi hiccup 하나가 곧장 주저앉기(damping)가 되니 주의.")
    ap.add_argument("--fallback-clip", action="store_true",
                    help="VR 끊김 시 clip 재생(valid=0, 테스트/데모용). 기본=안전 mode1 fallback(clip 무시).")
    ap.add_argument("--height", type=float, default=None, help="operator height (m) for GMR scaling")
    ap.add_argument("--gmr-iter", type=int, default=10,
                    help="GMR IK max iterations per stage (TWIST2 스톡=10). warm-start(mink.Configuration "
                         "프레임간 유지)+0.001 조기수렴이라 대부분 몇 iter만 씀 → 10이어도 저렴. "
                         "지연은 iter가 아니라 GMR 프로세스 분리로 잡는다(TWIST2 레시피).")
    # ── 예측 스무딩. ⚠ 기본 ON 이고 실배포(co-located)에도 그대로 걸린다.
    #    고정 rate 출력 루프 + One-Euro(적응형 저역통과) + gap 속도 extrapolation + slew-rate limit.
    #    근거: TWIST2(co-locate·hot-loop 무보간), h2_compact(extrapolation for latency jitter), One-Euro(VR/mocap 표준).
    #
    #    비용(오프라인 실측, A=0.5rad 정현파, 기본값 mincut 1.0/beta 0.7 기준):
    #      느린 동작(0.25~1Hz) 위상지연 41~58ms, 빠른 reach 75ms(2Hz 부근은 slew limit 지배).
    #      명령 사슬(손→관절 명령) 지연의 45~60%가 여기서 나온다. GMR 리타게팅은 1.6ms(2%)로 무시 가능.
    #      → 체감 지연을 줄이려면 GMR 이 아니라 --smooth-mincut / --smooth-max-jointvel 를 완화한다.
    #
    #    ⚠ --no-smooth 로 끄는 것은 권장하지 않는다. 두 가지가 같이 사라진다:
    #      (1) slew-rate limit 이 One-Euro 앞단(:342)에 있어 outlier clamp 가 통째로 없어진다
    #          (트래커 dropout·GMR IK 튐·quat flip 이 물리한계 없이 모터로 나간다).
    #      (2) shm 의 dof_vel 정의가 바뀐다(smooth=One-Euro dx_hat clamp / no-smooth=finite-diff+EMA).
    #          이 값은 정책 obs 의 qd_ref 로 들어가므로(State_Mimic.cpp) 정책이 다른 스케일의 입력을 받는다.
    ap.add_argument("--smooth", action=argparse.BooleanOptionalAction, default=True,
                    help="예측 스무딩(고정 rate + One-Euro + gap extrapolation + slew-rate limit). "
                         "기본 ON — 실배포 안전장치(outlier 차단)로도 유용. 끄려면 --no-smooth.")
    ap.add_argument("--smooth-hz", type=float, default=50.0,
                    help="스무딩 출력 rate(Hz). 제어 루프(50Hz)에 맞춤.")
    ap.add_argument("--smooth-mincut", type=float, default=1.0,
                    help="One-Euro min_cutoff(Hz). 낮을수록 정지 시 강한 스무딩(튐 억제), 높을수록 반응 빠름.")
    ap.add_argument("--smooth-beta", type=float, default=0.7,
                    help="One-Euro beta(속도 계수). 높을수록 빠른 모션에서 lag↓.")
    ap.add_argument("--smooth-maxgap", type=int, default=4,
                    help="손실 gap에서 속도 extrapolation 최대 tick 수(이후 hold). run-away 방지 상한.")
    ap.add_argument("--diag", action=argparse.BooleanOptionalAction, default=True,
                    help="1초마다 [diag] 줄 출력: output tick 간격 p50/p95/max + overrun 수 + GMR 소요. "
                         "'50Hz에 밀리는가'를 눈으로 확인하는 유일한 수단. 기본 ON(비용 무시 가능).")
    ap.add_argument("--log", type=str, default=None, metavar="PATH",
                    help="output tick 단위 진단 CSV 기록(사후 분석용). 예: --log /tmp/teleop_$(date +%%H%%M).csv "
                         "컬럼: t_s,tick_dt_ms,active,have_new,gap,cmd_mode,gmr_ms,in_rate_hz,in_age_ms,reason")
    ap.add_argument("--smooth-max-jointvel", type=float, default=12.0,
                    help="slew-rate limit: reference가 tick당 이동 가능한 최대 관절속도(rad/s). outlier/garbage "
                         "프레임(트래커 dropout·GMR IK 튐·quat flip)을 물리한계로 clamp → 주기적 엄청난 튐 차단.")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, _sigterm)

    print("[bridge] loading GMR (xrobot -> unitree_g1) ...")
    gmr = GeneralMotionRetargeting("xrobot", "unitree_g1", actual_human_height=args.height)
    gmr.max_iter = args.gmr_iter   # TWIST2 스톡=10. warm-start(mink.Configuration 프레임간 유지)+0.001
                                   # 조기수렴 → 상한 10이어도 실사용 iter 적음. CPU 경합은 프로세스 분리로.
    print(f"[bridge] GMR ready. (IK max_iter={gmr.max_iter})")

    # ── 레퍼런스 발 world-z ([z_L, z_R]) 원천 ────────────────────────────────────────
    # mode3(전신)은 학습 cfg 에 FOOT_GEN 이 «없다» → mjlab calc_ref_foot_height 가 생성기가
    # 아니라 «레퍼런스 발 world-z» 를 obs 로 먹인다. 그래서 텔레옵도 그 값을 같이 보내야
    # 학습과 같은 obs 가 된다. GMR 은 retarget() 안에서 이미 FK 를 돌려 놓으므로
    # configuration.data.xpos 를 읽는 비용은 0 이다 (offset_to_ground=True → 지면 기준).
    # 이름으로 찾는다 — 인덱스로 박으면 GMR 에셋이 바뀔 때 조용히 틀어진다.
    try:
        import mujoco as _mj
        _foot_bids = [_mj.mj_name2id(gmr.model, _mj.mjtObj.mjOBJ_BODY, nm)
                      for nm in ("left_ankle_roll_link", "right_ankle_roll_link")]
        if any(b < 0 for b in _foot_bids):
            _foot_bids = None
    except Exception as _e:                                    # noqa: BLE001
        print(f"[bridge] ⚠ 발-z FK 준비 실패({_e})")
        _foot_bids = None
    if _foot_bids is None:
        print("[bridge] ⚠ ankle_roll body 를 못 찾음 -> 발-z 미전송 "
              "(mode3 은 stance 상수로 폴백 = 학습과 불일치)")
    else:
        print(f"[bridge] 발-z FK 준비 완료 (body ids {_foot_bids}, [L, R])")

    def _gmr_foot_z():
        """직전 retarget() 결과의 발 world-z [z_L, z_R]. 못 구하면 None."""
        if _foot_bids is None:
            return None
        xp = gmr.configuration.data.xpos
        return [float(xp[_foot_bids[0], 2]), float(xp[_foot_bids[1], 2])]

    # ── pose 스트림 수신자 (transport 교체 가능). udp=기본(WAN/LAN 공통), zmq=롤백.
    if args.transport == "udp":
        from udp_receiver import UdpReceiver
        rx = UdpReceiver(port=args.port)
        print(f"[bridge] transport=UDP  bind *:{args.port}/udp  "
              f"default mode={args.mode}  grip_enable={args.grip_enable}")
    elif args.transport == "local":
        import xrobotoolkit_sdk as xrt        # 온보드 로컬: PC-Service에 붙음(xrt.init 인자없음)
        from xrt_receiver import XrtReceiver
        xrt.init()
        rx = XrtReceiver(xrt)
        print(f"[bridge] transport=LOCAL  xrt.init() (로컬 PC-Service)  "
              f"default mode={args.mode}  grip_enable={args.grip_enable}")
    else:
        rx = ZmqReceiver(port=args.port)
        print(f"[bridge] transport=ZMQ  bind tcp://*:{args.port}  "
              f"default mode={args.mode}  grip_enable={args.grip_enable}")

    # ── 안전 감시: 워치독(stale/저수신 → mode1) + E-stop(우측 A 래치, 우측 menu 1s 홀드 해제).
    safety = SafetyMonitor(stale_ms=200, min_rate_hz=30)
    prev_reason = None
    POLL_DT = 0.003   # latest()가 논블록 → 사이클당 소량 sleep으로 busy-poll(100% CPU) 방지

    prev_dof = None
    dof_vel = np.zeros(29, dtype=np.float32)
    user_mode = args.mode                 # 유저 의도 teleop 모드(2/3, 버튼으로 변경). 출력 mode와 분리.
    seq_out = 0
    estop_seq = 0
    # 하드 E-stop 무장: 기본 auto(local만) — sim2sim(zmq/udp)은 기존대로 soft mode1.
    # 노트북 PICO + 실로봇은 --arm-estop 으로 명시 무장(워치독은 auto로 soft 유지).
    estop_armed, estop_on_watchdog = resolve_estop_arming(
        args.transport, args.arm_estop, args.estop_on_watchdog)
    print("[bridge] hard E-stop: "
          + (f"ARMED (A버튼·브릿지死 → 강제 Passive, watchdog->Passive={estop_on_watchdog})"
             if estop_armed else "DISARMED (soft mode1 fallback만; 무장하려면 --arm-estop)"))
    last_t = None
    # status
    n = 0
    t_report = time.monotonic()
    proc = 0

    # ── 예측 스무딩(opt-in): receiver는 최신 target만 _shared에 갱신, output thread가 고정 rate로
    #    One-Euro+extrapolation 적용해 shm write. smooth off면 _shared/thread 미사용(기존 경로 그대로).
    _lock = threading.Lock()
    _shared = {"seq": 0, "active": False, "cmd_mode": user_mode,
               "stick": [0.0, 0.0, 0.0], "quat": list(IDENTITY_QUAT), "dof": list(ZERO29),
               "foot_z": None}
    _stop = threading.Event()

    # ── 진단 계측(S1): output tick 간격 / overrun / GMR 소요. deque append 는 GIL 하에서 원자적이라
    #    RT thread(output_loop)와 report()가 락 없이 주고받아도 안전(진단 목적엔 충분).
    _tick_ms = collections.deque(maxlen=8000)     # ~160s @50Hz
    _gmr_ms = collections.deque(maxlen=8000)
    _over = [0]                                   # output tick overrun 누적(리스트=가변 참조)
    _diag = {"gmr_ms": 0.0, "rate_hz": 0.0, "age_ms": 0.0, "reason": "init"}
    _log_f = None
    if args.log:
        _log_f = open(args.log, "w", buffering=1 << 16)
        _log_f.write("t_s,tick_dt_ms,active,have_new,gap,cmd_mode,gmr_ms,in_rate_hz,in_age_ms,reason\n")
        print(f"[bridge] diag CSV -> {args.log}")

    def _pct(vals, q):
        if not vals:
            return float("nan")
        s = sorted(vals)
        return s[min(len(s) - 1, int(q * len(s)))]

    def _push_target(cmd_mode, stick, quat, dof, foot_z=None):
        with _lock:
            _shared["seq"] += 1
            _shared["active"] = True
            _shared["cmd_mode"] = cmd_mode
            _shared["stick"] = list(stick)
            _shared["quat"] = list(quat)
            _shared["dof"] = dof.tolist() if hasattr(dof, "tolist") else list(dof)
            _shared["foot_z"] = None if foot_z is None else list(foot_z)

    def _push_inactive(cmd_mode):
        with _lock:
            _shared["active"] = False
            _shared["cmd_mode"] = cmd_mode

    def output_loop():
        out_dt = 1.0 / args.smooth_hz
        oe = _OneEuro(out_dt, args.smooth_mincut, args.smooth_beta)
        cur = None
        cur_foot_z = None
        cur_vel = np.zeros(29, dtype=np.float32)
        cur_quat = np.asarray(IDENTITY_QUAT, dtype=np.float32)
        quat_a = 1.0 - float(np.exp(-out_dt / 0.05))    # ~50ms 방향 저역통과
        VMAX, VDECAY = 15.0, 0.85                        # 속도 clamp / gap 감쇠(오래 끊기면 hold)
        max_step = args.smooth_max_jointvel * out_dt     # slew-rate limit: tick당 최대 관절 이동(rad)
        seq, last_seen, gap = 0, -1, 0
        nxt = time.monotonic()
        t_prev, t_log0 = None, nxt
        while not _stop.is_set():
            t_now = time.monotonic()
            tick_dt = (t_now - t_prev) * 1000.0 if t_prev is not None else 0.0
            if t_prev is not None:
                _tick_ms.append(tick_dt)
            t_prev = t_now
            have_new = False
            with _lock:
                s_seq, active, um = _shared["seq"], _shared["active"], _shared["cmd_mode"]
                st = list(_shared["stick"]); tq = list(_shared["quat"]); tgt = list(_shared["dof"])
                tfz = _shared["foot_z"]
                tfz = None if tfz is None else list(tfz)
            if not active:
                if args.fallback_clip:
                    vr_shm.write(seq, 0, um, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)
                else:
                    vr_shm.write(seq, 1, 1, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)
                seq += 1; oe.reset(); cur = None; cur_foot_z = None
                cur_vel[:] = 0.0; last_seen = s_seq; gap = 0
            else:
                have_new = (s_seq != last_seen)
                if have_new:
                    last_seen = s_seq; gap = 0
                    z_raw = np.asarray(tgt, dtype=np.float32)
                    if cur is not None:
                        # slew-rate limit: outlier/garbage(트래커 dropout·GMR IK 튐)를 물리한계로 clamp.
                        # 실동작은 물리속도 이하라 통과, 단발 garbage는 흡수 → 다음 정상프레임이 되돌림.
                        z = cur + np.clip(z_raw - cur, -max_step, max_step)
                    else:
                        z = z_raw                        # 첫 프레임(active 진입): operator 포즈에 snap
                elif cur is not None and gap < args.smooth_maxgap:
                    z = cur + cur_vel * out_dt           # gap: 속도 extrapolation(도착 지연 은폐)
                    gap += 1
                else:
                    z = cur if cur is not None else np.asarray(tgt, dtype=np.float32)   # 상한 초과→hold
                x_hat, dx_hat = oe(z)
                cur = x_hat
                if have_new:
                    cur_vel = np.clip(dx_hat, -VMAX, VMAX)
                else:
                    cur_vel = cur_vel * VDECAY
                cur_quat = _nlerp(cur_quat, tq, quat_a)
                # 발-z 도 quat 과 같은 계수로 1차 저역통과. None(=원천 없음)이면 그대로 None 을
                # 넘겨 C++ 이 «모름 -> stance 폴백» 으로 처리하게 한다 (0 을 보내면 거짓 obs).
                if tfz is None:
                    cur_foot_z = None
                else:
                    cur_foot_z = tfz if cur_foot_z is None else [
                        c + (t - c) * quat_a for c, t in zip(cur_foot_z, tfz)]
                vr_shm.write(seq, 1, um, st, cur_quat.tolist(), cur.tolist(), cur_vel.tolist(),
                             foot_z=cur_foot_z)
                seq += 1
            if _log_f is not None:
                _log_f.write("%.4f,%.3f,%d,%d,%d,%d,%.3f,%.1f,%.1f,%s\n" % (
                    t_now - t_log0, tick_dt, int(active), int(have_new), gap, um,
                    _diag["gmr_ms"], _diag["rate_hz"], _diag["age_ms"], _diag["reason"]))
            nxt += out_dt
            slp = nxt - time.monotonic()
            if slp > 0:
                time.sleep(slp)
            else:
                # S2: 밀렸을 때 위상(격자)을 버리지 않는다. 놓친 tick 수만큼만 건너뛰어
                # 누적은 막고 50Hz 격자 정렬은 유지 (기존: nxt=now 리셋 → 이후 박자가 어긋남).
                missed = int((time.monotonic() - nxt) // out_dt) + 1
                nxt += out_dt * missed
                _over[0] += missed
        vr_shm.write(seq, 1, 1, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)  # 종료 안전 write
        if _log_f is not None:
            _log_f.flush()

    _out_thread = None
    if args.smooth:
        _out_thread = threading.Thread(target=output_loop, daemon=True)
        _out_thread.start()
        print(f"[bridge] smooth ON: {args.smooth_hz:.0f}Hz out, One-Euro(mincut={args.smooth_mincut}, "
              f"beta={args.smooth_beta}), extrap maxgap={args.smooth_maxgap} ticks.")

    def fallback():
        """VR 비활성(body 끊김/미착용/grip 놓음/종료): 기본=안전 mode1 자동복귀.
        valid=1 + cmd_mode=1 + base_vel=0 → 정책이 레퍼런스(clip/VR)를 masking으로 무시하고
        full-auto mode1로 정지 → clip이 흐르지 않음. --fallback-clip이면 valid=0(clip 재생, 데모)."""
        nonlocal seq_out
        if args.smooth:
            _push_inactive(user_mode)                    # smooth: output thread가 안전 write 담당
            return
        if args.fallback_clip:
            vr_shm.write(seq_out, 0, user_mode, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)
        else:
            vr_shm.write(seq_out, 1, 1, [0.0, 0.0, 0.0], IDENTITY_QUAT, ZERO29, ZERO29)  # cmd_mode=1 safe
        seq_out += 1

    def report(now, tag):
        nonlocal proc, t_report
        if now - t_report >= 1.0:
            dt = max(now - t_report, 1e-6)
            print(f"[bridge] {proc / dt:4.0f}Hz  {tag}")
            if args.diag and args.smooth:
                ticks = list(_tick_ms); _tick_ms.clear()
                gms = list(_gmr_ms); _gmr_ms.clear()
                over, _over[0] = _over[0], 0
                tgt = 1000.0 / args.smooth_hz
                line = (f"[diag] out {len(ticks) / dt:4.1f}Hz  tick p50={_pct(ticks, .5):5.1f} "
                        f"p95={_pct(ticks, .95):5.1f} max={max(ticks) if ticks else float('nan'):5.1f}ms "
                        f"(target {tgt:.0f})  overrun={over}")
                if gms:
                    line += (f"  |  gmr p50={_pct(gms, .5):.2f} p95={_pct(gms, .95):.2f} "
                             f"max={max(gms):.2f}ms")
                print(line)
            proc = 0
            t_report = now

    try:
        while True:
            f = rx.latest()                             # UDP/ZMQ 공통: drain-to-latest + dedup (or None)
            dec = safety.update(f, rx.age_ms())         # 워치독(stale/저수신) + E-stop(우측 A) 래치
            _diag["rate_hz"] = dec["rate_hz"]; _diag["age_ms"] = rx.age_ms()
            _diag["reason"] = dec["reason"].replace(",", ";")   # CSV 안전
            if estop_armed:
                estop_seq = (estop_seq + 1) & 0xFFFFFFFF   # u32 wrap (하트비트)
                estop_shm.write(estop_seq, estop_flag(dec, estop_on_watchdog))
            if dec["mode"] is not None:                 # ESTOP 또는 SAFE(통신불량/스톨) → 안전 개입
                user_mode = 1                           # 안전 기본모드(복귀 시에도 mode1부터)
                fallback()                              # 안전 mode1 + base_vel 0 (smooth면 _push_inactive)
                # 실로봇 Phase2 훅: if dec["estop"]: <Unitree SDK damping/비상정지>. sim은 mode1로 충분.
                prev_dof = None
                if dec["reason"] != prev_reason:
                    print(f"[safety] {dec['reason']}  rate={dec['rate_hz']:.0f}Hz gap={dec['gap_ms']:.0f}ms")
                    prev_reason = dec["reason"]
                time.sleep(POLL_DT)
                continue
            if prev_reason is not None:                 # 안전상태에서 정상 복귀
                print("[safety] OK (정상 복귀)")
                prev_reason = None
            if f is None:                               # 새 프레임 없음(정상 범위) → 이전상태 유지
                time.sleep(POLL_DT)                     # (smooth output thread가 gap을 extrapolation)
                continue
            proc += 1
            now = time.monotonic()

            # --- controllers: 버튼->user_mode, grip->deadman, 썸스틱->base_vel (body와 무관) ---
            ctrl = f.get("controllers")
            if ctrl:
                L, R = ctrl["left"], ctrl["right"]
                if L.get("primary"):    user_mode = 1   # 왼쪽 X → mode1 자율보행
                if L.get("secondary"):  user_mode = 2   # 왼쪽 Y → mode2 상체 teleop
                if R.get("secondary"):  user_mode = 3   # 오른쪽 B → mode3 전신 teleop
                grip_held = L.get("grip", 0.0) > 0.5 or R.get("grip", 0.0) > 0.5
                stick = [L["axis"][1] * args.vx, -L["axis"][0] * args.vy, -R["axis"][0] * args.wz]
            else:
                grip_held, stick = False, [0.0, 0.0, 0.0]

            body_ok = bool(f.get("streaming")) and bool(f.get("body", {}).get("available"))
            active = body_ok and (not args.grip_enable or grip_held)

            if not active:
                fallback()                              # 안전 mode1 (또는 clip) — VR 미적용
                prev_dof = None
                report(now, f"INACTIVE -> {'clip' if args.fallback_clip else 'mode1 safe'}"
                            f"  (body_ok={body_ok} grip={grip_held})")
                continue

            # --- active: body[24] -> GMR -> qpos -> VrRef (VR 텔레옵) ---
            j = f["body"]["joints"]                     # [24][7] = raw [x,y,z, qx,qy,qz,qw]
            human = _msg_body_to_human(j, transform=not args.no_body_transform)
            _t_gmr = time.perf_counter()
            qpos = gmr.retarget(human, offset_to_ground=True)   # [36]
            _diag["gmr_ms"] = (time.perf_counter() - _t_gmr) * 1000.0
            _gmr_ms.append(_diag["gmr_ms"])
            root_quat = [float(x) for x in qpos[3:7]]           # wxyz
            dof_pos = np.asarray(qpos[7:36], dtype=np.float32)  # 29, == deploy JOINT_ORDER
            foot_z = _gmr_foot_z()                             # [z_L, z_R] world (지면 기준) or None
            if args.smooth:
                # receiver는 최신 target만 갱신 → output thread가 고정 rate로 스무딩/보간해 shm write.
                _push_target(user_mode, stick, root_quat, dof_pos, foot_z)
                n += 1
                report(now, f"TELEOP(smooth) mode={user_mode}  base_vel=({stick[0]:+.2f},"
                            f"{stick[1]:+.2f},{stick[2]:+.2f})  arm(L_sho_pitch)={dof_pos[15]:+.2f}")
                continue
            # ↓ --no-smooth 경로 전용. args.smooth(기본 True)면 위에서 continue 하므로 여기 도달 안 함.
            if prev_dof is not None and last_t is not None:
                dt = max(now - last_t, 1e-3)
                raw = (dof_pos - prev_dof) / dt
                dof_vel = args.ema * raw + (1.0 - args.ema) * dof_vel
            prev_dof = dof_pos.copy()
            last_t = now

            vr_shm.write(seq_out, 1, user_mode, stick, root_quat, dof_pos.tolist(), dof_vel.tolist(),
                         foot_z=foot_z)
            seq_out += 1
            n += 1
            report(now, f"TELEOP mode={user_mode}  base_vel=({stick[0]:+.2f},{stick[1]:+.2f},"
                        f"{stick[2]:+.2f})  arm(L_sho_pitch)={dof_pos[15]:+.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        if args.smooth and _out_thread is not None:
            _stop.set()                                 # output thread가 종료 시 안전 write 후 exit
            _out_thread.join(timeout=0.5)
        else:
            fallback()                                  # 종료: 안전 mode1 (또는 clip)
        if estop_armed:
            estop_shm.clear()   # 정상 종료 -> disarm(파일 제거). 크래시면 하트비트 stale로 C++가 damping.
        rx.close()
        if _log_f is not None:
            _log_f.close()
            print(f"[bridge] diag CSV written: {args.log}")
        print(f"\n[bridge] stopped (safe fallback). total {n} teleop frames.")


if __name__ == "__main__":
    main()
