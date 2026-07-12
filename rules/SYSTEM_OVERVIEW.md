# G1 Sim2Sim / Sim2Real + PICO VR 텔레옵 — 시스템 구조

> 작성 목적: 보행은 되는데 **PICO VR 연결이 안 되는** 상황에서, 전체 구조와 데이터 흐름을
> 한눈에 파악하고 "어디서 끊기는지"를 단계별로 짚기 위한 문서.
> (repo: `unitree_rl_mjlab`, branch: `wose_obs`, deploy: `deploy/robots/g1/`)

---

## 0. 한 줄 요약

C++ 제어기 **`g1_ctrl`가 "뇌"**이고, PICO VR 입력은 그 뇌에 **`/dev/shm` 공유메모리 파일**로
간접 주입된다. 뇌와 PICO가 파일로 분리돼 있어서 **걷기(뇌만으로 가능)는 되지만**, PICO 텔레옵은
그 주입 파이프라인(특히 **PICO를 읽어 네트워크로 쏘는 publisher**)이 비어 있어 동작하지 않는다.

---

## 1. 전체 데이터 흐름

```
[PICO 헤드셋]
   │  WiFi/USB
   ▼
[XRoboToolkit PC-Service]  ── xrobotoolkit_sdk(xrt) ──┐   (헤드셋이 물린 PC에서 실행)
                                                      │
   ┌──────────────────────────────────────────────────────────────────────┐
   │  ★ PICO 입력 경로 2가지 ★                                             │
   │                                                                       │
   │  [A] pico_control_bridge.py  ── xrt 로컬 직접읽기 ──►                  │
   │       썸스틱/버튼만 → /dev/shm/g1_masked_gui   (base_vel + mode)       │
   │       (g1_ctrl과 같은 PC에서만 동작. body/GMR 없음)                    │
   │                                                                       │
   │  [B] (★누락된 publisher★) ──네트워크 ZMQ/UDP :5556──► vr_teleop_bridge.py │
   │       PICO 24관절 body + 컨트롤러 → GMR 리타겟 → /dev/shm/g1_vr_ref    │
   │       (풀바디 mode2/3 + base_vel/mode 모두 이 한 브릿지가 처리)        │
   └──────────────────────────────────────────────────────────────────────┘
                              │  /dev/shm  (파일 IPC, atomic replace)
                              ▼
[g1_ctrl (C++)]  ── Unitree DDS (--network=lo | 실로봇 iface) ──►  [unitree_mujoco | 실로봇]
   매 스텝 g_poll_gui() + g_poll_vr()로 shm 읽어 ONNX 정책 obs에 주입
```

---

## 2. 구성요소

### 2.1 뇌 — `g1_ctrl` (C++)
- 소스: `deploy/robots/g1/src/State_Mimic.cpp`, `main.cpp`
- FSM(`config/config.yaml`): `Passive → FixStand → Velocity / Mimic_Masked`
  - `Velocity`(id 3): 순수 보행 정책. **PICO 불필요.**
  - `Mimic_Masked`(id 6, `gmt_multihead_v0` ONNX): 텔레옵 정책. **여기가 PICO 대상.**
- **cmd_mode 3단 마스킹** (`State_Mimic.cpp:17-24`):
  | mode | 의미 | 다리 | 상체(팔·waist) | VR ref |
  |---|---|---|---|---|
  | 1 | full-auto 보행 | 자율 | 자율 | 무시(썸스틱 base_vel만) |
  | 2 | 상체 teleop | 자율 | VR 추종 | 상체만 사용 |
  | 3 | 전신 teleop | VR 추종 | VR 추종 | 전신 사용 |
  | 4/5 | 데모 클립 | 클립 | 클립 | VR 무시 |
- shm 폴링: `g_poll_gui()`(`State_Mimic.cpp:65`), `g_poll_vr()`(`:109`) — 매 제어 스텝.
- 시뮬/로봇 연결: `./g1_ctrl --network=<iface>` (`main.cpp:37` ChannelFactory Init).
  - **sim2sim = `lo`**, **실로봇 = 실제 iface(예: enp5s0)**.
- 키보드 백업(터미널 포커스): `1/2/3`=mode, `WASD/QE`=속도, `p`=정지, `v`=Velocity, `m`=Mimic_Masked.

### 2.2 IPC 채널 (파일 계약 — Python↔C++ 레이아웃 동기 필수)
| 파일 | magic | 내용 | 쓰는 쪽 | 읽는 쪽 |
|---|---|---|---|---|
| `/dev/shm/g1_masked_gui` | 0x6701 | base_vel + mode + gait params | `tools/gui_shm.py`(masked_gui / pico_control_bridge) | `g_poll_gui()` |
| `/dev/shm/g1_vr_ref` | 0x6702 | base_vel, root_quat, dof_pos[29], dof_vel[29] | `teleop/vr_shm.py`(vr_teleop_bridge / vr_replay) | `g_poll_vr()` |

- 계약 정의: `deploy/robots/g1/tools/gui_shm.py`, `deploy/robots/g1/teleop/vr_shm.py`
- `valid=0`으로 쓰면 C++가 VR override를 해제(클립/자율로 복귀).

### 2.3 PICO 입력 — 두 variant
**[A] `tools/pico_control_bridge.py`** — 가벼운 컨트롤러 경로
- `xrobotoolkit_sdk`로 PICO **컨트롤러(썸스틱/버튼)를 로컬에서 직접** 읽음.
- 썸스틱→base_vel, X/Y/A→mode1/2/3, B→정지 → `/dev/shm/g1_masked_gui`.
- **body 트래킹·GMR 없음.** g1_ctrl과 **같은 PC**에서 돌아야 함(공유 /dev/shm, 로컬 PC-Service).

**[B] `teleop/vr_teleop_bridge.py`** — 풀바디 텔레옵 (mode2/3의 핵심)
- 네트워크(:5556)로 **PICO body 24관절 + 컨트롤러 프레임**을 받음(transport: 기본 `zmq`/TCP, 옵션 `udp`).
- GMR 리타겟(xrobot→unitree_g1, ~15ms IK) → qpos[36] → root_quat + dof_pos[29].
- One-Euro 스무딩 + gap extrapolation + slew-rate limit, dof_vel는 finite-diff+EMA.
- 컨트롤러도 같은 프레임에 실려오므로 **base_vel/mode까지 이 브릿지가 함께 처리** → 한 브릿지로 "둘 다" 커버.
- 안전: 워치독(stale>200ms 또는 rate<30Hz → mode1) + E-stop(우측 A 래치, 우측 menu 1s 홀드 해제).
- 수신 계층: `teleop/udp_receiver.py`(UDP), 브릿지 내부 `ZmqReceiver`(ZMQ). 패킷 포맷: `teleop/pico_wire.py`(806B 고정 바이너리).
- **GMR을 제어 호스트에서 빼는 게 원칙** → 노트북/별도 PC에서 브릿지 실행, `/dev/shm`(로컬) 또는 네트워크로 g1_ctrl에 전달.

---

## 3. 두 시나리오별 토폴로지

### 3.1 Sim2Sim (현재)
- **PICO = 윈도우 노트북**, **시뮬 = com1**. 노트북에서 com1으로 SSH.
- com1: `unitree_mujoco` + `g1_ctrl --network=lo` + `vr_teleop_bridge.py`(:5556 bind).
- **PICO 데이터는 노트북→com1 네트워크로 와야 함 → 경로 [B] 사용.**
- 필요한 것: **노트북에서 도는 publisher**(xrt로 PICO 읽어 com1:5556으로 송신). ← 지금 없음.

### 3.2 Sim2Real (온보드 형태)
- **PICO = 로봇 옆 컴퓨터에 직접 연결**. 노트북은 SSH로 "재생"만 누름.
- 로봇 컴퓨터: PICO PC-Service + 브릿지 + `g1_ctrl --network=<real iface>`가 **전부 로컬**.
- 이 경우 PICO가 g1_ctrl과 **같은 PC(co-located)** → 네트워크 홉 불필요.
  - 컨트롤러만이면 [A] `pico_control_bridge.py`(로컬 xrt)로 충분.
  - 풀바디면 [B]를 **로컬**에서(publisher+bridge 같은 PC, `--com1 127.0.0.1`) 돌리거나, xrt를 직접 읽는 통합 브릿지가 필요.

> 주의: 코드 주석은 "GMR=노트북 / g1_ctrl=Jetson" 2홉 배포를 상정하지만,
> 사용자의 실제 sim2real은 "PICO도 로봇 컴퓨터에 직접" 온보드형이라 홉 구성이 다르다. 배포 시 이 차이를 반영할 것.

---

## 4. PICO 연결이 안 되는 구조적 갭 (핵심)

1. **[치명] 풀바디용 publisher 스크립트가 repo에 없음.**
   `vr_teleop_bridge.py:6` 주석은 `노트북 pico_publisher.py --ZMQ--> :5556`이라 하지만
   **`pico_publisher.py`는 repo에도, git 히스토리에도 존재한 적 없음.** 수신측(`udp_receiver`/`com1_subscriber`/`ZmqReceiver`)과 패킷 포맷(`pico_wire`)만 있고 **송신자가 비어 있음.**
   → 브릿지는 :5556에서 계속 대기 → 워치독이 200ms 후 mode1(안전)로 폴백 → PICO 움직여도 무반응.
   - 만들 근거: `reference_code/.../XRoboToolkit-PC-Service-Pybind.../examples/example_body_tracking.py`
     (`xrt.get_body_joints_pose()` → 24×[x,y,z,qx,qy,qz,qw]) + `pico_wire.pack_frame()`로 패킹 → ZMQ PUB(connect com1:5556) 또는 UDP sendto.

2. **컨트롤러 경로[A]의 xrt는 로컬 PC-Service에만 붙음.**
   `pico_control_bridge.py:49` `xrt.init()`은 인자 없이 로컬 PC-Service에 연결.
   sim2sim은 PICO가 노트북에 있고 시뮬이 com1이라, com1에서 [A]를 돌리면 PICO를 못 봄(원격 host 지정 없음). → sim2sim에선 [A] 대신 [B](네트워크)가 맞음.

3. **XRoboToolkit PC-Service + SDK가 아직 세팅 안 됨(추정).**
   현재 로컬 어떤 env에도 `xrobotoolkit_sdk` 미설치(import 실패 확인).
   PICO body/컨트롤러를 읽으려면 헤드셋이 물린 PC에 **PC-Service 실행 + SDK 설치**가 선행돼야 함.

---

## 5. 단계별 점검 절차 (어디서 끊기는지 bisect)

> 아래로 내려가며 각 홉을 독립 검증. 실패하는 첫 홉이 원인.

### Hop 0 — PICO 없이 `/dev/shm` → g1_ctrl 경로부터 확인 (제일 먼저)
- **VR ref 경로**: `python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2`
  (com1에서 `unitree_mujoco` + `g1_ctrl --network=lo` 띄운 상태) → 정책이 클립 팔을 따라하면
  **/dev/shm/g1_vr_ref → g1_ctrl 파이프라인은 정상.** 남은 건 순수 PICO 입력부.
- **GUI ctrl 경로**: `tools/run_g1_with_gui.sh`(브라우저 :8080)로 base_vel/mode 넣어 보행/모드전환 확인.
- 여기까지 되면 뇌·IPC·정책은 OK. 문제는 **PICO→shm 입력** 뿐임이 확정된다.

### Hop 1 — PICO 헤드셋 → PC-Service → SDK (헤드셋 물린 PC에서)
- XRoboToolkit PC-Service 실행 + 헤드셋 페어링(body tracking on).
- `example_body_tracking.py` 실행 → `is_body_data_available()` True, 24관절 값이 흐르는지.
- 실패면: PC-Service/헤드셋/캘리브레이션 문제 (SDK 설치·PC-Service 버전 확인).

### Hop 2 — publisher → :5556 수신 확인  ← **현재 여기가 비어 있음**
- 수신 테스트(com1): `python deploy/robots/g1/teleop/udp_receiver.py --port 5556` (UDP)
  또는 `python deploy/robots/g1/teleop/com1_subscriber.py --port 5556` (ZMQ).
- publisher(노트북)에서 프레임을 쏴야 위 수신기에 Hz가 찍힘.
  **→ publisher가 없으므로 지금은 "수신 없음"이 정상. 이 스크립트를 먼저 만들어야 함(갭 #1).**

### Hop 3 — bridge → `/dev/shm/g1_vr_ref`
- `.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --mode 1`
  (com1은 기존 `gmr` conda env도 가능). 로그에 `TELEOP mode=.. base_vel=..`가 수십 Hz로 찍히면 OK.
- `[safety] SAFE/STALE`만 반복 = 상류(Hop 2 publisher) 미수신.

### Hop 4 — g1_ctrl 마스킹/모드 반영
- 브릿지가 write 중인데 로봇이 안 따라오면: cmd_mode(2/3 진입?), grip 데드맨(`--grip-enable`), 마스킹 로직 확인.

---

## 6. 실행 명령어 요약

```bash
# ── com1(제어 PC): 시뮬 + 뇌 ──
#   (터미널1) unitree_mujoco 실행
#   (터미널2) 뇌 + 브라우저 GUI
deploy/robots/g1/tools/run_g1_with_gui.sh            # sim2sim(lo). 실로봇: run_g1_with_gui.sh <iface>

# ── PICO 경로 B (풀바디 + base_vel/mode) ──
#   (com1) 브릿지 — 5556 bind, GMR 리타겟
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --mode 1
#   (노트북) publisher — ★작성 필요★: xrt read → pico_wire.pack_frame → com1:5556

# ── PICO 경로 A (컨트롤러만, co-located: sim2real 온보드) ──
uv run --with xrobotoolkit_sdk python deploy/robots/g1/tools/pico_control_bridge.py

# ── 점검용 ──
python deploy/robots/g1/teleop/udp_receiver.py --port 5556        # :5556 수신 확인(UDP)
python deploy/robots/g1/teleop/com1_subscriber.py --port 5556     # :5556 수신 확인(ZMQ)
python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2 # PICO 없이 VR경로 검증
```

---

## 7. 다음 할 일 (우선순위)

1. **publisher 작성** (`pico_publisher.py`, 노트북/헤드셋 PC용) — 갭 #1. `example_body_tracking.py` + `pico_wire.pack_frame` + ZMQ PUB(connect)/UDP. 컨트롤러(썸스틱/버튼)도 프레임에 포함해 [B] 하나로 base_vel/mode까지 커버.
2. **Hop 0 검증**: `vr_replay.py`로 뇌·IPC 정상 확인(PICO 문제와 분리).
3. **Hop 1 검증**: PC-Service + SDK 설치, `example_body_tracking.py`로 body 데이터 확인.
4. publisher ↔ bridge 연결(Hop 2·3), transport(zmq LAN / udp WAN) 선택.
5. sim2real 온보드형: co-located에 맞게 publisher/bridge를 로컬(`127.0.0.1`)로 구성.

---

## 8. 검증 로그 (2026-07-11)

### GMR 실시간 retargeting — ✅ 정상 + 빠름
- conda `gmr` env(`~/miniconda3/envs/gmr`)로 검증. `.gmr`/`.venv-teleop`는 이 워크스페이스에 없음(com1은 conda env 사용).
- 빌드: `GeneralMotionRetargeting("xrobot","unitree_g1")` 0.13s에 완료. IK config `reference_code/GMR/.../ik_configs/xrobot_to_g1.json`, g1 STL 38개 resolve, `max_iter=10`.
- `retarget()` → `qpos[36]`. **타이밍: median 3.2ms / p95 3.3ms** → 50Hz(20ms budget) **여유롭게 실시간 OK**(README가 걱정한 ~15ms보다 훨씬 빠름 — warm-start + 조기수렴).
- **관절 순서 일치 확인**: GMR 29-motor 순서(leg_L 6 / leg_R 6 / waist 3 / arm_L 7 / arm_R 7) == `deploy.yaml` JOINT_ORDER `leg_L[0:6] leg_R[6:12] waist[12:15] arm_L[15:22] arm_R[22:29]`. **완전 일치 → remap 불필요**(브릿지 `dof_pos=qpos[7:36]` 주석대로). 조용한 관절 뒤섞임 버그 없음.

### 온보드(로봇 컴퓨터 clone-to-build) 주의
- **publisher는 repo 안에 있어야 함**: 현재 publisher는 윈도우 노트북에만 있고 repo 밖. 로봇 컴퓨터에 이 repo를 clone하면 publisher가 없다 → **repo에 포함 필요**(갭 #1과 동일 결론).
- **xrobotoolkit_sdk 아키텍처**: 로봇 컴퓨터(ARM64 Jetson / x86)용 빌드 필요. `reference_code/.../XRoboToolkit-PC-Service-Pybind_X86_and_ARM64`에 양쪽 빌드 있음. **윈도우용 SDK는 이식 불가.** GMR 자체엔 xrobotoolkit_sdk 불필요(publisher만 필요).
- **GMR 재현**: 로봇 컴퓨터엔 conda `gmr` env가 없음 → `setup_teleop.sh`가 `.venv-teleop` 생성 + GMR을 **GitHub에서 새로 clone**(현재 com1이 쓰는 `reference_code/GMR` editable은 clone에 없음) + g1 메시 복구. 동작 재현성이 필요하면 `setup_teleop.sh`의 `GMR_COMMIT`을 known-good SHA로 핀.
- **온보드는 네트워크 홉 불필요**: PICO·publisher·bridge·g1_ctrl이 전부 로컬 → publisher를 `127.0.0.1:5556`으로 쏘거나, 더 단순하게 xrt를 직접 읽어 GMR→`/dev/shm/g1_vr_ref`를 쓰는 **통합 로컬 브릿지**로 합쳐도 됨.
</content>
</invoke>
