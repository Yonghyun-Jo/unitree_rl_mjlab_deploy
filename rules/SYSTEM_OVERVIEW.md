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
- **모드 마스킹** — 모드의 성질은 표가 갖는다(`config/modes.yaml` + 학습 `mode_spec.py` → 생성 `include/ModeTable.h`). C++는 번호를 비교하지 않는다(`rules/ADDING_A_MODE.md`):
  | mode | 표 이름 | 의미 | 다리 | 상체(팔·waist) | VR ref |
  |---|---|---|---|---|---|
  | 1 | `loco` | full-auto 보행 | 자율 | 자율 | 무시(썸스틱 base_vel만) |
  | 2 | `upper` | 상체 teleop | 자율 | VR 추종 | 상체만 사용 |
  | 3 | `track` | 전신 teleop | VR 추종 | VR 추종 | 전신 사용 |
  | 4 | `playback` | **고른** 클립 재생 | 클립 | 클립 | VR 무시 |

  로그 라벨(`[cmd_mode] -> N (이름)`)은 이 표 이름이다. 표엔 `5`(`ground`)·`6`(`crawl`) 행도 있지만 **지금 슬롯의 ONNX가 모르므로** 요청하면 거부 한 줄만 찍힌다(`이 슬롯(ONNX)이 모르는 모드`).
- shm 폴링: `g_poll_gui()`(`State_Mimic.cpp:65`), `g_poll_vr()`(`:109`) — 매 제어 스텝.
- 시뮬/로봇 연결: `./g1_ctrl --network=<iface>` (`main.cpp:37` ChannelFactory Init).
  - **sim2sim = `lo`**, **실로봇 = 실제 iface(예: enp5s0)**.
- 키보드 백업(터미널 포커스): `1/2/3/4`=mode(표의 `key` 열), `[`/`]`=재생할 클립 고르기(**클립 재생 모드가 아닐 때만** — 재생 중엔 거부 한 줄), `5`/`6`=표엔 있으나 지금 슬롯이 모르는 모드 → 거부 로그, `config/mode5_keys.yaml` 의 키(지금 `z x c b n h`)=mode5 자세(직립·네발·포복·앉기·눕기·크랩, **mode5 에서만** — 다른 모드에선 한 줄 안내만), `WASD/QE`=속도, `p`=정지, `v`=Velocity, `m`=Mimic_Masked.
  mode4(와 mode5)는 **직립일 때만**(골반 높이 추정 `z_fk ≥ 0.65 m` ∧ 걸러진 기울기 `< 30°`) 떠날 수 있다 — 아니면 요청이 거부되고 `[cmd_mode] … 거부: …` 한 줄이 남는다(계약 v1 슬롯의 mode4 도 같다. mode5 는 직립 모드로 나갈 때 직립 버튼 `z` 의 유지 상태까지 요구하고, 저자세 모드끼리는 막지 않는다).
  안전층도 GroundCapable 모드(지금 4·5, 6 은 예약)에선 다르다(`SafetyPolicy.h`, 계약 v1 슬롯의 mode4 포함): **낮거나 기운 자세**(`z_fk < 0.65` ∨ 기울기 `≥ 30°`)에서 qd_warn 이 걸리면 폴백 모드 대신 **Passive** 로 간다(로그 `qd_warn LATCHED … -> Passive. 복귀=p→f→m` — 복귀는 `p` → `f` → `m`). 넘어짐 판정(기울기 `> 57.3°` → Passive)은 **명령이 직립이고**(재생 클립의 현재 프레임 골반 높이 / mode5 자세의 목표 높이 `≥ 0.65 m`) **최근 1 s 안에 섰을** 때만 건다. «섰음» = 명령이 직립인 동안(mode1~3 은 정의상 직립 명령이라 늘 채운다 — 넘어지는 도중 4·5 로 바꿔도 판정이 이어진다) `z_fk ≥ 0.65` ∧ 원시·걸러진 기울기 둘 다 `< 57.3°` 이고, 명령이 낮아지면 그 기억을 지운다. 그래서 서 있다 넘어지면 잡고, 일부러 내려가기·누운 데서 일어나기(엉덩이부터 드는 기립 포함)는 넘어짐으로 치지 않는다. **낮은 자세를 눌렀다 직립 버튼으로 되돌리면**, 로봇이 다시 «섰음»(`z_fk ≥ 0.65` ∧ 기울기 `< 57.3°`)이 될 때까지 넘어짐 판정이 꺼져 있다 — 그사이 넘어지면 이 판정은 못 잡는다(qd_warn·qd_crit·E-stop 은 그대로). 대가 둘(알고 쓴다): ① mode4 의 «명령 직립» 은 클립 골반 높이만 본다 — 다리를 편 채 깊이 숙이는 클립(골반 ≥ 0.65 m · 기울기 > 57.3°)은 Passive 로 간다(B2 이전과 같음). ② 클립이 잠깐이라도 0.65 m 아래로 내려가면 기억이 지워져, 다시 «섰음» 이 될 때까지 판정이 꺼진다. mode6 은 명령에 높이가 없어 들어오면 넘어짐 판정을 안 건다. mode1~3 은 종전 그대로(넘어짐 판정 항상 · qd_warn → 폴백 모드).

### 2.2 IPC 채널 (파일 계약 — Python↔C++ 레이아웃 동기 필수)
| 파일 | magic | 내용 | 쓰는 쪽 | 읽는 쪽 |
|---|---|---|---|---|
| `/dev/shm/g1_masked_gui` | 0x6704 (v2, 48 B) | 1회성 `mode_req` + base_vel + gait params + mode5 자세(`m5_preset`·`m5_press_seq`) + 1회성 `clip_req` | `tools/gui_shm.py`(masked_gui / pico_control_bridge) | `g_poll_gui()` |
| `/dev/shm/g1_vr_ref` | 0x6702 | base_vel, root_quat, dof_pos[29], dof_vel[29] | `teleop/vr_shm.py`(vr_teleop_bridge / vr_replay) | `g_poll_vr()` |

- 계약 정의: `deploy/robots/g1/tools/gui_shm.py`, `deploy/robots/g1/teleop/vr_shm.py`
- **GUI 채널 v2 (2026-09-22)** — 바이트 배치는 `tools/gui_shm.py` `FMT` ↔ `State_Mimic.cpp` `struct GuiCtrl` 을 `tests/test_gui_shm_layout.py` 가 대조한다(실행기에 포함).
  - `mode_req` = **1회성** 모드 요청(0 = 요청 없음). 쓰는 쪽이 한 번 보내고 0 으로 되돌린다 → 속도 슬라이더·자세 버튼만 움직여서는 모드가 다시 요청되지 않는다(v1 은 매 쓰기마다 `cmd_mode` 를 다시 보내, 키보드로 바꾼 모드가 GUI 쪽 모드로 되돌아갔다). 제어기의 **첫 읽기**는 파일에 남은 지난 요청(모드·클립)을 재생하지 않는다(한 줄 알림).
  - `m5_preset` = 0 없음, 1.. = `Mode5Presets.h` 표 index + 1 · `m5_press_seq` = 누를 때마다 +1(같은 자세 재입력 = 새 목표). **mode5 에서만** 먹는다. 브라우저 GUI 는 **키가 있는 자세만**(`config/mode5_keys.yaml`, 지금 6개) 버튼으로 낸다 — 표의 나머지 자세(키 없음)는 검증 뒤에 연다.
  - `clip_req` = −1 그대로, 0.. = 고를 클립 칸(`g_build_clips` 순서 primary·light·demo6 중 실린 것). **1회성**, 클립 재생 모드에 있는 동안은 거부(키보드 `[`/`]` 와 같은 문 `g_select_clip`).
  - **요청할 수 있는 모드가 채널마다 다르다**: GUI = 표의 모든 모드(슬롯 지원·이탈 조건은 `ModeRuntime` 이 본다) · VR(`g1_vr_ref`) = 직립 모드(`safety: upright_only`)만.
  - magic 은 채널마다 다르다(0x6701 = GUI v1 · 0x6702 = VR · 0x6703 = E-stop `g1_estop` · 0x6704 = GUI v2). 옛 GUI(0x6701)가 쓰면 제어기는 무시하고 `[gui] 옛 형식(0x6701) — 무시` 를 한 번 찍는다 → GUI·PICO 브리지를 이 브랜치의 `gui_shm.py` 로 다시 띄울 것.
- `valid=0`으로 쓰면 C++가 VR override를 해제(클립/자율로 복귀).

### 2.3 PICO 입력 — 두 variant
**[A] `tools/pico_control_bridge.py`** — 가벼운 컨트롤러 경로
- `xrobotoolkit_sdk`로 PICO **컨트롤러(썸스틱/버튼)를 로컬에서 직접** 읽음.
- 썸스틱→base_vel, X/Y/A→mode1/2/3, B→정지 → `/dev/shm/g1_masked_gui`.
- 모드는 **브리지 자신이 아는 모드가 바뀔 때만** 요청한다(1회성 `mode_req`, v1 과 같은 조건). 그래서 키보드·GUI 로 모드를 바꾼 뒤 브리지가 이미 그 모드라고 알고 있는 버튼(예: 브리지 기억 = 1 인데 X)을 누르면 아무 요청도 안 나간다 — 다른 버튼을 한 번 눌렀다 돌아올 것. 스틱을 움직여도 모드는 다시 요청되지 않는다(v1 은 되돌렸다).
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

> **갱신(2026-08-04)**: 갭 #1의 publisher는 이후 **윈도우 노트북 로컬에 작성되어 사용 중**이다
> (`pico_publisher.py` ZMQ / `pico_publisher_udp.py` UDP — 패킷 포맷은 `teleop/pico_wire.py`,
> 통합 메모는 `teleop/UDP_INTEGRATION.md`). **여전히 repo 밖**이라 다른 PC에서 발행하려면 옮겨야
> 한다는 점만 유효하다. 아래 갭 목록은 당시 진단 기록으로 남긴다.

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
#   (com1) 브릿지 — 5556 bind, GMR 리타겟. com1은 conda gmr env로:
~/miniconda3/envs/gmr/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport udp --mode 1
#   (노트북) publisher — repo 밖(노트북 로컬). xrt read → pico_wire.pack_frame → com1:5556
#       python pico_publisher_udp.py --com1 <com1 IP> --port 5556      # UDP_INTEGRATION.md
#   ※ 실로봇을 이 네트워크 토폴로지로 돌릴 땐 브릿지에 --arm-estop 추가(하드 E-stop 무장).
#      기본 auto는 --transport local일 때만 무장 → 네트워크는 DISARMED로 뜬다.
#      시작 로그 `[bridge] hard E-stop: ARMED/DISARMED` 로 확인. (RUNBOOK §2-3b)

# ── PICO 경로 A (컨트롤러만, co-located: sim2real 온보드) ──
uv run --with xrobotoolkit_sdk python deploy/robots/g1/tools/pico_control_bridge.py

# ── 점검용 ──
python deploy/robots/g1/teleop/udp_receiver.py --port 5556        # :5556 수신 확인(UDP)
python deploy/robots/g1/teleop/com1_subscriber.py --port 5556     # :5556 수신 확인(ZMQ)
python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2 # PICO 없이 VR경로 검증
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py     # LAFAN 클립 재생(구간/속도/mode2·3)
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

---

## 9. 텔레옵 관절 안전 3층

> `Mimic_Masked`(텔레옵) 정책의 모터 출력 경로에 붙은 joint-space 안전장치. 정책이 OOD(학습
> 밖 상황)나 발산으로 이상 출력을 내도 로봇을 보호하기 위한 3개의 독립 층.

**철학**: 한계값을 기계한계/in-distribution **밖**에 두어 정상 동작에는 절대 닿지 않고,
OOD/발산 상황에서만 발동하게 한다 → 학습 parity(관측 `last_action`은 raw 정책 출력, 안전층은
출력 전용 필터)를 그대로 보존. 세 층 모두 config-gated이고 **커밋 기본값은 셋 다 `enable=false`**
(배포해도 동작 0 변화). config는 `deploy/robots/g1/config/policy/mimic_masked/gmt_multihead_v0/params/deploy.yaml`의
`safety:` 블록 + `action.clip`. **fail-safe**: `safety:` 블록이 없거나 배열 길이/파싱이 이상하면
해당 층은 자동으로 비활성될 뿐, 0으로 clamp하거나 throw하지 않는다(`State_Mimic::load_safety_cfg`).

### Layer 1 — 위치 clamp (passive, 항상 유효 시 gated)

각 관절 목표 q를 기계한계 `[min, max]`(`src/assets/robots/unitree_g1/xmls/g1.xml`의 29개 관절
range)로 clamp한다. 적용 지점은 2곳:
1. **`action.clip`** (`process_actions`, `joint_actions.h:54-57`) — deploy.yaml `actions.JointPositionAction.clip`에
   기계한계 29쌍이 채워져 있어 **항상** 적용된다. `default_joint_pos`/학습 q가 전부 범위 안쪽이라
   in-distribution에서는 no-op — L1의 1차 방어선.
2. **C++ 최종 clamp** (`enable_pos_clamp`, `State_Mimic.cpp` `run()`) — `apply_arm_blend`/`apply_switch_blend`
   이후, `motor_cmd.q()`에 쓰기 직전에 한 번 더 clamp하는 최종 보루. `safety.enable_pos_clamp: true`일
   때만 동작(기본 false).

### Layer 2 — 속도 rate-limit (passive, `enable_rate_limit`)

motor에 쓰는 q_target의 **per-tick 변화량**을 `vel_max · dt`로 캡한다(`js_rate_limit`, L1 clamp
직후·모터 쓰기 직전).
- **`run()`은 CtrlFSM에서 1kHz로 호출**되므로 `dt = 0.001`(`CtrlFSM.h`의 `dt=0.001` tick과 동일 —
  정책 자체는 `policy_thread`에서 50Hz로 별도 스텝). 즉 `max_step = vel_max * 0.001`이고, 이게 매
  1kHz tick마다 적용되므로 **effective cap ≈ vel_max rad/s**.
- `q_prev`(이전 tick 목표 q)는 `State_Mimic::enter()`에서 **측정 pose**(`env->robot->data.joint_pos`)로
  초기화 — 나중에 이 층을 켜도 첫 틱에서 stale q_prev로 인한 lurch가 나지 않는다.
- 비활성일 때도 `q_prev`는 계속 추적만 한다(값은 그대로 통과) — 나중에 킬 때 점프 방지.

### Layer 3 — 측정 qd 폭주 → warn/crit (active, `enable_qd_guard`, policy_thread 50Hz)

`policy_thread`(50Hz — `g_mode`(`ModeRuntime`)/`notify_mode_switch`와 같은 스레드) 루프에서 측정
`joint_vel`의 `max|qd|`를 매 틱 감시한다(`js_qd_severity`). **`over_ticks`(기본 5 = 0.1s@50Hz)
연속 초과** 시:
- **warn**(`max|qd| > qd_warn`) → **폴백 모드(표의 첫 행 = mode1) 강제(래치)** — 단 GroundCapable 모드(지금 4·5,
  6 은 예약)의 낮거나 기운 자세면 Passive(§2.1 끝, `SafetyPolicy.h`). 처분은 그 틱의 모드로 정한다 — 래치 중(폴백
  모드) 4·5 를 누른 그 틱에 낮거나 기운 자세면 Passive. 래치 중엔 매 틱
  `g_mode.force(G1_FALLBACK_MODE)`로 덮어써 유지(soft — `force()`는 이탈 조건·슬롯 지원을 안 본다).
  **수동 복귀**: 조작자가 폴백 모드를 **명시적으로 요청**(X버튼 또는 키보드 `'1'`)해서
  `g_mode.requested() == G1_FALLBACK_MODE`가 되어야 래치 해제 — qd가 아직 높으면 다음 sustained
  구간에서 재래치될 수 있다. ⚠ `requested()`는 요청이 **받아들여질 때만** 갱신되므로, 슬롯
  `deploy.yaml`의 `modes:`에 폴백 모드가 없으면 래치를 영영 못 푼다(기동 시 거부하고 죽는다).
- **crit**(`max|qd| > qd_crit`) → `js_qd_crit_latched_`(atomic) 래치 → FSM `registered_checks`가
  이를 읽어 **Passive(damping) 전이**(래치, hard). 복귀: 키보드 `'f'`로 FixStand 재기립 후,
  `Mimic_Masked`를 **다시 진입**(FSM 재진입, `enter()`에서 warn/crit 래치·카운터 리셋)해야 해제.
- 넘어짐(기울기 `> 57.3°`) 판정은 이 층이 아니라 FSM `registered_checks` 의 `bad_orientation` → Passive 다
  (검사는 FSM 스레드 1 kHz, 읽는 `projected_gravity_b` 는 policy_thread 가 50 Hz 로 갱신 — 걸러지지 않은
  원시 기울기). 모드별 관문은 `SafetyPolicy.h`: UprightOnly(1·2·3)는 항상 적용, GroundCapable 은
  «명령 직립 ∧ 최근 1 s 에 섰음» 일 때만(§2.1 끝). 관문은 policy_thread 가 매 틱 계산해 원자값으로 넘긴다.
  qd-guard 는 기울기를 **처분**(폴백 모드 / Passive)에만 쓴다.

### config 필드 (`deploy.yaml` `safety:` 블록)

| 필드 | 타입 | 의미 |
|---|---|---|
| `enable_pos_clamp` / `enable_rate_limit` / `enable_qd_guard` | bool | 층별 on/off (커밋 기본 전부 false) |
| `pos_min[29]` / `pos_max[29]` | rad | 기계한계(`action.clip`과 동일 값) |
| `vel_max` | rad/s (scalar) | rate-limit effective cap |
| `qd_warn` / `qd_crit` | rad/s | L3 경보/치명 임계 |
| `over_ticks` | int | 연속 초과 판정 틱 수(@50Hz, 기본 5 = 0.1s) |

전부 **재컴파일 없이** deploy.yaml 값만 바꿔 튜닝 가능.

### 증분 enable 절차 (안전 — 반드시 순서대로, sim2sim 먼저)

1. **코드 배포 시점** = 3층 전부 off. `action.clip`(기계한계)만 항상 유효하지만 in-distribution
   에선 no-op이므로 실질적으로 동작 무변화.
2. **sim2sim**(`unitree_mujoco` + `g1_ctrl --network=lo` + 브릿지)에서 `enable_pos_clamp: true` →
   mode1/2/3 보행이 off일 때와 동일한지 확인.
3. `enable_rate_limit: true`(`vel_max` 넉넉히) → 보행 lag/이상 없는지 확인 → Task 5(아래)로 측정한
   in-distribution qd p99로 `vel_max`를 조인다.
4. `enable_qd_guard: true` → 정상 보행에서 오발동 없는지(`qd_warn`/`qd_crit` ≫ 정상 보행 qd) +
   인위적으로 빠른 동작/외란을 줬을 때 warn→mode1(X로 복귀), crit→Passive(`f`로 재기립) 폴백이
   실제로 동작하는지 확인.
5. **실로봇**: 위 sim 검증을 전부 통과한 뒤, mode1부터 시작하고 사람이 하드웨어 E-stop 옆에 대기.

### in-distribution 임계 측정 (Task 5, 사용자 수행)

sim2sim에서 정상 보행(mode1/2/3, 팔 동작 포함)을 몇 분간 돌리며 관절별 `max|qd|`/p99를 수집한다.
- `vel_max = p99 × 1.5~2`
- `qd_warn ≈ vel_max`
- `qd_crit ≈ 1.5 × vel_max`
- 위치(`pos_min/max`)는 기계한계 유지 — 이때 q도 한계 안쪽인지 함께 확인.
</content>
</invoke>
