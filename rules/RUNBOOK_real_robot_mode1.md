# 실로봇 온보드 운용 매뉴얼 — G1 VR 텔레옵 (mode1 → 2 → 3) + 하드 E-stop

> 대상: Unitree G1 실로봇. **온보드 토폴로지** — PICO를 로봇 옆 컴퓨터에 직접 연결, 그 컴퓨터에서
> `g1_ctrl` + 텔레옵 브릿지가 모두 로컬로 돈다(네트워크 홉 없음). 조작자는 윈도우 노트북에서
> **SSH로 로봇 컴퓨터에 접속**해 실행/재생만 누른다.
> 관련: [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) (구조), repo `docs/superpowers/specs/2026-07-11-onboard-local-vr-teleop-design.md` (설계).

```
[PICO 헤드셋] ─WiFi/USB─ [로봇 컴퓨터: XRoboToolkit PC-Service]
                              │ (로컬)
   vr_teleop_bridge.py --transport local ── xrt→GMR→/dev/shm/g1_vr_ref ──┐
                                          └─ SafetyMonitor→/dev/shm/g1_estop┤
                                                                            ▼
                              g1_ctrl --network=<real_iface> ── Unitree DDS ── [G1 로봇]
[윈도우 노트북] ──SSH──> 로봇 컴퓨터 (터미널에서 위 두 프로세스 실행)
```

---

## ⚠️ 0. 안전 원칙 (먼저 읽기)

- **항상 사람이 하드웨어 E-stop(로봇 전원/리모컨 킬스위치) 옆에 대기.** 소프트 E-stop(우측 A)은 정책·damping 레벨이지 전원 차단이 아니다.
- **처음엔 반드시 로봇을 매달거나(게이지) 넘어져도 되는 환경에서** mode1부터 시작. mode2/3(팔·전신 추종)은 mode1 안정 확인 후.
- **`--grip-enable`(데드맨) 필수 사용**: grip을 놓으면 즉시 안전(mode1)으로 복귀.
- Passive(damping) = kp0/kd3 → 로봇이 **힘을 잃고 주저앉는다**. 서 있는 상태에서 E-stop을 걸면 **넘어질 수 있으므로** 잡거나 낮은 자세에서.

---

## 1. 사전 준비 (로봇 컴퓨터에서 1회)

```bash
# (1) repo clone + 시스템 deps
sudo apt install cmake build-essential libboost-all-dev libyaml-cpp-dev zlib1g-dev libfmt-dev libeigen3-dev
git clone <repo> unitree_rl_mjlab && cd unitree_rl_mjlab
git switch smooth_mode_switch          # 또는 배포 대상 branch

# (2) unitree_sdk2 + CycloneDDS (repo-local .deps, 1회)
bash deploy/scripts/build_deps.sh

# (3) g1_ctrl 빌드 (onnxruntime는 vendored)
cd deploy/robots/g1 && mkdir -p build && cd build && cmake .. && make -j4
cd -                                   # repo root로 복귀

# (4) 텔레옵 env: GMR + xrobotoolkit_sdk 를 한 venv에 (--transport local 필수)
XRT_SRC=<.../XRoboToolkit-PC-Service-Pybind_X86_and_ARM64> \
  bash deploy/robots/g1/teleop/setup_teleop.sh
#   - x86: 사전 git-lfs 로 libPXREARobotSDK.so materialize 필요 (setup 안내 따름)
#   - aarch64(Jetson): setup이 orin 브랜치 build.sh 로 네이티브 lib 빌드
#   - 성공 시: import general_motion_retargeting, xrobotoolkit_sdk 둘 다 OK

# (5) XRoboToolkit PC-Service 설치·실행 + PICO 헤드셋 페어링
#   body 트래킹 쓰려면(mode3) Pico Swift 트래커 2개 이상 캘리브레이션.
#   확인: $XRT_SRC/examples/example.py (컨트롤러) / example_body_tracking.py (24관절)
```

> **자립성**: ONNX 정책·모션 npz·deploy.yaml·onnxruntime는 이미 커밋되어 clone에 포함. 경로는 바이너리 상대(`/proc/self/exe`)라 어디 두든 동작. 유일한 선행 = `build_deps.sh`(.deps는 gitignore) + `setup_teleop.sh`(.venv-teleop·.gmr는 gitignore).

---

## 1.5 PICO 연결 & 전신(body) 트래킹 (매 세션, 헤드셋 안 — **가장 자주 막히는 곳**)

> **핵심 구분**: `streaming`(머리 pose OK) ≠ `body_available`(전신 24관절 OK). 텔레옵은 **body_available=True 여야 활성**.
> `streaming=True`인데 `body=False`면 → 연결은 됐지만 **전신 트래킹 미활성** → 브릿지가 `INACTIVE(mode1 safe)`로 남고 `total 0 teleop frames`. (이건 로봇이 문제가 아니라 PICO 앱 설정 문제.)

1. **로봇 컴퓨터에서 XRoboToolkit PC-Service 실행** (헤드셋이 붙을 대상 = 로봇 컴퓨터).
2. **PICO ↔ PC-Service 연결** (헤드셋 착용하고 앱 안에서):
   - 연결 모드 = **com/USB** (WiFi 아님 — WiFi로 바뀌어 있으면 USB로 안 붙는다). USB 케이블 연결.
   - PC-Service가 헤드셋 인식 → `server connect` + **device found** 떠야 함.
   - (안 뜨면) USB 재연결 / PC-Service 재시작 / **adb 서버 내리기**(`adb kill-server` — adb가 USB 채널을 물고 있으면 PC-Service가 못 붙음).
3. **전신 트래킹 켜기** (헤드셋 안):
   - **Full Body** 모드 선택 (Head만이 아니라).
   - **모션 트래커(Pico Swift)** 페어링 + 착용 (전신 24관절의 소스).
   - **T-pose 캘리브레이션** 수행 ← **이거 안 하면 `body_available=False`**.
   - **Send ON**.
4. **확인** (로봇 컴퓨터에서):
   ```bash
   .venv-teleop/bin/python $XRT_SRC/examples/example_body_tracking.py
   #   -> is_body_data_available() True + 24관절 값이 흐르면 성공
   ```
   그 다음 브릿지(2-3)를 띄우면 `INACTIVE` → **`TELEOP mode=.. arm(...)=..`** 프레임이 뜬다.

> **실로봇(--transport local)은 별도 publisher 없이 브릿지가 xrt를 직접 읽는다.** 그래서 sim2sim(노트북 publisher)과 달리 "publisher body=True"를 볼 필요 없이, **브릿지 자체 로그에서 `body_ok`가 True→TELEOP** 인지로 확인한다. body 트래킹 요구조건(Full Body+트래커+T-pose+Send)은 sim이든 실로봇이든 **동일**하다.

---

## 1.6 실로봇 하드웨어 준비 (매 세션, `g1_ctrl` **실행 전**)

> **핵심(검증됨)**: `g1_ctrl`은 고수준 서비스를 코드로 release하지 않는다(`main.cpp` = DDS init → FSM Passive/damping → LowCmd). 그런데 이 팀의 실로봇 배포는 **L2+R2 없이 잘 동작해 왔다** = **이 로봇은 clean power-on 상태에서 LowCmd가 그냥 먹는다.** (전원 후 zero-torque에서 리모컨 motion 버튼을 안 누르고 키보드로만 제어하면 내장 sport가 적극 개입 안 함 → g1_ctrl LowCmd가 모터를 가져감. 또는 로봇이 sport off 설정.)

**표준 절차 (이 로봇 = 검증된 방식)**:
1. **갠트리에 매단 상태로** 로봇 전원 ON → **zero-torque**(관절 전부 힘 빠짐) 도달까지 대기(~1분). **리모컨 motion 버튼(R2+X 등)은 누르지 말 것** — 내장 sport를 깨우지 않기 위해.
2. **iface 이름 확인** (g1_ctrl `--network=`에 넣을 값):
   ```bash
   ip addr        # 192.168.123.x 를 가진 iface (온보드 Jetson은 보통 eth0)
   ```
   → 온보드(Jetson)는 로봇 내부망에 **이미 붙어 있음**(공장 설정) — IP 수동 설정 불필요, iface 이름만 확인.
3. 바로 §2(g1_ctrl → `f` → `m` + 브릿지). **L2+R2 불필요.**

> **fallback — 새 로봇에서 g1_ctrl "Connected"인데 로봇이 안 움직이거나 떨리면(내장 sport가 적극 개입하는 펌웨어)**: zero-torque에서 **`L2+R2`**(debug mode, README §4.2)로 저수준 활성화, 또는 앱 **sport_mode off**, 또는 SDK **MotionSwitcher `ReleaseMode()`**. (기종/펌웨어 따라 이게 필요할 수 있음 — 이 로봇은 필요 없었다.)
>
> **sim**이면 이 §1.6 전체 생략, `--network=lo`.

---

## 2. 매 실행 절차 (bring-up)

윈도우 노트북에서 로봇 컴퓨터로 `ssh`. **터미널 2개**(g1_ctrl, 브릿지) 권장(tmux/screen).
> ⚠ **실로봇이면 §1.6(전원 ON → `L2+R2` 저수준 활성화)이 먼저.** sim이면 생략하고 `--network=lo`.

### 2-1. g1_ctrl 실행 (터미널 A)
```bash
cd unitree_rl_mjlab
# GUI 동반(브라우저 :8080에서 base_vel/mode 백업 조작 가능) + 터미널 키보드 백업
./deploy/robots/g1/tools/run_g1_with_gui.sh <real_iface>     # 예: enp5s0
#   (GUI 불필요 시) ./deploy/robots/g1/build/g1_ctrl --network=<real_iface>
```
- 부팅 시 `/dev/shm/g1_estop` orphan 파일을 자동 제거한다(이전 세션 잔재로 인한 오작동 방지).
- "Waiting for connection to robot..." → "Connected to robot." 확인.

### 2-2. FSM 진입 (터미널 A 키보드, 또는 리모컨)
| 목표 | 키보드 | 리모컨 |
|---|---|---|
| Passive→FixStand(기립) | `f` | L2+Up |
| FixStand→Velocity(순수 보행) | `v` | R2+A |
| FixStand→Mimic_Masked(텔레옵 정책) | `m` | RB+B |
| 정지(Passive/damping) | `p` | L2+B |

**텔레옵은 `Mimic_Masked`(m) 상태에서.** 여기서 cmd_mode 1/2/3가 동작한다.

### 2-3. 텔레옵 브릿지 실행 (터미널 B)
```bash
cd unitree_rl_mjlab
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py \
    --transport local --grip-enable --mode 1
#   (com1의 기존 gmr conda env를 쓰면) ~/miniconda3/envs/gmr/bin/python 로 실행
```
- 로그 `[bridge] transport=LOCAL  xrt.init() (로컬 PC-Service)` + `TELEOP mode=1 ...`가 수십 Hz로 뜨면 정상.
- 로그 `[bridge] hard E-stop: ARMED ...` 확인 — DISARMED면 §4 하드 E-stop이 안 걸린다.
- `[safety] SAFE:STALE` 반복 = xrt/PC-Service 미연결 또는 body 미착용 → §7 트러블슈팅.

### 2-3b. 변형 — PICO를 **윈도우 노트북**에 연결하는 경우 (sim2sim과 같은 토폴로지)

> 📕 **노트북 쪽 세션 절차·트러블슈팅 전체 → [RUNBOOK_laptop_pico_teleop.md](RUNBOOK_laptop_pico_teleop.md)**
> (PC-Service 실행, 앱 IP 칸에 뭘 넣나, `adb reverse`, Full Body/캘리브/Send, TCP failed 진단 4종)

PICO를 로봇 컴퓨터가 아니라 **노트북에 그대로 물린 채** 실로봇을 돌릴 수 있다. 제약은 하나 —
**브릿지와 `g1_ctrl`은 같은 호스트**여야 한다(`/dev/shm` 공유). 즉 로봇에 유선으로 붙은 제어
PC(com1 또는 온보드 Jetson)에서 둘 다 돌리고, PICO 데이터만 노트북→그 PC로 네트워크(:5556).
그 PC엔 GMR만 있으면 되고 `xrobotoolkit_sdk`는 불필요하다.

```bash
# (노트북) publisher — PICO 읽어서 제어PC로 발행
python pico_publisher_udp.py --com1 <제어PC IP> --port 5556

# (제어PC) 브릿지 — UDP 수신 + 하드 E-stop 명시 무장
~/miniconda3/envs/gmr/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py \
    --transport udp --arm-estop --grip-enable --mode 1
```

- **`--arm-estop` 필수**: 하드 E-stop 무장은 기본이 auto(=`--transport local`일 때만)이라,
  네트워크 transport에서는 명시하지 않으면 **DISARMED**(soft mode1 폴백만)로 뜬다.
  무장하면 §4의 A버튼 래치 + 브릿지 死 하트비트 fail-safe가 온보드와 동일하게 동작한다.
- **워치독은 기본 soft로 남는다**(`estop-on-watchdog` auto=False on network): WiFi hiccup
  하나가 곧장 강제 Passive(주저앉기)가 되는 게 더 위험하기 때문. 통신이 끊기면 mode1(제자리)
  으로만 폴백한다. 온보드와 같이 통신불량도 하드 정지로 올리려면 `--estop-on-watchdog`.
- ⚠ **네트워크가 안전 경로에 들어온다**: WiFi가 끊기면 A버튼 E-stop 자체가 전달되지 않는다.
  → 이 토폴로지에서는 **하드웨어 킬스위치 담당자 대기가 특히 필수**(§0).
- transport는 **udp 권장**(TCP head-of-line stall 제거 + age 기반 staleness 감지). 노트북
  publisher가 구 ZMQ 버전뿐이면 `--transport zmq`.
- ⚠ publisher(`pico_publisher_udp.py`)는 **repo 밖(노트북)에만 있다**. 노트북을 계속 쓰는 한
  문제없지만, 다른 PC에서 발행하려면 그 파일을 옮겨야 한다.

---

## 3. 조작 매핑 (PICO)

| 입력 | 동작 |
|---|---|
| 왼쪽 썸스틱 | base_vel vx(앞뒤) / vy(좌우) |
| 오른쪽 썸스틱 좌우 | 회전 wz |
| 왼쪽 X | mode1 (자율보행) |
| 왼쪽 Y | mode2 (상체 teleop: 팔·waist 추종, 다리 자율) |
| 오른쪽 B | mode3 (전신 teleop: 다리까지 추종) |
| grip (좌/우, 데드맨) | 눌러야 텔레옵 활성. 놓으면 안전 mode1 |
| **오른쪽 A** | **E-stop 래치** (§4) |
| 오른쪽 menu (1초 홀드) | E-stop 해제 (§4) |

**권장 진행 순서: mode1(자율보행 안정 확인) → mode2(상체만) → mode3(전신).** 각 단계에서 넘어짐 대비.

---

## 4. 하드 E-stop (핵심 안전)

**래치 권한은 브릿지 SafetyMonitor 한 곳**, C++(g1_ctrl)은 flag를 미러 + 하트비트 fail-safe.

| 트리거 | 지연 | 결과 |
|---|---|---|
| **우측 A 버튼** | ≈즉시(다음 50Hz 폴) | flag=1 래치 → g1_ctrl **강제 Passive(damping)** |
| **워치독**(통신불량/SDK 스톨, rate<30Hz 또는 age>200ms 지속) | ≈즉시 | flag=1 → 강제 Passive |
| **브릿지 프로세스 死**(크래시/kill) | ~0.5s | 하트비트 stale → g1_ctrl fail-safe **강제 Passive** |

> **무장 조건**: 위 표는 브릿지가 `/dev/shm/g1_estop`을 쓰는 동안(=무장)만 성립한다. 무장은
> 기본 auto = `--transport local`일 때만. **네트워크 transport(노트북 PICO)는 `--arm-estop`을
> 줘야 무장**된다(§2-3b). 시작 로그 `[bridge] hard E-stop: ARMED / DISARMED`로 확인할 것.
> 워치독(통신불량) 행은 무장 + `estop_on_watchdog`일 때만 하드 Passive이고, 네트워크 기본값은
> soft mode1이다.

**해제 & 재기립**:
1. 우측 **menu 버튼 1초 홀드** → SafetyMonitor E-stop 해제(flag=0) → g1_ctrl 강제 중단.
2. 로봇은 이미 Passive(damping) 상태 → 키보드 **`f`**(FixStand)로 재기립.
3. 다시 `m`(Mimic_Masked) → 텔레옵 재개.

> **주의**: E-stop 중엔 전이검사가 bypass되어 자동 재보행이 차단된다(안전). 반드시 수동 재기립.

---

## 5. 종료 절차

1. 로봇을 안전 자세로(mode1 정지 또는 낮은 자세).
2. 터미널 A: `p`(Passive) → 로봇 damping.
3. 터미널 B: 브릿지 **Ctrl-C** → 정상 종료 시 `/dev/shm/g1_estop` 자동 제거(disarm).
4. 터미널 A: `g1_ctrl` Ctrl-C.
5. 로봇 전원 차단.

---

## 6. 안전 체크리스트 (매 실행 전)

- [ ] 하드웨어 킬스위치 담당자 대기
- [ ] 로봇 게이지/안전 환경, 넘어짐 대비
- [ ] **(실로봇) 전원 ON → zero-torque (리모컨 motion 버튼 안 누름)** — §1.6. 이 로봇은 이 상태에서 LowCmd 바로 먹음(L2+R2 불필요). sim은 생략
- [ ] `--grip-enable` 플래그 포함했는지
- [ ] g1_ctrl `--network=<real_iface>` (sim의 `lo` 아님) 확인
- [ ] 브릿지 로그 `transport=LOCAL` + `TELEOP` 프레임 정상
- [ ] mode1 자율보행부터. mode2/3는 안정 확인 후
- [ ] E-stop(우측 A) 즉시 눌러 damping 되는지 **처음에 1회 테스트**

---

## 7. 트러블슈팅

| 증상 | 원인 / 조치 |
|---|---|
| g1_ctrl이 브릿지 없는데 계속 Passive | 이전 세션 orphan `/dev/shm/g1_estop`. → g1_ctrl 재시작(부팅 시 자동 clear) 또는 `rm /dev/shm/g1_estop` |
| 브릿지 `[safety] SAFE:STALE` 반복 | PC-Service 미실행 / PICO 미착용 / body tracking off / 트래커 미캘리브. `example_body_tracking.py`로 body 데이터 확인 |
| **연결·수신은 되는데 `body_ok=False` + `INACTIVE mode1 safe` + `total 0 teleop frames`** | **가장 흔함.** streaming(머리)은 OK인데 전신 트래킹 미활성. → §1.5: **Full Body 선택 + 트래커 착용 + T-pose 재캘리브 + Send ON**. 로봇/브릿지 문제 아님(PICO 앱 설정). |
| `device found` 안 뜸 / 앱 `IDLE` | 앱↔PC-Service 미연결. 연결 모드 **com/USB** 확인(WiFi 아님), USB 재연결, PC-Service 재시작, `adb kill-server`(USB 채널 충돌 방지). |
| `xrobotoolkit_sdk not found` | `--transport local` env에 xrt 미설치. `setup_teleop.sh`(XRT_SRC) 재실행. x86은 git-lfs materialize 필요 |
| 브릿지 뜨는데 로봇 무반응 | (1) FSM이 `Mimic_Masked`인지(터미널 `m`) (2) grip 눌렀는지(데드맨) (3) cmd_mode 2/3 진입했는지 |
| IsaacLab/sim에선 걷는데 실로봇 못 걸음 | obs/action term 불일치 가능 → `verify-deploy-obs-parity` 절차. deploy.yaml JOINT_ORDER 확인 |
| GMR 지연으로 끊김 | GMR median ~3.2ms(50Hz 여유). 로봇 컴퓨터 CPU 경합 시 다른 부하 정리. `--gmr-iter` 조정 |
| E-stop 걸었는데 안 풀림 | 우측 menu **1초 이상** 홀드 필요. 브릿지 死 상태면 브릿지 재실행(flag=0 하트비트) 후 `f` |

---

## 8. sim2sim(사전 검증)과의 차이

| | sim2sim (개발/검증) | 실로봇 온보드 (이 문서) |
|---|---|---|
| **저수준 활성화** | 개념 자체 없음(실물 모터 없음) | 이 로봇은 clean power-on에서 LowCmd 바로 먹음 → **L2+R2 불필요**. 새 로봇이 방해하면 그때만 fallback(§1.6) |
| g1_ctrl network | `--network=lo` | `--network=<real_iface>` |
| PICO 데이터 | 노트북(PICO)→com1, `--transport udp`(또는 zmq) | 로봇 직결, `--transport local` / 노트북 유지면 §2-3b |
| 하드 E-stop | 무장 안 함(soft mode1 fallback) | **무장** (local=auto, 네트워크는 `--arm-estop`) |
| 실물 모터 | 없음(mujoco) | 있음 → 갠트리·킬스위치 필수 |

- **실로봇 투입 전 반드시 sim2sim으로 정책·obs·조작을 먼저 확인.** (당신은 지금까지 이 sim 단계에 있었고, 그래서 L2+R2를 안 해봤다 — 실로봇에서 §1.6이 새로 필요하다.)

<!-- AI_APPENDED -->

---

## 9. 터미널 키보드가 안 먹을 때 (2026-08-13)

`f`/`v`/`m`/`p` 가 안 먹으면 **코드를 뒤지기 전에 30초 안에 판정된다.** 컨트롤러가 tty를
`-echo` 로 두기 때문에 화면에 아무 흔적도 안 남아 전부 "키가 죽었다"로 보인다.

```bash
P=$(pgrep -x g1_ctrl)
for t in $(ls /proc/$P/task); do echo -n "$t "; awk '/^syscr/{print $2}' /proc/$P/task/$t/io; done
```

Keyboard 스레드(= `wchan` 이 `poll_schedule_timeout` 이고 read 수가 작은 쪽)의 `syscr` 로 갈린다:

| 관측 | 원인 | 조치 |
|---|---|---|
| `syscr` 가 **안 늘어남** | 키가 프로세스까지 도달 못 함 | 포커스/창 확인. 그래도 안 되면 아래 X 레벨 캡처 |
| `syscr` 가 **3씩** 늘어남 | **한글 IME** (UTF-8 한글 = 3바이트. `f`→`ㄹ`) — `keyboard.h` 는 1바이트씩 읽어 `"f"` 와 영영 불일치 | 한/영 전환 |
| `syscr` 가 **1씩** 느는데 `FSM: Change state ...` 로그가 없음 | 그때 비로소 코드/상태 문제 | `/dev/shm/g1_estop` 확인(asserted면 `CtrlFSM::run_()` 이 전이검사 bypass) |

**특정 키 하나만** 안 먹으면 X 입력 레벨이다 (AnyDesk 원격은 물리 키보드가 아니라
`Keyboard passthrough` 가상 장치로 들어와 스캔코드가 어긋날 수 있다):

```bash
DISPLAY=:1 timeout 30 xinput test-xi2 --root > /tmp/keys.log   # 그동안 문제 키를 누른다
awk '/EVENT type .*KeyPress/{f=1} f&&/detail:/{print $2; f=0}' /tmp/keys.log | sort -n | uniq -c
DISPLAY=:1 xmodmap -pke | grep -E "^keycode +(41|<잡힌값>) "     # 41 = f
```

2026-08-13 실제: `f` 가 keycode **93**(evdev 85 `KEY_ZENKAKUHANKAKU`, keysym 비어 있음)으로
도착 → 어느 앱에도 안 찍힘. 우회 `DISPLAY=:1 xmodmap -e 'keycode 93 = f F f F'`
(되돌리기 `keycode 93 =`, X 세션 재시작 시 소멸). 근본 해결은 AnyDesk 재연결 / 노트북 레이아웃 점검.

### 종료 후 터미널이 깨지는 문제 (해결됨, `3b79d12`)

`g1_ctrl` 이 키 텔레옵을 위해 tty 를 `-icanon -echo` 로 바꾸는데 Ctrl-C 는 소멸자를 안 태워
셸이 raw 로 남았다. SIGINT/SIGTERM 핸들러에서 termios 를 복구하도록 고쳤고,
`run_g1_with_gui.sh` 도 `stty -g` 로 이중 안전망을 둔다. 구버전 바이너리로 깨졌으면 `stty sane`.

---

## 10. base_vel 캡 — 세 입력 경로 (2026-08-13 상향)

| 경로 | 풀스틱/슬라이더 최대 (vx, vy, wz) | 위치 |
|---|---|---|
| PICO 썸스틱 (브릿지) | **2.5 / 0.8 / 2.0** | `vr_teleop_bridge.py` `--vx/--vy/--wz` 기본값 |
| 브라우저 GUI 슬라이더 | +2.5 / −1.5 / 0.8 / 2.0 | `tools/gui_shm.py` `VXCAP/VXCAP_BWD/VYCAP/WCAP` |
| 게임패드 스틱 | +2.5 / −1.5 / 0.8 / 2.0 | `State_Mimic.cpp` `g_joystick_base_vel` |
| **C++ 하드캡(최종 클램프)** | **+2.5 / −1.5 / 0.8 / 2.0** | `State_Mimic.cpp` `VX_MAX_FWD/VX_MAX_BWD/KB_MAXVY/KB_MAXW` → `clamp_vx` |

- 🔴 **하드캡 = 학습 봉투** `stage4_mode1_env_cfg.CMD_BASE_VEL` = `vx(-1.5, 2.5) · vy(-0.8, 0.8) · wz(-2.0, 2.0)`.
  **vx 는 비대칭이다** — 후진 상한이 전진의 60 % 다.
- ⚠ 2026-08-25 이전 값(3.0 / 1.5 / 2.0)의 근거는 «20-motion manifest 클립 속도 p99» 였는데,
  지금 mode1 은 클립이 아니라 `CMD_BASE_VEL` 로 학습한다 = **낡은 근거**였다.
  그 값으로는 후진이 학습의 **2.0배**, 횡이 **1.9배** OOD 로 나갔다.
- ⚠ **mode2 봉투는 더 좁다**(`stage4_mode2_env_cfg` — `vx(-1.0, 1.5)`). 위 캡은 mode1 기준이라
  mode2 에선 여전히 전진 1.67배가 가능하다. 모드별 봉투 분리는 미구현.
- ⚠ 최종 클램프는 `g_kb_* + 게임패드 스틱`의 **합**에 걸린다 → 패드가 꽂혀 있으면 데드존(0.08)
  밖 드리프트가 PICO 명령에 더해진다.
- 낮추려면 브릿지 실행에 `--vx 1.5` 처럼 준다(코드 수정 불필요). 2026-08-13 이전 기본값은
  1.5 / 0.8 / 1.5 였으므로, 같은 스틱 각도에서 나가는 속도가 vx 기준 1.67배가 됐다.
</content>

## 11. IMU 편향 보정(`imu_cal`) — 2026-09-03: **켜지 않았다.** 켜기 전 판별 절차

**있는 것**: `config.yaml: imu_cal` (기본 0 = 꺼짐), sim(`--network=lo`)에서는 자동 꺼짐,
환경변수 `G1_IMU_CAL_DEG="pitch,roll"` 이 config 를 덮음, 측정기 `deploy/scripts/imu_bias_fit.py`.

**왜 안 켰나** — «IMU 가 4.2° 기울어 붙었다» 는 가설이 정책 OFF 구간에서 깨졌다:

| 구간 | 발바닥 법선 vs IMU 중력 (pitch 잔차) |
|---|---|
| 정책 ON (Mimic, kp 40) 정지 | **−3.0°** (118 블록 중앙값, 3일·방위 345°) |
| 정책 OFF (FixStand, kp 80/100/150) | **−0.5 ± 1°** |

장착 편향이면 정책과 무관하게 같아야 한다. 정책이 켜졌을 때만 −3° 면 **정책이 선 자세**(뒤꿈치
하중·발끝 들림)다. 그 상태에서 «보정» 을 걸면 로봇을 **더 뒤로** 민다(sim 주입: 골반 −2.5°, CoM −3.3 cm).

**판별 (로봇 앞에서 2 분)**
1. FixStand(`f`) 로 세우고 **발이 눈으로 평평하게 바닥에 닿고 하네스 장력이 0** 인지 본다.
2. 20 초 정지 → 로봇을 **180° 돌려** 다시 20 초. (온보드 로거가 돌고 있으면 그걸로 충분, `--dump` 불필요.)
3. 회수 후: `uv run --no-project --with pyarrow --with numpy --with mujoco python deploy/scripts/imu_bias_fit.py --onboard <그 시각 parquet>`
   → kp 100 블록의 P 를 본다.
   - 두 방위 모두 **|P| < 1°** → 편향 없음. 이 블록은 0 그대로. 뒤로 기움은 다른 원인.
   - 두 방위 모두 **−3° 이하로 같은 부호** → 장착 편향. 아래 «켜는 절차» 로.
   - 두 방위가 **부호 반대** → 바닥 경사. 편향 아님.

**켜는 절차 (편향이 확인된 경우에만)**
- 스크립트가 찍는 b 를 **그대로** 넣는다(`pitch_deg: -4.2` 처럼). **부호를 뒤집지 않는다** —
  2026-09-01 노트의 «뒤집어 넣으라» 는 틀렸다(실데이터에 ImuCal(−4.22) 가 잔차를 0 으로, +4.22 는 두 배로).
- 첫 실행은 **하네스 + 절반**, config 는 두고 환경변수로: `G1_IMU_CAL_DEG="-2.1,0" ./tools/run_g1_with_gui.sh eth0 --policy v1 --dump`
  기동 로그에 `[imu_cal] pitch=-2.10° … ← 환경변수 override` 가 찍혀야 한다.
- 판정: 뒤로 기움·정지 중 잔발이 **줄면** 계속(그다음 전량), **늘면** 즉시 `p`. 부호가 반대면 넘어진다(sim 65°).

