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

## 2. 매 실행 절차 (bring-up)

윈도우 노트북에서 로봇 컴퓨터로 `ssh`. **터미널 2개**(g1_ctrl, 브릿지) 권장(tmux/screen).

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
- `[safety] SAFE:STALE` 반복 = xrt/PC-Service 미연결 또는 body 미착용 → §7 트러블슈팅.

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
| `xrobotoolkit_sdk not found` | `--transport local` env에 xrt 미설치. `setup_teleop.sh`(XRT_SRC) 재실행. x86은 git-lfs materialize 필요 |
| 브릿지 뜨는데 로봇 무반응 | (1) FSM이 `Mimic_Masked`인지(터미널 `m`) (2) grip 눌렀는지(데드맨) (3) cmd_mode 2/3 진입했는지 |
| IsaacLab/sim에선 걷는데 실로봇 못 걸음 | obs/action term 불일치 가능 → `verify-deploy-obs-parity` 절차. deploy.yaml JOINT_ORDER 확인 |
| GMR 지연으로 끊김 | GMR median ~3.2ms(50Hz 여유). 로봇 컴퓨터 CPU 경합 시 다른 부하 정리. `--gmr-iter` 조정 |
| E-stop 걸었는데 안 풀림 | 우측 menu **1초 이상** 홀드 필요. 브릿지 死 상태면 브릿지 재실행(flag=0 하트비트) 후 `f` |

---

## 8. sim2sim(사전 검증)과의 차이

- **sim2sim**(개발/검증): 노트북(PICO)→com1(시뮬). `g1_ctrl --network=lo` + `--transport zmq`(기본). **하드 E-stop 무장 안 함**(soft mode1 fallback). 실로봇 전 파이프라인 dry-run용.
- **실로봇 온보드**(이 문서): 전부 로컬. `--network=<real_iface>` + `--transport local`. **하드 E-stop 무장**.
- 실로봇 투입 전 반드시 sim2sim으로 정책·obs·조작을 먼저 확인.
</content>
