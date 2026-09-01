# G1 VR Teleop — setup

이 repo를 `git clone`한 뒤 VR 텔레옵을 세팅하는 절차. 현재 실기 토폴로지:

```
[노트북] PICO publisher  ──UDP(--transport udp)──►  [온보드 Jetson] vr_teleop_bridge.py (GMR retarget)  ──/dev/shm──►  g1_ctrl (C++)
                                                        .venv-teleop (deploy/onboard/bootstrap.sh ⑤)      .deps (build_deps.sh)
```

> **bridge 는 온보드에서 돈다:** GMR IK(~15ms)는 g1_ctrl 의 50Hz 제어 루프와 CPU 경합하므로 별도
> 프로세스(다른 코어)로 뺀다 — 단, 지금은 그 프로세스도 g1_ctrl 과 같은 온보드(Jetson)에서 돌고,
> `.venv-teleop` 은 `deploy/onboard/bootstrap.sh` ⑤ 가 `setup_teleop.sh` 로 만든다. 개발용으로
> com1/노트북 리눅스에서도 같은 스크립트로 bridge 를 띄울 수 있다(§B).

---

## A. 로봇 온보드 (C++ g1_ctrl) — clone-to-build

```bash
# 1) 시스템 deps
sudo apt install cmake build-essential libboost-all-dev libyaml-cpp-dev zlib1g-dev libfmt-dev libeigen3-dev
# 2) unitree_sdk2 + CycloneDDS를 repo-local .deps/ 로 빌드 (gitignore라 clone엔 없음 → 1회 실행)
bash deploy/scripts/build_deps.sh
# 3) g1_ctrl 빌드 (onnxruntime aarch64는 vendored, 자동 선택)
cd deploy/robots/g1 && mkdir -p build && cd build && cmake .. && make -j4
# 4) 실행
./g1_ctrl --network=<robot_iface>     # 실로봇 예: enp5s0 / sim2sim: lo
```
ONNX 정책·모션 npz·deploy.yaml·onnxruntime는 **이미 커밋**되어 clone에 포함. 경로는 바이너리 기준 상대(`/proc/self/exe`)라 어디에 두든 동작.

## B. 브릿지 머신(온보드 Jetson 또는 개발 PC) — 1-command 세팅

```bash
# venv + GMR clone + g1 mesh 복구(upstream GMR엔 STL 없음 → repo STL 복사) + editable install + smoke test
bash deploy/robots/g1/teleop/setup_teleop.sh
# 실행 (repo root에서)
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --mode 1
```
> com1에는 기존 `gmr` conda env가 이미 있으므로 그걸 써도 된다(`setup_teleop.sh`는 새 머신용).
> `general_motion_retargeting`은 PyPI에 없어 GitHub(YanjieZe/GMR)에서 clone하며, g1 mesh 38개는 이 repo의 `src/assets/robots/unitree_g1/xmls/assets/*.STL`로 복구된다.

### bridge 컨트롤 매핑 (PICO)
| 입력 | 동작 |
|---|---|
| 왼쪽 스틱 | base_vel vx(앞뒤)/vy(좌우) — mode1·2 |
| 오른쪽 스틱 좌우 | 회전 wz |
| 왼쪽 X / Y | mode1(자율보행) / mode2(상체 teleop) |
| 오른쪽 B | mode3(전신, sim 전용) |
| grip (`--grip-enable`) | 데드맨: 놓으면 mode1 안전 복귀 |

주요 플래그: `--mode`(기본 cmd_mode) · `--gmr-iter`(IK 반복 상한, 기본 10=TWIST2 스톡; warm-start라 저렴) · `--grip-enable` · `--fallback-clip`(끊김 시 clip, 데모용) · `--vx/--vy/--wz`(속도 cap).

> 실시간 성능은 iter가 아니라 **GMR을 sim/제어와 별도 프로세스로 분리**하는 게 핵심(TWIST2 레시피). 실기 토폴로지는 bridge 도 g1_ctrl 도 같은 온보드(Jetson)에서 돌아 VrRef 는 그대로 `/dev/shm`으로 넘어간다 — 네트워크를 타는 건 PICO 입력 쪽(`--transport udp`, 노트북→온보드)뿐이다.

---

## C. 온보드 로컬 텔레옵 (`--transport local`, network-free)

> ⛔ **JetPack 5(glibc 2.31) Jetson 에서는 불가** — PC-Service/xrt 가 glibc 2.34 를 요구해 로드 자체가 안 된다
> (`rules/RUNBOOK_onboard_pico_teleop.md`). center_g1 이 그렇다. 실기는 노트북 publisher → 온보드 bridge
> `--transport udp --arm-estop` (`rules/RUNBOOK_laptop_pico_teleop.md`). 판정은 `deploy/onboard/bootstrap.sh --check`.

로봇 컴퓨터에 PICO를 직접 연결(co-located)한 경우. 네트워크 홉 없이 xrt→GMR→shm.

```bash
# 0) 1회: GMR + xrt 설치 (xrt는 PC-Service pybind 소스빌드; x86은 git-lfs, Jetson은 orin build.sh)
XRT_SRC=<.../XRoboToolkit-PC-Service-Pybind_X86_and_ARM64> bash deploy/robots/g1/teleop/setup_teleop.sh
# 1) XRoboToolkit PC-Service 데몬 실행 + PICO 헤드셋 페어링(body는 트래커 2개 캘리브)
# 2) 온보드에서 g1_ctrl (실로봇 iface) + 브릿지(로컬)
./deploy/robots/g1/tools/run_g1_with_gui.sh <robot_iface>
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport local --grip-enable --mode 1
```

> 텔레옵 정책(`Mimic_Masked`) 자체의 관절 안전장치(위치 clamp / 속도 rate-limit / 측정 qd 폭주
> 감지 3층, 전부 config-gated·기본 off)는 이 E-stop과 별개다 → `rules/SYSTEM_OVERVIEW.md`의
> "텔레옵 관절 안전 3층" 섹션 참고.

### 하드 E-stop (`/dev/shm/g1_estop`)
- 기본(auto)은 `--transport local` 에서만 무장. 네트워크 transport(`udp`/`zmq`)는 **`--arm-estop` 으로 명시 무장 — 실로봇은 필수** (`rules/RUNBOOK_laptop_pico_teleop.md`).
- 우측 A → E-stop 래치(브릿지 SafetyMonitor). 우측 menu 1s 홀드 → 해제 후 `f`로 재기립.
- **지연**: 버튼 E-stop / 워치독(통신불량)은 flag=1을 라이브로 써서 다음 50Hz 폴(≈즉시)에 Passive. **브릿지 프로세스 死(하트비트 정지)만** ~0.5s stale 감지 후 Passive.
- 브릿지가 매 사이클 `estop_shm.write(seq++, flag)` 하트비트. 정상 종료 시 파일 제거(disarm).
- **문제해결**: g1_ctrl이 브릿지 없이 계속 Passive면: 이전 세션의 orphan 파일 → g1_ctrl 재시작(부팅 시 자동 clear) 또는 `rm /dev/shm/g1_estop`.

> sim2sim(노트북→com1)은 기존대로 `--transport zmq`(기본). local은 PICO가 g1_ctrl과 같은 PC일 때만.

---

## 점검 도구
- `tools/pico_sniff.py` — PICO 앱/publisher 가 실제로 무엇을 어디로 쏘는지 (UDP+TCP :5556 동시 리슨, pico_wire 806 B 해석). Hop-2 진단.
- `legacy/` — 안 쓰지만 되살릴 조건이 분명한 것 (`legacy/README.md`).

## 자립성 메모
- 로봇 clone-to-build의 유일한 선행 = `build_deps.sh`(`.deps`는 gitignore).
- bridge/GMR은 laptop 사정 — `.venv-teleop`·`.gmr`는 gitignore, `setup_teleop.sh`로 재현.
- 완전 오프라인 재현(네트워크 없이 clone만)이 필요하면 GMR을 repo에 vendoring(Option 2)으로 승격.

## LAFAN Motion Player (`motion_player/`)

학습에 쓴 LAFAN 클립을 배포 정책의 모션 참조로 재생한다. PICO 없이 동작한다.

```bash
# com1: unitree_mujoco + g1_ctrl 이 떠 있어야 한다. 재기동 전 pkill -x g1_ctrl 필수.
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run   # 계획만
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py            # 실제 송출
```

- **mode2** = 상체만 클립, 다리는 자율 보행 (`base_vel: clip` 이면 클립대로 이동). **실기 첫 시도용.**
- **mode3** = 전신 클립. base_vel 은 C++ 가 0 으로 덮는다.
- **mode1 은 재생 불가** — 참조가 마스킹돼 정책에 도달하지 않는다.
- 재생 중: `Space` 중단(램프아웃 0.8 s), `x` E-stop(즉시 Passive, 복구는 `f` 재기립).
- 클립 목록은 배포 정책의 `ONNX_META.json` → 매니페스트를 따라 자동으로 정해진다.
  슬롯이 바뀌면 목록도 따라 바뀐다. 매니페스트를 못 찾으면 `presets.yaml` 의
  `slot_overrides` 에 실제 경로를 적는다.

테스트: `for t in resolver clips frames publisher playlist; do .venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_$t.py; done`
