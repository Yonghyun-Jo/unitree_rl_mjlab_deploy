# 온보드 로컬 VR 텔레옵 (network-free) + 하드 E-stop — 설계 spec

- 날짜: 2026-07-11
- repo: `unitree_rl_mjlab`, branch: `wose_obs`
- 관련 문서: [`rules/SYSTEM_OVERVIEW.md`](../../../rules/SYSTEM_OVERVIEW.md)
- 상태: 설계 승인됨(사용자) → 사실관계 검증 완료(5-agent adversarial verify) → **구현 plan 대기**

---

## 1. 목표

로봇 컴퓨터에 PICO가 **직접 연결**되고 이 repo를 그 컴퓨터에 clone하는 **온보드(co-located)** 배포에서,
네트워크 홉 없이 **xrt(PICO) → GMR retarget → `/dev/shm/g1_vr_ref` → g1_ctrl** 로 도는 통합 로컬 텔레옵을 만든다.
동시에 E-stop/워치독 발동 시 **실제 로봇을 하드 damping**(강제 Passive)한다.

**핵심 제약(사용자 확정)**
- 기존 네트워크 경로(zmq/udp, sim2sim 노트북→com1)는 **공존**한다. `--transport local`로 스위치.
- E-stop(우측 A)·워치독(통신끊김/SDK 스톨) → **하드 damping**.

---

## 2. 검증된 사실 (5-agent adversarial verify, 2026-07-11)

모든 주장 file:line 확인. 교정된 항목은 ⚠.

**receiver 계약 (`vr_teleop_bridge.py`)**
- `main()`이 receiver를 건드리는 지점은 `latest()`(:364), `age_ms()`(:365), `close()`(:440) **3개뿐** → `XrtReceiver`의 공개 표면도 이 3개.
- ⚠ `seq`는 매 프레임 **증가 필수**. `main()`은 seq를 안 읽지만 `SafetyMonitor.update()`(teleop_safety.py:55)가 `seq != last_seq`일 때만 도착으로 기록 → seq 고정이면 rate=0 → `rate<30` → **영구 SAFE mode1 래치**, 텔레옵 영영 미활성.
- ⚠ `controllers.right.primary`(safety:62), `controllers.right.menu`(safety:67), `L/R["axis"]`(bridge:393)는 **하드 브래킷**(KeyError 위험). → `ZmqReceiver.latest()`의 setdefault 백필 블록(bridge:114-121)을 마지막에 그대로 실행해야 안전.
- body.joints는 **raw xyzw**(Unity 프레임) 그대로 emit. Unity→RH + xyzw→wxyz 변환은 브릿지의 `_msg_body_to_human`(bridge:63-79)이 담당(=네트워크 경로와 동일).

**xrt API (pybind `py_bindings.cpp`, 24 getter 전수확인)**
- 전부 정확한 이름으로 존재. 교정 3건:
  - ⚠ `xrt.init()` **인자 없음** — `PXREAInit(NULL,...)`로 **로컬 PC-Service 전용**. 원격 host 인자 불가.
  - ⚠ primary/secondary 전용 getter 없음 → `get_X_button`(L primary), `get_Y_button`(L secondary), `get_A_button`(R primary), `get_B_button`(R secondary).
  - ⚠ teardown은 `xrt.close()` (`deinit` 아님).
- 확인된 getter: `get_headset_pose()`→[7], `get_left/right_controller_pose()`→[7], `get_left/right_trigger()`→float, `get_left/right_grip()`→float, `get_left/right_axis()`→[2], `get_left/right_axis_click()`→bool, `get_left/right_menu_button()`→bool, `is_body_data_available()`→bool, `get_body_joints_pose()`→24×7[x,y,z,qx,qy,qz,qw], `get_body_timestamp_ns()`→int. 모든 pose는 scalar-last(xyzw).

**C++ 강제 Passive (`CtrlFSM.h`/`FSMState.h`/`config.yaml`)**
- `run_()`(CtrlFSM.h:82-112)는 **dt=0.001(1kHz) 전 상태 공통 choke point**. pre_run/run/post_run + 전이검사 전부 여기.
- `states`는 **public** `std::vector<shared_ptr<BaseState>>`(:78), id로 스캔(`state->isState(id)`, BaseState.h:31). `currentState`는 **private, setter 없음**(:115) → **E-stop은 CtrlFSM 멤버(또는 run_ 인라인)** 여야 함.
- 상태 스왑 관용구 `exit()/assign/enter()`가 이미 :99-112에 존재 → 재사용.
- Passive id=1(config.yaml:4), `FSMStringMap.right.at("Passive")==1`(FSMState.h:77 관용).
- Passive = **순수 damping**: kp 키 없음, kd 전관절 3, mode 전관절 1, target pose 없음(config.yaml:20-38). **강제 Passive == 관절 damping.**

**shm 폴링 패턴 (`State_Mimic.cpp`)**
- `g_poll_gui()`(:65-73): `fopen("rb")→fread(&s,sizeof,1,f)→fclose`, `#pragma pack(push,1)` 구조체, `n!=1 || magic!=... || seq==last` 검증 → estop 템플릿.
- Python 계약: `struct.pack`+`os.replace` 원자적 write(gui_shm.py:32-38, vr_shm.py:25-30). magic 사용중: 0x6701(gui), 0x6702(vr) → **estop=0x6703**.
- 하트비트 fail-safe 선례: `g_poll_vr` `VR_STALE_MAX=25(~0.5s@50Hz)`(:105-127). **단 안전 기본값이 반대**: VR는 dead-writer→release(permissive), **E-stop은 dead-writer→damp(restrictive)**.

**셋업 현실 (`setup_teleop.sh`/`install_pico.sh`/pybind repo)**
- ⚠ `xrobotoolkit_sdk`는 **pip 휠 아님** — pybind11/CMake **소스 빌드**(`pip install --no-build-isolation -e $XRT`).
- ⚠ 동봉 `libPXREARobotSDK.so`(x86 & aarch64 둘 다 존재)는 **git-LFS 포인터 스텁**(133B). com1에 **git-lfs 미설치** → materialize(`git lfs pull`) 또는 소스 빌드 필요.
- aarch64(Jetson): `orin` 브랜치 `build.sh`로 네이티브 lib 빌드(install_pico.sh:73-92).
- gmr conda env엔 xrt 없음(확인). GMR은 xrt 없어도 import됨.
- ⚠ `setup_teleop.sh`엔 xrt 스텝 없음 → **신규 스텝 필요**. system-python venv는 python3-dev(Python.h)+C++17 필요(conda는 헤더 있음).
- xrt 런타임 전제: **PC-Service 데몬 로컬 실행 + PICO 페어링**. com1(PICO=노트북)에선 merged 프로세스가 **smoke만** 가능(라이브 캡처는 headset co-located 머신에서만).

---

## 3. 아키텍처

```
[PICO 헤드셋] ─WiFi/USB─ [XRoboToolkit PC-Service (로컬 데몬)]       (전부 로봇 컴퓨터)
        │ xrobotoolkit_sdk (같은 프로세스, xrt.init() 로컬)
        ▼
[vr_teleop_bridge.py  --transport local]
   ┌─ XrtReceiver.latest() ─ xrt getter → frame dict(raw xyzw, seq++)
   ├─ (기존) _msg_body_to_human → GMR.retarget → root_quat+dof_pos[29]
   ├─ (기존) One-Euro smooth thread 50Hz → vr_shm.write(/dev/shm/g1_vr_ref)
   └─ (기존) SafetyMonitor(워치독+E-stop) → estop_shm.write(seq++, flag)  ← 신규
        │ /dev/shm/g1_vr_ref (기존)                    │ /dev/shm/g1_estop (신규)
        ▼                                              ▼
[g1_ctrl (C++)]
   State_Mimic::g_poll_vr()(기존, 50Hz)     CtrlFSM::run_() g_estop_asserted_()(신규, 1kHz)
                                            asserted → 강제 Passive(damping) + 전이검사 bypass
```

**원칙**: 네트워크 경로와 **receiver만 다르고** GMR·안전·스무딩·`vr_shm` 계약은 100% 재사용.

---

## 4. 컴포넌트 상세

### C1. `XrtReceiver` (Python, `teleop/vr_teleop_bridge.py` 내 신규 클래스)

공개 표면 `latest()/age_ms()/close()`. `__init__`에서 `xrt.init()`(ImportError/RuntimeError 시 명확한 SystemExit, pico_control_bridge 패턴).

`latest()` — 매 호출 xrt 현재상태로 frame dict 구성, **seq 내부 카운터++**:
```python
def latest(self):
    self._seq += 1
    body_ok = xrt.is_body_data_available()
    joints = xrt.get_body_joints_pose() if body_ok else None   # 24×7 raw xyzw
    def ctrl(pose, trig, grip, axis, aclick, menu, primary, secondary):
        return {"pose": list(pose), "trigger": float(trig), "grip": float(grip),
                "axis": [float(axis[0]), float(axis[1])], "axis_click": bool(aclick),
                "menu": bool(menu), "primary": bool(primary), "secondary": bool(secondary)}
    f = {
        "seq": self._seq,
        "streaming": bool(body_ok),          # tracking valid일 때 True
        "headset": list(xrt.get_headset_pose()),
        "controllers": {
            "left":  ctrl(xrt.get_left_controller_pose(),  xrt.get_left_trigger(),  xrt.get_left_grip(),
                          xrt.get_left_axis(),  xrt.get_left_axis_click(),  xrt.get_left_menu_button(),
                          xrt.get_X_button(), xrt.get_Y_button()),
            "right": ctrl(xrt.get_right_controller_pose(), xrt.get_right_trigger(), xrt.get_right_grip(),
                          xrt.get_right_axis(), xrt.get_right_axis_click(), xrt.get_right_menu_button(),
                          xrt.get_A_button(), xrt.get_B_button()),
        },
        "body": {"available": bool(body_ok), "joints": [list(j) for j in joints] if joints else None},
    }
    # KeyError 방지: ZmqReceiver.latest()의 setdefault 백필 블록(bridge:114-121)과 동일하게 마지막에 backfill
    self._backfill_controls(f)
    self._last_bts = xrt.get_body_timestamp_ns()
    self._last_rx = time.monotonic()
    return f
```
- `age_ms()`: `get_body_timestamp_ns()`가 정지하면(SDK 스톨) age↑. 구현: 마지막으로 bts가 **바뀐** 시각 기준 `(monotonic - last_change)*1000`. 첫 샘플 전 `inf`.
- `close()`: `xrt.close()`.
- ⚠ body는 raw xyzw 그대로(변환 금지). 브릿지 `_msg_body_to_human(transform=True)`이 처리.
- **정책 결정**: `latest()`는 항상 dict 반환(로컬 xrt는 늘 "현재값" 존재). None 반환 안 함 — 대신 age_ms/streaming/body.available로 유효성 표현.

### C2. `--transport local` 배선 (`main()`)
- argparse choices에 `"local"` 추가(:187).
- ⚠ 현재 `if udp / else zmq` 구조 → **명시적 `elif`** 삽입(안 그러면 local이 else→zmq로 샘):
```python
if args.transport == "udp":
    from udp_receiver import UdpReceiver; rx = UdpReceiver(port=args.port)
elif args.transport == "local":
    import xrobotoolkit_sdk as xrt   # 지역 import(설치 안 된 머신에서 udp/zmq는 계속 동작)
    xrt.init(); rx = XrtReceiver()
else:
    rx = ZmqReceiver(port=args.port)
```
- 온보드는 `--grip-enable` 데드맨 기본 권장(실로봇 안전).

### C3. `teleop/estop_shm.py` (신규, gui_shm/vr_shm 계약 미러)
```python
SHM_PATH = "/dev/shm/g1_estop"
MAGIC = 0x6703                 # 0x6701 gui, 0x6702 vr, 0x6703 estop
FMT = "<iIi"                   # magic seq flag = 12 bytes
def write(seq: int, flag: int) -> None:
    buf = struct.pack(FMT, MAGIC, int(seq), int(flag))
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f: f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish
def clear() -> None:           # 정상 종료 시 disarm(파일 제거)
    try: os.remove(SHM_PATH)
    except FileNotFoundError: pass
```

### C4. 하트비트 writer (bridge `main()` 루프)
- 매 루프 iteration에서 `dec = safety.update(...)` 직후 1회 write(하트비트):
```python
estop_seq += 1
estop_shm.write(estop_seq, 1 if dec["mode"] is not None else 0)   # mode not None = estop 또는 워치독 safe
```
- 프레임이 끊겨도(rx 항상 dict지만 age_ms↑ → safety가 safe) **프로세스가 살아있는 한 하트비트 지속**. 프로세스 死 시에만 정지.
- `finally`에서 `estop_shm.clear()` (정상 종료 = graceful disarm).

### C5. C++ `g_estop_asserted_()` + `CtrlFSM::run_()` 강제 Passive
- `EstopCtrl` 구조체(`#pragma pack(1)`, `int32 magic; uint32 seq; int32 flag`, 12B), `g_poll_gui` fopen/fread 미러.
- **assertion = (fresh flag!=0) OR (heartbeat stale > MAX)**. **영구 C++ 래치 없음**(§5 근거):
```cpp
// CtrlFSM 멤버(currentState mutate 필요 → 멤버여야 함)
uint32_t estop_last_seq_ = 0; int estop_stale_ = 0;
static constexpr int ESTOP_STALE_MAX = 25;   // ~0.5s @50Hz heartbeat (VR_STALE_MAX와 동일)
bool g_estop_asserted_() {
    FILE* f = std::fopen("/dev/shm/g1_estop", "rb");
    if (!f) { estop_stale_ = 0; return false; }          // 파일 없음 = 브릿지 미실행(disarmed)
    EstopCtrl e{}; size_t n = std::fread(&e, sizeof(e), 1, f); std::fclose(f);
    if (n != 1 || e.magic != 0x6703) return false;       // 손상/짧은읽기 무시
    if (e.seq == estop_last_seq_) {                       // 하트비트 정지 → writer 死
        if (++estop_stale_ > ESTOP_STALE_MAX) return true;   // fail-safe: damp
        return false;
    }
    estop_last_seq_ = e.seq; estop_stale_ = 0;
    return e.flag != 0;
}
void run_() {
    if (g_estop_asserted_()) {
        int pid = FSMStringMap.right.at("Passive");       // ==1
        if (!currentState->isState(pid))
            for (auto& s : states) if (s->isState(pid)) {
                spdlog::warn("FSM: E-STOP -> Passive from {}", currentState->getStateString());
                currentState->exit(); currentState = s; currentState->enter(); break;
            }
        currentState->pre_run(); currentState->run(); currentState->post_run();
        return;                                            // 전이검사 bypass(asserted 동안 이탈 금지)
    }
    // ── 기존 정상 경로(CtrlFSM.h:84-112) 그대로 ──
    currentState->pre_run(); currentState->run(); currentState->post_run();
    /* ... 기존 nextStateMode 루프 ... */
}
```
- `#include <cstdio>` 추가.

---

## 5. E-stop 안전 시맨틱 (설계 결정)

**래치 권한 = 브릿지 `SafetyMonitor`(단일 소스).** C++는 flag를 미러할 뿐, **영구 래치를 두지 않는다.**

근거 및 흐름:
- **버튼 E-stop(우측 A)**: SafetyMonitor가 estop 래치(teleop_safety.py) → flag=1 지속 → C++ 강제 Passive. **해제**: 우측 menu 1s 홀드 → SafetyMonitor estop 해제 → flag=0 → C++ 미assert(강제 중단). 로봇은 이미 Passive(damping) → 조작자가 키보드 `f`로 재기립.
- **워치독(SDK 스톨/통신불량)**: SafetyMonitor 히스테리시스(rate<30 지속 또는 age>200ms)로 safe 판정 → flag=1 → damping. 단발 hiccup은 스무더가 흡수(트립 안 함). 회복 시 flag=0 → 미assert.
- **브릿지 프로세스 死(크래시)**: 하트비트 정지 → C++ stale>MAX → **assert(fail-safe damping)**. 브릿지 재시작해 flag=0 하트비트 오면 해제. 계속 死면 Passive 유지.
- **정상 종료(Ctrl+C)**: `estop_shm.clear()`로 파일 제거 → C++ 파일없음=disarmed → 정상 FSM 반환.

**왜 영구 C++ 래치를 안 두나**: Agent 3/4 템플릿의 `estop_latched_`는 해제에 **g1_ctrl 재시작**이 필요 → 워치독 단발/조작 실수에 과중. assertion-미러 방식은 (a) 래치 소스를 브릿지 한 곳에 두고 (b) fail-safe(브릿지 死)를 하트비트로 보장하며 (c) 재기립이 키보드 `f`로 가능. asserted 동안엔 전이검사를 bypass하므로 자동 재보행 위험 없음(조작자 수동 재기립만 허용).

---

## 6. 셋업 (전제 조건)

**한 Python env에 GMR + xrobotoolkit_sdk 공존** 필요.

- (A) com1 sim 드라이런(x86): gmr conda env에 xrt 추가.
  ```bash
  XRT=/home/piene/reference_code/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64
  conda activate gmr && pip install cmake pybind11 setuptools
  export CMAKE_PREFIX_PATH="$(python -m pybind11 --cmakedir)"
  sudo apt-get install -y git-lfs && git lfs install
  git -C <GR00T repo> lfs pull --include="**/lib/libPXREARobotSDK.so"   # 스텁→ELF materialize
  file "$XRT/lib/libPXREARobotSDK.so"   # "ELF 64-bit x86-64" 확인
  pip install --no-build-isolation -e "$XRT/"
  ```
  LFS 불가 시: `XRoboToolkit-PC-Service`(default 브랜치) clone → `PXREARobotSDK/build.sh` → .h/nlohmann/.so 복사.
- (B) 로봇 온보드: `setup_teleop.sh`에 **신규 4.5 스텝**(arch-aware, install_pico.sh:60-94 미러) 추가. aarch64는 `orin` 브랜치 build.sh, x86은 LFS materialize. 그 후 `pip install --no-build-isolation -e $XRT`.
- **vendor 고려**: XRT pybind 소스는 다른 repo(reference_code)에 있음. 온보드 clone 자립성(README "자립성" 노트)을 위해 이 repo로 vendoring 고려(YAGNI 판단은 plan에서).
- **런타임**: XRoboToolkit PC-Service 데몬 로컬 실행 + PICO 페어링. `$XRT/examples/example.py`(컨트롤러), `example_body_tracking.py`(body 24관절)로 sanity check. body는 트래커 2개 캘리브 필요.

---

## 7. 데이터 흐름 / rate
- xrt 폴링 메인루프(~POLL_DT 3ms) → `_push_target` → output thread **50Hz** → `vr_shm` write. 하트비트 `estop_shm`는 메인루프마다.
- g1_ctrl: `CtrlFSM::run_` **1kHz**에서 `g_estop_asserted_`, `State_Mimic` 50Hz에서 `g_poll_vr`.

## 8. 에러 처리
- xrt import/init 실패 → 명확한 SystemExit(다른 transport는 영향 없음, 지역 import).
- body 미가용 → 기존 fallback(mode1 safe) 그대로 + 하트비트 flag=1.
- 브릿지 死 → 하트비트 stale → C++ damping(fail-safe).
- estop_shm 손상/짧은읽기 → C++ 무시(assert 안 함) — 단 이는 파일존재+정상seq 전제라 위험시나리오 아님.

## 9. 테스트 계획 (sim 우선, 실로봇 전 필수)
1. **Mock xrt (com1, 하드웨어 없이)**: `XrtReceiver`를 가짜 xrt 모듈(합성 body/컨트롤러)로 구동 → frame dict shape·seq 증가·setdefault 백필·age_ms 검증. transport=local 배선이 zmq로 안 새는지.
2. **estop_shm round-trip**: Python write ↔ C++ `g_poll_estop` 비트 일치(12B, magic 0x6703). 하트비트 stale→assert, flag→assert, 파일없음→disarm 단위 확인.
3. **sim(lo) 통합**: `unitree_mujoco` + `g1_ctrl --network=lo` + (PICO 페어링된 머신에서) `--transport local` → mode2/3 팔 추종. PICO 없는 com1은 mock으로 대체.
4. **하드 E-stop 시나리오**: (a) 우측 A → Passive 전환·damping 확인, 우측 menu 1s → 해제 후 `f` 재기립. (b) 브릿지 kill → ~0.5s 후 damping. (c) 워치독(PC-Service 종료) → damping.
5. **실로봇**: 위 전부 sim에서 통과 후에만. 첫 실로봇은 `--grip-enable` + mode2(상체)부터.

## 10. 파일 변경 요약
| 파일 | 변경 | 유형 |
|---|---|---|
| `teleop/vr_teleop_bridge.py` | `XrtReceiver` 클래스 추가, `--transport local` 배선(elif), 하트비트 write, finally clear | 수정 |
| `teleop/estop_shm.py` | 신규(0x6703, `<iIi>`, write/clear) | 신규 |
| `src/State_Mimic.cpp` | `EstopCtrl` 구조체 정의(공유) | 수정(소) |
| `include/FSM/CtrlFSM.h` | `g_estop_asserted_()` 멤버 + `run_()` 강제 Passive, `#include <cstdio>` | 수정 |
| `teleop/setup_teleop.sh` | 4.5 스텝: xrt(arch-aware) 빌드/설치 | 수정 |
| `teleop/README.md` | transport=local 사용법·셋업 | 수정(문서) |

## 11. 범위 밖 (YAGNI)
- 네트워크 경로(zmq/udp) 삭제 안 함(공존).
- `pico_control_bridge.py`(variant A, 컨트롤러 전용)와 통합 안 함(별개 유지).
- 손(hand) 트래킹·모션 트래커 미포함.
- E-stop 하드 데핑을 별도 torque 경로로 구현 안 함(Passive 재사용으로 충분).

## 12. 결정된 사항 (재확인용)
- 구조: 기존 브릿지에 `transport=local` 추가(공존). ✔ 사용자 승인
- 안전: E-stop·워치독 → 하드 damping(강제 Passive). ✔ 사용자 승인
- E-stop 래치 권한 = 브릿지 SafetyMonitor, C++는 flag 미러 + 하트비트 fail-safe(영구 C++ 래치 없음). ← 검증 기반 설계 refinement
</content>
