> **2026-09-01 이관 메모** — 이 문서는 2026-07-14 에 로봇 온보드에만 있던 것을 repo 로 옮긴 것이다.
> **JetPack 6(Ubuntu 22.04, glibc ≥ 2.34) 로봇에서만 아래 절차가 유효하다.** center_g1(JetPack 5.1.1) 은
> 아래 ⛔ 대로 불가라서 현재 실기 토폴로지는 [RUNBOOK_laptop_pico_teleop.md](RUNBOOK_laptop_pico_teleop.md) 의
> 노트북 publisher → 온보드 bridge `--transport udp --arm-estop` 이다. 새 로봇의 판정은 `deploy/onboard/bootstrap.sh --check` 가 glibc 로 알려준다.
> 그리고 ⛔ 의 «`.deb` 를 설치하지 말 것» 은 사후 결론이다 — center_g1 에는 `/opt/apps/roboticsservice/` 가 **이미 설치돼 있다**(무해, 안 뜸). «되돌리기」절 참조.

# 온보드 PICO 텔레옵 런북 — 이 Jetson 한 대에서 (2026-07-14)

> ## ⛔ 2026-07-14 확정: 온보드 `--transport local`은 이 Jetson에서 불가능
> 벤더 xrt/PC-Service **v1.0.0 orin 릴리스는 glibc 2.34(Ubuntu 22.04 / JetPack 6)를 요구**한다.
> 이 Jetson은 **JetPack 5 (L4T R35.3.1, Ubuntu 20.04, glibc 2.31)**.
> - `.deb` 안 `libPXREARobotSDK.so`·`RoboticsServiceProcess` 둘 다 GLIBC_2.34 참조 → **로드 자체 불가.**
> - 소스 빌드도 막힘: 벤더 grpc `.a`가 `__libc_single_threaded`(glibc 2.32)·`std::__throw_bad_array_new_length`(gcc-11)
>   참조 → 20.04(gcc-9.4, gcc-11 없음)에서 링크 실패. PC-Service는 Qt6 바이너리라 소스 빌드 비현실적.
> - **`.deb`를 `dpkg -i` 하지 말 것 — 설치돼도 안 뜬다.**
>
> **되는 경로 = `--transport zmq`** (xrt 불필요, GMR만 필요 → 이 Jetson에 설치 완료):
>   PC-Service + xrt publisher는 **glibc-2.34 머신(Windows 노트북의 win 빌드, 또는 com1/22.04)** 에서 돌리고,
>   bridge(`--transport zmq`) + g1_ctrl은 이 Jetson. ⚠ 단 **하드 E-stop은 local 전용이라 zmq에선 꺼짐**
>   (soft mode1 fallback만) → 물리 E-stop + 게트리 행잉으로 커버.
> 하드 E-stop까지 원하면 = com1(glibc 2.34) 복귀 후 그 위에서 full bridge(local) + `vr_relay_*`로 /dev/shm 중계(structure B).
>
> 아래 원래 절차(1~6)는 **JetPack 6 Jetson이거나 소스 전체 재빌드가 가능할 때만** 유효. 현 머신엔 §Blocked 참고.

com1 없이 이 Jetson(Orin NX, aarch64, `unitree@ubuntu`)에서 PICO를 직접 읽어 로봇에 전달한다.
전부 이 한 대에서 돈다:

```
PICO ─▶ PC-Service(헤드리스 데몬) ─▶ xrt ─▶ GMR ─▶ /dev/shm/g1_vr_ref ─▶ g1_ctrl ─▶ 로봇
```

## 검증된 사실 (2026-07-14 조사)

- Jetson 기본 Python은 3.8, GMR은 `>=3.10` 요구 → **uv로 3.10 venv**(`.venv-teleop`) 생성해 회피.
- GMR 의존성에 **torch + CUDA13 wheel 포함**(smplx 경유, ~1.5GB). 무겁지만 문제 아님
  (xrobot→G1 리타게팅은 CPU로 돌고 torch는 import만 되면 됨).
- xrt(`xrobotoolkit_sdk`)는 `setup_teleop.sh` + `XRT_SRC`로 설치(venv python 사용, 검증된 경로).
  ⚠ Pybind의 `setup_orin.sh`를 직접 돌리지 말 것 — 비-conda 경로 23행 `pip install pybind11 -y`의
  `-y`가 pip에 없는 옵션이라 에러난다.
- **PC-Service는 Qt6 앱이지만 prebuilt 헤드리스 ARM64 .deb가 있다** →
  `XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb`. Qt 소스빌드 불필요.
  설치 위치 `/opt/apps/roboticsservice/`, 실행 `runService.sh`(→ `RoboticsServiceProcess &`).

## 절차

### 1) Python 3.10 venv (uv) — 1회
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy
uv venv --seed --python 3.10 .venv-teleop
.venv-teleop/bin/python --version        # Python 3.10.x
```

### 2) GMR + deps — 1회 (torch/CUDA 다운로드로 20~40분)
```bash
bash deploy/robots/g1/teleop/setup_teleop.sh
#   XRT_SRC 안 걸면 "SKIP xrt" 뜨고 GMR만 깔림. 끝에:
#   "[setup_teleop] OK: xrobot->unitree_g1 retargeter built" 이면 성공
```

### 3) xrt (xrobotoolkit_sdk) — 1회
```bash
cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git .xrt-pybind
# setup_teleop의 aarch64 xrt 경로 발동: PC-Service orin clone → PXREARobotSDK build.sh → pip install
XRT_SRC="$PWD/.xrt-pybind" bash deploy/robots/g1/teleop/setup_teleop.sh
.venv-teleop/bin/python -c "import xrobotoolkit_sdk; print('xrt OK')"
#   실패 시: build.sh 빌드 의존성(cmake/g++/그 외 라이브러리) 확인
```

### 4) PC-Service 헤드리스 데몬 — 1회 설치 + 매번 실행 (⚠ sudo는 사용자가)
```bash
cd ~/Downloads
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
sudo dpkg -i XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
sudo apt-get install -f            # Qt 런타임 의존성 있으면
/opt/apps/roboticsservice/runService.sh    # 헤드리스 데몬 기동 (RoboticsServiceProcess &)
```

### 5) PICO 페어링 + 검증
- PICO 헤드셋 앱에서 이 Jetson(`192.168.3.2`)에 연결. **Full Body 모드** + 트래커 2개 착용/캘리브(T-pose).
- 검증:
```bash
.venv-teleop/bin/python .xrt-pybind/examples/example_body_tracking.py
#   "Body tracking data is available!" + 24관절 흐르면 PICO→xrt 성공
#   안 뜨면: PICO 연결 / Full Body 활성 / 트래커 2개 캘리브 재확인 (여기까진 로봇 무관)
```

### 6) 로봇 구동 — ⚠ 게트리 행잉 + E-stop + 사용자 확인 후
```bash
# 터미널1: 컨트롤러 (전원→zero-torque→리모컨 L2+R2 후)
./deploy/robots/g1/build/g1_ctrl --network=eth0      # f(기립) → m(Mimic_Masked)
# 터미널2: 브릿지 (온보드 로컬, PICO 직접 읽기)
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport local --grip-enable --mode 1
```
PICO 컨트롤러: `X`/`Y`/`B` = mode1/2/3, grip 놓기 = mode1 안전복귀, 우측 A = 하드 E-stop.
**`--transport local`이라 하드 E-stop(`/dev/shm/g1_estop`) 무장됨.** mode2까지 먼저 확인, mode3은 장력 확보 후.

## 남은 위험 (돌려봐야 아는 것)
1. `PXREARobotSDK` orin `build.sh`가 이 Orin에서 빌드되는지 (3번)
2. 헤드리스 .deb의 Qt 런타임 의존성 (`apt-get install -f`로 대개 해결) (4번)
3. **PICO가 헤드리스 PC-Service에 붙어 Full Body 데이터를 주는지** — 최종 관문 (5번)
4. xrt 버전(Pybind는 orin HEAD 빌드) vs .deb(v1.0.0) 호환 — xrt.init() 실패 시 의심

## 되돌리기
`rm -rf ~/.local/bin/uv unitree_rl_mjlab_deploy/{.venv-teleop,.gmr,.xrt-pybind}` +
`sudo dpkg -r xrobotoolkit-pc-service-headless` (또는 설치된 패키지명). 시스템은 안 건드림.
