# G1 새 로봇 처음부터 세팅 런북 (개봉 → unitree_rl_mjlab 배포)

> **용도**: 새 연구실에 방치된(세팅 안 된) Unitree G1을 **개봉 상태에서** 시작해 하드웨어 → 네트워크/SDK →
> 온보드 Jetson 개발환경 → `unitree_rl_mjlab` 배포 → VR 텔레옵까지 올리는 전체 절차.
> 하드웨어가 이미 살아있는 뒤(zero-torque/damping 이후)의 배포만 필요하면 [RUNBOOK_real_robot_mode1.md](RUNBOOK_real_robot_mode1.md) 참조.
>
> ⚠ **이 문서는 종합 가이드다. 최종 권위는 (1) 당신 로봇 상자의 종이 매뉴얼, (2) 현재 support.unitree.com, (3) 로봇에 붙은 리모컨 커맨드 스티커.**
> 버튼 조합·IP·펌웨어는 기종/펌웨어 버전에 따라 다르니, 무게를 지탱하는 상태에서 버튼을 누르기 전 **반드시 실물에서 확인**한다.

---

## 🚨 시작 전 CRITICAL 체크 (이거 틀리면 배포 자체가 불가/위험)

> (연구실 유닛 = **G1-EDU + 온보드 Jetson** 전제. 소비자/AIR는 저수준 미지원이지만 연구실은 해당 없음.)

1. **펌웨어 버전 → 버튼 조합이 바뀐다.** 예: V1.0 = `L1+A`(damping)/`L1+Up`(ready)/`R2+X`(motion), V1.0.4 = `L2+B`/`L2+Up`/`R2+A`. **리모컨 스티커 + Unitree Explore 앱의 리모컨 매뉴얼**로 당신 로봇 조합을 확인하고 사용. (참고: **이 팀 로봇은 저수준 활성화에 L2+R2가 불필요했다** — clean power-on에서 LowCmd 직행. L2+R2는 새 로봇이 방해할 때의 fallback. Phase 2-5.)
2. **갠트리/호이스트 + 사람이 킬스위치 옆 대기.** G1엔 별도 물리 E-stop 버튼이 없다 — 비상정지 = damping 조합(= 제어된 soft collapse, 로봇이 스르륵 주저앉음). 첫 기립·저수준 제어는 **반드시 프레임에 매단 상태**에서. 클리어런스 ≥2m.

---

## Phase 0 — 준비물 & 안전

- **하드웨어**: G1 본체, 배터리 2팩, 공식 충전기, 리모컨, (있으면) Pico Swift 트래커·PICO 헤드셋.
- **안전 장비**: 갠트리/호이스트 프레임(어깨 서스펜션 버클용 로프), 바퀴 잠금, 사람 1명 킬스위치 담당.
- **개발 PC**(선택): 이더넷 케이블, sim2sim dry-run용.
- **계정**: Unitree Explore 앱은 **판매처가 발급하는 기업(business) 계정** 필요 → 미리 발급 요청.
- **환경**: 0~40°C, 평평·비미끄럼·건조 바닥. 방수/방진 아님.

---

## Phase 1 — 하드웨어 첫 세팅 (Unitree 공식)

> 출처: G1 User Manual (EN, V1.0), support.unitree.com, docs.quadruped.de. **버튼 조합은 펌웨어 의존 — 실물 확인.**

### 1-1. 개봉 & 사전 점검
- 평평한 곳에서 상자 열고 본체·리모컨·충전기 분리해 꺼냄, 본체는 눕힘.
- 점검: 정품 부품만, 이물질(물/기름/모래) 없음, 배터리·리모컨 완충, 보호 브래킷 정상, 바닥 캐스터 바퀴 **잠금**.

### 1-2. 배터리 충전 (완충 후 시작)
- 배송 중 방전 → 첫 사용 전 완충. **로봇에서 분리**한 상태로 충전.
- 순서: AC 입력측 먼저 연결 → 그 다음 배터리에 연결. 배터리 전원 OFF 상태에서 연결. 뜨거운 팩은 식힌 뒤.
- **공식 Unitree 충전기만.** LED 게이지(LED1~4 = 25%씩), 충전 중 1Hz 점멸, LED 완전 소등 = 완충 → 뽑기.

### 1-3. 배터리 장착 & 전원 ON
- 배터리 2팩을 **측면**에서 삽입, **전원 스위치가 로봇 뒤쪽**을 향하게. 안 들어가면 방향 재확인(**억지로 밀지 말 것** — 버클 손상). "click" 소리 + 버클 완전 체결 확인.
- 전원 ON (각 팩): 스위치 **짧게 1회 → 2초 이상 길게 누름**. (OFF도 동일 패턴.)
- 부팅 ~1분 → **zero-torque 상태**(관절 전부 힘 빠짐) 도달까지 대기. (매단 상태면 ~2분 뒤 발목이 리밋에 닿는 소리로 init 성공 신호. 2분 무음 = init 실패 → 재배치·재시도.)

### 1-4. 리모컨 바인딩
- 리모컨 전원: 짧게 → 2~3초 길게 ("beep").
- 첫 사용 = 앱에서 바인딩: **Unitree Explore → Settings → Remote Control Settings → 리모컨 코드 입력**. 연결되면 우측 'DL' LED 점등.

### 1-5. 첫 기립 (⚠ 갠트리에 매단 상태 권장)
- 상태 사다리: **zero-torque → damping → ready → operation/motion**.
- (V1.0 펌웨어 예) `L1+A`=damping(제어 해금) → 어깨 잡고 `L1+Up`=ready(기립) → 똑바로 서면 `R2+X`=motion → `START`로 stand↔walk 토글, 조이스틱으로 이동.
- (V1.0.4 예) `L2+B`/`L2+Up`/`R2+A`. **→ 당신 로봇 조합은 리모컨 스티커/앱에서 확인.**
- 매단 경우: `R2+X`(motion) **전에** 발이 바닥에 닿게 슬링 내리고, 안정된 뒤 후크 완전 해제.

### 1-6. 앱: 상태 모니터링 · 펌웨어 업데이트 · (하지 말 것) 캘리브
- Unitree Explore: 온도/알람 모니터, 모터·IMU 보조 캘리브, 펌웨어 OTA, 리모컨 바인딩.
- **펌웨어 업데이트**: 충전기 연결, 안정 자세, 배터리 ≥50%, **업데이트 중 절대 전원 끄지 말 것**. 앱 안내 따름.
- ⚠ **공장 출하 유닛의 관절 캘리브를 임의로 다시 하지 말 것** — Unitree 기술지원 지시 있을 때만.

### 1-7. 눕히기 / 종료 (⚠ 서 있는 채로 전원 끄지 말 것)
- (V1.0 예) 어깨 뒤 잡고 `L1+Left`로 앉히기 → `L1+A`(damping) → 배터리 스위치 길게 눌러 종료.
- 매단 경우: 다시 매달고 로프 텐션 → 정적 기립 → damping → 배터리 종료.
- **서 있는 채 전원 끄면 그대로 넘어짐.** 반드시 damping + 매단/앉힌 상태에서 종료.

---

## Phase 2 — 네트워크 & 저수준 제어 활성화 (⚠ 여기가 제일 위험)

> 출처: unitree_sdk2, unitree_sdk2_python, Weston Robot G1 dev guide, xr_teleoperate wiki, unitree-g1-replay FSM_AND_SAFETY.
> 목표: **당신의 커스텀 컨트롤러(g1_ctrl)가 모터에 직접 명령**할 수 있게 함. 잘못하면 **두 컨트롤러가 모터를 두고 싸워 손상**.

### 2-1. 내부 네트워크 (192.168.123.0/24)
G1 내부에 L2 이더넷 스위치. 표준 IP (기종/펌웨어 따라 다름 — **ping으로 확인**):

| 호스트 | IP | 로그인 |
|---|---|---|
| Locomotion 컴퓨터(저수준 뇌, 접근 불가) | `192.168.123.161` | — |
| **개발 컴퓨터 = Jetson Orin NX** | `192.168.123.164` | `unitree` / `123` |
| Livox Mid-360 라이다 | `192.168.123.20` | — |

> 지금 당신이 쓰던 `unitree@ubuntu`(Ubuntu20.04/py3.8/aarch64)가 바로 이 **Jetson(.164)** 이다.

> **핵심**: **Jetson(.164)에서 g1_ctrl을 돌리면(당신 방식) 네트워크는 이미 공장 설정**이라 수동 IP 지정 불필요 — Jetson이 이미 로봇 내부망(.164)에 붙어 있다. `ip addr`로 iface 이름만 확인해 `--network=`에 넣으면 끝(2-3). 아래 static IP(.222 등) 설정은 **외부 노트북에서 원격으로 돌릴 때만** 필요.

### 2-2. (외부 PC에서 돌릴 때만) 개발 PC 연결
- 이더넷을 G1 스위치(포트 4/5)에 연결. PC NIC를 **static `192.168.123.x/24`** (예 `.99`/`.222`, 단 161/164/20 피함), netmask `255.255.255.0`.
- 확인: `ping 192.168.123.161`. (Jetson SSH 접속도 이 경로: `ssh unitree@192.168.123.164`, pw `123`.)
- **Jetson 온보드 실행이면 이 단계 스킵.**

### 2-3. iface 이름 찾기 (SDK `--network=<iface>`)
SDK엔 **IP가 아니라 인터페이스 이름**을 준다:
```bash
ip addr        # 192.168.123.x 를 가진 iface 찾기 (온보드 Jetson=보통 eth0, 외부PC=enpXsY)
```
- 그게 `g1_ctrl --network=<iface>`의 값. (sim은 `--network=lo`.)

### 2-4. DDS (CycloneDDS)
- unitree_sdk2 = CycloneDDS 기반. 프로그램마다 1회: C++ `ChannelFactory::Instance()->Init(0, iface)` / Py `ChannelFactoryInitialize(0, iface)`.
- **G1은 `unitree_hg` IDL** (topics `rt/lowcmd`/`rt/lowstate`) — Go2(`unitree_go`)와 다름. 틀리면 **조용히 무통신**.
- 확인: `cyclonedds ps` → `rt/lowstate`/`rt/lowcmd` 참가자 보이면 OK. iface당 DDS 소유자 1개만(ROS2 노드가 domain0 잡고 있으면 재-Init 충돌).

### 2-5. 저수준 제어 활성화 (당신 로봇은 L2+R2 없이 됐다 — 아래 검증)

**개념**: 고수준(high-level) = Unitree 내장 컨트롤러(리모컨/앱 "앞으로 가" → 관절을 알아서). 저수준(low-level) = **매 관절 q/kp/kd/tau를 당신이 직접 명령** — **당신 RL 정책(g1_ctrl)이 하는 게 이것.** g1_ctrl은 `motor_cmd.q()/kp()/kd()`를 직접 씀(State_Mimic.cpp) = 순수 저수준.

**검증된 사실 + 메커니즘 (조사 결론: 공장 기본이다, 멤버 세팅 아님)**:
1. `g1_ctrl`은 **고수준 서비스를 코드로 release하지 않는다** (`main.cpp` = DDS init → FSM Passive(damping) → LowCmd. MotionSwitcher/ReleaseMode 호출 없음). 이 팀은 **실로봇 배포를 여러 번 성공했고 L2+R2를 한 적이 없다.**
2. **왜 되나(메커니즘)**: 스톡 G1은 전원만 켜면 **zero-torque(모터 free)에 머물고, 고수준 sport/loco 서비스가 부팅 시 모터를 자동으로 붙잡지 않는다.** 리모컨 R2+X/앱으로 **명시적으로 모션을 시작해야** 붙잡음. 이 팀은 리모컨을 안 만지니 모터가 계속 free → LowCmd가 그대로 먹음. L2+R2/`ReleaseMode`는 **고수준이 *이미 모터를 잡고 있을 때만* 떼는 용도** → 잡은 적이 없으니 불필요.
3. **"이전 멤버가 영속 세팅한 것 아닌가?" → 거의 아니다**: (a) repo에 로봇을 설정하는 부팅스크립트/systemd/rc.local이 **전무**(build_deps.sh·setup_teleop.sh는 호스트측 빌드/venv일 뿐 로봇 안 건드림). (b) 실제 모터를 잡는 서비스는 **접근 불가한 `.161`(공장 제어, debug 상태는 재부팅 시 초기화)**에 살아 멤버가 영속 변경 불가. Jetson에 release 스크립트를 넣어도 *안 켜진 서비스*엔 no-op. → **당신은 거의 노세팅 베이스(clone만)에서 하고 있는 것이고, 이 동작은 공장 기본이다.**

**표준 절차(이 로봇, 검증된 방식)**: 전원 ON → zero-torque 대기 → **리모컨 motion 버튼 누르지 말 것** → 바로 g1_ctrl. **L2+R2 불필요.** g1_ctrl은 Passive(damping)에서 시작해 `f`(FixStand)로 서서히 기립.

- **fallback — 새 로봇이 방해할 때만** (`g1_ctrl` "Connected"인데 안 움직이거나 떨림 = 그 펌웨어의 내장 sport가 적극 구동): zero-torque에서 **`L2+R2`(debug mode, README §4.2)**, 또는 앱 **sport_mode off**, 또는 SDK **MotionSwitcher `ReleaseMode()`**(복귀 `SelectMode("ai")`). 펌웨어(버튼 조합)는 리모컨 스티커/앱으로 확인. **이 로봇은 이게 필요 없었음** — 새 로봇에서 증상이 나올 때만 쓴다.
- ⚠ debug/release를 쓰면 **그 순간 로봇이 limp** → 갠트리 필수. damping→`f` 순서.

**🔎 새 로봇 첫날 A/B 확인 (딱 하나 남은 변수 = 펌웨어).** 멤버 세팅 걱정은 위에서 배제됐지만, 새 로봇의 펌웨어가 **부팅 시 자동기립(auto-standup)**하는 버전이면 그 로봇에선 처음으로 L2+R2가 필요해진다. 전원 ON(리모컨 안 만짐) 직후, **g1_ctrl 켜기 전**에 한 번 확인:
- **눈으로**: 계속 limp면 → 당신 로봇과 동일(공장기본, 그대로 됨). 스스로 뻣뻣/기립하면 → 자동기립 펌웨어 → L2+R2 fallback 필요.
- **확실히(SDK CheckMode)**: `name=[]`(deactivated)면 A(공장기본), `name=[normal/ai/mcf]`(active)인데 LowCmd가 먹으면 어딘가 release되는 것 → 부팅 audit. Jetson에서(iface는 g1_ctrl과 동일, 예 `enp5s0`/`eth0`):
  ```cpp
  // /tmp/check_mode.cpp — MotionSwitcherClient::CheckMode
  #include <iostream>
  #include <unitree/robot/channel/channel_factory.hpp>
  #include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
  using namespace unitree::robot; using namespace unitree::robot::b2;
  int main(int argc,char**argv){ std::string ifc=(argc>1)?argv[1]:"eth0";
    ChannelFactory::Instance()->Init(0,ifc); MotionSwitcherClient m; m.SetTimeout(5.f); m.Init();
    std::string f,n; int r=m.CheckMode(f,n);
    std::cout<<"ret="<<r<<" form=["<<f<<"] name=["<<n<<"]\n";
    std::cout<<(r?"RPC FAIL(iface?)":(n.empty()?"DEACTIVATED: LowCmd free (A)":"ACTIVE: "+n+" (would need Release)"))<<"\n"; }
  ```
  ```bash
  cd ~/unitree_rl_mjlab && g++ -std=c++17 -O2 /tmp/check_mode.cpp -o /tmp/check_mode \
    -I.deps/include -I.deps/include/ddscxx -L.deps/lib .deps/lib/libunitree_sdk2.a -lddscxx -lddsc -lpthread -lrt -ldl \
    && /tmp/check_mode enp5s0    # ret!=0 이면 iface 틀림 → `ip -o link`로 확인 후 재시도
  ```
- Jetson 부팅 audit(멤버 잔재 최종 확인, read-only): `systemctl list-unit-files | grep -iE 'unitree|sport|loco|motion'` · `sudo cat /etc/rc.local 2>/dev/null` · `crontab -l 2>/dev/null` · `grep -rilE 'ReleaseMode|MotionSwitcher|SelectMode' ~ --include='*.sh' --include='*.py' | grep -v '/.deps/'` → 전부 비면 A 확정.

### 2-6. 공식 저수준 예제로 SDK↔모터 확인 (LowCmd 쓰기 전 필수)
```bash
# unitree_sdk2 clone/build (deps: libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev)
# example/g1/low_level/ 아래 예제 빌드 후:
./g1_low_level_example <iface>          # 예: eth0
```
- **먼저 rt/lowstate(관절 q/dq, IMU)가 실시간으로 들어오는지 확인** → DDS/IDL/iface가 end-to-end 정상이라는 증거. 그 다음에야 LowCmd(500Hz, q·dq·kp·kd·tau).

---

## Phase 3 — 온보드 Jetson 개발환경

> 출처: Weston G1 dev guide, NVIDIA Jetson docs, uv/astral. 당신의 배포 뇌(g1_ctrl)는 여기서 돈다.

### 3-1. Jetson 접속
```bash
ssh unitree@192.168.123.164        # pw: 123 (인터넷 노출 전 반드시 변경)
#   또는 Type-C→HDMI로 모니터+키보드 직결
```
- 하드웨어: **Jetson Orin NX 16GB**(EDU), **Unitree 커스텀 캐리어보드**.
- ⚠ **온보드 Orin에 서드파티 JetPack 이미지 절대 플래시 금지** — Unitree 커스텀 BSP라 벽돌됨. 위험 작업 전 이미지 백업.

### 3-2. OS/버전 확인
```bash
cat /etc/nv_tegra_release   # L4T R35.x → JetPack 5.1.x
lsb_release -a              # Ubuntu 20.04
python3 --version           # 3.8 (시스템 python — 건드리지 말 것)
```
- Ubuntu 20.04 + py3.8 = JetPack 5.1.x / CUDA 11.4. (JetPack6면 22.04/py3.10.)
- ⚠ **시스템 python3.8을 apt remove/심링크 변경 금지** — L4T·apt가 의존.

### 3-3. 빌드 툴체인
```bash
sudo apt update && sudo apt install -y build-essential cmake git
```
- C++는 Jetson에서 **네이티브 aarch64 컴파일**(크로스컴파일 X). 실시간 제어엔 C++ SDK 권장(python SDK는 Jetson에서 성능 이슈).

### 3-4. 새 Python(3.10) — 시스템 3.8 안 건드리고 (uv 권장)
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
uv venv --seed --python 3.10 .venv-teleop   # ★ --seed 필수(pip 포함), 3.10 자동 다운로드
```
- ⚠ **`pip install torch`는 Jetson에서 CPU-only**(JetPack CUDA와 안 맞음). GPU torch 필요하면 NVIDIA의 JetPack-매칭 **cp38 aarch64 wheel**(3.10엔 설치 안 됨). → GPU 추론은 3.8 or C++/ONNX-Runtime로, 3.10은 non-CUDA 앱코드용. (당신 g1_ctrl은 **C++ + 벤더드 ONNX-Runtime**이라 torch 불필요.)

---

## Phase 4 — unitree_rl_mjlab 배포 (당신 스택)

> 출처: build_deps.sh, teleop/README, RUNBOOK, CMakeLists. (Phase 1~2로 로봇이 살아있고 저수준이 열린 뒤.)

```bash
# (1) 시스템 deps + clone + 배포 branch
sudo apt install -y cmake build-essential libboost-all-dev libyaml-cpp-dev zlib1g-dev libfmt-dev libeigen3-dev
git clone <repo:unitree_rl_mjlab_deploy> unitree_rl_mjlab_deploy && cd unitree_rl_mjlab_deploy
git switch smooth_mode_switch        # 배포 대상 branch (ONNX 정책 이미 커밋됨)

# (2) .deps 채우기 (unitree_sdk2 + CycloneDDS, fresh clone당 1회) — aarch64/x86 자동
bash deploy/scripts/build_deps.sh

# (3) g1_ctrl 빌드 (ONNX 추론 + PD). onnxruntime는 arch 자동 선택(aarch64 벤더드)
cd deploy/robots/g1 && mkdir -p build && cd build && cmake .. && make -j4 && cd -
#   -> deploy/robots/g1/build/g1_ctrl

# (4, 선택) sim2sim 브리지 (실로봇 전 dry-run)
cd simulate && mkdir -p build && cd build && cmake .. && make -j8 && cd -   # ./simulate/build/unitree_mujoco (게임패드 필요)
```

**텔레옵 env** (Phase 3-4의 3.10 venv 위에):
```bash
# GMR 리타겟 (setup_teleop이 기존 .venv-teleop 3.10을 그대로 씀)
bash deploy/robots/g1/teleop/setup_teleop.sh
#   -> GMR clone + g1 mesh 복구 + numpy/scipy/pyzmq + 스모크. "OK: xrobot->unitree_g1 retargeter built"
#   ⚠ GMR이 smplx->torch(대용량 CUDA)를 끌어옴 — Jetson에선 CPU import만 되면 OK. 디스크 확인(df -h).

# --transport local(온보드) 쓰려면 xrt 추가 (aarch64=orin 빌드):
XRT_SRC=<.../XRoboToolkit-PC-Service-Pybind_X86_and_ARM64> bash deploy/robots/g1/teleop/setup_teleop.sh
#   XRT_SRC 없거나 실패하면 xrt SKIP(그럼 --transport local 불가, zmq/udp만)
```

---

## Phase 5 — VR 텔레옵 (PICO)

> 상세: [RUNBOOK_real_robot_mode1.md](RUNBOOK_real_robot_mode1.md) §1.5(PICO 연결)·§2(실행), [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md).

- **온보드(local)**: PICO를 **로봇 컴퓨터(Jetson)에 직결** → XRoboToolkit **PC-Service(aarch64) 실행** + 헤드셋 페어링(**com/USB**, WiFi 아님) → **Full Body + 트래커 + T-pose 캘리브 + Send ON** (← body_available 되게 하는 핵심; streaming≠body). → `vr_teleop_bridge.py --transport local`.
- **split(zmq)**: PICO를 별도 PC(노트북)에 연결, publisher가 네트워크로 로봇에 스트림 → 로봇은 `--transport zmq`. (단 repo에 publisher 미완성 — SYSTEM_OVERVIEW §4.)
- 확인: `.venv-teleop/bin/python $XRT_SRC/examples/example_body_tracking.py` → `is_body_data_available()==True`.

---

## Phase 6 — 안전-우선 Bring-up (실로봇은 마지막)

> 출처: RUNBOOK §0/§6/§8, SYSTEM_OVERVIEW.

1. **sim2sim 먼저**: `unitree_mujoco` + `g1_ctrl --network=lo` + 브리지로 정책·obs·조작 검증. 관절 안전 3층도 sim에서 하나씩 켜 검증(SYSTEM_OVERVIEW §9).
2. **실로봇**: 갠트리에 매달고, 사람 킬스위치 대기. **전원 ON → zero-torque → (리모컨 motion 버튼 안 누름) → 바로 g1_ctrl.** 이 로봇은 L2+R2 불필요(Phase 2-5). 새 로봇이 안 움직이거나 떨리면 그때만 L2+R2 fallback.
   ```bash
   # 터미널 A: 뇌 (실제 iface! lo 아님. ip addr로 192.168.123.x iface 확인)
   ./deploy/robots/g1/tools/run_g1_with_gui.sh <real_iface>   # "Connected to robot." → f(기립) → m(Mimic_Masked)
   # 터미널 B: 브리지
   .venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport local --grip-enable --mode 1
   ```
3. **mode1(자율보행)부터** 안정 확인 → mode2(상체) → mode3(전신). `--grip-enable` 데드맨 필수.
4. **세션 시작마다 우측 A(E-stop)가 실제로 damping 시키는지 1회 테스트.**
5. 종료: `p`(Passive) → 브리지 Ctrl-C(estop 파일 자동 제거) → g1_ctrl Ctrl-C → 로봇 종료(Phase 1-7).

---

## 로봇 도착하면 제일 먼저 확인할 것 (체크리스트)
```bash
# 온보드 Jetson에서:
uname -m; lsb_release -a; python3 --version; cat /etc/nv_tegra_release   # 기종/JetPack
ip addr; ping -c1 192.168.123.161                                        # 네트워크/저수준 뇌
```
- [ ] **G1-EDU + Jetson Orin 맞는지** (아니면 저수준 배포 불가)
- [ ] 펌웨어 버전 + 리모컨 버튼 조합 (앱/스티커)
- [ ] IP 실측 (161 locomotion, 164 Jetson) + iface 이름
- [ ] Unitree Explore 기업계정 발급됨
- [ ] 갠트리 + 킬스위치 담당자
- [ ] 저수준: 전원 ON → zero-torque → 바로 g1_ctrl 시도(이 로봇은 L2+R2 불필요). 안 먹으면 `L2+R2`(debug mode, README §4.2) fallback
- [ ] `g1_low_level_example`로 rt/lowstate 수신 확인 (LowCmd 쓰기 전)

## 주요 출처
- G1 User Manual (EN): cistemlabs.ai G1-USER-MANUAL-EN.pdf / support.unitree.com / docs.quadruped.de
- SDK/네트워크: github.com/unitreerobotics/unitree_sdk2 · unitree_sdk2_python · Weston Robot G1 dev guide · xr_teleoperate wiki(CycloneDDS)
- 저수준 안전: github.com/GetSoloTech/unitree-g1-replay FSM_AND_SAFETY.md · unitree_sdk2_python issue #43
- Jetson: NVIDIA JetPack/PyTorch-Jetson docs · astral.sh/uv
- 배포: 이 repo의 build_deps.sh · teleop/README · RUNBOOK_real_robot_mode1.md · SYSTEM_OVERVIEW.md
</content>
