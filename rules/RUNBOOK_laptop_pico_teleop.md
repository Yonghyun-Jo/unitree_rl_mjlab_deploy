# RUNBOOK — 윈도우 노트북 PICO 텔레옵 세션 (sim2sim / 실로봇 공통)

> **범위**: PICO 를 **윈도우 노트북**에 물리고, 브릿지·`g1_ctrl` 은 **제어PC(com1 또는 온보드)** 에서 돌리는 토폴로지.
> PICO 를 제어PC 에 직결하는 온보드 방식은 [RUNBOOK_real_robot_mode1.md](RUNBOOK_real_robot_mode1.md) §1.5 / `--transport local`.
> 실로봇 안전 절차(갠트리·킬스위치·E-stop)는 이 문서 범위 밖 — 반드시 위 런북 §0·§4 를 같이 볼 것.

```
[윈도우 노트북]                              [제어PC = com1 / 온보드]
 PICO4U ──USB(고정)──▶ PC-Service             vr_teleop_bridge.py (GMR retarget)
                            │                        │  /dev/shm/g1_vr_ref
                     pico_publisher.py ──ZMQ:5556──▶  g1_ctrl
                     (PUB connect)              (SUB bind)
```

- **노트북 = PUB connect / 제어PC = SUB bind.** 노트북이 이동해도 제어PC 는 노트북 IP 를 몰라도 된다.
- 브릿지와 `g1_ctrl` 은 `/dev/shm` 공유 때문에 **반드시 같은 호스트**.
- 🔴 **PICO↔노트북 구간은 USB 로 고정한다** (2026-08-11 결정). WiFi 는 쓰지 않는다 —
  2.4/5GHz 경합·로밍이 그대로 텔레옵 지터가 되고, 브릿지 워치독(입력 30Hz)이 그걸 mode1 강제복귀로 갚는다.
  **노트북↔제어PC 구간은 별개**(LAN/Tailscale). USB 고정은 앞 구간에만 해당.

---

## ⚡ 매 세션 첫 명령 — 자동화 스크립트

```powershell
cd C:\dev\pico-capture\windows
.\pico_session.ps1                       # USB 경로 수립 (PC-Service + 포트 + adb reverse)
.\pico_session.ps1 -CheckOnly            # 안 될 때 원인만 보기
.\pico_session.ps1 -Tune                 # 관리자 PowerShell. 최초 1회 + 재부팅 후
```

원본: [deploy/robots/g1/teleop/windows/pico_session.ps1](../deploy/robots/g1/teleop/windows/pico_session.ps1)
(`.bat` 래퍼도 같이 있음 — 더블클릭용, ExecutionPolicy 우회)

**왜 매 세션 필요한가** — 아래 셋이 전부 세션마다 사라진다. 하나라도 빠지면 앱에는 `TCP failed` 로만 보인다:

| 휘발하는 것 | 왜 |
|---|---|
| PC-Service | 윈도우 *서비스*가 아니라 콘솔 앱 → 재부팅하면 안 뜸 |
| 리스닝 포트 | **동적 할당** → 기동할 때마다 번호가 바뀜 |
| `adb reverse` 터널 | 재부팅 · USB 재연결 · `adb kill-server` 로 소멸 |

§1 은 이 스크립트가 하는 일을 손으로 하는 절차다. 스크립트가 실패했을 때 읽는다.

### 전자동 — PICO 를 꽂기만 하면 되게

```powershell
# 최초 1회, 관리자 PowerShell
cd C:\dev\pico-capture\windows
.\pico_autostart.ps1 -Install
```

로그온 시 감시자가 뜬다. 3초마다 adb 를 폴링해서 **PICO 가 붙는 순간** `pico_session.ps1` 을 돌리고,
**터널이 소실되거나 PC-Service 가 죽으면 스스로 재수립**한다. 부팅당 첫 수립 때 `-Tune` 도 같이 건다
(작업이 '가장 높은 수준의 권한'으로 등록되므로 UAC 프롬프트 없이 적용된다).

```powershell
.\pico_autostart.ps1 -Status      # 등록 상태 + 최근 로그 25줄
.\pico_autostart.ps1 -Uninstall   # 해제 (관리자)
```

로그: `C:\dev\pico-capture\windows\autostart.log` — `READY` 줄이 나오면 앱에서 connect 하면 된다.

**자동화되지 않는 것**(전부 헤드셋 안 UI 라 PC 에서 손댈 수단이 없다):
앱에서 `127.0.0.1` connect · Full Body · 트래커 · **T-pose 캘리브** · **Send ON**.
감시자가 보장하는 것은 "connect 를 누르면 반드시 붙는 상태"까지다.

### 다음 세션 요약

| | 자동 등록 안 함 | 자동 등록함 |
|---|---|---|
| 1 | PICO USB 연결 | PICO USB 연결 |
| 2 | `pico_session.bat` | *(감시자가 알아서 — `-Status` 로 READY 확인)* |
| 3 | 앱: `127.0.0.1` connect → Full Body → T-pose → **Send ON** | 동일 |
| 4 | `python scripts\test_pico_pose.py` → `body=True` 확인 | 동일 |
| 5 | `python pico_publisher.py --com1 <제어PC>` | 동일 |
| 6 | 제어PC: sim / `g1_ctrl` / 브릿지 (§1-7) | 동일 |

### 실로봇(G1 온보드)으로 갈 때 — 노트북 쪽은 그대로

USB 고정의 이득이 여기서 나온다. **PICO↔노트북 구간은 물리적으로 동일**하므로 노트북 설정은 손댈 게 없다.

| | |
|---|---|
| `pico_autostart.ps1` 등록 | **재등록 불필요** |
| PC-Service / 포트 / `adb reverse` / 앱 `127.0.0.1` / `-Tune` | 그대로 |
| 등록 시 넣은 `-Com1` 값 | **바꿀 필요 없음** — 감시자는 publisher 를 띄우지 않으므로 그 값은 안내 문구용이다 |

바뀌는 것은 뒤 구간뿐:

```powershell
python C:\dev\pico-capture\pico_publisher.py --com1 <G1 온보드 IP>
```
```bash
# 온보드
./g1_ctrl --network=<로봇 iface>          # sim 의 --network=lo 대신
python .../vr_teleop_bridge.py --mode 1 --grip-enable --arm-estop --log /tmp/teleop_....csv
```
🔴 **실로봇은 `--arm-estop` 필수** (네트워크 transport 는 자동 무장 안 됨 — RUNBOOK_real_robot_mode1 §2-3b).

**실로봇에서 새로 생기는 문제 둘 — 자동화가 못 덮는다. 미리 정할 것:**

1. **노트북↔로봇 경로.** 실험장에 인터넷이 없으면 Tailscale 이 안 된다. 선택지 =
   온보드 이더넷 직결(가장 안정, 사람이 따라다녀야 함) / 휴대용 라우터로 로컬 WiFi(인터넷 불필요) / 로봇 자체 AP.
   → 앞 구간이 USB 로 고정돼 있으므로, **실로봇에서 지터가 늘면 원인은 이 구간 하나로 좁혀진다.**
2. **Orin 에서의 GMR 비용.** com1 x86 에서 1.63ms(워치독 여유 20배)인데 Orin 추정 5.5ms 는 **실측 앵커가 없다.**
   온보드 첫 브릿지 실행의 `--log` CSV `gmr_ms` p95 로 바로 확정된다.

---

## 0. 노트북 자산 위치 (2026-08-10 확인)

| | 경로 |
|---|---|
| 캡처/발행 repo | `C:\dev\pico-capture\` |
| **세션 자동화** | **`C:\dev\pico-capture\windows\pico_session.ps1`** (+`.bat`) — 이 repo 의 `deploy/robots/g1/teleop/windows/` 에서 복사 |
| conda env | **`pico`** |
| PC-Service | `C:\dev\pico-capture\pcservice\RoboticsServiceProcess.exe` (최초 1회 `install.bat` → VC_redist) |
| publisher (ZMQ) | `C:\dev\pico-capture\pico_publisher.py` ← **마지막 known-good** |
| publisher (UDP) | `C:\dev\pico-capture\udp\pico_publisher_udp.py` (원격 WAN 용, 미검증) |
| 캡처 검증 | `C:\dev\pico-capture\scripts\test_pico_pose.py` |
| 수신 참조구현 | `C:\dev\pico-capture\tools\com1_subscriber.py` |
| 노트북 자체 매뉴얼 | `C:\dev\pico-capture\README_SETUP.md` |

**PICO 에 설치된 XRoboToolkit 은 ROS1 / ROS2 / plain 3종.** → **plain 만 쓴다.**
ROS 판은 pose 를 ROS 토픽으로 직접 발행하는 용도라 PC-Service 경로를 안 탄다. 나머지 둘은 종료할 것(채널 경합).

**제어PC 주소**: 같은 LAN `192.168.50.211` / Tailscale `100.121.81.113`.

---

## 1. 세션 순서 (이 순서를 지킬 것)

### 1-1. PC-Service 먼저 (⚠ 최우선)

```powershell
cd C:\dev\pico-capture\pcservice
.\RoboticsServiceProcess.exe        # 또는 run3D.exe
```

**창을 닫지 말 것.** 이게 서비스 본체다.

> 🔴 **이 단계 누락이 "TCP failed" 의 1순위 원인이다.** 2026-08-10 에 여기서 한참 헤맸다 —
> PC-Service 가 안 뜬 상태에서는 **앱에 어떤 IP 를 넣어도**(노트북 IP·`127.0.0.1` 모두) 붙지 않는다.
> 받아줄 프로세스 자체가 없기 때문. IP 를 의심하기 전에 **프로세스부터 확인**한다(§3).

### 1-2. USB 터널 설치

```powershell
adb devices                                                       # device 로 보여야 함
Get-NetTCPConnection -State Listen -OwningProcess <PC-Service PID> # 오늘의 포트 확인
adb reverse --remove-all
adb reverse tcp:<PORT> tcp:<PORT>                                 # TCP 포트마다
adb reverse --list                                                # 확인
```

`adb reverse tcp:P tcp:P` = **헤드셋의 `127.0.0.1:P` → 노트북의 `127.0.0.1:P`**.
이게 없으면 앱의 `127.0.0.1` 은 헤드셋 자기 자신을 가리켜 **원리상 절대 안 붙는다**(`TCP failed`).

⚠ **포트는 동적 할당**이다(2026-08-10 관측: TCP `::`63901 / `127.0.0.1`60061, UDP `::`59950).
**PC-Service 를 재시작하면 번호가 바뀌므로 터널도 다시 건다.** 연결된 뒤에는 서비스 창을 닫지 말 것.

⚠ **`adb reverse` 는 TCP 전용**이다. PC-Service 가 여는 UDP 포트는 USB 로 안 넘어간다.
→ **connect 는 되는데 pose 가 계속 0.0** 이면 이 케이스를 의심한다(§3-4).

### 1-3. PICO 앱 연결

PICO 를 USB-C 로 **노트북 본체 포트에 직결**(허브·도킹 금지) → **plain XRoboToolkit** 실행
→ IP 칸에 **`127.0.0.1`** → connect → **"WORKING"** 확인.

앱에 USB/WiFi 토글은 **없다**(IP 칸 하나뿐). 경로는 그 칸에 넣는 값으로만 갈린다.
**우리는 USB 고정이므로 항상 `127.0.0.1`.** 노트북 WiFi IP 를 넣으면 그건 WiFi 경로다 — 쓰지 않는다.

### 1-4. 전신 트래킹 켜기 (mode3 필수)

헤드셋 착용한 채로 앱 안에서:

1. **Full Body tracking** 모드 (Head 만이면 `body_available=False`)
2. **Pico Swift 트래커 2개 이상** 연결 + 착용 (발 트래커)
3. **T-pose 캘리브레이션** ← **이거 없으면 `body_available=False`**
4. ⭐ **"Send" 토글 ON**

> **`streaming`(머리 pose) ≠ `body_available`(전신 24관절).** 텔레옵은 `body_available=True` 여야 활성.
> `streaming=True` 인데 `body=False` → 로봇 문제가 아니라 **앱 설정 문제**(Full Body/트래커/캘리브 중 하나).

⚠ **함정 두 개**
- **"Switch w/ A Button"** 옵션이 켜져 있으면 **오른쪽 A 버튼으로 Send 가 토글**된다. 컨트롤러 잡다 눌러 꺼지는 사고가 잦다.
- **헤드셋을 벗으면 절전으로 pose 가 0 으로 정지**한다. 세션 내내 착용 유지.

### 1-5. 캡처 검증 (⚠ publisher 전에)

```powershell
conda activate pico
python C:\dev\pico-capture\scripts\test_pico_pose.py
```

머리를 흔들며 확인 — **세 갈래로 원인이 갈린다**:

| 결과 | 원인 | 조치 |
|---|---|---|
| pos/quat 이 **계속 0.0** | PC-Service 미실행 / 페어링 끊김 | §1-1, §3 |
| 값은 변하는데 **body 만 False** | Full Body·트래커·캘리브 누락 | §1-4 |
| 값 변하고 **body True** | 정상 | §1-6 |

### 1-6. publisher 실행

```powershell
conda activate pico     # ⚠ 필수. 이걸 빼면 SDK 가 없어 "Python" 한 줄 찍고 즉시 죽는다
python C:\dev\pico-capture\udp\pico_publisher_udp.py --com1 100.121.81.113 --port 5556
```

⚠ **§1-5 의 `test_pico_pose.py` 를 먼저 Ctrl+C 로 끌 것.** PICO SDK 는 한 프로세스만 잡을 수 있어,
켜둔 채 publisher 를 띄우면 그대로 죽는다. 같은 이유로 **ZMQ publisher 와 동시 실행 금지.**

롤백(구 ZMQ, 브릿지도 `--transport zmq` 로):
```powershell
python C:\dev\pico-capture\pico_publisher.py --com1 100.121.81.113
```

정상이면 `[stat] seq=... 50.0Hz ... body=True`.
**`IDLE(Send OFF/헤드셋 벗음) body=False` 가 계속 뜨면 §1-4 로 돌아간다** — publisher 는 정상이고 SDK 에서 데이터가 안 나오는 것이다.

### 1-7. 제어PC 쪽

```bash
pkill -x g1_ctrl; pkill -x unitree_mujoco        # ⚠ orphan 정리 필수
pgrep -x g1_ctrl                                  # 비어야 정상

# 터미널 A (sim2sim 만)
cd /home/piene/unitree_rl_mjlab && ./simulate/build/unitree_mujoco
# 터미널 B — 뜨면 키보드 f(FixStand) → m(Mimic)
cd /home/piene/unitree_rl_mjlab && ./deploy/robots/g1/tools/run_g1_with_gui.sh
# 터미널 C — 브릿지 (진단 로깅 켜기). transport 생략 = udp(기본, §4)
~/miniconda3/envs/gmr/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py \
    --mode 1 --grip-enable --log /tmp/teleop_$(date +%m%d_%H%M%S).csv
```

시작 줄 **`[bridge] transport=UDP  bind *:5556/udp`** 를 확인할 것.
`--log` 경로에 **초(`%S`)까지** 넣는다 — 분 단위면 재실행 시 앞 세션 CSV 를 덮어쓴다.

**실로봇이면 `--arm-estop` 을 반드시 추가**(네트워크 transport 는 auto 로 무장 안 된다 — RUNBOOK_real_robot_mode1 §2-3b).

조작: **grip 쥐어야 활성**(놓으면 안전 mode1) / 좌 X→mode1 · 좌 Y→mode2 · 우 B→**mode3** / 좌스틱 이동 · 우스틱 회전.
**mode1 → 2 → 3 순으로 올릴 것.**

---

## 2. 원격(Tailscale)으로 할 때

```powershell
ping 100.121.81.113
tailscale status
```

- com1 줄에 **`direct`** 여야 한다. **`relay`(DERP 중계)면 지연이 몇 배로 뛰어 텔레옵 평가가 무의미**하다.
- 2026-07-10 원격 세션(30km) 실측: direct P2P, 평균 RTT 17ms, 그러나 **패킷 손실 10% + 지터 max 48ms** → 툭툭 끊김.
  현재의 One-Euro 스무딩·slew limit 이 **이 세션 때문에** 들어갔다.
- 2026-08-10 실측(다른 건물): direct, 손실 0%, RTT 6/17/30/**225**ms. **225ms 스파이크는 브릿지 `stale_ms=200` 을 넘긴다** → 잦으면 워치독.

⚠ **워치독 컷 = 입력 30Hz** (`teleop_safety.py` `min_rate_hz=30`). 밑으로 떨어지면 자동 mode1 복귀 = 화면상
**"팔이 갑자기 기본자세로 돌아감"**. 터미널 C 에 `[safety] SAFE:UNHEALTHY(rate=..Hz)` 로 찍히므로
**policy 고장과 구분된다.** 원격에서 이게 뜨면 원인은 네트워크다.

---

## 3. 트러블슈팅 — TCP failed / connect 안 됨

**IP 를 바꿔보기 전에 `.\pico_session.ps1 -CheckOnly` 를 먼저 돌린다.**
2026-08-10 에 IP 만 `192.168.0.97 → .79 → 127.0.0.1` 로 바꿔가며 시간을 태웠는데, 실제 원인은 **PC-Service 미실행**이었다.
**USB 고정이므로 앱에 넣을 값은 언제나 `127.0.0.1` 하나다 — IP 는 변수가 아니다.**

### 3-1. 진단 4종 (스크립트가 자동으로 하는 것)

```powershell
Get-Process | ? {$_.ProcessName -match "run3D|Robotic|XRobo"}  # PC-Service 떠 있나  ← 1순위
adb devices                                                    # USB 채널 살아있나
Get-NetTCPConnection -State Listen -OwningProcess <PID> | select LocalAddress,LocalPort
adb reverse --list                                             # 터널이 오늘 포트와 맞나
```

| 증상 | 원인 | 조치 |
|---|---|---|
| 1번째가 **비어 있음** | **PC-Service 미실행** — 어떤 값도 안 붙는다 | §1-1 |
| `adb devices` 비어 있음 | 개발자 모드 off / USB 디버깅 미허용 / **충전전용 케이블** / 허브 경유 | §3-2 |
| `adb devices` = `unauthorized` | 헤드셋 착용 후 **[항상 허용]** | — |
| `adb devices` = `offline` | adb 상태 꼬임 | `adb kill-server` → 케이블 재연결 |
| `reverse --list` 가 비었거나 **포트가 다름** | 서비스 재기동으로 포트가 바뀜 | 스크립트 재실행 |

### 3-2. USB 링크 자체가 안 잡힐 때 (위에서부터)

1. **케이블** — 충전 전용 아닌지. PICO 동봉 케이블 또는 USB 3.x 데이터 케이블.
2. **포트** — 노트북 **본체 직결**. USB 허브·도킹스테이션·모니터 경유 금지(대역폭·전원 불안정).
3. 헤드셋 설정 > 일반 > **개발자 모드 ON**, **USB 디버깅 ON**.
4. 헤드셋 USB 모드가 '충전만'으로 잡혔으면 알림에서 **파일 전송/MTP** 로 변경.
5. `adb kill-server` 후 재시도.

### 3-3. 터널이 세션 도중 사라짐

`adb reverse` 는 다음에 **전부 날아간다**: 케이블 재연결 · `adb kill-server` · **다른 adb 도구 실행**
(Android Studio, scrcpy, 다른 platform-tools 버전 → adb 서버가 재시작되며 터널 소멸).
→ 세션 중에는 adb 를 쓰는 다른 프로그램을 띄우지 않는다. 날아갔으면 스크립트 재실행.

### 3-4. connect 는 되는데 pose 가 계속 0.0

**`adb reverse` 는 TCP 전용**이다. PC-Service 가 UDP 포트도 열고 있고(2026-08-10: `::`59950)
pose 스트림이 그쪽을 타면 **USB 로는 못 넘어온다.** `pico_session.ps1` §2 에서 UDP 경고가 떴다면 이 케이스.

- 확인: `test_pico_pose.py` 에서 **connect 상태인데 값이 전혀 안 변함** (streaming 조차 안 옴)
- 구분: 값은 변하는데 `body=False` 면 이게 아니라 **앱 설정**(§1-4)
- 미해결 시 대안: PC-Service 의 UDP 포트도 헤드셋→노트북 방향으로 넘길 방법이 adb 에 없다.
  `adb forward`(반대 방향)로는 안 되고, 헤드셋에 소켓 릴레이를 올리거나 앱이 TCP 만 쓰도록 설정하는 수밖에 없다.
  **2026-08-11 현재 미검증 — 실제로 이 증상이 나오면 그때 파고든다.**

### 3-5. 기타

- **VirtualBox 호스트전용 어댑터(`192.168.56.1`)**: PC-Service 가 이 가상 어댑터를 자기 주소로 잘못 고를 수 있다. 증상 지속 시 비활성화 후 재시작.
- **`python` 이 "Python" 만 찍고 끝남**: Microsoft Store 스텁. `where python` 으로 `WindowsApps` 확인 → `conda activate pico`.

---

## 3-A. USB 링크 품질 고정 (최초 1회 + 재부팅 후)

```powershell
# 관리자 PowerShell
.\pico_session.ps1 -Tune
```

스크립트가 적용하는 것과 그 이유:

| 항목 | 왜 |
|---|---|
| **USB selective suspend OFF** | 윈도우가 유휴로 판단해 USB 를 재우면 프레임이 툭툭 끊긴다. 50Hz 스트림에서 가장 흔한 원인 |
| **USB 허브 절전 해제** | 장치관리자의 "전원 절약을 위해 이 장치를 끌 수 있음" 체크 해제와 동일 |
| **전원 계획 = 고성능** | 절전 계획은 USB 컨트롤러와 CPU 를 같이 조인다 |
| **PC-Service 우선순위 High** | 50Hz 스트림이 백그라운드 작업에 밀리지 않게 |

자동화할 수 없어 사람이 지켜야 하는 것:
- 케이블·포트를 **세션 중 건드리지 않는다**(재연결 = 터널 소멸).
- **PC-Service 창을 닫지 않는다**(포트가 바뀐다).
- **헤드셋을 벗지 않는다**(절전 진입 시 pose 가 0 으로 정지).
- 다른 adb 도구를 띄우지 않는다(§3-3).

---

## 4. transport — **UDP 로 결정 (2026-08-13 A/B 실측으로 닫음)**

브릿지 `--transport` **기본값 = `udp`**. 되돌리려면 `--transport zmq`(구 `pico_publisher.py` 와 짝).

### 실측 (연구실 노트북 ↔ 본가 com1, Tailscale 직결 P2P)

| 지표 | ZMQ (287s) | **UDP (168s)** |
|---|---|---|
| `in_age_ms` **최대** | **1173 ms** | **164 ms** |
| age > 200 ms (워치독 문턱 초과) | 152틱 = **3.0초** | **0틱** |
| `in_age_ms` p99 | 214 ms | 49 ms |
| rate < 30 Hz | 3.4 % | 0.7 % |
| 안전 트립 총 시간 | 10.3초 / 287초 (**3.6 %**) | 0.58초 / 168초 (**0.34 %**) |
| teleop 활성 비율 | 60.1 % | 81.9 % |

CSV: `/tmp/teleop_UDP_134022.csv`(UDP) · `/tmp/teleop_0811_sim.csv` t=5.9~292.6 구간(ZMQ).

### 왜 UDP 인가 — 대역폭이 아니라 **실패 모드**가 다르다

- **ZMQ(TCP)**: 재전송 stall → **2초 완전 침묵** → slow-start restart 로 6→50Hz 램프.
  이때 `age` 는 **도착 시각** 기준이라 2초 묵은 포즈가 `age=0` 으로 들어온다.
  **워치독이 못 잡는 유일한 실패 모드**(= 팔이 과거 궤적을 고속 재생).
- **UDP**: 재전송이 없어 "도착 = 방금 보낸 것"이 참 → `age` 가 정직하다.
  최악이 **프레임 성김**(50→25Hz, `age` 는 0~18ms 유지)이라 팔이 얼지 않고 성기게 따라온다.
- 프레임 크기: ZMQ JSON **2388 B(MTU 1280에서 2세그먼트, 둘 다 와야 1프레임)** vs
  UDP 바이너리 **806 B(1세그먼트) + 3중 중복**. 대역폭은 119 vs 121 KB/s 로 **거의 동일** → 중복이 공짜.

### 남은 것

UDP 세션의 유일한 트립(0.58초, t≈118.5)은 **`age` 가 신선한 채 rate 만 절반**으로 떨어진 형태다.
3중 중복을 감안하면 네트워크 손실로 설명되지 않는다(패킷 79% 손실 필요) → **노트북 publisher 측 정체**로 추정.
publisher 창의 자체 Hz 표시를 그 순간에 보면 확정된다. **미확인.**

---

## 5. 세션 후 — 진단 CSV

브릿지 `--log` 가 tick 단위로 남긴다:
`t_s, tick_dt_ms, active, have_new, gap, cmd_mode, gmr_ms, in_rate_hz, in_age_ms, reason`

화면의 `[diag]` 줄(1초 요약)로도 즉시 확인 가능:
```
[diag] out 50.0Hz  tick p50=20.0 p95=20.0 max=20.0ms (target 20)  overrun=0  |  gmr p50=2.3 p95=2.8 max=3.2ms
```

- `overrun > 0` / `tick p95 ≫ 20` → 출력 50Hz 가 밀린 것 (계산 여유는 x86 기준 10배 이상이라 드물어야 정상)
- `in_rate_hz` 가 30 근처 → **워치독 절벽**. 네트워크 문제
- `gmr_ms` 스파이크 → 트래커 dropout 구간

---

## 참고
- [RUNBOOK_real_robot_mode1.md](RUNBOOK_real_robot_mode1.md) — 실로봇 절차 전반, §1.5 PICO(온보드 직결), §2-3b 노트북 토폴로지, §4 하드 E-stop
- [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) — 전체 데이터 흐름
- `C:\dev\pico-capture\README_SETUP.md` (노트북 로컬) — SDK API 요약, APK sideload
