# 온보드 로컬 VR 텔레옵 (network-free) + 하드 E-stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 로봇 컴퓨터에 PICO가 직접 연결된 온보드 배포에서, 네트워크 없이 xrt(PICO)→GMR→`/dev/shm/g1_vr_ref`로 도는 통합 로컬 텔레옵(`--transport local`)을 추가하고, E-stop/워치독 시 실로봇을 하드 damping(강제 Passive)한다.

**Architecture:** 기존 `vr_teleop_bridge.py`에 receiver만 교체(`XrtReceiver`)해 GMR·안전·스무딩·`vr_shm` 계약을 100% 재사용. 별도 shm 채널 `/dev/shm/g1_estop`(하트비트+flag)을 브릿지가 쓰고, 공유 `CtrlFSM::run_()`(1kHz)이 폴링해 asserted면 강제 Passive(순수 damping). E-stop 래치 권한은 브릿지 `SafetyMonitor` 한 곳, C++는 flag 미러 + 하트비트 fail-safe(브릿지 死→damping).

**Tech Stack:** Python 3.10(stdlib struct/os/time + GMR), C++17(g1_ctrl, CtrlFSM), `xrobotoolkit_sdk`(pybind/CMake 소스빌드), `/dev/shm` tmpfs 파일 IPC.

## Global Constraints

- **커밋 메시지에 AI 흔적 금지**: `Co-Authored-By` 등 어떤 AI 트레일러도 넣지 않는다(사용자 repo 규칙 §6).
- **한국어 주석/문서**, 기존 파일 스타일·주석 밀도 유지.
- **shm magic 예약**: 0x6701=gui, 0x6702=vr → **estop=0x6703**. 12바이트 `<iIi>`(magic, seq, flag).
- **frame dict seq는 매 프레임 증가 필수** (안 그러면 SafetyMonitor rate=0 → 영구 SAFE mode1).
- **body.joints는 raw xyzw**(Unity 프레임) 그대로 emit — 변환은 브릿지 `_msg_body_to_human`이 담당.
- **xrt API 정확 이름**: `xrt.init()`(인자없음), `xrt.close()`(deinit 아님), primary/secondary = `get_X/Y/A/B_button`.
- **네트워크 경로(zmq/udp) 공존** — 삭제·회귀 금지.
- **테스트 실행**: Python은 stdlib-only standalone assert 스크립트(`python3 <file>`, 종료코드로 판정). C++는 `g++ -std=c++17` standalone(기존 `tests/test_masked_loco_controller.cpp` 관례).
- 작업 브랜치: `wose_obs` (현재 브랜치, 편집 전 `git branch --show-current`로 확인).

**spec 대비 정밀 교정(계획 단계 확정, 이유 명시):**
1. E-stop 폴 로직을 self-contained 헤더 `deploy/include/FSM/EstopChannel.h`로 분리 → 독립 테스트 가능 + 공유 FSM 프레임워크에 G1 종속성 안 새게 path 파라미터화(default `/dev/shm/g1_estop`).
2. `ESTOP_STALE_MAX`: `CtrlFSM::run_()`는 1kHz라, 폴을 **20틱마다(=50Hz) 게이팅**하고 MAX=25로 두어야 VR과 동일한 ~0.5s dead-writer 감지가 된다(매틱 폴+MAX25면 25ms로 오판).
3. `XrtReceiver`는 별도 모듈 `teleop/xrt_receiver.py`로 분리하고 xrt를 생성자 주입(DI) → GMR/zmq 없이 mock으로 단위 테스트.

---

## File Structure

**신규**
- `deploy/robots/g1/teleop/estop_shm.py` — E-stop shm 계약(Python writer/clear). 단일 책임: 12B struct 원자적 write.
- `deploy/robots/g1/teleop/xrt_receiver.py` — `XrtReceiver`(xrt→frame dict, xrt DI). 단일 책임: PICO 로컬 읽기→네트워크 receiver와 동일 shape.
- `deploy/include/FSM/EstopChannel.h` — self-contained C++ 폴 로직(구조체+`fsm_estop_poll`). 단일 책임: estop 채널 판독+하트비트 stale.
- 테스트: `deploy/robots/g1/teleop/tests/test_estop_shm.py`, `teleop/tests/test_xrt_receiver.py`, `teleop/tests/_mock_xrt.py`(가짜 xrt), `deploy/robots/g1/tests/test_estop_channel.cpp`.

**수정**
- `deploy/robots/g1/teleop/vr_teleop_bridge.py` — `--transport local` 배선(elif), 하트비트 write, finally clear, zmq lazy-import.
- `deploy/include/FSM/CtrlFSM.h` — `EstopChannel.h` include, 멤버 추가, `run_()` 강제 Passive 훅.
- `deploy/robots/g1/teleop/setup_teleop.sh` — xrt(arch-aware) 빌드/설치 스텝.
- `deploy/robots/g1/teleop/README.md` — transport=local 사용법·셋업.

---

## Task 1: E-stop shm 계약 (`estop_shm.py`)

**Files:**
- Create: `deploy/robots/g1/teleop/estop_shm.py`
- Test: `deploy/robots/g1/teleop/tests/test_estop_shm.py`

**Interfaces:**
- Produces: `estop_shm.write(seq: int, flag: int) -> None` (원자적 12B publish), `estop_shm.clear() -> None` (파일 제거), 상수 `SHM_PATH="/dev/shm/g1_estop"`, `MAGIC=0x6703`, `FMT="<iIi"`.

- [ ] **Step 1: 실패 테스트 작성** — `deploy/robots/g1/teleop/tests/test_estop_shm.py`

```python
"""test_estop_shm.py — estop_shm 바이트 레이아웃/원자성 자체 검증 (stdlib only)."""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import estop_shm


def main():
    # 테스트 격리: 실제 /dev/shm/g1_estop 대신 임시 경로로 덮어씀
    estop_shm.SHM_PATH = "/dev/shm/g1_estop_test"
    estop_shm.clear()

    # 1) write -> 12바이트 정확, 필드 파싱 일치
    estop_shm.write(7, 1)
    with open(estop_shm.SHM_PATH, "rb") as f:
        buf = f.read()
    assert len(buf) == 12, f"expected 12B, got {len(buf)}"
    magic, seq, flag = struct.unpack("<iIi", buf)
    assert magic == 0x6703 and seq == 7 and flag == 1, (magic, seq, flag)

    # 2) FMT/상수 계약 고정
    assert estop_shm.FMT == "<iIi" and estop_shm.MAGIC == 0x6703
    assert struct.calcsize(estop_shm.FMT) == 12

    # 3) clear -> 파일 제거, 재호출 무해
    estop_shm.clear()
    assert not os.path.exists(estop_shm.SHM_PATH)
    estop_shm.clear()  # FileNotFoundError 안 나야 함

    print("[test_estop_shm] ALL PASS")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 deploy/robots/g1/teleop/tests/test_estop_shm.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'estop_shm'`

- [ ] **Step 3: 구현** — `deploy/robots/g1/teleop/estop_shm.py`

```python
"""E-stop 채널 계약 — /dev/shm/g1_estop.

Python 생산자(브릿지 SafetyMonitor)가 이 packed 12B struct를 하트비트로 write하면,
C++(EstopChannel.h `fsm_estop_poll`)가 읽어 flag!=0 또는 하트비트 stale 시 강제 Passive(damping).
⚠ 이 레이아웃을 EstopChannel.h 구조체와 반드시 동기화.

레이아웃(little-endian, packed):  magic(i) seq(I) flag(i) = 12 bytes
"""
from __future__ import annotations

import os
import struct

SHM_PATH = "/dev/shm/g1_estop"
MAGIC = 0x6703             # 0x6701 gui, 0x6702 vr, 0x6703 estop
FMT = "<iIi"              # magic seq flag = 12 bytes


def write(seq: int, flag: int) -> None:
    """원자적 publish. flag=0 -> run, !=0 -> 강제 Passive. seq는 매 호출 증가시켜 하트비트로 쓴다
    (정지 시 C++가 ~0.5s 후 dead-writer로 판단해 fail-safe damping)."""
    buf = struct.pack(FMT, MAGIC, int(seq), int(flag))
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)  # atomic publish (no torn reads)


def clear() -> None:
    """정상 종료 시 disarm: 파일 제거 -> C++ 파일없음=미무장(정상 FSM)."""
    try:
        os.remove(SHM_PATH)
    except FileNotFoundError:
        pass
```

- [ ] **Step 4: 통과 확인**

Run: `python3 deploy/robots/g1/teleop/tests/test_estop_shm.py`
Expected: PASS — `[test_estop_shm] ALL PASS`

- [ ] **Step 5: 커밋**

```bash
git add deploy/robots/g1/teleop/estop_shm.py deploy/robots/g1/teleop/tests/test_estop_shm.py
git commit -m "teleop: add estop_shm 채널 계약(0x6703, 12B) + 레이아웃 테스트"
```

---

## Task 2: XrtReceiver (`xrt_receiver.py`)

**Files:**
- Create: `deploy/robots/g1/teleop/xrt_receiver.py`
- Test: `deploy/robots/g1/teleop/tests/test_xrt_receiver.py`, `deploy/robots/g1/teleop/tests/_mock_xrt.py`

**Interfaces:**
- Consumes: 주입되는 xrt 모듈(아래 getter들). 
- Produces: `class XrtReceiver` — `__init__(self, xrt)`, `latest() -> dict`(항상 dict, None 아님), `age_ms() -> float`, `close() -> None`. frame dict shape는 `ZmqReceiver`/`pico_wire.unpack_frame`과 동일: `{"seq":int, "streaming":bool, "headset":[7], "controllers":{"left":C,"right":C}, "body":{"available":bool,"joints":[24][7]|None}}`, C=`{"pose":[7],"trigger":f,"grip":f,"axis":[2],"axis_click":bool,"menu":bool,"primary":bool,"secondary":bool}`.

- [ ] **Step 1: 가짜 xrt 모듈 작성** — `deploy/robots/g1/teleop/tests/_mock_xrt.py`

```python
"""_mock_xrt.py — XrtReceiver 단위 테스트용 가짜 xrobotoolkit_sdk (실 하드웨어 없이)."""


class MockXrt:
    def __init__(self):
        self._bts = 1000            # body timestamp(ns) — 테스트에서 수동 증가
        self.body_ok = True
        self.closed = False
        self.X = self.Y = self.A = self.B = False
        self.left_menu = self.right_menu = False

    # body
    def is_body_data_available(self):
        return self.body_ok

    def get_body_joints_pose(self):
        # 24관절 × 7 raw [x,y,z,qx,qy,qz,qw] (값은 관절 인덱스로 구분되게)
        return [[float(i)] * 7 for i in range(24)]

    def get_body_timestamp_ns(self):
        return self._bts

    # headset / controllers
    def get_headset_pose(self):
        return [0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0]

    def get_left_controller_pose(self):
        return [0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def get_right_controller_pose(self):
        return [-0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def get_left_trigger(self):
        return 0.0

    def get_right_trigger(self):
        return 0.0

    def get_left_grip(self):
        return 0.0

    def get_right_grip(self):
        return 0.0

    def get_left_axis(self):
        return [0.2, 0.5]

    def get_right_axis(self):
        return [0.3, 0.0]

    def get_left_axis_click(self):
        return False

    def get_right_axis_click(self):
        return False

    def get_left_menu_button(self):
        return self.left_menu

    def get_right_menu_button(self):
        return self.right_menu

    def get_X_button(self):
        return self.X

    def get_Y_button(self):
        return self.Y

    def get_A_button(self):
        return self.A

    def get_B_button(self):
        return self.B

    def close(self):
        self.closed = True
```

- [ ] **Step 2: 실패 테스트 작성** — `deploy/robots/g1/teleop/tests/test_xrt_receiver.py`

```python
"""test_xrt_receiver.py — XrtReceiver가 네트워크 receiver와 동일 shape/계약을 내는지 검증."""
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))   # teleop/
sys.path.insert(0, _HERE)                     # tests/ (_mock_xrt)
from _mock_xrt import MockXrt
from xrt_receiver import XrtReceiver


def main():
    m = MockXrt()
    rx = XrtReceiver(m)

    # 1) latest()는 항상 dict (None 아님), 필수 키 존재
    f = rx.latest()
    assert f is not None
    for k in ("seq", "streaming", "headset", "controllers", "body"):
        assert k in f, k
    # 2) seq는 호출마다 증가 (SafetyMonitor rate watchdog 요구)
    f2 = rx.latest()
    assert f2["seq"] == f["seq"] + 1, (f["seq"], f2["seq"])
    # 3) 하드-브래킷 키 전부 존재 (KeyError 방지): controllers.{left,right}.{primary,secondary,menu,axis_click,grip,axis}
    for side in ("left", "right"):
        c = f["controllers"][side]
        for k in ("primary", "secondary", "menu", "axis_click", "grip", "axis"):
            assert k in c, (side, k)
        assert isinstance(c["axis"], list) and len(c["axis"]) == 2
    # 4) 버튼 매핑: X->left.primary, Y->left.secondary, A->right.primary, B->right.secondary
    m.X = True; m.B = True
    f3 = rx.latest()
    assert f3["controllers"]["left"]["primary"] is True
    assert f3["controllers"]["right"]["secondary"] is True
    assert f3["controllers"]["left"]["secondary"] is False
    # 5) body raw passthrough(변환 없음): joint i의 값 == i
    assert f3["body"]["available"] is True
    assert f3["body"]["joints"][5] == [5.0] * 7
    # 6) body 미가용 -> streaming False, joints None
    m.body_ok = False
    f4 = rx.latest()
    assert f4["streaming"] is False and f4["body"]["available"] is False
    assert f4["body"]["joints"] is None
    # 7) age_ms: bts 정지하면 증가, bts 변하면 리셋
    m.body_ok = True
    rx.latest(); a0 = rx.age_ms()
    time.sleep(0.02)
    rx.latest()                      # bts 그대로 -> age 누적
    assert rx.age_ms() >= a0
    m._bts += 1
    rx.latest()
    assert rx.age_ms() < 5.0         # bts 변함 -> 방금 리셋
    # 8) close 위임
    rx.close()
    assert m.closed is True

    print("[test_xrt_receiver] ALL PASS")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 실패 확인**

Run: `python3 deploy/robots/g1/teleop/tests/test_xrt_receiver.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'xrt_receiver'`

- [ ] **Step 4: 구현** — `deploy/robots/g1/teleop/xrt_receiver.py`

```python
"""xrt_receiver.py — PICO(xrobotoolkit_sdk)를 로컬에서 직접 읽어, 네트워크 receiver
(ZmqReceiver/UdpReceiver)와 동일한 frame dict를 내는 receiver. 온보드(co-located) 전용.

vr_teleop_bridge.py의 --transport local에서 사용. GMR/zmq를 import하지 않으므로
(xrt를 생성자 주입) 단위 테스트가 가벼움. body는 raw xyzw 그대로 emit — Unity->RH +
xyzw->wxyz 변환은 브릿지의 _msg_body_to_human이 담당(네트워크 경로와 동일).

⚠ xrt.init()은 호출측(브릿지)에서 이미 수행하고 xrt 모듈을 주입한다.
"""
from __future__ import annotations

import time


class XrtReceiver:
    def __init__(self, xrt):
        self._xrt = xrt
        self._seq = 0
        self._last_bts = None       # 마지막으로 관측한 body timestamp(ns)
        self._last_change = None    # bts가 바뀐 마지막 monotonic 시각

    def latest(self) -> dict:
        xrt = self._xrt
        self._seq += 1
        body_ok = bool(xrt.is_body_data_available())
        joints = None
        if body_ok:
            joints = [list(j) for j in xrt.get_body_joints_pose()]   # 24×7 raw xyzw

        # staleness 추적: body timestamp가 진행하면 살아있음
        bts = xrt.get_body_timestamp_ns()
        now = time.monotonic()
        if bts != self._last_bts:
            self._last_bts = bts
            self._last_change = now

        def ctrl(pose, trig, grip, axis, aclick, menu, primary, secondary):
            return {
                "pose": list(pose),
                "trigger": float(trig),
                "grip": float(grip),
                "axis": [float(axis[0]), float(axis[1])],
                "axis_click": bool(aclick),
                "menu": bool(menu),
                "primary": bool(primary),
                "secondary": bool(secondary),
            }

        return {
            "seq": self._seq,
            "streaming": body_ok,
            "headset": list(xrt.get_headset_pose()),
            "controllers": {
                "left": ctrl(xrt.get_left_controller_pose(), xrt.get_left_trigger(),
                             xrt.get_left_grip(), xrt.get_left_axis(), xrt.get_left_axis_click(),
                             xrt.get_left_menu_button(), xrt.get_X_button(), xrt.get_Y_button()),
                "right": ctrl(xrt.get_right_controller_pose(), xrt.get_right_trigger(),
                              xrt.get_right_grip(), xrt.get_right_axis(), xrt.get_right_axis_click(),
                              xrt.get_right_menu_button(), xrt.get_A_button(), xrt.get_B_button()),
            },
            "body": {"available": body_ok, "joints": joints},
        }

    def age_ms(self) -> float:
        """마지막으로 body timestamp가 바뀐 뒤 경과(ms). SDK 스톨 시 증가 -> 워치독이 감지.
        아직 아무것도 못 읽었으면 inf."""
        if self._last_change is None:
            return float("inf")
        return (time.monotonic() - self._last_change) * 1000.0

    def close(self) -> None:
        try:
            self._xrt.close()
        except Exception:
            pass
```

- [ ] **Step 5: 통과 확인**

Run: `python3 deploy/robots/g1/teleop/tests/test_xrt_receiver.py`
Expected: PASS — `[test_xrt_receiver] ALL PASS`

- [ ] **Step 6: 커밋**

```bash
git add deploy/robots/g1/teleop/xrt_receiver.py deploy/robots/g1/teleop/tests/test_xrt_receiver.py deploy/robots/g1/teleop/tests/_mock_xrt.py
git commit -m "teleop: XrtReceiver(로컬 xrt->frame dict, xrt DI) + mock 단위테스트"
```

---

## Task 3: 브릿지 배선 (transport=local + 하트비트 + zmq lazy)

**Files:**
- Modify: `deploy/robots/g1/teleop/vr_teleop_bridge.py`

**Interfaces:**
- Consumes: `XrtReceiver`(Task 2), `estop_shm.write/clear`(Task 1), `xrobotoolkit_sdk`(런타임).
- Produces: `--transport local` 실행 경로 + `/dev/shm/g1_estop` 하트비트.

- [ ] **Step 1: zmq lazy-import** — `import zmq`(파일 상단, 현재 ~:31)를 try/except로 감싼다. `--transport local`이 pyzmq 미설치 머신에서도 돌게.

바꾸기: 상단의
```python
import zmq
```
→
```python
try:
    import zmq            # zmq transport 전용. 로컬/UDP만 쓰면 미설치여도 됨.
except ImportError:
    zmq = None
```
그리고 `ZmqReceiver.__init__` 첫 줄에 가드 추가:
```python
    def __init__(self, port):
        if zmq is None:
            raise SystemExit("pyzmq 미설치: --transport zmq 쓰려면 `pip install pyzmq`. (local/udp는 불필요)")
        ctx = zmq.Context.instance()
```

- [ ] **Step 2: `--transport` choices에 local 추가 + estop_shm import**

`ap.add_argument("--transport", choices=["udp", "zmq"], ...)` → `choices=["udp", "zmq", "local"]`, help에 `local(온보드 co-located: xrt 직접읽기, 네트워크 없음)` 추가.
파일 상단 import 블록(`import vr_shm` 옆)에 추가:
```python
import estop_shm            # /dev/shm/g1_estop 하트비트 (same teleop/ dir)
```

- [ ] **Step 3: receiver 선택에 elif local 삽입** — 현재 `if udp / else zmq`(~:234-242)를 아래로 교체:

```python
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
```
⚠ 명시적 `elif` 필수: 안 그러면 local이 `else`(zmq)로 샌다.

- [ ] **Step 4: 하트비트 writer** — 메인 루프에서 `dec = safety.update(f, rx.age_ms())` **직후**(현재 :365, `if dec["mode"] is not None:` 분기 **이전**)에 삽입:

```python
            estop_seq += 1
            estop_shm.write(estop_seq, 1 if dec["mode"] is not None else 0)  # 하트비트+flag
```
그리고 루프 진입 전 카운터 초기화(다른 `seq_out = 0` 근처, :253 부근)에 추가:
```python
    estop_seq = 0
```

- [ ] **Step 5: 종료 시 disarm** — `finally:` 블록(현재 :434-441) 안, `rx.close()` 직전에 추가:

```python
        estop_shm.clear()   # 정상 종료 -> disarm(파일 제거). 크래시면 하트비트 stale로 C++가 damping.
```

- [ ] **Step 6: 통합 스모크(mock xrt)로 검증** — GMR 있는 env(com1 `gmr`)에서 가짜 xrt를 PYTHONPATH로 주입해 3초 구동. 배너가 LOCAL이고 zmq 없이 뜨며 하트비트 파일이 생기는지.

Run:
```bash
cd /home/piene/unitree_rl_mjlab
cat > /tmp/xrtmock/xrobotoolkit_sdk.py <<'PY'  # mkdir -p /tmp/xrtmock 먼저
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from _mock_xrt import MockXrt
_m = MockXrt()
def init(): pass
def close(): _m.close()
def __getattr__(name): return getattr(_m, name)   # 나머지 getter 위임(PEP 562)
PY
cp deploy/robots/g1/teleop/tests/_mock_xrt.py /tmp/xrtmock/
( PYTHONPATH=/tmp/xrtmock timeout 3 ~/miniconda3/envs/gmr/bin/python \
    deploy/robots/g1/teleop/vr_teleop_bridge.py --transport local --no-smooth 2>&1 || true ) | tee /tmp/bridge_local.log
grep -q "transport=LOCAL" /tmp/bridge_local.log && echo "OK: local banner"
test -f /dev/shm/g1_estop && echo "OK: heartbeat file present (또는 종료 후 clear됨)"
```
Expected: 로그에 `transport=LOCAL` 출력, `Traceback`/`No module named 'zmq'` 없음, `OK: local banner`. (3초 후 timeout 종료 시 `estop_shm.clear()`로 파일이 지워질 수 있음 — 구동 중 존재하면 통과.)

- [ ] **Step 7: 커밋**

```bash
git add deploy/robots/g1/teleop/vr_teleop_bridge.py
git commit -m "teleop: --transport local(xrt 직접) 배선 + estop 하트비트 write + zmq lazy-import"
```

---

## Task 4: C++ E-stop 채널 폴 (`EstopChannel.h`)

**Files:**
- Create: `deploy/include/FSM/EstopChannel.h`
- Test: `deploy/robots/g1/tests/test_estop_channel.cpp`

**Interfaces:**
- Produces: `struct EstopState { uint32_t last_seq; int stale; }`(0으로 초기화), `bool fsm_estop_poll(EstopState& st, const char* path="/dev/shm/g1_estop")`, `constexpr int ESTOP_STALE_MAX=25`. 반환 true = E-stop asserted(호출측이 Passive로 전환해야 함).
- Consumes: `estop_shm.py`가 쓰는 12B `<iIi>` (Task 1).

- [ ] **Step 1: 실패 테스트 작성** — `deploy/robots/g1/tests/test_estop_channel.cpp`

```cpp
// test_estop_channel.cpp — fsm_estop_poll 자체 검증 (기존 test_masked_loco_controller 관례).
//   빌드/실행:
//   cd deploy/robots/g1/tests
//   g++ -std=c++17 -I../../../include -O2 test_estop_channel.cpp -o /tmp/test_estop_channel && /tmp/test_estop_channel
#include "FSM/EstopChannel.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>

static const char* P = "/dev/shm/g1_estop_test";

static void write_frame(uint32_t seq, int32_t flag) {
    struct { int32_t magic; uint32_t seq; int32_t flag; } e{0x6703, seq, flag};
    FILE* f = std::fopen(P, "wb");
    std::fwrite(&e, sizeof(e), 1, f);
    std::fclose(f);
}

int main() {
    std::remove(P);
    EstopState st{};
    int fail = 0;
    auto chk = [&](bool got, bool want, const char* name) {
        if (got != want) { std::printf("FAIL %s: got=%d want=%d\n", name, got, want); ++fail; }
    };

    // 1) 파일 없음 -> 미무장(false), stale 리셋
    chk(fsm_estop_poll(st, P), false, "absent");

    // 2) 신선 seq + flag=1 -> asserted
    write_frame(1, 1);
    chk(fsm_estop_poll(st, P), true, "flag=1");

    // 3) 신선 seq + flag=0 -> 해제
    write_frame(2, 0);
    chk(fsm_estop_poll(st, P), false, "flag=0 fresh");

    // 4) seq 정지(하트비트 frozen): MAX 이하 폴은 false, 초과하면 asserted(dead-writer)
    for (int i = 0; i < ESTOP_STALE_MAX; ++i)
        chk(fsm_estop_poll(st, P), false, "frozen<=MAX");   // seq=2 그대로
    chk(fsm_estop_poll(st, P), true, "frozen>MAX -> stale assert");

    // 5) 회복: 신선 seq + flag=0 -> 해제 + stale 리셋
    write_frame(3, 0);
    chk(fsm_estop_poll(st, P), false, "recover");

    // 6) 손상(magic 틀림) -> false
    { FILE* f = std::fopen(P, "wb"); int32_t bad[3] = {0x1111, 9, 1}; std::fwrite(bad, sizeof(bad), 1, f); std::fclose(f); }
    chk(fsm_estop_poll(st, P), false, "bad magic");

    std::remove(P);
    if (fail) { std::printf("[test_estop_channel] %d FAIL\n", fail); return 1; }
    std::printf("[test_estop_channel] ALL PASS\n");
    return 0;
}
```

- [ ] **Step 2: 실패 확인**

Run:
```bash
cd deploy/robots/g1/tests && g++ -std=c++17 -I../../../include -O2 test_estop_channel.cpp -o /tmp/test_estop_channel
```
Expected: FAIL — `fatal error: FSM/EstopChannel.h: No such file or directory`

- [ ] **Step 3: 구현** — `deploy/include/FSM/EstopChannel.h`

```cpp
#pragma once
// EstopChannel.h — /dev/shm/g1_estop 판독(하트비트+flag). CtrlFSM::run_()가 폴링해
// asserted면 강제 Passive(damping). self-contained(FSM 프레임워크 의존 없음) → 단위 테스트 가능.
// Python 생산자: deploy/robots/g1/teleop/estop_shm.py (동일 12B <iIi> 레이아웃).
//
// 시맨틱: flag!=0(브릿지 SafetyMonitor가 E-stop/워치독 래치) 또는 하트비트 stale(브릿지 死,
// fail-safe) 시 true. 파일 없음 = 미무장(브릿지 미실행) = false. 래치 권한은 브릿지 한 곳,
// 여기선 미러 + dead-writer fail-safe만.
#include <cstdio>
#include <cstdint>

struct EstopState {
    uint32_t last_seq = 0;
    int      stale = 0;      // seq 정지가 지속된 폴 횟수
};

// 폴 게이팅 전제: 호출측(CtrlFSM)이 ~50Hz로 호출(1kHz면 20틱마다). MAX=25 -> ~0.5s dead-writer.
static constexpr int ESTOP_STALE_MAX = 25;
static constexpr int32_t ESTOP_MAGIC = 0x6703;

inline bool fsm_estop_poll(EstopState& st, const char* path = "/dev/shm/g1_estop")
{
    FILE* f = std::fopen(path, "rb");
    if (!f) { st.stale = 0; return false; }            // 파일 없음 = 미무장
    struct { int32_t magic; uint32_t seq; int32_t flag; } e{};
    size_t n = std::fread(&e, sizeof(e), 1, f);
    std::fclose(f);
    if (n != 1 || e.magic != ESTOP_MAGIC) return false; // 짧은읽기/손상 무시
    if (e.seq == st.last_seq) {                         // 하트비트 frozen
        if (++st.stale > ESTOP_STALE_MAX) return true;  // writer 死 -> fail-safe assert
        return false;
    }
    st.last_seq = e.seq;
    st.stale = 0;
    return e.flag != 0;
}
```

- [ ] **Step 4: 통과 확인**

Run:
```bash
cd deploy/robots/g1/tests && g++ -std=c++17 -I../../../include -O2 test_estop_channel.cpp -o /tmp/test_estop_channel && /tmp/test_estop_channel
```
Expected: PASS — `[test_estop_channel] ALL PASS` (종료코드 0)

- [ ] **Step 5: 커밋**

```bash
git add deploy/include/FSM/EstopChannel.h deploy/robots/g1/tests/test_estop_channel.cpp
git commit -m "FSM: EstopChannel.h(estop 채널 폴+하트비트 fail-safe) + 자체 테스트"
```

---

## Task 5: CtrlFSM 강제 Passive 훅

**Files:**
- Modify: `deploy/include/FSM/CtrlFSM.h`

**Interfaces:**
- Consumes: `fsm_estop_poll`, `EstopState`, `ESTOP_STALE_MAX`(Task 4). 기존 `states`(public vector), `currentState`(private), `FSMStringMap`, `spdlog`.
- Produces: E-stop asserted 시 어떤 상태에서든 Passive(damping) 강제 + 전이검사 bypass. `/dev/shm/g1_estop` 소비자.

- [ ] **Step 1: include 추가** — `CtrlFSM.h` 상단 include 블록에:

```cpp
#include "FSM/EstopChannel.h"
```

- [ ] **Step 2: 멤버 추가** — `private:` 아래(예: `currentState` 근처, ~:115)에:

```cpp
    EstopState estop_st_{};       // /dev/shm/g1_estop 하트비트 추적
    bool       estop_asserted_ = false;
    int        estop_poll_tick_ = 0;   // 1kHz run_을 ~50Hz로 게이팅(20틱마다 폴)
```

- [ ] **Step 3: run_()에 훅 삽입** — 기존 `void run_()`(~:82) 본문 **맨 앞**, `currentState->pre_run();`(:84) **이전**에:

```cpp
    void run_()
    {
        // ── 하드 E-STOP: /dev/shm/g1_estop 폴(≈50Hz) → asserted면 강제 Passive(damping) ──
        if (++estop_poll_tick_ >= 20) {          // 1kHz/20 = 50Hz (ESTOP_STALE_MAX=25 → ~0.5s)
            estop_poll_tick_ = 0;
            estop_asserted_ = fsm_estop_poll(estop_st_);   // default path /dev/shm/g1_estop
        }
        if (estop_asserted_) {
            int passive_id = FSMStringMap.right.at("Passive");   // == 1 (config FSM)
            if (!currentState->isState(passive_id)) {
                for (auto& state : states) {                     // public vector, id 스캔(:101과 동일)
                    if (state->isState(passive_id)) {
                        spdlog::warn("FSM: E-STOP -> forcing Passive from {}",
                                     currentState->getStateString());
                        currentState->exit();
                        currentState = state;
                        currentState->enter();                   // Passive: kp=0, kd, dq=0, tau=0
                        break;
                    }
                }
            }
            currentState->pre_run();
            currentState->run();
            currentState->post_run();
            return;                                              // 전이검사 bypass(자동 재보행 차단)
        }

        // ── 정상 경로(기존 코드 그대로) ──
        currentState->pre_run();
        currentState->run();
        currentState->post_run();
        // ... 이하 기존 nextStateMode 전이 로직 유지 ...
```
(주의: 기존 `currentState->pre_run()/run()/post_run()`와 전이 루프는 그대로 두고, 위 블록만 앞에 추가.)

- [ ] **Step 4: 빌드 확인** — g1_ctrl가 컴파일되는지(회귀 없음).

Run:
```bash
cd /home/piene/unitree_rl_mjlab/deploy/robots/g1/build && cmake .. >/dev/null && make -j4 2>&1 | tail -5
```
Expected: `g1_ctrl` 링크 성공, 에러 없음. (`.deps` 미빌드면 먼저 `bash deploy/scripts/build_deps.sh` — README A 참고.)

- [ ] **Step 5: 통합 라운드트립 검증(sim, 하드웨어 불필요)** — Python으로 estop 채널을 직접 흔들어 FSM이 Passive로 가는지. 별도 터미널:

```bash
# 터미널 A: unitree_mujoco 실행 후 g1_ctrl (sim). 키보드로 v(Velocity) 또는 m(Mimic_Masked) 진입.
./deploy/robots/g1/build/g1_ctrl --network=lo
# 터미널 B: E-stop 트립 -> Passive 강제 확인 (하트비트 3초)
python3 - <<'PY'
import sys, time
sys.path.insert(0, "deploy/robots/g1/teleop")
import estop_shm
seq = 0
for _ in range(150):            # ~3s @50Hz
    seq += 1
    estop_shm.write(seq, 1)     # flag=1 -> assert
    time.sleep(0.02)
estop_shm.clear()
print("done: g1_ctrl 로그에 'E-STOP -> forcing Passive' 떠야 함")
PY
```
Expected: g1_ctrl 로그에 `FSM: E-STOP -> forcing Passive from Velocity`(또는 현재 상태) 출력, 로봇이 damping(Passive)로 전환. `clear()` 후 정상 FSM 복귀(조작자 `f`로 재기립 가능). 브릿지 kill 시나리오는 `write(seq,1)` 루프를 중간에 Ctrl+C로 끊으면 ~0.5s 뒤 stale로 Passive 유지되는지 확인.

- [ ] **Step 6: 커밋**

```bash
git add deploy/include/FSM/CtrlFSM.h
git commit -m "FSM: run_()에 하드 E-stop 훅(강제 Passive+전이 bypass, 50Hz 게이팅)"
```

---

## Task 6: setup_teleop.sh — xrt 빌드/설치 스텝

**Files:**
- Modify: `deploy/robots/g1/teleop/setup_teleop.sh`

**Interfaces:**
- Produces: `.venv-teleop`(또는 지정 env)에 `import xrobotoolkit_sdk` 가능. arch-aware(x86 LFS materialize / aarch64 orin build).

- [ ] **Step 1: 4.5 스텝 삽입** — 현재 step 4(`requirements.txt` 설치, :56)와 step 5(smoke test, :58) **사이**에 추가:

```bash
# 4.5) xrobotoolkit_sdk (pybind/CMake 소스 빌드) — ONE 프로세스에 GMR + xrt 공존시킴.
#      네트워크 없는 온보드(--transport local)에 필요. XRT_SRC 미존재/실패 시 SKIP(브릿지는 udp/zmq로 동작).
XRT_SRC="${XRT_SRC:-/home/piene/reference_code/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64}"
if [ -d "$XRT_SRC" ]; then
  echo "[setup_teleop] installing xrobotoolkit_sdk from $XRT_SRC ..."
  ARCH="$(uname -m)"
  command -v cc >/dev/null || echo "[setup_teleop] WARN: build-essential(C++17) 필요"
  "$PY" -m pip install -q cmake pybind11 setuptools
  export CMAKE_PREFIX_PATH="$("$PY" -m pybind11 --cmakedir)"
  if [ "$ARCH" = "aarch64" ]; then
    NATIVE="$XRT_SRC/lib/aarch64/libPXREARobotSDK.so"
    if ! file "$NATIVE" 2>/dev/null | grep -q ELF; then
      echo "[setup_teleop] building PXREARobotSDK (aarch64, orin branch) ..."
      T="$XRT_SRC/tmp"; mkdir -p "$T"
      [ -d "$T/XRoboToolkit-PC-Service" ] || git clone -b orin https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git "$T/XRoboToolkit-PC-Service"
      ( cd "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK" && bash build.sh )
      mkdir -p "$XRT_SRC/lib/aarch64" "$XRT_SRC/include/aarch64"
      cp "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h" "$XRT_SRC/include/aarch64/"
      cp -r "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann" "$XRT_SRC/include/aarch64/nlohmann/"
      cp "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so" "$XRT_SRC/lib/aarch64/"
      rm -rf "$T"
    fi
  else   # x86_64
    NATIVE="$XRT_SRC/lib/libPXREARobotSDK.so"
    if ! file "$NATIVE" 2>/dev/null | grep -q ELF; then
      echo "[setup_teleop] ERROR: $NATIVE 가 git-LFS 스텁. 실행: git lfs install && (해당 repo에서) git lfs pull;"
      echo "                또는 소스 빌드(build.sh, default branch)."
      echo "[setup_teleop] xrt 스킵하고 계속(로컬 transport 불가, udp/zmq는 동작)."
    fi
  fi
  if file "$NATIVE" 2>/dev/null | grep -q ELF; then
    "$PY" -m pip install --no-build-isolation -e "$XRT_SRC/"
    "$PY" -c "import xrobotoolkit_sdk as xrt; print('[setup_teleop] OK: xrobotoolkit_sdk imports')"
  fi
else
  echo "[setup_teleop] SKIP xrt (XRT_SRC 없음); 브릿지는 split/UDP/ZMQ 모드만."
fi
```

- [ ] **Step 2: 검증(환경 의존, com1 gmr env로 스모크)** — 실제 설치는 무겁고 LFS/네트워크 의존이라, 계획 검증은 **conda gmr env**에 수동 설치 후 import 확인으로 대신.

Run (com1, 1회):
```bash
XRT=/home/piene/reference_code/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64
~/miniconda3/envs/gmr/bin/pip install -q cmake pybind11 setuptools
export CMAKE_PREFIX_PATH="$(~/miniconda3/envs/gmr/bin/python -m pybind11 --cmakedir)"
sudo apt-get install -y git-lfs && git lfs install
git -C /home/piene/reference_code/GR00T-WholeBodyControl lfs pull --include="**/lib/libPXREARobotSDK.so"
file "$XRT/lib/libPXREARobotSDK.so"     # "ELF 64-bit ... x86-64" 확인
~/miniconda3/envs/gmr/bin/pip install --no-build-isolation -e "$XRT/"
~/miniconda3/envs/gmr/bin/python -c "import general_motion_retargeting, xrobotoolkit_sdk as xrt; print('both import OK')"
```
Expected: `file`이 ELF 확인, 마지막 줄 `both import OK`. (LFS 불가 시 spec §6의 소스빌드 폴백.)

- [ ] **Step 3: 커밋**

```bash
git add deploy/robots/g1/teleop/setup_teleop.sh
git commit -m "teleop(setup): xrobotoolkit_sdk arch-aware 빌드/설치 스텝(GMR+xrt 공존)"
```

---

## Task 7: 문서 (README) 업데이트

**Files:**
- Modify: `deploy/robots/g1/teleop/README.md`

- [ ] **Step 1: transport=local + 온보드 + estop 사용법 추가** — README 끝(자립성 메모 앞)에 섹션 추가:

````markdown
## C. 온보드 로컬 텔레옵 (`--transport local`, network-free)

로봇 컴퓨터에 PICO를 직접 연결(co-located)한 경우. 네트워크 홉 없이 xrt→GMR→shm.

```bash
# 0) 1회: GMR + xrt 설치 (xrt는 PC-Service pybind 소스빌드; x86은 git-lfs, Jetson은 orin build.sh)
XRT_SRC=<.../XRoboToolkit-PC-Service-Pybind_X86_and_ARM64> bash deploy/robots/g1/teleop/setup_teleop.sh
# 1) XRoboToolkit PC-Service 데몬 실행 + PICO 헤드셋 페어링(body는 트래커 2개 캘리브)
# 2) 온보드에서 g1_ctrl (실로봇 iface) + 브릿지(로컬)
./deploy/robots/g1/tools/run_g1_with_gui.sh <robot_iface>
.venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport local --grip-enable --mode 1
```

### 하드 E-stop (`/dev/shm/g1_estop`)
- 우측 A → E-stop 래치(브릿지 SafetyMonitor). 우측 menu 1s 홀드 → 해제 후 `f`로 재기립.
- 워치독(SDK 스톨) / 브릿지 프로세스 死 → g1_ctrl가 하트비트 stale 감지(~0.5s) → 강제 Passive(damping).
- 브릿지가 매 사이클 `estop_shm.write(seq++, flag)` 하트비트. 정상 종료 시 파일 제거(disarm).

> sim2sim(노트북→com1)은 기존대로 `--transport zmq`(기본). local은 PICO가 g1_ctrl과 같은 PC일 때만.
````

- [ ] **Step 2: 검증** — 렌더 확인(코드펜스/표 깨짐 없나).

Run: `sed -n '/## C. 온보드/,$p' deploy/robots/g1/teleop/README.md | head -30`
Expected: 새 섹션이 온전히 출력.

- [ ] **Step 3: 커밋**

```bash
git add deploy/robots/g1/teleop/README.md
git commit -m "teleop(docs): --transport local 온보드 사용법 + 하드 E-stop 설명"
```

---

## Self-Review (작성자 체크 결과)

**1. Spec coverage** — spec 각 컴포넌트 → 태스크 매핑:
- C1 XrtReceiver → Task 2 ✓ · C2 transport=local → Task 3 ✓ · C3 estop_shm → Task 1 ✓ · C4 하트비트 writer → Task 3(Step 4) ✓ · C5 C++ g_estop + CtrlFSM → Task 4+5 ✓ · §6 셋업 → Task 6 ✓ · §5 E-stop 시맨틱 → Task 4(fail-safe)+5(force Passive)+3(하트비트/clear) ✓ · 테스트계획 → 각 Task 테스트 + Task5 Step5 통합 ✓ · README → Task 7 ✓.
- 갭 없음. spec §6 vendor(YAGNI)는 의도적으로 범위 밖(Task 6가 reference_code 경로 참조로 충분).

**2. Placeholder scan** — "TBD/TODO/적절히/에러처리 추가" 없음. 모든 코드 스텝에 실제 코드. ✓

**3. Type consistency** — `fsm_estop_poll(EstopState&, const char*)`·`ESTOP_STALE_MAX=25`·magic `0x6703`·`<iIi>` 12B가 Task1(py)/Task4(cpp)/Task5(호출)에서 일치. `XrtReceiver(xrt)`·`latest()/age_ms()/close()`가 Task2 정의 = Task3 사용 일치. frame dict 키가 Task2 = 브릿지 소비(검증됨) 일치. ✓

**정밀 교정 반영**: EstopChannel.h 분리(테스트+레이어링), 50Hz 게이팅(1kHz→20틱), XrtReceiver DI(경량 테스트) — 본문 Global Constraints에 근거 기재.
</content>
