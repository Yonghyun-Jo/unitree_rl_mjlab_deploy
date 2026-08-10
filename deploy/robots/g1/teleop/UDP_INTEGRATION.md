# PICO UDP 스트림 — com1 통합 가이드 (Phase 1)

실시간 pose 스트림을 ZMQ/TCP → **UDP(바이너리+seq+중복)** 로 교체.
목적: WAN(원격) 손실 시 TCP head-of-line stall("툭툭 튐") 제거. 손실=스킵, 최신값만.

## 파일 (Taildrop 으로 com1에 전달됨)
- `pico_wire.py` — 패킷 바이너리 레이아웃(806B 고정). **노트북·com1 공유 단일 소스.**
- `udp_receiver.py` — `UdpReceiver` 클래스 (drain-to-latest + seq dedup + staleness).

## 노트북 쪽 (참고)
기존 `pico_publisher.py`(ZMQ/TCP) 대신 `udp/pico_publisher_udp.py` 실행:
```
python pico_publisher_udp.py --com1 100.121.81.113 --port 5556
```
- SDK 읽기 동일, 전송만 UDP. 매 프레임 2회 + 직전 프레임 1회(20ms 간격) 중복송신.
- ⚠ TCP 발행자와 **동시 실행 금지**(둘 다 SDK 물어서 충돌). UDP로 완전 전환.

## com1 bridge 수정 (ZMQ SUB 블록만 교체)

**기존 (ZMQ):**
```python
ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.RCVHWM, 4)
sub.setsockopt_string(zmq.SUBSCRIBE, "")
sub.setsockopt(zmq.RCVTIMEO, 2000)
sub.bind(f"tcp://*:{args.port}")
...
# 루프 안
topic, payload = sub.recv_multipart()   # 또는 drain
f = json.loads(payload.decode())
```

**교체 (UDP):**
```python
from udp_receiver import UdpReceiver
rx = UdpReceiver(port=args.port)         # bind *:5556/udp
...
# 루프 안 (매 제어 사이클)
f = rx.latest()                          # 소켓 전부 비우고 seq 최신 1개 (or None)
if f is None:
    # 새 프레임 없음: 200ms 이상 끊기면 안전 폴백
    if rx.age_ms() > 200:
        user_mode = 1                    # safe (VR 끊김 → 자율/정지)
    # 이전 상태 유지하고 다음 사이클로 (continue) 또는 마지막 f 재사용
    continue
# ---- 이하 기존 로직 100% 그대로 ----
# f 는 예전 JSON 프레임과 동일 shape:
#   f["headset"], f["streaming"],
#   f["controllers"]["left"/"right"]["pose"/"trigger"/"grip"/"axis"/
#                                    "primary"/"secondary"/"menu"/"axis_click"],
#   f["body"]["available"], f["body"]["joints"]  (24×7 or None)
# GMR / base_vel / 버튼→mode 계산 전부 수정 불필요.
```

### 핵심 포인트
- `rx.latest()` 가 **drain-to-latest + dedup** 를 내장 → 백로그 안 쌓임(WAN에서 필수).
- `rx.age_ms()` **staleness 감지** → 예전 `is_body_data_available()` latching(항상 True) 문제 해결.
  VR/트래커 끊기면 age 커짐 → 안전 mode1 폴백.
- 프레임 shape 동일 → bridge 하위 로직(GMR·VrRef·정책) 그대로.

## com1에서 먼저 단독 테스트 (bridge 붙이기 전)
노트북에서 UDP 발행 중일 때, com1에서:
```
python udp_receiver.py --port 5556 --duration 15
```
→ `Xnew/s seq=... age=..ms stream=True body=True head=(...) Pelvis=(...)` 가 찍히면 수신 정상.
(앉기 하면 Pelvis 값이 변하는지도 여기서 확인 가능.)

## 포트/롤백
- UDP 5556 사용(TCP였던 5556과 포트번호 같아도 UDP/TCP는 독립).
- 롤백: 기존 ZMQ `pico_publisher.py` + bridge의 ZMQ SUB 버전을 지우지 말 것.

## 패킷 레이아웃 (참고, 상세는 pico_wire.py)
```
헤더16B: magic('PC') ver(1) flags(u8) seq(u32) t_ns(u64)
headset 28B: 7×f32
L/R ctrl 각 45B: pose7×f32, trigger,grip, axis2, buttons(u8 비트마스크)
body 672B: 24×7×f32 (available=0이면 0)
= 806B, little-endian, <MTU 단일 데이터그램
flags: bit0=body_available bit1=streaming
buttons: bit0=primary bit1=secondary bit2=menu bit3=axis_click
```

## Phase 2 (나중, 신뢰성 명령용 TCP)
E-stop/안전정지, 세션 start/stop, calibrate 같은 **일회성 치명 명령**만 별도 TCP(5558).
실 로봇 테스트 때는 **필수**(watchdog + 원격 비상정지). sim 단계선 생략 가능.
```
