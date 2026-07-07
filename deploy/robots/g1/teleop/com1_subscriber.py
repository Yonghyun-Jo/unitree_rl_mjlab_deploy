"""
com1_subscriber.py — com1(고정 제어 PC)에서 도는 참조 수신자. 노트북 로컬 자체 테스트에도 사용.

설계: 고정된 com1 이 bind, 이동하는 노트북 발행자가 connect 로 들어온다.
      따라서 여기(com1)는 tcp://*:PORT 로 bind 하고, 노트북 IP 는 몰라도 된다.

사용:
  # com1(리눅스)에서:
  python com1_subscriber.py --port 5556
  # 노트북 로컬 loopback 테스트: 이걸 먼저 띄우고, 발행자를 --com1 127.0.0.1 로 실행
"""
import argparse
import json
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import zmq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--topic", default="")  # "" = 전체 구독
    ap.add_argument("--duration", type=float, default=0.0, help="테스트용 실행 시간(초), 0=무한")
    a = ap.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 100)
    sub.setsockopt_string(zmq.SUBSCRIBE, a.topic)
    sub.setsockopt(zmq.RCVTIMEO, 2000)
    sub.bind(f"tcp://*:{a.port}")  # 고정 endpoint 로 bind
    print(f"[sub ] bind tcp://*:{a.port}  topic='{a.topic or '(all)'}'  대기 중... (Ctrl+C 종료)")

    n = 0
    last = time.monotonic()
    recv = 0
    t_stop = time.monotonic() + a.duration if a.duration > 0 else None
    try:
        while True:
            try:
                topic, payload = sub.recv_multipart()
            except zmq.Again:
                print("[sub ] ...수신 없음(발행자 미실행/네트워크 확인)")
                if t_stop and time.monotonic() >= t_stop:
                    break
                continue
            f = json.loads(payload.decode())
            n += 1
            recv += 1
            now = time.monotonic()
            if now - last >= 1.0:
                hz = recv / (now - last)
                h = f["headset"]
                print(f"[sub ] seq={f['seq']} {hz:5.1f}Hz  streaming={f['streaming']} "
                      f"head=({h[0]:+.3f},{h[1]:+.3f},{h[2]:+.3f})  body={f['body']['available']}")
                recv = 0
                last = now
            if t_stop and now >= t_stop:
                break
    except KeyboardInterrupt:
        print("\n[sub ] 종료 요청.")
    finally:
        sub.close(0)
        print(f"[done] 총 {n}개 수신.")


if __name__ == "__main__":
    main()
