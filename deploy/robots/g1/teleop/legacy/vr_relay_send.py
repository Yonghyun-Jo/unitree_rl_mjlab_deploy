#!/usr/bin/env python3
"""vr_relay_send.py — [com1 (GMR/bridge가 도는 머신)에서 실행]

vr_teleop_bridge.py 가 자기 로컬 /dev/shm/g1_vr_ref 에 쓴 276B VrRef 프레임을
읽어 ZMQ PUB 로 Jetson 에 보낸다. bridge 는 전혀 건드리지 않는다 (평소처럼 따로 실행).

  [com1]  bridge → /dev/shm/g1_vr_ref → 이 스크립트 (PUB connect)
                                            │
  [jetson] vr_relay_recv.py (SUB bind :5557)┘ → /dev/shm/g1_vr_ref → g1_ctrl

의존성: pyzmq 뿐 (numpy·GMR 불필요). bridge 와 같은 머신이면 어느 python 이든 된다.

사용 (com1, bridge 를 띄운 뒤 다른 터미널):
    python3 -u vr_relay_send.py --to 192.168.3.2
    python3 -u vr_relay_send.py --to 192.168.3.2 --port 5557 --hz 200

seq 가 바뀔 때만 전송한다 (bridge 가 os.replace 로 원자적 발행하므로 찢긴 읽기는 없다).
읽기 전용 — /dev/shm 에 쓰지 않고, 로봇에 아무 명령도 보내지 않는다. Python 3.8 호환.
"""
import argparse
import os
import struct
import sys
import time

import zmq

SHM_PATH = "/dev/shm/g1_vr_ref"
MAGIC = 0x6702
FMT = "<iIii" + "f" * (3 + 4 + 29 + 29)
FRAME_LEN = struct.calcsize(FMT)   # 276


def read_frame():
    """(buf, seq) 또는 (None, None). bridge 의 os.replace 덕분에 원자적."""
    try:
        with open(SHM_PATH, "rb") as f:
            buf = f.read()
    except (FileNotFoundError, OSError):
        return None, None
    if len(buf) != FRAME_LEN:
        return None, None
    magic, seq, _valid, _mode = struct.unpack_from("<iIii", buf, 0)
    if magic != MAGIC:
        return None, None
    return buf, seq


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", required=True, help="Jetson IP (예: 192.168.3.2)")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--hz", type=float, default=200.0, help="shm 폴링 주기 (기본 200Hz)")
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 4)          # 밀리면 오래된 프레임을 버린다
    pub.setsockopt(zmq.LINGER, 0)
    endpoint = "tcp://%s:%d" % (args.to, args.port)
    pub.connect(endpoint)                  # 고정 호스트(Jetson)가 bind, 여기가 connect

    # ZMQ PUB slow joiner: connect 직후에 보낸 프레임은 구독이 붙기 전이라 조용히 버려진다.
    # bridge 는 50Hz 로 계속 쓰므로 실전에선 무해하지만, 시작 직후 몇 프레임을 잃지 않도록 안정화.
    time.sleep(0.3)

    print("[relay-send] %s → PUB %s  (%.0fHz 폴링)" % (SHM_PATH, endpoint, args.hz), flush=True)
    print("[relay-send] bridge 를 먼저 띄우세요. Ctrl+C 로 종료.\n", flush=True)

    dt = 1.0 / args.hz
    last_seq = None
    n = 0
    warned = False

    try:
        while True:
            buf, seq = read_frame()
            if buf is None:
                if not warned:
                    print("[relay-send] %s 없음/무효 — bridge 가 안 돌고 있는 듯. 대기 중..."
                          % SHM_PATH, flush=True)
                    warned = True
                time.sleep(0.1)
                continue
            warned = False
            if seq != last_seq:
                last_seq = seq
                pub.send(buf)
                n += 1
                if n <= 3 or n % 500 == 0:
                    _m, _s, valid, mode = struct.unpack_from("<iIii", buf, 0)
                    print("[relay-send] #%d  seq=%d valid=%d cmd_mode=%d" % (n, seq, valid, mode),
                          flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        pub.close()
        print("\n[relay-send] 종료. 총 %d 프레임 전송." % n)
        print("[relay-send] 참고: 수신측 워치독이 곧 valid=0 을 써서 클립으로 복귀시킨다.")
        sys.exit(0)


if __name__ == "__main__":
    main()
