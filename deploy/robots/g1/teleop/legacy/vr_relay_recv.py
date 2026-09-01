#!/usr/bin/env python3
"""vr_relay_recv.py — [로봇 온보드 Jetson에서 실행]

com1의 vr_teleop_bridge.py가 자기 로컬 /dev/shm/g1_vr_ref 에 쓰는 VrRef 프레임을
ZMQ로 받아, 이 머신의 /dev/shm/g1_vr_ref 에 그대로 재현한다.
g1_ctrl(State_Mimic.cpp g_poll_vr)이 그 파일을 읽는다.

  [com1]  bridge → /dev/shm/g1_vr_ref → vr_relay_send.py --to <jetson_ip>
                                              │ ZMQ PUB (connect)
  [jetson] vr_relay_recv.py (SUB bind :5557) ─┘ → /dev/shm/g1_vr_ref → g1_ctrl

왜 필요한가: /dev/shm 은 호스트를 못 넘는다. teleop/README 각주가 남겨둔
"실로봇에선 VrRef를 네트워크로 넘겨야 한다"가 정확히 이것.

안전장치: --stale-ms 동안 새 프레임이 없으면 valid=0 을 써서 C++이 VR override를
버리고 클립으로 되돌아가게 한다. 종료 시에도 valid=0 을 남긴다.
(vr_shm.py 계약: valid=0 -> C++ reverts to the clip)

사용 (Jetson):
    python3 -u vr_relay_recv.py                 # 0.0.0.0:5557 에 bind
    python3 -u vr_relay_recv.py --port 5557 --stale-ms 200

리포를 수정하지 않는다. vr_shm.py 의 레이아웃만 그대로 따른다. Python 3.8 호환.
"""
import argparse
import os
import struct
import sys
import time

import zmq

# vr_shm.py 계약 (deploy/robots/g1/teleop/vr_shm.py). 바꾸면 C++ struct VrRef 와 어긋난다.
SHM_PATH = "/dev/shm/g1_vr_ref"
MAGIC = 0x6702
FMT = "<iIii" + "f" * (3 + 4 + 29 + 29)   # 4 ints + 65 floats
FRAME_LEN = struct.calcsize(FMT)          # 276


def shm_write_raw(buf: bytes) -> None:
    """vr_shm.write 와 동일한 원자적 발행 (tmp 파일 → os.replace)."""
    tmp = SHM_PATH + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, SHM_PATH)


def make_invalid(cmd_mode: int = 1) -> bytes:
    """valid=0 프레임. C++ 은 이걸 보면 VR override 를 버리고 클립으로 되돌아간다."""
    return struct.pack(FMT, MAGIC, 0, 0, int(cmd_mode),
                       *([0.0] * 3), *[1.0, 0.0, 0.0, 0.0], *([0.0] * 29), *([0.0] * 29))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557, help="SUB bind 포트 (기본 5557)")
    ap.add_argument("--stale-ms", type=float, default=200.0,
                    help="이 시간 동안 프레임이 없으면 valid=0 (기본 200ms)")
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 4)          # 최신 프레임만 중요. 쌓이면 버린다.
    sub.setsockopt(zmq.CONFLATE, 1)        # 큐 깊이 1 → 항상 최신
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.bind("tcp://*:%d" % args.port)     # 고정 호스트가 bind (bridge 와 같은 관례)

    print("[relay-recv] SUB bind tcp://*:%d  → %s" % (args.port, SHM_PATH), flush=True)
    print("[relay-recv] stale > %.0fms 이면 valid=0 (클립 복귀). Ctrl+C 로 종료.\n" % args.stale_ms,
          flush=True)

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    last_rx = 0.0
    last_seq = -1
    n = 0
    stale_written = True   # 시작 시엔 아직 유효 프레임 없음

    try:
        while True:
            socks = dict(poller.poll(timeout=50))
            now = time.time()

            if sub in socks:
                buf = sub.recv()
                if len(buf) != FRAME_LEN:
                    print("[relay-recv] 길이 %d != %d — 버림" % (len(buf), FRAME_LEN), flush=True)
                    continue
                magic, seq, valid, mode = struct.unpack_from("<iIii", buf, 0)
                if magic != MAGIC:
                    print("[relay-recv] magic 0x%x != 0x%x — 버림" % (magic, MAGIC), flush=True)
                    continue
                if seq == last_seq:
                    continue                      # 중복
                last_seq = seq
                last_rx = now
                stale_written = False
                shm_write_raw(buf)
                n += 1
                if n <= 3 or n % 500 == 0:
                    print("[relay-recv] #%d  seq=%d valid=%d cmd_mode=%d" % (n, seq, valid, mode),
                          flush=True)

            # 워치독: 링크가 끊기면 VR override 를 놓아 준다.
            if not stale_written and last_rx and (now - last_rx) * 1000.0 > args.stale_ms:
                shm_write_raw(make_invalid())
                stale_written = True
                print("[relay-recv] STALE (%.0fms 무수신) → valid=0, 클립 복귀"
                      % ((now - last_rx) * 1000.0), flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        shm_write_raw(make_invalid())
        print("\n[relay-recv] 종료: valid=0 기록 (stale VR override 남기지 않음). 총 %d 프레임." % n)
        sys.exit(0)


if __name__ == "__main__":
    main()
