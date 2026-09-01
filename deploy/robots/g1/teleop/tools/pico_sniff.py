#!/usr/bin/env python3
"""pico_sniff.py — PICO(XRobot 앱)가 실제로 무엇을 어디로 쏘는지 관측한다.

왜 필요한가: 이 리포에는 pico_publisher.py 가 없어서 PICO 측 wire 계약을 읽을 수 없다.
추측 대신 실제 바이트를 본다.

UDP :PORT 와 TCP :PORT 를 동시에 열고, 들어온 것을 요약 출력한다.
pico_wire 의 806B 레이아웃(magic=b"PC")이면 그것도 알려준다.

사용 (Jetson, 로봇 내부망이 아니라 WiFi 192.168.3.2 로 들어온다):
    python3 pico_sniff.py                # 포트 5556
    python3 pico_sniff.py --port 5555

그 다음 PICO XRobot 앱에 host=192.168.3.2, port=5556 을 넣고 스트리밍 시작.
아무것도 안 뜨면 → 앱이 다른 포트/프로토콜을 쓰거나, PICO가 같은 WiFi(192.168.3.x)에 없다.

읽기 전용 진단 도구. 로봇에 아무 명령도 보내지 않는다. Ctrl+C 로 종료.
Python 3.8 호환.
"""
import argparse
import binascii
import socket
import struct
import sys
import threading
import time

PICO_MAGIC = b"PC"
PICO_FRAME_LEN = 806

_lock = threading.Lock()
_seen = {"udp": 0, "tcp": 0}
_t0 = time.time()


def describe(buf: bytes) -> str:
    """pico_wire 806B 프레임이면 해석해서, 아니면 앞부분만 덤프."""
    n = len(buf)
    if n >= 16 and buf[:2] == PICO_MAGIC:
        ver, flags, seq, t_ns = struct.unpack_from("<BBIQ", buf, 2)
        body = bool(flags & 0x1)
        streaming = bool(flags & 0x2)
        tag = "pico_wire OK" if n == PICO_FRAME_LEN else f"pico_wire magic이지만 길이 {n}!={PICO_FRAME_LEN}"
        return (f"{tag}  ver={ver} seq={seq} body_available={body} streaming={streaming}")
    head = binascii.hexlify(buf[:32]).decode()
    try:
        txt = buf[:60].decode("utf-8")
        printable = "".join(c if 32 <= ord(c) < 127 else "." for c in txt)
        return f"len={n}  hex={head}  ascii='{printable}'"
    except UnicodeDecodeError:
        return f"len={n}  hex={head}"


def report(kind: str, peer, buf: bytes) -> None:
    with _lock:
        _seen[kind] += 1
        k = _seen[kind]
    if k <= 3 or k % 100 == 0:
        print(f"[{time.time()-_t0:7.1f}s] {kind.upper():3s} #{k:<6d} from {peer[0]}:{peer[1]}  {describe(buf)}",
              flush=True)


def udp_loop(port: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    print(f"[sniff] UDP  listening on 0.0.0.0:{port}", flush=True)
    while True:
        buf, peer = s.recvfrom(65535)
        report("udp", peer, buf)


def tcp_loop(port: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.listen(4)
    print(f"[sniff] TCP  listening on 0.0.0.0:{port}", flush=True)
    while True:
        conn, peer = s.accept()
        print(f"[sniff] TCP  연결됨: {peer[0]}:{peer[1]}", flush=True)
        threading.Thread(target=_tcp_conn, args=(conn, peer), daemon=True).start()


def _tcp_conn(conn, peer) -> None:
    # ZMQ면 첫 바이트가 0xFF...0x7F (ZMTP greeting). 그것도 그대로 보인다.
    with conn:
        while True:
            buf = conn.recv(65535)
            if not buf:
                print(f"[sniff] TCP  끊김: {peer[0]}:{peer[1]}", flush=True)
                return
            report("tcp", peer, buf)


def local_ipv4():
    """(iface, ip) 목록. netifaces 없이 /proc + ioctl 대신 간단히 socket 으로."""
    out = []
    try:
        import subprocess
        raw = subprocess.check_output(["ip", "-4", "-o", "addr", "show"], text=True)
        for line in raw.splitlines():
            f = line.split()
            if len(f) >= 4 and f[2] == "inet":
                ip = f[3].split("/")[0]
                if not ip.startswith("127."):
                    out.append((f[1], ip))
    except Exception:
        pass
    return out or [("?", "이 호스트의 IP를 직접 확인하세요 (ip -4 addr)")]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5556,
                    help="vr_teleop_bridge.py 기본값과 동일 (기본 5556)")
    args = ap.parse_args()

    print("[sniff] PICO를 이 호스트의 주소 중 하나로 향하게 하세요 (PICO와 같은 망이어야 함):")
    for name, ip in local_ipv4():
        note = "  ← 로봇 내부망. PICO에서 안 닿음" if ip.startswith("192.168.123.") else ""
        print(f"           {ip}:{args.port}   ({name}){note}")
    print("[sniff] Ctrl+C 로 종료. 로봇에 아무 명령도 보내지 않습니다.\n")

    for fn in (udp_loop, tcp_loop):
        threading.Thread(target=fn, args=(args.port,), daemon=True).start()

    try:
        while True:
            time.sleep(5)
            with _lock:
                u, t = _seen["udp"], _seen["tcp"]
            if u == 0 and t == 0:
                print(f"[{time.time()-_t0:7.1f}s] 아직 수신 없음 — 앱 설정/포트/WiFi 확인", flush=True)
    except KeyboardInterrupt:
        with _lock:
            print(f"\n[sniff] 종료. UDP {_seen['udp']} 프레임, TCP {_seen['tcp']} 청크 수신.")
        sys.exit(0)


if __name__ == "__main__":
    main()
