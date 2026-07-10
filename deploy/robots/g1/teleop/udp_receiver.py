"""
udp_receiver.py — com1 UDP 수신자 (bridge 의 ZMQ SUB 블록을 이걸로 교체).

핵심: 매 제어 사이클에 latest() 호출 → 소켓에 쌓인 데이터그램을 전부 비우고(drain)
      seq 로 dedup + 가장 최신 프레임만 반환. 손실/중복/순서역전을 여기서 흡수.
      반환 dict 는 기존 JSON 프레임과 동일 shape → bridge 하위 로직 수정 불필요.

staleness: age_ms() 로 마지막 수신 후 경과시간. N ms 초과면 안전 mode1 폴백 권장
           (예전 is_body_data_available latching 문제도 이걸로 해결).

bridge 통합 (기존 ZMQ SUB 부분 대체):
    from udp_receiver import UdpReceiver
    rx = UdpReceiver(port=5556)
    ...
    while running:
        f = rx.latest()                 # 최신 프레임(or None)
        if f is None:
            if rx.age_ms() > 200:       # 200ms 무수신 → 안전 폴백
                user_mode = 1           # safe
            continue                    # 새 프레임 없으면 이전 상태 유지
        # ---- 이하 기존 로직 그대로 (f["controllers"], f["body"] ...) ----
"""
import argparse
import socket
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import pico_wire as wire


class UdpReceiver:
    def __init__(self, port=5556, bind_addr="0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((bind_addr, port))
        self.sock.setblocking(False)
        self.last_seq = -1
        self.last_rx = None          # time.monotonic() of last valid packet
        self._port = port

    def latest(self):
        """소켓을 전부 drain → seq 가장 큰 새 프레임 반환. 새 게 없으면 None."""
        newest = None
        newest_seq = self.last_seq
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            f = wire.unpack_frame(data)
            if f is None:
                continue
            self.last_rx = time.monotonic()
            # seq wrap(u32) 안전: 부호 없는 최신 비교. 실사용 시간엔 wrap 안 옴.
            if f["seq"] > newest_seq or (self.last_seq - f["seq"]) > (1 << 31):
                newest_seq = f["seq"]
                newest = f
        if newest is not None:
            self.last_seq = newest_seq
        return newest

    def age_ms(self):
        """마지막 유효 패킷 이후 경과(ms). 아직 아무것도 못 받았으면 큰 값."""
        if self.last_rx is None:
            return float("inf")
        return (time.monotonic() - self.last_rx) * 1000.0

    def close(self):
        self.sock.close()


def _demo():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--duration", type=float, default=0.0)
    a = ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    rx = UdpReceiver(port=a.port)
    print(f"[udp-rx] bind *:{a.port}/udp  packet={wire.PACKET_SIZE}B  대기...")
    got = 0
    last = time.monotonic()
    recv_window = 0
    t_stop = time.monotonic() + a.duration if a.duration > 0 else None
    while True:
        f = rx.latest()
        if f is not None:
            got += 1
            recv_window += 1
        now = time.monotonic()
        if now - last >= 1.0:
            h = f["headset"] if f else [0] * 7
            b = f["body"]["available"] if f else False
            pv = f["body"]["joints"][0][:3] if (f and f["body"]["available"]) else None
            print(f"[udp-rx] {recv_window:3d}new/s  seq={rx.last_seq}  age={rx.age_ms():.0f}ms  "
                  f"stream={f['streaming'] if f else '-'}  body={b}  head=({h[0]:+.3f},{h[1]:+.3f},{h[2]:+.3f})"
                  + (f"  Pelvis=({pv[0]:+.3f},{pv[1]:+.3f},{pv[2]:+.3f})" if pv else ""))
            recv_window = 0
            last = now
        if t_stop and now >= t_stop:
            break
        time.sleep(0.002)
    print(f"[udp-rx] 총 {got} 최신프레임 처리.")
    rx.close()


if __name__ == "__main__":
    _demo()
