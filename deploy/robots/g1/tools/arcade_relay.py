#!/usr/bin/env python3
"""G1 아케이드 중계 — 브라우저 의도 → GuiCtrl 0x6705(50 Hz 하트비트) · 로봇 상태(logger + 상태 칸) → 브라우저.

고정 자료 = g1_arcade docs/protocol_relay_v1.md (여기서 바꾸지 않는다 — 어긋나면 그 문서가 이긴다).
  - WebSocket(JSON 텍스트) 서버 하나 · logger WS 구독 하나(msgpack) · 50 Hz 쓰기/송신 루프 하나.
  - 처음 붙은 연결 = controller, 나머지 = viewer. controller 가 끊기거나 1 s 넘게 cmd 가 없으면 «새로 붙는» 연결이 승계.
  - cmd 가 0.3 s 끊기면 하트비트는 계속 쓰되 속도·1회성 칸은 0. 종료 시그널에서 0 프레임 한 번.
  - 느린 클라이언트는 최신 한 장만(이전 송신이 끝나지 않았거나 전송 버퍼가 차면 그 클라이언트의 이번 프레임은 건너뛴다).

런타임 = logger 의 venv python(py3.8, websockets 12/13, msgpack). python 3.8 문법만 쓴다.
로봇 실행:
  ~/dyros_ws/piene_ws/piene_g1_logger/.venv/bin/python arcade_relay.py --bind 192.168.3.2 --port 8710
"""
import argparse
import asyncio
import collections
import json
import logging
import math
import signal
import sys
import time
from typing import Optional

import msgpack
import websockets

import gui_shm
import status_shm

log = logging.getLogger("arcade_relay")

RELAY_NAME = "arcade_relay v1"
CMD_STALE_S = 0.3          # 브라우저 cmd 끊김 → 0 프레임
CONTROLLER_SILENCE_S = 1.0  # 이만큼 cmd 가 없으면 새 연결이 controller 승계
LOGGER_STALE_S = 0.5
PERIOD_S = 0.02
WRITE_BUF_LIMIT = 64 * 1024
INT32_MAX = 2 ** 31 - 1

# piene_g1_logger g1_logger/joints.py G1_JOINT_NAMES 의 복사본(SDK motor index 순서) — 시험이 원본과 대조한다.
G1_JOINT_NAMES = [
    "L_HipPitch", "L_HipRoll", "L_HipYaw", "L_Knee", "L_AnklePitch", "L_AnkleRoll",
    "R_HipPitch", "R_HipRoll", "R_HipYaw", "R_Knee", "R_AnklePitch", "R_AnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "L_ShoulderPitch", "L_ShoulderRoll", "L_ShoulderYaw",
    "L_Elbow", "L_WristRoll", "L_WristPitch", "L_WristYaw",
    "R_ShoulderPitch", "R_ShoulderRoll", "R_ShoulderYaw",
    "R_Elbow", "R_WristRoll", "R_WristPitch", "R_WristYaw",
]
NJ = len(G1_JOINT_NAMES)


class RateLimitedLog:
    def __init__(self, interval=1.0):
        self.interval = interval
        self._last = {}
        self._suppressed = {}

    def warn(self, key, msg):
        now = time.monotonic()
        last = self._last.get(key)
        if last is not None and now - last < self.interval:
            self._suppressed[key] = self._suppressed.get(key, 0) + 1
            return
        n = self._suppressed.pop(key, 0)
        self._last[key] = now
        log.warning("%s%s", msg, " (+%d 건 생략)" % n if n else "")


def _is_num(x):
    return type(x) in (int, float) and math.isfinite(x)


def _is_int(x, lo, hi):
    return type(x) is int and lo <= x <= hi


def validate_cmd(obj):
    """(cmd dict, None) 또는 (None, 사유). 속도는 gui_shm 상한으로 한 번 더 자른다."""
    for k in ("seq", "mode_req", "m5_preset", "m5_press_seq", "vx", "vy", "wz"):
        if k not in obj:
            return None, "missing " + k
    if not _is_int(obj["seq"], -2 ** 63, 2 ** 63):
        return None, "bad seq"
    if not _is_int(obj["mode_req"], 0, INT32_MAX):
        return None, "bad mode_req"
    if not _is_int(obj["m5_preset"], 0, INT32_MAX):
        return None, "bad m5_preset"
    if not _is_int(obj["m5_press_seq"], 0, 2 ** 64):
        return None, "bad m5_press_seq"
    for k in ("vx", "vy", "wz"):
        if not _is_num(obj[k]):
            return None, "bad " + k
    return {
        "seq": obj["seq"] & 0xFFFFFFFF, "mode_req": obj["mode_req"], "m5_preset": obj["m5_preset"],
        "m5_press_seq": obj["m5_press_seq"] & 0xFFFFFFFF,
        "vx": gui_shm.clamp(float(obj["vx"]), -gui_shm.VXCAP_BWD, gui_shm.VXCAP),
        "vy": gui_shm.clamp(float(obj["vy"]), -gui_shm.VYCAP, gui_shm.VYCAP),
        "wz": gui_shm.clamp(float(obj["wz"]), -gui_shm.WCAP, gui_shm.WCAP),
    }, None


class Intent:
    """controller 의 의도. 속도 = 최근 cmd, mode_req·m5_preset = 0 이 아니면 라치(다음 쓰기 한 번, 나중 값이 이김)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.vx = self.vy = self.wz = 0.0
        self.mode_latch = 0
        self.preset_latch = 0
        self.press_seq = 0
        self.t = None   # 마지막 cmd 받은 시각

    def apply(self, cmd, now):
        self.vx, self.vy, self.wz = cmd["vx"], cmd["vy"], cmd["wz"]
        if cmd["mode_req"] != 0:
            self.mode_latch = cmd["mode_req"]
        if cmd["m5_preset"] != 0:
            self.preset_latch = cmd["m5_preset"]
        self.press_seq = cmd["m5_press_seq"]
        self.t = now

    def fresh(self, now):
        return self.t is not None and now - self.t <= CMD_STALE_S

    def take(self, now):
        """이번 쓰기에 실을 칸. 낡았으면 속도·1회성 0(라치도 버린다)."""
        if not self.fresh(now):
            self.vx = self.vy = self.wz = 0.0
            self.mode_latch = self.preset_latch = 0
        f = {"vx": self.vx, "vy": self.vy, "wz": self.wz, "mode_req": self.mode_latch,
             "m5_preset": self.preset_latch, "m5_press_seq": self.press_seq}
        self.mode_latch = self.preset_latch = 0
        return f


class Client:
    def __init__(self, ws, cid, now):
        self.ws = ws
        self.cid = cid
        self.role = "viewer"
        self.last_cmd_t = now
        self.send_task = None
        self.tx = collections.deque()
        self.viewer_cmd_logged = False

    def name(self):
        return "#%d %s" % (self.cid, getattr(self.ws, "remote_address", None))


def _clean(v):
    if isinstance(v, float):
        return round(v, 5) if math.isfinite(v) else None
    return v


class Relay:
    def __init__(self, bind="127.0.0.1", port=8710, gui_shm=gui_shm.SHM_PATH, status_shm=status_shm.SHM_PATH,
                 logger_url="ws://127.0.0.1:8700"):
        self.bind, self.port = bind, port
        self.gui_path, self.status_path, self.logger_url = gui_shm, status_shm, logger_url
        self.clients = set()
        self.controller = None
        self.intent = Intent()
        self.rx = collections.deque()
        self.seq = 0
        self.t0 = time.monotonic()
        self.logger_data = None
        self.logger_t = None
        self.logger_ok = False
        self.was_fresh = False
        self.in_loss = False
        self._cid = 0
        self._rl = RateLimitedLog()
        self._server = None
        self._tasks = []
        self._stopped = False

    # ── 수명 ────────────────────────────────────────────────────────────
    async def start(self):
        _gui_mod.SHM_PATH = self.gui_path
        self._server = await websockets.serve(self._handler, self.bind, self.port, max_size=65536)
        self.port = self._server.sockets[0].getsockname()[1]
        log.info("listening ws://%s:%d  gui=%s status=%s logger=%s", self.bind, self.port, self.gui_path,
                 self.status_path, self.logger_url)
        self._tasks = [asyncio.ensure_future(self._tick_loop()), asyncio.ensure_future(self._logger_loop())]

    async def stop(self):
        if self._stopped:
            return
        self._stopped = True
        for t in self._tasks:
            t.cancel()
        for t in self._tasks:
            try:
                await t
            except BaseException:
                pass
        self._write_frame({"vx": 0.0, "vy": 0.0, "wz": 0.0, "mode_req": 0, "m5_preset": 0,
                           "m5_press_seq": self.intent.press_seq})
        log.info("종료: 0 프레임 기록")
        if self._server is not None:
            self._server.close()
            try:
                await asyncio.wait_for(self._server.wait_closed(), 2.0)
            except Exception:
                pass

    # ── 접속 ────────────────────────────────────────────────────────────
    async def _handler(self, ws):
        self._cid += 1
        now = time.monotonic()
        c = Client(ws, self._cid, now)
        self._assign_role(c, now)
        self.clients.add(c)
        log.info("client connect %s role=%s (총 %d)", c.name(), c.role, len(self.clients))
        try:
            await ws.send(self._hello(c.role))
            async for msg in ws:
                await self._on_message(c, msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(c)
            if c.send_task is not None:
                c.send_task.cancel()
            if self.controller is c:
                self.controller = None
                self.intent.reset()
                log.info("controller %s 끊김 — 속도 0, 조종권 비움", c.name())
            log.info("client disconnect %s (남은 %d)", c.name(), len(self.clients))

    def _assign_role(self, c, now):
        old = self.controller
        if old is None or now - old.last_cmd_t > CONTROLLER_SILENCE_S:
            if old is not None:
                old.role = "viewer"
                log.info("role change %s controller→viewer (cmd 침묵 %.1fs), 새 controller %s",
                         old.name(), now - old.last_cmd_t, c.name())
                asyncio.ensure_future(self._send_quiet(old, self._hello("viewer")))
            self.controller = c
            c.role = "controller"
            c.last_cmd_t = now
            self.intent.reset()

    def _hello(self, role):
        return json.dumps({"v": 1, "type": "hello", "role": role, "joint_names": G1_JOINT_NAMES,
                           "relay": RELAY_NAME})

    async def _send_quiet(self, c, text):
        try:
            await c.ws.send(text)
        except Exception:
            pass

    # ── 받기 ────────────────────────────────────────────────────────────
    async def _on_message(self, c, msg):
        if not isinstance(msg, str):
            self._rl.warn("bad", "bad frame (binary) from %s" % c.name())
            return
        try:
            obj = json.loads(msg)
        except ValueError:
            self._rl.warn("bad", "bad frame (not json) from %s" % c.name())
            return
        if not isinstance(obj, dict) or type(obj.get("v")) is not int or obj.get("v") != 1:
            self._rl.warn("bad", "bad frame (v != 1) from %s" % c.name())
            return
        typ = obj.get("type")
        if typ == "ping":
            if "id" in obj and "t" in obj and _is_num(obj["id"]) and _is_num(obj["t"]):
                await c.ws.send(json.dumps({"v": 1, "type": "pong", "id": obj["id"], "t": obj["t"]}))
            else:
                self._rl.warn("bad", "bad frame (ping) from %s" % c.name())
        elif typ == "cmd":
            if c is not self.controller:
                if not c.viewer_cmd_logged:
                    c.viewer_cmd_logged = True
                    log.info("viewer cmd 무시 %s", c.name())
                return
            cmd, why = validate_cmd(obj)
            if cmd is None:
                self._rl.warn("bad", "bad frame (%s) from %s" % (why, c.name()))
                return
            now = time.monotonic()
            self.intent.apply(cmd, now)
            c.last_cmd_t = now
            self.rx.append(now)
        else:
            self._rl.warn("bad", "bad frame (type=%r) from %s" % (typ, c.name()))

    # ── logger 구독 ─────────────────────────────────────────────────────
    async def _logger_loop(self):
        last = None
        while True:
            try:
                async with websockets.connect(self.logger_url, open_timeout=2) as ws:
                    log.info("logger up %s", self.logger_url)
                    last = "up"
                    async for raw in ws:
                        self._on_logger(raw)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                if last != "down":
                    log.info("logger down (%s: %s)", type(e).__name__, e)
                    last = "down"
            else:
                if last != "down":
                    log.info("logger down (닫힘)")
                    last = "down"
            await asyncio.sleep(1.0)

    def _on_logger(self, raw):
        if isinstance(raw, str):
            return
        try:
            d = msgpack.unpackb(raw, raw=False)
            quat = [float(x) for x in d["imu"]["quat"]]
            gyro = [float(x) for x in d["imu"]["ang_vel"]]
            q = [float(x) for x in d["motor"]["q"]]
            dq = [float(x) for x in d["motor"]["dq"]]
            if (len(quat), len(gyro), len(q), len(dq)) != (4, 3, NJ, NJ):
                raise ValueError("length")
            if not all(math.isfinite(x) for x in quat + gyro + q + dq):
                raise ValueError("non-finite")
        except Exception as e:
            self._rl.warn("logger_bad", "bad logger frame (%s)" % e)
            return
        self.logger_data = ([round(x, 5) for x in quat], [round(x, 5) for x in gyro],
                            [round(x, 5) for x in q], [round(x, 5) for x in dq])
        self.logger_t = time.monotonic()

    # ── 50 Hz 루프 ──────────────────────────────────────────────────────
    async def _tick_loop(self):
        nxt = time.monotonic()
        while True:
            try:
                self._tick(time.monotonic())
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._rl.warn("tick", "tick error %s: %s" % (type(e).__name__, e))
            nxt += PERIOD_S
            now = time.monotonic()
            if now - nxt > PERIOD_S:        # 한 주기보다 더 늦었다 → 몰아쓰기 없이 기한을 다시 잡는다
                nxt = now + PERIOD_S
            await asyncio.sleep(max(0.0, nxt - now))

    def _write_frame(self, f):
        state = {"seq": self.seq, "vx": f["vx"], "vy": f["vy"], "wz": f["wz"], "period_steps": 0,
                 "height_scale": 1.0, "turn_k": 1.0, "mode_req": f["mode_req"], "m5_preset": f["m5_preset"],
                 "m5_press_seq": f["m5_press_seq"], "clip_req": -1}
        try:
            _gui_mod.write(state, magic=_gui_mod.MAGIC_GUI_RELAY)
        except OSError as e:
            self._rl.warn("write", "gui shm write failed: %s" % e)
        self.seq = state["seq"]

    def _tick(self, now):
        fresh = self.intent.fresh(now)
        if self.was_fresh and not fresh:
            self.in_loss = True
            log.info("browser loss: 0 프레임 시작 (cmd %.0f ms 없음)", CMD_STALE_S * 1000)
        elif fresh and self.in_loss:
            self.in_loss = False
            log.info("browser loss 끝: cmd 재개")
        self.was_fresh = fresh
        self._write_frame(self.intent.take(now))
        if self.clients:
            self._broadcast(now)

    def _broadcast(self, now):
        lg = self.logger_t is not None and now - self.logger_t <= LOGGER_STALE_S and self.logger_data is not None
        if lg != self.logger_ok:
            self.logger_ok = lg
            log.info("logger %s", "up(데이터 수신)" if lg else "down(0.5 s 넘게 프레임 없음)")
        st = status_shm.read(self.status_path)
        quat, gyro, q, dq = self.logger_data if lg else (None, None, None, None)
        base = {"v": 1, "type": "state", "t": round(now - self.t0, 3), "quat": quat, "gyro": gyro, "q": q,
                "dq": dq, "status": None if st is None else dict((k, _clean(v)) for k, v in st.items())}
        prefix = json.dumps(base)[:-1]
        while self.rx and now - self.rx[0] > 1.0:
            self.rx.popleft()
        rx_hz = float(len(self.rx))
        for c in list(self.clients):
            while c.tx and now - c.tx[0] > 1.0:
                c.tx.popleft()
            link = {"rx_hz": rx_hz, "tx_hz": float(len(c.tx)), "logger": lg, "status": st is not None,
                    "controller": c is self.controller}
            self._push(c, prefix + ',"link":' + json.dumps(link) + "}")

    def _push(self, c, text):
        if c.send_task is not None and not c.send_task.done():
            return
        tr = getattr(c.ws, "transport", None)
        if tr is not None and tr.get_write_buffer_size() > WRITE_BUF_LIMIT:
            return
        c.send_task = asyncio.ensure_future(self._send_state(c, text))

    async def _send_state(self, c, text):
        try:
            await c.ws.send(text)
            c.tx.append(time.monotonic())
        except asyncio.CancelledError:
            raise
        except Exception:
            pass


_gui_mod = gui_shm


def main(argv=None):
    ap = argparse.ArgumentParser(description="G1 아케이드 중계 (protocol_relay_v1)")
    ap.add_argument("--bind", default="127.0.0.1", help="로봇에선 --bind 192.168.3.2 를 명시")
    ap.add_argument("--port", type=int, default=8710)
    ap.add_argument("--gui-shm", default=gui_shm.SHM_PATH)
    ap.add_argument("--status-shm", default=status_shm.SHM_PATH)
    ap.add_argument("--logger", default="ws://127.0.0.1:8700")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [relay] %(message)s", stream=sys.stderr)

    async def amain():
        relay = Relay(a.bind, a.port, a.gui_shm, a.status_shm, a.logger)
        await relay.start()
        stop = asyncio.Event()
        loop = asyncio.get_event_loop()
        for s in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(s, stop.set)
        await stop.wait()
        log.info("signal 받음 — 종료")
        await relay.stop()

    asyncio.run(amain())
    return 0


if __name__ == "__main__":
    sys.exit(main())
