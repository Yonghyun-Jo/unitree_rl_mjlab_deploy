#!/usr/bin/env python3
"""G1 아케이드 중계 v2 — 브라우저 intent → 의도 칸 0x6707(프레임마다 하트비트) · 상태 칸 0x6708 + 이름표 + 관절각 → 브라우저.

고정 자료 = g1_arcade docs/protocol_relay_v2.md (여기서 바꾸지 않는다 — 어긋나면 그 문서가 이긴다).
2A arcade_relay.py 의 WS · 역할 · logger · 50 Hz 루프 틀을 가져오고, 의도 · 상태만 바꿨다(REAL_CAPS · hold_seq · GuiCtrl 0x6705 없음).
  - msgpack 바이너리 프레임. 처음 붙은 연결 = controller, 나머지 = viewer.
  - 의도 칸 seq 는 controller 의 intent 프레임 1 개당 +1(받을 때 바로 씀) — 중계 타이머는 칸을 쓰지 않는다(I6).
  - 시작 직후 events=0 — 브라우저 이벤트가 오기 전엔 이벤트 비트를 싣지 않는다(재기동해도 옛 이벤트 재생 없음).
  - 50 Hz 루프(몰아쓰기 없음)는 상태 칸·logger 를 읽어 state 를 브로드캐스트한다. robot_stale = 상태 칸 tick 이 300 ms 안 바뀜.

런타임 = logger 의 venv python(py3.8, websockets 12/13, msgpack). python 3.8 문법만 쓴다.
로봇 실행:
  ~/dyros_ws/piene_ws/piene_g1_logger/.venv/bin/python arcade_relay2.py --bind 192.168.3.2 --port 8711
"""
import argparse
import asyncio
import collections
import logging
import math
import signal
import sys
import time

import msgpack
import websockets

import arcade_intent_shm
import arcade_status_shm

log = logging.getLogger("arcade_relay2")
# websockets 13 은 serve/connect 를 처음 쓸 때 지연 import 한다 — 50 Hz 루프가 돌기 시작한 뒤 멈추지 않게 미리 불러 둔다.
_PRELOAD = (websockets.serve, websockets.connect, websockets.exceptions.ConnectionClosed)

PROTO = 2
CONTROLLER_SILENCE_S = 1.0
LOGGER_STALE_S = 0.5
ROBOT_STALE_S = 0.3
PERIOD_S = 0.02
WRITE_BUF_LIMIT = 64 * 1024
NJ = 29
U32 = 0xFFFFFFFF


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


def _is_u32(x):
    return type(x) is int and 0 <= x <= U32


def _axis(x):
    """축 값 — 숫자가 아니면 None(프레임 거절), 비유한은 0 은 write_intent 가 처리한다."""
    if type(x) not in (int, float):
        return None
    return float(x)


def validate_intent(obj):
    """(intent dict, None) 또는 (None, 사유)."""
    for k in ("seq", "fwd", "side", "turn", "run", "ev", "ev_seq", "ev_age_ms"):
        if k not in obj:
            return None, "missing " + k
    if not _is_u32(obj["seq"]):
        return None, "bad seq"
    axes = [_axis(obj[k]) for k in ("fwd", "side", "turn")]
    if any(a is None for a in axes):
        return None, "bad axis"
    if obj["run"] not in (0, 1) or type(obj["run"]) not in (int, bool):
        return None, "bad run"
    ev = obj["ev"]
    if ev is not None and type(ev) is not str:
        return None, "bad ev"
    if not _is_u32(obj["ev_seq"]) or not _is_u32(obj["ev_age_ms"]):
        return None, "bad ev_seq/ev_age_ms"
    return {"fwd": axes[0], "side": axes[1], "turn": axes[2], "run": int(obj["run"]), "ev": ev,
            "ev_seq": obj["ev_seq"], "ev_age_ms": obj["ev_age_ms"]}, None


class Client:
    def __init__(self, ws, cid, now):
        self.ws = ws
        self.cid = cid
        self.role = "viewer"
        self.last_intent_t = now
        self.send_task = None
        self.viewer_logged = False
        self.hello_task = None

    def name(self):
        return "#%d %s" % (self.cid, getattr(self.ws, "remote_address", None))


def _r(v):
    return round(v, 5) if isinstance(v, float) and math.isfinite(v) else v


def _name(table, idx):
    try:
        if type(idx) is int and 0 <= idx < len(table):
            return str(table[idx])
    except TypeError:
        pass
    return idx


class Relay:
    def __init__(self, bind="127.0.0.1", port=8711, intent_shm=arcade_intent_shm.SHM_PATH,
                 status_shm=arcade_status_shm.SHM_PATH, names_path=arcade_status_shm.NAMES_PATH,
                 logger_url="ws://127.0.0.1:8700"):
        self.bind, self.port = bind, port
        self.intent_path, self.status_path, self.names_path = intent_shm, status_shm, names_path
        self.logger_url = logger_url
        self.clients = set()
        self.controller = None
        self.last_ev_seq = None
        self.dropped = 0
        self.names = arcade_status_shm.read_names(self.names_path)
        self.st_tick = None
        self.st_change_t = None
        self.logger_data = None
        self.logger_t = None
        self.logger_ok = False
        self.t0 = time.monotonic()
        self._cid = 0
        self._rl = RateLimitedLog()
        self._server = None
        self._tasks = []
        self._stopped = False

    # ── 수명 ────────────────────────────────────────────────────────────
    async def start(self):
        self._server = await websockets.serve(self._handler, self.bind, self.port, max_size=65536)
        self.port = self._server.sockets[0].getsockname()[1]
        log.info("listening ws://%s:%d  intent=%s status=%s names=%s logger=%s", self.bind, self.port,
                 self.intent_path, self.status_path, self.names_path, self.logger_url)
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
        self._write(0.0, 0.0, 0.0, 0, 0, 0)         # 종료: 축 0 프레임 한 번(이벤트 새로 싣지 않음)
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
                log.info("controller %s 끊김 — 조종권 비움(의도 칸 seq 정지)", c.name())
            log.info("client disconnect %s (남은 %d)", c.name(), len(self.clients))

    def _assign_role(self, c, now):
        old = self.controller
        if old is None or now - old.last_intent_t > CONTROLLER_SILENCE_S:
            if old is not None:
                old.role = "viewer"
                log.info("role change %s controller→viewer (intent 침묵 %.1fs), 새 controller %s",
                         old.name(), now - old.last_intent_t, c.name())
                old.hello_task = asyncio.ensure_future(self._send_quiet(old, self._hello("viewer")))
            self.controller = c
            c.role = "controller"
            c.last_intent_t = now
            self.last_ev_seq = None

    def _hello(self, role):
        return msgpack.packb({"t": "hello", "proto": PROTO, "role": role, "names": self.names})

    async def _send_quiet(self, c, data):
        try:
            await c.ws.send(data)
        except Exception:
            pass

    # ── 받기 ────────────────────────────────────────────────────────────
    async def _on_message(self, c, msg):
        try:
            await self._on_message_inner(c, msg, time.monotonic())
        except websockets.exceptions.ConnectionClosed:
            raise
        except Exception as e:
            self._rl.warn("bad", "bad frame (%s) from %s" % (type(e).__name__, c.name()))

    async def _on_message_inner(self, c, msg, recv_t):
        if not isinstance(msg, bytes):
            self._rl.warn("bad", "bad frame (text) from %s" % c.name())
            return
        try:
            obj = msgpack.unpackb(msg, raw=False)
        except Exception:
            self._rl.warn("bad", "bad frame (not msgpack) from %s" % c.name())
            return
        if not isinstance(obj, dict):
            self._rl.warn("bad", "bad frame (not a map) from %s" % c.name())
            return
        typ = obj.get("t")
        if typ == "ping":
            if "id" in obj:
                await c.ws.send(msgpack.packb({"t": "pong", "id": obj["id"]}))
            return
        if typ != "intent":
            self._rl.warn("bad", "bad frame (t=%r) from %s" % (typ, c.name()))
            return
        if c is not self.controller:
            if not c.viewer_logged:
                c.viewer_logged = True
                log.info("viewer intent 무시 %s", c.name())
            return
        it, why = validate_intent(obj)
        if it is None:
            self._rl.warn("bad", "bad frame (%s) from %s" % (why, c.name()))
            return
        c.last_intent_t = recv_t
        bits, age = 0, 0
        if it["ev"] is not None:
            if it["ev"] not in arcade_intent_shm.EVENT_BITS:
                self.dropped += 1
                self._rl.warn("drop", "ev %r 는 실기 이벤트가 아님 — 버림(누적 %d)" % (it["ev"], self.dropped))
            elif it["ev_seq"] != self.last_ev_seq:
                self.last_ev_seq = it["ev_seq"]
                bits = arcade_intent_shm.EVENT_BITS[it["ev"]]
                age = it["ev_age_ms"] + int(round((time.monotonic() - recv_t) * 1000.0))
        self._write(it["fwd"], it["side"], it["turn"], it["run"], bits, age)

    def _write(self, fwd, side, turn, run, events, age):
        try:
            arcade_intent_shm.write_intent(fwd, side, turn, run, events, age, path=self.intent_path)
        except OSError as e:
            self._rl.warn("write", "intent shm write failed: %s" % e)

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
            q = [float(x) for x in d["motor"]["q"]]
            if (len(quat), len(q)) != (4, NJ):
                raise ValueError("length")
            if not all(math.isfinite(x) for x in quat + q):
                raise ValueError("non-finite")
        except Exception as e:
            self._rl.warn("logger_bad", "bad logger frame (%s)" % e)
            return
        self.logger_data = ([round(x, 5) for x in quat], [round(x, 5) for x in q])
        self.logger_t = time.monotonic()

    # ── 50 Hz 루프(상태 송신만 — 의도 칸은 쓰지 않는다) ─────────────────
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
            if now - nxt > PERIOD_S:        # 몰아쓰기 없이 기한을 다시 잡는다
                nxt = now + PERIOD_S
            await asyncio.sleep(max(0.0, nxt - now))

    def _reload_names(self, why):
        new = arcade_status_shm.read_names(self.names_path)
        if new is not None or self.names is not None:
            log.info("이름표 다시 읽음 (%s) boot_id=%s", why, None if new is None else new.get("boot_id"))
        self.names = new

    def _tick(self, now):
        st = arcade_status_shm.read(self.status_path)
        if st is not None:
            tick = st["tick"]
            if self.st_tick is None or tick != self.st_tick:
                if self.st_tick is not None and tick < self.st_tick:
                    self._reload_names("상태 칸 tick 감소 = 재기동")
                self.st_tick = tick
                self.st_change_t = now
        stale = st is None or self.st_change_t is None or now - self.st_change_t > ROBOT_STALE_S
        cur_boot = None if self.names is None else self.names.get("boot_id")
        if st is not None and not stale:
            disk = arcade_status_shm.read_names(self.names_path)
            if disk is not None and disk.get("boot_id") != cur_boot:
                self.names = disk
                log.info("이름표 다시 읽음 (boot_id %s → %s)", cur_boot, disk.get("boot_id"))
        if self.clients:
            self._broadcast(now, st, stale)

    def _decode(self, st):
        if st is None:
            return None
        n = self.names or {}
        notices = n.get("notices") if isinstance(n.get("notices"), dict) else {}
        notice = notices.get(str(st["notice"]), st["notice"]) if self.names is not None else st["notice"]
        return {"role": _name(n.get("roles", []), st["role"]) if self.names is not None else st["role"],
                "skill": _name(n.get("skills", []), st["skill"]) if self.names is not None else st["skill"],
                "notice": notice, "safety_class": st["safety_class"], "busy": st["busy"],
                "qd_warn": st["qd_warn"], "qd_crit": st["qd_crit"], "intent_stale": st["intent_stale"],
                "rearmed": st["rearmed"], "fsm_state": st["fsm_state"], "tick": st["tick"],
                "tick_us": st["tick_us"], "z_fk": _r(st["z_fk"]), "tilt_deg": _r(st["tilt_deg"]),
                "cmd": [_r(st["cmd_vx"]), _r(st["cmd_vy"]), _r(st["cmd_wz"])]}

    def _broadcast(self, now, st, stale):
        lg = self.logger_t is not None and now - self.logger_t <= LOGGER_STALE_S and self.logger_data is not None
        if lg != self.logger_ok:
            self.logger_ok = lg
            log.info("logger %s", "up(데이터 수신)" if lg else "down(0.5 s 넘게 프레임 없음)")
        quat, q = self.logger_data if lg else (None, None)
        boot = None if self.names is None else self.names.get("boot_id")
        base = {"t": "state", "q": q, "quat": quat, "st": self._decode(st), "robot_stale": bool(stale),
                "names_boot": boot, "dropped": self.dropped}
        for c in list(self.clients):
            base["controller"] = c is self.controller
            self._push(c, msgpack.packb(base))

    def _push(self, c, data):
        if c.send_task is not None and not c.send_task.done():
            return
        tr = getattr(c.ws, "transport", None)
        if tr is not None and tr.get_write_buffer_size() > WRITE_BUF_LIMIT:
            return
        c.send_task = asyncio.ensure_future(self._send_state(c, data))

    async def _send_state(self, c, data):
        try:
            await c.ws.send(data)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="G1 아케이드 중계 v2 (protocol_relay_v2)")
    ap.add_argument("--bind", default="127.0.0.1", help="로봇에선 --bind 192.168.3.2 를 명시")
    ap.add_argument("--port", type=int, default=8711)
    ap.add_argument("--intent-shm", default=arcade_intent_shm.SHM_PATH)
    ap.add_argument("--status-shm", default=arcade_status_shm.SHM_PATH)
    ap.add_argument("--names", default=arcade_status_shm.NAMES_PATH)
    ap.add_argument("--logger", default="ws://127.0.0.1:8700")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [relay2] %(message)s", stream=sys.stderr)

    async def amain():
        relay = Relay(a.bind, a.port, a.intent_shm, a.status_shm, a.names, a.logger)
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
