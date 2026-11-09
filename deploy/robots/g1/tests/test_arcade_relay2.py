#!/usr/bin/env python3
"""arcade_relay2.py 시험 — 브라우저 intent → 의도 칸 0x6707(프레임마다 하트비트) · 상태 칸 + 이름표 + 관절각 → 브라우저.
고정 자료 = g1_arcade docs/protocol_relay_v2.md. 진짜 websockets 서버를 127.0.0.1 임시 포트에, shm 은 tmp 경로에.
  python3 -m pytest deploy/robots/g1/tests/test_arcade_relay2.py -q
"""
import asyncio
import contextlib
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

import msgpack
import pytest
import websockets

G1 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(G1 / "tools"))
import arcade_intent_shm as ish  # noqa: E402
import arcade_status_shm as ssh  # noqa: E402
import arcade_relay2  # noqa: E402

NO_LOGGER = "ws://127.0.0.1:1"
NAMES = {"proto": 2, "boot_id": "A1", "slot": "s1", "bundle": "b", "roles": ["walk", "jump", "crawl"],
         "skills": ["idle", "walk", "leap"], "notices": {"0": "", "1": "busy: event ignored"}}


def run(coro):
    return asyncio.run(coro)


def write_status(path, tick, **kw):
    v = dict.fromkeys(ssh.FIELDS, 0)
    v.update(magic=ssh.MAGIC, seq=2, tick=tick, z_fk=0.75, tilt_deg=2.0)
    v.update(kw)
    buf = struct.pack(ssh.FMT, *[v[k] for k in ssh.FIELDS])
    tmp = str(path) + ".tmp"
    with open(tmp, "wb") as f:
        f.write(buf)
    os.replace(tmp, str(path))


def write_names(path, **kw):
    d = dict(NAMES)
    d.update(kw)
    with open(str(path), "w") as f:
        json.dump(d, f)


@contextlib.asynccontextmanager
async def started(tmp_path, logger_url=NO_LOGGER, names=True, reset=True):
    if reset:
        ish._reset()
    if names:
        write_names(tmp_path / "names.json")
    relay = arcade_relay2.Relay(bind="127.0.0.1", port=0, intent_shm=str(tmp_path / "intent"),
                                status_shm=str(tmp_path / "status"), names_path=str(tmp_path / "names.json"),
                                logger_url=logger_url)
    await relay.start()
    try:
        yield relay
    finally:
        await relay.stop()


async def connect(relay):
    ws = await websockets.connect("ws://127.0.0.1:%d" % relay.port, max_queue=None)  # 안 읽는 동안 쌓여도 close 가 막히지 않게
    hello = msgpack.unpackb(await asyncio.wait_for(ws.recv(), 2), raw=False)
    return ws, hello


def intent(seq, fwd=0.0, side=0.0, turn=0.0, run=0, ev=None, ev_seq=0, ev_age_ms=0):
    return msgpack.packb({"t": "intent", "seq": seq, "fwd": fwd, "side": side, "turn": turn, "run": run,
                          "ev": ev, "ev_seq": ev_seq, "ev_age_ms": ev_age_ms})


def rd(tmp_path):
    return ish.read(str(tmp_path / "intent"))


async def next_state(ws, timeout=2.0):
    end = time.monotonic() + timeout
    while True:
        m = msgpack.unpackb(await asyncio.wait_for(ws.recv(), max(0.05, end - time.monotonic())), raw=False)
        if m.get("t") == "state":
            return m


async def fresh_state(ws, wait=0.15):
    """소켓에 쌓인 옛 프레임을 흘려보내고 wait 뒤의 최신 state."""
    await asyncio.sleep(wait)
    end = time.monotonic() + 0.3
    last = await next_state(ws)
    while time.monotonic() < end:
        try:
            m = msgpack.unpackb(await asyncio.wait_for(ws.recv(), 0.03), raw=False)
        except asyncio.TimeoutError:
            break
        if m.get("t") == "state":
            last = m
    return last


async def settle():
    await asyncio.sleep(0.1)


def test_heartbeat_seq_follows_browser_frames(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, hello = await connect(relay)
            assert hello["t"] == "hello" and hello["proto"] == 2 and hello["role"] == "controller"
            assert hello["names"]["boot_id"] == "A1"
            await ws.send(intent(0))
            await settle()
            s0 = rd(tmp_path)["seq"]
            for i in range(10):
                await ws.send(intent(i + 1, fwd=0.2))
                await asyncio.sleep(0.02)
            await settle()
            assert (rd(tmp_path)["seq"] - s0) & 0xFFFFFFFF == 10
            s1 = rd(tmp_path)["seq"]
            await asyncio.sleep(0.5)                       # 브라우저 멈춤 → 중계 타이머는 칸을 올리지 않는다
            assert rd(tmp_path)["seq"] == s1
            await ws.close()
    run(go())


def test_axes_clamped_and_nonfinite_zeroed(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            await ws.send(intent(1, fwd=5.0, side=-9.0, turn=float("nan"), run=1))
            await settle()
            f = rd(tmp_path)
            assert (f["fwd"], f["side"], f["turn"], f["run"]) == (1.0, -1.0, 0.0, 1)
            await ws.send(intent(2, fwd=float("inf"), side=float("-inf"), turn=0.25))
            await settle()
            f = rd(tmp_path)
            assert (f["fwd"], f["side"], f["turn"]) == (0.0, 0.0, 0.25)
            await ws.close()
    run(go())


def test_real_event_names_only(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            await ws.send(intent(1, ev="interact", ev_seq=1))
            await ws.send(intent(2, ev="posture:b", ev_seq=2))
            await settle()
            assert rd(tmp_path)["events"] == 0
            st = await next_state(ws)
            assert st["dropped"] == 2
            for i in range(50):                             # 같은 알 수 없는 ev · 같은 ev_seq 재전송 = 한 번만
                await ws.send(intent(10 + i, ev="interact", ev_seq=2))
            await settle()
            assert (await fresh_state(ws, 0.0))["dropped"] == 2
            await ws.send(intent(70, ev="interact", ev_seq=3))
            await settle()
            assert (await fresh_state(ws, 0.0))["dropped"] == 3
            await ws.send(intent(3, ev="jump", ev_seq=3))
            await settle()
            assert rd(tmp_path)["events"] == ish.EVENT_BITS["jump"]
            await ws.send(intent(4, ev="posture:z", ev_seq=4))
            await settle()
            assert rd(tmp_path)["events"] == 64
            await ws.close()
    run(go())


def test_repeated_ev_seq_is_not_a_new_event(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            await ws.send(intent(1, ev="jump", ev_seq=7))
            await settle()
            e1 = rd(tmp_path)["event_seq"]
            for i in range(5):
                await ws.send(intent(2 + i, ev="jump", ev_seq=7))
            await settle()
            assert rd(tmp_path)["event_seq"] == e1
            await ws.close()
    run(go())


def test_event_age_adds_relay_delay(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            await ws.send(intent(1, ev="jump", ev_seq=1, ev_age_ms=100))
            await settle()
            a = rd(tmp_path)["event_age_ms"]
            assert 100 <= a < 250, a
            await asyncio.sleep(0.3)
            await ws.send(intent(2))                        # 같은 이벤트가 끈적하게 남고 나이만 자란다
            await settle()
            b = rd(tmp_path)["event_age_ms"]
            assert b >= 100 + 300, b
            await ws.close()
    run(go())


def test_second_browser_is_viewer(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            c, h1 = await connect(relay)
            v, h2 = await connect(relay)
            assert (h1["role"], h2["role"]) == ("controller", "viewer")
            await c.send(intent(1, fwd=0.1))
            await settle()
            s = rd(tmp_path)["seq"]
            await v.send(intent(2, fwd=0.9))
            await settle()
            f = rd(tmp_path)
            assert f["seq"] == s and f["fwd"] == pytest.approx(0.1)
            assert (await next_state(v))["controller"] is False
            await c.close()
            await v.close()
    run(go())


def poll_applies(frame, last_ev_seq, recovering=False):
    """g1_ctrl poll 규칙 흉내(ArcadeIntent.h poll_intent): events != 0 이고 event_seq 가 바뀌었고 나이 ≤ 500 ms 이고
    stale 이 풀리는 틱이 아닐 때만 적용. recovering = 끊김(15 틱 무변화 · 파일 없음) 뒤 새 seq 가 처음 온 틱 — 기준선만(소비)."""
    return (not recovering and frame["events"] != 0 and frame["event_seq"] != last_ev_seq
            and frame["event_age_ms"] <= 500)


def test_relay_restart_does_not_replay_last_event(tmp_path):
    async def go():
        async with started(tmp_path) as relay1:
            ws, _ = await connect(relay1)
            await ws.send(intent(1, ev="jump", ev_seq=3))
            await settle()
            f1 = rd(tmp_path)
            assert f1["events"] == 1
            await ws.close()
        ish._reset()                                        # 중계 프로세스 2 = 새 시드
        async with started(tmp_path, reset=False) as relay2:
            ws, _ = await connect(relay2)
            await ws.send(intent(1))
            await settle()
            f2 = rd(tmp_path)
            assert f2["events"] == 0
            assert not poll_applies(f2, f1["event_seq"])
            await ws.close()
    run(go())


def test_frames_buffered_during_stall_are_young_but_not_applied_on_recovery(tmp_path):
    """V2 경우 3: 중계가 멈춘 동안(> 0.3 s = g1_ctrl stale) 소켓에 쌓인 브라우저 프레임은 풀린 뒤 한꺼번에 쓰이고, 나이는 «받은 뒤» 만
    더해져 젊다(≤ 500 ms — 나이 규칙만으로는 못 거른다). g1_ctrl 은 stale 이 풀리는 틱의 새 event_seq 를 소비만 한다."""
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            await ws.send(intent(1, fwd=1.0))
            await settle()
            f0 = rd(tmp_path)
            await asyncio.sleep(0.35)                       # 브라우저 프레임 공백 > 15 틱 × 20 ms — g1_ctrl 은 stale
            burst = [intent(2 + i, fwd=1.0) for i in range(5)]
            burst += [intent(7, fwd=1.0, ev="jump", ev_seq=1, ev_age_ms=0)]
            burst += [intent(8 + i, fwd=1.0, ev=None, ev_seq=1) for i in range(5)]
            for m in burst:                                 # 풀린 뒤 한꺼번에(소켓 버퍼)
                await ws.send(m)
            await settle()
            f1 = rd(tmp_path)
            assert f1["seq"] != f0["seq"] and f1["events"] == 1 and f1["event_seq"] != f0["event_seq"]
            assert f1["event_age_ms"] <= 500, f1             # 나이로는 걸러지지 않는다(멈춘 시간이 안 들어감)
            assert poll_applies(f1, f0["event_seq"])         # stale 이 아니었다면 적용될 프레임
            assert not poll_applies(f1, f0["event_seq"], recovering=True)   # 풀리는 틱 = 소비만
            await ws.send(intent(13, fwd=1.0, ev="jump", ev_seq=2, ev_age_ms=0))   # 회복 뒤 새로 누른 것은 평소대로
            await settle()
            f2 = rd(tmp_path)
            assert poll_applies(f2, f1["event_seq"])
            await ws.close()
    run(go())


def test_names_reloaded_when_status_restarts(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            write_status(tmp_path / "status", 1000)
            ws, _ = await connect(relay)
            st = await next_state(ws)
            assert st["names_boot"] == "A1"
            write_names(tmp_path / "names.json", boot_id="B2", roles=["x", "y", "z"])
            write_status(tmp_path / "status", 10)           # tick 감소 = 재기동
            for _ in range(20):
                st = await next_state(ws)
                if st["names_boot"] == "B2":
                    break
            assert st["names_boot"] == "B2"
            write_names(tmp_path / "names.json", boot_id="C3")
            write_status(tmp_path / "status", 11)           # boot_id 만 바뀜
            for _ in range(20):
                st = await next_state(ws)
                if st["names_boot"] == "C3":
                    break
            assert st["names_boot"] == "C3"
            await ws.close()
    run(go())


def test_state_frame_decodes_names_and_flags_robot_stale(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            st = await next_state(ws)
            assert st["robot_stale"] is True and st["st"] is None   # 상태 칸 없음
            write_status(tmp_path / "status", 50, role=1, skill=2, notice=1, safety_class=2, busy=1, cmd_vx=0.3)
            await asyncio.sleep(0.1)
            st = await next_state(ws)
            assert st["st"]["role"] == "jump" and st["st"]["skill"] == "leap"
            assert st["st"]["notice"] == "busy: event ignored"
            assert st["st"]["safety_class"] == 2 and st["st"]["busy"] == 1
            assert st["st"]["cmd"][0] == pytest.approx(0.3) and len(st["st"]["cmd"]) == 3
            assert st["robot_stale"] is False
            await asyncio.sleep(0.45)                       # tick 그대로 300 ms 넘게
            st = await fresh_state(ws, 0.0)
            assert st["robot_stale"] is True
            write_status(tmp_path / "status", 51, role=9, skill=0)   # 표 밖 번호는 그대로
            await asyncio.sleep(0.1)
            st = await next_state(ws)
            assert st["st"]["role"] == 9 and st["robot_stale"] is False
            await ws.close()
    run(go())


def test_no_names_file_gives_numbers_and_null(tmp_path):
    async def go():
        async with started(tmp_path, names=False) as relay:
            ws, hello = await connect(relay)
            assert hello["names"] is None
            write_status(tmp_path / "status", 5, role=1, skill=2, notice=1)
            await asyncio.sleep(0.1)
            st = await next_state(ws)
            assert (st["st"]["role"], st["st"]["skill"], st["st"]["notice"]) == (1, 2, 1)
            assert st["names_boot"] is None
            await ws.close()
    run(go())


async def _fake_logger():
    async def handler(ws):
        try:
            while True:
                fr = {"imu": {"quat": [1.0, 0.0, 0.0, 0.0], "ang_vel": [0, 0, 0]},
                      "motor": {"q": [0.01 * i for i in range(29)], "dq": [0.0] * 29}}
                await ws.send(msgpack.packb(fr, use_single_float=True))
                await asyncio.sleep(0.02)
        except websockets.exceptions.ConnectionClosed:
            return
    server = await websockets.serve(handler, "127.0.0.1", 0)
    return server, "ws://127.0.0.1:%d" % server.sockets[0].getsockname()[1]


def test_logger_q_quat_forwarded_then_null(tmp_path):
    async def go():
        server, url = await _fake_logger()
        async with started(tmp_path, logger_url=url) as relay:
            ws, _ = await connect(relay)
            await asyncio.sleep(0.3)
            st = await next_state(ws)
            assert len(st["q"]) == 29 and st["q"][3] == pytest.approx(0.03) and st["quat"] == [1.0, 0.0, 0.0, 0.0]
            await ws.close()
        server.close()
        await server.wait_closed()
    run(go())


def test_no_logger_gives_null_q(tmp_path):
    async def go():
        async with started(tmp_path) as relay:
            ws, _ = await connect(relay)
            st = await next_state(ws)
            assert st["q"] is None and st["quat"] is None
            await ws.close()
    run(go())


def test_shutdown_zero_frame_is_last_write(tmp_path):
    async def go():
        ish._reset()
        write_names(tmp_path / "names.json")
        relay = arcade_relay2.Relay(bind="127.0.0.1", port=0, intent_shm=str(tmp_path / "intent"),
                                    status_shm=str(tmp_path / "status"), names_path=str(tmp_path / "names.json"),
                                    logger_url=NO_LOGGER)
        calls = []                                          # (fwd, 쓴 뒤 칸 seq) — 마지막 쓰기가 0 프레임인지 본다
        orig = ish.write_intent

        def rec(*a, **k):
            orig(*a, **k)
            calls.append((a[0], rd(tmp_path)["seq"]))
        ish.write_intent = rec
        await relay.start()
        ws, _ = await connect(relay)
        await ws.send(intent(0, fwd=1.0, side=1.0, turn=1.0))
        await settle()
        sender_stop = False

        async def flood():
            i = 1
            try:
                while not sender_stop:
                    await ws.send(intent(i, fwd=1.0, side=1.0, turn=1.0, run=1))
                    i += 1
                    await asyncio.sleep(0.002)
            except Exception:
                pass
        t = asyncio.ensure_future(flood())
        await asyncio.sleep(0.2)
        seq_before = rd(tmp_path)["seq"]
        assert rd(tmp_path)["fwd"] == 1.0
        await relay.stop()
        f = rd(tmp_path)
        assert (f["fwd"], f["side"], f["turn"], f["run"]) == (0.0, 0.0, 0.0, 0)
        seq_end = f["seq"]
        await asyncio.sleep(0.2)                            # 종료 뒤 더 쓰이지 않는다
        assert rd(tmp_path)["seq"] == seq_end
        sender_stop = True
        t.cancel()
        await asyncio.gather(t, return_exceptions=True)
        assert (seq_end - seq_before) & 0xFFFFFFFF >= 1
        ish.write_intent = orig
        assert calls[-1][0] == 0.0 and calls[-2][0] == 1.0          # 0 프레임이 마지막 쓰기, 그 앞은 브라우저 프레임
        assert (calls[-1][1] - calls[-2][1]) & 0xFFFFFFFF == 1      # 마지막 브라우저 프레임 뒤 seq 는 정확히 +1
        assert sum(1 for c in calls if c[0] == 0.0) == 1
    run(go())
