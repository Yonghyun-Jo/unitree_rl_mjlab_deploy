#!/usr/bin/env python3
"""arcade_relay.py 시험 — 브라우저 의도 → GuiCtrl 0x6705(50 Hz 하트비트) · 로봇 상태 → 브라우저.
고정 자료 = g1_arcade docs/protocol_relay_v1.md. 가짜 shm 경로 · 가짜 logger WS(asyncio), 진짜 websockets 서버.
로봇과 같은 환경(py3.8 + websockets 12)에서 돈다:  python -m pytest deploy/robots/g1/tests/test_arcade_relay.py
"""
import asyncio
import contextlib
import importlib.util
import json
import logging
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import msgpack
import pytest
import websockets

G1 = Path(__file__).resolve().parent.parent
TOOLS = G1 / "tools"
sys.path.insert(0, str(TOOLS))
import gui_shm  # noqa: E402
import status_shm  # noqa: E402
import arcade_relay  # noqa: E402

NO_LOGGER = "ws://127.0.0.1:1"
GUI_FIELDS = ("magic", "seq", "mode_req", "vx", "vy", "wz", "period_steps", "height_scale", "turn_k",
              "m5_preset", "m5_press_seq", "clip_req")
LOGGER_JOINTS = Path("/home/piene/piene_g1_logger/g1_logger/joints.py")


def parse_gui(buf):
    return dict(zip(GUI_FIELDS, struct.unpack(gui_shm.FMT, buf)))


@contextlib.asynccontextmanager
async def started_relay(tmp_path, logger_url=NO_LOGGER, status_path=None):
    """Relay 를 port 0 으로 띄우고, gui_shm.write 를 감싸 «쓸 때마다 (시각, 프레임)» 을 기록한다."""
    frames = []
    orig_write, orig_path = gui_shm.write, gui_shm.SHM_PATH

    def rec(state, magic=gui_shm.MAGIC):
        orig_write(state, magic=magic)
        with open(gui_shm.SHM_PATH, "rb") as f:
            frames.append((time.monotonic(), parse_gui(f.read())))

    gui_shm.write = rec
    relay = arcade_relay.Relay(bind="127.0.0.1", port=0, gui_shm=str(tmp_path / "gui"),
                               status_shm=status_path or str(tmp_path / "no_status"), logger_url=logger_url)
    await relay.start()
    try:
        yield relay, frames
    finally:
        await relay.stop()
        gui_shm.write, gui_shm.SHM_PATH = orig_write, orig_path


async def connect(relay):
    ws = await websockets.connect("ws://127.0.0.1:%d" % relay.port)
    hello = json.loads(await asyncio.wait_for(ws.recv(), 2))
    return ws, hello


def cmd(seq, vx=0.0, vy=0.0, wz=0.0, mode_req=0, m5_preset=0, m5_press_seq=0):
    return json.dumps({"v": 1, "type": "cmd", "seq": seq, "mode_req": mode_req, "m5_preset": m5_preset,
                       "m5_press_seq": m5_press_seq, "vx": vx, "vy": vy, "wz": wz})


async def stream(ws, secs, **kw):
    """cmd 를 50 Hz 로 secs 동안. 마지막으로 보낸 시각(monotonic)을 돌려준다."""
    t_end = time.monotonic() + secs
    seq, last = 0, time.monotonic()
    while time.monotonic() < t_end:
        seq += 1
        await ws.send(cmd(seq, **kw))
        last = time.monotonic()
        await asyncio.sleep(0.02)
    return last


async def next_state(ws, timeout=2.0):
    end = time.monotonic() + timeout
    while True:
        m = json.loads(await asyncio.wait_for(ws.recv(), max(0.05, end - time.monotonic())))
        if m.get("type") == "state":
            return m


def run(coro):
    return asyncio.run(coro)


# ── 브리프의 여섯 시험 ──────────────────────────────────────────────────────────
def test_writes_relay_frames_at_50hz(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, hello = await connect(relay)
            assert hello["role"] == "controller"
            t0 = time.monotonic()
            await stream(ws, 1.4, vx=0.3)
            await ws.close()
            win = [f for t, f in frames if t0 + 0.2 <= t < t0 + 1.2]
            assert 45 <= len(win) <= 55, len(win)           # 50 Hz ± 10 %
            assert all(f["magic"] == 0x6705 for _, f in frames)
            seqs = [f["seq"] for _, f in frames]
            assert all((b - a) & 0xFFFFFFFF == 1 for a, b in zip(seqs, seqs[1:]))   # 매번 +1
            assert all(abs(f["vx"] - 0.3) < 1e-6 for f in win)
            f = win[0]
            assert (f["period_steps"], f["height_scale"], f["turn_k"], f["clip_req"]) == (0, 1.0, 1.0, -1)
    run(go())


def test_browser_loss_writes_zero_within_300ms(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, _ = await connect(relay)
            last = await stream(ws, 0.5, vx=0.5, vy=0.2, wz=0.3)
            await asyncio.sleep(0.6)
            after = [(t, f) for t, f in frames if t > last]
            first_zero = next(t for t, f in after if f["vx"] == 0 and f["vy"] == 0 and f["wz"] == 0)
            assert first_zero - last <= 0.30, first_zero - last
            late = [f for t, f in after if t >= first_zero]
            assert len(late) >= 15                                   # 하트비트는 계속
            assert all(f["vx"] == 0 and f["mode_req"] == 0 and f["m5_preset"] == 0 for f in late)
            seqs = [f["seq"] for _, f in frames]
            assert all((b - a) & 0xFFFFFFFF == 1 for a, b in zip(seqs, seqs[1:]))
    run(go())


def test_second_client_is_view_only(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            a, ha = await connect(relay)
            b, hb = await connect(relay)
            assert (ha["role"], hb["role"]) == ("controller", "viewer")
            t0 = time.monotonic()
            await stream(b, 0.3, vx=1.0)
            assert all(f["vx"] == 0 for t, f in frames if t >= t0)   # viewer cmd 는 무시
            await stream(a, 0.3, vx=0.4)
            assert any(abs(f["vx"] - 0.4) < 1e-6 for _, f in frames)
            assert not any(abs(f["vx"] - 1.0) < 1e-6 for _, f in frames)
            st = await next_state(b)
            assert st["link"]["controller"] is False
    run(go())


def test_no_logger_sends_status_only(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, hello = await connect(relay)
            assert hello["type"] == "hello" and len(hello["joint_names"]) == 29
            await stream(ws, 0.2, vx=0.5)
            st = await next_state(ws)
            assert st["link"]["logger"] is False and st["link"]["status"] is False
            assert st["quat"] is None and st["gyro"] is None and st["q"] is None and st["dq"] is None
            assert st["status"] is None
            await stream(ws, 0.3, vx=0.5)
            assert abs(frames[-1][1]["vx"] - 0.5) < 1e-6              # cmd 는 계속 받는다
    run(go())


def test_clamps_and_rejects_bad_frames(tmp_path, caplog):
    caplog.set_level(logging.INFO, logger="arcade_relay")

    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, _ = await connect(relay)
            await ws.send(cmd(1, vx=99, vy=-99, wz=99))
            await asyncio.sleep(0.1)
            f = frames[-1][1]
            assert (f["vx"], f["wz"]) == (pytest.approx(gui_shm.VXCAP), pytest.approx(gui_shm.WCAP))
            assert f["vy"] == pytest.approx(-gui_shm.VYCAP)
            await ws.send(cmd(2, vx=-99, vy=99, wz=-99))
            await asyncio.sleep(0.1)
            f = frames[-1][1]
            assert f["vx"] == pytest.approx(-gui_shm.VXCAP_BWD) and f["vy"] == pytest.approx(gui_shm.VYCAP)
            assert f["wz"] == pytest.approx(-gui_shm.WCAP)

            good = {"v": 1, "type": "cmd", "seq": 3, "mode_req": 0, "m5_preset": 0, "m5_press_seq": 0,
                    "vx": 0.5, "vy": 0, "wz": 0}
            bad = [dict(good, v=2, vx=2.0),
                   dict((k, v) for k, v in good.items() if k != "wz"),
                   '{"v":1,"type":"cmd","seq":4,"mode_req":0,"m5_preset":0,"m5_press_seq":0,"vx":NaN,"vy":0,"wz":0}',
                   '{"v":1,"type":"cmd","seq":4,"mode_req":0,"m5_preset":0,"m5_press_seq":0,"vx":Infinity,"vy":0,"wz":0}',
                   dict(good, vx="2.0"), dict(good, type="bogus", vx=2.0), "not json", b"\x00\x01"]
            t0 = time.monotonic()
            for i in range(25):
                await ws.send(cmd(10 + i, vx=0.5))
                b = bad[i % len(bad)]
                await ws.send(b if isinstance(b, (str, bytes)) else json.dumps(b))
                await asyncio.sleep(0.02)
            win = [f for t, f in frames if t >= t0]
            assert win and all(abs(f["vx"] - 0.5) < 1e-6 for f in win)   # 나쁜 프레임은 값을 못 바꾼다
            await ws.send(cmd(99, vx=0.5))                               # 연결은 살아 있다
    run(go())
    n = [r for r in caplog.records if "bad frame" in r.getMessage()]
    assert 1 <= len(n) <= 3, [r.getMessage() for r in n]               # 로그는 속도 제한(프레임마다 아님)


def test_one_shot_fields(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, _ = await connect(relay)
            await stream(ws, 0.2, vx=0.1, m5_press_seq=7)
            await ws.send(cmd(500, vx=0.1, mode_req=3, m5_preset=2, m5_press_seq=8))
            await stream(ws, 0.3, vx=0.1, m5_press_seq=8)
            fs = [f for _, f in frames]
            assert sum(1 for f in fs if f["mode_req"] == 3) == 1
            assert sum(1 for f in fs if f["m5_preset"] == 2) == 1
            assert sum(1 for f in fs if f["mode_req"] != 0) == 1
            hit = next(f for f in fs if f["mode_req"] == 3)
            assert hit["m5_preset"] == 2 and hit["m5_press_seq"] == 8
            assert fs[-1]["mode_req"] == 0 and fs[-1]["m5_preset"] == 0 and fs[-1]["m5_press_seq"] == 8
    run(go())


# ── 프로토콜 규칙 ─────────────────────────────────────────────────────────────
def test_latch_survives_several_cmds_in_one_write_period():
    it = arcade_relay.Intent()
    t = 100.0
    it.apply(dict(seq=1, mode_req=3, m5_preset=2, m5_press_seq=9, vx=0.1, vy=0.0, wz=0.0), t)
    it.apply(dict(seq=2, mode_req=0, m5_preset=0, m5_press_seq=9, vx=0.2, vy=0.0, wz=0.0), t + 0.001)
    it.apply(dict(seq=3, mode_req=0, m5_preset=0, m5_press_seq=9, vx=0.3, vy=0.0, wz=0.0), t + 0.002)
    f = it.take(t + 0.01)
    assert (f["mode_req"], f["m5_preset"], f["m5_press_seq"]) == (3, 2, 9)   # 0 이 라치를 지우지 못한다
    assert f["vx"] == pytest.approx(0.3)                                      # 속도는 가장 최근 값
    f = it.take(t + 0.03)
    assert (f["mode_req"], f["m5_preset"]) == (0, 0) and f["vx"] == pytest.approx(0.3)
    it.apply(dict(seq=4, mode_req=3, m5_preset=0, m5_press_seq=9, vx=0.0, vy=0.0, wz=0.0), t + 0.04)
    it.apply(dict(seq=5, mode_req=5, m5_preset=0, m5_press_seq=9, vx=0.0, vy=0.0, wz=0.0), t + 0.041)
    assert it.take(t + 0.05)["mode_req"] == 5                                 # 나중 값이 이긴다


def test_intent_zeroes_when_stale_and_drops_latch():
    it = arcade_relay.Intent()
    it.apply(dict(seq=1, mode_req=3, m5_preset=0, m5_press_seq=4, vx=1.0, vy=0.0, wz=0.0), 10.0)
    assert it.take(10.27)["vx"] == 1.0
    f = it.take(10.29)
    assert f["vx"] == 0 and f["mode_req"] == 0 and f["m5_preset"] == 0 and f["m5_press_seq"] == 4


def test_controller_handover_after_1s_silence(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            a, ha = await connect(relay)
            assert ha["role"] == "controller"
            await a.send(cmd(1, vx=0.2))
            b, hb = await connect(relay)                   # 아직 1 s 안 지났다 → viewer
            assert hb["role"] == "viewer"
            await asyncio.sleep(1.2)                       # a 가 침묵(반쯤 열린 TCP)
            c, hc = await connect(relay)                   # 새로 붙는 연결이 controller
            assert hc["role"] == "controller"
            demoted = json.loads(await asyncio.wait_for(a.recv(), 2))
            while demoted.get("type") != "hello":
                demoted = json.loads(await asyncio.wait_for(a.recv(), 2))
            assert demoted["role"] == "viewer"
            t0 = time.monotonic()
            await stream(a, 0.2, vx=1.0)                   # 옛 controller 와 기존 viewer 는 조종권이 없다
            await stream(b, 0.2, vx=1.5)
            assert all(f["vx"] == 0 for t, f in frames if t >= t0)
            await stream(c, 0.2, vx=0.6)
            assert any(abs(f["vx"] - 0.6) < 1e-6 for _, f in frames)
    run(go())


def test_ping_pong_echo(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            a, _ = await connect(relay)
            b, hb = await connect(relay)                   # viewer 도 ping 가능
            assert hb["role"] == "viewer"
            for ws in (a, b):
                await ws.send(json.dumps({"v": 1, "type": "ping", "id": 5, "t": 1234.5}))
                while True:
                    m = json.loads(await asyncio.wait_for(ws.recv(), 2))
                    if m["type"] == "pong":
                        break
                assert m == {"v": 1, "type": "pong", "id": 5, "t": 1234.5}
    run(go())


async def _fake_logger(frames_hz=50):
    async def handler(ws):
        n = 0
        try:
            while True:
                n += 1
                fr = {"t": n * 0.02, "wall_t": time.time(),
                      "imu": {"quat": [1.0, 0.0, 0.0, 0.0], "ang_vel": [0.1, 0.2, 0.3], "lin_acc": [0, 0, 9.8]},
                      "motor": {"q": [0.01 * i for i in range(29)], "dq": [0.0] * 29, "tau_est": [0.0] * 29}}
                await ws.send(msgpack.packb(fr, use_single_float=True))
                await asyncio.sleep(1.0 / frames_hz)
        except websockets.exceptions.ConnectionClosed:
            return
    server = await websockets.serve(handler, "127.0.0.1", 0)
    return server, "ws://127.0.0.1:%d" % server.sockets[0].getsockname()[1]


def test_logger_data_forwarded_then_nulled_after_500ms(tmp_path):
    async def go():
        server, url = await _fake_logger()
        async with started_relay(tmp_path, logger_url=url) as (relay, frames):
            ws, _ = await connect(relay)
            await asyncio.sleep(0.3)
            st = await next_state(ws)
            assert st["link"]["logger"] is True
            assert st["quat"] == [1.0, 0.0, 0.0, 0.0] and len(st["q"]) == 29 and len(st["dq"]) == 29
            assert st["gyro"] == pytest.approx([0.1, 0.2, 0.3], abs=1e-5)
            assert st["q"][3] == pytest.approx(0.03, abs=1e-5)
            server.close()
            await server.wait_closed()
            await asyncio.sleep(0.8)
            st = await next_state(ws)
            for _ in range(100):
                if not st["link"]["logger"]:
                    break
                st = await next_state(ws)
            assert st["link"]["logger"] is False
            assert st["quat"] is None and st["gyro"] is None and st["q"] is None and st["dq"] is None
    run(go())


def test_status_block_forwarded(tmp_path):
    p = tmp_path / "status"
    vals = dict(zip(status_shm.FIELDS, [status_shm.MAGIC, 2, 77, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0,
                                        0.75, 2.0, 0.0, 0.0, 0.0]))
    p.write_bytes(struct.pack(status_shm.FMT, *[vals[k] for k in status_shm.FIELDS]))

    async def go():
        async with started_relay(tmp_path, status_path=str(p)) as (relay, frames):
            ws, _ = await connect(relay)
            st = await next_state(ws)
            assert st["link"]["status"] is True
            assert set(st["status"]) == set(status_shm.FIELDS[1:])
            assert st["status"]["tick"] == 77 and st["status"]["active_mode"] == 1
            assert st["status"]["z_fk"] == pytest.approx(0.75)
    run(go())


def test_hello_joint_names_match_logger_repo():
    names = arcade_relay.G1_JOINT_NAMES
    assert len(names) == 29 and len(set(names)) == 29
    if not LOGGER_JOINTS.exists():
        pytest.skip("piene_g1_logger 가 이 기계에 없다")
    spec = importlib.util.spec_from_file_location("g1_joints_ref", str(LOGGER_JOINTS))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert list(names) == list(mod.G1_JOINT_NAMES)


def test_sigterm_writes_zero_frame_and_exits_0(tmp_path):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    gui = tmp_path / "gui"
    proc = subprocess.Popen([sys.executable, str(TOOLS / "arcade_relay.py"), "--bind", "127.0.0.1",
                             "--port", str(port), "--gui-shm", str(gui), "--status-shm", str(tmp_path / "ns"),
                             "--logger", NO_LOGGER], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    async def go():
        ws = None
        for _ in range(100):
            try:
                ws = await websockets.connect("ws://127.0.0.1:%d" % port)
                break
            except OSError:
                await asyncio.sleep(0.1)
        assert ws is not None
        await stream(ws, 0.4, vx=1.0, mode_req=0)
        before = parse_gui(gui.read_bytes())
        assert before["vx"] == pytest.approx(1.0) and before["magic"] == 0x6705
        proc.send_signal(signal.SIGTERM)
        for _ in range(60):                                  # 종료하는 동안에도 cmd 는 계속 온다
            if proc.poll() is not None:
                break
            with contextlib.suppress(Exception):
                await ws.send(cmd(9999, vx=1.0, mode_req=2))
            await asyncio.sleep(0.05)
        return before
    try:
        before = run(go())
        assert proc.poll() == 0, proc.poll()
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.communicate()
    after = parse_gui(gui.read_bytes())
    assert after["magic"] == 0x6705 and after["vx"] == 0 and after["vy"] == 0 and after["wz"] == 0
    assert after["mode_req"] == 0 and after["m5_preset"] == 0 and after["seq"] != before["seq"]


def test_run_script_has_no_gui_flag():
    sh = G1 / "tools" / "run_g1_with_gui.sh"
    assert subprocess.run(["bash", "-n", str(sh)]).returncode == 0
    text = sh.read_text(encoding="utf-8")
    assert "--no-gui" in text and "arcade" in text


def test_requirements_pin():
    req = (TOOLS / "arcade_relay_requirements.txt").read_text(encoding="utf-8").split()
    assert "websockets>=12,<14" in req and "msgpack>=1.0" in req


# ── 1회성 프레임 붙잡기(소비 확인) · 파싱 오류 · 연결 끊김 승계 ───────────────────────
def _write_status(path, applied, n=[0]):
    n[0] += 2
    vals = dict.fromkeys(status_shm.FIELDS, 0)
    vals.update(magic=status_shm.MAGIC, seq=n[0], gui_seq_applied=applied, z_fk=0.75)
    tmp = str(path) + ".tmp"
    with open(tmp, "wb") as f:
        f.write(struct.pack(status_shm.FMT, *[vals[k] for k in status_shm.FIELDS]))
    os.replace(tmp, str(path))


def test_hold_one_shot_with_slow_reader_sees_every_request_once(tmp_path):
    """g1_ctrl 흉내: 25 Hz 로 읽고(중계의 절반) 읽은 seq 를 gui_seq_applied 로 돌려준다."""
    sp = tmp_path / "status"

    async def go():
        seen = []
        async with started_relay(tmp_path, status_path=str(sp)) as (relay, frames):
            stop = False

            async def reader():
                last = None
                while not stop:
                    try:
                        f = parse_gui((tmp_path / "gui").read_bytes())
                    except (OSError, struct.error):
                        f = None
                    if f is not None and f["seq"] != last:
                        last = f["seq"]
                        if f["mode_req"] or f["m5_preset"]:
                            seen.append((f["mode_req"], f["m5_preset"]))
                        _write_status(sp, f["seq"])
                    await asyncio.sleep(0.04)
            rt = asyncio.ensure_future(reader())
            ws, _ = await connect(relay)
            for i in range(1, 11):
                await ws.send(cmd(i, mode_req=i % 6 + 1, m5_preset=(i % 3) + 1 if i % 2 else 0, m5_press_seq=i))
                await asyncio.sleep(0.15)
            stop = True
            await rt
            return seen, frames
    seen, frames = run(go())
    want = [(i % 6 + 1, (i % 3) + 1 if i % 2 else 0) for i in range(1, 11)]
    assert seen == want, seen
    sent = [f for _, f in frames if f["mode_req"] or f["m5_preset"]]
    assert len(sent) == 10                                   # 중계도 정확히 한 번씩만 실었다


def test_hold_released_after_100ms_without_consumption(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):   # 상태 칸 없음 = 소비 확인 불가
            ws, _ = await connect(relay)
            await asyncio.sleep(0.1)
            await ws.send(cmd(1, mode_req=3))
            await stream(ws, 0.5, vx=0.4)
            return frames
    frames = run(go())
    i = next(k for k, (_, f) in enumerate(frames) if f["mode_req"] == 3)
    gap = frames[i + 1][0] - frames[i][0]
    assert 0.08 <= gap <= 0.16, gap                          # 약 100 ms 뒤 풀림
    after = frames[i + 1:]
    assert after[0][1]["mode_req"] == 0 and abs(after[0][1]["vx"] - 0.4) < 1e-6   # 붙잡는 동안 온 속도도 반영
    gaps = [b[0] - a[0] for a, b in zip(after[:-1], after[1:-1])]   # 마지막 = 종료 0 프레임
    assert gaps and max(gaps) < 0.1                          # 하트비트 재개(20 ms 주기, 지터 여유)
    allg = [b[0] - a[0] for (a, b) in zip(frames, frames[1:])]
    assert max(allg) < 15 * 0.02, max(allg)                  # 붙잡는 동안 간격도 g1_ctrl 감시(15 틱) 안


def test_non_dict_and_pathological_json_dropped_connection_kept(tmp_path, caplog):
    caplog.set_level(logging.INFO, logger="arcade_relay")

    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            ws, _ = await connect(relay)
            deep = "[" * 20000 + "]" * 20000
            huge = ('{"v":1,"type":"cmd","seq":1,"mode_req":0,"m5_preset":0,"m5_press_seq":0,"vx":1%s,"vy":0,"wz":0}'
                    % ("0" * 400))
            for m in ("[1]", '"s"', "7", "null", deep, huge):
                await ws.send(m)
                await asyncio.sleep(0.02)
            await ws.send(json.dumps({"v": 1, "type": "ping", "id": 1, "t": 2.0}))
            while json.loads(await asyncio.wait_for(ws.recv(), 2))["type"] != "pong":
                pass
            await stream(ws, 0.2, vx=0.3)                    # 연결·조종권 모두 살아 있다
            assert abs(frames[-1][1]["vx"] - 0.3) < 1e-6
    run(go())
    assert any("bad frame" in r.getMessage() for r in caplog.records)


def test_new_connection_becomes_controller_after_controller_disconnect(tmp_path):
    async def go():
        async with started_relay(tmp_path) as (relay, frames):
            a, ha = await connect(relay)
            b, hb = await connect(relay)
            assert (ha["role"], hb["role"]) == ("controller", "viewer")
            await a.close()
            await asyncio.sleep(0.1)
            c, hc = await connect(relay)                    # 1 s 기다리지 않아도 승계
            assert hc["role"] == "controller"
            t0 = time.monotonic()
            await stream(b, 0.2, vx=1.0)                    # 기존 viewer 는 승격하지 않았다
            assert all(f["vx"] == 0 for t, f in frames if t >= t0)
    run(go())


def test_help_prints_no_gui_lines():
    out = subprocess.run(["bash", str(G1 / "tools" / "run_g1_with_gui.sh"), "--help"],
                         stdout=subprocess.PIPE, universal_newlines=True).stdout
    assert "--no-gui" in out and "arcade_relay" in out
