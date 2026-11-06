#!/usr/bin/env python3
"""arcade_sim2sim_smoke.py — 아케이드 경로(G1_ARCADE=1) sim2sim 연기. com1 · unitree_mujoco · g1_ctrl --network=lo 만.

deploy/scripts/sim2sim_headless.py 와 같은 순서(Xvfb → unitree_mujoco → g1_ctrl → f → m → 밴드 8,8,9 + 발목 |tau| 확인)로 띄우고,
밴드가 풀린 시각을 t=0 으로 의도 칸(tools/arcade_intent_shm.py, 50 Hz)에 대본을 쓴다(기본: 서기 5 s → 걷기 3 s → Space(점프) →
걷기 3 s → 서기 3 s → p). 아케이드 상태 칸(tools/arcade_status_shm.py)을 틱마다 읽어 role 열 · tick_us · z_fk · 기울기를 남기고,
콘솔에서 넘어짐(Passive 전이 · bad_orientation) · LoopDiag overrun 을 센다. 측정만 한다 — 판정 기준은 요약 끝에 사실로 찍는다.

    python3 deploy/robots/g1/tests/arcade_sim2sim_smoke.py --bundle /home/piene/g1_arcade_2b/build/bundle_bt5 --out <폴더>
    (+ --strace <파일>: g1_ctrl 을 strace -f -tt 아래에서 — 정책 스레드 한 틱의 시스템 콜 세기용. 밴드 확인은 건너뛴다)

🔴 남의 프로세스는 안 죽인다: g1_ctrl · unitree_mujoco 가 떠 있거나 :97 이 쓰이면 거부만 한다(sim2sim_headless 의 검사 그대로).
   정리는 이 스크립트가 띄운 Popen(과 strace 아래의 그 자식 g1_ctrl, comm 확인)만. 끝나면 이 스크립트가 만든
   /dev/shm/g1_arcade_intent · g1_arcade_status 를 지운다(시작 때 이미 있으면 거부 — 남의 것일 수 있다).
python 3.8 문법(로봇에서도 돈다 — 단 이 연기는 com1 sim 전용).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import pty
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
G1 = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(G1)))
sys.path.insert(0, os.path.join(REPO, "deploy/scripts"))
sys.path.insert(0, os.path.join(G1, "tools"))
import sim2sim_headless as s2s          # noqa: E402  (읽기만 — 기동 거부 검사 · XTEST 키 · 콘솔 따라 읽기)
import arcade_intent_shm as ish         # noqa: E402
import arcade_status_shm as ssh         # noqa: E402

V7 = "261009_v7_f6_s1456_109k"
INTENT = ish.SHM_PATH
STATUS = ssh.SHM_PATH


def plan_axes(t, fwd):
    """대본(밴드 해제 뒤 t 초) → fwd 축. 서기 0-5 · 걷기 5-11(8 에 Space) · 서기 11-14."""
    return fwd if 5.0 <= t < 11.0 else 0.0


SPACE_AT = 8.0
END_AT = 14.0


def ankle_tau(csv_path, win_s=2.0):
    try:
        rows = open(csv_path, encoding="utf-8", errors="ignore").read().splitlines()
    except OSError:
        return None
    if len(rows) < 20:
        return None
    hdr = rows[0].split(",")
    try:
        i4, i10 = hdr.index("tau_est_4"), hdr.index("tau_est_10")
    except ValueError:
        return None
    vals = []
    for ln in rows[-int(win_s * 50):]:
        f = ln.split(",")
        if len(f) <= i10:
            continue
        try:
            vals.append((abs(float(f[i4])) + abs(float(f[i10]))) / 2)
        except ValueError:
            pass
    return sum(vals) / len(vals) if vals else None


def child_g1_ctrl(pid):
    """strace(pid) 의 자식 중 comm == g1_ctrl 인 PID(없으면 None)."""
    try:
        for tid in os.listdir("/proc/%d/task" % pid):
            for c in open("/proc/%d/task/%s/children" % (pid, tid)).read().split():
                if open("/proc/%s/comm" % c).read().strip() == "g1_ctrl":
                    return int(c)
    except OSError:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bundle", required=True, help="아케이드 묶음 폴더(g1_arcade build/bundle_bt5)")
    ap.add_argument("--out", required=True, help="결과 폴더(상태 칸 기록 · StateDump CSV · 콘솔 사본 · 요약)")
    ap.add_argument("--fwd", type=float, default=0.5, help="걷기 구간의 fwd 축 [-1,1] (봉투 walk fwd 1.0 m/s 기준)")
    ap.add_argument("--display", default=":97")
    ap.add_argument("--strace", default=None, help="g1_ctrl 을 strace -f -tt -o <파일> 아래에서(밴드 확인 생략)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.display == ":99":
        print("🔴 :99 는 예약 브라우저 전용 — 다른 번호")
        return 1
    err = s2s.check_no_other_sim() or s2s.check_display_free(a.display)
    if err:
        print("🔴 " + err)
        return 1
    for p in [INTENT, STATUS, INTENT + ".tmp"] + glob.glob(INTENT + ".*.tmp"):   # 임시 파일 = <칸>.<pid>.tmp(쓰는 쪽마다)
        if os.path.exists(p):
            print("🔴 %s 가 이미 있다 — 남의 것일 수 있어 거부(지우지 않는다)" % p)
            return 1

    csv_path = os.path.join(a.out, "state.csv")
    ctl_log = os.path.join(a.out, "ctl.log")
    for p in (csv_path, ctl_log):
        if os.path.exists(p):
            os.remove(p)
    xv = subprocess.Popen(["Xvfb", a.display, "-screen", "0", "1280x1024x24"],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    sim = subprocess.Popen([os.path.join(REPO, "simulate/build/unitree_mujoco")], env=dict(os.environ, DISPLAY=a.display),
                           stdout=open(os.path.join(a.out, "mj.log"), "w"), stderr=subprocess.STDOUT)
    ctl = None
    ctl_pid = None
    stop = threading.Event()
    t0_box = [None]
    rows = []                     # [(mono, dict)]
    sent = []                     # [(t, what)]
    threads = []
    try:
        time.sleep(8)
        if sim.poll() is not None:
            print("🔴 시뮬이 안 떴다")
            return 1
        cenv = dict(os.environ, G1_ARCADE="1", G1_ARCADE_BUNDLE=a.bundle, G1_POLICY_SLOT=V7, G1_STATE_CSV=csv_path)
        cmd = [os.path.join(G1, "build/g1_ctrl"), "--network=lo"]
        if a.strace:
            cmd = ["strace", "-f", "-tt", "-o", a.strace] + cmd
        mfd, sfd = pty.openpty()
        ctl = subprocess.Popen(cmd, stdin=sfd, stdout=open(ctl_log, "w"), stderr=subprocess.STDOUT, env=cenv, close_fds=True)
        os.close(sfd)
        tail = s2s.LogTail(ctl_log)

        def writer():             # 의도 칸 50 Hz — t0 전은 서기(중립). Space 는 한 번(끈적한 이벤트는 쓰는 쪽 모듈이 싣는다)
            jumped = False
            nxt = time.monotonic()
            while not stop.is_set():
                t0 = t0_box[0]
                t = (time.monotonic() - t0) if t0 is not None else -1.0
                ev = 0
                if t0 is not None and not jumped and t >= SPACE_AT:
                    ev, jumped = ish.EV_JUMP, True
                    sent.append((t, "space"))
                ish.write_intent(plan_axes(t, a.fwd) if t >= 0 else 0.0, 0.0, 0.0, 0, ev, 0)
                nxt += 0.02
                time.sleep(max(0.0, nxt - time.monotonic()))

        def monitor():            # 아케이드 상태 칸 — 틱이 바뀐 줄만
            last = None
            while not stop.is_set():
                r = ssh.read()
                if r is not None and r["tick"] != last:
                    last = r["tick"]
                    rows.append((time.monotonic(), r))
                time.sleep(0.004)

        for f in (writer, monitor):
            th = threading.Thread(target=f, daemon=True)
            th.start()
            threads.append(th)
        if a.strace:
            for _ in range(100):
                ctl_pid = child_g1_ctrl(ctl.pid)
                if ctl_pid:
                    break
                time.sleep(0.1)
        else:
            ctl_pid = ctl.pid
        time.sleep(10 if a.strace else 6)
        os.write(mfd, b"f")
        print("[키] f (FixStand)")
        time.sleep(6)
        os.write(mfd, b"m")
        print("[키] m (Mimic_Masked — 아케이드 경로)")
        time.sleep(4)
        released = False
        if a.strace:
            print("[밴드] strace 판 — 밴드 확인을 건너뛴다(시스템 콜 세기만)")
        else:
            for attempt in range(1, 4):
                keys = ["56", "56", "57"] if attempt == 1 else ["57"]
                r = subprocess.run([s2s.UV, "run", "--no-project", "--with", "python-xlib", "python", "-c", s2s.SENDKEY,
                                    a.display] + keys, capture_output=True, text=True, timeout=120)
                if "KEYS_SENT" not in r.stdout:
                    print("  ⚠ 키 전송 실패: %s" % r.stdout.strip().replace("\n", " "))
                    continue
                time.sleep(3.0)
                tau = ankle_tau(csv_path)
                print("[밴드] 시도 %d — 발목 |tau| %s Nm" % (attempt, "%.2f" % tau if tau is not None else "?"))
                if tau is not None and tau >= 1.0:
                    released = True
                    break
            if not released:
                print("🔴 밴드를 못 풀었다")
                return 1
        tail.poll()
        i0 = len(tail.lines)
        pre_fail = s2s.pre_t0_failure([s for _, s in tail.lines])
        if pre_fail is not None:
            print("🔴 t=0 에 Mimic_Masked 가 아니다: %s" % pre_fail)
            return 1
        t0_box[0] = time.monotonic()
        print("[대본] t=0 — 서기 0-5 s · 걷기(fwd %.2f) 5-11 s · Space %.1f s · 서기 11-14 s" % (a.fwd, SPACE_AT))
        while time.monotonic() - t0_box[0] < END_AT:
            tail.poll()
            if ctl.poll() is not None:
                print("🔴 g1_ctrl 이 대본 도중 끝났다 rc=%s" % ctl.returncode)
                break
            time.sleep(0.05)
        t_p = time.monotonic() - t0_box[0]
        os.write(mfd, b"p")
        sent.append((t_p, "p"))
        for _ in range(100):
            tail.poll()
            time.sleep(0.02)
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=2)
        if ctl is not None:
            if a.strace and ctl_pid and child_g1_ctrl(ctl.pid) == ctl_pid:
                os.kill(ctl_pid, signal.SIGINT)        # strace 아래의 그 자식(comm 확인) — strace 는 자식이 끝나면 따라 끝난다
            else:
                ctl.send_signal(signal.SIGINT)
            try:
                ctl.wait(timeout=15)
            except Exception:
                ctl.kill()
        sim.terminate()
        try:
            sim.wait(timeout=8)
        except Exception:
            sim.kill()
        xv.terminate()
        try:
            xv.wait(timeout=8)
        except Exception:
            xv.kill()
        for p in (INTENT, STATUS, ish.tmp_path(INTENT)):   # 이 스크립트(와 그 g1_ctrl)가 만든 것 — 시작 때 없음을 확인했다
            if os.path.exists(p):
                os.remove(p)

    tail.close()
    t0 = t0_box[0]
    lines = [(m - t0 if t0 is not None else None, s2s.SPDLOG_TS.sub("", s.strip())) for m, s in tail.lines]
    with open(os.path.join(a.out, "ctl_t.log"), "w", encoding="utf-8") as f:
        for t, s in lines:
            f.write("%s | %s\n" % ("   pre" if t is None else "%7.2f" % t, s))
    roles = []
    for _, s in lines:
        m = re.search(r"\[arcade\] bundle=.* roles \[([^\]]*)\]", s)
        if m:
            roles = m.group(1).split(",")
    name = (lambda i: roles[i] if 0 <= i < len(roles) else str(i))
    post = [(m - t0, r) for m, r in rows if t0 is not None and m >= t0 and (m - t0) <= t_p]
    with open(os.path.join(a.out, "status.jsonl"), "w", encoding="utf-8") as f:
        for m, r in rows:
            d = dict(r)
            d["t"] = round(m - t0, 4) if t0 is not None else None
            d["role_name"] = name(r["role"])
            f.write(json.dumps(d) + "\n")
    seq = []
    for t, r in post:
        nm = name(r["role"])
        if not seq or seq[-1][1] != nm:
            seq.append((t, nm))
    us = sorted(r["tick_us"] for _, r in post)
    ticks = [r["tick"] for _, r in post]
    missed = (ticks[-1] - ticks[0] + 1 - len(set(ticks))) if ticks else 0
    passive = [(t, s) for t, s in lines if t is not None and t < t_p and "FSM: Change state from Mimic_Masked" in s]
    trips = [(t, s) for t, s in lines if t is not None and re.search(r"\[safety\].*(TRIPPED|LATCHED|래치 중)", s)]
    diag = [(t, s) for t, s in lines if "[diag:policy]" in s]
    over = []
    for t, s in diag:
        m = re.search(r"overrun=(\d+)/(\d+)", s)
        if m:
            over.append((t, int(m.group(1)), int(m.group(2))))
    zmin = min((r["z_fk"] for _, r in post), default=float("nan"))
    tmax = max((r["tilt_deg"] for _, r in post), default=float("nan"))
    stale = sum(1 for _, r in post if r["intent_stale"])
    summ = {
        "role_sequence": [[round(t, 2), n] for t, n in seq],
        "walk_jump_walk": [n for _, n in seq][:3] == ["walk", "jump", "walk"] or
                          any([n for _, n in seq][i:i + 3] == ["walk", "jump", "walk"] for i in range(len(seq))),
        "space_sent_t": [round(t, 2) for t, w in sent if w == "space"],
        "status_rows": len(post), "missed_ticks_in_monitor": missed,
        "tick_us_median": statistics.median(us) if us else None, "tick_us_p99": us[int(len(us) * 0.99)] if us else None,
        "tick_us_max": us[-1] if us else None,
        "z_fk_min": zmin, "tilt_deg_max": tmax, "intent_stale_rows": stale,
        "fall": bool(passive) or zmin < 0.25 or tmax >= 100.0,
        "passive_before_p": [s for _, s in passive], "safety_events": [s for _, s in trips],
        "diag_windows": len(over), "overrun_total": sum(o for _, o, _ in over),
        "overrun_ticks_total": sum(n for _, _, n in over),
        "band_released": not a.strace,
    }
    with open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summ, f, ensure_ascii=False, indent=1)
    print("\n══════ 아케이드 연기 요약 (t = 밴드 해제 뒤 초) ══════")
    print("role 열       : %s" % " → ".join("%s(%.2f)" % (n, t) for t, n in seq))
    print("walk→jump→walk: %s   (Space 보냄 t=%s)" % (summ["walk_jump_walk"], summ["space_sent_t"]))
    print("상태 칸 줄    : %d (모니터가 놓친 틱 %d)" % (len(post), missed))
    if us:
        print("tick_us       : 중앙 %d · p99 %d · 최대 %d µs" % (summ["tick_us_median"], summ["tick_us_p99"], summ["tick_us_max"]))
    print("z_fk 최소 %.3f m · 기울기 최대 %.1f° · 의도 끊김 줄 %d" % (zmin, tmax, stale))
    print("넘어짐        : %s %s" % (summ["fall"], summ["passive_before_p"]))
    print("안전 사건     : %s" % (summ["safety_events"] or "(없음)"))
    print("LoopDiag      : 창 %d · overrun %d / %d 틱" % (len(over), summ["overrun_total"], summ["overrun_ticks_total"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
