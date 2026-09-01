#!/usr/bin/env python3
"""sim2sim_headless.py — 사람 없이 sim2sim 을 돌리고 **밴드까지 푼다**.

# 왜 필요했나
MuJoCo 고무밴드는 기본 켜짐이고 **시뮬 «창» 의 키 `9`** 로만 꺼진다. 그래서 헤드리스로 찍은
로그는 전부 «로봇이 매달린» 상태였고(발목 |tau| 0.25 Nm vs 실기 5.19), 그걸 sim 기준선으로
쓰면 «sim 은 멀쩡한데 실기만» 이라는 틀린 결론이 나온다 (2026-09-01 실제로 그랬다).

해결: **Xvfb + XTEST 로 가짜 키 이벤트**를 창에 보낸다. `xdotool`(sudo 필요) 대신
`python-xlib` 를 격리 실행(`uv run --no-project --with`)으로 쓴다 — 시스템도 프로젝트
venv 도 안 건드린다.

실측: 키를 보낸 순간 발목 |tau| 합이 **0.5 → 15 Nm** 으로 점프한다(= 로봇이 제 무게를 짐).

    python3 deploy/scripts/sim2sim_headless.py --policy v1 --out /tmp/sim_v1.csv
    python3 deploy/scripts/sim2sim_headless.py --policy v1 --replay <실기 gait.csv>
    python3 deploy/scripts/sim2sim_headless.py --policy v1 --stand 60   # 정지만 60초

끝나면 `check_band_released.py` 로 **스스로 판정**한다 — 밴드가 안 풀렸으면 실패로 끝난다.
"""
from __future__ import annotations

import argparse
import os
import pty
import signal
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
REPO = os.path.dirname(REPO) if os.path.basename(REPO) == "deploy" else REPO
G1 = os.path.join(REPO, "deploy/robots/g1")
UV = os.path.expanduser("~/.local/bin/uv")

SENDKEY = r'''
import sys, time
from Xlib import display, X
from Xlib.ext import xtest
disp = display.Display(sys.argv[1]); root = disp.screen().root
def find(w=None):
    w = w or root
    for c in w.query_tree().children:
        try:
            if c.get_wm_name(): return c
        except Exception: pass
        r = find(c)
        if r: return r
    return None
win = None
for _ in range(60):
    win = find()
    if win: break
    time.sleep(0.5)
if not win:
    print("WINDOW_NOT_FOUND"); raise SystemExit(1)
print("win:", win.get_wm_name(), flush=True)
disp.set_input_focus(win, X.RevertToParent, X.CurrentTime); disp.sync()
for k in sys.argv[2:]:
    c = disp.keysym_to_keycode(int(k))
    xtest.fake_input(disp, X.KeyPress, c); disp.sync(); time.sleep(0.05)
    xtest.fake_input(disp, X.KeyRelease, c); disp.sync(); time.sleep(0.4)
print("KEYS_SENT", flush=True)
'''


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=isinstance(cmd, str), **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True, help="슬롯 이름 또는 ACTIVE.yaml 별칭")
    ap.add_argument("--out", default="/tmp/sim2sim.csv")
    ap.add_argument("--mode", default="1")
    ap.add_argument("--stand", type=float, default=35.0, help="밴드 해제 뒤 정지 유지 [s]")
    ap.add_argument("--replay", default=None, help="이 실기 gait CSV 의 명령을 재생")
    ap.add_argument("--display", default=":99")
    a = ap.parse_args()

    slot = a.policy
    act = os.path.join(G1, "config/policy/ACTIVE.yaml")
    if not os.path.isdir(os.path.join(G1, "config/policy/mimic_masked", slot)) and os.path.exists(act):
        for line in open(act, encoding="utf-8"):
            line = line.split("#", 1)[0].strip()
            if line.startswith(a.policy + ":"):
                slot = line.split(":", 1)[1].strip()
                print("[별칭] %s -> %s" % (a.policy, slot)); break

    print("[정리] 잔류 프로세스")
    for p in ("g1_ctrl", "unitree_mujoco"):
        sh(["pkill", "-x", p], stderr=subprocess.DEVNULL)
    sh(["pkill", "-f", "Xvfb %s" % a.display], stderr=subprocess.DEVNULL)
    time.sleep(2)
    sh(["rm", "-f", "/tmp/.X%s-lock" % a.display.lstrip(":")], stderr=subprocess.DEVNULL)

    xv = subprocess.Popen(["Xvfb", a.display, "-screen", "0", "1280x1024x24"],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    env = dict(os.environ, DISPLAY=a.display)
    sim = subprocess.Popen([os.path.join(REPO, "simulate/build/unitree_mujoco")],
                           env=env, stdout=open("/tmp/s2s_mj.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(8)
    if sim.poll() is not None:
        print("🔴 시뮬이 안 떴다 — /tmp/s2s_mj.log"); xv.kill(); return 1
    print("[sim] 기동")

    if os.path.exists(a.out):
        os.remove(a.out)
    cenv = dict(os.environ, G1_STATE_CSV=a.out, G1_POLICY_SLOT=slot)
    mfd, sfd = pty.openpty()
    ctl = subprocess.Popen([os.path.join(G1, "build/g1_ctrl"), "--network=lo"],
                           stdin=sfd, stdout=open("/tmp/s2s_ctl.log", "w"),
                           stderr=subprocess.STDOUT, env=cenv, close_fds=True)
    os.close(sfd)
    try:
        time.sleep(6); os.write(mfd, b"f"); print("[키] f (FixStand)")
        time.sleep(6); os.write(mfd, b"m"); print("[키] m (Mimic_Masked)")
        time.sleep(4); os.write(mfd, a.mode.encode()); print("[키] %s (mode)" % a.mode)
        time.sleep(3)

        print("[밴드] 8,8,9 전송 — 🔴 여기서부터가 데이터다")
        r = subprocess.run([UV, "run", "--no-project", "--with", "python-xlib",
                            "python", "-c", SENDKEY, a.display, "56", "56", "57"],
                           capture_output=True, text=True, timeout=120)
        sys.stdout.write("  " + r.stdout.replace("\n", "\n  "))
        if "KEYS_SENT" not in r.stdout:
            print("🔴 키 전송 실패 — 밴드가 안 풀렸다"); raise RuntimeError("band")

        if a.replay:
            print("[재생] %s" % os.path.basename(a.replay))
            subprocess.run([sys.executable, os.path.join(G1, "tools/replay_cmd.py"), a.replay])
        else:
            print("[정지] %.0f 초 유지" % a.stand)
            time.sleep(a.stand)
        os.write(mfd, b"p"); time.sleep(2)
    finally:
        ctl.send_signal(signal.SIGINT)
        try: ctl.wait(timeout=10)
        except Exception: ctl.kill()
        sim.terminate()
        try: sim.wait(timeout=8)
        except Exception: sim.kill()
        xv.terminate()
        for p in ("g1_ctrl", "unitree_mujoco"):
            sh(["pkill", "-x", p], stderr=subprocess.DEVNULL)

    print("\n[판정] 밴드가 정말 풀렸나 — 데이터로 확인한다")
    chk = subprocess.run([sys.executable, os.path.join(REPO, "deploy/scripts/check_band_released.py"), a.out])
    if chk.returncode != 0:
        print("🔴 밴드가 안 풀린 로그다. 판정에 쓰지 말 것."); return 1
    print("🟢 %s — 쓸 수 있는 sim2sim 로그" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
