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
# 🔴 2026-09-22 사고: 이 스크립트가 시작할 때마다 부킹 에이전트 전용 Xvfb(:99,
# xvfb99.service, "예약 브라우저 전용")를 이름으로 pkill 하고 그 자리에 자기 Xvfb 를
# 띄웠다 — 하루 433회 systemd 재시작 + 예약 브라우저 점검 실패. g1_ctrl/unitree_mujoco
# 도 이름으로 pkill 해 다른 사람의 sim 까지 죽일 수 있었다. 처방: 기본 디스플레이를
# :97 로 옮기고 :99 는 아예 거부, 시작 시 "이미 떠 있으면 죽이지 말고 거부"로 바꾸고,
# 종료 시에는 이 스크립트가 **직접 띄운 Popen 객체만** 정리한다 — 이름 기반 pkill 금지.
from __future__ import annotations

import argparse
import os
import pty
import re
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


def _pgrep_af(pattern):
    """pgrep -af <정규식>: 매치 줄('pid cmdline') 리스트. 없으면 빈 리스트."""
    r = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def _pgrep_ax(name):
    """pgrep -a -x <name>: comm 이 정확히 일치하는 프로세스만."""
    r = subprocess.run(["pgrep", "-a", "-x", name], capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def _pid_alive(pid):
    return os.path.exists("/proc/%d" % pid)


def check_display_free(display):
    """이 display 를 이미 쓰는 Xvfb(또는 살아있는 락)가 있는지 검사만 한다 — 절대 안 죽인다.

    반환: 사용 가능하면 None. 아니면 사람이 읽을 에러 메시지(str) — 호출부가 찍고 종료한다.
    죽은 프로세스의 stale lock 파일은 여기서 지운다(로그 남김); 살아있는 서버의 락은 건드리지 않는다.
    """
    hits = _pgrep_af(r"Xvfb %s( |$)" % re.escape(display))
    if hits:
        lines = "\n".join("   " + h for h in hits)
        return "디스플레이 %s 는 이미 다른 Xvfb 가 쓰는 중 — 죽이지 않는다:\n%s" % (display, lines)

    lock_path = "/tmp/.X%s-lock" % display.lstrip(":")
    if os.path.exists(lock_path):
        try:
            pid = int(open(lock_path, encoding="utf-8").read().strip())
        except (ValueError, OSError):
            pid = None
        if pid is not None and _pid_alive(pid):
            return "%s 락이 살아있는 pid %d 를 가리킨다 — 그 프로세스가 쓰는 중, 안 건드린다." % (lock_path, pid)
        print("[정리] %s 는 죽은 pid(%s) 의 잔류 락 — 삭제" % (lock_path, pid))
        try:
            os.remove(lock_path)
        except OSError:
            pass
    return None


def check_no_other_sim():
    """g1_ctrl / unitree_mujoco 가 이미 돌고 있으면 에러 메시지, 없으면 None. 절대 안 죽인다."""
    for name in ("g1_ctrl", "unitree_mujoco"):
        hits = _pgrep_ax(name)
        if hits:
            lines = "\n".join("   " + h for h in hits)
            return "%s 가 이미 돌고 있다 — 다른 sim 이 돌고 있다, 끝난 뒤 다시:\n%s" % (name, lines)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", required=True, help="슬롯 이름 또는 ACTIVE.yaml 별칭")
    ap.add_argument("--out", default="/tmp/sim2sim.csv")
    ap.add_argument("--mode", default="1")
    ap.add_argument("--stand", type=float, default=35.0, help="밴드 해제 뒤 정지 유지 [s]")
    ap.add_argument("--replay", default=None, help="이 실기 gait CSV 의 명령을 재생")
    ap.add_argument("--display", default=":97",
                    help="Xvfb 디스플레이. :99 는 예약 브라우저 전용(xvfb99.service) — 사용 금지")
    ap.add_argument("--floor-friction", type=float, default=None,
                    help="바닥 마찰 μ 를 이 값으로 바꿔 돌린다(끝나면 원복). 학습 DR 은 (0.3,1.6).")
    ap.add_argument("--scene", default=None,
                    help="unitree_mujoco -s 로 줄 장면 (예: src/assets/robots/unitree_g1/xmls/scene_g1_prim.xml). "
                         "생략 = simulate/config.yaml")
    a = ap.parse_args()

    if a.display == ":99":
        print("🔴 :99 는 예약 브라우저 전용(xvfb99.service) — 다른 번호를 쓸 것")
        return 1
    if a.scene is not None and a.floor_friction is not None:
        # --floor-friction 은 scene_g1.xml 을 고쳐 쓰고 되돌린다 — 다른 장면을 돌리면서 그 파일을
        # 건드리면 엉뚱한 파일을 고치고 되돌리게 된다. (scene_g1_prim.xml 은 로봇 geom 이 priority 1 이라
        # 바닥 μ 를 바꿔도 접촉 μ 가 안 바뀐다.)
        print("🔴 --scene 과 --floor-friction 은 같이 못 쓴다")
        return 1
    if a.scene is not None and not os.path.isfile(a.scene):
        print("🔴 장면 파일이 없다: %s" % a.scene)
        return 1

    print("[확인] 남의 프로세스는 안 죽인다 — 떠 있으면 거부만 한다")
    err = check_no_other_sim()
    if err:
        print("🔴 " + err)
        return 1
    err = check_display_free(a.display)
    if err:
        print("🔴 " + err)
        return 1

    slot = a.policy
    act = os.path.join(G1, "config/policy/ACTIVE.yaml")
    if not os.path.isdir(os.path.join(G1, "config/policy/mimic_masked", slot)) and os.path.exists(act):
        for line in open(act, encoding="utf-8"):
            line = line.split("#", 1)[0].strip()
            if line.startswith(a.policy + ":"):
                slot = line.split(":", 1)[1].strip()
                print("[별칭] %s -> %s" % (a.policy, slot)); break

    # 🔴 바닥 마찰 스윕. tracked XML 을 손으로 고치지 않게 여기서 «바꿨다 되돌린다».
    #    sim 의 발은 코너 4점 구이고 바닥과 priority 가 같아 «최대값» 이 쓰이므로 바닥 한 줄이면 된다.
    #    (실측: μ 1.0 -> 정지 중 골반 pitch 폭 0.1° / 1.6 -> 13.5° / 3.0 -> 31.3°.
    #     실기 11.7° 는 μ≈1.5~1.6 자리 = 학습 DR 상한.)
    scene = os.path.join(REPO, "src/assets/robots/unitree_g1/xmls/scene_g1.xml")
    scene_backup = None
    # 🔴 장면을 고쳐 쓴 뒤의 «모든» 출구(시뮬 기동 실패 조기 return · 예외 · Ctrl-C)에서 원복한다.
    #    예전엔 원복이 안쪽 finally 에만 있어 «시뮬이 안 떴다» 경로가 고친 scene_g1.xml 을 남겼다.
    #    쓰기 자체도 try 안 — 쓰다 죽어도 원본으로 되돌린다.
    try:
        if a.floor_friction is not None:
            import re as _re
            src = open(scene, encoding="utf-8").read()
            new_geom = ('<geom name="floor" size="0 0 0.05" type="plane" material="groundplane" '
                        'friction="%g 0.005 0.0001"/>' % a.floor_friction)
            src2 = _re.sub(r'<geom name="floor"[^/]*/>', new_geom, src, count=1)
            if src2 == src:
                print("🔴 바닥 geom 을 못 찾았다 — 마찰을 못 바꾼다"); return 1
            scene_backup = src
            open(scene, "w", encoding="utf-8").write(src2)
            print("[바닥] μ = %g (끝나면 원복한다)" % a.floor_friction)

        xv = subprocess.Popen(["Xvfb", a.display, "-screen", "0", "1280x1024x24"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        env = dict(os.environ, DISPLAY=a.display)
        sim_cmd = ([os.path.join(REPO, "simulate/build/unitree_mujoco")]
                   + (["-s", os.path.abspath(a.scene)] if a.scene else []))
        if a.scene:
            print("[장면] %s" % os.path.abspath(a.scene))
        sim = subprocess.Popen(sim_cmd,
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

            # 🔴 키 `9` 는 **토글**이다 — 안 먹었다고 무작정 다시 보내면 도로 켜진다.
            #    그래서 «보내고 → 확인하고 → 안 됐으면 한 번 더» 로 간다. 확인은 계측 파일의
            #    발목 토크로 한다(쓰기 스레드가 100 ms 마다 비우므로 실행 중에 읽을 수 있다).
            #    실측: 3판 중 2판에서 첫 키가 유실됐다(창 포커스/이벤트 루프 경합).
            def ankle_tau(win_s=2.0):
                try:
                    rows = open(a.out, encoding="utf-8", errors="ignore").read().splitlines()
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
                    if len(f) <= i10: continue
                    try: vals.append((abs(float(f[i4])) + abs(float(f[i10]))) / 2)
                    except ValueError: pass
                return sum(vals) / len(vals) if vals else None

            released = False
            for attempt in range(1, 4):
                keys = ["56", "56", "57"] if attempt == 1 else ["57"]
                print("[밴드] %s 전송 (시도 %d)" % (",".join("8" if k == "56" else "9" for k in keys), attempt))
                r = subprocess.run([UV, "run", "--no-project", "--with", "python-xlib",
                                    "python", "-c", SENDKEY, a.display, *keys],
                                   capture_output=True, text=True, timeout=120)
                if "KEYS_SENT" not in r.stdout:
                    print("  ⚠ 키 전송 자체가 실패: %s" % r.stdout.strip().replace("\n", " ")); continue
                time.sleep(3.0)
                tau = ankle_tau()
                print("  확인: 발목 |tau| %s Nm" % ("%.2f" % tau if tau is not None else "(아직 표본 없음)"))
                if tau is not None and tau >= 1.0:
                    released = True; print("  🟢 밴드 풀림 — 여기서부터가 데이터다"); break
            if not released:
                print("🔴 밴드를 못 풀었다 (3회 시도). 이 로그는 판정에 못 쓴다."); raise RuntimeError("band")

            if a.replay:
                print("[재생] %s" % os.path.basename(a.replay))
                subprocess.run([sys.executable, os.path.join(G1, "tools/replay_cmd.py"), a.replay])
            else:
                print("[정지] %.0f 초 유지" % a.stand)
                time.sleep(a.stand)
            os.write(mfd, b"p"); time.sleep(2)
        finally:
            # 🔴 여기서 정리하는 건 이 스크립트가 직접 띄운 Popen(ctl/sim/xv)뿐이다.
            #    이름 기반 pkill 은 남의 프로세스(부킹 에이전트 Xvfb 등)를 같이 죽일 수 있어 금지.
            ctl.send_signal(signal.SIGINT)
            try: ctl.wait(timeout=10)
            except Exception: ctl.kill()
            sim.terminate()
            try: sim.wait(timeout=8)
            except Exception: sim.kill()
            xv.terminate()
            try: xv.wait(timeout=8)
            except Exception: xv.kill()
    finally:
        if scene_backup is not None:                 # 🔴 무슨 일이 있어도 원복
            open(scene, "w", encoding="utf-8").write(scene_backup)
            print("[바닥] 씬 원복")

    print("\n[판정] 밴드가 정말 풀렸나 — 데이터로 확인한다")
    chk = subprocess.run([sys.executable, os.path.join(REPO, "deploy/scripts/check_band_released.py"), a.out])
    if chk.returncode != 0:
        print("🔴 밴드가 안 풀린 로그다. 판정에 쓰지 말 것."); return 1
    print("🟢 %s — 쓸 수 있는 sim2sim 로그" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
