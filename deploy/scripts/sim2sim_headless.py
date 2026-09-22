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
    python3 deploy/scripts/sim2sim_headless.py --policy <slot> --script deploy/scripts/s2s_scripts/b2_mode5_loop.txt --stand 110

`--script <파일>`: 한 줄에 `<밴드 해제 뒤 초> <키>` (예 `12 x`, 공백 키는 `space`, `#` 뒤는 주석).
밴드가 풀린 시각이 t=0 이고, 줄마다 그 시각에 g1_ctrl 표준입력으로 키 한 글자를 보낸다.
끝나면 자세별 도착 시각 · 안전 이벤트 · 구간별 |qd| 최댓값을 표로 찍는다(측정 — 판정은 사람이).

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


# ── --script: 시각별 키 ──────────────────────────────────────────────────────────
# 🔴 키 사이 최소 간격. g1_ctrl 의 키보드 스레드는 80 ms 동안 입력이 없어야 키를 "" 로 되돌리고
#    (deploy/include/isaaclab/devices/keyboard/keyboard.h), 정책 스레드는 20 ms 마다 «바뀐 키» 만
#    먹는다. 두 키가 한 틱 안에 오면 앞 키가 사라지고, 같은 키 둘이 80 ms 안에 오면 한 번으로 읽힌다.
#    조용히 밀어 주지 않고 거부한다 — 스크립트의 시각이 곧 기록이다.
SCRIPT_MIN_GAP_S = 0.25
ARRIVE_WINDOW_S = 10.0      # 누름 뒤 이 안에 «도착» 이 없으면 미도착 (계획 Task 11)
REPLY_WINDOW_S = 1.0        # 키 뒤 이 안의 제어기 응답 줄을 그 키의 답으로 본다
# CSV 의 dq_i 는 모터 순서다. 슬롯 deploy.yaml 의 joint_ids_map 이 항등이면 정책 순서와 같다
# (State_Mimic.cpp G1_JOINT_NAME 과 같은 이름을 쓴다 — 로그의 관절 이름과 맞춰 읽게).
JOINT_NAME = ("L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
              "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
              "waist_yaw", "waist_roll", "waist_pitch",
              "L_sho_pitch", "L_sho_roll", "L_sho_yaw", "L_elbow", "L_wri_roll", "L_wri_pitch", "L_wri_yaw",
              "R_sho_pitch", "R_sho_roll", "R_sho_yaw", "R_elbow", "R_wri_roll", "R_wri_pitch", "R_wri_yaw")
ANSI = re.compile(r"\x1b\[[0-9;]*m")
SPDLOG_TS = re.compile(r"^\[\d{4}-\d\d-\d\d [\d:.]+\] ")   # 요약에선 날짜만 뗀다(수준 [info]/[error] 는 남김)


def parse_script(path):
    """`<초> <키>` 줄 목록 → [(t, key, 줄번호)]. 틀린 줄은 ValueError(파일:줄 포함) — 기동 전에 부른다."""
    steps = []
    with open(path, encoding="utf-8") as f:
        for n, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 2:
                raise ValueError("%s:%d: «<초> <키>» 두 칸이어야 한다: %r" % (path, n, raw.rstrip()))
            try:
                t = float(parts[0])
            except ValueError:
                raise ValueError("%s:%d: 시각이 숫자가 아니다: %r" % (path, n, parts[0]))
            key = " " if parts[1] == "space" else parts[1]
            if len(key) != 1:
                raise ValueError("%s:%d: 키는 한 글자(공백은 space): %r" % (path, n, parts[1]))
            if t < 0:
                raise ValueError("%s:%d: 시각이 음수다: %g" % (path, n, t))
            if steps and t - steps[-1][0] < SCRIPT_MIN_GAP_S:
                raise ValueError("%s:%d: 앞 줄(%g s)과 %.2f s — %.2f s 이상 띄울 것 (한 틱에 두 키가 오면 앞 키가 사라진다)"
                                 % (path, n, steps[-1][0], t - steps[-1][0], SCRIPT_MIN_GAP_S))
            steps.append((t, key, n))
    if not steps:
        raise ValueError("%s: 키 줄이 하나도 없다" % path)
    return steps


class LogTail:
    """g1_ctrl 콘솔 파일을 따라 읽으며 줄마다 «읽은 시각»(time.monotonic) 을 붙인다.

    printf 줄([m5] → · [cmd_mode] · [clip])에는 시각이 없어서, «누름 → 도착» 을 재려면 이렇게
    붙일 수밖에 없다. 해상도 = poll() 간격(스크립트 루프 20 ms). 파일은 g1_ctrl 이 그대로 쓴다
    (이 클래스는 읽기만 한다 — 기존 /tmp/s2s_ctl.log 는 안 바뀐다)."""

    def __init__(self, path):
        self.f = open(path, "rb")
        self.buf = b""
        self.lines = []            # [(mono, text)]

    def poll(self):
        chunk = self.f.read()
        if not chunk:
            return
        now = time.monotonic()
        self.buf += chunk
        *done, self.buf = self.buf.split(b"\n")
        for b in done:
            s = ANSI.sub("", b.decode("utf-8", "replace")).replace("\r", "").rstrip()
            if s.strip():
                self.lines.append((now, s))

    def close(self):
        self.poll()
        if self.buf.strip():
            self.lines.append((time.monotonic(), self.buf.decode("utf-8", "replace").replace("\r", "").rstrip()))
            self.buf = b""
        self.f.close()


def _read_state_csv(path):
    """StateDump CSV → [(wall_time, dq[29], tilt_deg, cmd_mode)]. 파일·열이 없으면 빈 리스트."""
    import csv
    import math
    try:
        f = open(path, encoding="utf-8", errors="ignore")
    except OSError:
        return []
    rows = []
    with f:
        rd = csv.reader(f)
        try:
            hdr = next(rd)
        except StopIteration:
            return []
        try:
            iw = hdr.index("wall_time")
            idq = [hdr.index("dq_%d" % i) for i in range(29)]
            ipz = hdr.index("pg_z")
            icm = hdr.index("cmd_mode")
        except ValueError:
            return []
        for r in rd:
            if len(r) != len(hdr):
                continue           # 프로세스가 끊긴 반쪽 줄
            try:
                w = float(r[iw])
                dq = [float(r[i]) for i in idq]
                pz = float(r[ipz])
                cm = int(r[icm])
            except ValueError:
                continue
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -pz))))
            rows.append((w, dq, tilt, cm))
    return rows


def _fmt_t(t):
    return "   pre" if t is None else "%6.2f" % t


def summarize(steps, sent, tail, i0, t0, csv_path, stand):
    """스크립트 실행 결과 요약(stdout). 측정만 한다 — 좋다/나쁘다는 사람이 판정한다.

    steps = parse_script 결과, sent = [(t_보낸, key)], tail = LogTail, i0 = t0 시점까지 읽은 줄 수,
    t0 = 밴드 해제 확인 시각(monotonic, CSV wall_time 과 같은 시계 — StateDump 는 steady_clock)."""
    L = [(None if i < i0 else m - t0, SPDLOG_TS.sub("", s.strip())) for i, (m, s) in enumerate(tail.lines)]
    post = [(t, s) for t, s in L if t is not None]
    print("\n══════ 스크립트 요약 (t = 밴드 해제 뒤 초, 로그 시각 해상도 ≈ 20 ms) ══════")

    # 1) 키마다 제어기의 답
    print("\n[키 → 제어기 응답] (키 뒤 %.0f s 안 — 다음 키 전까지 — 의 [cmd_mode]·[clip]·[m5]·[diag:switch]·[safety] 줄)"
          % REPLY_WINDOW_S)
    reply_pat = re.compile(r"\[(cmd_mode|clip|m5|diag:switch|safety)\]|FSM: Change state")
    for i, ((ts, key), (tp, _, n)) in enumerate(zip(sent, steps)):
        t_end = min(ts + REPLY_WINDOW_S, sent[i + 1][0] if i + 1 < len(sent) else float("inf"))
        replies = [s.strip() for t, s in post if ts - 0.02 <= t < t_end and reply_pat.search(s)]
        k = "space" if key == " " else key
        print("  t=%6.2f (예정 %6.2f, 줄 %d) 키 %-5s → %s" % (ts, tp, n, k, " | ".join(replies) if replies else "(응답 줄 없음)"))

    # 2) mode5 자세: 누름(진입 포함) → 첫 도착
    press_re = re.compile(r"\[m5\] (?:(진입|gui) )?→ «([^»]+)»")
    arrive_re = re.compile(r"\[m5\] 도착 «([^»]+)»\s+z_fk=([-\d.]+)\s+기울기=([-\d.]+)°")
    leave_re = re.compile(r"\[cmd_mode\] -> (\d+) \(|FSM: Change state from Mimic_Masked")
    presses = []                                       # [t, name, via, arrive(t,z,tilt)|None, end_t]
    for t, s in post:
        m = press_re.search(s)
        if m:
            if presses and presses[-1][4] is None:
                presses[-1][4] = t
            presses.append([t, m.group(2), m.group(1) or "키", None, None])
            continue
        m = arrive_re.search(s)
        if m and presses and presses[-1][3] is None and presses[-1][4] is None and m.group(1) == presses[-1][1]:
            presses[-1][3] = (t, float(m.group(2)), float(m.group(3)))
            continue
        m = leave_re.search(s)
        if m and presses and presses[-1][4] is None and (m.group(1) is None or m.group(1) != "5"):
            presses[-1][4] = t
    if presses:
        print("\n[mode5 자세] 누름 → 첫 «도착» (%.0f s 안이 아니면 미도착)" % ARRIVE_WINDOW_S)
        print("  %-7s %-14s %-5s %-24s %-6s %s" % ("누름 t", "자세", "경로", "도착", "z_fk", "기울기"))
        for t, name, via, arr, end in presses:
            if arr is None:
                span = "다음 누름/이탈까지 %.1f s" % (end - t) if end is not None else "끝까지"
                res, z, tl = "미도착 (%s)" % span, "-", "-"
            else:
                dt = arr[0] - t
                res = ("+%.2f s" % dt) if dt <= ARRIVE_WINDOW_S else ("미도착 (%.0f s 안) — +%.2f s 에 도착" % (ARRIVE_WINDOW_S, dt))
                z, tl = "%.2f" % arr[1], "%.0f°" % arr[2]
            print("  %7.2f %-14s %-5s %-24s %-6s %s" % (t, name, via, res, z, tl))

    # 3) 모드 요청·전환·클립
    print("\n[모드·클립] 수락/거부/전환")
    mode_pat = re.compile(r"\[cmd_mode\]|\[diag:switch\]|\[clip\]|FSM: Change state")
    for t, s in post:
        if mode_pat.search(s):
            print("  %s  %s" % (_fmt_t(t), s.strip()))

    # 4) 안전 이벤트 (+ CSV 의 그 순간 상태)
    rows = _read_state_csv(csv_path)
    rel = [(w - t0, dq, tilt, cm) for w, dq, tilt, cm in rows]

    def ctx(t):
        near = [r for r in rel if abs(r[0] - t) <= 0.2]
        if not near:
            return "(그 시각 CSV 행 없음)"
        tmax = max(r[2] for r in near)
        qm = max(((abs(v), j) for r in near for j, v in enumerate(r[1])), key=lambda x: x[0])
        return "CSV ±0.2 s: 기울기(원시) ≤ %.0f° · |qd| ≤ %.2f @ %s · 모드 %s" % (
            tmax, qm[0], JOINT_NAME[qm[1]], "/".join(sorted({str(r[3]) for r in near})))

    print("\n[안전 이벤트] Passive · bad_orientation · qd_warn · qd_crit")
    safety_pat = re.compile(r"\[safety\].*(TRIPPED|LATCHED|래치 중|Passive)")   # FSM 전환 줄은 위 [모드·클립] 에
    ev = [(t, s) for t, s in L if safety_pat.search(s)]
    if not ev:
        print("  (없음)")
    for t, s in ev:
        print("  %s  %s" % (_fmt_t(t), s.strip()))
        if t is not None:
            print("          %s" % ctx(t))

    # 5) 구간별 |qd| 최댓값 (CSV dq_*, 50 Hz)
    edges = [0.0] + [ts for ts, _ in sent] + [stand]
    labels = ["(해제 뒤)"] + ["키 %s" % ("space" if k == " " else k) for _, k in sent]
    print("\n[구간별 |qd| 최댓값] (CSV dq_* = FSM 20틱마다 한 줄 — sim 에선 FSM 이 ~870 Hz 라 ≈43 Hz 표본. "
          "정책 스레드 50 Hz 의 최댓값은 아래 체류 요약 · 기울기 = pg 로 구한 원시값)")
    print("  %-17s %-10s %5s %-24s %-9s %s" % ("구간 [s]", "시작", "행", "|qd| max @ 관절 (t)", "기울기max", "모드"))
    for a, b, lab in zip(edges[:-1], edges[1:], labels):
        seg = [r for r in rel if a <= r[0] < b]
        if not seg:
            print("  [%6.2f, %6.2f) %-10s %5d %s" % (a, b, lab, 0, "(행 없음 — Mimic 밖/Passive)"))
            continue
        v, j, tq = max(((abs(x), jj, r[0]) for r in seg for jj, x in enumerate(r[1])), key=lambda x: x[0])
        print("  [%6.2f, %6.2f) %-10s %5d %6.2f @ %-11s (%6.2f)  %6.1f°   %s" % (
            a, b, lab, len(seg), v, JOINT_NAME[j], tq, max(r[2] for r in seg),
            "/".join(sorted({str(r[3]) for r in seg}))))
    if not rel:
        print("  ⚠ CSV 행이 없다 — 체류 요약의 |qd| 최대만 본다")

    # 6) 체류 요약 (나갈 때 g1_ctrl 이 찍는 덩어리) + 정책 스레드 overrun
    print("\n[체류 요약] (g1_ctrl 가 Mimic 을 나갈 때 찍는 것)")
    for i, (t, s) in enumerate(L):
        if "[Mimic_Masked] 체류" in s:
            print("  %s  %s" % (_fmt_t(t), s.strip()))
            for _, s2 in L[i + 1:i + 6]:
                if re.search(r"pos_clamp|rate_limit|nan_hold|\|qd\| 최대|기울기 최대", s2):
                    print("            %s" % s2.strip())
    n_diag = sum(1 for _, s in post if "[diag:policy]" in s)
    n_over = sum(1 for _, s in post if "[diag:policy]" in s and "[warning]" in s)
    print("\n[diag:policy] 창 %d 개 중 overrun 경고 %d 개" % (n_diag, n_over))


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
    ap.add_argument("--script", default=None,
                    help="시각별 키 파일: 한 줄에 `<밴드 해제 뒤 초> <키>` (예 `12 x`). --stand 는 마지막 시각보다 커야 한다")
    a = ap.parse_args()

    steps = None
    if a.script is not None:
        if a.replay:
            print("🔴 --script 와 --replay 는 같이 못 쓴다")
            return 1
        try:
            steps = parse_script(a.script)
        except (OSError, ValueError) as e:
            print("🔴 스크립트: %s" % e)
            return 1
        if a.stand <= steps[-1][0]:
            print("🔴 --stand %.1f 는 스크립트 마지막 시각 %.1f s 보다 커야 한다" % (a.stand, steps[-1][0]))
            return 1
        print("[스크립트] %s — 키 %d 개, 마지막 %.1f s, 유지 %.1f s" % (a.script, len(steps), steps[-1][0], a.stand))

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
        if steps is not None and not os.environ.get("G1_SAFETY_CSV"):
            # 스크립트는 자세·모드를 바꾸는 측정이다 — 안전층 이벤트 원장(SafetyLog)도 옆에 남긴다.
            cenv["G1_SAFETY_CSV"] = os.path.splitext(a.out)[0] + "_safety.csv"
            print("[안전 기록] %s" % cenv["G1_SAFETY_CSV"])
        mfd, sfd = pty.openpty()
        ctl = subprocess.Popen([os.path.join(G1, "build/g1_ctrl"), "--network=lo"],
                               stdin=sfd, stdout=open("/tmp/s2s_ctl.log", "w"),
                               stderr=subprocess.STDOUT, env=cenv, close_fds=True)
        os.close(sfd)
        tail = LogTail("/tmp/s2s_ctl.log") if steps is not None else None
        t0 = None; i0 = 0; sent = []; pre_fail = None
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
            elif steps is not None:
                # t=0 = 밴드 풀림을 «확인한» 시각. 그 전에 읽힌 줄은 해제 전(pre)으로 친다.
                tail.poll(); i0 = len(tail.lines); t0 = time.monotonic()
                # 🔴 해제 확인(발목 |tau| ≥ 1)은 «밴드에서 떨어져 넘어지는 중» 에도 통과한다 — 넘어지는 로봇의
                #    발목도 토크를 문다(실측: 해제 1.8 s 뒤 bad_orientation → Passive 인데 15 Nm 로 «풀림»).
                #    그 판은 스크립트 키가 Passive 로 들어가 아무것도 안 잰다. t=0 에 아직 Mimic 인지 본다.
                fsm = [s.strip() for _, s in tail.lines if "FSM: Change state from" in s]
                if not fsm or not fsm[-1].endswith("to Mimic_Masked"):
                    pre_fail = fsm[-1] if fsm else "FSM 전환 줄 없음"
                    print("🔴 t=0 에 Mimic_Masked 가 아니다 (%s) — 밴드 해제 중에 넘어졌다. 이 판은 스크립트 측정에 못 쓴다"
                          % pre_fail)
                else:
                    print("[스크립트] t=0 — %.0f 초 동안 키 %d 개" % (a.stand, len(steps)))
                k = 0
                while pre_fail is None:
                    now = time.monotonic() - t0
                    if now >= a.stand:
                        break
                    while k < len(steps) and now >= steps[k][0]:
                        key = steps[k][1]
                        os.write(mfd, key.encode())
                        ts = time.monotonic() - t0
                        sent.append((ts, key))
                        print("  [키] t=%6.2f  %s" % (ts, "space" if key == " " else key), flush=True)
                        k += 1
                    tail.poll()
                    if ctl.poll() is not None:
                        print("🔴 g1_ctrl 이 스크립트 도중 끝났다 (rc=%s) — 여기까지 요약한다" % ctl.returncode)
                        break
                    time.sleep(0.02)
            else:
                print("[정지] %.0f 초 유지" % a.stand)
                time.sleep(a.stand)
            os.write(mfd, b"p")
            if tail is not None:                       # 나갈 때의 체류 요약 줄도 시각을 붙여 받는다
                for _ in range(100):
                    tail.poll(); time.sleep(0.02)
            else:
                time.sleep(2)
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

    band_csv = a.out
    if steps is not None and t0 is not None:
        tail.close()
        tlog = os.path.splitext(a.out)[0] + "_ctl_t.log"   # 줄마다 t(밴드 해제 뒤 초)를 붙인 콘솔 사본
        with open(tlog, "w", encoding="utf-8") as f:
            for i, (m, s) in enumerate(tail.lines):
                f.write("%s | %s\n" % ("   pre" if i < i0 else "%7.2f" % (m - t0), s))
        print("[콘솔] %s (원본 /tmp/s2s_ctl.log)" % tlog)
        if pre_fail is not None:
            print("🔴 스크립트를 돌리지 않았다 — t=0 전에 Mimic 을 나갔다: %s" % pre_fail)
            return 1
        summarize(steps, sent, tail, i0, t0, a.out, a.stand)
        # 밴드 판정은 «해제 뒤 첫 키 전» 의 서 있는 구간으로 한다. check_band_released 는 뒤쪽 절반의
        # 정지 발목 토크를 보는데, 스크립트는 뒤쪽에서 앉거나 눕혀 그 토크가 원래 작다(판정이 틀린다).
        # 해제 자체는 위에서 발목 |tau| ≥ 1 Nm 로 이미 확인했다 — 이것은 같은 것의 데이터 쪽 재확인.
        rows = open(a.out, encoding="utf-8", errors="ignore").read().splitlines() if os.path.exists(a.out) else []
        if rows:
            hdr = rows[0].split(",")
            iw = hdr.index("wall_time") if "wall_time" in hdr else None
            t_first = sent[0][0] if sent else a.stand
            keep = [rows[0]]
            for ln in rows[1:]:
                f = ln.split(",")
                try:
                    if iw is not None and 0.0 <= float(f[iw]) - t0 < t_first:
                        keep.append(ln)
                except (ValueError, IndexError):
                    pass
            band_csv = os.path.splitext(a.out)[0] + "_prescript.csv"
            with open(band_csv, "w", encoding="utf-8") as f:
                f.write("\n".join(keep) + "\n")

    print("\n[판정] 밴드가 정말 풀렸나 — 데이터로 확인한다")
    chk = subprocess.run([sys.executable, os.path.join(REPO, "deploy/scripts/check_band_released.py"), band_csv])
    if chk.returncode != 0:
        print("🔴 밴드가 안 풀린 로그다. 판정에 쓰지 말 것."); return 1
    print("🟢 %s — 쓸 수 있는 sim2sim 로그" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
