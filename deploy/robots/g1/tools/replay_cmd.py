#!/usr/bin/env python3
"""replay_cmd.py — 실기에서 준 base_vel 명령을 sim2sim 에 «그대로» 다시 먹인다.

# 왜
「sim 은 되는데 실기만 뒤로 쳐진다」를 가르려면 **명령이 같아야** 한다. 사람이 키보드를
톡톡 치는 패턴은 두 번 다시 같게 안 나온다 — 실기 로그에 남은 명령을 재생한다.

    python3 replay_cmd.py <실기 gait_*.csv>            # 실시간 재생
    python3 replay_cmd.py <csv> --dry-run              # 무엇을 보낼지만 출력
    python3 replay_cmd.py <csv> --from 40 --to 70      # 그 구간만

# 쓰는 법 (sim2sim)
  ① 시뮬 + 제어기를 띄우고  f → m → 1
  ② 🔴 시뮬 창에서 8,8 → 9 로 **밴드를 푼다** (안 풀면 데이터가 아니다)
  ③ 이 스크립트를 다른 터미널에서 실행. 키보드는 건드리지 않는다.
  ④ 끝나면 gait 덤프를 실기 것과 나란히 비교.

# 🔴 한계 두 가지 — 읽고 시작할 것
1. CSV 의 bv_x 는 «스플라인 **후**» 값이다(GaitAux 주석). 그걸 다시 입력으로 넣으면
   제어기가 한 번 더 완만하게 만든다 = 재생본이 원본보다 «약간 더 부드럽다».
   자세·처짐 비교에는 충분하지만, 급가감속 자체를 재려면 이 점을 감안할 것.
2. 이 채널은 mode 와 base_vel 만 나른다. 실기의 지형·접촉·외란은 재현되지 않는다.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gui_shm  # noqa: E402


def num(r, k, d=0.0):
    try:
        return float(r[k])
    except (TypeError, ValueError, KeyError):
        return d


def load(path, t0, t1):
    out = []
    for r in csv.DictReader(open(path)):
        if not r.get("bv_x"):
            continue
        t = num(r, "time")
        if t < t0 or (t1 and t > t1):
            continue
        out.append((t, int(num(r, "cmd_mode", 1)), num(r, "bv_x"), num(r, "bv_y"), num(r, "bv_wz")))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=0.0)
    ap.add_argument("--dead-band", type=float, default=0.02,
                    help="이만큼 안 변하면 안 보낸다 (제어기가 seq 마다 한 줄 찍어서 시끄럽다)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    rows = load(a.csv, a.t0, a.t1)
    if not rows:
        print("🔴 재생할 표본이 없다 (bv_x 열이 있는 gait 덤프인지 확인)"); return 1
    span = rows[-1][0] - rows[0][0]
    print("표본 %d · %.1f 초 · %s" % (len(rows), span, os.path.basename(a.csv)))
    vx = [r[2] for r in rows]
    print("  bv_x  %+.3f ~ %+.3f  (평균 %+.3f)   모드 %s"
          % (min(vx), max(vx), sum(vx) / len(vx), sorted({r[1] for r in rows})))

    st = {"seq": 0, "cmd_mode": rows[0][1], "vx": 0.0, "vy": 0.0, "wz": 0.0,
          "period_steps": 0, "height_scale": 0.0, "turn_k": 0.3}
    if a.dry_run:
        print("  (dry-run — shm 에 쓰지 않는다)")
        return 0
    if not os.path.isdir("/dev/shm"):
        print("🔴 /dev/shm 이 없다"); return 1

    print("🔴 밴드를 풀었는지 확인했나? (시뮬 창 8,8 → 9)   3 초 뒤 시작한다")
    time.sleep(3)
    t_start = time.time()
    base = rows[0][0]
    sent = last = 0
    lv = (None, None, None, None)
    try:
        for t, mode, x, y, w in rows:
            due = t - base
            slp = due - (time.time() - t_start)
            if slp > 0:
                time.sleep(slp)
            if lv[0] is None or mode != lv[0] or max(abs(x - lv[1]), abs(y - lv[2]), abs(w - lv[3])) >= a.dead_band:
                st.update(cmd_mode=mode, vx=x, vy=y, wz=w)
                gui_shm.write(st)
                lv = (mode, x, y, w); sent += 1
            if due - last >= 10:
                last = due
                print("   %5.0f s / %.0f s · 보낸 명령 %d" % (due, span, sent))
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        st.update(vx=0.0, vy=0.0, wz=0.0)      # 🔴 끝나면 반드시 0 으로 — 안 하면 계속 걷는다
        gui_shm.write(st)
        print("재생 끝 · 명령 %d 건 · base_vel 0 으로 되돌림" % sent)
    return 0


if __name__ == "__main__":
    sys.exit(main())
