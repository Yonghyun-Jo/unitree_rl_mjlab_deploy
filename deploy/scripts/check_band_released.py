#!/usr/bin/env python3
"""check_band_released.py — gait 계측 CSV 가 «밴드를 푼 뒤» 찍힌 것인지 판정한다.

# 왜 필요한가
MuJoCo sim 의 고무밴드는 **기본 켜짐**이고 키 `9` 로만 꺼진다. 켜져 있으면 로봇이 매달려
다리 하중이 1/10 이 된다 — 그 로그로는 자세도 처짐도 추종도 판정할 수 없다.
2026-09-01 에 내가 밴드가 켜진 로그를 «sim 기준선» 으로 써서 «sim 은 멀쩡한데 실기만
처진다» 는 표를 만들었다. 그건 «밴드 vs 밴드없음» 이었다.

# 어떻게 판별하나
사람이 「풀었다」고 기억하는 것에 기대지 않는다. **정지 중 발목 pitch 유지토크**로 본다 —
로봇이 제 무게로 서 있으면 발목은 CoM 편차만큼 토크를 문다(체중 33 kg 기준 1 cm 당 3.3 Nm).
매달려 있으면 그게 사라진다.

    실측 (2026-09-01, 같은 제어기·같은 슬롯)
      밴드 ON (sim, xvfb)   발목 |tau| 평균 0.55 Nm · 무릎 2.8 Nm
      밴드 OFF (실기)        발목 |tau| 평균 5.6  Nm · 무릎 9.2 Nm

    임계 1.0 Nm — 두 실측 사이가 10 배라 여유가 크다.

⚠ 실기 로그에는 밴드가 없다(항상 «풀린» 상태). 이 검사는 **sim 로그가 쓸 수 있는 것인지**
  를 보는 용도이고, 실기 로그에 돌리면 당연히 통과한다.

    python3 deploy/scripts/check_band_released.py <gait_*.csv> [...]
"""
from __future__ import annotations

import csv
import sys

ANKLE_PITCH = (4, 10)          # L/R ankle_pitch 모터 인덱스
THRESH_NM = 1.0


def num(row, key, dflt=0.0):
    try:
        return float(row[key])
    except (TypeError, ValueError, KeyError):
        return dflt


def judge(path):
    """🔴 «뒤쪽 절반» 만 본다.

    밴드는 보통 기동 «도중에» 풀린다(f -> m -> 정책 ON -> 그때 9). 전체를 평균하면 해제
    전 구간이 값을 끌어내려 **풀렸는데도 «안 풀림» 으로 오판한다** — 실제로 그랬다
    (해제 t=2.6 s 인 로그를 0.34 Nm 로 읽고 거부했다).
    로그 뒤쪽은 정의상 해제 이후이므로 거기서 판정한다."""
    rows = []
    for r in csv.DictReader(open(path)):
        if not r.get("kp_0") or num(r, "kp_0") <= 0:
            continue
        if abs(num(r, "eff")) > 0.01:                # 걷는 중 제외 — 정지 중만 본다
            continue
        rows.append(sum(abs(num(r, "tau_est_%d" % j)) for j in ANKLE_PITCH) / 2.0)
    if len(rows) < 50:
        return None, len(rows), 0.0
    tail = rows[len(rows) // 2:]                     # 뒤쪽 절반
    mean = sum(tail) / len(tail)
    return mean >= THRESH_NM, len(tail), mean


def main(argv):
    if not argv:
        print(__doc__.strip().splitlines()[0]); return 2
    bad = 0
    for p in argv:
        ok, n, mean = judge(p)
        name = p.split("/")[-1]
        if ok is None:
            print("  ⚠  %-40s 정지 표본이 %d 개뿐 — 판정 불가" % (name, n)); continue
        if ok:
            print("  🟢 %-40s 발목 |tau| %.2f Nm (정지 %d 줄) — 밴드 «풀림». 판정에 써도 된다"
                  % (name, mean, n))
        else:
            bad += 1
            print("  🔴 %-40s 발목 |tau| %.2f Nm (정지 %d 줄) < %.1f" % (name, mean, n, THRESH_NM))
            print("      → 로봇이 밴드에 «매달린» 로그다. 자세·처짐·추종 판정에 쓰지 말 것.")
            print("      → 다시 찍을 것: f → m → 1 → 시뮬 창에서 8,8 → 9(해제) → 그 다음부터 데이터")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
