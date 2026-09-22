#!/usr/bin/env python3
"""sim2sim_headless.pre_t0_failure — «밴드 해제 확인 시각(t=0)에 아직 Mimic_Masked 인가» 판정.

해제 확인(발목 |tau| ≥ 1 Nm)은 밴드에서 떨어져 넘어지는 중에도 통과한다. 그 판을 «풀림» 으로 받으면 판정에 못 쓰는
로그가 쓸 수 있는 로그로 둔갑한다 — --script 판만 막혀 있었고 --stand/--replay 판은 구멍이었다(최종 검토 M-12).
표준 라이브러리만 쓴다:  python3 deploy/robots/g1/tests/test_headless_pre_t0.py
"""
from __future__ import annotations

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "scripts"))
import sim2sim_headless as H  # noqa: E402

fails = 0


def chk(c, what):
    global fails
    print(("  ok   " if c else "  FAIL ") + what)
    fails += (not c)


FS = "[2026-09-22 18:00:0%d.000] [info] FSM: Change state from %s to %s"
standing = [FS % (1, "Passive", "FixStand"), FS % (2, "FixStand", "Mimic_Masked"), "[m5] 진입 → «직립»"]
fell = standing + [FS % (4, "Mimic_Masked", "Passive")]
chk(H.pre_t0_failure(standing) is None, "t=0 에 Mimic_Masked → 통과")
chk(H.pre_t0_failure(fell) is not None and "Passive" in H.pre_t0_failure(fell), "해제 중에 넘어져 Passive → 실패(마지막 전환 줄)")
chk(H.pre_t0_failure([]) == "FSM 전환 줄 없음", "전환 줄이 없으면 실패")
chk(H.pre_t0_failure(fell + [FS % (6, "Passive", "FixStand"), FS % (7, "FixStand", "Mimic_Masked")]) is None,
    "마지막 전환이 Mimic_Masked 면 통과(다시 들어왔다)")
# --stand/--replay 판이 읽는 콘솔 파일 그대로: ANSI 색·\r 이 섞인 원문
with tempfile.NamedTemporaryFile("wb", suffix=".log", delete=False) as f:
    f.write(("\x1b[32m" + FS % (2, "FixStand", "Mimic_Masked") + "\x1b[0m\r\n").encode())
    f.write(("\x1b[31m" + FS % (4, "Mimic_Masked", "Passive") + "\x1b[0m\r\n").encode())
    path = f.name
try:
    lines = H._console_lines(path)
    chk(H.pre_t0_failure(lines) is not None, "콘솔 파일(ANSI·\\r 포함)에서도 해제 중 넘어짐을 잡는다")
    chk(H._console_lines(path + ".none") == [], "파일이 없으면 빈 목록(→ «전환 줄 없음» 으로 실패)")
finally:
    os.remove(path)
print("PASS" if not fails else "FAIL (%d)" % fails)
sys.exit(1 if fails else 0)
