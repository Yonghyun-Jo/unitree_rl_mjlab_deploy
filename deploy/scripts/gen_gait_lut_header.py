#!/usr/bin/env python3
"""GaitLut.h 의 «구운 배열» 을 mjlab_g1_motion/tasks/mdp/gait_lut_data.py 에서 기계 생성한다.

GaitLut.h 는 스스로 「손으로 고치지 않는다 — 배열은 gait_lut_data.py 에서 기계 생성한 값이다」
라고 적고 있었지만 **그 생성기가 없었다**. 없으니 상류가 재적합돼도 아무도 안 알려주고, 실제로
2026-08-25 COLMOv2 재적합(d94c7d9) 이후 배포 표가 통째로 낡은 채였다.

표는 **한 벌이 아니다.** head 마다 학습 시점이 달라 그 시점의 표로 학습됐다:
  V1  motions 적합   STANCE_Z 6.6877 cm  (2026-07-14 4578357 ~ 08-25)  <- mode2 head 가 이걸로 학습
  V2  COLMOv2 재적합 STANCE_Z 3.5000 cm  (2026-08-25 d94c7d9 ~)        <- 새 mode1 head 가 이걸로

사용:
  python3 deploy/scripts/gen_gait_lut_header.py --check     # 현재 헤더와 일치하는지만 본다
  python3 deploy/scripts/gen_gait_lut_header.py --write     # 헤더의 배열 구역을 다시 쓴다
"""
from __future__ import annotations

import argparse
import importlib.util
import pathlib
import subprocess
import sys

MJLAB = pathlib.Path.home() / "mjlab1.4" / "mjlab_g1_motion"
DATA_REL = "src/mjlab_g1_motion/tasks/mdp/gait_lut_data.py"
HEADER = pathlib.Path(__file__).resolve().parents[1] / "robots/g1/include/GaitLut.h"

# V1 = COLMOv2 재적합 «직전» 판. 이 커밋을 옮기면 mode2 head 와 배포가 어긋난다.
V1_COMMIT = "d94c7d9~1"

BEGIN = "// ── BEGIN GENERATED (gen_gait_lut_header.py) ──"
END = "// ── END GENERATED ──"


def load_at(commit: str | None):
    """gait_lut_data.py 를 특정 커밋(None=워킹트리)에서 읽어 모듈로 만든다."""
    if commit is None:
        src = (MJLAB / DATA_REL).read_text()
    else:
        src = subprocess.run(["git", "-C", str(MJLAB), "show", f"{commit}:{DATA_REL}"],
                             capture_output=True, text=True, check=True).stdout
    path = pathlib.Path("/tmp") / f"_gld_{(commit or 'work').replace('~', '_').replace(':', '_')}.py"
    path.write_text(src)
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def farr(vals) -> str:
    return ", ".join(f"{v:.6f}f" for v in vals)


def table_block(tag: str, m, note: str) -> str:
    nw, nr, nb = len(m.WALK_V), len(m.RUN_V), m.NB
    # stand_eps: 학습의 정지 게이트. 2026-08-25 09fb616 이전 = 0.05, 이후 = STAND_EPS(1e-6).
    # 표 판번호와 게이트 변경이 같은 날 연속 커밋이라 표에 묶어 둔다(V1 표 = 옛 게이트).
    stand_eps = "0.05f" if tag == "V1" else "1e-6f"
    L: list[str] = []
    a = L.append
    a(f"// ── {tag}: {note}")
    a(f"inline constexpr int   GL_{tag}_NB = {nb};")
    a(f"inline constexpr int   GL_{tag}_NW = {nw};")
    a(f"inline constexpr int   GL_{tag}_NR = {nr};")
    a(f"inline constexpr float GL_{tag}_STANCE_Z = {m.STANCE_Z:.6f}f;")
    a(f"inline constexpr float GL_{tag}_WALK_V[GL_{tag}_NW] = {{{farr(m.WALK_V)}}};")
    a(f"inline constexpr float GL_{tag}_RUN_V [GL_{tag}_NR] = {{{farr(m.RUN_V)}}};")
    a(f"inline constexpr float GL_{tag}_WALK_F[GL_{tag}_NW] = {{{farr(m.WALK_F)}}};")
    a(f"inline constexpr float GL_{tag}_RUN_F [GL_{tag}_NR] = {{{farr(m.RUN_F)}}};")
    for name, rows, n in (("WALK_P", m.WALK_P, nw), ("RUN_P", m.RUN_P, nr)):
        a(f"inline constexpr float GL_{tag}_{name}[GL_{tag}_N{name[0]}][GL_{tag}_NB] = {{")
        for r in rows:
            a(f"    {{{farr(r)}}},")
        a("};")
    a(f"inline constexpr GlTable GL_T_{tag} = {{")
    a(f"    GL_{tag}_NB, GL_{tag}_NW, GL_{tag}_NR, GL_{tag}_STANCE_Z,")
    a(f"    GL_{tag}_WALK_V, GL_{tag}_RUN_V, GL_{tag}_WALK_F, GL_{tag}_RUN_F,")
    a(f"    &GL_{tag}_WALK_P[0][0], &GL_{tag}_RUN_P[0][0],")
    a(f"    {m.ASYM_K_WALK:.6f}f, {m.ASYM_WREF_WALK:.6f}f, "
      f"{m.ASYM_K_RUN:.6f}f, {m.ASYM_WREF_RUN:.6f}f,")
    a(f"    {stand_eps},")
    a("};")
    return "\n".join(L)


def generate() -> str:
    v1, v2 = load_at(V1_COMMIT), load_at(None)
    head = subprocess.run(["git", "-C", str(MJLAB), "rev-parse", "--short", "HEAD"],
                          capture_output=True, text=True, check=True).stdout.strip()
    out = [
        BEGIN,
        f"// 생성기: deploy/scripts/gen_gait_lut_header.py",
        f"// 상류:   mjlab_g1_motion {DATA_REL}",
        f"//         V1 = {V1_COMMIT}  ·  V2 = {head} (워킹트리)",
        "// ⚠ 이 구역을 손으로 고치지 않는다. 상류 재적합 후 --write 로 다시 뽑고,",
        "//   tests/test_gait_lut.cpp 의 패리티 벡터도 mjlab 에서 다시 뽑을 것.",
        "",
        table_block("V1", v1,
                    "motions 적합 (2026-07-14 4578357). mode2 head 가 이 표로 학습됐다."),
        "",
        table_block("V2", v2,
                    "COLMOv2 재적합 (2026-08-25 d94c7d9). 새 mode1 head 가 이 표로 학습됐다."),
        END,
    ]
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    block = generate()
    text = HEADER.read_text()
    if BEGIN not in text or END not in text:
        if not args.write:
            print("헤더에 GENERATED 구역 표지가 없다. 먼저 --write 로 구역을 만들 것.", file=sys.stderr)
            return 2
        print("헤더에 GENERATED 구역이 없어 새로 만들지 못한다 — 표지를 손으로 한 번 넣을 것.",
              file=sys.stderr)
        return 2

    i, j = text.index(BEGIN), text.index(END) + len(END)
    cur = text[i:j]
    if cur == block:
        print("일치 — 헤더가 상류와 같다.")
        return 0
    if args.check:
        print("🔴 헤더가 상류와 다르다. --write 로 갱신할 것.", file=sys.stderr)
        return 1
    HEADER.write_text(text[:i] + block + text[j:])
    print(f"갱신했다: {HEADER}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
