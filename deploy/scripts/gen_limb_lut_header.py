#!/usr/bin/env python3
"""LimbLut.h 의 «구운 표» · tests/golden_limb_lut.inc 를 학습 원장(mjlab final_v6)에서 기계 생성한다.

mode6(네발 기기) 사지 높이 명령 = 학습 `tasks/mdp/limb_lut.py` 의 위상 1차원 표(`limb_lut_data.py`).
배포 C++ 는 그 표를 이 스크립트가 옮긴 배열로만 갖는다 — 손으로 옮겨 적지 않는다(GaitLut.h 가 생성기
없이 낡은 전례, gen_gait_lut_header.py 머리). golden = 학습 함수 `limb_lut.limb_z_scalar` 를 그대로 돌린
출력이라, C++ 보간·정지 게이트·위상 시계가 학습과 같은지 test_limb_lut.cpp 가 대조한다.

사용:
  python3 deploy/scripts/gen_limb_lut_header.py --check            # 헤더·golden 이 원장과 같은지
  python3 deploy/scripts/gen_limb_lut_header.py --write            # 다시 쓴다
  --mjlab <경로>  limb_lut.py 가 있는 mjlab 워크트리 (기본 ~/mjlab1.4/mjlab_g1_final)
golden 은 학습 함수를 import 해야 해서(torch) 그 워크트리의 .venv 파이썬으로 돌린다.
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import pathlib
import subprocess
import sys

DATA_REL = "src/mjlab_g1_motion/tasks/mdp/limb_lut_data.py"
G1 = pathlib.Path(__file__).resolve().parents[1] / "robots/g1"
HEADER = G1 / "include/LimbLut.h"
GOLDEN = G1 / "tests/golden_limb_lut.inc"
BEGIN = "// ── BEGIN GENERATED (gen_limb_lut_header.py) ──"
END = "// ── END GENERATED ──"

# golden 표본: 위상(경계·구간 중심·임의) × 유효 속력(정지 게이트 양쪽) + 위상 시계 한 바퀴.
_GOLDEN_PY = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
from mjlab_g1_motion.tasks.mdp import limb_lut as L
ph = [0.0, 1e-6, 0.5 / L.NB, 1.0 / L.NB, 0.123, 0.25, 0.4999, 0.5, 0.66, 0.75, 0.9, 0.99, 0.999999]
ph += [((k * 0.6180339887) % 1.0) for k in range(1, 40)]
eff = [0.0, 0.049, 0.05, 0.051, 0.3, 1.0]
pts = [(p, e, list(L.limb_z_scalar(p, e))) for p in ph for e in eff]
rate = L.phase_rate(1.0 / 50.0)
clock, p = [], 0.0
for t in range(200):                       # advance_limb 와 같은 순서: 위상 먼저, 그 위상으로 높이
    p = (p + rate) % 1.0
    clock.append((p, list(L.limb_z_scalar(p, 0.3))))
print(json.dumps(dict(pts=pts, rate=rate, clock=clock, stand_eps=L.STAND_EPS)))
'''


def load_data(mjlab: pathlib.Path) -> dict:
    """limb_lut_data.py 를 import 없이 읽는다(리터럴만 있는 파일)."""
    path = mjlab / DATA_REL
    if not path.exists():
        sys.exit(f"limb_lut_data.py 가 없다: {path}")
    out = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and hasattr(node.targets[0], "id"):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    for k in ("NB", "LIMB_STANCE", "PERIOD_S", "LIMB_P"):
        if k not in out:
            sys.exit(f"limb_lut_data.py 에 {k} 가 없다")
    if len(out["LIMB_P"]) != out["NB"] or any(len(r) != 4 for r in out["LIMB_P"]):
        sys.exit("LIMB_P 모양이 (NB, 4) 가 아니다")
    commit = subprocess.run(["git", "-C", str(mjlab), "log", "-1", "--format=%H", "--", DATA_REL],
                            capture_output=True, text=True).stdout.strip()[:12] or "unknown"
    if subprocess.run(["git", "-C", str(mjlab), "status", "--porcelain", "--", DATA_REL],
                      capture_output=True, text=True).stdout.strip():
        commit += "+dirty"
    out["_commit"] = commit
    return out


def golden_values(mjlab: pathlib.Path) -> dict:
    py = mjlab / ".venv/bin/python"
    if not py.exists():
        sys.exit(f"mjlab venv 파이썬이 없다: {py}")
    env = {k: v for k, v in os.environ.items() if k not in ("LD_LIBRARY_PATH", "CONDA_PREFIX", "PYTHONPATH")}
    r = subprocess.run([str(py), "-c", _GOLDEN_PY, str(mjlab / "src")], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        sys.exit("학습 함수 limb_lut 을 못 돌렸다: " + (r.stderr or r.stdout).strip()[-400:])
    return json.loads(r.stdout.strip().splitlines()[-1])


def f(x: float) -> str:
    return repr(float(x)) + "f"


def generated_block(d: dict) -> str:
    rows = ",\n".join("    {%s}" % ", ".join(f(v) for v in r) for r in d["LIMB_P"])
    return "\n".join([
        BEGIN,
        f"// 원장 = mjlab {DATA_REL} @ {d['_commit']} (손으로 고치지 말 것 — --write 로 다시 만든다)",
        f"inline constexpr int NB = {int(d['NB'])};",
        f"inline constexpr float PERIOD_S = {f(d['PERIOD_S'])};",
        "inline constexpr std::array<float, N_LIMBS> LIMB_STANCE = {%s};" % ", ".join(f(v) for v in d["LIMB_STANCE"]),
        "inline constexpr float LIMB_P[NB][N_LIMBS] = {",
        rows,
        "};",
        END,
    ])


def golden_text(d: dict, g: dict) -> str:
    pts = ",\n".join("  {%s, %s, {%s}}" % (repr(p), repr(e), ", ".join(f(z) for z in zs)) for p, e, zs in g["pts"])
    clk = ",\n".join("  {%s, {%s}}" % (repr(p), ", ".join(f(z) for z in zs)) for p, zs in g["clock"])
    return (f"// 🔴 생성 파일. deploy/scripts/gen_limb_lut_header.py --write\n"
            f"//   학습 함수 mjlab tasks/mdp/limb_lut.py (limb_lut_data @ {d['_commit']}) 의 limb_z_scalar 출력.\n"
            "struct GoldenLimb { double phase; double eff; float z[4]; };\n"
            "static const GoldenLimb kGoldenLimb[] = {\n" + pts + "\n};\n"
            f"static constexpr double kGoldenLimbRate = {repr(g['rate'])};   // phase_rate(dt=1/50)\n"
            "struct GoldenLimbClock { double phase; float z[4]; };   // 0 에서 출발, eff 0.3, 200 틱\n"
            "static const GoldenLimbClock kGoldenLimbClock[] = {\n" + clk + "\n};\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjlab", default=os.path.expanduser("~/mjlab1.4/mjlab_g1_final"))
    g_ = ap.add_mutually_exclusive_group(required=True)
    g_.add_argument("--check", action="store_true")
    g_.add_argument("--write", action="store_true")
    a = ap.parse_args(argv)
    mj = pathlib.Path(a.mjlab)
    d = load_data(mj)
    blk = generated_block(d)
    hdr = HEADER.read_text()
    if BEGIN not in hdr or END not in hdr:
        sys.exit(f"{HEADER} 에 GENERATED 구역 표지가 없다")
    new_hdr = hdr[:hdr.index(BEGIN)] + blk + hdr[hdr.index(END) + len(END):]
    gold = golden_text(d, golden_values(mj))
    strip = lambda s: "\n".join(l for l in s.splitlines() if "원장 = mjlab" not in l and "limb_lut_data @" not in l)  # noqa: E731
    if a.check:
        bad = []
        if strip(new_hdr) != strip(hdr):
            bad.append("LimbLut.h 의 표")
        if not GOLDEN.exists() or strip(gold) != strip(GOLDEN.read_text()):
            bad.append("golden_limb_lut.inc")
        if bad:
            print("[gen] 🔴 원장과 다르다: %s — `gen_limb_lut_header.py --write` 후 단위 테스트" % ", ".join(bad))
            return 1
        print(f"[gen] LimbLut.h · golden_limb_lut.inc == 원장 (limb_lut_data @ {d['_commit']})")
        return 0
    HEADER.write_text(new_hdr)
    GOLDEN.write_text(gold)
    print(f"[gen] 썼다: {HEADER.name} · {GOLDEN.name} (limb_lut_data @ {d['_commit']}, golden {len(gold.splitlines())} 줄)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
