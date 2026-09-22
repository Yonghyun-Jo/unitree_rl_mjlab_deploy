#!/usr/bin/env python3
"""Mode5Presets.h · tools/mode5_presets_gen.py · tests/golden_mode5_driver.inc 를 학습 원장에서 생성한다.

  학습 사실  mjlab_g1_motion/mode5_presets.py   PRESETS · build_cmd · GoalDriver · ARRIVE_*
             mjlab_g1_motion/mode_spec.py       MODE5_SLOTS · MODE5_T_GOAL_MAX
  배포 사실  deploy/robots/g1/config/mode5_keys.yaml   키 · 진입 자세

골든 = 파이썬 GoalDriver 를 **그대로** 돌린 출력열이다. C++ Mode5Driver 는 이것과 틱 단위로 같아야 한다.

torch 와 mjlab_g1_motion 을 import 하므로 mjlab 의 uv 환경에서 돈다:
  conda deactivate
  ~/.local/bin/uv run --project ~/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/scripts/gen_mode5_presets_header.py --write
  (…같은 명령…) --check
"""
from __future__ import annotations

import argparse
import inspect
import pathlib
import subprocess
import sys

import numpy as np
import torch
import yaml

from mjlab_g1_motion import mode5_presets as MP
from mjlab_g1_motion import mode_spec

REPO = pathlib.Path(__file__).resolve().parents[2]
G1 = REPO / "deploy/robots/g1"
KEYS = G1 / "config/mode5_keys.yaml"
MODES_YAML = G1 / "config/modes.yaml"
HEADER = G1 / "include/Mode5Presets.h"
PYGEN = G1 / "tools/mode5_presets_gen.py"
GOLDEN = G1 / "tests/golden_mode5_driver.inc"
PROVENANCE_TAG = "mode5_presets.py @ "
RESERVED = set("wasdqe pvfm[]")                # 조작 키 (State_Mimic g_poll_inputs · FSM keyboard_transitions)


def f32(x) -> str:
    """float32 값을 C++ 에서 비트 그대로 되살리는 표기.

    🔴 `%.9g` 는 정수값(1.0·0.0·-1.0)을 소수점 없이 찍는다("1"·"0") — 그 뒤에 그냥 `f` 를
    붙이면 `1f`(정수 리터럴 + 부동소수 접미사)가 되어 C++ 이 거부한다("unable to find numeric
    literal operator 'operator\"\"f'"). 소수점도 지수도 없으면 `.0` 을 끼워 부동소수 리터럴로 만든다.
    """
    s = f"{float(np.float32(x)):.9g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def provenance() -> str:
    src = pathlib.Path(MP.__file__).resolve()
    root = src.parents[2]
    full = subprocess.run(["git", "-C", str(root), "log", "-1", "--format=%H", "--", str(src)],
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain", "--", str(src)],
                           capture_output=True, text=True).stdout.strip()
    return (full[:12] if full else "unknown") + ("+dirty" if dirty else "")


def load_keys() -> tuple[int, dict[int, str]]:
    cfg = yaml.safe_load(KEYS.read_text())
    names = [p.name for p in MP.PRESETS]
    if cfg["enter_preset"] not in names:
        sys.exit(f"mode5_keys.yaml enter_preset '{cfg['enter_preset']}' 가 원장에 없다: {names}")
    mode_keys = {str(v["key"]) for v in yaml.safe_load(MODES_YAML.read_text())["modes"].values()}
    keys: dict[int, str] = {}
    for name, k in cfg["keys"].items():
        if name not in names:
            sys.exit(f"mode5_keys.yaml 의 '{name}' 가 원장(mode5_presets.PRESETS)에 없다: {names}")
        k = str(k)
        if len(k) != 1 or k in RESERVED or k in mode_keys or k in keys.values():
            sys.exit(f"자세 키 '{k}' ('{name}') 가 한 글자가 아니거나 다른 키와 겹친다")
        keys[names.index(name)] = k
    return names.index(cfg["enter_preset"]), keys


def driver_defaults() -> tuple[float, float, float]:
    sig = inspect.signature(MP.GoalDriver.__init__).parameters
    return float(sig["dt"].default), float(sig["tol"].default), float(sig["hold_s"].default)


def scenarios() -> list[dict]:
    """골든 시나리오. 각 원소 = {reset, press, z, g} 한 틱. 문턱 바로 옆 값은 피한다(아래 margin 검사)."""
    steps: list[dict] = []
    enter = next(i for i, p in enumerate(MP.PRESETS) if p.name == "직립")
    for i, p in enumerate(MP.PRESETS):
        u = MP._unit(p.g_pelvis).numpy().astype(np.float64)
        up = MP._unit(MP.PRESETS[enter].g_pelvis).numpy().astype(np.float64)
        # A) 직립에서 출발해 40틱에 걸쳐 목표 높이·자세로, 그 뒤 60틱 머묾 (도착 → hold 전이 포함)
        z0 = 0.76
        for t in range(100):
            a = min(t / 40.0, 1.0)
            g = (1 - a) * up + a * u
            g = g / np.linalg.norm(g)
            steps.append(dict(reset=(t == 0), press=(i if t == 0 else -1), z=z0 + a * (p.z - z0), g=g))
    # B) 재입력: 네발 기기로 30틱 머문 뒤 같은 버튼을 다시 → 트랜짓 재시작(t_goal 0 부터)
    q = next(i for i, p in enumerate(MP.PRESETS) if p.name == "네발 기기")
    uq = MP._unit(MP.PRESETS[q].g_pelvis).numpy().astype(np.float64)
    for t in range(80):
        steps.append(dict(reset=(t == 0), press=(q if t in (0, 30) else -1), z=MP.PRESETS[q].z, g=uq))
    # C) 자세 규칙으로만 도착: 높이가 8 cm 낮고(z 규칙 불성립) 자세는 일치(g 규칙 성립)
    for t in range(60):
        steps.append(dict(reset=(t == 0), press=(q if t == 0 else -1), z=MP.PRESETS[q].z - 0.08, g=uq))
    # D) 깜빡임: 도착 20틱 → 1틱 벗어남 → 다시 도착 (연속 시간이 0 으로 리셋되는가)
    for t in range(80):
        off = (t == 20)
        steps.append(dict(reset=(t == 0), press=(q if t == 0 else -1),
                          z=MP.PRESETS[q].z + (0.3 if off else 0.0), g=uq))
    return steps


def run_python(steps: list[dict], dt: float) -> list[dict]:
    names = [p.name for p in MP.PRESETS]
    out, drv = [], None
    for s in steps:
        if s["reset"]:
            drv = MP.GoalDriver(dt=dt)
        if s["press"] >= 0:
            drv.press(names[s["press"]])
        p = MP.BY_NAME[drv.preset]
        g32 = torch.as_tensor(s["g"], dtype=torch.float32)
        dot = float((g32 * MP._unit(p.g_pelvis)).sum())
        dz = abs(s["z"] - p.z)
        # 🔴 문턱 바로 옆이면 C++ float 연산 순서 차이로 판정이 갈릴 수 있다 — 골든을 그런 값으로 만들지 않는다.
        if abs(dot - MP.ARRIVE_G_DOT) < 1e-4 or abs(dz - drv.tol) < 1e-6 or abs(dz - MP.ARRIVE_Z_LOOSE) < 1e-6:
            sys.exit(f"골든 시나리오가 문턱에 너무 가깝다: dot={dot} dz={dz} ({p.name})")
        cmd = drv.tick(s["z"], g_now=g32)
        out.append(dict(s, cmd=cmd.numpy(), hold=(drv.phase == "hold")))
    return out


def build() -> dict[pathlib.Path, str]:
    enter, keys = load_keys()
    dt, tol, hold_s = driver_defaults()
    SL = mode_spec.MODE5_SLOTS
    commit = provenance()
    rows = []
    for i, p in enumerate(MP.PRESETS):
        cmd = MP.build_cmd(p, z_mask=0.0).numpy()
        u = MP._unit(p.g_pelvis).numpy()
        key = f"'{keys[i]}'" if i in keys else "'\\0'"
        rows.append(f'  {{"{p.name}", {key}, {p.z!r}, {f32(p.transit_z_mask)}, '
                    f'{{{{{", ".join(f32(x) for x in u)}}}}}, '
                    f'{{{{{", ".join(f32(x) for x in cmd)}}}}}, {p.n}}},')
    header = f"""#pragma once
// Mode5Presets.h — 🔴 생성 파일. 손으로 고치지 않는다.  deploy/scripts/gen_mode5_presets_header.py --write
//   학습 사실: mjlab_g1_motion/mode5_presets.py @ {commit}
//   배포 사실: deploy/robots/g1/config/mode5_keys.yaml
// cmd = build_cmd(자세, z_mask=0). z_mask·t_goal 은 Mode5Driver 가 채운다.
#include <array>

namespace m5 {{

inline constexpr int CMD_DIM = {mode_spec.MODE5_CMD_DIM};
inline constexpr int SLOT_Z_MASK = {SL["z_mask"].start};
inline constexpr int SLOT_T_GOAL = {SL["t_goal"].start};
inline constexpr double T_GOAL_MAX = {float(mode_spec.MODE5_T_GOAL_MAX)!r};
inline constexpr double ARRIVE_G_DOT = {float(MP.ARRIVE_G_DOT)!r};
inline constexpr double ARRIVE_Z_LOOSE = {float(MP.ARRIVE_Z_LOOSE)!r};
inline constexpr double TOL = {tol!r};          // GoalDriver(tol=) 기본값
inline constexpr double HOLD_S = {hold_s!r};    // GoalDriver(hold_s=) 기본값

struct Preset {{
  const char* name; char key; double z; float transit_z_mask;
  std::array<float, 3> g_pelvis_unit;
  std::array<float, CMD_DIM> cmd;
  int n;                                   // 근거 구간 수 (버튼 라벨용)
}};

inline constexpr int N_PRESETS = {len(MP.PRESETS)};
inline constexpr int ENTER_PRESET = {enter};   // mode5 에 들어가면 이 자세로 시작 (spec §4.4)
inline constexpr std::array<Preset, N_PRESETS> PRESETS = {{{{
{chr(10).join(rows)}
}}}};

inline constexpr int preset_by_key(char k) {{
  for (int i = 0; i < N_PRESETS; ++i) if (PRESETS[i].key != '\\0' && PRESETS[i].key == k) return i;
  return -1;
}}

}}  // namespace m5
"""
    py_rows = "\n".join(f'    ({i}, "{p.name}", "{keys.get(i, "")}", {p.n}),' for i, p in enumerate(MP.PRESETS))
    py = f'''# 🔴 생성 파일. 손으로 고치지 않는다.  deploy/scripts/gen_mode5_presets_header.py --write
#   학습 사실: mjlab_g1_motion/mode5_presets.py @ {commit}
"""mode5 자세 버튼 표의 파이썬 판 — GUI 가 버튼을 이 표에서 만든다. shm 의 m5_preset = index + 1."""
ENTER_PRESET = {enter}
PRESETS = [  # (index, name, key, n)
{py_rows}
]
'''
    res = run_python(scenarios(), dt)
    lines = []
    for r in res:
        lines.append(f"  {{{int(r['reset'])}, {r['press']}, {r['z']!r}, "
                     f"{{{', '.join(f32(x) for x in r['g'])}}}, {int(r['hold'])}, "
                     f"{{{', '.join(f32(x) for x in r['cmd'])}}}}},")
    golden = f"""// 🔴 생성 파일. deploy/scripts/gen_mode5_presets_header.py --write
//   파이썬 mode5_presets.GoalDriver @ {commit} 를 그대로 돌린 출력열 (dt={dt!r}).
struct GoldenStep {{ int reset; int press; double z; float g[3]; int hold; float cmd[{mode_spec.MODE5_CMD_DIM}]; }};
static const GoldenStep kGoldenM5[] = {{
{chr(10).join(lines)}
}};
static const int kGoldenM5N = {len(res)};
static const double kGoldenM5Dt = {dt!r};
"""
    return {HEADER: header, PYGEN: py, GOLDEN: golden}


def body(s: str) -> str:
    return "\n".join(l for l in s.splitlines() if PROVENANCE_TAG not in l)


def main() -> None:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true"); g.add_argument("--write", action="store_true")
    a = ap.parse_args()
    out = build()
    if a.write:
        for p, t in out.items():
            p.write_text(t); print(f"[gen] wrote {p}")
        return
    bad = [p for p, t in out.items() if not p.exists() or body(p.read_text()) != body(t)]
    if bad:
        sys.exit("원장과 다르다 — --write 로 다시 생성할 것: " + ", ".join(str(p) for p in bad))
    print("[gen] Mode5Presets.h · mode5_presets_gen.py · golden_mode5_driver.inc == 원장")


if __name__ == "__main__":
    main()
