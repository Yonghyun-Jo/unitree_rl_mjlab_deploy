#!/usr/bin/env python3
"""ModeTable.h 를 «학습 사실 + 배포 사실» 에서 기계 생성한다.

  학습 사실  mjlab_g1_motion/mode_spec.py  MODES · MASK_DIM          (bits · 추종 · 명령 채널)
  배포 사실  deploy/robots/g1/config/modes.yaml                       (참조 출처 · 발 높이 원천 · 블렌드 · 안전 · 이탈 · 키)

두 표의 모드 집합이 다르거나 서로 모순이면 **실패한다** — 한쪽만 고친 채로 빌드되지 않게.

사용:
  python3 deploy/scripts/gen_mode_table_header.py --check    # 헤더가 원장과 일치하는지만 (CI·테스트용)
  python3 deploy/scripts/gen_mode_table_header.py --write    # 헤더를 다시 쓴다
  --mjlab <경로>  mode_spec.py 가 있는 mjlab 워크트리 (기본 ~/mjlab1.4/mjlab_g1_mode45)
"""
from __future__ import annotations

import argparse
import importlib.util
import pathlib
import subprocess
import sys
import types

import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
MODES_YAML = REPO / "deploy/robots/g1/config/modes.yaml"
HEADER = REPO / "deploy/robots/g1/include/ModeTable.h"
SPEC_REL = "src/mjlab_g1_motion/mode_spec.py"
PROVENANCE_TAG = "mode_spec.py @ "      # 헤더의 출처 줄 — --check 는 이 줄을 빼고 표만 비교한다

ENUMS = {
    "ref_source": ("RefSource", {"none": "None", "vr": "Vr", "clip": "Clip"}),
    "foot_z": ("FootZ", {"none": "None", "gen": "Gen", "ref": "Ref"}),
    "safety": ("Safety", {"upright_only": "UprightOnly", "ground_capable": "GroundCapable"}),
    "exit": ("Exit", {"always": "Always", "upright": "Upright", "standing_hold": "StandingHold", "via_ground": "ViaGround"}),
}
TRAIN_FLAGS = ("track_upper", "track_lower", "base_vel_live", "motion_preview", "foot_z_live",
               "mode5_cmd_live", "crawl_cmd_live")


def load_mode_spec(mjlab: pathlib.Path):
    """mode_spec.py 를 torch 없이 읽는다 — 모듈은 import 시점에 torch 를 호출하지 않는다(함수 안에서만 쓴다)."""
    path = mjlab / SPEC_REL
    if not path.exists():
        sys.exit(f"mode_spec.py 가 없다: {path}")
    sys.modules.setdefault("torch", types.ModuleType("torch"))
    spec = importlib.util.spec_from_file_location("_mode_spec_for_gen", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # dataclass 가 모듈을 찾을 수 있게
    spec.loader.exec_module(mod)
    # 🔴 길이를 여기서 고정한다. `%h` 의 축약 길이는 repo 크기·core.abbrev 에 따라 «머신마다» 달라서
    #    (7 vs 8 …) 그대로 헤더에 박으면 다른 머신의 --check 가 내용은 같은데 «원장과 다르다» 로
    #    헛되이 실패한다. %H 를 받아 파이썬에서 12 자로 자른다.
    full = subprocess.run(["git", "-C", str(mjlab), "log", "-1", "--format=%H", "--", SPEC_REL],
                          capture_output=True, text=True).stdout.strip()
    commit = full[:12] if full else "unknown"
    # 커밋 안 된 변경이 있으면 그 해시는 «방금 읽은 내용» 이 아니다 — 표시해 둔다(재현 불가 표시).
    dirty = subprocess.run(["git", "-C", str(mjlab), "status", "--porcelain", "--", SPEC_REL],
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        commit += "+dirty"
    return mod, commit


def build(mjlab: pathlib.Path) -> str:
    ms, commit = load_mode_spec(mjlab)
    dep = yaml.safe_load(MODES_YAML.read_text())["modes"]
    train_ids, dep_ids = sorted(ms.MODES), sorted(int(k) for k in dep)
    if train_ids != dep_ids:
        sys.exit(f"모드 집합이 다르다 — mode_spec.py {train_ids} vs modes.yaml {dep_ids}")
    if train_ids != list(range(1, len(train_ids) + 1)):
        sys.exit(f"모드 번호는 1 부터 빈칸 없이: {train_ids}")
    rows = []
    for m in train_ids:
        t, d = ms.MODES[m], dep[m] if m in dep else dep[str(m)]
        # 두 표의 모순 = 생성 실패
        if (d["foot_z"] == "none") != (not t.foot_z_live):
            sys.exit(f"mode{m}: modes.yaml foot_z={d['foot_z']} ↔ mode_spec foot_z_live={t.foot_z_live}")
        if (d["ref_source"] == "none") != (not (t.track_upper or t.track_lower)):
            sys.exit(f"mode{m}: ref_source={d['ref_source']} ↔ track_upper/lower={t.track_upper}/{t.track_lower}")
        if t.motion_preview and d["ref_source"] != "clip":
            sys.exit(f"mode{m}: motion_preview 는 클립에서만 나온다 (ref_source={d['ref_source']})")
        if len(str(d["key"])) != 1:
            sys.exit(f"mode{m}: key 는 한 글자")
        bits = list(t.bits) + [0.0] * (ms.MASK_DIM - len(t.bits))
        flags = ", ".join("true" if getattr(t, f) else "false" for f in TRAIN_FLAGS)
        enums = ", ".join(f"{ENUMS[k][0]}::{ENUMS[k][1][d[k]]}" for k in ("ref_source", "foot_z"))
        rows.append(
            f'  {{{m}, "{t.name}", \'{d["key"]}\', {{{{{", ".join(f"{float(b):.1f}f" for b in bits)}}}}}, '
            f'{flags}, {enums}, {"true" if d["arm_blend_enter"] else "false"}, '
            f'{"true" if d["crossfade_enter"] else "false"}, Safety::{ENUMS["safety"][1][d["safety"]]}, '
            f'Exit::{ENUMS["exit"][1][d["exit"]]}, "{d["gait"]}"}},')
    enum_src = "\n".join(f"enum class {name} : unsigned char {{ {', '.join(vals.values())} }};"
                         for name, vals in (v for v in ENUMS.values()))
    return f"""#pragma once
// ModeTable.h — 🔴 생성 파일. 손으로 고치지 않는다.  python3 deploy/scripts/gen_mode_table_header.py --write
//   학습 사실: mjlab_g1_motion/mode_spec.py @ {commit}
//   배포 사실: deploy/robots/g1/config/modes.yaml
// C++ 은 모드를 «번호» 로 비교하지 않고 이 표의 «성질» 을 묻는다 (rules/ADDING_A_MODE.md).
#include <array>

namespace mode_table {{

{enum_src}

inline constexpr int MASK_DIM = {ms.MASK_DIM};
inline constexpr int N_MODES = {len(train_ids)};

struct Row {{
  int id; const char* name; char key;
  std::array<float, MASK_DIM> bits;
  bool {", ".join(TRAIN_FLAGS)};
  RefSource ref_source; FootZ foot_z;
  bool arm_blend_enter, crossfade_enter;
  Safety safety; Exit exit;
  const char* gait_key;
}};

inline constexpr std::array<Row, N_MODES> ROWS = {{{{
{chr(10).join(rows)}
}}}};

inline constexpr bool valid(int mode) {{ return mode >= 1 && mode <= N_MODES; }}
// 범위 밖은 안전측(mode1 = 명령만으로 서서 걷는 모드)으로.
inline constexpr const Row& row(int mode) {{ return ROWS[valid(mode) ? mode - 1 : 0]; }}

}}  // namespace mode_table
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true"); g.add_argument("--write", action="store_true")
    ap.add_argument("--mjlab", type=pathlib.Path, default=pathlib.Path.home() / "mjlab1.4/mjlab_g1_mode45")
    a = ap.parse_args()
    text = build(a.mjlab)
    if a.write:
        HEADER.write_text(text); print(f"[gen] wrote {HEADER}"); return
    if not HEADER.exists():
        sys.exit("ModeTable.h 가 없다 — --write 로 생성할 것")
    # 🔴 «표» 와 «출처 줄» 을 따로 본다. mode_spec.py 에 주석만 고친 커밋이 생겨도 출처 해시는
    #    바뀌는데, 그걸 «표가 원장과 다르다» 로 실패시키면 배포 전 점검이 헛되이 빨개진다.
    def body(s: str) -> str:
        return "\n".join(l for l in s.splitlines() if PROVENANCE_TAG not in l)
    have = HEADER.read_text()
    if body(have) != body(text):
        sys.exit("ModeTable.h 가 원장(mode_spec.py + modes.yaml)과 다르다 — --write 로 다시 생성할 것")
    if have != text:
        print("[gen] ModeTable.h == 원장 (표 동일 · 출처 줄만 다름 — 다음 --write 때 갱신된다)")
    else:
        print("[gen] ModeTable.h == 원장")


if __name__ == "__main__":
    main()
