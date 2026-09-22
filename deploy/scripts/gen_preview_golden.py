#!/usr/bin/env python3
"""tests/golden_motion_preview.inc — mode4 미리보기의 골든을 «학습 함수 그 자체» 로 만든다.

학습 원장 = mjlab_g1_motion/tasks/g1_mimic_env.py G1MimicEnv.calc_future_motion_obs. 이 함수를 env 없이
가짜 self(SimpleNamespace)에 붙여 부른다 — 식을 여기에 다시 쓰지 않는다.
배포는 클립을 돌려 재생하므로(인덱스 mod n) 클립을 두 번 이어 붙여(tile) 학습 함수의 clamp 가 걸리지
않게 한다 = 학습 함수로 «mod n» 의미를 얻는다.

클립 = 실제 배포 클립의 구간 120 프레임(골반 = body 0). 이 구간의 원자료도 골든에 같이 싣는다
(C++ 테스트가 npz 없이 돈다).

mjlab 의 uv 환경 (conda deactivate 후):
  ~/.local/bin/uv run --project ~/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/scripts/gen_preview_golden.py --write | --check
"""
from __future__ import annotations

import argparse
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import torch

from mjlab_g1_motion.tasks.g1_mimic_env import TAR_MOTION_STEPS_PRIV, G1MimicEnv

REPO = pathlib.Path(__file__).resolve().parents[2]
GOLDEN = REPO / "deploy/robots/g1/tests/golden_motion_preview.inc"
CLIP = REPO / "deploy/robots/g1/config/policy/mimic_masked/260922_v1_m1gen_v2_torso_30k/params/g1_dance1_subject2_colmov2.npz"
START, F = 1200, 120
CURS = [0, 1, 7, 24, 60, 99, 118, 119]


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


def build() -> str:
    if not CLIP.exists():
        sys.exit(f"클립이 없다: {CLIP}")
    d = np.load(CLIP)
    sl = slice(START, START + F)
    jp = d["joint_pos"][sl].astype(np.float32)
    q = d["body_quat_w"][sl, 0].astype(np.float32)
    lin = d["body_lin_vel_w"][sl, 0].astype(np.float32)
    ang = d["body_ang_vel_w"][sl, 0].astype(np.float32)
    pos = d["body_pos_w"][sl, 0].astype(np.float32)
    tile = lambda a: torch.as_tensor(np.concatenate([a, a], 0))          # noqa: E731
    motion = SimpleNamespace(joint_pos=tile(jp), body_quat_w=tile(q)[:, None], body_lin_vel_w=tile(lin)[:, None],
                             body_ang_vel_w=tile(ang)[:, None], body_pos_w=tile(pos)[:, None], time_step_total=2 * F)
    if max(CURS) + max(TAR_MOTION_STEPS_PRIV) >= 2 * F:
        sys.exit("tile 이 짧다 — clamp 가 걸린다")
    cases = []
    for cur in CURS:
        mc = SimpleNamespace(motion=motion, time_steps=torch.tensor([cur]))
        fake = SimpleNamespace(_mc=lambda mc=mc: mc, _root_tracked_idx=0, _tar_steps=TAR_MOTION_STEPS_PRIV)
        prev = G1MimicEnv.calc_future_motion_obs(fake)[0].numpy()     # (20*35,) 학습 함수 그대로
        blk = np.concatenate([[1.0], prev[:35], prev])                  # [on][k1][preview] = calc_motion_* 배치
        cases.append(f"  {{{cur}, {{{', '.join(f32(x) for x in blk)}}}}},")
    arr = lambda a: ", ".join(f32(x) for x in a.reshape(-1))            # noqa: E731
    return f"""// 🔴 생성 파일. deploy/scripts/gen_preview_golden.py --write
//   학습 함수 G1MimicEnv.calc_future_motion_obs 를 그대로 부른 값. 클립 = {CLIP.name} [{START}:{START + F}] (골반 = body 0).
static const int kClipN = {F};
static const float kClipJp[{F}][29] = {{ {arr(jp)} }};
static const float kClipQ[{F}][4] = {{ {arr(q)} }};           // wxyz
static const float kClipLin[{F}][3] = {{ {arr(lin)} }};
static const float kClipAng[{F}][3] = {{ {arr(ang)} }};
static const float kClipZ[{F}] = {{ {arr(pos[:, 2])} }};
struct PreviewCase {{ int cur; float block[{1 + 35 + 20 * 35}]; }};
static const PreviewCase kGoldenPreview[] = {{
{chr(10).join(cases)}
}};
static const int kGoldenPreviewN = {len(CURS)};
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true"); g.add_argument("--write", action="store_true")
    a = ap.parse_args()
    text = build()
    if a.write:
        GOLDEN.write_text(text); print(f"[gen] wrote {GOLDEN}"); return
    if not GOLDEN.exists() or GOLDEN.read_text() != text:
        sys.exit("golden_motion_preview.inc 가 학습 함수 출력과 다르다 — --write")
    print("[gen] golden_motion_preview.inc == 학습 함수")


if __name__ == "__main__":
    main()
