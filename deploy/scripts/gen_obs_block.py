#!/usr/bin/env python3
"""ONNX 메타데이터의 obs 계약 → deploy.yaml 의 `observations:` 블록 (stdout).

계약 문자열(학습 쪽 이름:차원:history, 순서 = 입력 배치)이 원장이다. 사람이 700칸짜리 scale 목록을
손으로 쓰지 않는다. 학습 이름 → 배포 관측 항(C++ REGISTER_OBSERVATION) 이름은 아래 표 하나가 정한다.
  python3 deploy/scripts/gen_obs_block.py <policy.onnx>  > /tmp/obs_block.yaml
`onnx` 가 없는 파이썬이면 mjlab 의 uv 환경으로 돌린다 (conda 는 먼저 끈다):
  ~/.local/bin/uv run --project /home/piene/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/scripts/gen_obs_block.py <policy.onnx>
"""
from __future__ import annotations

import sys

try:
    import onnx
except ImportError:
    sys.exit("이 파이썬에 onnx 가 없다 — mjlab 의 uv 환경으로 돌릴 것 (머리 주석의 두 번째 명령)")

# 학습 이름 → (배포 항 이름, params). mask 는 폭으로 가른다(v1 = 2칸 command_mask, v2 = 8칸 mode_mask).
DEPLOY_NAME = {
    "base_ang_vel": ("base_ang_vel", "{}"),
    "projected_gravity": ("projected_gravity", "{}"),
    "command": ("masked_joint_command", "{command_name: motion}"),
    "motion_root_ori_b": ("masked_root_ori_b", "{command_name: motion}"),
    "joint_pos": ("joint_pos_rel", "{}"),
    "joint_vel": ("joint_vel_rel", "{}"),
    "actions": ("last_action", "{}"),
    "base_vel": ("base_vel_command", "{}"),
    "foot_z": ("ref_foot_height", "{}"),
    "mode5_cmd": ("mode5_command", "{}"),
    "preview_on": ("motion_preview_on", "{}"),
    "preview_k1": ("motion_preview_k1", "{}"),
    "preview": ("motion_preview", "{}"),
}

ONES_PER_LINE = 20   # scale 목록 줄바꿈 (읽기·diff 용 — 값은 전부 1.0: policy 그룹에 항별 scale 이 없다)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    meta = {p.key: p.value for p in onnx.load(sys.argv[1], load_external_data=False).metadata_props}
    contract = meta.get("obs_contract")
    if not contract:
        sys.exit("ONNX 메타에 obs_contract 가 없다")
    print("observations:")
    for item in contract.split(","):
        name, dim, hist = item.strip().split(":")
        dim, hist = int(dim), int(hist)
        if name == "mask":
            deploy, params = ("command_mask" if dim == 2 else "mode_mask"), "{}"
        elif name in DEPLOY_NAME:
            deploy, params = DEPLOY_NAME[name]
        else:
            sys.exit(f"배포 항 이름을 모르는 학습 항 '{name}' — DEPLOY_NAME 표에 넣고 C++ 항을 만들 것")
        rows = [", ".join(["1.0"] * min(ONES_PER_LINE, dim - i)) for i in range(0, dim, ONES_PER_LINE)]
        ones = (",\n" + " " * 12).join(rows)
        print(f"  {deploy}:              # {dim} × history {hist}\n"
              f"    train_term: {name}\n    params: {params}\n    clip: null\n"
              f"    scale: [{ones}]\n    history_length: {hist}")


if __name__ == "__main__":
    main()
