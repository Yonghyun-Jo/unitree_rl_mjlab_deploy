#!/usr/bin/env python3
"""레퍼런스 발-z 의 «body 인덱스» 가 아직 발을 가리키는지 검사한다.

# 왜 이 테스트가 있나
mode>=3 의 foot_z obs 는 모션 npz 의 `body_pos_w[frame, idx, 2]` 에서 온다 (학습
mjlab_g1_motion 의 calc_ref_foot_height 가 FOOT_BODIES 의 world-z 를 쓰기 때문).
npz 에는 body «이름» 이 없어서 배포 코드가 인덱스를 숫자로 박을 수밖에 없다:

    include/State_Mimic.h   MotionLoader_::NPZ_FOOT_IDX = {6, 12}
    teleop/vr_replay.py     FOOT_BODY_IDX               = (6, 12)

로봇 에셋(g1.xml)의 body 가 하나만 늘거나 순서가 바뀌면 이 숫자가 «조용히» 다른 링크를
가리킨다 — 빌드도 통과하고 실행도 되고, 정책만 엉뚱한 높이를 먹는다. 그 사고를 여기서 막는다.

실행:  python3 deploy/robots/g1/tests/test_ref_foot_idx.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent          # deploy/robots/g1
REPO = G1.parent.parent.parent                        # unitree_rl_mjlab
XML = REPO / "src/assets/robots/unitree_g1/xmls/g1.xml"

# 학습(mjlab_g1_motion assets/robot_spec.py)의 FOOT_BODIES 와 같아야 한다.
EXPECTED_NAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
EXPECTED_NUM_BODIES = 30                              # world 제외


def _cpp_idx() -> tuple[int, ...]:
    src = (G1 / "include/State_Mimic.h").read_text(encoding="utf-8")
    m = re.search(r"NPZ_FOOT_IDX\[2\]\s*=\s*\{\s*(\d+)\s*,\s*(\d+)\s*\}", src)
    assert m, "State_Mimic.h 에서 NPZ_FOOT_IDX 를 못 찾았다"
    n = re.search(r"NPZ_NUM_BODIES\s*=\s*(\d+)", src)
    assert n, "State_Mimic.h 에서 NPZ_NUM_BODIES 를 못 찾았다"
    return (int(m.group(1)), int(m.group(2)), int(n.group(1)))


def _py_idx() -> tuple[int, ...]:
    src = (G1 / "teleop/vr_replay.py").read_text(encoding="utf-8")
    m = re.search(r"FOOT_BODY_IDX\s*=\s*\(\s*(\d+)\s*,\s*(\d+)\s*\)", src)
    assert m, "vr_replay.py 에서 FOOT_BODY_IDX 를 못 찾았다"
    n = re.search(r"NPZ_NUM_BODIES\s*=\s*(\d+)", src)
    assert n, "vr_replay.py 에서 NPZ_NUM_BODIES 를 못 찾았다"
    return (int(m.group(1)), int(m.group(2)), int(n.group(1)))


def test_cpp_and_python_agree():
    assert _cpp_idx() == _py_idx(), (
        f"C++ {_cpp_idx()} 와 python {_py_idx()} 가 다르다 — 두 곳이 같은 body 를 가리켜야 한다")


def test_index_points_at_the_feet():
    """모델을 실제로 열어 그 인덱스가 ankle_roll_link 인지 확인한다."""
    import mujoco as mj
    model = mj.MjModel.from_xml_path(str(XML))
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]
    assert names[0] == "world", f"첫 body 가 world 가 아니다: {names[0]}"
    bodies = names[1:]                                # npz body 축 = world 제외
    li, ri, nb = _cpp_idx()
    assert len(bodies) == nb == EXPECTED_NUM_BODIES, (
        f"body 수가 바뀌었다: 모델 {len(bodies)} vs 코드 {nb}. "
        f"NPZ_FOOT_IDX / FOOT_BODY_IDX 를 다시 정할 것.")
    assert (bodies[li], bodies[ri]) == EXPECTED_NAMES, (
        f"인덱스 ({li},{ri}) 가 {(bodies[li], bodies[ri])} 를 가리킨다 — "
        f"{EXPECTED_NAMES} 이어야 한다")


def test_shipped_motion_npz_shape():
    """배포 슬롯에 실린 모션 npz 들이 실제로 30-body 인지 (= 발-z 를 쓸 수 있는지)."""
    import numpy as np
    npzs = sorted((G1 / "config/policy").rglob("*.npz"))
    assert npzs, "배포 슬롯에 모션 npz 가 없다"
    bad = []
    for f in npzs:
        with np.load(f) as d:
            if "body_pos_w" not in d or d["body_pos_w"].shape[1] != EXPECTED_NUM_BODIES:
                bad.append(f.relative_to(G1))
    assert not bad, f"body_pos_w 가 없거나 {EXPECTED_NUM_BODIES}-body 가 아닌 npz: {bad}"


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            fn()
            print(f"  ok   {name}")
        except Exception as e:                        # noqa: BLE001
            fails += 1
            print(f"  FAIL {name}: {e}")
    print(("모두 통과" if not fails else f"{fails}개 실패"))
    sys.exit(1 if fails else 0)
