#!/usr/bin/env python3
"""scene_g1_prim.xml 이 학습의 충돌 모양(구·캡슐 33개 · μ 0.6 · condim 3 · priority 1)과 같고,
액추에이터·센서는 기존 scene_g1.xml 과 같은가. mujoco·mjlab 이 필요하다(mjlab uv 환경):
  ~/.local/bin/uv run --project ~/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/robots/g1/tests/test_prim_scene.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[4]
XMLS = REPO / "src/assets/robots/unitree_g1/xmls"


def _train_model():
    from mjlab_g1_motion.assets.unitree_g1.g1_constants import FULL_COLLISION, get_spec
    spec = get_spec()
    FULL_COLLISION.edit_spec(spec)          # 학습 로봇 cfg 가 거는 것과 같은 충돌 설정
    return spec.compile()


def _prims(m):
    out = {}
    for g in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if re.search(r"_collision$", n):
            out[n] = g
    return out


def _excludes(m):
    """<contact><exclude> 쌍을 body 이름 쌍(정렬)의 집합으로. signature = (b1 << 16) + b2."""
    bn = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)  # noqa: E731
    return {tuple(sorted((bn(int(s) >> 16), bn(int(s) & 0xFFFF)))) for s in m.exclude_signature[:m.nexclude]}


def main() -> int:
    tr = _train_model()
    pr = mujoco.MjModel.from_xml_path(str(XMLS / "scene_g1_prim.xml"))
    base = mujoco.MjModel.from_xml_path(str(XMLS / "scene_g1.xml"))
    fails = 0

    def chk(c, what):
        nonlocal fails
        print(f"  {'ok  ' if c else 'FAIL'} {what}")
        fails += (not c)

    tp, pp = _prims(tr), _prims(pr)
    chk(set(tp) == set(pp) and len(pp) == 33, f"프리미티브 33개 이름 일치 ({len(pp)})")
    for n in sorted(tp):
        a, b = tp[n], pp.get(n)
        if b is None:
            continue
        bn = lambda m, g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])  # noqa: E731
        same = (bn(tr, a) == bn(pr, b) and tr.geom_type[a] == pr.geom_type[b]
                and np.allclose(tr.geom_size[a], pr.geom_size[b], atol=1e-6)
                and np.allclose(tr.geom_pos[a], pr.geom_pos[b], atol=1e-6)
                and np.allclose(np.abs(tr.geom_quat[a] @ pr.geom_quat[b]), 1.0, atol=1e-6)
                and np.allclose(tr.geom_friction[a], pr.geom_friction[b], atol=1e-6)
                and tr.geom_condim[a] == pr.geom_condim[b] and tr.geom_priority[a] == pr.geom_priority[b]
                and tr.geom_contype[a] == pr.geom_contype[b] and tr.geom_conaffinity[a] == pr.geom_conaffinity[b]
                and np.allclose(tr.geom_solref[a], pr.geom_solref[b], atol=1e-9)
                and np.allclose(tr.geom_solimp[a], pr.geom_solimp[b], atol=1e-9)
                and np.isclose(tr.geom_margin[a], pr.geom_margin[b], atol=1e-9)
                and np.isclose(tr.geom_gap[a], pr.geom_gap[b], atol=1e-9)
                and np.isclose(tr.geom_solmix[a], pr.geom_solmix[b], atol=1e-9))
        if not same:
            chk(False, f"{n}: 학습과 다르다")
    # 로봇 = 월드(0) 밖의 모든 body. 프리미티브가 없는 body(허리·어깨 pitch/roll·손목 roll…)의
    # 메시 충돌도 여기서 잡힌다 — 프리미티브가 붙은 body 만 보면 그 body 들이 빠진다.
    robot_bodies = set(range(1, pr.nbody))
    extra = [g for g in range(pr.ngeom) if pr.geom_bodyid[g] in robot_bodies and g not in pp.values()
             and (pr.geom_contype[g] or pr.geom_conaffinity[g])]
    chk(not extra, f"로봇의 다른 geom 은 충돌 안 함 ({len(extra)}개 남음)")
    chk(_excludes(pr) == _excludes(tr), f"자기충돌 제외 쌍 = 학습 ({sorted(_excludes(pr))})")
    names = lambda m, t, k: [mujoco.mj_id2name(m, t, i) for i in range(k)]  # noqa: E731
    chk(names(pr, mujoco.mjtObj.mjOBJ_ACTUATOR, pr.nu) == names(base, mujoco.mjtObj.mjOBJ_ACTUATOR, base.nu),
        "액추에이터 이름·순서 = scene_g1.xml")
    chk(np.array_equal(pr.actuator_ctrlrange, base.actuator_ctrlrange), "액추에이터 ctrlrange = scene_g1.xml")
    # 충돌만 바꾼 장면이다 — 관절·구동 동역학이 조금이라도 다르면 두 장면의 sim2sim 비교가 무효가 된다(최종 검토 M-12).
    same = lambda a, b: a.shape == b.shape and np.array_equal(a, b)  # noqa: E731
    chk(same(pr.actuator_gainprm, base.actuator_gainprm) and same(pr.actuator_biasprm, base.actuator_biasprm)
        and same(pr.actuator_gaintype, base.actuator_gaintype) and same(pr.actuator_biastype, base.actuator_biastype),
        "액추에이터 gain·bias (종류·파라미터) = scene_g1.xml")
    chk(same(pr.actuator_forcerange, base.actuator_forcerange) and same(pr.actuator_forcelimited, base.actuator_forcelimited)
        and same(pr.actuator_ctrllimited, base.actuator_ctrllimited) and same(pr.actuator_gear, base.actuator_gear),
        "액추에이터 forcerange·gear·limited = scene_g1.xml")
    chk(names(pr, mujoco.mjtObj.mjOBJ_JOINT, pr.njnt) == names(base, mujoco.mjtObj.mjOBJ_JOINT, base.njnt)
        and same(pr.jnt_range, base.jnt_range) and same(pr.jnt_limited, base.jnt_limited),
        "관절 이름·순서·range·limited = scene_g1.xml")
    chk(same(pr.dof_damping, base.dof_damping) and same(pr.dof_armature, base.dof_armature)
        and same(pr.dof_frictionloss, base.dof_frictionloss),
        "dof damping·armature·frictionloss = scene_g1.xml")
    chk(names(pr, mujoco.mjtObj.mjOBJ_SENSOR, pr.nsensor) == names(base, mujoco.mjtObj.mjOBJ_SENSOR, base.nsensor),
        "센서 이름·순서 = scene_g1.xml")
    chk(pr.nq == base.nq and pr.nv == base.nv, "자유도 = scene_g1.xml")
    # 충돌 geom 을 더해도 질량·관성은 그대로여야 한다(모든 body 에 <inertial> 이 있어 geom 은 질량에 안 들어감).
    chk(pr.nbody == base.nbody and np.allclose(pr.body_mass, base.body_mass, atol=1e-9)
        and np.allclose(pr.body_inertia, base.body_inertia, atol=1e-9)
        and np.allclose(pr.body_ipos, base.body_ipos, atol=1e-9),
        "body 질량·관성 = scene_g1.xml")
    wg = lambda m: [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0]  # noqa: E731
    pw, bw = wg(pr), wg(base)
    chk(len(pw) == len(bw) and all(
        mujoco.mj_id2name(pr, mujoco.mjtObj.mjOBJ_GEOM, p) == mujoco.mj_id2name(base, mujoco.mjtObj.mjOBJ_GEOM, q)
        and np.array_equal(pr.geom_friction[p], base.geom_friction[q])
        and pr.geom_priority[p] == base.geom_priority[q] and pr.geom_condim[p] == base.geom_condim[q]
        for p, q in zip(pw, bw)), "바닥(월드 geom) = scene_g1.xml")
    print("PASS" if not fails else f"FAIL ({fails})")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
