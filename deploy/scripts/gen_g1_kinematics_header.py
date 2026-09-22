#!/usr/bin/env python3
"""G1Kinematics.h · tests/golden_g1_zfk.inc 를 «학습이 쓰는» g1.xml 에서 생성한다 (spec §4.3).

  학습 사실  mjlab_g1_motion/assets/unitree_g1/xmls/g1.xml  (get_spec() — 학습 env 와 같은 파일)
             mjlab_g1_motion/assets/robot_spec.py JOINT_ORDER (관절 순서 = 배포 관절 순서)

골든 = MuJoCo 자신의 FK(mj_kinematics)로 잰 z_fk. 무작위 자세 200개(관절 범위 안 · 롤·피치·요 전 범위).
C++ z_fk 는 이것과 1 mm 안이어야 한다.

mujoco 를 import 하므로 mjlab 의 uv 환경에서 돈다 (conda deactivate 후):
  ~/.local/bin/uv run --project ~/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/scripts/gen_g1_kinematics_header.py --write|--check
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

import mujoco
import numpy as np

from mjlab_g1_motion.assets import robot_spec as R
from mjlab_g1_motion.assets.unitree_g1.g1_constants import G1_XML, get_spec

REPO = pathlib.Path(__file__).resolve().parents[2]
HEADER = REPO / "deploy/robots/g1/include/G1Kinematics.h"
GOLDEN = REPO / "deploy/robots/g1/tests/golden_g1_zfk.inc"
PROVENANCE_TAG = "g1.xml md5 "
N_CASES = 200


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


def model() -> mujoco.MjModel:
    m = get_spec().compile()
    if m.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        sys.exit("첫 관절이 free 가 아니다")
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, m.njnt)]
    if names != list(R.JOINT_ORDER):
        sys.exit(f"g1.xml 관절 순서가 robot_spec.JOINT_ORDER 와 다르다:\n{names}\n{list(R.JOINT_ORDER)}")
    for j in range(1, m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE or np.abs(m.jnt_pos[j]).max() > 0:
            sys.exit(f"관절 {names[j - 1]}: hinge·원점 관절만 가정한다")
    return m


def collision_geoms(m) -> list[int]:
    gid = [g for g in range(m.ngeom)
           if re.search(r"_collision$", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
    types = {int(m.geom_type[g]) for g in gid}
    if len(gid) != 33 or not types <= {int(mujoco.mjtGeom.mjGEOM_SPHERE), int(mujoco.mjtGeom.mjGEOM_CAPSULE)}:
        sys.exit(f"충돌 프리미티브 33개(구·캡슐)를 가정한다: {len(gid)}개, 종류 {types}")
    return gid


def z_fk_mujoco(m, d, gid) -> float:
    low = np.inf
    for g in gid:
        c = d.geom_xpos[g]
        ax_z = d.geom_xmat[g].reshape(3, 3)[2, 2]
        half = m.geom_size[g][1] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0
        low = min(low, c[2] - half * abs(ax_z) - m.geom_size[g][0])
    return -low


def quat_rpy(roll, pitch, yaw) -> np.ndarray:
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) 의 wxyz."""
    def q(ax, a):
        v = np.zeros(4); v[0] = np.cos(a / 2); v[1 + ax] = np.sin(a / 2); return v
    out = np.zeros(4); tmp = np.zeros(4)
    mujoco.mju_mulQuat(tmp, q(2, yaw), q(1, pitch))
    mujoco.mju_mulQuat(out, tmp, q(0, roll))
    return out


def build() -> dict[pathlib.Path, str]:
    m = model()
    gid = collision_geoms(m)
    md5 = hashlib.md5(G1_XML.read_bytes()).hexdigest()[:12]
    bodies = []
    for b in range(1, m.nbody):                                   # 0 = world 는 뺀다 → pelvis = 0
        parent = int(m.body_parentid[b]) - 1
        nj = int(m.body_jntnum[b]); adr = int(m.body_jntadr[b])
        joint, axis = -1, np.zeros(3)
        if b > 1:
            if nj != 1:
                sys.exit(f"body {b}: 관절 1개를 가정한다 ({nj})")
            joint, axis = adr - 1, m.jnt_axis[adr]
        bodies.append(f"  {{{parent if b > 1 else -1}, {{{{{', '.join(f32(x) for x in m.body_pos[b])}}}}}, "
                      f"{{{{{', '.join(f32(x) for x in m.body_quat[b])}}}}}, {joint}, "
                      f"{{{{{', '.join(f32(x) for x in axis)}}}}}}},  // {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)}")
    geoms = []
    for g in gid:
        cap = m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE
        geoms.append(f"  {{{int(m.geom_bodyid[g]) - 1}, Prim::{'Capsule' if cap else 'Sphere'}, "
                     f"{{{{{', '.join(f32(x) for x in m.geom_pos[g])}}}}}, "
                     f"{{{{{', '.join(f32(x) for x in m.geom_quat[g])}}}}}, {f32(m.geom_size[g][0])}, "
                     f"{f32(m.geom_size[g][1] if cap else 0.0)}, "
                     f"\"{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)}\"}},")
    header = f"""#pragma once
// G1Kinematics.h — 🔴 생성 파일. 손으로 고치지 않는다.  deploy/scripts/gen_g1_kinematics_header.py --write
//   학습 사실: mjlab_g1_motion/assets/unitree_g1/xmls/g1.xml md5 {md5}
// 관절 트리(부모·위치·쿼터니언 wxyz·관절 축) + 충돌 프리미티브 33개. HeightEstimator.h 의 z_fk 가 쓴다.
// body 0 = pelvis. joint = 배포 관절 인덱스(0..28) = robot_spec.JOINT_ORDER 순서.
#include <array>

namespace g1kin {{

struct Body {{ int parent; std::array<float, 3> pos; std::array<float, 4> quat; int joint; std::array<float, 3> axis; }};
inline constexpr int N_BODY = {m.nbody - 1};
inline constexpr std::array<Body, N_BODY> BODIES = {{{{
{chr(10).join(bodies)}
}}}};

enum class Prim : unsigned char {{ Sphere, Capsule }};
struct Geom {{ int body; Prim type; std::array<float, 3> pos; std::array<float, 4> quat; float radius; float half_len; const char* name; }};
inline constexpr int N_GEOM = {len(gid)};
inline constexpr std::array<Geom, N_GEOM> GEOMS = {{{{
{chr(10).join(geoms)}
}}}};

}}  // namespace g1kin
"""
    rng = np.random.default_rng(0)
    d = mujoco.MjData(m)
    lo, hi = m.jnt_range[1:, 0], m.jnt_range[1:, 1]
    cases = []
    for _ in range(N_CASES):
        q = rng.uniform(lo, hi)
        quat = quat_rpy(rng.uniform(-np.pi, np.pi), rng.uniform(-np.pi / 2, np.pi / 2), rng.uniform(-np.pi, np.pi))
        d.qpos[:] = 0.0; d.qpos[3:7] = quat; d.qpos[7:] = q
        mujoco.mj_kinematics(m, d)
        cases.append(f"  {{{{{', '.join(f32(x) for x in q)}}}, {{{', '.join(f32(x) for x in quat)}}}, "
                     f"{f32(z_fk_mujoco(m, d, gid))}}},")
    golden = f"""// 🔴 생성 파일. deploy/scripts/gen_g1_kinematics_header.py --write  (g1.xml md5 {md5}, seed 0)
// MuJoCo mj_kinematics 로 잰 z_fk = −min(충돌 프리미티브 최저점 − 골반 원점 z).
struct ZfkCase {{ float q[29]; float quat[4]; float z; }};
static const ZfkCase kGoldenZfk[] = {{
{chr(10).join(cases)}
}};
static const int kGoldenZfkN = {N_CASES};
"""
    return {HEADER: header, GOLDEN: golden}


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
    print("[gen] G1Kinematics.h · golden_g1_zfk.inc == 원장")


if __name__ == "__main__":
    main()
