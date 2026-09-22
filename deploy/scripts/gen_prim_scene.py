#!/usr/bin/env python3
"""scene_g1_prim.xml 을 만든다 — 로봇이 «학습과 같은 모양» 으로 부딪히는 sim2sim 장면.

  배포 사실  src/assets/robots/unitree_g1/xmls/scene_g1.xml   (읽기만 — 로봇 body·관절·액추에이터·센서·바닥)
  학습 사실  mjlab_g1_motion/assets/unitree_g1/g1_constants.py  get_spec() + FULL_COLLISION
             (= 학습 로봇 cfg get_g1_robot_cfg() 의 collisions=(FULL_COLLISION,) 와 같은 적용)

만드는 것 = scene_g1.xml 을 그대로 옮기되
  1. 로봇 body 아래의 충돌 geom(메시·발 코너 구·어깨 원기둥)은 전부 contype=0 conaffinity=0 (보이는 건 그대로)
  2. 학습 모델의 *_collision 구·캡슐 33개를 이름이 같은 body 에 — 값은 «컴파일된» 학습 모델에서 읽는다
     (종류·크기·위치·쿼터니언·μ·condim·priority·contype·conaffinity·solref·solimp·margin·gap·solmix).
     클래스 기본값에 기대지 않는다.
  3. 학습 g1.xml 의 자기충돌 제외 쌍(<contact><exclude>)도 옮긴다 — 없으면 골반↔엉덩이 등이 학습엔 없는 접촉을 낸다.
기존 파일은 어느 것도 고치지 않는다.

mujoco·mjlab_g1_motion 을 import 하므로 mjlab 의 uv 환경에서 돈다 (conda deactivate 후):
  ~/.local/bin/uv run --project ~/mjlab1.4/mjlab_g1_mode45 --no-sync python deploy/scripts/gen_prim_scene.py --write|--check
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

import mujoco

from mjlab_g1_motion.assets.unitree_g1.g1_constants import FULL_COLLISION, G1_XML, get_spec

REPO = pathlib.Path(__file__).resolve().parents[2]
XMLS = REPO / "src/assets/robots/unitree_g1/xmls"
SRC = XMLS / "scene_g1.xml"
OUT = XMLS / "scene_g1_prim.xml"
PROVENANCE_TAG = "원장 md5 "
N_PRIMS = 33
PRIM_GROUP = "3"                               # MuJoCo 뷰어 기본값에서 숨는 그룹 (학습 g1.xml collision 클래스와 같음)
PRIM_RGBA = "0.2 0.6 0.9 0.3"
# 종류별로 쓰는 size 칸 수. 나머지 칸은 0 이어야 한다(아니면 생성기 가정이 깨진 것).
NSIZE = {int(mujoco.mjtGeom.mjGEOM_SPHERE): 1, int(mujoco.mjtGeom.mjGEOM_CAPSULE): 2}
TYPE_NAME = {int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere", int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule"}


def train_model() -> tuple[mujoco.MjSpec, mujoco.MjModel]:
    """학습 로봇과 같은 충돌 설정의 모델. get_g1_robot_cfg() 가 spec_fn=get_spec · collisions=(FULL_COLLISION,)
    로 거는 것을 그대로 한다(mjlab Entity._apply_spec_editors → CollisionCfg.edit_spec)."""
    spec = get_spec()
    FULL_COLLISION.edit_spec(spec)
    return spec, spec.compile()


def r(x) -> str:
    """컴파일된 값을 그대로 되살리는 표기 (파이썬 float 의 repr = 왕복 보존)."""
    return repr(float(x))


def rs(v) -> str:
    return " ".join(r(x) for x in v)


def prims(m) -> list[int]:
    gid = [g for g in range(m.ngeom)
           if re.search(r"_collision$", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]
    types = {int(m.geom_type[g]) for g in gid}
    if len(gid) != N_PRIMS or not types <= set(NSIZE):
        sys.exit(f"학습 충돌 프리미티브 {N_PRIMS}개(구·캡슐)를 가정한다: {len(gid)}개, 종류 {types}")
    return gid


def parse(path: pathlib.Path) -> ET.ElementTree:
    # 주석(액추에이터 한계의 근거 등)을 잃지 않게 주석까지 읽는다.
    return ET.parse(path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))


def build() -> str:
    tree = parse(SRC)
    root = tree.getroot()
    if root.find(".//include") is not None:
        sys.exit("scene_g1.xml 이 <include> 를 쓴다 — 이 생성기는 한 파일 장면만 가정한다(합치기를 더할 것)")
    for d in root.iter("default"):
        if d.find("geom") is not None:
            sys.exit("scene_g1.xml 에 geom 기본값(<default><geom>)이 있다 — «속성 없음 = 충돌 1» 가정이 깨진다")
    if root.find("contact") is not None:
        sys.exit("scene_g1.xml 에 <contact> 가 이미 있다 — 학습 제외 쌍과 합치는 규칙을 먼저 정할 것")

    # 로봇 body = 월드바디 아래의 모든 body. 바닥(월드바디에 바로 붙은 geom)은 그대로 둔다.
    bodies: dict[str, ET.Element] = {}
    for wb in root.findall("worldbody"):
        for top in wb.findall("body"):
            for b in top.iter("body"):
                bodies[b.get("name")] = b
    n_off = 0
    for b in bodies.values():
        for g in b.findall("geom"):
            if g.get("contype", "1") != "0" or g.get("conaffinity", "1") != "0":
                g.set("contype", "0"); g.set("conaffinity", "0"); n_off += 1

    spec, m = train_model()
    bname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)  # noqa: E731
    for g in prims(m):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        body = bodies.get(bname(m.geom_bodyid[g]))
        if body is None:
            sys.exit(f"{name}: body '{bname(m.geom_bodyid[g])}' 가 scene_g1.xml 에 없다")
        t = int(m.geom_type[g]); k = NSIZE[t]
        if any(float(x) != 0.0 for x in m.geom_size[g][k:]):
            sys.exit(f"{name}: 쓰지 않는 size 칸이 0 이 아니다 {m.geom_size[g]}")
        el = ET.Element("geom", {
            "name": name, "type": TYPE_NAME[t], "size": rs(m.geom_size[g][:k]),
            "pos": rs(m.geom_pos[g]), "quat": rs(m.geom_quat[g]),
            "friction": rs(m.geom_friction[g]), "condim": str(int(m.geom_condim[g])),
            "priority": str(int(m.geom_priority[g])),
            "contype": str(int(m.geom_contype[g])), "conaffinity": str(int(m.geom_conaffinity[g])),
            "solref": rs(m.geom_solref[g]), "solimp": rs(m.geom_solimp[g]),
            "margin": r(m.geom_margin[g]), "gap": r(m.geom_gap[g]), "solmix": r(m.geom_solmix[g]),
            "group": PRIM_GROUP, "rgba": PRIM_RGBA,
        })
        kids = list(body)
        last_geom = max((i for i, c in enumerate(kids) if c.tag == "geom"), default=None)
        if last_geom is None:                   # geom 이 없는 body 면 inertial·joint·site 뒤, 자식 body 앞
            last_geom = max((i for i, c in enumerate(kids) if c.tag != "body"), default=-1)
        body.insert(last_geom + 1, el)

    excl = sorted((e.bodyname1, e.bodyname2) for e in spec.excludes)
    if excl:
        for b1, b2 in excl:
            if b1 not in bodies or b2 not in bodies:
                sys.exit(f"제외 쌍 {b1}–{b2}: body 가 scene_g1.xml 에 없다")
        contact = ET.Element("contact")
        for b1, b2 in excl:
            ET.SubElement(contact, "exclude", {"body1": b1, "body2": b2})
        ai = list(root).index(root.find("actuator"))  # 로봇 정의(worldbody) 뒤, 액추에이터 앞
        root.insert(ai, contact)

    ET.indent(tree, space="  ")
    md5 = lambda p: hashlib.md5(p.read_bytes()).hexdigest()[:12]  # noqa: E731
    head = (
        "<!-- 🔴 생성 파일: deploy/scripts/gen_prim_scene.py. 학습 충돌(구·캡슐 33) — mesh 충돌 장면은 scene_g1.xml -->\n"
        "<!-- 손으로 고치지 않는다. 생성기의 write 모드로만 다시 만들고 check 모드가 원장과 대조한다.\n"
        f"     로봇 메시·코너 구 충돌 {n_off}개 끔 · 학습 프리미티브 {N_PRIMS}개 · 제외 쌍 {len(excl)}개.\n"
        "     바닥은 priority 0, 프리미티브는 priority 1 이라 로봇-바닥 접촉의 μ 는 프리미티브 값(0.6)이다\n"
        "     (바닥 friction 을 바꿔도 안 먹는다 — 학습과 같은 규칙). -->\n"
        f"<!-- {PROVENANCE_TAG}scene_g1.xml {md5(SRC)} · 학습 g1.xml {md5(G1_XML)} -->\n"
    )
    return head + ET.tostring(tree.getroot(), encoding="unicode") + "\n"


def body_text(s: str) -> str:
    return "\n".join(l for l in s.splitlines() if PROVENANCE_TAG not in l)


def main() -> None:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true"); g.add_argument("--write", action="store_true")
    a = ap.parse_args()
    out = build()
    if a.write:
        OUT.write_text(out, encoding="utf-8"); print(f"[gen] wrote {OUT}")
        return
    if not OUT.exists() or body_text(OUT.read_text(encoding="utf-8")) != body_text(out):
        sys.exit(f"원장과 다르다 — --write 로 다시 생성할 것: {OUT}")
    print("[gen] scene_g1_prim.xml == 원장")


if __name__ == "__main__":
    main()
