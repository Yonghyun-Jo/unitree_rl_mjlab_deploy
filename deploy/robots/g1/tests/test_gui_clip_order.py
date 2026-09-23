#!/usr/bin/env python3
"""GUI 의 클립 버튼 번호 ↔ 제어기의 클립 칸이 같은가.

제어기는 번호만 받는다(shm clip_req · 키 `[`/`]`). GUI 가 만드는 번호가 제어기의 g_build_clips 순서와
어긋나면 «다른 클립이 재생된다» — 화면엔 아무 표시도 안 난다. 특히 2026-09-23 에 0번 칸이 합성 «stand»
(제어기가 짓는다, 설정 파일에 없다)로 바뀌면서 뒤 칸이 한 칸씩 밀렸다.

그래서 여기선 두 쪽을 각자의 «원천» 에서 읽어 맞춰 본다:
  제어기 = src/State_Mimic.cpp 의 g_build_clips() push_back 순서 (+ 어떤 설정 키가 있어야 그 칸이 생기나)
  GUI    = tools/masked_gui.py 의 CLIPS (config/config.yaml 을 읽어 만든 라벨)
표준 라이브러리만 쓴다(uv·viser 불필요).
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
G1 = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(G1, "tools"))

# 로더 심볼 → 그 칸이 생기려면 config.yaml 의 Mimic_Masked 에 있어야 하는 키 (없는 = 항상 생김)
NEEDS_KEY = {"motion_stand": None, "motion": "motion_file",
             "motion_light": "motion_file_light", "motion_demo6": "motion_file_demo6"}


def controller_slots() -> list:
    """State_Mimic.cpp 의 g_build_clips() 에서 (로더 심볼, 칸 이름) 을 «쓰인 순서대로»."""
    src = open(os.path.join(G1, "src", "State_Mimic.cpp"), encoding="utf-8").read()
    body = src[src.index("static void g_build_clips()"):]
    body = body[:body.index("\n}")]
    out = []
    for sym, name in re.findall(r"g_clips\.push_back\(\{State_Mimic::(\w+),\s*(G1_STAND_CLIP|\"[^\"]+\")", body):
        out.append((sym, "stand" if name == "G1_STAND_CLIP" else name.strip('"')))
    return out


def config_keys() -> set:
    """config/config.yaml 의 Mimic_Masked 블록에 실제로 있는 motion_file* 키."""
    keys, inside = set(), False
    for ln in open(os.path.join(G1, "config", "config.yaml"), encoding="utf-8").read().splitlines():
        if re.match(r"^  Mimic_Masked:", ln):
            inside = True
            continue
        if inside and ln.strip() and not ln.startswith("   "):
            break
        m = re.match(r"^    (motion_file(?:_light|_demo6)?):\s*\S", ln) if inside else None
        if m:
            keys.add(m.group(1))
    return keys


def main() -> int:
    slots = controller_slots()
    if not slots:
        print("FAIL: g_build_clips() 에서 칸을 하나도 못 읽었다 (함수가 바뀌었으면 이 테스트를 고칠 것)")
        return 1
    if slots[0][1] != "stand":
        print(f"FAIL: 제어기의 0번 칸이 «stand» 가 아니다 — {slots[0]}")
        return 1

    have = config_keys()
    expect = [name for sym, name in slots if NEEDS_KEY.get(sym, "?") is None or NEEDS_KEY.get(sym) in have]

    import masked_gui                                   # viser 를 import 하지 않는 모듈 수준 값만 읽는다
    labels = list(masked_gui.CLIPS)
    if len(labels) != len(expect):
        print(f"FAIL: 칸 수가 다르다 — 제어기 {len(expect)}개 {expect} / GUI {len(labels)}개 {labels}")
        return 1
    for i, (label, name) in enumerate(zip(labels, expect)):
        if not label.startswith(f"{i} "):
            print(f"FAIL: GUI {i}번 라벨이 번호로 시작하지 않는다 — {label!r} "
                  f"(제어기는 라벨 앞 숫자를 칸 번호로 받는다)")
            return 1
        if name == "stand" and "stand" not in label:
            print(f"FAIL: GUI 0번이 stand 칸이 아니다 — {label!r}")
            return 1
    print(f"ok: 클립 칸 {len(expect)}개 일치 — 제어기 {expect} / GUI {labels}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
