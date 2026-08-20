#!/usr/bin/env python3
"""/dev/shm/g1_vr_ref 레이아웃이 C++ `struct VrRef` 와 python `vr_shm.FMT` 사이에서 안 갈렸는지.

한쪽에만 필드를 더하면 컴파일도 되고 실행도 되는데 정책이 쓰레기 obs 를 먹는다.
길이로만 판별하는 채널이라(구버전 호환) 특히 조용히 틀어진다.

실행:  python3 deploy/robots/g1/tests/test_vr_shm_layout.py
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(G1 / "teleop"))

_SIZEOF = {"int32_t": 4, "uint32_t": 4, "float": 4}


def _cpp_struct_bytes() -> tuple[int, int]:
    """State_Mimic.cpp 의 packed struct VrRef 크기 (신, 구) 를 필드에서 계산한다."""
    src = (G1 / "src/State_Mimic.cpp").read_text(encoding="utf-8")
    m = re.search(r"struct VrRef\s*\{(.*?)\n\};", src, re.S)
    assert m, "State_Mimic.cpp 에서 struct VrRef 를 못 찾았다"
    total = 0
    for typ, _name, arr in re.findall(
            r"^\s*(int32_t|uint32_t|float)\s+(\w+)(?:\[(\d+)\])?\s*;", m.group(1), re.M):
        total += _SIZEOF[typ] * (int(arr) if arr else 1)
    legacy = re.search(r"VR_REF_LEGACY_BYTES\s*=\s*sizeof\(VrRef\)\s*-\s*sizeof\(float\)\s*\*\s*2",
                       src)
    assert legacy, "VR_REF_LEGACY_BYTES 정의가 예상과 다르다 — 손으로 확인할 것"
    return total, total - 8


def test_python_and_cpp_sizes_match():
    import vr_shm
    cpp_new, cpp_old = _cpp_struct_bytes()
    assert vr_shm.SIZE == cpp_new, f"신 레이아웃 python {vr_shm.SIZE} != C++ {cpp_new}"
    assert vr_shm.SIZE_V1 == cpp_old, f"구 레이아웃 python {vr_shm.SIZE_V1} != C++ {cpp_old}"
    assert cpp_new - cpp_old == 8, "확장분은 foot_z[2] = 8 바이트여야 한다"


def test_write_lengths():
    """foot_z 를 주면 신 길이, 생략하면 구 길이로 나가야 한다 (C++ 이 길이로 판별한다)."""
    import vr_shm
    z29 = [0.0] * 29
    v1 = struct.pack(vr_shm.FMT_V1, vr_shm.MAGIC, 1, 1, 3, *[0.0] * 3, *[1.0, 0, 0, 0], *z29, *z29)
    v2 = struct.pack(vr_shm.FMT, vr_shm.MAGIC, 1, 1, 3, *[0.0] * 3, *[1.0, 0, 0, 0], *z29, *z29,
                     0.07, 0.07)
    assert len(v1) == vr_shm.SIZE_V1 and len(v2) == vr_shm.SIZE
    # 앞부분은 바이트 동일 — 구버전 리더가 신 파일의 앞을 그대로 읽을 수 있어야 한다.
    assert v2[:len(v1)] == v1


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
