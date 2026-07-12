"""test_xrt_receiver.py — XrtReceiver가 네트워크 receiver와 동일 shape/계약을 내는지 검증."""
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))   # teleop/
sys.path.insert(0, _HERE)                     # tests/ (_mock_xrt)
from _mock_xrt import MockXrt
from xrt_receiver import XrtReceiver


def main():
    m = MockXrt()
    rx = XrtReceiver(m)

    # 1) latest()는 항상 dict (None 아님), 필수 키 존재
    f = rx.latest()
    assert f is not None
    for k in ("seq", "streaming", "headset", "controllers", "body"):
        assert k in f, k
    # 2) seq는 호출마다 증가 (SafetyMonitor rate watchdog 요구)
    f2 = rx.latest()
    assert f2["seq"] == f["seq"] + 1, (f["seq"], f2["seq"])
    # 3) 하드-브래킷 키 전부 존재 (KeyError 방지): controllers.{left,right}.{primary,secondary,menu,axis_click,grip,axis}
    for side in ("left", "right"):
        c = f["controllers"][side]
        for k in ("primary", "secondary", "menu", "axis_click", "grip", "axis"):
            assert k in c, (side, k)
        assert isinstance(c["axis"], list) and len(c["axis"]) == 2
    # 4) 버튼 매핑: X->left.primary, Y->left.secondary, A->right.primary, B->right.secondary
    m.X = True; m.B = True
    f3 = rx.latest()
    assert f3["controllers"]["left"]["primary"] is True
    assert f3["controllers"]["right"]["secondary"] is True
    assert f3["controllers"]["left"]["secondary"] is False
    # 5) body raw passthrough(변환 없음): joint i의 값 == i
    assert f3["body"]["available"] is True
    assert f3["body"]["joints"][5] == [5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 1.0]
    # 6) body 미가용 -> streaming False, joints None
    m.body_ok = False
    f4 = rx.latest()
    assert f4["streaming"] is False and f4["body"]["available"] is False
    assert f4["body"]["joints"] is None
    # 7) age_ms: bts 정지하면 증가, bts 변하면 리셋
    m.body_ok = True
    rx.latest(); a0 = rx.age_ms()
    time.sleep(0.02)
    rx.latest()                      # bts 그대로 -> age 누적
    assert rx.age_ms() >= a0
    m._bts += 1
    rx.latest()
    assert rx.age_ms() < 5.0         # bts 변함 -> 방금 리셋
    # 8) close 위임
    rx.close()
    assert m.closed is True

    print("[test_xrt_receiver] ALL PASS")


if __name__ == "__main__":
    main()
