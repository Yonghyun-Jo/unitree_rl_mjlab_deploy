"""xrt_receiver.py — PICO(xrobotoolkit_sdk)를 로컬에서 직접 읽어, 네트워크 receiver
(ZmqReceiver/UdpReceiver)와 동일한 frame dict를 내는 receiver. 온보드(co-located) 전용.

vr_teleop_bridge.py의 --transport local에서 사용. GMR/zmq를 import하지 않으므로
(xrt를 생성자 주입) 단위 테스트가 가벼움. body는 raw xyzw 그대로 emit — Unity->RH +
xyzw->wxyz 변환은 브릿지의 _msg_body_to_human이 담당(네트워크 경로와 동일).

⚠ xrt.init()은 호출측(브릿지)에서 이미 수행하고 xrt 모듈을 주입한다.
"""
from __future__ import annotations

import time


class XrtReceiver:
    def __init__(self, xrt):
        self._xrt = xrt
        self._seq = 0
        self._last_bts = None       # 마지막으로 관측한 body timestamp(ns)
        self._last_change = None    # bts가 바뀐 마지막 monotonic 시각

    def latest(self) -> dict:
        xrt = self._xrt
        self._seq += 1
        body_ok = bool(xrt.is_body_data_available())
        joints = None
        if body_ok:
            joints = [list(j) for j in xrt.get_body_joints_pose()]   # 24×7 raw xyzw

        # staleness 추적: body timestamp가 진행하면 살아있음
        bts = xrt.get_body_timestamp_ns()
        now = time.monotonic()
        if bts != self._last_bts:
            self._last_bts = bts
            self._last_change = now

        def ctrl(pose, trig, grip, axis, aclick, menu, primary, secondary):
            return {
                "pose": list(pose),
                "trigger": float(trig),
                "grip": float(grip),
                "axis": [float(axis[0]), float(axis[1])],
                "axis_click": bool(aclick),
                "menu": bool(menu),
                "primary": bool(primary),
                "secondary": bool(secondary),
            }

        return {
            "seq": self._seq,
            "streaming": body_ok,
            "headset": list(xrt.get_headset_pose()),
            "controllers": {
                "left": ctrl(xrt.get_left_controller_pose(), xrt.get_left_trigger(),
                             xrt.get_left_grip(), xrt.get_left_axis(), xrt.get_left_axis_click(),
                             xrt.get_left_menu_button(), xrt.get_X_button(), xrt.get_Y_button()),
                "right": ctrl(xrt.get_right_controller_pose(), xrt.get_right_trigger(),
                              xrt.get_right_grip(), xrt.get_right_axis(), xrt.get_right_axis_click(),
                              xrt.get_right_menu_button(), xrt.get_A_button(), xrt.get_B_button()),
            },
            "body": {"available": body_ok, "joints": joints},
        }

    def age_ms(self) -> float:
        """마지막으로 body timestamp가 바뀐 뒤 경과(ms). SDK 스톨 시 증가 -> 워치독이 감지.
        아직 아무것도 못 읽었으면 inf."""
        if self._last_change is None:
            return float("inf")
        return (time.monotonic() - self._last_change) * 1000.0

    def close(self) -> None:
        try:
            self._xrt.close()
        except Exception:
            pass
