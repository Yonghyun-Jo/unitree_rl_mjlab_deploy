"""
network_xr_client.py  (com1 에서 실행)

XRoboToolkit-Teleop 의 XrClient 와 '동일한 인터페이스'를 가지되, 로컬 xrobotoolkit_sdk
대신 노트북(PICO 캡처 노드)이 보내는 ZMQ 스트림에서 최신 프레임을 읽는 drop-in 대체.

원본 XrClient 는 좌표 변환을 하지 않으므로(원시 pose 그대로 반환), 노트북이 발행하는
원시 pose 를 그대로 돌려주면 리타게팅 컨트롤러 입장에서 완전히 호환된다.

토폴로지: com1(고정) 이 SUB 로 bind, 노트북(이동) PUB 이 connect. (pico_publisher 와 짝)

컨트롤러가 호출하는 메서드:
  get_pose_by_name / get_key_value_by_name / get_motion_tracker_data
  (+ 완전성을 위해 button/joystick/body/hand/timestamp/close 도 구현)
"""
import json
import threading

import numpy as np
import zmq

# 프레임 미수신 시 안전 기본값: 위치 0, 단위 쿼터니언(w=1)
_IDENTITY_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

# 버튼 이름 → (컨트롤러, 발행 메시지 키)
_BUTTON_MAP = {
    "A": ("right", "primary"),
    "B": ("right", "secondary"),
    "X": ("left", "primary"),
    "Y": ("left", "secondary"),
    "left_menu_button": ("left", "menu"),
    "right_menu_button": ("right", "menu"),
    "left_axis_click": ("left", "axis_click"),
    "right_axis_click": ("right", "axis_click"),
}
_KEY_MAP = {
    "left_trigger": ("left", "trigger"),
    "right_trigger": ("right", "trigger"),
    "left_grip": ("left", "grip"),
    "right_grip": ("right", "grip"),
}


class NetworkXrClient:
    """ZMQ 수신 기반 XrClient 대체. 인자 없이 생성 가능(원본과 동일 시그니처)."""

    def __init__(self, port: int = 5556, topic: str = ""):
        self._latest = None                 # 최신 프레임(dict). 참조 스왑만 하므로 락 최소화
        self._lock = threading.Lock()
        self._stop = threading.Event()

        ctx = zmq.Context.instance()
        self._sub = ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RCVHWM, 10)          # 최신 우선(밀리면 드롭)
        self._sub.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._sub.setsockopt(zmq.RCVTIMEO, 500)
        self._sub.bind(f"tcp://*:{port}")              # 고정 endpoint 로 bind

        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        print(f"[NetworkXrClient] SUB bind tcp://*:{port} (topic='{topic or 'all'}'). 노트북 발행자 대기...")

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                _topic, payload = self._sub.recv_multipart()
            except zmq.Again:
                continue
            except Exception:
                continue
            try:
                frame = json.loads(payload.decode())
            except Exception:
                continue
            with self._lock:
                self._latest = frame

    def _frame(self):
        with self._lock:
            return self._latest

    # --- XrClient 호환 API ---
    def get_pose_by_name(self, name: str) -> np.ndarray:
        f = self._frame()
        if f is None:
            return _IDENTITY_POSE.copy()
        if name == "headset":
            return np.asarray(f["headset"], dtype=float)
        if name == "left_controller":
            return np.asarray(f["controllers"]["left"]["pose"], dtype=float)
        if name == "right_controller":
            return np.asarray(f["controllers"]["right"]["pose"], dtype=float)
        raise ValueError(f"Invalid name: {name}")

    def get_key_value_by_name(self, name: str) -> float:
        f = self._frame()
        if name not in _KEY_MAP:
            raise ValueError(f"Invalid name: {name}")
        if f is None:
            return 0.0
        side, key = _KEY_MAP[name]
        return float(f["controllers"][side][key])

    def get_button_state_by_name(self, name: str) -> bool:
        f = self._frame()
        if name not in _BUTTON_MAP:
            raise ValueError(f"Invalid name: {name}")
        if f is None:
            return False
        side, key = _BUTTON_MAP[name]
        return bool(f["controllers"][side][key])

    def get_joystick_state(self, controller: str) -> list:
        f = self._frame()
        if f is None:
            return [0.0, 0.0]
        return list(f["controllers"][controller.lower()]["axis"])

    def get_motion_tracker_data(self) -> dict:
        # 트래커 미발행 → 빈 dict (컨트롤러가 안전하게 skip)
        return {}

    def get_hand_tracking_state(self, hand: str):
        return None  # 손 트래킹 미발행

    def get_body_tracking_data(self):
        f = self._frame()
        if f is None:
            return None
        body = f.get("body") or {}
        if not body.get("available"):
            return None
        return {"poses": np.asarray(body["joints"], dtype=float)}

    def get_timestamp_ns(self) -> int:
        f = self._frame()
        return int(f["sdk_ts_ns"]) if f else 0

    def is_streaming(self) -> bool:
        f = self._frame()
        return bool(f and f.get("streaming"))

    def close(self):
        self._stop.set()
        try:
            self._rx.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._sub.close(0)
        except Exception:
            pass
        print("[NetworkXrClient] closed.")
