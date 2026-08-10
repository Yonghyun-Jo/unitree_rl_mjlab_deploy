"""
teleop_safety.py — 텔레옵 안전 감시(Watchdog + E-stop). com1 bridge 에 끼움.

두 가지:
  1) WATCHDOG → 안전 mode1
     - 하드 스톨: 마지막 수신 age_ms > stale_ms  (발행자 죽음/네트워크 끊김/케이블 빠짐)
     - 통신 불량("너무 튐"): 최근 1초 수신율 < min_rate_hz  또는  최대 프레임 간격 > max_gap_ms
     → 안전상태 = user_mode 1(base_vel 모드) + base_vel 0(제자리). 히스테리시스로 깜빡임 방지.
  2) E-STOP → PICO 우측 A 버튼(controllers.right.primary)
     - 누르면 래치(latched) 완전정지. 실수 해제 방지: 우측 menu 버튼 reset_hold_s 홀드로만 해제.

노트북은 수정 불필요(A/menu 버튼은 이미 UDP 스트림에 실려 옴).

사용(bridge):
    from teleop_safety import SafetyMonitor
    safety = SafetyMonitor()
    ...
    f = rx.latest()
    dec = safety.update(f, rx.age_ms())     # f 는 None 가능
    if dec["estop"]:
        user_mode = 1; base_vel = [0,0,0]; hard_stop = True     # 정책에 정지/damping
    elif dec["safe"]:
        user_mode = 1; base_vel = [0,0,0]                       # 통신불량 → 제자리
    else:
        ... 기존 정상 로직(버튼→user_mode, 스틱→base_vel) ...
"""
import time
from collections import deque


class SafetyMonitor:
    def __init__(self, stale_ms=200.0, min_rate_hz=30.0, max_gap_ms=120.0,
                 recover_cycles=25, window_s=1.0, reset_hold_s=1.0):
        self.stale_ms = stale_ms
        self.min_rate_hz = min_rate_hz
        self.max_gap_ms = max_gap_ms
        self.recover_cycles = recover_cycles
        self.window_s = window_s
        self.reset_hold_s = reset_hold_s

        self.estop = False           # 래치된 E-stop
        self._arrivals = deque()     # 최근 프레임 도착 monotonic 시각
        self._last_seq = None
        self._healthy_run = 0
        self._safe = False           # 워치독 안전상태(히스테리시스)
        self._menu_since = None      # 우측 menu 홀드 시작 시각(리셋용)
        self._last_reason = "init"

    def update(self, frame, age_ms, now=None):
        """매 제어 사이클 호출. frame: rx.latest() (None 가능). 반환 dict: estop/safe/mode/reason."""
        if now is None:
            now = time.monotonic()

        # 1) 새 프레임 도착 기록(seq 기준 dedup)
        if frame is not None and frame.get("seq") != self._last_seq:
            self._arrivals.append(now)
            self._last_seq = frame.get("seq")
        while self._arrivals and (now - self._arrivals[0]) > self.window_s:
            self._arrivals.popleft()

        # 2) E-stop: A(우측 primary) 누르면 래치
        a_btn = bool(frame and frame["controllers"]["right"]["primary"])
        if a_btn:
            self.estop = True

        # 3) E-stop 해제: 우측 menu 를 reset_hold_s 이상 홀드
        menu = bool(frame and frame["controllers"]["right"]["menu"])
        if self.estop and menu:
            if self._menu_since is None:
                self._menu_since = now
            elif (now - self._menu_since) >= self.reset_hold_s:
                self.estop = False
                self._menu_since = None
        else:
            self._menu_since = None

        # 4) 통신 건강도 (age = 현재 스톨, rate = 지속적 손실. 단발 hiccup 은 UDP/스무더가 흡수)
        rate = len(self._arrivals) / self.window_s if self._arrivals else 0.0
        max_gap_ms = self._max_gap_ms(now)   # 로그/가시성용
        hard_stale = age_ms > self.stale_ms
        unhealthy = hard_stale or (rate < self.min_rate_hz)

        # 5) 히스테리시스: 나빠지면 즉시 safe, 회복은 recover_cycles 연속 정상 필요
        if unhealthy:
            self._healthy_run = 0
            self._safe = True
            reason = ("STALE(age=%.0fms)" % age_ms if hard_stale
                      else "UNHEALTHY(rate=%.0fHz)" % rate)
        else:
            self._healthy_run += 1
            if self._safe and self._healthy_run >= self.recover_cycles:
                self._safe = False
            reason = "OK"

        # 6) 결정 (E-stop 최우선)
        if self.estop:
            self._last_reason = "ESTOP(A)"
            return {"estop": True, "safe": True, "mode": 1, "zero_base": True,
                    "reason": self._last_reason, "rate_hz": rate, "gap_ms": max_gap_ms}
        if self._safe:
            self._last_reason = "SAFE:" + reason
            return {"estop": False, "safe": True, "mode": 1, "zero_base": True,
                    "reason": self._last_reason, "rate_hz": rate, "gap_ms": max_gap_ms}
        self._last_reason = "OK"
        return {"estop": False, "safe": False, "mode": None, "zero_base": False,
                "reason": "OK", "rate_hz": rate, "gap_ms": max_gap_ms}

    def _max_gap_ms(self, now):
        if len(self._arrivals) < 2:
            return float("inf") if self._arrivals else float("inf")
        gaps = [(self._arrivals[i] - self._arrivals[i - 1]) for i in range(1, len(self._arrivals))]
        gaps.append(now - self._arrivals[-1])   # 현재 진행 중인 간격도 포함
        return max(gaps) * 1000.0


# ---- 하드 E-stop(/dev/shm/g1_estop) 정책 (순수함수 → 브릿지 main()에서 분리, 테스트 가능) ----

def resolve_estop_arming(transport, arm=None, on_watchdog=None):
    """(armed, estop_on_watchdog) 결정. None = auto = transport가 'local'일 때만 True.

    - armed: 하드 E-stop 채널 자체를 무장할지. 무장하면 A버튼 래치와 '브릿지 死 → 하트비트
      stale' fail-safe가 g1_ctrl 강제 Passive(damping)로 이어진다(EstopChannel.h).
      노트북 PICO로 실로봇을 돌리는 토폴로지(publisher=노트북, bridge+g1_ctrl=제어PC)는
      transport가 udp/zmq라 auto로는 무장되지 않으므로 --arm-estop 으로 명시 무장한다.
    - estop_on_watchdog: 워치독 안전상태(통신 스톨/저수신)까지 하드 Passive로 올릴지.
      local(온보드)은 통신 끊김 = 조작 불능이라 True가 맞지만, 네트워크 경로에서는 WiFi
      hiccup 하나가 곧장 주저앉기(damping)가 되어 오히려 위험 → auto=False(soft mode1 유지).
    """
    local = (transport == "local")
    return (local if arm is None else bool(arm),
            local if on_watchdog is None else bool(on_watchdog))


def estop_flag(dec, on_watchdog):
    """SafetyMonitor 결정 dict -> /dev/shm/g1_estop 의 flag(0/1).

    A버튼 E-stop 래치는 항상 1. 워치독 안전상태(dec['safe'])는 on_watchdog일 때만 1.
    (on_watchdog=True면 기존 동작 `dec['mode'] is not None`과 완전히 동일 — estop이 safe를 함의.)
    """
    return 1 if (dec.get("estop") or (on_watchdog and dec.get("safe"))) else 0


# ---- 자체 테스트 ----
def _selftest():
    print("[selftest] SafetyMonitor")
    sm = SafetyMonitor(stale_ms=200, min_rate_hz=30, max_gap_ms=120, recover_cycles=5)
    t = 0.0

    def frame(seq, a=False, menu=False):
        c = {"pose": [0] * 7, "trigger": 0, "grip": 0, "axis": [0, 0],
             "primary": False, "secondary": False, "menu": False, "axis_click": False}
        r = dict(c); r["primary"] = a; r["menu"] = menu
        return {"seq": seq, "streaming": True, "headset": [0] * 7,
                "controllers": {"left": dict(c), "right": r},
                "body": {"available": True, "joints": [[0] * 7] * 24}}

    # 정상 50Hz 흐름
    for i in range(40):
        t += 0.02
        d = sm.update(frame(i), age_ms=5, now=t)
    print("  정상흐름:", d["reason"], "-> safe=", d["safe"], "estop=", d["estop"])
    assert not d["safe"] and not d["estop"]

    # 스톨(새 프레임 없음, age 큼)
    for _ in range(3):
        t += 0.05
        d = sm.update(None, age_ms=400, now=t)
    print("  스톨:", d["reason"], "-> safe=", d["safe"], "mode=", d["mode"])
    assert d["safe"] and d["mode"] == 1

    # 회복(정상 프레임 recover_cycles 이상)
    seq = 100
    for _ in range(10):
        t += 0.02; seq += 1
        d = sm.update(frame(seq), age_ms=5, now=t)
    print("  회복:", d["reason"], "-> safe=", d["safe"])
    assert not d["safe"]

    # E-stop: A 누름
    t += 0.02; seq += 1
    d = sm.update(frame(seq, a=True), age_ms=5, now=t)
    print("  A눌림:", d["reason"], "-> estop=", d["estop"], "mode=", d["mode"])
    assert d["estop"]
    # A 떼도 래치 유지
    t += 0.02; seq += 1
    d = sm.update(frame(seq), age_ms=5, now=t)
    print("  A뗌(래치유지):", d["reason"], "-> estop=", d["estop"])
    assert d["estop"]
    # 우측 menu 1초 홀드 → 해제
    for _ in range(60):
        t += 0.02; seq += 1
        d = sm.update(frame(seq, menu=True), age_ms=5, now=t)
    print("  menu홀드 해제:", d["reason"], "-> estop=", d["estop"])
    assert not d["estop"]

    # 통신 불량(간헐 수신, 큰 갭)
    sm2 = SafetyMonitor(stale_ms=200, min_rate_hz=30, max_gap_ms=120)
    t = 0.0; seq = 0
    for _ in range(10):
        t += 0.20; seq += 1                     # 5Hz + 200ms 갭 → 불량
        d = sm2.update(frame(seq), age_ms=50, now=t)
    print("  저수신율/큰갭:", d["reason"], "-> safe=", d["safe"], "rate=%.0f gap=%.0f" % (d["rate_hz"], d["gap_ms"]))
    assert d["safe"]

    # ---- 하드 E-stop 무장 정책 ----
    print("[selftest] estop arming/flag")
    # auto: local만 무장 + 워치독까지 하드 (기존 동작 유지)
    assert resolve_estop_arming("local") == (True, True)
    assert resolve_estop_arming("udp") == (False, False)
    assert resolve_estop_arming("zmq") == (False, False)
    # 명시 override: 노트북 PICO(udp) + 실로봇 → 무장하되 워치독은 soft 유지(auto)
    assert resolve_estop_arming("udp", arm=True) == (True, False)
    assert resolve_estop_arming("udp", arm=True, on_watchdog=True) == (True, True)
    assert resolve_estop_arming("local", arm=False) == (False, True)   # 무장 해제(테스트용)

    OK = {"estop": False, "safe": False, "mode": None}
    WD = {"estop": False, "safe": True, "mode": 1}     # 워치독 안전상태
    ES = {"estop": True, "safe": True, "mode": 1}      # A버튼 래치
    assert estop_flag(OK, True) == 0 and estop_flag(OK, False) == 0
    assert estop_flag(ES, True) == 1 and estop_flag(ES, False) == 1    # A는 항상 하드
    assert estop_flag(WD, True) == 1                                   # 기존(local) 동작
    assert estop_flag(WD, False) == 0                                   # 네트워크: soft mode1 유지
    # on_watchdog=True는 기존 `dec["mode"] is not None`과 동치여야 함(회귀 방지)
    for dec in (OK, WD, ES):
        assert estop_flag(dec, True) == (1 if dec["mode"] is not None else 0)
    print("  arming/flag: OK")

    print("[selftest] ALL PASS")


if __name__ == "__main__":
    _selftest()
