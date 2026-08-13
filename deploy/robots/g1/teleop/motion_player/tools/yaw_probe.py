#!/usr/bin/env python3
"""yaw_probe.py — sim 로봇 pelvis yaw 를 주기적으로 찍어 mode2 재생의 yaw 드리프트를 잰다.

unitree_sdk2py 로 rt/lowstate 의 IMU quaternion 을 구독한다. 없으면 그 사실을 알리고 종료한다
(없는 계측을 지어내지 않는다).

    .venv/bin/python deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py --seconds 90
"""
from __future__ import annotations

import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--hz", type=float, default=5.0)
    args = ap.parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError:
        print("unitree_sdk2py 없음 — 이 계측은 수행할 수 없습니다.")
        print("대안: sim GUI 를 60s 녹화해 육안 판정하고 그 사실을 spec §12 O1 에 명시하세요.")
        return 2

    ChannelFactoryInitialize(0, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()

    t0 = time.perf_counter()
    print("t_s,yaw_rad")
    while time.perf_counter() - t0 < args.seconds:
        msg = sub.Read(200)
        if msg is not None:
            w, x, y, z = msg.imu_state.quaternion
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            print(f"{time.perf_counter() - t0:.2f},{yaw:.4f}", flush=True)
        time.sleep(1.0 / args.hz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
