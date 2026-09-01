# teleop/legacy — 지금은 안 쓰지만 «되살릴 조건» 이 분명한 것

## vr_relay_send.py / vr_relay_recv.py (2026-07-10)
com1 에서 도는 `vr_teleop_bridge.py --transport local`(xrt 직접 읽기 + 하드 E-stop) 이 자기 `/dev/shm/g1_vr_ref` 에
쓴 276 B VrRef 프레임을 ZMQ(:5557, CONFLATE) 로 Jetson 의 `/dev/shm/g1_vr_ref` 에 그대로 재현한다.
200 ms 무수신 → `valid=0` 을 써서 g1_ctrl 이 클립으로 복귀. 종료 시에도 `valid=0`.

**왜 legacy 인가**: 하드 E-stop 이 `--transport local` 전용이던 시절의 우회(«structure B»). 지금은 네트워크
transport 도 `--arm-estop` 으로 무장되므로 브릿지를 온보드에서 `--transport udp` 로 돌린다.

**되살릴 조건**: PC-Service/xrt 가 도는 **리눅스 PC(glibc ≥ 2.34)** 가 로봇 옆에 있고, 브릿지를 로봇이 아니라
그 PC 에서 돌리고 싶을 때. 이 두 파일은 `vr_shm.py` 의 레이아웃(`<iIii` + 65f = 276 B, magic 0x6702)을 그대로
따른다 — `vr_shm.py`/C++ `struct VrRef` 가 바뀌면 여기도 같이 바꿔야 한다.
