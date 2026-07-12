"""test_estop_shm.py — estop_shm 바이트 레이아웃/원자성 자체 검증 (stdlib only)."""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import estop_shm


def main():
    # 테스트 격리: 실제 /dev/shm/g1_estop 대신 임시 경로로 덮어씀
    estop_shm.SHM_PATH = "/dev/shm/g1_estop_test"
    estop_shm.clear()

    # 1) write -> 12바이트 정확, 필드 파싱 일치
    estop_shm.write(7, 1)
    with open(estop_shm.SHM_PATH, "rb") as f:
        buf = f.read()
    assert len(buf) == 12, f"expected 12B, got {len(buf)}"
    magic, seq, flag = struct.unpack("<iIi", buf)
    assert magic == 0x6703 and seq == 7 and flag == 1, (magic, seq, flag)

    # 2) FMT/상수 계약 고정
    assert estop_shm.FMT == "<iIi" and estop_shm.MAGIC == 0x6703
    assert struct.calcsize(estop_shm.FMT) == 12

    # 3) clear -> 파일 제거, 재호출 무해
    estop_shm.clear()
    assert not os.path.exists(estop_shm.SHM_PATH)
    estop_shm.clear()  # FileNotFoundError 안 나야 함

    print("[test_estop_shm] ALL PASS")


if __name__ == "__main__":
    main()
