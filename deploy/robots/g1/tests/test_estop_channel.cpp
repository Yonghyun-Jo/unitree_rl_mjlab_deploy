// test_estop_channel.cpp — fsm_estop_poll 자체 검증 (기존 test_masked_loco_controller 관례).
//   빌드/실행:
//   cd deploy/robots/g1/tests
//   g++ -std=c++17 -I../../../include -O2 test_estop_channel.cpp -o /tmp/test_estop_channel && /tmp/test_estop_channel
#include "FSM/EstopChannel.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>

static const char* P = "/dev/shm/g1_estop_test";

static void write_frame(uint32_t seq, int32_t flag) {
    struct { int32_t magic; uint32_t seq; int32_t flag; } e{0x6703, seq, flag};
    FILE* f = std::fopen(P, "wb");
    std::fwrite(&e, sizeof(e), 1, f);
    std::fclose(f);
}

int main() {
    std::remove(P);
    EstopState st{};
    int fail = 0;
    auto chk = [&](bool got, bool want, const char* name) {
        if (got != want) { std::printf("FAIL %s: got=%d want=%d\n", name, got, want); ++fail; }
    };

    // 1) 파일 없음 -> 미무장(false), stale 리셋
    chk(fsm_estop_poll(st, P), false, "absent");

    // 2) 신선 seq + flag=1 -> asserted
    write_frame(1, 1);
    chk(fsm_estop_poll(st, P), true, "flag=1");

    // 3) 신선 seq + flag=0 -> 해제
    write_frame(2, 0);
    chk(fsm_estop_poll(st, P), false, "flag=0 fresh");

    // 4) seq 정지(하트비트 frozen): MAX 이하 폴은 false, 초과하면 asserted(dead-writer)
    for (int i = 0; i < ESTOP_STALE_MAX; ++i)
        chk(fsm_estop_poll(st, P), false, "frozen<=MAX");   // seq=2 그대로
    chk(fsm_estop_poll(st, P), true, "frozen>MAX -> stale assert");

    // 5) 회복: 신선 seq + flag=0 -> 해제 + stale 리셋
    write_frame(3, 0);
    chk(fsm_estop_poll(st, P), false, "recover");

    // 6) 손상(magic 틀림) -> false
    { FILE* f = std::fopen(P, "wb"); int32_t bad[3] = {0x1111, 9, 1}; std::fwrite(bad, sizeof(bad), 1, f); std::fclose(f); }
    chk(fsm_estop_poll(st, P), false, "bad magic");

    // 7) 누적된 stale을 0으로 리셋 (파일 부재 시):
    //    (a) 신선 seq + 폴링으로 stale 누적 (>0이지만 ≤MAX)
    write_frame(100, 0);
    chk(fsm_estop_poll(st, P), false, "setup_fresh");
    for (int i = 0; i < 3; ++i)
        chk(fsm_estop_poll(st, P), false, "accumulate_stale");  // stale -> 1,2,3
    //    (b) 파일 제거
    std::remove(P);
    //    (c) 폴링: false 반환 + stale 리셋 확인 (즉시 다시 폴링했을 때 stale이 0이어야 frozen 판정 안 됨)
    chk(fsm_estop_poll(st, P), false, "absent_resets_stale");
    //    (d) 신선 seq + 폴링: frozen 판정 안 나야 함 (stale이 리셋되었으므로 stale=1이지만, 이건 새로운 시작)
    write_frame(101, 0);
    chk(fsm_estop_poll(st, P), false, "after_reset_fresh");

    std::remove(P);
    if (fail) { std::printf("[test_estop_channel] %d FAIL\n", fail); return 1; }
    std::printf("[test_estop_channel] ALL PASS\n");
    return 0;
}
