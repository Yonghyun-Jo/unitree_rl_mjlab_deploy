#pragma once
// EstopChannel.h — /dev/shm/g1_estop 판독(하트비트+flag). CtrlFSM::run_()가 폴링해
// asserted면 강제 Passive(damping). self-contained(FSM 프레임워크 의존 없음) → 단위 테스트 가능.
// Python 생산자: deploy/robots/g1/teleop/estop_shm.py (동일 12B <iIi> 레이아웃).
//
// 시맨틱: flag!=0(브릿지 SafetyMonitor가 E-stop/워치독 래치) 또는 하트비트 stale(브릿지 死,
// fail-safe) 시 true. 파일 없음 = 미무장(브릿지 미실행) = false. 래치 권한은 브릿지 한 곳,
// 여기선 미러 + dead-writer fail-safe만.
#include <cstdio>
#include <cstdint>

struct EstopState {
    uint32_t last_seq = 0;
    int      stale = 0;      // seq 정지가 지속된 폴 횟수
};

// Wire protocol: 12-byte frame (matches Python estop_shm.py <iIi> layout).
struct EstopFrame {
    int32_t magic;
    uint32_t seq;
    int32_t flag;
};
static_assert(sizeof(EstopFrame) == 12, "EstopFrame must be 12B to match estop_shm.py <iIi>");

// 폴 게이팅 전제: 호출측(CtrlFSM)이 ~50Hz로 호출(1kHz면 20틱마다). MAX=25 -> ~0.5s dead-writer.
static constexpr int ESTOP_STALE_MAX = 25;
static constexpr int32_t ESTOP_MAGIC = 0x6703;

inline bool fsm_estop_poll(EstopState& st, const char* path = "/dev/shm/g1_estop")
{
    FILE* f = std::fopen(path, "rb");
    if (!f) { st.stale = 0; return false; }            // 파일 없음 = 미무장
    EstopFrame e{};
    size_t n = std::fread(&e, sizeof(e), 1, f);
    std::fclose(f);
    if (n != 1 || e.magic != ESTOP_MAGIC) return false; // 짧은읽기/손상 무시
    if (e.seq == st.last_seq) {                         // 하트비트 frozen
        if (++st.stale > ESTOP_STALE_MAX) return true;  // writer 死 -> fail-safe assert
        return false;
    }
    // correctness는 writer가 seq를 1부터 발행(0을 라이브 프레임으로 안 씀)한다는 전제에 의존.
    // estop_shm.py/브릿지가 estop_seq=0에서 +=1 후 write하므로 첫 프레임 seq=1.
    // (last_seq default 0이라 seq=0 라이브 프레임은 frozen으로 오인됨.)
    st.last_seq = e.seq;
    st.stale = 0;
    return e.flag != 0;
}
