#pragma once
// ArcadeIntent.h — 아케이드 «의도» 칸 /dev/shm/g1_arcade_intent (magic 0x6707, 36 B). g1 전용(🅐 구역).
//   브라우저는 «의도»(이동 축 + 눌린 이벤트)만 보내고, 로봇 위 중계가 이 칸에 50 Hz 로 쓴다(tools/arcade_intent_shm.py
//   write_intent). g1_ctrl 의 정책 스레드가 틱마다 읽어 core 에 넘긴다. 칸 배치는 tools/arcade_intent_shm.py FMT 와 같은
//   커밋으로만 바꾼다(tests/test_arcade_shm_layout.py 가 정규식으로 대조 — 칸은 int32_t · uint32_t · float 만).
//
// 헤더 온리 · 순수 C++17 · core 헤더를 include 하지 않는다(아케이드 없는 빌드에서도 시험이 돈다). Event 비트는 아래 상수로 두고
// tests/test_arcade_shm.cpp 가 core 의 arcade::Event 와 같은지 대조한다(G1_HAVE_ARCADE 일 때만).
//
// poll_intent(frame, state) = 파일을 읽지 않는 순수 함수(무할당·무시스템콜). 파일 읽기는 read_intent_file 이 따로 한다
// (GuiPoll 의 fopen/fread 와 같은 방식). 둘 다 정책 스레드에서 50 Hz.
//
// 판정 규칙 (2A 안전 점검 A#2 · B HIGH-1 · B HIGH-2):
//   · 끊김(stale): seq 가 INTENT_STALE_TICKS(15) 정책 틱 안 바뀌거나 파일이 없으면(frame == nullptr · magic 다름) 이동 0 · run 0 ·
//     이벤트 버림. stale 중에 바뀐 event_seq 도 «먹은 것으로 치고» 버린다(이동 0 과 같은 원칙 — 되살아난 뒤 늦게 발동하지 않게).
//   · 이벤트: event_seq 가 바뀐 틱에 한 번만. 그 틱의 event_age_ms > INTENT_EVENT_MAX_AGE_MS(500) 이면 버린다(바뀐 event_seq 는 소비).
//     체류 시작(또는 처음 보는 프레임)의 event_seq 는 기준선만 잡는다 — 체류 전에 눌린 이벤트가 들어오며 재생되지 않게.
//   · 재무장(rearm): 체류 시작 · 끊김 뒤에는 세 축이 한 번 |·| < INTENT_NEUTRAL_EPS(0.05) 가 될 때까지 이동 0 · run 0.
//     중립을 본 틱도 이동 0 이고 그 다음 틱부터 값이 먹는다. 비유한 축이 있는 프레임은 중립으로 치지 않는다.
//   · 축은 [-1, 1] 로 clamp, 비유한은 0.
#include <cmath>
#include <cstdint>
#include <cstdio>

#pragma pack(push, 1)
struct ArcadeIntentFrame {       // tools/arcade_intent_shm.py FMT 와 같은 커밋으로만 바꾼다
    int32_t  magic;              // ARCADE_INTENT_MAGIC
    uint32_t seq;                // 쓰는 쪽이 50 Hz 로 +1 (하트비트)
    float    fwd, side, turn;    // 이동 축 [-1, 1]
    int32_t  run;                // 0/1
    uint32_t events;             // 이번 프레임에 새로 눌린 비트(INTENT_EV_*) = core Event
    uint32_t event_seq;          // 이벤트가 실린 프레임마다 +1
    uint32_t event_age_ms;       // 쓰는 쪽이 아는 이벤트 나이
};
#pragma pack(pop)
static_assert(sizeof(ArcadeIntentFrame) == 9 * 4, "ArcadeIntentFrame = arcade_intent_shm.FMT 9칸 36 바이트 (tests/test_arcade_shm_layout.py)");

static constexpr int32_t ARCADE_INTENT_MAGIC = 0x6707;   // 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay · 0x6706 상태 칸 · 0x6708 아케이드 상태 와 다르게
static constexpr int      INTENT_STALE_TICKS = 15;       // 0.3 s @ 50 Hz
static constexpr uint32_t INTENT_EVENT_MAX_AGE_MS = 500;
static constexpr float    INTENT_NEUTRAL_EPS = 0.05f;
static constexpr const char* ARCADE_INTENT_PATH = "/dev/shm/g1_arcade_intent";

// core 의 arcade::Event 와 같은 값(types.h: Jump 1 · Crawl 2 · Getup 4 · Lie 8 · Box 16)
static constexpr uint32_t INTENT_EV_JUMP = 1, INTENT_EV_CRAWL = 2, INTENT_EV_GETUP = 4, INTENT_EV_LIE = 8, INTENT_EV_BOX = 16;
static constexpr uint32_t INTENT_EV_ALL = INTENT_EV_JUMP | INTENT_EV_CRAWL | INTENT_EV_GETUP | INTENT_EV_LIE | INTENT_EV_BOX;

struct IntentPollState {
    bool     have_frame = false;    // 이 체류에서 유효 프레임을 본 적 있나(없으면 첫 프레임의 event_seq 는 기준선만)
    uint32_t last_seq = 0;
    int      stale_ticks = 0;       // 마지막으로 seq 가 바뀐 뒤의 poll 수(15 에서 멈춘다)
    uint32_t last_event_seq = 0;
    bool     armed = false;         // 재무장 완료(축이 중립을 한 번 봤다). 체류 시작 = false
};

// 체류 시작(State_Mimic::enter() — 정책 스레드가 뜨기 전)에 호출부가 부른다. «체류 시작» 을 알리는 방법이 이것 하나다.
inline void reset_for_stay_start(IntentPollState& st) { st = IntentPollState{}; }

struct IntentApply {
    float    fwd = 0.f, side = 0.f, turn = 0.f;   // core 에 넘길 값(재무장 전 · 끊김 중 = 0)
    int32_t  run = 0;
    uint32_t events = 0;                          // 이 틱에 core 로 보낼 이벤트 비트(대개 0)
    bool     stale = false;                       // 끊김 중(상태 칸 intent_stale)
    bool     rearmed = false;                     // 재무장 완료 상태(상태 칸 rearmed)
};

inline float intent_axis(float v) noexcept {
    if (!std::isfinite(v)) return 0.f;
    return v > 1.f ? 1.f : (v < -1.f ? -1.f : v);
}

// frame == nullptr 이면 «이번 틱엔 프레임이 없다»(파일 없음 · 짧음 · magic 다름 — read_intent_file 이 false 일 때).
inline IntentApply poll_intent(const ArcadeIntentFrame* f, IntentPollState& st) noexcept {
    IntentApply r;
    const bool valid = f != nullptr && f->magic == ARCADE_INTENT_MAGIC;
    if (!valid) {
        if (st.stale_ticks < INTENT_STALE_TICKS) ++st.stale_ticks;
    } else if (!st.have_frame || f->seq != st.last_seq) {
        st.last_seq = f->seq;
        st.stale_ticks = 0;
    } else if (st.stale_ticks < INTENT_STALE_TICKS) {
        ++st.stale_ticks;
    }
    r.stale = st.stale_ticks >= INTENT_STALE_TICKS;
    if (r.stale) st.armed = false;                      // 끊김 = 재무장 필요(되살아나도 중립을 한 번 봐야 한다)

    if (valid) {
        const bool first = !st.have_frame;
        const bool ev_changed = !first && f->event_seq != st.last_event_seq;
        st.have_frame = true;
        st.last_event_seq = f->event_seq;               // 적용·버림 모두 소비
        if (ev_changed && !r.stale && f->event_age_ms <= INTENT_EVENT_MAX_AGE_MS)
            r.events = f->events & INTENT_EV_ALL;
    }

    if (valid && !r.stale) {
        const bool finite = std::isfinite(f->fwd) && std::isfinite(f->side) && std::isfinite(f->turn);
        const float a = intent_axis(f->fwd), b = intent_axis(f->side), c = intent_axis(f->turn);
        if (st.armed) {
            r.fwd = a; r.side = b; r.turn = c; r.run = f->run != 0 ? 1 : 0;
        } else if (finite && std::fabs(a) < INTENT_NEUTRAL_EPS && std::fabs(b) < INTENT_NEUTRAL_EPS && std::fabs(c) < INTENT_NEUTRAL_EPS) {
            st.armed = true;                            // 중립을 본 틱은 이동 0 · 다음 틱부터
        }
    }
    r.rearmed = st.armed;
    return r;
}

// 파일 읽기(정책 스레드 선례 — GuiPoll 의 fopen/fread). 36 B 가 다 읽히고 magic 이 맞을 때만 true. 파일은 쓰는 쪽이 os.replace 로
// 통째 바꾸므로 열린 파일은 한 프레임이다(찢김 없음).
inline bool read_intent_file(const char* path, ArcadeIntentFrame& out) noexcept {
    FILE* fp = std::fopen(path ? path : ARCADE_INTENT_PATH, "rb");
    if (!fp) return false;
    ArcadeIntentFrame g{};
    const size_t n = std::fread(&g, 1, sizeof(g), fp);
    std::fclose(fp);
    if (n != sizeof(g) || g.magic != ARCADE_INTENT_MAGIC) return false;
    out = g;
    return true;
}
