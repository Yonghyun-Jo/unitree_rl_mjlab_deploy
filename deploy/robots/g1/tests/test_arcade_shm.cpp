// test_arcade_shm.cpp — 아케이드 의도 칸(ArcadeIntent.h, 0x6707) 판정 + 아케이드 상태 칸(ArcadeStatus.h, 0x6708) seqlock.
//   intent_stale_zero_after_15_ticks · event_applied_once_per_event_seq · old_event_dropped ·
//   rearm_requires_neutral_after_stale_and_stay_start · nonfinite_axis_is_zero · stale_drops_event_seq(stale 중 바뀐 event_seq) ·
//   event_bits_match_core(G1_HAVE_ARCADE 일 때만) · read_intent_file(임시 경로 — 없음 · 36 B · 짧음 · 긺 37 B · 남의 magic) · status_open_failure_reasons(U4) · status_seqlock_never_torn(1e5) · status_write_allocates_nothing
//   시험 경로는 임시 폴더다 — 실제 /dev/shm/g1_arcade_* 를 건드리지 않는다.
//   g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include -pthread test_arcade_shm.cpp -o /tmp/t && /tmp/t
//   (core 대조: -DG1_HAVE_ARCADE -I<g1_arcade>/core/include)
#include "ArcadeIntent.h"
#include "ArcadeStatus.h"
#ifdef G1_HAVE_ARCADE
#include <arcade/types.h>
#endif

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <thread>

#include <unistd.h>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}

#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
static thread_local bool g_count_alloc = false;
static std::atomic<long> g_allocs{0};
// glibc 안쪽(fopen 의 FILE 등)의 malloc 도 세려고 malloc 자체를 가로챈다(operator new 는 malloc 을 부르므로 같이 센다).
extern "C" void* __libc_malloc(std::size_t);
extern "C" void* malloc(std::size_t n) {
    if (g_count_alloc) g_allocs.fetch_add(1, std::memory_order_relaxed);
    return __libc_malloc(n);
}
void* operator new(std::size_t n) {
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

static ArcadeIntentFrame mk(uint32_t seq, float fwd = 0.f, float side = 0.f, float turn = 0.f, int run = 0,
                            uint32_t ev = 0, uint32_t evseq = 0, uint32_t age = 0) {
    ArcadeIntentFrame f{};
    f.magic = ARCADE_INTENT_MAGIC; f.seq = seq; f.fwd = fwd; f.side = side; f.turn = turn; f.run = run;
    f.events = ev; f.event_seq = evseq; f.event_age_ms = age;
    return f;
}

// 새 체류에서 중립 프레임을 두 번 보내 재무장까지 끝낸 상태(seq 는 이어서 센다)
static uint32_t arm(IntentPollState& st, uint32_t seq) {
    reset_for_stay_start(st);
    ArcadeIntentFrame f = mk(seq);
    poll_intent(&f, st);
    f.seq = ++seq;
    const IntentApply r = poll_intent(&f, st);
    chk(r.rearmed, "arm: 중립 프레임으로 재무장");
    return seq;
}

static void intent_stale_zero_after_15_ticks() {
    IntentPollState st; uint32_t seq = arm(st, 100);
    ArcadeIntentFrame f = mk(seq, 0.5f, -0.25f, 0.75f, 1);
    f.seq = ++seq;
    IntentApply r = poll_intent(&f, st);
    chk(r.fwd == 0.5f && r.side == -0.25f && r.turn == 0.75f && r.run == 1 && !r.stale, "fresh 프레임은 그대로 먹는다");
    for (int i = 1; i <= INTENT_STALE_TICKS - 1; ++i) {            // seq 안 바뀜 14 틱 — 아직 먹는다
        r = poll_intent(&f, st);
        chk(!r.stale && r.fwd == 0.5f, "14 틱까지는 stale 아님");
    }
    r = poll_intent(&f, st);                                        // 15 번째 — 이동 0
    chk(r.stale && r.fwd == 0.f && r.side == 0.f && r.turn == 0.f && r.run == 0, "15 틱째 이동 0·run 0");
    r = poll_intent(nullptr, st);
    chk(r.stale && r.fwd == 0.f, "stale 은 이어진다");
    // 파일이 사라져도(nullptr) 15 틱 뒤 stale
    IntentPollState s2; uint32_t q = arm(s2, 7);
    f = mk(++q, 1.f); r = poll_intent(&f, s2);
    chk(r.fwd == 1.f, "파일 없음 시험 준비");
    for (int i = 0; i < INTENT_STALE_TICKS - 1; ++i) r = poll_intent(nullptr, s2);
    chk(!r.stale, "nullptr 14 틱");
    r = poll_intent(nullptr, s2);
    chk(r.stale && r.fwd == 0.f, "nullptr 15 틱째 stale");
    // magic 이 다른 프레임도 없는 것과 같다
    ArcadeIntentFrame bad = mk(5, 1.f); bad.magic = 0x6705;
    IntentPollState s3; for (int i = 0; i < INTENT_STALE_TICKS; ++i) r = poll_intent(&bad, s3);
    chk(r.stale, "남의 magic 은 프레임이 아니다");
}

static void event_applied_once_per_event_seq() {
    IntentPollState st; uint32_t seq = arm(st, 10);
    ArcadeIntentFrame f = mk(++seq, 0, 0, 0, 0, 0, 40);             // event_seq 40 으로 바뀜(이벤트 비트 0)
    poll_intent(&f, st);
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_JUMP, 41, 20);
    IntentApply r = poll_intent(&f, st);
    chk(r.events == INTENT_EV_JUMP, "event_seq 가 바뀐 틱에 이벤트 한 번");
    f.seq = ++seq;                                                  // 같은 event_seq 로 하트비트만
    r = poll_intent(&f, st);
    chk(r.events == 0, "같은 event_seq 는 다시 안 먹는다");
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_CRAWL | INTENT_EV_BOX, 42, 0);
    r = poll_intent(&f, st);
    chk(r.events == (INTENT_EV_CRAWL | INTENT_EV_BOX), "새 event_seq 의 비트 묶음");
    f = mk(++seq, 0, 0, 0, 0, 0x80 | INTENT_EV_LIE, 43, 0);
    r = poll_intent(&f, st);
    chk(r.events == INTENT_EV_LIE, "모르는 비트는 거른다");
    // 체류 시작 직후 처음 보는 프레임의 event_seq 는 기준선만(체류 전에 눌린 이벤트가 재생되지 않는다)
    IntentPollState s2; reset_for_stay_start(s2);
    f = mk(1, 0, 0, 0, 0, INTENT_EV_GETUP, 9, 10);
    r = poll_intent(&f, s2);
    chk(r.events == 0, "첫 프레임의 event_seq 는 기준선");
}

static void old_event_dropped() {
    IntentPollState st; uint32_t seq = arm(st, 10);
    ArcadeIntentFrame f = mk(++seq, 0, 0, 0, 0, 0, 5); poll_intent(&f, st);
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_JUMP, 6, INTENT_EVENT_MAX_AGE_MS + 1);
    IntentApply r = poll_intent(&f, st);
    chk(r.events == 0, "501 ms 이벤트는 버린다");
    f.seq = ++seq; r = poll_intent(&f, st);
    chk(r.events == 0, "버린 이벤트는 소비된 것(다음 틱에 되살아나지 않는다)");
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_JUMP, 7, INTENT_EVENT_MAX_AGE_MS);
    r = poll_intent(&f, st);
    chk(r.events == INTENT_EV_JUMP, "500 ms 는 먹는다(경계 포함)");
}

static void stale_drops_event_seq() {
    IntentPollState st; uint32_t seq = arm(st, 10);
    ArcadeIntentFrame f = mk(++seq, 0, 0, 0, 0, 0, 1); poll_intent(&f, st);
    IntentApply r;
    for (int i = 0; i < INTENT_STALE_TICKS; ++i) r = poll_intent(&f, st);
    chk(r.stale, "stale 준비");
    f.events = INTENT_EV_JUMP; f.event_seq = 2; f.event_age_ms = 0;  // seq 는 그대로인데 event_seq 만 바뀜 → stale 중이라 버린다
    r = poll_intent(&f, st);
    chk(r.stale && r.events == 0, "stale 중 바뀐 event_seq 는 버린다");
    f.seq = ++seq;                                                  // 하트비트가 돌아온 틱 — 그 event_seq 는 이미 소비됨
    r = poll_intent(&f, st);
    chk(!r.stale && r.events == 0, "되살아난 틱에 낡은 이벤트가 터지지 않는다");
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_CRAWL, 3, 0);
    r = poll_intent(&f, st);
    chk(r.events == INTENT_EV_CRAWL, "그 뒤 새 이벤트는 먹는다");
}

static void rearm_requires_neutral_after_stale_and_stay_start() {
    IntentPollState st; reset_for_stay_start(st);
    // 체류 시작 — 키를 누른 채 들어온 경우
    uint32_t seq = 1;
    ArcadeIntentFrame f = mk(seq, 1.f, 0.f, 0.f, 1);
    IntentApply r = poll_intent(&f, st);
    chk(r.fwd == 0.f && r.run == 0 && !r.rearmed, "체류 시작: 축이 눌려 있으면 이동 0");
    f.seq = ++seq; r = poll_intent(&f, st);
    chk(r.fwd == 0.f && !r.rearmed, "계속 눌려 있으면 계속 0");
    f = mk(++seq, 0.049f, -0.049f, 0.f, 1);                         // 한 틱 중립
    r = poll_intent(&f, st);
    chk(r.fwd == 0.f && r.rearmed, "중립을 본 틱은 이동 0 이지만 재무장 완료");
    f = mk(++seq, 0.6f, 0.f, 0.f, 1);
    r = poll_intent(&f, st);
    chk(r.fwd == 0.6f && r.run == 1 && r.rearmed, "그 다음 틱부터 먹는다");
    f = mk(++seq, 0.049f, 0.f, 0.f);                                // 한 번 무장하면 다시 눌러도 유지
    poll_intent(&f, st);
    f = mk(++seq, 0.f, 0.05f, 0.f);                                 // 0.05 정확히는 중립이 아니다(|·| < 0.05)
    r = poll_intent(&f, st);
    chk(r.side == 0.05f, "무장 뒤에는 그대로");
    // 끊김 뒤
    for (int i = 0; i < INTENT_STALE_TICKS; ++i) r = poll_intent(&f, st);
    chk(r.stale && !r.rearmed, "끊김 = 재무장 해제");
    f = mk(++seq, 0.f, 0.9f, 0.f, 1);                               // 하트비트 재개, 축은 눌려 있다
    r = poll_intent(&f, st);
    chk(!r.stale && r.side == 0.f && r.run == 0 && !r.rearmed, "끊김 뒤 눌려 있으면 이동 0");
    f = mk(++seq, 0.f, 0.05f, 0.f);                                 // 0.05 는 중립 아님
    r = poll_intent(&f, st);
    chk(!r.rearmed, "|축| = 0.05 는 중립이 아니다");
    f = mk(++seq, 0.f, 0.f, 0.f);
    r = poll_intent(&f, st);
    chk(r.rearmed && r.side == 0.f, "중립 → 재무장");
    f = mk(++seq, 0.f, 0.9f, 0.f, 1);
    r = poll_intent(&f, st);
    chk(r.side == 0.9f && r.run == 1, "재무장 뒤 이동");
    // reset_for_stay_start 가 무장을 지운다
    reset_for_stay_start(st);
    f = mk(++seq, 0.f, 0.9f, 0.f, 1);
    r = poll_intent(&f, st);
    chk(r.side == 0.f && !r.rearmed, "새 체류는 다시 재무장해야 한다");
}

static void nonfinite_axis_is_zero() {
    IntentPollState st; uint32_t seq = arm(st, 10);
    const float nan = std::nanf(""), inf = INFINITY;
    ArcadeIntentFrame f = mk(++seq, nan, inf, -inf, 1);
    IntentApply r = poll_intent(&f, st);
    chk(r.fwd == 0.f && r.side == 0.f && r.turn == 0.f, "비유한 축 = 0 (무장 뒤)");
    chk(r.run == 1, "run 은 축과 별개");
    f = mk(++seq, 3.f, -7.f, 0.5f, 1);
    r = poll_intent(&f, st);
    chk(r.fwd == 1.f && r.side == -1.f && r.turn == 0.5f, "[-1,1] clamp");
    f = mk(++seq, nan, 0.4f, 0.f, 1);                               // 한 축만 비유한 — 그 축만 0
    r = poll_intent(&f, st);
    chk(r.fwd == 0.f && r.side == 0.4f, "비유한은 그 축만 0");
    // 재무장 전: 비유한 프레임은 중립으로 치지 않는다
    IntentPollState s2; reset_for_stay_start(s2);
    f = mk(1, nan, 0.f, 0.f);
    r = poll_intent(&f, s2);
    chk(!r.rearmed && r.fwd == 0.f, "비유한 프레임으로 재무장되지 않는다");
}


// 끈적한 이벤트(F2): 쓰는 쪽이 같은 비트·event_seq 를 하트비트마다 싣고 age 만 는다. 읽는 쪽 규칙은 그대로.
static void sticky_event_read_rules() {
    // (b) 이벤트를 실은 첫 프레임을 건너뛰고 다음 프레임에서 처음 읽어도 정확히 한 번 적용
    IntentPollState st; uint32_t seq = arm(st, 10);
    ArcadeIntentFrame f = mk(++seq, 0, 0, 0, 0, 0, 5); poll_intent(&f, st);      // 기준선 event_seq 5
    // (이벤트 프레임 mk(++seq, …, JUMP, 6, 0) 은 읽지 않고 건너뛴다)
    ++seq;
    f = mk(++seq, 0, 0, 0, 0, INTENT_EV_JUMP, 6, 20);                            // 같은 이벤트가 다음 프레임에도 실려 있다(age 20)
    IntentApply r = poll_intent(&f, st);
    chk(r.events == INTENT_EV_JUMP, "(b) 첫 프레임을 건너뛰어도 다음 프레임에서 한 번 적용");
    // (c) 같은 event_seq 를 계속 읽어도 두 번 적용 안 됨
    int applied = 0;
    for (int i = 0; i < 20; ++i) { f.seq = ++seq; f.event_age_ms += 20; applied += poll_intent(&f, st).events != 0 ? 1 : 0; }
    chk(applied == 0, "(c) 같은 event_seq 는 다시 안 먹는다");
    // (d) 나이가 500 을 넘은 뒤 처음 읽으면 버림(그 뒤에도 안 터짐)
    ArcadeIntentFrame g = mk(++seq, 0, 0, 0, 0, INTENT_EV_CRAWL, 7, 501);
    r = poll_intent(&g, st);
    chk(r.events == 0, "(d) 501 ms 가 된 뒤 처음 읽으면 버림");
    g.seq = ++seq; g.event_age_ms = 520;
    chk(poll_intent(&g, st).events == 0, "(d) 버린 뒤에도 안 터진다");
}

// F4: 체류 시작 리셋 뒤 첫 프레임은 기준선만 — 하트비트로 치지 않는다
static void stay_start_first_frame_is_baseline() {
    IntentPollState st; uint32_t seq = arm(st, 50);
    ArcadeIntentFrame f = mk(++seq, 0.8f, 0, 0, 1, INTENT_EV_JUMP, 9, 10); poll_intent(&f, st);   // 끈적한 이벤트가 실린 채
    reset_for_stay_start(st);
    // 리셋 뒤 같은 seq 만 계속 온다(죽은 중계가 남긴 낡은 프레임) → 이동 0 · 이벤트 0 · 15 번째 poll 에서 stale
    IntentApply r;
    bool any_move = false, any_event = false;
    for (int i = 1; i <= INTENT_STALE_TICKS; ++i) {
        r = poll_intent(&f, st);
        any_move = any_move || r.fwd != 0.f || r.run != 0;
        any_event = any_event || r.events != 0;
        chk(r.stale == (i >= INTENT_STALE_TICKS), "stale 은 리셋 뒤 15 번째 poll 에서");
    }
    chk(!any_move, "낡은 프레임이면 이동 0");
    chk(!any_event, "끈적한 이벤트가 실린 채 리셋돼도 적용 안 됨");
    // 새 seq 가 와야 (재무장 후) 이동
    f = mk(++seq); r = poll_intent(&f, st);
    chk(!r.stale && r.rearmed, "새 seq 중립 → 재무장");
    f = mk(++seq, 0.5f, 0, 0, 1); r = poll_intent(&f, st);
    chk(r.fwd == 0.5f, "새 seq 가 오면 이동");
}

static void read_intent_file_allocates_nothing() {
    char dir[] = "/tmp/g1_arcade_shm_test_XXXXXX";
    if (!mkdtemp(dir)) { chk(false, "mkdtemp"); return; }
    const std::string p = std::string(dir) + "/intent";
    ArcadeIntentFrame f = mk(1, 0.5f), out{};
    FILE* fp = std::fopen(p.c_str(), "wb"); std::fwrite(&f, 1, sizeof f, fp); std::fclose(fp);
    g_allocs.store(0); g_count_alloc = true;
    bool ok = true;
    for (int i = 0; i < 1000; ++i) ok = read_intent_file(p.c_str(), out) && ok;
    g_count_alloc = false;
    const long allocs = g_allocs.load();
    chk(ok, "읽기 1000 회 성공");
    chk(allocs == 0, "read_intent_file 1000 회 할당 0");
    std::printf("  read_intent_file: 1000 회 할당 %ld\n", allocs);
    std::remove(p.c_str()); ::rmdir(dir);
}

static void event_bits_match_core() {
#ifdef G1_HAVE_ARCADE
    chk(INTENT_EV_JUMP == static_cast<uint32_t>(arcade::Event::Jump) && INTENT_EV_CRAWL == static_cast<uint32_t>(arcade::Event::Crawl) &&
        INTENT_EV_GETUP == static_cast<uint32_t>(arcade::Event::Getup) && INTENT_EV_LIE == static_cast<uint32_t>(arcade::Event::Lie) &&
        INTENT_EV_BOX == static_cast<uint32_t>(arcade::Event::Box), "event 비트 == core arcade::Event");
#else
    std::printf("  skip event_bits_match_core (core 없음 — G1_HAVE_ARCADE 아님)\n");
#endif
}

static void read_intent_file_roundtrip() {
    char dir[] = "/tmp/g1_arcade_shm_test_XXXXXX";
    if (!mkdtemp(dir)) { chk(false, "mkdtemp"); return; }
    const std::string p = std::string(dir) + "/intent";
    ArcadeIntentFrame out{}, f = mk(77, 0.25f, -0.5f, 1.f, 1, INTENT_EV_BOX, 9, 12);
    chk(!read_intent_file(p.c_str(), out), "파일 없음 = false");
    FILE* fp = std::fopen(p.c_str(), "wb"); std::fwrite(&f, 1, sizeof f, fp); std::fclose(fp);
    chk(read_intent_file(p.c_str(), out) && std::memcmp(&out, &f, sizeof f) == 0, "36 B 프레임을 그대로 읽는다");
    fp = std::fopen(p.c_str(), "wb"); std::fwrite(&f, 1, sizeof f - 1, fp); std::fclose(fp);
    chk(!read_intent_file(p.c_str(), out), "짧은 파일 = false");
    {   // U3(검토 M2): 긴 파일(맞는 36 B 프레임 + 1 B) — 앞 36 B 를 프레임으로 치지 않는다
        const unsigned char extra = 0;
        fp = std::fopen(p.c_str(), "wb"); std::fwrite(&f, 1, sizeof f, fp); std::fwrite(&extra, 1, 1, fp); std::fclose(fp);
        ArcadeIntentFrame keep = mk(5);
        out = keep;
        chk(!read_intent_file(p.c_str(), out) && std::memcmp(&out, &keep, sizeof keep) == 0, "37 B 파일 = false(out 그대로)");
        fp = std::fopen(p.c_str(), "wb");
        for (int k = 0; k < 4; ++k) std::fwrite(&f, 1, sizeof f, fp);   // 프레임 넷이 이어 붙은 144 B
        std::fclose(fp);
        chk(!read_intent_file(p.c_str(), out), "144 B 파일(프레임 넷) = false");
    }
    f.magic = 0x6704;
    fp = std::fopen(p.c_str(), "wb"); std::fwrite(&f, 1, sizeof f, fp); std::fclose(fp);
    chk(!read_intent_file(p.c_str(), out), "남의 magic = false");
    std::remove(p.c_str()); ::rmdir(dir);
}

// ── 상태 칸 ────────────────────────────────────────────────────────────────────────────────────────────────
using g1::arcade_status::ArcadeStatusWriter;
using g1::arcade_status::read_words;

static ArcadeStatus fill(uint32_t k) {
    ArcadeStatus s{};
    uint32_t* w = reinterpret_cast<uint32_t*>(&s);
    for (int i = 0; i < g1::arcade_status::N_WORDS; ++i) w[i] = k;   // 모든 칸이 같은 k (float 칸은 비트로 같다)
    return s;
}

struct TmpShm {
    char dir[64]; std::string path;
    TmpShm() { std::snprintf(dir, sizeof dir, "/dev/shm/g1_arcade_shm_test_XXXXXX"); if (!mkdtemp(dir)) dir[0] = 0; path = std::string(dir) + "/status"; }
    ~TmpShm() { std::remove(path.c_str()); if (dir[0]) ::rmdir(dir); }
};

// U4(검토 M10): 열기 실패 · flock(다른 쓰는 쪽) 실패의 이유가 last_error() 에 남고, G1_ARCADE_STATUS_SHM=0 은 «끔» 으로 구분된다
// (ArcadeLink::open 이 실패면 기동 거부 · 끔이면 칸 없이 — tests/test_arcade_gate.cpp ⑤ 가 그 거부를 본다).
static void status_open_failure_reasons() {
    {
        ArcadeStatusWriter w;
        chk(!w.open("/nonexistent_g1_arcade_shm_dir/status") && !w.on() && std::strstr(w.last_error(), "open 실패") != nullptr,
            "상태 칸 open 실패 → last_error «open 실패»");
    }
    {
        TmpShm t; ArcadeStatusWriter a, b;
        chk(a.open(t.path.c_str()) && a.on(), "상태 칸 첫 쓰는 쪽");
        chk(!b.open(t.path.c_str()) && !b.on() && std::strstr(b.last_error(), "다른 프로세스") != nullptr,
            "두 번째 쓰는 쪽 → flock 실패 · last_error «다른 프로세스»");
        a.close_for_test();
        chk(b.open(t.path.c_str()) && b.on() && b.last_error()[0] == '\0', "첫 쪽이 닫으면 열린다 · last_error 비움");
        b.close_for_test();
    }
    {
        ArcadeStatusWriter w;
        setenv("G1_ARCADE_STATUS_SHM", "0", 1);
        chk(!w.open_from_env_or_default() && w.off_by_env() && w.last_error()[0] == '\0', "G1_ARCADE_STATUS_SHM=0 → 끔(off_by_env · 오류 아님)");
        setenv("G1_ARCADE_STATUS_SHM", "/nonexistent_g1_arcade_shm_dir/status", 1);
        chk(!w.open_from_env_or_default() && !w.off_by_env() && w.last_error()[0] != '\0', "환경 경로 열기 실패 → 끔 아님 · 이유 있음");
        unsetenv("G1_ARCADE_STATUS_SHM");
    }
}

static void status_seqlock_never_torn() {
    TmpShm t; ArcadeStatusWriter w;
    chk(w.open(t.path.c_str()), "상태 칸 열기");
    chk(w.on(), "on");
    const g1::arcade_status::Word* words = w.words();
    constexpr uint32_t N = 100000;
    std::atomic<bool> done{false};
    std::thread wr([&] { for (uint32_t k = 1; k <= N; ++k) { ArcadeStatus s = fill(k); w.write(s); } done.store(true); });
    long reads = 0, torn = 0, bad_magic = 0; uint32_t last_k = 0; long back = 0;
    ArcadeStatus s{};
    while (!done.load() || reads < 1000) {
        if (!read_words(words, s, 1 << 20)) { ++bad_magic; continue; }
        ++reads;
        const uint32_t* v = reinterpret_cast<const uint32_t*>(&s);
        const uint32_t k = v[2];
        for (int i = 3; i < g1::arcade_status::N_WORDS; ++i) if (v[i] != k) { ++torn; break; }
        if (k < last_k) ++back;
        last_k = k;
        if (done.load() && reads >= 1000) break;
    }
    wr.join();
    ArcadeStatus fin{};
    chk(read_words(words, fin, 8) && fin.tick == N && fin.seq == 2u * (N + 1), "마지막 프레임 · seq 는 짝수로 끝난다");
    std::printf("  status_seqlock: 쓰기 %u · 읽기 %ld · 찢김 %ld · magic/재시도 실패 %ld · 뒤로 감김 %ld\n", N, reads, torn, bad_magic, back);
    chk(torn == 0, "seqlock 찢김 0");
    chk(back == 0, "tick 이 거꾸로 가지 않는다");
    chk(reads > 0, "읽기가 한 번은 성공");
    w.close_for_test();
}

static void status_write_allocates_nothing() {
    TmpShm t; ArcadeStatusWriter w;
    chk(w.open(t.path.c_str()), "열기");
    ArcadeStatus s = fill(3);
    w.write(s);
    g_allocs.store(0); g_count_alloc = true;
    for (int i = 0; i < 1000; ++i) { s.tick = static_cast<uint32_t>(i); w.write(s); }
    g_count_alloc = false;
    const long allocs = g_allocs.load();
    chk(allocs == 0, "write() 1000 회 할당 0");
    // 계측기 자기검증 — 카운터가 실제로 움직인다
    g_allocs.store(0); g_count_alloc = true;
    int* a = new int(1);
    g_count_alloc = false;
    chk(g_allocs.load() >= 1, "할당 카운터가 움직인다");
    delete a;
    std::printf("  status_write: 1000 회 할당 %ld\n", allocs);
    w.close_for_test();
}

int main() {
    intent_stale_zero_after_15_ticks();
    event_applied_once_per_event_seq();
    old_event_dropped();
    stale_drops_event_seq();
    rearm_requires_neutral_after_stale_and_stay_start();
    nonfinite_axis_is_zero();
    sticky_event_read_rules();
    stay_start_first_frame_is_baseline();
    event_bits_match_core();
    read_intent_file_allocates_nothing();
    read_intent_file_roundtrip();
    status_open_failure_reasons();
    status_seqlock_never_torn();
    status_write_allocates_nothing();
    if (fail) { std::printf("%d FAIL\n", fail); return 1; }
    std::printf("test_arcade_shm: all ok\n");
    return 0;
}
