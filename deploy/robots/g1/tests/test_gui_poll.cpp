// GuiPoll.h 판정 시험. 실행: g++ -std=gnu++17 -O2 -Wall -Wextra -I../include test_gui_poll.cpp && ./a.out
#include "GuiPoll.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)

static GuiCtrl mk(int32_t magic, uint32_t seq, float vx = 0, float vy = 0, float wz = 0) {
    GuiCtrl g{};
    g.magic = magic; g.seq = seq; g.vx = vx; g.vy = vy; g.wz = wz;
    g.period_steps = 43; g.height_scale = 1.0f; g.turn_k = 0.3f; g.clip_req = -1;
    return g;
}

// ── 기준: 이 변경 전 State_Mimic.cpp g_poll_gui 의 판정을 그대로 옮긴 것(파일 읽기·출력·모드 호출만 뺐다) ──
struct Legacy {
    uint32_t last_seq = 0, last_m5 = 0; bool rebase = true;
    struct Out { bool apply=false; int mode_req=0; bool m5=false; int m5_preset=0; int clip=-1;
                 float vx=0,vy=0,wz=0; int period=0; float hs=0, tk=0; bool rebased=false, nonfinite=false; };
    Out poll(const GuiCtrl* fp) {
        Out o;
        if (!fp) { rebase = false; return o; }
        const GuiCtrl& g = *fp;
        if (g.magic != GUI_CTRL_MAGIC) { rebase = false; return o; }
        if (rebase) {
            rebase = false; last_seq = g.seq;
            if (g.m5_preset != 0) last_m5 = g.m5_press_seq;
            o.rebased = true; return o;
        }
        if (!std::isfinite(g.vx) || !std::isfinite(g.vy) || !std::isfinite(g.wz) ||
            !std::isfinite(g.turn_k) || !std::isfinite(g.height_scale)) {
            if (g.seq != last_seq) { last_seq = g.seq; o.nonfinite = true; }
            return o;
        }
        if (g.seq == last_seq) return o;
        last_seq = g.seq;
        o.apply = true; o.mode_req = g.mode_req;
        if (g.m5_preset != 0 && g.m5_press_seq != last_m5) { last_m5 = g.m5_press_seq; o.m5 = true; o.m5_preset = g.m5_preset; }
        o.clip = g.clip_req;
        o.vx = g.vx; o.vy = g.vy; o.wz = g.wz;
        o.period = g.period_steps; o.hs = g.height_scale; o.tk = g.turn_k;
        return o;
    }
};

static void v2_frames_behave_as_before() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<GuiCtrl> frames;
    GuiCtrl a = mk(GUI_CTRL_MAGIC, 100, 0.5f, 0.1f, -0.2f); a.mode_req = 3; a.m5_preset = 4; a.m5_press_seq = 7; a.clip_req = 2;
    GuiCtrl b = a; b.seq = 101; b.mode_req = 0; b.clip_req = -1;                 // 같은 자세 seq → 누름 아님
    GuiCtrl c = a; c.seq = 102; c.m5_press_seq = 8; c.period_steps = 60; c.height_scale = 1.4f; c.turn_k = 0.5f;
    GuiCtrl d = mk(GUI_CTRL_MAGIC, 103, nan, 0, 0);                              // NaN
    GuiCtrl d2 = d;                                                              // 같은 seq NaN → 알림 없음
    GuiCtrl e = mk(GUI_CTRL_MAGIC, 104, 0.3f);
    GuiCtrl foreign = mk(0x6702, 105, 9, 9, 9);                                  // 남의 magic
    GuiCtrl f2 = mk(GUI_CTRL_MAGIC, 106, 0.1f);
    GuiCtrl pico = f2; pico.seq = 107; pico.m5_preset = 0; pico.m5_press_seq = 0;
    // (nullptr 은 따로 끼운다)
    std::vector<const GuiCtrl*> seq = {&a, &a, &b, &b, &c, &c, &d, &d2, &e, nullptr, &e, &foreign, &f2, &pico, &pico};
    Legacy lg; GuiPollState st;
    for (size_t i = 0; i < seq.size(); ++i) {
        Legacy::Out o = lg.poll(seq[i]);
        GuiApply r = poll_gui_frame(seq[i], st);
        CHECK(r.apply == o.apply); CHECK(r.rebased == o.rebased); CHECK(r.rejected_nonfinite == o.nonfinite);
        CHECK(st.last_seq == lg.last_seq); CHECK(st.last_m5_seq == lg.last_m5); CHECK(st.rebase == lg.rebase);
        CHECK(!r.zero_vel);
        if (o.apply) {
            CHECK(r.mode_req == o.mode_req); CHECK(r.m5_press == o.m5);
            if (o.m5) CHECK(r.m5_preset == o.m5_preset);
            CHECK(r.clip_req == o.clip); CHECK(r.set_vel && r.set_foot);
            CHECK(r.vx == o.vx && r.vy == o.vy && r.wz == o.wz);
            CHECK(r.period_steps == o.period && r.height_scale == o.hs && r.turn_k == o.tk);
            CHECK(r.log_line);                                                   // 0x6704 는 적용 프레임마다 로그
        } else CHECK(!r.log_line);
    }
    // 명시 기대값: 첫 프레임은 기준선, seq 같으면 무시
    GuiPollState s2; GuiApply r = poll_gui_frame(&a, s2); CHECK(!r.apply && r.rebased);
    r = poll_gui_frame(&a, s2); CHECK(!r.apply);
    r = poll_gui_frame(&c, s2); CHECK(r.apply && r.m5_press && r.m5_preset == 4 && r.period_steps == 60 && r.clip_req == 2);
    // stale 판정 없음: seq 가 1000 틱 멈춰도 zero_vel 없음
    for (int i = 0; i < 1000; ++i) { r = poll_gui_frame(&c, s2); CHECK(!r.zero_vel); }
    for (int i = 0; i < 1000; ++i) { r = poll_gui_frame(nullptr, s2); CHECK(!r.zero_vel); }
}

static void relay_frames_apply_only_mode_preset_velocity() {
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 1);
    poll_gui_frame(&g, st);                                                      // 기준선
    g = mk(MAGIC_GUI_RELAY, 2, 0.6f, -0.1f, 0.4f);
    g.mode_req = 2; g.m5_preset = 3; g.m5_press_seq = 5;
    g.period_steps = 99; g.height_scale = 2.0f; g.turn_k = 0.9f; g.clip_req = 4;
    GuiApply r = poll_gui_frame(&g, st);
    CHECK(r.apply); CHECK(!r.set_foot); CHECK(r.clip_req == -1);
    CHECK(r.mode_req == 2); CHECK(r.m5_press && r.m5_preset == 3);
    CHECK(r.set_vel && r.vx == 0.6f && r.vy == -0.1f && r.wz == 0.4f);
    // NaN 프레임은 0x6704 와 똑같이 버린다
    GuiCtrl n = mk(MAGIC_GUI_RELAY, 3, std::numeric_limits<float>::infinity());
    r = poll_gui_frame(&n, st); CHECK(!r.apply && r.rejected_nonfinite);
    // 첫 프레임 기준선은 0x6705 에도 적용
    GuiPollState s2; g = mk(MAGIC_GUI_RELAY, 50, 1, 1, 1); g.mode_req = 5;
    r = poll_gui_frame(&g, s2); CHECK(!r.apply && r.rebased && !r.log_line);
}

static void relay_frames_stale_zero_velocity() {
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 10, 0.5f);
    poll_gui_frame(&g, st);                                                      // 기준선
    g.seq = 11; GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply && !r.zero_vel);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel); CHECK(!gui_relay_stale(st)); }   // 같은 seq 14 틱
    r = poll_gui_frame(&g, st); CHECK(r.zero_vel); CHECK(gui_relay_stale(st));   // 15 틱째 — 끊김 에피소드 시작
    for (int i = 0; i < 50; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel); CHECK(gui_relay_stale(st)); }    // 1 회뿐 · 에피소드는 이어진다
    g.seq = 12; r = poll_gui_frame(&g, st); CHECK(r.apply && !r.zero_vel);       // 되살아남 → 정상
    CHECK(!gui_relay_stale(st));                                                 // 에피소드 끝
    CHECK(r.log_line);                                                           // 속도가 0 → 0.5 로 바뀜
    // 파일이 사라져도 같다
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(nullptr, st); CHECK(!r.zero_vel); }
    r = poll_gui_frame(nullptr, st); CHECK(r.zero_vel);
    r = poll_gui_frame(nullptr, st); CHECK(!r.zero_vel);
    // 되살아난 뒤 다시 멈추면 다시 한 번
    g.seq = 13; r = poll_gui_frame(&g, st); CHECK(r.apply);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel); }
    r = poll_gui_frame(&g, st); CHECK(r.zero_vel);
    // 중계를 본 적 없으면 파일 없음은 감시 대상이 아니다
    GuiPollState s2; for (int i = 0; i < 100; ++i) { r = poll_gui_frame(nullptr, s2); CHECK(!r.zero_vel); }
    // v2 가 이어받으면 감시 끝
    GuiPollState s3; GuiCtrl rl = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&rl, s3);
    for (int i = 0; i < 5; ++i) poll_gui_frame(&rl, s3);
    GuiCtrl v2 = mk(GUI_CTRL_MAGIC, 2, 0.2f); r = poll_gui_frame(&v2, s3); CHECK(r.apply && !s3.relay_seen);
    for (int i = 0; i < 100; ++i) { r = poll_gui_frame(&v2, s3); CHECK(!r.zero_vel); CHECK(!gui_relay_stale(s3)); }
    // 끊김 에피소드 중 체류가 다시 시작되면 에피소드도 끝난다(새 체류는 0x6705 를 다시 볼 때까지 감시 안 함)
    GuiPollState s8; GuiCtrl e = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&e, s8);
    for (int i = 0; i < 20; ++i) poll_gui_frame(nullptr, s8);
    CHECK(gui_relay_stale(s8));
    gui_poll_begin_stay(s8); CHECK(!gui_relay_stale(s8));
    // 틱 14 직전에 하트비트가 오면 카운트가 리셋된다
    GuiPollState s4; GuiCtrl h = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&h, s4);
    for (int round = 0; round < 5; ++round) {
        for (int i = 0; i < 13; ++i) { r = poll_gui_frame(&h, s4); CHECK(!r.zero_vel); }
        ++h.seq; r = poll_gui_frame(&h, s4); CHECK(!r.zero_vel);
    }
    // 중계를 본 뒤: seq 가 그대로인 비유한 프레임도 «안 바뀜» 으로 센다(죽은 중계가 NaN 을 남긴 채 멈춘 경우)
    GuiPollState s5; GuiCtrl q = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&q, s5);
    q.seq = 2; q.vx = 0.4f; r = poll_gui_frame(&q, s5); CHECK(r.apply);
    q.vx = std::numeric_limits<float>::quiet_NaN();                             // 같은 seq 2, NaN
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&q, s5); CHECK(!r.zero_vel && !r.apply); }
    r = poll_gui_frame(&q, s5); CHECK(r.zero_vel);
    // 모르는 magic(남의 파일)도 «안 바뀜» 으로 센다
    GuiPollState s6; GuiCtrl w = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&w, s6);
    w.seq = 2; r = poll_gui_frame(&w, s6); CHECK(r.apply);
    GuiCtrl other = mk(0x1234, 3, 9, 9, 9);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&other, s6); CHECK(!r.zero_vel && !r.apply); }
    r = poll_gui_frame(&other, s6); CHECK(r.zero_vel);
    // 크기가 다른 파일 = State_Mimic 이 nullptr 로 넘긴다(n != sizeof(GuiCtrl)) — 모르는 magic 과 섞여도 이어서 센다
    GuiPollState s7; GuiCtrl z = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&z, s7);
    z.seq = 2; r = poll_gui_frame(&z, s7); CHECK(r.apply);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame((i % 2) ? nullptr : &other, s7); CHECK(!r.zero_vel); }
    r = poll_gui_frame(nullptr, s7); CHECK(r.zero_vel);
}

// 0x6705 의 비유한 프레임이 하트비트로 이어지면(seq 가 매 틱 바뀐다) 알림은 그 연속의 첫 번만 — 정책 스레드 printf 50 회/s 방지.
// 0x6704 는 종전 그대로 나쁜 프레임마다. 하트비트로는 계속 센다(끊김 아님).
static void relay_nonfinite_reported_once_per_run() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&g, st);   // 기준선
    int reported = 0;
    for (uint32_t s = 2; s < 52; ++s) {                                          // 50 틱 NaN 하트비트
        GuiCtrl n = mk(MAGIC_GUI_RELAY, s, nan);
        GuiApply r = poll_gui_frame(&n, st);
        CHECK(!r.apply && !r.zero_vel);
        reported += r.rejected_nonfinite ? 1 : 0;
        if (s == 2) CHECK(r.rejected_nonfinite);                                // 첫 번은 알린다
    }
    CHECK(reported == 1);
    g = mk(MAGIC_GUI_RELAY, 52, 0.2f); GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply);   // 정상 프레임 → 다시 무장
    GuiCtrl n2 = mk(MAGIC_GUI_RELAY, 53, 0, nan); r = poll_gui_frame(&n2, st); CHECK(r.rejected_nonfinite);
    n2.seq = 54; r = poll_gui_frame(&n2, st); CHECK(!r.rejected_nonfinite);
    // 체류 시작도 다시 무장한다
    gui_poll_begin_stay(st);
    GuiCtrl b = mk(MAGIC_GUI_RELAY, 60); poll_gui_frame(&b, st);                // 기준선
    n2.seq = 61; r = poll_gui_frame(&n2, st); CHECK(r.rejected_nonfinite);
    // 0x6704: 바뀐 seq 의 NaN 프레임마다 알린다(종전)
    GuiPollState v; GuiCtrl a = mk(GUI_CTRL_MAGIC, 200); poll_gui_frame(&a, v);
    for (uint32_t s = 201; s < 206; ++s) { GuiCtrl m = mk(GUI_CTRL_MAGIC, s, nan); r = poll_gui_frame(&m, v); CHECK(r.rejected_nonfinite); }
}

// 중계 로그 판정의 기준 속도(last_v*)는 «지금 GUI 속도» 를 따라간다 — 0x6704 가 바꾼 뒤·체류 시작 뒤에 낡은 값이 남으면
// 중계가 그 낡은 값과 같은 속도로 돌아올 때 로그가 빠진다(속도는 바뀌었는데 기록이 없다).
static void relay_log_baseline_follows_v2_and_stay() {
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&g, st);
    g = mk(MAGIC_GUI_RELAY, 2, 0.5f); GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);
    GuiCtrl v2 = mk(GUI_CTRL_MAGIC, 3, 0.8f); r = poll_gui_frame(&v2, st); CHECK(r.apply);   // GUI 가 0.8 로
    g.seq = 4; r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);         // 중계가 0.5 로 되돌림 → 로그
    CHECK(st.last_vx == 0.5f);
    // 체류 시작(파일 없음 → 기준선 없이 첫 쓰기가 먹는다): 전역 속도는 0 이 됐다 → 중계 0.5 는 바뀜이다
    gui_poll_begin_stay(st);
    CHECK(st.rebase && !st.relay_seen && st.stale_ticks == 0);
    CHECK(st.last_vx == 0.f && st.last_vy == 0.f && st.last_wz == 0.f);
    r = poll_gui_frame(nullptr, st); CHECK(!st.rebase && !r.apply);
    g.seq = 5; r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);
}

static void relay_log_only_on_change() {
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 1);
    poll_gui_frame(&g, st);
    g = mk(MAGIC_GUI_RELAY, 2, 0.4f, 0, 0);
    GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);           // 속도 변화
    for (uint32_t s = 3; s < 20; ++s) { g.seq = s; r = poll_gui_frame(&g, st); CHECK(r.apply && !r.log_line); }   // 하트비트
    g.seq = 20; g.wz = 0.1f; r = poll_gui_frame(&g, st); CHECK(r.log_line);       // wz 변화
    g.seq = 21; g.mode_req = 3; r = poll_gui_frame(&g, st); CHECK(r.log_line);    // 모드
    g.seq = 22; g.mode_req = 0; r = poll_gui_frame(&g, st); CHECK(!r.log_line);
    g.seq = 23; g.m5_preset = 2; g.m5_press_seq = 1; r = poll_gui_frame(&g, st); CHECK(r.log_line && r.m5_press);   // 자세 누름
    g.seq = 24; r = poll_gui_frame(&g, st); CHECK(!r.log_line);                  // 같은 누름 seq
    g.seq = 25; g.vy = 0.2f; r = poll_gui_frame(&g, st); CHECK(r.log_line);       // vy 변화
}

int main() {
    v2_frames_behave_as_before();
    relay_frames_apply_only_mode_preset_velocity();
    relay_frames_stale_zero_velocity();
    relay_log_only_on_change();
    relay_nonfinite_reported_once_per_run();
    relay_log_baseline_follows_v2_and_stay();
    if (g_fail) { std::printf("%d failure(s)\n", g_fail); return 1; }
    std::printf("test_gui_poll ok\n");
    return 0;
}
