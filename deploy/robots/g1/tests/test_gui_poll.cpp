// GuiPoll.h 판정 시험. 실행: g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include test_gui_poll.cpp && ./a.out
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
    g.seq = 2; poll_gui_frame(&g, st);                                           // 속도 0 — 속도 문 열림
    g = mk(MAGIC_GUI_RELAY, 3, 0.6f, -0.1f, 0.4f);
    g.mode_req = 2; g.m5_preset = 3; g.m5_press_seq = 5;
    g.period_steps = 99; g.height_scale = 2.0f; g.turn_k = 0.9f; g.clip_req = 4;
    GuiApply r = poll_gui_frame(&g, st);
    CHECK(r.apply); CHECK(!r.set_foot); CHECK(r.clip_req == -1);
    CHECK(r.mode_req == 2); CHECK(r.m5_press && r.m5_preset == 3);
    CHECK(r.set_vel && r.vx == 0.6f && r.vy == -0.1f && r.wz == 0.4f);
    // NaN 프레임은 0x6704 와 똑같이 버린다
    GuiCtrl n = mk(MAGIC_GUI_RELAY, 4, std::numeric_limits<float>::infinity());
    r = poll_gui_frame(&n, st); CHECK(!r.apply && r.rejected_nonfinite);
    // 첫 프레임 기준선은 0x6705 에도 적용
    GuiPollState s2; g = mk(MAGIC_GUI_RELAY, 50, 1, 1, 1); g.mode_req = 5;
    r = poll_gui_frame(&g, s2); CHECK(!r.apply && r.rebased && !r.log_line);
}

static void relay_frames_stale_zero_velocity() {
    GuiPollState st; GuiCtrl g = mk(MAGIC_GUI_RELAY, 10, 0.5f);
    poll_gui_frame(&g, st);                                                      // 기준선
    GuiCtrl z0 = mk(MAGIC_GUI_RELAY, 11); poll_gui_frame(&z0, st);              // 속도 0 — 속도 문 열림
    g.seq = 12; GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply && !r.zero_vel && r.vx == 0.5f);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel); CHECK(!gui_relay_stale(st)); }   // 같은 seq 14 틱
    r = poll_gui_frame(&g, st); CHECK(r.zero_vel); CHECK(gui_relay_stale(st));   // 15 틱째 — 끊김 에피소드 시작
    CHECK(r.vel_gate_closed);                                                    // 열려 있던 속도 문도 같은 틱에 닫힌다
    for (int i = 0; i < 50; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel && !r.vel_gate_closed); CHECK(gui_relay_stale(st)); }    // 1 회뿐 · 에피소드는 이어진다
    g.seq = 13; r = poll_gui_frame(&g, st); CHECK(r.apply && !r.zero_vel);       // 되살아남 → 정상(하트비트)
    CHECK(!gui_relay_stale(st));                                                 // 에피소드 끝
    CHECK(r.vx == 0.f && !r.log_line);                                           // 단 문이 닫혀 속도 0(찍힌 0 과 같아 줄 없음)
    z0.seq = 14; r = poll_gui_frame(&z0, st); CHECK(r.vel_gate_opened);          // 0 프레임 → 다시 열림
    g.seq = 15; r = poll_gui_frame(&g, st); CHECK(r.log_line && r.vx == 0.5f);   // 속도가 0 → 0.5 로 바뀜
    // 파일이 사라져도 같다
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(nullptr, st); CHECK(!r.zero_vel); }
    r = poll_gui_frame(nullptr, st); CHECK(r.zero_vel && r.vel_gate_closed);
    r = poll_gui_frame(nullptr, st); CHECK(!r.zero_vel && !r.vel_gate_closed);
    // 되살아난 뒤 다시 멈추면 다시 한 번(문이 이미 닫혀 있으면 «닫힘» 알림은 없다)
    g.seq = 16; r = poll_gui_frame(&g, st); CHECK(r.apply && r.vx == 0.f);
    for (int i = 1; i <= 14; ++i) { r = poll_gui_frame(&g, st); CHECK(!r.zero_vel); }
    r = poll_gui_frame(&g, st); CHECK(r.zero_vel && !r.vel_gate_closed);
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

// 중계 로그 판정의 기준 속도(last_v*) = «마지막으로 줄에 찍힌» GUI 속도 — 0x6704 가 찍은 줄·체류 시작 뒤에 낡은 값이 남으면
// 중계가 그 낡은 값과 같은 속도로 돌아올 때 로그가 빠진다(속도는 바뀌었는데 기록이 없다).
static void relay_log_baseline_follows_v2_and_stay() {
    GuiPollState st; uint32_t s = 1;
    GuiCtrl g = mk(MAGIC_GUI_RELAY, s++); poll_gui_frame(&g, st);               // 기준선
    g.seq = s++; poll_gui_frame(&g, st);                                         // 속도 0 — 속도 문 열림
    g = mk(MAGIC_GUI_RELAY, s++, 0.5f); GuiApply r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);
    GuiCtrl v2 = mk(GUI_CTRL_MAGIC, s++, 0.8f); r = poll_gui_frame(&v2, st); CHECK(r.apply && r.log_line);   // GUI 가 0.8 로(찍힘)
    CHECK(st.last_vx == 0.8f);
    for (int i = 0; i < GUI_RELAY_LOG_MIN_TICKS; ++i) poll_gui_frame(&v2, st);  // 같은 seq — 시간만 흐른다
    g.seq = s++; r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line);       // 중계가 0.5 로 되돌림 → 로그(기준 = 찍힌 0.8)
    CHECK(st.last_vx == 0.5f);
    // 체류 시작(파일 없음 → 기준선 없이 첫 쓰기가 먹는다): 전역 속도는 0 이 됐다 → 기준도 0 · 로그 시계 다 참 · 속도 문 닫힘
    gui_poll_begin_stay(st);
    CHECK(st.rebase && !st.relay_seen && st.stale_ticks == 0);
    CHECK(st.last_vx == 0.f && st.last_vy == 0.f && st.last_wz == 0.f);
    CHECK(st.log_age == GUI_RELAY_LOG_MIN_TICKS && !st.relay_vel_ok && !st.relay_announced);
    r = poll_gui_frame(nullptr, st); CHECK(!st.rebase && !r.apply);
    GuiCtrl z = mk(MAGIC_GUI_RELAY, s++); r = poll_gui_frame(&z, st); CHECK(r.apply && r.vel_gate_opened && !r.log_line);
    g.seq = s++; r = poll_gui_frame(&g, st); CHECK(r.apply && r.log_line && r.vx == 0.5f);   // 새 체류의 첫 변화는 바로 찍는다
}

// M1 — 0x6705 로그: mode_req·자세 누름은 바로. 속도만 바뀐 것은 GUI_RELAY_LOG_MIN_TICKS(25) 틱에 한 번까지, 마지막으로 찍은 값에서
// 한 축이라도 0.05 이상 바뀌었을 때만, 찍을 땐 그 틱의 최신 값. 적용은 매 틱(로그만 줄인다). 0x6704 는 종전 그대로(적용마다).
static void relay_velocity_log_rate_limited() {
    GuiPollState st; uint32_t s = 1;
    GuiCtrl g = mk(MAGIC_GUI_RELAY, s++); poll_gui_frame(&g, st);               // 기준선
    g.seq = s++; GuiApply r = poll_gui_frame(&g, st);
    CHECK(r.apply && r.vel_gate_opened && !r.log_line);                          // 속도 0 — 문 열림 · 찍힌 0 과 같아 줄 없음
    // 아날로그 패드: 매 틱 0.02 씩 100 틱(0 → 2.0). 첫 줄 = 0.05 를 처음 넘는 틱(0.06, k=3), 그 뒤 25 틱마다(28·53·78).
    std::vector<int> line_ticks;
    for (int k = 1; k <= 100; ++k) {
        g.seq = s++; g.vx = 0.02f * float(k);
        r = poll_gui_frame(&g, st);
        CHECK(r.apply && r.set_vel && r.vx == g.vx);                             // 적용은 매 틱
        if (r.log_line) { line_ticks.push_back(k); CHECK(st.last_vx == g.vx); } // 찍는 값 = 그 틱의 최신 값
    }
    CHECK((line_ticks == std::vector<int>{3, 28, 53, 78}));                      // 100 틱(2 s)에 4 줄 — 종전엔 100 줄
    // 멈춘 뒤: 마지막 줄(1.56) 뒤 25 틱이 되는 k=103 에 최신 값(2.0)을 한 번 찍고, 그 뒤엔 조용
    int lines = 0, at = -1;
    for (int k = 101; k <= 160; ++k) {
        g.seq = s++; r = poll_gui_frame(&g, st);
        if (r.log_line) { ++lines; at = k; CHECK(st.last_vx == g.vx); }
    }
    CHECK(lines == 1 && at == 103);
    // 0.05 미만의 바뀜은 아무리 기다려도 안 찍는다(0.04 — 문턱과 거리를 둔 값)
    g.vx += 0.04f;
    for (int i = 0; i < 60; ++i) { g.seq = s++; r = poll_gui_frame(&g, st); CHECK(r.apply && !r.log_line && r.vx == g.vx); }
    // 한 축만 바뀌어도(vy · wz) 찍는다 — 간격이 찼으면 바로
    g.seq = s++; g.vy = 0.1f; r = poll_gui_frame(&g, st); CHECK(r.log_line);
    for (int i = 0; i < GUI_RELAY_LOG_MIN_TICKS - 1; ++i) { g.seq = s++; poll_gui_frame(&g, st); }
    g.seq = s++; g.wz = -0.1f; r = poll_gui_frame(&g, st); CHECK(r.log_line);   // 정확히 25 틱 뒤
    // 모드 요청·자세 누름은 간격과 상관없이 바로(직전 줄 바로 다음 틱이어도)
    g.seq = s++; g.mode_req = 3; r = poll_gui_frame(&g, st); CHECK(r.log_line);
    g.seq = s++; g.mode_req = 0; g.m5_preset = 2; g.m5_press_seq = 1; r = poll_gui_frame(&g, st); CHECK(r.log_line && r.m5_press);
    g.seq = s++; r = poll_gui_frame(&g, st); CHECK(!r.log_line);                 // 같은 누름 seq — 줄 없음
    // 모드 줄도 시계를 다시 건다(그 줄에 지금 속도가 찍혔다) — 바로 뒤의 큰 속도 바뀜은 25 틱을 기다린다
    g.m5_preset = 0; g.vx = 0.f;
    int wait = 0;
    for (;;) { g.seq = s++; r = poll_gui_frame(&g, st); ++wait; if (r.log_line || wait > 100) break; }
    CHECK(wait == GUI_RELAY_LOG_MIN_TICKS - 1 && st.last_vx == 0.f);            // 누름 줄(틱 0) 뒤 25 번째 틱
    // 0x6704: 종전 그대로 — 매 틱 0.01 씩 바뀌어도 적용 프레임마다 줄
    GuiPollState v; GuiCtrl a = mk(GUI_CTRL_MAGIC, 500); poll_gui_frame(&a, v);
    for (int k = 1; k <= 30; ++k) { a.seq = 500u + uint32_t(k); a.vx = 0.01f * float(k); r = poll_gui_frame(&a, v); CHECK(r.apply && r.log_line); }
}

// M2 — 체류마다 «속도 문»: 그 체류에서 속도 0 인 0x6705 를 적용하기 전엔 0x6705 속도를 0 으로 친다(W 를 누른 채 m 해도 바로 안 걷는다).
// mode_req·자세는 문과 상관없이 먹는다. 기준선 프레임은 세지 않는다. 0x6704 는 문과 상관없다.
static void relay_velocity_gate_per_stay() {
    GuiPollState st; gui_poll_begin_stay(st); uint32_t s = 100;
    GuiCtrl w = mk(MAGIC_GUI_RELAY, s++, 0.4f, 0.1f, -0.2f);                    // 브라우저가 W·A·E 를 누른 채 Mimic 진입
    GuiApply r = poll_gui_frame(&w, st); CHECK(r.rebased && !r.apply);          // 기준선(적용 없음)
    for (int i = 0; i < 30; ++i) {
        w.seq = s++; r = poll_gui_frame(&w, st);
        CHECK(r.apply && r.set_vel && r.vx == 0.f && r.vy == 0.f && r.wz == 0.f);   // 문 닫힘 → 0 을 적용(키보드 속도도 덮는다)
        CHECK(!r.vel_gate_opened && !r.log_line);
    }
    // 문이 닫혀 있어도 모드 요청·자세는 먹는다(줄은 바로 · 찍히는 속도 = 0)
    w.seq = s++; w.mode_req = 2; w.m5_preset = 3; w.m5_press_seq = 9;
    r = poll_gui_frame(&w, st);
    CHECK(r.apply && r.mode_req == 2 && r.m5_press && r.m5_preset == 3 && r.log_line);
    CHECK(r.vx == 0.f && r.vy == 0.f && r.wz == 0.f && !r.vel_gate_opened);
    w.mode_req = 0; w.m5_preset = 0;
    // 0 프레임 하나 → 문 열림(그 프레임도 0) → 다음부터 따라간다
    GuiCtrl z = mk(MAGIC_GUI_RELAY, s++); r = poll_gui_frame(&z, st);
    CHECK(r.apply && r.vel_gate_opened && r.vx == 0.f && r.vy == 0.f && r.wz == 0.f);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.4f && r.vy == 0.1f && r.wz == -0.2f && !r.vel_gate_opened);
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.vx == 0.f && !r.vel_gate_opened);   // 다시 0 이어도 «열림» 은 한 번
    w.seq = s++; w.vx = 0.7f; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.7f);           // 열린 채
    // 끊김(zero_vel)은 문을 다시 닫는다(R-F2) — 아래 relay_stale_recloses_velocity_gate 가 자세히
    for (int i = 0; i < GUI_RELAY_STALE_TICKS; ++i) poll_gui_frame(nullptr, st);
    CHECK(gui_relay_stale(st) && !st.relay_vel_ok);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.f);
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.vel_gate_opened);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.7f);
    // 다음 체류: 기준선이 0 이어도 문은 닫힌 채 — 진입 «뒤» 새 seq 의 0 을 봐야 연다
    gui_poll_begin_stay(st);
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.rebased);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.apply && r.vx == 0.f && r.vy == 0.f && r.wz == 0.f);
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.vel_gate_opened);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.7f);
    // 체류 시작에 파일이 없었으면(기준선 없음) 첫 중계 프레임부터 문이 걸린다
    gui_poll_begin_stay(st);
    r = poll_gui_frame(nullptr, st); CHECK(!r.apply);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.apply && r.vx == 0.f);
    // 비유한 프레임·같은 seq 프레임은 문을 열지 않는다
    GuiCtrl n = mk(MAGIC_GUI_RELAY, s++, 0.f, std::numeric_limits<float>::quiet_NaN(), 0.f);
    r = poll_gui_frame(&n, st); CHECK(!r.apply && !r.vel_gate_opened && !st.relay_vel_ok);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.apply && r.vx == 0.f);
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.vel_gate_opened);
    // 0x6704 는 문과 상관없다: 새 체류에서 바로 그 값을 적용한다
    GuiPollState v; gui_poll_begin_stay(v);
    poll_gui_frame(nullptr, v);
    GuiCtrl a = mk(GUI_CTRL_MAGIC, 7, 0.3f, 0.2f, 0.1f); r = poll_gui_frame(&a, v);
    CHECK(r.apply && r.vx == 0.3f && r.vy == 0.2f && r.wz == 0.1f && !r.vel_gate_opened && !r.relay_first);
}

// R-F2 — 끊김 에피소드(zero_vel)가 속도 문을 다시 닫는다(같은 틱, 열려 있었을 때만 알림). 같은 체류 안에서 중계가 다시 떠
// 브라우저가 키를 누른 채여도, 새 seq 의 속도 0 프레임을 한 번 볼 때까지 속도는 0. mode_req·자세는 그동안에도 먹는다.
static void relay_stale_recloses_velocity_gate() {
    GuiPollState st; gui_poll_begin_stay(st); uint32_t s = 1;
    GuiCtrl z = mk(MAGIC_GUI_RELAY, s++); poll_gui_frame(&z, st);               // 기준선
    z.seq = s++; GuiApply r = poll_gui_frame(&z, st); CHECK(r.vel_gate_opened); // 체류 → 문 열림
    GuiCtrl w = mk(MAGIC_GUI_RELAY, s++, 0.4f, -0.2f, 0.3f);
    r = poll_gui_frame(&w, st); CHECK(r.vx == 0.4f && r.vy == -0.2f && r.wz == 0.3f);   // 따라간다
    // 중계 죽음(SIGKILL — 마지막 프레임이 W 인 채 멈춤): 15 틱째 zero_vel 과 같은 틱에 문 닫힘
    for (int i = 1; i < GUI_RELAY_STALE_TICKS; ++i) { r = poll_gui_frame(&w, st); CHECK(!r.zero_vel && !r.vel_gate_closed && st.relay_vel_ok); }
    r = poll_gui_frame(&w, st); CHECK(r.zero_vel && r.vel_gate_closed && !st.relay_vel_ok);
    for (int i = 0; i < 30; ++i) { r = poll_gui_frame(&w, st); CHECK(!r.zero_vel && !r.vel_gate_closed); }   // 알림은 한 번
    // 중계가 다시 뜸 — 브라우저는 W 를 누른 채: 새 seq 하트비트는 적용되지만 속도는 0
    for (int i = 0; i < 20; ++i) {
        w.seq = s++; r = poll_gui_frame(&w, st);
        CHECK(r.apply && r.set_vel && r.vx == 0.f && r.vy == 0.f && r.wz == 0.f && !r.vel_gate_opened && !r.zero_vel);
    }
    CHECK(!gui_relay_stale(st));                                                 // 끊김 에피소드는 끝났다(하트비트는 살아 있다)
    // 문이 닫혀 있어도 모드 요청·자세는 먹는다
    w.seq = s++; w.mode_req = 1; w.m5_preset = 2; w.m5_press_seq = 4;
    r = poll_gui_frame(&w, st); CHECK(r.apply && r.mode_req == 1 && r.m5_press && r.m5_preset == 2 && r.vx == 0.f);
    w.mode_req = 0; w.m5_preset = 0;
    // 0 프레임 하나 → 다시 열림 → 다음부터 따라간다
    z.seq = s++; r = poll_gui_frame(&z, st); CHECK(r.apply && r.vel_gate_opened && r.vx == 0.f);
    w.seq = s++; r = poll_gui_frame(&w, st); CHECK(r.vx == 0.4f && r.vy == -0.2f && r.wz == 0.3f);
    // 문이 이미 닫힌 채 끊기면(열린 적 없는 체류) zero_vel 만 — «닫힘» 알림은 없다
    GuiPollState c; gui_poll_begin_stay(c);
    GuiCtrl k = mk(MAGIC_GUI_RELAY, 1, 0.4f); poll_gui_frame(&k, c);
    k.seq = 2; r = poll_gui_frame(&k, c); CHECK(r.apply && r.vx == 0.f);
    bool zeroed = false, closed = false;
    for (int i = 0; i < GUI_RELAY_STALE_TICKS + 5; ++i) { r = poll_gui_frame(&k, c); zeroed |= r.zero_vel; closed |= r.vel_gate_closed; }
    CHECK(zeroed && !closed);
    // 파일이 사라지는 끊김도 같다(열린 문을 닫는다)
    GuiPollState d; gui_poll_begin_stay(d);
    GuiCtrl q = mk(MAGIC_GUI_RELAY, 1); poll_gui_frame(&q, d);
    q.seq = 2; r = poll_gui_frame(&q, d); CHECK(r.vel_gate_opened);
    for (int i = 1; i < GUI_RELAY_STALE_TICKS; ++i) poll_gui_frame(nullptr, d);
    r = poll_gui_frame(nullptr, d); CHECK(r.zero_vel && r.vel_gate_closed);
    // 0x6704 는 끊김 감시도 문도 없다 — 영향 없음
    GuiPollState v; gui_poll_begin_stay(v);
    GuiCtrl a = mk(GUI_CTRL_MAGIC, 1); poll_gui_frame(&a, v);
    for (int i = 0; i < 100; ++i) { r = poll_gui_frame(nullptr, v); CHECK(!r.zero_vel && !r.vel_gate_closed); }
    a.seq = 2; a.vx = 0.3f; r = poll_gui_frame(&a, v); CHECK(r.apply && r.vx == 0.3f);
}

// M3 — 체류마다 첫 0x6705 적용에서 한 번 «중계가 키보드 속도를 덮는다». 기준선·새 seq 없는(남은) 파일은 «적용» 이 아니다.
static void relay_active_announced_once_per_stay() {
    GuiPollState st; gui_poll_begin_stay(st);
    GuiCtrl g = mk(MAGIC_GUI_RELAY, 1); GuiApply r = poll_gui_frame(&g, st); CHECK(r.rebased && !r.relay_first);
    g.seq = 2; r = poll_gui_frame(&g, st); CHECK(r.apply && r.relay_first);
    int n = 0;
    for (uint32_t s = 3; s < 60; ++s) { g.seq = s; r = poll_gui_frame(&g, st); n += r.relay_first ? 1 : 0; }
    for (int i = 0; i < 20; ++i) poll_gui_frame(nullptr, st);                   // 끊김
    g.seq = 60; r = poll_gui_frame(&g, st); n += r.relay_first ? 1 : 0;         // 되살아남(같은 체류)
    CHECK(n == 0);
    // 다음 체류: 다시 한 번(파일이 없다가 생겨도)
    gui_poll_begin_stay(st);
    poll_gui_frame(nullptr, st);
    g.seq = 61; r = poll_gui_frame(&g, st); CHECK(r.apply && r.relay_first);
    // 지난 중계가 남긴 파일(새 seq 없음)만 있는 체류 — 알리지 않는다(끊김 한 줄만)
    gui_poll_begin_stay(st);
    bool any = false, zeroed = false;
    for (int i = 0; i < 40; ++i) { r = poll_gui_frame(&g, st); any |= r.relay_first; zeroed |= r.zero_vel; }
    CHECK(!any && zeroed);
    // 0x6704 는 알리지 않는다
    GuiPollState v; gui_poll_begin_stay(v);
    GuiCtrl a = mk(GUI_CTRL_MAGIC, 1); poll_gui_frame(&a, v);
    a.seq = 2; r = poll_gui_frame(&a, v); CHECK(r.apply && !r.relay_first);
}

int main() {
    v2_frames_behave_as_before();
    relay_frames_apply_only_mode_preset_velocity();
    relay_frames_stale_zero_velocity();
    relay_velocity_log_rate_limited();
    relay_nonfinite_reported_once_per_run();
    relay_log_baseline_follows_v2_and_stay();
    relay_velocity_gate_per_stay();
    relay_stale_recloses_velocity_gate();
    relay_active_announced_once_per_stay();
    if (g_fail) { std::printf("%d failure(s)\n", g_fail); return 1; }
    std::printf("test_gui_poll ok\n");
    return 0;
}
