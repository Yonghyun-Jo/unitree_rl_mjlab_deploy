// GuiCtrl 공유 메모리 프레임(/dev/shm/g1_masked_gui)의 구조체와 «이 프레임을 어떻게 적용할까» 판정.
// State_Mimic.cpp 의 g_poll_gui 가 파일 읽기 + 이 판정 + 결과 적용만 한다. 헤더 온리·무할당·무시스템콜 —
// 정책 스레드에서 매 틱 불린다. tools/gui_shm.py FMT 와 같은 커밋으로만 바꾼다(tests/test_gui_shm_layout.py).
//
//   0x6704 (v2)    GUI·PICO 브리지·replay_cmd 가 값이 바뀔 때만 seq 를 올린다. 끊김 감시 없음.
//   0x6705 (relay) 로봇 위 중계 프로세스가 50 Hz 로 seq 를 올린다(하트비트). 이 프레임을 본 뒤 seq 가
//                  STALE_TICKS 정책 틱 안 바뀌거나 파일이 사라지면 속도를 0 으로 한다(zero_vel, 에피소드당 1회).
//                  보행 파라미터(period·height_scale·turn_k)와 clip_req 는 무시한다.
//                  체류마다 «속도 문»: 그 체류에서 속도 0 인 0x6705 프레임을 한 번 적용할 때까지 0x6705 속도는 0 으로 친다
//                  (W 를 누른 채 Mimic 에 들어가면 바로 걷지 않게). 끊김(zero_vel)도 문을 다시 닫는다 — 같은 체류 안에서
//                  다시 뜬 중계도 속도 0 프레임을 한 번 보내야 속도가 먹는다. mode_req·자세는 문과 상관없이 먹는다.
//                  로그: mode_req·자세 누름은 바로. 속도만 바뀐 것은 GUI_RELAY_LOG_MIN_TICKS 틱에 한 번까지, 그리고 마지막으로
//                  찍은 값에서 한 축이라도 GUI_RELAY_LOG_MIN_DV 이상 바뀌었을 때만(아날로그 패드는 속도가 매 틱 바뀐다 —
//                  정책 스레드에서 printf·fflush 50 회/s 를 막는다). 찍을 때는 그 틱의 최신 값.
#pragma once
#include <cmath>
#include <cstdint>

#pragma pack(push, 1)
struct GuiCtrl {                 // v2 (2026-09-22). tools/gui_shm.py FMT 와 같은 커밋으로만 바꾼다
    int32_t  magic;              // GUI_CTRL_MAGIC / MAGIC_GUI_RELAY
    uint32_t seq;                // GUI 가 바꿀 때마다 +1 (에지 적용) · 중계는 50 Hz 로 항상 +1
    int32_t  mode_req;           // 0 = 요청 없음(1회성). 1.. = 표의 모드 (슬롯 지원·이탈 조건은 ModeRuntime)
    float    vx, vy, wz;         // base_vel 명령 (배포 상한은 State_Mimic 에서 한 번 더)
    int32_t  period_steps;       // foot-gen gait period
    float    height_scale;       // foot-gen swing-height multiplier
    float    turn_k;             // foot-gen |wz|->step gain
    int32_t  m5_preset;          // 0 = 없음, 1..N = Mode5Presets.h PRESETS[i-1]
    uint32_t m5_press_seq;       // 누를 때마다 +1 (같은 자세 재입력 = 새 목표)
    int32_t  clip_req;           // −1 = 그대로, 0.. = 고를 클립 (재생 중이면 거부). 1회성
};
#pragma pack(pop)
static_assert(sizeof(GuiCtrl) == 12 * 4, "GuiCtrl = gui_shm.FMT 12칸 48 바이트 (tests/test_gui_shm_layout.py)");

static constexpr int32_t GUI_CTRL_MAGIC = 0x6704;      // 0x6701 gui v1 · 0x6702 vr · 0x6703 estop 과 겹치지 않게
static constexpr int32_t GUI_CTRL_MAGIC_V1 = 0x6701;   // 옛 형식 — 읽지 않는다(한 번 알린다)
static constexpr int32_t MAGIC_GUI_RELAY = 0x6705;     // 중계 전용(하트비트) — 같은 48 B 배치
static constexpr int GUI_RELAY_STALE_TICKS = 15;       // 0.3 s @ 50 Hz
static constexpr int   GUI_RELAY_LOG_MIN_TICKS = 25;   // 0.5 s @ 50 Hz — 0x6705 «속도만 바뀐» 로그 줄의 최소 간격(poll 수)
static constexpr float GUI_RELAY_LOG_MIN_DV = 0.05f;   // 그리고 마지막으로 찍은 값에서 한 축이라도 이만큼 바뀌었을 때만

struct GuiPollState {
    uint32_t last_seq = 0;
    uint32_t last_m5_seq = 0;    // 자세가 실린 프레임만 옮긴다(PICO·replay 는 자세 칸이 0)
    bool     rebase = true;      // 체류(Mimic 진입)마다 enter() 가 세운다. 첫 poll 이 내린다
    int      stale_ticks = 0;    // 중계를 본 뒤 seq 가 안 바뀐(또는 파일이 없는) 연속 poll 수
    bool     relay_seen = false; // 마지막으로 읽은 유효 프레임이 0x6705 인가
    // 마지막으로 «로그 줄에 찍힌» GUI 속도 — 0x6704·0x6705 어느 쪽이 찍었든(중계 속도 로그의 기준). 체류 시작·
    // 끊김(zero_vel)에서는 0 — 그때 State_Mimic 이 GUI 속도 전역을 0 으로 하고 그 사실을 찍으므로 기준도 같이.
    float    last_vx = 0.f, last_vy = 0.f, last_wz = 0.f;
    bool     nonfinite_run = false;   // 0x6705 비유한 프레임이 이어지는 중(첫 번만 알린다). 유한 프레임·기준선이 내린다
    int      log_age = GUI_RELAY_LOG_MIN_TICKS;   // 마지막 GUI 로그 줄 뒤의 poll 수(이 값에서 멈춘다). 처음·체류 시작 = 다 참
                                                  // → 그 체류의 첫 속도 변화는 바로 찍는다
    bool     relay_vel_ok = false;     // 속도 0 인 0x6705 프레임을 적용했나(«속도 문» 열림) — 체류 시작·끊김(zero_vel)이 닫는다.
                                       // 열리기 전엔 0x6705 속도 = 0
    bool     relay_announced = false;  // 이 체류에서 0x6705 를 적용한 적 있나(«중계가 키보드 속도를 덮는다» 한 줄은 체류마다 한 번)
};

// 체류(Mimic 진입)마다 State_Mimic::enter() 가 부른다 — 정책 스레드가 뜨기 전이라 경합 없음.
// 첫 poll 이 기준선만 잡게 하고(rebase), 끊김 감시는 0x6705 를 다시 볼 때까지 끈다. last_seq 는 남긴다(종전 그대로).
inline void gui_poll_begin_stay(GuiPollState& st) {
    st.rebase = true;
    st.stale_ticks = 0;
    st.relay_seen = false;
    st.last_vx = st.last_vy = st.last_wz = 0.f;   // enter() 가 GUI 속도 전역(g_kb_*)을 0 으로 한다 — 파일이 없으면
                                                  // 기준선이 이 칸을 안 지우므로 여기서(안 지우면 중계 로그 일부가 빠진다)
    st.nonfinite_run = false;
    st.log_age = GUI_RELAY_LOG_MIN_TICKS;         // 새 체류의 첫 속도 변화는 바로 찍는다
    st.relay_vel_ok = false;                      // 속도 문을 닫는다 — 속도 0 인 0x6705 를 적용할 때까지 중계 속도 = 0
    st.relay_announced = false;
}

// 중계 끊김 «에피소드» 중인가 — 0x6705 를 본 뒤 seq 가 GUI_RELAY_STALE_TICKS 틱 안 바뀌어 zero_vel 이 났고, 아직 새 하트비트가
// 없다(새 seq 의 0x6705 · 0x6704 이어받기 · 체류 시작이 끝낸다). 상태 칸 relay_stale 의 원천(MaskedStatus.h) — 판정은 여기 하나.
inline bool gui_relay_stale(const GuiPollState& st) {
    return st.relay_seen && st.stale_ticks >= GUI_RELAY_STALE_TICKS;
}

struct GuiApply {
    bool apply = false;          // true 면 아래 mode_req·m5·속도를 적용한다
    int  mode_req = 0;
    int  m5_preset = 0;          // m5_press 일 때 g.m5_preset (범위 검사는 호출부)
    bool m5_press = false;
    bool set_vel = false;
    float vx = 0.f, vy = 0.f, wz = 0.f;
    bool set_foot = false;       // 0x6705 는 항상 false
    int  period_steps = 0;
    float height_scale = 0.f;
    float turn_k = 0.f;
    int  clip_req = -1;          // 0x6705 는 항상 -1
    bool zero_vel = false;       // 끊김 15 틱째 1회 — 호출부가 GUI 속도 전역만 0 으로
    bool log_line = false;
    bool rebased = false;        // 기준선만 잡았다(적용 없음) — 호출부가 안내 한 줄
    bool relay_first = false;    // 이 체류의 첫 0x6705 적용 — 호출부가 «중계가 키보드 속도를 덮는다» 한 줄
    bool vel_gate_opened = false; // 이 체류의 속도 문이 이 프레임(속도 0 인 0x6705)에서 열렸다 — 호출부가 한 줄
    bool vel_gate_closed = false; // 끊김(zero_vel)이 열려 있던 속도 문을 다시 닫았다 — 호출부가 한 줄(zero_vel 과 같은 틱)
    // 비유한 값 프레임 — 호출부가 알림 한 줄. 0x6704 = seq 가 바뀐 나쁜 프레임마다 · 0x6705 = 이어지는 비유한 프레임의
    // 첫 번만(50 Hz 하트비트라 seq 가 매 틱 바뀐다 — 정책 스레드에서 printf·fflush 50 회/s 를 막는다)
    bool rejected_nonfinite = false;
};

namespace guipoll_detail {
// 중계를 본 뒤의 «seq 안 바뀜» 한 틱. 15 틱째에 한 번만 zero_vel.
inline void stale_tick(GuiPollState& st, GuiApply& r) {
    if (!st.relay_seen) return;
    if (st.stale_ticks < GUI_RELAY_STALE_TICKS) {
        if (++st.stale_ticks == GUI_RELAY_STALE_TICKS) {
            r.zero_vel = true;
            st.last_vx = st.last_vy = st.last_wz = 0.f;
            // 속도 문도 다시 닫는다 — 같은 체류 안에서 중계가 다시 떠도(브라우저가 키를 누른 채) 새 seq 의 속도 0 프레임을
            // 한 번 볼 때까지 그 속도를 안 받는다. 열려 있던 문을 닫을 때만 알린다(이미 닫혀 있으면 조용히).
            if (st.relay_vel_ok) { st.relay_vel_ok = false; r.vel_gate_closed = true; }
        }
    }
}
}  // namespace guipoll_detail

// f == nullptr = 파일 없음·크기 불일치. 알 수 없는 magic 도 같게 본다(남의 파일).
inline GuiApply poll_gui_frame(const GuiCtrl* f, GuiPollState& st)
{
    GuiApply r;
    if (st.log_age < GUI_RELAY_LOG_MIN_TICKS) ++st.log_age;   // 정책 틱(= poll)마다 하나 — 로그 간격의 시계
    if (!f || (f->magic != GUI_CTRL_MAGIC && f->magic != MAGIC_GUI_RELAY)) {
        st.rebase = false;                                // 기준선 없음 → 첫 실제 쓰기가 먹는다
        guipoll_detail::stale_tick(st, r);
        return r;
    }
    const bool relay = (f->magic == MAGIC_GUI_RELAY);
    if (relay) {
        st.relay_seen = true;
    } else {                                              // v2 쓰는 쪽이 이어받음 → 끊김 판정 없음
        st.relay_seen = false;
        st.stale_ticks = 0;
    }
    if (st.rebase) {
        st.rebase = false;
        st.last_seq = f->seq;
        if (f->m5_preset != 0) st.last_m5_seq = f->m5_press_seq;
        st.stale_ticks = 0;
        st.last_vx = st.last_vy = st.last_wz = 0.f;
        st.nonfinite_run = false;
        r.rebased = true;
        return r;
    }
    if (!std::isfinite(f->vx) || !std::isfinite(f->vy) || !std::isfinite(f->wz) ||
        !std::isfinite(f->turn_k) || !std::isfinite(f->height_scale)) {
        if (f->seq != st.last_seq) {
            st.last_seq = f->seq;
            st.stale_ticks = 0;
            if (relay) {                                  // 이어지는 비유한 하트비트는 첫 번만 알린다
                r.rejected_nonfinite = !st.nonfinite_run;
                st.nonfinite_run = true;
            } else {
                r.rejected_nonfinite = true;              // 0x6704: 종전 그대로(나쁜 프레임마다)
            }
        } else if (relay) {
            guipoll_detail::stale_tick(st, r);
        }
        return r;
    }
    st.nonfinite_run = false;                             // 유한 프레임 — 다음 비유한 프레임은 다시 알린다
    if (f->seq == st.last_seq) {
        if (relay) guipoll_detail::stale_tick(st, r);
        return r;
    }
    st.last_seq = f->seq;
    st.stale_ticks = 0;
    r.apply = true;
    r.mode_req = f->mode_req;
    if (f->m5_preset != 0 && f->m5_press_seq != st.last_m5_seq) {
        st.last_m5_seq = f->m5_press_seq;
        r.m5_press = true;
        r.m5_preset = f->m5_preset;
    }
    r.set_vel = true;
    r.vx = f->vx; r.vy = f->vy; r.wz = f->wz;
    if (!relay) {
        r.set_foot = true;
        r.period_steps = f->period_steps;
        r.height_scale = f->height_scale;
        r.turn_k = f->turn_k;
        r.clip_req = f->clip_req;
        r.log_line = true;                                // 0x6704: 종전 그대로(적용 프레임마다)
    } else {
        if (!st.relay_announced) { st.relay_announced = true; r.relay_first = true; }
        // 속도 문: 이 체류에서 속도 0 인 프레임을 적용하기 전엔 중계 속도를 0 으로 친다(그 프레임 자체도 0 이다).
        // 기준선 프레임(rebase)은 세지 않는다 — 진입 «뒤» 새 seq 로 온 0 을 봐야 연다. mode_req·자세는 위에서 이미 실렸다.
        if (!st.relay_vel_ok && f->vx == 0.f && f->vy == 0.f && f->wz == 0.f) {
            st.relay_vel_ok = true;
            r.vel_gate_opened = true;
        }
        if (!st.relay_vel_ok) { r.vx = 0.f; r.vy = 0.f; r.wz = 0.f; }
        // 속도만 바뀐 줄: GUI_RELAY_LOG_MIN_TICKS 틱에 한 번까지 · 마지막으로 찍은 값에서 GUI_RELAY_LOG_MIN_DV 이상일 때만.
        // 비교는 «적용할 값»(문이 닫혀 있으면 0)과 — 줄에 찍히는 것이 그 값이다.
        const bool vel_due = st.log_age >= GUI_RELAY_LOG_MIN_TICKS &&
                             (std::fabs(r.vx - st.last_vx) >= GUI_RELAY_LOG_MIN_DV ||
                              std::fabs(r.vy - st.last_vy) >= GUI_RELAY_LOG_MIN_DV ||
                              std::fabs(r.wz - st.last_wz) >= GUI_RELAY_LOG_MIN_DV);
        r.log_line = (f->mode_req != 0) || r.m5_press || vel_due;   // 모드 요청·자세 누름은 바로
    }
    // 줄을 찍는 프레임만 기준을 옮긴다(어느 쪽이 찍었든 — 그 줄에 지금 GUI 속도가 찍힌다)
    if (r.log_line) {
        st.last_vx = r.vx; st.last_vy = r.vy; st.last_wz = r.wz;
        st.log_age = 0;
    }
    return r;
}
