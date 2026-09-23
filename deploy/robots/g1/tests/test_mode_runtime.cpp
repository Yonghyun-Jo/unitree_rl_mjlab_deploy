// test_mode_runtime.cpp — 모드 요청의 허용/거부, 폴백, 전환 감지, 클립 선택.
//   cd deploy/robots/g1/tests && g++ -std=gnu++17 -O2 -I../include test_mode_runtime.cpp -o /tmp/tmr && /tmp/tmr
#include "ModeRuntime.h"
#include <cstdio>
#include <cstring>
using namespace g1;
static int fail = 0;
#define CHK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fail; } } while (0)

int main() {
    ModeRuntime rt;                                  // 기본: mode1, 지원 = {1,2,3,4} (계약 v1)
    CHK(rt.mode() == 1 && rt.requested() == 1 && !rt.consume_switch());
    CHK(rt.supports(4) && !rt.supports(5));
    // 지원하는 모드로의 전환 + 전환 감지는 한 번만
    CHK(rt.request(3).accepted && rt.mode() == 3 && rt.requested() == 3);
    CHK(rt.consume_switch() && !rt.consume_switch());
    CHK(rt.row().track_lower);
    // 같은 모드 재요청 = 허용, 전환 아님
    CHK(rt.request(3).accepted && !rt.consume_switch());
    // 슬롯이 모르는 모드 / 없는 모드 = 거부, 상태 불변
    ModeResult r = rt.request(5);
    CHK(!r.accepted && std::strstr(r.reason, "슬롯") && rt.mode() == 3);
    CHK(!rt.request(0).accepted && !rt.request(99).accepted && rt.mode() == 3);
    // ── 지금 표의 데이터: 이탈·진입 조건이 전부 always (2026-09-23 사용자 지시 «모드 사이는 언제든지») ──
    // 🔴 이 블록은 «표가 지금 무엇인가» 를 적어 둔 것이다. 다시 조이면(modes.yaml 의 exit 열) 여기가 깨진다 —
    //    그때는 아래 «조건 자체» 블록이 그 거동을 이미 보증하고 있으니, 이 기대값만 고치면 된다.
    for (int m = 1; m <= mode_table::N_MODES; ++m)
        CHK(mode_table::row(m).exit == mode_table::Exit::Always);
    ExitContext low;  low.z_fk = 0.30f;
    ExitContext tilt; tilt.tilt_deg = 45.f;
    ExitContext sitting; sitting.z_fk = 0.45f; sitting.tilt_deg = 20.f;
    ExitContext standing; standing.z_fk = 0.76f; standing.tilt_deg = 3.f;
    CHK(rt.request(4).accepted);
    CHK(rt.request(1, low).accepted && rt.mode() == 1);          // 낮아도 나간다
    CHK(rt.request(4, tilt).accepted && rt.mode() == 4);         // 기울어도 들어간다
    CHK(rt.request(1, sitting).accepted && rt.mode() == 1);
    rt.consume_switch();

    // ── 조건 «자체» — 표에 그 값이 없어도(데이터로 꺼 두어도) 여기서 전부 돌려 본다 ──
    //    이 순수 함수들(ModeRuntime.h exit_allowed / enter_allowed)이 request() 의 판정이다.
    {
        using mode_table::Exit; using mode_table::Safety;
        mode_table::Row from{}, to{};
        const char* why = "";
        // Always = 언제나 나간다
        from.exit = Exit::Always;  to.exit = Exit::Always;
        CHK(exit_allowed(from, to, low, &why) && exit_allowed(from, to, standing, &why));
        // Upright = 직립일 때만 나가고, 들어갈 때도 직립이어야 한다 (클립 재생 모드가 쓰던 조건)
        from.exit = Exit::Upright;
        CHK(!exit_allowed(from, to, low, &why) && !exit_allowed(from, to, tilt, &why));
        CHK(exit_allowed(from, to, standing, &why) && std::strstr(why, "직립"));
        to.exit = Exit::Upright;
        CHK(!enter_allowed(to, sitting) && !enter_allowed(to, low) && enter_allowed(to, standing));
        to.exit = Exit::Always;
        CHK(enter_allowed(to, sitting));
        // StandingHold = 직립 «버튼 유지» + 직립에서만. 단 땅을 거쳐 들어가는 모드로는 그냥 나간다 (mode5 가 쓰던 조건)
        from.exit = Exit::StandingHold;
        ExitContext moving_ctx; moving_ctx.m5_standing_hold = false; moving_ctx.z_fk = 0.76f; moving_ctx.tilt_deg = 3.f;
        CHK(!exit_allowed(from, to, moving_ctx, &why) && std::strstr(why, "직립 버튼"));
        ExitContext hold_ctx; hold_ctx.m5_standing_hold = true; hold_ctx.z_fk = 0.76f; hold_ctx.tilt_deg = 3.f;
        CHK(exit_allowed(from, to, hold_ctx, &why));
        CHK(!exit_allowed(from, to, sitting, &why));                    // 버튼은 직립인데 실측이 낮으면 거부
        to.exit = Exit::ViaGround;
        CHK(exit_allowed(from, to, moving_ctx, &why));                  // 땅 모드로는 hold 없이도
        // ViaGround = 바닥 가능 모드로만 나간다 (mode6 가 쓰던 조건 — 나갈 곳이 없는 방이 되지 않게
        //             «StandingHold 모드» 가 아니라 «바닥 가능» 으로 본다: 2026-09-23 exit 완화 때 고침)
        from.exit = Exit::ViaGround;
        to.exit = Exit::Always; to.safety = Safety::GroundCapable;
        CHK(exit_allowed(from, to, moving_ctx, &why));
        to.safety = Safety::UprightOnly;
        CHK(!exit_allowed(from, to, moving_ctx, &why) && std::strstr(why, "바닥 모드"));
    }

    // 체류 시작 (I-1(b)): 바닥 모드 중 스스로 일어서는 명령이 없는 행(클립 재생·기기)으로는 체류를 시작하지 않는다
    CHK(ModeRuntime::may_start_stay_in(mode_table::ROWS[0]));                             // 폴백(첫 행)은 시작할 수 있다
    CHK(ModeRuntime::may_start_stay_in(mode_table::row(1)) && ModeRuntime::may_start_stay_in(mode_table::row(5)));
    CHK(!ModeRuntime::may_start_stay_in(mode_table::row(4)) && !ModeRuntime::may_start_stay_in(mode_table::row(6)));
    {
        ModeRuntime s;                                    // 지난 체류가 mode4 로 끝났다(넘어짐 → Passive)
        CHK(s.request(4, standing).accepted && s.consume_switch());
        const char* why = s.begin_stay(1);
        CHK(why && std::strstr(why, "바닥 모드") && s.mode() == 1);    // 재진입은 폴백으로 시작
        CHK(s.begin_stay(1) == nullptr && s.mode() == 1);              // 이미 폴백이면 그대로
        ModeRuntime u;                                    // 슬롯이 모르는 모드(v2 에서 5 로 나갔다가 v1 슬롯)
        u.set_supported({1, 2, 3, 4, 5}); CHK(u.request(5).accepted);
        u.set_supported({1, 2, 3, 4});
        const char* why_u = u.begin_stay(1);
        CHK(why_u && std::strstr(why_u, "모른다") && u.mode() == 1);
        ModeRuntime m5;                                   // mode5 는 체류를 시작할 수 있다(진입 자세 = 직립 버튼)
        m5.set_supported({1, 4, 5}); CHK(m5.request(5).accepted);
        CHK(m5.begin_stay(1) == nullptr && m5.mode() == 5);
    }
    rt.set_supported({1, 2, 3, 4, 5, 6});
    CHK(rt.request(5).accepted && rt.request(6).accepted && rt.request(1).accepted);   // 6 도 언제나 나간다(표가 always)

    // 클립 전환 에지 — 모드 전환과 같은 모양(정책 루프가 한 틱에 한 번 소비해 전환 처리를 태운다)
    {
        ModeRuntime c;
        CHK(!c.consume_clip_switch());                     // 처음엔 변화 없음
        CHK(c.select_clip(2, 4) && c.consume_clip_switch() && !c.consume_clip_switch());
        CHK(c.select_clip(2, 4) && !c.consume_clip_switch());   // 같은 칸 재선택 = 전환 아님
        CHK(!c.select_clip(9, 4) && !c.consume_clip_switch());  // 없는 칸 = 상태 불변
    }

    // 안전 폴백: 가드 무시, 조작자 요청은 그대로 남는다
    rt.request(4); rt.consume_switch();
    rt.force(1);
    CHK(rt.mode() == 1 && rt.requested() == 4 && rt.consume_switch());
    // 클립 선택
    CHK(rt.clip_id() == 0 && rt.select_clip(2, 3) && rt.clip_id() == 2);
    CHK(!rt.select_clip(3, 3) && !rt.select_clip(-1, 3) && rt.clip_id() == 2);

    // ── 전환 감지는 «직전 consume 때의 모드 ↔ 지금» 이다 (전이마다 세는 래치가 아니다) ──
    // 왜: 조작 채널(50 Hz)이 모드를 요청한 같은 틱에 안전 폴백이 되돌리면, 래치식은 그 틱을
    //     «전환» 으로 보고 crossfade·base_vel 램프를 매 틱 재무장시킨다(= 조작이 안 먹는다).
    {
        ModeRuntime e;                                    // mode1 에서 시작
        CHK(!e.consume_switch());                         // 아무 일도 없었다
        e.request(2); e.force(1);                         // 요청 -> 같은 틱에 가드가 되돌림
        CHK(e.mode() == 1 && e.requested() == 2 && !e.consume_switch());   // 전환 아님
        CHK(e.request(2).accepted && e.consume_switch()); // 이제 진짜로 달라졌다
        e.request(3); e.request(2);                       // 갔다가 그 틱 안에 되돌아옴
        CHK(e.mode() == 2 && !e.consume_switch());        // 전환 아님
        CHK(e.request(3).accepted);
        CHK(e.consume_switch() && !e.consume_switch());   // 한 번만
    }
    std::printf(fail ? "FAILED (%d)\n" : "OK test_mode_runtime\n", fail);
    return fail ? 1 : 0;
}
