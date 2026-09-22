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
    // 이탈 조건 — mode4(exit=upright): 낮거나 기울면 못 나간다
    CHK(rt.request(4).accepted);
    ExitContext low;  low.z_fk = 0.30f;
    ExitContext tilt; tilt.tilt_deg = 45.f;
    CHK(!rt.request(1, low).accepted && !rt.request(1, tilt).accepted && rt.mode() == 4);
    CHK(rt.request(1, ExitContext{}).accepted && rt.mode() == 1);
    // mode5(exit=standing_hold) · mode6(exit=via_ground) — 계약 v2 슬롯
    rt.set_supported({1, 2, 3, 4, 5, 6});
    CHK(rt.request(5).accepted);
    ExitContext moving; moving.m5_standing_hold = false;
    CHK(!rt.request(1, moving).accepted && rt.mode() == 5);
    // 5 -> 4: mode4 는 ground_capable 이지만 via_ground 는 아니다 — 네발 hold 에선 거부,
    // 직립 hold 에서만 허용 (Ruling 33, 옛 GroundCapable 바이패스 버그 회귀 방지)
    ExitContext crawl_hold; crawl_hold.m5_standing_hold = false; crawl_hold.z_fk = 0.42f; crawl_hold.tilt_deg = 64.f;
    ModeResult r54 = rt.request(4, crawl_hold);
    CHK(!r54.accepted && std::strstr(r54.reason, "직립 버튼") && rt.mode() == 5);
    ExitContext upright_hold; upright_hold.m5_standing_hold = true; upright_hold.z_fk = 0.76f; upright_hold.tilt_deg = 3.f;
    CHK(rt.request(4, upright_hold).accepted && rt.mode() == 4);
    CHK(rt.request(5, ExitContext{}).accepted && rt.mode() == 5);  // mode4 는 exit=upright — 직립이면 5 로 복귀
    CHK(rt.request(6, moving).accepted && rt.mode() == 6);      // 5 -> 6 은 via_ground 라 hold 없이도 허용(불변)
    CHK(!rt.request(1).accepted && rt.mode() == 6);             // 6 은 5 로만 나간다
    CHK(rt.request(5).accepted && rt.request(1).accepted);
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
