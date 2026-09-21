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
    CHK(rt.request(6, moving).accepted && rt.mode() == 6);      // 5 -> 6 은 이탈 조건 밖(짝 spec: 네발 hold 가드는 B2)
    CHK(!rt.request(1).accepted && rt.mode() == 6);             // 6 은 5 로만 나간다
    CHK(rt.request(5).accepted && rt.request(1).accepted);
    // 안전 폴백: 가드 무시, 조작자 요청은 그대로 남는다
    rt.request(4); rt.consume_switch();
    rt.force(1);
    CHK(rt.mode() == 1 && rt.requested() == 4 && rt.consume_switch());
    // 클립 선택
    CHK(rt.clip_id() == 0 && rt.select_clip(2, 3) && rt.clip_id() == 2);
    CHK(!rt.select_clip(3, 3) && !rt.select_clip(-1, 3) && rt.clip_id() == 2);
    std::printf(fail ? "FAILED (%d)\n" : "OK test_mode_runtime\n", fail);
    return fail ? 1 : 0;
}
