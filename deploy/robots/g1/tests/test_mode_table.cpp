// test_mode_table.cpp — 생성된 표가 (1) 지금 배포의 «번호 비교» 와 mode1~4 에서 같은 답을 내고
// (2) 학습 표(mode_spec.py)의 mode5·6 을 담고 있는지.
//   cd deploy/robots/g1/tests && g++ -std=gnu++17 -O2 -I../include test_mode_table.cpp -o /tmp/tmt && /tmp/tmt
#include "ModeTable.h"
#include <cstdio>
#include <cstring>
using namespace mode_table;
static int fail = 0;
#define CHK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fail; } } while (0)

int main() {
    CHK(N_MODES >= 6); CHK(MASK_DIM == 8);
    // (1) 옛 번호 비교와의 동치 — 이 계획이 옮기는 23곳의 진리표
    for (int m = 1; m <= 4; ++m) {
        const Row& r = row(m);
        CHK(r.id == m);
        CHK(r.track_upper == (m >= 2));                         // g_mask_upper()
        CHK(r.track_lower == (m >= 3));                         // g_mask_lower()
        CHK((r.ref_source == RefSource::Clip) == (m == 4));     // g_is_demo() (옛 5·6 은 mode4+clip_id)
        CHK(r.base_vel_live == (m <= 2));                       // update(): cmd_mode>=3 -> bv=0
        CHK(r.arm_blend_enter == (m == 1));                     // notify_mode_switch: new_mode==1
        CHK(r.crossfade_enter == (m >= 2));                     // notify_mode_switch: new_mode>=2
        CHK((r.foot_z == FootZ::Ref) == (m >= 3));              // ref_foot_height: mode>=3 = 참조 발 z
        CHK((r.foot_z == FootZ::Gen) == (m <= 2));
        CHK(r.bits[0] == (m >= 2 ? 1.f : 0.f)); CHK(r.bits[1] == (m >= 3 ? 1.f : 0.f));
    }
    CHK(std::strcmp(row(1).gait_key, "mode1") == 0); CHK(std::strcmp(row(4).gait_key, "mode4") == 0);
    // (2) 학습 표의 새 모드
    CHK(row(4).motion_preview && row(4).bits[2] == 1.f);
    CHK(row(5).mode5_cmd_live && !row(5).track_upper && !row(5).track_lower && !row(5).base_vel_live);
    CHK(row(5).bits[3] == 1.f && row(5).foot_z == FootZ::None && row(5).safety == Safety::GroundCapable);
    CHK(row(5).exit == Exit::StandingHold && row(5).ref_source == RefSource::None);
    CHK(row(6).base_vel_live && row(6).crawl_cmd_live && row(6).bits[4] == 1.f && row(6).exit == Exit::ViaGround);
    for (int m = 1; m <= 3; ++m) CHK(row(m).safety == Safety::UprightOnly && row(m).exit == Exit::Always);
    CHK(row(4).exit == Exit::Upright);
    // 범위 밖은 안전측(mode1 행)
    CHK(!valid(0) && !valid(N_MODES + 1) && valid(1) && valid(N_MODES));
    CHK(row(0).id == 1 && row(99).id == 1);
    std::printf(fail ? "FAILED (%d)\n" : "OK test_mode_table\n", fail);
    return fail ? 1 : 0;
}
