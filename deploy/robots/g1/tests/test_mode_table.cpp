// test_mode_table.cpp — 생성된 표가 (1) 지금 배포의 «번호 비교» 와 mode1~4 에서 같은 답을 내고
// (2) 학습 표(mode_spec.py)의 mode5·6 을 담고 있는지.
//   cd deploy/robots/g1/tests && g++ -std=gnu++17 -O2 -I../include test_mode_table.cpp -o /tmp/tmt && /tmp/tmt
#include "ModeTable.h"
#include <cstdio>
#include <cstring>
using namespace mode_table;
static int fail = 0;
#define CHK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fail; } } while (0)

// ── 계약 v2 상수 (학습 원장에서 생성) — 값이 바뀌면 ONNX 입력 배치가 바뀐다. 바뀌었다면 여기도 같이.
static_assert(mode_table::N_DOF == 29, "G1 29-DOF");
static_assert(mode_table::MODE5_CMD_DIM == 53, "mode5 명령 53칸 (mode_spec.MODE5_CMD_DIM)");
static_assert(mode_table::M5_Z_MASK.lo == 4 && mode_table::M5_Z_MASK.hi == 5, "z_mask 슬롯");
static_assert(mode_table::M5_T_GOAL.lo == 52 && mode_table::M5_T_GOAL.hi == 53, "t_goal 슬롯 = 끝");
static_assert(mode_table::M5_C.hi - mode_table::M5_C.lo == 13 && mode_table::M5_M.lo == mode_table::M5_C.hi,
              "«c 13 뒤 m 13» (interleave 아님)");
static_assert(mode_table::MOTION_STEP_DIM == 35, "미리보기 한 스텝 = root6 + dof29");
static_assert(mode_table::N_PREVIEW == 20, "미리보기 20스텝");
static_assert(mode_table::PREVIEW_OFFSETS[0] == 1 && mode_table::PREVIEW_OFFSETS[19] == 95, "[1, 5, …, 95]");
static_assert(mode_table::MOTION_BLOCK_DIM == 736, "on 1 + k1 35 + 20×35");

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
    CHK(row(5).ref_source == RefSource::None);
    CHK(row(6).base_vel_live && row(6).crawl_cmd_live && row(6).bits[4] == 1.f);
    for (int m = 1; m <= 3; ++m) CHK(row(m).safety == Safety::UprightOnly);
    // 이탈 조건은 지금 전부 always (2026-09-23 «모드 사이는 언제든지» — modes.yaml 의 exit 열).
    // 조건 자체의 거동은 test_mode_runtime 이 순수 함수(exit_allowed/enter_allowed)로 네 값 다 돌린다.
    for (int m = 1; m <= N_MODES; ++m) CHK(row(m).exit == Exit::Always);
    // 범위 밖은 안전측(mode1 행)
    CHK(!valid(0) && !valid(N_MODES + 1) && valid(1) && valid(N_MODES));
    CHK(row(0).id == 1 && row(99).id == 1);
    std::printf(fail ? "FAILED (%d)\n" : "OK test_mode_table\n", fail);
    return fail ? 1 : 0;
}
