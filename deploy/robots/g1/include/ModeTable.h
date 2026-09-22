#pragma once
// ModeTable.h — 🔴 생성 파일. 손으로 고치지 않는다.  python3 deploy/scripts/gen_mode_table_header.py --write
//   학습 사실: mjlab_g1_motion/mode_spec.py @ 45dc2c9c9be0
//   배포 사실: deploy/robots/g1/config/modes.yaml
// C++ 은 모드를 «번호» 로 비교하지 않고 이 표의 «성질» 을 묻는다 (rules/ADDING_A_MODE.md).
#include <array>

namespace mode_table {

enum class RefSource : unsigned char { None, Vr, Clip };
enum class FootZ : unsigned char { None, Gen, Ref };
enum class Safety : unsigned char { UprightOnly, GroundCapable };
enum class Exit : unsigned char { Always, Upright, StandingHold, ViaGround };

inline constexpr int MASK_DIM = 8;
inline constexpr int N_MODES = 6;

struct Row {
  int id; const char* name; char key;
  std::array<float, MASK_DIM> bits;
  bool track_upper, track_lower, base_vel_live, motion_preview, foot_z_live, mode5_cmd_live, crawl_cmd_live;
  RefSource ref_source; FootZ foot_z;
  bool arm_blend_enter, crossfade_enter;
  Safety safety; Exit exit;
  const char* gait_key;
};

inline constexpr std::array<Row, N_MODES> ROWS = {{
  {1, "loco", '1', {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, false, false, true, false, true, false, false, RefSource::None, FootZ::Gen, true, false, Safety::UprightOnly, Exit::Always, "mode1"},
  {2, "upper", '2', {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, true, false, true, false, true, false, false, RefSource::Vr, FootZ::Gen, false, true, Safety::UprightOnly, Exit::Always, "mode2"},
  {3, "track", '3', {{1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, true, true, false, false, true, false, false, RefSource::Vr, FootZ::Ref, false, true, Safety::UprightOnly, Exit::Always, "mode3"},
  {4, "playback", '4', {{1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, true, true, false, true, true, false, false, RefSource::Clip, FootZ::Ref, false, true, Safety::GroundCapable, Exit::Upright, "mode4"},
  {5, "ground", '5', {{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}}, false, false, false, false, false, true, false, RefSource::None, FootZ::None, false, true, Safety::GroundCapable, Exit::StandingHold, "mode5"},
  {6, "crawl", '6', {{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}}, false, false, true, false, false, false, true, RefSource::None, FootZ::None, false, true, Safety::GroundCapable, Exit::ViaGround, "mode6"},
}};

inline constexpr bool valid(int mode) { return mode >= 1 && mode <= N_MODES; }
// 범위 밖은 안전측(mode1 = 명령만으로 서서 걷는 모드)으로.
inline constexpr const Row& row(int mode) { return ROWS[valid(mode) ? mode - 1 : 0]; }

// ── 관측 계약 v2 (spec §4.2) ─────────────────────────────────────────────────
//   미리보기 오프셋: mjlab_g1_motion/src/mjlab_g1_motion/tasks/g1_mimic_env.py TAR_MOTION_STEPS_PRIV
inline constexpr int N_DOF = 29;
inline constexpr int MODE5_CMD_DIM = 53;
struct Slot { int lo, hi; };
inline constexpr Slot M5_BASE_VEL{0, 3};
inline constexpr Slot M5_Z{3, 4};
inline constexpr Slot M5_Z_MASK{4, 5};
inline constexpr Slot M5_C{5, 18};
inline constexpr Slot M5_M{18, 31};
inline constexpr Slot M5_G_TORSO{31, 34};
inline constexpr Slot M5_G_PELVIS{34, 37};
inline constexpr Slot M5_GM{37, 39};
inline constexpr Slot M5_SITE_Z{39, 52};
inline constexpr Slot M5_T_GOAL{52, 53};
inline constexpr float M5_T_GOAL_MAX = 3.0f;
inline constexpr int MOTION_STEP_DIM = 6 + N_DOF;
inline constexpr int N_PREVIEW = 20;
inline constexpr std::array<int, N_PREVIEW> PREVIEW_OFFSETS = {{1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95}};
inline constexpr int MOTION_BLOCK_DIM = 1 + MOTION_STEP_DIM + N_PREVIEW * MOTION_STEP_DIM;

}  // namespace mode_table
