#pragma once
// ModeTable.h — 🔴 생성 파일. 손으로 고치지 않는다.  python3 deploy/scripts/gen_mode_table_header.py --write
//   학습 사실: mjlab_g1_motion/mode_spec.py @ 45dc2c9
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

}  // namespace mode_table
