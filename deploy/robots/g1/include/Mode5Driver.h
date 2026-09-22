#pragma once
// Mode5Driver.h — mode5 자세 버튼 → 53칸 명령. 학습 원장 mjlab_g1_motion/mode5_presets.GoalDriver 의
// C++ 판 (로직 불변, spec §4.4):
//   press(p)  → transit · t=0 · 도착 누적 0
//   tick(z,g) → transit 이면 도착 판정(|z−z*|<TOL ∨ (g·g* ≥ ARRIVE_G_DOT ∧ |z−z*| < ARRIVE_Z_LOOSE))을
//               누적해 HOLD_S 이상이면 hold 로(t=0). 명령 = 프리셋 + z_mask(transit 이면 프리셋의 값, hold 면 0)
//               + t_goal = min(t, T_GOAL_MAX). 그 뒤 t += dt.
// 🔴 수치 규약까지 같게 한다: 시간·도착 누적·높이 차는 double(파이썬 float), g 내적은 float(torch float32).
//    이 규약이 어긋나면 도착 틱이 하나 밀려 hold 전환 시각이 달라진다 — 골든이 그것을 잡는다.
#include "Mode5Presets.h"
#include "ModeTable.h"
#include <algorithm>
#include <array>
#include <cmath>

// 두 생성 헤더가 같은 53칸 계약을 말하는지 컴파일 때 묶는다. Mode5Presets.h(mode5_presets.py + mode_spec.py 에서)와
// ModeTable.h(mode_spec.py + modes.yaml 에서)는 생성기가 따로라 각자의 --check 만으로는 한쪽만 재생성된 채
// 둘 다 통과할 수 있다 — 그러면 드라이버가 채우는 z_mask·t_goal 칸이 관측 계약의 칸과 어긋난다(최종 검토 M-15).
static_assert(m5::CMD_DIM == mode_table::MODE5_CMD_DIM, "Mode5Presets.h CMD_DIM != ModeTable.h MODE5_CMD_DIM");
static_assert(m5::SLOT_Z_MASK == mode_table::M5_Z_MASK.lo && mode_table::M5_Z_MASK.hi == m5::SLOT_Z_MASK + 1,
              "Mode5Presets.h SLOT_Z_MASK != ModeTable.h M5_Z_MASK");
static_assert(m5::SLOT_T_GOAL == mode_table::M5_T_GOAL.lo && mode_table::M5_T_GOAL.hi == m5::SLOT_T_GOAL + 1,
              "Mode5Presets.h SLOT_T_GOAL != ModeTable.h M5_T_GOAL");
static_assert(m5::T_GOAL_MAX == static_cast<double>(mode_table::M5_T_GOAL_MAX),
              "Mode5Presets.h T_GOAL_MAX != ModeTable.h M5_T_GOAL_MAX");

namespace g1 {

class Mode5Driver {
 public:
  using Cmd = std::array<float, m5::CMD_DIM>;

  explicit Mode5Driver(double dt = 0.02) : dt_(dt) {}

  // 버튼 = 새 목표. 같은 자세를 다시 눌러도 새 목표(트랜짓 재시작) — 학습의 «목표가 바뀌면 t_goal 리셋».
  bool press(int preset) {
    if (preset < 0 || preset >= m5::N_PRESETS) return false;
    preset_ = preset; hold_ = false; t_ = 0.0; arrive_ = 0.0;
    return true;
  }
  void reset() { preset_ = -1; hold_ = false; t_ = 0.0; arrive_ = 0.0; }

  bool active() const { return preset_ >= 0; }
  int preset() const { return preset_; }
  bool holding() const { return active() && hold_; }
  // 이탈 조건(ModeRuntime 의 Exit::StandingHold)이 묻는 것: 진입 자세(직립)에 도착해 유지 중인가.
  bool standing_hold() const { return preset_ == m5::ENTER_PRESET && hold_; }

  bool arrived(double z_now, const std::array<float, 3>& g) const {
    const m5::Preset& p = m5::PRESETS[preset_];
    const double dz = std::fabs(z_now - p.z);
    if (dz < m5::TOL) return true;
    const float dot = g[0] * p.g_pelvis_unit[0] + g[1] * p.g_pelvis_unit[1] + g[2] * p.g_pelvis_unit[2];
    return static_cast<double>(dot) >= m5::ARRIVE_G_DOT && dz < m5::ARRIVE_Z_LOOSE;
  }

  // 한 제어 스텝(50 Hz). z_now = 골반 높이 추정, g = 골반 좌표계 중력(projected_gravity_b).
  // press() 전이면 0 명령 — 호출자는 mode5 가 아니면 부르지 않는다.
  Cmd tick(double z_now, const std::array<float, 3>& g) {
    Cmd cmd{};
    if (!active()) return cmd;
    const m5::Preset& p = m5::PRESETS[preset_];
    if (!hold_) {
      arrive_ = arrived(z_now, g) ? arrive_ + dt_ : 0.0;
      if (arrive_ >= m5::HOLD_S) { hold_ = true; t_ = 0.0; arrive_ = 0.0; }
    }
    cmd = p.cmd;
    cmd[m5::SLOT_Z_MASK] = hold_ ? 0.0f : p.transit_z_mask;
    cmd[m5::SLOT_T_GOAL] = static_cast<float>(std::min(t_, m5::T_GOAL_MAX));
    t_ += dt_;
    return cmd;
  }

 private:
  double dt_;
  int preset_ = -1;
  bool hold_ = false;
  double t_ = 0.0, arrive_ = 0.0;
};

}  // namespace g1
