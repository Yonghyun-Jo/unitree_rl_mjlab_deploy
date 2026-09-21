#pragma once
// ModeRuntime.h — «지금 어느 모드인가» 의 유일한 자리. 순수 C++ (DDS·ONNX·Eigen 무의존 = 단위 테스트 가능).
//   · 조작자 요청(request)은 표의 이탈 조건과 슬롯이 아는 모드로 걸러진다. 안전 폴백(force)은 거르지 않는다.
//   · 코드 어디서도 모드를 번호로 비교하지 않는다 — row() 의 성질을 묻는다 (rules/ADDING_A_MODE.md).
#include "ModeTable.h"
#include <algorithm>
#include <vector>

namespace g1 {

// 이탈 판정에 쓰는 로봇 상태. 기본값 = «서 있다» — B1 에는 높이 추정이 없어 기본값이 그대로 쓰인다(B2 가 실측을 채운다).
struct ExitContext {
  float z_fk = 1e9f;              // 골반 높이 추정 [m]
  float tilt_deg = 0.f;           // 기울기 [deg]
  bool  m5_standing_hold = true;  // mode5: 직립 프리셋의 hold 단계인가
};
struct ModeResult { bool accepted; const char* reason; };

inline constexpr float EXIT_MIN_Z = 0.65f;          // spec §4.6
inline constexpr float EXIT_MAX_TILT_DEG = 30.f;

class ModeRuntime {
 public:
  void set_supported(const std::vector<int>& modes) { supported_ = modes; }
  bool supports(int m) const { return std::find(supported_.begin(), supported_.end(), m) != supported_.end(); }

  int mode() const { return mode_; }
  int requested() const { return requested_; }
  const mode_table::Row& row() const { return mode_table::row(mode_); }

  ModeResult request(int m, const ExitContext& ctx = {}) {
    if (!mode_table::valid(m)) return {false, "없는 모드"};
    if (!supports(m))          return {false, "이 슬롯(ONNX)이 모르는 모드"};
    if (m != mode_) {
      using mode_table::Exit;
      const bool upright = ctx.z_fk >= EXIT_MIN_Z && ctx.tilt_deg < EXIT_MAX_TILT_DEG;
      switch (row().exit) {
        case Exit::Always: break;
        case Exit::Upright:
          if (!upright) return {false, "먼저 직립 (높이·기울기 조건)"};
          break;
        case Exit::StandingHold:
          // 저자세끼리(→ 다른 ground_capable 모드)는 여기서 막지 않는다 — 그 가드는 들어가는 쪽 모드의 몫(B2).
          if (mode_table::row(m).safety != mode_table::Safety::GroundCapable && !(ctx.m5_standing_hold && upright))
            return {false, "먼저 직립 버튼 (직립 유지 상태에서만 나간다)"};
          break;
        case Exit::ViaGround:
          if (mode_table::row(m).exit != Exit::StandingHold) return {false, "ground 모드를 거쳐서만 나간다"};
          break;
      }
      mode_ = m; switched_ = true;
    }
    requested_ = m;
    return {true, ""};
  }

  // 안전 폴백 — 가드를 타지 않고, 조작자의 요청(requested)은 남긴다(수동 복귀 판정용).
  void force(int m) { if (mode_table::valid(m) && m != mode_) { mode_ = m; switched_ = true; } }

  // 직전 호출 이후 모드가 바뀌었으면 한 번 true.
  bool consume_switch() { const bool s = switched_; switched_ = false; return s; }

  int  clip_id() const { return clip_id_; }
  bool select_clip(int id, int n_clips) { if (id < 0 || id >= n_clips) return false; clip_id_ = id; return true; }

 private:
  int mode_ = 1, requested_ = 1, clip_id_ = 0;
  bool switched_ = false;
  std::vector<int> supported_ = {1, 2, 3, 4};      // 계약 v1 슬롯의 기본
};

}  // namespace g1
