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
      // 기본은 «거부». 각 case 가 허용을 켠다 — switch 에 default 를 두지 않는 것은 새 Exit 값이
      // 생겼을 때 빠진 case 를 잡으려는 것. 단위 테스트 러너(run_unit_tests.sh)가 이 헤더를
      // -Werror=switch 로 빌드하므로 빠뜨리면 거기서 빌드가 죽는다. 🔴 컨트롤러 자체의 CMake
      // 빌드는 이 경고를 켜지 않는다(-Wall/-Wswitch 없음) — 그래도 놓치면 여기 런타임은
      // fail-closed(ok=false)로 그 모드에서 나가는 전환을 전부 거부한다.
      bool ok = false;
      const char* why = "알 수 없는 이탈 조건";
      switch (row().exit) {
        case Exit::Always:
          ok = true;
          break;
        case Exit::Upright:
          ok = upright; why = "먼저 직립 (높이·기울기 조건)";
          break;
        case Exit::StandingHold:
          // 저자세끼리(→ 다른 ground_capable 모드)는 여기서 막지 않는다 — 그 가드는 들어가는 쪽 모드의 몫(B2).
          ok = mode_table::row(m).safety == mode_table::Safety::GroundCapable || (ctx.m5_standing_hold && upright);
          why = "먼저 직립 버튼 (직립 유지 상태에서만 나간다)";
          break;
        case Exit::ViaGround:
          ok = mode_table::row(m).exit == Exit::StandingHold; why = "ground 모드를 거쳐서만 나간다";
          break;
      }
      if (!ok) return {false, why};
      mode_ = m;
    }
    requested_ = m;
    return {true, ""};
  }

  // 안전 폴백 — 가드를 타지 않고, 조작자의 요청(requested)은 남긴다(수동 복귀 판정용).
  void force(int m) { if (mode_table::valid(m)) mode_ = m; }

  // 직전 consume 때의 모드와 지금이 다르면 true — 한 틱 안에서 바뀌었다가 되돌아온 것은 전환이 아니다.
  // (조작 채널이 모드를 요청한 같은 틱에 안전 폴백이 그것을 되돌리는 경우가 그렇다. 전이마다 래치를
  //  세우면 그 틱이 «전환» 으로 보여 crossfade·램프가 매 틱 재무장한다.)
  bool consume_switch() { const bool s = mode_ != last_consumed_; last_consumed_ = mode_; return s; }

  int  clip_id() const { return clip_id_; }
  bool select_clip(int id, int n_clips) { if (id < 0 || id >= n_clips) return false; clip_id_ = id; return true; }

 private:
  int mode_ = 1, requested_ = 1, clip_id_ = 0;
  int last_consumed_ = 1;                         // consume_switch() 가 마지막으로 본 모드
  std::vector<int> supported_ = {1, 2, 3, 4};      // 계약 v1 슬롯의 기본
};

}  // namespace g1
