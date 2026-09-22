#pragma once
// ModeRuntime.h — «지금 어느 모드인가» 의 유일한 자리. 순수 C++ (DDS·ONNX·Eigen 무의존 = 단위 테스트 가능).
//   · 조작자 요청(request)은 표의 이탈 조건(지금 모드의 exit)·진입 조건(목적지의 exit — upright 면 직립에서만)과
//     슬롯이 아는 모드로 걸러진다. 안전 폴백(force)은 거르지 않는다.
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
          // 직립 유지 없이 나갈 수 있는 곳은 «땅을 거쳐 들어가는» 모드(row(m).exit==ViaGround, 지금은
          // 예약된 mode6)뿐이다 — 그 모드 자체가 네발 진입을 전제하기 때문. 나머지(1·2·3·4, mode4 의
          // ground_capable 포함)는 전부 직립 버튼 유지(m5_standing_hold) ∧ upright 를 요구한다 (Ruling 33).
          ok = mode_table::row(m).exit == Exit::ViaGround || (ctx.m5_standing_hold && upright);
          why = "먼저 직립 버튼 (직립 유지 상태에서만 나간다)";
          break;
        case Exit::ViaGround:
          ok = mode_table::row(m).exit == Exit::StandingHold; why = "ground 모드를 거쳐서만 나간다";
          break;
      }
      if (!ok) return {false, why};
      // 들어가는 쪽 조건 — 목적지 행의 exit 가 정한다. exit == Upright 인 모드(지금 클립 재생)는 «직립에서만
      // 나가는» 모드라 들어갈 때도 직립이어야 한다: 바닥에서 들어가면 그 모드는 스스로 일어서는 명령이 없고
      // 이탈도 거부돼 출구가 p 뿐이다(최종 검토 I-1, Ruling 34). 나가는 쪽과 같이 기본은 거부, case 가 허용을 켠다.
      bool enter_ok = false;
      switch (mode_table::row(m).exit) {
        case Exit::Upright:
          enter_ok = upright;
          break;
        case Exit::Always:
        case Exit::StandingHold:
        case Exit::ViaGround:
          enter_ok = true;
          break;
      }
      if (!enter_ok) return {false, "직립에서만 들어간다 (높이·기울기 조건)"};
      mode_ = m;
    }
    requested_ = m;
    return {true, ""};
  }

  // 안전 폴백 — 가드를 타지 않고, 조작자의 요청(requested)은 남긴다(수동 복귀 판정용).
  void force(int m) { if (mode_table::valid(m)) mode_ = m; }

  // 체류(FSM 진입, p→f→m 재진입 포함)를 지금 모드로 «시작» 해도 되나. 모드는 체류를 넘어 남는다.
  // 바닥 모드 중 스스로 일어서는 명령이 없는 것(GroundCapable ∧ !mode5_cmd_live = 클립 재생·기기)은 안 된다 —
  // 지난 체류가 넘어짐·Passive 로 끝났으면 로봇은 바닥에 있고, 그 모드는 이탈이 거부돼(직립에서만·ground 경유)
  // 출구가 p 뿐이다. mode5 는 된다(진입 자세 = 직립 버튼으로 시작, State_Mimic::enter). (Ruling 34, I-1)
  static bool may_start_stay_in(const mode_table::Row& r) {
    return !(r.safety == mode_table::Safety::GroundCapable && !r.mode5_cmd_live);
  }
  // 체류 시작(State_Mimic::enter, set_supported 뒤·행을 읽는 어떤 코드보다 앞). 지난 체류의 모드로 이 체류를
  // 시작하면 안 되면 fallback 으로 내린다(force — requested 는 enter 가 뒤에서 맞춘다).
  // 반환 = 내린 이유(로그용), 그대로 두면 nullptr.
  const char* begin_stay(int fallback) {
    const char* why = nullptr;
    if (!supports(mode_))                 why = "이 슬롯이 모른다";
    else if (!may_start_stay_in(row()))   why = "스스로 일어서는 명령이 없는 바닥 모드다(클립 재생류) — 체류는 폴백으로 시작한다";
    if (why) force(fallback);
    return why;
  }

  // 직전 consume 때의 모드와 지금이 다르면 true — 한 틱 안에서 바뀌었다가 되돌아온 것은 전환이 아니다.
  // (조작 채널이 모드를 요청한 같은 틱에 안전 폴백이 그것을 되돌리는 경우가 그렇다. 전이마다 래치를
  //  세우면 그 틱이 «전환» 으로 보여 crossfade·램프가 매 틱 재무장한다.)
  //
  // 🔴 계약: 이름은 질문처럼 생겼지만 «상태를 소비한다» — 한 번 true 를 내면 그 에지는 사라진다.
  //    ▸ 제어 틱마다 **호출자는 정확히 하나**다 = 정책 루프(State_Mimic.cpp policy_thread).
  //    ▸ 둘째 소비자를 두지 않는다. 두면 먼저 부른 쪽이 에지를 «훔쳐» 다른 쪽은 전환을 영영 못 본다
  //      (crossfade·다리 램프·되감기·재앵커가 조용히 빠진다 — 에러 없이 거동만 틀려진다).
  //    ▸ 전환을 알아야 하는 코드가 더 생기면 여기를 또 부르지 말고 **첫 호출자의 결과(bool)를 받아 쓴다.**
  //    ▸ 「지금 무슨 모드인가」만 알고 싶으면 mode()/row() 를 쓴다 — 이건 그 질문의 답이 아니다.
  bool consume_switch() { const bool s = mode_ != last_consumed_; last_consumed_ = mode_; return s; }

  int  clip_id() const { return clip_id_; }
  bool select_clip(int id, int n_clips) { if (id < 0 || id >= n_clips) return false; clip_id_ = id; return true; }

 private:
  int mode_ = 1, requested_ = 1, clip_id_ = 0;
  int last_consumed_ = 1;                         // consume_switch() 가 마지막으로 본 모드
  std::vector<int> supported_ = {1, 2, 3, 4};      // 계약 v1 슬롯의 기본
};

}  // namespace g1
