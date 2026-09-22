#pragma once
// SafetyPolicy.h — 모드·자세를 아는 안전 규칙 (spec §4.7 앞 두 줄). 판단만 한다 — 실행은 State_Mimic.
//   UprightOnly(1·2·3): 종전과 «문자 그대로» — 넘어짐 판정 적용 · qd_warn → 폴백 모드(mode1).
//   GroundCapable(4·5·6): 바닥 자세가 목적인 모드다.
//     넘어짐(기울기 > 57.3°) 판정은 «지금 명령이 직립» ∧ «최근 1 s 안에 섰음» 일 때만 적용한다.
//       높이 하나로는 «일부러 눕기» 와 «넘어짐» 을 못 가른다 — 서 있던 G1 이 기본 자세 그대로 58° 기울면
//       z_fk 는 이미 0.45~0.53 m 다(tests/test_safety_policy.cpp 가 FK 로 잰다). 의도는 명령만 안다:
//         서 있다 넘어짐       = 명령 직립 ∧ 최근에 섰음  → 적용(잡는다)
//         일부러 내려가기     = 명령이 낮음              → 끔 (섰던 기억도 지운다)
//         누운 데서 일어나기  = 최근에 선 적 없음         → 끔 (다 서면 켜진다)
//       «섰음» = 명령이 직립인 동안 z_fk ≥ 0.65 ∧ 원시·걸러진 기울기 둘 다 < 57.3° — 그 자체로 넘어짐 판정에
//       안 걸리는 상태다. 높이만 보면 엉덩이부터 드는 기립(골반 60° 숙임·다리 곧음, z_fk ≈ 0.75)에서 기억이
//       차고 관문이 열려 거짓 Passive 가 난다(Ruling 13 — 네발 기기 자세의 기울기 64.5° 에서 직립 버튼).
//       걸러진 기울기만 보면 숙이며 내려가는 중(원시가 먼저 57.3° 를 넘고 걸러진 쪽은 늦다) 직립을 누른 틱에
//       기억이 찬다 → 둘 중 큰 쪽으로 본다(Ruling 14). UprightOnly 모드는 정의상 직립 명령이라 기억을 계속
//       채운다(upright_for_memory) — 넘어지는 도중 GroundCapable 모드로 바꿔도 판정이 이어진다.
//       «명령이 직립인가» 는 표의 성질로 가른다(commanded_upright — 클립 골반 높이 · mode5 자세의 z).
//       명령에 높이가 없는 GroundCapable 모드(기기)는 넘어짐 판정을 하지 않는다.
//     qd_warn → 직립(z_fk ≥ 0.65 ∧ 기울기 < 30°)이면 폴백 모드, 아니면 Passive — 이미 바닥에 있는 로봇을
//     «서서 걷는 정책» 으로 바꾸면 더 위험하다.
// ⚠ qd_warn «임계값» 자체(12 rad/s)는 그대로다. mode4·5 전용 값은 sim 실측 분포로 B3 가 정한다.
#include "ModeTable.h"
#include <optional>

namespace g1::safety {

// «섰다» 의 골반 높이 — 넘어짐 관문(명령·최근 이력)과 qd_warn 처분이 같이 쓴다.
// 이탈 조건(ModeRuntime.h EXIT_MIN_Z · EXIT_MAX_TILT_DEG)과 같은 값 — State_Mimic.cpp 가 static_assert 로 묶는다.
inline constexpr float UPRIGHT_MIN_Z = 0.65f;
inline constexpr float QD_FALLBACK_MAX_TILT = 30.f;
inline constexpr int RECENT_HIGH_TICKS = 50;     // 1 s @ 50 Hz (정책 틱)
// 넘어짐 판정 한계 = isaaclab::mdp::bad_orientation(env, limit_angle) 의 limit_angle [rad].
// State_Mimic.cpp 의 판정 호출이 «이 상수» 를 넘긴다 — 한계가 두 곳에 따로 적히지 않는다.
inline constexpr float ORIENT_TRIP_RAD = 1.0f;
inline constexpr float ORIENT_TRIP_DEG = ORIENT_TRIP_RAD * 57.29578f;   // TiltFilter 와 같은 환산 (57.2958°)

// «섰음» 기억을 채울 때의 «명령 직립». UprightOnly 모드는 정의상 직립 명령이다(명령에 높이가 없어도).
// 관문 식(orientation_check_applies)에 넘기는 commanded_upright 는 이것이 아니라 commanded_upright 그대로다.
inline bool upright_for_memory(mode_table::Safety s, bool commanded_upright) {
  switch (s) {
    case mode_table::Safety::UprightOnly:   return true;
    case mode_table::Safety::GroundCapable: return commanded_upright;
  }
  return true;                                   // 도달 불가 — 모르는 등급은 기억을 이어 채운다(판정이 걸리는 쪽)
}

// 최근 hold_ticks 틱 안에 «섰음» 이 있었나. 정책 스레드가 매 틱 update 한다.
//   섰음 = upright ∧ z_fk ≥ UPRIGHT_MIN_Z ∧ max(원시, 걸러진 기울기) < ORIENT_TRIP_DEG.
//     (두 비교로 쓴다 — std::max 는 인자 순서에 따라 NaN 을 삼킨다. NaN 은 어느 쪽이든 «섰음» 이 아니다.)
//   선 틱을 포함해 hold_ticks 틱 동안 true, 그다음 틱에 풀린다(선 틱 = 1번째 → 50번째 true, 51번째 false).
//   upright(= upright_for_memory) 가 false 인 틱엔 기억을 지운다 — 내려가라 했으면 섰던 기억으로 관문을 열지 않는다.
class RecentHigh {
 public:
  explicit RecentHigh(int hold_ticks = RECENT_HIGH_TICKS) : hold_(hold_ticks) {}
  bool update(float z_fk, float tilt_raw_deg, float tilt_filt_deg, bool upright) {
    if (!upright) n_ = 0;
    else if (z_fk >= UPRIGHT_MIN_Z && tilt_raw_deg < ORIENT_TRIP_DEG && tilt_filt_deg < ORIENT_TRIP_DEG) n_ = hold_;
    else if (n_ > 0) --n_;
    return n_ > 0;
  }
  bool value() const { return n_ > 0; }
  void reset() { n_ = 0; }

 private:
  int hold_;
  int n_ = 0;
};

// 지금 명령이 «직립» 인가 — 표의 성질로만 가른다(모드 번호를 보지 않는다).
//   ref_source == Clip → 재생 중인 클립의 현재 프레임 골반 높이 clip_pelvis_z ≥ UPRIGHT_MIN_Z
//   mode5_cmd_live     → 활성 자세의 목표 높이 m5_goal_z ≥ UPRIGHT_MIN_Z
//   그 밖              → 아님 (명령에 높이가 없다)
// 값이 없으면(nullopt = 활성 클립·자세 없음) «아님». 호출자(State_Mimic)가 이 틱의 값을 모아 넘긴다.
inline bool commanded_upright(const mode_table::Row& row, std::optional<float> clip_pelvis_z,
                              std::optional<double> m5_goal_z) {
  if (row.ref_source == mode_table::RefSource::Clip) return clip_pelvis_z && *clip_pelvis_z >= UPRIGHT_MIN_Z;
  if (row.mode5_cmd_live) return m5_goal_z && *m5_goal_z >= UPRIGHT_MIN_Z;
  return false;
}

inline bool orientation_check_applies(mode_table::Safety s, bool commanded_upright, bool recently_high) {
  switch (s) {
    case mode_table::Safety::UprightOnly:   return true;
    case mode_table::Safety::GroundCapable: return commanded_upright && recently_high;
  }
  return true;                                   // 도달 불가 — 모르는 등급은 안전측(판정 적용)
}

enum class QdWarnAction { Fallback, Passive };

inline QdWarnAction qd_warn_action(mode_table::Safety s, float z_fk, float tilt_deg) {
  switch (s) {
    case mode_table::Safety::UprightOnly:
      return QdWarnAction::Fallback;
    case mode_table::Safety::GroundCapable:
      return (z_fk >= UPRIGHT_MIN_Z && tilt_deg < QD_FALLBACK_MAX_TILT) ? QdWarnAction::Fallback
                                                                        : QdWarnAction::Passive;
  }
  return QdWarnAction::Passive;                  // 도달 불가 — 모르는 등급은 안전측
}

}  // namespace g1::safety
