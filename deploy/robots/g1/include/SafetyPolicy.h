#pragma once
// SafetyPolicy.h — 모드·높이를 아는 안전 규칙 (spec §4.7 앞 두 줄). 판단만 한다 — 실행은 State_Mimic.
//   UprightOnly(1·2·3): 종전과 «문자 그대로» — 넘어짐 판정 적용 · qd_warn → 폴백 모드(mode1).
//   GroundCapable(4·5·6): 바닥 자세가 목적인 모드다.
//     넘어짐(기울기 > 57.3°) 판정은 z_fk ≥ ORIENT_MIN_Z 일 때만 — 높은 데서 넘어지는 것만 넘어짐이다.
//     qd_warn → 직립(z_fk ≥ 0.65 ∧ 기울기 < 30°)이면 폴백 모드, 아니면 Passive — 이미 바닥에 있는 로봇을
//     «서서 걷는 정책» 으로 바꾸면 더 위험하다.
// ⚠ qd_warn «임계값» 자체(12 rad/s)는 그대로다. mode4·5 전용 값은 sim 실측 분포로 B3 가 정한다.
#include "ModeTable.h"

namespace g1::safety {

inline constexpr float ORIENT_MIN_Z = 0.55f;
inline constexpr float QD_FALLBACK_MIN_Z = 0.65f;
inline constexpr float QD_FALLBACK_MAX_TILT = 30.f;

inline bool orientation_check_applies(mode_table::Safety s, float z_fk) {
  switch (s) {
    case mode_table::Safety::UprightOnly:   return true;
    case mode_table::Safety::GroundCapable: return z_fk >= ORIENT_MIN_Z;
  }
  return true;                                   // 도달 불가 — 모르는 등급은 안전측(판정 적용)
}

enum class QdWarnAction { Fallback, Passive };

inline QdWarnAction qd_warn_action(mode_table::Safety s, float z_fk, float tilt_deg) {
  switch (s) {
    case mode_table::Safety::UprightOnly:
      return QdWarnAction::Fallback;
    case mode_table::Safety::GroundCapable:
      return (z_fk >= QD_FALLBACK_MIN_Z && tilt_deg < QD_FALLBACK_MAX_TILT) ? QdWarnAction::Fallback
                                                                            : QdWarnAction::Passive;
  }
  return QdWarnAction::Passive;                  // 도달 불가 — 모르는 등급은 안전측
}

}  // namespace g1::safety
