#pragma once
// ArcadeGate.h — 아케이드 경로(G1_ARCADE=1)에서 g1_ctrl 안전층이 «core 가 알려 준 지금 skill 의 안전 등급» 으로 하드 보호를
//   판단하는 순수 함수(판단만 — 실행은 State_Mimic). g1 전용(🅐 구역). 헤더 온리 · core 헤더를 include 하지 않는다
//   (아케이드 없는 빌드에서도 tests/test_arcade_gate.cpp 가 돈다). 시험 = tests/test_arcade_gate.cpp.
//
// 왜 필요한가(2b-facts B14): 아케이드 경로는 core 로만 skill 을 바꾸고 g1_ctrl 의 g_mode 를 건드리지 않는다. g_mode 행으로
//   관문을 열면 g_mode 는 늘 mode1(UprightOnly)이라 crawl(골반 65°)·바닥 자세·점프 착지에서 즉시 bad_orientation → Passive 다.
//   qd_warn 처분도 g_mode 표로 하면 «폴백 모드 강제(g_mode.force)» 가 core 의 skill 선택과 무관한 거짓 상태만 만든다.
//
// 규칙(설계 2B §9 사용자 답 11-06 · global-constraints):
//   · 넘어짐 관문 = SafetyPolicy.h 의 기존 함수 그대로, 입력만 core 값(safety_class · commanded_upright · core 모드)으로.
//       core 모드가 표 안이고 그 행의 등급이 core 등급과 같으면 그 «행» 으로(클립 행 = 브리지 추종기 모드 4 는 «최근 섰음» 을 묻지
//       않는다 — g1_ctrl 의 mode4 규칙 그대로), 아니면 «등급» 만으로(점프 mode7 = 표 밖, 등급 UprightOnly = 늘 판정).
//   · «섰음» 기억의 명령 직립 = upright_for_memory(core 등급, core 명령 직립).
//   · qd_warn 래치: 풀기 = core 가 슬롯 폴백 모드(걷기, mode1)로 돌아왔을 때(조작자가 '1' 을 누른 것과 같은 자리 —
//       ModeRuntime.h qd_warn_may_release 에 «요청 모드 = core 모드» 를 넘긴다). 처분 = 낮은 자세 Passive 만(SafetyPolicy.h
//       qd_warn_action 을 core 등급으로). 🔴 «폴백 모드 강제» 는 없다 — 이 함수의 결과에 그런 값이 없다(UprightOnly 의 Fallback 은
//       Hold = 래치만, core 가 걷기로 돌아오면 풀린다). core 가 처분을 보류하는 skill(점프 = 표 밖 모드 · 브리지 role)은 Hold
//       (core Runtime::step_safety 와 같은 자리 — 설계 «점프·브리지 도중 qd_warn 은 처분 보류»).
//   · qd_crit → Passive · pos_clamp · NaN 홀드 · E-stop · lowstate 타임아웃은 여기와 무관하게 늘 켬(State_Mimic · CtrlFSM).
#include "ModeRuntime.h"
#include "ModeTable.h"
#include "SafetyPolicy.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace g1::arcade_gate {

// 환경 G1_ARCADE: 없음 · "" · "0" = 끔, "1" = 켬, 그 밖 = 모르는 값(기동 거부 — 로봇에서 «켰다고 믿었는데 꺼짐» 을 막는다).
enum class Request { Off, On, Bad };
inline Request arcade_request(const char* v) {
  if (!v || !*v || std::strcmp(v, "0") == 0) return Request::Off;
  if (std::strcmp(v, "1") == 0) return Request::On;
  return Request::Bad;
}

// core TickInfo.safety_class(skill.h: 0 UprightOnly · 1 GroundCapable) → 표의 등급. 모르는 값 = UprightOnly(판정 적용 = 안전측).
inline mode_table::Safety safety_of_class(int c) {
  return c == 1 ? mode_table::Safety::GroundCapable : mode_table::Safety::UprightOnly;
}

// 정책 틱이 core 에서 받은 «지금 skill 의 안전» (ArcadeLink::core_safety — 마지막 end_tick 뒤 TickInfo).
//   mode = 활성 skill 의 모드(표 밖 가능 — 점프 7) · holds_qd = core 가 qd_warn 처분을 보류하는 skill(점프 · 브리지)
struct CoreSafety {
  int mode = -1;
  int safety_class = 0;
  bool commanded_upright = false;
  bool holds_qd = false;
};

// 관문이 쓸 표의 행: core 모드가 표 안이고 그 행의 등급 = core 등급이면 그 행, 아니면 nullptr(등급만으로 판정).
inline const mode_table::Row* row_of(const CoreSafety& c) {
  if (!mode_table::valid(c.mode)) return nullptr;
  const mode_table::Row& r = mode_table::row(c.mode);
  return r.safety == safety_of_class(c.safety_class) ? &r : nullptr;
}

// g_recent_high.update 의 upright 인자.
inline bool memory_upright(const CoreSafety& c) {
  return g1::safety::upright_for_memory(safety_of_class(c.safety_class), c.commanded_upright);
}

// orient_gate_ 에 넣을 값(넘어짐 판정을 이 틱에 적용하나).
inline bool orient_gate(const CoreSafety& c, bool recently_high) {
  if (const mode_table::Row* r = row_of(c)) return g1::safety::orientation_check_applies(*r, c.commanded_upright, recently_high);
  return g1::safety::orientation_check_applies(safety_of_class(c.safety_class), c.commanded_upright, recently_high);
}

// qd_warn 래치가 걸려 있는 틱의 처분. Release = 래치를 푼다 · Hold = 래치만(처분 없음) · Passive = 낮은 자세 Passive.
enum class QdWarn { Release, Hold, Passive };
inline QdWarn qd_warn_verdict(const CoreSafety& c, const mode_table::Row& fallback, float z_fk, float tilt_deg) {
  // 자세 버튼 폴백(한 skill 슬롯)은 «직립 유지 도착» 이 core 상태 칸에 없어 풀지 않는다 — 아케이드 경로는 v7 슬롯(폴백 mode1)만 연다
  if (g1::qd_warn_may_release(fallback, c.mode, false)) return QdWarn::Release;
  if (c.holds_qd) return QdWarn::Hold;
  return g1::safety::qd_warn_action(safety_of_class(c.safety_class), z_fk, tilt_deg) == g1::safety::QdWarnAction::Passive
             ? QdWarn::Passive
             : QdWarn::Hold;
}

// 슬롯 deploy.yaml 의 joint_ids_map 이 0..28 항등인가(F-B12 — 아케이드 경로는 정책 순서 = SDK 순서 = core Vec29 순서로 쓴다).
template <class V>
inline bool ids_identity(const V& m) {
  if (m.size() != 29) return false;
  for (int i = 0; i < 29; ++i)
    if (static_cast<double>(m[i]) != static_cast<double>(i)) return false;
  return true;
}

// 측정 자세를 거부할 때(비유한 칸이 있다) 1 kHz «직전 q» 를 채우는 값: 유한한 칸 = 측정값, 비유한 칸 = 슬롯 default_joint_pos.
// kp 0 상태로 한 틱 나가더라도 0 rad 가 아니다(F1).
inline void refusal_fallback_q(const float* q_meas, const float* q_default, float* out) {
  for (int i = 0; i < 29; ++i) out[i] = std::isfinite(q_meas[i]) ? q_meas[i] : q_default[i];
}

// mlockall(MCL_CURRENT) 을 시도할 만한가(R-B7): memlock 한도(soft)가 무한이거나 지금 RSS 의 2 배 이상일 때만. MCL_FUTURE 는 쓰지
// 않는다(로봇 한도를 넘는 뒤 할당이 실패해 제어기가 죽을 수 있다 — 안전 > 폴트 0). 실패해도 기동은 계속한다(호출부).
inline bool mlock_worth_trying(bool soft_unlimited, std::uint64_t soft_bytes, std::uint64_t rss_bytes) {
  return soft_unlimited || (rss_bytes > 0 && soft_bytes / 2 >= rss_bytes);
}

}  // namespace g1::arcade_gate
