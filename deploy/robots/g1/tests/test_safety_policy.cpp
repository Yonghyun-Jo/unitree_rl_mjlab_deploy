// test_safety_policy.cpp — 넘어짐·qd_warn 규칙 표 (spec §4.7). UprightOnly 모드는 «종전과 문자 그대로».
//   g++ -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include test_safety_policy.cpp -o /tmp/t && /tmp/t
#include "SafetyPolicy.h"
#include <cstdio>

static int g_fail = 0;
static void chk(bool c, const char* what) { std::printf("  %s %s\n", c ? "ok  " : "FAIL", what); if (!c) ++g_fail; }

int main() {
  using mode_table::Safety;
  using g1::safety::QdWarnAction;
  namespace s = g1::safety;
  for (float z : {0.05f, 0.40f, 0.60f, 0.80f}) {
    chk(s::orientation_check_applies(Safety::UprightOnly, z), "UprightOnly: 넘어짐 판정은 높이와 무관하게 적용");
    chk(s::qd_warn_action(Safety::UprightOnly, z, 80.f) == QdWarnAction::Fallback, "UprightOnly: qd_warn → 폴백");
  }
  chk(s::orientation_check_applies(Safety::GroundCapable, 0.80f), "GroundCapable 높음: 넘어짐 판정 적용");
  chk(s::orientation_check_applies(Safety::GroundCapable, 0.55f), "경계 0.55 는 적용");
  chk(!s::orientation_check_applies(Safety::GroundCapable, 0.30f), "GroundCapable 낮음: 적용 안 함");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.75f, 10.f) == QdWarnAction::Fallback, "직립: 폴백");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.75f, 35.f) == QdWarnAction::Passive, "높지만 기울면: Passive");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.60f, 5.f) == QdWarnAction::Passive, "0.65 아래: Passive");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.20f, 80.f) == QdWarnAction::Passive, "바닥: Passive");
  return g_fail ? 1 : 0;
}
