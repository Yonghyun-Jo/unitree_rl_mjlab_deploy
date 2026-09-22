// test_mode5_driver.cpp — Mode5Driver 가 학습 쪽 GoalDriver 와 «틱 단위로» 같은가.
//   골든 = tests/golden_mode5_driver.inc (gen_mode5_presets_header.py 가 파이썬 GoalDriver 를 돌려 만든다).
//   g++ -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include test_mode5_driver.cpp -o /tmp/t && /tmp/t
#include "Mode5Driver.h"
#include <cmath>
#include <cstdio>
#include "golden_mode5_driver.inc"

static int g_fail = 0;
static void chk(bool c, const char* what) { std::printf("  %s %s\n", c ? "ok  " : "FAIL", what); if (!c) ++g_fail; }

int main() {
  // 1) 골든: 53칸 명령 · hold 상태가 파이썬과 같다
  g1::Mode5Driver d(kGoldenM5Dt);
  double worst = 0.0; int bad = 0;
  for (int i = 0; i < kGoldenM5N; ++i) {
    const GoldenStep& s = kGoldenM5[i];
    if (s.reset) d.reset();
    if (s.press >= 0) d.press(s.press);
    const g1::Mode5Driver::Cmd c = d.tick(s.z, {s.g[0], s.g[1], s.g[2]});
    for (int k = 0; k < m5::CMD_DIM; ++k) {
      const double e = std::fabs(double(c[k]) - double(s.cmd[k]));
      if (e > worst) worst = e;
      if (e > 1e-6 && bad < 5) { std::printf("  step %d slot %d: cpp %.7f py %.7f\n", i, k, c[k], s.cmd[k]); ++bad; }
    }
    if (d.holding() != bool(s.hold) && bad < 5) { std::printf("  step %d hold: cpp %d py %d\n", i, d.holding(), s.hold); ++bad; }
  }
  std::printf("[m5 golden] %d steps, worst %.2e\n", kGoldenM5N, worst);
  chk(bad == 0, "golden: 명령·hold 가 파이썬 GoalDriver 와 같다");

  // 2) 성질
  g1::Mode5Driver e;
  chk(!e.active() && e.tick(0.7, {0.f, 0.f, -1.f})[m5::SLOT_T_GOAL] == 0.f, "press 전 tick = 0 명령");
  chk(!e.press(-1) && !e.press(m5::N_PRESETS), "범위 밖 press 거부");
  chk(e.press(m5::ENTER_PRESET) && !e.standing_hold(), "진입 자세 press 직후는 hold 아님");
  for (int t = 0; t < 40; ++t) e.tick(m5::PRESETS[m5::ENTER_PRESET].z, {0.f, 0.f, -1.f});
  chk(e.standing_hold(), "직립 높이에 0.5 s 넘게 머물면 standing_hold");
  const int other = (m5::ENTER_PRESET + 1) % m5::N_PRESETS;
  e.press(other);
  chk(!e.standing_hold(), "다른 자세를 누르면 standing_hold 해제");
  int n_keys = 0;
  for (const m5::Preset& p : m5::PRESETS) if (p.key != '\0') { ++n_keys; chk(m5::preset_by_key(p.key) >= 0, p.name); }
  chk(n_keys == 6 && m5::preset_by_key('1') < 0 && m5::preset_by_key('w') < 0, "자세 키 6개, 모드·조작 키와 안 겹침");
  return g_fail ? 1 : 0;
}
