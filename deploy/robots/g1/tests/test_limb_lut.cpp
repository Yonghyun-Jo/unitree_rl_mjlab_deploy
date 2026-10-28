// test_limb_lut.cpp — LimbLut.h == 학습 limb_lut.limb_z (golden = 학습 함수의 출력, gen_limb_lut_header.py).
//   g++ -std=gnu++17 -O2 -I../include test_limb_lut.cpp -o /tmp/t && /tmp/t
// 보는 것: 표 보간(원형·구간 중심) · 정지 게이트(eff < 0.05 → 사지별 접지) · 위상 시계(속도 무관, 50 Hz) · 유효 속력.
#include "LimbLut.h"
#include "golden_limb_lut.inc"
#include <cmath>
#include <cstdio>

static int fails = 0;
static void chk(bool ok, const char* what) {
  if (!ok) { std::printf("FAIL %s\n", what); ++fails; }
}

int main() {
  using namespace g1::limb_lut;
  // ① 표 보간·정지 게이트 — 학습 limb_z_scalar 와 float 오차 안
  double worst = 0.0;
  for (const auto& g : kGoldenLimb) {
    const auto z = limb_z(static_cast<float>(g.phase), static_cast<float>(g.eff));
    for (int i = 0; i < N_LIMBS; ++i) worst = std::fmax(worst, std::fabs(double(z[i]) - double(g.z[i])));
  }
  std::printf("limb_z vs 학습: %zu 점, 최대 |Δ| = %.2e m\n", sizeof(kGoldenLimb) / sizeof(kGoldenLimb[0]), worst);
  chk(worst < 1e-6, "limb_z 가 학습 limb_lut.limb_z 와 다르다");

  // ② 위상 시계 — 0 에서 출발, 틱마다 위상 먼저 그다음 높이 (advance_limb 순서), eff 0.3 (= vx 0.3)
  chk(std::fabs(double(phase_rate()) - kGoldenLimbRate) < 1e-9, "phase_rate(1/50) 가 학습과 다르다");
  Clock c;
  double wp = 0.0, wz = 0.0;
  for (const auto& g : kGoldenLimbClock) {
    const auto z = c.tick(0.3f, 0.0f, 0.0f);
    const double dp = std::fabs(double(c.phase) - g.phase);
    wp = std::fmax(wp, std::fmin(dp, 1.0 - dp));                 // 원형 거리 (0.99999 ↔ 0.0 은 같은 위상)
    for (int i = 0; i < N_LIMBS; ++i) wz = std::fmax(wz, std::fabs(double(z[i]) - double(g.z[i])));
  }
  std::printf("위상 시계 200 틱: 위상 최대 |Δ| %.2e · 높이 %.2e m\n", wp, wz);
  chk(wp < 1e-5 && wz < 1e-5, "위상 시계(Clock::tick)가 학습 advance_limb 와 다르다");

  // ③ 정지 = 사지별 접지 높이 (0 이 아니다) · 유효 속력 = hypot + 0.3|wz| (제자리 회전도 «움직임»)
  const auto s = limb_z(0.37f, 0.0f);
  for (int i = 0; i < N_LIMBS; ++i) chk(s[i] == LIMB_STANCE[i] && s[i] > 0.07f, "정지인데 접지 높이가 아니다");
  chk(std::fabs(effective_speed(0.3f, 0.4f, 0.0f) - 0.5f) < 1e-6f, "effective_speed hypot");
  chk(effective_speed(0.0f, 0.0f, 0.2f) >= STAND_EPS, "제자리 회전 0.2 rad/s 가 정지로 읽힌다");
  chk(limb_z(0.37f, effective_speed(0.0f, 0.0f, 0.2f))[0] != LIMB_STANCE[0] ||
      limb_z(0.37f, effective_speed(0.0f, 0.0f, 0.2f))[2] != LIMB_STANCE[2], "회전 명령에서 사지가 얼었다");
  // ④ 위상 주기성: 0 과 1 은 같은 값
  const auto a = limb_z(0.0f, 1.0f), b = limb_z(1.0f, 1.0f);
  for (int i = 0; i < N_LIMBS; ++i) chk(std::fabs(a[i] - b[i]) < 1e-7f, "위상 0 과 1 이 다르다(원형 보간)");

  if (fails) { std::printf("%d 개 실패\n", fails); return 1; }
  std::printf("ok\n");
  return 0;
}
