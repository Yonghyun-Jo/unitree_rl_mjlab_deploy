// test_height_estimator.cpp — z_fk 가 MuJoCo FK 와 1 mm 안인가 · yaw 에 무관한가 · 기울기 필터.
//   g++ -std=gnu++17 -O2 -Wall -Wextra -I../include -I/usr/include/eigen3 test_height_estimator.cpp -o /tmp/t && /tmp/t
#include "HeightEstimator.h"
#include <cmath>
#include <cstdio>
#include "golden_g1_zfk.inc"

static int g_fail = 0;
static void chk(bool c, const char* what) { std::printf("  %s %s\n", c ? "ok  " : "FAIL", what); if (!c) ++g_fail; }

int main() {
  double worst = 0.0; int bad = 0;
  for (int i = 0; i < kGoldenZfkN; ++i) {
    const ZfkCase& c = kGoldenZfk[i];
    const Eigen::Quaternionf q(c.quat[0], c.quat[1], c.quat[2], c.quat[3]);
    const float z = g1::z_fk(c.q, q);
    const double e = std::fabs(double(z) - double(c.z));
    if (e > worst) worst = e;
    if (e > 1e-3 && bad < 5) { std::printf("  case %d: cpp %.5f mujoco %.5f\n", i, z, c.z); ++bad; }
  }
  std::printf("[z_fk golden] %d cases, worst %.2e m\n", kGoldenZfkN, worst);
  chk(bad == 0, "z_fk: MuJoCo FK 와 1 mm 안");

  // yaw 불변: 같은 자세에 yaw 를 더해도 값이 같다
  const ZfkCase& c0 = kGoldenZfk[0];
  const Eigen::Quaternionf q0(c0.quat[0], c0.quat[1], c0.quat[2], c0.quat[3]);
  const Eigen::Quaternionf qy(Eigen::AngleAxisf(1.3f, Eigen::Vector3f::UnitZ()));
  chk(std::fabs(g1::z_fk(c0.q, qy * q0) - g1::z_fk(c0.q, q0)) < 1e-5f, "yaw 를 더해도 z_fk 불변");

  // 기울기 필터: 직립 = 0°, 90° 계단 입력 → 한 τ(0.2 s = 10틱) 뒤 약 63 %
  g1::TiltFilter f(0.02f, 0.2f);
  chk(std::fabs(f.update({0.f, 0.f, -1.f})) < 1e-4f, "직립 = 0°");
  float v = 0.f;
  for (int t = 0; t < 11; ++t) v = f.update({1.f, 0.f, 0.f});
  chk(v > 50.f && v < 65.f, "90° 계단, 11틱 뒤 55~65° (τ=0.2 s)");
  return g_fail ? 1 : 0;
}
