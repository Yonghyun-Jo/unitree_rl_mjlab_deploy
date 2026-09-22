// test_motion_preview.cpp — 미리보기 736칸이 학습 함수(calc_future_motion_obs)와 같은가.
//   g++ -std=gnu++17 -O2 -Wall -Wextra -I../include -I/usr/include/eigen3 test_motion_preview.cpp -o /tmp/t && /tmp/t
#include "MotionPreview.h"
#include <cmath>
#include <cstdio>
#include "golden_motion_preview.inc"

struct FakeClip {                    // MotionLoader_ 와 같은 접근자
  int n() const { return kClipN; }
  Eigen::Quaternionf quat(int i) const { return Eigen::Quaternionf(kClipQ[i][0], kClipQ[i][1], kClipQ[i][2], kClipQ[i][3]); }
  Eigen::Vector3f lin(int i) const { return Eigen::Vector3f(kClipLin[i][0], kClipLin[i][1], kClipLin[i][2]); }
  Eigen::Vector3f ang(int i) const { return Eigen::Vector3f(kClipAng[i][0], kClipAng[i][1], kClipAng[i][2]); }
  float z(int i) const { return kClipZ[i]; }
  const float* dof(int i) const { return kClipJp[i]; }
};

static int g_fail = 0;
static void chk(bool c, const char* what) { std::printf("  %s %s\n", c ? "ok  " : "FAIL", what); if (!c) ++g_fail; }

int main() {
  FakeClip clip;
  std::vector<float> out;
  double worst = 0.0; int bad = 0;
  for (int i = 0; i < kGoldenPreviewN; ++i) {
    const PreviewCase& c = kGoldenPreview[i];
    g1::preview::block(clip, c.cur, true, out);
    for (int k = 0; k < g1::preview::DIM; ++k) {
      const double e = std::fabs(double(out[k]) - double(c.block[k]));
      if (e > worst) worst = e;
      if (e > 1e-5 && bad < 5) { std::printf("  cur %d idx %d: cpp %.6f py %.6f\n", c.cur, k, out[k], c.block[k]); ++bad; }
    }
  }
  std::printf("[preview golden] %d cases, worst %.2e\n", kGoldenPreviewN, worst);
  chk(bad == 0, "미리보기: 학습 함수와 1e-5 안");
  g1::preview::block(clip, 5, false, out);
  bool zeros = (int)out.size() == g1::preview::DIM;
  for (float v : out) zeros = zeros && v == 0.f;
  chk(zeros, "on=false 면 736칸 «정확히 0» (유효비트 포함)");
  return g_fail ? 1 : 0;
}
