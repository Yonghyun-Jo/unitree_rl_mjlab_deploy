// Golden parity test for MaskedLocoController (C++) vs the mjlab_g1_motion Python
// controller. Re-runs the exact 60-step scripted sequence from golden_loco_controller.json
// and checks a few embedded checkpoints (full bit-identical comparison was done at port time:
// max abs err 0.00e+00). NOT part of the build (lives outside src/). Run manually:
//
//   g++ -std=c++17 -I../include -O2 test_masked_loco_controller.cpp -o /tmp/test_loco && /tmp/test_loco
//
// To regenerate golden_loco_controller.json see mjlab_g1_motion tasks/mdp/golden_loco_controller.json.
#include "MaskedLocoController.h"
#include <cstdio>
#include <cmath>
#include <vector>

static bool close(float a, float b) { return std::fabs(a - b) < 1e-4f; }

int main() {
  MaskedLocoController c;  // defaults must match golden "params"
  struct S { std::array<float, 3> jb; int mode; int sw; };  // sw=-1 none
  std::vector<S> seq;
  for (int i = 0; i < 10; ++i) seq.push_back({{0.5f, 0, 0}, 1, -1});
  seq.push_back({{0.5f, 0, 0}, 2, 2});
  for (int i = 0; i < 9; ++i) seq.push_back({{0.5f, 0, 0}, 2, -1});
  seq.push_back({{0, 0, 1.0f}, 1, 1});
  for (int i = 0; i < 29; ++i) seq.push_back({{0, 0, 1.0f}, 1, -1});
  seq.push_back({{0.3f, 0, 0}, 3, 3});
  for (int i = 0; i < 9; ++i) seq.push_back({{0.3f, 0, 0}, 3, -1});

  std::vector<MaskedLocoController> snap;  // store output snapshots
  std::vector<std::array<float, 6>> out;
  for (auto& s : seq) {
    if (s.sw >= 0) c.notify_mode_switch(s.sw);
    c.update(s.jb, s.mode);
    out.push_back({c.base_vel[0], c.base_vel[1], c.base_vel[2], c.foot_z[0], c.foot_z[1], c.arm_scale});
  }

  int fail = 0;
  auto chk = [&](int i, float bvx, float bvz, float arm) {
    if (!close(out[i][0], bvx) || !close(out[i][2], bvz) || !close(out[i][5], arm)) {
      printf("FAIL step%d: bv=[%.4f,_,%.4f] arm=%.4f (want bvx=%.4f bvz=%.4f arm=%.4f)\n",
             i, out[i][0], out[i][2], out[i][5], bvx, bvz, arm);
      ++fail;
    }
  };
  // checkpoints from golden_loco_controller.json
  chk(0,  0.5f,      0.0f,      1.0f);       // mode1, full speed, no blend
  chk(20, 0.493333f, 0.013333f, 1.0f);       // mid base_vel spline (mode2->1 + turn), arm ease-in start
  chk(40, 0.36f,     0.28f,     0.733333f);  // mid spline, mid arm ease-in
  chk(50, 0.0f,      0.0f,      0.6f);        // switch to mode3: base_vel masked to 0 (arm-blend mid-ease)

  // reset_command(): 체류 경계(State_Mimic::enter)에서 속도 명령·진행 중 램프를 버린다 → 다음 update(0) 이 곧 0.
  // 대조군(리셋 없음)은 램프가 옛 속도에서 풀려 0 이 아니어야 한다 — 그래야 이 검사가 뭔가를 잡는다.
  {
    auto run = [](bool reset) {
      MaskedLocoController r;
      for (int i = 0; i < 10; ++i) r.update({2.0f, 0, 0}, 1);
      r.notify_mode_switch(1);                 // 램프 시작 (bv_ramp_from = 2.0)
      r.update({2.0f, 0, 0}, 1);
      if (reset) r.reset_command();
      r.update({0, 0, 0}, 1);
      return r;
    };
    const MaskedLocoController a = run(true), b = run(false);
    if (a.base_vel[0] != 0.0f || a.base_vel[1] != 0.0f || a.base_vel[2] != 0.0f || a.bv_ramp_rem != 0) {
      printf("FAIL reset_command: base_vel=[%.4f %.4f %.4f] ramp_rem=%d (want 0)\n",
             a.base_vel[0], a.base_vel[1], a.base_vel[2], a.bv_ramp_rem);
      ++fail;
    }
    if (!(b.base_vel[0] > 0.5f)) {
      printf("FAIL reset_command control: 리셋 없이도 0 (%.4f) — 검사가 아무것도 안 잡는다\n", b.base_vel[0]);
      ++fail;
    }
  }

  if (fail == 0) printf("OK: MaskedLocoController C++ matches golden checkpoints\n");
  return fail;
}
