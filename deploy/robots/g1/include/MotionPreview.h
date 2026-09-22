#pragma once
// MotionPreview.h — mode4 «클립의 미래» (student `motion` 그룹, ONNX 입력의 마지막 736칸).
//   학습 원장 = g1_mimic_env.calc_future_motion_obs · calc_motion_preview(_on/_k1).
//   배치 = [on 1][k1 35][preview 20×35]. k1 = 첫 미리보기 스텝(t+1)의 사본(압축 병목 우회로).
//   한 스텝 35 = 골반 속도 xy(yaw 기준) 2 · 골반 높이 1 · roll 1 · pitch 1 · yaw 각속도 1 · 관절 29.
//   전부 클립(명령)에서 계산한다 — 로봇 상태는 들어가지 않는다.
// 🔴 프레임 = (cur + k) mod n. 학습은 끝에서 clamp 하지만 배포는 클립을 돌려 재생한다
//    (State_Mimic MotionLoader_::update). 로봇이 실제로 받을 미래를 준다.
// 🔴 on=false 면 736칸 전부 «정확히 0» (유효비트 포함) — 학습의 «0 = 해당 없음» 규약.
#include "ModeTable.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace g1::preview {

inline constexpr int STEP = mode_table::MOTION_STEP_DIM;
inline constexpr int DIM = mode_table::MOTION_BLOCK_DIM;

// 한 프레임 → 35칸. q = 골반 world 자세(wxyz), lin/ang = 골반 world 선·각속도, z = 골반 world 높이.
inline void step(const Eigen::Quaternionf& q, const Eigen::Vector3f& lin, const Eigen::Vector3f& ang, float z,
                 const float* dof, float* out) {
  const float w = q.w(), x = q.x(), y = q.y(), qz = q.z();
  const float yaw = std::atan2(2.f * (w * qz + x * y), 1.f - 2.f * (y * y + qz * qz));
  const float c = std::cos(yaw), s = std::sin(yaw);
  out[0] = c * lin.x() + s * lin.y();                           // yaw 회전의 역 · v (학습 quat_apply_inverse(yaw_quat))
  out[1] = -s * lin.x() + c * lin.y();
  out[2] = z;
  out[3] = std::atan2(2.f * (w * x + y * qz), 1.f - 2.f * (x * x + y * y));          // roll (−π, π]
  const float sp = 2.f * (w * y - qz * x);
  out[4] = std::fabs(sp) >= 1.f ? std::copysign(1.5707963267948966f, sp) : std::asin(sp);   // pitch
  out[5] = ang.z();                                              // yaw_rate = world 각속도 z
  for (int j = 0; j < mode_table::N_DOF; ++j) out[6 + j] = dof[j];
}

template <class Clip>
inline void block(const Clip& c, int cur, bool on, std::vector<float>& out) {
  out.assign(DIM, 0.f);
  const int n = c.n();
  if (!on || n <= 0) return;
  out[0] = 1.f;
  for (int s = 0; s < mode_table::N_PREVIEW; ++s) {
    const int i = ((cur + mode_table::PREVIEW_OFFSETS[s]) % n + n) % n;
    step(c.quat(i), c.lin(i), c.ang(i), c.z(i), c.dof(i), &out[1 + STEP + s * STEP]);
  }
  std::copy(out.begin() + 1 + STEP, out.begin() + 1 + 2 * STEP, out.begin() + 1);   // k1
}

}  // namespace g1::preview
