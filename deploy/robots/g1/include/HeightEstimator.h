#pragma once
// HeightEstimator.h — 엔코더 29 + IMU 자세만으로 «골반 높이» 와 «기울기» 를 낸다 (spec §4.4).
//   z_fk = −min_k( 충돌 프리미티브 k 의 최저점 z ),  좌표 = 골반 원점 · 세계 수직축 · yaw 제거.
//   가정: 평지 + «무언가는 바닥에 닿아 있다». 전역 위치·yaw 는 쓰지 않는다(실 G1 에 없다).
//   실측(학습 sim, 09-21 게이트 G2): 5자세 버튼 32/32 · IMU ±3° 오차에서 p95 3.2 cm.
// 🔴 Eigen 식을 auto 로 받지 않는다 — 고정 크기 타입으로 받는다(-O3 댕글링).
#include "G1Kinematics.h"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace g1 {

// 쿼터니언(wxyz)에서 yaw 만 떼어낸다 — 남는 것은 roll·pitch(수직축 기준 기울기).
inline Eigen::Quaternionf remove_yaw(const Eigen::Quaternionf& q) {
  const float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
                               1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
  const Eigen::Quaternionf qy(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  return (qy.conjugate() * q).normalized();
}

// q29 = 배포 관절 순서(= robot_spec.JOINT_ORDER), pelvis_quat_w = IMU 골반 자세(wxyz).
inline float z_fk(const float* q29, const Eigen::Quaternionf& pelvis_quat_w) {
  std::array<Eigen::Matrix3f, g1kin::N_BODY> R;
  std::array<Eigen::Vector3f, g1kin::N_BODY> p;
  const Eigen::Quaternionf qt = remove_yaw(pelvis_quat_w.normalized());
  for (int b = 0; b < g1kin::N_BODY; ++b) {
    const g1kin::Body& B = g1kin::BODIES[b];
    if (B.parent < 0) {
      R[b] = qt.toRotationMatrix();
      p[b] = Eigen::Vector3f::Zero();
      continue;
    }
    const Eigen::Quaternionf bq(B.quat[0], B.quat[1], B.quat[2], B.quat[3]);
    const Eigen::Vector3f bp(B.pos[0], B.pos[1], B.pos[2]);
    p[b] = p[B.parent] + R[B.parent] * bp;
    Eigen::Matrix3f Rb = R[B.parent] * bq.toRotationMatrix();
    if (B.joint >= 0) {
      const Eigen::Vector3f ax(B.axis[0], B.axis[1], B.axis[2]);
      const Eigen::Matrix3f Rj = Eigen::AngleAxisf(q29[B.joint], ax).toRotationMatrix();
      Rb = Rb * Rj;
    }
    R[b] = Rb;
  }
  float low = std::numeric_limits<float>::infinity();
  for (const g1kin::Geom& G : g1kin::GEOMS) {
    const Eigen::Quaternionf gq(G.quat[0], G.quat[1], G.quat[2], G.quat[3]);
    const Eigen::Vector3f gp(G.pos[0], G.pos[1], G.pos[2]);
    const Eigen::Vector3f c = p[G.body] + R[G.body] * gp;
    const Eigen::Matrix3f Rg = R[G.body] * gq.toRotationMatrix();
    low = std::min(low, c.z() - G.half_len * std::fabs(Rg(2, 2)) - G.radius);
  }
  return -low;
}

// 기울기 [deg] = 골반 z축과 세계 수직의 각 = acos(−g_z), g = projected_gravity_b (직립이면 (0,0,−1)).
// 1차 저역통과(τ): 이탈 조건이 한 틱의 스파이크로 거부되지 않게(B1 Ruling 7 — «걸러서» 넣는다).
class TiltFilter {
 public:
  explicit TiltFilter(float dt = 0.02f, float tau = 0.2f) : a_(dt / (tau + dt)) {}
  float update(const std::array<float, 3>& g_b) {
    raw_ = std::acos(std::clamp(-g_b[2], -1.f, 1.f)) * 57.29578f;
    y_ = init_ ? y_ + a_ * (raw_ - y_) : raw_;
    init_ = true;
    return y_;
  }
  float value() const { return y_; }
  // 직전 update() 의 걸러지기 전 기울기 [deg]. 기울기가 오르는 중엔 value() 가 늦게 따라오므로(낮게 나온다)
  // 안전 규칙(SafetyPolicy.h RecentHigh)은 둘 다 본다.
  float raw() const { return raw_; }
  void reset() { init_ = false; y_ = 0.f; raw_ = 0.f; }

 private:
  float a_;
  float y_ = 0.f;
  float raw_ = 0.f;
  bool init_ = false;
};

}  // namespace g1
