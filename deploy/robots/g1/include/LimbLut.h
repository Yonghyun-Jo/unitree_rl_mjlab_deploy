#pragma once
// LimbLut.h — mode6(네발 기기) 사지 높이 명령. 학습 mjlab tasks/mdp/limb_lut.py 의 1:1 이식 (순수 C++, 단위 테스트 가능).
//
// 학습이 하는 일 (LocoController.advance_limb, 정책 틱마다 «관측 직전»):
//   phase ← (phase + dt / PERIOD_S) mod 1                 위상 시계는 속도와 무관하게 늘 돈다
//   eff   = hypot(vx, vy) + TURN_K · |wz|                  정책이 보는 base_vel 명령으로 (foot_gen.effective_speed)
//   limb_z = eff < STAND_EPS ? LIMB_STANCE                 정지 = 사지별 접지 높이(0 이 아니다 — 링크 원점 z)
//                            : 표(LIMB_P)의 원형 선형보간    높이도 주기도 속도와 무관(위상 1차원)
// 순서 = [L_hand, R_hand, L_knee, R_knee] (robot_spec.CRAWL_LIMB_BODIES). 표 = 아래 GENERATED 구역(손으로 고치지 않는다).
#include <array>
#include <cmath>

namespace g1::limb_lut {

inline constexpr int N_LIMBS = 4;

// ── BEGIN GENERATED (gen_limb_lut_header.py) ──
// 원장 = mjlab src/mjlab_g1_motion/tasks/mdp/limb_lut_data.py @ 4cd60406bc2c (손으로 고치지 말 것 — --write 로 다시 만든다)
inline constexpr int NB = 32;
inline constexpr float PERIOD_S = 1.64f;
inline constexpr std::array<float, N_LIMBS> LIMB_STANCE = {0.084295f, 0.084295f, 0.074519f, 0.074519f};
inline constexpr float LIMB_P[NB][N_LIMBS] = {
    {0.1247f, 0.09025f, 0.11686f, 0.07458f},
    {0.1287f, 0.08804f, 0.13022f, 0.07453f},
    {0.1395f, 0.08643f, 0.12663f, 0.07452f},
    {0.14795f, 0.08528f, 0.11476f, 0.07453f},
    {0.14838f, 0.0846f, 0.10349f, 0.07464f},
    {0.14345f, 0.0844f, 0.09826f, 0.0747f},
    {0.13638f, 0.0843f, 0.09421f, 0.0749f},
    {0.13985f, 0.08431f, 0.09174f, 0.07551f},
    {0.13629f, 0.08436f, 0.08952f, 0.07676f},
    {0.13235f, 0.08461f, 0.08636f, 0.07815f},
    {0.12991f, 0.0859f, 0.08231f, 0.08072f},
    {0.12541f, 0.08898f, 0.07829f, 0.08247f},
    {0.11962f, 0.09412f, 0.07657f, 0.0843f},
    {0.11384f, 0.09994f, 0.07586f, 0.0879f},
    {0.10931f, 0.10401f, 0.07527f, 0.09452f},
    {0.1043f, 0.10751f, 0.07479f, 0.10354f},
    {0.09025f, 0.1247f, 0.07458f, 0.11686f},
    {0.08804f, 0.1287f, 0.07453f, 0.13022f},
    {0.08643f, 0.1395f, 0.07452f, 0.12663f},
    {0.08528f, 0.14795f, 0.07453f, 0.11476f},
    {0.0846f, 0.14838f, 0.07464f, 0.10349f},
    {0.0844f, 0.14345f, 0.0747f, 0.09826f},
    {0.0843f, 0.13638f, 0.0749f, 0.09421f},
    {0.08431f, 0.13985f, 0.07551f, 0.09174f},
    {0.08436f, 0.13629f, 0.07676f, 0.08952f},
    {0.08461f, 0.13235f, 0.07815f, 0.08636f},
    {0.0859f, 0.12991f, 0.08072f, 0.08231f},
    {0.08898f, 0.12541f, 0.08247f, 0.07829f},
    {0.09412f, 0.11962f, 0.0843f, 0.07657f},
    {0.09994f, 0.11384f, 0.0879f, 0.07586f},
    {0.10401f, 0.10931f, 0.09452f, 0.07527f},
    {0.10751f, 0.1043f, 0.10354f, 0.07479f}
};
// ── END GENERATED ──

inline constexpr float STAND_EPS = 0.05f;   // limb_lut.STAND_EPS
inline constexpr float TURN_K = 0.3f;       // LocoController.turn_k 기본값 (mode1 과 같은 규약)

inline float phase_rate(float dt = 1.0f / 50.0f) { return dt / PERIOD_S; }

inline float effective_speed(float vx, float vy, float wz) { return std::hypot(vx, vy) + TURN_K * std::fabs(wz); }

// (위상 [0,1), 유효 속력) → 사지 높이 4. 학습 limb_lut.limb_z(source="table", height_scale=1).
inline std::array<float, N_LIMBS> limb_z(float phase01, float eff) {
  if (eff < STAND_EPS) return LIMB_STANCE;
  const float p = phase01 - std::floor(phase01);
  const float x = p * static_cast<float>(NB) - 0.5f;          // 구간 중심 좌표계 (구간 b 의 중심 = (b+0.5)/NB)
  int lo = static_cast<int>(std::floor(x));
  const float frac = x - static_cast<float>(lo);
  lo = ((lo % NB) + NB) % NB;                                   // 원형 (파이썬 % 와 같은 부호 규칙)
  const int hi = (lo + 1) % NB;
  std::array<float, N_LIMBS> out{};
  for (int i = 0; i < N_LIMBS; ++i)
    out[i] = LIMB_P[lo][i] * (1.0f - frac) + LIMB_P[hi][i] * frac;
  return out;
}

// 위상 시계 하나 (mode6 에 들어올 때 reset). tick() = advance_limb 와 같은 순서: 위상 먼저, 그 위상으로 높이.
struct Clock {
  float phase = 0.0f;
  void reset() { phase = 0.0f; }
  std::array<float, N_LIMBS> tick(float vx, float vy, float wz, float dt = 1.0f / 50.0f) {
    phase = std::fmod(phase + phase_rate(dt), 1.0f);
    return limb_z(phase, effective_speed(vx, vy, wz));
  }
};

}  // namespace g1::limb_lut
