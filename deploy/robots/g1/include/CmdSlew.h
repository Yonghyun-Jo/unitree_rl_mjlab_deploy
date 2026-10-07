#pragma once
// base_vel 명령 필터 = 선형 slew (변화율 제한). 조작자 입력(조이스틱·키보드·GUI·PICO) → 컨트롤러 입력.
//
// 🔴 이 필터 하나뿐이다. 2026-10-07 EMA(A=0.25)를 걷어냈다 — 옛 코드·스위치를 되살리지 말 것.
//    EMA 는 0 에 지수로만 다가가서 «정확히 0» 정지 게이트(GaitLut stand_eps 1e-6)가 스틱을 놓고 ~0.9 s 뒤에
//    열렸고, 그 사이 0<eff<0.1 이 0.76 s 남아 min_swing 8 cm 스윙이 정지마다 꼬리 걸음 2회를 만들었다
//    (실기 09-01/03 로그 정지 221회, 감쇠비 0.75 고정).
//    sim2sim 3판(계단 입력 출발·정지 11건, 최악값 평균) ema → slew 1.5: 정지 IMU 피크 54.1 → 40.8 · 출발 골반
//    pitch 27.1 → 10.5° · 다리 토크 피크 103 → 84 Nm · 체류 기울기 24.8 → 10.3°. 대가 = 90 % 도달 0.73 → 0.86 s.
//    실기(10-07)에서 사용자 «무조건 더 좋다, 상당히 안정적». 근거 노트 = AI_workspace/unitree_rl_mjlab/experiments/
//    261007_cmd_filter_slew_ab.md. 변화율 sweep 3·2·1.5·1 과 ease-in 변형도 그 노트에 있다(1.5 가 최적).
//
// 동작: 변화율로 정규화한 «가장 늦는 축» 기준 한 비율로 세 축을 같이 움직인다 → 같은 틱에 도착(학습 명령 램프
//       torch.lerp 와 같은 직선). 이번 틱에 닿으면 x = target — 0 에 «정확히» 도착한다(정지 게이트가 제때 열림).
//       정지 시간 = |Δ| / 변화율 (1.0→0 0.67 s · 2.0→0 1.33 s · 2.5→0 1.67 s).
//
// ⚠ 값(1.5)을 바꾸면 tests/test_cmd_slew.cpp 가 실패한다 — 일부러 그렇게 묶었다. 바꾸려면 sim2sim A/B 를 다시 하고
//    노트·테스트를 같은 커밋에서 고친다.
#include <algorithm>
#include <array>
#include <cmath>

namespace g1::cmd_slew {

inline constexpr float RATE_V = 1.5f;   // vx·vy 최대 변화율 [m/s²]
inline constexpr float RATE_W = 1.5f;   // wz 최대 변화율 [rad/s²]

struct State {
    std::array<float, 3> x = {0.f, 0.f, 0.f};   // 필터 «후» 명령 [vx, vy, wz] (컨트롤러 입력)
};

// dt 초 한 틱 전진. 체류(Mimic 진입)마다 State 를 새로 만든다 = «멈춤» 에서 시작.
inline void step(State& s, const std::array<float, 3>& tgt, float dt) {
    const float r[3] = {RATE_V, RATE_V, RATE_W};
    float k = 0.f;                       // 가장 늦는 축이 이번 틱에 필요한 «한도 배수»
    for (int i = 0; i < 3; ++i) k = std::max(k, std::abs(tgt[i] - s.x[i]) / (r[i] * dt));
    if (k <= 1.0f) { s.x = tgt; return; }               // 이번 틱에 도착 — 정확히 목표(0 포함)
    for (int i = 0; i < 3; ++i) s.x[i] += (tgt[i] - s.x[i]) / k;
}

}  // namespace g1::cmd_slew
