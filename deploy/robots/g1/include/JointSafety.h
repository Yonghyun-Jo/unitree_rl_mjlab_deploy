#pragma once
// JointSafety.h — 텔레옵 출력 경로용 joint-space 안전 순수함수. self-contained(단위테스트 가능).
// 철학: 한계는 기계한계/in-distribution 밖 → 정상 동작 불변, OOD/발산만 방어. NaN-safe.
#include <algorithm>
#include <cmath>
#include <cstddef>

// 위치 clamp: isfinite면 [lo,hi]로, 아니면 그대로(앞단 NaN 가드가 처리).
inline void js_clamp_position(float* q, const float* lo, const float* hi, int n) {
    for (int i = 0; i < n; ++i)
        if (std::isfinite(q[i])) q[i] = std::clamp(q[i], lo[i], hi[i]);
}

// 속도 rate-limit: per-tick 이동을 ±max_step로 캡. NaN이면 이전값 hold. q_prev를 최종값으로 갱신.
inline void js_rate_limit(float* q, float* q_prev, const float* max_step, int n) {
    for (int i = 0; i < n; ++i) {
        float target = std::isfinite(q[i]) ? q[i] : q_prev[i];
        float d = std::clamp(target - q_prev[i], -max_step[i], max_step[i]);
        q[i] = q_prev[i] + d;
        q_prev[i] = q[i];
    }
}

// 측정 qd 심각도: 0=정상, 1=warn(>warn), 2=crit(>crit or NaN).
inline int js_qd_severity(const float* qd, int n, float warn, float crit) {
    float m = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(qd[i])) return 2;
        float a = std::fabs(qd[i]);
        if (a > m) m = a;
    }
    if (m > crit) return 2;
    if (m > warn) return 1;
    return 0;
}
