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

// L3 qd-guard 상태 스텝(순수·테스트가능). sev(0/1/2, js_qd_severity 반환)로 카운터/래치 갱신.
// warn은 sev>=1(crit도 warn 포함), crit은 sev>=2로 **독립** 누적 -> 지속 abnormal이 두 경계를 오가도
// warn은 반드시 걸린다(단조). 래치는 여기서 해제 안 함(호출측이 warn=조작자 mode1, crit=FSM 재진입으로 해제).
inline void js_qd_step(int sev, int over_ticks, int& warn_run, int& crit_run,
                       bool& warn_latched, bool& crit_latched) {
    if (sev >= 2) { if (++crit_run >= over_ticks) crit_latched = true; } else crit_run = 0;
    if (sev >= 1) { if (++warn_run >= over_ticks) warn_latched = true; } else warn_run = 0;
}
