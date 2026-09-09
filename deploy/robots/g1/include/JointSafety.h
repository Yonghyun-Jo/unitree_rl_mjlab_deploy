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

// 관절별 임계 확장(순수·테스트가능). v 가 1개면 전 관절에 복사, n개면 그대로, 그 외엔 false.
//   2026-09-09: vel_max/qd_crit 를 «하드웨어 최고속도의 90 %» 로 관절별로 건다 (무릎·hip roll 20 → 18,
//   hip pitch/yaw 32 → 29, 발목·어깨·허리 37 → 34). 스칼라 하나로는 hip pitch 를 무릎 값에 묶는다.
inline bool js_expand_per_joint(const float* v, int nv, float* out, int n) {
    if (!v || !out || n <= 0) return false;
    if (nv == 1) { for (int i = 0; i < n; ++i) out[i] = v[0]; return true; }
    if (nv == n) { for (int i = 0; i < n; ++i) out[i] = v[i]; return true; }
    return false;
}

// L3 심각도 — 관절별 warn/crit (js_qd_severity 의 배열판). 비유한값은 sev=2.
inline int js_qd_severity_v(const float* qd, int n, const float* warn, const float* crit) {
    int sev = 0;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(qd[i])) return 2;
        const float a = std::fabs(qd[i]);
        if (a > crit[i]) return 2;
        if (a > warn[i]) sev = 1;
    }
    return sev;
}

// 모니터링용: |qd|가 가장 큰 관절 인덱스. n<=0 이면 -1.
// 비유한값이 있으면 그 관절을 즉시 반환한다(js_qd_severity가 sev=2로 보는 것과 같은 우선순위).
// 판정에는 쓰지 않는다 — 로그 메시지에 "어느 관절이었나"를 붙이기 위한 것.
inline int js_qd_argmax(const float* qd, int n) {
    int best = -1; float m = -1.0f;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(qd[i])) return i;
        float a = std::fabs(qd[i]);
        if (a > m) { m = a; best = i; }
    }
    return best;
}

// L3 qd-guard 상태 스텝(순수·테스트가능). sev(0/1/2, js_qd_severity 반환)로 카운터/래치 갱신.
// warn은 sev>=1(crit도 warn 포함), crit은 sev>=2로 **독립** 누적 -> 지속 abnormal이 두 경계를 오가도
// warn은 반드시 걸린다(단조). 래치는 여기서 해제 안 함(호출측이 warn=조작자 mode1, crit=FSM 재진입으로 해제).
inline void js_qd_step(int sev, int over_ticks, int& warn_run, int& crit_run,
                       bool& warn_latched, bool& crit_latched) {
    if (sev >= 2) { if (++crit_run >= over_ticks) crit_latched = true; } else crit_run = 0;
    if (sev >= 1) { if (++warn_run >= over_ticks) warn_latched = true; } else warn_run = 0;
}
