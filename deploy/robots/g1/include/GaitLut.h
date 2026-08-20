#pragma once
// GaitLut.h — mjlab_g1_motion tasks/mdp/gait_lut.py + gait_lut_data.py 의 C++ 포팅.
// 데이터 적합 발-z 참조 G(phase, v, gait): 모션 클립의 위상정규화 평균 발-z 프로파일을
// (gait, 속도)별로 구운 표. walk<->run 은 추론이 아니라 히스테리시스 1비트 상태다
// (속도만으로는 걷기/뛰기를 못 가른다). L 발 = R 발을 반주기 민 것.
//
// ⚠ 이 파일은 손으로 고치지 않는다 — 배열은 gait_lut_data.py 에서 기계 생성한 값이다.
//   상류가 재적합(fit_gait_lut.py)되면 배열을 다시 뽑고 tests/test_gait_lut.cpp 의
//   패리티 벡터도 같이 갱신할 것.
// 순수 C++ (Eigen·torch 없음) — 단위테스트가 stand-alone 으로 돈다.
#include <algorithm>
#include <cmath>

// ───────────────────────── 구운 배열 (gait_lut_data.py) ─────────────────────────
static constexpr int   GL_NB = 32;
static constexpr float GL_STANCE_Z = 0.066877f;
static constexpr int   GL_NW = 5;   // walk 속도 격자 개수
static constexpr int   GL_NR = 6;   // run  속도 격자 개수
static constexpr float GL_WALK_V[GL_NW] = {0.100000f, 0.200000f, 0.400000f, 0.800000f, 1.600000f};
static constexpr float GL_RUN_V [GL_NR] = {1.200000f, 1.600000f, 2.000000f, 2.400000f, 2.800000f, 3.200000f};
static constexpr float GL_WALK_F[GL_NW] = {0.936400f, 0.848300f, 0.782700f, 0.863800f, 1.057100f};
static constexpr float GL_RUN_F [GL_NR] = {1.120000f, 1.215500f, 1.315800f, 1.391500f, 1.469100f, 1.553100f};
static constexpr float GL_WALK_P[GL_NW][GL_NB] = {
    {0.091350f, 0.087400f, 0.084810f, 0.083490f, 0.083070f, 0.082910f, 0.082840f, 0.083090f, 0.083110f, 0.083090f, 0.083220f, 0.083550f, 0.084120f, 0.085090f, 0.086240f, 0.087460f, 0.087600f, 0.088450f, 0.089250f, 0.090210f, 0.091230f, 0.092180f, 0.093110f, 0.093880f, 0.094530f, 0.095710f, 0.097110f, 0.098910f, 0.100120f, 0.100130f, 0.098510f, 0.095570f},
    {0.092380f, 0.086860f, 0.083420f, 0.081640f, 0.080770f, 0.080260f, 0.080070f, 0.080060f, 0.080020f, 0.079760f, 0.079650f, 0.079930f, 0.080520f, 0.081480f, 0.082610f, 0.083830f, 0.084730f, 0.085490f, 0.086260f, 0.087540f, 0.089110f, 0.090860f, 0.092900f, 0.095060f, 0.097430f, 0.100110f, 0.103060f, 0.105110f, 0.106370f, 0.106220f, 0.103780f, 0.098930f},
    {0.103880f, 0.094780f, 0.088130f, 0.083350f, 0.079770f, 0.077300f, 0.075360f, 0.073860f, 0.072920f, 0.072410f, 0.072100f, 0.072130f, 0.072560f, 0.073280f, 0.074310f, 0.075460f, 0.076950f, 0.078040f, 0.079470f, 0.081700f, 0.085320f, 0.090430f, 0.097010f, 0.104500f, 0.112380f, 0.119990f, 0.126800f, 0.131820f, 0.133380f, 0.130780f, 0.124200f, 0.114880f},
    {0.109520f, 0.097230f, 0.089610f, 0.083940f, 0.078450f, 0.074510f, 0.072090f, 0.070620f, 0.069760f, 0.069420f, 0.069670f, 0.070380f, 0.071350f, 0.072810f, 0.074130f, 0.075350f, 0.079070f, 0.081890f, 0.085720f, 0.091330f, 0.099180f, 0.109530f, 0.122450f, 0.137100f, 0.152020f, 0.165160f, 0.174550f, 0.177790f, 0.173580f, 0.162540f, 0.146160f, 0.127080f},
    {0.110880f, 0.099280f, 0.091160f, 0.084850f, 0.078160f, 0.072290f, 0.068840f, 0.067180f, 0.066880f, 0.067730f, 0.069220f, 0.071080f, 0.072780f, 0.074400f, 0.076420f, 0.078750f, 0.086670f, 0.091780f, 0.098300f, 0.106690f, 0.117260f, 0.130740f, 0.146170f, 0.162320f, 0.176840f, 0.186770f, 0.190490f, 0.187130f, 0.177480f, 0.162900f, 0.145110f, 0.127240f},
};
static constexpr float GL_RUN_P[GL_NR][GL_NB] = {
    {0.111640f, 0.098850f, 0.089780f, 0.084380f, 0.081670f, 0.080280f, 0.079910f, 0.080910f, 0.083140f, 0.087270f, 0.093140f, 0.100620f, 0.109400f, 0.119070f, 0.129190f, 0.139940f, 0.139510f, 0.147790f, 0.156080f, 0.164260f, 0.172080f, 0.179280f, 0.184670f, 0.187810f, 0.188300f, 0.185450f, 0.179730f, 0.171350f, 0.161400f, 0.150580f, 0.138850f, 0.126020f},
    {0.111750f, 0.097840f, 0.088590f, 0.083760f, 0.082020f, 0.081990f, 0.082930f, 0.085100f, 0.089120f, 0.094940f, 0.102680f, 0.111980f, 0.122710f, 0.134270f, 0.146380f, 0.159710f, 0.165830f, 0.177130f, 0.187030f, 0.194940f, 0.200210f, 0.203450f, 0.204380f, 0.202940f, 0.198900f, 0.192070f, 0.183110f, 0.172940f, 0.162720f, 0.152260f, 0.140720f, 0.127530f},
    {0.110960f, 0.095620f, 0.085640f, 0.080710f, 0.079070f, 0.079160f, 0.080600f, 0.083910f, 0.089660f, 0.097810f, 0.108160f, 0.120200f, 0.133870f, 0.148480f, 0.163690f, 0.179960f, 0.198660f, 0.212300f, 0.222860f, 0.229780f, 0.232290f, 0.231450f, 0.227970f, 0.222040f, 0.213510f, 0.202700f, 0.190510f, 0.178280f, 0.167020f, 0.155960f, 0.143510f, 0.128770f},
    {0.109690f, 0.092930f, 0.082480f, 0.077970f, 0.077070f, 0.077810f, 0.079950f, 0.084240f, 0.091380f, 0.101480f, 0.114220f, 0.129080f, 0.145910f, 0.164160f, 0.183710f, 0.204070f, 0.228040f, 0.244070f, 0.255260f, 0.261000f, 0.260830f, 0.256330f, 0.248920f, 0.238750f, 0.225990f, 0.211160f, 0.195710f, 0.181570f, 0.169720f, 0.158420f, 0.145320f, 0.129550f},
    {0.110050f, 0.092720f, 0.081770f, 0.077080f, 0.076110f, 0.076910f, 0.079500f, 0.084530f, 0.092870f, 0.104610f, 0.119230f, 0.136290f, 0.155610f, 0.176650f, 0.199160f, 0.221880f, 0.252520f, 0.270040f, 0.281470f, 0.286500f, 0.284740f, 0.277970f, 0.267950f, 0.254850f, 0.239080f, 0.221130f, 0.203450f, 0.187800f, 0.174850f, 0.162520f, 0.148230f, 0.131210f},
    {0.112510f, 0.095960f, 0.085300f, 0.080510f, 0.079290f, 0.080020f, 0.082890f, 0.088760f, 0.097950f, 0.110540f, 0.126030f, 0.144040f, 0.164360f, 0.186660f, 0.210800f, 0.234730f, 0.272710f, 0.291030f, 0.302480f, 0.306520f, 0.303040f, 0.294020f, 0.281260f, 0.265120f, 0.246540f, 0.226320f, 0.207030f, 0.190930f, 0.177920f, 0.165540f, 0.150920f, 0.133670f},
};

// ───────────────────────── 조회 함수 (gait_lut.py 1:1) ─────────────────────────

// 히스테리시스 1비트 gait: run_min 위면 run, walk_max 아래면 walk, 사이는 유지.
inline bool gl_select_gait(float eff, bool is_run_prev, float walk_max, float run_min) {
    if (eff > run_min)  return true;
    if (eff < walk_max) return false;
    return is_run_prev;
}

// 속도격자 1D 보간. torch.searchsorted(right=False) 와 같은 구간 선택:
// grid[idx] >= v 인 첫 idx 를 찾고 j = clamp(idx-1, 0, n-2).
inline float gl_interp1(const float* F, const float* grid, int n, float v) {
    v = std::max(grid[0], std::min(grid[n - 1], v));
    int idx = 0;
    while (idx < n && grid[idx] < v) ++idx;
    const int j = std::max(0, std::min(idx - 1, n - 2));
    const float a = (v - grid[j]) / (grid[j + 1] - grid[j]);
    return F[j] * (1.0f - a) + F[j + 1] * a;
}

// 보폭 주파수 (Hz).
inline float gl_stride_freq(float eff, bool is_run) {
    return is_run ? gl_interp1(GL_RUN_F,  GL_RUN_V,  GL_NR, eff)
                  : gl_interp1(GL_WALK_F, GL_WALK_V, GL_NW, eff);
}

// (속도, 위상) 2D 보간 — 위상축은 순환(wrap).
inline float gl_interp2(const float* P, int stride, const float* grid, int n,
                        float phase, float v) {
    v = std::max(grid[0], std::min(grid[n - 1], v));
    int idx = 0;
    while (idx < n && grid[idx] < v) ++idx;
    const int j = std::max(0, std::min(idx - 1, n - 2));
    const float a = (v - grid[j]) / (grid[j + 1] - grid[j]);

    float ph = std::fmod(phase, 1.0f);
    if (ph < 0.0f) ph += 1.0f;                       // 파이썬 % 는 항상 양수
    const float x  = ph * GL_NB;
    const float fx = std::floor(x);
    const int   i0 = static_cast<int>(fx) % GL_NB;
    const int   i1 = (i0 + 1) % GL_NB;
    const float b  = x - fx;

    const float r0 = P[j * stride + i0]       * (1.0f - b) + P[j * stride + i1]       * b;
    const float r1 = P[(j + 1) * stride + i0] * (1.0f - b) + P[(j + 1) * stride + i1] * b;
    return r0 * (1.0f - a) + r1 * a;
}

// 위상 [0,1) + 유효속도 + gait -> 발 높이 [z_L, z_R].
// eff < 0.05 는 서있기 게이트(양발 stance).
inline void gl_foot_z(float phase, float eff, bool is_run, float& z_l, float& z_r) {
    if (eff < 0.05f) { z_l = GL_STANCE_Z; z_r = GL_STANCE_Z; return; }
    const float* P    = is_run ? &GL_RUN_P[0][0] : &GL_WALK_P[0][0];
    const float* grid = is_run ? GL_RUN_V        : GL_WALK_V;
    const int    n    = is_run ? GL_NR           : GL_NW;
    z_r = gl_interp2(P, GL_NB, grid, n, phase,          eff);
    z_l = gl_interp2(P, GL_NB, grid, n, phase + 0.5f,   eff);   // L = R 반주기 시프트
}
