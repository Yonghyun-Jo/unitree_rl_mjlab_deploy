#pragma once
// GaitLut.h — mjlab_g1_motion tasks/mdp/gait_lut.py + gait_lut_data.py 의 C++ 포팅.
// 데이터 적합 발-z 참조 G(phase, v, gait): 모션 클립의 위상정규화 평균 발-z 프로파일을
// (gait, 속도)별로 구운 표. walk<->run 은 추론이 아니라 히스테리시스 1비트 상태다
// (속도만으로는 걷기/뛰기를 못 가른다). L 발 = R 발을 반주기 민 것.
//
// 🔴 표는 «한 벌이 아니다» (2026-08-26).
//   head 마다 학습 시점이 달라 그 시점의 표로 학습됐다. 하나로 통일하면 반드시 어느 head 가
//   학습과 다른 발-z 명령을 받는다 — MaskedLocoController::ModeGait 가 모드별로 고른다.
//     V1  motions 적합    STANCE_Z 6.6877 cm   2026-07-14(4578357) ~ 08-25
//     V2  COLMOv2 재적합  STANCE_Z 3.5000 cm   2026-08-25(d94c7d9) ~
//   V1 이 명령하던 스탠스 6.69 cm 는 로봇이 발을 디뎠을 때의 3.50 cm 보다 3.2 cm 위였다
//   (소스 리타겟이 떠 있었다). 즉 V1 은 «영원히 떠 있는 발» 을 명령한다. V2 가 그 수정판이다.
//   두 표는 STANCE_Z 뿐 아니라 진폭·케이던스가 전부 다르다 — 섞으면 안 된다.
//
// ⚠ 아래 GENERATED 구역은 손으로 고치지 않는다.
//     python3 deploy/scripts/gen_gait_lut_header.py --check   # 상류와 일치하는지
//     python3 deploy/scripts/gen_gait_lut_header.py --write   # 재적합 후 갱신
//   갱신하면 tests/test_gait_lut.cpp 의 패리티 벡터도 mjlab 에서 다시 뽑을 것.
// 순수 C++ (Eigen·torch 없음) — 단위테스트가 stand-alone 으로 돈다.
#include <algorithm>
#include <cmath>

// 한 벌의 구운 표. 배열은 GENERATED 구역이 소유하고 여기서는 가리키기만 한다.
struct GlTable {
    int   nb, nw, nr;
    float stance_z;
    const float* walk_v;  const float* run_v;
    const float* walk_f;  const float* run_f;
    const float* walk_p;  const float* run_p;    // [n][nb] 를 편 것
    // 회전 비대칭 (gait_lut.turn_asym): 바깥발이 더 긴 호를 걸어 더 높이 든다.
    float asym_k_walk, asym_wref_walk, asym_k_run, asym_wref_run;
    // 정지 게이트 임계. 2026-08-25(09fb616) 이전 0.05, 이후 «정확히 0»(1e-6).
    // 표 판번호와 게이트 변경이 같은 날 연속 커밋이라 표에 묶어 둔다.
    float stand_eps;
};

// ── BEGIN GENERATED (gen_gait_lut_header.py) ──
// 생성기: deploy/scripts/gen_gait_lut_header.py
// 상류:   mjlab_g1_motion src/mjlab_g1_motion/tasks/mdp/gait_lut_data.py
//         V1 = d94c7d9~1  ·  V2 = 95cc965 (워킹트리)
// ⚠ 이 구역을 손으로 고치지 않는다. 상류 재적합 후 --write 로 다시 뽑고,
//   tests/test_gait_lut.cpp 의 패리티 벡터도 mjlab 에서 다시 뽑을 것.

// ── V1: motions 적합 (2026-07-14 4578357). mode2 head 가 이 표로 학습됐다.
inline constexpr int   GL_V1_NB = 32;
inline constexpr int   GL_V1_NW = 5;
inline constexpr int   GL_V1_NR = 6;
inline constexpr float GL_V1_STANCE_Z = 0.066877f;
inline constexpr float GL_V1_WALK_V[GL_V1_NW] = {0.100000f, 0.200000f, 0.400000f, 0.800000f, 1.600000f};
inline constexpr float GL_V1_RUN_V [GL_V1_NR] = {1.200000f, 1.600000f, 2.000000f, 2.400000f, 2.800000f, 3.200000f};
inline constexpr float GL_V1_WALK_F[GL_V1_NW] = {0.936400f, 0.848300f, 0.782700f, 0.863800f, 1.057100f};
inline constexpr float GL_V1_RUN_F [GL_V1_NR] = {1.120000f, 1.215500f, 1.315800f, 1.391500f, 1.469100f, 1.553100f};
inline constexpr float GL_V1_WALK_P[GL_V1_NW][GL_V1_NB] = {
    {0.091350f, 0.087400f, 0.084810f, 0.083490f, 0.083070f, 0.082910f, 0.082840f, 0.083090f, 0.083110f, 0.083090f, 0.083220f, 0.083550f, 0.084120f, 0.085090f, 0.086240f, 0.087460f, 0.087600f, 0.088450f, 0.089250f, 0.090210f, 0.091230f, 0.092180f, 0.093110f, 0.093880f, 0.094530f, 0.095710f, 0.097110f, 0.098910f, 0.100120f, 0.100130f, 0.098510f, 0.095570f},
    {0.092380f, 0.086860f, 0.083420f, 0.081640f, 0.080770f, 0.080260f, 0.080070f, 0.080060f, 0.080020f, 0.079760f, 0.079650f, 0.079930f, 0.080520f, 0.081480f, 0.082610f, 0.083830f, 0.084730f, 0.085490f, 0.086260f, 0.087540f, 0.089110f, 0.090860f, 0.092900f, 0.095060f, 0.097430f, 0.100110f, 0.103060f, 0.105110f, 0.106370f, 0.106220f, 0.103780f, 0.098930f},
    {0.103880f, 0.094780f, 0.088130f, 0.083350f, 0.079770f, 0.077300f, 0.075360f, 0.073860f, 0.072920f, 0.072410f, 0.072100f, 0.072130f, 0.072560f, 0.073280f, 0.074310f, 0.075460f, 0.076950f, 0.078040f, 0.079470f, 0.081700f, 0.085320f, 0.090430f, 0.097010f, 0.104500f, 0.112380f, 0.119990f, 0.126800f, 0.131820f, 0.133380f, 0.130780f, 0.124200f, 0.114880f},
    {0.109520f, 0.097230f, 0.089610f, 0.083940f, 0.078450f, 0.074510f, 0.072090f, 0.070620f, 0.069760f, 0.069420f, 0.069670f, 0.070380f, 0.071350f, 0.072810f, 0.074130f, 0.075350f, 0.079070f, 0.081890f, 0.085720f, 0.091330f, 0.099180f, 0.109530f, 0.122450f, 0.137100f, 0.152020f, 0.165160f, 0.174550f, 0.177790f, 0.173580f, 0.162540f, 0.146160f, 0.127080f},
    {0.110880f, 0.099280f, 0.091160f, 0.084850f, 0.078160f, 0.072290f, 0.068840f, 0.067180f, 0.066880f, 0.067730f, 0.069220f, 0.071080f, 0.072780f, 0.074400f, 0.076420f, 0.078750f, 0.086670f, 0.091780f, 0.098300f, 0.106690f, 0.117260f, 0.130740f, 0.146170f, 0.162320f, 0.176840f, 0.186770f, 0.190490f, 0.187130f, 0.177480f, 0.162900f, 0.145110f, 0.127240f},
};
inline constexpr float GL_V1_RUN_P[GL_V1_NR][GL_V1_NB] = {
    {0.111640f, 0.098850f, 0.089780f, 0.084380f, 0.081670f, 0.080280f, 0.079910f, 0.080910f, 0.083140f, 0.087270f, 0.093140f, 0.100620f, 0.109400f, 0.119070f, 0.129190f, 0.139940f, 0.139510f, 0.147790f, 0.156080f, 0.164260f, 0.172080f, 0.179280f, 0.184670f, 0.187810f, 0.188300f, 0.185450f, 0.179730f, 0.171350f, 0.161400f, 0.150580f, 0.138850f, 0.126020f},
    {0.111750f, 0.097840f, 0.088590f, 0.083760f, 0.082020f, 0.081990f, 0.082930f, 0.085100f, 0.089120f, 0.094940f, 0.102680f, 0.111980f, 0.122710f, 0.134270f, 0.146380f, 0.159710f, 0.165830f, 0.177130f, 0.187030f, 0.194940f, 0.200210f, 0.203450f, 0.204380f, 0.202940f, 0.198900f, 0.192070f, 0.183110f, 0.172940f, 0.162720f, 0.152260f, 0.140720f, 0.127530f},
    {0.110960f, 0.095620f, 0.085640f, 0.080710f, 0.079070f, 0.079160f, 0.080600f, 0.083910f, 0.089660f, 0.097810f, 0.108160f, 0.120200f, 0.133870f, 0.148480f, 0.163690f, 0.179960f, 0.198660f, 0.212300f, 0.222860f, 0.229780f, 0.232290f, 0.231450f, 0.227970f, 0.222040f, 0.213510f, 0.202700f, 0.190510f, 0.178280f, 0.167020f, 0.155960f, 0.143510f, 0.128770f},
    {0.109690f, 0.092930f, 0.082480f, 0.077970f, 0.077070f, 0.077810f, 0.079950f, 0.084240f, 0.091380f, 0.101480f, 0.114220f, 0.129080f, 0.145910f, 0.164160f, 0.183710f, 0.204070f, 0.228040f, 0.244070f, 0.255260f, 0.261000f, 0.260830f, 0.256330f, 0.248920f, 0.238750f, 0.225990f, 0.211160f, 0.195710f, 0.181570f, 0.169720f, 0.158420f, 0.145320f, 0.129550f},
    {0.110050f, 0.092720f, 0.081770f, 0.077080f, 0.076110f, 0.076910f, 0.079500f, 0.084530f, 0.092870f, 0.104610f, 0.119230f, 0.136290f, 0.155610f, 0.176650f, 0.199160f, 0.221880f, 0.252520f, 0.270040f, 0.281470f, 0.286500f, 0.284740f, 0.277970f, 0.267950f, 0.254850f, 0.239080f, 0.221130f, 0.203450f, 0.187800f, 0.174850f, 0.162520f, 0.148230f, 0.131210f},
    {0.112510f, 0.095960f, 0.085300f, 0.080510f, 0.079290f, 0.080020f, 0.082890f, 0.088760f, 0.097950f, 0.110540f, 0.126030f, 0.144040f, 0.164360f, 0.186660f, 0.210800f, 0.234730f, 0.272710f, 0.291030f, 0.302480f, 0.306520f, 0.303040f, 0.294020f, 0.281260f, 0.265120f, 0.246540f, 0.226320f, 0.207030f, 0.190930f, 0.177920f, 0.165540f, 0.150920f, 0.133670f},
};
inline constexpr GlTable GL_T_V1 = {
    GL_V1_NB, GL_V1_NW, GL_V1_NR, GL_V1_STANCE_Z,
    GL_V1_WALK_V, GL_V1_RUN_V, GL_V1_WALK_F, GL_V1_RUN_F,
    &GL_V1_WALK_P[0][0], &GL_V1_RUN_P[0][0],
    0.116900f, 1.250000f, 0.282700f, 1.800000f,
    0.05f,
};

// ── V2: COLMOv2 재적합 (2026-08-25 d94c7d9). 새 mode1 head 가 이 표로 학습됐다.
inline constexpr int   GL_V2_NB = 32;
inline constexpr int   GL_V2_NW = 5;
inline constexpr int   GL_V2_NR = 6;
inline constexpr float GL_V2_STANCE_Z = 0.035000f;
inline constexpr float GL_V2_WALK_V[GL_V2_NW] = {0.100000f, 0.200000f, 0.400000f, 0.800000f, 1.600000f};
inline constexpr float GL_V2_RUN_V [GL_V2_NR] = {1.200000f, 1.600000f, 2.000000f, 2.400000f, 2.800000f, 3.200000f};
inline constexpr float GL_V2_WALK_F[GL_V2_NW] = {0.646800f, 0.704200f, 0.943400f, 0.998000f, 1.111100f};
inline constexpr float GL_V2_RUN_F [GL_V2_NR] = {1.351400f, 1.445100f, 1.515200f, 1.515200f, 1.612900f, 1.724100f};
inline constexpr float GL_V2_WALK_P[GL_V2_NW][GL_V2_NB] = {
    {0.051800f, 0.045400f, 0.042630f, 0.041150f, 0.040470f, 0.040370f, 0.040960f, 0.040950f, 0.039570f, 0.038530f, 0.038370f, 0.038270f, 0.037720f, 0.037010f, 0.036510f, 0.036650f, 0.038090f, 0.039730f, 0.041560f, 0.043750f, 0.044700f, 0.044520f, 0.044420f, 0.044130f, 0.044750f, 0.045990f, 0.047710f, 0.050030f, 0.053950f, 0.059680f, 0.063270f, 0.060140f},
    {0.053020f, 0.045550f, 0.042030f, 0.040280f, 0.039380f, 0.038820f, 0.039130f, 0.039380f, 0.038440f, 0.037470f, 0.036930f, 0.036620f, 0.036520f, 0.036350f, 0.036390f, 0.036960f, 0.037620f, 0.038990f, 0.040880f, 0.043760f, 0.046230f, 0.048110f, 0.049570f, 0.050750f, 0.052520f, 0.055040f, 0.058390f, 0.062480f, 0.067240f, 0.071200f, 0.070880f, 0.063700f},
    {0.060980f, 0.051980f, 0.046010f, 0.042150f, 0.040070f, 0.038820f, 0.038030f, 0.037370f, 0.036640f, 0.036050f, 0.035390f, 0.035000f, 0.035020f, 0.035380f, 0.036110f, 0.037200f, 0.038960f, 0.040690f, 0.043180f, 0.046890f, 0.051550f, 0.057090f, 0.063250f, 0.070040f, 0.077280f, 0.084080f, 0.089720f, 0.093330f, 0.094590f, 0.092570f, 0.085370f, 0.073960f},
    {0.062940f, 0.053630f, 0.047700f, 0.043580f, 0.040640f, 0.038770f, 0.037850f, 0.037530f, 0.037410f, 0.037380f, 0.037260f, 0.037090f, 0.037050f, 0.037180f, 0.037710f, 0.038920f, 0.043390f, 0.047370f, 0.053240f, 0.061430f, 0.071640f, 0.083280f, 0.095560f, 0.107480f, 0.118000f, 0.125860f, 0.129610f, 0.127940f, 0.120320f, 0.107840f, 0.092400f, 0.076630f},
    {0.064250f, 0.054330f, 0.047150f, 0.042660f, 0.040290f, 0.039580f, 0.039990f, 0.040820f, 0.041350f, 0.041260f, 0.041310f, 0.041850f, 0.042770f, 0.044320f, 0.047050f, 0.051590f, 0.064330f, 0.073500f, 0.084790f, 0.097860f, 0.111530f, 0.123720f, 0.132710f, 0.137650f, 0.138920f, 0.136350f, 0.128570f, 0.116820f, 0.104430f, 0.093490f, 0.084450f, 0.075340f},
};
inline constexpr float GL_V2_RUN_P[GL_V2_NR][GL_V2_NB] = {
    {0.070150f, 0.060910f, 0.055150f, 0.052660f, 0.051990f, 0.052460f, 0.053530f, 0.055520f, 0.058750f, 0.062940f, 0.068560f, 0.075620f, 0.083950f, 0.093930f, 0.105470f, 0.118730f, 0.115820f, 0.125300f, 0.134230f, 0.141860f, 0.147090f, 0.150180f, 0.150920f, 0.149330f, 0.145660f, 0.139610f, 0.131990f, 0.123450f, 0.114710f, 0.104960f, 0.094010f, 0.082630f},
    {0.069700f, 0.060070f, 0.054910f, 0.053440f, 0.054000f, 0.055180f, 0.056750f, 0.059150f, 0.062790f, 0.068200f, 0.075330f, 0.083970f, 0.094190f, 0.105830f, 0.118920f, 0.133060f, 0.141060f, 0.152620f, 0.162290f, 0.169200f, 0.172610f, 0.172730f, 0.169890f, 0.164630f, 0.157220f, 0.147910f, 0.137990f, 0.127670f, 0.117500f, 0.106990f, 0.095510f, 0.083050f},
    {0.068500f, 0.057970f, 0.052710f, 0.051430f, 0.051990f, 0.053240f, 0.055490f, 0.059120f, 0.064540f, 0.072080f, 0.081500f, 0.092990f, 0.106650f, 0.122200f, 0.139230f, 0.156700f, 0.176080f, 0.189450f, 0.199180f, 0.203710f, 0.202870f, 0.197640f, 0.189220f, 0.178670f, 0.166590f, 0.153890f, 0.142010f, 0.130960f, 0.120580f, 0.109820f, 0.097480f, 0.083690f},
    {0.066500f, 0.055140f, 0.049720f, 0.048660f, 0.049580f, 0.051470f, 0.054630f, 0.059480f, 0.066310f, 0.075510f, 0.087000f, 0.101090f, 0.118020f, 0.137520f, 0.158800f, 0.180080f, 0.202720f, 0.218180f, 0.228150f, 0.231110f, 0.227070f, 0.217700f, 0.204740f, 0.189580f, 0.173340f, 0.157320f, 0.143260f, 0.131490f, 0.121260f, 0.110750f, 0.098020f, 0.083060f},
    {0.064910f, 0.052870f, 0.047290f, 0.046400f, 0.047680f, 0.050290f, 0.054880f, 0.061200f, 0.069380f, 0.079940f, 0.092910f, 0.108880f, 0.128260f, 0.150860f, 0.175610f, 0.199860f, 0.225920f, 0.242990f, 0.253030f, 0.254650f, 0.248390f, 0.235950f, 0.219330f, 0.200590f, 0.181110f, 0.162680f, 0.147100f, 0.134600f, 0.123900f, 0.112730f, 0.098810f, 0.082710f},
    {0.064360f, 0.052540f, 0.047390f, 0.047070f, 0.049170f, 0.052980f, 0.059010f, 0.066660f, 0.075940f, 0.087550f, 0.101810f, 0.119400f, 0.140850f, 0.165850f, 0.192990f, 0.218880f, 0.250830f, 0.267940f, 0.276380f, 0.275190f, 0.265370f, 0.248930f, 0.227850f, 0.204930f, 0.182110f, 0.161340f, 0.144760f, 0.132690f, 0.122900f, 0.112590f, 0.098830f, 0.082530f},
};
inline constexpr GlTable GL_T_V2 = {
    GL_V2_NB, GL_V2_NW, GL_V2_NR, GL_V2_STANCE_Z,
    GL_V2_WALK_V, GL_V2_RUN_V, GL_V2_WALK_F, GL_V2_RUN_F,
    &GL_V2_WALK_P[0][0], &GL_V2_RUN_P[0][0],
    0.102200f, 1.200000f, 0.179900f, 2.000000f,
    1e-6f,
};
// ── END GENERATED ──

// 종전 이름 유지 (State_Mimic / test_loco_gait_modes 가 쓴다). V1 = 기존 배포 표.
inline constexpr float GL_STANCE_Z = GL_V1_STANCE_Z;

// ───────────────────────── 조회 함수 (gait_lut.py 1:1) ─────────────────────────

// 히스테리시스 1비트 gait: run_min 위면 run, walk_max 아래면 walk, 사이는 유지.
// 표에 의존하지 않는다(속도 경계는 cfg).
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
inline float gl_stride_freq(const GlTable& T, float eff, bool is_run) {
    return is_run ? gl_interp1(T.run_f,  T.run_v,  T.nr, eff)
                  : gl_interp1(T.walk_f, T.walk_v, T.nw, eff);
}

// (속도, 위상) 2D 보간 — 위상축은 순환(wrap).
inline float gl_interp2(const float* P, int nb, const float* grid, int n,
                        float phase, float v) {
    v = std::max(grid[0], std::min(grid[n - 1], v));
    int idx = 0;
    while (idx < n && grid[idx] < v) ++idx;
    const int j = std::max(0, std::min(idx - 1, n - 2));
    const float a = (v - grid[j]) / (grid[j + 1] - grid[j]);

    float ph = std::fmod(phase, 1.0f);
    if (ph < 0.0f) ph += 1.0f;                       // 파이썬 % 는 항상 양수
    const float x  = ph * nb;
    const float fx = std::floor(x);
    const int   i0 = static_cast<int>(fx) % nb;
    const int   i1 = (i0 + 1) % nb;
    const float b  = x - fx;

    const float r0 = P[j * nb + i0]       * (1.0f - b) + P[j * nb + i1]       * b;
    const float r1 = P[(j + 1) * nb + i0] * (1.0f - b) + P[(j + 1) * nb + i1] * b;
    return r0 * (1.0f - a) + r1 * a;
}

// 회전 비대칭 배율 [s_L, s_R] (gait_lut.turn_asym). wz=0 에서 정확히 1.0/1.0.
// 부호: +wz = CCW = 좌회전 -> 왼발이 «안쪽»(낮게 든다). 두 배율의 평균은 정확히 1 이라
// 두 발 평균 스윙은 안 변하고 «분배»만 바뀐다.
inline void gl_turn_asym(const GlTable& T, float wz, bool is_run, float& s_l, float& s_r) {
    const float k    = is_run ? T.asym_k_run    : T.asym_k_walk;
    const float wref = is_run ? T.asym_wref_run : T.asym_wref_walk;
    const float u    = std::min(1.0f, std::max(0.0f, std::abs(wz) / wref));
    const float r    = 1.0f + k * u;
    const float s_in  = 2.0f / (1.0f + r);
    const float s_out = 2.0f * r / (1.0f + r);
    const bool left_inside = wz > 0.0f;
    s_l = left_inside ? s_in  : s_out;
    s_r = left_inside ? s_out : s_in;
}

// 위상 [0,1) + 유효속도 + gait -> 발 높이 [z_L, z_R].
//   use_asym=false, wz=0 이면 2026-08-17 이전 거동과 비트 동일(zL = zR 반주기 시프트).
//   정지 게이트 임계는 표가 갖는다(T.stand_eps).
inline void gl_foot_z(const GlTable& T, float phase, float eff, bool is_run,
                      bool use_asym, float wz, float& z_l, float& z_r) {
    if (eff < T.stand_eps) { z_l = T.stance_z; z_r = T.stance_z; return; }
    const float* P    = is_run ? T.run_p : T.walk_p;
    const float* grid = is_run ? T.run_v : T.walk_v;
    const int    n    = is_run ? T.nr    : T.nw;
    z_r = gl_interp2(P, T.nb, grid, n, phase,        eff);
    z_l = gl_interp2(P, T.nb, grid, n, phase + 0.5f, eff);   // L = R 반주기 시프트
    if (use_asym) {
        float s_l = 1.0f, s_r = 1.0f;
        gl_turn_asym(T, wz, is_run, s_l, s_r);
        // 스탠스 바닥은 고정하고 «그 위의 스윙» 만 배율. 파이썬과 같은 형태로 써서
        // 배율 1 이 정확히 0.0 을 더하게 한다(왕복형은 마지막 mantissa 비트를 잃는다).
        z_l = z_l + (z_l - T.stance_z) * (s_l - 1.0f);
        z_r = z_r + (z_r - T.stance_z) * (s_r - 1.0f);
    }
}

// 위상이 이번 스텝에 target 을 «지났는가». 위상은 [0,1) 를 감으므로 wrap 을 본다.
// target 이 [0,1) 밖이면 절대 안 지난다 = 위상조건 없음(예산 소진만으로 끝남).
// 파이썬 loco_controller._phase_crossed 와 1:1.
inline bool gl_phase_crossed(float prev, float cur, float target) {
    if (!(target >= 0.0f && target < 1.0f)) return false;
    if (cur < prev) return (prev < target) || (cur >= target);   // wrap
    return (prev < target) && (cur >= target);
}

// ── 종전 시그니처 (표 인자 없음) = V1 + asym 없음 = 2026-08-26 이전 배포 거동 그대로.
inline float gl_stride_freq(float eff, bool is_run) {
    return gl_stride_freq(GL_T_V1, eff, is_run);
}
inline void gl_foot_z(float phase, float eff, bool is_run, float& z_l, float& z_r) {
    gl_foot_z(GL_T_V1, phase, eff, is_run, false, 0.0f, z_l, z_r);
}
