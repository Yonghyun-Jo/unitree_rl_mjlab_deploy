// test_joint_safety.cpp — JointSafety 순수함수 자체검증 (기존 test_masked_loco_controller 관례).
//   cd deploy/robots/g1/tests && g++ -std=c++17 -I../include -O2 test_joint_safety.cpp -o /tmp/tjs && /tmp/tjs
#include "JointSafety.h"
#include <cstdio>
#include <cmath>
#include <limits>

static int fail = 0;
static bool close(float a, float b) { return std::fabs(a - b) < 1e-6f; }
#define CHK(c, msg) do{ if(!(c)){ std::printf("FAIL %s\n", msg); ++fail; } }while(0)

int main() {
    // clamp: wide -> no-op; tight -> clamp; NaN -> untouched
    { float q[3]={0.5f,-3.0f,2.0f}; float lo[3]={-10,-10,-10}, hi[3]={10,10,10};
      js_clamp_position(q,lo,hi,3); CHK(close(q[0],0.5f)&&close(q[1],-3.0f)&&close(q[2],2.0f),"clamp wide no-op"); }
    { float q[3]={0.5f,-3.0f,2.0f}; float lo[3]={-1,-1,-1}, hi[3]={1,1,1};
      js_clamp_position(q,lo,hi,3); CHK(close(q[0],0.5f)&&close(q[1],-1.0f)&&close(q[2],1.0f),"clamp tight"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float q[1]={nan}; float lo[1]={-1},hi[1]={1};
      js_clamp_position(q,lo,hi,1); CHK(std::isnan(q[0]),"clamp NaN untouched"); }

    // qd_argmax: 부호 무시 최대, 빈 배열 -1, NaN 우선
    { float qd[4]={1.0f,-7.5f,3.0f,7.4f}; CHK(js_qd_argmax(qd,4)==1,"qd_argmax picks max |qd|"); }
    { CHK(js_qd_argmax(nullptr,0)==-1,"qd_argmax empty -> -1"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float qd[3]={9.0f,nan,1.0f};
      CHK(js_qd_argmax(qd,3)==1,"qd_argmax NaN takes priority over larger finite"); }

    // rate_limit: within step -> no-op + prev updates; over step -> capped; NaN -> hold prev
    { float q[1]={0.1f}; float prev[1]={0.0f}; float step[1]={1.0f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.1f)&&close(prev[0],0.1f),"rate within"); }
    { float q[1]={5.0f}; float prev[1]={0.0f}; float step[1]={0.5f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.5f)&&close(prev[0],0.5f),"rate cap up"); }
    { float q[1]={-5.0f}; float prev[1]={0.0f}; float step[1]={0.5f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],-0.5f)&&close(prev[0],-0.5f),"rate cap down"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float q[1]={nan}; float prev[1]={0.3f}; float step[1]={1.0f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.3f)&&close(prev[0],0.3f),"rate NaN hold"); }

    // qd_severity: tiers + NaN->crit
    { float qd[3]={1,2,3}; CHK(js_qd_severity(qd,3,10,20)==0,"qd ok"); }
    { float qd[3]={1,15,3}; CHK(js_qd_severity(qd,3,10,20)==1,"qd warn"); }
    { float qd[3]={1,-25,3}; CHK(js_qd_severity(qd,3,10,20)==2,"qd crit"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float qd[2]={1,nan};
      CHK(js_qd_severity(qd,2,10,20)==2,"qd NaN->crit"); }

    // 정확한 경계 테스트: js_qd_severity strict > (>=가 아님) 검증
    { float qd[1]={10.0f}; CHK(js_qd_severity(qd,1,10,20)==0,"qd exact warn boundary (==warn, not >warn)"); }
    { float qd[1]={20.0f}; CHK(js_qd_severity(qd,1,10,20)==1,"qd exact crit boundary (>warn, not >crit)"); }

    // 정확한 경계 테스트: js_rate_limit delta == max_step (캡핑 아님)
    { float q[1]={0.5f}; float prev[1]={0.0f}; float step[1]={0.5f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.5f)&&close(prev[0],0.5f),"rate delta==max_step (exact boundary)"); }

    // 정확한 경계 테스트: js_clamp_position q == lo, hi 경계 (std::clamp 포함경계)
    { float q[2]={-1.0f,1.0f}; float lo[2]={-1,-1}, hi[2]={1,1};
      js_clamp_position(q,lo,hi,2); CHK(close(q[0],-1.0f)&&close(q[1],1.0f),"clamp exact boundaries (lo/hi inclusive)"); }

    // js_qd_step: warn/crit 독립 누적 (I1 회귀 방지 — 경계를 오가는 발산이 두 래치를 서로 지우면 안 됨)
    { // 5연속 sev=1(over_ticks=5) -> warn_latched true, crit_latched false
      int warn_run=0, crit_run=0; bool warn_l=false, crit_l=false;
      for (int i=0;i<5;++i) js_qd_step(1,5,warn_run,crit_run,warn_l,crit_l);
      CHK(warn_l && !crit_l, "qd_step 5x sev1 -> warn latched only"); }
    { // 5연속 sev=2 -> 둘 다 래치 (crit이 warn을 함의)
      int warn_run=0, crit_run=0; bool warn_l=false, crit_l=false;
      for (int i=0;i<5;++i) js_qd_step(2,5,warn_run,crit_run,warn_l,crit_l);
      CHK(warn_l && crit_l, "qd_step 5x sev2 -> both latched"); }
    { // 오실레이션 sev=1,2,1,2,1 (5틱) -> warn_latched true(I1 회귀 케이스), crit_latched false(sev=1 틱마다 crit_run 리셋)
      int warn_run=0, crit_run=0; bool warn_l=false, crit_l=false;
      int seq[5] = {1,2,1,2,1};
      for (int i=0;i<5;++i) js_qd_step(seq[i],5,warn_run,crit_run,warn_l,crit_l);
      CHK(warn_l && !crit_l, "qd_step oscillating sev1/2 -> warn latched, crit not (I1 regression)"); }
    { // 4x sev=1 후 sev=0 -> warn_run 리셋, 래치 안 됨(디바운스)
      int warn_run=0, crit_run=0; bool warn_l=false, crit_l=false;
      for (int i=0;i<4;++i) js_qd_step(1,5,warn_run,crit_run,warn_l,crit_l);
      js_qd_step(0,5,warn_run,crit_run,warn_l,crit_l);
      CHK(!warn_l && !crit_l && warn_run==0, "qd_step 4x sev1 then sev0 -> reset, not latched (debounce)"); }

    // ── 관절별 임계 (2026-09-09) ────────────────────────────────────────────
    {
        float out[4]; float one[1] = {15.f}; float four[4] = {29.f,18.f,29.f,18.f}; float bad[2] = {1.f,2.f};
        CHK(js_expand_per_joint(one, 1, out, 4) && close(out[0],15.f) && close(out[3],15.f), "스칼라 → 전 관절 복사");
        CHK(js_expand_per_joint(four, 4, out, 4) && close(out[1],18.f) && close(out[2],29.f), "n개 → 그대로");
        CHK(!js_expand_per_joint(bad, 2, out, 4), "길이 불일치는 거부");
        CHK(!js_expand_per_joint(nullptr, 1, out, 4), "null 거부");
        // 관절별 severity: hip pitch 25 rad/s 는 crit 31 아래(sev 1: warn 12 위), 무릎 25 는 crit 19 위(sev 2)
        float warn[2] = {12.f,12.f}, crit[2] = {31.f,19.f};
        float qd_a[2] = {25.f, 5.f};  CHK(js_qd_severity_v(qd_a, 2, warn, crit) == 1, "hip 25 → warn 만 (crit 31)");
        float qd_b[2] = {5.f, -25.f}; CHK(js_qd_severity_v(qd_b, 2, warn, crit) == 2, "knee 25 → crit (19)");
        float qd_c[2] = {1.f, 1.f};   CHK(js_qd_severity_v(qd_c, 2, warn, crit) == 0, "조용하면 0");
        float qd_d[2] = {1.f, std::numeric_limits<float>::quiet_NaN()}; CHK(js_qd_severity_v(qd_d, 2, warn, crit) == 2, "NaN 은 crit");
        // 스칼라판과 동치: 임계가 전 관절 같으면 결과가 같아야 한다
        float w15[2] = {12.f,12.f}, c18[2] = {18.f,18.f}; float qd_e[2] = {17.f, 3.f};
        CHK(js_qd_severity_v(qd_e, 2, w15, c18) == js_qd_severity(qd_e, 2, 12.f, 18.f), "스칼라판과 동치");
    }

    if (fail) { std::printf("[test_joint_safety] %d FAIL\n", fail); return 1; }
    std::printf("[test_joint_safety] ALL PASS\n"); return 0;
}
