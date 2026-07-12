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

    if (fail) { std::printf("[test_joint_safety] %d FAIL\n", fail); return 1; }
    std::printf("[test_joint_safety] ALL PASS\n"); return 0;
}
