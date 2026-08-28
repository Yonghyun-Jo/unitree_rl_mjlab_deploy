// 최소 스윙 클리어런스 (2026-08-28).
//
// 왜: V2 재적합이 «떠 있는» 스탠스를 바로잡으면서(발목 6.69 -> 3.50 cm) 저속 스윙이 같이
// 낮아졌다. 실측 명령 진폭 eff 0.10 = 2.68 cm · 0.15 = 3.06 cm. 초고마찰 바닥(실기)에서
// 그 높이는 발이 끌리거나 걸린다. 정책은 명령을 충실히 따르므로(실측 추종비 1.03~1.23)
// 고쳐야 할 것은 정책이 아니라 «명령» 이다.
//
// 무엇: 표 진폭이 min_swing 보다 작으면 «모양은 유지한 채» 그만큼 늘린다.
//   s(eff) = max(1, min_swing / A(eff)),  z' = stance_z + (z - stance_z) * s
// 스탠스 바닥은 곱셈의 고정점이라 접지 높이가 안 변하고, 이미 큰 속도에는 정확히 no-op 다.
// 기본값 0 = 끔 = 종전 거동 비트 동일(V1·mode2·옛 슬롯 전부 무영향).
#include "GaitLut.h"
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL %s (line %d)\n", msg, __LINE__); ++fails; } } while (0)

// 한 속도에서의 실제 스윙 진폭 (위상 전체를 훑어 잰다 — 구현과 독립적인 측정)
static float amp_at(const GlTable& T, float eff, bool run, float ms) {
    float lo = 1e9f, hi = -1e9f, zl, zr;
    for (int i = 0; i < 4000; ++i) {
        gl_foot_z(T, i / 4000.0f, eff, run, false, 0.0f, zl, zr, ms);
        lo = std::fmin(lo, zr); hi = std::fmax(hi, zr);
    }
    return hi - lo;
}

// 배포에서 실제로 쓰는 값 전부를 덮는다 — 테스트에 없는 값이 슬롯에 들어가면 잠긴 게 아니다.
static int run_all(float MS) {

    {   // ① 기본값(0) 은 정확히 no-op — 옛 슬롯·mode2 가 한 비트도 안 변해야 한다
        for (float e : {0.05f, 0.1f, 0.3f, 0.8f, 1.5f}) {
            for (float p : {0.0f, 0.13f, 0.5f, 0.87f}) {
                float a1, b1, a2, b2;
                gl_foot_z(GL_T_V2, p, e, false, true, 0.7f, a1, b1);          // 기존 시그니처
                gl_foot_z(GL_T_V2, p, e, false, true, 0.7f, a2, b2, 0.0f);    // 명시 0
                CHECK(a1 == a2 && b1 == b2, "min_swing 기본값이 no-op 이 아니다");
            }
        }
    }
    {   // ② 저속이 정확히 min_swing 으로 올라온다
        for (float e : {0.05f, 0.10f, 0.15f, 0.20f, 0.30f}) {
            const float a = amp_at(GL_T_V2, e, false, MS);
            CHECK(std::fabs(a - MS) < 1e-4f, "저속 진폭이 min_swing 이 아니다");
        }
    }
    {   // ③ 이미 큰 속도는 안 건드린다 (진폭 > min_swing 이면 s=1)
        for (float e : {0.80f, 1.00f, 1.20f}) {   // 8 cm 에서도 원래 진폭이 더 크다
            const float a0 = amp_at(GL_T_V2, e, false, 0.0f);
            const float a1 = amp_at(GL_T_V2, e, false, MS);
            CHECK(a0 > MS, "전제: 이 속도는 원래 min_swing 보다 크다");
            CHECK(std::fabs(a0 - a1) < 1e-6f, "큰 속도인데 진폭이 바뀌었다");
        }
        const float r0 = amp_at(GL_T_V2, 1.8f, true, 0.0f), r1 = amp_at(GL_T_V2, 1.8f, true, MS);
        CHECK(std::fabs(r0 - r1) < 1e-6f, "RUN 이 바뀌었다");
    }
    {   // ④ 접지 높이(궤적의 골)는 «한 비트도» 안 움직인다 — 변하면 발이 땅을 파거나 뜬다.
        //    표의 골은 stance_z 와 1~2 mm 다르므로 stance_z 가 아니라 «원래 골» 과 비교한다.
        for (float e : {0.05f, 0.10f, 0.30f}) {
            float lo0 = 1e9f, lo1 = 1e9f, zl, zr;
            for (int i = 0; i < 4000; ++i) {
                gl_foot_z(GL_T_V2, i / 4000.0f, e, false, false, 0.f, zl, zr, 0.0f);
                lo0 = std::fmin(lo0, zr);
                gl_foot_z(GL_T_V2, i / 4000.0f, e, false, false, 0.f, zl, zr, MS);
                lo1 = std::fmin(lo1, zr);
            }
            CHECK(std::fabs(lo0 - lo1) < 1e-6f, "접지 높이(골)가 움직였다");
        }
    }
    {   // ⑤ 모양 보존 — 모든 위상에서 같은 배율이어야 한다 (구간마다 다르면 궤적이 일그러진다)
        const float e = 0.10f;
        // 배율의 기준점은 stance_z 가 아니라 «그 속도 궤적의 골» 이다(구현과 같은 정의).
        float trough = 1e9f, tzl, tzr;
        for (int i = 0; i < 4000; ++i) {
            gl_foot_z(GL_T_V2, i / 4000.0f, e, false, false, 0.f, tzl, tzr, 0.0f);
            trough = std::fmin(trough, tzr);
        }
        float s_ref = -1.0f;
        for (int i = 0; i < 200; ++i) {
            const float p = i / 200.0f;
            float a0, b0, a1, b1;
            gl_foot_z(GL_T_V2, p, e, false, false, 0.f, a0, b0, 0.0f);
            gl_foot_z(GL_T_V2, p, e, false, false, 0.f, a1, b1, MS);
            const float d0 = b0 - trough, d1 = b1 - trough;
            if (std::fabs(d0) < 1e-4f) continue;             // 접지 근처는 0/0
            const float s = d1 / d0;
            if (s_ref < 0) s_ref = s;
            CHECK(std::fabs(s - s_ref) < 1e-3f, "위상마다 배율이 다르다 (모양 깨짐)");
        }
        CHECK(s_ref > 1.5f, "저속 배율이 1 에 가깝다 — 적용이 안 됐다");
    }
    {   // ⑥ 정지 게이트가 먼저다 — 명령 0 에서는 여전히 정확히 접지
        float zl, zr;
        gl_foot_z(GL_T_V2, 0.25f, 0.0f, false, true, 0.f, zl, zr, MS);
        CHECK(zl == GL_T_V2.stance_z && zr == GL_T_V2.stance_z, "명령 0 인데 발이 떴다");
    }
    {   // ⑦ 회전 비대칭과 같이 써도 두 발 «평균» 은 올라간 진폭을 따른다
        const float e = 0.10f;
        float sum_no = 0.f, sum_ms = 0.f;
        for (int i = 0; i < 400; ++i) {
            const float p = i / 400.0f;
            float a0, b0, a1, b1;
            gl_foot_z(GL_T_V2, p, e, false, true, 1.0f, a0, b0, 0.0f);
            gl_foot_z(GL_T_V2, p, e, false, true, 1.0f, a1, b1, MS);
            sum_no += (a0 + b0); sum_ms += (a1 + b1);
        }
        CHECK(sum_ms > sum_no, "회전 중에는 최소 클리어런스가 안 먹었다");
    }
    {   // ⑧ V1(mode2) 에도 같은 규칙이 적용 가능해야 한다 — 다만 켤지는 yaml 이 정한다
        const float a = amp_at(GL_T_V1, 0.10f, false, MS);
        CHECK(std::fabs(a - MS) < 1e-4f, "V1 에서 동작 안 함");
    }

    return fails;
}

int main() {
    for (float ms : {0.06f, 0.08f}) {           // v1b = 6 cm, v1c = 8 cm
        const int before = fails;
        run_all(ms);
        if (fails > before) printf("  ↑ min_swing=%.2f 에서 실패\n", ms);
    }
    printf(fails ? "[test_gait_min_swing] %d FAIL\n" : "[test_gait_min_swing] ALL PASS (6cm, 8cm)\n", fails);
    return fails ? 1 : 0;
}
