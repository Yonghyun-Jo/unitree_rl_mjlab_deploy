// 정지 정착(settle) — 명령이 0 이 되면 «한 걸음 더» 굴러 발을 모으고 멈춘다 (2026-08-26).
// 파이썬 loco_controller 의 정착 상태기계와 1:1. 상류 대조는
//   mjlab_g1_motion tests/test_settle_stop.py + 세션 교차검증(9600 값 일치)에서 이미 했고,
//   여기서는 «배포가 그 계약을 계속 지키는가» 를 불변식으로 잠근다.
// 순수 C++ — g++ -std=c++17 -I ../include test_settle_stop.cpp
#include "MaskedLocoController.h"
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL %s (line %d)\n", msg, __LINE__); ++fails; } } while (0)

static MaskedLocoController make(int settle_steps, float settle_phase = 0.07f) {
    MaskedLocoController c;
    // mode1 = V2 표 · cadence 1.15 · asym on (= 2026-08-26 배포 mode1)
    c.mode_gait[1] = {true, 1.0f, GL_V2_STANCE_Z, 0.0f, 2, 1.15f, true, settle_steps};
    c.settle_eff = 0.15f; c.settle_phase = settle_phase; c.bv_ramp_steps = 0;
    return c;
}
static void walk(MaskedLocoController& c, int n) { for (int i=0;i<n;++i) c.update({0.6f,0.f,0.f},1); }
static bool planted(const MaskedLocoController& c) {
    return std::fabs(c.foot_z[0]-GL_V2_STANCE_Z) < 1e-6f && std::fabs(c.foot_z[1]-GL_V2_STANCE_Z) < 1e-6f;
}

int main() {
    {   // 명령 0 직후 정착이 열리고, 아직 접지가 아니다
        auto c = make(100); walk(c, 60); c.update({0.f,0.f,0.f}, 1);
        CHECK(c.settling, "정지 직후 정착이 안 열렸다");
        CHECK(!planted(c), "정착 중인데 벌써 양발 접지");
    }
    {   // φ_stop 을 지나면 종료 + 접지, 종료 위상이 목표 바로 뒤
        auto c = make(100); walk(c, 60);
        bool seen = false;
        for (int i = 0; i < 400; ++i) {
            c.update({0.f,0.f,0.f}, 1);
            if (c.settling) { seen = true; continue; }
            CHECK(seen, "정착이 한 번도 안 열렸다");
            CHECK(planted(c), "종료 후 접지가 아니다");
            CHECK(c.phase_f >= 0.07f && c.phase_f < 0.07f + 0.02f, "종료 위상이 목표 밖");
            break;
        }
    }
    {   // 위상을 절대 못 지나도 예산에서 끝난다 — «정지를 눌렀는데 안 멈춤» 금지
        auto c = make(30, -1.0f); walk(c, 60);
        int i = 0; for (; i < 200; ++i) { c.update({0.f,0.f,0.f},1); if (!c.settling) break; }
        CHECK(i <= 30, "예산 상한이 안 걸렸다");
        CHECK(planted(c), "타임아웃 후 접지가 아니다");
    }
    {   // 명령이 돌아오면 즉시 취소 (조작 지연 금지)
        auto c = make(100); walk(c, 60);
        c.update({0.f,0.f,0.f},1); CHECK(c.settling, "정착 미개시");
        c.update({0.6f,0.f,0.f},1); CHECK(!c.settling, "명령 복귀에도 정착이 안 풀렸다");
    }
    {   // 계속 서 있으면 한 번만 — 재발동하면 제자리 마칭이 된다
        auto c = make(100); walk(c, 60);
        int opened = 0; bool prev = false;
        for (int i = 0; i < 600; ++i) { c.update({0.f,0.f,0.f},1);
            if (c.settling && !prev) ++opened; prev = c.settling; }
        CHECK(opened == 1, "정착이 한 번을 넘어 열렸다");
    }
    {   // 기본값 = 꺼짐 = 2026-08-26 이전 거동 그대로
        auto c = make(0); walk(c, 60); c.update({0.f,0.f,0.f},1);
        CHECK(!c.settling && planted(c), "settle_steps=0 인데 즉시 접지가 아니다");
    }
    {   // 모드별이다 — mode1 만 켜고 mode2 로 돌리면 안 열린다
        auto c = make(100);
        c.mode_gait[2] = {true, 1.0f, GL_V1_STANCE_Z, 0.15f, 1, 1.0f, false, 0};
        for (int i=0;i<60;++i) c.update({0.6f,0.f,0.f},2);
        c.update({0.f,0.f,0.f}, 2);
        CHECK(!c.settling, "mode2 까지 정착이 켜졌다 (모드별이 아니다)");
    }
    {   // 첫 스텝에 열리면 안 된다 — mode3 는 base_vel 이 항상 0 이다
        auto c = make(100);
        c.update({0.f,0.f,0.f}, 1);
        CHECK(!c.settling, "시작하자마자 정착이 열렸다");
    }
    if (fails) { printf("[test_settle_stop] %d FAILED\n", fails); return 1; }
    printf("[test_settle_stop] ALL PASS\n");
    return 0;
}
