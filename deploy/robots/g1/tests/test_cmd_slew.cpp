// test_cmd_slew.cpp — base_vel 명령 필터(CmdSlew.h)의 «채택된 성질» 을 고정한다.
// 이 값·성질이 바뀌면 실기 거동이 바뀐다(2026-10-07 EMA → slew 1.5 채택). 바꾸려면 sim2sim A/B 를 다시 하고
// 이 테스트·CmdSlew.h·노트(261007_cmd_filter_slew_ab)를 같은 커밋에서 고친다.
#include "CmdSlew.h"
#include <cmath>
#include <cstdio>

using namespace g1::cmd_slew;
static int fails = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fails; }
}

int main() {
    constexpr float DT = 0.02f;

    // 1) 채택값 고정
    chk(RATE_V == 1.5f, "RATE_V == 1.5 (채택값)");
    chk(RATE_W == 1.5f, "RATE_W == 1.5 (채택값)");

    // 2) 정지: 1.0 → 0 이 ceil(1.0/(1.5·0.02)) = 34 틱에 «정확히» 0 (EMA 처럼 꼬리가 남지 않는다)
    {
        State s; s.x = {1.0f, 0.f, 0.f};
        int n = 0; float max_step = 0.f; bool overshoot = false;
        while (s.x[0] != 0.0f && n < 1000) {
            const float prev = s.x[0];
            step(s, {0.f, 0.f, 0.f}, DT); ++n;
            max_step = std::max(max_step, std::abs(s.x[0] - prev));
            if (s.x[0] < 0.f) overshoot = true;
        }
        chk(s.x[0] == 0.0f && s.x[1] == 0.0f && s.x[2] == 0.0f, "정지 명령이 정확히 0 에 도착");
        chk(n == 34, "1.0→0 = 34 틱 (0.68 s)");
        chk(max_step <= RATE_V * DT + 1e-6f, "틱당 변화 ≤ 변화율·dt");
        chk(!overshoot, "0 을 지나치지 않는다");
    }

    // 3) 세 축이 같은 틱에 도착 (학습 램프 lerp 와 같은 직선) — vx 2.0 · wz 1.0 동시 출발
    {
        State s; int n = 0, done_v = -1, done_w = -1;
        const std::array<float, 3> tgt = {2.0f, 0.f, 1.0f};
        while ((done_v < 0 || done_w < 0) && n < 1000) {
            step(s, tgt, DT); ++n;
            if (done_v < 0 && s.x[0] == tgt[0]) done_v = n;
            if (done_w < 0 && s.x[2] == tgt[2]) done_w = n;
            // 도중에도 방향이 유지된다: wz/vx = 0.5
            if (s.x[0] > 1e-6f) chk(std::abs(s.x[2] / s.x[0] - 0.5f) < 1e-4f, "도중 방향 유지 (wz/vx = 0.5)");
        }
        chk(done_v == done_w, "vx·wz 가 같은 틱에 도착");
        chk(done_v == 67, "가장 늦는 축(vx 2.0) 기준 67 틱 (1.34 s)");
    }

    // 4) 작은 변화는 한 틱에 그대로 통과
    {
        State s; step(s, {0.02f, 0.f, 0.f}, DT);
        chk(s.x[0] == 0.02f, "r·dt(0.03) 이하 변화는 즉시 도착");
    }

    if (fails == 0) std::printf("test_cmd_slew: all ok\n");
    return fails ? 1 : 0;
}
