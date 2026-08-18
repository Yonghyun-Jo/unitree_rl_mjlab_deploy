// test_loop_diag.cpp — LoopDiag.h 계약. 계측이 «제어를 방해하지 않고» 사실을 말하는지만 본다.
//   cd deploy/robots/g1/tests && g++ -std=c++17 -I../include -O2 test_loop_diag.cpp -o /tmp/tld && /tmp/tld
#include "LoopDiag.h"
#include <cstdio>
#include <cstring>
#include <chrono>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}
static const char* SEG[6] = {"poll","safe","ctrl","obs","ort","act"};

int main() {
    // ── 예산을 넘긴 틱만 overrun 으로 센다 (같으면 안 넘긴 것)
    {
        LoopDiag d(20.0, 1.0);
        d.tick(19.9, 20.0); d.tick(20.0, 20.0); d.tick(20.1, 20.0); d.tick(50.0, 20.0);
        chk(d.overrun() == 2, "overrun 이 «넘긴 것만» 세지 않는다");
        chk(d.ticks() == 4, "tick 수");
    }
    // ── work 와 period 는 «다른 것»이다: 계산은 가벼운데 루프가 밀리는 경우를 구분해야 한다
    {
        LoopDiag d(20.0, 0.05);
        for (int i = 0; i < 10; ++i) d.tick(2.0, 40.0);     // 일은 2 ms, 간격은 40 ms
        chk(d.overrun() == 0, "일이 예산 안인데 overrun 으로 셌다");
        char buf[512]; d.format(buf, sizeof buf, SEG);
        chk(std::strstr(buf, "25.0Hz") != nullptr, "period 40ms 를 25Hz 로 못 읽는다");
        chk(std::strstr(buf, "period p95=40.0") != nullptr, "period 를 안 보고한다");
    }
    // ── 분위수 = nearest rank (numpy method="inverted_cdf"). 1..100 이면 p50=50, p95=95.
    //    파이썬 브릿지의 [diag] 와 나란히 읽어야 하므로 규칙이 갈리면 안 된다.
    {
        LoopDiag d(20.0, 1.0);
        for (int i = 1; i <= 100; ++i) d.tick((double)i, 20.0);
        char buf[512]; d.format(buf, sizeof buf, SEG);
        chk(std::strstr(buf, "p50=50.0") != nullptr, "p50");
        chk(std::strstr(buf, "p95=95.0") != nullptr, "p95");
        chk(std::strstr(buf, "max=100.0") != nullptr, "max");
    }
    // ── 창은 «표본이 있을 때만» 닫힌다 (빈 창을 찍으면 정렬이 빈 배열을 본다)
    {
        LoopDiag d(20.0, 1.0);
        chk(!d.window_closed(), "표본 0 인데 창이 닫혔다");
        for (int i = 0; i < 49; ++i) d.tick(1.0, 20.0);
        chk(!d.window_closed(), "0.98s 인데 1s 창이 닫혔다");
        d.tick(1.0, 20.0);
        chk(d.window_closed(), "1.00s 인데 창이 안 닫힌다");
        d.reset();
        chk(!d.window_closed() && d.ticks() == 0, "reset 이 안 지운다");
    }
    // ── 구간 평균은 «틱당» 이다 (창 합계가 아니다 — 예산과 나란히 못 읽는다)
    {
        LoopDiag d(20.0, 0.05);
        for (int i = 0; i < 10; ++i) { d.tick(5.0, 20.0); d.seg(4, 0.4); }
        char buf[512]; d.format(buf, sizeof buf, SEG);
        chk(std::strstr(buf, "ort=0.40") != nullptr, "구간 평균이 틱당이 아니다");
    }
    // ── CSV 헤더와 열 수가 맞아야 한다 (어긋나면 나중 분석이 조용히 밀린다)
    {
        LoopDiag d(20.0, 1.0);
        d.tick(3.0, 20.0);
        char buf[512]; d.format_csv(buf, sizeof buf);
        auto cols = [](const char* s){ int c = 1; for (; *s; ++s) if (*s == ',') ++c; return c; };
        chk(cols(buf) == cols(LoopDiag::csv_header()), "CSV 열 수 != 헤더 열 수");
    }
    // ── 🔴 hot path 비용: tick+seg 가 제어에 영향을 주면 계측이 관측을 바꾼다
    {
        LoopDiag d(20.0, 3600.0);
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100000; ++i) { d.tick(1.0, 20.0); d.seg(i % 6, 0.1); }
        double ns = std::chrono::duration<double, std::nano>(
                        std::chrono::high_resolution_clock::now() - t0).count() / 100000.0;
        std::printf("hot path %.1f ns/tick (예산 20,000,000 ns)\n", ns);
        chk(ns < 1000.0, "tick+seg 가 1 µs 를 넘는다 — 계측이 관측을 바꾼다");
    }

    std::printf(fail ? "%d FAILED\n" : "all passed\n", fail);
    return fail ? 1 : 0;
}
