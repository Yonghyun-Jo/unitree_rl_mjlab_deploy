#pragma once
// LoopDiag.h — 제어 루프의 «속도계». 로봇 동작은 하나도 안 바꾸고 "지금 몇 ms 쓰고 있나"만 본다.
//
// 왜 필요한가: policy_thread 는 `env->step(); sleep_until(sleepTill); sleepTill += dt;` 뿐이라
// 한 바퀴가 실제로 몇 ms 걸렸는지 «아무도 안 재고 있었다». 늦어도 늦은 줄 모르고, 게다가
// sleepTill 은 따라잡기를 안 하므로 한 번 밀리면 누적된다. 파이썬 브릿지(vr_teleop_bridge)에는
// 같은 계측이 이미 있는데 C++ 쪽에만 없었다.
//
// 무엇에 쓰나: 새 계산(전환 생성기 등)을 얹기 «전에» 남은 예산을 안다.
//   20 ms 중 지금 몇 ms 를 쓰는가 -> 얼마를 더 얹을 수 있는가.
//
// 설계 제약 (제어 루프 안에서 도는 코드다):
//   - hot path 에 «할당도 I/O 도 없다». 고정 배열에 숫자 하나 넣는 게 전부.
//   - 정렬·출력은 창(기본 1 s)이 닫힐 때 한 번. 50 Hz 면 50개 정렬 = 수 µs.
//   - 순수 C++ (Eigen·spdlog 없음) — 단위테스트가 stand-alone 으로 돈다.
//
// 두 시간을 따로 본다 — 헷갈리면 진단이 반대로 간다:
//   work   = 한 틱의 «일» 시간.       예산(20 ms) 대비. 이게 넘으면 계산이 무겁다.
//   period = 틱 시작~시작 «간격».      실제 Hz. work 는 짧은데 period 가 길면 계산이 아니라
//            스케줄러/경합 문제다 (우선순위 설정이 없으므로 실제로 일어날 수 있다).
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cmath>

class LoopDiag {
public:
    static constexpr int CAP  = 512;   // 창 표본 상한 (50 Hz · 1 s = 50)
    static constexpr int NSEG = 6;     // 구간 수

    explicit LoopDiag(double budget_ms = 20.0, double window_s = 1.0)
        : budget_ms_(budget_ms), window_s_(window_s) {}

    void reset() {
        n_ = 0; head_ = 0; overrun_ = 0; ticks_ = 0; elapsed_s_ = 0.0;
        work_max_ = period_max_ = 0.0;
        for (int i = 0; i < NSEG; ++i) seg_sum_[i] = seg_max_[i] = 0.0;
    }

    // 매 틱 1회. period_ms 는 첫 틱에서 0 을 주면 표본에서 빠진다.
    //
    // 🔴 표본은 «링버퍼»다 (2026-08-18 수정). 처음엔 CAP 개까지만 담고 나머지를 버렸는데,
    // 1 kHz 스레드는 1 초 창에 1000 틱이라 «창의 뒤쪽 절반을 통째로 못 보는» 편향이 생겼다.
    // max·overrun·ticks 는 표본과 무관하게 «매 틱» 갱신하므로 항상 정확하다 — 분위수만
    // 마지막 CAP 개에 대한 값이다.
    void tick(double work_ms, double period_ms) {
        work_[head_] = work_ms; period_[head_] = period_ms;
        head_ = (head_ + 1) % CAP;
        if (n_ < CAP) ++n_;
        if (work_ms > budget_ms_) ++overrun_;
        if (work_ms > work_max_) work_max_ = work_ms;
        if (period_ms > period_max_) period_max_ = period_ms;
        ++ticks_;
        elapsed_s_ += (period_ms > 0.0 ? period_ms : budget_ms_) * 1e-3;
    }

    void seg(int i, double ms) {
        if (i < 0 || i >= NSEG) return;
        seg_sum_[i] += ms;
        if (ms > seg_max_[i]) seg_max_[i] = ms;
    }

    bool window_closed() const { return elapsed_s_ >= window_s_ && n_ > 0; }

    // 창 요약 두 줄을 buf 에 쓴다. 부른 뒤 reset() 하는 것은 호출자 몫(로그 레벨을 고르게).
    int format(char* buf, int cap, const char* const* seg_names) const {
        double w[CAP], p[CAP];
        const int n = n_;
        for (int i = 0; i < n; ++i) { w[i] = work_[i]; p[i] = period_[i]; }
        std::sort(w, w + n);
        int np = 0;
        for (int i = 0; i < n; ++i) if (period_[i] > 0.0) p[np++] = period_[i];
        std::sort(p, p + np);
        const double hz = (elapsed_s_ > 0.0) ? ticks_ / elapsed_s_ : 0.0;
        // max 는 정렬 배열이 아니라 «매 틱 갱신한 값」을 쓴다 — 링버퍼는 마지막 CAP 개만 갖는다.
        int k = std::snprintf(buf, cap,
            "[diag] %.1fHz  work p50=%.1f p95=%.1f max=%.1f ms (예산 %.0f)  overrun=%d/%d"
            "  period p95=%.1f max=%.1f ms",
            hz, q(w, n, 0.50), q(w, n, 0.95), work_max_, budget_ms_, overrun_, ticks_,
            np ? q(p, np, 0.95) : 0.0, period_max_);
        if (k < 0 || k >= cap) return k;
        k += std::snprintf(buf + k, cap - k, "\n       ");
        for (int i = 0; i < NSEG && k < cap; ++i)
            k += std::snprintf(buf + k, cap - k, "%s=%.2f(max %.2f) ",
                               seg_names[i], seg_sum_[i] / (ticks_ ? ticks_ : 1), seg_max_[i]);
        return k;
    }

    // CSV 한 줄 = 창 하나. «틱마다» 파일을 쓰지 않는 이유: 제어 루프 안의 I/O 는 그 자체가
    // 지터원이다. 스파이크는 max/p95 가 잡는다.
    int format_csv(char* buf, int cap) const {
        double w[CAP];
        const int n = n_;
        for (int i = 0; i < n; ++i) w[i] = work_[i];
        std::sort(w, w + n);
        int k = std::snprintf(buf, cap, "%.3f,%d,%d,%.4f,%.4f,%.4f",
                              elapsed_s_, ticks_, overrun_,
                              q(w, n, 0.50), q(w, n, 0.95), work_max_);
        for (int i = 0; i < NSEG && k < cap; ++i)
            k += std::snprintf(buf + k, cap - k, ",%.4f", seg_sum_[i] / (ticks_ ? ticks_ : 1));
        return k;
    }
    static const char* csv_header() {
        return "window_s,ticks,overrun,work_p50_ms,work_p95_ms,work_max_ms,"
               "poll_ms,safe_ms,ctrl_ms,obs_ms,ort_ms,act_ms";
    }

    int overrun() const { return overrun_; }
    int ticks() const { return ticks_; }
    double work_max() const { return work_max_; }

    // ── 🔴 실시간 스레드는 «절대 여기서 로그를 찍지 않는다» ──────────────────────────
    // spdlog 의 기본 sink 는 동기(stdout)다. 1 kHz 안전 루프에서 부르면 파이프/터미널이
    // 느린 순간 그 한 번이 «밀리초 단위로» 루프를 잡아먹는다 — 계측이 관측을 바꾸는 정도가
    // 아니라 계측이 «위험 요인»이 된다. 그래서 소유 스레드는 문자열만 만들어 슬롯에 넣고,
    // 로깅은 낮은 우선순위의 보고 스레드가 대신 한다 (SPSC: 생산자 1 · 소비자 1).
    void publish(const char* const* seg_names) {
        if (ready_.load(std::memory_order_acquire)) return;   // 아직 안 가져갔다 -> 이번 창은 건너뜀
        format(slot_, sizeof slot_, seg_names);
        ready_.store(true, std::memory_order_release);
    }
    bool take(char* out, int cap) {
        if (!ready_.load(std::memory_order_acquire)) return false;
        std::snprintf(out, cap, "%s", slot_);
        ready_.store(false, std::memory_order_release);
        return true;
    }

private:
    // 정렬된 배열의 분위수 — «nearest rank» (numpy 의 method="inverted_cdf" 와 같다):
    //   i = ceil(frac * n) - 1
    // 선형보간을 안 쓰는 이유: 표본이 50개(1 s @ 50 Hz)라 보간이 의미가 없고, 보간·반올림
    // 규칙이 도구마다 갈리면 파이썬 브릿지의 [diag] 와 «나란히 못 놓는다».
    static double q(const double* sorted, int n, double frac) {
        if (n <= 0) return 0.0;
        int i = (int)std::ceil(frac * n) - 1;
        if (i < 0) i = 0;
        if (i > n - 1) i = n - 1;
        return sorted[i];
    }

    double budget_ms_, window_s_;
    double work_[CAP] = {0}, period_[CAP] = {0};
    double seg_sum_[NSEG] = {0}, seg_max_[NSEG] = {0};
    int n_ = 0, head_ = 0, overrun_ = 0, ticks_ = 0;
    double work_max_ = 0.0, period_max_ = 0.0;
    double elapsed_s_ = 0.0;
    char slot_[768] = {0};
    std::atomic<bool> ready_{false};
};
