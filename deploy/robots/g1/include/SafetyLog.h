#pragma once
// SafetyLog.h — 안전층이 «언제 무엇에» 걸렸는지 CSV 로 남긴다.  (g1 전용, 🅐 구역)
//
// # 왜 있는가
// 실기 발산은 0.2 초라 눈으로 못 보고, Passive 로 떨어져도 «사유» 는 터미널 spdlog 에만
// 찍혔다가 사라진다. 온보드 g1_logger 의 341컬럼 CSV 는 DDS 의 lowstate/lowcmd 만 보므로
// **결과로 나간 명령**은 알지만 **왜 그 명령이 나왔는지**(안전층이 깎았는지, 정책이 그렇게
// 냈는지)를 구분할 수 없다. 이 파일이 정확히 그 빈칸을 채운다.
//
// # 중복하지 않는다 — 세는 일은 이미 mon_track() 이 하고 있다
// 커밋 8751a0a 가 넣은 mon_clamp_ticks_/mon_clamp_max_/... 가 이미 1 kHz 에서 누적된다.
// 여기서 병렬로 다시 세지 않는다. **그 값을 1 Hz 로 표본화해 시간축에 펴는 것**이 전부다
// (누적값의 기울기가 «언제 걸렸나» 이다). 종료 시 요약 한 줄만 찍던 것을 곡선으로 바꾼다.
//
// # 🔴 RT 제약 — 계측이 위험 요인이 되면 안 된다
// 파일 I/O 는 **50 Hz 정책 스레드에서 1 Hz 로만** 일어난다. 1 kHz 안전 루프는 이 파일을
// 절대 건드리지 않는다(커밋 79c3114 가 고친 병을 되풀이하지 않는다).
// mon_* 를 다른 스레드에서 읽는 것은 32비트 정렬 값의 양성 경쟁이다 — 관측용이라 한 틱
// 어긋나도 무해하고, 그 대가로 1 kHz 쪽에 원자연산을 얹지 않는다.
//
// # 조인
// wall_time 은 g1_logger CSV 와 **같은 시계(system_clock)** 라 시간축으로 그대로 붙는다.
//
// # 켜는 법
//   G1_SAFETY_CSV=<path> ./g1_ctrl ...     (G1_DIAG_CSV 와 같은 관례. 없으면 «완전 비활성»)
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace g1 {

class SafetyLog {
public:
    void open_from_env(const char* env_name = "G1_SAFETY_CSV") {
        const char* path = std::getenv(env_name);
        if (!path || !*path) return;                 // 기본은 꺼짐. 켤 때만 비용을 낸다
        f_ = std::fopen(path, "w");
        if (!f_) return;
        t0_ = wall_now();
        std::fprintf(f_, "wall_time,t,kind,event,joint,joint_name,value,limit,detail\n");
        std::fflush(f_);
    }
    bool enabled() const { return f_ != nullptr; }
    void close() { if (f_) { std::fflush(f_); std::fclose(f_); f_ = nullptr; } }

    // ── 드문 에지: 그 자리에서 한 줄 ──────────────────────────────────
    // bad_orientation / qd_warn / qd_crit. 종단·모드강제 사건이라 빈도가 낮다.
    // t = 이 로그가 열린 뒤 경과초. 바깥 시계에 «결합하지 않는다» — 조인은 wall_time 이 한다.
    void event(const char* ev, int joint, const char* jname,
               float value, float limit, const char* detail = "") {
        if (!f_) return;
        const double t = wall_now() - t0_;
        std::fprintf(f_, "%.6f,%.4f,edge,%s,%d,%s,%.5f,%.5f,%s\n",
                     wall_now(), t, ev, joint, jname ? jname : "", value, limit, detail);
        std::fflush(f_);   // 발산 직전일 수 있다 — 버퍼에 남기지 않는다
    }

    // ── 1 Hz 표본: 정책 스레드(50 Hz)에서 부른다. 누적값이 «움직였을 때만» 기록 ──
    void sample(
                uint32_t clamp_ticks, float clamp_max, int clamp_joint,
                uint32_t rate_ticks,  float rate_max,  int rate_joint,
                float tilt_deg, const char* (*jname)(int)) {
        if (!f_) return;
        const double w = wall_now();
        if (w - last_ < 1.0) return;
        const double t = w - t0_;
        last_ = w;
        if (clamp_ticks != prev_clamp_) {
            std::fprintf(f_, "%.6f,%.4f,rate,pos_clamp,%d,%s,%u,%.5f,ticks_cum;max_rad\n",
                         w, t, clamp_joint, jname && clamp_joint >= 0 ? jname(clamp_joint) : "",
                         clamp_ticks - prev_clamp_, clamp_max);
            prev_clamp_ = clamp_ticks;
        }
        if (rate_ticks != prev_rate_) {
            std::fprintf(f_, "%.6f,%.4f,rate,rate_limit,%d,%s,%u,%.5f,ticks_cum;max_rad\n",
                         w, t, rate_joint, jname && rate_joint >= 0 ? jname(rate_joint) : "",
                         rate_ticks - prev_rate_, rate_max);
            prev_rate_ = rate_ticks;
        }
        // tilt 는 항상 한 줄 — 안 걸려도 «얼마나 여유였나» 가 진단에 필요하다.
        std::fprintf(f_, "%.6f,%.4f,rate,tilt,-1,,%.3f,57.300,deg_max_so_far\n", w, t, tilt_deg);
        std::fflush(f_);
    }

    static double wall_now() {
        using namespace std::chrono;
        return duration<double>(system_clock::now().time_since_epoch()).count();
    }

private:
    std::FILE* f_ = nullptr;
    double last_ = 0.0, t0_ = 0.0;
    uint32_t prev_clamp_ = 0, prev_rate_ = 0;
};

}  // namespace g1
