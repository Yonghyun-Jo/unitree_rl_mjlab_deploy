#pragma once
// StateDump.h — «측정 q / 명령 q_des / 게인 / IMU» 를 로봇 온보드 로거와 «같은 열 이름» 으로
//               CSV 에 남긴다. sim2sim ↔ 실기를 같은 분석 스크립트로 대조하기 위한 계측이다.
//               (g1 전용, 🅐 구역)
//
// 🔴 기본 꺼짐. 환경변수 G1_STATE_CSV 가 있을 때만 파일을 연다 — 없으면 비용 0(포인터 검사 1회).
//     $ G1_STATE_CSV=/tmp/sim_state.csv ./g1_ctrl --network=lo
//
// 🔴 run() 은 1 kHz FSM 스레드다 — **여기서 파일을 만지지 않는다.**
//    예전 구조는 그 스레드에서 fprintf 했다. stdio 버퍼가 차는 순간 같은 스레드에서 write()
//    가 터지고, eMMC 에서 그건 수십 ms 다. 걷는 중이면 그만큼 제어가 멈춘다(그래서 주석이
//    «실기에서 켜지 말 것» 이었고, 결국 실기 로그에 gait 계측이 한 번도 안 남았다).
//
//    지금은 갈랐다:
//      RT 스레드 : fmemopen 으로 «메모리에» 한 줄 찍고 링에 memcpy 하고 인덱스만 올린다.
//                 syscall 0 · malloc 0 · 락 0. 링이 차면 **막지 않고 버린다**(버린 수를 센다).
//      쓰기 스레드: 100 ms 마다 링을 비워 파일에 쓴다. 여기서 블록해도 제어는 안 멈춘다.
//    ⇒ 실기에서 켜도 된다. 상시 로깅은 여전히 온보드 piene_g1_logger 의 역할이다.
//
//    ⚠ SIGKILL 로 죽이면 링에 남은 최대 100 ms 가 사라진다(정상 종료·Ctrl-C 는 close() 가 비운다).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <atomic>
#include <thread>
#include <memory>
#include <initializer_list>

namespace g1 {

class StateDump {
public:
    // extra_header = 뒤에 붙일 열 이름(선행 콤마 포함). 소유 모듈의 `Aux::header()` 를 그대로
    // 넘긴다 — StateDump 는 그 열이 무엇인지 «모른다». 그래서 필드가 늘어도 여기는 안 바뀐다.
    void open_from_env(const char* env_name = "G1_STATE_CSV", const char* extra_header = nullptr) {
        const char* path = std::getenv(env_name);
        if (!path || !*path) return;
        f_ = std::fopen(path, "w");
        if (!f_) return;
        // 🔴 stdio 가 «한 줄 도중에» 자동 flush 하면, 프로세스가 갑자기 죽었을 때 파일 끝이
        //    반쪽 줄로 남는다(실측: Ctrl-C 로 261열 중 108열짜리 꼬리). 분석기가 거기서 깨진다.
        //    버퍼를 크게 잡고 drain() 이 «줄 경계에서만» flush 하게 해서 그 창을 없앤다.
        std::setvbuf(f_, nullptr, _IOFBF, kFlushRows * kRowBytes);
        t0_ = now();
        // 온보드 로거(piene_g1_logger) 의 341열 중 분석에 쓰는 열만, «이름을 그대로» 쓴다.
        std::fprintf(f_, "time,wall_time,quat_w,quat_x,quat_y,quat_z,"
                         "ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z,"
                         "rpy_r,rpy_p,rpy_y");
        for (const char* k : {"q","dq","tau_est","q_des","dq_des","kp","kd","tau_ff"})
            for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%s_%d", k, i);
        // ▼ 온보드 로거에는 «없는» 열. 뒤에 붙인다 → 앞 341열의 위치가 안 바뀌어
        //   실기 로그 분석 스크립트가 그대로 돈다.
        if (extra_header) std::fputs(extra_header, f_);
        std::fprintf(f_, "\n");
        std::fflush(f_);

        // 링과 «한 줄 짜리 메모리 FILE» 을 미리 잡는다 — RT 경로에서 할당이 없게.
        ring_.reset(new char[size_t(kRows) * kRowBytes]);
        scratch_.reset(new char[kRowBytes]);
        mem_ = fmemopen(scratch_.get(), kRowBytes, "w");
        if (!mem_) { std::fclose(f_); f_ = nullptr; return; }   // 조용히 꺼진다(거동 무변화)
        run_.store(true, std::memory_order_release);
        th_ = std::thread([this] { drain_loop(); });
    }

    void close() {
        if (run_.exchange(false, std::memory_order_acq_rel) && th_.joinable()) th_.join();
        if (mem_) { std::fclose(mem_); mem_ = nullptr; }
        if (f_) {
            drain();                                   // 링에 남은 것을 마저 비운다
            const unsigned long long d = dropped_.load(std::memory_order_relaxed);
            if (d) std::fprintf(stderr,
                "[state_dump] \u26a0 %llu 줄을 버렸다 (링이 찼다 — 디스크가 느리거나 링이 작다)\n", d);
            std::fflush(f_); std::fclose(f_); f_ = nullptr;
        }
    }
    bool on() const { return f_ != nullptr; }

    // LowState(측정) + LowCmd(명령) 한 쌍을 한 줄로. 20틱마다 = 50Hz.
    // aux 는 `void write(FILE*) const` 만 있으면 무엇이든 된다 — 열 이름과 값이 그쪽 한 곳에
    // 같이 있어서 «헤더만 고치고 값을 빠뜨리는» 종류의 어긋남이 구조적으로 안 생긴다.
    template <class LowStateMsg, class LowCmdMsg, class Aux>
    void tick(const LowStateMsg& st, const LowCmdMsg& cmd, const Aux& aux) {
        if (!f_) return;
        if (++n_ % 20) return;
        // 🔴 여기부터 끝까지 파일 접근이 없다. mem_ 은 fmemopen 이라 전부 메모리다.
        std::rewind(mem_);
        const auto& im = st.imu_state();
        const auto& ms = st.motor_state();
        const auto& mc = cmd.motor_cmd();
        std::fprintf(mem_, "%.4f,%.6f", now() - t0_, now());
        for (int i = 0; i < 4; ++i) std::fprintf(mem_, ",%.6f", im.quaternion()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(mem_, ",%.6f", im.gyroscope()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(mem_, ",%.6f", im.accelerometer()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(mem_, ",%.6f", im.rpy()[i]);
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", ms[i].q());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", ms[i].dq());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", ms[i].tau_est());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", mc[i].q());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", mc[i].dq());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", mc[i].kp());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", mc[i].kd());
        for (int i = 0; i < 29; ++i) std::fprintf(mem_, ",%.6f", mc[i].tau());
        aux.write(mem_);
        std::fputc('\n', mem_);
        std::fflush(mem_);                             // 메모리 flush — syscall 아니다
        const long n = std::ftell(mem_);
        if (n <= 0 || n >= kRowBytes) { dropped_.fetch_add(1, std::memory_order_relaxed); return; }

        const unsigned long long h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= (unsigned long long)kRows) {
            dropped_.fetch_add(1, std::memory_order_relaxed);   // 막지 않는다 — 제어가 우선
            return;
        }
        char* row = ring_.get() + (h % kRows) * kRowBytes;
        std::memcpy(row, scratch_.get(), size_t(n));
        len_[h % kRows] = int(n);
        head_.store(h + 1, std::memory_order_release);          // 여기서 쓰기 스레드에 보인다
    }
    ~StateDump() { close(); }

private:
    // 링을 비운다. 🔴 **쓰기 스레드에서만** 부른다(close 의 마지막 한 번 포함 — 그땐 조인 뒤다).
    void drain() {
        unsigned long long t = tail_.load(std::memory_order_relaxed);
        const unsigned long long h = head_.load(std::memory_order_acquire);
        unsigned long long since_flush = 0;
        for (; t < h; ++t) {
            const size_t i = size_t(t % kRows);
            std::fwrite(ring_.get() + i * kRowBytes, 1, size_t(len_[i]), f_);
            // 버퍼가 차서 stdio 가 알아서 flush 하기 «전에» 우리가 줄 경계에서 비운다.
            if (++since_flush >= kFlushRows / 2) { std::fflush(f_); since_flush = 0; }
        }
        if (since_flush) std::fflush(f_);
        tail_.store(t, std::memory_order_release);
    }
    void drain_loop() {
        while (run_.load(std::memory_order_acquire)) {
            drain();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        drain();
    }

    static double now() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // 50 Hz 기준 링 4096 줄 = 81 초. 쓰기 스레드가 100 ms 마다 비우므로 평시엔 몇 줄만 찬다.
    // 디스크가 초 단위로 멈춰도 버티라고 이만큼 잡았다(4096 x 4 KB = 16 MB).
    static constexpr int kRows = 4096;
    static constexpr int kRowBytes = 4096;
    static constexpr int kFlushRows = 256;   // stdio 버퍼 = 이만큼(1 MB). 그 절반마다 비운다.

    std::FILE* f_ = nullptr;         // 실제 파일 — 쓰기 스레드만 만진다
    std::FILE* mem_ = nullptr;       // 한 줄짜리 메모리 FILE — RT 스레드만 만진다
    std::unique_ptr<char[]> ring_, scratch_;
    int len_[kRows] = {0};
    std::atomic<unsigned long long> head_{0}, tail_{0}, dropped_{0};
    std::atomic<bool> run_{false};
    std::thread th_;
    double t0_ = 0.0;
    unsigned n_ = 0;
};

}  // namespace g1
