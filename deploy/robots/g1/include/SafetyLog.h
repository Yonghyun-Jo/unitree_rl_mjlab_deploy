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
// # 🔴 프로세스에 하나뿐인 공유 인스턴스 — `SafetyLog::shared()` 로만 쓴다 (2026-09-22, Ruling 30)
//    이전 설계는 safety_log_ 가 State_Mimic 인스턴스(Mimic_Masked / Mimic_Dance1_subject2 처럼
//    FSM 상태마다 하나씩)의 멤버였고, open_from_env() 는 매번 fopen(path,"w") 로 다시 열었다.
//    Mimic 재진입(예: 넘어짐 -> Passive -> 폴백 -> Mimic 복귀, "fall -> p -> f -> m")마다 그
//    "w" 가 파일을 잘라 **그 재진입을 일으킨 넘어짐의 안전 이벤트 — 유일한 사후 증거 — 를
//    지웠다.** 두 State_Mimic 인스턴스가 서로를 truncate 하는 것도 같은 근원(StateDump.h 가
//    먼저 고친 것과 같은 패턴, Ruling 27). 그래서 그대로 따른다:
//    ① State_Mimic 은 이 객체를 소유하지 않고 `shared()` 를 통해서만 만진다.
//    ② open_from_env() 는 멱등이다 — 이미 열려 있으면(경로가 같든 다르든) 아무것도 안 한다.
//       다른 경로가 요청되면 한 번만 경고하고 원래 파일을 계속 쓴다.
//    ③ stay 끝에서 close() 하지 않는다 — 닫는 것은 프로세스 종료 시 소멸자 하나뿐.
//    ④ event()/sample() 은 매 호출마다 그대로 fflush 한다(발산 직전일 수 있어 버퍼에 남기지
//       않는다 — 이 거동은 이번 변경으로 바뀌지 않는다).
//
// # 켜는 법
//   G1_SAFETY_CSV=<path> ./g1_ctrl ...     (G1_DIAG_CSV 와 같은 관례. 없으면 «완전 비활성»)
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace g1 {

class SafetyLog {
public:
    // 프로세스에 하나뿐인 인스턴스. Mimic_Masked / Mimic_Dance1_subject2 등 State_Mimic 인스턴스가
    // 몇 개든 전부 이 하나를 본다. 함수-지역 static(C++11 매직 스태틱, 스레드-세이프) — 첫 호출에서
    // 생성되고, 프로세스가 정상 종료할 때 `~SafetyLog()` 가 한 번 불려 닫는다.
    static SafetyLog& shared() {
        static SafetyLog inst;
        return inst;
    }

    // 🔴 멱등: 이미 열려 있으면(f_ != nullptr) 아무것도 안 한다 — 같은 경로든 다른 경로든. 여러
    //    State_Mimic 인스턴스가 각자 enter() 에서 이걸 부르지만(FSM 전환·재진입마다), 두 번째
    //    부터는 전부 no-op 이어야 한다: 다시 열면 "w" 가 truncate 해 앞선 stay 의 안전 이벤트가
    //    사라진다(바로 이번에 고치는 결함). 다른 경로가 요청되면(설정 실수 등) 한 번만 stderr 로
    //    경고하고 원래 파일을 계속 쓴다 — 조용히 무시하지 않는다.
    void open_from_env(const char* env_name = "G1_SAFETY_CSV") {
        const char* path = std::getenv(env_name);
        if (!path || !*path) return;                 // 기본은 꺼짐. 켤 때만 비용을 낸다
        if (f_) {                                     // 이미 열려 있다 — 프로세스당 하나뿐
            if (opened_path_ != path && !warned_diff_path_) {
                std::fprintf(stderr,
                    "[safety_log] ⚠ 이미 %s 로 열려 있다 — %s 요청은 무시한다"
                    "(프로세스당 안전 로그 하나)\n", opened_path_.c_str(), path);
                warned_diff_path_ = true;
            }
            return;
        }
        f_ = std::fopen(path, "w");
        if (!f_) return;
        t0_ = wall_now();
        std::fprintf(f_, "wall_time,t,kind,event,joint,joint_name,value,limit,detail\n");
        std::fflush(f_);
        opened_path_ = path;
    }
    bool enabled() const { return f_ != nullptr; }

    // ── 드문 에지: 그 자리에서 한 줄 ──────────────────────────────────
    // bad_orientation / qd_warn / qd_crit / stay_enter. 종단·모드강제·전환 사건이라 빈도가 낮다.
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

    // 🔴 프로덕션에서는 ~SafetyLog() 를 통해 «프로세스 종료 시 한 번» 만 불린다(shared() 의
    //    함수-지역 static 소멸자). open_from_env() 는 멱등이라 stay/FSM 전환마다 다시 열 필요가
    //    없고, 그래서 stay 끝에서 이걸 부르지 않는다 — StateDump.h 와 같은 이유(Ruling 27/30).
    //    close() 자체는 여러 번 불러도 안전(멱등)하고, 한 번도 안 열렸어도 안전하다.
    void close() { if (f_) { std::fflush(f_); std::fclose(f_); f_ = nullptr; } }
    ~SafetyLog() { close(); }

private:
    std::FILE* f_ = nullptr;
    double last_ = 0.0, t0_ = 0.0;
    uint32_t prev_clamp_ = 0, prev_rate_ = 0;
    // 지금 열려 있는(또는 마지막으로 열렸던) 경로. "다른 경로가 요청됐다" 경고 메시지에 쓴다.
    std::string opened_path_;
    bool warned_diff_path_ = false;   // 다른-경로 요청 경고는 프로세스 수명 동안 한 번만
};

}  // namespace g1
