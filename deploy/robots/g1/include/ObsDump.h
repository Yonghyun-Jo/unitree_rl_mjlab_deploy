#pragma once
// ObsDump.h — 정책 틱마다 «g1_ctrl 이 실제로 쓴 값» 을 CSV 한 줄로 남긴다: ONNX 에 넣은 obs 벡터 전체 · ONNX 원출력 ·
//             run() 이 lowcmd 에 실은 q_des·kp·kd · 그 틱의 명령(키·속도·모드·자세) · 그 obs 를 만든 상태(q·dq·IMU).
//             g1_arcade core 의 관측 접착부를 «실행으로» 대조하는 원장(P1)이다. (g1 전용, 🅐 구역)
//
// 🔴 기본 꺼짐. 환경변수 G1_OBS_CSV 가 있을 때만 연다 — 없으면 비용 0(enter() 의 getenv 1회 · 틱마다 bool 검사).
//     $ G1_OBS_CSV=/path/obs.csv ./g1_ctrl --network=lo
//
// 🔴 기록이지 재계산이 아니다(memory instrumentation-must-record-not-recompute). 칸마다 원본이 그 값을 쓰거나 읽은
//    «그 자리» 에서 복사한다 — obs·원출력 = ONNX 입력 직전(Algorithms 감싸개, State_Mimic.cpp ObsDumpAlg),
//    q_des = run() 이 lowcmd 에 실은 값, 명령·모드 = 정책 루프가 그 값을 만든 직후.
//
// 구조는 StateDump.h 와 같다: 프로세스 공유 shared() · 멱등 open · 링 + 쓰기 스레드 · 차면 막지 않고 버리고 센다 ·
// 닫기는 소멸자 하나(⚠ SIGINT/SIGTERM 은 main.cpp 가 기본 처분을 다시 일으켜 소멸자가 안 돈다 — 링에 남은 ≤100 ms 를
// 잃는다. StateDump 와 같다). 다른 점 하나: 한 줄이 3000 칸이 넘는다(StateDump 의 13 배) — 그래서 RT 쪽에서 글자로
// 찍지 않는다. 정책 스레드는 이진 기록을 링에 memcpy 만 하고, 글자 변환(snprintf)은 쓰기 스레드가 한다.
// RT 쪽(정책 스레드 · run()) = syscall 0 · malloc 0 · 락 0.
//
// 스레드 셋:
//   정책 스레드(50 Hz, 생산자 하나): begin_row() 로 줄을 열고 stage()·stage_obs() 를 채우고 publish_step()
//     (env->step() 직후 — «이 줄의 action 이 이제 processed_actions 에 있다»). 다음 begin_row() 가 그 줄을 run() 의
//     기록과 합쳐 링에 넣는다 — 다음 틱 첫머리는 sleep 뒤라 run() 이 그 action 을 이미 lowcmd 에 실었다.
//   run()(1 kHz): step_seq() 를 processed_actions() «전에» 읽고, lowcmd 를 다 실은 «뒤» record_out(). 시퀀스 잠금
//     (seqlock, 칸은 원자 변수)이라 정책 스레드가 반쯤 쓰인 기록을 읽지 않는다. 기록마다 «어느 줄의 action 이었나»
//     (from)를 단다 — 받는 줄 번호와 다르면 out_from 이 그 차이를 그대로 보인다(숨기지 않는다).
//   쓰기 스레드: 100 ms 마다 링을 비워 글자로 파일에 쓴다. 블록해도 제어는 안 멈춘다.
//     🔴 첫 줄에서 자기를 SCHED_OTHER nice +10 으로 내린다 — enter() 를 부르는 FSM 스레드는 SCHED_FIFO 40
//     (CtrlFSM.h run_)이고 std::thread 는 만든 스레드의 정책을 물려받는다(glibc PTHREAD_INHERIT_SCHED). 안 내리면 글자
//     변환(줄당 ~3300 칸 snprintf)이 RT 우선순위로 1 kHz 안전 루프·50 Hz 정책과 CPU 를 다툰다. 관례 = RtPriority.h
//     lower_this_thread(CtrlFSM.h diag 보고 스레드) — 같은 두 호출을 여기 직접 쓴다(그 헤더는 spdlog 를 끌어와 이 헤더만
//     쓰는 단위 시험·로봇 게이트 컴파일에 링크 플래그가 필요해진다).
//
// 🔴 프로세스에 하나 — `ObsDump::shared()` 로만 쓴다(StateDump 와 같은 이유: State_Mimic 은 FSM 상태마다 하나씩
//    만들어진다). obs 배치가 다른 인스턴스(예 Mimic_Dance1_subject2)가 열려 있는 파일에 붙으려 하면 경고 1회 뒤
//    false — 그 인스턴스는 기록하지 않는다(열 이름이 첫 open 의 것이라 섞으면 칸이 어긋난다).
//
// 열 = ObsDumpRow::fields() 하나가 이름과 값을 같이 낸다 — 헤더만 고치고 값을 빠뜨리는 어긋남이 구조적으로 없다.
// 그 뒤에 obs 칸(`<ONNX 입력 이름>_<i>`, 그룹은 Meta.groups 순서). 실수는 %.9g(float32 왕복 정확), 시각은 %.6f.
// 기록이 없는 칸은 float = nan · int = −1 (0 과 구별한다 — 0 은 진짜 값일 수 있다).
#include <algorithm>
#include <cerrno>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace g1 {

// 한 줄(이진). 정책 스레드가 채우고 쓰기 스레드가 글자로 바꾼다. 관절 순서 = 정책 순서(joint_ids_map 적용 뒤).
struct ObsDumpRow {
    static constexpr int NJ = 29;
    // ── 줄 정체
    int64_t row = -1;       // 줄 번호(프로세스 전체, 1 부터)
    int32_t kind = -1;      // 0 = 체류 시작(정책 스레드 첫 env->reset() 직후 — obs·raw 없음, q_des = 첫 추론 전에 나가던 값) · 1 = 정책 틱
    int32_t stay = -1;      // 체류 번호(enter() 마다 +1, 1 부터)
    int64_t episode = -1;   // env->episode_length (틱 줄 = env->step() 뒤)
    double  t = 0;          // 열린 뒤 초(steady) — 틱 줄 = env->step() 이 끝난 시각, 체류 줄 = env->reset() 뒤
    double  t_enter = 0;    // 체류 줄만: State_Mimic::enter() 첫머리(덤프를 열기 «전») 시각. 첫 체류는 음수(파일이 열리기 전). 틱 줄 = nan
    double  t_gain = 0;     // 체류 줄만: 덤프 열기·감싸기를 마치고 게인을 싣기 직전 시각(t_gain − t_enter = 열기 비용). 틱 줄 = nan
    // ── 명령 (정책 루프가 이 틱에 쓴 것)
    int32_t key = -1;       // g_poll_inputs 가 이 틱에 먹은 키(글자 코드, 없으면 0, 한 글자가 아니면 −바이트 수)
    int32_t mode = -1;      // 전환 처리 뒤 g_mode.mode()
    int32_t mode_req = -1;  // 전환 처리 뒤 g_mode.requested()
    int32_t switched = -1;  // 이 틱 전환 처리: 0 없음 · 1 모드 · 2 클립 · 3 둘 다
    int32_t clip = -1;      // g_mode.clip_id()
    int32_t preset = -1;    // g_m5.preset() (mode5 명령을 만든 뒤)
    int32_t m5_hold = -1;   // g_m5.holding()
    int32_t qd_warn = -1;   // js_qd_warn_latched_ (qd 가드가 모드를 덮었나)
    int32_t vr = -1;        // motion->vr_override
    int32_t gui_seq = -1;   // g_gui_last_seq — 체류 중 바뀌면 GUI 가 끼어든 것
    float kb[3];            // g_kb_vx·vy·wz (g_poll_inputs 뒤)
    float bv[3];            // g_joystick_base_vel() — 키보드+조이스틱 합을 모드 봉투로 자른 명령
    float bvf[3];           // slew 뒤(bv_f.x) = g_loco.update 입력
    float base_vel[3];      // g_loco.base_vel (update 뒤 — base_vel_command 관측의 원천)
    // ── 루프 첫머리 robot->update() (첫 읽기) — z_fk · 기울기 · mode5 도착 판정이 본 상태
    float z_fk, tilt, tilt_raw;
    float g1[3];            // 그때 projected_gravity_b (Mode5Driver::tick 이 받은 g)
    float quat1[4];         // 그때 root_quat_w (w,x,y,z)
    float q1[NJ];           // 그때 joint_pos
    float init_quat[4];     // 전환 처리 뒤 init_quat (w,x,y,z) — 클립 재앵커 결과
    // ── obs 를 만든 상태 (env->step() 안 robot->update(), 체류 줄 = env->reset() 안)
    float q[NJ], dq[NJ];
    float quat[4];          // root_quat_w (w,x,y,z)
    float gyro[3];          // root_ang_vel_b
    float pg[3];            // projected_gravity_b
    // ── 출력
    float raw[NJ];          // ONNX 원출력(감싸개가 act() 반환값을 복사)
    float proc[NJ];         // run() 이 읽은 processed_actions (블렌드·clamp 전)
    float q_des[NJ];        // run() 이 lowcmd 에 실은 q
    float kp[NJ], kd[NJ];   // 그때 lowcmd 의 kp·kd
    int64_t out_from = -1;  // 위 네 칸을 실은 run() 이 읽은 step_seq — 이 줄 번호면 «이 줄의 action», −2 = run() 기록 없음

    // 이름·값을 한 곳에서. v(이름, 포인터, 개수) — 개수 1 이면 이름 그대로, 아니면 이름_0 ..
    template <class V> void fields(V& v) const {
        v("row", &row, 1);           v("kind", &kind, 1);         v("stay", &stay, 1);
        v("episode", &episode, 1);   v("t", &t, 1);               v("t_enter", &t_enter, 1);
        v("t_gain", &t_gain, 1);
        v("key", &key, 1);           v("mode", &mode, 1);         v("mode_req", &mode_req, 1);
        v("switched", &switched, 1); v("clip", &clip, 1);         v("preset", &preset, 1);
        v("m5_hold", &m5_hold, 1);   v("qd_warn", &qd_warn, 1);   v("vr", &vr, 1);
        v("gui_seq", &gui_seq, 1);
        v("kb", kb, 3);              v("bv", bv, 3);              v("bvf", bvf, 3);
        v("base_vel", base_vel, 3);
        v("z_fk", &z_fk, 1);         v("tilt", &tilt, 1);         v("tilt_raw", &tilt_raw, 1);
        v("g1", g1, 3);              v("quat1", quat1, 4);        v("q1", q1, NJ);
        v("init_quat", init_quat, 4);
        v("q", q, NJ);               v("dq", dq, NJ);             v("quat", quat, 4);
        v("gyro", gyro, 3);          v("pg", pg, 3);
        v("raw", raw, NJ);           v("proc", proc, NJ);         v("q_des", q_des, NJ);
        v("kp", kp, NJ);             v("kd", kd, NJ);             v("out_from", &out_from, 1);
    }
    // 기록 전 상태 = float nan · int −1 (row·kind·stay 는 begin_row 가 채운다)
    void clear() {
        Clear c;
        const_cast<const ObsDumpRow*>(this)->fields(c);
    }

private:
    struct Clear {
        void operator()(const char*, const int64_t* p, int n) const { for (int i = 0; i < n; ++i) const_cast<int64_t*>(p)[i] = -1; }
        void operator()(const char*, const int32_t* p, int n) const { for (int i = 0; i < n; ++i) const_cast<int32_t*>(p)[i] = -1; }
        void operator()(const char*, const double* p, int n) const {
            for (int i = 0; i < n; ++i) const_cast<double*>(p)[i] = std::numeric_limits<double>::quiet_NaN(); }
        void operator()(const char*, const float* p, int n) const {
            for (int i = 0; i < n; ++i) const_cast<float*>(p)[i] = std::numeric_limits<float>::quiet_NaN(); }
    };
};

class ObsDump {
public:
    static constexpr int NJ = ObsDumpRow::NJ;

    // 파일 머리(# 줄)에 싣는 정체. groups = ONNX 입력 이름 · 폭 (obs 칸의 순서).
    struct Meta {
        std::string slot, state, network, obs_contract, obs_contract_version, obs_terms, commit;
        int dirty = -1;   // 빌드할 때 git 작업 트리가 깨끗했나: 0 깨끗 · 1 고친 파일 있음 · −1 모름
        std::vector<std::pair<std::string, int>> groups;
    };

    static ObsDump& shared() {
        static ObsDump inst;
        return inst;
    }

    // 🔴 멱등: 이미 열려 있으면 아무것도 안 하고 «이 호출의 obs 배치가 열린 파일과 같은가» 만 돌려준다
    //    (다르면 경고 1회 뒤 false). 처음 여는 호출은 파일 · 링 · 쓰기 스레드를 만든다(여기서만 할당).
    bool open_from_env(const char* env_name, const Meta& m) {
        const char* path = std::getenv(env_name);
        if (!path || !*path) return false;
        if (f_) {
            if (m.groups != meta_.groups) {
                if (!warned_layout_) {
                    std::fprintf(stderr, "[obs_dump] ⚠ 이미 %s 로 열려 있다(슬롯 %s) — obs 배치가 다른 '%s'(%s) 는 "
                                         "기록하지 않는다(프로세스당 obs 덤프 하나)\n",
                                 opened_path_.c_str(), meta_.slot.c_str(), m.state.c_str(), m.slot.c_str());
                    warned_layout_ = true;
                }
                return false;
            }
            if (opened_path_ != path && !warned_path_) {
                std::fprintf(stderr, "[obs_dump] ⚠ 이미 %s 로 열려 있다 — %s 요청은 무시한다\n", opened_path_.c_str(), path);
                warned_path_ = true;
            }
            return true;
        }
        int n = 0;
        for (const auto& g : m.groups) n += std::max(0, g.second);
        f_ = std::fopen(path, "w");
        if (!f_) return false;
        meta_ = m;
        n_obs_ = n;
        row_bytes_ = (sizeof(ObsDumpRow) + size_t(n_obs_) * sizeof(float) + 7) & ~size_t(7);
        std::setvbuf(f_, nullptr, _IOFBF, kFileBuf);
        t0_ = now();
        write_header_();
        opened_path_ = path;
        // 링 · 줄 버퍼 · 정책 스레드 기록칸을 «여기서» 잡는다 — RT 경로에서 할당이 없게.
        ring_.reset(new unsigned char[size_t(kRows) * row_bytes_]);
        // 🔴 미리 만진다 — 새로 잡은 큰 메모리는 쪽(page)이 처음 쓰일 때 커널이 채운다(minor fault). 안 만지면 정책 스레드의
        //    첫 512 줄이 쪽마다 fault 를 낸다. 여기(FSM 스레드 enter, 정책 스레드가 뜨기 전)서 한 번에 치른다.
        std::memset(ring_.get(), 0, size_t(kRows) * row_bytes_);
        stage_obs_.assign(size_t(n_obs_), std::numeric_limits<float>::quiet_NaN());
        line_.assign(text_bytes_(), '\0');
        drain_obs_.assign(size_t(n_obs_), 0.f);
        stage_.clear();
        run_.store(true, std::memory_order_release);
        th_ = std::thread([this] { drain_loop_(); });
        return true;
    }

    bool on() const { return f_ != nullptr; }
    int n_obs() const { return n_obs_; }
    const Meta& meta() const { return meta_; }
    unsigned long long dropped() const { return dropped_.load(std::memory_order_relaxed); }
    unsigned long long rows_pushed() const { return head_.load(std::memory_order_relaxed); }

    // ── FSM 스레드: enter() 에서 열기·감싸기를 마친 뒤(정책 스레드가 없을 때). 다음 체류 줄의 t_enter(= enter_abs,
    //    열기 전에 now() 로 잰 값) · t_gain(지금) · 체류 번호.
    void mark_enter(double enter_abs) {
        if (!f_) return;
        t_enter_ = enter_abs - t0_;
        t_gain_ = now() - t0_;
        ++stay_;
    }
    void mark_enter() { mark_enter(now()); }
    // 쓰기 스레드의 tid(0 = 아직 안 떴다) · 자기를 내리는 데 성공했나(정책 · nice 둘 다)
    int writer_tid() const { return writer_tid_.load(std::memory_order_acquire); }
    bool writer_lowered() const { return writer_lowered_.load(std::memory_order_acquire); }

    // ── 정책 스레드 (생산자 하나) ────────────────────────────────────────────────────────────────────
    ObsDumpRow& stage() { return stage_; }
    float* stage_obs() { return stage_obs_.data(); }

    // 앞 줄을 마무리해 링에 넣고(run() 기록과 합쳐) 새 줄을 연다. kind 0 = 체류 시작 · 1 = 정책 틱.
    void begin_row(int kind) {
        if (!f_) return;
        push_pending_();
        stage_.clear();
        std::fill(stage_obs_.begin(), stage_obs_.end(), std::numeric_limits<float>::quiet_NaN());
        stage_.row = ++row_id_;
        stage_.kind = kind;
        stage_.stay = stay_;
        stage_.t_enter = (kind == 0) ? t_enter_ : std::numeric_limits<double>::quiet_NaN();
        stage_.t_gain = (kind == 0) ? t_gain_ : std::numeric_limits<double>::quiet_NaN();
        pending_ = true;
    }
    // 이 줄의 action 이 processed_actions 에 실렸다(env->step() 직후 · 체류 줄은 env->reset() 직후) — 이제부터
    // run() 의 기록에 이 줄 번호가 붙는다.
    void publish_step() {
        if (!f_ || !pending_) return;
        stage_.t = now() - t0_;
        step_seq_.store(stage_.row, std::memory_order_release);
    }
    // 체류 끝(정책 스레드가 나가기 직전) — 남은 줄을 마무리한다.
    void flush_pending() {
        if (!f_) return;
        push_pending_();
    }

    // ── run() (1 kHz FSM 스레드) ─────────────────────────────────────────────────────────────────────
    // processed_actions() «전에» 읽는다 — 이 번호 이상의 action 이 보인다.
    int64_t step_seq() const { return step_seq_.load(std::memory_order_acquire); }
    // lowcmd 를 다 실은 «뒤». proc = 블렌드 전 processed · q_des/kp/kd = lowcmd 에 실린 값(정책 관절 순서).
    void record_out(int64_t from, const float* proc, const float* q_des, const float* kp, const float* kd) {
        if (!f_) return;
        const uint32_t s = out_seq_.load(std::memory_order_relaxed);
        out_seq_.store(s + 1, std::memory_order_relaxed);   // 홀수 = 쓰는 중
        std::atomic_thread_fence(std::memory_order_release);
        out_from_.store(from, std::memory_order_relaxed);
        for (int i = 0; i < NJ; ++i) {
            out_proc_[i].store(proc[i], std::memory_order_relaxed);
            out_qdes_[i].store(q_des[i], std::memory_order_relaxed);
            out_kp_[i].store(kp[i], std::memory_order_relaxed);
            out_kd_[i].store(kd[i], std::memory_order_relaxed);
        }
        out_seq_.store(s + 2, std::memory_order_release);
    }

    // 프로덕션에서는 소멸자에서 «프로세스 종료 때 한 번». 여러 번 · 안 열린 채 불러도 안전.
    void close() {
        if (run_.exchange(false, std::memory_order_acq_rel) && th_.joinable()) th_.join();
        if (f_) {
            push_pending_();   // 정책 스레드는 이미 없다(exit() 가 join) — 남은 줄
            drain_();
            const unsigned long long d = dropped_.load(std::memory_order_relaxed);
            if (d) std::fprintf(stderr, "[obs_dump] ⚠ %llu 줄을 버렸다 (링이 찼다 — 디스크가 느리거나 링이 작다)\n", d);
            std::fflush(f_);
            std::fclose(f_);
            f_ = nullptr;
        }
    }
    ~ObsDump() { close(); }

    // 열 이름 전부(헤더 줄과 같은 순서) — 시험·분석기가 쓴다
    std::vector<std::string> column_names() const {
        std::vector<std::string> out;
        Names nm{&out};
        ObsDumpRow r;
        r.fields(nm);
        for (const auto& g : meta_.groups)
            for (int i = 0; i < g.second; ++i) out.push_back(g.first + "_" + std::to_string(i));
        return out;
    }

    // 50 Hz 기준 링 512 줄 = 10 초. 쓰기 스레드가 100 ms 마다 비우므로 평시엔 몇 줄만 찬다.
    static constexpr int kRows = 512;
    static constexpr size_t kFileBuf = size_t(4) << 20;   // stdio 버퍼 4 MB. 줄 경계에서 비운다(아래 drain_)

private:
    struct Names {
        std::vector<std::string>* out;
        template <class T> void operator()(const char* n, const T*, int k) const {
            if (k == 1) { out->push_back(n); return; }
            for (int i = 0; i < k; ++i) out->push_back(std::string(n) + "_" + std::to_string(i));
        }
    };
    // 한 줄 글자 — 쓰기 스레드 전용 버퍼에. 칸마다 «,값».
    struct Values {
        char* p; char* end;
        void put_(const char* fmt, double v) {
            if (p >= end) return;
            const int n = std::snprintf(p, size_t(end - p), fmt, v);
            p += (n > 0) ? std::min<ptrdiff_t>(n, end - p) : 0;
        }
        void operator()(const char*, const int64_t* x, int k) {
            for (int i = 0; i < k; ++i) {
                if (p >= end) return;
                const int n = std::snprintf(p, size_t(end - p), ",%lld", static_cast<long long>(x[i]));
                p += (n > 0) ? std::min<ptrdiff_t>(n, end - p) : 0;
            }
        }
        void operator()(const char*, const int32_t* x, int k) {
            for (int i = 0; i < k; ++i) {
                if (p >= end) return;
                const int n = std::snprintf(p, size_t(end - p), ",%d", static_cast<int>(x[i]));
                p += (n > 0) ? std::min<ptrdiff_t>(n, end - p) : 0;
            }
        }
        void operator()(const char*, const double* x, int k) { for (int i = 0; i < k; ++i) put_(",%.6f", x[i]); }
        void operator()(const char*, const float* x, int k) { for (int i = 0; i < k; ++i) put_(",%.9g", double(x[i])); }
    };
    struct Count { int n = 0; template <class T> void operator()(const char*, const T*, int k) { n += k; } };

    size_t text_bytes_() const {
        Count c; ObsDumpRow r; r.fields(c);
        return size_t(c.n + n_obs_) * 26 + 64;   // 칸당 «,» + %.9g 최대 15 자(−1.23456789e−38) — 넉넉히
    }

    void write_header_() {
        std::fprintf(f_, "# g1_obs_dump v1\n");
        std::fprintf(f_, "# slot=%s\n", meta_.slot.c_str());
        std::fprintf(f_, "# state=%s\n", meta_.state.c_str());
        std::fprintf(f_, "# network=%s\n", meta_.network.c_str());
        std::fprintf(f_, "# commit=%s dirty=%d\n", meta_.commit.c_str(), meta_.dirty);
        std::fprintf(f_, "# obs_contract_version=%s\n", meta_.obs_contract_version.c_str());
        std::fprintf(f_, "# obs_contract=%s\n", meta_.obs_contract.c_str());
        std::fprintf(f_, "# obs_terms=%s\n", meta_.obs_terms.c_str());
        std::fprintf(f_, "# obs_groups=");
        for (size_t i = 0; i < meta_.groups.size(); ++i)
            std::fprintf(f_, "%s%s:%d", i ? "," : "", meta_.groups[i].first.c_str(), meta_.groups[i].second);
        std::fprintf(f_, "\n");
        const auto names = column_names();
        for (size_t i = 0; i < names.size(); ++i) std::fprintf(f_, "%s%s", i ? "," : "", names[i].c_str());
        std::fprintf(f_, "\n");
        std::fflush(f_);
    }

    // run() 의 마지막 기록을 읽는다(시퀀스 잠금). 못 읽으면 false.
    bool read_out_(ObsDumpRow& r) const {
        for (int tries = 0; tries < 4096; ++tries) {
            const uint32_t s0 = out_seq_.load(std::memory_order_acquire);
            if (s0 == 0) return false;          // run() 이 아직 한 번도 안 썼다
            if (s0 & 1u) continue;              // 쓰는 중
            const int64_t from = out_from_.load(std::memory_order_relaxed);
            for (int i = 0; i < NJ; ++i) {
                r.proc[i] = out_proc_[i].load(std::memory_order_relaxed);
                r.q_des[i] = out_qdes_[i].load(std::memory_order_relaxed);
                r.kp[i] = out_kp_[i].load(std::memory_order_relaxed);
                r.kd[i] = out_kd_[i].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if (out_seq_.load(std::memory_order_relaxed) == s0) { r.out_from = from; return true; }
        }
        return false;
    }

    // 정책 스레드 — 열린 줄을 run() 기록과 합쳐 링에 넣는다. 파일 접근 없음.
    void push_pending_() {
        if (!pending_) return;
        pending_ = false;
        if (!read_out_(stage_)) {
            stage_.out_from = -2;
            for (int i = 0; i < NJ; ++i)
                stage_.proc[i] = stage_.q_des[i] = stage_.kp[i] = stage_.kd[i] = std::numeric_limits<float>::quiet_NaN();
        }
        const unsigned long long h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= (unsigned long long)kRows) {
            dropped_.fetch_add(1, std::memory_order_relaxed);   // 막지 않는다 — 제어가 우선
            return;
        }
        unsigned char* slot = ring_.get() + size_t(h % kRows) * row_bytes_;
        std::memcpy(slot, &stage_, sizeof(ObsDumpRow));
        if (n_obs_ > 0) std::memcpy(slot + sizeof(ObsDumpRow), stage_obs_.data(), size_t(n_obs_) * sizeof(float));
        head_.store(h + 1, std::memory_order_release);
    }

    // 쓰기 스레드 전용(close 의 마지막 한 번 포함 — 그땐 조인 뒤).
    void drain_() {
        unsigned long long t = tail_.load(std::memory_order_relaxed);
        const unsigned long long h = head_.load(std::memory_order_acquire);
        size_t since_flush = 0;
        ObsDumpRow r;
        std::vector<float>& obs = drain_obs_;
        for (; t < h; ++t) {
            const unsigned char* slot = ring_.get() + size_t(t % kRows) * row_bytes_;
            std::memcpy(&r, slot, sizeof(ObsDumpRow));
            if (n_obs_ > 0) std::memcpy(obs.data(), slot + sizeof(ObsDumpRow), size_t(n_obs_) * sizeof(float));
            Values v{line_.data(), line_.data() + line_.size() - 2};
            r.fields(v);
            for (int i = 0; i < n_obs_; ++i) v.put_(",%.9g", double(obs[size_t(i)]));
            // 첫 칸 앞 «,» 를 빼고 줄바꿈을 붙인다
            *v.p++ = '\n';
            const size_t n = size_t(v.p - line_.data());
            std::fwrite(line_.data() + 1, 1, n - 1, f_);
            since_flush += n;
            // stdio 가 줄 도중에 스스로 비우기 «전에» 줄 경계에서 비운다(StateDump 와 같은 이유 — 반쪽 줄 금지).
            if (since_flush >= kFileBuf / 2) { std::fflush(f_); since_flush = 0; }
        }
        if (since_flush) std::fflush(f_);
        tail_.store(t, std::memory_order_release);
    }
    void drain_loop_() {
        lower_this_thread_();   // 🔴 첫 줄 — FSM 스레드의 SCHED_FIFO 를 물려받았다(머리 주석)
        while (run_.load(std::memory_order_acquire)) {
            drain_();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        drain_();
    }

    // RtPriority.h lower_this_thread 와 같은 두 호출: RT 였다면 SCHED_OTHER 로 내리고, nice 는 «이 스레드» 에만
    // (setpriority 의 PRIO_PROCESS + tid = 스레드. 0 을 주면 프로세스 전체가 밀린다). 내리는 것이라 권한이 필요 없다.
    void lower_this_thread_() {
        sched_param sp{};
        sp.sched_priority = 0;
        const int rc1 = pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
        const pid_t tid = static_cast<pid_t>(::syscall(SYS_gettid));
        const int rc2 = ::setpriority(PRIO_PROCESS, static_cast<id_t>(tid), kWriterNice);
        if (rc1 != 0 || rc2 != 0)
            std::fprintf(stderr, "[obs_dump] ⚠ 쓰기 스레드를 SCHED_OTHER nice +%d 로 못 내렸다(%s) — RT 우선순위로 돌 수 있다\n",
                         kWriterNice, rc1 != 0 ? std::strerror(rc1) : std::strerror(errno));
        writer_lowered_.store(rc1 == 0 && rc2 == 0, std::memory_order_release);
        writer_tid_.store(static_cast<int>(tid), std::memory_order_release);
    }

public:
    static constexpr int kWriterNice = 10;
    // steady 시계 초(절대). enter() 가 덤프를 열기 전에 잰다(mark_enter).
    static double now() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

private:
    std::FILE* f_ = nullptr;
    Meta meta_;
    int n_obs_ = 0;
    size_t row_bytes_ = 0;
    std::unique_ptr<unsigned char[]> ring_;
    std::vector<char> line_;               // 쓰기 스레드 전용
    std::vector<float> drain_obs_;         // 쓰기 스레드 전용
    std::atomic<unsigned long long> head_{0}, tail_{0}, dropped_{0};
    std::atomic<bool> run_{false};
    std::thread th_;
    double t0_ = 0.0, t_enter_ = std::numeric_limits<double>::quiet_NaN(), t_gain_ = std::numeric_limits<double>::quiet_NaN();
    std::atomic<int> writer_tid_{0};
    std::atomic<bool> writer_lowered_{false};
    std::string opened_path_;
    bool warned_layout_ = false, warned_path_ = false;
    // 정책 스레드 전용
    ObsDumpRow stage_;
    std::vector<float> stage_obs_;
    bool pending_ = false;
    int64_t row_id_ = 0;
    int32_t stay_ = 0;
    // 정책 스레드 → run()
    std::atomic<int64_t> step_seq_{0};
    // run() → 정책 스레드 (시퀀스 잠금)
    std::atomic<uint32_t> out_seq_{0};
    std::atomic<int64_t> out_from_{-2};
    std::array<std::atomic<float>, NJ> out_proc_{}, out_qdes_{}, out_kp_{}, out_kd_{};
};

}  // namespace g1
