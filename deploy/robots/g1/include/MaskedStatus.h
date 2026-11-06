#pragma once
// MaskedStatus.h — g1_ctrl → 바깥 상태 칸 /dev/shm/g1_masked_status (magic 0x6706, 80 B). g1 전용(🅐 구역).
//   GuiCtrl(/dev/shm/g1_masked_gui, 바깥 → g1_ctrl)은 단방향이라, 받는 쪽(로봇 위 중계 tools/arcade_relay.py → 브라우저)은
//   지금 모드 · 지난 모드 요청이 먹었나/왜 거부됐나 · mode5 자세 도착 · 안전 등급 · 정책이 실제로 받은 속도 명령을 알 길이 없었다.
//   State_Mimic 의 정책 스레드가 틱(50 Hz)마다 이 칸에 한 번 쓰고, tools/status_shm.py read() 가 읽는다
//   (칸 이름·형 = 아래 구조체, magic 만 뺀 dict — g1_arcade docs/protocol_relay_v1.md §3).
//   칸 배치는 tools/status_shm.py FMT·FIELDS 와 같은 커밋으로만 바꾼다(tests/test_status_shm_layout.py 가 정규식으로 대조 —
//   그래서 칸은 int32_t · uint32_t · float 만).
//
// 🔴 기록이지 재계산이 아니다(memory instrumentation-must-record-not-recompute) — 칸마다 원본이 그 틱에 쓰거나 본 «그 값».
//    무엇을 어디서 옮기는지는 State_Mimic.cpp g_status_frame() 이 한 곳에서 적는다.
// 🔴 정책 스레드 경로 = write() = 원자 저장 20 개뿐 — syscall 0 · 할당 0 · 락 0 · 쪽 fault 0 (tests/test_masked_status.cpp 가 센다).
//    open·ftruncate·mmap 은 State_Mimic::enter()(FSM 스레드, 정책 스레드가 뜨기 전)에서 «한 번» 하고 그때 쪽을 미리 만진다(첫 프레임
//    쓰기). 그 뒤 enter() 는 같은 매핑을 다시 쓴다(멱등). 닫거나 unmap 하지 않는다 — 프로세스가 끝날 때 OS 가 거둔다(정적 소멸자가
//    정책 스레드보다 먼저 돌아도 매핑이 살아 있게).
//
// 켜는 조건: 기본 켬(파일 1 개 80 B). 환경변수 G1_STATUS_SHM:
//   없음 · "1"  → /dev/shm/g1_masked_status
//   "0"         → 끔 (write() 는 아무것도 안 한다)
//   "/…"        → 그 경로 (시험·sim2sim 을 /dev/shm 기본값과 갈라 돌릴 때). ⚠ tmpfs(/dev/shm) 아래로 줄 것 — 디스크
//                 파일시스템(com1 의 /tmp 는 ext4)이면 첫 쓰기가 major fault 이고, writeback 이 쪽을 쓰기 금지로 되돌리면 정책
//                 스레드의 다음 쓰기가 다시 fault 를 낼 수 있다(추정 — 시험은 /dev/shm 에서 잰다)
//   그 밖       → 기본 경로 + 경고 한 줄
// 쓰는 쪽은 하나여야 한다 — open 이 파일에 flock(LOCK_EX|LOCK_NB)을 잡아 둔다. 다른 g1_ctrl(남은 orphan 등)이 이미 잡고 있으면
// 경고하고 이 프로세스는 쓰지 않는다(두 쓰는 쪽이 seq 를 엇갈리게 올리면 찢긴 프레임이 «짝수» 로 보일 수 있다).
//
// 찢긴 읽기 막기 = 시퀀스 잠금(seqlock). 쓰는 쪽(정책 스레드 하나 · exit() 의 마지막 줄은 join 뒤 FSM 스레드 — join 이 순서를 맺는다):
//   seq 를 홀수로(relaxed) → release 울타리 → 칸 저장(relaxed) → seq 를 짝수로(release 저장).
//   읽는 쪽(read_words): seq(acquire) → 칸(relaxed) → acquire 울타리 → seq 다시(relaxed). 둘이 같고 짝수일 때만 받는다(아니면 다시,
//   64 번까지). 칸이 쓰는 중 바뀐 것을 하나라도 읽었다면 울타리 짝(release↔acquire)이 «홀수 seq 저장» 을 두 번째 seq 읽기보다 앞에
//   보이게 해서 s0 ≠ s1 로 걸린다 — Boehm, «Can seqlocks get along with programming language memory models?» (2012) 의 배치.
//   칸은 std::atomic<uint32_t> 낱말(4 B · lock-free = 주소 무관 → 프로세스 사이 공유 메모리에서도 같은 의미)로 저장·읽기 —
//   C++ 메모리 모형 안에서 데이터 경합이 없다(같은 프로세스 시험 스레드가 읽어도 UB 아님). 파이썬 읽기는 status_shm.py 머리 주석.
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#pragma pack(push, 1)
struct MaskedStatus {            // tools/status_shm.py FMT·FIELDS 와 같은 커밋으로만 바꾼다
    int32_t  magic;              // MASKED_STATUS_MAGIC (쓰는 쪽이 매번 채운다)
    uint32_t seq;                // seqlock — 쓰는 중 홀수 (쓰는 쪽이 매번 채운다 · 넘긴 값은 무시)
    uint32_t tick;               // 쓴 줄 수 — 정책 틱마다 +1(50 Hz). 받는 쪽은 이것이 늘지 않으면 «상태 없음» 으로 친다
    int32_t  active_mode;        // g_mode.mode() — 0 = 정책이 떠났다(State_Mimic::exit() 의 마지막 줄 · open 직후 빈 줄)
    int32_t  last_req_mode;      // 지난 «모드 요청» 의 모드(채널 상관없이 · 쓰인 그대로). 0 = 아직 없음. 자세 버튼은 안 건드린다
    int32_t  last_req_result;    // 지난 모드 요청: 0 없음 · 1 수락 · 2 거부. 자세 버튼은 안 건드린다
    int32_t  refusal_code;       // 0 없음 · 1 없는 모드 · 2 슬롯이 모르는 모드 · 3 이탈 불가 · 4 진입 불가(창 밖) · 5 채널이 못 부름 · 6 자세 거부
    int32_t  refused_mode;       // 거부된 모드 — 모드 요청 거부면 그 모드, 자세 거부(6)면 자세 버튼 모드(표의 mode5_cmd_live 행 = 5). 수락이면 0
    int32_t  m5_preset;          // 지금 mode5 자세: 0 없음 · 1..N = Mode5Presets.h PRESETS[i-1] (GuiCtrl·중계 cmd 와 같은 번호)
    int32_t  m5_arrived;         // Mode5Driver::holding() — 드라이버의 도착 선언(0/1)
    uint32_t m5_arrive_seq;      // 도착 선언(holding 이 켜지는 에지)마다 +1 (프로세스 전체)
    int32_t  safety;             // qd 가드 래치: 0 없음 · 1 warn · 2 crit
    int32_t  fsm_state;          // 이 줄을 쓴 FSM 상태 번호(config.yaml FSM._ id — 5 Mimic_Dance1_subject2 · 6 Mimic_Masked)
    uint32_t gui_seq_applied;    // GuiPoll 이 마지막으로 받아들인 GuiCtrl seq (g_gui_poll.last_seq)
    int32_t  relay_stale;        // 1 = 중계(0x6705) 하트비트 끊김 중(속도 0 으로 한 뒤 아직 새 하트비트 없음)
    float    z_fk;               // 골반 높이 추정 [m] — 모드 이탈·진입 조건이 본 그 값
    float    tilt_deg;           // 걸러진 기울기 [deg] — 모드 이탈·진입 조건이 본 그 값
    float    vx_cmd;             // 정책이 받은 base_vel 명령(g_loco.base_vel — 봉투로 자르고 slew·모드 마스크를 지난 값)
    float    vy_cmd;
    float    wz_cmd;
};
#pragma pack(pop)
static_assert(sizeof(MaskedStatus) == 20 * 4, "MaskedStatus = status_shm.FMT 20칸 80 바이트 (tests/test_status_shm_layout.py)");

static constexpr int32_t MASKED_STATUS_MAGIC = 0x6706;   // 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay 와 다르게

namespace g1::masked_status {

inline constexpr const char* DEFAULT_PATH = "/dev/shm/g1_masked_status";
inline constexpr int N_WORDS = 20;
inline constexpr int SEQ_WORD = 1;                 // seq 칸의 낱말 번호 (offsetof(MaskedStatus, seq) / 4)
inline constexpr int MAX_READ_TRIES = 64;
static_assert(offsetof(MaskedStatus, seq) == SEQ_WORD * 4, "seq 낱말 번호");

// last_req_result
inline constexpr int32_t REQ_NONE = 0, REQ_ACCEPTED = 1, REQ_REFUSED = 2;
// refusal_code
inline constexpr int32_t REFUSE_NONE = 0;
inline constexpr int32_t REFUSE_UNKNOWN_MODE = 1;     // 표에 없는 모드
inline constexpr int32_t REFUSE_SLOT_LACKS_MODE = 2;  // 이 슬롯(ONNX)이 모르는 모드
inline constexpr int32_t REFUSE_EXIT_BLOCKED = 3;     // 지금 모드의 이탈 조건
inline constexpr int32_t REFUSE_ENTER_WINDOW = 4;     // 목적지의 진입 조건(직립 창 · 기는 모드의 네발 창)
inline constexpr int32_t REFUSE_CHANNEL = 5;          // 이 채널은 그 모드를 부를 수 없다(VR = 직립 모드만)
inline constexpr int32_t REFUSE_POSTURE = 6;          // 자세 버튼 거부(범위 밖 · 키 없는 자세 · mode5 아님)
// safety
inline constexpr int32_t SAFETY_OK = 0, SAFETY_WARN = 1, SAFETY_CRIT = 2;

// ── ModeRuntime::request 가 내는 거부 문자열 → 번호 ────────────────────────────────────────────────────────────
// 🔴 ModeRuntime.h 는 고치지 않는다(g1_arcade 에 바이트 그대로 복사돼 있다) — 그래서 «그 문자열» 로 분류한다.
//    tests/test_masked_status.cpp refusal_strings_map_to_codes 가 ModeRuntime.h 소스에서 request()·exit_allowed() 안의
//    문자열을 전부 긁어 이 표와 «양쪽으로» 대조한다(빠진 문자열 · 낡은 줄 둘 다 FAIL). ModeRuntime.h 의 문구를 바꾸면 여기도 같이.
struct RefusalString { const char* text; int32_t code; };
inline constexpr RefusalString MODE_RUNTIME_REFUSALS[] = {
    {"없는 모드", REFUSE_UNKNOWN_MODE},
    {"이 슬롯(ONNX)이 모르는 모드", REFUSE_SLOT_LACKS_MODE},
    {"알 수 없는 이탈 조건", REFUSE_EXIT_BLOCKED},
    {"먼저 직립 (높이·기울기 조건)", REFUSE_EXIT_BLOCKED},
    {"먼저 직립 버튼 (직립 유지 상태에서만 나간다)", REFUSE_EXIT_BLOCKED},
    {"바닥 모드를 거쳐서만 나간다", REFUSE_EXIT_BLOCKED},
    {"네발 자세에서만 들어간다 (mode5 에서 «네발 기기» x 를 먼저)", REFUSE_ENTER_WINDOW},
    {"직립에서만 들어간다 (높이·기울기 조건)", REFUSE_ENTER_WINDOW},
};
// 모르는 문자열은 0(번호 없음) — 거부 자체는 last_req_result = 2 로 남는다. 시험이 표의 낡음을 잡으므로 실행 중엔 오지 않는다.
inline int32_t refusal_code_of(const char* reason) noexcept {
    if (!reason || !*reason) return REFUSE_NONE;
    for (const RefusalString& r : MODE_RUNTIME_REFUSALS)
        if (std::strcmp(reason, r.text) == 0) return r.code;
    return REFUSE_NONE;
}
// 채널 관문(State_Mimic.cpp g_channel_may_request)에서 막힌 요청: 표에 없는 모드면 1, 있는데 그 채널이 못 부르면 5.
inline constexpr int32_t channel_refusal_code(bool mode_in_table) noexcept {
    return mode_in_table ? REFUSE_CHANNEL : REFUSE_UNKNOWN_MODE;
}
// qd 가드 래치 → 등급. 래치는 State_Mimic 의 안전 감시가 이미 들고 있는 것(js_qd_crit_latched_ · js_qd_warn_latched_).
inline constexpr int32_t safety_level(bool crit_latched, bool warn_latched) noexcept {
    return crit_latched ? SAFETY_CRIT : (warn_latched ? SAFETY_WARN : SAFETY_OK);
}

// «지난 요청» 넷. 요청이 들어오는 문(State_Mimic.cpp g_request_mode · g_request_mode_from_channel · 자세 버튼)이 결과를 그 자리에서 적는다.
// 정책 스레드 한 곳에서만 바뀌고, 틱 끝의 write() 한 번(seqlock 한 줄)에 넷이 같이 실린다 — 받는 쪽은 넷이 어긋난 줄을 못 본다.
// 칸 뜻은 g1_arcade 의 받는 쪽(core RealController)과 맞춘 것(controller 판정 2026-11-06).
struct RequestRecord {
    int32_t last_req_mode = 0, last_req_result = REQ_NONE, refusal_code = REFUSE_NONE, refused_mode = 0;
    // 모드 요청(키보드·리모컨·GUI·중계·VR). 넷을 같이 쓴다. 거부면 code(1..5 — 모르는 문자열이면 0 일 수 있다), 수락이면 code 무시.
    void mode(int m, bool accepted, int32_t code) noexcept {
        last_req_mode = m;
        last_req_result = accepted ? REQ_ACCEPTED : REQ_REFUSED;
        refusal_code = accepted ? REFUSE_NONE : code;
        refused_mode = accepted ? 0 : m;
    }
    // 자세 버튼 거부 — refusal_code 6 · refused_mode = 자세 버튼 모드(호출부가 표에서: mode5_cmd_live 행의 id).
    // last_req_mode·last_req_result 는 «모드 요청» 만 말하므로 건드리지 않는다. 자세 수락은 아무 칸도 안 바꾼다
    // (수락·도착은 m5_preset·m5_arrived·m5_arrive_seq 가 말한다).
    void posture_refused(int posture_mode) noexcept {
        refusal_code = REFUSE_POSTURE;
        refused_mode = posture_mode;
    }
};

// ── 경로 고르기 (순수 함수 — 시험) ── 반환 nullptr = 끔. *unknown = 모르는 값이라 기본 경로로 갔다.
inline const char* path_from_env_value(const char* v, bool* unknown) noexcept {
    if (unknown) *unknown = false;
    if (!v || !*v || std::strcmp(v, "1") == 0) return DEFAULT_PATH;
    if (std::strcmp(v, "0") == 0) return nullptr;
    if (v[0] == '/') return v;
    if (unknown) *unknown = true;
    return DEFAULT_PATH;
}

using Word = std::atomic<uint32_t>;
static_assert(Word::is_always_lock_free, "lock-free 가 아니면 공유 메모리 낱말로 못 쓴다(주소 무관 아님)");
static_assert(sizeof(Word) == 4 && alignof(Word) == 4, "원자 낱말 = 4 바이트 칸");

// 읽는 쪽(시험 · C++ 소비자). 안정된(짝수 · 앞뒤 같은 seq) 프레임을 max_tries 안에 못 얻거나 magic 이 다르면 false.
inline bool read_words(const Word* w, MaskedStatus& out, int max_tries = MAX_READ_TRIES) noexcept {
    if (!w) return false;
    for (int t = 0; t < max_tries; ++t) {
        const uint32_t s0 = w[SEQ_WORD].load(std::memory_order_acquire);
        if (s0 & 1u) continue;
        uint32_t buf[N_WORDS];
        for (int i = 0; i < N_WORDS; ++i) buf[i] = w[i].load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t s1 = w[SEQ_WORD].load(std::memory_order_relaxed);
        if (s0 != s1) continue;
        buf[SEQ_WORD] = s0;
        std::memcpy(&out, buf, sizeof(out));
        return out.magic == MASKED_STATUS_MAGIC;
    }
    return false;
}

class MaskedStatusWriter {
public:
    // 🔴 g1_ctrl 은 이것 하나만 쓴다 — State_Mimic 은 FSM 상태마다 하나씩(Mimic_Dance1_subject2 · Mimic_Masked) 만들어지지만
    //    한 번에 하나만 활성이라 쓰는 쪽은 여전히 하나다. (시험은 따로 인스턴스를 만들어 다른 경로로 연다.)
    static MaskedStatusWriter& shared() { static MaskedStatusWriter w; return w; }

    // FSM 스레드(enter)에서. 이미 열렸으면 true(같은 매핑). 끔(G1_STATUS_SHM=0)·실패면 false — 그때 write() 는 no-op.
    bool open_from_env_or_default() {
        if (words_) return true;
        bool unknown = false;
        const char* env = std::getenv("G1_STATUS_SHM");
        const char* p = path_from_env_value(env, &unknown);
        if (unknown) std::fprintf(stderr, "[status] G1_STATUS_SHM='%s' 는 모르는 값 — 기본 경로 %s\n", env, DEFAULT_PATH);
        if (!p) {
            if (!said_off_) { said_off_ = true; std::fprintf(stderr, "[status] G1_STATUS_SHM=0 — 상태 칸을 쓰지 않는다\n"); }
            return false;
        }
        return open(p);
    }

    // 멱등 — 이미 열렸으면 경로와 상관없이 true(첫 매핑을 계속 쓴다). 실패하면 이유를 stderr 한 줄.
    bool open(const char* path) {
        if (words_) return true;
        if (!path || !*path || std::strlen(path) >= sizeof(path_)) return fail_("경로가 비었거나 너무 길다", path);
        const int fd = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0) return fail_("open", path);
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            const int e = errno;
            ::close(fd);
            if (e == EWOULDBLOCK) {
                std::fprintf(stderr, "[status] 🔴 %s 를 다른 프로세스가 쓰고 있다(남은 g1_ctrl? `pgrep -x g1_ctrl`) — 이 프로세스는 상태 칸을 쓰지 않는다\n", path);
                return false;
            }
            errno = e;
            return fail_("flock", path);
        }
        if (::ftruncate(fd, static_cast<off_t>(sizeof(MaskedStatus))) != 0) { const int e = errno; ::close(fd); errno = e; return fail_("ftruncate", path); }
        void* p = ::mmap(nullptr, sizeof(MaskedStatus), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) { const int e = errno; ::close(fd); errno = e; return fail_("mmap", path); }
        fd_ = fd;                                         // 닫지 않는다 — flock 을 프로세스 끝까지 쥔다(쓰는 쪽 하나 보증)
        (void)::mlock(p, sizeof(MaskedStatus));           // 쪽을 RAM 에 묶는다. 실패해도 계속(아래 첫 쓰기가 fault 를 이미 치른다)
        Word* w = static_cast<Word*>(p);
        for (int i = 0; i < N_WORDS; ++i) new (&w[i]) Word(0u);
        std::memcpy(path_, path, std::strlen(path) + 1);
        seq_ = 0;
        words_ = w;
        MaskedStatus first{};                             // 선터치(쪽 fault 는 여기서 — 정책 스레드 첫 틱이 아니라) + «정책 없음» 한 줄
        write(first);
        return true;
    }

    // 정책 스레드(50 Hz)에서 틱마다 한 번. 원자 저장 20 개 — syscall·할당·락 없음. magic·seq 칸은 여기서 채운다.
    void write(const MaskedStatus& s) noexcept {
        Word* const w = words_;
        if (!w) return;
        uint32_t buf[N_WORDS];
        std::memcpy(buf, &s, sizeof(buf));
        buf[0] = static_cast<uint32_t>(MASKED_STATUS_MAGIC);
        const uint32_t odd = seq_ + 1u;
        w[SEQ_WORD].store(odd, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);   // 홀수 seq 가 아래 칸 저장보다 먼저 보이게
        for (int i = 0; i < N_WORDS; ++i)
            if (i != SEQ_WORD) w[i].store(buf[i], std::memory_order_relaxed);
        seq_ = odd + 1u;
        w[SEQ_WORD].store(seq_, std::memory_order_release);    // 칸 저장이 짝수 seq 보다 먼저 보이게
    }

    bool on() const { return words_ != nullptr; }
    const char* path() const { return path_; }
    const Word* words() const { return words_; }

    // 시험 전용 — 정책 스레드가 없을 때만. g1_ctrl 은 부르지 않는다(위 머리 주석: 프로세스 끝까지 둔다).
    void close_for_test() {
        if (words_) ::munmap(static_cast<void*>(words_), sizeof(MaskedStatus));
        if (fd_ >= 0) ::close(fd_);
        words_ = nullptr; fd_ = -1; path_[0] = '\0'; seq_ = 0;
    }

private:
    bool fail_(const char* what, const char* path) {
        std::fprintf(stderr, "[status] %s 실패 (%s): %s — 상태 칸을 쓰지 않는다\n", what, path ? path : "(null)", std::strerror(errno));
        return false;
    }
    Word* words_ = nullptr;
    int fd_ = -1;
    uint32_t seq_ = 0;          // 쓰는 쪽만 만진다(정책 스레드 → join → FSM 스레드 exit 줄 → 다음 enter 의 새 정책 스레드)
    bool said_off_ = false;
    char path_[256] = {};
};

}  // namespace g1::masked_status
