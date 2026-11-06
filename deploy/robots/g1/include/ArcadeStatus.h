#pragma once
// ArcadeStatus.h — g1_ctrl → 바깥 «아케이드 상태» 칸 /dev/shm/g1_arcade_status (magic 0x6708, 88 B). g1 전용(🅐 구역).
//   State_Mimic(T8)의 정책 스레드가 틱(50 Hz)마다 한 번 쓰고, 중계가 tools/arcade_status_shm.py read() 로 읽어 브라우저로 돌려준다.
//   role · skill · notice 는 core 표(role_names() · skill_names() · notice_text(code))의 «인덱스/코드»다 — 문자열을 칸에 싣지 않는다.
//   safety_class 는 core 값 그대로. 칸 배치는 tools/arcade_status_shm.py FMT·FIELDS 와 같은 커밋으로만 바꾼다
//   (tests/test_arcade_shm_layout.py 가 정규식으로 대조 — 칸은 int32_t · uint32_t · float 만).
// 쓰는 방식은 MaskedStatus.h 와 같다: 열기·flock·ftruncate·posix_fallocate·mmap 은 기동 때 한 번(FSM 스레드), 틱 경로 write() =
// 원자 저장 22 개뿐(syscall 0 · 할당 0 · 락 0) · seqlock(홀수 = 쓰는 중). 쓰는 쪽은 하나(flock).
// 환경변수 G1_ARCADE_STATUS_SHM: 없음·"1" → 기본 경로 · "0" → 끔 · "/…" → 그 경로(시험).
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
struct ArcadeStatus {            // tools/arcade_status_shm.py FMT·FIELDS 와 같은 커밋으로만 바꾼다
    int32_t  magic;              // ARCADE_STATUS_MAGIC (쓰는 쪽이 매번 채운다)
    uint32_t seq;                // seqlock — 쓰는 중 홀수 (쓰는 쪽이 매번 채운다 · 넘긴 값은 무시)
    uint32_t tick;               // 호출부가 채운다(write 는 magic·seq 만) — 보통 정책 틱마다 +1
    int32_t  role;               // core role_names() 인덱스
    int32_t  skill;              // core skill_names() 인덱스
    int32_t  mode;
    int32_t  preset;
    int32_t  route_step;
    int32_t  notice;             // core notice_text(code) 의 코드
    int32_t  safety_class;       // core 값 그대로
    int32_t  busy;               // 0/1
    int32_t  qd_warn;            // 0/1
    int32_t  qd_crit;            // 0/1
    int32_t  intent_stale;       // 0/1 — IntentApply.stale
    int32_t  rearmed;            // 0/1 — IntentApply.rearmed
    int32_t  fsm_state;          // 이 줄을 쓴 FSM 상태 번호
    uint32_t tick_us;            // 이 정책 틱의 소요 시간 [us]
    float    z_fk;               // 골반 높이 추정 [m]
    float    tilt_deg;           // 걸러진 기울기 [deg]
    float    cmd_vx;             // 정책이 받은 base_vel 명령
    float    cmd_vy;
    float    cmd_wz;
};
#pragma pack(pop)
static_assert(sizeof(ArcadeStatus) == 22 * 4, "ArcadeStatus = arcade_status_shm.FMT 22칸 88 바이트 (tests/test_arcade_shm_layout.py)");

static constexpr int32_t ARCADE_STATUS_MAGIC = 0x6708;   // 0x6701 gui v1 · 0x6702 vr · 0x6703 estop · 0x6704 gui v2 · 0x6705 relay · 0x6706 상태 칸 · 0x6707 의도 칸 과 다르게

namespace g1::arcade_status {

inline constexpr const char* DEFAULT_PATH = "/dev/shm/g1_arcade_status";
inline constexpr int N_WORDS = 22;
inline constexpr int SEQ_WORD = 1;
inline constexpr int MAX_READ_TRIES = 64;
static_assert(offsetof(ArcadeStatus, seq) == SEQ_WORD * 4, "seq 낱말 번호");

inline const char* path_from_env_value(const char* v, bool* unknown) noexcept {
    if (unknown) *unknown = false;
    if (!v || !*v || std::strcmp(v, "1") == 0) return DEFAULT_PATH;
    if (std::strcmp(v, "0") == 0) return nullptr;
    if (v[0] == '/') return v;
    if (unknown) *unknown = true;
    return DEFAULT_PATH;
}

using Word = std::atomic<uint32_t>;
static_assert(Word::is_always_lock_free, "lock-free 가 아니면 공유 메모리 낱말로 못 쓴다");
static_assert(sizeof(Word) == 4 && alignof(Word) == 4, "원자 낱말 = 4 바이트 칸");

// 읽는 쪽(시험 · C++ 소비자). 안정된(짝수 · 앞뒤 같은 seq) 프레임을 max_tries 안에 못 얻거나 magic 이 다르면 false.
inline bool read_words(const Word* w, ArcadeStatus& out, int max_tries = MAX_READ_TRIES) noexcept {
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
        return out.magic == ARCADE_STATUS_MAGIC;
    }
    return false;
}

class ArcadeStatusWriter {
public:
    static ArcadeStatusWriter& shared() { static ArcadeStatusWriter w; return w; }

    // false = 끔(G1_ARCADE_STATUS_SHM=0 — off_by_env()) 또는 실패(last_error() 에 이유). 아케이드 경로(ArcadeLink::open)는 실패면
    // 기동 거부한다(HUD 없이 뜨지 않는다 — 검토 M10).
    bool open_from_env_or_default() {
        if (words_) return true;
        bool unknown = false;
        const char* env = std::getenv("G1_ARCADE_STATUS_SHM");
        const char* p = path_from_env_value(env, &unknown);
        if (unknown) std::fprintf(stderr, "[arcade_status] G1_ARCADE_STATUS_SHM='%s' 는 모르는 값 — 기본 경로 %s\n", env, DEFAULT_PATH);
        off_by_env_ = (p == nullptr);
        if (!p) return false;
        return open(p);
    }

    // 멱등 — 이미 열렸으면 true. 실패하면 이유를 stderr 한 줄 + last_error()(그 뒤 write() 는 no-op).
    bool open(const char* path) {
        if (words_) return true;
        err_[0] = '\0';
        if (!path || !*path || std::strlen(path) >= sizeof(path_)) return fail_("경로가 비었거나 너무 길다", path);
        const int fd = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0) return fail_("open", path);
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            const int e = errno;
            ::close(fd);
            if (e == EWOULDBLOCK) {
                std::snprintf(err_, sizeof(err_), "%s 를 다른 프로세스가 쓰고 있다(flock — 다른 g1_ctrl?)", path);
                std::fprintf(stderr, "[arcade_status] %s — 이 프로세스는 쓰지 않는다\n", err_);
                return false;
            }
            errno = e;
            return fail_("flock", path);
        }
        if (::ftruncate(fd, static_cast<off_t>(sizeof(ArcadeStatus))) != 0) { const int e = errno; ::close(fd); errno = e; return fail_("ftruncate", path); }
        if (const int e = ::posix_fallocate(fd, 0, static_cast<off_t>(sizeof(ArcadeStatus))); e != 0) {
            ::close(fd); errno = e; return fail_("posix_fallocate(자리 잡기)", path);
        }
        void* p = ::mmap(nullptr, sizeof(ArcadeStatus), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) { const int e = errno; ::close(fd); errno = e; return fail_("mmap", path); }
        fd_ = fd;                                         // 닫지 않는다 — flock 을 프로세스 끝까지 쥔다
        (void)::mlock(p, sizeof(ArcadeStatus));
        Word* w = static_cast<Word*>(p);
        for (int i = 0; i < N_WORDS; ++i) new (&w[i]) Word(0u);
        std::memcpy(path_, path, std::strlen(path) + 1);
        seq_ = 0;
        words_ = w;
        ArcadeStatus first{};                             // 선터치(쪽 fault 는 여기서) + 빈 줄
        write(first);
        return true;
    }

    // 정책 스레드(50 Hz)에서 틱마다 한 번. 원자 저장 22 개 — syscall·할당·락 없음. magic·seq 칸은 여기서 채운다.
    void write(const ArcadeStatus& s) noexcept {
        Word* const w = words_;
        if (!w) return;
        uint32_t buf[N_WORDS];
        std::memcpy(buf, &s, sizeof(buf));
        buf[0] = static_cast<uint32_t>(ARCADE_STATUS_MAGIC);
        const uint32_t odd = seq_ + 1u;
        w[SEQ_WORD].store(odd, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (int i = 0; i < N_WORDS; ++i)
            if (i != SEQ_WORD) w[i].store(buf[i], std::memory_order_relaxed);
        seq_ = odd + 1u;
        w[SEQ_WORD].store(seq_, std::memory_order_release);
    }

    bool on() const { return words_ != nullptr; }
    bool off_by_env() const { return off_by_env_; }   // 마지막 open_from_env_or_default 가 G1_ARCADE_STATUS_SHM=0 으로 껐다
    const char* last_error() const { return err_; }    // 마지막 실패 이유(없으면 "")
    const char* path() const { return path_; }
    const Word* words() const { return words_; }

    // 시험 전용 — 정책 스레드가 없을 때만.
    void close_for_test() {
        if (words_) ::munmap(static_cast<void*>(words_), sizeof(ArcadeStatus));
        if (fd_ >= 0) ::close(fd_);
        words_ = nullptr; fd_ = -1; path_[0] = '\0'; seq_ = 0; err_[0] = '\0'; off_by_env_ = false;
    }

private:
    bool fail_(const char* what, const char* path) {
        const int e = errno;
        std::snprintf(err_, sizeof(err_), "%s 실패 (%s): %s", what, path ? path : "(null)", std::strerror(e));
        std::fprintf(stderr, "[arcade_status] %s — 아케이드 상태 칸을 쓰지 않는다\n", err_);
        return false;
    }
    Word* words_ = nullptr;
    int fd_ = -1;
    uint32_t seq_ = 0;
    bool off_by_env_ = false;
    char path_[256] = {};
    char err_[320] = {};
};

}  // namespace g1::arcade_status
