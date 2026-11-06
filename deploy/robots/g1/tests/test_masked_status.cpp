// test_masked_status.cpp — 상태 칸 /dev/shm/g1_masked_status (MaskedStatus.h) 를 잠근다.
//
//   ① layout_is_80_bytes_and_magic_0x6706 — 칸 20 개 · 오프셋 · magic · open 직후 «정책 없음» 한 줄
//   ② write_is_rt_safe — 계측기 자기검증(⓪) 뒤 write() 1000 회: write/read syscall 0 · 할당 0 · 쪽 fault 0 · 자발 문맥 교환 0.
//      그리고 seccomp 자식: «파이프에 write» 와 exit 만 허용한 필터 아래서 write() 1000 회 — 다른 syscall 을 하나라도 하면 SIGSYS 로
//      죽는다. 변이(1000 회 중간에 getppid 한 번)는 죽어야 한다(필터가 실제로 잡는다)
//   ③ seqlock_reader_never_sees_torn_frame — 쓰는 스레드 + 읽는 스레드 10⁵ 회. 받은 프레임의 모든 칸이 같은 k 에서 왔나.
//      변이(seq 를 안 올리는 쓰기)는 찢김을 보여야 한다(검사가 실제로 잡는다)
//   ④ refusal_strings_map_to_codes — ModeRuntime::request(·exit_allowed)가 실제로 내는 거부 문자열 → 1~4 · ModeRuntime.h 소스의
//      거부 문자열 전부 ↔ 표(양쪽) · 채널 관문 5 · 자세 6
//   ⑤ meter_moves — 쓸 때마다 읽은 tick·seq 가 움직인다. 다른 경로에 쓴 것은 안 보인다(멈춘 계측기를 이 검사가 가려낸다)
//   ⑥ env·멱등·쓰는 쪽 하나·요청 기록 규칙
//   ⑦ open_without_space_runs_disabled — /dev/shm 이 꽉 찬 경우(seccomp 자식에서 fallocate → ENOSPC): open 이 false · write() no-op ·
//      SIGBUS 없음 · 자물쇠(flock)를 놓아 다음 쓰는 쪽이 열 수 있다. posix_fallocate 를 빼면 open 이 성공해 FAIL(자리 잡기를 실제로 한다)
//
//   g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include -pthread test_masked_status.cpp -o /tmp/t && /tmp/t
//   (ModeRuntime.h 를 소스로 읽는다 — tests/ 에서 실행하거나 G1_MODE_RUNTIME_H 로 경로를 준다)
#include "MaskedStatus.h"
#include "ModeRuntime.h"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#include <linux/audit.h>
#include <linux/filter.h>
#include <linux/seccomp.h>

using namespace g1::masked_status;

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}

// ── 할당 카운터 (test_obs_dump_writer.cpp 와 같은 방식) ──────────────────────────────────────────────────────
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
static thread_local bool g_count_alloc = false;
static std::atomic<long> g_allocs{0};
void* operator new(std::size_t n) {
    if (g_count_alloc) g_allocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// 이 스레드의 write·read 계열 syscall 누적 수(/proc/thread-self/io 의 syscw·syscr). 못 읽으면 −1.
// 🔴 «/proc/self/syscw» 같은 파일은 없다 — 필드다(StateDump 시험이 −1 − (−1) = 0 으로 통과하는 척한 사고).
static void io_pair(long& w, long& r) {
    w = r = -1;
    std::FILE* f = std::fopen("/proc/thread-self/io", "r");
    if (!f) return;
    char k[64]; long v = 0;
    while (std::fscanf(f, "%63s %ld", k, &v) == 2) {
        if (std::strcmp(k, "syscr:") == 0) r = v;
        if (std::strcmp(k, "syscw:") == 0) w = v;
    }
    std::fclose(f);
}
struct IoMeter {   // 재는 일 자체(fopen·fscanf)의 syscall 을 뺀다
    long bw = 0, br = 0, w0 = -1, r0 = -1;
    IoMeter() { long a, b, c, d; io_pair(a, b); io_pair(c, d); bw = c - a; br = d - b; }
    void start() { io_pair(w0, r0); }
    void stop(long& w, long& r) const {   // 못 읽으면 큰 값 — 판정이 «통과하는 척» 하지 않게
        long w1, r1; io_pair(w1, r1);
        const bool ok = w0 >= 0 && r0 >= 0 && w1 >= 0 && r1 >= 0;
        w = ok ? (w1 - w0) - bw : 1000000;
        r = ok ? (r1 - r0) - br : 1000000;
    }
};
// 쪽 fault = minor + major. 🔴 둘 다 센다 — 디스크 파일시스템(com1 의 /tmp 는 ext4)에서 새 파일의 첫 쓰기 fault 는 major 로 잡혀
//    minor 만 세면 «선터치 없음» 변이가 0 으로 통과했다(2026-11-06 변이 시험에서 드러남).
static long minflt_thread() { rusage u{}; return getrusage(RUSAGE_THREAD, &u) == 0 ? u.ru_minflt + u.ru_majflt : -1; }
static long nvcsw_thread()  { rusage u{}; return getrusage(RUSAGE_THREAD, &u) == 0 ? u.ru_nvcsw : -1; }

static std::string g_dir;
static std::string tmp_path(const char* name) { return g_dir + "/" + name; }

static bool read_file(const std::string& path, MaskedStatus& out) {   // 따로 매핑해서(받는 쪽처럼) seqlock 읽기
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    void* p = ::mmap(nullptr, sizeof(MaskedStatus), PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (p == MAP_FAILED) return false;
    const bool ok = read_words(static_cast<const Word*>(p), out);
    ::munmap(p, sizeof(MaskedStatus));
    return ok;
}

// ── ① ───────────────────────────────────────────────────────────────────────────────────────────────────
static void layout_is_80_bytes_and_magic_0x6706() {
    chk(sizeof(MaskedStatus) == 80, "① sizeof == 80");
    chk(MASKED_STATUS_MAGIC == 0x6706, "① magic == 0x6706");
    for (int m = 0x6701; m <= 0x6705; ++m) chk(MASKED_STATUS_MAGIC != m, "① magic 이 다른 채널(0x6701..0x6705)과 겹친다");
    const size_t off[] = {offsetof(MaskedStatus, magic), offsetof(MaskedStatus, seq), offsetof(MaskedStatus, tick),
                          offsetof(MaskedStatus, active_mode), offsetof(MaskedStatus, last_req_mode),
                          offsetof(MaskedStatus, last_req_result), offsetof(MaskedStatus, refusal_code),
                          offsetof(MaskedStatus, refused_mode), offsetof(MaskedStatus, m5_preset),
                          offsetof(MaskedStatus, m5_arrived), offsetof(MaskedStatus, m5_arrive_seq),
                          offsetof(MaskedStatus, safety), offsetof(MaskedStatus, fsm_state),
                          offsetof(MaskedStatus, gui_seq_applied), offsetof(MaskedStatus, relay_stale),
                          offsetof(MaskedStatus, z_fk), offsetof(MaskedStatus, tilt_deg), offsetof(MaskedStatus, vx_cmd),
                          offsetof(MaskedStatus, vy_cmd), offsetof(MaskedStatus, wz_cmd)};
    static_assert(sizeof(off) / sizeof(off[0]) == N_WORDS, "칸 20 개");
    for (int i = 0; i < N_WORDS; ++i) chk(off[i] == size_t(4 * i), "① 칸 오프셋이 4·i 가 아니다(브리프 칸 순서)");
    // open 직후: magic · 짝수 seq · tick 0 · active_mode 0(«정책 없음») 이 이미 보인다
    MaskedStatusWriter w;
    const std::string p = tmp_path("layout");
    chk(w.open(p.c_str()), "① open");
    struct stat sb{};
    chk(::stat(p.c_str(), &sb) == 0 && sb.st_size == 80, "① 파일 크기 80 B");
    MaskedStatus s{};
    chk(read_file(p, s), "① open 직후 읽기");
    chk(s.magic == 0x6706 && s.seq == 2 && s.tick == 0 && s.active_mode == 0, "① open 직후 첫 줄 = magic · seq 2 · tick 0 · active_mode 0");
    w.close_for_test();
}

// ── ② ───────────────────────────────────────────────────────────────────────────────────────────────────
#if defined(__x86_64__)
#define TEST_AUDIT_ARCH AUDIT_ARCH_X86_64
#elif defined(__aarch64__)
#define TEST_AUDIT_ARCH AUDIT_ARCH_AARCH64
#endif
// 자식에서 write() 를 seccomp 필터 아래 1000 회. 반환: 0 = 정상 종료('K' 받음) · 1 = SIGSYS 로 죽음 · 2 = 그 밖 · −1 = seccomp 못 씀.
static int seccomp_child(MaskedStatusWriter& w, bool mutate) {
#ifdef TEST_AUDIT_ARCH
    int pfd[2];
    if (::pipe(pfd) != 0) return 2;
    std::fflush(stdout);
    const pid_t pid = ::fork();
    if (pid < 0) return 2;
    if (pid == 0) {
        ::close(pfd[0]);
        const uint32_t wfd = static_cast<uint32_t>(pfd[1]);
        sock_filter f[] = {
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, arch)),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, TEST_AUDIT_ARCH, 1, 0),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL),
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, nr)),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, __NR_exit_group, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, __NR_write, 1, 0),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL),
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, args[0])),   // 리틀 엔디언 — 아래 32 bit
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, wfd, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL),
        };
        sock_fprog prog{static_cast<unsigned short>(sizeof(f) / sizeof(f[0])), f};
        if (::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0) ::_exit(3);
        if (::prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &prog) != 0) ::_exit(4);
        MaskedStatus s{};
        for (uint32_t k = 1; k <= 1000; ++k) {
            s.tick = 5000u + k;
            s.vx_cmd = 0.001f * float(k);
            w.write(s);
            if (mutate && k == 500) ::syscall(SYS_getppid);   // 변이: syscall 하나
        }
        const char ok = 'K';
        ::syscall(SYS_write, pfd[1], &ok, 1);
        ::syscall(SYS_exit_group, 0);
    }
    ::close(pfd[1]);
    char c = 0;
    const ssize_t n = ::read(pfd[0], &c, 1);
    ::close(pfd[0]);
    int st = 0;
    ::waitpid(pid, &st, 0);
    if (WIFEXITED(st) && (WEXITSTATUS(st) == 3 || WEXITSTATUS(st) == 4)) return -1;
    if (WIFEXITED(st) && WEXITSTATUS(st) == 0 && n == 1 && c == 'K') return 0;
    if (WIFSIGNALED(st) && WTERMSIG(st) == SIGSYS) return 1;
    std::printf("  seccomp 자식: 예상 밖 종료 (exited=%d code=%d signaled=%d sig=%d n=%zd)\n", WIFEXITED(st),
                WIFEXITED(st) ? WEXITSTATUS(st) : -1, WIFSIGNALED(st), WIFSIGNALED(st) ? WTERMSIG(st) : -1, n);
    return 2;
#else
    (void)w; (void)mutate;
    return -1;
#endif
}

static void write_is_rt_safe() {
    // ⓪ 계측기 자기검증 — 알려진 자극에 바늘이 그만큼 움직이는가. 안 움직이면 아래 «0» 판정은 아무것도 안 잰다.
    {
        IoMeter io;
        const std::string tp = tmp_path("meter");
        std::FILE* f = std::fopen(tp.c_str(), "w+");
        chk(f != nullptr, "⓪ 임시 파일");
        if (f) {
            long w0, r0, w1, r1, w2, r2;
            io_pair(w0, r0);
            for (int i = 0; i < 200; ++i) { std::fputc('x', f); std::fflush(f); }
            io_pair(w1, r1);
            std::setvbuf(f, nullptr, _IONBF, 0);
            std::rewind(f);
            char c;
            for (int i = 0; i < 50; ++i) if (std::fread(&c, 1, 1, f) != 1) break;
            io_pair(w2, r2);
            std::fclose(f);
            chk(w0 >= 0 && w1 - w0 >= 200, "⓪ syscw 카운터가 안 움직인다 — ② syscall 판정이 무력하다");
            chk(r1 >= 0 && r2 - r1 >= 50, "⓪ syscr 카운터가 안 움직인다 — ② syscall 판정이 무력하다");
            std::printf("  ⓪ fflush 200 → syscw +%ld · fread 50 → syscr +%ld\n", w1 - w0, r2 - r1);
        }
        g_allocs.store(0); g_count_alloc = true;
        int* a = new int(1); int* b = new int(2);
        g_count_alloc = false;
        delete a; delete b;
        chk(g_allocs.load() >= 2, "⓪ 할당 카운터가 안 움직인다");
        const size_t big = 8u << 20;
        unsigned char* m = static_cast<unsigned char*>(std::malloc(big));
        const long f0 = minflt_thread();
        for (size_t i = 0; i < big; i += 4096) m[i] = 1;
        const long f1 = minflt_thread();
        std::free(m);
        chk(f0 >= 0 && f1 - f0 >= 1000, "⓪ 쪽 fault 카운터(RUSAGE_THREAD)가 안 움직인다");
        const long c0 = nvcsw_thread();
        for (int i = 0; i < 10; ++i) ::usleep(1000);
        const long c1 = nvcsw_thread();
        chk(c0 >= 0 && c1 - c0 >= 10, "⓪ 자발 문맥 교환 카운터가 안 움직인다");
        std::printf("  ⓪ new 2 → 할당 %ld · 8 MB 만짐 → fault +%ld · usleep 10 → nvcsw +%ld\n", g_allocs.load(), f1 - f0, c1 - c0);
    }
    // 본 측정: open(FSM 스레드 몫)은 밖에서, write() 1000 회만 잰다.
    MaskedStatusWriter w;
    const std::string p = tmp_path("rt");
    chk(w.open(p.c_str()), "② open");
    MaskedStatus s{};
    IoMeter io;
    io.start();
    const long f0 = minflt_thread(), c0 = nvcsw_thread();
    g_allocs.store(0); g_count_alloc = true;
    for (uint32_t k = 1; k <= 1000; ++k) {
        s.tick = k; s.active_mode = int32_t(k % 6) + 1; s.z_fk = 0.7f + 1e-4f * float(k); s.vx_cmd = 0.001f * float(k);
        w.write(s);
    }
    g_count_alloc = false;
    const long allocs = g_allocs.load();
    const long f1 = minflt_thread(), c1 = nvcsw_thread();
    long sw = 0, sr = 0;
    io.stop(sw, sr);
    const long mf = (f0 >= 0 && f1 >= 0) ? f1 - f0 : 1000000;
    const long cs = (c0 >= 0 && c1 >= 0) ? c1 - c0 : 1000000;
    std::printf("  ② write() 1000 회: write syscall %ld · read syscall %ld · 할당 %ld · 쪽 fault %ld · 자발 문맥 교환 %ld\n",
                sw, sr, allocs, mf, cs);
    chk(sw == 0, "② write() 가 write 계열 syscall 을 했다");
    chk(sr == 0, "② write() 가 read 계열 syscall 을 했다");
    chk(allocs == 0, "② write() 가 할당했다");
    chk(mf == 0, "② write() 가 쪽 fault 를 냈다 — open 에서 미리 만지지 않았다");
    chk(cs == 0, "② write() 중 자발 문맥 교환 — 막히는 호출(락·futex·sleep)이 있다");
    MaskedStatus back{};
    chk(read_file(p, back) && back.tick == 1000 && back.seq == 2u + 2u * 1000u, "② 쓴 1000 줄이 파일에 보인다(tick 1000 · seq 2002)");

    // seccomp: 필터 아래 1000 회 — 정상이면 살아서 'K', 변이(getppid)는 SIGSYS.
    const int r_ok = seccomp_child(w, false);
    if (r_ok == -1) {
        std::printf("FAIL ② seccomp 를 쓸 수 없다(이 아키텍처·커널) — syscall 0 의 강한 판정이 빠진다\n"); ++fail;
    } else {
        MaskedStatus c{};
        chk(r_ok == 0, "② seccomp(파이프 write·exit 만 허용) 아래 write() 1000 회가 죽었다 = 다른 syscall 을 했다");
        chk(read_file(p, c) && c.tick == 6000u, "② seccomp 자식이 쓴 1000 줄이 보인다(tick 6000) — 자식이 정말 돌았다");
        const int r_mut = seccomp_child(w, true);
        chk(r_mut == 1, "② 변이(getppid 한 번)가 SIGSYS 로 안 죽었다 — seccomp 판정이 무력하다");
        std::printf("  ② seccomp: write() 1000 회 = %s · 변이(getppid 1회) = %s\n", r_ok == 0 ? "살아남음" : "죽음",
                    r_mut == 1 ? "SIGSYS 로 죽음" : "안 죽음");
    }
    w.close_for_test();
}

// ── ③ ───────────────────────────────────────────────────────────────────────────────────────────────────
// 프레임 k: 낱말 i(2..19) = k·(2i+1) + i. 찢긴 프레임(두 k 가 섞임)은 consistent() 가 거짓.
static MaskedStatus frame_of(uint32_t k) {
    uint32_t wd[N_WORDS] = {};
    wd[2] = k;
    for (int i = 3; i < N_WORDS; ++i) wd[i] = k * uint32_t(2 * i + 1) + uint32_t(i);
    MaskedStatus s;
    std::memcpy(&s, wd, sizeof(s));
    return s;
}
static bool consistent(const MaskedStatus& s) {
    uint32_t wd[N_WORDS];
    std::memcpy(wd, &s, sizeof(wd));
    if (int32_t(wd[0]) != MASKED_STATUS_MAGIC) return false;
    const uint32_t k = wd[2];
    for (int i = 3; i < N_WORDS; ++i) if (wd[i] != k * uint32_t(2 * i + 1) + uint32_t(i)) return false;
    return true;
}
struct TornResult { long reads = 0, ok = 0, torn = 0, retry_fail = 0, distinct = 0; uint32_t last_tick = 0; };
// 쓰는 쪽 fn(k) 을 돌리는 스레드 + 읽는 스레드(n_reads 회). 읽기는 받는 쪽처럼 «따로 매핑한» 주소로.
template <class WriteFn>
static TornResult run_torn(const std::string& path, long n_reads, WriteFn fn) {
    TornResult r;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    void* p = fd >= 0 ? ::mmap(nullptr, sizeof(MaskedStatus), PROT_READ, MAP_SHARED, fd, 0) : MAP_FAILED;
    if (fd >= 0) ::close(fd);
    if (p == MAP_FAILED) { r.torn = -1; return r; }
    const Word* rw = static_cast<const Word*>(p);
    std::atomic<bool> stop{false};
    std::atomic<bool> started{false};
    std::thread wt([&] {
        uint32_t k = 1;
        while (!stop.load(std::memory_order_relaxed)) { fn(k++); started.store(true, std::memory_order_relaxed); }
    });
    while (!started.load(std::memory_order_relaxed)) {}
    for (long i = 0; i < n_reads; ++i) {
        MaskedStatus s{};
        ++r.reads;
        if (!read_words(rw, s)) { ++r.retry_fail; continue; }
        ++r.ok;
        if (!consistent(s)) ++r.torn;
        if (s.tick != r.last_tick) { ++r.distinct; r.last_tick = s.tick; }
    }
    stop.store(true);
    wt.join();
    ::munmap(p, sizeof(MaskedStatus));
    return r;
}

static void seqlock_reader_never_sees_torn_frame() {
    MaskedStatusWriter w;
    const std::string p = tmp_path("torn");
    chk(w.open(p.c_str()), "③ open");
    w.write(frame_of(0));
    const TornResult r = run_torn(p, 100000, [&](uint32_t k) { w.write(frame_of(k)); });
    std::printf("  ③ seqlock: 읽기 %ld · 받음 %ld · 찢김 %ld · 64 회 실패 %ld · 다른 tick %ld 개\n", r.reads, r.ok, r.torn,
                r.retry_fail, r.distinct);
    chk(r.torn == 0, "③ seqlock 읽기가 찢긴 프레임을 받았다");
    // 이 쓰는 쪽은 쉬지 않고 쓴다(실제는 20 ms 에 한 번) — 64 회 재시도가 다 막히는 읽기가 생기는 게 정상이다. 받은 것만 따진다.
    chk(r.ok >= 1000, "③ 받은 프레임이 너무 적다 — 읽기가 거의 다 막혔다");
    chk(r.distinct >= 100, "③ 읽는 쪽이 움직이는 프레임을 봐야 한다(쓰기와 겹쳐 돌았나)");
    // 변이: seq 를 안 올리고(짝수 그대로) 낱말 사이에 틈을 두고 쓴다 → 같은 검사가 찢김을 봐야 한다.
    Word* raw = const_cast<Word*>(w.words());
    const TornResult m = run_torn(p, 100000, [&](uint32_t k) {
        const MaskedStatus s = frame_of(k);
        uint32_t wd[N_WORDS];
        std::memcpy(wd, &s, sizeof(wd));
        wd[0] = static_cast<uint32_t>(MASKED_STATUS_MAGIC);
        for (int i = 2; i < N_WORDS; ++i) {
            raw[i].store(wd[i], std::memory_order_relaxed);
            for (volatile int d = 0; d < 20; d = d + 1) {}
        }
    });
    std::printf("  ③ 변이(seqlock 없음): 받음 %ld · 찢김 %ld\n", m.ok, m.torn);
    chk(m.torn > 0, "③ 변이(seq 를 안 올리는 쓰기)에서 찢김을 못 봤다 — 찢김 검사가 무력하다");
    w.close_for_test();
}

// ── ④ ───────────────────────────────────────────────────────────────────────────────────────────────────
static std::string slurp(const char* path) {
    std::string s;
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return s;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) s.append(buf, n);
    std::fclose(f);
    return s;
}
// head(«… 이름(» 까지)로 시작하는 함수의 몸통({…}). 매개변수 목록의 괄호를 먼저 닫는다 — 기본 인자 «= {}» 의 중괄호를 몸통으로
// 잡지 않게. 문자열 안에 괄호·중괄호가 없다는 전제(ModeRuntime.h 가 그렇다 — 아래 개수 검사가 지킨다).
static std::string body_of(const std::string& src, const char* head) {
    const size_t a = src.find(head);
    if (a == std::string::npos) return "";
    size_t i = a + std::strlen(head);           // head 가 '(' 로 끝난다 → 깊이 1 에서 시작
    for (int par = 1; i < src.size() && par > 0; ++i) par += (src[i] == '(') - (src[i] == ')');
    const size_t b = src.find('{', i);
    if (b == std::string::npos) return "";
    int depth = 0;
    for (size_t j = b; j < src.size(); ++j) {
        if (src[j] == '{') ++depth;
        else if (src[j] == '}' && --depth == 0) return src.substr(b, j - b + 1);
    }
    return "";
}
static std::vector<std::string> literals(const std::string& body) {
    std::vector<std::string> out;
    const std::regex re("\"((?:[^\"\\\\]|\\\\.)*)\"");
    for (std::sregex_iterator it(body.begin(), body.end(), re), end; it != end; ++it)
        if (!(*it)[1].str().empty()) out.push_back((*it)[1].str());
    return out;
}

static void refusal_strings_map_to_codes() {
    using mode_table::Exit;
    using mode_table::Safety;
    // (가) ModeRuntime::request 가 실제로 내는 문자열
    g1::ModeRuntime rt;
    rt.set_supported({1, 4, 5, 6});
    const g1::ExitContext standing{0.76f, 2.f, true};
    g1::ModeResult r = rt.request(0, standing);
    chk(!r.accepted && refusal_code_of(r.reason) == REFUSE_UNKNOWN_MODE, "④ request(0) → 1 없는 모드");
    r = rt.request(mode_table::N_MODES + 1, standing);
    chk(!r.accepted && refusal_code_of(r.reason) == REFUSE_UNKNOWN_MODE, "④ request(N+1) → 1 없는 모드");
    r = rt.request(2, standing);
    chk(!r.accepted && refusal_code_of(r.reason) == REFUSE_SLOT_LACKS_MODE, "④ request(슬롯에 없는 2) → 2");
    int crawl = 0;
    for (int m = 1; m <= mode_table::N_MODES; ++m) if (mode_table::row(m).crawl_cmd_live) crawl = m;
    chk(crawl > 0, "④ 표에 기는 모드가 있다");
    r = rt.request(crawl, standing);   // 서서 기는 모드 → 네발 창 밖
    chk(!r.accepted && refusal_code_of(r.reason) == REFUSE_ENTER_WINDOW, "④ request(기는 모드, 서 있음) → 4 진입 불가");
    r = rt.request(1, standing);
    chk(r.accepted && refusal_code_of(r.reason) == REFUSE_NONE, "④ 수락의 이유(\"\") → 0");
    // 이탈 조건 — 지금 표는 전부 Always 라 request() 로는 못 낸다 → request() 가 부르는 순수 함수(가짜 행)로
    {
        mode_table::Row from = mode_table::row(1), to = mode_table::row(2);
        const g1::ExitContext low{0.30f, 60.f, false}, moving{0.76f, 2.f, false};
        const char* why = "";
        from.exit = Exit::Upright;
        chk(!g1::exit_allowed(from, to, low, &why) && refusal_code_of(why) == REFUSE_EXIT_BLOCKED, "④ 이탈 Upright → 3");
        from.exit = Exit::StandingHold; to.exit = Exit::Always;
        chk(!g1::exit_allowed(from, to, moving, &why) && refusal_code_of(why) == REFUSE_EXIT_BLOCKED, "④ 이탈 StandingHold → 3");
        from.exit = Exit::ViaGround; to.safety = Safety::UprightOnly;
        chk(!g1::exit_allowed(from, to, moving, &why) && refusal_code_of(why) == REFUSE_EXIT_BLOCKED, "④ 이탈 ViaGround → 3");
    }
    // (나) ModeRuntime.h 소스의 거부 문자열 전부 ↔ 표 (빠진 문자열 · 표의 낡은 줄 둘 다). 기대 번호는 표가 아니라 소스 «구조» 에서
    //     따로 유도한다 — request() 안에서 어느 검사 줄의 문자열인가(valid → 1 · supports → 2 · why 초기값 → 3 · enter_allowed 뒤 → 4),
    //     exit_allowed() 안 → 3. 그래야 표의 번호를 잘못 적은 것(도달 못 하는 문자열 포함)도 잡힌다.
    const char* env = std::getenv("G1_MODE_RUNTIME_H");
    const std::string src = slurp(env && *env ? env : "../include/ModeRuntime.h");
    chk(!src.empty(), "④ ModeRuntime.h 를 못 읽었다(tests/ 에서 실행하거나 G1_MODE_RUNTIME_H)");
    const std::string req = body_of(src, "ModeResult request(");
    const std::string ex = body_of(src, "inline bool exit_allowed(");
    chk(!req.empty() && !ex.empty(), "④ ModeRuntime.h 에서 request()·exit_allowed() 몸통을 못 찾았다");
    std::set<std::string> found;
    {
        bool after_enter = false;
        size_t a = 0;
        while (a < req.size()) {
            size_t b = req.find('\n', a);
            if (b == std::string::npos) b = req.size();
            const std::string line = req.substr(a, b - a);
            a = b + 1;
            if (line.find("enter_allowed(") != std::string::npos) after_enter = true;
            const int32_t want = after_enter                                     ? REFUSE_ENTER_WINDOW
                               : line.find("valid(m)") != std::string::npos     ? REFUSE_UNKNOWN_MODE
                               : line.find("supports(m)") != std::string::npos  ? REFUSE_SLOT_LACKS_MODE
                               : line.find("why =") != std::string::npos        ? REFUSE_EXIT_BLOCKED
                                                                                : REFUSE_NONE;
            for (const std::string& s : literals(line)) {
                found.insert(s);
                chk(want != REFUSE_NONE, ("④ request() 의 어느 검사인지 모르는 줄의 문자열(시험의 구조 규칙을 넓힐 것): " + s).c_str());
                chk(refusal_code_of(s.c_str()) == want, ("④ 표의 번호가 ModeRuntime.h 구조와 다르다: " + s).c_str());
            }
        }
    }
    const std::vector<std::string> ex_lits = literals(ex);
    for (const std::string& s : ex_lits) {
        found.insert(s);
        chk(refusal_code_of(s.c_str()) == REFUSE_EXIT_BLOCKED, ("④ exit_allowed 의 문자열이 3 이 아니다: " + s).c_str());
    }
    for (const std::string& s : found)
        chk(refusal_code_of(s.c_str()) != REFUSE_NONE, ("④ MaskedStatus.h 표에 없는 ModeRuntime 거부 문자열: " + s).c_str());
    size_t n_table = 0;
    for (const RefusalString& t : MODE_RUNTIME_REFUSALS) {
        ++n_table;
        chk(found.count(t.text) == 1, (std::string("④ 표의 줄이 ModeRuntime.h 에 없다(낡음): ") + t.text).c_str());
        chk(t.code >= REFUSE_UNKNOWN_MODE && t.code <= REFUSE_ENTER_WINDOW, "④ ModeRuntime 문자열의 번호는 1..4");
    }
    chk(found.size() == n_table && n_table >= 8, "④ 긁은 문자열 수 == 표 줄 수(≥ 8) — 긁개가 움직이나");
    std::printf("  ④ ModeRuntime.h 거부 문자열 %zu 개 (exit_allowed %zu) ↔ 표 %zu 줄\n", found.size(), ex_lits.size(), n_table);
    // (다) 채널 관문 · 자세 · 모르는 문자열
    chk(channel_refusal_code(true) == REFUSE_CHANNEL && channel_refusal_code(false) == REFUSE_UNKNOWN_MODE, "④ 채널 관문 5 / 표 밖 1");
    RequestRecord rr;
    rr.posture_refused(5);
    chk(rr.refusal_code == REFUSE_POSTURE && rr.refused_mode == 5, "④ 자세 거부 → 6 · refused_mode = 자세 버튼 모드");
    chk(refusal_code_of("모르는 문자열") == REFUSE_NONE && refusal_code_of(nullptr) == REFUSE_NONE, "④ 모르는 문자열 → 0");
}

// ── ⑤ ───────────────────────────────────────────────────────────────────────────────────────────────────
static void meter_moves() {
    MaskedStatusWriter a, b;
    const std::string pa = tmp_path("meter_a"), pb = tmp_path("meter_b");
    chk(a.open(pa.c_str()) && b.open(pb.c_str()), "⑤ open 둘");
    MaskedStatus s{}, r0{}, r{};
    chk(read_file(pa, r0), "⑤ 첫 읽기");
    uint32_t tick = r0.tick, seq = r0.seq;
    int moved = 0;
    for (uint32_t k = 1; k <= 10; ++k) {
        s.tick = tick + 1;
        a.write(s);
        if (read_file(pa, r) && r.tick == tick + 1 && r.seq == seq + 2) ++moved;
        tick = r.tick; seq = r.seq;
    }
    chk(moved == 10, "⑤ 쓸 때마다 tick +1 · seq +2 가 읽혀야 한다");
    // 같은 검사가 «멈춘 계측기» 를 가려내는가: 다른 경로(b)에만 쓰면 a 의 tick·seq 는 그대로다
    MaskedStatus before{};
    chk(read_file(pa, before), "⑤ 읽기");
    for (uint32_t k = 1; k <= 5; ++k) { s.tick = 900 + k; b.write(s); }
    chk(read_file(pa, r) && r.tick == before.tick && r.seq == before.seq, "⑤ 다른 칸에 쓴 것이 a 에 보였다 — 검사가 멈춤을 못 가린다");
    std::printf("  ⑤ 10 회 쓰기 → tick %u→%u · seq %u→%u · 다른 칸 5 회 쓰기 → a 그대로(tick %u)\n", r0.tick, tick, r0.seq, seq, r.tick);
    a.close_for_test(); b.close_for_test();
}

// ── ⑥ ───────────────────────────────────────────────────────────────────────────────────────────────────
static void env_open_and_record_rules() {
    bool unk = true;
    chk(std::strcmp(path_from_env_value(nullptr, &unk), DEFAULT_PATH) == 0 && !unk, "⑥ 없음 → 기본");
    chk(std::strcmp(path_from_env_value("", &unk), DEFAULT_PATH) == 0 && !unk, "⑥ 빈 값 → 기본");
    chk(std::strcmp(path_from_env_value("1", &unk), DEFAULT_PATH) == 0 && !unk, "⑥ 1 → 기본");
    chk(path_from_env_value("0", &unk) == nullptr && !unk, "⑥ 0 → 끔");
    chk(std::strcmp(path_from_env_value("/tmp/x", &unk), "/tmp/x") == 0 && !unk, "⑥ /… → 그 경로");
    chk(std::strcmp(path_from_env_value("yes", &unk), DEFAULT_PATH) == 0 && unk, "⑥ 모르는 값 → 기본 + 경고");
    // 환경변수로 열기 · 끄기
    const std::string pe = tmp_path("env");
    ::setenv("G1_STATUS_SHM", pe.c_str(), 1);
    MaskedStatusWriter e;
    chk(e.open_from_env_or_default() && std::strcmp(e.path(), pe.c_str()) == 0, "⑥ G1_STATUS_SHM=/경로 로 열린다");
    e.close_for_test();
    ::setenv("G1_STATUS_SHM", "0", 1);
    MaskedStatusWriter off;
    chk(!off.open_from_env_or_default() && !off.on(), "⑥ G1_STATUS_SHM=0 → 끔");
    off.write(MaskedStatus{});   // no-op (죽지 않는다)
    ::unsetenv("G1_STATUS_SHM");
    // 멱등: 두 번째 open 은 다른 경로를 줘도 첫 매핑 그대로(파일을 새로 안 만든다)
    MaskedStatusWriter w;
    const std::string p1 = tmp_path("idem1"), p2 = tmp_path("idem2");
    chk(w.open(p1.c_str()), "⑥ open");
    const Word* first = w.words();
    chk(w.open(p2.c_str()) && w.words() == first && std::strcmp(w.path(), p1.c_str()) == 0, "⑥ 두 번째 open = 같은 매핑");
    struct stat sb{};
    chk(::stat(p2.c_str(), &sb) != 0, "⑥ 두 번째 open 이 다른 파일을 만들었다");
    // 쓰는 쪽 하나: 같은 파일을 두 번째 쓰는 쪽이 열면 거부(flock)
    MaskedStatusWriter second;
    chk(!second.open(p1.c_str()) && !second.on(), "⑥ 같은 파일의 두 번째 쓰는 쪽은 거부된다");
    w.close_for_test();
    MaskedStatusWriter third;
    chk(third.open(p1.c_str()), "⑥ 앞 쓰는 쪽이 닫으면 열린다");
    third.close_for_test();
    // 요청 기록 규칙
    RequestRecord r;
    chk(r.last_req_mode == 0 && r.last_req_result == REQ_NONE && r.refusal_code == 0 && r.refused_mode == 0, "⑥ 처음 = 없음");
    r.mode(6, false, REFUSE_ENTER_WINDOW);
    chk(r.last_req_mode == 6 && r.last_req_result == REQ_REFUSED && r.refusal_code == 4 && r.refused_mode == 6, "⑥ 거부 기록");
    r.mode(5, true, REFUSE_ENTER_WINDOW);
    chk(r.last_req_mode == 5 && r.last_req_result == REQ_ACCEPTED && r.refusal_code == 0 && r.refused_mode == 0, "⑥ 수락은 거부를 지운다");
    r.mode(3, false, REFUSE_NONE);
    chk(r.last_req_result == REQ_REFUSED && r.refusal_code == 0 && r.refused_mode == 3, "⑥ 번호 모르는 거부도 «거부»");
    // 자세 거부: 6 · refused_mode = 자세 버튼 모드 · last_req_mode·last_req_result 는 모드 요청만 말하므로 그대로
    r.mode(5, true, REFUSE_NONE);
    r.posture_refused(5);
    chk(r.last_req_mode == 5 && r.last_req_result == REQ_ACCEPTED && r.refusal_code == REFUSE_POSTURE && r.refused_mode == 5,
        "⑥ 자세 거부는 6·5 만 쓰고 지난 모드 요청 칸은 그대로");
    r.mode(6, false, REFUSE_ENTER_WINDOW);
    chk(r.last_req_mode == 6 && r.last_req_result == REQ_REFUSED && r.refusal_code == 4 && r.refused_mode == 6,
        "⑥ 다음 모드 요청이 넷을 같이 덮는다");
    chk(safety_level(true, true) == SAFETY_CRIT && safety_level(false, true) == SAFETY_WARN && safety_level(false, false) == SAFETY_OK,
        "⑥ 안전 등급");
}

// ── ⑦ ───────────────────────────────────────────────────────────────────────────────────────────────────
// 자식에서 fallocate syscall 만 ENOSPC 로 돌려주는 필터를 걸고 open → 결과를 종료 코드로. 반환: 종료 코드 · −1 = seccomp 못 씀 ·
// 100+신호 = 신호로 죽음(SIGBUS 면 107).
//   0 = open 이 false · on() 거짓 · write() 가 죽지 않음(기대) · 1 = open 이 성공(자리 잡기를 안 했다) · 2 = on() 이 참
static int fallocate_enospc_child(const std::string& path) {
#ifdef TEST_AUDIT_ARCH
    std::fflush(stdout); std::fflush(stderr);
    const pid_t pid = ::fork();
    if (pid < 0) return -2;
    if (pid == 0) {
        sock_filter f[] = {
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, arch)),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, TEST_AUDIT_ARCH, 1, 0),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL),
            BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, nr)),
            BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, __NR_fallocate, 0, 1),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | (ENOSPC & SECCOMP_RET_DATA)),
            BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW),
        };
        sock_fprog prog{static_cast<unsigned short>(sizeof(f) / sizeof(f[0])), f};
        if (::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0) ::_exit(3);
        if (::prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &prog) != 0) ::_exit(4);
        MaskedStatusWriter w;
        if (w.open(path.c_str())) ::_exit(1);
        if (w.on()) ::_exit(2);
        MaskedStatus s{}; s.tick = 1;
        w.write(s);                                   // no-op 이어야 한다(매핑 없음)
        ::_exit(0);
    }
    int st = 0;
    ::waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) return 100 + WTERMSIG(st);
    if (!WIFEXITED(st)) return -2;
    if (WEXITSTATUS(st) == 3 || WEXITSTATUS(st) == 4) return -1;
    return WEXITSTATUS(st);
#else
    (void)path;
    return -1;
#endif
}

static void open_without_space_runs_disabled() {
    const std::string p = tmp_path("nospace");
    const int rc = fallocate_enospc_child(p);
    if (rc == -1) { std::printf("FAIL ⑦ seccomp 를 쓸 수 없다 — 자리 없음 판정을 못 잰다\n"); ++fail; return; }
    chk(rc != 1, "⑦ fallocate 가 ENOSPC 인데 open 이 성공했다 — 자리 잡기(posix_fallocate)를 안 한다(꽉 찬 /dev/shm 에서 SIGBUS)");
    chk(rc != 2, "⑦ open 실패 뒤 on() 이 참");
    chk(rc != 100 + SIGBUS, "⑦ SIGBUS 로 죽었다");
    chk(rc == 0, "⑦ 자리 없음 → open false · write() no-op · 정상 종료");
    // 실패한 open 은 자물쇠를 놓았다(자식이 끝났으니 어차피 풀리지만, 같은 파일을 정상 환경에서 다시 열 수 있어야 한다)
    MaskedStatusWriter w;
    chk(w.open(p.c_str()), "⑦ 자리가 있으면 같은 파일이 열린다");
    MaskedStatus s{};
    chk(read_file(p, s) && s.magic == MASKED_STATUS_MAGIC, "⑦ 다시 연 뒤 첫 줄이 보인다");
    w.close_for_test();
    std::printf("  ⑦ fallocate → ENOSPC: 자식 종료 코드 %d (0 = open false · write no-op · SIGBUS 없음)\n", rc);
}

int main() {
    // 실제 칸과 같은 tmpfs(/dev/shm)에서 잰다(쪽 fault 의 모양이 파일시스템마다 다르다). 없으면 /tmp.
    char dir_shm[] = "/dev/shm/test_masked_status.XXXXXX";
    char dir_tmp[] = "/tmp/test_masked_status.XXXXXX";
    char* dir = ::mkdtemp(dir_shm);
    if (!dir) dir = ::mkdtemp(dir_tmp);
    if (!dir) { std::printf("FAIL mkdtemp\n"); return 1; }
    g_dir = dir;
    layout_is_80_bytes_and_magic_0x6706();
    write_is_rt_safe();                    // fork 하므로 스레드를 띄우는 ③ 보다 먼저
    seqlock_reader_never_sees_torn_frame();
    refusal_strings_map_to_codes();
    meter_moves();
    env_open_and_record_rules();
    open_without_space_runs_disabled();    // fork — 스레드를 띄운 ③ 뒤지만 자식은 fork 직후 단일 스레드 코드만 돈다
    if (fail) { std::printf("%d failure(s) — 임시 파일은 %s 에 남긴다\n", fail, dir); return 1; }
    const char* names[] = {"layout", "meter", "rt", "torn", "meter_a", "meter_b", "env", "idem1", "nospace"};
    for (const char* n : names) ::unlink(tmp_path(n).c_str());
    ::rmdir(dir);
    std::printf("test_masked_status ok\n");
    return 0;
}
