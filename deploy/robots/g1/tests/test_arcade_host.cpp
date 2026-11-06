// test_arcade_host.cpp — ArcadeHost(include/ArcadeHost.h: 아케이드 묶음 → core · ORT → q_des 두 벌 잠금)를 잠근다.
//
//   ① opens_bundle_and_warms_sessions — 없는 폴더는 false + 이유 · 닫힌 호스트는 아무것도 안 한다. 묶음을 열면 세션 4 ·
//      꾸러미 id · role · step_dt 0.02. 첫 틱(enter 직후 v7)이 예열된 세션으로 돈다 = 그 틱의 ORT 안쪽 할당이 평소 수준(예열 안
//      한 세션의 첫 Run 은 지연 초기화로 평소보다 많다 — 대조로 같이 재 판별력을 확인) · 우리 코드 할당 0. 시간은 찍기만(계측 훅
//      켬 — 예산은 ④)
//   ② prefills_measured_pose_before_first_tick — enter 전엔 q_des 없음(false). enter(측정 상태) 직후 read_q_des = 측정 q(첫
//      추론 전 0 rad 창 없음) · 틱 번호 0 · role = 처음 role. 틱 뒤엔 PD 목표, 다시 enter 하면 새 측정 자세 · 틱 번호 0
//   ③ tick_allocates_nothing — 서 있는 상태 고정 입력(기본 자세 · 직립 · 속도 0), 의도 = 서기 · 걷기 · 달리기 · Space(점프)
//      500 틱 한 바퀴. 예열 한 바퀴 뒤 1000 틱(두 바퀴) 구간 = tick + info() + read_q_des:
//        · 우리 코드(실행 파일 = ArcadeHost · core · host 층, libstdc++ 경유 포함) operator new 0 · operator delete 0
//        · ORT 안쪽(부른 곳 = libonnxruntime, R-B5 가 뺀다) = 꾸러미마다 맨 Session::Run 바닥(test_ort_pool 과 같은 방식 —
//          세션 5 개 중 최소) + 2/run 이내. 2/run = com1 실측 세션 간 흔들림(같은 모델 · 같은 설정이라도 세션마다 v7 507~509 —
//          세션 안에서는 run 마다 같다). 틱이 ORT API 를 더 부르면(CreateTensor +3/run 등) 걸린다
//        · 페이지 폴트(RUSAGE_THREAD minflt + majflt) — 시험 시작의 mlockall(MCL_CURRENT|MCL_FUTURE)이 성공했을 때만 0 을
//          단언(R-B6), 실패면 수치만 + skip 한 줄
//        · 구간이 실제로 점프 · 달리기를 지났나(점프 role 진입 ≥ 2 · 꾸러미 전환 ≥ 4)
//      구간 앞뒤에 write(-1, "ARCADE_WIN_BEGIN"/"ARCADE_WIN_END") 표지(EBADF — 아무것도 안 쓴다): strace -f 로 두 표지 사이의
//      시스템 콜을 셀 수 있게(보고서용 측정 — 시험은 strace 에 기대지 않는다). 예열 바퀴(ARCADE_WARM_*) · ④ 의 틱들
//      (ARCADE_JUMP_*)에도 같은 표지.
//   ④ first_jump_tick_within_budget — 새로 연 호스트, 서기 100 틱 → Space → 200 틱. 활성 꾸러미가 jump_soft 로 바뀌는 틱을
//      포함한 최대 tick_us < 5000 µs(com1 예산 — 계측 훅 꺼진 상태). 중앙 · p99 · 최대 · Space 틱 값을 찍는다
//   ⑤ kp_kd_match_slot — v7 슬롯 deploy.yaml 의 stiffness/damping = 꾸러미 4 개의 kp/kd(check_gains · open 오버로드).
//      한 칸을 바꾸면 false + 그 꾸러미 · 관절 · kp/kd, open 오버로드는 닫힌 호스트로 남는다
//   ⑥ q_des_latch_never_tears — 쓰는 스레드(모든 칸 = k, 쉬지 않고) + 읽는 스레드: 받은 프레임의 29 칸이 같은 k 이고 k 가 줄지
//      않는다. 대조: 잠금 없이 같은 칸을 읽으면 찢김이 보인다(검사가 실제로 잡는다). 쓰기 간격을 둔 경우(1 ms — 실제 20 ms 보다
//      촘촘)의 읽기 실패 수도 찍는다(쉬지 않는 쓰기에선 쓰는 쪽이 읽기 한 번 사이에 두 반쪽을 지나 실패가 흔하다 — 1 ms 간격은 실측 0)
//
//   묶음 = 환경 G1_ARCADE_BUNDLE(g1_arcade build/bundle_bt5). 없으면 «skip …» 한 줄 + 종료 0.
//   빌드 = CMake 타깃 test_arcade_host(G1_ARCADE_DIR 이 있을 때, EXCLUDE_FROM_ALL) — tests/run_unit_tests.sh 가 빌드해 돌린다:
//     cmake -S .. -B /tmp/a -DG1_ARCADE_DIR=/home/piene/g1_arcade_2b && cmake --build /tmp/a --target test_arcade_host &&
//     G1_ARCADE_BUNDLE=/home/piene/g1_arcade_2b/build/bundle_bt5 /tmp/a/test_arcade_host
#include "ArcadeHost.h"

#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "arcade_host/bundle_loader.h"
#include "arcade_host/ort_pool.h"

#if !G1_HAVE_ARCADE
#error "test_arcade_host 는 G1_HAVE_ARCADE=1 로만 빌드한다(CMakeLists.txt)"
#endif

using g1::ArcadeHost;

static int fail = 0;
static void chk(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++fail;
  }
}

// ── 할당 계수(core/tests/test_rt_alloc.cpp · host/tests/test_ort_pool.cpp 와 같은 방식) ─────────────────────────────────
// 이 스레드의 계수 구간 operator new · delete 를 «부른 곳» 으로 나눈다: 부른 곳이 libonnxruntime 이면 ORT 안쪽, 아니면 우리 코드
// (정적으로 들어간 ArcadeHost · core · host 층, libstdc++ 를 거친 std::string 등). noinline: __builtin_return_address(0) = 부른 곳.
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
namespace {
thread_local long g_allocs = 0, g_ort = 0, g_frees = 0, g_frees_ort = 0;
thread_local bool g_count = false;

const void* ort_base() {
  static const void* base = [] {
    Dl_info i{};
    return dladdr(reinterpret_cast<const void*>(&OrtGetApiBase), &i) ? i.dli_fbase : nullptr;
  }();
  return base;
}
bool from_ort(const void* caller) {
  Dl_info i{};
  return caller && dladdr(caller, &i) && i.dli_fbase && i.dli_fbase == ort_base();
}
__attribute__((noinline)) void note_new(const void* caller) {
  g_count = false;  // dladdr 는 할당하지 않지만 재진입을 막는다
  ++g_allocs;
  if (from_ort(caller)) ++g_ort;
  g_count = true;
}
__attribute__((noinline)) void note_free(const void* caller) {
  g_count = false;
  ++g_frees;
  if (from_ort(caller)) ++g_frees_ort;
  g_count = true;
}
void* aligned(std::size_t n, std::size_t al) {
  void* p = nullptr;
  return posix_memalign(&p, al < sizeof(void*) ? sizeof(void*) : al, n ? n : 1) == 0 ? p : nullptr;
}
void zero_counts() { g_allocs = g_ort = g_frees = g_frees_ort = 0; }
}  // namespace

#define COUNTED_NEW(expr)                                 \
  do {                                                    \
    if (g_count) note_new(__builtin_return_address(0));   \
    if (void* p = (expr)) return p;                       \
  } while (0)
#define COUNTED_DELETE(p)                                           \
  do {                                                              \
    if (g_count && (p)) note_free(__builtin_return_address(0));     \
    std::free(p);                                                   \
  } while (0)
__attribute__((noinline)) void* operator new(std::size_t n) {
  COUNTED_NEW(std::malloc(n ? n : 1));
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new[](std::size_t n) {
  COUNTED_NEW(std::malloc(n ? n : 1));
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  COUNTED_NEW(std::malloc(n ? n : 1));
  return nullptr;
}
__attribute__((noinline)) void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
  COUNTED_NEW(std::malloc(n ? n : 1));
  return nullptr;
}
__attribute__((noinline)) void* operator new(std::size_t n, std::align_val_t al) {
  COUNTED_NEW(aligned(n, std::size_t(al)));
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new[](std::size_t n, std::align_val_t al) {
  COUNTED_NEW(aligned(n, std::size_t(al)));
  throw std::bad_alloc();
}
__attribute__((noinline)) void operator delete(void* p) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete[](void* p) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete(void* p, std::size_t) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete[](void* p, std::size_t) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete(void* p, std::align_val_t) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete[](void* p, std::align_val_t) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete(void* p, std::size_t, std::align_val_t) noexcept { COUNTED_DELETE(p); }
__attribute__((noinline)) void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { COUNTED_DELETE(p); }

namespace {
using arcade::Event;
using arcade::Intent;
using arcade::ObservableState;

long faults_now() {
  rusage u{};
  return getrusage(RUSAGE_THREAD, &u) == 0 ? u.ru_minflt + u.ru_majflt : -1;
}

struct Ctx {
  std::string bundle;
  arcade::Bundle b;    // 시험이 따로 읽은 묶음(기본 자세 · ONNX 경로 — 맨 세션 바닥용)
  arcade::Vec29 default_q{};
  bool mlock_ok = false;
  int mlock_errno = 0;
};

ObservableState standing(const Ctx& c) {
  ObservableState s;
  s.quat_wxyz = {1, 0, 0, 0};
  s.q = c.default_q;
  return s;
}

Intent move(double fwd, bool run, bool jump) {
  Intent in;
  in.move.fwd = fwd;
  in.move.run = run;
  if (jump) in.events.push_back(Event::Jump);
  return in;
}

int index_of(const std::vector<std::string>& v, const char* name) {
  for (size_t i = 0; i < v.size(); ++i)
    if (v[i] == name) return static_cast<int>(i);
  return -1;
}

std::string str(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
std::string str(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return buf;
}

// 같은 세션 설정(g1_ctrl OrtRunner · OrtPool)으로 따로 연 맨 세션 — 고정 버퍼 위 Ort::Value · 같은 Run 오버로드(test_ort_pool)
struct BareSession {
  std::unique_ptr<Ort::Session> s;
  std::vector<float> in;
  std::array<float, 29> out{};
  Ort::MemoryInfo mi = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value iv{nullptr}, ov{nullptr};
  BareSession(Ort::Env& env, const std::string& path, int obs_dim) : in(static_cast<size_t>(obs_dim), 0.f) {
    Ort::SessionOptions so;
    so.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
    so.SetIntraOpNumThreads(1);
    so.SetInterOpNumThreads(1);
    so.AddConfigEntry("session.intra_op.allow_spinning", "0");
    s = std::make_unique<Ort::Session>(env, path.c_str(), so);
    const int64_t ish[2] = {1, int64_t(in.size())}, osh[2] = {1, 29};
    iv = Ort::Value::CreateTensor<float>(mi, in.data(), in.size(), ish, 2);
    ov = Ort::Value::CreateTensor<float>(mi, out.data(), out.size(), osh, 2);
  }
  void run() {
    static const char* in_name = "obs";
    static const char* out_name = "actions";
    s->Run(Ort::RunOptions{nullptr}, &in_name, &iv, 1, &out_name, &ov, 1);
  }
};

// 꾸러미의 맨 Session::Run ORT 쪽 할당 수(세션 5 개 × runs 회 중 세션 최소)
constexpr long kSessionJitterPerRun = 2;  // 세션 간 ORT 안쪽 흔들림(머리 주석 ③)
long bare_floor(Ort::Env& env, const std::string& path, int obs_dim, int runs) {
  long best = -1;
  for (int b = 0; b < 5; ++b) {
    BareSession bare(env, path, obs_dim);
    for (int k = 0; k < 20; ++k) bare.run();
    zero_counts();
    g_count = true;
    for (int k = 0; k < runs; ++k) bare.run();
    g_count = false;
    if (g_allocs != g_ort) return -2;  // 맨 Run 의 할당이 전부 ORT 안쪽이어야 한다(계수 방식 전제)
    best = best < 0 ? g_ort : std::min(best, g_ort);
  }
  return best;
}

// ── ① ─────────────────────────────────────────────────────────────────────────────────────────────────────────
void opens_bundle_and_warms_sessions(const Ctx& c, ArcadeHost& h) {
  ArcadeHost bad;
  std::string err;
  chk(!bad.open("/nonexistent/arcade_bundle", err), "① 없는 폴더 open 이 false");
  chk(!err.empty(), "① 없는 폴더 open 의 이유가 있다");
  chk(!bad.is_open(), "① 실패한 open 뒤 닫힌 상태");
  std::array<float, 29> q{};
  chk(!bad.read_q_des(q), "① 닫힌 호스트 read_q_des = false");
  const ArcadeHost::TickOut t0 = bad.tick(standing(c), Intent{});
  chk(!t0.ok && t0.tick_us == 0, "① 닫힌 호스트 tick 은 아무것도 안 한다(ok=false)");
  chk(bad.session_count() == 0 && bad.skill_names().empty(), "① 닫힌 호스트 = 세션 0 · 이름 없음");
  std::printf("  · ① 없는 폴더: %s\n", err.c_str());

  err.clear();
  const auto o0 = std::chrono::steady_clock::now();
  const bool ok = h.open(c.bundle, err);
  const double open_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - o0).count();
  chk(ok, "① 묶음 open");
  if (!ok) {
    std::printf("  · ① open 실패: %s\n", err.c_str());
    return;
  }
  chk(h.is_open(), "① is_open");
  chk(h.session_count() == 4, "① 세션 4 개");
  const std::set<std::string> skills(h.skill_names().begin(), h.skill_names().end());
  chk(skills == std::set<std::string>{"v7", "posture_i5", "jump_soft", "indep4_bridge"}, "① 꾸러미 id 4 개");
  for (const char* r : {"walk", "play", "crawl", "posture", "jump", "bridge"})
    chk(index_of(h.role_names(), r) >= 0, "① role 이 있다(walk · play · crawl · posture · jump · bridge)");
  chk(std::fabs(h.step_dt() - 0.02) < 1e-12, "① step_dt 0.02");

  // 예열 효과: 세션의 첫 Run 은 지연 초기화로 ORT 안쪽 할당이 평소보다 많다(com1 실측 v7 651 vs 508 — 대조 = 예열 안 한 v7
  // 세션의 첫 · 둘째 Run).
  // ArcadeHost 의 첫 틱(enter 직후, v7 첫 틱)은 평소 수준이어야 한다 = open 이 warm_up 을 했다. 시간은 계측 훅(ORT 할당마다
  // dladdr)이 켜진 값이라 찍기만 한다(예산 단언은 훅을 끈 ④).
  long cold_first = -1, cold_second = -1;
  double cold_ms = -1, warm_ms = -1;
  {
    arcade::OrtPool cold;
    std::string e2;
    std::map<std::string, std::string> one{{"v7", c.b.onnx_paths.at("v7")}};
    if (cold.open(one, e2)) {
      std::vector<float> z(static_cast<size_t>(cold.obs_dim("v7")), 0.f);
      ort_base();
      const auto a = std::chrono::steady_clock::now();
      zero_counts();
      g_count = true;
      const bool r1 = cold.run(0, z.data(), static_cast<int>(z.size())) != nullptr;
      g_count = false;
      cold_first = g_ort;
      const auto b = std::chrono::steady_clock::now();
      zero_counts();
      g_count = true;
      const bool r2 = cold.run(0, z.data(), static_cast<int>(z.size())) != nullptr;
      g_count = false;
      cold_second = g_ort;
      const auto d = std::chrono::steady_clock::now();
      if (!r1 || !r2) cold_first = cold_second = -1;
      cold_ms = std::chrono::duration<double, std::milli>(b - a).count();
      warm_ms = std::chrono::duration<double, std::milli>(d - b).count();
    }
  }
  h.enter(standing(c));
  ort_base();
  zero_counts();
  g_count = true;
  const ArcadeHost::TickOut t1 = h.tick(standing(c), Intent{});
  g_count = false;
  const long first_ort = g_ort, first_ours = (g_allocs - g_ort) + (g_frees - g_frees_ort);
  chk(t1.ok, "① 첫 틱 ok");
  chk(t1.info.skill == index_of(h.skill_names(), "v7"), "① 첫 틱 꾸러미 = v7(처음 role walk)");
  chk(cold_first > 0 && cold_second > 0, "① 대조 세션 첫 · 둘째 Run");
  chk(cold_first >= cold_second + 50, "① 대조: 예열 안 한 세션의 첫 Run 은 ORT 할당이 평소보다 50 개 이상 많다(판별력)");
  chk(first_ort <= cold_second + kSessionJitterPerRun, "① ArcadeHost 첫 틱의 ORT 할당 = 평소 수준(세션이 open 때 예열됐다)");
  chk(first_ours == 0, "① ArcadeHost 첫 틱(enter 직후) 우리 코드 할당 · 해제 0");
  std::printf("  · ① open %.2f s · 첫 틱 %d µs · ORT 안쪽 new %ld · 우리 코드 new+delete %ld · (대조) 예열 없는 v7 세션 첫 Run "
              "%.2f ms · new %ld / 둘째 Run %.2f ms · new %ld\n",
              open_s, t1.tick_us, first_ort, first_ours, cold_ms, cold_first, warm_ms, cold_second);
}

// ── ② ─────────────────────────────────────────────────────────────────────────────────────────────────────────
void prefills_measured_pose_before_first_tick(const Ctx& c, ArcadeHost& h) {
  if (!h.is_open()) {
    chk(false, "② 호스트가 열리지 않았다(① 실패)");
    return;
  }
  // 새로 연 호스트(enter 전) = q_des 없음. ① 이 이미 enter · tick 했으니 다시 연다(같은 묶음).
  std::string err;
  chk(h.open(c.bundle, err), "② 다시 open");
  std::array<float, 29> q{};
  chk(!h.read_q_des(q), "② enter 전 read_q_des = false(q_des 없음)");

  ObservableState m = standing(c);
  for (int i = 0; i < 29; ++i) m.q[i] = c.default_q[i] + 0.01 * (i + 1) * ((i % 2) ? 1 : -1);  // 기본 자세와 다른 측정 자세
  m.t = 123.0;  // 넘겨준 t 는 쓰지 않는다(틱 번호 × dt)
  h.enter(m);
  q.fill(-99.f);
  const bool got = h.read_q_des(q);
  chk(got, "② enter 직후 read_q_des = true");
  bool same = got;
  for (int i = 0; i < 29; ++i) same = same && q[i] == static_cast<float>(m.q[i]);
  chk(same, "② enter 직후 q_des = 측정 q(첫 추론 전 0 rad 창 없음)");
  chk(h.ticks() == 0, "② enter 뒤 틱 번호 0");
  chk(h.info().role == index_of(h.role_names(), "walk"), "② enter 뒤 role = 처음 role(walk)");

  // 틱 뒤 = PD 목표(측정 자세와 다르다 — 정책 출력), 유한
  const ArcadeHost::TickOut t = h.tick(m, Intent{});
  chk(t.ok, "② 첫 틱 ok");
  std::array<float, 29> q2{};
  chk(h.read_q_des(q2), "② 틱 뒤 read_q_des = true");
  bool finite = true, differs = false;
  for (int i = 0; i < 29; ++i) {
    finite = finite && std::isfinite(q2[i]);
    differs = differs || q2[i] != q[i];
  }
  chk(finite, "② 틱 뒤 q_des 유한");
  chk(differs, "② 틱 뒤 q_des = PD 목표(측정 자세 그대로가 아니다)");
  chk(h.ticks() == 1, "② 틱 번호 1");

  // 다시 enter(재진입) = 새 측정 자세 · 틱 번호 0
  ObservableState m2 = standing(c);
  for (int i = 0; i < 29; ++i) m2.q[i] = c.default_q[i] - 0.02;
  h.enter(m2);
  std::array<float, 29> q3{};
  bool same2 = h.read_q_des(q3);
  for (int i = 0; i < 29; ++i) same2 = same2 && q3[i] == static_cast<float>(m2.q[i]);
  chk(same2, "② 다시 enter = 새 측정 자세");
  chk(h.ticks() == 0, "② 다시 enter 뒤 틱 번호 0");
}

// ② 보강 — enter 는 Runtime 을 처음 상태로 되돌린다 · 비유한 측정 자세는 q_des 에 싣지 않는다
void enter_resets_runtime_and_rejects_nonfinite(const Ctx& c) {
  ArcadeHost h;
  std::string err;
  if (!h.open(c.bundle, err)) {
    chk(false, "②b open");
    return;
  }
  const ObservableState s = standing(c);
  const Intent idle = move(0, false, false), jump = move(0, false, true);
  const int v7 = index_of(h.skill_names(), "v7"), jump_skill = index_of(h.skill_names(), "jump_soft");
  const int walk = index_of(h.role_names(), "walk");
  h.enter(s);
  for (int k = 0; k < 120; ++k) h.tick(s, k == 100 ? jump : idle);
  const ArcadeHost::TickOut t = h.tick(s, idle);
  chk(t.ok && t.info.skill == jump_skill, "②b 전제: Space 뒤 활성 꾸러미 = jump_soft");
  h.enter(s);  // 재진입 — Runtime::reset() 이 role · skill · 경로를 처음으로
  const arcade::TickInfo i = h.info();
  chk(i.role == walk, "②b 재진입 뒤 role = walk(기본 role)");
  chk(i.skill == v7, "②b 재진입 뒤 활성 꾸러미 = v7");
  chk(i.route_step == -1, "②b 재진입 뒤 경로 없음(route_step = -1)");

  // 비유한 측정 자세
  ArcadeHost g;
  chk(g.open(c.bundle, err), "②b 다시 open");
  ObservableState bad = standing(c);
  bad.q[5] = std::numeric_limits<double>::quiet_NaN();
  g.enter(bad);
  std::array<float, 29> q{};
  chk(!g.read_q_des(q), "②b 비유한 q 로 첫 enter → read_q_des = false(싣지 않음)");
  ObservableState good = standing(c);
  for (int k = 0; k < 29; ++k) good.q[k] = c.default_q[k] + 0.03;
  g.enter(good);
  ObservableState inf = good;
  inf.q[0] = std::numeric_limits<double>::infinity();
  g.enter(inf);
  bool kept = g.read_q_des(q);
  for (int k = 0; k < 29; ++k) kept = kept && q[k] == static_cast<float>(good.q[k]);
  chk(kept, "②b 비유한 q 로 다시 enter → 이전 q_des 유지");
}

// ── ③ ─────────────────────────────────────────────────────────────────────────────────────────────────────────
void tick_allocates_nothing(const Ctx& c, ArcadeHost& h) {
  if (!h.is_open()) {
    chk(false, "③ 호스트가 열리지 않았다");
    return;
  }
  constexpr int kCycle = 500, kWarm = 500, kMeasure = 1000;
  // 의도 표(구간 밖에서 만든다 — 틱마다 가리키기만): 서기 · 걷기 · 걷기+Space · 달리기 · 달리기+Space
  const Intent idle = move(0, false, false), walk = move(1.0, false, false), walk_jump = move(1.0, false, true),
               run = move(1.0, true, false), run_jump = move(1.0, true, true);
  auto intent_at = [&](int k) -> const Intent& {
    const int p = k % kCycle;
    if (p < 50) return idle;
    if (p == 150) return walk_jump;
    if (p < 250) return walk;
    if (p == 350) return run_jump;
    if (p < 450) return run;
    return walk;
  };
  const ObservableState s = standing(c);
  h.enter(s);
  std::array<float, 29> q{};
  syscall(SYS_write, -1, "ARCADE_WARM_BEGIN", 17);  // 예열 바퀴(처음 점프 · 달리기 포함)도 strace 로 볼 수 있게(단언 없음)
  for (int k = 0; k < kWarm; ++k) {
    h.tick(s, intent_at(k));
    (void)h.info();
    (void)h.read_q_des(q);
  }
  syscall(SYS_write, -1, "ARCADE_WARM_END", 15);

  // 측정 칸(구간 밖에서 잡고 한 번 써 둔다)
  constexpr int kMaxSkills = 8;
  long ours_new = 0, ours_free = 0, ort_free = 0, ort_new[kMaxSkills] = {}, runs[kMaxSkills] = {};
  int bad_ticks = 0;
  std::vector<int> role(kMeasure, 0), skill(kMeasure, 0), tick_us(kMeasure, 0), ours_ticks(kMeasure, 0);
  int n_ours_ticks = 0;
  ort_base();  // 첫 dladdr 를 구간 밖에서
  const long f0 = faults_now();
  syscall(SYS_write, -1, "ARCADE_WIN_BEGIN", 16);
  for (int i = 0; i < kMeasure; ++i) {
    const Intent& in = intent_at(kWarm + i);
    zero_counts();
    g_count = true;
    const ArcadeHost::TickOut t = h.tick(s, in);
    const arcade::TickInfo inf = h.info();
    const bool r = h.read_q_des(q);
    g_count = false;
    const long mine = (g_allocs - g_ort) + (g_frees - g_frees_ort);
    ours_new += g_allocs - g_ort;
    ours_free += g_frees - g_frees_ort;
    ort_free += g_frees_ort;
    if (mine) ours_ticks[n_ours_ticks++] = kWarm + i;
    if (inf.skill >= 0 && inf.skill < kMaxSkills) {
      ort_new[inf.skill] += g_ort;
      ++runs[inf.skill];
    }
    if (!t.ok || !r || inf.skill != t.info.skill) ++bad_ticks;
    role[i] = inf.role;
    skill[i] = inf.skill;
    tick_us[i] = t.tick_us;
  }
  syscall(SYS_write, -1, "ARCADE_WIN_END", 14);
  const long faults = faults_now() - f0;

  const std::vector<std::string>& sn = h.skill_names();
  const std::vector<std::string>& rn = h.role_names();
  // 구간이 실제로 점프 · 달리기 · 꾸러미 전환을 지났나
  const int jump_role = index_of(rn, "jump");
  int jump_entries = 0, switches = 0;
  for (int i = 1; i < kMeasure; ++i) {
    jump_entries += role[i] == jump_role && role[i - 1] != jump_role;
    switches += skill[i] != skill[i - 1];
  }
  std::string roles_seen;
  for (int i = 0; i < kMeasure; ++i)
    if (i == 0 || role[i] != role[i - 1])
      roles_seen += (i ? " > " : "") + (role[i] >= 0 && role[i] < int(rn.size()) ? rn[role[i]] : std::string("?")) +
                    "@" + std::to_string(kWarm + i);
  std::printf("  · ③ 구간 role 열: %s\n", roles_seen.c_str());
  chk(bad_ticks == 0, "③ 구간 모든 틱 ok · read_q_des true · info() = TickOut.info");
  chk(jump_entries >= 2, "③ 구간이 점프 role 에 2 번 이상 들어갔다");
  chk(switches >= 4, "③ 구간 꾸러미 전환 ≥ 4(v7 ↔ jump_soft)");

  std::string at;
  for (int i = 0; i < n_ours_ticks && i < 12; ++i) at += (i ? "," : " · 틱 ") + std::to_string(ours_ticks[i]);
  std::printf("  · ③ 구간 %d 틱(예열 %d 틱 뒤): 우리 코드 new %ld · delete %ld%s · ORT 안쪽 delete %ld\n", kMeasure, kWarm, ours_new,
              ours_free, at.c_str(), ort_free);
  chk(ours_new == 0, "③ 우리 코드 operator new 0(예열 뒤)");
  chk(ours_free == 0, "③ 우리 코드 operator delete 0(예열 뒤)");

  // ORT 안쪽 = 꾸러미마다 맨 Session::Run 바닥 + 1/run
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_arcade_host_bare");
  for (int k = 0; k < int(sn.size()) && k < kMaxSkills; ++k) {
    if (runs[k] == 0) continue;
    const auto path = c.b.onnx_paths.find(sn[k]);
    int dim = -1;
    for (const arcade::SkillSpec& sp : c.b.setup.skills)
      if (sp.id == sn[k]) dim = sp.obs_dim;
    if (path == c.b.onnx_paths.end() || dim <= 0) {
      chk(false, "③ 꾸러미 ONNX 경로 · obs_dim 을 못 찾았다");
      continue;
    }
    constexpr int kBareRuns = 200;
    const long floor = bare_floor(env, path->second, dim, kBareRuns);
    chk(floor >= 0, "③ 맨 Session::Run 의 할당이 전부 ORT 안쪽(계수 전제)");
    const double per = double(ort_new[k]) / runs[k], base = double(floor) / kBareRuns;
    std::printf("  · ③ %s: run %ld 회 · ORT 안쪽 new %ld (%.2f /run) · 맨 Run 바닥 %.2f /run · 허용 +%ld /run\n", sn[k].c_str(),
                runs[k], ort_new[k], per, base, kSessionJitterPerRun);
    chk(floor >= 0 && ort_new[k] * kBareRuns <= (floor + kSessionJitterPerRun * kBareRuns) * runs[k],
        "③ ORT 안쪽 할당 ≤ 맨 Run 바닥 + 2/run");
  }

  if (c.mlock_ok) {
    std::printf("  · ③ 페이지 폴트(틱 전체 · mlockall 성공): %ld\n", faults);
    chk(faults == 0, "③ 틱 전체 페이지 폴트 0(mlockall 뒤)");
  } else {
    std::printf("skip ③ 틱 전체 페이지 폴트 단언 — mlockall 실패(errno=%d %s) · 수치만: %ld\n", c.mlock_errno,
                std::strerror(c.mlock_errno), faults);
  }
  std::vector<int> sorted(tick_us);
  std::sort(sorted.begin(), sorted.end());
  std::printf("  · ③ tick_us(계측 훅 켬 — ORT 할당마다 dladdr): 중앙 %d · p99 %d · 최대 %d\n", sorted[kMeasure / 2],
              sorted[kMeasure * 99 / 100], sorted.back());
}

// ── ④ ─────────────────────────────────────────────────────────────────────────────────────────────────────────
void first_jump_tick_within_budget(const Ctx& c) {
  ArcadeHost h;  // 새로 연 호스트 — 점프 꾸러미가 처음 활성이 되는 틱(예열 바퀴 없음)
  std::string err;
  if (!h.open(c.bundle, err)) {
    chk(false, "④ open");
    std::printf("  · ④ open 실패: %s\n", err.c_str());
    return;
  }
  constexpr int kStand = 100, kAfter = 200, kN = kStand + 1 + kAfter;
  const Intent idle = move(0, false, false), jump = move(0, false, true);
  const ObservableState s = standing(c);
  h.enter(s);
  const int jump_skill = index_of(h.skill_names(), "jump_soft");
  std::vector<int> us(kN, 0), skill(kN, 0);
  bool all_ok = true;
  syscall(SYS_write, -1, "ARCADE_JUMP_BEGIN", 17);  // 새 호스트의 첫 점프 틱들 — strace 로 볼 수 있게(단언 없음)
  for (int k = 0; k < kN; ++k) {
    const ArcadeHost::TickOut t = h.tick(s, k == kStand ? jump : idle);
    us[k] = t.tick_us;
    skill[k] = t.info.skill;
    all_ok = all_ok && t.ok;
  }
  syscall(SYS_write, -1, "ARCADE_JUMP_END", 15);
  int first_jump = -1, back = -1;
  for (int k = 0; k < kN; ++k) {
    if (first_jump < 0 && skill[k] == jump_skill) first_jump = k;
    if (first_jump >= 0 && back < 0 && skill[k] != jump_skill) back = k;
  }
  chk(all_ok, "④ 모든 틱 ok");
  chk(first_jump == kStand, "④ Space 틱에 활성 꾸러미가 jump_soft 로 바뀐다");
  chk(back > first_jump, "④ 점프 뒤 다른 꾸러미로 돌아온다");
  std::vector<int> sorted(us);
  std::sort(sorted.begin(), sorted.end());
  const int mx = sorted.back();
  std::printf("  · ④ tick_us(%d 틱 · 계측 훅 끔): 중앙 %d · p99 %d · 최대 %d · enter 뒤 첫 틱 %d · Space(점프 전환) 틱 %d · 그다음 %d · "
              "돌아온 틱(%d) %d · max_tick_us() %d\n",
              kN, sorted[kN / 2], sorted[kN * 99 / 100], mx, us[0], first_jump >= 0 ? us[first_jump] : -1,
              first_jump >= 0 && first_jump + 1 < kN ? us[first_jump + 1] : -1, back, back >= 0 ? us[back] : -1,
              h.max_tick_us());
  chk(mx < 5000, "④ Space 틱 포함 최대 tick_us < 5000 µs");
  chk(h.max_tick_us() == mx, "④ max_tick_us() = 틱 최대");
}

// ── ⑤ ─────────────────────────────────────────────────────────────────────────────────────────────────────────
bool read_slot_gains(std::array<float, 29>& kp, std::array<float, 29>& kd, std::string& why) {
  const std::string path =
      std::string(G1_SOURCE_DIR) + "/config/policy/mimic_masked/261009_v7_f6_s1456_109k/params/deploy.yaml";
  try {
    const YAML::Node y = YAML::LoadFile(path);
    const YAML::Node s = y["stiffness"], d = y["damping"];
    if (!s || !d || s.size() != 29 || d.size() != 29) {
      why = path + ": stiffness/damping 이 29 칸이 아니다";
      return false;
    }
    for (int i = 0; i < 29; ++i) {
      kp[i] = s[i].as<float>();
      kd[i] = d[i].as<float>();
    }
    return true;
  } catch (const std::exception& e) {
    why = path + ": " + e.what();
    return false;
  }
}

void kp_kd_match_slot(const Ctx& c, ArcadeHost& h) {
  std::array<float, 29> kp{}, kd{};
  std::string why;
  if (!read_slot_gains(kp, kd, why)) {
    chk(false, "⑤ 슬롯 deploy.yaml 읽기");
    std::printf("  · ⑤ %s\n", why.c_str());
    return;
  }
  std::string err;
  chk(h.is_open() || h.open(c.bundle, err), "⑤ 호스트 열기");
  chk(h.check_gains(kp, kd, err), "⑤ 꾸러미 4 개 kp/kd = v7 슬롯 stiffness/damping");
  if (!err.empty()) std::printf("  · ⑤ %s\n", err.c_str());

  std::array<float, 29> kp_bad = kp, kd_bad = kd;
  kp_bad[4] *= 1.01f;
  err.clear();
  chk(!h.check_gains(kp_bad, kd, err), "⑤ kp 한 칸 다르면 false");
  chk(err.find("관절 4") != std::string::npos && err.find("kp") != std::string::npos, "⑤ 이유에 관절 4 · kp");
  std::printf("  · ⑤ kp 변이: %s\n", err.c_str());
  kd_bad[20] += 0.05f;
  err.clear();
  chk(!h.check_gains(kp, kd_bad, err), "⑤ kd 한 칸 다르면 false");
  chk(err.find("관절 20") != std::string::npos && err.find("kd") != std::string::npos, "⑤ 이유에 관절 20 · kd");

  // open 오버로드(State_Mimic 이 부를 것): 같으면 열림, 다르면 false + 닫힌 상태
  err.clear();
  chk(h.open(c.bundle, kp, kd, err), "⑤ open(묶음, 슬롯 kp, kd) = true");
  chk(h.is_open(), "⑤ open 오버로드 뒤 열림");
  err.clear();
  chk(!h.open(c.bundle, kp_bad, kd, err), "⑤ open(묶음, 틀린 kp, kd) = false");
  chk(!h.is_open(), "⑤ 틀린 게인 open 뒤 닫힌 상태");
  chk(err.find("kp") != std::string::npos, "⑤ 틀린 게인 open 의 이유");
  ArcadeHost closed;
  err.clear();
  chk(!closed.check_gains(kp, kd, err), "⑤ 닫힌 호스트 check_gains = false");
}

// ── ⑥ ─────────────────────────────────────────────────────────────────────────────────────────────────────────
// 대조용: 잠금 없이 같은 칸을 쓰고 읽는다(찢김이 보여야 검사가 실제로 잡는 것)
struct NoLatch {
  std::atomic<uint32_t> w[29] = {};
  void publish(const float* q) noexcept {
    for (int i = 0; i < 29; ++i) {
      uint32_t x;
      std::memcpy(&x, &q[i], 4);
      w[i].store(x, std::memory_order_relaxed);
    }
  }
  bool read(std::array<float, 29>& out) const noexcept {
    for (int i = 0; i < 29; ++i) {
      const uint32_t x = w[i].load(std::memory_order_relaxed);
      std::memcpy(&out[i], &x, 4);
    }
    return true;
  }
};

template <class L>
void hammer(L& latch, int n_frames, long& reads, long& torn, long& backwards, long& misses, int gap_us = 0) {
  std::atomic<bool> done{false};
  reads = torn = backwards = misses = 0;
  std::thread reader([&] {
    std::array<float, 29> q{};
    float last = 0;
    while (!done.load(std::memory_order_acquire)) {
      if (!latch.read(q)) {
        ++misses;
        continue;
      }
      ++reads;
      bool same = true;
      for (int i = 1; i < 29; ++i) same = same && q[i] == q[0];
      if (!same) ++torn;
      if (same && q[0] < last) ++backwards;
      if (same) last = q[0];
    }
  });
  std::array<float, 29> f{};
  for (int k = 1; k <= n_frames; ++k) {
    f.fill(static_cast<float>(k));
    latch.publish(f.data());
    if (gap_us > 0) {  // 바쁜 기다림(쓰는 쪽 스레드 — 시스템 콜 없이 간격)
      const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(gap_us);
      while (std::chrono::steady_clock::now() < until) {
      }
    }
  }
  done.store(true, std::memory_order_release);
  reader.join();
}

void q_des_latch_never_tears() {
  g1::QDesLatch l;
  std::array<float, 29> q{};
  chk(!l.read(q), "⑥ 쓰기 전 read = false");
  constexpr int kFrames = 300000;
  long reads = 0, torn = 0, backwards = 0, misses = 0;
  hammer(l, kFrames, reads, torn, backwards, misses);
  std::printf("  · ⑥ 잠금: 프레임 %d · 읽기 %ld · 찢김 %ld · 역행 %ld · 실패(재시도 소진) %ld\n", kFrames, reads, torn, backwards,
              misses);
  chk(reads > 1000, "⑥ 읽는 쪽이 충분히 읽었다");
  chk(torn == 0, "⑥ 찢긴 프레임 0");
  chk(backwards == 0, "⑥ 프레임이 거꾸로 가지 않는다");
  chk(l.read(q) && q[0] == float(kFrames) && q[28] == float(kFrames), "⑥ 마지막 프레임을 읽는다");
  g1::QDesLatch g;
  long r3 = 0, t3 = 0, b3 = 0, m3 = 0;
  hammer(g, 300, r3, t3, b3, m3, 1000);
  std::printf("  · ⑥ 잠금(쓰기 간격 1 ms · 300 프레임): 읽기 %ld · 찢김 %ld · 역행 %ld · 실패 %ld\n", r3, t3, b3, m3);
  chk(t3 == 0 && b3 == 0, "⑥ 쓰기 간격 있을 때도 찢김 · 역행 0");
  l.clear();
  chk(!l.read(q), "⑥ clear 뒤 read = false");
  std::array<float, 29> nine{};
  nine.fill(9.f);
  l.publish(nine.data());
  chk(l.read(q) && q[0] == 9.f && q[28] == 9.f, "⑥ clear 뒤 첫 publish 를 읽는다(앞 프레임이 아니라)");

  NoLatch nl;
  long r2 = 0, t2 = 0, b2 = 0, m2 = 0;
  for (int rep = 0; rep < 5 && t2 == 0; ++rep) hammer(nl, kFrames, r2, t2, b2, m2);
  std::printf("  · ⑥ 대조(잠금 없음): 읽기 %ld · 찢김 %ld\n", r2, t2);
  chk(t2 > 0, "⑥ 대조: 잠금 없이 읽으면 찢김이 보인다(검사가 잡는다)");
}
}  // namespace

int main() {
  Ctx c;
  // R-B6: 시험 시작에 프로세스 메모리를 잠근다(모델 적재보다 앞 — MCL_FUTURE 가 이후 매핑도). 성공했을 때만 폴트 0 을 단언.
  // 한도가 모델·스레드 스택까지 못 덮으면 MCL_FUTURE 가 뒤 mmap/brk 를 실패시켜 혼란스러운 FAIL 이 된다 → 충분할 때만 시도.
  constexpr rlim_t kMemlockNeed = rlim_t(1) << 30;  // 1 GiB
  struct rlimit rl {};
  const bool rl_ok = getrlimit(RLIMIT_MEMLOCK, &rl) == 0;
  const bool enough = rl_ok && (rl.rlim_cur == RLIM_INFINITY || rl.rlim_cur >= kMemlockNeed);
  if (enough) {
    c.mlock_ok = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
    c.mlock_errno = c.mlock_ok ? 0 : errno;
    std::printf("  · mlockall(MCL_CURRENT|MCL_FUTURE): %s\n",
                c.mlock_ok ? "성공" : str("실패 errno=%d %s", c.mlock_errno, std::strerror(c.mlock_errno)).c_str());
  } else {
    c.mlock_errno = rl_ok ? ENOMEM : errno;
    if (rl_ok)
      std::printf("  · memlock 한도 %llu — 잠금 생략, 폴트 단언 생략\n", static_cast<unsigned long long>(rl.rlim_cur));
    else
      std::printf("  · memlock 한도를 읽지 못함(errno=%d) — 잠금 생략, 폴트 단언 생략\n", errno);
  }

  q_des_latch_never_tears();  // 묶음 없이도 돈다
  std::printf("%s q_des_latch_never_tears\n", fail ? "FAIL" : "ok  ");

  const char* b = std::getenv("G1_ARCADE_BUNDLE");
  if (!b || !*b) {
    std::printf("skip test_arcade_host (G1_ARCADE_BUNDLE 없음 — g1_arcade build/bundle_bt5 를 준다)\n");
    return fail ? 1 : 0;
  }
  c.bundle = b;
  std::string err;
  if (!arcade::load_bundle(c.bundle, c.b, err)) {
    std::printf("FAIL load_bundle(%s): %s\n", b, err.c_str());
    return 1;
  }
  for (const arcade::SkillSpec& sp : c.b.setup.skills)
    if (sp.id == "v7") c.default_q = sp.default_q;

  ArcadeHost h;
  struct Case {
    const char* name;
    std::function<void()> fn;
  };
  const Case cases[] = {
      {"opens_bundle_and_warms_sessions", [&] { opens_bundle_and_warms_sessions(c, h); }},
      {"prefills_measured_pose_before_first_tick", [&] { prefills_measured_pose_before_first_tick(c, h); }},
      {"enter_resets_runtime_and_rejects_nonfinite", [&] { enter_resets_runtime_and_rejects_nonfinite(c); }},
      {"tick_allocates_nothing", [&] { tick_allocates_nothing(c, h); }},
      {"first_jump_tick_within_budget", [&] { first_jump_tick_within_budget(c); }},
      {"kp_kd_match_slot", [&] { kp_kd_match_slot(c, h); }},
  };
  for (const Case& k : cases) {
    const int before = fail;
    k.fn();
    std::printf("%s %s\n", fail == before ? "ok  " : "FAIL", k.name);
  }
  std::printf("%s test_arcade_host (실패 %d)\n", fail ? "FAIL" : "ok  ", fail);
  return fail ? 1 : 0;
}
