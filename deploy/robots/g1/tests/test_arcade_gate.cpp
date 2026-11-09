// test_arcade_gate.cpp — 아케이드 경로(G1_ARCADE=1)의 안전 판정(ArcadeGate.h) · 아케이드 전용 FSM config · 한 정책 틱의 할당.
//   ① arcade_orient_gate_follows_core_class — 넘어짐 관문이 core 등급으로 열린다: GroundCapable · 명령 직립 아님 · 기울기 65°
//      (crawl) → 관문 꺼짐 / UprightOnly(걷기 · 점프) → 켜짐 / 브리지(모드 4 클립 행) = 클립 기울기만 / 자세(모드 5) = 명령 직립 ∧
//      최근 섰음. 대조: 옛 g_mode 행(mode1 그대로)이면 crawl 에서 켜진다(= 즉시 Passive 였던 고장)
//   ② arcade_qd_warn_does_not_force_mode — qd_warn 판정 함수의 결과는 Release · Hold · Passive 뿐(«폴백 모드 강제» 없음):
//      걷기(mode1) = Release · UprightOnly 선 자세 = Hold(옛 경로는 g_mode.force) · GroundCapable 낮은 자세 = Passive ·
//      점프 · 브리지(core 가 처분 보류) = Hold
//   ③ 작은 것: G1_ARCADE 값 · joint_ids_map 항등 · mlockall 시도 조건(CAP_IPC_LOCK ∨ 무한 ∨ memlock ≥ VmSize × 1.2 · CapEff 비트 14)
//   ④ arcade_config_from_drops_six_transitions — 실제 config.yaml 을 읽어 ArcadeConfig.h arcade_config_from 결과를 대조(사본 파일 없음 —
//      정본 = config.yaml 하나): 어느 상태에도 Velocity · Mimic_Dance1_subject2 로 가는 전이(리모컨 · 키보드)와 키 v 가 없다 ·
//      Passive(LT+B · p) · f · m 은 남았다 · Passive 에서 닿는 상태 = {Passive, FixStand, Mimic_Masked} · 원본에서 6 전이(시험 쪽
//      독립 목록)만 지운 것과 그 밖 키가 전부 같다 · 원본 노드는 그대로 · 지울 전이가 없는 config · 새 상태에 닿는 config · m 없는
//      config 는 거부 + 이유 · 6 개 밖 위험 전이도 지운다
//   ⑤ (G1_HAVE_ARCADE 빌드만 — CMake 타깃 test_arcade_gate) arcade_link_tick_allocates_nothing — ArcadeLink(묶음 bt5): 기동 거부
//      (상태 칸 open · flock 실패 — U4 · 슬롯 다름 · 항등 아님 · 게인 다름) · G1_ARCADE_STATUS_SHM=0 은 칸 없이 · 기동 요약의 묶음
//      정보(manifest md5 · board · g1_arcade 커밋 · dirty, 없으면 «?» — U5) · enter 의 측정 자세 비유한 = 거부 · enter 직후 run_q_des = 측정 q(0 rad 창 없음) · 한 정책 틱
//      (의도 칸 읽기 → poll → core 등급 · 관문 · qd 판정 → ArcadeHost::tick → 상태 칸 쓰기)의 우리 코드 malloc · calloc · realloc ·
//      operator new · free · delete 0(예열 뒤, 걷기 · Space 점프 포함) — glibc 안쪽(open 등)까지 malloc 수준으로 센다.
//      그 틱들이 기록한 감시 계수기(ArcadeWatchdog.h — last_tick_ns · fail_streak · policy_dead, enter 직후 0)도 대조.
//      ORT Run 안쪽(부른 곳 = libonnxruntime)은 R-B5 대로 따로 세어 찍기만.
//   ⑥ (G1_HAVE_ARCADE 빌드만) arcade_logger_drains_events_in_order — 로그 스레드 본체(ArcadeLogMailbox::run_logger, 정의 =
//      src/ArcadeLink.cpp)를 실제로 돌린다: 안전 사건 4 종(qd_crit · qd_warn Passive/보류 · qd_warn_lowpose) + 창 하나를 넣고
//      running=false 로 마지막 비우기 → SafetyLog CSV 에 사건이 들어온 순서대로 · wall_time = 사건 시각(event_at) · 이름 · 처분 글 ·
//      관절 · 값 · 임계 그대로, spdlog 형식 오류 0(오류 처리기로 센다).
//   g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include test_arcade_gate.cpp -lyaml-cpp -o /tmp/t && /tmp/t   (①~④)
//   ⑤ = cmake -S .. -B /tmp/a -DG1_ARCADE_DIR=/home/piene/g1_arcade_2b && cmake --build /tmp/a --target test_arcade_gate &&
//       G1_ARCADE_BUNDLE=/home/piene/g1_arcade_2b/build/bundle_bt5 /tmp/a/test_arcade_gate   (tests/run_unit_tests.sh 가 돌린다)
#include "ArcadeConfig.h"
#include "ArcadeGate.h"
#if G1_HAVE_ARCADE
#include "ArcadeLink.h"
#include "ArcadeLogMailbox.h"
#include "SafetyLog.h"
#include "arcade_host/bundle_loader.h"  // arcade::md5_file(U5 시험)
#include <spdlog/spdlog.h>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#endif

#include <yaml-cpp/yaml.h>
#if G1_HAVE_ARCADE
#include <fstream>
#include <iterator>
#endif

#include <dlfcn.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <new>
#include <set>
#include <string>
#include <vector>

#ifndef G1_SOURCE_DIR
#define G1_SOURCE_DIR ".."  // run_unit_tests.sh 는 tests/ 에서 돈다
#endif

static int fail = 0;
static void chk(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++fail;
  }
}

// ── 할당 계수(⑤) — malloc 수준. 이 스레드의 계수 구간만, 부른 곳이 libonnxruntime 이면 ORT 안쪽(R-B5)으로 따로. ───────────────
//   operator new 는 __libc_malloc 을 바로 불러 malloc 계수와 겹치지 않는다(new 한 번 = 한 번). glibc 안쪽의 malloc(fopen 의 FILE 등)은
//   malloc 을 가로채야 보인다(T7 수정 1회차). posix_memalign 은 가로채지 않는다 — 우리 코드는 쓰지 않고 ORT 아레나가 쓴다.
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
extern "C" void* __libc_malloc(std::size_t);
extern "C" void* __libc_calloc(std::size_t, std::size_t);
extern "C" void* __libc_realloc(void*, std::size_t);
extern "C" void __libc_free(void*);
namespace {
thread_local bool g_count = false;
thread_local long g_ours = 0, g_ours_free = 0, g_ort = 0;
const void* g_ort_base = nullptr;  // 구간 밖에서 정한다(⑤ 시작)
__attribute__((noinline)) void note(const void* caller, bool is_free) {
  g_count = false;  // dladdr 재진입 막기
  bool ort = false;
  if (g_ort_base && caller) {
    Dl_info i{};
    ort = dladdr(caller, &i) && i.dli_fbase == g_ort_base;
  }
  if (ort) ++g_ort;
  else if (is_free) ++g_ours_free;
  else ++g_ours;
  g_count = true;
}
}  // namespace
extern "C" __attribute__((noinline)) void* malloc(std::size_t n) {
  if (g_count) note(__builtin_return_address(0), false);
  return __libc_malloc(n);
}
extern "C" __attribute__((noinline)) void* calloc(std::size_t a, std::size_t b) {
  if (g_count) note(__builtin_return_address(0), false);
  return __libc_calloc(a, b);
}
extern "C" __attribute__((noinline)) void* realloc(void* p, std::size_t n) {
  if (g_count) note(__builtin_return_address(0), false);
  return __libc_realloc(p, n);
}
extern "C" __attribute__((noinline)) void free(void* p) {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void* operator new(std::size_t n) {
  if (g_count) note(__builtin_return_address(0), false);
  if (void* p = __libc_malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new[](std::size_t n) {
  if (g_count) note(__builtin_return_address(0), false);
  if (void* p = __libc_malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  if (g_count) note(__builtin_return_address(0), false);
  return __libc_malloc(n ? n : 1);
}
__attribute__((noinline)) void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
  if (g_count) note(__builtin_return_address(0), false);
  return __libc_malloc(n ? n : 1);
}
static void* aligned_raw(std::size_t n, std::size_t al) {
  void* p = nullptr;
  return posix_memalign(&p, al < sizeof(void*) ? sizeof(void*) : al, n ? n : 1) == 0 ? p : nullptr;
}
__attribute__((noinline)) void* operator new(std::size_t n, std::align_val_t al) {
  if (g_count) note(__builtin_return_address(0), false);
  if (void* p = aligned_raw(n, std::size_t(al))) return p;
  throw std::bad_alloc();
}
__attribute__((noinline)) void* operator new[](std::size_t n, std::align_val_t al) {
  if (g_count) note(__builtin_return_address(0), false);
  if (void* p = aligned_raw(n, std::size_t(al))) return p;
  throw std::bad_alloc();
}
__attribute__((noinline)) void operator delete(void* p, std::align_val_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete[](void* p, std::align_val_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete(void* p) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete[](void* p) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete(void* p, std::size_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}
__attribute__((noinline)) void operator delete[](void* p, std::size_t) noexcept {
  if (g_count && p) note(__builtin_return_address(0), true);
  __libc_free(p);
}

namespace ag = g1::arcade_gate;
using mode_table::Safety;

// ── ① ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
static ag::CoreSafety core(int mode, int cls, bool cmd_upright, bool holds = false) {
  ag::CoreSafety c;
  c.mode = mode;
  c.safety_class = cls;
  c.commanded_upright = cmd_upright;
  c.holds_qd = holds;
  return c;
}

static void arcade_orient_gate_follows_core_class() {
  // crawl = v7 mode6(GroundCapable · 명령에 자세 없음 → 명령 직립 아님). 골반 65° · z_fk 0.43 (test_mimic 기록)
  const ag::CoreSafety crawl = core(6, 1, false);
  chk(!ag::orient_gate(crawl, false) && !ag::orient_gate(crawl, true), "① crawl(GroundCapable · 명령 직립 아님): 관문 꺼짐");
  // 걷다가 crawl 로 — «섰음» 기억은 crawl 첫 틱에 지워지고(명령이 낮음) 관문은 기울기 65° 동안 계속 꺼져 있다
  g1::safety::RecentHigh rh;
  const ag::CoreSafety walk = core(1, 0, false);
  for (int k = 0; k < 60; ++k) rh.update(0.75f, 3.f, 3.f, ag::memory_upright(walk));
  chk(rh.value() && ag::orient_gate(walk, rh.value()), "① 걷기(UprightOnly): 관문 켜짐 · 섰음 기억 참");
  bool any_open = false;
  for (int k = 0; k < 100; ++k) {
    const float tilt = std::fmin(65.f, 3.f + k * 2.f);
    rh.update(0.43f, tilt, tilt, ag::memory_upright(crawl));
    any_open = any_open || ag::orient_gate(crawl, rh.value());
  }
  chk(!any_open && !rh.value(), "① crawl 100 틱(기울기 → 65°): 관문 한 번도 안 열림 · 기억 지워짐");
  // 대조: 옛 경로 = g_mode 는 mode1 그대로 → UprightOnly 행 → 관문 켜짐(기울기 65° > 57.3° = 즉시 bad_orientation → Passive)
  chk(g1::safety::orientation_check_applies(mode_table::row(1), false, false), "① 대조: 옛 g_mode(mode1) 행이면 crawl 에서도 켜짐");

  for (bool cu : {false, true})
    for (bool r : {false, true}) {
      chk(ag::orient_gate(core(1, 0, cu), r), "① 걷기(mode1 · UprightOnly): 늘 켜짐");
      chk(ag::orient_gate(core(7, 0, cu), r), "① 점프(mode7 표 밖 · UprightOnly): 늘 켜짐");
      chk(ag::orient_gate(core(-1, 0, cu), r), "① 모드 없음 · UprightOnly: 늘 켜짐");
      chk(ag::orient_gate(core(6, 9, cu), r), "① 모르는 등급 = UprightOnly(안전측): 켜짐");
    }
  // 브리지 추종기 = 모드 4 행(클립 · GroundCapable): «최근 섰음» 을 묻지 않는다(g1_ctrl mode4 규칙 그대로) — 클립 골반 기울기만
  chk(ag::orient_gate(core(4, 1, true), false), "① 브리지: 클립이 직립이면 최근 섰음 없이도 켜짐");
  chk(!ag::orient_gate(core(4, 1, false), true), "① 브리지: 클립이 57.3° 넘게 눕히면 꺼짐");
  // 자세(posture i5 = 모드 5 행): 명령 직립 ∧ 최근 섰음
  chk(ag::orient_gate(core(5, 1, true), true), "① 자세: 명령 직립 ∧ 최근 섰음 → 켜짐");
  chk(!ag::orient_gate(core(5, 1, true), false), "① 자세: 최근에 선 적 없음(누운 데서 일어나기) → 꺼짐");
  chk(!ag::orient_gate(core(5, 1, false), true), "① 자세: 명령이 낮음 → 꺼짐");
  // core 등급과 행 등급이 어긋나면(오지 않는다) 등급만으로: mode1 인데 GroundCapable → 명령 직립 ∧ 최근 섰음
  chk(ag::row_of(core(1, 1, true)) == nullptr, "① 행 등급 ≠ core 등급 → 행을 쓰지 않는다");
  chk(!ag::orient_gate(core(1, 1, true), false) && ag::orient_gate(core(1, 1, true), true), "① 어긋남 → core 등급식");
  // «섰음» 기억의 명령 직립
  chk(ag::memory_upright(core(7, 0, false)), "① 기억: UprightOnly 는 명령에 자세가 없어도 직립");
  chk(!ag::memory_upright(core(6, 1, false)) && ag::memory_upright(core(5, 1, true)), "① 기억: GroundCapable = core 명령 직립");
}

// ── ② ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
static void arcade_qd_warn_does_not_force_mode() {
  const mode_table::Row& fb = mode_table::row(1);  // v7 슬롯 폴백 = mode1(ModeRuntime.h slot_fallback_mode)
  chk(ag::qd_warn_verdict(core(1, 0, false), fb, 0.75f, 3.f) == ag::QdWarn::Release, "② 걷기(core 가 폴백 모드로 돌아옴) → 래치 풀기");
  // 옛 경로: UprightOnly 선 자세 = qd_warn_action Fallback → g_mode.force(fallback). 아케이드는 Hold(래치만)
  chk(g1::safety::qd_warn_action(Safety::UprightOnly, 0.75f, 3.f) == g1::safety::QdWarnAction::Fallback, "② 대조: 옛 처분 = 폴백 강제");
  chk(ag::qd_warn_verdict(core(2, 0, false), fb, 0.75f, 3.f) == ag::QdWarn::Hold, "② UprightOnly(mode2) 선 자세 → Hold(모드 강제 없음)");
  chk(ag::qd_warn_verdict(core(7, 0, false, true), fb, 0.3f, 80.f) == ag::QdWarn::Hold, "② 점프(표 밖 · core 보류) → Hold");
  chk(ag::qd_warn_verdict(core(4, 1, true, true), fb, 0.4f, 50.f) == ag::QdWarn::Hold, "② 브리지(core 보류) 낮은 자세 → Hold");
  chk(ag::qd_warn_verdict(core(6, 1, false), fb, 0.43f, 65.f) == ag::QdWarn::Passive, "② crawl 낮은 자세 → Passive");
  chk(ag::qd_warn_verdict(core(5, 1, true), fb, 0.75f, 5.f) == ag::QdWarn::Hold, "② 자세 선 채 → Hold");
  chk(ag::qd_warn_verdict(core(5, 1, false), fb, 0.30f, 5.f) == ag::QdWarn::Passive, "② 자세 낮음(z < 0.65) → Passive");
  chk(ag::qd_warn_verdict(core(5, 1, true), fb, 0.80f, 31.f) == ag::QdWarn::Passive, "② 자세 기움(≥ 30°) → Passive");
  chk(ag::qd_warn_verdict(core(4, 1, true, false), fb, 0.4f, 50.f) == ag::QdWarn::Passive, "② 보류 아닌 GroundCapable 낮은 자세 → Passive");
  // 전 영역: 결과는 세 값 중 하나 · UprightOnly 는 절대 Passive 가 아니다(옛 규칙과 같다) · 걷기(mode1)는 언제나 Release
  int bad = 0;
  for (int m = -1; m <= 8; ++m)
    for (int cls : {0, 1})
      for (bool h : {false, true})
        for (float z : {0.1f, 0.5f, 0.64f, 0.66f, 0.9f})
          for (float t : {0.f, 29.f, 31.f, 70.f}) {
            const ag::QdWarn v = ag::qd_warn_verdict(core(m, cls, false, h), fb, z, t);
            if (v != ag::QdWarn::Release && v != ag::QdWarn::Hold && v != ag::QdWarn::Passive) ++bad;
            if (cls == 0 && v == ag::QdWarn::Passive) ++bad;
            if (mode_table::valid(m) && mode_table::row(m).id == fb.id && v != ag::QdWarn::Release) ++bad;
          }
  chk(bad == 0, "② 전 영역: Release/Hold/Passive 뿐 · UprightOnly 는 Passive 아님 · 폴백 모드면 Release");
  // 자세 버튼 폴백(한 skill 슬롯)은 «직립 유지 도착» 을 모르므로 풀지 않는다(아케이드는 v7 슬롯만 연다)
  chk(ag::qd_warn_verdict(core(5, 1, true), mode_table::row(5), 0.8f, 3.f) == ag::QdWarn::Hold, "② 자세 버튼 폴백 → 풀지 않음");
}

// ── ③ ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
static void small_rules() {
  chk(ag::arcade_request(nullptr) == ag::Request::Off && ag::arcade_request("") == ag::Request::Off &&
          ag::arcade_request("0") == ag::Request::Off,
      "③ G1_ARCADE 없음 · '' · '0' = 끔");
  chk(ag::arcade_request("1") == ag::Request::On, "③ G1_ARCADE=1 = 켬");
  chk(ag::arcade_request("yes") == ag::Request::Bad && ag::arcade_request("2") == ag::Request::Bad &&
          ag::arcade_request(" 1") == ag::Request::Bad,
      "③ 모르는 값 = 기동 거부");
  std::vector<float> id(29);
  for (int i = 0; i < 29; ++i) id[i] = static_cast<float>(i);
  chk(ag::ids_identity(id), "③ 0..28 항등");
  std::vector<float> sw = id;
  std::swap(sw[3], sw[4]);
  chk(!ag::ids_identity(sw), "③ 자리 바뀜 → 항등 아님");
  chk(!ag::ids_identity(std::vector<float>(id.begin(), id.begin() + 28)), "③ 28 개 → 항등 아님");
  const std::uint64_t MB = 1u << 20;
  {  // U2 mlock 판정(최종 검토 I7): CAP_IPC_LOCK ∨ 무한 ∨ soft ≥ VmSize × 1.2 — 기준은 RSS 가 아니라 VmSize(MCL_CURRENT 가 매핑 전체를 잠근다)
    const std::uint64_t vm = 1149 * MB;                            // com1 실측 VmSize(VmRSS 830 MB)
    chk(!ag::mlock_worth_trying(false, 1000 * MB, vm, false), "③ 한도 부족(1000 MB < 1.2×VmSize 1379 MB) → 생략");
    chk(ag::mlock_worth_trying(false, 1400 * MB, vm, false), "③ 1400 MB ≥ 1.2×VmSize → 시도(옛 2×RSS 기준 1660 MB 면 생략이었다)");
    chk(ag::mlock_worth_trying(false, 7732 * MB, vm, false), "③ 한도 충분(com1 memlock 7.5 GB) → 시도");
    chk(ag::mlock_worth_trying(false, vm + vm / 5, vm, false) && !ag::mlock_worth_trying(false, vm + vm / 5 - 1, vm, false),
        "③ 경계: soft = VmSize × 1.2 → 시도 · 1 B 모자라면 생략");
    chk(!ag::mlock_worth_trying(false, 1300 * MB, vm, false), "③ 1300 MB < 1.2×VmSize → 생략(옛 2×RSS 기준은 RSS 600 MB 면 시도 — 실제 잠글 양은 VmSize)");
    chk(ag::mlock_worth_trying(true, 0, vm, false), "③ memlock 무한 → 시도");
    chk(ag::mlock_worth_trying(false, 64 * 1024, vm, true), "③ CAP_IPC_LOCK → 한도(64 KB)와 무관하게 시도");
    chk(ag::mlock_worth_trying(false, 0, 0, true), "③ CAP_IPC_LOCK → VmSize 를 못 읽어도 시도");
    chk(!ag::mlock_worth_trying(false, 7732 * MB, 0, false), "③ VmSize 를 못 읽음(CAP 없음 · 한도 유한) → 생략");
    chk(std::strcmp(ag::mlock_verdict(false, 64 * 1024, vm, true).why, "CAP_IPC_LOCK") == 0 &&
            std::strcmp(ag::mlock_verdict(true, 0, vm, false).why, "memlock unlimited") == 0 &&
            std::strstr(ag::mlock_verdict(false, 1000 * MB, vm, false).why, "<") != nullptr,
        "③ 판정 근거 글(로그)");
    // CapEff(/proc/self/status, 16 진) 비트 14
    std::uint64_t ce = 0, kb = 0;
    chk(ag::proc_status_field("CapEff:\t0000000000004000\n", "CapEff:", 16, ce) && ce == 0x4000 && ag::cap_ipc_lock_in(ce),
        "③ CapEff 0x4000 → CAP_IPC_LOCK");
    chk(ag::proc_status_field("CapEff:\t000001ffffffffff\n", "CapEff:", 16, ce) && ag::cap_ipc_lock_in(ce), "③ CapEff 전부(root) → CAP_IPC_LOCK");
    chk(ag::proc_status_field("CapEff:\t0000000000003fff\n", "CapEff:", 16, ce) && !ag::cap_ipc_lock_in(ce) &&
            ag::proc_status_field("CapEff:\t0000000000000000\n", "CapEff:", 16, ce) && !ag::cap_ipc_lock_in(ce),
        "③ CapEff 0x3fff · 0 → CAP 없음");
    chk(ag::proc_status_field("VmSize:\t 1176632 kB\n", "VmSize:", 10, kb) && kb == 1176632, "③ VmSize kB 줄");
    chk(!ag::proc_status_field("VmRSS:\t 1 kB\n", "VmSize:", 10, kb) && !ag::proc_status_field("CapEff:\t\n", "CapEff:", 16, ce),
        "③ 다른 칸 · 수 없음 → false");
  }
  {  // F1: 거부 때 직전 q — 유한 칸 = 측정값 · 비유한 칸 = default
    float qm[29], qd[29], out[29];
    for (int i = 0; i < 29; ++i) { qm[i] = 0.1f * i + 0.05f; qd[i] = -1.f - i; out[i] = 777.f; }
    qm[3] = std::nanf(""); qm[28] = INFINITY;
    ag::refusal_fallback_q(qm, qd, out);
    bool ok = out[3] == qd[3] && out[28] == qd[28];
    for (int i = 0; i < 29; ++i) if (i != 3 && i != 28) ok = ok && out[i] == qm[i];
    chk(ok, "F1 refusal_fallback_q: 비유한 칸 → default · 유한 칸 → 측정값");
  }
}

// ── ④ ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
// 시험 쪽 독립 목록(ArcadeConfig.h 의 kArcadeRemoved 를 쓰지 않는다 — 함수가 지운 것을 따로 대조): 상태 · 칸 · 대상
struct Tr { const char* st; const char* sect; const char* to; };
static const Tr kSix[] = {{"FixStand", "transitions", "Velocity"},          {"FixStand", "keyboard_transitions", "Velocity"},
                          {"Velocity", "transitions", "Mimic_Dance1_subject2"}, {"Mimic_Dance1_subject2", "transitions", "Velocity"},
                          {"Mimic_Masked", "transitions", "Velocity"},      {"Mimic_Masked", "keyboard_transitions", "Velocity"}};
static bool gone_target(const std::string& t) { return t == "Velocity" || t == "Mimic_Dance1_subject2"; }

// 두 노드가 같은가(스칼라 = 글자, 맵 = 키 집합 · 값, 목록 = 차례).
static bool same(const YAML::Node& a, const YAML::Node& b, const std::string& at, std::string& where) {
  if (a.Type() != b.Type()) { where = at + " (형이 다름)"; return false; }
  switch (a.Type()) {
    case YAML::NodeType::Scalar:
      if (a.Scalar() != b.Scalar()) { where = at + " ('" + a.Scalar() + "' ≠ '" + b.Scalar() + "')"; return false; }
      return true;
    case YAML::NodeType::Sequence:
      if (a.size() != b.size()) { where = at + " (길이)"; return false; }
      for (std::size_t i = 0; i < a.size(); ++i)
        if (!same(a[i], b[i], at + "[" + std::to_string(i) + "]", where)) return false;
      return true;
    case YAML::NodeType::Map: {
      std::set<std::string> ka, kb;
      for (const auto& kv : a) ka.insert(kv.first.as<std::string>());
      for (const auto& kv : b) kb.insert(kv.first.as<std::string>());
      if (ka != kb) { where = at + " (키 집합)"; return false; }
      for (const std::string& k : ka)
        if (!same(a[k], b[k], at + "." + k, where)) return false;
      return true;
    }
    default:
      return true;
  }
}

// 결과 노드의 FSM 에 Velocity · Dance 로 가는 전이 · 키 v 가 몇 개 남았나
static int dangerous_left(const YAML::Node& fsm) {
  int bad = 0;
  for (const auto& st : fsm) {
    const std::string name = st.first.as<std::string>();
    if (name == "_" || !st.second.IsMap()) continue;
    for (const char* sect : {"transitions", "keyboard_transitions"}) {
      const YAML::Node t = st.second[sect];
      if (!t || !t.IsMap()) continue;
      for (const auto& kv : t) {
        if (gone_target(kv.first.as<std::string>())) { std::printf("  남은 전이: %s.%s.%s\n", name.c_str(), sect, kv.first.as<std::string>().c_str()); ++bad; }
        if (std::string(sect) == "keyboard_transitions" && kv.second.IsScalar() && kv.second.Scalar() == "v") ++bad;
      }
    }
  }
  return bad;
}

static void arcade_config_from_drops_six_transitions() {
  namespace ac = g1::arcade_config;
  const std::string path = std::string(G1_SOURCE_DIR) + "/config/config.yaml";
  YAML::Node base;
  try {
    base = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    chk(false, (std::string("④ config.yaml 을 못 읽었다: ") + e.what()).c_str());
    return;
  }
  const YAML::Node before = YAML::Clone(base);  // 원본이 그대로인지 볼 사본
  YAML::Node arc;
  std::string err, removed, where;
  const bool ok = ac::arcade_config_from(base, arc, err, &removed);
  chk(ok, ("④ 실제 config.yaml → 아케이드 설정: " + err).c_str());
  if (!ok) return;
  std::printf("  · ④ 지움 %s\n", removed.c_str());
  chk(same(before, base, "", where), ("④ 원본 노드는 그대로(사본에서 지움): " + where).c_str());
  const YAML::Node fsm = static_cast<const YAML::Node&>(arc)["FSM"];
  chk(dangerous_left(fsm) == 0, "④ 결과에 Velocity · Mimic_Dance1_subject2 로 가는 전이 · 키 v 없음");
  auto is = [&](const char* st, const char* sect, const char* tgt, const char* want) {
    const YAML::Node n = fsm[st][sect][tgt];
    return n && n.IsScalar() && n.Scalar() == want;
  };
  chk(is("Passive", "transitions", "FixStand", "LT + up.on_pressed") && is("Passive", "keyboard_transitions", "FixStand", "f"),
      "④ Passive → FixStand (LT+up · f)");
  chk(is("FixStand", "transitions", "Passive", "LT + B.on_pressed") && is("FixStand", "keyboard_transitions", "Passive", "p") &&
          is("FixStand", "keyboard_transitions", "Mimic_Masked", "m"),
      "④ FixStand → Passive (LT+B · p) · Mimic_Masked (m)");
  chk(is("Mimic_Masked", "transitions", "Passive", "LT + B.on_pressed") && is("Mimic_Masked", "keyboard_transitions", "Passive", "p"),
      "④ Mimic_Masked → Passive (LT+B · p)");
  chk(ac::reachable_from_passive(fsm) == std::set<std::string>({"Passive", "FixStand", "Mimic_Masked"}),
      "④ 닿는 상태 = Passive · FixStand · Mimic_Masked");
  // 원본에서 시험 쪽 목록 6 개(만)를 지운 것 == 함수 결과 (그 밖 키 전부 같음 — FSM._ · 정책 · 클립 · 게인 · imu_cal …)
  YAML::Node want = YAML::Clone(before);
  int present = 0;
  for (const Tr& t : kSix) {
    YAML::Node sec = want["FSM"][t.st][t.sect];
    if (sec && sec.IsMap() && sec[t.to]) { sec.remove(t.to); ++present; }
  }
  chk(present == 6, ("④ config.yaml 에 위험 전이 6 개가 있다 — 실제 " + std::to_string(present)).c_str());
  chk(removed.rfind("6 개:", 0) == 0, ("④ 함수가 지운 수 = 6 — " + removed).c_str());
  where.clear();
  chk(same(want, arc, "", where), ("④ 6 전이 말고는 config.yaml 과 같다: " + where).c_str());

  // 거부: 지울 전이가 원본에 없다(이미 바뀐 config) — 조용히 넘어가지 않는다
  for (const Tr& t : kSix) {
    YAML::Node b2 = YAML::Clone(before);
    YAML::Node sec = b2["FSM"][t.st][t.sect];
    sec.remove(t.to);
    YAML::Node out2 = YAML::Node(42);  // 거부면 손대지 않는다
    std::string e2;
    const std::string name = std::string(t.st) + "." + t.sect + "." + t.to;
    chk(!ac::arcade_config_from(b2, out2, e2) && e2.find(name) != std::string::npos && out2.IsScalar() && out2.as<int>() == 42,
        ("④ " + name + " 이 없는 config → 거부 + 이유(그 전이 이름) · out 그대로").c_str());
  }
  {  // 거부: 지운 뒤 Passive 에서 새 상태에 닿는다(새 위험 전이)
    YAML::Node b3 = YAML::Clone(before);
    b3["FSM"]["FixStand"]["keyboard_transitions"]["Mimic_Dance1_subject3"] = "d";
    std::string e3;
    YAML::Node out3;
    chk(!ac::arcade_config_from(b3, out3, e3) && e3.find("Mimic_Dance1_subject3") != std::string::npos,
        "④ Passive 에서 닿는 새 상태(FixStand d → Mimic_Dance1_subject3) → 거부 + 그 이름");
  }
  {  // 거부: Mimic_Masked end_state 없음(기본 Velocity) · 다른 상태 — 클립 길이 뒤 시간 초과로 빠진다
    for (const char* bad : {"", "Velocity", "FixStand"}) {
      YAML::Node b6 = YAML::Clone(before);
      if (*bad) b6["FSM"]["Mimic_Masked"]["end_state"] = bad;
      else b6["FSM"]["Mimic_Masked"].remove("end_state");
      std::string e6;
      YAML::Node out6 = YAML::Node(42);
      chk(!ac::arcade_config_from(b6, out6, e6) && e6.find("Mimic_Masked.end_state") != std::string::npos && out6.IsScalar(),
          (std::string("④ Mimic_Masked end_state «") + (*bad ? bad : "없음") + "» → 거부 + 이유 · out 그대로").c_str());
    }
  }
  {  // 거부: 아케이드 진입(m)이 없어짐
    YAML::Node b4 = YAML::Clone(before);
    b4["FSM"]["FixStand"]["keyboard_transitions"].remove("Mimic_Masked");
    std::string e4;
    YAML::Node out4;
    chk(!ac::arcade_config_from(b4, out4, e4) && e4.find("{FixStand, Passive}") != std::string::npos,
        "④ FixStand m 없음 → 거부(닿는 상태 {FixStand, Passive})");
  }
  {  // 6 개 밖의 위험 전이(새로 생긴 Passive RT+A → Velocity · 키 v)도 지운다 — 목록에 나온다
    YAML::Node b5 = YAML::Clone(before);
    b5["FSM"]["Passive"]["transitions"]["Velocity"] = "RT + A.on_pressed";
    b5["FSM"]["FixStand"]["keyboard_transitions"]["Passive2"] = "v";
    std::string e5, r5;
    YAML::Node out5;
    const bool ok5 = ac::arcade_config_from(b5, out5, e5, &r5);
    chk(ok5 && dangerous_left(static_cast<const YAML::Node&>(out5)["FSM"]) == 0 && r5.rfind("8 개:", 0) == 0 &&
            r5.find("Passive.transitions.Velocity") != std::string::npos && r5.find("FixStand.keyboard_transitions.Passive2(v)") != std::string::npos,
        ("④ 6 개 밖 위험 전이도 지우고 로그 목록에 — " + (ok5 ? r5 : e5)).c_str());
  }
  {  // FSM 이 없다
    YAML::Node b6 = YAML::Load("a: 1");
    std::string e6;
    YAML::Node out6;
    chk(!ac::arcade_config_from(b6, out6, e6) && e6.find("FSM") != std::string::npos, "④ FSM 없는 노드 → 거부");
  }
}

// ── ⑤ ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
#if G1_HAVE_ARCADE
namespace {
struct FakeData {  // isaaclab::ArticulationData 의 sense_of 가 읽는 칸만
  Eigen::Quaternionf root_quat_w{1, 0, 0, 0};
  Eigen::Vector3f root_ang_vel_b{0, 0, 0};
  Eigen::VectorXf joint_pos = Eigen::VectorXf::Zero(29), joint_vel = Eigen::VectorXf::Zero(29);
};

// 의도 칸을 os.replace 처럼 통째로 바꾼다(쓰는 쪽 — 계수 구간 밖)
bool write_intent(const std::string& path, uint32_t seq, float fwd, uint32_t ev, uint32_t evseq) {
  ArcadeIntentFrame f{};
  f.magic = ARCADE_INTENT_MAGIC;
  f.seq = seq;
  f.fwd = fwd;
  f.events = ev;
  f.event_seq = evseq;
  f.event_age_ms = 0;
  const std::string tmp = path + ".tmp";
  std::FILE* o = std::fopen(tmp.c_str(), "wb");
  if (!o) return false;
  const bool ok = std::fwrite(&f, sizeof(f), 1, o) == 1;
  std::fclose(o);
  return ok && std::rename(tmp.c_str(), path.c_str()) == 0;
}
}  // namespace

static void arcade_link_tick_allocates_nothing() {
  const char* bundle = std::getenv("G1_ARCADE_BUNDLE");
  if (!bundle || !*bundle) {
    std::printf("skip ⑤ arcade_link_tick_allocates_nothing (G1_ARCADE_BUNDLE 없음)\n");
    return;
  }
  // 시험용 경로(실제 /dev/shm/g1_arcade_* 가 아니다)
  const std::string base = "/dev/shm/g1_arcade_gate_test_" + std::to_string(getpid());
  const std::string ipath = base + "_intent", spath = base + "_status";
  setenv("G1_ARCADE_STATUS_SHM", spath.c_str(), 1);
  const std::string npath = base + "_names.json";   // 이름표 파일(Task 3) — 실제 /dev/shm/g1_arcade_names.json 을 건드리지 않는다
  setenv("G1_ARCADE_NAMES", npath.c_str(), 1);

  // 묶음의 처음 role 슬롯 → 그 슬롯 deploy.yaml 의 게인 · joint_ids_map(State_Mimic 이 env->robot->data 에서 읽는 값과 같은 원천)
  std::string slot, err, summary;
  {
    g1::ArcadeHost probe;
    if (!probe.open(bundle, err)) { chk(false, ("⑤ 묶음 열기: " + err).c_str()); return; }
    slot = probe.base_slot();
  }
  chk(!slot.empty(), "⑤ 묶음에 처음 role 슬롯이 있다(bt5 = v7)");
  YAML::Node d;
  try {
    d = YAML::LoadFile(std::string(G1_SOURCE_DIR) + "/config/policy/mimic_masked/" + slot + "/params/deploy.yaml");
  } catch (const std::exception& e) {
    chk(false, (std::string("⑤ 슬롯 deploy.yaml: ") + e.what()).c_str());
    return;
  }
  g1::ArcadeLink::OpenArgs a;
  a.bundle_dir = bundle;
  a.slot_name = slot;
  a.intent_path = ipath;
  const std::vector<float> kp = d["stiffness"].as<std::vector<float>>(), kd = d["damping"].as<std::vector<float>>();
  for (int i = 0; i < 29; ++i) { a.kp[i] = kp[i]; a.kd[i] = kd[i]; }
  a.joint_ids_map = d["joint_ids_map"].as<std::vector<float>>();

  {  // U4(검토 M10): 아케이드 상태 칸(0x6708) 열기 · flock 실패 → 기동 거부 + 이유(묶음을 싣기 전) · 일부러 끔(0)만 칸 없이 뜬다
    g1::arcade_status::ArcadeStatusWriter& sw0 = g1::arcade_status::ArcadeStatusWriter::shared();
    sw0.close_for_test();
    g1::ArcadeLink l;
    setenv("G1_ARCADE_STATUS_SHM", "/nonexistent_g1_arcade_gate_dir/status", 1);
    chk(!l.open(a, err, summary) && err.find("상태 칸(0x6708)") != std::string::npos && err.find("open 실패") != std::string::npos,
        ("U4 상태 칸 open 실패 → 기동 거부 + 이유: " + err).c_str());
    const int fd = ::open(spath.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);  // 다른 쓰는 쪽(flock 은 열린 파일마다 — 같은 프로세스도 막힌다)
    chk(fd >= 0 && ::flock(fd, LOCK_EX | LOCK_NB) == 0, "U4 시험: 다른 쓰는 쪽이 상태 칸 flock");
    setenv("G1_ARCADE_STATUS_SHM", spath.c_str(), 1);
    chk(!l.open(a, err, summary) && err.find("다른 프로세스") != std::string::npos && !sw0.on(),
        ("U4 상태 칸 flock 실패(다른 쓰는 쪽) → 기동 거부 + 이유: " + err).c_str());
    if (fd >= 0) ::close(fd);  // flock 풀림
    setenv("G1_ARCADE_STATUS_SHM", "0", 1);
    g1::ArcadeLink l0;
    chk(l0.open(a, err, summary) && !sw0.on() && summary.find("status (off — G1_ARCADE_STATUS_SHM=0)") != std::string::npos,
        ("U4 G1_ARCADE_STATUS_SHM=0(일부러 끔) → 칸 없이 뜬다 · 로그에 off: " + err).c_str());
    setenv("G1_ARCADE_STATUS_SHM", spath.c_str(), 1);
  }
  {  // U5(검토 M4): 기동 요약에 묶음 manifest md5 · board · g1_arcade 커밋 · dirty(정보만)
    std::string md5;
    chk(arcade::md5_file(std::string(bundle) + "/manifest.json", md5), "U5 시험: manifest md5");
    const YAML::Node m = YAML::LoadFile(std::string(bundle) + "/manifest.json");
    const std::string want = "manifest md5 " + md5 + " · board " + m["board"].Scalar() + " · g1_arcade " + m["g1_arcade_commit"].Scalar() +
                             " · dirty " + m["dirty"].Scalar();
    chk(g1::bundle_manifest_info(bundle) == want, ("U5 묶음 정보 = " + want + " — 실제 " + g1::bundle_manifest_info(bundle)).c_str());
    char tdir[] = "/tmp/g1_arcade_gate_manifest_XXXXXX";
    if (mkdtemp(tdir)) {
      const std::string mp = std::string(tdir) + "/manifest.json";
      if (std::FILE* f = std::fopen(mp.c_str(), "w")) { std::fputs("{\"version\": 1, \"board\": \"bt5\"}\n", f); std::fclose(f); }
      std::string m2;
      arcade::md5_file(mp, m2);
      chk(g1::bundle_manifest_info(tdir) == "manifest md5 " + m2 + " · board bt5 · g1_arcade ? · dirty ?", "U5 없는 칸 → «?»");
      std::remove(mp.c_str());
      ::rmdir(tdir);
    }
    chk(g1::bundle_manifest_info("/nonexistent_g1_arcade_bundle") == "manifest md5 ? · board ? · g1_arcade ? · dirty ?", "U5 manifest 없음 → 전부 «?»");
  }

  {  // 기동 거부
    g1::ArcadeLink l;
    g1::ArcadeLink::OpenArgs b = a;
    b.slot_name = "261103_v1_i5_groundvar_90k";
    chk(!l.open(b, err, summary) && err.find("G1_POLICY_SLOT=" + slot) != std::string::npos, "⑤ 다른 슬롯 → 기동 거부 + 띄울 슬롯");
    b = a;
    std::swap(b.joint_ids_map[0], b.joint_ids_map[1]);
    chk(!l.open(b, err, summary) && err.find("joint_ids_map") != std::string::npos, "⑤ joint_ids_map 항등 아님 → 기동 거부");
    b = a;
    b.kp[3] += 1.f;
    chk(!l.open(b, err, summary) && err.find("kp") != std::string::npos, "⑤ 게인 다름 → 기동 거부");
  }

  g1::ArcadeLink link;
  if (!link.open(a, err, summary)) { chk(false, ("⑤ ArcadeLink::open: " + err).c_str()); return; }
  std::printf("  · open: %s\n", summary.c_str());
  chk(summary.find("manifest md5 ") != std::string::npos && summary.find("status " + spath) != std::string::npos,
      "U5 기동 요약 한 줄에 묶음 정보 · 상태 칸 경로");

  {  // 이름표 파일(Task 3): open 이 한 번 쓴다 — roles · skills = core 표 순서 그대로 · notices 에 code 0 = "" · 상태 칸 배치는 그대로
    std::string md5;
    arcade::md5_file(std::string(bundle) + "/manifest.json", md5);
    nlohmann::json j;
    {
      std::ifstream nf(npath);
      const std::string txt((std::istreambuf_iterator<char>(nf)), std::istreambuf_iterator<char>());
      chk(nlohmann::json::accept(txt), "names_file: 파일이 있고 JSON");
      j = nlohmann::json::parse(txt, nullptr, false);
      if (j.is_discarded()) j = nlohmann::json::object();
    }
    chk(j.value("proto", 0) == 2 && j.value("slot", "") == slot && j.value("bundle", "") == md5, "names_file: proto 2 · slot · bundle md5");
    chk(j.contains("boot_id") && j["boot_id"].is_string() && !j["boot_id"].get<std::string>().empty() &&
            j["boot_id"].get<std::string>().find_first_not_of("0123456789") == std::string::npos,
        "names_file: boot_id = 십진 ns 문자열");
    chk(j.contains("roles") && j["roles"].get<std::vector<std::string>>() == link.host().role_names(), "names_file: roles == role_names()");
    chk(j.contains("skills") && j["skills"].get<std::vector<std::string>>() == link.host().skill_names(), "names_file: skills == skill_names()");
    bool nt = j.contains("notices") && j["notices"].is_object() && (int)j["notices"].size() == arcade::kNoticeCount &&
              j["notices"].value("0", "x") == "";
    for (int c = 0; nt && c < arcade::kNoticeCount; ++c) nt = j["notices"].value(std::to_string(c), "?") == arcade::notice_text(c);
    chk(nt, "names_file: notices 0..kNoticeCount-1 == notice_text(code) · code 0 은 \"\"");
    // "0" → 안 쓴다 · 쓰기 실패(없는 폴더)는 기동을 거부하지 않는다
    std::remove(npath.c_str());
    setenv("G1_ARCADE_NAMES", "0", 1);
    { g1::ArcadeLink l1; std::string e1, s1;
      chk(l1.open(a, e1, s1) && ::access(npath.c_str(), F_OK) != 0, "names_file: G1_ARCADE_NAMES=0 → 안 쓰고 기동 성공"); }
    setenv("G1_ARCADE_NAMES", "/nonexistent_g1_arcade_gate_dir/names.json", 1);
    { g1::ArcadeLink l2; std::string e2, s2;
      chk(l2.open(a, e2, s2) && s2.find("이름표") != std::string::npos, "names_file: 쓰기 실패 → 기동 성공 + 요약에 경고 한 줄"); }
    setenv("G1_ARCADE_NAMES", npath.c_str(), 1);
    std::remove(npath.c_str());
  }

  // 서 있는 상태(슬롯 기본 자세 · 직립 · 속도 0)
  FakeData fd;
  const std::vector<float> dq0 = d["default_joint_pos"].as<std::vector<float>>();
  for (int i = 0; i < 29; ++i) fd.joint_pos[i] = dq0[i];
  g1::ArcadeSense s;
  g1::sense_of(fd, s);
  chk(s.quat_wxyz[0] == 1.f && s.q[3] == dq0[3], "⑤ sense_of: quat wxyz · q 그대로");

  g1::ArcadeSense bad = s;
  bad.q[7] = std::nanf("");
  chk(!link.enter(bad, dq0.data()) && link.refused(), "⑤ 측정 자세 비유한 → enter 거부(refused)");
  {
    std::array<float, 29> qr;
    link.run_q_des(qr);
    bool ok = qr[7] == dq0[7];
    for (int i = 0; i < 29; ++i) if (i != 7) ok = ok && qr[i] == s.q[i];
    chk(ok, "F1 거부 뒤 run_q_des = 유한 칸 측정 · 비유한 칸 default(0 rad 아님)");
  }
  chk(link.enter(s, dq0.data()) && !link.refused(), "⑤ 유효 측정 → enter");
  std::array<float, 29> q0;
  link.run_q_des(q0);
  bool same_q = true;
  for (int i = 0; same_q && i < 29; ++i) same_q = q0[i] == s.q[i];
  chk(same_q, "⑤ enter 직후 run_q_des = 측정 자세(첫 추론 전 0 rad 없음)");
  chk(link.last_tick_ns() == 0 && link.fail_streak() == 0 && !link.policy_dead(g1::arcade_wd::now_ns() + 10'000'000'000LL),
      "⑤ enter 직후 틱 건강 0 — 첫 틱 전엔 멈춤 판정 안 함(ArcadeWatchdog.h)");
  chk(link.fault().kind == g1::arcade_wd::Fault::None, "⑤ G1_ARCADE_FAULT 없음 = 주입 꺼짐");

  const mode_table::Row& fb = mode_table::row(1);
  g1::safety::RecentHigh rh;
  const int jump_role = [&] {
    const auto& r = link.host().role_names();
    for (size_t i = 0; i < r.size(); ++i)
      if (r[i] == "jump") return static_cast<int>(i);
    return -1;
  }();
  // 한 바퀴 = 중립 60 → 걷기 100 → Space(점프, 끈적한 이벤트) 240 = 400 틱. 예열 한 바퀴 뒤 한 바퀴를 센다.
  uint32_t seq = 1000, evseq = 7;
  int jump_ticks = 0, frames = 0, ok_ticks = 0, gate_on = 0;
  long ort_total = 0;
  auto lap = [&](bool count) {
    for (int k = 0; k < 400; ++k) {
      const float fwd = (k >= 60) ? 0.5f : 0.f;
      if (k == 160) ++evseq;
      write_intent(ipath, ++seq, fwd, evseq != 7 ? INTENT_EV_JUMP : 0u, evseq);
      if (count) g_count = true;
      const ag::CoreSafety cs = link.core_safety();
      rh.update(0.75f, 2.f, 2.f, ag::memory_upright(cs));
      const bool gate = ag::orient_gate(cs, rh.value());
      const g1::ArcadeLink::Tick t = link.tick(s);
      const ag::QdWarn v = ag::qd_warn_verdict(link.core_safety(), fb, 0.75f, 2.f);
      link.write_status(t, 6, 2.f, v == ag::QdWarn::Passive, false);
      if (count) {
        g_count = false;
        if (t.out.info.role == jump_role) ++jump_ticks;
        frames += t.frame ? 1 : 0;
        ok_ticks += t.out.ok ? 1 : 0;
        gate_on += gate ? 1 : 0;
      }
    }
  };
  lap(false);
  g_ort_base = [] {
    Dl_info i{};
    return dladdr(reinterpret_cast<const void*>(&OrtGetApiBase), &i) ? i.dli_fbase : nullptr;
  }();
  chk(g_ort_base != nullptr, "⑤ libonnxruntime 위치");
  g_ours = g_ours_free = g_ort = 0;
  lap(true);
  ort_total = g_ort;
  std::printf("  · ⑤ 400 틱: 우리 코드 malloc/new %ld · free/delete %ld · ORT 안쪽(R-B5 제외) %ld · 점프 role 틱 %d · 의도 프레임 %d · "
              "추론 ok %d · 관문 켜짐 %d\n",
              g_ours, g_ours_free, ort_total, jump_ticks, frames, ok_ticks, gate_on);
  chk(g_ours == 0 && g_ours_free == 0, "⑤ 한 정책 틱(의도 칸 → core → 상태 칸) 우리 코드 할당 · 해제 0 (malloc 수준)");
  chk(jump_ticks > 0, "⑤ 측정 구간이 점프 role 을 지났다");
  chk(frames == 400 && ok_ticks == 400 && gate_on == 400, "⑤ 프레임 400 · 추론 ok 400 · 관문(걷기 · 점프 = UprightOnly) 늘 켜짐");
  chk(ort_total > 0, "⑤ 대조: 계수기가 ORT Run 안쪽 할당을 본다(살아 있음)");
  {  // 감시 계수기(같은 실제 틱이 기록했다 — 위 할당 0 에 포함): 성공 틱 뒤 fail 0 · last = 마지막 틱 · 100 ms 넘으면 멈춤
    const int64_t lt = link.last_tick_ns();
    chk(lt > 0 && link.fail_streak() == 0, "⑤ 성공 틱 뒤 last_tick_ns > 0 · fail_streak 0");
    chk(!link.policy_dead(lt + 99'000'000) && link.policy_dead(lt + 100'000'001) &&
            link.dead_verdict(lt + 100'000'001) == g1::arcade_wd::Dead::Stall,
        "⑤ policy_dead: 마지막 틱 뒤 99 ms = 살아 있음 · 100 ms 넘음 = Stall");
  }

  // 상태 칸 한 줄 — 칸 대응(core 표 번호 · 안전층 래치 · 의도)
  g1::arcade_status::ArcadeStatusWriter& sw = g1::arcade_status::ArcadeStatusWriter::shared();
  ArcadeStatus st{};
  chk(sw.on() && g1::arcade_status::read_words(sw.words(), st), "⑤ 상태 칸 읽기");
  chk(st.tick == 800 && st.fsm_state == 6 && st.rearmed == 1 && st.intent_stale == 0 && st.qd_crit == 0,
      "⑤ 상태 칸: tick 800 · fsm 6 · 재무장 · 끊김 없음");
  const arcade::TickInfo inf = link.host().info();
  chk(st.role == inf.role && st.skill == inf.skill && st.mode == inf.mode && st.z_fk == inf.z_fk, "⑤ 상태 칸: role · skill · mode · z_fk = core");
  sw.close_for_test();
  std::remove(ipath.c_str());
  std::remove(spath.c_str());
}

// ── ⑥ ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
static void arcade_logger_drains_events_in_order() {
  const std::string csv = "/tmp/g1_arcade_gate_safety_" + std::to_string(getpid()) + ".csv";
  setenv("G1_SAFETY_CSV", csv.c_str(), 1);
  g1::SafetyLog::shared().open_from_env();
  unsetenv("G1_SAFETY_CSV");
  if (!g1::SafetyLog::shared().enabled()) { chk(false, "⑥ SafetyLog 열기"); return; }
  static int spdlog_errors = 0;
  spdlog::set_error_handler([](const std::string& m) { ++spdlog_errors; std::printf("  · ⑥ spdlog 오류: %s\n", m.c_str()); });
  static const char* const kJ[3] = {"L_knee", "R_hip_pitch", "waist_yaw"};
  struct In { int kind; const char* name; const char* verdict; int joint; float value, limit; double wall; };
  const In ins[4] = {{g1::ArcadeSafetyEvent::QdCrit, "qd_crit", "-> Passive", 3, 21.25f, 19.f, 1700000000.125},
                     {g1::ArcadeSafetyEvent::QdWarnHold, "qd_warn", "-> 처분 보류(아케이드)", 6, 12.5f, 12.f, 1700000000.250},
                     {g1::ArcadeSafetyEvent::QdWarnPassive, "qd_warn", "-> Passive. 복귀=p→f→m", 12, 13.5f, 12.f, 1700000000.375},
                     {g1::ArcadeSafetyEvent::QdWarnLowpose, "qd_warn_lowpose", "-> Passive. 복귀=p→f→m", 3, 14.75f, 12.f, 1700000000.500}};
  g1::ArcadeLogMailbox mb;
  for (int i = 0; i < 4; ++i) {
    g1::ArcadeSafetyEvent e{};
    e.kind = ins[i].kind;
    g1::arcade_copy_text(e.name, ins[i].name);
    g1::arcade_copy_text(e.verdict, ins[i].verdict);
    e.joint = ins[i].joint;
    e.jname = kJ[i % 3];
    e.value = ins[i].value;
    e.limit = ins[i].limit;
    e.over_ticks = 5;
    e.mode = 7;
    e.safety_class = 1;
    e.holds_qd = true;
    e.z_fk = 0.42f;
    e.tilt_deg = 61.f;
    e.wall = ins[i].wall;
    e.mono_ns = g1::arcade_wd::now_ns();
    chk(mb.post_event(e), "⑥ post_event");
  }
  g1::ArcadeLogItem it{};
  std::snprintf(it.diag, sizeof it.diag, "[diag] 50.0Hz  work p50=1.0 (시험)");
  it.bad_ticks = 2;
  it.skipped_ticks = 3;
  it.dropped_event_bits = 0x220u;
  it.dropped_event_count = 1;
  chk(mb.post(it), "⑥ post(창)");
  std::atomic<bool> running{false};   // 이미 내려감 = 한 번 비우고 끝(멈출 때의 마지막 비우기와 같은 길)
  mb.run_logger(running);
  g1::ArcadeLogItem left{};
  g1::ArcadeSafetyEvent left_e{};
  chk(!mb.take(left) && !mb.take_event(left_e), "⑥ run_logger 가 칸 · 고리를 다 비웠다");
  g1::SafetyLog::shared().close();
  std::vector<std::vector<std::string>> rows;
  if (std::FILE* f = std::fopen(csv.c_str(), "r")) {
    char line[512];
    while (std::fgets(line, sizeof line, f)) {
      std::vector<std::string> cols;
      std::string cur;
      for (const char* p = line; *p && *p != '\n'; ++p) {
        if (*p == ',') { cols.push_back(cur); cur.clear(); } else cur.push_back(*p);
      }
      cols.push_back(cur);
      if (cols.size() >= 9 && cols[2] == "edge") rows.push_back(cols);
    }
    std::fclose(f);
  }
  std::remove(csv.c_str());
  chk(rows.size() == 4, "⑥ SafetyLog edge 줄 4 개");
  for (size_t i = 0; i < rows.size() && i < 4; ++i) {
    char wall[32], val[32], lim[32];
    std::snprintf(wall, sizeof wall, "%.6f", ins[i].wall);
    std::snprintf(val, sizeof val, "%.5f", ins[i].value);
    std::snprintf(lim, sizeof lim, "%.5f", ins[i].limit);
    char what[160];
    std::snprintf(what, sizeof what, "⑥ %zu 번째 줄 = 들어온 순서 · wall_time = 사건 시각 · %s · 관절 %d · 값 · 임계 · 처분 글", i + 1, ins[i].name,
                  ins[i].joint);
    chk(rows[i][0] == wall && rows[i][3] == ins[i].name && rows[i][4] == std::to_string(ins[i].joint) && rows[i][5] == kJ[i % 3] &&
            rows[i][6] == val && rows[i][7] == lim && rows[i][8] == ins[i].verdict,
        what);
  }
  chk(spdlog_errors == 0, "⑥ spdlog 형식 오류 0(사건 4 종 · 창 줄)");
  spdlog::set_error_handler(nullptr);
}
#endif

int main() {
  arcade_orient_gate_follows_core_class();
  std::printf("%s arcade_orient_gate_follows_core_class\n", fail ? "FAIL" : "ok  ");
  int f0 = fail;
  arcade_qd_warn_does_not_force_mode();
  std::printf("%s arcade_qd_warn_does_not_force_mode\n", fail > f0 ? "FAIL" : "ok  ");
  f0 = fail;
  small_rules();
  std::printf("%s small_rules (G1_ARCADE · joint_ids_map · mlockall)\n", fail > f0 ? "FAIL" : "ok  ");
  f0 = fail;
  arcade_config_from_drops_six_transitions();
  std::printf("%s arcade_config_from_drops_six_transitions\n", fail > f0 ? "FAIL" : "ok  ");
#if G1_HAVE_ARCADE
  f0 = fail;
  arcade_link_tick_allocates_nothing();
  std::printf("%s arcade_link_tick_allocates_nothing\n", fail > f0 ? "FAIL" : "ok  ");
  f0 = fail;
  arcade_logger_drains_events_in_order();
  std::printf("%s arcade_logger_drains_events_in_order\n", fail > f0 ? "FAIL" : "ok  ");
#else
  std::printf("skip arcade_link_tick_allocates_nothing (아케이드 없는 빌드 — CMake 타깃 test_arcade_gate 가 돈다)\n");
#endif
  std::printf("%s test_arcade_gate (실패 %d)\n", fail ? "FAIL" : "ok  ", fail);
  return fail ? 1 : 0;
}
