// test_arcade_watchdog.cpp — 아케이드 정책 스레드의 감시(ArcadeWatchdog.h) · 로그 우편함(ArcadeLogMailbox.h). 헤더 온리 — 아케이드
// (g1_arcade core · ORT) 없이 돈다. ArcadeLink 에 붙은 같은 계수기(last_tick_ns · fail_streak · policy_dead)의 실제 틱 대조와
// 할당 0 은 test_arcade_gate ⑤(아케이드 켬 CMake 갈래)가 한다.
//   ① next_deadline_on_time        — 제때 끝난 틱: 다음 깨어날 시각 = 이번 예정 + dt, 놓친 틱 0
//   ② next_deadline_no_catch_up    — 136 ms 멈춤 뒤: 밀린 틱을 몰아 돌리지 않는다(다음 = now + dt) · 놓친 틱 6
//   ③ watchdog_not_armed_before_first_tick — 이 체류에서 첫 틱이 안 끝났으면(last 0) 멈춤 판정 안 함
//   ④ stalled_after_limit          — 마지막 틱 뒤 100 ms 까지는 살아 있음, 그 뒤는 멈춤
//   ⑤ fail_streak_trips_at_five    — 추론 실패(TickOut.ok false) 4 연속 = 살아 있음, 5 번째 = 죽음(inference_fail), 성공 한 번 = 0
//   ⑥ mailbox_post_full_drops      — 비우지 않은 칸에 두 번째 post = false · 첫 항목 그대로 · 버린 수 1
//   ⑦ mailbox_post_rt_safe         — post · post_event(정책 스레드 쪽)가 할당 0 · read/write 계열 syscall 0
//   ⑨ mailbox_event_ring_order_and_drop — 안전 사건 고리: 들어온 순서 · 16 칸이 차면 버림 + 계수 · 넘침(wrap) · 고정 칸 글 자르기
//   ⑩ mailbox_event_ring_spsc_threads   — 두 스레드(정책 · 로그 흉내)가 동시에 20 만 건(찬 고리면 보내는 쪽이 다시 시도 — 모두 건넌다):
//      빠짐 · 겹침 없는 연속 순서 · 칸 찢김 없음
//   ⑧ fault_hook_parse_and_apply   — G1_ARCADE_FAULT 형식(stall:<tick>:<ms> · fail:<tick>:<n>) · 적용(프로세스에 한 번)
//   g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include -pthread test_arcade_watchdog.cpp -o /tmp/t && /tmp/t
#include "ArcadeLogMailbox.h"
#include "ArcadeWatchdog.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <thread>

static int fail = 0;
static void chk(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++fail;
  }
}

// ── 할당 계수(⑦) — 이 스레드가 켠 동안의 operator new · malloc 수 ────────────────────────────────────────────────────────
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
static thread_local bool g_count = false;
static std::atomic<long> g_allocs{0};
void* operator new(std::size_t n) {
  if (g_count) g_allocs.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// 이 스레드의 write · read 계열 syscall 누적 수(/proc/thread-self/io 의 syscw · syscr). 못 읽으면 −1.
static void io_pair(long& w, long& r) {
  w = r = -1;
  std::FILE* f = std::fopen("/proc/thread-self/io", "r");
  if (!f) return;
  char k[64];
  long v = 0;
  while (std::fscanf(f, "%63s %ld", k, &v) == 2) {
    if (std::strcmp(k, "syscr:") == 0) r = v;
    if (std::strcmp(k, "syscw:") == 0) w = v;
  }
  std::fclose(f);
}

namespace wd = g1::arcade_wd;

static constexpr int64_t MS = 1'000'000;

static void next_deadline_on_time() {
  uint32_t skipped = 0;
  const int64_t n = wd::next_deadline(100 * MS, 105 * MS, 20 * MS, skipped);
  chk(n == 120 * MS, "① 다음 깨어날 시각 = 100 + 20 = 120 ms");
  chk(skipped == 0, "① 놓친 틱 0");
}

static void next_deadline_no_catch_up() {
  uint32_t skipped = 0;
  const int64_t n = wd::next_deadline(100 * MS, 236 * MS, 20 * MS, skipped);
  chk(n == 256 * MS, "② 136 ms 멈춤 뒤 다음 = now + dt = 256 ms (밀린 120..220 을 몰아 돌리지 않는다)");
  chk(skipped == 6, "② 놓친 틱 = (236 − 100) / 20 = 6");
  // 누적: 창 안에서 두 번 밀리면 더한다
  const int64_t n2 = wd::next_deadline(256 * MS, 300 * MS, 20 * MS, skipped);
  chk(n2 == 320 * MS && skipped == 8, "② 두 번째 밀림(256 → 300) — 다음 320 · 놓친 틱 누적 6 + 2 = 8");
  // 경계: 끝난 시각이 다음 예정과 같으면(늦지 않았으면) 그것을 돌려주지 않는다 — «늦으면» 은 엄격히 뒤
  uint32_t s3 = 0;
  const int64_t n3 = wd::next_deadline(100 * MS, 120 * MS, 20 * MS, s3);
  chk(n3 == 140 * MS && s3 == 1, "② 경계: now = 예정 + dt → 다음 now + dt · 놓친 틱 1");
}

static void watchdog_not_armed_before_first_tick() {
  chk(!wd::stalled(0, 10'000'000'000), "③ last 0(첫 틱 전) = 멈춤 판정 안 함");
  wd::TickHealth h;
  h.reset();
  chk(h.last_tick_ns() == 0 && h.fail_streak() == 0, "③ reset 뒤 last 0 · fail 0");
  chk(!h.dead(10'000'000'000) && h.verdict(10'000'000'000) == wd::Dead::No, "③ reset 뒤 아무리 지나도 죽음 아님");
}

static void stalled_after_limit() {
  static_assert(wd::STALL_LIMIT_NS == 100'000'000, "100 ms(틱 끝 → 틱 끝) — 느린 틱 하나의 실제 예산은 ≈ 100 − dt ≈ 80 ms");
  chk(!wd::stalled(1'000'000'000, 1'099'000'000), "④ 99 ms = 살아 있음");
  chk(!wd::stalled(1'000'000'000, 1'100'000'000), "④ 정확히 100 ms = 살아 있음(엄격히 넘어야)");
  chk(wd::stalled(1'000'000'000, 1'100'000'001), "④ 100 ms + 1 ns = 멈춤");
  chk(!wd::stalled(1'000'000'000, 990'000'000), "④ now 가 last 보다 앞(읽는 순서 엇갈림) = 살아 있음");
  wd::TickHealth h;
  h.reset();
  h.on_tick(true, 1'000'000'000);
  chk(!h.dead(1'099'000'000), "④ TickHealth: 성공 틱 뒤 99 ms = 살아 있음");
  chk(h.dead(1'100'000'001) && h.verdict(1'100'000'001) == wd::Dead::Stall, "④ TickHealth: 100 ms 넘음 = Stall");
  chk(std::strcmp(wd::dead_reason(wd::Dead::Stall), "policy_stall") == 0, "④ 사유 이름 policy_stall");
}

static void fail_streak_trips_at_five() {
  static_assert(wd::FAIL_LIMIT == 5, "연속 추론 실패 5 틱 = 0.1 s");
  wd::TickHealth h;
  h.reset();
  int64_t t = 1'000'000'000;
  for (int i = 0; i < 4; ++i) h.on_tick(false, t += 20 * MS);  // 가짜 Tick 결과: ok false 4 번
  chk(h.fail_streak() == 4, "⑤ 실패 4 연속 → fail_streak 4");
  chk(!h.dead(t + MS), "⑤ 실패 4 연속 = 살아 있음");
  h.on_tick(false, t += 20 * MS);
  chk(h.fail_streak() == 5, "⑤ 실패 5 연속 → fail_streak 5");
  chk(h.dead(t + MS) && h.verdict(t + MS) == wd::Dead::InferenceFail, "⑤ 실패 5 연속 = 죽음(InferenceFail)");
  chk(std::strcmp(wd::dead_reason(wd::Dead::InferenceFail), "inference_fail") == 0, "⑤ 사유 이름 inference_fail");
  chk(h.last_tick_ns() == t, "⑤ 실패 틱도 «틱이 끝났다» 는 센다(last = 마지막 틱 시각) — 멈춤과 추론 실패를 가른다");
  h.on_tick(true, t += 20 * MS);
  chk(h.fail_streak() == 0 && !h.dead(t + MS), "⑤ 성공 한 번 → 0 · 살아 있음");
  // 멈춤이 겹치면 Stall 이 먼저(사유 하나)
  for (int i = 0; i < 5; ++i) h.on_tick(false, t += 20 * MS);
  chk(h.verdict(t + 200 * MS) == wd::Dead::Stall, "⑤ 실패 5 연속 ∧ 200 ms 멈춤 → 사유 Stall");
  h.reset();
  chk(h.fail_streak() == 0 && h.last_tick_ns() == 0, "⑤ reset(새 체류) = 0");
}

static g1::ArcadeLogItem item_with(const char* diag, int bad) {
  g1::ArcadeLogItem it{};
  std::snprintf(it.diag, sizeof it.diag, "%s", diag);
  it.bad_ticks = bad;
  it.skipped_ticks = 6;
  it.overrun = true;
  it.dropped_event_bits = 0x220u;
  it.dropped_event_count = 3;
  return it;
}

static void mailbox_post_full_drops() {
  g1::ArcadeLogMailbox mb;
  const g1::ArcadeLogItem a = item_with("[diag] first", 1);
  const g1::ArcadeLogItem b = item_with("[diag] second", 2);
  chk(mb.post(a), "⑥ 빈 칸에 post = true");
  chk(!mb.post(b), "⑥ 비우지 않은 칸에 두 번째 post = false(버림)");
  chk(mb.posts_dropped() == 1, "⑥ 버린 수 1");
  g1::ArcadeLogItem out{};
  chk(mb.take(out), "⑥ take = true");
  chk(std::strcmp(out.diag, "[diag] first") == 0 && out.bad_ticks == 1 && out.skipped_ticks == 6 && out.overrun &&
          out.dropped_event_bits == 0x220u && out.dropped_event_count == 3,
      "⑥ 첫 항목 그대로(두 번째가 덮지 않았다)");
  chk(!mb.take(out), "⑥ 비운 뒤 take = false");
  chk(mb.post(b) && mb.take(out) && std::strcmp(out.diag, "[diag] second") == 0, "⑥ 비운 뒤 다시 post · take");
}

static g1::ArcadeSafetyEvent event_with(int seq) {
  g1::ArcadeSafetyEvent e{};
  e.kind = seq % 4;
  g1::arcade_copy_text(e.name, seq % 2 ? "qd_warn" : "qd_crit");
  g1::arcade_copy_text(e.verdict, "-> Passive. 복귀=p→f→m");
  e.joint = seq % 29;
  e.value = static_cast<float>(seq);
  e.limit = static_cast<float>(seq) * 2.f;
  e.mono_ns = seq;
  e.wall = static_cast<double>(seq) * 0.5;
  return e;
}

static void mailbox_post_rt_safe() {
  g1::ArcadeLogMailbox mb;
  const g1::ArcadeLogItem a = item_with("[diag] rt", 0);
  const g1::ArcadeSafetyEvent ev = event_with(7);
  g1::ArcadeSafetyEvent ev_out{};
  g1::ArcadeLogItem out{};
  long w0, r0, w1, r1, wb0, rb0, wb1, rb1;
  io_pair(wb0, rb0);  // 재는 일 자체(fopen · fscanf)의 syscall 기준선
  io_pair(wb1, rb1);
  const long base_w = wb1 - wb0, base_r = rb1 - rb0;
  io_pair(w0, r0);
  g_count = true;
  int ok = 0, dropped = 0, ev_ok = 0, ev_dropped = 0;
  for (int i = 0; i < 1000; ++i) {
    if (mb.post(a)) ++ok; else ++dropped;  // 정책 스레드 쪽만 잰다(가득 찬 칸 = 버림 경로도 지난다)
    if (mb.post(a)) ++ok; else ++dropped;
    for (int k = 0; k < 20; ++k)            // 사건 고리 16 칸 + 넘침 4(버림 경로)
      if (mb.post_event(ev)) ++ev_ok; else ++ev_dropped;
    g_count = false;
    (void)mb.take(out);                     // 로그 스레드 쪽(재지 않는다)
    while (mb.take_event(ev_out)) {}
    g_count = true;
  }
  g_count = false;
  io_pair(w1, r1);
  const bool io_ok = w0 >= 0 && r0 >= 0 && w1 >= 0 && r1 >= 0;
  const long dw = io_ok ? (w1 - w0) - base_w : 1000000, dr = io_ok ? (r1 - r0) - base_r : 1000000;
  std::printf("  · ⑦ post 2000 번(실음 %d · 버림 %d) + post_event 20000 번(실음 %d · 버림 %d): 할당 %ld · write syscall %ld · "
              "read syscall %ld\n",
              ok, dropped, ev_ok, ev_dropped, g_allocs.load(), dw, dr);
  chk(ok == 1000 && dropped == 1000, "⑦ 비운 뒤 첫 post 는 실리고 둘째는 버려진다");
  chk(ev_ok == 16000 && ev_dropped == 4000 && mb.events_dropped() == 4000, "⑦ 사건 고리 16 칸까지 실리고 넘침은 버림 + 계수");
  chk(g_allocs.load() == 0, "⑦ post 할당 0");
  chk(dw <= 0 && dr <= 0, "⑦ post read/write syscall 0");
  static_assert(std::atomic<int>::is_always_lock_free, "칸 상태는 lock-free 원자");
}

static void mailbox_event_ring_order_and_drop() {
  g1::ArcadeLogMailbox mb;
  g1::ArcadeSafetyEvent out{};
  chk(!mb.take_event(out), "⑨ 빈 고리 take_event = false");
  for (int i = 0; i < 3; ++i) chk(mb.post_event(event_with(i)), "⑨ 빈 고리에 post_event = true");
  bool order = true;
  for (int i = 0; i < 3; ++i)
    order = order && mb.take_event(out) && out.value == float(i) && out.kind == i % 4 && out.joint == i % 29 &&
            std::strcmp(out.name, i % 2 ? "qd_warn" : "qd_crit") == 0 && std::strcmp(out.verdict, "-> Passive. 복귀=p→f→m") == 0;
  chk(order && !mb.take_event(out), "⑨ 들어온 순서 · 칸 그대로 · 다 비면 false");
  static_assert(g1::ArcadeLogMailbox::kEventCap == 16, "고리 16 칸");
  for (int i = 0; i < 16; ++i) chk(mb.post_event(event_with(100 + i)), "⑨ 16 칸까지 실린다");
  chk(!mb.post_event(event_with(999)) && mb.events_dropped() == 1, "⑨ 17 번째 = 버림 · 계수 1");
  bool full_order = true;
  for (int i = 0; i < 16; ++i) full_order = full_order && mb.take_event(out) && out.value == float(100 + i);
  chk(full_order && !mb.take_event(out), "⑨ 찬 고리를 순서대로 다 비운다(버린 999 는 없다)");
  // 넘침(wrap): 5 넣고 5 빼기를 많이 — 인덱스가 칸 수를 수천 번 돈다
  bool wrap = true;
  int seq = 0, got = 0;
  for (int c = 0; c < 4000 && wrap; ++c) {
    for (int k = 0; k < 5; ++k) wrap = wrap && mb.post_event(event_with(seq++));
    for (int k = 0; k < 5; ++k) wrap = wrap && mb.take_event(out) && out.value == float(got++);
  }
  chk(wrap && got == 20000, "⑨ 2 만 건 넘침 — 순서 · 값 그대로");
  // 고정 칸 글 자르기: 넘치는 글은 잘리고 늘 NUL
  g1::ArcadeSafetyEvent e{};
  g1::arcade_copy_text(e.name, "0123456789012345678901234567890123456789");
  chk(std::strlen(e.name) == sizeof(e.name) - 1 && e.name[sizeof(e.name) - 1] == '\0', "⑨ 긴 이름 = 23 자로 잘림 · NUL");
  g1::arcade_copy_text(e.name, nullptr);
  chk(e.name[0] == '\0', "⑨ nullptr = 빈 글");
}

static void mailbox_event_ring_spsc_threads() {
  g1::ArcadeLogMailbox mb;
  constexpr int kN = 200000;
  std::atomic<bool> done{false};
  int received = 0, last = -1;
  bool ok = true;
  std::thread consumer([&] {
    g1::ArcadeSafetyEvent out{};
    for (;;) {
      const bool fin = done.load(std::memory_order_acquire);
      while (mb.take_event(out)) {
        const int s = static_cast<int>(out.value);
        // 연속 순서(빠짐 · 겹침 없음) · 찢김 없음(칸끼리 같은 seq 에서 나온 값)
        ok = ok && s == last + 1 && out.limit == float(s) * 2.f && out.mono_ns == s && out.joint == s % 29 && out.kind == s % 4 &&
             out.wall == double(s) * 0.5;
        last = s;
        ++received;
      }
      if (fin) break;
    }
  });
  long full = 0;
  for (int i = 0; i < kN; ++i) {
    const g1::ArcadeSafetyEvent e = event_with(i);
    while (!mb.post_event(e)) {   // 찬 고리 = 받는 쪽이 비울 때까지 다시(정책 스레드는 이러지 않는다 — 시험만)
      ++full;
      std::this_thread::yield();
    }
  }
  done.store(true, std::memory_order_release);
  consumer.join();
  std::printf("  · ⑩ 보냄 %d · 받음 %d · 찬 고리 만남 %ld 번\n", kN, received, full);
  chk(ok, "⑩ 연속 순서(빠짐 · 겹침 없음) · 칸 찢김 없음");
  chk(received == kN, "⑩ 받은 수 = 보낸 수");
}

static void fault_hook_parse_and_apply() {
  wd::Fault f;
  chk(wd::parse_fault(nullptr, f) && f.kind == wd::Fault::None, "⑧ 없음 = 꺼짐(형식 ok)");
  chk(wd::parse_fault("", f) && f.kind == wd::Fault::None, "⑧ 빈 문자열 = 꺼짐");
  chk(wd::parse_fault("stall:300:150", f) && f.kind == wd::Fault::Stall && f.tick == 300 && f.arg == 150, "⑧ stall:300:150");
  chk(wd::parse_fault("fail:300:5", f) && f.kind == wd::Fault::Fail && f.tick == 300 && f.arg == 5, "⑧ fail:300:5");
  const char* bad[] = {"stall", "stall:300", "stall:300:", "stall:-1:150", "stall:300:0", "fail:3x0:5", "fail:300:5:1",
                       "hang:300:5", "stall:300:150 ", "fail:300:99999999999"};
  for (const char* s : bad) {
    wd::Fault g;
    g.kind = wd::Fault::Stall;
    char what[96];
    std::snprintf(what, sizeof what, "⑧ 틀린 형식 '%s' = false · 꺼짐", s);
    chk(!wd::parse_fault(s, g) && g.kind == wd::Fault::None, what);
  }
  // stall: 그 틱에서 한 번만(프로세스에 한 번 — 재진입해 다시 그 틱이 와도 안 잔다)
  {
    wd::FaultInjector inj;
    wd::parse_fault("stall:3:150", inj.f);
    int slept = 0, at = -1;
    for (int k = 0; k < 10; ++k)
      if (const int ms = inj.stall_ms(k)) { slept += ms; at = k; }
    inj.enter();
    for (int k = 0; k < 10; ++k) slept += inj.stall_ms(k);
    chk(slept == 150 && at == 3, "⑧ stall:3:150 — 틱 3 에서 150 ms 한 번(재진입에도 다시 안 함)");
    for (int k = 0; k < 10; ++k) chk(inj.filter_ok(k, true), "⑧ stall 훅은 ok 를 건드리지 않는다");
  }
  // fail: 그 틱부터 n 틱 ok = false, 한 번만 · 새 체류(enter)는 남은 창을 이어 가지 않는다
  {
    wd::FaultInjector inj;
    wd::parse_fault("fail:3:5", inj.f);
    int forced = 0, first = -1, last = -1;
    for (int k = 0; k < 20; ++k)
      if (!inj.filter_ok(k, true)) { ++forced; if (first < 0) first = k; last = k; }
    chk(forced == 5 && first == 3 && last == 7, "⑧ fail:3:5 — 틱 3..7 ok false");
    inj.enter();
    for (int k = 0; k < 20; ++k) chk(inj.filter_ok(k, true), "⑧ fail 은 한 번만(재진입에 다시 안 함)");
    wd::FaultInjector cut;
    wd::parse_fault("fail:3:10", cut.f);
    for (int k = 0; k < 6; ++k) (void)cut.filter_ok(k, true);  // 3 · 4 · 5 실패 뒤 Passive → 새 체류
    cut.enter();
    bool all_ok = true;
    for (int k = 0; k < 20; ++k) all_ok = all_ok && cut.filter_ok(k, true);
    chk(all_ok, "⑧ 체류가 중간에 끝나면 남은 실패 틱을 새 체류로 넘기지 않는다");
    for (int k = 0; k < 4; ++k) chk(cut.stall_ms(k) == 0, "⑧ fail 훅은 자지 않는다");
  }
  // 꺼짐 = 아무것도 안 한다
  {
    wd::FaultInjector off;
    bool clean = true;
    for (int k = 0; k < 1000; ++k) clean = clean && off.stall_ms(k) == 0 && off.filter_ok(k, true) && !off.filter_ok(k, false);
    chk(clean, "⑧ 꺼짐 = ok 그대로 · 잠 0");
  }
}

int main() {
  struct T {
    void (*fn)();
    const char* name;
  } const tests[] = {
      {next_deadline_on_time, "next_deadline_on_time"},
      {next_deadline_no_catch_up, "next_deadline_no_catch_up"},
      {watchdog_not_armed_before_first_tick, "watchdog_not_armed_before_first_tick"},
      {stalled_after_limit, "stalled_after_limit"},
      {fail_streak_trips_at_five, "fail_streak_trips_at_five"},
      {mailbox_post_full_drops, "mailbox_post_full_drops"},
      {mailbox_post_rt_safe, "mailbox_post_rt_safe"},
      {mailbox_event_ring_order_and_drop, "mailbox_event_ring_order_and_drop"},
      {mailbox_event_ring_spsc_threads, "mailbox_event_ring_spsc_threads"},
      {fault_hook_parse_and_apply, "fault_hook_parse_and_apply"},
  };
  for (const T& t : tests) {
    const int f0 = fail;
    t.fn();
    std::printf("%s %s\n", fail > f0 ? "FAIL" : "ok  ", t.name);
  }
  std::printf("%s test_arcade_watchdog (실패 %d)\n", fail ? "FAIL" : "ok  ", fail);
  return fail ? 1 : 0;
}
