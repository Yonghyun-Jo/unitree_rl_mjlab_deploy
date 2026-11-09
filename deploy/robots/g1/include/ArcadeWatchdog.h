#pragma once
// ArcadeWatchdog.h — 아케이드 정책 스레드(State_Mimic::arcade_policy_loop_)의 박자 · 감시 판정. g1 전용(🅐 구역). 순수 함수 · 상수 ·
// 원자 계수기 둘 — spdlog · core · ORT 없이 단위 시험이 돈다(tests/test_arcade_watchdog.cpp).
//
// 왜 있나(2026-11-09 사용자 결정 «멈추면 감시기 + Passive + 몰아치기 없음»): 옛 루프는 `sleep_until(sleepTill); sleepTill += dt;` 라
//   한 번 밀리면 놓친 틱을 연달아 돌렸다(com1 sim2sim: 136 ms 멈춤 뒤 7 틱을 11 ms 안에). 그동안 1 kHz 는 나이 확인 없이 직전
//   q_des 를 계속 냈다. 이제:
//   · next_deadline — 밀리면 «지금 + dt» 로 다시 잡고 놓친 틱 수만 센다(몰아 돌리지 않는다).
//   · TickHealth — 정책 틱이 끝날 때마다 시각(steady_clock ns)과 연속 추론 실패 수를 원자로 둔다. FSM 1 kHz 의 등록된 검사가
//     dead() 를 보고 Passive 로 보낸다(감시용 새 스레드 없음).
//   · FaultInjector — 기동 때 한 번 읽는 G1_ARCADE_FAULT 시험 주입(없으면 꺼짐 — 틱 경로엔 미리 읽은 int 분기뿐).
//
// 🔴 스레드 계약: TickHealth::on_tick = 정책 스레드 하나 · reset = FSM 스레드(정책 스레드가 돌기 전 — std::thread 시작이 순서를
//   맺는다) · last_tick_ns · fail_streak · dead · verdict = 아무 스레드(원자 relaxed 읽기). FaultInjector = 정책 스레드(enter 는 정책
//   스레드 전 FSM 스레드). 전부 할당 · 락 · 시스템 콜 0(steady_clock = vDSO).
#include <atomic>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdlib>

namespace g1 {
namespace arcade_wd {

// 마지막 정책 틱이 끝난 뒤 이만큼 새 틱이 안 끝나면 «멈춤»(100 ms = 50 Hz 정책 5 틱).
constexpr int64_t STALL_LIMIT_NS = 100'000'000;
// 추론 실패(TickOut.ok == false) 연속 이만큼이면 «죽음»(5 틱 = 0.1 s — 그동안 1 kHz 는 직전 q_des).
constexpr int FAIL_LIMIT = 5;

// 지금 [ns] — steady_clock(벽시계가 바뀌어도 안 뛴다). vDSO 라 시스템 콜 아님.
inline int64_t now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 다음 정책 틱의 깨어날 시각. deadline_ns = 이번 틱이 깨어나기로 한 시각, now_ns = 이번 틱 일이 끝난 시각.
//   deadline + dt 가 now 보다 늦으면(엄격히) 그것(제때 — 옛 루프와 같은 절대 박자), 아니면 now + dt 를 돌려주고 놓친 틱 수
//   (now − deadline) / dt 를 skipped 에 더한다 — 밀린 틱을 몰아 돌리지 않는다.
inline int64_t next_deadline(int64_t deadline_ns, int64_t now_ns, int64_t dt_ns, uint32_t& skipped) noexcept {
  if (dt_ns <= 0) return now_ns;
  const int64_t next = deadline_ns + dt_ns;
  if (next > now_ns) return next;
  const int64_t missed = (now_ns - deadline_ns) / dt_ns;
  skipped += missed > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(missed);
  return now_ns + dt_ns;
}

// 마지막 틱(last_tick_ns) 뒤 STALL_LIMIT_NS 를 엄격히 넘었나. last_tick_ns == 0(이 체류에서 아직 첫 틱이 안 끝남) = false.
//   now 가 last 보다 앞(다른 스레드가 now 를 읽은 뒤 정책 스레드가 last 를 새로 씀) = false.
inline bool stalled(int64_t last_tick_ns, int64_t now_ns) noexcept {
  return last_tick_ns != 0 && now_ns - last_tick_ns > STALL_LIMIT_NS;
}

enum class Dead { No, Stall, InferenceFail };
// 사유 하나 — 멈춤이 먼저(멈춘 스레드는 실패 수도 못 센다).
inline Dead verdict(int64_t last_tick_ns, int fail_streak, int64_t now_ns) noexcept {
  if (stalled(last_tick_ns, now_ns)) return Dead::Stall;
  if (fail_streak >= FAIL_LIMIT) return Dead::InferenceFail;
  return Dead::No;
}
// 체류 요약 «종료사유» · SafetyLog 사건 이름.
inline const char* dead_reason(Dead d) noexcept {
  return d == Dead::Stall ? "policy_stall" : d == Dead::InferenceFail ? "inference_fail" : "";
}

// 정책 틱 건강 — 마지막 틱 끝 시각 · 연속 추론 실패 수(둘 다 원자 — FSM 1 kHz 가 읽는다).
class TickHealth {
 public:
  // 체류 시작(FSM 스레드, 정책 스레드 전): 0 · 0 — 첫 틱이 끝나기 전엔 멈춤 판정을 하지 않는다.
  void reset() noexcept {
    last_tick_ns_.store(0, std::memory_order_relaxed);
    fail_streak_.store(0, std::memory_order_relaxed);
  }
  // 정책 스레드 — 틱 하나가 끝났다(q_des 잠금 단계를 지난 뒤). ok = TickOut.ok. 실패 틱도 «틱이 끝났다» 로 센다(멈춤 ≠ 추론 실패).
  void on_tick(bool ok, int64_t now_ns) noexcept {
    const int f = fail_streak_.load(std::memory_order_relaxed);
    fail_streak_.store(ok ? 0 : (f < INT_MAX ? f + 1 : f), std::memory_order_relaxed);
    last_tick_ns_.store(now_ns, std::memory_order_relaxed);
  }
  int64_t last_tick_ns() const noexcept { return last_tick_ns_.load(std::memory_order_relaxed); }
  int fail_streak() const noexcept { return fail_streak_.load(std::memory_order_relaxed); }
  Dead verdict(int64_t now_ns) const noexcept { return arcade_wd::verdict(last_tick_ns(), fail_streak(), now_ns); }
  bool dead(int64_t now_ns) const noexcept { return verdict(now_ns) != Dead::No; }

 private:
  static_assert(std::atomic<int64_t>::is_always_lock_free, "lock-free 원자(1 kHz 가 읽는다)");
  std::atomic<int64_t> last_tick_ns_{0};
  std::atomic<int> fail_streak_{0};
};

// ── 시험 주입 훅 G1_ARCADE_FAULT(V2 — Task 9) ─────────────────────────────────────────────────────────────────────────────
//   "stall:<tick>:<ms>" = 그 정책 틱(체류 안 0 부터)에서 한 번 sleep_for(ms) · "fail:<tick>:<n>" = 그 틱부터 n 틱 TickOut.ok 를 false
//   로(q_des 는 그 틱도 갱신된다 — 판정 시험용). 둘 다 프로세스에 한 번(재진입해 그 틱이 다시 와도 안 한다). 없으면 꺼짐.
struct Fault {
  enum Kind { None, Stall, Fail } kind = None;
  int tick = 0;  // ≥ 0
  int arg = 0;   // stall = ms · fail = 틱 수, ≥ 1
};

// 형식 판정(기동 때 한 번). null · "" = 꺼짐(true). 맞으면 true + out. 틀리면 false + out = 꺼짐(호출부가 기동을 거부한다 — 오타로
// 주입 없이 시험이 도는 것을 막는다). 숫자 = 10 진 · 부호 없음 · 앞뒤 공백 없음 · int 안.
inline bool parse_fault(const char* s, Fault& out) noexcept {
  out = Fault{};
  if (!s || !*s) return true;
  Fault f;
  const char* p = s;
  auto starts = [&](const char* w, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i)
      if (p[i] != w[i]) return false;
    return true;
  };
  if (starts("stall:", 6)) { f.kind = Fault::Stall; p += 6; }
  else if (starts("fail:", 5)) { f.kind = Fault::Fail; p += 5; }
  else return false;
  auto num = [&](int& v, char end) {
    if (*p < '0' || *p > '9') return false;
    long long x = 0;
    while (*p >= '0' && *p <= '9') {
      x = x * 10 + (*p - '0');
      if (x > INT_MAX) return false;
      ++p;
    }
    if (*p != end) return false;
    if (end) ++p;
    v = static_cast<int>(x);
    return true;
  };
  if (!num(f.tick, ':') || !num(f.arg, '\0') || f.arg < 1) return false;
  out = f;
  return true;
}

// 정책 스레드 — 미리 읽은 Fault 를 틱 번호(체류 안 0 부터, ArcadeHost::ticks() — host tick 전 값)에 건다. 꺼짐이면 분기 하나.
struct FaultInjector {
  Fault f{};
  bool fired = false;  // 프로세스에 한 번
  int fail_left = 0;   // 이 체류에서 남은 강제 실패 틱
  // 체류 시작(FSM 스레드, 정책 스레드 전) — 남은 실패 창을 새 체류로 넘기지 않는다.
  void enter() noexcept { fail_left = 0; }
  // 틱 시작: 이 틱에 잘 ms(0 = 없음).
  int stall_ms(int k) noexcept {
    if (f.kind != Fault::Stall || fired || k != f.tick) return 0;
    fired = true;
    return f.arg;
  }
  // 틱 끝: 덮어쓴 ok.
  bool filter_ok(int k, bool ok) noexcept {
    if (f.kind != Fault::Fail) return ok;
    if (!fired && k == f.tick) {
      fired = true;
      fail_left = f.arg;
    }
    if (fail_left > 0) {
      --fail_left;
      return false;
    }
    return ok;
  }
};

}  // namespace arcade_wd
}  // namespace g1
