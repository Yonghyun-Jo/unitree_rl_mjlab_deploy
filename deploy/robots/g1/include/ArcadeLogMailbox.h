#pragma once
// ArcadeLogMailbox.h — 아케이드 정책 스레드 → 로그 스레드 한 칸 우편함. g1 전용(🅐 구역).
//
// 왜 있나(2026-11-09 사용자 결정 «로그는 정책 스레드 밖»): 정책 스레드가 1 Hz 로 직접 부르던 spdlog(동기 — stdout · 회전 파일,
//   deploy/include/param.h)와 SafetyLog::sample(fflush)은 터미널 · 디스크가 느린 순간 그 틱을 밀리초 단위로 잡아먹는다. 이제 정책
//   스레드는 창(1 s)이 닫힐 때 숫자 · 문자열을 이 칸에 «넣기만» 하고, 보통 우선순위 로그 스레드가 100 ms 마다 칸을 비워 찍는다.
//   (LoopDiag::publish/take 와 같은 SPSC 한 칸 — 그쪽은 문자열만이라 여기서 항목을 넓혔다. LoopDiag.h 는 🅑 라 건드리지 않는다.)
//
//   정책 스레드의 안전 사건(arcade_qd_guard_ 의 qd_crit · qd_warn 래치 — 옛날엔 그 자리에서 spdlog + SafetyLog::event(fprintf ·
//   fflush)를 불렀다)도 같은 이유로 고정 크기 SPSC 고리(kEventCap 칸)에 넣고, 로그 스레드가 들어온 순서대로 찍는다. 100 ms 감시기
//   (ArcadeWatchdog.h)가 붙은 뒤로는 느린 stdout · eMMC 쓰기 한 번이 거짓 policy_stall → Passive 가 될 수 있다.
//
// 🔴 스레드 계약: post · post_event = 정책 스레드 하나(할당 · 락 · 시스템 콜 0 — 원자 정수 + 고정 칸 복사). take · take_event ·
//   run_logger = 로그 스레드 하나. wake = 멈추는 쪽(FSM 스레드 — exit 에서 running 을 내린 뒤). 칸 · 고리가 차 있으면(로그 스레드가
//   못 돎) 버리고 false(버린 수는 원자 계수 — 로그 스레드가 다음 줄에 찍는다).
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <type_traits>

namespace g1 {

// 정책 창(1 s) 하나의 로그 재료 — 정책 스레드가 창이 닫힐 때 채운다. 평범한 값만(복사 = memcpy).
struct ArcadeLogItem {
  char diag[768];               // LoopDiag::format 결과 두 줄(«[diag] » 로 시작)
  bool overrun;                 // 이 창에 work > 예산 틱이 있었나(warn 으로 찍는다)
  int bad_ticks;                // TickOut.ok == false 틱 수(추론 실패 · 비유한 q_des — 그 틱은 q_des 를 안 바꿨다)
  uint32_t skipped_ticks;       // 밀려서 건너뛴 정책 틱 수(arcade_wd::next_deadline — 몰아 돌리지 않았다)
  // SafetyLog::sample 7 값(1 kHz 모니터 누적 — 정책 스레드에서 읽은 그대로) + 관절 이름 함수
  uint32_t clamp_ticks;
  float clamp_max;
  int clamp_joint;
  uint32_t rate_ticks;
  float rate_max;
  int rate_joint;
  float tilt_deg;
  const char* (*jname)(int);
  // 실기에서 안 먹는(INTENT_EV_REAL 밖) 이벤트를 버린 기록 — 이 창의 비트 OR · 횟수(ArcadeLink 창 계수, 정책 스레드에서 읽음)
  uint32_t dropped_event_bits;
  uint32_t dropped_event_count;
};
static_assert(std::is_trivially_copyable<ArcadeLogItem>::value, "post = 고정 칸 복사(할당 없음)");

// 정책 스레드의 안전 사건 한 건(State_Mimic::arcade_qd_guard_ 의 래치 에지). 평범한 값만 — 이름 · 처분 글은 고정 크기 칸(정책 스레드가
// 문자열 상수에서 잘라 복사), 관절 이름은 정적 표(G1_JOINT_NAME — 프로세스 수명)의 포인터. kind 는 로그 스레드가 콘솔 줄 모양을 고른다.
struct ArcadeSafetyEvent {
  enum Kind : int32_t { QdCrit = 0, QdWarnPassive = 1, QdWarnHold = 2, QdWarnLowpose = 3 };
  int32_t kind;
  char name[24];            // SafetyLog event 이름(qd_crit · qd_warn · qd_warn_lowpose)
  char verdict[48];         // SafetyLog detail(-> Passive … · -> 처분 보류(아케이드))
  int32_t joint;            // −1 = 없음
  const char* jname;        // 정적 관절 이름표의 포인터(nullptr 가능)
  float value;              // |qd| [rad/s]
  float limit;              // 임계 [rad/s]
  int32_t over_ticks;       // 연속 초과 틱 기준(로그 줄)
  int32_t mode;             // core mode(로그 줄)
  int32_t safety_class;     // core 안전 등급(로그 줄)
  bool holds_qd;            // core 가 처분을 보류하는 skill(점프 · 브리지)
  float z_fk;               // [m]
  float tilt_deg;           // [°]
  double wall;              // SafetyLog 시계(system_clock) — 사건이 난 순간(CSV 의 wall_time 이 늦게 찍혀도 이 값)
  int64_t mono_ns;          // steady_clock — 로그 스레드가 «몇 ms 뒤에 찍었나» 를 붙인다
};
static_assert(std::is_trivially_copyable<ArcadeSafetyEvent>::value, "post_event = 고정 칸 복사(할당 없음)");

// 문자열 상수 → 고정 칸(잘림 · 항상 NUL) — 할당 · 시스템 콜 없음.
template <std::size_t N>
inline void arcade_copy_text(char (&dst)[N], const char* src) noexcept {
  std::size_t i = 0;
  for (; src && src[i] && i + 1 < N; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

class ArcadeLogMailbox {
 public:
  static constexpr int kLoggerPeriodMs = 100;  // 로그 스레드가 칸을 보는 간격

  // 정책 스레드. 칸이 비었으면 복사하고 true, 차 있으면 버리고 false. 할당 · 락 · 시스템 콜 0.
  bool post(const ArcadeLogItem& it) noexcept {
    if (state_.load(std::memory_order_acquire) != kEmpty) {  // 로그 스레드가 아직 안 가져갔다 → 이번 창은 버린다
      posts_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    slot_ = it;
    state_.store(kFull, std::memory_order_release);
    return true;
  }
  // 로그 스레드. 칸이 차 있으면 꺼내고 true(칸은 다시 빈다).
  bool take(ArcadeLogItem& out) noexcept {
    if (state_.load(std::memory_order_acquire) != kFull) return false;
    out = slot_;
    state_.store(kEmpty, std::memory_order_release);
    return true;
  }
  // 칸이 차서 버린 창 수(누적 — 아무 스레드).
  uint32_t posts_dropped() const noexcept { return posts_dropped_.load(std::memory_order_relaxed); }

  // 안전 사건 고리(SPSC, kEventCap 칸 — 래치 에지라 체류 하나에 몇 건뿐). 정책 스레드: 자리가 있으면 복사하고 true, 차 있으면 버리고
  // false(계수). 할당 · 락 · 시스템 콜 0.
  static constexpr uint32_t kEventCap = 16;   // 2 의 거듭제곱(인덱스 = 계수 & (cap − 1), uint32 넘침도 맞는다)
  bool post_event(const ArcadeSafetyEvent& e) noexcept {
    const uint32_t h = ev_head_.load(std::memory_order_relaxed);
    if (h - ev_tail_.load(std::memory_order_acquire) >= kEventCap) {
      events_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    ev_[h & (kEventCap - 1u)] = e;
    ev_head_.store(h + 1u, std::memory_order_release);
    return true;
  }
  // 로그 스레드: 가장 오래된 사건 하나(들어온 순서).
  bool take_event(ArcadeSafetyEvent& out) noexcept {
    const uint32_t t = ev_tail_.load(std::memory_order_relaxed);
    if (t == ev_head_.load(std::memory_order_acquire)) return false;
    out = ev_[t & (kEventCap - 1u)];
    ev_tail_.store(t + 1u, std::memory_order_release);
    return true;
  }
  uint32_t events_dropped() const noexcept { return events_dropped_.load(std::memory_order_relaxed); }

  // 로그 스레드 본체(보통 우선순위 — 시작할 때 SCHED_OTHER 로 내린다: FSM 스레드(SCHED_FIFO)에서 띄우면 그 우선순위를 물려받는다).
  //   kLoggerPeriodMs 마다 안전 사건 고리를 들어온 순서대로 비우고(spdlog + SafetyLog::event_at — 사건 시각 그대로), 이어 창 칸을
  //   비워 spdlog([diag:policy] · 추론 실패 · 건너뛴 틱 · 버린 이벤트 · 버린 창 · 버린 사건) · SafetyLog::sample 을 부른다.
  //   running 이 false 가 되면(wake 로 바로 깨어) 마지막으로 한 번 비우고 끝난다. 정의 = src/ArcadeLink.cpp(아케이드 빌드만 —
  //   spdlog · SafetyLog 를 이 헤더에 끌어들이지 않는다).
  void run_logger(std::atomic<bool>& running);
  // 멈추는 쪽(FSM 스레드): running.store(false) 뒤에 부르면 로그 스레드가 100 ms 를 다 기다리지 않고 깬다(exit 의 join 이 짧게).
  void wake() {
    { std::lock_guard<std::mutex> g(m_); }
    cv_.notify_all();
  }

 private:
  static constexpr int kEmpty = 0, kFull = 1;
  static_assert(std::atomic<int>::is_always_lock_free, "칸 상태는 lock-free 원자");
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "고리 계수는 lock-free 원자");
  static_assert((kEventCap & (kEventCap - 1u)) == 0, "kEventCap 은 2 의 거듭제곱");
  std::atomic<int> state_{kEmpty};
  std::atomic<uint32_t> posts_dropped_{0};
  ArcadeLogItem slot_{};
  std::atomic<uint32_t> ev_head_{0}, ev_tail_{0};   // head = 정책 스레드만 씀 · tail = 로그 스레드만 씀
  std::atomic<uint32_t> events_dropped_{0};
  ArcadeSafetyEvent ev_[kEventCap] = {};
  std::mutex m_;               // run_logger 의 기다림 · wake 전용(post · take 는 쓰지 않는다)
  std::condition_variable cv_;
};

}  // namespace g1
