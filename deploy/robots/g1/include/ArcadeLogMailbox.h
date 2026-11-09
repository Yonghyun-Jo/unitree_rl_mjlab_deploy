#pragma once
// ArcadeLogMailbox.h — 아케이드 정책 스레드 → 로그 스레드 한 칸 우편함. g1 전용(🅐 구역).
//
// 왜 있나(2026-11-09 사용자 결정 «로그는 정책 스레드 밖»): 정책 스레드가 1 Hz 로 직접 부르던 spdlog(동기 — stdout · 회전 파일,
//   deploy/include/param.h)와 SafetyLog::sample(fflush)은 터미널 · 디스크가 느린 순간 그 틱을 밀리초 단위로 잡아먹는다. 이제 정책
//   스레드는 창(1 s)이 닫힐 때 숫자 · 문자열을 이 칸에 «넣기만» 하고, 보통 우선순위 로그 스레드가 100 ms 마다 칸을 비워 찍는다.
//   (LoopDiag::publish/take 와 같은 SPSC 한 칸 — 그쪽은 문자열만이라 여기서 항목을 넓혔다. LoopDiag.h 는 🅑 라 건드리지 않는다.)
//
// 🔴 스레드 계약: post = 정책 스레드 하나(할당 · 락 · 시스템 콜 0 — 원자 int 하나 + 고정 칸 복사). take · run_logger = 로그 스레드 하나.
//   wake = 멈추는 쪽(FSM 스레드 — exit 에서 running 을 내린 뒤). 칸이 차 있으면(로그 스레드가 1 s 넘게 못 돎) post 는 버리고 false
//   (버린 수는 원자 계수 — 로그 스레드가 다음 줄에 찍는다).
#include <atomic>
#include <condition_variable>
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

  // 로그 스레드 본체(보통 우선순위 — 시작할 때 SCHED_OTHER 로 내린다: FSM 스레드(SCHED_FIFO)에서 띄우면 그 우선순위를 물려받는다).
  //   kLoggerPeriodMs 마다 칸을 비워 spdlog([diag:policy] · 추론 실패 · 건너뛴 틱 · 버린 이벤트 · 버린 창) · SafetyLog::sample 을
  //   부른다. running 이 false 가 되면(wake 로 바로 깨어) 마지막으로 한 번 비우고 끝난다. 정의 = src/ArcadeLink.cpp(아케이드 빌드만 —
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
  std::atomic<int> state_{kEmpty};
  std::atomic<uint32_t> posts_dropped_{0};
  ArcadeLogItem slot_{};
  std::mutex m_;               // run_logger 의 기다림 · wake 전용(post · take 는 쓰지 않는다)
  std::condition_variable cv_;
};

}  // namespace g1
