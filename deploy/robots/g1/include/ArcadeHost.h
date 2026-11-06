#pragma once
// ArcadeHost.h — g1_ctrl 안에서 아케이드 core(전환 기계 + skill, g1_arcade core/)를 돌리는 호스트. g1 전용(🅐 구역).
//   한 정책 틱 = 측정 상태 + 의도 → core begin_tick(관측) → 활성 꾸러미 ONNX(OrtPool) → core end_tick(PD 목표) → q_des 를
//   두 벌 시퀀스 잠금(QDesLatch)에 기록. 1 kHz run() 은 read_q_des 로 그 값을 읽는다(락 없음 · 기다림 없음).
//   묶음(g1_arcade tools/arcade bundle 이 만든 폴더 — manifest · data/ · skills/<id>/{skill.json,policy.onnx})은 open 이 연다.
//   빌드: deploy/robots/g1/CMakeLists.txt 가 G1_ARCADE_DIR(g1_arcade 폴더)이 있을 때만 이 파일을 쓰고 G1_HAVE_ARCADE=1 을 준다.
//   없으면 이 헤더는 비어 있다(아케이드 없는 g1_ctrl 은 지금과 같다).
//
// 🔴 스레드 계약(설계 2B §5 · R-B5):
//   · open · enter = 정책 스레드 밖(기동 · State_Mimic::enter 의 FSM 스레드, 정책 스레드가 돌기 전). 할당 · 파일 읽기 · ORT 세션
//     생성 · 예열(warm_up 3) · 브리지 은행 칸 잡기가 여기서 끝난다.
//   · tick · info = 정책 스레드 하나(50 Hz). 예열 뒤 우리 코드(이 파일 · core · host 층)는 할당 · 해제 · 시스템 콜 · 락 0
//     (ORT Session::Run 안쪽 할당은 R-B5 가 뺀다 — 시험이 Run 당 수를 따로 센다). 시계 = steady_clock(vDSO, 시스템 콜 아님).
//     core Runtime 은 이 스레드만 만진다(tick_info 도 end_tick 직후 여기서 읽어 둔 사본을 info() 가 돌려준다).
//   · read_q_des = 1 kHz 스레드(다른 스레드). 원자 낱말 읽기뿐.
//   · mlockall 은 부르지 않는다(프로세스 전역 — g1_ctrl main/State_Mimic 이 정할 일).
// 시간: ObservableState::t = enter 뒤 틱 번호 × step_dt(호스트가 채운다 — 넘겨받은 s.t 는 쓰지 않는다).
// 물체 관측(ObservableState::objects)은 쓰지 않는다(로봇에는 물체 관측이 없다 — 복사도 하지 않아 할당이 없다).
#if G1_HAVE_ARCADE
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "arcade/runtime.h"
#include "arcade/types.h"

namespace arcade {
class OrtPool;
}

namespace g1 {

// q_des 29 개를 한 쓰는 쪽(정책 스레드) → 읽는 쪽(1 kHz) 으로 넘기는 두 벌 시퀀스 잠금(seqcount latch — Linux
// raw_write_seqcount_latch 와 같은 배치). 쓰기 = seq 홀수 → 0 번 칸 → seq 짝수 → 1 번 칸. 읽기 = seq & 1 번 칸을 읽고 seq 가
// 그대로인지 본다 — 읽는 동안 쓰는 쪽이 건드리지 않는 칸을 고르므로 쓰기 한복판에도 직전 프레임을 바로 얻는다(기다림 없음).
// 칸은 std::atomic<uint32_t> 낱말(float 비트) — C++ 메모리 모형 안에서 데이터 경합이 없다(MaskedStatus.h 와 같은 이유).
// 울타리 짝(release ↔ acquire)은 MaskedStatus.h 머리 설명(Boehm 2012)과 같은 논리: 읽는 쪽이 쓰는 중인 칸의 낱말을 하나라도
// 봤다면 그 앞의 seq 증가가 두 번째 seq 읽기에 보여 s0 ≠ s1 로 걸린다.
class QDesLatch {
 public:
  static constexpr int kN = 29;
  static constexpr int kTries = 4;  // 쓰는 쪽이 한 번 읽는 사이에 두 반쪽을 다 지나야 실패 — 50 Hz 쓰기엔 사실상 없다

  // 쓰는 쪽 하나(정책 스레드 · 정책 스레드가 돌기 전엔 enter 의 FSM 스레드). 할당 · 시스템 콜 · 락 없음.
  void publish(const float* q) noexcept {
    uint32_t w[kN];
    std::memcpy(w, q, sizeof(w));
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);  // 앞 쓰기의 1 번 칸이 «s+1 을 본 읽는 쪽» 에게 다 보이게
    seq_.store(s + 1u, std::memory_order_relaxed);         // 홀수: 읽는 쪽은 1 번 칸
    std::atomic_thread_fence(std::memory_order_release);  // s+1 이 아래 0 번 칸 저장보다 먼저 보이게(찢긴 읽기 감지)
    for (int i = 0; i < kN; ++i) buf_[0][i].store(w[i], std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);  // 0 번 칸이 «s+2 를 본 읽는 쪽» 에게 다 보이게
    seq_.store(s + 2u, std::memory_order_relaxed);         // 짝수: 읽는 쪽은 0 번 칸
    std::atomic_thread_fence(std::memory_order_release);  // s+2 가 아래 1 번 칸 저장보다 먼저 보이게
    for (int i = 0; i < kN; ++i) buf_[1][i].store(w[i], std::memory_order_relaxed);
  }

  // 읽는 쪽(몇이든). 첫 쓰기가 아직 안 끝났거나(seq 0 · 1 — seq 1 이 가리키는 1 번 칸은 아직 한 번도 안 썼다) kTries 안에
  // 안정된 프레임을 못 얻으면 false(out 은 그대로).
  bool read(std::array<float, kN>& out) const noexcept {
    for (int t = 0; t < kTries; ++t) {
      const uint32_t s0 = seq_.load(std::memory_order_acquire);
      if (s0 < 2u) return false;
      const std::atomic<uint32_t>* b = buf_[s0 & 1u];
      uint32_t w[kN];
      for (int i = 0; i < kN; ++i) w[i] = b[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) != s0) continue;
      std::memcpy(out.data(), w, sizeof(w));
      return true;
    }
    return false;
  }

  // «아직 쓰지 않음» 으로(seq 0 — read 가 false, 다음 publish 가 끝나야 다시 true). 쓰는 쪽이 없을 때만(ArcadeHost 를 닫을 때 ·
  // 정책 스레드 밖).
  void clear() noexcept { seq_.store(0u, std::memory_order_release); }

 private:
  static_assert(std::atomic<uint32_t>::is_always_lock_free, "lock-free 원자 낱말이어야 한다");
  static_assert(sizeof(float) == sizeof(uint32_t), "float 비트 = 낱말 하나");
  std::atomic<uint32_t> seq_{0u};
  std::atomic<uint32_t> buf_[2][kN] = {};
};

class ArcadeHost {
 public:
  struct TickOut {
    bool ok = false;        // 추론 성공 ∧ 유한 q_des 를 기록했다. false = 열지 않음 · 추론 실패(NaN 29 → core 가 직전 목표
                            // 유지) · 비유한 q_des(기록하지 않음 — 1 kHz 는 직전 값) · 틱 안 예외(정상 경로엔 없다)
    int tick_us = 0;        // tick 전체(begin_tick · 추론 · end_tick · tick_info · q_des 기록) 경과 시간 [µs]
    arcade::TickInfo info{};  // end_tick 직후 tick_info()
  };

  ArcadeHost();
  ~ArcadeHost();
  ArcadeHost(const ArcadeHost&) = delete;
  ArcadeHost& operator=(const ArcadeHost&) = delete;

  // 묶음을 연다(기동 때 1 회, 정책 스레드 밖): load_bundle → OrtPool::open → check_pool_against_bundle(꾸러미 ↔ 세션 1:1 ·
  // 입력 폭) → warm_up(3)(실패 수 > 0 이면 실패) → Runtime(묶음 설정) + 브리지 은행(set_bridge_bank). 꾸러미의 step_dt 는 모두
  // 같아야 한다. 실패하면 false + err(이유), 호스트는 닫힌 상태(tick 은 ok=false 로 아무것도 안 한다). 다시 부르면 새로 연다.
  bool open(const std::string& bundle_dir, std::string& err);
  // open + check_gains — State_Mimic(T8)이 슬롯 deploy.yaml 의 stiffness/damping(env->robot->data.joint_stiffness/damping)을
  // 넘긴다. kp/kd 는 State_Mimic::enter 가 lowcmd 에 한 번만 싣는다(F-B13) — 꾸러미가 다르면 아케이드 경로가 다른 게인으로 돈다.
  // 다르면 false + err, 호스트는 닫힌 상태.
  bool open(const std::string& bundle_dir, const std::array<float, 29>& expected_kp,
            const std::array<float, 29>& expected_kd, std::string& err);
  // 연 묶음의 모든 꾸러미 kp/kd(skill.json)가 주어진 값과 같은가(상대 1e-6 — float ↔ double 읽기 차만 허용). 다르면 false +
  // err(꾸러미 id · 관절 번호 · 두 값). 열지 않았으면 false.
  bool check_gains(const std::array<float, 29>& kp, const std::array<float, 29>& kd, std::string& err) const;

  // 체류 시작(State_Mimic::enter, 정책 스레드가 돌기 전): Runtime::reset(처음 role · 경로 · 안전 래치 비움) + 틱 번호 0 +
  // q_des 칸을 측정 자세(measured.q)로 채움 — 첫 추론 전 1 kHz 가 0 rad 를 내는 창이 없다. 열지 않았으면 아무것도 안 한다.
  void enter(const arcade::ObservableState& measured);
  // 정책 틱(정책 스레드). 머리 설명의 순서. s.t · s.objects 는 쓰지 않는다. 할당 · 시스템 콜 · 락 없음(예열 뒤).
  TickOut tick(const arcade::ObservableState& s, const arcade::Intent& in) noexcept;
  // 1 kHz 스레드. 마지막으로 기록한 q_des(enter 의 측정 자세 또는 마지막 틱 목표). 아직 없거나 읽기 실패면 false(out 그대로 —
  // 호출부는 직전 값 유지).
  bool read_q_des(std::array<float, 29>& out) const noexcept;
  // 마지막 tick 의 TickInfo 사본(정책 스레드에서만 — enter 직후엔 reset 뒤 값). 열지 않았으면 0 값.
  arcade::TickInfo info() const noexcept;

  // 기동 · 상태 칸 · 시험용(open 뒤 바뀌지 않는다). 열지 않았으면 빈 값.
  bool is_open() const noexcept { return open_; }
  int session_count() const noexcept;  // ORT 세션 수(= 꾸러미 수)
  double step_dt() const noexcept { return dt_; }
  const std::vector<std::string>& skill_names() const noexcept;  // TickInfo.skill 번호의 꾸러미 id
  const std::vector<std::string>& role_names() const noexcept;   // TickInfo.role 번호의 role
  int ticks() const noexcept { return tick_; }                    // enter 뒤 돈 틱 수(정책 스레드)
  int max_tick_us() const noexcept { return max_tick_us_; }       // enter 뒤 tick_us 최대(정책 스레드)

 private:
  struct Gains {
    std::string id;
    arcade::Vec29 kp{}, kd{};
  };
  void close() noexcept;

  std::unique_ptr<arcade::OrtPool> pool_;
  std::unique_ptr<arcade::Runtime> rt_;
  std::vector<Gains> gains_;
  bool open_ = false;
  double dt_ = 0;
  int tick_ = 0;
  int max_tick_us_ = 0;
  arcade::ObservableState s_;      // 틱마다 덮어쓰는 작업 칸(t = 틱 × dt, objects 는 늘 빈 채)
  std::array<float, 29> nan29_{};  // 추론 실패 때 end_tick 에 넘기는 NaN 29
  std::array<float, 29> qf_{};     // q_des float 작업 칸
  arcade::TickInfo info_{};
  QDesLatch q_des_;
};

}  // namespace g1
#endif  // G1_HAVE_ARCADE
