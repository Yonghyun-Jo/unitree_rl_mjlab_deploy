#pragma once
// ArcadeLink.h — State_Mimic(Mimic_Masked) ↔ ArcadeHost 를 잇는 얇은 층. g1 전용(🅐 구역). G1_ARCADE=1 일 때만 쓰인다.
//   한 정책 틱 = 의도 칸(ArcadeIntent.h, open/read/close) → poll_intent(끊김 · 이벤트 · 재무장) → core Intent(이벤트 vector 는 미리 잡은
//   용량 안에서만) → ArcadeHost::tick(core begin_tick → ORT → end_tick → q_des 잠금) → (State_Mimic 의 qd 가드) → 아케이드 상태 칸
//   (ArcadeStatus.h, 0x6708) 한 줄. 1 kHz run() 은 run_q_des() 로 q_des 를 받는다(읽기 실패 = 직전 q — 첫 값은 enter 의 측정 자세).
//   판단(넘어짐 관문 · qd_warn 처분)은 ArcadeGate.h 의 순수 함수다 — 여기는 값을 나르기만 한다.
//
// 🔴 스레드 계약(ArcadeHost.h 와 같다):
//   · open = State_Mimic 생성자(실시간 스레드 밖). enter = State_Mimic::enter(FSM 스레드, 정책 스레드를 시작하기 «전» —
//     std::thread 시작이 정책 스레드에 happens-before 를 준다).
//   · tick · core_safety · write_status = 정책 스레드 하나. 예열 뒤 우리 코드 할당 0(ORT Run 안쪽은 R-B5 가 뺀다) · 시스템 콜 =
//     의도 칸 읽기 open/read/close 3 개뿐(ArcadeIntent.h 의 명시된 예외) · 락 0.
//   · run_q_des = 1 kHz FSM 스레드(enter 와 같은 스레드 — 직전 q 칸은 이 스레드 것).
#if G1_HAVE_ARCADE
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "ArcadeGate.h"
#include "ArcadeHost.h"
#include "ArcadeIntent.h"
#include "ArcadeStatus.h"

namespace g1 {

// 정책 틱의 측정 상태 한 벌(G1Articulation::update 결과의 복사 — float, 정책 순서 = SDK 순서). sense_of 가 채운다.
struct ArcadeSense {
  std::array<float, 4> quat_wxyz{};
  std::array<float, 3> gyro{};
  std::array<float, 29> q{}, dq{};
};
// isaaclab::ArticulationData(root_quat_w · root_ang_vel_b · joint_pos · joint_vel)에서 — 할당 없음(Eigen 원소 읽기만).
template <class Data>
inline void sense_of(const Data& d, ArcadeSense& s) noexcept {
  s.quat_wxyz = {d.root_quat_w.w(), d.root_quat_w.x(), d.root_quat_w.y(), d.root_quat_w.z()};
  for (int i = 0; i < 3; ++i) s.gyro[i] = d.root_ang_vel_b[i];
  for (int i = 0; i < 29; ++i) {
    s.q[i] = d.joint_pos[i];
    s.dq[i] = d.joint_vel[i];
  }
}

class ArcadeLink {
 public:
  struct OpenArgs {
    std::string bundle_dir;
    std::string slot_name;                 // g1_ctrl 슬롯(G1_POLICY_SLOT) — 묶음의 base_slot 과 같아야 한다
    std::array<float, 29> kp{}, kd{};      // 슬롯 deploy.yaml(env->robot->data.joint_stiffness/damping)
    std::vector<float> joint_ids_map;      // 슬롯 deploy.yaml — 0..28 항등이어야 한다
    std::string intent_path = ARCADE_INTENT_PATH;
  };
  struct Tick {
    ArcadeHost::TickOut out{};   // ok · tick_us · info(이 틱 end_tick 뒤)
    IntentApply in{};            // 이 틱에 core 로 보낸 의도(끊김 · 재무장 포함)
    bool frame = false;          // 의도 칸 프레임을 읽었나
    int poll_us = 0;             // 의도 칸 읽기 + 판정 [µs]
  };

  ArcadeLink();
  // 기동(정책 스레드 밖, 1 회). 실패하면 false + err(기동 거부 이유 — 호출부가 die_startup). 성공하면 summary 에 기동 로그 한 줄.
  bool open(const OpenArgs& a, std::string& err, std::string& summary);
  // 체류 시작(FSM 스레드, 정책 스레드 전). 측정 자세 q 에 비유한 값이 있으면 false — 체류를 시작하지 않는다(refused() = true,
  // 호출부가 kp 0 · Passive). 그 밖엔 ArcadeHost::enter(측정) · 의도 칸 체류 시작 리셋 · 직전 q 칸 = 측정 q · 상태 칸 틱 0.
  bool enter(const ArcadeSense& measured);
  bool refused() const noexcept { return refused_.load(std::memory_order_relaxed); }

  Tick tick(const ArcadeSense& s) noexcept;                     // 정책 스레드
  arcade_gate::CoreSafety core_safety() const noexcept;         // 정책 스레드 — 마지막 tick(enter 직후엔 reset 뒤) 기준
  // 정책 스레드 — 이 틱 아케이드 상태 칸 한 줄(원자 저장 22 개). qd_warn · qd_crit = g1_ctrl 안전층 래치(실제 처분이 보는 값).
  void write_status(const Tick& t, int fsm_state, float tilt_deg, bool qd_warn, bool qd_crit) noexcept;
  std::vector<float> run_q_des();                               // 1 kHz FSM 스레드

  const ArcadeHost& host() const noexcept { return host_; }

 private:
  ArcadeHost host_;
  std::string intent_path_;
  int bridge_role_ = -1;                       // role_names() 의 "bridge"(core 가 처분을 보류하는 role) — 없으면 -1
  IntentPollState poll_{};
  arcade::Intent intent_{};                    // events 는 open 에서 용량을 잡는다(push_back 이 할당하지 않게)
  arcade::ObservableState obs_{};              // objects 는 늘 빈 채(로봇엔 물체 관측이 없다)
  uint32_t status_tick_ = 0;
  std::atomic<bool> refused_{false};
  std::array<float, 29> q_last_{};             // 1 kHz 의 «직전 q» — enter 가 측정 자세로 채운다(0 rad 를 내지 않는다)
};

// 아케이드 상태 칸(0x6708) 한 줄을 짓는다(순수 — 할당 없음). role · skill · notice = core 표 번호 · cmd = core 명령 · z_fk = core 값.
inline ArcadeStatus arcade_status_frame(const ArcadeLink::Tick& t, uint32_t tick, int fsm_state, float tilt_deg, bool qd_warn,
                                        bool qd_crit) noexcept {
  const arcade::TickInfo& i = t.out.info;
  ArcadeStatus s{};
  s.tick = tick;
  s.role = i.role;
  s.skill = i.skill;
  s.mode = i.mode;
  s.preset = i.preset;
  s.route_step = i.route_step;
  s.notice = i.notice;
  s.safety_class = i.safety_class;
  s.busy = i.busy ? 1 : 0;
  s.qd_warn = qd_warn ? 1 : 0;
  s.qd_crit = qd_crit ? 1 : 0;
  s.intent_stale = t.in.stale ? 1 : 0;
  s.rearmed = t.in.rearmed ? 1 : 0;
  s.fsm_state = fsm_state;
  s.tick_us = t.out.tick_us > 0 ? static_cast<uint32_t>(t.out.tick_us) : 0u;
  s.z_fk = i.z_fk;
  s.tilt_deg = tilt_deg;
  s.cmd_vx = i.cmd_vx;
  s.cmd_vy = i.cmd_vy;
  s.cmd_wz = i.cmd_wz;
  return s;
}

// mlockall(MCL_CURRENT) 을 «시도만» 한다(R-B7 — arcade_gate::mlock_worth_trying 일 때만, MCL_FUTURE 없음). 결과 한 줄(기동 로그).
std::string arcade_try_mlock_current();

}  // namespace g1
#endif  // G1_HAVE_ARCADE
