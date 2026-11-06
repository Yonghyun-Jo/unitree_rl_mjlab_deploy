// ArcadeLink — 머리 설명은 include/ArcadeLink.h. CMake 가 G1_ARCADE_DIR 이 있을 때만 이 파일을 넣는다(ArcadeHost.cpp 와 같다).
#if G1_HAVE_ARCADE
#include "ArcadeLink.h"

#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace g1 {

ArcadeLink::ArcadeLink() = default;

bool ArcadeLink::open(const OpenArgs& a, std::string& err, std::string& summary) {
  if (!arcade_gate::ids_identity(a.joint_ids_map)) {
    err = "슬롯 deploy.yaml 의 joint_ids_map 이 0..28 항등이 아니다 — 아케이드 경로는 정책 순서 = SDK 순서 = core 순서로만 쓴다";
    return false;
  }
  if (a.bundle_dir.empty()) {
    err = "G1_ARCADE_BUNDLE(아케이드 묶음 폴더 — g1_arcade tools/arcade bundle 이 만든 것)이 없다";
    return false;
  }
  std::string why;
  if (!host_.open(a.bundle_dir, a.kp, a.kd, why)) {
    err = "ArcadeHost::open(" + a.bundle_dir + "): " + why;
    return false;
  }
  if (host_.base_slot().empty() || host_.base_slot() != a.slot_name) {
    err = "g1_ctrl 슬롯 '" + a.slot_name + "' ≠ 묶음의 처음 role 꾸러미 슬롯 '" + host_.base_slot() +
          "' — 안전층(pos_clamp 한계 · qd 임계 · 폴백 모드)이 그 슬롯의 것이어야 한다. G1_POLICY_SLOT=" + host_.base_slot() +
          " 로 띄울 것";
    return false;
  }
  intent_path_ = a.intent_path;
  bridge_role_ = -1;
  const std::vector<std::string>& roles = host_.role_names();
  for (size_t i = 0; i < roles.size(); ++i)
    if (roles[i] == "bridge") bridge_role_ = static_cast<int>(i);
  intent_.events.clear();
  intent_.events.reserve(8);  // 이벤트 비트 5 종(ArcadeIntent.h INTENT_EV_*) — 틱의 push_back 이 이 용량 안
  obs_ = arcade::ObservableState{};
  arcade_status::ArcadeStatusWriter& sw = arcade_status::ArcadeStatusWriter::shared();
  sw.open_from_env_or_default();  // 실패 · 끔이면 write 는 no-op(이유는 writer 가 stderr 한 줄)

  std::string role_list;
  for (const std::string& r : roles) role_list += (role_list.empty() ? "" : ",") + r;
  char buf[640];
  std::snprintf(buf, sizeof(buf),
                "bundle=%s sessions=%d warm_up ok · roles [%s] · gains match slot '%s' (꾸러미 %d 개 kp/kd) · joint_ids_map "
                "identity · intent %s · status %s",
                a.bundle_dir.c_str(), host_.session_count(), role_list.c_str(), a.slot_name.c_str(), host_.session_count(),
                intent_path_.c_str(), sw.on() ? sw.path() : "(off)");
  summary = buf;
  return true;
}

bool ArcadeLink::enter(const ArcadeSense& m, const float* default_q) {
  bool finite = true;
  for (int i = 0; i < 29; ++i) finite = finite && std::isfinite(m.q[i]);
  refused_.store(!finite, std::memory_order_relaxed);
  if (!finite) {  // 잠금은 그대로(ArcadeHost::enter 도 비유한 측정은 싣지 않는다) — 호출부가 Passive. 직전 q = 유한 칸은 측정, 아니면 default
    arcade_gate::refusal_fallback_q(m.q.data(), default_q, q_last_.data());
    return false;
  }
  arcade::ObservableState s;
  for (int i = 0; i < 4; ++i) s.quat_wxyz[i] = static_cast<double>(m.quat_wxyz[i]);
  for (int i = 0; i < 3; ++i) s.gyro[i] = static_cast<double>(m.gyro[i]);
  for (int i = 0; i < 29; ++i) {
    s.q[i] = static_cast<double>(m.q[i]);
    s.dq[i] = static_cast<double>(m.dq[i]);
  }
  host_.enter(s);           // Runtime::reset · 틱 번호 0 · q_des 잠금 = 측정 q
  reset_for_stay_start(poll_);  // 체류 시작: 첫 프레임은 기준선만 · 재무장 필요
  q_last_ = m.q;            // 1 kHz 의 직전 q = 측정 자세(잠금 읽기가 실패해도 0 rad 가 나가지 않는다)
  status_tick_ = 0;
  return true;
}

ArcadeLink::Tick ArcadeLink::tick(const ArcadeSense& s) noexcept {
  Tick t;
  const auto t0 = std::chrono::steady_clock::now();
  ArcadeIntentFrame f{};
  t.frame = read_intent_file(intent_path_.c_str(), f);
  t.in = poll_intent(t.frame ? &f : nullptr, poll_);
  intent_.move.fwd = t.in.fwd;
  intent_.move.side = t.in.side;
  intent_.move.turn = t.in.turn;
  intent_.move.run = t.in.run != 0;
  intent_.events.clear();  // 용량은 그대로 — 아래 push_back 은 할당하지 않는다
  static constexpr uint32_t kBits[] = {INTENT_EV_JUMP, INTENT_EV_CRAWL, INTENT_EV_GETUP, INTENT_EV_LIE, INTENT_EV_BOX};
  for (uint32_t b : kBits)
    if ((t.in.events & b) && intent_.events.size() < intent_.events.capacity())
      intent_.events.push_back(static_cast<arcade::Event>(b));
  for (int i = 0; i < 4; ++i) obs_.quat_wxyz[i] = static_cast<double>(s.quat_wxyz[i]);
  for (int i = 0; i < 3; ++i) obs_.gyro[i] = static_cast<double>(s.gyro[i]);
  for (int i = 0; i < 29; ++i) {
    obs_.q[i] = static_cast<double>(s.q[i]);
    obs_.dq[i] = static_cast<double>(s.dq[i]);
  }
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
  t.poll_us = static_cast<int>(us);
  t.out = host_.tick(obs_, intent_);  // obs_.t 는 ArcadeHost 가 틱 번호 × step_dt 로 채운다
  return t;
}

arcade_gate::CoreSafety ArcadeLink::core_safety() const noexcept {
  const arcade::TickInfo i = host_.info();
  arcade_gate::CoreSafety c;
  c.mode = i.mode;
  c.safety_class = i.safety_class;
  c.commanded_upright = i.commanded_upright;
  // core Runtime::step_safety 가 처분을 보류하는 skill: 모드 표 밖(점프 head mode7) · 브리지 추종기(role bridge, 폴백 없음)
  c.holds_qd = !mode_table::valid(i.mode) || (bridge_role_ >= 0 && i.role == bridge_role_);
  return c;
}

void ArcadeLink::write_status(const Tick& t, int fsm_state, float tilt_deg, bool qd_warn, bool qd_crit) noexcept {
  ++status_tick_;
  arcade_status::ArcadeStatusWriter::shared().write(arcade_status_frame(t, status_tick_, fsm_state, tilt_deg, qd_warn, qd_crit));
}

void ArcadeLink::run_q_des(std::array<float, 29>& out) {
  std::array<float, 29> q;
  if (host_.read_q_des(q)) q_last_ = q;  // 실패(첫 기록 전 · 읽기 4 번 엇갈림) = 직전 q 그대로
  out = q_last_;
}

// /proc/self/status 의 VmSize · VmRSS · VmLck [kB] — 못 읽으면 -1. (잠근 양을 로봇에서 실제로 보려고)
static std::string vm_tail() {
  long sz = -1, rss = -1, lck = -1;
  if (std::FILE* f = std::fopen("/proc/self/status", "r")) {
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
      long v;
      if (std::sscanf(line, "VmSize: %ld kB", &v) == 1) sz = v;
      else if (std::sscanf(line, "VmRSS: %ld kB", &v) == 1) rss = v;
      else if (std::sscanf(line, "VmLck: %ld kB", &v) == 1) lck = v;
    }
    std::fclose(f);
  }
  char b[120];
  std::snprintf(b, sizeof(b), " · VmSize %.1f MB · VmRSS %.1f MB · VmLck %.1f MB", sz / 1024.0, rss / 1024.0, lck / 1024.0);
  return b;
}

std::string arcade_try_mlock_current() {
  rlimit rl{};
  if (getrlimit(RLIMIT_MEMLOCK, &rl) != 0) return std::string("skip (getrlimit 실패: ") + std::strerror(errno) + ")";
  long pages = 0, rss_pages = 0;
  if (std::FILE* f = std::fopen("/proc/self/statm", "r")) {
    if (std::fscanf(f, "%ld %ld", &pages, &rss_pages) != 2) rss_pages = 0;
    std::fclose(f);
  }
  const std::uint64_t page = static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE));
  const std::uint64_t rss = static_cast<std::uint64_t>(rss_pages > 0 ? rss_pages : 0) * page;
  const bool unlimited = rl.rlim_cur == RLIM_INFINITY;
  const std::uint64_t soft = unlimited ? 0 : static_cast<std::uint64_t>(rl.rlim_cur);
  char lim[48];
  if (unlimited) std::snprintf(lim, sizeof(lim), "unlimited");
  else std::snprintf(lim, sizeof(lim), "%.1f MB", soft / 1048576.0);
  char buf[320];
  if (!arcade_gate::mlock_worth_trying(unlimited, soft, rss)) {
    std::snprintf(buf, sizeof(buf), "skip (memlock %s < 2×RSS %.1f MB — 잠그지 않는다)", lim, rss / 1048576.0);
    return std::string(buf) + vm_tail();
  }
  if (mlockall(MCL_CURRENT) != 0) {
    const int e = errno;
    std::snprintf(buf, sizeof(buf), "fail errno=%d %s (RSS %.1f MB · memlock %s) — 잠그지 않고 계속", e, std::strerror(e),
                  rss / 1048576.0, lim);
    return std::string(buf) + vm_tail();
  }
  std::snprintf(buf, sizeof(buf), "ok (MCL_CURRENT · RSS %.1f MB · memlock %s · MCL_FUTURE 안 씀)", rss / 1048576.0, lim);
  return std::string(buf) + vm_tail();  // 잠근 뒤라 VmLck 가 실제 잠긴 양
}

}  // namespace g1
#endif  // G1_HAVE_ARCADE
