// ArcadeLink — 머리 설명은 include/ArcadeLink.h. CMake 가 G1_ARCADE_DIR 이 있을 때만 이 파일을 넣는다(ArcadeHost.cpp 와 같다).
#if G1_HAVE_ARCADE
#include "ArcadeLink.h"

#include <yaml-cpp/yaml.h>

#include "arcade_host/bundle_loader.h"  // arcade::md5_file(묶음 manifest md5 — 기동 로그)

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
  // 아케이드 상태 칸(0x6708) — 열기 · flock 이 실패하면 거부(HUD 없이 뜨지 않는다, 검토 M10). 묶음 · ORT 를 싣기 전에(빨리 거부).
  //   G1_ARCADE_STATUS_SHM=0(일부러 끔)만 상태 칸 없이 뜬다(기동 로그 «status (off — G1_ARCADE_STATUS_SHM=0)»).
  arcade_status::ArcadeStatusWriter& sw = arcade_status::ArcadeStatusWriter::shared();
  if (!sw.open_from_env_or_default() && !sw.off_by_env()) {
    err = std::string("아케이드 상태 칸(0x6708)을 못 연다: ") + sw.last_error() +
          " — HUD 없이 뜨지 않는다(다른 g1_ctrl 이 쥐고 있으면 그것부터 끌 것 · 경로 = G1_ARCADE_STATUS_SHM)";
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
  intent_.events.reserve(8);  // 이벤트 비트 5 종(ArcadeIntent.h INTENT_EV_REAL) — 틱의 push_back 이 이 용량 안
  obs_ = arcade::ObservableState{};

  std::string role_list;
  for (const std::string& r : roles) role_list += (role_list.empty() ? "" : ",") + r;
  char buf[640];
  std::snprintf(buf, sizeof(buf),
                "bundle=%s sessions=%d warm_up ok · roles [%s] · gains match slot '%s' (꾸러미 %d 개 kp/kd) · joint_ids_map "
                "identity · intent %s · status %s",
                a.bundle_dir.c_str(), host_.session_count(), role_list.c_str(), a.slot_name.c_str(), host_.session_count(),
                intent_path_.c_str(), sw.on() ? sw.path() : "(off — G1_ARCADE_STATUS_SHM=0)");
  summary = std::string(buf) + " · " + bundle_manifest_info(a.bundle_dir);  // 묶음 출처(정보만 — 거부 조건 아님, 검토 M4)
  return true;
}

std::string bundle_manifest_info(const std::string& bundle_dir) {
  const std::string mp = bundle_dir + "/manifest.json";
  std::string md5;
  if (!arcade::md5_file(mp, md5)) md5 = "?";
  std::string board = "?", commit = "?", dirty = "?";
  try {  // manifest = JSON(= YAML 흐름 꼴) — yaml-cpp 로 읽는다. 칸이 없거나 형이 다르면 «?»
    const YAML::Node m = YAML::LoadFile(mp);
    auto scalar = [&](const char* k) -> std::string {
      const YAML::Node n = m[k];
      return (n && n.IsScalar() && !n.Scalar().empty()) ? n.Scalar() : std::string("?");
    };
    if (m.IsMap()) {
      board = scalar("board");
      commit = scalar("g1_arcade_commit");
      dirty = scalar("dirty");
    }
  } catch (const std::exception&) {
  }
  return "manifest md5 " + md5 + " · board " + board + " · g1_arcade " + commit + " · dirty " + dirty;
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
  static constexpr uint32_t kBits[] = {INTENT_EV_JUMP, INTENT_EV_CRAWL, INTENT_EV_GETUP, INTENT_EV_POSTURE_Z, INTENT_EV_POSTURE_N};
  if (t.in.dropped_events) {  // 실기에서 안 먹는 이벤트 — 센다만(로그는 정책 스레드 밖) · 할당 0 · 시스템 콜 0
    dropped_event_bits_ |= t.in.dropped_events;
    ++dropped_event_count_;
  }
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

// /proc/self/status 의 VmSize · VmRSS · VmLck [kB] · CapEff(16 진). 못 읽은 칸은 -1 / 0(기동 때 한 번 — 실시간 경로 밖).
struct ProcStatus {
  long long vmsize_kb = -1, vmrss_kb = -1, vmlck_kb = -1;
  std::uint64_t cap_eff = 0;
  bool have_cap = false;
};
static ProcStatus read_proc_status() {
  ProcStatus ps;
  if (std::FILE* f = std::fopen("/proc/self/status", "r")) {
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
      std::uint64_t v = 0;
      if (arcade_gate::proc_status_field(line, "VmSize:", 10, v)) ps.vmsize_kb = static_cast<long long>(v);
      else if (arcade_gate::proc_status_field(line, "VmRSS:", 10, v)) ps.vmrss_kb = static_cast<long long>(v);
      else if (arcade_gate::proc_status_field(line, "VmLck:", 10, v)) ps.vmlck_kb = static_cast<long long>(v);
      else if (arcade_gate::proc_status_field(line, "CapEff:", 16, v)) { ps.cap_eff = v; ps.have_cap = true; }
    }
    std::fclose(f);
  }
  return ps;
}

// 잠근 양을 로봇에서 실제로 보려고(잠근 뒤라 VmLck 가 실제 잠긴 양)
static std::string vm_tail() {
  const ProcStatus ps = read_proc_status();
  char b[120];
  std::snprintf(b, sizeof(b), " · VmSize %.1f MB · VmRSS %.1f MB · VmLck %.1f MB", ps.vmsize_kb / 1024.0, ps.vmrss_kb / 1024.0,
                ps.vmlck_kb / 1024.0);
  return b;
}

std::string arcade_try_mlock_current() {
  rlimit rl{};
  if (getrlimit(RLIMIT_MEMLOCK, &rl) != 0) return std::string("skip (getrlimit 실패: ") + std::strerror(errno) + ")";
  const ProcStatus ps = read_proc_status();
  const std::uint64_t vmsize = ps.vmsize_kb > 0 ? static_cast<std::uint64_t>(ps.vmsize_kb) * 1024u : 0u;
  const bool cap = ps.have_cap && arcade_gate::cap_ipc_lock_in(ps.cap_eff);
  const bool unlimited = rl.rlim_cur == RLIM_INFINITY;
  const std::uint64_t soft = unlimited ? 0 : static_cast<std::uint64_t>(rl.rlim_cur);
  const arcade_gate::MlockVerdict v = arcade_gate::mlock_verdict(unlimited, soft, vmsize, cap);
  char lim[48];
  if (unlimited) std::snprintf(lim, sizeof(lim), "unlimited");
  else std::snprintf(lim, sizeof(lim), "%.1f MB", soft / 1048576.0);
  // 판정 근거: 한도 · VmSize(×1.2) · CAP_IPC_LOCK(CapEff 비트 14) — 로봇에서 «왜 잠갔나/안 잠갔나» 를 로그 한 줄로
  char basis[256];
  std::snprintf(basis, sizeof(basis), "memlock soft %s · VmSize %.1f MB (x1.2 = %.1f MB) · CAP_IPC_LOCK %s · 판정 %s", lim,
                vmsize / 1048576.0, (vmsize + vmsize / 5) / 1048576.0,
                !ps.have_cap ? "? (CapEff 못 읽음)" : (cap ? "있음" : "없음"), v.why);
  if (!v.try_lock) return std::string("skip (") + basis + " — 잠그지 않는다)" + vm_tail();
  if (mlockall(MCL_CURRENT) != 0) {
    const int e = errno;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "fail errno=%d %s (", e, std::strerror(e));
    return std::string(buf) + basis + ") — 잠그지 않고 계속" + vm_tail();
  }
  return std::string("ok (MCL_CURRENT · ") + basis + " · MCL_FUTURE 안 씀)" + vm_tail();
}

// 정책 스레드 스택 선터치(최종 검토 I7) — 머리 설명은 ArcadeLink.h. 따로 된 함수(noinline)라 돌아온 뒤 그 자리를 뒤 호출
// (ArcadeLink::tick → ArcadeHost::tick → ORT Run)이 다시 쓴다(부르는 쪽 지역 변수로 두면 그 프레임 «아래» 가 쓰여 소용없다).
__attribute__((noinline)) void arcade_pretouch_stack() noexcept {
  volatile unsigned char touch[kArcadePretouchStackBytes];
  for (std::size_t i = sizeof(touch); i > 0; i -= 4096) touch[i - 1] = 0;  // 위(지금 스택 꼭대기 쪽)에서 아래로, 쪽마다 한 번
  touch[0] = 0;
}

}  // namespace g1
#endif  // G1_HAVE_ARCADE
