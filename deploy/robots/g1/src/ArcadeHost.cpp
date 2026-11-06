// ArcadeHost — 머리 설명은 include/ArcadeHost.h. CMake 가 G1_ARCADE_DIR 이 있을 때만 이 파일을 넣고 G1_HAVE_ARCADE=1 을 준다.
#if G1_HAVE_ARCADE
#include "ArcadeHost.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <utility>

#include "arcade_host/bundle_loader.h"
#include "arcade_host/ort_pool.h"

namespace g1 {

namespace {
const std::vector<std::string>& empty_names() {
  static const std::vector<std::string> v;
  return v;
}

bool same_gain(double pkg, float want) {
  const double w = static_cast<double>(want);
  return std::fabs(pkg - w) <= 1e-6 * std::fmax(1.0, std::fabs(w));
}
}  // namespace

ArcadeHost::ArcadeHost() { nan29_.fill(std::numeric_limits<float>::quiet_NaN()); }
ArcadeHost::~ArcadeHost() = default;

void ArcadeHost::close() noexcept {
  open_ = false;
  rt_.reset();
  pool_.reset();
  gains_.clear();
  dt_ = 0;
  tick_ = 0;
  max_tick_us_ = 0;
  info_ = arcade::TickInfo{};
  q_des_.clear();
}

bool ArcadeHost::open(const std::string& bundle_dir, std::string& err) {
  close();
  try {
    arcade::Bundle b;
    std::string why;
    if (!arcade::load_bundle(bundle_dir, b, why)) {
      err = "load_bundle: " + why;
      return false;
    }
    if (b.setup.skills.empty()) {
      err = "묶음에 꾸러미가 없다: " + bundle_dir;
      return false;
    }
    // 정책 스레드 주기는 꾸러미 하나의 step_dt — 꾸러미마다 다르면 ObservableState::t(틱 × dt)가 skill 마다 어긋난다
    const double dt = b.setup.skills.front().step_dt;
    for (const arcade::SkillSpec& sp : b.setup.skills) {
      if (!(sp.step_dt > 0) || sp.step_dt != dt) {
        err = "꾸러미 step_dt 가 서로 다르거나 0 이하: " + sp.id + " " + std::to_string(sp.step_dt) + " ≠ " +
              std::to_string(dt);
        return false;
      }
    }
    auto pool = std::make_unique<arcade::OrtPool>();
    if (!pool->open(b.onnx_paths, why)) {
      err = "OrtPool::open: " + why;
      return false;
    }
    if (!arcade::check_pool_against_bundle(b, *pool, why)) {
      err = "check_pool_against_bundle: " + why;
      return false;
    }
    // 첫 Run 의 지연 초기화를 틱 경로 밖으로 — 실패가 하나라도 있으면 그 세션은 틱에서도 nullptr 를 낼 것이라 열지 않는다
    const int failed = pool->warm_up(3);
    if (failed != 0) {
      err = "OrtPool::warm_up(3): 실패 " + std::to_string(failed) + " 회";
      return false;
    }
    std::vector<Gains> gains;
    gains.reserve(b.setup.skills.size());
    for (const arcade::SkillSpec& sp : b.setup.skills) gains.push_back(Gains{sp.id, sp.kp, sp.kd});
    // 풀 · 브리지 은행 칸은 여기서(정책 스레드 밖) 잡는다. 계획 시계는 주지 않는다(로봇 정책 스레드 — plan_ms 0, 결정 무관).
    auto rt = std::make_unique<arcade::Runtime>(b.setup);
    if (b.bank) rt->set_bridge_bank(b.bank_plan, b.bank);
    pool_ = std::move(pool);
    rt_ = std::move(rt);
    gains_ = std::move(gains);
    dt_ = dt;
    s_ = arcade::ObservableState{};
    info_ = rt_->tick_info();
    open_ = true;
    return true;
  } catch (const std::exception& e) {
    close();
    err = std::string("ArcadeHost::open: ") + e.what();
    return false;
  }
}

bool ArcadeHost::open(const std::string& bundle_dir, const std::array<float, 29>& expected_kp,
                      const std::array<float, 29>& expected_kd, std::string& err) {
  if (!open(bundle_dir, err)) return false;
  if (!check_gains(expected_kp, expected_kd, err)) {
    close();
    return false;
  }
  return true;
}

bool ArcadeHost::check_gains(const std::array<float, 29>& kp, const std::array<float, 29>& kd,
                             std::string& err) const {
  if (!open_) {
    err = "ArcadeHost 가 열리지 않았다";
    return false;
  }
  for (const Gains& g : gains_) {
    for (int i = 0; i < arcade::kNJ; ++i) {
      const bool kp_ok = same_gain(g.kp[i], kp[i]), kd_ok = same_gain(g.kd[i], kd[i]);
      if (kp_ok && kd_ok) continue;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "꾸러미 %s 관절 %d: %s = %.6g, 슬롯 = %.6g", g.id.c_str(), i, kp_ok ? "kd" : "kp",
                    kp_ok ? g.kd[i] : g.kp[i], static_cast<double>(kp_ok ? kd[i] : kp[i]));
      err = buf;
      return false;
    }
  }
  return true;
}

void ArcadeHost::enter(const arcade::ObservableState& measured) {
  if (!open_) return;
  rt_->reset();
  tick_ = 0;
  max_tick_us_ = 0;
  info_ = rt_->tick_info();
  for (int i = 0; i < arcade::kNJ; ++i) qf_[i] = static_cast<float>(measured.q[i]);
  q_des_.publish(qf_.data());
}

ArcadeHost::TickOut ArcadeHost::tick(const arcade::ObservableState& s, const arcade::Intent& in) noexcept {
  TickOut out;
  if (!open_) return out;
  const auto t0 = std::chrono::steady_clock::now();
  try {
    s_.t = tick_ * dt_;
    s_.quat_wxyz = s.quat_wxyz;
    s_.gyro = s.gyro;
    s_.q = s.q;
    s_.dq = s.dq;
    const std::vector<float>& obs = rt_->begin_tick(s_, in);
    const int idx = pool_->index_of(rt_->active_skill());
    const float* a = idx >= 0 ? pool_->run(idx, obs.data(), static_cast<int>(obs.size())) : nullptr;
    const arcade::PdTarget pd = rt_->end_tick(a ? a : nan29_.data(), arcade::kNJ);
    info_ = rt_->tick_info();
    bool finite = true;
    for (int i = 0; i < arcade::kNJ; ++i) {
      qf_[i] = static_cast<float>(pd.q_des[i]);
      finite = finite && std::isfinite(qf_[i]);
    }
    if (finite) q_des_.publish(qf_.data());
    out.ok = a != nullptr && finite;
  } catch (...) {
    out.ok = false;  // 정상 경로엔 없다(core 틱은 던지지 않는다) — q_des 는 그대로(1 kHz 는 직전 값)
  }
  ++tick_;
  out.info = info_;
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
  out.tick_us = us > std::numeric_limits<int>::max() ? std::numeric_limits<int>::max() : static_cast<int>(us);
  if (out.tick_us > max_tick_us_) max_tick_us_ = out.tick_us;
  return out;
}

bool ArcadeHost::read_q_des(std::array<float, 29>& out) const noexcept { return q_des_.read(out); }

arcade::TickInfo ArcadeHost::info() const noexcept { return info_; }

int ArcadeHost::session_count() const noexcept { return pool_ ? pool_->size() : 0; }

const std::vector<std::string>& ArcadeHost::skill_names() const noexcept {
  return rt_ ? rt_->skill_names() : empty_names();
}

const std::vector<std::string>& ArcadeHost::role_names() const noexcept {
  return rt_ ? rt_->role_names() : empty_names();
}

}  // namespace g1
#endif  // G1_HAVE_ARCADE
