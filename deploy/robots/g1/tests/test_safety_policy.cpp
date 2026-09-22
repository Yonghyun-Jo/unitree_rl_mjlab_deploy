// test_safety_policy.cpp — 넘어짐·qd_warn 규칙 표 (spec §4.7). UprightOnly 모드는 «종전과 문자 그대로».
//   g++ -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include -I/usr/include/eigen3 test_safety_policy.cpp -o /tmp/t && /tmp/t
// HeightEstimator.h(z_fk)를 써서 «서 있다 넘어짐» 을 FK 로 만든다 → Eigen 이 필요하다(run_unit_tests.sh 의 Eigen 루프).
#include "SafetyPolicy.h"
#include "HeightEstimator.h"
#include "Mode5Presets.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>

static int g_fail = 0;
static void chk(bool c, const char* what) { std::printf("  %s %s\n", c ? "ok  " : "FAIL", what); if (!c) ++g_fail; }

// 배포 기본 자세 = config/policy/mimic_masked/260922_v1_m1gen_v2_torso_30k/params/deploy.yaml 의 default_joint_pos
// (배포 관절 순서 legL·legR·waist·armL·armR, 무릎 0.669).
static const float kDefaultPose[29] = {
    -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f, -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.2f, 0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f, 0.2f, -0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f};

int main() {
  using mode_table::Safety;
  using g1::safety::QdWarnAction;
  namespace s = g1::safety;

  std::printf("-- UprightOnly (mode1·2·3): 종전 그대로 --\n");
  for (bool cu : {false, true})
    for (bool rh : {false, true})
      chk(s::orientation_check_applies(Safety::UprightOnly, cu, rh), "UprightOnly: 넘어짐 판정은 명령·이력과 무관하게 적용");
  for (float z : {0.05f, 0.40f, 0.60f, 0.80f})
    chk(s::qd_warn_action(Safety::UprightOnly, z, 80.f) == QdWarnAction::Fallback, "UprightOnly: qd_warn → 폴백");

  std::printf("-- GroundCapable (mode4·5·6): 넘어짐 관문 = 명령 직립 ∧ 최근에 섰음 --\n");
  chk(s::orientation_check_applies(Safety::GroundCapable, true, true), "명령 직립 ∧ 최근에 섰음: 적용 (서 있다 넘어짐)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, false, true), "명령이 낮음: 적용 안 함 (일부러 내려가기)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, true, false), "최근에 선 적 없음: 적용 안 함 (누운 데서 일어나기)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, false, false), "둘 다 아님: 적용 안 함");

  std::printf("-- commanded_upright: 표의 성질로만 (모드 번호 없음) --\n");
  {
    const std::optional<float> NOF;
    const std::optional<double> NOD;
    int n_clip = 0, n_m5 = 0, n_none = 0;
    for (const mode_table::Row& r : mode_table::ROWS) {
      if (r.ref_source == mode_table::RefSource::Clip) {
        ++n_clip;
        chk(s::commanded_upright(r, 0.78f, NOD), "클립 모드: 클립 골반 0.78 → 직립");
        chk(s::commanded_upright(r, 0.65f, NOD), "클립 모드: 경계 0.65 → 직립");
        chk(!s::commanded_upright(r, 0.40f, 0.76), "클립 모드: 클립 골반 0.40 → 아님 (mode5 값은 안 본다)");
        chk(!s::commanded_upright(r, NOF, 0.76), "클립 모드: 활성 클립 없음 → 아님");
        chk(!s::commanded_upright(r, std::nanf(""), NOD), "클립 모드: 골반 NaN → 아님");
      } else if (r.mode5_cmd_live) {
        ++n_m5;
        chk(s::commanded_upright(r, NOF, 0.76), "mode5 명령: 자세 z 0.76 → 직립");
        chk(!s::commanded_upright(r, 0.78f, 0.42), "mode5 명령: 자세 z 0.42 → 아님 (클립 값은 안 본다)");
        chk(!s::commanded_upright(r, 0.78f, NOD), "mode5 명령: 활성 자세 없음 → 아님");
        for (const m5::Preset& p : m5::PRESETS)
          chk(s::commanded_upright(r, NOF, p.z) == (p.z >= s::UPRIGHT_MIN_Z), "mode5 명령: 프리셋마다 z ≥ 0.65 ⇔ 직립");
        chk(s::commanded_upright(r, NOF, m5::PRESETS[m5::ENTER_PRESET].z),
            "mode5 진입 자세는 직립 — 서서 들어오면 넘어짐 판정이 켜진 채로 시작");
      } else {
        ++n_none;
        chk(!s::commanded_upright(r, 0.78f, 0.76), "명령에 높이가 없는 모드: 아님");
      }
    }
    std::printf("     표 %d 행: 클립 %d · mode5 명령 %d · 높이 없음 %d\n",
                static_cast<int>(mode_table::ROWS.size()), n_clip, n_m5, n_none);
    chk(n_clip > 0 && n_m5 > 0, "표에 클립 모드와 mode5 명령 모드가 있다 (위 행들이 실제로 돌았다)");
  }

  std::printf("-- GroundCapable: qd_warn 처분 (종전과 같다) --\n");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.75f, 10.f) == QdWarnAction::Fallback, "직립: 폴백");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.65f, 10.f) == QdWarnAction::Fallback, "경계 z 0.65 는 직립");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.75f, 30.f) == QdWarnAction::Passive, "경계 기울기 30° 는 기울었음: Passive");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.75f, 35.f) == QdWarnAction::Passive, "높지만 기울면: Passive");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.60f, 5.f) == QdWarnAction::Passive, "0.65 아래: Passive");
  chk(s::qd_warn_action(Safety::GroundCapable, 0.20f, 80.f) == QdWarnAction::Passive, "바닥: Passive");

  std::printf("-- RecentHigh (최근 1 s = 50틱 안에 «섰음»: 명령 직립 ∧ z ≥ 0.65 ∧ 기울기 < 57.3°) --\n");
  {
    const float T = s::ORIENT_TRIP_DEG;
    std::printf("     ORIENT_TRIP_RAD = %.4f rad = %.4f°\n", s::ORIENT_TRIP_RAD, T);
    s::RecentHigh rh;                                   // 기본 50틱
    chk(!rh.update(0.30f, 0.f, true), "처음부터 낮으면 최근에 선 적 없음");
    chk(rh.update(0.65f, 0.f, true), "경계 z 0.65 는 «섰음» (1번째 틱)");
    bool held = true;
    for (int t = 2; t <= 50; ++t) held = rh.update(0.30f, 0.f, true) && held;
    chk(held && rh.value(), "선 틱 포함 50틱(1 s) 동안 유지");
    chk(!rh.update(0.30f, 0.f, true), "51번째 틱에 풀림");
    rh.update(0.80f, 0.f, true);
    rh.reset();
    chk(!rh.value() && !rh.update(0.30f, 0.f, true), "reset() 뒤엔 이력 없음");
    chk(!s::RecentHigh().update(0.6499f, 0.f, true), "z 0.65 아래는 «섰음» 이 아니다");
    chk(s::RecentHigh().update(0.80f, T - 0.01f, true), "기울기 57.3° 바로 아래는 «섰음»");
    chk(!s::RecentHigh().update(0.80f, T, true), "기울기 57.3° 부터는 «섰음» 이 아니다 (그 자체로 넘어짐 판정 대상)");
    chk(!s::RecentHigh().update(0.80f, 0.f, false), "명령이 직립이 아니면 높아도 «섰음» 이 아니다");
    chk(!s::RecentHigh().update(std::nanf(""), 0.f, true), "z NaN 은 «섰음» 이 아니다");
    chk(!s::RecentHigh().update(0.80f, std::nanf(""), true), "기울기 NaN 은 «섰음» 이 아니다");
    s::RecentHigh rn;
    rn.update(0.80f, 0.f, true);
    bool held_nan = true;
    for (int t = 2; t <= 50; ++t) held_nan = rn.update(std::nanf(""), 0.f, true) && held_nan;
    chk(held_nan && !rn.update(std::nanf(""), 0.f, true), "NaN 은 카운터를 다시 채우지 않는다 (51번째 틱에 풀림)");
    s::RecentHigh r2;
    r2.update(0.80f, 0.f, true);
    for (int t = 0; t < 30; ++t) r2.update(0.30f, 0.f, true);
    r2.update(0.70f, 5.f, true);                        // 다시 서면 카운터가 새로 찬다
    bool held2 = true;
    for (int t = 2; t <= 50; ++t) held2 = r2.update(0.30f, 0.f, true) && held2;
    chk(held2 && !r2.update(0.30f, 0.f, true), "다시 선 틱부터 50틱 새로 센다");
    // (b) 섰다 → 내려가라는 명령 한 틱 → 기억이 즉시 지워진다 (낮은 자세 누른 뒤 1 s 안에 z 로 되돌리기)
    s::RecentHigh rb;
    rb.update(0.80f, 0.f, true);
    chk(!rb.update(0.78f, 5.f, false) && !rb.value(), "명령이 낮아진 틱에 섰던 기억이 즉시 지워진다");
    chk(!rb.update(0.50f, 60.f, true), "다시 직립 명령이어도 낮고 기운 자세로는 기억이 되살아나지 않는다");
    chk(!s::orientation_check_applies(Safety::GroundCapable, true, rb.value()),
        "→ 마음 바꾸기(낮은 자세 → 곧바로 직립 버튼) 중엔 넘어짐 판정이 닫혀 있다");
  }

  std::printf("-- FK (a): 엉덩이부터 드는 기립 — 골반 60° 숙임·다리 곧음 --\n");
  {
    float bow[29];
    for (int i = 0; i < 29; ++i) bow[i] = kDefaultPose[i];
    const float r60 = 60.f * 3.14159265f / 180.f;
    bow[0] = bow[6] = -r60;                             // hip_pitch: 골반이 숙인 만큼 굽혀 다리를 세운다
    bow[3] = bow[9] = 0.05f;                            // knee 거의 곧음
    bow[4] = bow[10] = 0.f;                             // ankle_pitch
    const Eigen::Quaternionf qb(Eigen::AngleAxisf(r60, Eigen::Vector3f::UnitY()));
    const float z_bow = g1::z_fk(bow, qb);
    const Eigen::Vector3f gb = qb.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f);
    const float tilt_bow = g1::TiltFilter().update({gb.x(), gb.y(), gb.z()});   // 첫 update = 원시값
    std::printf("     인사 자세 z_fk = %.4f m, 기울기 = %.2f°\n", z_bow, tilt_bow);
    chk(z_bow >= s::UPRIGHT_MIN_Z, "인사 자세는 z_fk ≥ 0.65 — 높이만 보던 규칙이면 여기서 기억이 찼다");
    chk(tilt_bow > s::ORIENT_TRIP_DEG, "인사 자세는 기울기 > 57.3° — 관문이 열리면 곧바로 넘어짐 판정");
    s::RecentHigh rh;                                   // 이전 «섰음» 없음 (네발 기기 자세에서 직립 버튼)
    bool closed = true;
    for (int t = 0; t < 60; ++t) {
      const bool recent = rh.update(z_bow, tilt_bow, /*commanded_upright=*/true);
      closed = !s::orientation_check_applies(Safety::GroundCapable, true, recent) && closed;
    }
    chk(closed, "명령 직립 + 인사 자세 60틱: 관문은 닫혀 있다 (거짓 Passive 없음)");
    const float z_up = g1::z_fk(kDefaultPose, Eigen::Quaternionf::Identity());
    chk(s::orientation_check_applies(Safety::GroundCapable, true, rh.update(z_up, 3.f, true)),
        "다 서면(기울기 < 57.3°) 그 틱에 관문이 열린다");
  }

  std::printf("-- FK (c): 기본 자세를 통째로 58° 기울임 (서 있다 넘어짐) --\n");
  {
    const float z_up = g1::z_fk(kDefaultPose, Eigen::Quaternionf::Identity());
    std::printf("     z_fk 직립 = %.4f m\n", z_up);
    chk(z_up >= s::UPRIGHT_MIN_Z, "직립 기본 자세는 «섰다» (z_fk ≥ 0.65)");
    const float r = 58.f * 3.14159265f / 180.f;
    const Eigen::Vector3f axes[4] = {Eigen::Vector3f::UnitY(), -Eigen::Vector3f::UnitY(),
                                     Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX()};
    const char* names[4] = {"+pitch", "-pitch", "+roll", "-roll"};
    // 보고용 숫자: 그 방향으로 기울일 때 z_fk 가 0.65 아래로 내려가는 각 · 57.3°(판정 한계, 1 rad) 의 z_fk.
    for (int a = 0; a < 4; ++a) {
      float cross_deg = -1.f;
      for (int d10 = 0; d10 <= 900; ++d10) {
        const Eigen::Quaternionf qd(Eigen::AngleAxisf(d10 * 0.1f * 3.14159265f / 180.f, axes[a]));
        if (g1::z_fk(kDefaultPose, qd) < s::UPRIGHT_MIN_Z) { cross_deg = d10 * 0.1f; break; }
      }
      const Eigen::Quaternionf q573(Eigen::AngleAxisf(s::ORIENT_TRIP_RAD, axes[a]));
      std::printf("     %s: z_fk < 0.65 가 되는 각 = %.1f°,  57.3° 의 z_fk = %.4f m\n",
                  names[a], cross_deg, g1::z_fk(kDefaultPose, q573));
    }
    for (int a = 0; a < 4; ++a) {
      const Eigen::Quaternionf q(Eigen::AngleAxisf(r, axes[a]));
      const float z58 = g1::z_fk(kDefaultPose, q);
      std::printf("     z_fk 58° %s = %.4f m\n", names[a], z58);
      chk(z58 < s::UPRIGHT_MIN_Z, "58° 에선 z_fk < 0.65 — 높이만으로는 «서 있음» 이 아니다");
      chk(z58 < 0.55f, "58° 에선 z_fk < 0.55 — 옛 규칙(z_fk ≥ 0.55 일 때만 판정)은 여기서 이미 꺼져 있었다");
      s::RecentHigh rh;
      rh.update(z_up, 0.f, true);                       // 10틱 전에 (기울기 < 57.3° 로) 서 있었다
      bool recent = false;
      for (int t = 0; t < 10; ++t) recent = rh.update(z58, 58.f, true);
      chk(s::orientation_check_applies(Safety::GroundCapable, /*commanded_upright=*/true, recent),
          "명령 직립 + 10틱 전 섰음 → 58° 넘어짐을 판정한다");
      s::RecentHigh rl;
      rl.update(z_up, 0.f, true);
      for (int t = 0; t < 9; ++t) rl.update(z58, 58.f, true);
      const bool recent_low = rl.update(z58, 58.f, /*commanded_upright=*/false);
      chk(!s::orientation_check_applies(Safety::GroundCapable, /*commanded_upright=*/false, recent_low),
          "같은 자세라도 명령이 낮으면(일부러 눕기) 판정하지 않는다");
    }
    // 기울기 필터(τ 0.2 s)를 넣은 넘어짐 궤적: 서 있다(10틱) → 일정 속도로 58° 까지. 원시 기울기가 57.3° 를
    // 넘는 틱(과 그 앞 틱 — FSM 스레드는 직전 정책 틱의 관문을 본다)에 관문이 열려 있어야 한다.
    auto fall_caught = [&](const Eigen::Vector3f& axis, int ramp_ticks) {
      s::RecentHigh rh;
      g1::TiltFilter tf;
      bool prev_gate = true, ok = false, crossed = false;
      for (int k = -10; k <= ramp_ticks && !crossed; ++k) {
        const float ang = (k <= 0 ? 0.f : 58.f * k / ramp_ticks) * 3.14159265f / 180.f;
        const Eigen::Quaternionf q(Eigen::AngleAxisf(ang, axis));
        const Eigen::Vector3f g = q.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f);
        const float filt = tf.update({g.x(), g.y(), g.z()});
        const bool gate = s::orientation_check_applies(Safety::GroundCapable, true,
                                                       rh.update(g1::z_fk(kDefaultPose, q), filt, true));
        if (std::acos(std::clamp(-g.z(), -1.f, 1.f)) > s::ORIENT_TRIP_RAD) { crossed = true; ok = gate && prev_gate; }
        prev_gate = gate;
      }
      return crossed && ok;
    };
    for (int a = 0; a < 4; ++a)
      for (int ramp : {25, 50, 100}) {                  // 0.5 s · 1 s · 2 s 에 58°
        char what[128];
        std::snprintf(what, sizeof what, "%s: %.1f s 에 58° 까지 넘어짐 (필터 지연 포함) → 57.3° 틱에 관문 열림",
                      names[a], ramp * 0.02f);
        chk(fall_caught(axes[a], ramp), what);
      }
    for (int a = 0; a < 4; ++a) {                       // 보고용: 이 규칙이 잡는 가장 느린 등속 넘어짐
      int slowest = 0;
      for (int ramp = 5; ramp <= 400; ++ramp) if (fall_caught(axes[a], ramp)) slowest = ramp;
      std::printf("     %s: 등속 넘어짐을 잡는 한계 ≈ %.2f s 에 58° (그보다 느리면 기억이 먼저 풀린다)\n",
                  names[a], slowest * 0.02f);
    }
  }
  return g_fail ? 1 : 0;
}
