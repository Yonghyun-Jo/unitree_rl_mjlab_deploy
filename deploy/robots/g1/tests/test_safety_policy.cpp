// test_safety_policy.cpp — 넘어짐·qd_warn 규칙 표 (spec §4.7). UprightOnly 모드는 «종전과 문자 그대로».
//   g++ -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include -I/usr/include/eigen3 test_safety_policy.cpp -o /tmp/t && /tmp/t
// HeightEstimator.h(z_fk)를 써서 «서 있다 넘어짐» 을 FK 로 만든다 → Eigen 이 필요하다(run_unit_tests.sh 의 Eigen 루프).
#include "SafetyPolicy.h"
#include "HeightEstimator.h"
#include "Mode5Presets.h"
#include "golden_motion_preview.inc"   // 실기 v1 슬롯 demo6 클립(dance1_subject2) 의 실제 프레임 120개 — 골반 자세·높이
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

  std::printf("-- GroundCapable 등급식 (mode5 행이 쓰는 식): 넘어짐 관문 = 명령 직립 ∧ 최근에 섰음 --\n");
  chk(s::orientation_check_applies(Safety::GroundCapable, true, true), "명령 직립 ∧ 최근에 섰음: 적용 (서 있다 넘어짐)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, false, true), "명령이 낮음: 적용 안 함 (일부러 내려가기)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, true, false), "최근에 선 적 없음: 적용 안 함 (누운 데서 일어나기)");
  chk(!s::orientation_check_applies(Safety::GroundCapable, false, false), "둘 다 아님: 적용 안 함");

  std::printf("-- commanded_upright · 관문(행): 표의 성질로만 (모드 번호 없음) --\n");
  {
    const std::optional<float> NOF;
    const std::optional<double> NOD;
    const float T = s::ORIENT_TRIP_DEG;
    int n_clip = 0, n_m5 = 0, n_none = 0;
    for (const mode_table::Row& r : mode_table::ROWS) {
      if (r.ref_source == mode_table::RefSource::Clip) {
        ++n_clip;
        // 클립 모드의 «명령 직립» = 클립 현재 프레임 골반 «기울기» < 57.3° (Ruling 34 I-2 — 높이가 아니다)
        chk(s::commanded_upright(r, 5.f, NOD), "클립 모드: 골반 기울기 5° → 직립");
        chk(s::commanded_upright(r, 39.f, NOD), "클립 모드: 깊이 앉는 프레임(골반 낮음·기울기 39°) → 직립 (높이는 안 본다)");
        chk(s::commanded_upright(r, T - 0.01f, NOD), "클립 모드: 57.3° 바로 아래 → 직립");
        chk(!s::commanded_upright(r, T, NOD), "클립 모드: 57.3° 부터 → 아님 (그 구간은 클립이 눕힌다)");
        chk(!s::commanded_upright(r, 70.f, 0.76), "클립 모드: 기울기 70° → 아님 (mode5 값은 안 본다)");
        chk(s::commanded_upright(r, NOF, 0.76), "클립 모드: 활성 클립 없음 → 직립(모르면 판정 적용, 안전측)");
        chk(s::commanded_upright(r, std::nanf(""), NOD), "클립 모드: 기울기 NaN → 직립(모르면 판정 적용, 안전측)");
        // 관문은 «최근 섰음» 을 묻지 않는다 — 클립은 참조 궤적이다
        chk(!s::gate_needs_recent_high(r), "클립 모드: 관문은 «최근 1 s 에 섰음» 을 묻지 않는다");
        chk(s::orientation_check_applies(r, s::commanded_upright(r, 39.f, NOD), /*recently_high=*/false),
            "클립 모드: 깊은 앉기 프레임(39°) · 최근 섰음 없음 → 넘어짐 판정 적용");
        chk(!s::orientation_check_applies(r, s::commanded_upright(r, 70.f, NOD), /*recently_high=*/true),
            "클립 모드: 70° 로 눕히는 프레임 → 판정 끔 (최근 섰어도)");
      } else if (r.mode5_cmd_live) {
        ++n_m5;
        chk(s::commanded_upright(r, NOF, 0.76), "mode5 명령: 자세 z 0.76 → 직립");
        chk(!s::commanded_upright(r, 5.f, 0.42), "mode5 명령: 자세 z 0.42 → 아님 (클립 값은 안 본다)");
        chk(!s::commanded_upright(r, 5.f, NOD), "mode5 명령: 활성 자세 없음 → 아님");
        for (const m5::Preset& p : m5::PRESETS)
          chk(s::commanded_upright(r, NOF, p.z) == (p.z >= s::UPRIGHT_MIN_Z), "mode5 명령: 프리셋마다 z ≥ 0.65 ⇔ 직립");
        chk(s::commanded_upright(r, NOF, m5::PRESETS[m5::ENTER_PRESET].z),
            "mode5 진입 자세는 직립 — 서서 들어오면 넘어짐 판정이 켜진 채로 시작");
        // 관문은 종전 그대로: 명령 직립 ∧ 최근 1 s 에 섰음
        chk(s::gate_needs_recent_high(r), "mode5 명령: 관문은 «최근 섰음» 을 묻는다 (종전 그대로)");
        chk(s::orientation_check_applies(r, true, true) && !s::orientation_check_applies(r, true, false)
            && !s::orientation_check_applies(r, false, true), "mode5 명령: 명령 직립 ∧ 최근 섰음 일 때만 (종전 그대로)");
      } else {
        ++n_none;
        chk(!s::commanded_upright(r, 5.f, 0.76), "명령에 자세가 없는 모드: 아님");
        if (r.safety == Safety::UprightOnly)
          chk(s::orientation_check_applies(r, false, false), "UprightOnly 행: 관문은 늘 열림 (종전 그대로)");
        else
          chk(!s::orientation_check_applies(r, false, true), "명령에 자세가 없는 GroundCapable 행(기기): 판정 안 함");
      }
    }
    std::printf("     표 %d 행: 클립 %d · mode5 명령 %d · 명령 자세 없음 %d\n",
                static_cast<int>(mode_table::ROWS.size()), n_clip, n_m5, n_none);
    chk(n_clip > 0 && n_m5 > 0, "표에 클립 모드와 mode5 명령 모드가 있다 (위 행들이 실제로 돌았다)");
  }

  std::printf("-- quat_tilt_deg: 쿼터니언의 기울기 (yaw 무관 · TiltFilter 의 acos(-g_z) 와 같은 양) --\n");
  {
    chk(std::fabs(g1::quat_tilt_deg(Eigen::Quaternionf::Identity())) < 1e-3f, "단위 쿼터니언 → 0°");
    const Eigen::Quaternionf q39(Eigen::AngleAxisf(39.f * 3.14159265f / 180.f, Eigen::Vector3f::UnitY()));
    chk(std::fabs(g1::quat_tilt_deg(q39) - 39.f) < 1e-3f, "pitch 39° → 39°");
    const Eigen::Quaternionf q70 = Eigen::Quaternionf(Eigen::AngleAxisf(2.1f, Eigen::Vector3f::UnitZ()))
                                 * Eigen::Quaternionf(Eigen::AngleAxisf(70.f * 3.14159265f / 180.f, Eigen::Vector3f::UnitX()));
    chk(std::fabs(g1::quat_tilt_deg(q70) - 70.f) < 1e-3f, "yaw 120° · roll 70° → 70° (yaw 무관)");
    const Eigen::Quaternionf q2(2.f * q39.w(), 2.f * q39.x(), 2.f * q39.y(), 2.f * q39.z());
    chk(std::fabs(g1::quat_tilt_deg(q2) - 39.f) < 1e-3f, "정규화 안 된 쿼터니언(|q|=2)도 같은 각");
    chk(std::isnan(g1::quat_tilt_deg(Eigen::Quaternionf(std::nanf(""), 0.f, 0.f, 0.f))), "NaN 쿼터니언 → NaN (받는 쪽이 «아님»)");
    float worst = 0.f;
    for (int i = 0; i < 64; ++i) {                     // 로봇 쪽 측정(TiltFilter: acos(-g_z), g = Rᵀ(0,0,-1))과 같은 양
      const Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::AngleAxisf(0.37f * i, Eigen::Vector3f::UnitZ()))
          * Eigen::Quaternionf(Eigen::AngleAxisf(0.05f * i, Eigen::Vector3f(1.f, 0.3f, 0.f).normalized()));
      const Eigen::Vector3f g = q.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f);
      worst = std::max(worst, std::fabs(g1::quat_tilt_deg(q) - g1::TiltFilter().update({g.x(), g.y(), g.z()})));
    }
    std::printf("     quat_tilt_deg ↔ TiltFilter 원시값 최대 차 %.2e°\n", worst);
    chk(worst < 1e-2f, "쿼터니언 기울기 = 로봇 기울기 측정과 같은 정의");
  }

  std::printf("-- 실제 클립: 실기 v1 슬롯 demo6(dance1_subject2) 프레임 %d 개 — 낮게 앉는 구간에서도 관문이 열려 있다 --\n", kClipN);
  {
    const mode_table::Row* clip_row = nullptr;
    for (const mode_table::Row& r : mode_table::ROWS) if (r.ref_source == mode_table::RefSource::Clip) { clip_row = &r; break; }
    chk(clip_row != nullptr, "표에 클립 재생 행이 있다");
    int n_low = 0, n_open = 0, n_open_old = 0;
    float tilt_max = 0.f, z_min = 1e9f;
    for (int i = 0; clip_row && i < kClipN; ++i) {
      const Eigen::Quaternionf q(kClipQ[i][0], kClipQ[i][1], kClipQ[i][2], kClipQ[i][3]);   // wxyz
      const float tilt = g1::quat_tilt_deg(q);
      tilt_max = std::max(tilt_max, tilt); z_min = std::min(z_min, kClipZ[i]);
      if (kClipZ[i] < s::UPRIGHT_MIN_Z) ++n_low;
      // 새 관문: 클립 기울기만. 최근 섰음은 «없음» 으로 준다 — 묻지 않으므로 결과가 같아야 한다.
      if (s::orientation_check_applies(*clip_row, s::commanded_upright(*clip_row, tilt, std::nullopt), false)) ++n_open;
      // 옛 관문(B2 이전 원고, 714cb3a): 클립 골반 높이 ≥ 0.65 ∧ 최근 섰음 — 여기선 «섰음» 을 준다(가장 너그럽게)
      if (kClipZ[i] >= s::UPRIGHT_MIN_Z) ++n_open_old;
    }
    std::printf("     골반 높이 최소 %.3f m · < 0.65 m 프레임 %d · 골반 기울기 최대 %.1f° · 관문 열림 %d/%d (옛 높이 규칙 %d/%d)\n",
                z_min, n_low, tilt_max, n_open, kClipN, n_open_old, kClipN);
    chk(n_low > 0, "이 구간엔 골반 < 0.65 m 인 프레임이 있다 (옛 높이 규칙이 판정을 끄던 곳)");
    chk(n_open == kClipN, "모든 프레임에서 넘어짐 판정 관문이 열려 있다 (base 와 같은 보호)");
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
    chk(!rh.update(0.30f, 0.f, 0.f, true), "처음부터 낮으면 최근에 선 적 없음");
    chk(rh.update(0.65f, 0.f, 0.f, true), "경계 z 0.65 는 «섰음» (1번째 틱)");
    bool held = true;
    for (int t = 2; t <= 50; ++t) held = rh.update(0.30f, 0.f, 0.f, true) && held;
    chk(held && rh.value(), "선 틱 포함 50틱(1 s) 동안 유지");
    chk(!rh.update(0.30f, 0.f, 0.f, true), "51번째 틱에 풀림");
    rh.update(0.80f, 0.f, 0.f, true);
    rh.reset();
    chk(!rh.value() && !rh.update(0.30f, 0.f, 0.f, true), "reset() 뒤엔 이력 없음");
    chk(!s::RecentHigh().update(0.6499f, 0.f, 0.f, true), "z 0.65 아래는 «섰음» 이 아니다");
    chk(s::RecentHigh().update(0.80f, T - 0.01f, T - 0.01f, true), "기울기 57.3° 바로 아래는 «섰음»");
    chk(!s::RecentHigh().update(0.80f, T, T, true), "기울기 57.3° 부터는 «섰음» 이 아니다 (그 자체로 넘어짐 판정 대상)");
    chk(!s::RecentHigh().update(0.80f, 0.f, 0.f, false), "명령이 직립이 아니면 높아도 «섰음» 이 아니다");
    chk(!s::RecentHigh().update(std::nanf(""), 0.f, 0.f, true), "z NaN 은 «섰음» 이 아니다");
    chk(!s::RecentHigh().update(0.80f, std::nanf(""), std::nanf(""), true), "기울기 NaN 은 «섰음» 이 아니다");
    chk(!s::RecentHigh().update(0.80f, T + 1.f, T - 10.f, true), "원시만 57.3° 이상이어도 «섰음» 이 아니다 (숙이는 중 — 걸러진 쪽이 늦다)");
    chk(!s::RecentHigh().update(0.80f, T - 10.f, T + 1.f, true), "걸러진 쪽만 57.3° 이상이어도 «섰음» 이 아니다 (일어나는 중)");
    chk(!s::RecentHigh().update(0.80f, std::nanf(""), 0.f, true) && !s::RecentHigh().update(0.80f, 0.f, std::nanf(""), true),
        "기울기 한쪽만 NaN 이어도 «섰음» 이 아니다");
    s::RecentHigh rn;
    rn.update(0.80f, 0.f, 0.f, true);
    bool held_nan = true;
    for (int t = 2; t <= 50; ++t) held_nan = rn.update(std::nanf(""), 0.f, 0.f, true) && held_nan;
    chk(held_nan && !rn.update(std::nanf(""), 0.f, 0.f, true), "NaN 은 카운터를 다시 채우지 않는다 (51번째 틱에 풀림)");
    s::RecentHigh r2;
    r2.update(0.80f, 0.f, 0.f, true);
    for (int t = 0; t < 30; ++t) r2.update(0.30f, 0.f, 0.f, true);
    r2.update(0.70f, 5.f, 5.f, true);                        // 다시 서면 카운터가 새로 찬다
    bool held2 = true;
    for (int t = 2; t <= 50; ++t) held2 = r2.update(0.30f, 0.f, 0.f, true) && held2;
    chk(held2 && !r2.update(0.30f, 0.f, 0.f, true), "다시 선 틱부터 50틱 새로 센다");
    // (b) 섰다 → 내려가라는 명령 한 틱 → 기억이 즉시 지워진다 (낮은 자세 누른 뒤 1 s 안에 z 로 되돌리기)
    s::RecentHigh rb;
    rb.update(0.80f, 0.f, 0.f, true);
    chk(!rb.update(0.78f, 5.f, 5.f, false) && !rb.value(), "명령이 낮아진 틱에 섰던 기억이 즉시 지워진다");
    chk(!rb.update(0.50f, 60.f, 60.f, true), "다시 직립 명령이어도 낮고 기운 자세로는 기억이 되살아나지 않는다");
    chk(!s::orientation_check_applies(Safety::GroundCapable, true, rb.value()),
        "→ 마음 바꾸기(낮은 자세 → 곧바로 직립 버튼) 중엔 넘어짐 판정이 닫혀 있다");
  }

  std::printf("-- upright_for_memory · TiltFilter::raw --\n");
  {
    chk(s::upright_for_memory(Safety::UprightOnly, false) && s::upright_for_memory(Safety::UprightOnly, true),
        "UprightOnly 모드는 명령에 높이가 없어도 기억을 채운다 (정의상 직립)");
    chk(s::upright_for_memory(Safety::GroundCapable, true) && !s::upright_for_memory(Safety::GroundCapable, false),
        "GroundCapable 모드는 명령 그대로");
    g1::TiltFilter tf;
    tf.update({0.f, 0.f, -1.f});
    const float v = tf.update({1.f, 0.f, 0.f});        // 90° 계단
    chk(std::fabs(tf.raw() - 90.f) < 1e-3f && v < 20.f && tf.value() == v,
        "raw() = 직전 update 의 원시 기울기(90°), value() 는 걸러진 값(늦다)");
    tf.reset();
    chk(tf.raw() == 0.f && tf.value() == 0.f, "reset() 은 원시값도 0 으로");
    // 비유한 IMU 한 틱은 버린다 (최종 검토 M-10 — 옛 필터는 NaN 한 번에 체류 끝까지 NaN 이었다)
    g1::TiltFilter tn;
    tn.update({0.f, 0.f, -1.f});
    const float v0 = tn.update({0.5f, 0.f, -0.8660254f});            // 30° 쪽으로 한 틱
    const float r0 = tn.raw();
    const float vn = tn.update({std::nanf(""), std::nanf(""), std::nanf("")});
    chk(vn == v0 && tn.value() == v0 && tn.raw() == r0, "NaN 한 틱은 버린다 — 걸러진 값·원시값 그대로");
    const float v1 = tn.update({0.5f, 0.f, -0.8660254f});
    chk(std::isfinite(v1) && v1 > v0 && std::fabs(tn.raw() - 30.f) < 1e-2f, "다음 정상 틱부터 이어 걸러진다 (NaN 에 갇히지 않는다)");
    g1::TiltFilter tf0;                                               // 첫 틱이 NaN 이어도 다음 정상 틱이 원시값으로 시작
    tf0.update({std::nanf(""), 0.f, std::nanf("")});
    chk(std::fabs(tf0.update({0.5f, 0.f, -0.8660254f}) - 30.f) < 1e-2f, "첫 틱 NaN → 다음 정상 틱이 원시값으로 시작");
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
      const bool recent = rh.update(z_bow, tilt_bow, tilt_bow, /*commanded_upright=*/true);
      closed = !s::orientation_check_applies(Safety::GroundCapable, true, recent) && closed;
    }
    chk(closed, "명령 직립 + 인사 자세 60틱: 관문은 닫혀 있다 (거짓 Passive 없음)");
    const float z_up = g1::z_fk(kDefaultPose, Eigen::Quaternionf::Identity());
    chk(s::orientation_check_applies(Safety::GroundCapable, true, rh.update(z_up, 3.f, 3.f, true)),
        "다 서면(기울기 < 57.3°) 그 틱에 관문이 열린다");
  }

  std::printf("-- FK (ii): 엉덩이부터 숙이며 내려가다(1 s 에 65°) 원시 57.3° 를 넘는 틱에 직립 버튼 --\n");
  {
    const int N = 50;
    s::RecentHigh rh;
    g1::TiltFilter tf;
    bool found = false, closed = true;
    float z_at = 0.f, raw_at = 0.f, filt_at = 0.f;
    bool cmd = false;                                   // 내려가는 중 = 낮은 자세 명령
    for (int k = -10; k <= N + 60; ++k) {
      const float deg = k <= 0 ? 0.f : (k >= N ? 65.f : 65.f * k / N);
      const float rr = deg * 3.14159265f / 180.f;
      float pose[29];
      for (int i = 0; i < 29; ++i) pose[i] = kDefaultPose[i];
      pose[0] = pose[6] = -rr; pose[3] = pose[9] = 0.05f; pose[4] = pose[10] = 0.f;
      const Eigen::Quaternionf q(Eigen::AngleAxisf(rr, Eigen::Vector3f::UnitY()));
      const Eigen::Vector3f g = q.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f);
      const float filt = tf.update({g.x(), g.y(), g.z()});
      const float z = g1::z_fk(pose, q);
      if (!found && tf.raw() > s::ORIENT_TRIP_DEG) {    // 이 틱에 직립 버튼
        found = true; cmd = true; z_at = z; raw_at = tf.raw(); filt_at = filt;
      }
      rh.update(z, tf.raw(), filt, s::upright_for_memory(Safety::GroundCapable, cmd));
      if (found) closed = !s::orientation_check_applies(Safety::GroundCapable, cmd, rh.value()) && closed;
    }
    std::printf("     직립 버튼 틱: z_fk = %.4f m, 원시 = %.2f°, 걸러진 = %.2f°\n", z_at, raw_at, filt_at);
    chk(found && z_at >= s::UPRIGHT_MIN_Z && filt_at < s::ORIENT_TRIP_DEG,
        "그 틱엔 z_fk ≥ 0.65 ∧ 걸러진 기울기 < 57.3° — 걸러진 쪽만 보면 여기서 기억이 찼다");
    chk(closed, "원시·걸러진 둘 다 보므로 기억이 안 차고, 65° 로 60틱 버티는 동안 관문은 닫혀 있다");
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
      rh.update(z_up, 0.f, 0.f, true);                       // 10틱 전에 (기울기 < 57.3° 로) 서 있었다
      bool recent = false;
      for (int t = 0; t < 10; ++t) recent = rh.update(z58, 58.f, 58.f, true);
      chk(s::orientation_check_applies(Safety::GroundCapable, /*commanded_upright=*/true, recent),
          "명령 직립 + 10틱 전 섰음 → 58° 넘어짐을 판정한다");
      s::RecentHigh rl;
      rl.update(z_up, 0.f, 0.f, true);
      for (int t = 0; t < 9; ++t) rl.update(z58, 58.f, 58.f, true);
      const bool recent_low = rl.update(z58, 58.f, 58.f, /*commanded_upright=*/false);
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
                                                       rh.update(g1::z_fk(kDefaultPose, q), tf.raw(), filt, true));
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
    // (i) 서서 걷다(UprightOnly — 명령에 높이 없음) 넘어지는 도중, z_fk < 0.65 가 된 틱에 GroundCapable 모드로
    //     바꾼다(명령 직립 — mode5 진입 자세 · 선 클립). State_Mimic 과 같은 식으로 기억·관문을 굴린다.
    auto fall_with_switch_caught = [&](const Eigen::Vector3f& axis, int ramp_ticks) {
      s::RecentHigh rh;
      g1::TiltFilter tf;
      Safety cls = Safety::UprightOnly;
      bool cmd = false, prev_gate = true, ok = false, crossed = false, switched = false;
      for (int k = -10; k <= ramp_ticks && !crossed; ++k) {
        const float ang = (k <= 0 ? 0.f : 58.f * k / ramp_ticks) * 3.14159265f / 180.f;
        const Eigen::Quaternionf q(Eigen::AngleAxisf(ang, axis));
        const Eigen::Vector3f g = q.conjugate() * Eigen::Vector3f(0.f, 0.f, -1.f);
        const float filt = tf.update({g.x(), g.y(), g.z()});
        const float z = g1::z_fk(kDefaultPose, q);
        if (!switched && z < s::UPRIGHT_MIN_Z) { cls = Safety::GroundCapable; cmd = true; switched = true; }
        rh.update(z, tf.raw(), filt, s::upright_for_memory(cls, cmd));
        const bool gate = s::orientation_check_applies(cls, cmd, rh.value());
        if (tf.raw() > s::ORIENT_TRIP_DEG) { crossed = true; ok = switched && gate && prev_gate; }
        prev_gate = gate;
      }
      return crossed && ok;
    };
    for (int a = 0; a < 4; ++a)
      for (int ramp : {25, 50, 100}) {
        char what[160];
        std::snprintf(what, sizeof what, "%s: 직립 모드에서 넘어지는 중(%.1f s 에 58°) z_fk<0.65 틱에 4·5 로 전환 → 57.3° 틱에 관문 열림",
                      names[a], ramp * 0.02f);
        chk(fall_with_switch_caught(axes[a], ramp), what);
      }
    {
      s::RecentHigh rh;
      rh.update(z_up, 0.f, 0.f, s::upright_for_memory(Safety::UprightOnly, false));   // mode1 에서 서 있다
      const float z58 = g1::z_fk(kDefaultPose, Eigen::Quaternionf(Eigen::AngleAxisf(r, -Eigen::Vector3f::UnitY())));
      bool recent = false;
      for (int t = 0; t < 10; ++t) recent = rh.update(z58, 58.f, 58.f, s::upright_for_memory(Safety::GroundCapable, true));
      chk(s::orientation_check_applies(Safety::GroundCapable, true, recent),
          "직립 모드에서 섰음 → GroundCapable(명령 직립)로 바꾼 뒤 10틱 58° → 판정한다");
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
