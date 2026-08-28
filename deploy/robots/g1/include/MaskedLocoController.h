// Deploy-clean masked-locomotion controller — C++ port of mjlab_g1_motion
// tasks/mdp/loco_controller.py (+ foot_gen.py). KEEP IN SYNC via the golden vectors in
// that repo (tasks/mdp/golden_loco_controller.json); the two repos stay isolated (no shared
// code), parity is enforced by the copied golden test.
//
// Owns: foot_z generation (quintic swing + biped schedule + speed-coupled height + wz-aware
// gate), base_vel mode-switch spline, mode1 arm-blend. ALL inputs are deploy-available
// (cmd_mode, joystick base_vel, internal counters) — NO privileged sim state. Pure C++
// (no Eigen) so it compiles/tests stand-alone and ports anywhere. Single robot (N=1).
#pragma once
#include "GaitLut.h"
#include <array>
#include <cstdio>
#include <algorithm>
#include <cmath>

// 이 컨트롤러의 «내부 시계» 한 장. 계측(StateDump)이 CSV 로 남긴다.
// 🔴 열 이름(header)과 값(write)이 **이 구조체 안에 나란히** 있다 — 필드를 늘릴 때
//    둘 중 하나만 고치면 tests/test_state_dump.cpp 가 열 개수 불일치로 잡는다.
// 🔴 값은 전부 «컨트롤러가 실제로 쓴 것» 이다. 계측이 다시 계산하지 않는다 —
//    재계산하면 update() 의 로직이 바뀔 때 조용히 다른 값을 찍는다.
struct GaitAux {
  float phase     = 0.f;   // LUT 실수 위상 [0,1) (quintic 이면 phase/period_steps)
  float stride_hz = 0.f;   // update() 가 이 스텝에 실제로 쓴 보폭 주파수 [Hz]
  float eff       = 0.f;   // update() 가 실제로 표에 넣은 속도 (정착 중이면 settle_eff)
  float foot_z_l  = 0.f;
  float foot_z_r  = 0.f;
  float bv_x = 0.f, bv_y = 0.f, bv_wz = 0.f;   // 스플라인 «후» base_vel (obs 로 나가는 그 값)
  float arm_scale = 1.f;
  float switch_a  = 1.f;   // 모드전환 crossfade 가중
  float swing_sc  = 1.f;   // 이 스텝에 실제로 적용된 최소 스윙 배율 (1 = 안 걸림)
  int   is_run    = 0;
  int   cmd_mode  = 0;
  int   lut       = 0;     // 1 = LUT 분기, 0 = quintic 분기
  int   settling  = 0;     // 1 = 정지 정착 중 (한 걸음 더 굴러 발을 모으는 구간)

  static const char* header() {
    return ",phase,stride_hz,eff,foot_z_l,foot_z_r,bv_x,bv_y,bv_wz,"
           "arm_scale,switch_a,swing_sc,is_run,cmd_mode,lut,settling";
  }
  void write(std::FILE* f) const {
    std::fprintf(f, ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d",
                 phase, stride_hz, eff, foot_z_l, foot_z_r,
                 bv_x, bv_y, bv_wz, arm_scale, switch_a, swing_sc,
                 is_run, cmd_mode, lut, settling);
  }
};

struct MaskedLocoController {
  // ---- 모드별 발-z 생성 조건 (deploy.yaml `gait:` 에서 로드) ----------------------
  // ⚠ head 마다 학습 시점이 달라 foot_z 생성 조건이 다르다. 하나로 통일하면 반드시 절반이
  //   어긋난다 — 배포 중인 ONNX 의 각 head 가 무엇으로 학습됐는지는 ONNX_META.json 의
  //   mode{1,2,3}_ckpt 를 그 시점 stage4_mode{N}_env_cfg.py 의 FOOT_GEN 과 대조해 정한다.
  // 기본값 = 종전 C++ 동작(quintic, height_scale 1.0, deadzone 없음). yaml 이 없거나
  //   파싱이 실패하면 이 기본값이 남아 기존 거동이 그대로 유지된다(fail-safe).
  struct ModeGait {
    bool  lut             = false;    // true = 데이터 적합 LUT, false = quintic
    float height_scale    = 1.0f;     // quintic 전용 (LUT 은 표 자체가 진폭을 갖는다)
    float stance_z        = 0.066f;   // LUT 이면 로드 시 그 표의 stance_z 로 덮어씀
    float stand_deadzone  = 0.0f;     // eff < 이 값이면 base_vel = 0 (그 모드에서만)
    // ▼ 2026-08-26 추가. 셋 다 «학습 시점» 이 정하는 값이라 모드별이어야 한다.
    int   table           = 1;        // 구운 표 판번호: 1 = V1(motions), 2 = V2(COLMOv2)
    float cadence         = 1.0f;     // 학습 loco_controller.cadence_by_mode[mode]
    bool  turn_asym       = false;    // gait_lut.turn_asym — 회전 시 바깥발을 더 든다
    // ▼ 2026-08-26. 정지 정착(settle): 명령이 «움직임 -> 정지» 로 바뀌면 즉시 양발을 붙이지 않고
    //   위상 시계를 settle_phase 까지 더 돌려 «한 걸음 더» 구르게 한다. 그 자리가 두 발이 가장
    //   함께 낮은 위상이라 발이 모인 채 멈춘다. 0 = 끔 = 종전 거동 비트 동일.
    //   🔴 모드별이다 — 학습 시점이 정하는 값(cadence/table/turn_asym 과 같은 이유).
    int   settle_steps    = 0;        // 상한 step 수. 위상을 못 지나도 여기서 반드시 끝난다.
    // ▼ 2026-08-28. 최소 스윙 클리어런스 [m]. 표 진폭이 이보다 작으면 «모양은 유지한 채» 키운다.
    //   V2 재적합이 스탠스를 바로잡으면서 저속 스윙이 같이 낮아졌다(eff 0.10 = 2.68 cm).
    //   초고마찰 바닥에서 그 높이는 발이 끌린다. 정책은 명령을 충실히 따르므로(실측 추종비
    //   1.03~1.23) 고칠 것은 정책이 아니라 명령이다. 0 = 끔 = 종전 거동 비트 동일.
    float min_swing       = 0.0f;
  };
  // 모드의 표를 고른다. 기본(미지정) = V1 = 2026-08-26 이전 배포 거동 그대로.
  static const GlTable& table_of(const ModeGait& mg) {
    return (mg.table == 2) ? GL_T_V2 : GL_T_V1;
  }
  std::array<ModeGait, 6> mode_gait{};   // index = cmd_mode (1..5). [0]은 미사용.
  float walk_max = 1.2f, run_min = 1.7f; // LUT gait 히스테리시스 경계
  // 정착 파라미터 — 파이썬 loco_controller 와 같은 층위(steps 만 모드별, 이 둘은 스칼라).
  float settle_eff   = 0.15f;   // 정착 중 LUT 조회 속도 (작은 걸음: 진폭 ~3 cm)
  float settle_phase = 0.07f;   // 이 위상을 지나면 종료. <0 또는 >=1 = 위상조건 없음(예산만)
  // 정착 상태 (배포는 로봇 한 대라 스칼라). was_standing 은 «이미 서 있었다» 로 시작해야
  // 첫 스텝에 정착이 안 열린다 (mode3 는 base_vel 이 항상 0 이다).
  bool  settling      = false;
  int   settle_rem    = 0;
  bool  was_standing  = true;

  // ---- params (must match golden_loco_controller.json "params") ----
  int   period_steps = 43;
  float stance_z     = 0.066f;
  float height_scale = 1.0f;
  float turn_k       = 0.3f;
  float ds_ratio     = 0.2f;
  int   arm_blend_in  = 75;
  int   arm_blend_out = 20;
  int   bv_ramp_steps = 75;
  int   n_lower       = 12;
  // height fit + standing deadzone (foot_gen.py constants)
  static constexpr float H_A = 0.063f, H_B = 0.075f, H_LO = 0.05f, H_HI = 0.40f;
  static constexpr float STAND_EPS = 0.05f;

  // ---- outputs (read by obs terms / action) ----
  std::array<float, 3> base_vel = {0.f, 0.f, 0.f};   // [vx,vy,wz] yaw-local, masked
  std::array<float, 2> foot_z   = {0.066f, 0.066f};  // [z_L, z_R]
  float arm_scale = 1.0f;                            // 1=policy, <1=ease arms to default

  // ---- state ----
  int   phase   = 0;        // quintic: 정수 위상 (period_steps 로 나눔)
  float phase_f = 0.0f;     // LUT: 실수 위상 [0,1), 보폭 주파수로 진행
  float last_stride_hz = 0.0f;  // update() 가 «실제로 쓴» 보폭 주파수 [Hz].
  float last_eff       = 0.0f;  // update() 가 «실제로 표에 넣은» 속도. 정착 중이면 settle_eff.
  float last_swing_sc  = 1.0f;  // 최소 스윙 배율. gl_foot_z 가 «실제로 적용한» 값을 받아 둔다 —
                                // 계측이 eff+표로 되계산하면 표가 바뀔 때 조용히 갈린다.
                                // 둘 다 계측이 재계산하지 않게 기록해 둔다(probe()가 읽는다).
  bool  is_run  = false;    // LUT: walk/run 1비트 (히스테리시스, 속도로 추론하지 않음)
  std::array<float, 3> bv_last = {0.f, 0.f, 0.f};
  std::array<float, 3> bv_ramp_from = {0.f, 0.f, 0.f};
  float bv_blend = 1.0f;
  int   bv_ramp_rem = 0;
  int   arm_rem = 0;

  // 계측용 스냅샷. 🔴 전부 «멤버 직독» 이다 — 이름이 바뀌면 여기서 컴파일이 깨져
  //    (조용히 틀린 값이 아니라) 고치는 자리 바로 옆에서 알려 준다.
  //    호출부(State_Mimic)가 컨트롤러 내부를 알 필요가 없다.
  GaitAux probe(int cmd_mode) const {
    const ModeGait& mg = mode_gait[std::max(1, std::min(cmd_mode, 5))];
    GaitAux g;
    g.lut       = mg.lut ? 1 : 0;
    g.phase     = mg.lut ? phase_f : float(phase) / float(std::max(1, period_steps));
    g.stride_hz = last_stride_hz;                      // 재계산 아님 — update() 가 쓴 값
    g.eff       = last_eff;                            // 재계산 아님 (정착 중이면 settle_eff)
    g.settling  = settling ? 1 : 0;
    g.foot_z_l  = foot_z[0];
    g.foot_z_r  = foot_z[1];
    g.bv_x      = base_vel[0];
    g.bv_y      = base_vel[1];
    g.bv_wz     = base_vel[2];
    g.arm_scale = arm_scale;
    g.switch_a  = switch_alpha;
    g.swing_sc  = last_swing_sc;                       // 재계산 아님 — gl_foot_z 가 쓴 값
    g.is_run    = is_run ? 1 : 0;
    g.cmd_mode  = cmd_mode;
    return g;
  }

  // ---- mode-switch crossfade (mode -> {2,3,4,5}) -------------------------------------------
  // Ease the OUTPUT action from the frozen pre-switch pose to the live action over
  // switch_blend_steps ticks (smoothstep). Zero effect outside a switch (full-speed response).
  // Bridges BOTH the reference jump and the ONNX in-graph multihead head-flip a switch produces,
  // so no raw joint step reaches the motors (onboard velocity/torque protective stop).
  int   switch_blend_steps = 50;    // 1.0s @ 50Hz. config-tunable (config.yaml: switch_blend_steps).
  int   switch_blend_rem   = 0;
  int   switch_new_mode    = 1;
  float switch_alpha       = 1.0f;  // 0 = hold pre-switch pose, 1 = full live (set in update()).
  bool  switch_fresh       = false; // notify_mode_switch arms; run() captures a_hold on 1st apply.
  bool  a_prev_valid       = false;
  std::array<float, 29> a_hold = {};  // frozen pre-switch target (source pose of the crossfade)
  std::array<float, 29> a_prev = {};  // last published target (becomes a_hold at the next switch)
  // lower-body REFERENCE smoothing (mode -> {3,4,5}): the legs are NOT output-blended (that would
  // drag the feet along the ground). Instead the LEG q_ref is ramped from the robot's current
  // measured pose to the clip/VR target (reuses switch_alpha), so the POLICY tracks a smoothly
  // moving leg target and produces natural stepping. Captured/applied in masked_joint_command.
  bool  leg_fresh = false;
  std::array<float, 12> leg_from = {};  // robot leg joint_pos frozen at the switch (obs thread)

  static inline float clampf(float x, float lo, float hi) { return std::max(lo, std::min(hi, x)); }
  static inline float smoothstep01(float s) { s = clampf(s, 0.f, 1.f); return s * s * (3.0f - 2.0f * s); }

  // swing_phase in [0,1] -> swing height (m) above stance, quintic peak ~clearance.
  static float swing_z_profile(float swing_phase, float clearance) {
    static constexpr float A[6] = {0.0f, 0.1f, 5.0f, -18.8f, 12.0f, 9.6f};
    const float t = swing_phase * 0.5f;  // T = 0.5
    float z = 0.f, tp = 1.f;
    for (int k = 0; k < 6; ++k) { z += A[k] * tp; tp *= t; }
    return std::max(0.0f, z * (clearance / 0.10f));
  }

  // (vx,vy,wz) + turn -> effective speed (drives gate + height).
  inline float effective_speed(float vx, float vy, float wz) const {
    return std::hypot(vx, vy) + turn_k * std::abs(wz);
  }

  // eff -> (z_L, z_R) foot heights for the current phase.
  std::array<float, 2> gen_foot_z(float phase01, float eff) const {
    const float clearance = height_scale * clampf(H_A * eff + H_B, H_LO, H_HI);
    const float h = clampf(ds_ratio, 0.0f, 0.49f) / 4.0f;
    const float denom = std::max(1e-6f, 0.5f - 2.0f * h);
    float zr = 0.f, zl = 0.f;
    if (phase01 >= h && phase01 < (0.5f - h)) {          // SSP1: right swings
      const float phi = clampf((phase01 - h) / denom, 0.f, 1.f);
      zr = swing_z_profile(phi, clearance);
    }
    if (phase01 >= (0.5f + h) && phase01 < (1.0f - h)) { // SSP2: left swings
      const float phi = clampf((phase01 - (0.5f + h)) / denom, 0.f, 1.f);
      zl = swing_z_profile(phi, clearance);
    }
    std::array<float, 2> z = {zl + stance_z, zr + stance_z};
    if (eff < STAND_EPS) z = {stance_z, stance_z};       // standing gate
    return z;
  }

  // Call when cmd_mode changes: spline base_vel from last command + (mode1) start arm-blend.
  void notify_mode_switch(int new_mode) {
    bv_ramp_from = bv_last;
    bv_ramp_rem  = bv_ramp_steps;
    bv_blend     = 0.0f;
    if (new_mode == 1) arm_rem = arm_blend_in + arm_blend_out;
    if (new_mode >= 2) {                 // crossfade for mode2/3/4/5 (mode1 handled by arm-blend above)
      switch_new_mode  = new_mode;
      switch_blend_rem = switch_blend_steps;
      switch_fresh     = true;           // run() freezes the pre-switch pose on the next apply
    }
    if (new_mode >= 3) leg_fresh = true; // full-body: also ramp the LEG reference (masked_joint_command)
  }

  // Advance one control step; cache base_vel / foot_z / arm_scale. Call ONCE per step pre-obs.
  void update(const std::array<float, 3>& joystick_bv, int cmd_mode) {
    // 0) 이번 스텝에 쓸 모드별 발-z 조건을 고른다. gen_foot_z 가 멤버를 읽으므로 여기서
    //    멤버에 실어 준다(시그니처 유지 -> 기존 golden 테스트 그대로 통과).
    const ModeGait& mg = mode_gait[std::max(1, std::min(cmd_mode, 5))];
    height_scale = mg.height_scale;
    stance_z     = mg.stance_z;
    // 1) base_vel spline (lerp last->target), then mask mode3.
    if (bv_ramp_rem > 0) {
      bv_ramp_rem -= 1;
      bv_blend = 1.0f - float(bv_ramp_rem) / std::max(1, bv_ramp_steps);
    }
    std::array<float, 3> bv = joystick_bv;
    if (bv_blend < 1.0f) {
      for (int i = 0; i < 3; ++i) bv[i] = bv_ramp_from[i] + (bv[i] - bv_ramp_from[i]) * bv_blend;
    }
    if (cmd_mode >= 3) bv = {0.f, 0.f, 0.f};
    // 1-b) standing deadzone: 아주 작은 명령은 0 으로 눌러 «서있기» 로 보낸다. 학습(mode2)
    //      에서 clip 의 vx/wz 가 완전히 0 이 안 돼 계속 구르던 것을 막으려고 넣은 것이라,
    //      배포에 없으면 같은 명령에도 학습은 서있고 배포는 걷는다 = obs 두 항(base_vel +
    //      foot_z)이 동시에 어긋난다. 파이썬은 cmd_mode==2 로 하드게이팅하는데, 여기서는
    //      모드별 표가 그 역할을 한다(mode2 에만 값을 주면 동일).
    if (mg.stand_deadzone > 0.0f &&
        effective_speed(bv[0], bv[1], bv[2]) < mg.stand_deadzone) {
      bv = {0.f, 0.f, 0.f};
    }
    bv_last = bv;
    base_vel = bv;
    // 2) foot_z from the SPLINED command. LUT 이면 실수 위상시계(보폭 주파수로 진행),
    //    아니면 종전 고정주기 quintic. 파이썬과 같이 «활성 분기의 위상만» 진행시킨다.
    const float eff = effective_speed(bv[0], bv[1], bv[2]);
    if (mg.lut) {
      // 표·케이던스·정지임계·정착은 그 head 가 «무엇으로 학습됐는가» 다 — 모드에서 읽는다.
      // 파이썬: loco_controller.update 의 lut 분기와 1:1 (정착 상태기계 포함).
      const GlTable& T = table_of(mg);
      // ── 정착 창: «움직임 -> 정지» 로 바뀌는 순간 연다 ──
      const bool stand = (eff <= T.stand_eps);          // 파이썬 foot_gen.is_standing 과 같은 비교
      if (stand && !was_standing && mg.settle_steps > 0) settle_rem = mg.settle_steps;
      if (!stand) settle_rem = 0;                       // 명령이 돌아오면 즉시 취소
      was_standing = stand;
      settling = (settle_rem > 0);
      // 정착 중에는 «명령 0» 대신 저속 한 행을 조회해 낮은 스윙 궤적을 받는다.
      float eff_g = settling ? settle_eff : eff;
      is_run  = gl_select_gait(eff_g, is_run, walk_max, run_min);
      const float prev_ph = phase_f;
      last_stride_hz = gl_stride_freq(T, eff_g, is_run) * mg.cadence;
      last_eff       = eff_g;   // ⚠ eff 가 아니다 — 정착 중에는 settle_eff 로 표를 조회한다
      phase_f = std::fmod(phase_f + last_stride_hz / 50.0f, 1.0f);
      // 두 발이 «가장 함께 낮은» 위상을 지나면 종료. 못 지나도 예산이 다하면 끝난다
      // («정지를 눌렀는데 안 멈춤» 을 만들지 않는다 — 조작성 우선).
      if (settling) {
        if (gl_phase_crossed(prev_ph, phase_f, settle_phase)) settle_rem = 0;
        else if (settle_rem > 0) settle_rem -= 1;
      }
      settling = (settle_rem > 0);
      eff_g = settling ? settle_eff : eff;
      gl_foot_z(T, phase_f, eff_g, is_run, mg.turn_asym, bv[2], foot_z[0], foot_z[1],
                mg.min_swing, &last_swing_sc);
    } else {
      phase = (phase + 1) % period_steps;
      last_stride_hz = 50.0f / float(std::max(1, period_steps));
      last_eff       = eff;
      last_swing_sc  = 1.0f;    // quintic 분기엔 최소 스윙이 없다
      const float phase01 = float(phase) / period_steps;
      foot_z = gen_foot_z(phase01, eff);
    }
    // 3) arm-blend: ease-in (1->0 over _in) then release (0->1 over _out).
    if (arm_rem > 0) {
      const int out = arm_blend_out;
      arm_scale = (arm_rem > out) ? float(arm_rem - out) / std::max(1, arm_blend_in)
                                  : 1.0f - float(arm_rem) / std::max(1, out);
      arm_rem -= 1;
    } else {
      arm_scale = 1.0f;
    }
    // 4) mode-switch crossfade weight (0->1 over switch_blend_steps), same timeline as arm_rem.
    //    Applied to the OUTPUT action in apply_switch_blend (run()).
    if (switch_blend_rem > 0) {
      switch_alpha = smoothstep01(1.0f - float(switch_blend_rem) / std::max(1, switch_blend_steps));
      switch_blend_rem -= 1;
    } else {
      switch_alpha = 1.0f;
    }
  }

  // Ease the upper-joint TARGETS toward the default pose during the mode1 blend.
  // ⚠ Unlike the Python side (which scales the RAW action, where raw=0 ⇒ default), the C++
  // deploy applies this to PROCESSED targets (= raw*scale + offset), so we must interpolate
  // toward the default pose explicitly — scaling the processed value toward 0 would command
  // the arms to 0 rad (straight limbs), NOT the default pose. arm_scale itself is identical
  // to Python (golden-verified); only the application context differs.
  //   action:      processed motor targets (in place), length num_dof.
  //   default_pose: per-joint default position (= the action offset / default_joint_pos).
  void apply_arm_blend(float* action, const float* default_pose, int num_dof) const {
    if (arm_scale < 1.0f)
      for (int i = n_lower; i < num_dof; ++i)
        action[i] = default_pose[i] + arm_scale * (action[i] - default_pose[i]);
  }

  // Crossfade the OUTPUT action from the frozen pre-switch pose (a_hold) toward the live action
  // using switch_alpha (0->1 over the window, computed in update()). Applied ONLY during a
  // mode -> {2,3,4,5} transition; zero effect otherwise (full-speed response).
  // ⚠ UPPER BODY ONLY ([n_lower,num_dof) = waist+arms). The LEGS are NEVER output-blended: a
  // joint-space blend would drag the feet along the ground (no stepping). Legs are policy-live +
  // their REFERENCE is ramped in masked_joint_command (mode3/4/5) so the policy steps naturally.
  // Also NaN/Inf-guards (hold last-good target per joint) and tracks a_prev every call so a_hold
  // captures the true pre-switch pose at the next switch.
  //   action: processed motor targets (in place), length num_dof.
  void apply_switch_blend(float* action, int num_dof) {
    const int n = std::min(num_dof, static_cast<int>(a_prev.size()));
    if (a_prev_valid)
      for (int i = 0; i < n; ++i) if (!std::isfinite(action[i])) action[i] = a_prev[i];
    if (switch_fresh) {                                  // freeze the pre-switch pose once
      for (int i = 0; i < n; ++i) a_hold[i] = a_prev_valid ? a_prev[i] : action[i];
      switch_fresh = false;
    }
    if (switch_alpha < 1.0f)
      for (int i = n_lower; i < n; ++i)                  // waist+arms only; legs live (ref-smoothed)
        action[i] = (1.0f - switch_alpha) * a_hold[i] + switch_alpha * action[i];
    for (int i = 0; i < n; ++i) a_prev[i] = action[i];
    a_prev_valid = true;
  }
};
