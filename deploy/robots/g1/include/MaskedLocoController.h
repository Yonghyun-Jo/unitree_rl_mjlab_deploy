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
#include <algorithm>
#include <cmath>

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
    float stance_z        = 0.066f;   // LUT 이면 로드 시 GL_STANCE_Z 로 덮어씀
    float stand_deadzone  = 0.0f;     // eff < 이 값이면 base_vel = 0 (그 모드에서만)
  };
  std::array<ModeGait, 6> mode_gait{};   // index = cmd_mode (1..5). [0]은 미사용.
  float walk_max = 1.2f, run_min = 1.7f; // LUT gait 히스테리시스 경계

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
  bool  is_run  = false;    // LUT: walk/run 1비트 (히스테리시스, 속도로 추론하지 않음)
  std::array<float, 3> bv_last = {0.f, 0.f, 0.f};
  std::array<float, 3> bv_ramp_from = {0.f, 0.f, 0.f};
  float bv_blend = 1.0f;
  int   bv_ramp_rem = 0;
  int   arm_rem = 0;

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
      is_run  = gl_select_gait(eff, is_run, walk_max, run_min);
      phase_f = std::fmod(phase_f + gl_stride_freq(eff, is_run) / 50.0f, 1.0f);  // 50Hz 제어
      gl_foot_z(phase_f, eff, is_run, foot_z[0], foot_z[1]);
    } else {
      phase = (phase + 1) % period_steps;
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
