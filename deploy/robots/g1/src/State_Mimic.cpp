#include "State_Mimic.h"
#include "unitree_articulation.h"
#include "MaskedLocoController.h"   // deploy-clean foot_z gen + base_vel spline + arm-blend
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <algorithm>   // std::clamp
#include <array>
#include <string>
#include <cstdio>      // printf (base_vel command readout)

static Eigen::Quaternionf init_quat;
std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion = nullptr;

// ===== Masked-3mode student (mjlab_g1_motion stage2_masked) =====
// cmd_mode in {1,2,3}: 1=full-auto(0,0) 2=upper-teleop(1,0) 3=full-teleop(1,1).
// mask_upper = cmd_mode>=2 (upper joints/anchor tracked); mask_lower = cmd_mode>=3 (lower tracked).
// N_LOWER=12 (both legs). Default 1 = full-auto locomotion: fully observable on hardware
// (no global position needed), the safe deploy default. mode3 needs global position -> sim only.
static int g_cmd_mode = 1;
static constexpr int G1_N_LOWER = 12;
static inline bool g_mask_upper() { return g_cmd_mode >= 2; }
static inline bool g_mask_lower() { return g_cmd_mode >= 3; }

// Deploy-clean controller (1:1 with mjlab_g1_motion loco_controller.py; golden-verified).
// Updated once per policy step (see policy_thread) BEFORE obs are computed; obs terms +
// run() read its cached base_vel / foot_z / arm_scale. ⚠ params assume 50 Hz control.
static MaskedLocoController g_loco;
static int g_prev_cmd_mode = 1;

// Accumulated keyboard velocity command (walker_teleop.py style: each keypress ±STEP, space=reset).
// Coexists with the joystick stick (base_vel_command sums them). Edge-triggered so one tap = one step.
static float g_kb_vx = 0.0f, g_kb_vy = 0.0f, g_kb_wz = 0.0f;
static std::string g_kb_last = "";
static constexpr float KB_STEP = 0.1f, KB_MAXV = 1.0f, KB_MAXW = 0.6f;

// Poll joystick d-pad + keyboard for mode switch (1/2/3) and keyboard velocity accumulation.
// Called every policy step (same thread as obs). Both input modes are always live.
static void g_poll_inputs(isaaclab::ManagerBasedRLEnv* env)
{
    // --- joystick d-pad -> mode (avoids A/B used by FSM transitions) ---
    if (auto joy = env->robot->data.joystick) {
        if      (joy->left.on_pressed)  g_cmd_mode = 1;
        else if (joy->up.on_pressed)    g_cmd_mode = 2;
        else if (joy->right.on_pressed) g_cmd_mode = 3;
    }
    // --- keyboard (edge-triggered: act once per key change) ---
    if (!FSMState::keyboard) return;
    std::string k = FSMState::keyboard->key();
    if (k == g_kb_last) return;
    g_kb_last = k;
    if (k.empty()) return;
    bool vel_changed = false;
    if      (k == "w") { g_kb_vx = std::clamp(g_kb_vx + KB_STEP, -KB_MAXV, KB_MAXV); vel_changed = true; }  // forward
    else if (k == "s") { g_kb_vx = std::clamp(g_kb_vx - KB_STEP, -KB_MAXV, KB_MAXV); vel_changed = true; }  // backward
    else if (k == "a") { g_kb_vy = std::clamp(g_kb_vy + KB_STEP, -KB_MAXV, KB_MAXV); vel_changed = true; }  // strafe left
    else if (k == "d") { g_kb_vy = std::clamp(g_kb_vy - KB_STEP, -KB_MAXV, KB_MAXV); vel_changed = true; }  // strafe right
    else if (k == "q") { g_kb_wz = std::clamp(g_kb_wz + KB_STEP, -KB_MAXW, KB_MAXW); vel_changed = true; }  // yaw CCW (반시계)
    else if (k == "e") { g_kb_wz = std::clamp(g_kb_wz - KB_STEP, -KB_MAXW, KB_MAXW); vel_changed = true; }  // yaw CW  (시계)
    else if (k == " ") { g_kb_vx = g_kb_vy = g_kb_wz = 0.0f;                          vel_changed = true; }  // stop
    else if (k == "1") { g_cmd_mode = 1; printf("\r\n[cmd_mode] -> 1 (full-auto)\r\n");      fflush(stdout); }
    else if (k == "2") { g_cmd_mode = 2; printf("\r\n[cmd_mode] -> 2 (upper-teleop)\r\n");    fflush(stdout); }
    else if (k == "3") { g_cmd_mode = 3; printf("\r\n[cmd_mode] -> 3 (full-track/sim)\r\n");  fflush(stdout); }

    // Print the base_vel command ONLY when a velocity key changed it (not every frame).
    // mode3 zeroes base_vel downstream, so show that; modes 1/2 use it as-is.
    if (vel_changed) {
        bool active = !g_mask_lower();   // base_vel applied only in modes 1,2
        printf("\r\n[base_vel cmd] vx=%+.2f  vy=%+.2f  wz=%+.2f   (mode %d%s)\r\n",
               g_kb_vx, g_kb_vy, g_kb_wz, g_cmd_mode, active ? "" : ", base_vel=0 in mode3");
        fflush(stdout);
    }
}


// Raw joystick + keyboard base_vel target [vx,vy,wz] (UNmasked; the controller masks mode3).
// Fed to g_loco.update() each step; base_vel_command obs returns the controller's splined value.
// ⚠ V_LIN/V_ANG (joystick) and KB_MAXV/W (keyboard) should match the training base_vel
// distribution (clip pelvis velocity, ~m/s); tune in sim2sim.
static std::array<float, 3> g_joystick_base_vel(isaaclab::ManagerBasedRLEnv* env)
{
    float jx = 0.0f, jy = 0.0f, jw = 0.0f;
    if (auto joy = env->robot->data.joystick) {
        constexpr float V_LIN = 1.0f, V_ANG = 1.0f;
        jx =  V_LIN * joy->ly();
        jy = -V_LIN * joy->lx();
        jw = -V_ANG * joy->rx();
    }
    return { std::clamp(g_kb_vx + jx, -KB_MAXV, KB_MAXV),
             std::clamp(g_kb_vy + jy, -KB_MAXV, KB_MAXV),
             std::clamp(g_kb_wz + jw, -KB_MAXW, KB_MAXW) };
}

Eigen::Quaternionf robot_quat_w(isaaclab::ManagerBasedRLEnv* env)
{
    using G1Type = unitree::BaseArticulation<LowState_t::SharedPtr>;
    G1Type* robot = dynamic_cast<G1Type*>(env->robot.get());

    auto root_quat = env->robot->data.root_quat_w;
    auto & motors = robot->lowstate->msg_.motor_state();

    Eigen::Quaternionf torso_quat = root_quat \
        * Eigen::AngleAxisf(motors[12].q(), Eigen::Vector3f::UnitZ()) \
        * Eigen::AngleAxisf(motors[13].q(), Eigen::Vector3f::UnitX()) \
        * Eigen::AngleAxisf(motors[14].q(), Eigen::Vector3f::UnitY());

//    return root_quat;
    return torso_quat;
}

Eigen::Quaternionf motion_anchor_quat_w(std::shared_ptr<State_Mimic::MotionLoader_> loader)
{
    const auto root_quat = loader->root_quaternion();
    const auto joint_pos = loader->joint_pos();
    Eigen::Quaternionf torso_quat = root_quat \
        * Eigen::AngleAxisf(joint_pos[12], Eigen::Vector3f::UnitZ()) \
        * Eigen::AngleAxisf(joint_pos[13], Eigen::Vector3f::UnitX()) \
        * Eigen::AngleAxisf(joint_pos[14], Eigen::Vector3f::UnitY());

//    return root_quat;
    return torso_quat;
}


namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(motion_command)
{
    auto loader = State_Mimic::motion;
    std::vector<float> data;

    auto motion_joint_pos = loader->joint_pos();
    auto motion_joint_vel = loader->joint_vel();

    data.insert(data.end(),
                motion_joint_pos.data(),
                motion_joint_pos.data() + motion_joint_pos.size());
    data.insert(data.end(),
                motion_joint_vel.data(),
                motion_joint_vel.data() + motion_joint_vel.size());
    return data;
}

REGISTER_OBSERVATION(motion_anchor_ori_b)
{
    auto loader = State_Mimic::motion;
    std::vector<float> out;

    auto real_quat_w = robot_quat_w(env);
    auto ref_quat_w  = motion_anchor_quat_w(loader);

    auto rot_ = (init_quat * ref_quat_w).conjugate() * real_quat_w;
    auto rot = rot_.toRotationMatrix().transpose();

    Eigen::Matrix<float, 6, 1> data;
    data << rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1);
    return std::vector<float>(data.data(), data.data() + data.size());
}

// ---- Masked-3mode obs (mirror mjlab_g1_motion g1_mimic_env.calc_masked_*) ----

// q_ref(29)+q̇_ref(29) = 58, lower[:12]/upper[12:] groups zeroed per cmd_mode (mode3 = no mask).
REGISTER_OBSERVATION(masked_joint_command)
{
    auto loader = State_Mimic::motion;
    auto q  = loader->joint_pos();
    auto qd = loader->joint_vel();
    const bool mu = g_mask_upper(), ml = g_mask_lower();
    std::vector<float> data;
    data.reserve(q.size() + qd.size());
    for (const Eigen::VectorXf* v : {&q, &qd}) {
        for (int i = 0; i < v->size(); ++i) {
            float x = (*v)[i];
            if (i <  G1_N_LOWER && !ml) x = 0.0f;   // lower group generated -> 0
            if (i >= G1_N_LOWER && !mu) x = 0.0f;   // upper group generated -> 0
            data.push_back(x);
        }
    }
    return data;
}

// [mask_upper, mask_lower] as float.
REGISTER_OBSERVATION(command_mask)
{
    return std::vector<float>{ g_mask_upper() ? 1.0f : 0.0f, g_mask_lower() ? 1.0f : 0.0f };
}

// yaw-local base velocity command [vx, vy, wz]. The controller (g_loco, updated once per
// step) splines this across mode switches and zeroes it in mode3 (mask_lower). Raw joystick
// target is computed by g_joystick_base_vel() and fed to g_loco.update() in policy_thread.
REGISTER_OBSERVATION(base_vel_command)
{
    return std::vector<float>(g_loco.base_vel.begin(), g_loco.base_vel.end());
}

// Reference foot height [z_L, z_R] — foot-trajectory command (deploy source = controller's
// spline generator). Mirrors mjlab_g1_motion g1_mimic_env.calc_ref_foot_height. MUST be the
// LAST policy obs term (after base_vel + mask), matching the trained obs order.
REGISTER_OBSERVATION(ref_foot_height)
{
    return std::vector<float>(g_loco.foot_z.begin(), g_loco.foot_z.end());
}

// pelvis anchor orientation error as Rot6D in robot base frame. mjlab anchor = PELVIS (=root),
// NOT torso (so no waist-joint offset, unlike motion_anchor_ori_b). Masked to 0 in mode1.
REGISTER_OBSERVATION(masked_root_ori_b)
{
    std::vector<float> out(6, 0.0f);
    if (g_mask_upper()) {
        auto loader = State_Mimic::motion;
        Eigen::Quaternionf real_quat_w = env->robot->data.root_quat_w;   // pelvis (IMU)
        Eigen::Quaternionf ref_quat_w  = loader->root_quaternion();      // pelvis ref
        auto rot_ = (init_quat * ref_quat_w).conjugate() * real_quat_w;
        auto rot  = rot_.toRotationMatrix().transpose();
        out = { rot(0,0), rot(0,1), rot(1,0), rot(1,1), rot(2,0), rot(2,1) };
    }
    return out;
}

// pelvis anchor POSITION error in robot base frame. x,y kept only in mode3 (mask_lower),
// z kept in modes 2,3 (mask_upper).
// ⚠ SIM2REAL OBSERVABILITY: real G1 LowState has NO global position (only IMU orientation).
// mode1 -> [0,0,0] EXACT (matches training, all generated). mode2 z (height) and mode3 x,y need
// odometry/height estimation not yet wired -> currently 0 (placeholder). See plan Phase 3b.
REGISTER_OBSERVATION(masked_root_pos_b)
{
    return std::vector<float>(3, 0.0f);
}

}
}


State_Mimic::State_Mimic(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if(!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }

    // Motion
    motion_ = std::make_shared<MotionLoader_>(motion_file.string());
    spdlog::info("Loaded motion file '{}' with duration {:.2f}s", motion_file.stem().string(), motion_->duration);
    motion = motion_;
    if(cfg["time_start"]) {
        float time_start = cfg["time_start"].as<float>();
        time_range_[0] = std::clamp(time_start, 0.0f, motion_->duration);
    } else {
        time_range_[0] = 0.0f;
    }
    if(cfg["time_end"]) {
        float time_end = cfg["time_end"].as<float>();
        time_range_[1] = std::clamp(time_end, 0.0f, motion_->duration);
    } else {
        time_range_[1] = motion_->duration;
    }
    std::string end_state = "Velocity";
    if (cfg["end_state"]) {
        end_state = cfg["end_state"].as<std::string>();
    }

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        articulation
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    const auto & joy = FSMState::lowstate->joystick;
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return (env->episode_length * env->step_dt) > time_range_[1]; }, // time out
            FSMStringMap.right.at(end_state)
        )
    );
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); }, // bad orientation
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_Mimic::enter()
{
    // set gain
    for (int i = 0; i < env->robot->data.joint_stiffness.size(); i++)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[i].dq() = 0;
        lowcmd->msg_.motor_cmd()[i].tau() = 0;
    }

    motion = motion_; // set for specific motion
    env->reset();
    // Start policy thread
    policy_thread_running = true;
    policy_thread = std::thread([this]{
        using clock = std::chrono::high_resolution_clock;
        const std::chrono::duration<double> desiredDuration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

        // Initialize timing
        const auto start = clock::now();
        auto sleepTill = start + dt;

        motion->reset(env->robot->data, time_range_[0]);
        auto ref_yaw = isaaclab::yawQuaternion(motion->root_quaternion()).toRotationMatrix();
        auto robot_yaw = isaaclab::yawQuaternion(robot_quat_w(env.get())).toRotationMatrix();
        init_quat = robot_yaw * ref_yaw.transpose();
        env->reset();

        while (policy_thread_running)
        {
            env->robot->update();
            g_poll_inputs(env.get());   // joystick d-pad + keyboard (mode 1/2/3 + WASD/QE vel)
            // Controller: detect a mode switch (spline base_vel + mode1 arm-blend), then advance
            // one step so the obs (base_vel_command / ref_foot_height) see fresh values.
            if (g_cmd_mode != g_prev_cmd_mode) {
                g_loco.notify_mode_switch(g_cmd_mode);
                g_prev_cmd_mode = g_cmd_mode;
            }
            g_loco.update(g_joystick_base_vel(env.get()), g_cmd_mode);
            motion->update(env->episode_length * env->step_dt + time_range_[0]);
            env->step();

            // Sleep
            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}


void State_Mimic::run()
{
    auto action = env->action_manager->processed_actions();   // = raw*scale + offset (target q)
    // mode1 arm-blend: ease the upper-joint TARGETS toward the default pose, then release.
    // Interpolate toward default_joint_pos (= the action offset; joint_names=[.*] so action
    // index == robot joint index). arm_scale is 1.0 except during a mode1 transition.
    g_loco.apply_arm_blend(action.data(), env->robot->data.default_joint_pos.data(),
                           static_cast<int>(action.size()));
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}