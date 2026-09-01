#pragma once

#include "FSM/State_RLBase.h"
#include "JointSafety.h"
#include "SafetyLog.h"
#include "StateDump.h"
#include "ImuCal.h"
#include "G1Articulation.h"
#include <cnpy.h>
#include <array>
#include <atomic>


class State_Mimic : public FSMState
{
public:
    State_Mimic(int state_mode, std::string state_string);

    void enter();
    void run();
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
        mon_log_summary();   // join 이후라 policy_thread 가 쓴 mon_* 가 전부 보인다(경합 없음)
    }

    class MotionLoader_;

    static std::shared_ptr<MotionLoader_> motion; // for obs computation
    static std::shared_ptr<MotionLoader_> motion_light; // mode5 test-demo clip (stand + upper-body). null if unset.
    static std::shared_ptr<MotionLoader_> motion_demo6; // mode6 demo clip (keyboard '6'). null if unset.
private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<MotionLoader_> motion_; // for saving
    std::shared_ptr<MotionLoader_> motion_light_; // mode5 light-demo loader (owns; motion_light aliases this)
    std::shared_ptr<MotionLoader_> motion_demo6_; // mode6 demo loader (owns; motion_demo6 aliases this)

    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;

    // ── 모드별 발-z 생성 조건 (MaskedLocoController::mode_gait) ──
    void load_gait_cfg(const YAML::Node& g);

    // ── deploy 안전층 config/state (JointSafety.h) ──
    void load_safety_cfg(const YAML::Node& s);
    bool  js_enable_pos_clamp_ = false;
    std::array<float,29> js_pos_lo_{}, js_pos_hi_{};

    // L2: 관절 속도 rate-limit (gated, 기본 off)
    bool js_enable_rate_limit_ = false;
    std::array<float,29> js_max_step_{};   // = vel_max * dt(0.001)
    std::array<float,29> js_q_prev_{};
    bool js_q_prev_valid_ = false;

    // L3: 측정 qd 폭주 감지 (gated, 기본 off). policy_thread(50Hz)에서 검사 -> warn(mode1 래치)/crit(Passive 래치).
    bool  js_enable_qd_guard_ = false;
    float js_qd_warn_ = 0.f, js_qd_crit_ = 0.f;
    int   js_over_ticks_ = 5;               // @50Hz(policy_thread) => 0.1s sustained
    int   js_warn_run_ = 0, js_crit_run_ = 0;
    bool  js_qd_warn_latched_ = false;      // policy_thread 내부 전용
    std::atomic<bool> js_qd_crit_latched_{false};  // policy_thread set, registered_check(1kHz) read

    // ── 안전층 모니터링 (읽기 전용 관측 — 제어 경로에 영향 없음) ──
    // 원칙: 1kHz run()/검사 람다에서는 "세기만" 하고 I/O 를 하지 않는다. 출력은 드문 이벤트
    // (트립·래치)와 exit() 요약뿐 — 1kHz 스레드에서 spdlog 를 돌리면 실시간 지터가 생긴다.
    // 스레드: mon_clamp_*/mon_rate_*/mon_tilt_* = FSM 스레드(1kHz), mon_qd_* = policy_thread(50Hz).
    //   exit() 는 join() 뒤에 읽으므로 경합 없음.
    void mon_log_summary();
    void mon_reset();
    uint32_t mon_clamp_ticks_ = 0, mon_rate_ticks_ = 0;
    float mon_clamp_max_ = 0.f;  int mon_clamp_joint_ = -1;
    float mon_rate_max_  = 0.f;  int mon_rate_joint_  = -1;
    float mon_qd_max_    = 0.f;  int mon_qd_joint_    = -1;
    float mon_tilt_max_deg_ = 0.f;
    // 안전층 사건을 «파일로» 남긴다. 세는 일은 위 mon_* 가 이미 한다 — 여기선 기록만.
    g1::SafetyLog safety_log_;
    // 계측용 상태 덤프 (env G1_STATE_CSV 가 있을 때만. 기본 꺼짐 → 거동 변화 0)
    g1::StateDump state_dump_;
    // IMU 장착 편향 보정 (config.yaml: imu_cal). 기본 0 = 꺼짐 = 종전 거동 비트 동일.
    g1::ImuCal imu_cal_;
    const char* mon_exit_reason_ = nullptr;   // null = 조작자 전이(p/v) 또는 정상 종료
};


class State_Mimic::MotionLoader_
{
public:
    // npz `body_pos_w` 의 body 축 = MuJoCo g1.xml 의 body 순서에서 world 를 뺀 30개
    // (scripts/csv_to_npz.py 가 robot.data.body_link_pos_w[0, :] 를 그대로 저장한다):
    //   0 pelvis / 1 L_hip_pitch / 2 L_hip_roll / 3 L_hip_yaw / 4 L_knee /
    //   5 L_ankle_pitch / **6 L_ankle_roll** / 7 R_hip_pitch … **12 R_ankle_roll** / 13.. 상체
    // 학습(mjlab_g1_motion)의 calc_ref_foot_height 는 FOOT_BODIES =
    // (left_ankle_roll_link, right_ankle_roll_link) 의 world-z 를 쓴다 → 이 두 인덱스와 같다.
    // ⚠ 로봇/에셋이 바뀌면 여기가 조용히 틀어진다. body 수가 다르면 아래에서 has_foot_z=false
    //   로 떨어뜨려 «틀린 값» 대신 «없음» 이 되게 한다 (obs 는 stance 로 폴백).
    static constexpr int NPZ_NUM_BODIES  = 30;
    static constexpr int NPZ_FOOT_IDX[2] = {6, 12};   // [L, R]
    // 발-z 를 못 얻었을 때 쓰는 «두 발 접지» 높이 (= GaitLut GL_STANCE_Z, 생성기의 서있기 값).
    static constexpr float STANCE_Z_FALLBACK = 0.066877f;

    MotionLoader_(std::string motion_file)
    : dt(1.0f / 50.0f)
    {
        load_data_from_npz(motion_file);
        num_frames = dof_positions.size();
        duration = num_frames * dt;

        update(0.0f);
    }

    void load_data_from_npz(const std::string& motion_file)
    {
        cnpy::npz_t npz_data = cnpy::npz_load(motion_file);

        auto body_pos_w  = npz_data["body_pos_w"];   // [frame, body_id, 3]
        auto body_quat_w = npz_data["body_quat_w"];  // [frame, body_id, 4]
        auto joint_pos   = npz_data["joint_pos"];    // [frame, dof]
        auto joint_vel   = npz_data["joint_vel"];    // [frame, dof]

        root_positions.clear();
        root_quaternions.clear();
        dof_positions.clear();
        dof_velocities.clear();
        foot_z_frames.clear();

        const size_t num_frames_npz = body_pos_w.shape[0];
        // 레퍼런스 발 world-z ([z_L, z_R]) — mode>=3 의 foot_z obs 원천. 학습에서 이 모드는
        // 생성기(FOOT_GEN)가 없어 «클립 발 world-z» 를 그대로 obs 로 먹인다.
        has_foot_z = (body_pos_w.shape.size() == 3 &&
                      body_pos_w.shape[1] == NPZ_NUM_BODIES &&
                      body_pos_w.shape[2] == 3);
        if (!has_foot_z) {
            spdlog::warn("motion npz body_pos_w 가 예상({} bodies) 과 다르다 -> ref_foot_height "
                         "는 stance 로 폴백한다 (mode>=3 학습과 불일치)", NPZ_NUM_BODIES);
        }

        for (size_t i = 0; i < num_frames_npz; i++)
        {
            const size_t body_stride_pos  = body_pos_w.shape[1] * body_pos_w.shape[2];
            const size_t body_stride_quat = body_quat_w.shape[1] * body_quat_w.shape[2];

            Eigen::Vector3f root_pos = Eigen::Vector3f::Map(body_pos_w.data<float>() + i * body_stride_pos);
            root_positions.push_back(root_pos);

            Eigen::Quaternionf quat(
                body_quat_w.data<float>()[i * body_stride_quat + 0], // w
                body_quat_w.data<float>()[i * body_stride_quat + 1], // x
                body_quat_w.data<float>()[i * body_stride_quat + 2], // y
                body_quat_w.data<float>()[i * body_stride_quat + 3]  // z
            );
            root_quaternions.push_back(quat);

            if (has_foot_z) {
                const float* bp = body_pos_w.data<float>() + i * body_stride_pos;
                foot_z_frames.push_back({ bp[NPZ_FOOT_IDX[0] * 3 + 2],
                                          bp[NPZ_FOOT_IDX[1] * 3 + 2] });
            }

            Eigen::VectorXf joint_position(joint_pos.shape[1]);
            for (int j = 0; j < joint_pos.shape[1]; j++) {
                joint_position[j] = joint_pos.data<float>()[i * joint_pos.shape[1] + j];
            }

            Eigen::VectorXf joint_velocity(joint_vel.shape[1]);
            for (int j = 0; j < joint_vel.shape[1]; j++) {
                joint_velocity[j] = joint_vel.data<float>()[i * joint_vel.shape[1] + j];
            }

            dof_positions.push_back(joint_position);
            dof_velocities.push_back(joint_velocity);
        }
    }

    void update(float time)
    {
        // LOOP the clip so the masked/demo reference plays continuously (infinite run) —
        // e.g. mode4 dance demo keeps dancing instead of freezing at the last frame after
        // one pass. (VR override bypasses this: accessors return the VR buffer when active.)
        float t = std::max(time, 0.0f);
        int f = static_cast<int>(std::floor(t / dt));
        frame = (num_frames > 0) ? (((f % num_frames) + num_frames) % num_frames) : 0;
    }

    void reset(const isaaclab::ArticulationData & data, float t = 0.0f)
    {
        update(t);
        auto init_to_anchor = isaaclab::yawQuaternion(this->root_quaternion()).toRotationMatrix();
        auto world_to_anchor = isaaclab::yawQuaternion(data.root_quat_w).toRotationMatrix();
        world_to_init_ = world_to_anchor * init_to_anchor.transpose();
    }

    // ── VR teleop override (variant B) ──────────────────────────────────────────
    // When a VR provider is active, the masked obs read the VR reference instead of the
    // clip frame. Fed by g_poll_vr (State_Mimic.cpp) from /dev/shm/g1_vr_ref. obs terms
    // are unchanged — they just call joint_pos()/joint_vel()/root_quaternion().
    // foot_z: VR 트래커/GMR 이 준 레퍼런스 발 world-z [z_L, z_R]. has_foot_z=false 면
    // (구버전 publisher) 값을 안 믿고 obs 가 stance 로 폴백한다.
    void set_vr(const Eigen::VectorXf& dof_pos, const Eigen::VectorXf& dof_vel,
                const Eigen::Quaternionf& root_quat,
                const std::array<float, 2>& foot_z = {0.f, 0.f}, bool has_foot_z_ = false) {
        vr_dof_pos = dof_pos; vr_dof_vel = dof_vel; vr_root_quat = root_quat;
        vr_foot_z = foot_z; vr_has_foot_z = has_foot_z_;
        vr_override = true;
    }
    // Neutral standby (= robot default pose). mode2/3 are a pure VR structure: they read the VR
    // buffer, which holds VR when live or this neutral pose when not — NEVER the dance clip.
    // (Only mode4 replays the clip.) Set once at enter() via set_standby().
    void set_standby(const Eigen::VectorXf& default_dof) {
        vr_standby_dof = default_dof;
        vr_dof_pos = default_dof;
        vr_dof_vel = Eigen::VectorXf::Zero(default_dof.size());
        vr_root_quat = Eigen::Quaternionf::Identity();
        vr_has_foot_z = false;
        vr_override = false;
    }
    void clear_vr() {   // VR dropped -> reset buffer to neutral standby (NOT the clip)
        vr_override = false;
        vr_has_foot_z = false;
        if (vr_standby_dof.size() > 0) {
            vr_dof_pos = vr_standby_dof;
            vr_dof_vel = Eigen::VectorXf::Zero(vr_standby_dof.size());
            vr_root_quat = Eigen::Quaternionf::Identity();
        }
    }

    Eigen::VectorXf root_position() {
        return root_positions[frame];   // global root pos: clip only (mode3 sim; VR later)
    }
    Eigen::Quaternionf root_quaternion() {
        return vr_override ? vr_root_quat : root_quaternions[frame];
    }
    Eigen::VectorXf joint_pos() {
        return vr_override ? vr_dof_pos : dof_positions[frame];
    }
    Eigen::VectorXf joint_vel() {
        return vr_override ? vr_dof_vel : dof_velocities[frame];
    }

    // Clip-forced accessors: ALWAYS return the clip frame, ignoring any VR override.
    // mode4 (dance demo) uses these so it replays the motion ref regardless of VR state.
    Eigen::Quaternionf root_quaternion_clip() { return root_quaternions[frame]; }
    // 레퍼런스 발 높이 [z_L, z_R] (클립). 발-z 가 없으면 «두 발 접지» 로 안전 폴백한다
    // (has_foot_z 로 먼저 거르는 것이 정상 경로이고, 이건 그걸 잊었을 때의 보루).
    std::array<float, 2> foot_z_clip() const {
        if (frame < 0 || frame >= static_cast<int>(foot_z_frames.size()))
            return {STANCE_Z_FALLBACK, STANCE_Z_FALLBACK};
        return foot_z_frames[frame];
    }
    Eigen::VectorXf joint_pos_clip() { return dof_positions[frame]; }
    Eigen::VectorXf joint_vel_clip() { return dof_velocities[frame]; }

    // VR-buffer accessors: ALWAYS return the VR/standby buffer, NEVER the clip.
    // mode1/2/3 use these (mode1's values are masked to zero anyway). This is what makes
    // mode2/3 a VR-only structure: no VR -> neutral standby, VR live -> tracks VR, never dances.
    Eigen::Quaternionf root_quaternion_vr() { return vr_root_quat; }
    Eigen::VectorXf joint_pos_vr() { return vr_dof_pos; }
    Eigen::VectorXf joint_vel_vr() { return vr_dof_vel; }

    bool vr_override = false;
    bool vr_has_foot_z = false;
    std::array<float, 2> vr_foot_z = {0.f, 0.f};
    Eigen::VectorXf vr_dof_pos, vr_dof_vel, vr_standby_dof;
    Eigen::Quaternionf vr_root_quat = Eigen::Quaternionf::Identity();

    float dt;
    int num_frames;
    float duration;

    int frame;
    std::vector<Eigen::VectorXf> root_positions;
    std::vector<Eigen::Quaternionf> root_quaternions;
    std::vector<Eigen::VectorXf> dof_positions;
    std::vector<Eigen::VectorXf> dof_velocities;
    std::vector<std::array<float, 2>> foot_z_frames;   // [z_L, z_R] per frame (클립 레퍼런스)
    bool has_foot_z = false;
    Eigen::Matrix3f world_to_init_;
};


REGISTER_FSM(State_Mimic)