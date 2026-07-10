#pragma once

#include "FSM/State_RLBase.h"
#include <cnpy.h>


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
    }

    class MotionLoader_;

    static std::shared_ptr<MotionLoader_> motion; // for obs computation
    static std::shared_ptr<MotionLoader_> motion_light; // mode5 test-demo clip (stand + upper-body). null if unset.
private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<MotionLoader_> motion_; // for saving
    std::shared_ptr<MotionLoader_> motion_light_; // mode5 light-demo loader (owns; motion_light aliases this)

    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;
};


class State_Mimic::MotionLoader_
{
public:
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

        const size_t num_frames_npz = body_pos_w.shape[0];

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
    void set_vr(const Eigen::VectorXf& dof_pos, const Eigen::VectorXf& dof_vel,
                const Eigen::Quaternionf& root_quat) {
        vr_dof_pos = dof_pos; vr_dof_vel = dof_vel; vr_root_quat = root_quat;
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
        vr_override = false;
    }
    void clear_vr() {   // VR dropped -> reset buffer to neutral standby (NOT the clip)
        vr_override = false;
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
    Eigen::VectorXf joint_pos_clip() { return dof_positions[frame]; }
    Eigen::VectorXf joint_vel_clip() { return dof_velocities[frame]; }

    // VR-buffer accessors: ALWAYS return the VR/standby buffer, NEVER the clip.
    // mode1/2/3 use these (mode1's values are masked to zero anyway). This is what makes
    // mode2/3 a VR-only structure: no VR -> neutral standby, VR live -> tracks VR, never dances.
    Eigen::Quaternionf root_quaternion_vr() { return vr_root_quat; }
    Eigen::VectorXf joint_pos_vr() { return vr_dof_pos; }
    Eigen::VectorXf joint_vel_vr() { return vr_dof_vel; }

    bool vr_override = false;
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
    Eigen::Matrix3f world_to_init_;
};


REGISTER_FSM(State_Mimic)