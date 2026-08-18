// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "isaaclab/utils/utils.h"

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
    }

    // 마지막 step() 의 구간 시간 (µs). 계측 전용 — 읽는 쪽이 없으면 아무 일도 안 한다.
    // 20 ms 예산이 obs 조립 / ONNX 추론 / action 처리 중 «어디로» 가는지 이걸로만 알 수 있다.
    double last_obs_us = 0.0, last_ort_us = 0.0, last_act_us = 0.0;

    void step()
    {
        using diag_clk = std::chrono::high_resolution_clock;
        episode_length += 1;
        robot->update();
        const auto t0 = diag_clk::now();
        auto obs = observation_manager->compute();
        const auto t1 = diag_clk::now();
        auto action = alg->act(obs);
        const auto t2 = diag_clk::now();
        action_manager->process_action(action);
        const auto t3 = diag_clk::now();
        last_obs_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        last_ort_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
        last_act_us = std::chrono::duration<double, std::micro>(t3 - t2).count();
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
};

};