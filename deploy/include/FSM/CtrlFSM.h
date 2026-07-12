// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "FSM/EstopChannel.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        currentState->enter();

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        // ── 하드 E-STOP: /dev/shm/g1_estop 폴(≈50Hz) → asserted면 강제 Passive(damping) ──
        if (++estop_poll_tick_ >= 20) {          // 1kHz/20 = 50Hz (ESTOP_STALE_MAX=25 → ~0.5s)
            estop_poll_tick_ = 0;
            estop_asserted_ = fsm_estop_poll(estop_st_);   // default path /dev/shm/g1_estop
        }
        if (estop_asserted_) {
            int passive_id = FSMStringMap.right.at("Passive");   // == 1 (config FSM)
            if (!currentState->isState(passive_id)) {
                for (auto& state : states) {                     // public vector, id 스캔(:101과 동일)
                    if (state->isState(passive_id)) {
                        spdlog::warn("FSM: E-STOP -> forcing Passive from {}",
                                     currentState->getStateString());
                        currentState->exit();
                        currentState = state;
                        currentState->enter();                   // Passive: kp=0, kd, dq=0, tau=0
                        break;
                    }
                }
            }
            currentState->pre_run();
            currentState->run();
            currentState->post_run();
            return;                                              // 전이검사 bypass(자동 재보행 차단)
        }

        // ── 정상 경로(기존 코드 그대로) ──
        currentState->pre_run();
        currentState->run();
        currentState->post_run();

        // Check if need to change state
        int nextStateMode = 0;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].first())
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;

    EstopState estop_st_{};       // /dev/shm/g1_estop 하트비트 추적
    bool       estop_asserted_ = false;
    int        estop_poll_tick_ = 0;   // 1kHz run_을 ~50Hz로 게이팅(20틱마다 폴)
};
