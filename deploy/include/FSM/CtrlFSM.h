// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "RtPriority.h"
#include <atomic>
#include <thread>
#include "LoopDiag.h"
#include <chrono>
#include <cstring>
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
        // 진단 보고 스레드 — 실시간 스레드 대신 «로그를 대신 찍어 준다». nice 로 밀어 두므로
        // 이 스레드가 늦어도 제어에는 영향이 없다(늦으면 그 창의 요약이 한 박자 늦게 나올 뿐).
        diag_report_run_ = true;
        diag_report_th_ = std::thread([this]{
            rtprio::lower_this_thread("diag 보고", 10);
            char buf[768];
            while (diag_report_run_) {
                if (fsm_diag_.take(buf, sizeof buf)) {
                    if (char* nl = std::strchr(buf, '\n')) *nl = '\0';
                    if (fsm_overrun_seen_) spdlog::warn("[diag:fsm] {}", buf + 7);
                    else                   spdlog::info("[diag:fsm] {}", buf + 7);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
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
        diag_report_run_ = false;
        if (diag_report_th_.joinable()) diag_report_th_.join();
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    // 1 kHz 안전 루프의 속도계. 여기가 관절 안전·레이트리밋이 사는 스레드이고, 이 repo 에는
    // 우선순위 설정(sched_setscheduler / affinity)이 «하나도 없다» — 계산 스레드를 하나 더
    // 붙이면 이 루프와 «동등한 자격으로» CPU 를 다툰다. 그게 실제로 얼마나 나쁜지는 재야 안다.
    LoopDiag fsm_diag_{1.0, 1.0};      // 예산 1 ms (1 kHz)
    std::chrono::high_resolution_clock::time_point fsm_prev_;
    bool fsm_first_ = true;

    void run_()
    {
        using fsm_clk = std::chrono::high_resolution_clock;
        // 첫 틱에 자기 자신을 RT 로. 스레드를 SDK 가 만들므로 여기가 유일한 접점이다.
        static thread_local bool rt_done = false;
        if (!rt_done) { rt_done = true; rtprio::raise_this_thread("FSM 1kHz", rtprio::FSM_1KHZ); }
        const auto fsm_t0 = fsm_clk::now();
        struct FsmDiagScope {
            CtrlFSM* self; std::chrono::high_resolution_clock::time_point t0;
            ~FsmDiagScope() {
                const auto t1 = std::chrono::high_resolution_clock::now();
                auto ms = [](auto a, auto b) {
                    return std::chrono::duration<double, std::milli>(b - a).count(); };
                self->fsm_diag_.tick(ms(t0, t1),
                                     self->fsm_first_ ? 0.0 : ms(self->fsm_prev_, t0));
                self->fsm_prev_ = t0; self->fsm_first_ = false;
                if (self->fsm_diag_.overrun() > 0) self->fsm_overrun_seen_.store(true);
                if (self->fsm_diag_.window_closed()) {
                    // 🔴 여기서 spdlog 를 부르지 «않는다» — 기본 sink 가 동기 stdout 이라
                    // 파이프가 느린 순간 1 kHz 루프를 밀리초 단위로 잡아먹는다. 문자열만
                    // 만들어 슬롯에 넣고 보고는 낮은 우선순위 스레드가 한다.
                    static const char* SEG[6] = {"-","-","-","-","-","-"};
                    self->fsm_diag_.publish(SEG);
                    self->fsm_diag_.reset();
                }
            }
        } fsm_scope{this, fsm_t0};   // run_() 의 모든 return 경로에서 찍히게 (E-STOP 조기 return 포함)

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
    std::thread diag_report_th_;
    std::atomic<bool> diag_report_run_{false};
    std::atomic<bool> fsm_overrun_seen_{false};

    EstopState estop_st_{};       // /dev/shm/g1_estop 하트비트 추적
    bool       estop_asserted_ = false;
    int        estop_poll_tick_ = 0;   // 1kHz run_을 ~50Hz로 게이팅(20틱마다 폴)
};
