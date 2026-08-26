// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once
#include <string>

#include <deque>
#include <vector>
#include <functional>
#include <numeric>

namespace isaaclab
{

class ManagerBasedRLEnv;

using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv*, YAML::Node)>;

struct ObservationTermCfg
{
    YAML::Node params;
    ObsFunc func;
    std::vector<float> clip;
    std::vector<float> scale;
    int history_length = 1;
    bool scale_first = false;
    // obs 계약 대조용(순수 기술 정보 — 계산에는 안 쓴다).
    //   name       = deploy.yaml 의 항 이름 (= C++ REGISTER_OBSERVATION 이름)
    //   train_name = 학습(mjlab) 쪽 같은 항의 이름. deploy.yaml 의 `train_term:` 에서 온다.
    //                둘은 «다르다» — 10 중 8 이 다르고, 그 대응관계가 여태 아무 데도 없었다.
    std::string name;
    std::string train_name;

    // 한 프레임의 차원 (history 곱하기 전).
    // 🔴 «선언값»(deploy.yaml 의 scale 길이)을 먼저 본다. 버퍼는 그 항이 실제로 값을 낸
    //    뒤에야 채워지는데, 어떤 항은 State::enter() 에서 motion 이 붙기 전까지 빈 벡터를
    //    낸다(masked_joint_command 는 active_demo_loader() 가 비면 0 개). 버퍼를 자로 쓰면
    //    기동 시점에 dim 0 이 되어 «계약 위반» 으로 오진한다 — 2026-08-26 sim2sim 에서
    //    실제로 1640 대신 1060 (58x10 이 통째로 빠짐) 이 나와 기동이 거부됐다.
    //    scale 은 12개 슬롯 전부가 항마다 완비하고 있고(확인함), 그 길이가 곧 선언 폭이다.
    int dim() const {
        if (!scale.empty()) return static_cast<int>(scale.size());
        return buff_.empty() ? 0 : static_cast<int>(buff_.front().size());
    }

    void reset(std::vector<float> obs)
    {
        for(int i(0); i < history_length; ++i) add(obs);
    }

    void add(std::vector<float> obs)
    {
        for(int j = 0; j < obs.size(); ++j)
        {
            if(scale_first) {
                if(!scale.empty()) obs[j] *= scale[j];
                if (!clip.empty()) {
                    obs[j] = std::clamp(obs[j], clip[0], clip[1]);
                }
            } else {
                if (!clip.empty()) {
                    obs[j] = std::clamp(obs[j], clip[0], clip[1]);
                }
                if(!scale.empty()) obs[j] *= scale[j];
            }
        }
        buff_.push_back(obs);

        if (buff_.size() > history_length) buff_.pop_front();
    }

    const std::vector<float> & get(int n) const { return buff_[n]; }

    const std::vector<float> get() const
    {
        std::vector<float> concatenated;
        for (const auto& entry : buff_) {
            concatenated.insert(concatenated.end(), entry.begin(), entry.end());
        }
        return concatenated;
    }

    const std::size_t size() const { return std::accumulate(buff_.begin(), buff_.end(), 0,
        [](std::size_t sum, const auto& v) { return sum + v.size(); }); }

private:
    // Complete circular buffer with most recent entry at the end and oldest entry at the beginning.
    std::deque<std::vector<float>> buff_;
};

};