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

    // 한 프레임의 차원 (history 곱하기 전). reset() 뒤에 유효.
    int dim() const { return buff_.empty() ? 0 : static_cast<int>(buff_.front().size()); }

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