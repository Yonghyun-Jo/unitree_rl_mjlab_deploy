#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

#include <algorithm>
#include <cmath>

namespace isaaclab
{
namespace mdp
{

// 상체 기울기가 limit_angle(rad)을 넘으면 true → 등록한 FSM이 Passive(damping)로 전이.
// 직립이면 projected_gravity_b = (0,0,-1) → -data[2] = 1 → acos = 0.
// ⚠ clamp 필수: IMU 쿼터니언이 정확히 정규화돼 있지 않으면 |data| != 1 이라 acos 인자가 1을 넘어
//   NaN 이 되고, NaN > limit_angle 은 false 라 안전검사가 조용히 통과한다(fail-open).
inline bool bad_orientation(ManagerBasedRLEnv* env, float limit_angle = 1.0)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    const float cos_tilt = std::clamp(-data[2], -1.0f, 1.0f);
    return std::acos(cos_tilt) > limit_angle;
}

} 
} 