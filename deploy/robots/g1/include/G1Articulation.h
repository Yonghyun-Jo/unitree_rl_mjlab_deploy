#pragma once
// G1Articulation.h — 공용 BaseArticulation 에 «IMU 편향 보정» 만 얹은 g1 전용 파생. (🅐 구역)
//
// # 왜 파생인가 — 여기 말고는 걸 자리가 없다
// 처음엔 State_Mimic 의 정책 루프에서 `robot->update()` 직후에 보정을 걸었다. **안 먹었다.**
// `manager_based_rl_env.h::step()` 이 obs 를 계산하기 **직전에 `robot->update()` 를 한 번 더**
// 부르기 때문이다 — 보정이 그때 덮여 사라진다. (실측으로 잡았다: 주입 +4.22° 를 걸어도
// 정책이 본 앞기울기가 IMU 원본과 소수점까지 같았다.)
//
// 그 step() 은 **🅑 공용**(6대 로봇)이라 못 건드린다. 대신 `update()` 가 `virtual` 이고
// g1 이 articulation 을 **자기가 생성**하므로(State_Mimic.cpp), 파생에서 override 하면
// **어느 경로로 불리든** 보정이 따라간다. 공용 코드 무변경.
//
// # 교훈
// 「update() 뒤에 한 줄 넣으면 된다」가 아니라 **「그 자료를 마지막으로 쓰는 사람이 누구인가」**
// 를 봐야 했다. 계측(GaitAux.pg_*)이 없었으면 이 덮어쓰기를 못 봤다.
#include "unitree_articulation.h"
#include "ImuCal.h"

namespace g1 {

template <class LowStatePtr>
class G1Articulation : public unitree::BaseArticulation<LowStatePtr> {
public:
    using Base = unitree::BaseArticulation<LowStatePtr>;
    using Base::Base;

    // 기본 0 = 꺼짐 → Base::update() 만 부른 것과 «비트 동일».
    ImuCal imu_cal;

    void update() override {
        Base::update();
        imu_cal.apply(this->data);
    }
};

}  // namespace g1
