#pragma once
// ImuCal.h — IMU 장착 편향을 빼서 «골반의 참 자세» 를 만든다. (g1 전용, 🅐 구역)
//
// # 왜
// 이 정책은 상태추정이 없다. 자세 정보는 `projected_gravity_b`(3) + `ang_vel`(3) 뿐이고,
// 그중 **중력 벡터가 유일한 «어디가 수직인가» 신호**다. 그런데
//   · 학습은 projected_gravity 에 **zero-mean 지터**만 준다 — 「상수만큼 틀릴 수 있다」를
//     한 번도 안 가르친다 (관절엔 encoder_bias ±0.01 rad 가 있는데 IMU 에만 없다).
//   · 배포에는 보정·영점 항이 **repo 전체에 없었다**.
// 그래서 IMU 가 δ° 기울어 붙어 있으면 정책은 **몸을 δ° 기운 채로 «수직» 으로 유지한다.**
// 정책 입장에선 기운 게 아니다 — 「기울어진 걸 모르는 느낌」이 정확한 묘사다.
// 배경 = mjlab 실험노트 260821_deploy_imu_bias_diagnosis (기전) ·
//        unitree_rl_mjlab 260901_backward_lean_diagnosis (실기 측정 4.2°).
//
// # 수식
// IMU 가 골반에 대해 q_mount 만큼 돌아 붙어 있으면 보고값은 IMU 프레임의 자세다:
//     q_보고 = q_골반 · q_mount     ⇒     q_골반 = q_보고 · q_mount⁻¹
// 즉 **몸 프레임 오른쪽 곱**이다. (실측이 「몸에 고정된 성분」으로 나온 것과 같은 형태.)
//
// # 무엇을 안 하나
// **yaw 는 건드리지 않는다.** 진행 방향은 편향과 무관하고, mode>=2 의 앵커가 yaw 를 쓴다.
// **바닥 경사도 안 뺀다.** 그건 실제 지형이지 센서 오차가 아니다 — 몸에 고정된 성분만 뺀다.
//
// # 안전
// 기본값 0 → `apply()` 가 **아무것도 안 한다**(거동 비트 동일). 값이 틀리면 로봇의 유일한
// 균형 센서를 틀리게 만드는 것이므로, 넣기 전에 **방위를 바꿔가며 실측**해서 몸-고정 성분과
// 바닥 경사를 분리할 것. 재는 법 = 양발 지지 정지 구간에서 «발바닥 법선 vs IMU 중력»
// (`deploy/scripts/imu_bias_fit.py`).
//
// # sim 에서는 자동으로 꺼진다 (`applies_to_network`)
// config.yaml 은 **sim(--network=lo) 과 실기가 같은 파일**이다. 실기 IMU 를 상쇄하는 값을 sim 에
// 걸면 «편향 없는 로봇에 틀린 보정» 이 되어 넘어진다(2026-09-01 실측 65°). 그래서 `lo` 에서는
// config 의 값을 걸지 않는다. sim 에 편향을 «일부러 만들» 때는 환경변수(G1_IMU_CAL_DEG)로 —
// 그건 이 게이트 뒤에 적용된다.
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <Eigen/Dense>

namespace g1 {

class ImuCal {
public:
    // pitch/roll [deg]. 둘 다 0 이면 꺼진 것과 같다.
    void set(float pitch_deg, float roll_deg, const char* src = "config.yaml") {
        pitch_deg_ = pitch_deg; roll_deg_ = roll_deg; src_ = src ? src : "?";
        on_ = (std::fabs(pitch_deg_) > 1e-6f) || (std::fabs(roll_deg_) > 1e-6f);
        // q_mount⁻¹ = (roll 그다음 pitch) 의 역. 작은 각이라 순서 영향은 2차항이지만,
        // 재현성을 위해 순서를 고정해 둔다: q_mount = Rz(0)·Ry(pitch)·Rx(roll).
        const Eigen::Quaternionf qm =
            Eigen::Quaternionf(Eigen::AngleAxisf(pitch_deg_ * kDeg, Eigen::Vector3f::UnitY())) *
            Eigen::Quaternionf(Eigen::AngleAxisf(roll_deg_  * kDeg, Eigen::Vector3f::UnitX()));
        q_corr_ = qm.conjugate();          // 단위 쿼터니언이라 역 = 켤레
    }

    // 환경변수 override (실험용). "pitch,roll" [deg]. 없으면 아무것도 안 한다.
    //   G1_IMU_CAL_DEG="4.2,0"  — sim 에 «일부러» 편향을 만들어 증상을 재현할 때 쓴다.
    bool set_from_env(const char* name = "G1_IMU_CAL_DEG") {
        const char* v = std::getenv(name);
        if (!v || !*v) return false;
        float p = 0.f, r = 0.f;
        if (std::sscanf(v, "%f,%f", &p, &r) < 1) return false;
        set(p, r, name);
        return true;
    }

    // config 의 보정을 이 인터페이스에 걸어도 되는가. `lo` = sim(실기 온보드에서도 모터에 안 닿는다,
    // 2026-08-20 실측) → sim 의 IMU 는 정확하므로 걸지 않는다. 그 밖(eth0/enp5s0 …)은 실기.
    static bool applies_to_network(const std::string& iface) { return iface != "lo"; }

    // update() 직후에 부른다. data 는 isaaclab ArticulationData.
    template <class Data>
    void apply(Data& data) const {
        if (!on_) return;                                   // 🔴 꺼짐 = 종전 경로 비트 동일
        const Eigen::Quaternionf q_true = data.root_quat_w * q_corr_;
        data.root_quat_w = q_true;
        data.projected_gravity_b = q_true.conjugate() * data.GRAVITY_VEC_W;
    }

    bool on() const { return on_; }
    std::string describe() const {
        char b[160];
        if (!on_) std::snprintf(b, sizeof b, "꺼짐 (pitch=roll=0 → 종전 거동 그대로)");
        else std::snprintf(b, sizeof b, "pitch=%+.2f° roll=%+.2f°  [%s]",
                           pitch_deg_, roll_deg_, src_.c_str());
        return std::string(b);
    }

private:
    static constexpr float kDeg = 3.14159265358979323846f / 180.0f;
    float pitch_deg_ = 0.f, roll_deg_ = 0.f;
    bool  on_ = false;
    std::string src_ = "config.yaml";
    Eigen::Quaternionf q_corr_ = Eigen::Quaternionf::Identity();
};

}  // namespace g1
