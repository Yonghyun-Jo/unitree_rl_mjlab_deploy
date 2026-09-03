// test_imu_cal.cpp — IMU 편향 보정이 «주입한 편향을 정확히 되돌리는가».
//
// 이 헤더가 틀리면 로봇의 **유일한 균형 센서**가 틀어진다. 그래서 항등성(주입→보정=원본)과
// 「꺼지면 손대지 않는다」를 둘 다 잠근다.
//
//   g++ -std=gnu++17 -O3 -DNDEBUG -I../include -I../../include \
//       $(pkg-config --cflags eigen3) test_imu_cal.cpp -o /tmp/t && /tmp/t
#include "ImuCal.h"
#include <cstdio>
#include <cmath>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}
static void chkf(float got, float want, const char* what, float tol = 1e-4f) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("FAIL %s: got %.6f want %.6f\n", what, got, want); ++fail;
    }
}

struct FakeData {
    Eigen::Quaternionf root_quat_w = Eigen::Quaternionf::Identity();
    Eigen::Vector3f projected_gravity_b{0, 0, -1};
    Eigen::Vector3f GRAVITY_VEC_W{0, 0, -1};
};

static constexpr float D = 3.14159265358979323846f / 180.0f;

int main() {
    // ── ① 꺼지면 «아무것도 안 한다» ─────────────────────────────────────────
    {
        g1::ImuCal c;                       // set() 안 부름 = 기본 0
        FakeData d;
        d.root_quat_w = Eigen::Quaternionf(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitY()));
        const Eigen::Quaternionf before = d.root_quat_w;
        const Eigen::Vector3f gb = d.projected_gravity_b;
        c.apply(d);
        chk(!c.on(), "기본이 꺼짐이 아니다");
        chk(d.root_quat_w.coeffs() == before.coeffs(), "꺼졌는데 quat 를 바꿨다");
        chk(d.projected_gravity_b == gb, "꺼졌는데 gravity 를 바꿨다");
        c.set(0.f, 0.f);
        chk(!c.on(), "0,0 인데 켜졌다");
    }

    // ── ② 주입한 편향을 정확히 되돌린다 (여러 각도) ──────────────────────────
    for (float pitch : {-6.f, -4.22f, -1.f, 0.5f, 3.f, 7.f}) {
        for (float roll : {-2.f, 0.f, 0.34f, 1.5f}) {
            // 「IMU 가 mount 만큼 돌아 붙었다」를 흉내: q_보고 = q_골반 · q_mount
            const Eigen::Quaternionf q_pelvis(Eigen::AngleAxisf(0.17f, Eigen::Vector3f::UnitY()));
            const Eigen::Quaternionf q_mount =
                Eigen::Quaternionf(Eigen::AngleAxisf(pitch * D, Eigen::Vector3f::UnitY())) *
                Eigen::Quaternionf(Eigen::AngleAxisf(roll  * D, Eigen::Vector3f::UnitX()));
            FakeData d;
            d.root_quat_w = q_pelvis * q_mount;

            g1::ImuCal c; c.set(pitch, roll);
            if (std::fabs(pitch) < 1e-6f && std::fabs(roll) < 1e-6f) continue;
            c.apply(d);
            // 되돌린 quat 가 참 골반 자세와 같아야 한다 (부호 자유도 고려)
            const float dot = std::fabs(d.root_quat_w.coeffs().dot(q_pelvis.coeffs()));
            chkf(dot, 1.0f, "보정이 참 자세를 복원하지 못했다");
            // gravity 도 참 자세에서 나온 값과 같아야 한다
            const Eigen::Vector3f want = q_pelvis.conjugate() * d.GRAVITY_VEC_W;
            chkf((d.projected_gravity_b - want).norm(), 0.f, "gravity 복원 실패");
        }
    }

    // ── ③ 부호 규약을 «값으로» 못박는다 ────────────────────────────────────
    // 규약: 몸 프레임 중력 g 에 대해  앞기울기 φ = asin(g.x())   [+ = 앞으로 숙임]
    //   (몸이 +y 축으로 φ 만큼 앞으로 숙으면 g_b = (sin φ, 0, −cos φ) 이다.)
    // 그러면 **보정 pitch = +X 를 걸면 정책이 보는 앞기울기가 −X 만큼 이동한다.**
    // 즉 수직으로 선 로봇에 +4 를 걸면 정책은 「4° 뒤로 누웠다」고 본다.
    // sim 에 «일부러 편향을 만들» 때 이 방향을 쓴다.
    {
        FakeData d;                                   // 참 자세 = 수직 (φ = 0)
        g1::ImuCal c; c.set(4.0f, 0.0f);
        c.apply(d);
        const Eigen::Vector3f g = d.projected_gravity_b;
        chkf(g.norm(), 1.0f, "중력 크기가 1 이 아니다");
        const float phi = std::asin(g.x()) / D;       // 정책이 보는 앞기울기 [deg]
        chkf(phi, -4.0f, "보정 부호 규약이 깨졌다", 0.05f);
        std::printf("  부호 규약: 보정 +4.00° -> 정책이 보는 앞기울기 %+.2f° (뒤로 누운 것으로 본다)\n", phi);

        FakeData d2; g1::ImuCal c2; c2.set(-4.0f, 0.0f); c2.apply(d2);
        chkf(std::asin(d2.projected_gravity_b.x()) / D, +4.0f, "음의 보정 부호가 깨졌다", 0.05f);
    }

    // ── ④ 환경변수 override ────────────────────────────────────────────────
    {
        setenv("G1_IMU_CAL_DEG", "-4.22,0.34", 1);
        g1::ImuCal c;
        chk(c.set_from_env(), "환경변수를 못 읽었다");
        chk(c.on(), "환경변수를 읽었는데 꺼져 있다");
        unsetenv("G1_IMU_CAL_DEG");
        g1::ImuCal c2;
        chk(!c2.set_from_env(), "환경변수가 없는데 읽었다고 한다");
    }

    // ── ⑤ sim(lo) 에는 config 의 보정을 걸지 않는다 ─────────────────────────
    // config.yaml 이 sim·실기 공용이라, 실기 상쇄값이 sim 에 걸리면 «편향 없는 로봇에 틀린 보정»
    // 으로 넘어진다(2026-09-01 실측 65°). 인터페이스 이름으로 가른다.
    {
        chk(!g1::ImuCal::applies_to_network("lo"),     "lo(sim) 인데 보정을 건다");
        chk( g1::ImuCal::applies_to_network("eth0"),   "eth0(실기) 인데 보정을 안 건다");
        chk( g1::ImuCal::applies_to_network("enp5s0"), "enp5s0(실기) 인데 보정을 안 건다");
    }

    std::printf(fail ? "[test_imu_cal] %d FAIL\n" : "[test_imu_cal] ALL PASS\n", fail);
    return fail ? 1 : 0;
}
