#pragma once
// StateDump.h — «측정 q / 명령 q_des / 게인 / IMU» 를 로봇 온보드 로거와 «같은 열 이름» 으로
//               CSV 에 남긴다. sim2sim ↔ 실기를 같은 분석 스크립트로 대조하기 위한 계측이다.
//               (g1 전용, 🅐 구역)
//
// 🔴 기본 꺼짐. 환경변수 G1_STATE_CSV 가 있을 때만 파일을 연다 — 없으면 비용 0(포인터 검사 1회).
//     $ G1_STATE_CSV=/tmp/sim_state.csv ./g1_ctrl --network=lo
//
// ⚠ run() 은 1 kHz FSM 스레드다. 그래서 ① 기본 꺼짐 ② 켜도 20틱마다(=50Hz, 정책 주기와 같다)
//    ③ fflush 하지 않고 stdio 버퍼에 맡긴다. 그래도 «켠 채로 실기를 돌리지는 말 것» —
//    계측용이지 상시 로깅용이 아니다(그 역할은 온보드 piene_g1_logger 가 한다).
#include <cstdio>
#include <cstdlib>
#include <chrono>

namespace g1 {

class StateDump {
public:
    void open_from_env(const char* env_name = "G1_STATE_CSV") {
        const char* path = std::getenv(env_name);
        if (!path || !*path) return;
        f_ = std::fopen(path, "w");
        if (!f_) return;
        t0_ = now();
        // 온보드 로거(piene_g1_logger) 의 341열 중 분석에 쓰는 열만, «이름을 그대로» 쓴다.
        std::fprintf(f_, "time,wall_time,quat_w,quat_x,quat_y,quat_z,"
                         "ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z,"
                         "rpy_r,rpy_p,rpy_y");
        for (const char* k : {"q","dq","tau_est","q_des","dq_des","kp","kd","tau_ff"})
            for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%s_%d", k, i);
        std::fprintf(f_, "\n");
    }
    void close() { if (f_) { std::fflush(f_); std::fclose(f_); f_ = nullptr; } }
    bool on() const { return f_ != nullptr; }

    // LowState(측정) + LowCmd(명령) 한 쌍을 한 줄로. 20틱마다 = 50Hz.
    template <class LowStateMsg, class LowCmdMsg>
    void tick(const LowStateMsg& st, const LowCmdMsg& cmd) {
        if (!f_) return;
        if (++n_ % 20) return;
        const auto& im = st.imu_state();
        const auto& ms = st.motor_state();
        const auto& mc = cmd.motor_cmd();
        std::fprintf(f_, "%.4f,%.6f", now() - t0_, now());
        for (int i = 0; i < 4; ++i) std::fprintf(f_, ",%.6f", im.quaternion()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(f_, ",%.6f", im.gyroscope()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(f_, ",%.6f", im.accelerometer()[i]);
        for (int i = 0; i < 3; ++i) std::fprintf(f_, ",%.6f", im.rpy()[i]);
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", ms[i].q());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", ms[i].dq());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", ms[i].tau_est());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", mc[i].q());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", mc[i].dq());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", mc[i].kp());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", mc[i].kd());
        for (int i = 0; i < 29; ++i) std::fprintf(f_, ",%.6f", mc[i].tau());
        std::fprintf(f_, "\n");
    }
    ~StateDump() { close(); }

private:
    static double now() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    std::FILE* f_ = nullptr;
    double t0_ = 0.0;
    unsigned n_ = 0;
};

}  // namespace g1
