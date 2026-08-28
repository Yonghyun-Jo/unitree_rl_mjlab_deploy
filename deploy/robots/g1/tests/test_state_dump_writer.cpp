// test_state_dump_writer.cpp — StateDump 의 «쓰기 경로» 를 잠근다.
//
// 왜 필요한가: 계측을 실기에서 켜려면 1 kHz 스레드가 파일을 만지면 안 된다. 예전 구조는
// 그 스레드에서 fprintf 했고, stdio 버퍼가 차는 순간 같은 스레드에서 write() 가 터졌다.
// eMMC 에서 그건 수십 ms 다. 그래서 «주석으로 실기에서 켜지 말 것» 이라고만 적혀 있었고,
// 결국 실기 로그에 gait 계측이 한 번도 안 남았다.
//
// 여기서 재는 것:
//   ① 줄 수와 열 수가 맞는가            ② 값이 그대로 나르는가
//   ③ 평시에 버리는 줄이 없는가          ④ 🔴 RT 경로가 syscall 을 0 번 하는가 (/proc/self/syscr)
//
//   g++ -std=gnu++17 -O3 -DNDEBUG -I../include -pthread test_state_dump_writer.cpp -o /tmp/t && /tmp/t
#include "StateDump.h"
#include "MaskedLocoController.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}

// ── 최소 가짜 메시지 (unitree_sdk2 없이 템플릿을 만족시킨다) ──────────────────
struct FakeImu {
    float q_[4] = {1, 0, 0, 0}, g_[3] = {0, 0, 0}, a_[3] = {0, 0, 9.81f}, r_[3] = {0, 0, 0};
    const float* quaternion()   const { return q_; }
    const float* gyroscope()    const { return g_; }
    const float* accelerometer()const { return a_; }
    const float* rpy()          const { return r_; }
};
struct FakeMotorState { float q_=0, dq_=0, tau_=0;
    float q() const {return q_;} float dq() const {return dq_;} float tau_est() const {return tau_;} };
struct FakeLowState {
    FakeImu imu_; FakeMotorState ms_[29];
    const FakeImu& imu_state() const { return imu_; }
    const FakeMotorState* motor_state() const { return ms_; }
};
struct FakeMotorCmd { float q_=0, dq_=0, kp_=0, kd_=0, tau_=0;
    float q() const {return q_;} float dq() const {return dq_;}
    float kp() const {return kp_;} float kd() const {return kd_;} float tau() const {return tau_;} };
struct FakeLowCmd { FakeMotorCmd mc_[29]; const FakeMotorCmd* motor_cmd() const { return mc_; } };

// 이 프로세스가 한 write syscall 누적 수. 🔴 «/proc/self/syscw» 라는 파일은 없다 —
// 커널은 /proc/self/io 안에 필드로 준다(CONFIG_TASK_IO_ACCOUNTING). 처음에 파일 경로로
// 잘못 읽어 -1 이 나왔고, -1 - (-1) = 0 이라 **판정이 통과하는 것처럼 보였다.**
// 그래서 아래 main 이 «카운터가 실제로 움직이는지» 를 먼저 확인하고 시작한다.
static long syscw() {
    std::FILE* f = std::fopen("/proc/self/io", "r");
    if (!f) return -1;
    char key[64]; long v = 0, out = -1;
    while (std::fscanf(f, "%63s %ld", key, &v) == 2)
        if (std::strcmp(key, "syscw:") == 0) { out = v; break; }
    std::fclose(f); return out;
}
static std::vector<std::string> lines_of(const char* path) {
    std::vector<std::string> out; std::FILE* f = std::fopen(path, "r");
    if (!f) return out;
    char buf[8192];
    while (std::fgets(buf, sizeof buf, f)) { std::string s(buf); if (!s.empty() && s.back()=='\n') s.pop_back(); out.push_back(s); }
    std::fclose(f); return out;
}
static int commas(const std::string& s) { return (int)std::count(s.begin(), s.end(), ','); }

int main() {
    const char* path = "/tmp/_tsdw.csv";
    std::remove(path);
    setenv("G1_STATE_CSV", path, 1);

    const int kRows = 500;                 // 50 Hz 기준 10 초
    // ⓪ 계측기 자기검증 — 카운터가 안 움직이면 ④ 는 아무것도 재지 않는다.
    {
        const long a = syscw();
        std::FILE* t = std::fopen("/tmp/_tsdw_probe.txt", "w");
        for (int i = 0; i < 200; ++i) { std::fprintf(t, "x%d\n", i); std::fflush(t); }
        std::fclose(t);
        const long b = syscw();
        chk(a >= 0 && b - a >= 100, "syscw 카운터가 안 움직인다 — ④ 판정이 무력하다");
        if (a >= 0) std::printf("  계측기 자기검증: fflush 200회 -> syscw +%ld\n", b - a);
    }

    long sysw_rt = 0;
    {
        g1::StateDump d;
        d.open_from_env("G1_STATE_CSV", GaitAux::header());
        chk(d.on(), "열리지 않았다");

        FakeLowState st; FakeLowCmd cmd; GaitAux aux;
        // ④ RT 경로의 syscall 을 센다. tick() 을 kRows*20 번 부르는 동안(=kRows 줄)
        //    이 스레드가 write 계열 syscall 을 하면 안 된다. 쓰기 스레드는 별도 프로세스가
        //    아니므로 카운터에 섞이지만, 그쪽은 100 ms 마다 «몇 번» 이라 자릿수가 다르다.
        const long before = syscw();
        for (int k = 0; k < kRows; ++k) {
            for (int i = 0; i < 29; ++i) { st.ms_[i].q_ = float(k) + i * 0.001f; cmd.mc_[i].q_ = -float(k); }
            aux.eff = float(k) * 0.01f; aux.swing_sc = 1.0f + float(k) * 0.001f;
            for (int t = 0; t < 20; ++t) d.tick(st, cmd, aux);   // 20틱마다 한 줄
        }
        sysw_rt = syscw() - before;
        d.close();
    }

    auto L = lines_of(path);
    // ① 줄 수 = 헤더 1 + kRows
    chk((int)L.size() == kRows + 1, "줄 수가 안 맞는다");
    if ((int)L.size() != kRows + 1) std::printf("     got %zu want %d\n", L.size(), kRows + 1);

    if (L.size() > 1) {
        // 온보드 로거(341열) 중 «분석에 쓰는» 247열 + GaitAux 열.
        //   247 = time,wall_time(2) + quat(4) + ang_vel(3) + lin_acc(3) + rpy(3)
        //         + {q,dq,tau_est,q_des,dq_des,kp,kd,tau_ff} x 29 = 15 + 232
        // 여기 숫자를 박아 두는 이유: 열이 늘거나 줄면 실기 로그 분석 스크립트가 «조용히»
        // 어긋난다(앞쪽 열 위치가 밀린다). 그때 시험이 먼저 알려야 한다.
        const int nh = commas(L[0]);
        const int want_cols = (15 + 8 * 29) - 1 + commas(GaitAux::header());
        if (nh != want_cols) std::printf("     헤더 콤마 %d, 기대 %d\n", nh, want_cols);
        chk(nh == want_cols, "헤더 열 수가 기대와 다르다");
        bool same = true;
        for (size_t i = 1; i < L.size(); ++i) if (commas(L[i]) != nh) { same = false; break; }
        chk(same, "어떤 줄의 열 수가 헤더와 다르다");

        // ② 값이 그대로 나르는가 — 마지막 줄의 q_0 은 (kRows-1) 이어야 한다
        const std::string& last = L.back();
        char want[64]; std::snprintf(want, sizeof want, ",%.6f,", float(kRows - 1));
        chk(last.find(want) != std::string::npos, "마지막 줄에 마지막으로 넣은 q 가 없다");

        // 첫 줄은 k=0 → q_0 = 0.000000
        chk(L[1].find(",0.000000,") != std::string::npos, "첫 줄 값이 안 보인다");
    }

    // ③-a 🔴 «반쪽 줄» 이 없어야 한다. stdio 가 줄 도중에 flush 하면 프로세스가 갑자기
    //     죽었을 때 파일 끝이 잘리고 분석기가 거기서 깨진다(실측으로 겪었다).
    if (L.size() > 1) {
        const int nh0 = commas(L[0]);
        int torn = 0;
        for (size_t i = 1; i < L.size(); ++i) if (commas(L[i]) != nh0) ++torn;
        if (torn) std::printf("     반쪽 줄 %d 개\n", torn);
        chk(torn == 0, "반쪽 줄이 있다 — flush 가 줄 경계에서 안 일어난다");
    }

    // ③ 평시에 버린 줄이 없어야 한다 (있으면 close 가 stderr 로 경고한다 — 줄 수로 확인)
    chk((int)L.size() - 1 == kRows, "줄을 버렸다 (링이 작거나 쓰기 스레드가 못 따라간다)");

    // ④ 🔴 RT 경로 syscall. 쓰기 스레드가 100 ms 마다 도니 10 초면 100 번 안팎이 상한이다.
    //    예전 구조(fprintf 직접)는 4 KB 버퍼가 차는 대로 write() 를 불러 수백~수천 번이었다.
    std::printf("  RT 구간 write syscall: %ld 회 (줄 %d)\n", sysw_rt, kRows);
    chk(sysw_rt >= 0 && sysw_rt < kRows / 2, "RT 구간에서 syscall 이 너무 많다 = 파일을 만지고 있다");

    std::printf(fail ? "test_state_dump_writer: %d FAIL\n" : "test_state_dump_writer: OK\n", fail);
    return fail ? 1 : 0;
}
