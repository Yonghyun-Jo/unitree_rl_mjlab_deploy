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
//   ⑤ Mimic 재진입(닫지 않고 open_from_env() 재호출) 이 std::terminate 없이 멱등으로 no-op 되는가
//   ⑥ g1::StateDump::shared() — 서로 다른 State_Mimic "인스턴스" 가 같은 파일을 truncate 없이
//     이어쓰는가(2026-09-22 리뷰가 재현한 버그: Masked -> Dance1 전환이 truncate 했다)
//   ⑦ close() 가 소멸자에서(스코프 종료) 실제로 flush 하는가
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

    // ⑤ Mimic 재진입 회귀(직접 객체, shared() 와 무관 — 옛 헤더에도 컴파일된다): 옛 설계는
    //    State_Mimic::enter() 가 stay 마다 open_from_env() 를 다시 불렀는데 닫는 코드가 없어서,
    //    두 번째 open_from_env() 가 join 가능한 std::thread 에 새 std::thread 를 move-assign 해
    //    std::terminate 가 났다(원래 버그 — 회귀하면 이 프로세스 자체가 죽는다). Ruling 27 이후
    //    설계는 그 open_from_env() 를 **멱등**으로 만들어 이 상황(닫지 않고 또 여는 것) 자체를
    //    "정상"으로 바꿨다 — 그래서 이 블록은 옛 버그의 재현이면서 동시에 새 설계의 핵심 계약
    //    (헤더 1회 · 줄 유실 없음 · 재오픈은 no-op) 검증이다.
    {
        const char* rpath = "/tmp/_tsdw_reentry.csv";
        std::remove(rpath);
        setenv("G1_STATE_CSV", rpath, 1);
        FakeLowState st; FakeLowCmd cmd; GaitAux aux;

        // close on a never-opened dump — safe, no crash, stays off.
        {
            g1::StateDump never;
            chk(!never.on(), "never-opened dump 는 off");
            never.close();
            never.close();                        // 두 번 호출도 안전해야 한다
            chk(!never.on(), "never-opened dump 는 close 뒤에도 off");
        }

        g1::StateDump d;
        d.open_from_env("G1_STATE_CSV", GaitAux::header());   // "instance A" enter()
        chk(d.on(), "첫 open 이 열리지 않았다");
        for (int t = 0; t < 20 * 3; ++t) d.tick(st, cmd, aux);   // 3 rows

        // 🔴 close() 를 «부르지 않고» 아직 열려 있는(th_ joinable, f_ non-null) 객체에 다시
        //    open_from_env() — 옛 버그의 정확한 트리거 그 자체("instance B" 로 전환, exit() 가
        //    close() 를 안 부르는 지금 설계에서 실제로 매번 벌어지는 일). 옛 헤더로 빌드하면
        //    여기서 std::terminate 로 테스트 프로세스가 통째로 죽는다(별도로 재확인함 — 리포트
        //    참고) — 그래서 아래 줄들이 «찍히는 것 자체» 가 증거다. 지금 설계에서는 멱등이라
        //    no-op: 새 스레드도, 새 헤더도, truncate 도 없어야 한다.
        d.open_from_env("G1_STATE_CSV", GaitAux::header());   // "instance B" enter() — no-op 이어야 함
        chk(d.on(), "재오픈 뒤에도 열려 있어야 한다(여기서 크래시하면 이 줄 자체가 안 찍힌다)");
        for (int t = 0; t < 20 * 2; ++t) d.tick(st, cmd, aux);   // 2 more rows — 같은 파일에 이어져야 함

        d.close();
        chk(!d.on(), "close 뒤 off 여야 한다");
        d.close();                                 // 두 번째 close — 안전해야 한다

        auto RL = lines_of(rpath);
        int header_lines = 0;
        for (auto& s : RL) if (s.rfind("time,wall_time,", 0) == 0) ++header_lines;
        chk(header_lines == 1, "재오픈 뒤 헤더가 한 번만 있어야 한다(멱등 — 두 번째 open 은 no-op)");
        chk((int)RL.size() == 1 + 3 + 2, "재오픈 뒤 두 'instance' 의 줄이 모두 남아 있어야 한다(유실 없음)");
        if (header_lines != 1 || (int)RL.size() != 6)
            std::printf("     header_lines=%d lines=%zu\n", header_lines, RL.size());
        std::printf("  ⑤ 재진입(멱등) 회귀: crash 없이, 헤더 1회, 줄 보존 확인\n");
    }

    // ⑥ g1::StateDump::shared() — 실제 프로덕션 호출 경로. Mimic_Masked 와 Mimic_Dance1_subject2
    //    처럼 서로 다른 State_Mimic "인스턴스" 가 같은 G1_STATE_CSV 를 보면서 전환돼도(리뷰에서
    //    재현된 버그: 전환이 truncate 했다) 한 파일에 헤더 1회로 이어써야 한다. shared() 는
    //    프로세스에 하나뿐이라 이 블록이 테스트 안에서 이걸 쓰는 처음이자 유일한 자리여야 한다
    //    (다른 곳에서 또 열면 "다른 경로" 경고 분기를 타게 된다).
    {
        chk(&g1::StateDump::shared() == &g1::StateDump::shared(),
            "shared() 는 항상 같은 인스턴스를 돌려줘야 한다");

        const char* spath = "/tmp/_tsdw_shared.csv";
        std::remove(spath);
        setenv("G1_STATE_CSV", spath, 1);
        FakeLowState st; FakeLowCmd cmd; GaitAux aux;

        // "Mimic_Masked::enter()"
        g1::StateDump::shared().open_from_env("G1_STATE_CSV", GaitAux::header());
        chk(g1::StateDump::shared().on(), "shared() 첫 open 이 안 열렸다");
        for (int t = 0; t < 20 * 4; ++t) g1::StateDump::shared().tick(st, cmd, aux);   // 4 rows

        // "Mimic_Masked -> Mimic_Dance1_subject2 전환": exit() 는 close() 를 안 부른다(Ruling 27)
        // -> 다음 enter() 가 또 open_from_env() 를 부른다. 같은 경로면 no-op(헤더 재발급 없음).
        g1::StateDump::shared().open_from_env("G1_STATE_CSV", GaitAux::header());
        chk(g1::StateDump::shared().on(), "shared() 재오픈 뒤에도 열려 있어야 한다");
        for (int t = 0; t < 20 * 3; ++t) g1::StateDump::shared().tick(st, cmd, aux);   // 3 more rows

        // 다른 경로가 요청되면(설정 실수 등) 무시하고 원래 파일을 계속 쓴다 — 새 파일을 만들면 안 된다.
        const char* other = "/tmp/_tsdw_shared_other.csv";
        std::remove(other);
        setenv("G1_STATE_CSV", other, 1);
        g1::StateDump::shared().open_from_env("G1_STATE_CSV", GaitAux::header());
        {
            std::FILE* f = std::fopen(other, "r");
            chk(f == nullptr, "다른 경로 요청이 새 파일을 만들면 안 된다(무시해야 한다)");
            if (f) std::fclose(f);
        }
        setenv("G1_STATE_CSV", spath, 1);   // 원상복구(무해 — tick() 은 env 를 안 본다)
        for (int t = 0; t < 20; ++t) g1::StateDump::shared().tick(st, cmd, aux);   // 1 more row

        // "프로세스 종료" 를 흉내낸다 — 실서비스에서는 shared() 의 소멸자 하나가 이걸 부른다.
        g1::StateDump::shared().close();

        auto SL = lines_of(spath);
        int sh = 0; for (auto& s : SL) if (s.rfind("time,wall_time,", 0) == 0) ++sh;
        chk(sh == 1, "shared() 헤더가 한 번만 있어야 한다");
        chk((int)SL.size() == 1 + 4 + 3 + 1, "shared() 두 '진입' 의 줄이 모두 남아 있어야 한다(유실 없음)");
        if (sh != 1 || (int)SL.size() != 9)
            std::printf("     shared header_lines=%d lines=%zu\n", sh, SL.size());
        std::printf("  ⑥ shared() 싱글턴: 두 '진입' 이 한 파일에 헤더 1회로 보존됨\n");
    }

    // ⑦ close() 는 소멸자에서(스코프 종료) 일어나야 한다 — RAII 로 확인. shared() 와 별개의
    //    지역 객체를 써서, "소멸 == flush+close" 를 shared() 의 실제 소멸(프로세스 종료) 을
    //    기다리지 않고도 증명한다.
    {
        const char* dpath = "/tmp/_tsdw_dtor.csv";
        std::remove(dpath);
        setenv("G1_STATE_CSV", dpath, 1);
        FakeLowState st; FakeLowCmd cmd; GaitAux aux;
        {
            g1::StateDump tmp;
            tmp.open_from_env("G1_STATE_CSV", GaitAux::header());
            chk(tmp.on(), "dtor 테스트: open 이 안 됐다");
            for (int t = 0; t < 20 * 2; ++t) tmp.tick(st, cmd, aux);
        }   // tmp 소멸 -> ~StateDump() -> close() (드레인 스레드 join, 링 마저 비움, 파일 flush+close)
        auto DL = lines_of(dpath);
        chk((int)DL.size() == 1 + 2, "소멸자가 닫지 않았다(줄이 파일에 안 남았다)");
        std::printf("  ⑦ 소멸자 close(): 스코프 종료로 파일이 flush 됐다\n");
    }

    std::printf(fail ? "test_state_dump_writer: %d FAIL\n" : "test_state_dump_writer: OK\n", fail);
    return fail ? 1 : 0;
}
