#pragma once
// RtPriority.h — 스레드 스케줄링 정책. 계산 스레드가 «관절 안전 루프를 굶기지 않게» 한다.
//
// 왜 필요한가 (2026-08-18 실측): 이 repo 에는 sched_setscheduler / affinity 설정이 «하나도
// 없었다». 전부 SCHED_OTHER 기본 nice 라, 계산 스레드를 하나 붙이면 1 kHz 안전 루프
// (관절 레이트리밋·qd 가드)와 «동등한 자격으로» CPU 를 다툰다. sim2sim 에서 부하를 걸수록
// 1 kHz 틱의 꼬리가 단조로 늘었다 — 17 -> 30 -> 38 -> 56 ms (예산 1 ms).
// 56 ms 는 그동안 안전 가드가 «안 도는» 시간이다.
//
// 🔴 실패는 정상 경로다. RT 우선순위는 권한이 필요하고(CAP_SYS_NICE / RLIMIT_RTPRIO),
// 개발 기계에는 대개 없다. 실패하면 «경고 한 줄»만 남기고 종전과 똑같이 돈다 —
// 계측·안전 장치를 넣다가 기동을 못 하게 만드는 것이 더 나쁘다.
//
// 끄는 법: 환경변수 G1_NO_RT=1
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sched.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <spdlog/spdlog.h>

namespace rtprio {

// 계층 — 숫자가 클수록 먼저 돈다. 커널 스레드(보통 50~99)와 정면충돌하지 않게 낮게 잡는다.
static constexpr int FSM_1KHZ   = 40;   // 관절 안전. 이걸 지키는 것이 이 파일의 목적
static constexpr int POLICY_50HZ = 30;  // 정책. 늦으면 행동이 낡지만 안전은 FSM 이 잡는다
// 계산(생성기 등)은 RT 를 «주지 않는다» — SCHED_OTHER + nice. 아래 lower_this_thread 참고.

inline bool disabled() {
    const char* v = std::getenv("G1_NO_RT");
    return v && v[0] && std::strcmp(v, "0") != 0;
}

// 지금 이 스레드를 SCHED_FIFO 로. 성공 여부를 돌려준다(실패해도 계속 돈다).
inline bool raise_this_thread(const char* who, int prio) {
    if (disabled()) {
        spdlog::warn("[rt] {} — G1_NO_RT 로 비활성. 안전 루프가 계산 스레드와 동등 경쟁한다.", who);
        return false;
    }
    sched_param sp{};
    sp.sched_priority = prio;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0) {
        spdlog::info("[rt] {} -> SCHED_FIFO prio {}", who, prio);
        return true;
    }
    spdlog::warn("[rt] {} -> SCHED_FIFO prio {} 실패: {}. 종전(SCHED_OTHER)대로 돈다. "
                 "실로봇에서는 이걸 «반드시» 해결할 것 — 권한: setcap cap_sys_nice+ep <binary> "
                 "또는 /etc/security/limits.conf 의 rtprio, 또는 root.",
                 who, prio, std::strerror(rc));
    return false;
}

// 계산 스레드를 «뒤로» 민다. RT 를 주는 게 아니라 SCHED_OTHER 안에서 양보시키는 것 —
// 계산이 안전보다 앞설 이유는 어떤 경우에도 없다.
inline void lower_this_thread(const char* who, int nice_delta = 10) {
    sched_param sp{};
    sp.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);   // RT 였다면 내려온다
    // 🔴 nice 는 «스레드» 단위로 준다. 리눅스에서 setpriority(PRIO_PROCESS, tid) 의 tid 는
    // 스레드 id(gettid)이지 pid 가 아니다 — 0 을 주면 «프로세스 전체»가 밀려 정책·안전
    // 스레드까지 같이 느려진다. 정확히 반대 효과다.
    const pid_t tid = static_cast<pid_t>(::syscall(SYS_gettid));
    const int rc = ::setpriority(PRIO_PROCESS, tid, nice_delta);
    spdlog::info("[rt] {} -> SCHED_OTHER nice +{} ({})", who, nice_delta, rc == 0 ? "ok" : "무시됨");
}

// 특정 코어에 핀. cpu < 0 이면 아무것도 안 한다.
inline void pin_this_thread(const char* who, int cpu) {
    if (cpu < 0) return;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    spdlog::info("[rt] {} -> cpu {} ({})", who, cpu, rc == 0 ? "ok" : std::strerror(rc));
}

}  // namespace rtprio
