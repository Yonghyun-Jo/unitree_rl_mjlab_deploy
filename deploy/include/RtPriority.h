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

// 계산 스레드를 «맨 뒤»로 — SCHED_IDLE. nice 와 달리 «다른 게 없을 때만» 돈다.
//
// 왜 따로 있나 (2026-08-18 실측): 부하 스레드에 nice +10 을 «성공적으로» 걸었는데도 1 kHz
// 안전 루프의 꼬리가 80.6 ms 였다. nice 는 같은 SCHED_OTHER 안의 가중치일 뿐이라, 이미 실행
// 중인 계산이 안전 루프를 밀어내는 것을 못 막는다.
//
// 🟢 권한이 필요 없다 — 내리는 것이기 때문. RT(올리기)는 CAP_SYS_NICE 가 필요하지만
// SCHED_IDLE 은 아무나 된다.
//
// 🔴 그런데 «재봤더니 도움이 안 된다» (2026-08-18, com1 sim2sim). 1 kHz 안전 루프 꼬리:
//        27MB  매 틱 :  nice+10  15.0 ms   ->  SCHED_IDLE  18.8 ms
//        185MB n=5   :  nice+10  61.3 ms   ->  SCHED_IDLE 121.3 ms   (2배 «악화»)
//    이유: 이 꼬리는 «런큐 순번» 문제가 아니라 «메모리» 문제다. 스케줄링 클래스는 언제 도는지를
//    정할 뿐 도는 동안 얼마나 캐시·대역폭을 먹는지는 못 정한다. 게다가 뒤로 밀린 스레드는 더
//    «오래» 돌아(25.8 -> 30.4 ms) 캐시를 더 오래 짓밟는다.
//    ⇒ 이 손잡이는 답이 아니다. 답은 «모델 크기»다. 남겨 둔 이유는 재시도를 막기 위해서다.
inline bool idle_this_thread(const char* who) {
    sched_param sp{};
    sp.sched_priority = 0;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
    if (rc == 0) spdlog::info("[rt] {} -> SCHED_IDLE (다른 게 없을 때만 돈다)", who);
    else spdlog::warn("[rt] {} -> SCHED_IDLE 실패: {}", who, std::strerror(rc));
    return rc == 0;
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
