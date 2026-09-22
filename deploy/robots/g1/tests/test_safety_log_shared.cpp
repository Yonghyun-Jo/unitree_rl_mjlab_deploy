// test_safety_log_shared.cpp — SafetyLog 의 «프로세스 공유 싱글턴» 계약을 잠근다.
//
// 왜 필요한가: SafetyLog::open_from_env("G1_SAFETY_CSV") 는 매 호출마다 fopen(path,"w") 로
// 열렸다(옛 설계). safety_log_ 는 State_Mimic 인스턴스(Mimic_Masked/Mimic_Dance1_subject2
// 등)마다 따로 있고 enter() 에서 열리므로, Mimic 재진입(예: 넘어짐 -> Passive -> 폴백 ->
// Mimic 복귀, "fall -> p -> f -> m")마다 그 "w" 가 파일을 잘라 **그 재진입을 일으킨 넘어짐의
// 안전 이벤트 — 유일한 사후 증거 — 를 지웠다.** 두 State_Mimic 인스턴스가 같은 G1_SAFETY_CSV
// 를 보면서 서로 truncate 하는 것도 같은 근원. StateDump.h 가 먼저 고친 것과 같은 패턴
// (Ruling 27)을 SafetyLog 에도 적용한다(Ruling 30): ① SafetyLog::shared() 프로세스 공유
// 인스턴스 ② open_from_env() 멱등 ③ stay 끝 close() 제거(닫는 것은 소멸자 하나뿐).
//
// 여기서 재는 것:
//   A) 🔴 핵심 결함의 재현 — 직접 인스턴스 재진입(옛 헤더에도 그대로 컴파일된다. `git show
//      HEAD:deploy/robots/g1/include/SafetyLog.h` 로 뽑은 pre-fix 헤더로 이 파일을 그대로
//      다시 빌드하면 이 블록이 FAIL 한다 — 재오픈이 첫 배치의 줄을 지운다. StateDump 와 달리
//      스레드가 없어 std::terminate 는 안 나고 «조용히 데이터만 사라진다» — 이게 바로 이번에
//      고치는 결함이다.)
//   B) g1::SafetyLog::shared() — 서로 다른 "State_Mimic 인스턴스" 가 같은 파일에 헤더 1회로
//      이어쓰는가(실제 프로덕션 호출 경로).
//   C) 다른 경로 요청은 무시(경고만)하고 원래 파일을 계속 쓰는가.
//   D) 소멸자가 실제로 close() 를 부르는가(RAII) — never-opened 인스턴스의 close() 도 안전한가.
//
//   g++ -std=gnu++17 -O2 -Wall -Wextra -I../include test_safety_log_shared.cpp -o /tmp/t && /tmp/t
#include "SafetyLog.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}
static std::vector<std::string> lines_of(const char* path) {
    std::vector<std::string> out; std::FILE* f = std::fopen(path, "r");
    if (!f) return out;
    char buf[8192];
    while (std::fgets(buf, sizeof buf, f)) {
        std::string s(buf);
        if (!s.empty() && s.back() == '\n') s.pop_back();
        out.push_back(s);
    }
    std::fclose(f);
    return out;
}
static int header_count(const std::vector<std::string>& L) {
    int n = 0;
    for (auto& s : L) if (s.rfind("wall_time,t,kind,", 0) == 0) ++n;
    return n;
}
static int event_count(const std::vector<std::string>& L) {
    return (int)L.size() - header_count(L);   // 헤더가 아닌 줄 = 실제 event()/sample() 행
}

int main() {
    // ── A) 직접 인스턴스 재진입 — 핵심 결함의 재현 ─────────────────────────
    // 🔴 shared() 를 안 쓴다 — 옛 헤더에도 그대로 컴파일된다(SafetyLog 는 항상 기본 생성
    //    가능했다). 옛 설계는 open_from_env() 가 두 번째 호출에서도 "w" 로 다시 열어 첫 배치의
    //    줄을 지운다. 새 설계(멱등)에서는 두 번째 open 이 no-op 이라 두 배치가 모두 남는다.
    {
        const char* path = "/tmp/_tsl_reentry.csv";
        std::remove(path);
        setenv("G1_SAFETY_CSV", path, 1);

        g1::SafetyLog log;
        log.open_from_env();                                                     // "instance A" enter()
        chk(log.enabled(), "A: 첫 open 이 열리지 않았다");
        log.event("qd_warn", 3, "left_knee", 1.5f, 1.0f, "-> 폴백 모드 강제");     // stay A 사건 1
        log.event("qd_warn", 3, "left_knee", 1.7f, 1.0f, "-> 폴백 모드 강제");     // stay A 사건 2
        // fall -> p -> f -> m: exit() 는 close() 를 안 부른다(새 설계). 다음 enter() 가 또
        // open_from_env() 를 부른다 — "instance B"(재진입, 옛 설계에서 truncate 를 일으키던 그 자리).
        log.open_from_env();
        chk(log.enabled(), "A: 재오픈 뒤에도 열려 있어야 한다");
        log.event("bad_orientation", -1, "", 61.2f, 57.3f, "-> Passive");         // stay B 사건 1

        log.close();

        auto L = lines_of(path);
        chk(header_count(L) == 1, "A: 재오픈 뒤 헤더가 한 번만 있어야 한다(멱등)");
        chk(event_count(L) == 3,
            "A: 두 stay 의 사건(2+1=3건)이 모두 남아야 한다 "
            "— 옛 설계는 여기서 1건만 남는다(stay A 의 2건이 재오픈에 지워진다)");
        if (header_count(L) != 1 || event_count(L) != 3)
            std::printf("     header=%d events=%d (want 1 / 3)\n", header_count(L), event_count(L));
    }

    // ── B) g1::SafetyLog::shared() — 실제 프로덕션 호출 경로 ──────────────
    {
        chk(&g1::SafetyLog::shared() == &g1::SafetyLog::shared(),
            "B: shared() 는 항상 같은 인스턴스를 돌려줘야 한다");

        const char* spath = "/tmp/_tsl_shared.csv";
        std::remove(spath);
        setenv("G1_SAFETY_CSV", spath, 1);

        // "Mimic_Masked::enter()"
        g1::SafetyLog::shared().open_from_env();
        chk(g1::SafetyLog::shared().enabled(), "B: shared() 첫 open 이 안 열렸다");
        g1::SafetyLog::shared().event("qd_crit", 5, "right_hip", 4.1f, 3.5f, "-> Passive");

        // "Mimic_Masked -> Mimic_Dance1_subject2 전환": exit() 는 close() 를 안 부른다(Ruling 30)
        // -> 다음 enter() 가 또 open_from_env() 를 부른다. 같은 경로면 no-op(헤더 재발급 없음).
        g1::SafetyLog::shared().open_from_env();
        chk(g1::SafetyLog::shared().enabled(), "B: shared() 재오픈 뒤에도 열려 있어야 한다");
        g1::SafetyLog::shared().event("qd_warn", 3, "left_knee", 1.5f, 1.0f, "-> Passive. 복귀=p→f→m");

        const int before = event_count(lines_of(spath));
        g1::SafetyLog::shared().sample(10, 0.02f, 3,  0, 0.f, -1,  12.5f, nullptr);   // 1 Hz 표본도 같은 공유 인스턴스를 타는가
        const int after = event_count(lines_of(spath));
        chk(after > before, "B: sample() 도 같은 shared() 파일에 이어써야 한다");

        g1::SafetyLog::shared().close();   // "프로세스 종료" 를 흉내낸다(실서비스는 소멸자 하나)

        auto SL = lines_of(spath);
        chk(header_count(SL) == 1, "B: shared() 헤더가 한 번만 있어야 한다");
        chk(event_count(SL) == after,
            "B: close() 가 없어진 줄 없이 그대로 남겨야 한다(이미 매 호출 fflush 했으므로)");
        std::printf("  B) shared() 싱글턴: 두 '진입' + sample() 이 한 파일에 헤더 1회로 보존됨 "
                    "(총 %d행)\n", event_count(SL));
    }

    // ── C) 다른 경로 요청 — 무시(경고 1회)하고 원래 파일을 계속 쓴다 ────────
    // shared() 는 프로세스에 하나뿐이라(B 에서 이미 close() 됐지만 재사용하면 상태가 얽힌다)
    // 이 계약은 별도 직접 인스턴스로 확인한다.
    {
        const char* p1 = "/tmp/_tsl_c_first.csv";
        const char* other = "/tmp/_tsl_c_other.csv";
        std::remove(p1); std::remove(other);
        setenv("G1_SAFETY_CSV", p1, 1);

        g1::SafetyLog log;
        log.open_from_env();
        chk(log.enabled(), "C: 첫 open 실패");

        setenv("G1_SAFETY_CSV", other, 1);
        log.open_from_env();                 // 이미 열려 있다 -> 무시(경고만, 새 파일 생성 없음)
        {
            std::FILE* f = std::fopen(other, "r");
            chk(f == nullptr, "C: 다른 경로 요청이 새 파일을 만들면 안 된다(무시해야 한다)");
            if (f) std::fclose(f);
        }
        log.event("qd_warn", 1, "x", 1.f, 1.f, "after-other-request");
        log.close();

        auto L = lines_of(p1);
        chk(header_count(L) == 1 && event_count(L) == 1,
            "C: 다른 경로 요청 뒤에도 원래 파일에 계속 써야 한다");
    }

    // ── D) 소멸자가 실제로 close() 를 부르는가 (RAII) ───────────────────
    {
        const char* dpath = "/tmp/_tsl_dtor.csv";
        std::remove(dpath);
        setenv("G1_SAFETY_CSV", dpath, 1);
        {
            g1::SafetyLog tmp;
            tmp.open_from_env();
            chk(tmp.enabled(), "D: dtor 테스트, open 안 됐다");
            tmp.event("qd_warn", 2, "y", 2.f, 1.f, "d");
        }   // tmp 소멸 -> ~SafetyLog() -> close()
        auto DL = lines_of(dpath);
        chk(header_count(DL) == 1 && event_count(DL) == 1,
            "D: 소멸자가 스코프 종료로 안전하게 닫혔는지(파일 내용 보존) 확인");
    }

    // ── E) 체류 경계의 1 Hz 표본 기준 (최종 검토 M-7) ─────────────────────
    // State_Mimic::mon_reset() 이 체류마다 누적값(mon_clamp_ticks_ 등)을 0 으로 되돌린다. 표본의 기준(prev_*)을
    // 안 되돌리면 재진입 첫 표본이 «5 − 100» = uint32 ~4.29e9 로 찍혔다. stay_enter() 가 기준·표본 시계를 새로 잡는다.
    {
        const char* epath = "/tmp/_tsl_stay.csv";
        std::remove(epath);
        setenv("G1_SAFETY_CSV", epath, 1);
        g1::SafetyLog log;
        log.open_from_env();
        log.stay_enter("Mimic_Masked");
        log.sample(100, 0.02f, 3, 0, 0.f, -1, 5.f, nullptr);   // 첫 체류: 누적 100
        log.stay_enter("Mimic_Dance1_subject2");                // 재진입·전환 — 누적은 0 부터 다시
        log.sample(5, 0.01f, 3, 0, 0.f, -1, 5.f, nullptr);     // 둘째 체류: 누적 5 (1 s 를 안 기다리고 첫 표본)
        log.close();
        std::vector<std::string> vals;
        int n_stay = 0;
        for (auto& s : lines_of(epath)) {
            if (s.find(",edge,stay_enter,") != std::string::npos) ++n_stay;
            if (s.find(",rate,pos_clamp,") == std::string::npos) continue;
            std::vector<std::string> f;
            size_t a = 0, b;
            while ((b = s.find(',', a)) != std::string::npos) { f.push_back(s.substr(a, b - a)); a = b + 1; }
            f.push_back(s.substr(a));
            if (f.size() > 6) vals.push_back(f[6]);             // value 열 = 이번 표본의 증가분
        }
        chk(n_stay == 2, "E: stay_enter 사건이 체류마다 한 줄");
        chk(vals.size() == 2 && vals[0] == "100" && vals[1] == "5",
            "E: 재진입 첫 표본의 증가분 = 이번 체류 누적 그대로(5) — 옛 기준이면 5−100 이 uint 로 ~4.29e9");
        if (vals.size() == 2) std::printf("  E) pos_clamp 표본 증가분: 첫 체류 %s · 재진입 %s\n", vals[0].c_str(), vals[1].c_str());
    }

    // never-opened 인스턴스: close() 가 안전(두 번 호출도 안전)해야 한다.
    {
        g1::SafetyLog never;
        chk(!never.enabled(), "D: never-opened 는 off");
        never.close();
        never.close();
        chk(!never.enabled(), "D: close 뒤에도 off");
    }

    std::printf(fail ? "test_safety_log_shared: %d FAIL\n" : "test_safety_log_shared: OK\n", fail);
    return fail ? 1 : 0;
}
