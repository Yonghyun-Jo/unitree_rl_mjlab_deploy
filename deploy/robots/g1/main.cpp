#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "ArcadeGate.h"   // G1_ARCADE 값(arcade_request) — 아케이드 경로면 FSM 설정 = config.yaml 에서 위험 전이를 지운 노드
#if G1_HAVE_ARCADE
#include "ArcadeConfig.h" // arcade_config_from(config.yaml 노드) — 아케이드 경로의 FSM 설정(정본 = config.yaml 하나)
#endif
#include <atomic>
#include <cmath>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <termios.h>
#include <unistd.h>

namespace {
// 아래 Keyboard 전역은 생성자에서 tty 를 -icanon -echo 로 바꾸고 소멸자에서만 되돌린다.
// Ctrl-C(SIGINT)/pkill(SIGTERM) 은 소멸자를 태우지 않으므로 종료 후 셸 터미널이 raw 로
// 남는다(입력 echo 없음, 줄 단위 입력 안 됨). 여기서 원래 설정을 미리 떠 두고 시그널에서 되돌린다.
// ⚠ 같은 TU 안의 동적 초기화는 선언 순서대로 실행되므로 이 블록은 반드시 keyboard 정의보다 위.
termios g_tty_orig;
const bool g_tty_saved = (::tcgetattr(STDIN_FILENO, &g_tty_orig) == 0);

// 기동 거부(아케이드 설정 오류) — exit() 는 정적 소멸자에서 join 안 된 Keyboard 스레드로 abort(core dump)한다.
// State_Mimic.cpp die_startup 과 같이 로그만 비우고 _Exit 하되, 터미널은 여기서 되돌린다(Keyboard 소멸자가 안 돈다).
[[noreturn]] void die_startup_main()
{
    if(g_tty_saved) ::tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_orig);
    spdlog::default_logger()->flush();
    std::_Exit(1);
}

extern "C" void restore_tty_and_die(int sig)
{
    // tcsetattr/signal/raise 는 async-signal-safe. 로봇 관련 정리는 일절 하지 않는다 —
    // 기존과 동일하게 '그 시그널로 죽는' 것만 유지하고 터미널만 되살린다.
    if(g_tty_saved) ::tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_orig);
    ::signal(sig, SIG_DFL);
    ::raise(sig);
}
}  // namespace

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state(bool allow_lowcmd_conflict)
{
    // 이미 누군가 rt/lowcmd 를 발행 중 = 다른 컨트롤러가 살아있다(크래시 후 orphan 포함).
    // 둘 다 모터 명령을 인가하면 토크가 충돌해 policy 와 무관하게 즉시 낙상한다.
    //
    // 🔴 «몇 건 왔고 그 명령이 무엇인지» 까지 보고한다. 예전엔 "close it first" 와
    //    `pgrep -x g1_ctrl` 만 안내했는데, 발행자가 g1_ctrl 이 아니면(로봇 고수준 서비스,
    //    다른 사용자, ArmSdk 등) pgrep 이 비어서 조작자가 «오탐인가?» 로 오해하고
    //    --allow-lowcmd-conflict 로 우회하게 된다. 그게 이 검사가 막으려던 바로 그 상황이다.
    //    (2026-08-20 실제로 그렇게 오해했다.)
    std::atomic<int> lowcmd_seen{0};
    unitree_hg::msg::dds_::LowCmd_ lowcmd_sample{};
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>(
        "rt/lowcmd");
    usleep(0.6 * 1e6);   // timeout_ms_(1000) 보다 짧게 — 그 안에 오면 «지금» 발행 중이다
    if(!lowcmd_sub->isTimeout())
    {
        {   // 마지막 수신 명령을 들여다본다. kp>0 이면 «능동 제어», 전부 0 이면 관성/대기다.
            std::lock_guard<std::mutex> lk(lowcmd_sub->mutex_);
            lowcmd_sample = lowcmd_sub->msg_;
        }
        float kp_max = 0.f, tau_max = 0.f;
        for(size_t i = 0; i < 29 && i < lowcmd_sample.motor_cmd().size(); ++i) {
            kp_max  = std::max(kp_max,  std::abs(lowcmd_sample.motor_cmd()[i].kp()));
            tau_max = std::max(tau_max, std::abs(lowcmd_sample.motor_cmd()[i].tau()));
        }
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        spdlog::critical("  수신한 명령: kp_max={:.1f} tau_max={:.2f}  ({})",
                         kp_max, tau_max,
                         kp_max > 0.f ? "🔴 능동 제어 중 — 모터를 잡고 있다"
                                      : "kp=0 — 대기/댐핑 상태지만 채널은 점유 중");
        spdlog::critical("  확인: pgrep -x g1_ctrl   (반드시 -x. pgrep -f 는 검색 명령 자신을 self-match 해 오탐)");
        spdlog::critical("  ⚠ pgrep 이 비어도 «오탐이 아니다» — 발행자가 g1_ctrl 이 아닐 수 있다");
        spdlog::critical("     (로봇 고수준 서비스 / 다른 사용자 / ArmSdk 등). 채널에 실제로 명령이 흐르고 있다.");
        spdlog::critical("  정리: pkill -x g1_ctrl   (남의 세션일 수 있으니 확인 후)");
        unitree::robot::go2::shutdown();
        if(!allow_lowcmd_conflict)
        {
            spdlog::critical("  기동 거부. 의도적으로 무시하려면 --allow-lowcmd-conflict (권장하지 않음).");
            exit(1);
        }
        spdlog::warn("--allow-lowcmd-conflict: lowcmd 충돌을 무시하고 계속합니다 (위험 — 낙상 가능).");
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

// 부팅에 쓴 DDS 인터페이스. 진단용 부하 주입이 «실로봇인가»를 알아야 해서 남긴다
// (sim2sim = lo). 조작 경로가 아니다 — 다른 용도로 쓰지 말 것.
std::string g_network_iface;

int main(int argc, char** argv)
{
    ::signal(SIGINT,  restore_tty_and_die);   // 종료 시 터미널 복구 (동작은 기존과 동일)
    ::signal(SIGTERM, restore_tty_and_die);

    // Load parameters
    auto vm = param::helper(argc, argv);
    // 아케이드 경로(G1_ARCADE=1): FSM 설정 = param.h 가 읽은 config.yaml 노드에서 위험 전이(Velocity · Dance · 키 v)를 지운 사본
    // (ArcadeConfig.h arcade_config_from — 정본은 config.yaml 하나) — CtrlFSM 과 각 FSMState 생성자가 param::config 를 읽기 «전» 이다
    // (param.h 는 config.yaml 만 싣는다 — 🅑 무변경, 2b-facts B15).
    switch (g1::arcade_gate::arcade_request(std::getenv("G1_ARCADE"))) {
        case g1::arcade_gate::Request::Off:
            break;
        case g1::arcade_gate::Request::Bad:
            spdlog::critical("[arcade] G1_ARCADE='{}' 는 모르는 값 — 1(켬) 또는 0/없음(끔). 기동 거부.", std::getenv("G1_ARCADE"));
            die_startup_main();
        case g1::arcade_gate::Request::On: {
#if G1_HAVE_ARCADE
            const std::string src = (param::config_dir / "config.yaml").string();
            YAML::Node arc;
            std::string why, removed;
            bool ok = false;
            try {
                ok = g1::arcade_config::arcade_config_from(param::config, arc, why, &removed);
            } catch (const std::exception& e) {
                why = std::string("yaml: ") + e.what();
            }
            if (!ok) {
                spdlog::critical("[arcade] 아케이드 FSM 설정을 만들 수 없다({}): {} — 기동 거부.", src, why);
                die_startup_main();
            }
            param::config = arc;   // 원본 노드는 arcade_config_from 이 건드리지 않았다(사본에서 지움)
            spdlog::info("[arcade] G1_ARCADE=1 — FSM 설정 = {} 에서 위험 전이를 지운 노드(지움 {}) · 남은 전이 Passive LT+B/p · f · m",
                         src, removed);
#else
            spdlog::critical("[arcade] 이 g1_ctrl 은 아케이드 없이 빌드됨(G1_ARCADE_DIR 없음 — cmake 경고 «아케이드 없음») — "
                             "G1_ARCADE=1 을 쓸 수 없다. -DG1_ARCADE_DIR=<g1_arcade> 로 다시 빌드할 것. 기동 거부.");
            die_startup_main();
#endif
            break;
        }
    }

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    g_network_iface = vm["network"].as<std::string>();
    unitree::robot::ChannelFactory::Instance()->Init(0, g_network_iface);

    init_fsm_state(vm.count("allow-lowcmd-conflict") > 0);

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    std::remove("/dev/shm/g1_estop");   // 부팅 시 orphan estop 파일 제거(크래시로 남은 stale이 무장 상태로 오인되는 것 방지). 라이브 브릿지가 다음 프레임에 재무장.
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    std::cout << "And then press [R1 + A/B/Y/X] to control the robot dance.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

