#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include <cstdio>
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
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        // 이미 누군가 rt/lowcmd 를 발행 중 = 다른 컨트롤러가 살아있다(크래시 후 orphan 포함).
        // 둘 다 모터 명령을 인가하면 토크가 충돌해 policy 와 무관하게 즉시 낙상한다.
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        spdlog::critical("  확인: pgrep -x g1_ctrl   (반드시 -x. pgrep -f 는 검색 명령 자신을 self-match 해 오탐)");
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

int main(int argc, char** argv)
{
    ::signal(SIGINT,  restore_tty_and_die);   // 종료 시 터미널 복구 (동작은 기존과 동일)
    ::signal(SIGTERM, restore_tty_and_die);

    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

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

