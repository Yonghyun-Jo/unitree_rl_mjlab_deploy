#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>
#include <cstdlib>      // std::exit — obs 계약 불일치 시 기동 거부

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        cmd = key_commands[key];
    }
    return cmd;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    // 🔴 obs 계약을 «모터가 돌기 전에» 대조한다. deploy.yaml 의 항 합이 ONNX 입력과 다르면
    //    act() 가 obs 버퍼를 ONNX 개수만큼 읽어 범위 밖을 건드리거나(작을 때) 조용히
    //    자른다(클 때) — 둘 다 에러 없이 「정책이 이상하다」로만 보인다. 여기서 죽인다.
    //    (lowcmd 인터록과 같은 방식으로 읽히는 메시지 + exit(1). 우회 플래그는 두지 않는다.)
    try {
        // deploy.yaml 이 선언한 항 목록을 그대로 넘긴다 (선언 순서 = obs 배치 순서).
        std::vector<isaaclab::ObsTermSpec> obs_terms;
        for (const auto& t : env->observation_manager->group_terms("obs"))
            obs_terms.push_back({t.name, t.train_name, t.dim(), t.history_length});
        // 🔴 «선언» 을 대조한다 — compute() 가 아니다. 여기는 State::enter() 이전이라
        //    motion 이 아직 안 붙었고, masked_joint_command 같은 항은 빈 벡터를 낸다.
        //    compute() 로 재면 1640 대신 1060 이 나와 멀쩡한 정책이 기동 거부된다
        //    (2026-08-26 sim2sim 에서 실제로 발생). 폭은 deploy.yaml 이 이미 선언하고
        //    있고 그것이 ONNX 와 맞는지가 이 검사의 질문이다. 실행 중 실제 크기는
        //    OrtRunner::act() 의 크기 가드가 매 스텝 따로 지킨다.
        std::unordered_map<std::string, std::vector<float>> declared;
        for (const auto& kv : env->observation_manager->declared_sizes())
            declared[kv.first] = std::vector<float>(kv.second);
        env->alg->verify_inputs(declared, obs_terms);
    } catch (const std::exception& e) {
        spdlog::critical("[obs contract] {}", e.what());
        spdlog::critical("  정책: {}", (policy_dir / "exported" / "policy.onnx").string());
        spdlog::critical("  계약: {}", (policy_dir / "params" / "deploy.yaml").string());
        std::exit(1);
    }

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}