// test_obs_contract.cpp — obs 크기 계약이 «기동 시» 강제되는지.
//
// # 무엇을 막는 테스트인가
// OrtRunner::act() 는 입력 «이름» 이 없으면 throw 하지만 **크기는 안 봤다**:
//
//     auto& input_data = obs.at(name_str);                  // 크기 = deploy.yaml
//     Ort::Value::CreateTensor<float>(mem, input_data.data(),
//                                     input_sizes[i], ...); // 개수 = ONNX
//
// deploy.yaml 합이 ONNX 입력보다 «작으면» 범위 밖을 읽고(UB), «크면» 조용히 잘린다.
// 둘 다 에러가 안 나고 정책이 쓰레기를 먹는 것으로만 나타난다 = 낙상.
//
// verify_inputs() 는 그 대조를 기동 시점에 한 번 하고, 어긋나면 죽인다.
//
// 빌드/실행 (com1·로봇 공통):
//   ORT=deploy/thirdparty/onnxruntime-linux-$(uname -m | sed 's/x86_64/x64/')-1.22.0
//   g++ -O2 -std=gnu++17 -I include -I $ORT/include tests/test_obs_contract.cpp \
//       $ORT/lib/libonnxruntime.so.1.22.0 -o /tmp/test_obs_contract
//   /tmp/test_obs_contract [policy.onnx 경로]
#include "isaaclab/algorithms/algorithms.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

static int g_fail = 0;

static void chk(bool cond, const char* what)
{
    std::printf("  %s %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) g_fail++;
}

// f() 가 던지는가. 던지면 메시지도 돌려준다(문구가 조작자에게 쓸모 있는지 같이 본다).
template <typename F>
static bool throws(F&& f, std::string& msg)
{
    try {
        f();
    } catch (const std::exception& e) {
        msg = e.what();
        return true;
    } catch (...) {
        msg = "(non-std exception)";
        return true;
    }
    msg.clear();
    return false;
}

int main(int argc, char** argv)
{
    const std::string model =
        argc > 1 ? argv[1]
                 : "config/policy/mimic_masked/v3_multihead_s30k_steps6/exported/policy.onnx";

    isaaclab::OrtRunner runner(model);

    // ONNX 가 스스로 아는 입력이 «진실» 이다. 여기서 이름·크기를 받아 시나리오를 만든다.
    const auto want = runner.input_sizes_by_name();
    chk(!want.empty(), "ONNX 입력이 하나 이상 있다");
    if (want.empty()) return 1;
    for (const auto& kv : want)
        std::printf("     ONNX 입력 '%s' = %lld\n", kv.first.c_str(),
                    static_cast<long long>(kv.second));

    auto make = [&](long long delta) {
        std::unordered_map<std::string, std::vector<float>> o;
        for (const auto& kv : want)
            o[kv.first] = std::vector<float>(static_cast<size_t>(kv.second + delta), 0.0f);
        return o;
    };

    std::string msg;

    // ① 정확히 맞으면 통과해야 한다 (정상 경로를 막으면 안 된다)
    //    terms 를 안 넘기는 호출부(= g1 외 5대) 는 계약이 있어도 «크기만» 보고 통과한다.
    chk(!throws([&] { runner.verify_inputs(make(0)); }, msg),
        "크기가 맞고 항 정보를 안 넘기면 통과 (다른 로봇 호출부를 안 죽인다)");
    if (!msg.empty()) std::printf("       (예상 못한 예외: %s)\n", msg.c_str());

    // ② 하나 모자라면 죽어야 한다 — 이게 범위 밖 읽기(UB) 였다
    chk(throws([&] { runner.verify_inputs(make(-1)); }, msg),
        "obs 가 1 작으면 throw (UB 대신 죽는다)");
    chk(msg.find("obs") != std::string::npos || msg.find("입력") != std::string::npos,
        "메시지가 obs 계약 문제임을 말한다");
    std::printf("       msg: %s\n", msg.c_str());

    // ③ 하나 많으면 죽어야 한다 — 조용히 잘리던 경우
    chk(throws([&] { runner.verify_inputs(make(+1)); }, msg),
        "obs 가 1 크면 throw (조용한 절단 대신 죽는다)");

    // ④ 이름이 아예 없으면 죽어야 한다
    chk(throws([&] {
            std::unordered_map<std::string, std::vector<float>> o;
            o["존재하지_않는_그룹"] = std::vector<float>(4, 0.0f);
            runner.verify_inputs(o);
        }, msg),
        "ONNX 가 요구하는 입력이 obs 에 없으면 throw");


    // ─────────────────────────────────────────────────────────────────────
    // 항 계약 (이름·순서·차원·history). 크기만 보면 못 잡는 것들이다.
    // ─────────────────────────────────────────────────────────────────────
    std::printf("\n-- 계약 파서 --\n");
    {
        const auto t = isaaclab::parse_obs_contract("a:3:10, b:29:10 ,c:2:1");
        chk(t.size() == 3, "3개 항을 읽는다");
        chk(t[0].name == "a" && t[0].dim == 3 && t[0].history == 10, "이름·차원·history 파싱");
        chk(t[2].history == 1, "history 1 도 읽는다");
        chk(throws([&] { isaaclab::parse_obs_contract("a:3"); }, msg), "형식이 틀리면 throw");
    }

    // 실제 배포 obs 레이아웃 (deploy.yaml v3 슬롯). 학습 이름 기준.
    const char* GOOD =
        "base_ang_vel:3:10,projected_gravity:3:10,command:58:10,motion_root_ori_b:6:10,"
        "joint_pos:29:10,joint_vel:29:10,actions:29:10,base_vel:3:10,mask:2:10,foot_z:2:10";

    auto specs = [](std::initializer_list<const char*> train_names,
                    std::initializer_list<int> dims) {
        std::vector<isaaclab::ObsTermSpec> v;
        auto d = dims.begin();
        for (auto n : train_names) { v.push_back({std::string("deploy_") + n, n, *d++, 10}); }
        return v;
    };
    const auto OK_SPECS = specs(
        {"base_ang_vel","projected_gravity","command","motion_root_ori_b","joint_pos",
         "joint_vel","actions","base_vel","mask","foot_z"},
        {3,3,58,6,29,29,29,3,2,2});

    std::printf("\n-- 항 대조 (계약이 없는 ONNX = 구버전) --\n");
    {
        // 배포 중인 ONNX 에는 아직 계약이 없다 -> 경고만 하고 통과해야 한다.
        const bool has = !runner.obs_contract().empty();
        std::printf("     이 ONNX 의 계약: %s\n", has ? runner.obs_contract().c_str() : "(없음)");
        if (!has) {
            chk(!throws([&] { runner.verify_inputs(make(0), OK_SPECS); }, msg),
                "계약 없는 ONNX 는 경고만 하고 통과 (기존 슬롯이 안 죽는다)");
        } else {
            // 🔴 실제 기동 경로. 배포 항 목록을 «그대로» 먹여 ONNX 안의 계약과 대조한다.
            chk(!throws([&] { runner.verify_inputs(make(0), OK_SPECS); }, msg),
                "이 ONNX 의 계약이 실제 배포 항 배치와 일치한다");
            if (!msg.empty()) std::printf("       (예외: %s)\n", msg.c_str());

            // 그리고 «틀리면 진짜로 죽는지» — 29 짜리 둘을 바꿔 넣는다(합 1640 불변).
            auto swapped = OK_SPECS;
            std::swap(swapped[4], swapped[5]);
            chk(throws([&] { runner.verify_inputs(make(0), swapped); }, msg),
                "joint_pos/joint_vel 을 바꿔 넣으면 실제로 기동을 거부한다");

            auto renamed = OK_SPECS;
            renamed[9].train_name = "ref_foot_height";   // 배포 이름을 train_term 에 잘못 적은 경우
            chk(throws([&] { runner.verify_inputs(make(0), renamed); }, msg),
                "train_term 에 배포 이름을 적으면 거부한다");
        }
    }

    std::printf("\n-- 항 대조 (계약을 직접 먹여서) --\n");
    {
        // verify_terms 는 ONNX 메타데이터를 읽으므로 여기서는 파서+비교 로직을
        // 같은 규칙으로 재현해 검증한다. (메타데이터 주입은 export 쪽 책임)
        auto want = isaaclab::parse_obs_contract(GOOD);
        auto cmp = [&](const std::vector<isaaclab::ObsTermSpec>& got) {
            if (want.size() != got.size()) return false;
            for (size_t i = 0; i < want.size(); ++i)
                if (want[i].name != got[i].train_name || want[i].dim != got[i].dim
                    || want[i].history != got[i].history) return false;
            return true;
        };
        chk(cmp(OK_SPECS), "정상 배치는 계약과 일치");

        // 🔴 이것이 (a) 로는 절대 못 잡는 사고: 29 짜리 셋이 서로 뒤바뀜. 합은 1640 그대로.
        auto swapped = OK_SPECS;
        std::swap(swapped[4], swapped[5]);          // joint_pos <-> joint_vel
        chk(!cmp(swapped), "29 짜리 joint_pos/joint_vel 교환을 잡는다 (합은 1640 그대로)");
        int tot = 0;
        for (const auto& t : swapped) tot += t.dim * t.history;
        chk(tot == 1640, "  실제로 합은 1640 로 같다 = 크기검사만으론 통과했을 것");

        auto wrong_dim = OK_SPECS;
        wrong_dim[2].dim = 57;
        chk(!cmp(wrong_dim), "차원 불일치를 잡는다");

        auto wrong_hist = OK_SPECS;
        wrong_hist[0].history = 5;
        chk(!cmp(wrong_hist), "history 불일치를 잡는다");

        auto missing = OK_SPECS;
        missing.pop_back();
        chk(!cmp(missing), "항이 하나 빠지면 잡는다");

        auto undeclared = OK_SPECS;
        undeclared[3].train_name = "";              // deploy.yaml 에 train_term 미선언
        chk(!cmp(undeclared), "train_term 미선언을 잡는다");
    }

    std::printf("%s\n", g_fail ? "실패 있음" : "모두 통과");
    return g_fail ? 1 : 0;
}
