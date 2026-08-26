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
    chk(!throws([&] { runner.verify_inputs(make(0)); }, msg),
        "크기가 정확히 맞으면 통과");
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

    std::printf("%s\n", g_fail ? "실패 있음" : "모두 통과");
    return g_fail ? 1 : 0;
}
