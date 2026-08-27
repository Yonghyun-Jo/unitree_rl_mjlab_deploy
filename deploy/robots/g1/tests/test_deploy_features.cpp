// test_deploy_features.cpp — 슬롯의 `requires:` 와 바이너리의 기능 집합 대조.
//
// 막는 것: 「옛 바이너리 + 새 슬롯」. deploy.yaml 의 모르는 키는 yaml 파서가 그냥 지나치므로
// 에러 없이 옛 거동으로 돈다(2026-08-26 gait table/cadence/turn_asym 이 실제로 그럴 뻔했다).
//
//   g++ -std=gnu++17 -O2 -I../include test_deploy_features.cpp -o /tmp/tdf && /tmp/tdf
#include "DeployFeatures.h"

#include <cstdio>
#include <string>
#include <vector>

static int fail = 0;
static void chk(bool c, const char* what)
{
    std::printf("  %s %s\n", c ? "ok  " : "FAIL", what);
    if (!c) ++fail;
}

int main()
{
    using namespace g1_features;

    std::printf("-- 기능 집합 --\n");
    chk(!known().empty(), "아는 기능이 하나 이상 있다");
    chk(has("obs_contract"), "obs_contract 를 안다");
    chk(has("gait_mode_table"), "gait_mode_table 을 안다 (c3e103f 로 구현됨)");
    chk(!has("gait_mode_table_v99"), "없는 기능은 모른다고 한다");
    chk(!has(""), "빈 이름은 «안다» 가 아니다");

    std::printf("\n-- 대조 --\n");
    chk(missing({}).empty(), "요구가 없으면 통과 (requires: 없는 옛 슬롯)");
    chk(missing({"obs_contract", "gait_mode_table"}).empty(),
        "전부 아는 기능이면 통과");

    auto m = missing({"obs_contract", "quantum_gait"});
    chk(m.size() == 1 && m[0] == "quantum_gait", "모르는 것만 골라낸다");

    m = missing({"aaa", "bbb"});
    chk(m.size() == 2, "여러 개도 전부 보고한다");

    // 빈 문자열은 «요구» 가 아니다 (yaml 이 빈 항목을 낼 수 있다)
    chk(missing({""}).empty(), "빈 항목은 요구로 안 센다");

    std::printf("\n-- 사람이 읽는 문구 --\n");
    const std::string e = explain({"quantum_gait"}, "v9_test");
    chk(e.find("quantum_gait") != std::string::npos, "모르는 기능 이름이 들어간다");
    chk(e.find("v9_test") != std::string::npos, "슬롯 이름이 들어간다");
    chk(e.find("make") != std::string::npos, "재빌드 명령을 준다");
    chk(e.find("robot.sh deploy") != std::string::npos, "로봇일 때의 경로도 준다");
    chk(e.find("조용히 무시") != std::string::npos, "왜 위험한지 말한다");
    std::printf("     ---- 실제 문구 ----\n%s\n", e.c_str());

    std::printf("\n%s\n", fail ? "실패 있음" : "[test_deploy_features] ALL PASS");
    return fail ? 1 : 0;
}
