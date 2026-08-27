#pragma once
// DeployFeatures.h — 슬롯이 «요구하는 C++ 기능» 과 이 바이너리가 «아는 기능» 을 맞춘다.
//
// # 왜 필요한가 — deploy.yaml 의 모르는 키는 «조용히 무시된다»
// 2026-08-26, mode1 을 COLMOv2 표로 옮기면서 deploy.yaml 에 table/cadence/turn_asym 을 넣었다.
// 그 키들을 파싱하는 코드가 없는 바이너리로 그 슬롯을 돌리면 — **에러가 안 난다.** yaml 파서는
// 모르는 키를 그냥 지나치고, 컨트롤러는 옛 거동(V1 표·cadence 1.0)으로 조용히 돈다.
// 로그도 정상으로 보인다. 「재빌드가 필요한가?」를 사람 기억에 맡기고 있었던 것이다.
//
// # 어떻게 막나 — obs 계약과 정확히 같은 사고방식
//   학습이 굽는 계약  ↔  ONNX metadata      (obs 항 목록·순서·차원)
//   슬롯이 선언하는 계약 ↔ deploy.yaml `requires:`  (필요한 C++ 기능)
// 둘 다 «선언» 을 기동 시점에 대조하고, 어긋나면 죽인다. 사람이 판단하지 않는다.
//
// # 기능을 언제 추가하나
// **거동을 바꾸는 새 deploy.yaml 키/블록을 넣을 때.** 없으면 조용히 무시될 것 = 기능 이름을
// 만들 것. 이름은 한 번 정하면 안 바꾼다(옛 슬롯의 requires 가 그 이름을 적고 있다).
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace g1_features {

// 이 바이너리가 «실제로 구현하고 있는» 기능. 코드를 넣을 때 여기에 같이 넣는다.
inline const std::vector<std::string>& known()
{
    static const std::vector<std::string> k = {
        // 기동 시 ONNX 메타데이터의 obs 계약을 deploy.yaml 과 대조한다 (7e39148 · f8dcef8)
        "obs_contract",
        // gait: 모드별 table(V1/V2) · cadence · turn_asym 을 읽어 발-z 생성기에 적용 (c3e103f)
        "gait_mode_table",
        // mode>=3 의 ref_foot_height 를 «레퍼런스 발 world-z» 에서 얻는다 (aedcc77)
        "ref_foot_height_ref",
        // 슬롯을 G1_POLICY_SLOT 으로 갈아끼울 수 있다 (하루 여러 정책 시험)
        "policy_slot_env",
        // 정지 정착: 명령이 0 이 되면 위상을 settle_phase 까지 더 돌려 «한 걸음 더» 구른다.
        // gait: settle_eff · settle_phase (스칼라) + 모드별 settle_steps — 0f9b31e
        "gait_settle",
    };
    return k;
}

inline bool has(const std::string& f)
{
    const auto& k = known();
    return std::find(k.begin(), k.end(), f) != k.end();
}

// 요구 목록 중 «이 바이너리가 모르는 것» 을 돌려준다. 비어 있으면 통과.
inline std::vector<std::string> missing(const std::vector<std::string>& required)
{
    std::vector<std::string> out;
    for (const auto& f : required)
        if (!f.empty() && !has(f)) out.push_back(f);
    return out;
}

// 사람이 «무엇을 해야 하는지» 를 바로 알 수 있는 문구. 재빌드 명령까지 준다.
inline std::string explain(const std::vector<std::string>& miss, const std::string& slot)
{
    std::ostringstream m;
    m << "이 슬롯이 요구하는 기능을 이 바이너리가 모른다 — 기동을 거부한다.";
    if (!slot.empty()) m << "\n  슬롯: " << slot;
    for (const auto& f : miss) m << "\n  - 모르는 기능: '" << f << "'";
    m << "\n  이 바이너리가 아는 기능:";
    for (const auto& f : known()) m << " " << f;
    m << "\n  → 이 슬롯은 «지금 도는 바이너리보다 새 코드» 를 전제로 만들어졌다. 재빌드할 것:"
         "\n       cd deploy/robots/g1/build && cmake .. && make -j4"
         "\n     (로봇이면 먼저 com1 에서 push -> 로봇에서 robot.sh deploy)"
         "\n  ⚠ 이 검사가 없으면 모르는 키는 «조용히 무시» 되어 옛 거동으로 돈다 — 에러도 안 난다.";
    return m.str();
}

}  // namespace g1_features
