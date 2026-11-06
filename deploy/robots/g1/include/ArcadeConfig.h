#pragma once
// ArcadeConfig.h — 아케이드 경로(G1_ARCADE=1)의 FSM 설정 = config.yaml 을 읽은 노드에서 «위험 전이» 를 지운 사본. g1 전용(🅐 구역).
//   정본은 config.yaml 하나다(예전 사본 파일 config_arcade.yaml 은 없앴다 — stage_candidates.py 등이 config.yaml 만 고쳐도
//   아케이드 설정이 어긋나지 않게, 최종 검토 I4). main.cpp 가 param::helper 직후(CtrlFSM · FSMState 생성자가 param::config 를
//   읽기 «전») 한 번 부른다. 원본 노드는 건드리지 않는다(YAML::Clone 한 사본에서 지운다).
//
//   지우는 것: 어느 상태에서든 Velocity · Mimic_Dance1_subject2 로 가는 전이(리모컨 transitions · keyboard_transitions) +
//     키보드 'v' 전이. 오늘 config.yaml 에선 그것이 kArcadeRemoved 의 6 개다.
//   거부(false + err — 호출부가 기동 거부):
//     · kArcadeRemoved 의 전이가 원본에 하나라도 없다 = config 가 바뀌었다(조용히 넘어가지 않는다 — 규칙을 다시 볼 것)
//     · 지운 뒤 Passive 에서 닿는 상태(리모컨 · 키보드 전이)가 {Passive, FixStand, Mimic_Masked} 가 아니다
//       (새 위험 전이 · 새 상태 — 또는 아케이드 진입 m 이 없어짐)
//   남는 것: Passive(LT+up · f) · FixStand(LT+B · p · m) · Mimic_Masked(LT+B · p) 와 그 밖 모든 키(정책 · 클립 · 게인 · imu_cal …).
//   시험 = tests/test_arcade_gate.cpp ④(실제 config.yaml 로 — 함수 결과 대조).
// 헤더 온리 · yaml-cpp 만(core 헤더 없음 — 아케이드 없는 빌드에서도 시험이 돈다).
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace g1::arcade_config {

struct Transition {
  const char* state;    // FSM.<state>
  const char* section;  // transitions(리모컨) · keyboard_transitions(키보드)
  const char* target;   // 가는 상태
};

// 오늘 config.yaml 의 위험 전이 6 개(T8 의 config_arcade.yaml 이 뺀 것과 같다). 하나라도 없으면 거부.
inline constexpr Transition kArcadeRemoved[] = {
    {"FixStand", "transitions", "Velocity"},                // RT + A
    {"FixStand", "keyboard_transitions", "Velocity"},       // v
    {"Velocity", "transitions", "Mimic_Dance1_subject2"},   // RB + A
    {"Mimic_Dance1_subject2", "transitions", "Velocity"},   // RT + A
    {"Mimic_Masked", "transitions", "Velocity"},            // RT + A
    {"Mimic_Masked", "keyboard_transitions", "Velocity"},   // v
};
inline constexpr std::size_t kArcadeRemovedN = sizeof(kArcadeRemoved) / sizeof(kArcadeRemoved[0]);

inline bool is_gone_target(const std::string& t) { return t == "Velocity" || t == "Mimic_Dance1_subject2"; }

// Passive 에서 리모컨 · 키보드 전이로 닿는 상태(시작 = Passive). fsm = FSM 노드(const — 읽기만).
inline std::set<std::string> reachable_from_passive(const YAML::Node& fsm) {
  std::set<std::string> seen{"Passive"};
  std::vector<std::string> todo{"Passive"};
  while (!todo.empty()) {
    const std::string s = todo.back();
    todo.pop_back();
    const YAML::Node st = fsm[s];
    if (!st || !st.IsMap()) continue;
    for (const char* sect : {"transitions", "keyboard_transitions"}) {
      const YAML::Node t = st[sect];
      if (!t || !t.IsMap()) continue;
      for (const auto& kv : t) {
        const std::string to = kv.first.as<std::string>();
        if (seen.insert(to).second) todo.push_back(to);
      }
    }
  }
  return seen;
}

// base(= config.yaml 을 읽은 노드 — 바꾸지 않는다) → out(아케이드 설정). 실패면 false + err(out 은 손대지 않는다).
// removed(있으면) = 지운 전이 목록 «상태.칸.대상(값)» 을 « · » 로 이은 글(기동 로그).
inline bool arcade_config_from(const YAML::Node& base, YAML::Node& out, std::string& err, std::string* removed = nullptr) {
  const YAML::Node bfsm = base["FSM"];
  if (!bfsm || !bfsm.IsMap()) {
    err = "config.yaml 에 FSM 맵이 없다";
    return false;
  }
  // 1) 지울 전이가 원본에 다 있나(없으면 config 가 바뀐 것 — 거부)
  std::string missing;
  for (const Transition& t : kArcadeRemoved) {
    const YAML::Node st = bfsm[t.state];
    const YAML::Node sec = (st && st.IsMap()) ? st[t.section] : YAML::Node();
    if (!sec || !sec.IsMap() || !sec[t.target])
      missing += (missing.empty() ? "" : " · ") + std::string(t.state) + "." + t.section + "." + t.target;
  }
  if (!missing.empty()) {
    err = "config.yaml 에 지울 위험 전이가 없다(" + missing +
          ") — config 가 바뀌었다. 아케이드 전이 규칙(include/ArcadeConfig.h kArcadeRemoved)을 다시 볼 것";
    return false;
  }
  // 2) 사본에서 지운다: 어느 상태든 Velocity · Dance 로 가는 전이 + 키보드 'v'
  YAML::Node cfg = YAML::Clone(base);
  const YAML::Node cfsm = static_cast<const YAML::Node&>(cfg)["FSM"];  // const 로 찾는다(non-const [] 는 빈 칸을 만든다)
  std::string list;
  std::size_t n_removed = 0;
  for (const auto& st : cfsm) {
    const std::string name = st.first.as<std::string>();
    if (name == "_" || !st.second.IsMap()) continue;
    for (const char* sect : {"transitions", "keyboard_transitions"}) {
      const YAML::Node csec = st.second[sect];
      if (!csec || !csec.IsMap()) continue;
      std::vector<std::pair<std::string, std::string>> drop;
      for (const auto& kv : csec) {
        const std::string to = kv.first.as<std::string>();
        const std::string val = kv.second.IsScalar() ? kv.second.Scalar() : std::string("?");
        if (is_gone_target(to) || (std::string(sect) == "keyboard_transitions" && val == "v")) drop.emplace_back(to, val);
      }
      YAML::Node sec = csec;  // 같은 노드(손잡이 복사) — 지우기는 non-const 로
      for (const auto& d : drop) {
        sec.remove(d.first);
        ++n_removed;
        list += (list.empty() ? "" : " · ") + name + "." + sect + "." + d.first + "(" + d.second + ")";
      }
    }
  }
  // 3) 지운 뒤 Passive 에서 닿는 상태 = {Passive, FixStand, Mimic_Masked}
  const std::set<std::string> want{"Passive", "FixStand", "Mimic_Masked"};
  const std::set<std::string> got = reachable_from_passive(cfsm);
  if (got != want) {
    std::string g;
    for (const std::string& s : got) g += (g.empty() ? "" : ", ") + s;
    err = "위험 전이를 지운 뒤 Passive 에서 닿는 상태가 {Passive, FixStand, Mimic_Masked} 가 아니다: {" + g +
          "} — config.yaml 에 새 전이 · 상태가 생겼거나 진입(m)이 없어졌다";
    return false;
  }
  if (removed) *removed = std::to_string(n_removed) + " 개: " + list;
  out = cfg;
  return true;
}

}  // namespace g1::arcade_config
