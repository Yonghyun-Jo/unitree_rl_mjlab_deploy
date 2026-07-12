# 텔레옵 관절 안전 3층 (position clamp + velocity rate-limit + qd-guard) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 텔레옵 정책(Mimic_Masked)의 모터 출력 경로에, 정책이 OOD/발산해도 로봇을 보호하는 joint-space 안전 3층(위치 clamp, 속도 rate-limit, 측정 qd 폭주 감지→mode1/Passive)을 **잘못 구현해도 정상 배포를 깨지 않게** 증분·config-gated·no-op-default로 추가한다.

**Architecture:** 순수·테스트가능 함수를 `JointSafety.h`(self-contained)에 두고, `State_Mimic::run()`의 출력 경로(apply_switch_blend 이후 → motor_cmd.q() 쓰기 이전)에 config-gated로 끼운다. 한계값은 `deploy.yaml`의 `safety:` 블록에서 로드(fail-safe: 없거나 이상하면 비활성). 각 층은 독립 enable flag, **커밋 기본값 = 비활성(=배포 시 동작 0 변화)**, sim에서 하나씩 켜서 검증 후 실로봇.

**Tech Stack:** C++17 (g1_ctrl, State_Mimic, yaml-cpp), 순수함수 단위테스트(standalone g++), deploy.yaml config.

## Global Constraints

- **커밋 기본값 = 동작 무변화**: `enable_pos_clamp` / `enable_rate_limit` / `enable_qd_guard` 셋 다 committed deploy.yaml에서 **false**. 단, `action.clip`은 기계한계로 채움(in-distribution엔 provably no-op — default_joint_pos·학습 q가 전부 안쪽; OOD/beyond-mechanical만 clamp).
- **fail-safe 파싱**: `deploy.yaml`에 `safety:` 없거나 배열 길이≠29/파싱 실패 → 해당 층 **비활성**. **절대 0으로 clamp하거나 throw하지 않는다**(로봇 limp/컨트롤러 死 방지).
- **한계값 출처**: 위치 = `src/assets/robots/unitree_g1/xmls/g1.xml` 기계한계(아래 29값). 속도 = 학습 rollout qd p99×여유(Task 5에서 측정; 기본값은 clearly-high). in-distribution q/qd는 **한계 안쪽**이어야 함.
- **parity**: `last_action` obs는 **raw 정책 출력**(안전층 적용 전). 안전층은 **출력 전용 필터** — clamp/rate-limit된 값을 절대 obs로 되먹이지 않는다.
- **NaN-safe**: 안전함수는 non-finite 입력에 안전(위치: 건너뜀/hold, 속도: hold last-good, qd: NaN→crit).
- **순수·단위테스트**: clamp/rate-limit/qd-severity는 `JointSafety.h` 순수함수 → "비활성/wide면 입력==출력", "tight면 정확히 clamp" 증명. 기존 `test_masked_loco_controller.cpp` golden은 그대로 통과(MaskedLocoController 무변경).
- **증분 롤아웃**: L1 → L2 → L3, 각각 sim2sim에서 보행 검증 후 다음. 실로봇은 **마지막**, mode1부터. 각 층 enable은 검증된 단계에서만.
- **범위**: Mimic_Masked(gmt_multihead_v0) 텔레옵 정책. Velocity 정책은 범위 밖.
- **deploy JOINT_ORDER (29, 모든 배열 이 순서)**: `legL[0:6] legR[6:12] waist[12:15] armL[15:22] armR[22:29]`.
- 커밋 메시지: `Co-Authored-By` 등 AI 트레일러 **금지**. Korean 주석. 작업 브랜치: `smooth_mode_switch`(편집 전 `git branch --show-current` 확인).

**G1 기계한계 (deploy 순서 29, g1.xml에서 추출):**
```
# idx joint                     min        max
 0 left_hip_pitch      -2.5307   2.8798
 1 left_hip_roll       -0.5236   2.9671
 2 left_hip_yaw        -2.7576   2.7576
 3 left_knee           -0.087267 2.8798
 4 left_ankle_pitch    -0.87267  0.5236
 5 left_ankle_roll     -0.2618   0.2618
 6 right_hip_pitch     -2.5307   2.8798
 7 right_hip_roll      -2.9671   0.5236
 8 right_hip_yaw       -2.7576   2.7576
 9 right_knee          -0.087267 2.8798
10 right_ankle_pitch   -0.87267  0.5236
11 right_ankle_roll    -0.2618   0.2618
12 waist_yaw           -2.618    2.618
13 waist_roll          -0.52     0.52
14 waist_pitch         -0.52     0.52
15 left_shoulder_pitch -3.0892   2.6704
16 left_shoulder_roll  -1.5882   2.2515
17 left_shoulder_yaw   -2.618    2.618
18 left_elbow          -1.0472   2.0944
19 left_wrist_roll     -1.97222  1.97222
20 left_wrist_pitch    -1.61443  1.61443
21 left_wrist_yaw      -1.61443  1.61443
22 right_shoulder_pitch -3.0892  2.6704
23 right_shoulder_roll -2.2515   1.5882
24 right_shoulder_yaw  -2.618    2.618
25 right_elbow         -1.0472   2.0944
26 right_wrist_roll    -1.97222  1.97222
27 right_wrist_pitch   -1.61443  1.61443
28 right_wrist_yaw     -1.61443  1.61443
```
(검증: default_joint_pos = [-0.312,0,0,0.669,-0.363,0,...] 은 전부 위 범위 안쪽.)

---

## File Structure

**신규**
- `deploy/robots/g1/include/JointSafety.h` — 순수함수 3개(clamp_position, rate_limit, qd_severity) + `JointSafetyCfg`/`JointSafetyState` 구조체. self-contained(`<algorithm>`,`<cmath>`,`<array>`,`<cstddef>`만). 단일 책임: joint-space 안전 연산.
- `deploy/robots/g1/tests/test_joint_safety.cpp` — standalone 단위테스트(기존 test_masked_loco_controller 관례).

**수정**
- `deploy/robots/g1/config/policy/mimic_masked/gmt_multihead_v0/params/deploy.yaml` — `action.clip` 기계한계 채움 + `safety:` 블록 신설(enable=false, 한계값 pre-fill).
- `deploy/robots/g1/include/State_Mimic.h` — safety cfg/state 멤버 + qd-guard 플래그.
- `deploy/robots/g1/src/State_Mimic.cpp` — 생성자에서 `safety:` 로드(fail-safe); `run()` 출력 경로에 L1/L2 gated; `run()`에 L3 qd 검사; L3-crit registered_check→Passive; enter()에서 rate-limit state init.
- `deploy/robots/g1/teleop/README.md` (또는 `rules/SYSTEM_OVERVIEW.md`) — 3층·config·증분 enable 절차.

---

## Task 1: JointSafety.h 순수함수 + 단위테스트

**Files:**
- Create: `deploy/robots/g1/include/JointSafety.h`
- Test: `deploy/robots/g1/tests/test_joint_safety.cpp`

**Interfaces:**
- Produces:
  - `void js_clamp_position(float* q, const float* lo, const float* hi, int n)` — 각 i에 대해 `isfinite(q[i])`면 `q[i]=clamp(q[i],lo[i],hi[i])`, 아니면 그대로 둠.
  - `void js_rate_limit(float* q, float* q_prev, const float* max_step, int n)` — 각 i: `isfinite(q[i])`면 `q[i]=q_prev[i]+clamp(q[i]-q_prev[i], -max_step[i], max_step[i])`, 아니면 `q[i]=q_prev[i]`(hold); 그 후 `q_prev[i]=q[i]`.
  - `int js_qd_severity(const float* qd, int n, float warn, float crit)` — `m=max_i |qd[i]|`(NaN이면 즉시 2 반환); `m>crit`→2, `m>warn`→1, else 0.

- [ ] **Step 1: 실패 테스트 작성** — `deploy/robots/g1/tests/test_joint_safety.cpp`

```cpp
// test_joint_safety.cpp — JointSafety 순수함수 자체검증 (기존 test_masked_loco_controller 관례).
//   cd deploy/robots/g1/tests && g++ -std=c++17 -I../include -O2 test_joint_safety.cpp -o /tmp/tjs && /tmp/tjs
#include "JointSafety.h"
#include <cstdio>
#include <cmath>
#include <limits>

static int fail = 0;
static bool close(float a, float b) { return std::fabs(a - b) < 1e-6f; }
#define CHK(c, msg) do{ if(!(c)){ std::printf("FAIL %s\n", msg); ++fail; } }while(0)

int main() {
    // clamp: wide -> no-op; tight -> clamp; NaN -> untouched
    { float q[3]={0.5f,-3.0f,2.0f}; float lo[3]={-10,-10,-10}, hi[3]={10,10,10};
      js_clamp_position(q,lo,hi,3); CHK(close(q[0],0.5f)&&close(q[1],-3.0f)&&close(q[2],2.0f),"clamp wide no-op"); }
    { float q[3]={0.5f,-3.0f,2.0f}; float lo[3]={-1,-1,-1}, hi[3]={1,1,1};
      js_clamp_position(q,lo,hi,3); CHK(close(q[0],0.5f)&&close(q[1],-1.0f)&&close(q[2],1.0f),"clamp tight"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float q[1]={nan}; float lo[1]={-1},hi[1]={1};
      js_clamp_position(q,lo,hi,1); CHK(std::isnan(q[0]),"clamp NaN untouched"); }

    // rate_limit: within step -> no-op + prev updates; over step -> capped; NaN -> hold prev
    { float q[1]={0.1f}; float prev[1]={0.0f}; float step[1]={1.0f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.1f)&&close(prev[0],0.1f),"rate within"); }
    { float q[1]={5.0f}; float prev[1]={0.0f}; float step[1]={0.5f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.5f)&&close(prev[0],0.5f),"rate cap up"); }
    { float q[1]={-5.0f}; float prev[1]={0.0f}; float step[1]={0.5f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],-0.5f),"rate cap down"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float q[1]={nan}; float prev[1]={0.3f}; float step[1]={1.0f};
      js_rate_limit(q,prev,step,1); CHK(close(q[0],0.3f)&&close(prev[0],0.3f),"rate NaN hold"); }

    // qd_severity: tiers + NaN->crit
    { float qd[3]={1,2,3}; CHK(js_qd_severity(qd,3,10,20)==0,"qd ok"); }
    { float qd[3]={1,15,3}; CHK(js_qd_severity(qd,3,10,20)==1,"qd warn"); }
    { float qd[3]={1,-25,3}; CHK(js_qd_severity(qd,3,10,20)==2,"qd crit"); }
    { float nan=std::numeric_limits<float>::quiet_NaN(); float qd[2]={1,nan};
      CHK(js_qd_severity(qd,2,10,20)==2,"qd NaN->crit"); }

    if (fail) { std::printf("[test_joint_safety] %d FAIL\n", fail); return 1; }
    std::printf("[test_joint_safety] ALL PASS\n"); return 0;
}
```

- [ ] **Step 2: 실패 확인**

Run: `cd deploy/robots/g1/tests && g++ -std=c++17 -I../include -O2 test_joint_safety.cpp -o /tmp/tjs`
Expected: FAIL — `fatal error: JointSafety.h: No such file or directory`

- [ ] **Step 3: 구현** — `deploy/robots/g1/include/JointSafety.h`

```cpp
#pragma once
// JointSafety.h — 텔레옵 출력 경로용 joint-space 안전 순수함수. self-contained(단위테스트 가능).
// 철학: 한계는 기계한계/in-distribution 밖 → 정상 동작 불변, OOD/발산만 방어. NaN-safe.
#include <algorithm>
#include <cmath>
#include <cstddef>

// 위치 clamp: isfinite면 [lo,hi]로, 아니면 그대로(앞단 NaN 가드가 처리).
inline void js_clamp_position(float* q, const float* lo, const float* hi, int n) {
    for (int i = 0; i < n; ++i)
        if (std::isfinite(q[i])) q[i] = std::clamp(q[i], lo[i], hi[i]);
}

// 속도 rate-limit: per-tick 이동을 ±max_step로 캡. NaN이면 이전값 hold. q_prev를 최종값으로 갱신.
inline void js_rate_limit(float* q, float* q_prev, const float* max_step, int n) {
    for (int i = 0; i < n; ++i) {
        float target = std::isfinite(q[i]) ? q[i] : q_prev[i];
        float d = std::clamp(target - q_prev[i], -max_step[i], max_step[i]);
        q[i] = q_prev[i] + d;
        q_prev[i] = q[i];
    }
}

// 측정 qd 심각도: 0=정상, 1=warn(>warn), 2=crit(>crit or NaN).
inline int js_qd_severity(const float* qd, int n, float warn, float crit) {
    float m = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(qd[i])) return 2;
        float a = std::fabs(qd[i]);
        if (a > m) m = a;
    }
    if (m > crit) return 2;
    if (m > warn) return 1;
    return 0;
}
```

- [ ] **Step 4: 통과 확인**

Run: `cd deploy/robots/g1/tests && g++ -std=c++17 -I../include -O2 test_joint_safety.cpp -o /tmp/tjs && /tmp/tjs`
Expected: PASS — `[test_joint_safety] ALL PASS` (종료코드 0)

- [ ] **Step 5: 커밋**

```bash
git add deploy/robots/g1/include/JointSafety.h deploy/robots/g1/tests/test_joint_safety.cpp
git commit -m "safety: JointSafety.h(위치clamp/속도rate-limit/qd-severity 순수함수) + 단위테스트"
```

---

## Task 2: Config 배선 + Layer 1 위치 clamp

**Files:**
- Modify: `deploy/robots/g1/config/policy/mimic_masked/gmt_multihead_v0/params/deploy.yaml`
- Modify: `deploy/robots/g1/include/State_Mimic.h`
- Modify: `deploy/robots/g1/src/State_Mimic.cpp`

**Interfaces:**
- Consumes: `js_clamp_position`(Task 1).
- Produces: State_Mimic 멤버 `js_pos_lo_[29]`, `js_pos_hi_[29]`, `bool js_enable_pos_clamp_`(+ 이후 태스크가 채울 rate/qd 필드). deploy.yaml `safety:` 블록.

- [ ] **Step 1: deploy.yaml에 action.clip(기계한계) + safety 블록 추가**

`JointPositionAction.clip: null` → 기계한계 29쌍으로 교체(Global Constraints의 값). 그리고 파일에 `safety:` 블록 추가:
```yaml
# ── deploy 안전층 (State_Mimic이 로드; 없으면/이상하면 전부 비활성). 값은 sim 튜닝. ──
safety:
  enable_pos_clamp: false      # L1-2차 C++ 최종 clamp (action.clip이 L1-1차)
  enable_rate_limit: false     # L2 (Task 3)
  enable_qd_guard: false        # L3 (Task 4)
  # 위치 한계(기계, deploy 순서 29). action.clip과 동일.
  pos_min: [-2.5307,-0.5236,-2.7576,-0.087267,-0.87267,-0.2618,-2.5307,-2.9671,-2.7576,-0.087267,-0.87267,-0.2618,-2.618,-0.52,-0.52,-3.0892,-1.5882,-2.618,-1.0472,-1.97222,-1.61443,-1.61443,-3.0892,-2.2515,-2.618,-1.0472,-1.97222,-1.61443,-1.61443]
  pos_max: [ 2.8798, 2.9671, 2.7576, 2.8798, 0.5236, 0.2618, 2.8798, 0.5236, 2.7576, 2.8798, 0.5236, 0.2618, 2.618, 0.52, 0.52, 2.6704, 2.2515, 2.618, 2.0944, 1.97222, 1.61443, 1.61443, 2.6704, 1.5882, 2.618, 2.0944, 1.97222, 1.61443, 1.61443]
  # 속도 rate-limit (Task 3)·qd-guard (Task 4) 기본값 — clearly-high, sim에서 p99 기반 튜닝.
  vel_max: 20.0                 # rad/s (scalar; per-joint 확장 가능)
  qd_warn: 15.0                 # rad/s
  qd_crit: 25.0                 # rad/s
  over_ticks: 5                 # 연속 초과 틱(0.1s@50Hz)
  recover_ticks: 25             # 회복 틱(0.5s@50Hz)
```

- [ ] **Step 2: State_Mimic.h에 멤버 추가**

`State_Mimic` 클래스 private에:
```cpp
    // ── deploy 안전층 config/state (JointSafety.h) ──
    bool  js_enable_pos_clamp_ = false;
    std::array<float,29> js_pos_lo_{}, js_pos_hi_{};
```
(rate/qd 필드는 Task 3/4에서 추가.) 상단에 `#include "JointSafety.h"`, `#include <array>`.

- [ ] **Step 3: 생성자에서 safety 로드 (fail-safe)**

State_Mimic.cpp 생성자의 deploy.yaml 로드부(`YAML::LoadFile(policy_dir/"params"/"deploy.yaml")`, ~:434)를 지역 변수로 잡아 `safety`를 파싱. 예:
```cpp
    auto dcfg = YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string());
    // ... 기존: dcfg를 manager 생성에 사용 ...
    load_safety_cfg(dcfg["safety"]);   // 아래 헬퍼
```
헬퍼(클래스 메서드, fail-safe — 없거나 길이≠29면 비활성):
```cpp
void State_Mimic::load_safety_cfg(const YAML::Node& s) {
    js_enable_pos_clamp_ = false;
    if (!s || !s.IsMap()) { spdlog::warn("[safety] no safety block -> disabled"); return; }
    try {
        auto lo = s["pos_min"].as<std::vector<float>>();
        auto hi = s["pos_max"].as<std::vector<float>>();
        if (lo.size()!=29 || hi.size()!=29) { spdlog::warn("[safety] pos_min/max len!=29 -> clamp disabled"); return; }
        for (int i=0;i<29;++i){ js_pos_lo_[i]=lo[i]; js_pos_hi_[i]=hi[i]; }
        js_enable_pos_clamp_ = s["enable_pos_clamp"] && s["enable_pos_clamp"].as<bool>();
        spdlog::info("[safety] pos_clamp {} (mechanical limits loaded)", js_enable_pos_clamp_?"ON":"OFF");
    } catch (const std::exception& e) {
        js_enable_pos_clamp_ = false;
        spdlog::warn("[safety] parse error -> clamp disabled: {}", e.what());
    }
}
```
(선언을 State_Mimic.h에 추가: `void load_safety_cfg(const YAML::Node&);`)

- [ ] **Step 4: run() 출력 경로에 L1-2차 최종 clamp (gated)**

State_Mimic.cpp `run()`에서 `apply_switch_blend` 이후, `motor_cmd.q()` 쓰기 루프 **직전**에:
```cpp
    if (js_enable_pos_clamp_)
        js_clamp_position(action.data(), js_pos_lo_.data(), js_pos_hi_.data(),
                          static_cast<int>(action.size()));
```
기존 쓰기 루프는 그대로. (action.clip은 process_actions에서 이미 1차 적용 — 항상.)

- [ ] **Step 5: 빌드 + 비활성 no-op 확인**

Run: `cd /home/piene/unitree_rl_mjlab/deploy/robots/g1/build && cmake .. >/dev/null 2>&1; make -j4 2>&1 | tail -8`
Expected: `g1_ctrl` 링크 OK. `test_masked_loco_controller`(기존 golden) 여전히 통과:
`cd ../tests && g++ -std=c++17 -I../include -O2 test_masked_loco_controller.cpp -o /tmp/tml && /tmp/tml` → 기존 PASS 유지.
(enable_pos_clamp=false라 run() clamp는 no-op; action.clip=기계한계라 in-distribution 무변화.)

- [ ] **Step 6: 커밋**

```bash
git add deploy/robots/g1/config/policy/mimic_masked/gmt_multihead_v0/params/deploy.yaml deploy/robots/g1/include/State_Mimic.h deploy/robots/g1/src/State_Mimic.cpp
git commit -m "safety(L1): 위치 clamp — action.clip 기계한계 + safety블록 fail-safe 로드 + C++ 최종 clamp(gated, 기본 off)"
```

> **사용자 sim 검증(코드 후)**: sim2sim에서 `enable_pos_clamp: true`로 보행 → mode1/2/3에서 걸음걸이가 off일 때와 동일한지(육안+로그). 이상 없으면 유지.

---

## Task 3: Layer 2 속도 rate-limit

**Files:**
- Modify: `deploy/robots/g1/include/State_Mimic.h`, `deploy/robots/g1/src/State_Mimic.cpp`

**Interfaces:**
- Consumes: `js_rate_limit`(Task 1), safety cfg(Task 2).
- Produces: 멤버 `bool js_enable_rate_limit_`, `std::array<float,29> js_max_step_`, `std::array<float,29> js_q_prev_`, `bool js_q_prev_valid_`.

- [ ] **Step 1: State_Mimic.h 멤버 추가**
```cpp
    bool js_enable_rate_limit_ = false;
    std::array<float,29> js_max_step_{};   // = vel_max * dt(0.02)
    std::array<float,29> js_q_prev_{};
    bool js_q_prev_valid_ = false;
```

- [ ] **Step 2: load_safety_cfg에 vel_max 파싱 추가 (fail-safe)**
`load_safety_cfg` 안, pos 로드 뒤에:
```cpp
    js_enable_rate_limit_ = false;
    try {
        float vmax = s["vel_max"] ? s["vel_max"].as<float>() : 0.0f;
        const float dt = 0.02f;                       // 50Hz 제어
        if (vmax > 0.0f) {
            for (int i=0;i<29;++i) js_max_step_[i] = vmax * dt;
            js_enable_rate_limit_ = s["enable_rate_limit"] && s["enable_rate_limit"].as<bool>();
        }
        spdlog::info("[safety] rate_limit {} (vel_max={} rad/s)", js_enable_rate_limit_?"ON":"OFF", vmax);
    } catch (const std::exception& e) { js_enable_rate_limit_ = false; spdlog::warn("[safety] rate parse err: {}", e.what()); }
```

- [ ] **Step 3: enter()에서 q_prev init (측정 pose)**
State_Mimic::enter()에 (측정 joint_pos 접근 가능한 지점, 예: leg_from init 근처):
```cpp
    for (int i=0;i<29;++i) js_q_prev_[i] = env->robot->data.joint_pos[i];
    js_q_prev_valid_ = true;
```

- [ ] **Step 4: run()에 L2 rate-limit (L1 clamp 이후, 쓰기 이전, gated)**
Task 2의 clamp 블록 **이후**:
```cpp
    if (js_enable_rate_limit_ && js_q_prev_valid_)
        js_rate_limit(action.data(), js_q_prev_.data(), js_max_step_.data(),
                      static_cast<int>(action.size()));
    else
        for (int i=0;i<(int)action.size();++i) js_q_prev_[i] = action[i];  // 비활성 시에도 prev 추적(켤 때 점프 방지)
```

- [ ] **Step 5: 빌드 + golden 유지 확인**
Run: `cd deploy/robots/g1/build && make -j4 2>&1 | tail -5` → 링크 OK. `test_joint_safety`/`test_masked_loco_controller` 재실행 PASS.
(enable_rate_limit=false → run()은 prev만 추적, action 무변경.)

- [ ] **Step 6: 커밋**
```bash
git add deploy/robots/g1/include/State_Mimic.h deploy/robots/g1/src/State_Mimic.cpp
git commit -m "safety(L2): 관절 속도 rate-limit(gated, 기본 off) + enter() q_prev 측정pose init"
```
> **사용자 sim 검증**: `enable_rate_limit: true`(vel_max 넉넉히) → 보행 lag/이상 없는지. vel_max를 Task 5 p99로 조인다.

---

## Task 4: Layer 3 측정 qd 폭주 → mode1(warn)/Passive(crit), 둘 다 래치

> ⚠ **배치 = policy_thread(50Hz)** — `g_cmd_mode`/`notify_mode_switch`가 policy_thread 소관이므로 run()(1kHz)에 두면
> 스레드 레이스 + blend 카운터 오염. policy_thread라 `over_ticks`는 @50Hz(5=0.1s)가 맞다(recover_ticks 불필요=둘 다 래치).
> **warn/crit 둘 다 래치**(사용자 결정): warn→mode1 강제(수동복귀: 조작자 mode1 명시 시 해제), crit→Passive(FSM 재진입 시 해제).

**Files:** Modify `deploy/robots/g1/include/State_Mimic.h`, `deploy/robots/g1/src/State_Mimic.cpp`, `deploy/.../deploy.yaml`(recover_ticks 제거)

**Interfaces:**
- Consumes: `js_qd_severity`(Task 1), safety cfg.
- Produces: 멤버 `js_enable_qd_guard_`, `js_qd_warn_/js_qd_crit_`, `js_over_ticks_`, 카운터 `js_warn_run_/js_crit_run_`, 래치 `js_qd_warn_latched_`(bool, policy_thread 전용) + `js_qd_crit_latched_`(`std::atomic<bool>`, policy_thread set / CtrlFSM registered_check read).

- [ ] **Step 1: State_Mimic.h 멤버** (상단 `#include <atomic>`)
```cpp
    bool  js_enable_qd_guard_ = false;
    float js_qd_warn_ = 0.f, js_qd_crit_ = 0.f;
    int   js_over_ticks_ = 5;               // @50Hz(policy_thread) => 0.1s sustained
    int   js_warn_run_ = 0, js_crit_run_ = 0;
    bool  js_qd_warn_latched_ = false;      // policy_thread 내부 전용
    std::atomic<bool> js_qd_crit_latched_{false};  // policy_thread set, registered_check(1kHz) read
```

- [ ] **Step 2: load_safety_cfg에 qd 파싱 (fail-safe)** — pos/vel 파싱과 별개 try/catch:
```cpp
    js_enable_qd_guard_ = false;
    try {
        js_qd_warn_ = s["qd_warn"] ? s["qd_warn"].as<float>() : 0.f;
        js_qd_crit_ = s["qd_crit"] ? s["qd_crit"].as<float>() : 0.f;
        js_over_ticks_ = s["over_ticks"] ? s["over_ticks"].as<int>() : 5;
        if (js_over_ticks_ < 1) js_over_ticks_ = 1;
        if (js_qd_warn_ > 0.f && js_qd_crit_ > js_qd_warn_)   // warn>0 이고 crit>warn 이어야 무장
            js_enable_qd_guard_ = s["enable_qd_guard"] && s["enable_qd_guard"].as<bool>();
        spdlog::info("[safety] qd_guard {} (warn={} crit={} rad/s, over_ticks={})",
                     js_enable_qd_guard_?"ON":"OFF", js_qd_warn_, js_qd_crit_, js_over_ticks_);
    } catch (const std::exception& e) { js_enable_qd_guard_ = false; spdlog::warn("[safety] qd parse err: {}", e.what()); }
```
deploy.yaml `safety:`에서 `recover_ticks` 줄 제거(둘 다 래치라 미사용). `over_ticks: 5`(주석 "0.1s@50Hz") 유지.

- [ ] **Step 3: policy_thread에 qd 검사 삽입** — `g_poll_vr()`(현 ~:544) **직후**, 기존 `if (g_cmd_mode != g_prev_cmd_mode)`(현 ~:547) **직전**에:
```cpp
            // ── L3: 측정 qd 폭주 감지 (policy_thread 50Hz — g_cmd_mode/notify_mode_switch 같은 스레드) ──
            if (js_enable_qd_guard_) {
                const int reqd = g_req_mode;   // ★조작자 실제 요청 모드(별도 전역; g_poll_*만 세팅, 가드는 안 건드림). g_cmd_mode를 되읽으면 가드 자기 force값(1)에 자가해제됨 → g_req_mode 필수.
                int sev = js_qd_severity(env->robot->data.joint_vel.data(),
                                         (int)env->robot->data.joint_vel.size(), js_qd_warn_, js_qd_crit_);
                if (sev >= 2)      { js_warn_run_ = 0; if (++js_crit_run_ >= js_over_ticks_) js_qd_crit_latched_ = true; }
                else if (sev == 1) { js_crit_run_ = 0; if (++js_warn_run_ >= js_over_ticks_) js_qd_warn_latched_ = true; }
                else               { js_warn_run_ = 0; js_crit_run_ = 0; }
                // warn 수동복귀: 조작자가 mode1(X/'1')을 명시하면 해제(qd 아직 높으면 다음 sustained서 재래치).
                if (js_qd_warn_latched_ && reqd == 1) js_qd_warn_latched_ = false;
                if (js_qd_warn_latched_) g_cmd_mode = 1;   // g_poll_vr 뒤에 덮어써 mode1 유지(soft)
                // crit은 아래 registered_check가 Passive로 전이시킴(여기선 latch만).
            }
```
(이렇게 하면 g_cmd_mode 변화는 기존 :547 `notify_mode_switch` 흐름이 부드럽게 처리 — 수동 notify 호출 없음.)

- [ ] **Step 4: crit → Passive registered_check** — 생성자 registered_checks(기존 bad_orientation 등록부 ~:450 근처):
```cpp
    registered_checks.emplace_back(
        [this]()->bool{ return js_enable_qd_guard_ && js_qd_crit_latched_.load(); },  // qd crit -> Passive
        FSMStringMap.right.at("Passive"));
```
enter()에서 래치·카운터 리셋(재진입 시 깨끗이): `js_qd_crit_latched_ = false; js_qd_warn_latched_ = false; js_warn_run_ = js_crit_run_ = 0;`

- [ ] **Step 5: 빌드 + 테스트 유지**
Run: `cd deploy/robots/g1/build && make -j4 2>&1 | tail -5` → OK. `test_joint_safety`/`test_masked_loco_controller` PASS. `enable_qd_guard` 여전히 false 확인.

- [ ] **Step 6: 커밋**
```bash
git add deploy/robots/g1/include/State_Mimic.h deploy/robots/g1/src/State_Mimic.cpp deploy/robots/g1/config/policy/mimic_masked/gmt_multihead_v0/params/deploy.yaml
git commit -m "safety(L3): 측정 qd 폭주 -> warn(mode1 래치,수동복귀)/crit(Passive 래치), policy_thread 50Hz, gated 기본 off"
```
> **사용자 sim 검증**: `enable_qd_guard: true` → 정상 보행 오발동 없는지(warn/crit ≫ 보행 qd) + 인위적 빠른동작/외란 시 warn→mode1(X로 복귀), crit→Passive(f로 재기립). Task 5로 임계 확정.
> **수동복귀 UX**: qd-warn으로 mode1 강제되면 → 조작자가 **X(mode1)** 눌러 acknowledge → 이후 Y/B로 mode2/3 재개.

---

## Task 5: in-distribution qd/q 측정 + 임계 튜닝 (사용자 sim 절차)

**목표**: 안전 한계가 정상 동작에 **안 닿음**을 증명하고 vel_max/qd_warn/qd_crit를 데이터로 확정(기존 base_vel cap을 p99에서 정한 방식).

- [ ] **Step 1: g1_ctrl에 max|qd| 로깅(임시 디버그, 커밋 안 함 or `--log-qd` 플래그)**
State_Mimic::run()에 (사용자 실행용) 주기적 로그: `max_i |joint_vel[i]|`, per-joint max 누적 → 1s마다 출력. 또는 sim2sim 로그를 파싱.

- [ ] **Step 2: sim2sim 정상 보행 rollout → max/p99 qd 수집**
`unitree_mujoco` + `g1_ctrl --network=lo` + 브릿지로 mode1/2/3 정상 보행·팔동작 몇 분 → per-joint max|qd|, p99 기록. q도 한계 안쪽인지 확인.

- [ ] **Step 3: 임계 확정 → deploy.yaml 갱신**
`vel_max = ceil(p99 × 1.5~2)`, `qd_warn ≈ vel_max`, `qd_crit ≈ 1.5×vel_max`(sim 판단). pos는 기계한계 유지(margin 필요시 안쪽 δ). deploy.yaml `safety:` 갱신 + 커밋(값만).

> 이 태스크는 **하드웨어/sim 의존** — 코드 구현 태스크(1~4) 완료 후 사용자가 수행. 자동화 불가 부분.

---

## Task 6: 문서 + 증분 enable 절차

**Files:** Modify `rules/SYSTEM_OVERVIEW.md`(안전 섹션 추가) + `deploy/robots/g1/teleop/README.md`

- [ ] **Step 1: 3층 구조·config·enable 순서 문서화**
- 3층(L1 pos clamp / L2 rate-limit / L3 qd-guard) 설명 + deploy.yaml `safety:` 필드.
- **증분 enable 절차(안전)**: (1) 코드 배포=전부 off, action.clip만 기계한계(no-op). (2) sim2sim에서 L1 on→보행 확인. (3) L2 on(vel_max 넉넉)→확인→Task5로 조임. (4) L3 on→오발동 없음+외란 시 폴백 확인. (5) 실로봇: mode1부터, 사람 E-stop 대기.
- fail-safe: config 이상→해당 층 자동 비활성.

- [ ] **Step 2: 커밋**
```bash
git add rules/SYSTEM_OVERVIEW.md deploy/robots/g1/teleop/README.md
git commit -m "docs(safety): 텔레옵 관절 안전 3층 + config + 증분 enable 절차"
```

---

## Self-Review (작성자 체크)

**1. Spec coverage**: L1 위치clamp(둘 다: action.clip+C++ final)→T2 ✓; L2 rate-limit→T3 ✓; L3 qd→mode1/Passive(tilt 제외)→T4 ✓; config-tunable+G1 기본값→T2/T5 ✓; 순수·테스트가능→T1 ✓; no-op default·fail-safe·증분→Global+각 T의 gating/검증 ✓; 문서→T6 ✓. 갭 없음.

**2. Placeholder scan**: 모든 코드 스텝에 실제 코드. 임계 기본값은 명시(20/15/25) + T5에서 데이터 확정. "적절히" 류 없음.

**3. Type consistency**: `js_clamp_position/js_rate_limit/js_qd_severity` 시그니처가 T1 정의=T2/3/4 호출 일치. 멤버명(`js_pos_lo_/hi_`, `js_max_step_`, `js_q_prev_`, `js_qd_warn_/crit_`, `g_qd_crit_latched`) T2→T3→T4 일관. deploy JOINT_ORDER 29 전 배열 동일 순서. 출력 순서: process_actions(clip) → blend → L1 clamp → L2 rate-limit → write, L3는 측정 qd 독립.

**안전 우선 설계 반영**: 커밋 기본 no-op, fail-safe 파싱(→비활성, 0-clamp/throw 금지), last_action parity(출력 전용 필터), NaN-safe, 순수함수 단위테스트, golden 유지, 증분 롤아웃, 실로봇 마지막.
</content>
