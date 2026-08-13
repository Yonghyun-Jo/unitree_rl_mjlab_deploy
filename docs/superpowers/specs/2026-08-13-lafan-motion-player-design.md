# LAFAN Motion Player — 실로봇 모션 참조 재생 구조 설계

- 날짜: 2026-08-13
- repo: `unitree_rl_mjlab` / branch: `smooth_mode_switch`
- 상태: 설계 승인 대기 → 구현 계획(writing-plans)으로 이관 예정

---

## 1. 목적

학습에 쓴 LAFAN 모션 클립을, 배포된 `Mimic_Masked` 정책의 **모션 참조(motion ref)** 로 주입해
sim2sim과 **실로봇**에서 재생한다. 클립 선택·구간 지정·재생/중단을 SSH 터미널에서 한다.

구조는 앞으로의 확장을 흡수해야 한다.

- 리타겟팅 세대 교체: `motions/` → `COLMO/` → `COLMOv2/` → 그 다음
- 정책 슬롯 증가: `gmt_multihead_v0` / `cop_slip` / `cwc` / `cwc_scratch` / `v2_mode3_steps6` / …
- 모드 증가: 현재 1~5, 앞으로 더

---

## 2. 현재 구조에서 확인된 사실 (설계 근거)

전부 코드에서 확인했다. 추측이 아니다.

### 2.1 모드별 참조 마스킹

| mode | 상체 q_ref | 하체 q_ref | `masked_root_ori_b` | `base_vel` | 참조 출처 |
|---|---|---|---|---|---|
| 1 | **0** | **0** | **0** | 살아있음 | — |
| 2 | 클립/VR | 0 | VR pelvis | 살아있음 | **VR 버퍼** |
| 3 | 클립/VR | 클립/VR | VR pelvis | **0으로 강제** | **VR 버퍼** |
| 4 | 클립 | 클립 | 클립 pelvis | 0으로 강제 | `config.yaml: motion_file` |
| 5 | 클립 | 클립 | 클립 pelvis | 0으로 강제 | `config.yaml: motion_file_light` |

근거: `src/State_Mimic.cpp:17-32`(마스크 정의), `:279-318`(`masked_joint_command`),
`:344-367`(`masked_root_ori_b`), `include/MaskedLocoController.h:124`(`cmd_mode>=3 → bv=0`).

**중요**: mode2/3은 클립을 **절대 읽지 않는다.** `joint_pos_vr()` / `joint_pos_clip()`이 분리돼 있고
mode2/3은 VR 버퍼 전용이다 (`include/State_Mimic.h:184-189`). VR이 없으면 standby(로봇 기본자세).

**mode1 재생은 원리적으로 불가능하다.** 참조가 상·하체 전부 0으로 마스킹되고 anchor ori도 0이라
정책이 참조를 볼 수 없다. mode1의 정책 입력은 `base_vel_command`와 `ref_foot_height`뿐이다.

### 2.2 참조 주입 통로 — 이미 뚫려 있음

`/dev/shm/g1_vr_ref` (magic `0x6702`, 276B packed):
`magic seq valid cmd_mode | base_vel[3] root_quat[4:wxyz] dof_pos[29] dof_vel[29]`

- 계약: `teleop/vr_shm.py` ↔ C++ `struct VrRef` (`src/State_Mimic.cpp:94-105`)
- 소비: `g_poll_vr()` (`:110-137`) — 매 정책 스텝(50 Hz)
- 기존 생산자: `teleop/vr_teleop_bridge.py`(PICO+GMR), `teleop/vr_replay.py`(npz 재생 stub)

**패킷은 이미 mode-agnostic이다.** 생산자는 항상 29-DOF 전체 참조를 보내고, 어디까지 정책에
닿을지는 `cmd_mode` 필드 하나로 결정되며 마스킹은 C++가 한다. → 모드별 재생기를 따로 만들 필요가 없다.

### 2.3 확장 병목 (지금 고치지 않음, 명시만)

`g_poll_vr`가 `if (v.cmd_mode >= 1 && v.cmd_mode <= 3)` 로 하드코딩돼 있다
(`src/State_Mimic.cpp:129`). **VR 채널로는 4 이상의 모드를 보낼 수 없다.** 모드가 늘어날 때
반드시 넓혀야 하는 한 줄.

### 2.4 정책 슬롯 ↔ 학습 클립 연결고리

`config/policy/<family>/<slot>/ONNX_META.json` 이 이미 필요한 메타를 갖고 있다.

```json
{ "manifest": "motions/g1_flow_specialists_v2.yaml",
  "mode1_ckpt": "none", "mode2_ckpt": "none", "mode3_ckpt": ".../model_29999.pt",
  "obs_dim": "1640", "deployable_on_orin_nx": false }
```

매니페스트 엔트리는 `file:`(클립 경로) + `modes:`(그 클립이 유효한 모드) 를 갖는다.

| 슬롯 | 매니페스트 | 클립 | 유효 모드 | 실기 |
|---|---|---|---|---|
| `gmt_multihead_cwc_scratch` (현재 활성) | `g1_flow_specialists.yaml` | 24 (구 `motions/`) | 1,2,3 | ✅ |
| `v2_mode3_steps6` | `g1_flow_specialists_v2.yaml` | 29 (COLMOv2 11 + COLMO 18) | **3만** | ❌ 추론 1회 1052 MB |
| `gmt_multihead_v0` | (ONNX_META 없음) | 24 | 1,2,3 | ✅ |

**A 정책용 클립을 B 정책에 물리면 OOD → 실로봇에선 낙상.** 이 대응표가 플레이어의 중심 자산이다.

### 2.5 COLMOv2

**리타겟팅 데이터셋의 v2이지 입력 형식 변경이 아니다.** npz 스키마 동일
(`joint_pos[T,29]`, `joint_vel[T,29]`, `body_pos_w[T,30,3]`, `body_quat_w[T,30,4]`,
`body_lin_vel_w`, `body_ang_vel_w`, `fps`). 위치는 `<mjlab_g1_motion>/COLMOv2/<cat>/<name>.npz`,
77파일 / 11카테고리. 재생기 입장에선 **경로만 다르다** → 매니페스트가 흡수한다.

### 2.6 클립 실측 (설계에 직접 영향)

전 24클립 측정 결과:

- **길이 6,574~15,285 프레임 = 131~306초.** LAFAN 원본 시퀀스 통째다.
  실로봇에서 통짜 재생은 비현실적 → **구간(span) 개념 필수.**
  (이 repo도 이미 배포용으로 손수 자른 `g1_dance1_subject3_upper16s.npz`를 쓴다.)
- **standby에 가까운 프레임이 없다.** 전 클립 최소 `max|q − default_joint_pos|` = 0.40~0.64 rad.
  `default_joint_pos`가 KNEES_BENT 키프레임(무릎 0.669 rad)이라 LAFAN 기립자세와 애초에 다르다.
  → "안전 진입 프레임 자동 탐색"은 이득이 없다. **어느 시작점이든 램프-인이 똑같이 필요하다.**
- pelvis Z₀ = 0.78~0.84 → 전 클립이 서 있는 상태에서 시작한다.

### 2.7 안전 관련 기존 자산

| 항목 | 위치 | 동작 |
|---|---|---|
| VR stale 워치독 | `src/State_Mimic.cpp:121-125` | `seq` 0.5 s 정지 → `clear_vr()` |
| `clear_vr()` | `include/State_Mimic.h:156-163` | **q_ref를 standby로 계단 스냅** ⚠ |
| 모드전환 재앵커 + 크로스페이드 | `src/State_Mimic.cpp:596-612`, `config.yaml: switch_blend_steps: 50` | **모드가 바뀔 때만** 발동 |
| E-stop 하트비트 | `teleop/estop_shm.py` ↔ `include/FSM/EstopChannel.h` | flag≠0 또는 writer 정지 → 강제 Passive |
| 관절 안전 3층 | `deploy.yaml: safety:` | 현재 전부 `false` |
| 낙상 감지 | `bad_orientation` → Passive | 상시 |

여기서 두 가지가 설계를 강제한다.

1. **`clear_vr()`이 계단 스냅**이므로, 재생 종료 시 플레이어가 **직접 램프-아웃** 해야 한다.
2. **재앵커·크로스페이드가 모드 전환에만 걸리므로**, mode3을 유지한 채 클립만 갈아끼우면
   heading이 어긋난 채 시작한다 → **매 재생을 `mode1 → 재생모드 → mode1` 로 왕복**시킨다.

---

## 3. 요구사항

**해야 하는 것**

- R1. 현재 배포된 정책이 학습한 클립만 목록에 보인다.
- R2. 클립 + 임의 구간 `[t_start, t_end]` + 속도로 재생. 원본 npz는 수정하지 않는다.
- R3. 자주 쓰는 구간은 프리셋으로 저장해 재사용한다.
- R4. mode2와 mode3 재생을 모두 지원한다. mode1은 이유와 함께 거부한다.
- R5. 진입/이탈이 항상 램프를 거친다. 어떤 경로로도 관절 목표가 계단으로 튀지 않는다.
- R6. 사람이 언제든 한 키로 중단할 수 있다.
- R7. 배포 C++를 수정하지 않는다.
- R8. 새 리타겟 세대·새 정책 슬롯·새 클립은 플레이어 코드 수정 없이 흡수된다.

**하지 않는 것 (non-goals)**

- N1. mode1 관절 재생 (구조적 불가 — §2.1).
- N2. mode 4 이상을 VR 채널로 보내기 (§2.3 — C++ 한 줄이 선행돼야 함).
- N3. 클립 트리밍/편집/리타겟팅. 원본은 읽기 전용.
- N4. 배포 C++의 `clear_vr()` 램프화 (접근안 B — sim2sim에서 필요성을 확인한 뒤 별도 승인).
- N5. 클립 난이도 게이팅 (사용자 결정: 24개 전부 평등하게 노출). 대신 확인 프롬프트를 둔다.

---

## 4. 아키텍처

**설계 원칙: 플레이어는 LAFAN을 모른다.** 클립 목록을 갖지 않고,
"지금 g1_ctrl에 로드된 정책이 학습한 목록"을 매번 읽어온다.

```
┌── ① Resolver ────────────────────────────────────────────┐
│  아는 것: "지금 무슨 정책인가"                              │
│  config/config.yaml → FSM.Mimic_Masked.policy_dir         │
│    → <slot>/ONNX_META.json  (manifest, modeN_ckpt, 실기여부) │
│      → manifest YAML        (클립 경로 + 클립별 modes)      │
│  산출: PolicyContext { slot, clips[], valid_modes[], warn } │
│  ⇒ COLMOv2·COLMOv3·새 슬롯이 전부 여기서 흡수된다             │
└──────────────────────────────────────────────────────────┘
                          │ clips[]
┌── ② Playlist ────────────────────────────────────────────┐
│  아는 것: "무엇을 얼마나"                                   │
│  PlayItem { clip, span[2], mode, base_vel, speed }        │
│  프리셋: presets.yaml   /   임의구간: CLI "1 40 15 x0.5"    │
│  ⇒ 원본 npz는 건드리지 않는다 (131~306초 원본 그대로)         │
└──────────────────────────────────────────────────────────┘
                          │ PlayItem
┌── ③ Publisher ───────────────────────────────────────────┐
│  아는 것: "어떻게 안전히"                                   │
│  상태기계 + 50 Hz 절대시각 송출 → vr_shm.write()           │
│  안전 로직은 전부 여기에만 있다                              │
│  ⇒ 참조 소스가 뭐로 바뀌든 이 층은 바뀌지 않는다              │
└──────────────────────────────────────────────────────────┘
                          │ /dev/shm/g1_vr_ref (0x6702, 기존 계약)
                     [ g1_ctrl ]
```

각 겹이 하나만 안다. 경계는 순수 데이터 구조(`PolicyContext`, `PlayItem`)라 겹끼리
독립 테스트된다.

---

## 5. ① Resolver

**입력**: `deploy/robots/g1/config/config.yaml` 의 `FSM.Mimic_Masked.policy_dir`
**출력**:

```python
PolicyContext:
    slot: str                 # "gmt_multihead_cwc_scratch"
    manifest_path: Path
    clips: list[ClipInfo]     # name, path, modes[], n_frames, fps, duration
    valid_modes: set[int]     # ONNX_META 의 modeN_ckpt != "none" 인 것
    deployable: bool | None   # deployable_on_orin_nx
    warnings: list[str]
```

**해석 규칙**

1. `ONNX_META.json` 이 있으면 그것이 진실이다. `manifest` 로 매니페스트를 찾는다.
2. `modeN_ckpt == "none"` → mode N 비활성. (`v2_mode3_steps6` 이 실제로 mode1/2가 `"none"`)
3. `deployable_on_orin_nx == false` → 목록 상단에 실기 배포 불가 경고.
4. 매니페스트 엔트리의 `modes:` 가 클립별 유효 모드.
5. **`ONNX_META.json` 이 없는 구 슬롯**(`gmt_multihead_v0`)은 `presets.yaml` 의
   `slot_overrides:` 에서 매니페스트 경로를 수동 지정한다. 그것도 없으면
   **클립 0개 + "이 슬롯은 매니페스트 미상 — 재생 불가"** 로 안전 실패한다.
   (추측으로 24클립을 가정하지 않는다.)
6. 매니페스트의 `root_path` + 상대경로 또는 절대경로 모두 해석한다.
   존재하지 않는 파일은 경고와 함께 목록에서 제외한다.
7. `fps` 는 npz의 `fps` 키에서 읽는다. **50 Hz 하드코딩 금지.**
   없으면 50으로 보되 경고를 남긴다.

---

## 6. ② Playlist

```yaml
# deploy/robots/g1/teleop/motion_player/presets.yaml
slot_overrides:
  gmt_multihead_v0: motions/g1_flow_specialists.yaml   # ONNX_META 없는 구 슬롯 구제

presets:
  - clip: walk1_subject1
    label: "직진 보행"
    span: [12.0, 27.0]
    mode: 2
    base_vel: clip
    speed: 1.0
```

| 필드 | 값 | 의미 |
|---|---|---|
| `clip` | 매니페스트의 클립 이름 | 경로는 Resolver가 해석 |
| `span` | `[t_start, t_end]` 초 | 원본 시간축 기준 |
| `mode` | `2` \| `3` | `1`은 거부(§2.1 이유 안내) |
| `base_vel` | `clip` \| `zero` \| `manual` | **mode2에서만 의미.** mode3은 C++가 0으로 덮음 |
| `speed` | float | **클립 시간축만** 배속. 램프 구간 길이는 벽시계 고정(속도와 무관) |

`base_vel: manual` = 재생 내내 CLI/프리셋이 준 고정 `[vx, vy, wz]` 를 그대로 보낸다
(조이스틱과 합산하지 않는다 — 조이스틱은 `g_joystick_base_vel` 에서 별도로 더해진다).

**`base_vel: clip`** — npz의 `body_lin_vel_w[:, 0]`(pelvis)와 `body_ang_vel_w[:, 0]`에서
yaw-local 속도를 뽑아 넣는다:

```
vx, vy = R_yaw(pelvis_quat)ᵀ · lin_vel_w  의 x, y
wz     = ang_vel_w[2]
```

학습 base_vel 분포가 "clip pelvis velocity"라고 C++ 주석(`src/State_Mimic.cpp:187-188`)에
명시돼 있어 in-distribution이다. 이 값으로 **mode2 재생 = 클립이 걷는 대로 걸으면서 상체는
클립을 재생**이 된다. 배포 캡(`KB_MAXVX 3.0 / KB_MAXVY 1.5 / KB_MAXW 2.0`)으로 clamp한다.

**CLI 임의 구간 문법**

```
1a              프리셋 a
1 40 15         클립 #1, 40초부터 15초
1 40 15 x0.5    0.5배속
1 40 15 x0.5 m3 mode3 강제 (기본은 프리셋/기본값)
```

---

## 7. ③ Publisher

### 7.1 상태기계

```
IDLE ─Enter─► ARM ─► RAMP_IN ─► PLAY ─► RAMP_OUT ─► RELEASE ─► IDLE
                                 │                     ▲
                          Space/Ctrl-C ────────────────┘  (T_out 0.8 s 단축)
```

| 단계 | 송출 내용 | 근거 |
|---|---|---|
| **ARM** | 송출 없음 | 프리플라이트 + 확인 프롬프트 |
| **RAMP_IN** | `cmd_mode=<2\|3>, valid=1`<br>`q_ref = smoothstep(s)·(clip[f₀] − standby) + standby`<br>`qd_ref = s · clip_qd[f₀]`<br>`root_quat = slerp(identity, clip_quat[f₀], s)` | 첫 패킷이 mode1→2/3 전환을 유발 → C++ 재앵커 + 1.0 s 크로스페이드 + leg 램프가 동시 발동 |
| **PLAY** | 클립 프레임 그대로<br>`qd_ref = clip_qd · speed`<br>`base_vel` = 정책에 따라 | **속도를 늦추면 qd도 같이 늦춰야 한다.** 안 그러면 위치는 느린데 속도 피드포워드만 빨라 참조가 모순된다 |
| **RAMP_OUT** | `clip[f_end] → standby`, `qd → 0` | `clear_vr()`이 계단 스냅이라 필수 |
| **RELEASE** | `cmd_mode=1, valid=1` 패킷 → 0.5 s 유지 → `valid=0` → 파일 삭제 | 모드 전환을 일으켜야 arm-blend + 크로스페이드가 걸림 |

- `standby` = `deploy.yaml` 의 `default_joint_pos` (C++ `set_standby()` 가 잡는 것과 동일,
  `include/State_Mimic.h:149-155`). **`deploy.yaml`에서 읽는다. 하드코딩 금지.**
- **T_in 자동 산정**: `max|q[f₀] − standby| ÷ 0.35 rad/s`, 최소 1.5 s.
  실측 0.40~0.64 rad → 대략 1.5~1.9 s. C++ 크로스페이드(1.0 s)보다 항상 길다.
- **타이밍**: `time.sleep(dt)` 누적이 아니라 `perf_counter` **절대시각 데드라인**.
  밀리면 프레임을 떨어뜨리고 시간축을 지킨다 (참조가 느려지는 것보다 낫다).
- `seq` 는 매 송출 증가 (C++ liveness 판정 기준).

### 7.2 프리플라이트 (ARM 단계)

전부 통과해야 재생한다. 하나라도 실패하면 이유를 출력하고 IDLE로 돌아간다.

1. 클립이 현재 정책 매니페스트에 있는가
2. 요청 모드가 클립의 `modes:` 와 슬롯 `valid_modes` 양쪽에 있는가
3. `span` 이 `[0, duration]` 안이고 `t_start < t_end` 인가
4. 해당 구간에 NaN/Inf가 없는가
5. 구간의 `q` 가 `deploy.yaml` 의 `pos_min/pos_max` 안인가
6. `max|q[f₀] − standby|` 계산 → T_in 산정
7. mode1 요청이면 거부 + §2.1 이유 안내

### 7.3 확인 프롬프트

```
▶ walk1_subject1   40.0–55.0s (15.0s, ×0.50 → 벽시계 30.0s)   mode2 · base_vel=clip
  진입 자세차 0.52 rad (left_knee_joint) → 램프인 1.5s
  관절한계 위반 없음 · NaN 없음 · 매니페스트 ✓ mode2 ✓
  [Enter] 재생   [Space] 취소
```

24개를 평등하게 노출하되(사용자 결정), 오타로 jump/sprint를 거는 사고는 이 자리에서 막는다.

---

## 8. CLI

```
LAFAN Player — policy: gmt_multihead_cwc_scratch   (modes 1,2,3 · 실기 OK)
manifest: g1_flow_specialists.yaml  ·  24 clips

  #  clip                 length   modes   presets
  1  walk1_subject1       261.3s   1,2,3   [a] 직진보행 12–27s  [b] 회전 88–103s
  2  walk1_subject2       261.3s   1,2,3   —
 20  jumps1_subject1      244.4s     2,3   —

> 1a              프리셋 a 재생
> 1 40 15         40초부터 15초
> 1 40 15 x0.5    0.5배속
> l · q           목록 · 종료
```

재생 중:

```
▶ walk1_subject1 40.0–55.0s ×0.50 mode2   [████████░░░░] 18.2/30.0s   50.0 Hz
  Space=중단   x=E-stop(Passive)
```

---

## 9. 안전 계층

| 층 | 무엇 | 담당 | 상태 |
|---|---|---|---|
| 0 | 프리플라이트 (§7.2) | 플레이어 | **신규** |
| 1 | 램프인 / 램프아웃 | 플레이어 | **신규** |
| 2 | 사람 abort (Space / Ctrl-C / SIGTERM) | 플레이어 | **신규** |
| 3 | E-stop 키 → 강제 Passive | `estop_shm` + `fsm_estop_poll` | 기존 |
| 4 | 플레이어 사망 → 0.5 s 후 standby 복귀 | `g_poll_vr` stale | 기존 |
| 5 | 관절속도 폭주 → mode1 / Passive | `deploy.yaml: safety:` | 기존 (현재 off) |
| 6 | 낙상 → Passive | `bad_orientation` | 기존 |

**E-stop 하트비트는 기본 미무장** (`--arm-estop` 로 켬). 무장하면 플레이어 크래시 시 Passive(댐핑)로
로봇이 주저앉는데, 균형을 잡고 있던 상황에서는 4층(standby 복귀)이 더 부드럽다.
기존 브릿지 관례(`--transport local` 일 때만 자동 무장)와도 일치한다.
`x` 키는 **명시적 사람 판단**이므로 즉시 flag=1 → Passive.

SIGINT/SIGTERM 핸들러는 RAMP_OUT 경로를 탄다. SIGKILL/하드 크래시만 4층에 맡긴다.

---

## 10. 모드별 재생 의미 (사용자 관점)

| mode | 다리 | 상체 | 언제 쓰나 |
|---|---|---|---|
| **2** | 학습된 자율 보행 (base_vel 추종) | 클립 재생 | **실기 첫 시도.** 낙상 위험이 낮다. `base_vel: clip` 이면 클립이 걷는 대로 걸으면서 상체 재생 |
| **3** | 클립 재생 | 클립 재생 | **전신 재생.** base_vel은 C++가 0으로 덮으므로 다리 궤적이 곧 이동의 전부 |
| 1 | — | — | **재생 불가.** 참조가 마스킹돼 정책에 도달하지 않음 |

---

## 11. 확장 시나리오 — 무엇을 고치나

| 앞으로 생길 일 | 고칠 것 |
|---|---|
| COLMOv3 리타겟 | **없음.** 새 매니페스트가 새 경로를 가리키면 끝 |
| 새 정책 슬롯 | **없음.** `ONNX_META.json` 을 같이 넣으면 됨 |
| 클립 증가 (77 → 더) | **없음.** 매니페스트에 줄 추가 |
| 모드 6/7 신설 | 매니페스트 `modes:` + **C++ `State_Mimic.cpp:129` 한 줄** (§2.3) |
| obs 항목 자체 추가 | `vr_shm.py` + C++ `struct VrRef` — 여기만 진짜 계약 변경 |

---

## 12. 열린 항목 — 측정해서 정한다

**O1. mode2 장시간 재생의 yaw 드리프트.**
mode2는 다리가 `base_vel` 로 걷고 상체 참조는 클립 pelvis 자세다. 로봇 실제 yaw와 클립 yaw가
벌어지면 `masked_root_ori_b` 오차가 누적된다 (C++는 `init_quat` 을 모드 전환 시점에만 고정,
`src/State_Mimic.cpp:596-612`). `base_vel: clip` 이면 회전도 같이 따라가 대체로 상쇄되겠지만
보장은 아니다.
→ **sim2sim에서 60 s 이상 재생하며 yaw 오차 추이를 계측**한 뒤 대응을 정한다.
후보: (a) 그대로 둠 (b) 주기적 mode 왕복 재앵커 (c) 구간 길이 상한.
**지금 추측으로 정하지 않는다.**

**O2. 접근안 B (C++ `clear_vr()` 램프화)의 필요성.**
플레이어 하드 크래시 시 q_ref 계단 스냅이 실제로 문제인지 sim2sim에서 `kill -9` 로 확인한 뒤,
필요하면 별도 승인을 받아 붙인다 (N4).

---

## 13. 검증 순서

낮은 위험에서 높은 위험으로 올라간다. 각 단계를 통과해야 다음으로 간다.

1. **dry-run** — shm에 쓰지 않고 프레임 계산만. 구간·속도·T_in 산정·프리플라이트 확인.
2. **sim2sim mode2 ×0.5** — walk 클립 15 s. `base_vel: zero` 먼저, 그 다음 `clip`.
3. **sim2sim mode3 ×0.5 → ×1.0** — walk → dance → run 순.
4. **O1 계측** — mode2로 60 s 이상 재생하며 yaw 오차 추이 기록.
5. **중단 경로 전수** — Space / Ctrl-C / `kill -9` / `x`(E-stop). 각각 관절 목표가 튀지 않는지.
6. **실로봇** — 사람이 하드 E-stop 옆에서 대기. **mode2 walk 짧은 구간 ×0.5 부터.**

⚠ sim2sim 재기동 전 **반드시 `pkill -x g1_ctrl`**. orphan 컨트롤러 중복은 즉시 낙상으로 나타나
policy 문제로 오진된다 (CLAUDE.md 트러블슈팅 절).

---

## 14. 테스트

pytest로 잠근다. 전부 로봇/시뮬 없이 돈다.

| 테스트 | 무엇을 막나 |
|---|---|
| Resolver가 `ONNX_META.json` → 매니페스트 → 클립 목록을 정확히 읽는다 | 슬롯 추가 시 조용한 어긋남 |
| `modeN_ckpt == "none"` 인 모드가 `valid_modes` 에서 빠진다 | 무효 head로 재생 |
| ONNX_META 없는 슬롯이 **안전 실패**한다 (빈 목록 + 사유) | 추측으로 24클립 가정 |
| 램프 보간이 경계에서 연속 (`s=0` → standby, `s=1` → clip[f₀]) | 진입/이탈 점프 |
| `speed` 변경 시 `qd_ref` 가 함께 스케일된다 | 위치·속도 참조 모순 |
| 송출 패킷이 `vr_shm.FMT` 와 바이트 일치 (276 B, magic `0x6702`) | C++ 구조체 어긋남 |
| 프리플라이트가 NaN·관절한계·범위밖 span·mode1을 전부 거부한다 | 위험 재생 |
| `base_vel: clip` 의 yaw-local 변환이 알려진 입력에 대해 정확하다 | 좌표계 실수 |

---

## 15. 파일 배치

```
deploy/robots/g1/teleop/motion_player/
  __init__.py
  resolver.py     policy_dir → ONNX_META → manifest → PolicyContext
  playlist.py     presets.yaml 로드 + "1 40 15 x0.5" 파싱 → PlayItem
  publisher.py    상태기계 + 50 Hz 절대시각 송출 (vr_shm 재사용)
  cli.py          화면 + 키 입력
  presets.yaml    프리셋 구간 + slot_overrides
  tests/
docs/superpowers/specs/2026-08-13-lafan-motion-player-design.md   (이 문서)
```

기존 `teleop/vr_replay.py` 는 **손대지 않고 남긴다** — 배선 검증(SYSTEM_OVERVIEW Hop 0)용으로
계속 쓸모가 있다.

배포 C++는 수정하지 않는다 (R7).
