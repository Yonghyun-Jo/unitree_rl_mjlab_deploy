# 패리티 — `261009_v7_f6_s1456_109k`

판정 **KNOWN** (PASS 45 · KNOWN 1 · WARN 2 · SKIP 2 · FAIL 0) · 2026-10-28T16:45:20 · check_slot_parity.py v1

- 학습 원장(각 run 의 training_meta.json commit): base `d84f487` · mode1 `640aae0` · mode4 `77b6aa0` · mode5 `640aae0` · mode6 `88edb2f`
- 배포 (판정 당시): deploy repo `368a48a` · ONNX md5 `ec8802500368` · deploy.yaml sha256 `13c4a1012f53`
- 이 표가 유효한 조건: ONNX md5·deploy.yaml sha256 가 위와 같을 것 (policy_slot push · robot.sh verify 가 본다)

| # | 항목 | 학습 (원장) | 배포 | 판정 | 비고 |
|---|---|---|---|---|---|
| 1.1 | ONNX md5 == META 기록 | ec8802500368 | ec8802500368 | **PASS** |  |
| 1.2 | ONNX 메타(base·head·steps·계약) == ONNX_META.json | ONNX metadata | ONNX_META.json | **PASS** |  |
| 1.3 | ONNX 입출력 | 계약 합 3006 | obs[1, 3006] → actions[1, 29] | **PASS** |  |
| 1.4 | base 학습 원장 | 2026-10-23_13-02-11_stage2_v6final_m3all @d84f487 | ONNX_META base | **PASS** |  |
| 1.5 | mode1 학습 원장 | 2026-10-27_18-37-42_s4_mode1_m3all250k_r @640aae0 | ONNX_META mode1 | **PASS** |  |
| 1.6 | mode4 학습 원장 | 2026-10-26_12-11-17_s4_mode4_m3all250k_r @77b6aa0 | ONNX_META mode4 | **PASS** |  |
| 1.7 | mode5 학습 원장 | 2026-10-26_01-05-42_s4_mode5_m3all250k_r @640aae0 | ONNX_META mode5 | **WARN** | ⚠ 원격(dogi) 학습 run — training_meta.json 없음. launch 시각 2026-10-26 01:05:42 직전 final_v6 커밋 640aae0 로 추정(branch = 수확 기록의 branch · 서버 sync 는 com1 작업 트리를 보낸다 — 미커밋 변경은 모른다) · ⚠ 학습 manifest logs/_motion_data/mode5_gen/_subset_graphC_v2.yaml ≠ export manifest motions/g1_flow_specialists_c3_m6.yaml |
| 1.8 | mode6 학습 원장 | 2026-10-20_12-43-44_s4_mode6_v6final_reb @88edb2f | ONNX_META mode6 | **WARN** | ⚠ 원격(dogi) 학습 run — training_meta.json 없음. launch 시각 2026-10-20 12:43:44 직전 final_v6 커밋 88edb2f 로 추정(branch = 수확 기록 워크트리 mjlab_g1_final 의 브랜치 · 서버 sync 는 com1 작업 트리를 보낸다 — 미커밋 변경은 모른다) · ⚠ 학습 manifest logs/_motion_data/mode6_gen/_subset_ground1_subject5.yaml ≠ export manifest motions/g1_flow_specialists_c3_m6.yaml |
| 1.9 | base·head 의 obs 그룹이 같은가 | base@d84f487 · mode1@640aae0 · mode4@77b6aa0 · mode5@640aae0 · mode6@88edb2f | 하나의 obs 벡터·하나의 작동을 공유 | **PASS** |  |
| 1.10 | base·head 의 작동 상수(관절 순서·기본자세·PD·scale·dt)가 같은가 | base@d84f487 · mode1@640aae0 · mode4@77b6aa0 · mode5@640aae0 · mode6@88edb2f | 하나의 obs 벡터·하나의 작동을 공유 | **PASS** |  |
| 1.11 | export 시점 mjlab commit | - | 0b421bf | **PASS** |  |
| 2.1 | 1. base_ang_vel | base_ang_vel ← base_ang_vel · 3×10 · scale 없음(=1) · clip 없음 | base_ang_vel · train_term=base_ang_vel · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.2, 0.2) — 배포엔 없음(정상) |
| 2.2 | 2. projected_gravity | projected_gravity ← projected_gravity · 3×10 · scale 없음(=1) · clip 없음 | projected_gravity · train_term=projected_gravity · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.05, 0.05) — 배포엔 없음(정상) |
| 2.3 | 3. command | command ← masked_joint_command · 58×10 · scale 없음(=1) · clip 없음 | masked_joint_command · train_term=command · 58×10 · scale [1]×58 · clip 없음 | **PASS** |  |
| 2.4 | 4. motion_root_ori_b | motion_root_ori_b ← masked_root_ori_b · 6×10 · scale 없음(=1) · clip 없음 | masked_root_ori_b · train_term=motion_root_ori_b · 6×10 · scale [1, 1, 1, 1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.05, 0.05) — 배포엔 없음(정상) |
| 2.5 | 5. joint_pos | joint_pos ← joint_pos_rel · 29×10 · scale 없음(=1) · clip 없음 | joint_pos_rel · train_term=joint_pos · 29×10 · scale [1]×29 · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.01, 0.01) — 배포엔 없음(정상) |
| 2.6 | 6. joint_vel | joint_vel ← joint_vel_rel · 29×10 · scale 없음(=1) · clip 없음 | joint_vel_rel · train_term=joint_vel · 29×10 · scale [1]×29 · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.5, 0.5) — 배포엔 없음(정상) |
| 2.7 | 7. actions | actions ← last_action · 29×10 · scale 없음(=1) · clip 없음 | last_action · train_term=actions · 29×10 · scale [1]×29 · clip 없음 | **PASS** |  |
| 2.8 | 8. base_vel | base_vel ← base_vel_command · 3×10 · scale 없음(=1) · clip 없음 | base_vel_command · train_term=base_vel · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** |  |
| 2.9 | 9. mask | mask ← command_mask · 8×10 · scale 없음(=1) · clip 없음 | mode_mask · train_term=mask · 8×10 · scale [1]×8 · clip 없음 | **PASS** | 학습 함수 command_mask ↔ 배포 항 mode_mask (대응표로 잇는다) |
| 2.10 | 10. foot_z | foot_z ← ref_foot_height · 2×10 · scale 없음(=1) · clip 없음 | ref_foot_height · train_term=foot_z · 2×10 · scale [1, 1] · clip 없음 | **PASS** |  |
| 2.11 | 11. limb_z | limb_z ← ref_limb_height · 4×10 · scale 없음(=1) · clip 없음 | ref_limb_height · train_term=limb_z · 4×10 · scale [1, 1, 1, 1] · clip 없음 | **PASS** |  |
| 2.12 | 12. mode5_cmd | mode5_cmd ← mode5_cmd · 53×10 · scale 없음(=1) · clip 없음 | mode5_command · train_term=mode5_cmd · 53×10 · scale [1]×53 · clip 없음 | **PASS** | 학습 함수 mode5_cmd ↔ 배포 항 mode5_command (대응표로 잇는다) |
| 2.13 | 13. preview_on | preview_on ← motion_preview_on · 1×1 · scale 없음(=1) · clip 없음 | motion_preview_on · train_term=preview_on · 1×1 · scale [1] · clip 없음 | **PASS** |  |
| 2.14 | 14. preview_k1 | preview_k1 ← motion_preview_k1 · 35×1 · scale 없음(=1) · clip 없음 | motion_preview_k1 · train_term=preview_k1 · 35×1 · scale [1]×35 · clip 없음 | **PASS** |  |
| 2.15 | 15. preview | preview ← motion_preview · 700×1 · scale 없음(=1) · clip 없음 | motion_preview · train_term=preview · 700×1 · scale [1]×700 · clip 없음 | **PASS** |  |
| 2.16 | 합 = ONNX 입력 = META obs_dim | 계약 3006 | deploy 3006 · ONNX 3006 · META 3006 | **PASS** |  |
| 2.17 | history 배치 | mjlab: 항마다 history(오래된→최신)를 펴서 항 순서로 잇는다 | use_gym_history 미설정 → term-major | **PASS** |  |
| 2.18 | head 선택 mask 위치 (ONNX 그래프가 실제로 자르는 곳) | 그래프 Slice [1672:1680] | 배포 배치의 최신 mask = [-1334:-1326] | **PASS** |  |
| 3.1 | 관절 순서 | JOINT_ORDER 29 (left_hip_pitch_joint … right_wrist_yaw_joint) | joint_ids_map 항등 · 장면 src/assets/robots/unitree_g1/xmls/scene_g1.xml | **PASS** | 실기 SDK 모터 순서 = 이 장면 순서라는 전제(유니트리 G1 29dof 정의) |
| 3.2 | 기본 자세 default_joint_pos | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | **PASS** | 최대 \|Δ\| 0.0e+00 (허용 1e-06) |
| 3.3 | action offset | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | **PASS** | 최대 \|Δ\| 0.0e+00 (허용 1e-06) · 학습 use_default_offset=True |
| 3.4 | action scale | 29값 [0.5475, 0.3507, 0.5475, 0.3507, 0.4386, 0.4386, …] | 29값 [0.5475, 0.3507, 0.5475, 0.3507, 0.4386, 0.4386, …] | **PASS** | 최대 상대오차 1.1e-04 (허용 0.001) |
| 3.5 | PD stiffness | 29값 [40.1792, 99.0984, 40.1792, 99.0984, 28.5012, 28.5012, …] | 29값 [40.179, 99.098, 40.179, 99.098, 28.501, 28.501, …] | **PASS** | 최대 상대오차 2.6e-05 (허용 0.001) |
| 3.6 | PD damping | 29값 [2.5579, 6.3088, 2.5579, 6.3088, 1.8144, 1.8144, …] | 29값 [2.5579, 6.3088, 2.5579, 6.3088, 1.8144, 1.8144, …] | **PASS** | 최대 상대오차 3.9e-05 (허용 0.001) |
| 3.7 | 정책 주기 step_dt | 0.005 × 4 = 0.02 | 0.02 | **PASS** |  |
| 3.8 | action clip (배포 전용 기계한계) | 학습 clip 없음 (EnvelopeJointPositionActionCfg) | 29쌍 · 기본자세 포함 예 · safety pos_min/max 와 같다 | **PASS** |  |
| 3.9 | last_action = 정책 raw 출력 | mjlab last_action → raw_action | observations.h last_action → action_manager->action() | **PASS** |  |
| 4.1 | 모드 → mask 비트 | mask 8칸 (계약 v2) | - | **SKIP** | 계약 v2 mode_mask 대조는 gen_mode_table_header --check 몫 |
| 4.2 | mode1 명령 범위 (조이스틱 끝 = 학습 범위의 끝) | stage4_mode1 CMD_BASE_VEL vx [-1.5, 3] vy [-0.8, 0.8] wz [-2, 2] @640aae0 | C++ vx [-1.5, 2.5] vy [-0.8, 0.8] wz [-2, 2] (모드 공통) | **KNOWN** | vx: 학습 [-1.5, 3] ≠ 배포 [-1.5, 2.5] — K3 (사용자 결정 (2026-10-28): 첫 실기는 전진 상한 2.5 유지) |
| 4.3 | mode6 명령 범위 (기는 모드 봉투 = 그 head 의 학습 범위) | stage4_mode6 CMD_BASE_VEL vx [-0.3, 0.5] vy [-0.3, 0.3] wz [-1, 1] @88edb2f | C++ CRAWL vx [-0.3, 0.5] vy [-0.3, 0.3] wz [-1, 1] | **PASS** |  |
| 4.4 | 사지 높이 칸을 켜는 모드 (학습 limb_z_live ↔ 배포 crawl 행) | limb_z_live 모드 [6] @d84f487 | crawl_cmd_live 행 [6] · 내놓는 기는 모드 [6] | **PASS** |  |
| 4.5 | LimbLut.h 표 == 그 head 의 limb_lut_data (mode6) | limb_lut_data.py @88edb2f (NB 32 · 주기 1.64 s) | LimbLut.h GENERATED 구역 | **PASS** |  |
| 4.6 | FOOT_GEN 판독기 두 벌이 같은 값을 읽는가 (계측기 자기검증) | import (그 commit 의 cfg 모듈) | stage_candidates.foot_gen_at (META gait_parity 를 쓴 판독기) | **PASS** |  |
| 4.7 | mode1 발-z 생성 조건 (foot_z obs 의 값) | FOOT_GEN@640aae0: source=lut cadence=1.15 settle_steps=100 min_swing=0.08 stand_deadzone=0 turn_k=0.3 walk_max=1.2 run_min=1.7 table=2 turn_asym=True settle_eff=0.15 settle_phase=0.07 | deploy gait: source=lut cadence=1.15 settle_steps=100 min_swing=0.08 stand_deadzone=0 turn_k=0.3 walk_max=1.2 run_min=1.7 table=2 turn_asym=True settle_eff=0.15 settle_phase=0.07 | **PASS** |  |
| 4.8 | mode4 클립 = 학습 manifest(mode4) 클립 (OOD·다른 리타겟판 없음) | manifest mode4 58 개 | slot clips 58 개 · requires clip_list 있음 | **PASS** |  |
| 4.9 | GaitLut.h 배열 == 학습 gait_lut_data | gait_lut_data.py (V1=d94c7d9~1 · V2=워킹트리) | GaitLut.h GENERATED 구역 | **PASS** | gen_gait_lut_header --check: 표는 같다 (숫자 바이트 동일). |
| 4.10 | IMU 보정 (projected_gravity 를 바꾼다) | 학습: 보정 없음 | config.yaml imu_cal pitch 0° roll 0° | **PASS** |  |
| 4.11 | requires ⊆ 이 소스가 아는 기능 | deploy.yaml requires 13개 | DeployFeatures.h known() 14개 | **PASS** |  |
| 4.12 | 추적 패리티 (sim2sim obs 덤프 vs 학습 함수) | - | - | **SKIP** | Tier B 미도입 |

## KNOWN — 학습과 다르다는 것을 알고 배포하는 항목 (`parity_known.yaml`)

- **K3** `cmd.mode1` · vx — final_v6 skill1 head(공유 base m3all 250k 위, s4_mode1_m3all250k_resume89k_to109k)는 stage4_mode1 CMD_BASE_VEL vx (-1.5, 3.0)(mjlab beac041, 10-22)로 학습했는데 배포 전진 상한(VX_MAX_FWD)은 2.5 다. 조이스틱 끝 = 2.5 m/s 라 학습 봉투 «안쪽» 만 쓴다(OOD 아님) — 2.5~3.0 m/s 구간을 실기에서 안 쓸 뿐이다. 후진 1.5 · vy · wz 는 같다. 근거 위치: deploy/robots/g1/src/State_Mimic.cpp 의 VX_MAX_FWD · mjlab_g1_final stage4_mode1_env_cfg.CMD_BASE_VEL. **사용자 결정 (2026-10-28): 첫 실기는 전진 상한 2.5 유지**
