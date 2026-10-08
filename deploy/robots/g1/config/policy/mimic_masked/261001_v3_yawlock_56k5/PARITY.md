# 패리티 — `261001_v3_yawlock_56k5`

판정 **KNOWN** (PASS 39 · KNOWN 2 · WARN 0 · SKIP 2 · FAIL 0) · 2026-10-08T17:57:23 · check_slot_parity.py v1

- 학습 원장(각 run 의 training_meta.json commit): base `c8c3ea6` · mode1 `2318d2e` · mode2 `b28ceb3` · mode3 `f22eab5`
- 배포 (판정 당시): deploy repo `b5e1367` · ONNX md5 `e9882b629917` · deploy.yaml sha256 `3bbcb8b73fd3`
- 이 표가 유효한 조건: ONNX md5·deploy.yaml sha256 가 위와 같을 것 (policy_slot push · robot.sh verify 가 본다)

| # | 항목 | 학습 (원장) | 배포 | 판정 | 비고 |
|---|---|---|---|---|---|
| 1.1 | ONNX md5 == META 기록 | e9882b629917 | e9882b629917 | **PASS** |  |
| 1.2 | ONNX 메타(base·head·steps·계약) == ONNX_META.json | ONNX metadata | ONNX_META.json | **PASS** |  |
| 1.3 | ONNX 입출력 | 계약 합 1640 | obs[1, 1640] → actions[1, 29] | **PASS** |  |
| 1.4 | base 학습 원장 | 2026-08-11_01-09-08_v3_deploy_cap1024_4L @c8c3ea6 | ONNX_META base | **PASS** |  |
| 1.5 | mode1 학습 원장 | 2026-10-03_12-14-02_s4_mode1_yawlock2p0_ @2318d2e | ONNX_META mode1 | **PASS** |  |
| 1.6 | mode2 학습 원장 | 2026-08-20_18-11-18_s4_mode2_v3base_port @b28ceb3 | ONNX_META mode2 | **PASS** | launch 미커밋 변경 6 파일 — parity 무관 파일만 |
| 1.7 | mode3 학습 원장 | 2026-08-14_11-36-10_s4_mode3_v3base_cwcm @f22eab5 | ONNX_META mode3 | **PASS** |  |
| 1.8 | base·head 의 obs 그룹이 같은가 | base@c8c3ea6 · mode1@2318d2e · mode2@b28ceb3 · mode3@f22eab5 | 하나의 obs 벡터·하나의 작동을 공유 | **PASS** |  |
| 1.9 | base·head 의 작동 상수(관절 순서·기본자세·PD·scale·dt)가 같은가 | base@c8c3ea6 · mode1@2318d2e · mode2@b28ceb3 · mode3@f22eab5 | 하나의 obs 벡터·하나의 작동을 공유 | **PASS** |  |
| 1.10 | export 시점 mjlab commit | - | 기록 없음 | **SKIP** | 이 기록을 남기기 전에 만든 슬롯 — 판정은 launch commit 진실로 한다 |
| 2.1 | 1. base_ang_vel | base_ang_vel ← base_ang_vel · 3×10 · scale 없음(=1) · clip 없음 | base_ang_vel · train_term=base_ang_vel · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.2, 0.2) — 배포엔 없음(정상) |
| 2.2 | 2. projected_gravity | projected_gravity ← projected_gravity · 3×10 · scale 없음(=1) · clip 없음 | projected_gravity · train_term=projected_gravity · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.05, 0.05) — 배포엔 없음(정상) |
| 2.3 | 3. command | command ← masked_joint_command · 58×10 · scale 없음(=1) · clip 없음 | masked_joint_command · train_term=command · 58×10 · scale [1]×58 · clip 없음 | **PASS** |  |
| 2.4 | 4. motion_root_ori_b | motion_root_ori_b ← masked_root_ori_b · 6×10 · scale 없음(=1) · clip 없음 | masked_root_ori_b · train_term=motion_root_ori_b · 6×10 · scale [1, 1, 1, 1, 1, 1] · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.05, 0.05) — 배포엔 없음(정상) |
| 2.5 | 5. joint_pos | joint_pos ← joint_pos_rel · 29×10 · scale 없음(=1) · clip 없음 | joint_pos_rel · train_term=joint_pos · 29×10 · scale [1]×29 · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.01, 0.01) — 배포엔 없음(정상) |
| 2.6 | 6. joint_vel | joint_vel ← joint_vel_rel · 29×10 · scale 없음(=1) · clip 없음 | joint_vel_rel · train_term=joint_vel · 29×10 · scale [1]×29 · clip 없음 | **PASS** | 학습 노이즈 UniformNoiseCfg(-0.5, 0.5) — 배포엔 없음(정상) |
| 2.7 | 7. actions | actions ← last_action · 29×10 · scale 없음(=1) · clip 없음 | last_action · train_term=actions · 29×10 · scale [1]×29 · clip 없음 | **PASS** |  |
| 2.8 | 8. base_vel | base_vel ← base_vel_command · 3×10 · scale 없음(=1) · clip 없음 | base_vel_command · train_term=base_vel · 3×10 · scale [1, 1, 1] · clip 없음 | **PASS** |  |
| 2.9 | 9. mask | mask ← command_mask · 2×10 · scale 없음(=1) · clip 없음 | command_mask · train_term=mask · 2×10 · scale [1, 1] · clip 없음 | **PASS** |  |
| 2.10 | 10. foot_z | foot_z ← ref_foot_height · 2×10 · scale 없음(=1) · clip 없음 | ref_foot_height · train_term=foot_z · 2×10 · scale [1, 1] · clip 없음 | **PASS** |  |
| 2.11 | 합 = ONNX 입력 = META obs_dim | 계약 1640 | deploy 1640 · ONNX 1640 · META 1640 | **PASS** |  |
| 2.12 | history 배치 | mjlab: 항마다 history(오래된→최신)를 펴서 항 순서로 잇는다 | use_gym_history 미설정 → term-major | **PASS** |  |
| 2.13 | head 선택 mask 위치 (ONNX 그래프가 실제로 자르는 곳) | 그래프 Slice [-22:-20] | 배포 배치의 최신 mask = [-22:-20] | **PASS** |  |
| 3.1 | 관절 순서 | JOINT_ORDER 29 (left_hip_pitch_joint … right_wrist_yaw_joint) | joint_ids_map 항등 · 장면 src/assets/robots/unitree_g1/xmls/scene_g1.xml | **PASS** | 실기 SDK 모터 순서 = 이 장면 순서라는 전제(유니트리 G1 29dof 정의) |
| 3.2 | 기본 자세 default_joint_pos | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | **PASS** | 최대 \|Δ\| 0.0e+00 (허용 1e-06) |
| 3.3 | action offset | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | 29값 [-0.312, 0, 0, 0.669, -0.363, 0, …] | **PASS** | 최대 \|Δ\| 0.0e+00 (허용 1e-06) · 학습 use_default_offset=True |
| 3.4 | action scale | 29값 [0.5475, 0.3507, 0.5475, 0.3507, 0.4386, 0.4386, …] | 29값 [0.5475, 0.3507, 0.5475, 0.3507, 0.4386, 0.4386, …] | **PASS** | 최대 상대오차 1.1e-04 (허용 0.001) |
| 3.5 | PD stiffness | 29값 [40.1792, 99.0984, 40.1792, 99.0984, 28.5012, 28.5012, …] | 29값 [40.179, 99.098, 40.179, 99.098, 28.501, 28.501, …] | **PASS** | 최대 상대오차 2.6e-05 (허용 0.001) |
| 3.6 | PD damping | 29값 [2.5579, 6.3088, 2.5579, 6.3088, 1.8144, 1.8144, …] | 29값 [2.5579, 6.3088, 2.5579, 6.3088, 1.8144, 1.8144, …] | **PASS** | 최대 상대오차 3.9e-05 (허용 0.001) |
| 3.7 | 정책 주기 step_dt | 0.005 × 4 = 0.02 | 0.02 | **PASS** |  |
| 3.8 | action clip (배포 전용 기계한계) | 학습 clip 없음 (EnvelopeJointPositionActionCfg) | 29쌍 · 기본자세 포함 예 · safety pos_min/max 와 같다 | **PASS** |  |
| 3.9 | last_action = 정책 raw 출력 | mjlab last_action → raw_action | observations.h last_action → action_manager->action() | **PASS** |  |
| 4.1 | 모드 → [mask_upper, mask_lower] | mode1 [0, 0] · mode2 [1, 0] · mode3 [1, 1] | mode1 [0, 0] · mode2 [1, 0] · mode3 [1, 1] | **PASS** |  |
| 4.2 | mode1 명령 범위 (조이스틱 끝 = 학습 범위의 끝) | stage4_mode1 CMD_BASE_VEL vx [-1.5, 2.5] vy [-0.8, 0.8] wz [-2, 2] @2318d2e | C++ vx [-1.5, 2.5] vy [-0.8, 0.8] wz [-2, 2] (모드 공통) | **PASS** |  |
| 4.3 | mode2 명령 범위 (조이스틱 끝 = 학습 범위의 끝) | stage4_mode2 CMD_BASE_VEL vx [-1, 1.5] vy [-0.8, 0.8] wz [-2, 2] @b28ceb3 | C++ vx [-1.5, 2.5] vy [-0.8, 0.8] wz [-2, 2] (모드 공통) | **KNOWN** | vx: 학습 [-1, 1.5] ≠ 배포 [-1.5, 2.5] — K2 (사용자 결정 대기: 고칠지/유지할지) |
| 4.4 | FOOT_GEN 판독기 두 벌이 같은 값을 읽는가 (계측기 자기검증) | import (그 commit 의 cfg 모듈) | stage_candidates.foot_gen_at (META gait_parity 를 쓴 판독기) | **PASS** |  |
| 4.5 | mode1 발-z 생성 조건 (foot_z obs 의 값) | FOOT_GEN@2318d2e: source=lut cadence=1.15 settle_steps=100 min_swing=0.08 stand_deadzone=0 turn_k=0.3 walk_max=1.2 run_min=1.7 table=2 turn_asym=True settle_eff=0.15 settle_phase=0.07 | deploy gait: source=lut cadence=1.15 settle_steps=100 min_swing=0.08 stand_deadzone=0 turn_k=0.3 walk_max=1.2 run_min=1.7 table=2 turn_asym=True settle_eff=0.15 settle_phase=0.07 | **PASS** |  |
| 4.6 | mode2 발-z 생성 조건 (foot_z obs 의 값) | FOOT_GEN@b28ceb3: source=lut cadence=1 settle_steps=0 min_swing=0 stand_deadzone=0.15 turn_k=0.3 walk_max=1.2 run_min=1.7 table=1 turn_asym=True | deploy gait: source=lut cadence=1 settle_steps=0 min_swing=0 stand_deadzone=0.15 turn_k=0.3 walk_max=1.2 run_min=1.7 table=1 turn_asym=False | **KNOWN** | turn_asym: 학습 True ≠ 배포 False — K1 (사용자 결정 대기: 고칠지/유지할지) |
| 4.7 | mode3 발-z 원천 | stage4_mode3 FOOT_GEN 없음 → 레퍼런스 발 world-z | ModeTable mode3 foot_z=Ref · requires ref_foot_height_ref 있음 | **PASS** | VR 이 발 높이를 안 줄 때는 두 발 접지(stance)로 대신한다(설계) |
| 4.8 | GaitLut.h 배열 == 학습 gait_lut_data | gait_lut_data.py (V1=d94c7d9~1 · V2=워킹트리) | GaitLut.h GENERATED 구역 | **PASS** | gen_gait_lut_header --check: 표는 같다 (숫자 바이트 동일). |
| 4.9 | IMU 보정 (projected_gravity 를 바꾼다) | 학습: 보정 없음 | config.yaml imu_cal pitch 0° roll 0° | **PASS** |  |
| 4.10 | requires ⊆ 이 소스가 아는 기능 | deploy.yaml requires 6개 | DeployFeatures.h known() 11개 | **PASS** |  |
| 4.11 | 추적 패리티 (sim2sim obs 덤프 vs 학습 함수) | - | - | **SKIP** | Tier B 미도입 |

## KNOWN — 학습과 다르다는 것을 알고 배포하는 항목 (`parity_known.yaml`)

- **K1** `gait.mode2` · turn_asym — mode2 head(2026-08-20 s4_mode2_v3base_port5_m2lgen_steps6_30k, launch b28ceb3)는 회전 비대칭 (17995ba, 08-17)이 켜진 발-z 명령으로 학습됐는데 배포는 끈다. 회전 중에만 foot_z obs 2칸이 다르다 (걷기 1.25 rad/s 0.42~0.62 cm · 달리기 1.0 rad/s 1.20 cm, 두 발 평균 스윙은 같다). mode1 만 바꾸는 판에서 mode2 를 같이 움직이면 실기 회귀 원인을 못 가르기 때문에 꺼 두었다(재빌드 없이 한 줄로 켠다). 근거 위치: 슬롯 deploy.yaml 의 gait: mode2 줄 주석 (261001_v1~v4 · «mode2 는 turn_asym 을 일부러 껐다»). **사용자 결정 대기: 고칠지/유지할지**
- **K2** `cmd.mode2` · vx — C++ base_vel 상한(VX_MAX_FWD 2.5 · VX_MAX_BWD 1.5)은 mode1 학습 범위(stage4_mode1 CMD_BASE_VEL)라 모드 공통인데, mode2 head 는 stage4_mode2 CMD_BASE_VEL vx (-1.0, 1.5) 로 학습됐다. mode2 에서 조이스틱을 앞으로 60 %(1.5 m/s)·뒤로 67 %(1.0 m/s) 넘게 밀면 학습 범위 밖 명령이다(전진 최대 1.67배). vy ±0.8 · wz ±2.0 은 같다. 근거 위치: deploy/robots/g1/src/State_Mimic.cpp 의 VX_MAX_* 위 주석 (≈182행, «모드별 봉투 분리는 별건으로 남겨 둔다»). **사용자 결정 대기: 고칠지/유지할지**
