# 정책 슬롯 원장

> 자동 생성 — `deploy/scripts/policy_slot.py index`. 손으로 고치지 말 것.

**활성** = 가중치가 repo 워킹트리에 있다(로봇에 보낼 수 있다).
**보관** = 가중치가 `/home/piene/experiments/_realrobot/center_g1/_policies` 에 있고 repo 엔 정체(`deploy.yaml`·`ONNX_META.json`)만 남았다.
`restore` 로 언제든 되살린다. **기록은 어느 쪽이든 git 에 남아 있다.**

| 슬롯 | 별칭 | 상태 | mode1 출처 | sim2sim | 실기 |
|---|---|---|---|---|---|
| `260828_v1_settle55k` | — | 보관 | 2026-08-27_13-15-29_s4_mode1_settle_graft_resume44k_to60k/model_55500.pt | 미실행 — 사용자가 확인 | 미실행 |
| `260828_v1b_minswing6` | — | 활성 | 2026-08-27_13-15-29_s4_mode1_settle_graft_resume44k_to60k/model_55500.pt | 통과 — com1 sim2sim 에서 사용자 확인 (2026-08-28). 표 진폭 eff 0.05~0.40 | 배포 완료 · 시험 대기 (2026-08-28 11:55) |
| `260828_v1c_minswing8` | — | 보관 | 2026-08-27_13-15-29_s4_mode1_settle_graft_resume44k_to60k/model_55500.pt | 미실행 — 사용자가 확인 | 배포 완료 · 시험 대기 (2026-08-28 11:55) |
| `260828_v2_settle_scratch35k` | — | 보관 | 2026-08-27_23-09-25_s4_mode1_settle_scratch_to60k/model_35500.pt | 미실행 — 사용자가 확인 | 미실행 |
| `260828_v2b_minswing6` | — | 보관 | 2026-08-27_23-09-25_s4_mode1_settle_scratch_to60k/model_35500.pt | 미실행 — v1b 에서 같은 코드경로가 통과했고, 여기선 ONNX 만 v2 로 다르다 | 배포 완료 · 시험 대기 (2026-08-28) |
| `260828_v2c_minswing8` | — | 보관 | 2026-08-27_23-09-25_s4_mode1_settle_scratch_to60k/model_35500.pt | 미실행 — v1b 에서 같은 코드경로가 통과했고, 여기선 ONNX 만 v2 로 다르다 | 배포 완료 · 시험 대기 (2026-08-28) |
| `260829_v1_ms8_noslip1_30k` | v1 | 활성 | 2026-08-28_12-08-17_s4_mode1_minswing8_noslip1_parallel1_scratch30k/model_29999.pt | ★★★★☆ 아주 깔끔하게 걷는데? 발이 좀 높은 느낌이 있긴 함. 근데 잘 걸음. 실기가 기대됨. (2026 | 미실행 |
| `260829_v2_ms6_fz2p5_30k` | v2 | 활성 | 2026-08-28_18-02-25_s4_mode1_minswing6_fz2p5_scratch30k/model_29999.pt | 미실행 — 사용자가 확인 | 미실행 |
| `gmt_multihead_cop_slip` | — | 보관 | (불명) | — | — |
| `gmt_multihead_cwc` | — | 보관 | (불명) | — | — |
| `gmt_multihead_cwc_scratch` | — | 보관 | 2026-07-13_17-31-46_mode1_fh2p0_mirror0p5_v5/model_20000.pt | onnxruntime vs torch max|Δ| = 4.351e-06 (tol 1e-4). mask sli | — |
| `gmt_multihead_v0` | — | 보관 | (불명) | — | — |
| `masked_footz_v0` | — | 보관 | (불명) | — | — |
| `masked_v2_anchormask_modeterm` | — | 보관 | (불명) | — | — |
| `v2_mode3_steps6` | — | 보관 | none | — | — |
| `v3_multihead_s30k_steps6` | — | 보관 | 2026-08-21_16-29-26_s4_mode1_k0p85_symgen_resume15k_to30k/model_29999.pt | — | — |
| `v3_multihead_steps6` | — | 보관 | 2026-08-20_16-12-34_s4_mode1_torsotilt_k0p85_resume50k/model_52000.pt | — | — |
| `v4_multihead_m1colmov2_steps6` | — | 보관 | 2026-08-25_23-30-38_s4_mode1_cad1p15_slowcmd_scratch30k/model_18000.pt | — | 2026-08-26 15:48–15:49 실기 실행 — mode1 · 체 |

## 왜 갈라 두나

슬롯 하나가 60 MB 다. git 은 히스토리에서 그걸 **영원히 안 지운다** — 워킹트리에서
지워도 새로 clone 하는 쪽은 전부 받는다. 그래서 «무게» 만 git 밖으로 내보내고
«정체» 는 git 에 남긴다. 실험이 무엇이었는지는 슬롯을 치워도 사라지지 않는다.

