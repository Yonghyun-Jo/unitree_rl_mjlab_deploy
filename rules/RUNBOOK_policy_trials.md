# 정책 시험 런북 — 하루에 여러 후보를 실기에서 돌린다

> **무엇을 푸는 문서인가**: 「정책 1, 2, 3, 4… 를 하루에 바꿔 가며 시험하고 싶다.
> 2번은 어떻게 돌리나? 재빌드는 해야 하나? 결과는 어디에 남나?」
>
> 배포 «경로»(ONNX 굽기 → 슬롯 → sim2sim → push → 로봇) = `/deploy-g1-policy` 스킬
> 실기 «직전 점검» = `/preflight-g1-real` 스킬 · 실기 «운용» = `RUNBOOK_real_robot_mode1.md`
> 이 문서는 그 사이 — **하루치 시험을 굴리는 방법**이다.

---

## 한 줄 요약

**아침에 후보를 전부 push 해 두면, 하루 종일 파일수정 0 · pull 0 · 재빌드 0 이다.**

```bash
./deploy/robots/g1/tools/run_g1_with_gui.sh eth0 --policy <슬롯이름>
```

---

## 1. 하루 흐름

### 아침 (com1, 한 번)

> ⚡ **2026-08-29 부터는 한 명령이다.** 후보를 `candidates/<day>.yaml` 에 적고
> `python3 deploy/scripts/stage_candidates.py candidates/<day>.yaml --robot`
> → export · 슬롯 · `ACTIVE.yaml`(v1/v2/…) · 옛 활성 슬롯 보관 · 커밋 · rsync · `robot.sh deploy`.
> 로봇이 안 닿으면 세 명령(`git push` · `policy_slot.py push` · `robot.sh deploy`)을 찍어 준다.
> 규칙 = Obsidian `procedures/g1_realrobot_candidates.md`. 아래는 그 스크립트가 하는 일의 원본.

후보 슬롯을 **전부** 만들어 커밋·push 한다. 슬롯 하나 = 배포 단위.

```
config/policy/mimic_masked/<슬롯>/
├── exported/policy.onnx     ← mjlab export 산출물
├── params/deploy.yaml       ← obs 계약 · PD · action · safety · gait · requires
├── params/*.npz             ← motion 클립
└── ONNX_META.json           ← 출처·검증·활성기간
```

```bash
git add deploy/robots/g1/config/policy/mimic_masked/<슬롯>
git commit -m "deploy(<슬롯>): ..." && git push origin <branch>
~/piene_automation/robot_bridge/robot.sh deploy      # 로봇 pull + 빌드 + setcap + 게이트
```

⚠ **C++ 이 바뀌었으면 이때 한 번만 빌드하면 된다.** 이후 슬롯 전환에는 빌드가 없다.

### 로봇 앞 (하루 종일)

```bash
ssh g1 && cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy

./deploy/robots/g1/tools/run_g1_with_gui.sh --list      # 있는 슬롯 + requires 개수
./deploy/robots/g1/tools/run_g1_with_gui.sh eth0 --policy v4_multihead_m1colmov2_steps6
#   ... 시험 ... p 로 Passive → Ctrl-C

./deploy/robots/g1/tools/run_g1_with_gui.sh eth0 --policy v3_multihead_s30k_steps6
#   ... 다음 후보 ...
```

`--policy` 를 빼면 `config.yaml` 의 `policy_dir` 이 쓰인다(= 기본 정책).

### 저녁 (com1)

```bash
~/piene_automation/robot_bridge/robot.sh pull    # CSV + 시험 마커 회수, 자동 plot
```

대시보드 **Real Robot** 화면에서 실험마다 판정을 매긴다(아래 §4).

---

## 1-B. 🔴 sim2sim 은 «밴드를 푼 뒤» 가 아니면 데이터가 아니다

MuJoCo 의 고무밴드는 **기본 켜짐**(`simulate/config.yaml: enable_elastic_band: 1`,
`main.cc: bool enable_ = true`)이고 **시뮬 창에서 키 `9` 로만** 꺼진다. 켜져 있는 동안은
로봇이 매달려 **다리 하중이 1/10** 이 된다.

```
정지 중 |tau|      밴드 ON(sim)    밴드 OFF(실기)
  발목 pitch          0.55 Nm         5.6 Nm      ← 10 배
  무릎                2.8  Nm         9.2 Nm
```

**순서**: `f` → `m` → `1`(정책 ON) → 시뮬 창 `8`,`8`(내림) → **`9`(해제)** → 여기부터가 데이터.
FixStand 에서 먼저 풀면 고정 자세라 검증본도 4 초에 넘어간다.

⚠ **헤드리스(`xvfb-run`)로는 밴드를 못 푼다** — 시뮬 창에 키를 보내야 하는데 com1 에
`xdotool` 이 없다. 그래서 xvfb 로 찍은 sim 로그는 자동으로 «밴드 ON» 이다.

**기억에 기대지 말고 데이터로 확인한다:**
```bash
python3 deploy/scripts/check_band_released.py <gait_*.csv>
#   🟢 발목 |tau| ≥ 1 Nm → 풀림, 판정에 써도 된다
#   🔴 < 1 Nm         → 매달린 로그. 자세·처짐·추종 판정에 쓰지 말 것
```
(2026-09-01: 이 구분을 안 해서 «sim 은 멀쩡한데 실기만 처진다» 는 표를 만들었다가 물렀다.
그건 «밴드 vs 밴드없음» 이었다.)

## 1-C. 실기 명령을 sim 에 그대로 먹이기

사람이 키보드를 톡톡 치는 패턴은 두 번 다시 같게 안 나온다. 실기 로그의 명령을 재생한다:
```bash
# 시뮬+제어기 띄우고 f → m → 1 → 8,8 → 9(밴드 해제) 뒤, 다른 터미널에서
python3 deploy/robots/g1/tools/replay_cmd.py <실기 gait_*.csv> [--from 40 --to 70]
```
⚠ CSV 의 `bv_x` 는 «스플라인 후» 값이라 재생본이 원본보다 약간 더 부드럽다. 지형·접촉·외란은
재현되지 않는다.

## 2. 🔴 「재빌드 해야 하나?」 — 사람이 판단하지 않는다

**답: 안 해도 된다. 필요하면 g1_ctrl 이 거부하면서 알려준다.**

### 왜 이 질문이 위험했나

`deploy.yaml` 의 **모르는 키는 yaml 파서가 그냥 지나친다.** 2026-08-26 에 mode1 을 COLMOv2
표로 옮기며 `gait:` 에 `table`/`cadence`/`turn_asym` 을 넣었는데, 그 키를 파싱하는 코드가 없는
바이너리로 그 슬롯을 돌리면 —

> **에러가 안 난다.** 옛 거동(V1 표·cadence 1.0)으로 조용히 돈다. 로그도 정상으로 보인다.

즉 「재빌드 필요 여부」를 **사람 기억에 맡기고** 있었다. 슬롯 전환이 쉬워질수록 이 사고
확률이 올라간다.

### 그래서 슬롯이 «필요한 기능» 을 선언한다

```yaml
# deploy.yaml
requires:
  - obs_contract        # 기동 시 ONNX 계약 대조
  - gait_mode_table     # gait: 의 모드별 table / cadence / turn_asym
  - ref_foot_height_ref # mode>=3 발-z 를 레퍼런스에서
```

바이너리는 자기가 아는 기능 집합(`deploy/robots/g1/include/DeployFeatures.h`)을 갖고,
모르는 게 요구되면 **기동을 거부**한다:

```
🔴 이 슬롯이 요구하는 기능을 이 바이너리가 모른다 — 기동을 거부한다.
   슬롯: v5_something
   - 모르는 기능: 'gait_table_v3'
   이 바이너리가 아는 기능: obs_contract gait_mode_table ref_foot_height_ref policy_slot_env
   → 재빌드할 것:  cd deploy/robots/g1/build && cmake .. && make -j4
     (로봇이면 먼저 com1 에서 push -> 로봇에서 robot.sh deploy)
```

`requires:` 를 선언하지 않은 옛 슬롯은 그대로 통과한다(점진 도입).

### 새 기능을 추가할 때 (개발자용)

**거동을 바꾸는 새 `deploy.yaml` 키/블록을 넣으면** `DeployFeatures.h::known()` 에 이름을
추가하고, 그 키를 쓰는 슬롯의 `requires:` 에 적는다. 이름은 한 번 정하면 **바꾸지 않는다**
(옛 슬롯이 그 이름을 적고 있다).

---

## 3. 무엇이 바뀌면 무엇이 필요한가

| 바뀐 것 | push | 로봇 pull | **재빌드** | 슬롯 전환만으로 |
|---|---|---|---|---|
| ONNX 만 (같은 obs·같은 gait 키) | ✅ | ✅ | ❌ | ✅ |
| `deploy.yaml` 값 (safety·gait 수치) | ✅ | ✅ | ❌ | ✅ |
| `deploy.yaml` **새 키** (새 거동) | ✅ | ✅ | **✅** | ❌ |
| C++ (`src/`·`include/`) | ✅ | ✅ | **✅** | ❌ |
| `config.yaml` 기본 슬롯 | ✅ | ✅ | ❌ | — |

**판단하지 말 것** — 애매하면 그냥 돌려 본다. 재빌드가 필요하면 기동 시 거부당한다.

---

## 4. 결과를 남긴다 — 대시보드 Real Robot

> ⚡ 2026-08-29: **Live 아래 «📋 Trial Plan»** 이 오늘의 v1/v2 를 슬롯별로 보여 준다 — 무엇(META `note`) ·
> sim2sim(별점·메모, `python3 -m common.rr_plan sim2sim v1 --stars N --memo …` 또는 화면) · 실기(돌린 실험
> 링크). **Experiments** 의 각 실험에는 마커 `slot=` 로 «이 실험은 무엇이었나 / sim2sim / 같은 슬롯 실기 N회»
> 가 자동으로 붙는다. 판정 뒤 `python3 -m common.rr_plan sync` 가 실기 요약을 META `verify.real_robot` 에
> 되쓰고, 그걸 커밋하면 실험 자산이 슬롯과 함께 git 에 남는다. 전 과정 스킬 = `/realrobot-trial`.

`robot.sh pull` 이 CSV 와 **시험 마커**(`trial_*.txt`)를 같이 회수한다. 마커에는 그 실행의
슬롯·네트워크·커밋이 들어 있고, 이것이 **「이 실험이 어느 정책이었나」의 유일한 원장**이다
(CSV 에는 슬롯 이름이 안 들어간다).

대시보드 **Real Robot → 🧪 실험** (= `kp>0` 로 잘라낸 구간):

- 카드에 **정책 슬롯**과 판정이 칩으로 보인다
- 실험을 열면 **Trial — 정책과 판정**:
  - 정책 슬롯 (마커에서 자동, 없으면 직접)
  - 메모 한 줄
  - 4-tier 판정 — `★ perfect` · `○ ok` · `△ 아쉬움` · `✕ 실패`
    (학습 평가 화면과 **같은 어휘**다)
- `_plots/<실험>/notes.yaml` 에 저장 — 자동 계산된 `summary.json` 은 안 건드린다

⚠ 마커 없이 돌린 과거 실험은 「정책 미기록」으로 보인다. 직접 적을 수 있다.

---

## 5. 슬롯 인벤토리 규칙

`ONNX_META.json` 이 **슬롯마다 반드시** 있다. 최소한 이것:

| 항목 | 뜻 |
|---|---|
| `mode{1,2,3}_ckpt` · `flow_base_checkpoint` | 어느 학습에서 왔나 |
| `verify` | export 시 대조 수치 (parity · mask slice · initializer diff) |
| `retired` / `active_span` / `superseded_by` | 언제 활성이었고 무엇으로 대체됐나 |
| `requires` | 이 슬롯이 필요로 하는 C++ 기능 |
| `local_only` | (있으면) git 에 못 올리는 슬롯 |

**모르는 것은 「모른다」고 적는다.** 2026-08-27 에 옛 슬롯 7개의 META 를 git 이력에서
복원했는데, 아는 것은 배포 날짜·커밋뿐이라 체크포인트는 `(불명)` 으로 남겼다 — 지어내면
그게 다음 사람에게 «근거» 로 읽힌다.

### 🔴 100 MB 한도

`policy.onnx` 가 **100 MB 를 넘으면 GitHub 에 못 올린다**(LFS 미설정). 그런 슬롯은
`.gitignore` + `ONNX_META.json` 의 `local_only` 로 표시한다 — com1 sim2sim 은 되지만
**실기 배포는 불가**하다(로봇이 pull 로 못 받는다).
현재 해당: `v2_mode3_steps6` (177 MB).

---

## 6. 자주 걸리는 것

| 증상 | 원인 |
|---|---|
| `그런 슬롯이 없다` + 목록 | 오타이거나 아직 push/pull 안 됨. 목록이 같이 찍힌다 |
| `이 슬롯이 요구하는 기능을 모른다` | **재빌드가 필요하다.** 메시지가 명령을 준다 |
| `[obs contract] 불일치` | deploy.yaml 항 합 ≠ ONNX 입력. 슬롯이 깨졌다 |
| 기동 인터록 (`lowcmd` 점유) | 전원 새로 켜면 «ai» 가 잡는다 → 리모컨 `L2+R2`. ⚠ 로봇이 힘을 잃으니 매단 뒤에 |
| `[rt] ... 실패` | `setcap` 누락. 재빌드할 때마다 날아간다 |
| 즉시/전모드 낙상 | 1순위는 중복 `g1_ctrl` (`pgrep -x g1_ctrl`. `-f` 는 오탐) |

---

## 7. 되돌리기

```bash
# 그 자리에서: 직전 슬롯으로 다시 돌린다
./tools/run_g1_with_gui.sh eth0 --policy <직전 슬롯>

# 기본값 자체를 되돌린다 (com1)
#   config.yaml 의 policy_dir 한 줄. C++ 은 안 건드려도 된다 —
#   ModeGait 미지정 기본값이 종전 거동(V1 표·cadence 1.0·asym off)이라
#   옛 슬롯의 deploy.yaml 이 그대로 옛 거동을 낸다.
```

바이너리 자체를 되돌리려면 `robot.sh deploy` 가 남긴 `/tmp/g1_ctrl.backup_<날짜>`.
