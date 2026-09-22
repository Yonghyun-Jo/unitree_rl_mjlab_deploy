# 모드를 하나 더하는 법 (mode7, mode8 …)

모드는 **표의 한 줄**이다. C++ 은 모드를 번호로 비교하지 않는다 — `tests/test_no_mode_ordinals.sh` 가 막는다.

## 1. 표 두 줄

| 어디 | 무엇 |
|---|---|
| 학습 repo `src/mjlab_g1_motion/mode_spec.py` `MODES` | `name` · `bits`(8칸 중 새 비트, `MASK_DIM=8`) · `track_upper`/`track_lower` · `base_vel_live` · `motion_preview` · `foot_z_live` · `mode5_cmd_live` · `crawl_cmd_live` |
| 배포 repo `deploy/robots/g1/config/modes.yaml` | `key` · `ref_source`(none/vr/clip) · `foot_z`(none/gen/ref) · `arm_blend_enter` · `crossfade_enter` · `safety`(upright_only/ground_capable) · `exit`(always/upright/standing_hold/via_ground) · `gait` |

모드 번호는 1부터 빈칸 없이. 두 표의 모드 집합이 다르면 생성이 그 자리에서 죽는다.
`gait` 열은 **키일 뿐**이다 — `deploy.yaml` 의 어느 `gait:` 하위블록을 읽을지 고른다. 그 블록 자체는 여기 없다: `foot_z: gen` 모드라면 각 슬롯의 `params/deploy.yaml` 에 **따로** 채워야 한다(3절).

```bash
python3 deploy/scripts/gen_mode_table_header.py --write     # ModeTable.h 재생성. 두 표가 모순이면 여기서 실패한다
bash deploy/robots/g1/tests/run_unit_tests.sh
```
`--write` 대신 `--check` 로 «헤더가 원장과 같은가» 만 볼 수도 있다(CI 용, 파일을 안 바꾼다). `--mjlab <경로>` 로 다른 mjlab 워크트리의 `mode_spec.py` 를 가리킬 수 있다 — 기본값은 `~/mjlab1.4/mjlab_g1_mode45` 다.

생성기가 실제로 막는 모순 셋: `foot_z: none` ⇔ 학습 `foot_z_live=False`, `ref_source: none` ⇔ `track_upper`/`track_lower` 둘 다 False, `motion_preview=True` 면 `ref_source` 는 반드시 `clip`. 이 셋 밖의 모순(예: `safety`/`exit` 조합이 이상함)은 생성기가 안 잡는다 — 리뷰가 봐야 한다.

## 2. 코드가 필요한가

| 새 모드의 명령 | 필요한 코드 |
|---|---|
| 속도 · 관절 참조(VR/클립) · 미리보기(**구현됨** — `include/MotionPreview.h`) · 자세 버튼(**구현됨** — `include/Mode5Driver.h`)의 조합(기존 4종 안) | **없음.** 표 두 줄 + 재생성으로 끝 |
| 새 «종류» 의 명령 (기존 4종 밖) | 지금은 `CommandSources.h` 가 **없다.** 명령 값은 오늘은 `State_Mimic.cpp` 관측 항 함수들이 `g_mode.row()`(모드 성질 분기)와 `active_ref_loader()`(VR vs 클립 로더 선택)로 직접 읽는다 — 새 종류를 더하려면 여기에 관측 항을 손으로 추가하고 `modes.yaml` 에 그 종류를 고르는 열을 하나 더한다. 이 직접-분기들을 클래스로 모으는 일은 B2 가 하지 않았고 지금 계획에도 없다 — 함수 분기 그대로다. 새 종류가 넘어짐 관문에 «지금 서 있어야 하나» 를 말할 수 있으면 `SafetyPolicy.h` `commanded_upright` 에도 그 성질의 분기를 더한다(아래 함정 (f)) |
| 새 이탈 조건 | `ModeTable` 의 `Exit` enum 값 하나(`gen_mode_table_header.py` `ENUMS["exit"]`) + `modes.yaml` `exit` 열 + `ModeRuntime::request` 의 **두** `switch` — 나가는 쪽 `switch (row().exit)` 와 들어가는 쪽 `switch (mode_table::row(m).exit)` — 에 `case` 하나씩 + 테스트. 빠뜨리면 그 모드에서 **못 나가거나 못 들어간다**(둘 다 기본이 거부 — 아래 함정 (e)). `exit: upright` 는 들어가는 조건도 된다(직립에서만 들어간다) |

## 3. 슬롯

새 모드를 아는 ONNX 슬롯의 `deploy.yaml` 에 `modes: [1, 2, …, N]`. 없으면 계약 v1 = `{1,2,3,4}` 로 보고(`State_Mimic.h` 기본값) 새 모드 요청을 거부한다. 이 목록은 `enter()` 에서 `g_mode.set_supported()` 로 매 체류마다 다시 건다.

헤드 수·게이트는 ONNX 그래프 안에 있다 — C++ 은 헤드가 몇 개인지 모른다.

**`foot_z: gen` 모드는 그 모드를 아는 모든 슬롯의 `params/deploy.yaml` 에 `gait: <gait_key>: {...}` 블록이 있어야 한다** (그 head 를 학습시킨 시점의 FOOT_GEN 표에서 값을 옮긴다). `load_gait_cfg` 는 그 키가 없으면 **조용히 `continue`** 해 quintic/table=1(V1) 기본값으로 돈다 — 생성기도 테스트도 이걸 안 잡는다. 기동 로그의 `[gait] modeN: source=... table=V... ` 줄이 그 모드에 대해 찍혔는지로만 확인할 수 있다; 없으면 기본값으로 돈 것이다.

새 모드를 어느 채널로 요청할 수 있게 할지는 따로 정한다:
- **키보드**: 표의 `key` 로 자동 — 새 행을 채우면 그 키로 바로 요청된다(`State_Mimic.cpp` 가 `mode_table::row(m).key` 로 순회).
- **클립 고르기**: `[` / `]` 가 `g_clips` 를 순회한다(코드 불필요). 단 **`ref_source: clip` 인 모드에 있는 동안은 거부**된다(한 줄 남기고 아무 일도 안 일어난다) — 클립을 «재생 중에» 갈면 참조가 crossfade·다리 램프·되감기·재앵커 없이 점프한다. 고르는 것은 그 모드에 **들어가기 전**이다.
- **GUI(`/dev/shm/g1_masked_gui`, v2 magic 0x6704)**: `g_channel_may_request(m, Channel::Gui)` 가 표의 **모든** 모드를 통과시킨다(사람이 화면을 보고 누르는 버튼 — 슬롯이 아는가·지금 나갈 수 있는가는 `ModeRuntime` 이 본다). 모드 버튼은 `masked_gui.py` 가 생성된 `tools/mode_table_gen.py` `MODES` 로 만든다 — **새 행을 채우면 GUI 버튼도 코드 없이 생긴다.** 요청은 1회성 `mode_req`(0 = 없음)다.
- **VR(`/dev/shm/g1_vr_ref`)**: `g_channel_may_request(m, Channel::Vr)` 가 표에서 `safety: upright_only` 인 모드만 통과시킨다(옛 `1 <= cmd_mode <= 3` 검사를 성질로 옮긴 것 — 남의 프로그램이 50 Hz 로 쓰는 바이트라 쓰레기 값이 저자세·클립재생 모드를 켜지 못하게 하는 필터이기도 하다). 즉 새 모드를 `upright_only` 로 선언하면 **그 표 행이 생기는 순간 VR 에서도 바로 눌린다** — `safety` 를 고를 때 이걸 의식할 것. `ground_capable` 모드를 VR 에서 누르게 하려면 `Channel::Vr` 분기를 **의도적으로** 넓혀야 한다.

## 4. 확인

`test_mode_table.cpp` 에 새 행의 성질 한 줄 · sim2sim 에서 진입/이탈 · 실기 전 `/preflight-g1-real`.

## 5. 함정

- **(a) 슬롯의 `modes:` 에 폴백(mode1)을 빼먹지 않는다.** qd 안전가드는 `g_mode.requested() == mode1` 일 때만 래치를 푸는데, `requested()` 는 `ModeRuntime::request()` 가 `supports()` 를 통과해야만 갱신된다. `modes:` 에 1이 없으면 `force()` 로 로봇은 안전하게 mode1 에 묶이지만(이건 `supports()` 를 안 본다), 조작자가 키 `1` 을 눌러도 `request(1)` 이 "이 슬롯이 모르는 모드" 로 거부되어 **래치가 영원히 안 풀린다.**
- **(b) `ModeTable.h` 를 손으로 고치지 않는다.** 생성 파일이다 — 고치면 다음 `--write`/`--check` 가 조용히 덮어쓰거나 원장과 어긋난 채로 빌드된다.
- **(c) 두 표가 서로 맞아야 생성이 된다.** `foot_z: none` ⇔ `foot_z_live=False`, `ref_source: none` ⇔ 추종 없음, `motion_preview` ⇒ `ref_source: clip` — 어긋나면 `gen_mode_table_header.py` 가 그 자리에서 죽는다(1절 참고).
- **(d) `test_no_mode_ordinals.sh` 는 파서가 아니라 그렙이다.** `cmd_mode`/`new_mode`/`g_mode.mode()`/`.requested()` 를 리터럴로 숫자와 비교하는 패턴만 잡는다. `int m = g_mode.mode(); if (m >= 2) …` 처럼 지역 변수 뒤에 숨은 비교는 통과한다 — 그렙이 초록이어도 리뷰가 직접 봐야 한다.
- **(e) 새 `Exit` 는 `ModeRuntime::request` 의 두 `switch`(나가는 쪽·들어가는 쪽) 에 `case` 를 반드시 추가한다.** 두 스위치 다 `default:` 가 없다. 단위 테스트(`run_unit_tests.sh`)가 `test_mode_table`/`test_mode_runtime` 를 `-Werror=switch` 로 빌드하므로 빠뜨리면 **거기서 빌드가 실패한다**(경고가 아니라 에러). 🔴 컨트롤러 자체의 CMake 빌드는 이 경고를 켜지 않는다(`-Wall`/`-Wswitch` 없음) — 그래도 놓치면 런타임은 `ok=false`(기본값)로 **그 모드에서 나가는 모든 전환을 거부**한다(fail-closed, 조용히 통과되지 않는다). 들어가는 쪽 스위치도 같다 — 빠뜨리면 그 모드로 **들어가는** 전환을 전부 거부한다.
- **(f) `safety: ground_capable` 을 주면 넘어짐 판정(기울기 > 57.3° → Passive)의 관문이 «명령» 에서 열린다(`SafetyPolicy.h`).** 클립 재생(`ref_source: clip`) = 클립 현재 프레임 골반 기울기 `< 57.3°` 일 때(최근 이력을 묻지 않는다) · mode5 자세 명령(`mode5_cmd_live`) = 자세의 목표 높이 `≥ 0.65` ∧ 최근 1 s 에 섰음 · **그 밖(명령에 자세가 없는 모드 — `ref_source ≠ clip` ∧ `!mode5_cmd_live`)은 넘어짐 판정이 아예 안 걸린다**(`commanded_upright` 가 늘 «아님»). 다음 후보 mode6(기기, 골반 기울기 64°)이 바로 이 경우다 — 그 모드의 넘어짐 보호는 qd_warn(낮거나 기운 자세면 Passive)·qd_crit·E-stop 뿐이다. 명령이 «지금 서 있어야 하나» 를 말할 수 있으면 `commanded_upright` 에 그 성질의 분기를 더한다(번호 아님). 그리고 `ground_capable` ∧ `!mode5_cmd_live` 인 모드로는 Mimic 체류를 시작하지 않는다(`ModeRuntime::begin_stay` — `p→f→m` 재진입은 폴백 모드로 내린다).
