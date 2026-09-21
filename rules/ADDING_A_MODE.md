# 모드를 하나 더하는 법 (mode7, mode8 …)

모드는 **표의 한 줄**이다. C++ 은 모드를 번호로 비교하지 않는다 — `tests/test_no_mode_ordinals.sh` 가 막는다.

## 1. 표 두 줄

| 어디 | 무엇 |
|---|---|
| 학습 repo `src/mjlab_g1_motion/mode_spec.py` `MODES` | `name` · `bits`(8칸 중 새 비트, `MASK_DIM=8`) · `track_upper`/`track_lower` · `base_vel_live` · `motion_preview` · `foot_z_live` · `mode5_cmd_live` · `crawl_cmd_live` |
| 배포 repo `deploy/robots/g1/config/modes.yaml` | `key` · `ref_source`(none/vr/clip) · `foot_z`(none/gen/ref) · `arm_blend_enter` · `crossfade_enter` · `safety`(upright_only/ground_capable) · `exit`(always/upright/standing_hold/via_ground) · `gait` |

모드 번호는 1부터 빈칸 없이. 두 표의 모드 집합이 다르면 생성이 그 자리에서 죽는다.

```bash
python3 deploy/scripts/gen_mode_table_header.py --write     # ModeTable.h 재생성. 두 표가 모순이면 여기서 실패한다
bash deploy/robots/g1/tests/run_unit_tests.sh
```
`--write` 대신 `--check` 로 «헤더가 원장과 같은가» 만 볼 수도 있다(CI 용, 파일을 안 바꾼다). `--mjlab <경로>` 로 다른 mjlab 워크트리의 `mode_spec.py` 를 가리킬 수 있다 — 기본값은 `~/mjlab1.4/mjlab_g1_mode45` 다.

생성기가 실제로 막는 모순 셋: `foot_z: none` ⇔ 학습 `foot_z_live=False`, `ref_source: none` ⇔ `track_upper`/`track_lower` 둘 다 False, `motion_preview=True` 면 `ref_source` 는 반드시 `clip`. 이 셋 밖의 모순(예: `safety`/`exit` 조합이 이상함)은 생성기가 안 잡는다 — 리뷰가 봐야 한다.

## 2. 코드가 필요한가

| 새 모드의 명령 | 필요한 코드 |
|---|---|
| 속도 · 관절 참조(VR/클립) · 미리보기 · 자세 버튼의 조합(기존 4종 안) | **없음.** 표 두 줄 + 재생성으로 끝 |
| 새 «종류» 의 명령 (기존 4종 밖) | 지금은 `CommandSources.h` 가 **없다.** 명령 값은 오늘은 `State_Mimic.cpp` 관측 항 함수들이 `g_mode.row()`(모드 성질 분기)와 `active_ref_loader()`(VR vs 클립 로더 선택)로 직접 읽는다 — 새 종류를 더하려면 여기에 관측 항을 손으로 추가하고 `modes.yaml` 에 그 종류를 고르는 열을 하나 더한다. **B2 계획이 이 직접-분기들을 `CommandSources.h` 클래스로 모을 예정**(이 문서 밖의 다음 단계) — 그 전까지는 클래스가 아니라 함수 분기다 |
| 새 이탈 조건 | `ModeTable` 의 `Exit` enum 값 하나(`gen_mode_table_header.py` `ENUMS["exit"]`) + `modes.yaml` `exit` 열 + `ModeRuntime::request` 의 `switch (row().exit)` 에 `case` 하나 + 테스트. 빠뜨리면 그 모드에서 **못 나간다**(기본이 거부 — 아래 함정 (e)) |

## 3. 슬롯

새 모드를 아는 ONNX 슬롯의 `deploy.yaml` 에 `modes: [1, 2, …, N]`. 없으면 계약 v1 = `{1,2,3,4}` 로 보고(`State_Mimic.h` 기본값) 새 모드 요청을 거부한다. 이 목록은 `enter()` 에서 `g_mode.set_supported()` 로 매 체류마다 다시 건다.

헤드 수·게이트는 ONNX 그래프 안에 있다 — C++ 은 헤드가 몇 개인지 모른다.

새 모드를 어느 채널로 요청할 수 있게 할지는 따로 정한다:
- **키보드**: 표의 `key` 로 자동 — 새 행을 채우면 그 키로 바로 요청된다(`State_Mimic.cpp` 가 `mode_table::row(m).key` 로 순회).
- **클립 고르기**: `[` / `]` 는 모드와 무관하게 `g_clips` 를 순회한다(코드 불필요).
- **GUI(`/dev/shm/g1_masked_gui`)·VR**: `g_channel_may_request()` 가 **오늘은 `safety == upright_only` 인 모드만** 통과시킨다(옛 `1 <= cmd_mode <= 3` 검사를 성질로 옮긴 것). 새 모드를 GUI/VR 에서도 누르게 하려면 이 필터를 **의도적으로** 넓혀야 한다 — 저절로 넓어지지 않는다.

## 4. 확인

`test_mode_table.cpp` 에 새 행의 성질 한 줄 · sim2sim 에서 진입/이탈 · 실기 전 `/preflight-g1-real`.

## 5. 함정

- **(a) 슬롯의 `modes:` 에 폴백(mode1)을 빼먹지 않는다.** qd 안전가드는 `g_mode.requested() == mode1` 일 때만 래치를 푸는데, `requested()` 는 `ModeRuntime::request()` 가 `supports()` 를 통과해야만 갱신된다. `modes:` 에 1이 없으면 `force()` 로 로봇은 안전하게 mode1 에 묶이지만(이건 `supports()` 를 안 본다), 조작자가 키 `1` 을 눌러도 `request(1)` 이 "이 슬롯이 모르는 모드" 로 거부되어 **래치가 영원히 안 풀린다.**
- **(b) `ModeTable.h` 를 손으로 고치지 않는다.** 생성 파일이다 — 고치면 다음 `--write`/`--check` 가 조용히 덮어쓰거나 원장과 어긋난 채로 빌드된다.
- **(c) 두 표가 서로 맞아야 생성이 된다.** `foot_z: none` ⇔ `foot_z_live=False`, `ref_source: none` ⇔ 추종 없음, `motion_preview` ⇒ `ref_source: clip` — 어긋나면 `gen_mode_table_header.py` 가 그 자리에서 죽는다(1절 참고).
- **(d) `test_no_mode_ordinals.sh` 는 파서가 아니라 그렙이다.** `cmd_mode`/`new_mode`/`g_mode.mode()`/`.requested()` 를 리터럴로 숫자와 비교하는 패턴만 잡는다. `int m = g_mode.mode(); if (m >= 2) …` 처럼 지역 변수 뒤에 숨은 비교는 통과한다 — 그렙이 초록이어도 리뷰가 직접 봐야 한다.
- **(e) 새 `Exit` 는 `ModeRuntime::request` 의 `switch` 에 `case` 를 반드시 추가한다.** 그 스위치는 `default:` 가 없다 — 새 `Exit` 값을 빠뜨리면 컴파일러가 `-Wswitch` 로 경고하고, 그래도 놓치면 런타임은 `ok=false`(기본값)로 **그 모드에서 나가는 모든 전환을 거부**한다(fail-closed, 조용히 통과되지 않는다).
