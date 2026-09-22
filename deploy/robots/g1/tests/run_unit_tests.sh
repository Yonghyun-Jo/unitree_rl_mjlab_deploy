#!/usr/bin/env bash
# 독립 단위 테스트(헤더 온리, DDS·ONNX 무의존)를 전부 빌드·실행한다. 실패하면 종료코드 1.
# ONNX 가 필요한 test_obs_contract.cpp 와 Eigen 공용 헤더가 필요한 것은 각 파일 머리의 명령으로 따로 돌린다.
set -uo pipefail
cd "$(dirname "$0")"
OUT=$(mktemp -d); fail=0
trap 'rm -rf "$OUT"' EXIT       # 실패 경로로 나가도 /tmp 에 빌드 찌꺼기를 남기지 않는다
run() {  # run <name> <g++ 인자...>
  local n=$1; shift
  if g++ "$@" -o "$OUT/$n" 2>"$OUT/$n.log" && "$OUT/$n" >"$OUT/$n.out" 2>&1; then echo "ok   $n"
  else echo "FAIL $n"; tail -5 "$OUT/$n.log" "$OUT/$n.out" 2>/dev/null; fail=1; fi
}
for t in test_gait_lut test_gait_lut_v2 test_gait_min_swing test_joint_safety test_loco_gait_modes \
         test_masked_loco_controller test_settle_stop test_deploy_features; do
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -I../include $t.cpp
done
for t in test_estop_channel test_loop_diag; do
  [ -f $t.cpp ] && run $t -std=c++17 -O2 -I../../../include $t.cpp
done
for t in test_mode_table test_mode_runtime test_mode5_driver; do          # 이 계획이 더하는 것 (없으면 건너뜀)
  # -Werror=switch: Exit(등 enum class) 에 새 값이 생겼는데 ModeRuntime::request 의 switch 에
  # case 를 안 넣으면 여기서 빌드가 죽는다 (rules/ADDING_A_MODE.md 함정 (e)). 컨트롤러 본체
  # CMake 빌드는 이 경고를 켜지 않으므로 이 테스트가 유일한 기계 검증이다.
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include $t.cpp
done
for t in test_height_estimator test_motion_preview; do            # Eigen 이 필요한 순수 헤더 테스트
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include -I/usr/include/eigen3 $t.cpp
done
[ -f test_no_mode_ordinals.sh ] && { bash test_no_mode_ordinals.sh && echo "ok   no_mode_ordinals" || { echo "FAIL no_mode_ordinals"; fail=1; }; }

# 생성된 ModeTable.h 가 원장(mode_spec.py + config/modes.yaml)과 같은가 — 손으로 고친 헤더·한쪽만
# 고친 표를 잡는다. 🔴 원장의 «학습 쪽»(mode_spec.py)은 다른 repo 에 있어 로봇·깨끗한 clone 에는
# 아예 없다. 거기서 실패로 치면 배포 전 점검이 통째로 빨개지므로, 없으면 skip 하고 통과시킨다
# (없다는 사실은 줄로 남는다). 생성기 자체의 «파일 없음 = 실패» 거동은 그대로 둔다 — 존재 판정은
# 여기, 실행은 저기.
MJLAB="${G1_MJLAB:-$HOME/mjlab1.4/mjlab_g1_mode45}"
if [ -f "$MJLAB/src/mjlab_g1_motion/mode_spec.py" ] \
   && command -v python3 >/dev/null 2>&1 && python3 -c "import yaml" >/dev/null 2>&1; then
  if python3 ../../../scripts/gen_mode_table_header.py --check --mjlab "$MJLAB" >"$OUT/gen.out" 2>&1
  then echo "ok   mode_table_generated"
  else echo "FAIL mode_table_generated"; tail -5 "$OUT/gen.out"; fail=1
  fi
else
  echo "skip mode_table_generated (mode_spec.py 없음)"
fi

# torch·mujoco 가 필요한 생성기(mode5 프리셋·기구학·미리보기 골든)의 --check. mjlab 의 uv 환경이 있을 때만.
# 🔴 conda 가 켜진 셸에서는 LD_LIBRARY_PATH 때문에 uv 의 torch 가 깨진다 → 그 변수들을 뺀 환경으로 돈다.
UVPY=(env -u LD_LIBRARY_PATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV "$HOME/.local/bin/uv" run --project "$MJLAB" --no-sync python)
if [ -x "$HOME/.local/bin/uv" ] && [ -f "$MJLAB/src/mjlab_g1_motion/mode5_presets.py" ]; then
  for g in gen_mode5_presets_header gen_g1_kinematics_header; do
    if "${UVPY[@]}" ../../../scripts/$g.py --check >"$OUT/$g.out" 2>&1; then echo "ok   $g"
    else echo "FAIL $g"; tail -5 "$OUT/$g.out"; fail=1; fi
  done
  # gen_preview_golden 은 배포 슬롯 npz(git 밖)가 있어야 --check 가 된다. 없는 머신(로봇·깨끗한
  # clone)에서는 생성기 스스로가 "클립이 없다" 로 끝내므로 그 경우만 skip 으로 통과시킨다.
  if "${UVPY[@]}" ../../../scripts/gen_preview_golden.py --check >"$OUT/pv.out" 2>&1; then echo "ok   gen_preview_golden"
  elif grep -q "클립이 없다" "$OUT/pv.out"; then echo "skip gen_preview_golden (클립 npz 없음)"
  else echo "FAIL gen_preview_golden"; tail -5 "$OUT/pv.out"; fail=1; fi
else
  echo "skip uv 생성기 검사 (mjlab uv 환경 없음)"
fi
exit $fail
