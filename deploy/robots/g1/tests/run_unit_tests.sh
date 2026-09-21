#!/usr/bin/env bash
# 독립 단위 테스트(헤더 온리, DDS·ONNX 무의존)를 전부 빌드·실행한다. 실패하면 종료코드 1.
# ONNX 가 필요한 test_obs_contract.cpp 와 Eigen 공용 헤더가 필요한 것은 각 파일 머리의 명령으로 따로 돌린다.
set -uo pipefail
cd "$(dirname "$0")"
OUT=$(mktemp -d); fail=0
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
for t in test_mode_table test_mode_runtime; do          # 이 계획이 더하는 것 (없으면 건너뜀)
  # -Werror=switch: Exit(등 enum class) 에 새 값이 생겼는데 ModeRuntime::request 의 switch 에
  # case 를 안 넣으면 여기서 빌드가 죽는다 (rules/ADDING_A_MODE.md 함정 (e)). 컨트롤러 본체
  # CMake 빌드는 이 경고를 켜지 않으므로 이 테스트가 유일한 기계 검증이다.
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include $t.cpp
done
[ -f test_no_mode_ordinals.sh ] && { bash test_no_mode_ordinals.sh && echo "ok   no_mode_ordinals" || { echo "FAIL no_mode_ordinals"; fail=1; }; }
exit $fail
