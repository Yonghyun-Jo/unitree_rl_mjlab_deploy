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
# StateDump 는 쓰기 스레드(std::thread)가 있어 링크에 -pthread 가 필요하다 — 위 루프와 갈라 둔다.
# Mimic 재진입(stay 를 여러 번 도는 것)에서 close() 없이 다시 열면 std::terminate 가 나던
# 회귀(2026-09-22)를 여기서 잡는다.
for t in test_state_dump_writer; do
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -I../include -pthread $t.cpp
done
# SafetyLog 도 같은 프로세스-공유 싱글턴 계약(멱등 open, 닫기는 소멸자에서만)을 잠근다 — 2026-09-22
# Ruling 30. 스레드가 없어 -pthread 는 불필요.
for t in test_safety_log_shared; do
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -I../include $t.cpp
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
for t in test_height_estimator test_motion_preview test_safety_policy; do   # Eigen 이 필요한 순수 헤더 테스트 (-Werror=switch 유지)
  [ -f $t.cpp ] && run $t -std=gnu++17 -O2 -Wall -Wextra -Werror=switch -I../include -I/usr/include/eigen3 $t.cpp
done
[ -f test_no_mode_ordinals.sh ] && { bash test_no_mode_ordinals.sh && echo "ok   no_mode_ordinals" || { echo "FAIL no_mode_ordinals"; fail=1; }; }
# shm 바이트 배치(C++ 구조체 ↔ python FMT) + policy_slot push 게이트 + 헤드리스 도구의 t=0 판정 — 표준 라이브러리만 쓴다(uv 불필요)
for t in test_gui_shm_layout test_vr_shm_layout test_policy_slot_gate test_headless_pre_t0 test_masked_gui_posture_rows test_gui_clip_order; do
  [ -f $t.py ] && { python3 $t.py >"$OUT/$t.out" 2>&1 && echo "ok   $t" || { echo "FAIL $t"; tail -5 "$OUT/$t.out"; fail=1; }; }
done

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
  for g in gen_mode5_presets_header gen_g1_kinematics_header gen_prim_scene; do
    if "${UVPY[@]}" ../../../scripts/$g.py --check >"$OUT/$g.out" 2>&1; then echo "ok   $g"
    else echo "FAIL $g"; tail -5 "$OUT/$g.out"; fail=1; fi
  done
  # 구·캡슐 충돌 장면(scene_g1_prim.xml)이 학습 충돌 33개와 같고 액추에이터·센서가 scene_g1.xml 과 같은가
  if "${UVPY[@]}" test_prim_scene.py >"$OUT/prim.out" 2>&1; then echo "ok   test_prim_scene"
  else echo "FAIL test_prim_scene"; tail -5 "$OUT/prim.out"; fail=1; fi
  # gen_preview_golden 은 학습 repo 의 클립 npz(COLMOv2/dance/dance1_subject2.npz, git 밖)가 있어야 --check 가
  # 된다. 없는 머신(로봇·깨끗한 clone)에서는 생성기 스스로가 "클립이 없다" 로 끝내므로 그 경우만 skip 으로 통과.
  if "${UVPY[@]}" ../../../scripts/gen_preview_golden.py --check >"$OUT/pv.out" 2>&1; then echo "ok   gen_preview_golden"
  elif grep -q "클립이 없다" "$OUT/pv.out"; then echo "skip gen_preview_golden (클립 npz 없음)"
  else echo "FAIL gen_preview_golden"; tail -5 "$OUT/pv.out"; fail=1; fi
else
  echo "skip uv 생성기 검사 (mjlab uv 환경 없음)"
fi

# 계약 v2 슬롯의 deploy.yaml `observations:` 블록 == gen_obs_block.py(그 슬롯 ONNX 의 계약) — 배포 항 이름 ↔ 학습 항
# 매핑이 틀린 것은 C++ 기동 대조가 못 잡는다(최종 검토 M-5). ONNX 는 git 밖이라 exported/policy.onnx 가 있는 슬롯만,
# onnx 를 import 하는 파이썬(시스템 python3, 없으면 mjlab uv)이 없으면 skip. 계약 v1 슬롯의 블록은 손으로 쓴 것이라 대상 밖.
OBSPY=()
if command -v python3 >/dev/null 2>&1 && python3 -c "import onnx" >/dev/null 2>&1; then OBSPY=(python3)
elif [ -x "$HOME/.local/bin/uv" ] && [ -d "$MJLAB" ] && "${UVPY[@]}" -c "import onnx" >/dev/null 2>&1; then OBSPY=("${UVPY[@]}")
fi
n_obs=0; n_skip=0
for onnx_f in ../config/policy/*/*/exported/policy.onnx; do
  [ -f "$onnx_f" ] || continue
  slot=$(dirname "$(dirname "$onnx_f")")
  [ -f "$slot/ONNX_META.json" ] && [ -f "$slot/params/deploy.yaml" ] || continue
  command -v python3 >/dev/null 2>&1 || continue
  python3 -c 'import json,sys; sys.exit(0 if str(json.load(open(sys.argv[1])).get("obs_contract_version")) == "2" else 1)' \
    "$slot/ONNX_META.json" 2>/dev/null || continue
  name="obs_block_generated[$(basename "$slot")]"
  if [ ${#OBSPY[@]} -eq 0 ]; then echo "skip $name (onnx 를 import 하는 파이썬 없음)"; n_skip=$((n_skip + 1)); continue; fi
  n_obs=$((n_obs + 1))
  if "${OBSPY[@]}" ../../../scripts/gen_obs_block.py "$onnx_f" --check "$slot/params/deploy.yaml" >"$OUT/obs.out" 2>&1
  then echo "ok   $name"
  else echo "FAIL $name"; tail -5 "$OUT/obs.out"; fail=1; fi
done
[ $n_obs -eq 0 ] && [ $n_skip -eq 0 ] && echo "skip obs_block_generated (exported/policy.onnx 가 있는 계약 v2 슬롯 없음)"
exit $fail
