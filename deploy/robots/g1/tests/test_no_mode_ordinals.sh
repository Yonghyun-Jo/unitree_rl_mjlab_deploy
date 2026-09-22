#!/usr/bin/env bash
# 모드를 «번호» 로 비교하는 코드가 g1 제어기에 남아 있으면 실패한다 (ModeTable 의 성질을 물을 것).
cd "$(dirname "$0")/.."
# 모드를 다루는 순수 헤더(B2: ModeRuntime · SafetyPolicy · Mode5Driver · MotionPreview · HeightEstimator)도 같은 규칙이다.
files="src/State_Mimic.cpp include/MaskedLocoController.h include/State_Mimic.h include/ModeRuntime.h include/SafetyPolicy.h
       include/Mode5Driver.h include/MotionPreview.h include/HeightEstimator.h"
# 파일이 없어지거나 이름이 바뀌면 grep 이 조용히 빈 결과를 내서 «통과» 가 된다 — 먼저 막는다.
for f in $files; do
  [ -f "$f" ] || { echo "검사 대상이 없다: $f (이름이 바뀌었으면 이 목록을 고칠 것)"; exit 1; }
done
pat='(cmd_mode|new_mode|g_mode\.mode\(\)|\.requested\(\))[[:space:]]*(>=|<=|==|!=|>|<)[[:space:]]*[0-9]'
# grep 출력이 "파일:줄:" 로 시작하므로 주석 제외는 그 접두 뒤를 본다.
hits=$(grep -nE "$pat" $files | grep -vE ':[0-9]+:[[:space:]]*//')
if [ -n "$hits" ]; then echo "모드 번호 비교가 남아 있다:"; echo "$hits"; exit 1; fi
exit 0
