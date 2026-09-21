#!/usr/bin/env bash
# 모드를 «번호» 로 비교하는 코드가 g1 제어기에 남아 있으면 실패한다 (ModeTable 의 성질을 물을 것).
cd "$(dirname "$0")/.."
pat='(cmd_mode|new_mode|g_mode\.mode\(\)|\.requested\(\))[[:space:]]*(>=|<=|==|!=|>|<)[[:space:]]*[0-9]'
hits=$(grep -nE "$pat" src/State_Mimic.cpp include/MaskedLocoController.h include/State_Mimic.h | grep -v '^\s*//' | grep -vE ':[0-9]+:\s*//')
if [ -n "$hits" ]; then echo "모드 번호 비교가 남아 있다:"; echo "$hits"; exit 1; fi
exit 0
