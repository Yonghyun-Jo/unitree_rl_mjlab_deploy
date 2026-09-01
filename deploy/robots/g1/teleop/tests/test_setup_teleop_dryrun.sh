#!/usr/bin/env bash
# DRY_RUN 출력으로 «순서와 핀» 을 검사한다. 네트워크·설치 없음.
#
# 실제 repo 경로(deploy/robots/g1/teleop/)에서 바로 돌리지 않는다: 개발 머신에 이미
# <repo>/.gmr 이 있으면 스크립트가 clone 블록을 건너뛰어 bb1bbe4 / sparse-checkout set
# 검사가 "핀이 없어서"가 아니라 "이미 있어서 스킵돼서" 실패한다. 그래서 mktemp -d 로
# 격리된 임시 repo 뼈대를 만들고 그 안에서 돌린다 (.gmr/.venv-teleop 이 없는 상태 보장).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$HERE/../setup_teleop.sh"
REQUIREMENTS="$HERE/../requirements.txt"

T="$(mktemp -d)"
cleanup() { rm -rf "$T"; }
trap cleanup EXIT

mkdir -p "$T/deploy/robots/g1/teleop"
cp "$SETUP_SCRIPT" "$T/deploy/robots/g1/teleop/setup_teleop.sh"
cp "$REQUIREMENTS" "$T/deploy/robots/g1/teleop/requirements.txt"

OUT="$(DRY_RUN=1 TELEOP_PY=/usr/bin/python3 bash "$T/deploy/robots/g1/teleop/setup_teleop.sh" 2>&1)" || { echo "$OUT"; echo "FAIL: exit != 0"; exit 1; }
fail() { echo "$OUT"; echo "FAIL: $1"; exit 1; }
grep -q 'whl/cpu' <<<"$OUT"                          || fail "torch cpu index 없음"
grep -q 'torch==2.13.0' <<<"$OUT"                    || fail "torch 핀 없음"
grep -q 'bb1bbe4' <<<"$OUT"                          || fail "GMR 커밋 핀 없음"
grep -q 'sparse-checkout set' <<<"$OUT"              || fail "sparse checkout 없음"
grep -q 'assets/unitree_g1' <<<"$OUT"                || fail "unitree_g1 만 받는 sparse 목록 없음"
torch_line=$(grep -n 'whl/cpu' <<<"$OUT" | head -1 | cut -d: -f1)
gmr_line=$(grep -n 'pip install -e .*\.gmr' <<<"$OUT" | head -1 | cut -d: -f1)
[ -n "$torch_line" ] && [ -n "$gmr_line" ] && [ "$torch_line" -lt "$gmr_line" ] || fail "torch 가 GMR 보다 먼저 설치돼야 한다 (torch=$torch_line gmr=$gmr_line)"
echo "OK: setup_teleop dry-run order/pins"
