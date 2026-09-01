#!/usr/bin/env bash
# bootstrap.sh — G1 온보드(Jetson) 에서 이 repo 를 «돌 수 있는 상태» 로 만든다. 멱등.
#
#   ① 사전점검  ② .deps  ③ g1_ctrl(Release)  ④ python 3.10(uv)  ⑤ teleop 최소 환경  ⑥ CLAUDE.md  ⑦ 자기검증
#
# 규칙: sudo 를 실행하지 않는다(공용 기계) — 필요한 apt 는 명령만 출력하고 중단.
#       piene_ws 밖에 만드는 것은 uv 실행파일(~/.local/bin)뿐. 3.10 인터프리터는 $PIENE_WS/.uv-python 에.
# 사용: bash deploy/onboard/bootstrap.sh            # 전부
#       bash deploy/onboard/bootstrap.sh --check    # ① 만
#       bash deploy/onboard/bootstrap.sh --only 5   # 한 단계만
#       bash deploy/onboard/bootstrap.sh --skip-teleop
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PIENE_WS="$(cd "$REPO/.." && pwd)"
ONLY=""; SKIP_TELEOP=""; CHECK_ONLY=""
while [ $# -gt 0 ]; do case "$1" in
  --check) CHECK_ONLY=1 ;; --only) ONLY="$2"; shift ;; --skip-teleop) SKIP_TELEOP=1 ;; --ws) PIENE_WS="$2"; shift ;;
  *) echo "unknown arg $1"; exit 1 ;; esac; shift; done
step() { echo; echo "━━ $1 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }
want() { [ -z "$ONLY" ] || [ "$ONLY" = "$1" ]; }
die()  { echo "🔴 $*"; exit "${2:-1}"; }
G1="$REPO/deploy/robots/g1"
export UV_PYTHON_INSTALL_DIR="$PIENE_WS/.uv-python"

# ── ① 사전점검 ─────────────────────────────────────────────────────────────
if want 1; then
  step "① 사전점검  repo=$REPO  piene_ws=$PIENE_WS"
  [ "$(uname -m)" = "aarch64" ] || die "aarch64 전용 (현재 $(uname -m)). com1 에서는 sim2sim 만 — rules/RUNBOOK_g1_from_scratch.md Phase 4" 2
  glibc="$(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$')"
  if awk -v g="$glibc" 'BEGIN{split(g,a,"."); exit !(a[1]>2 || (a[1]==2 && a[2]>=34))}'; then
    echo "  glibc $glibc ≥ 2.34 → PC-Service/xrt 로드 가능: --transport local 후보 (rules/RUNBOOK_onboard_pico_teleop.md)"
  else
    echo "  glibc $glibc < 2.34 → xrt 불가: 실기는 노트북 publisher → 온보드 bridge --transport udp (rules/RUNBOOK_laptop_pico_teleop.md)"
  fi
  missing=""
  for p in cmake build-essential libboost-all-dev libyaml-cpp-dev zlib1g-dev libfmt-dev libeigen3-dev git curl; do
    dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"
  done
  [ -z "$missing" ] || die "apt 패키지 없음:$missing
  → 사람이 실행: sudo apt install -y$missing   (스크립트는 sudo 를 부르지 않는다)" 2
  timeout 8 git ls-remote https://github.com/YanjieZe/GMR.git HEAD >/dev/null 2>&1 || die "github 접근 불가 — 인터넷 필요 (GMR·torch·uv 다운로드)" 2
  echo "  🟢 aarch64 · apt deps · github"
  [ -z "$CHECK_ONLY" ] || exit 0
fi

# ── ② .deps ───────────────────────────────────────────────────────────────
if want 2; then
  step "② .deps (unitree_sdk2 + CycloneDDS, 네이티브 빌드)"
  if [ -f "$REPO/.deps/lib/libunitree_sdk2.a" ]; then echo "  이미 있음"; else bash "$REPO/deploy/scripts/build_deps.sh"; fi
fi

# ── ③ g1_ctrl ─────────────────────────────────────────────────────────────
if want 3; then
  step "③ g1_ctrl — Release (로봇 -O3 / com1 -O0 로 갈렸던 사고 재발 방지)"
  mkdir -p "$G1/build"
  ( cd "$G1/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && nice make -j4 2>&1 | tail -2 )
  [ -x "$G1/build/g1_ctrl" ] || die "g1_ctrl 빌드 실패"
  grep -q "O3" "$G1"/build/CMakeFiles/g1_ctrl.dir/flags.make || die "Release 플래그가 아니다: $(grep CXX_FLAGS "$G1"/build/CMakeFiles/g1_ctrl.dir/flags.make)"
  echo "  🟢 $G1/build/g1_ctrl   (setcap 은 robot.sh deploy 가 한다: sudo setcap cap_sys_nice+ep g1_ctrl)"
fi

# ── ④ python 3.10 ─────────────────────────────────────────────────────────
if want 4 && [ -z "$SKIP_TELEOP" ]; then
  step "④ python 3.10 (uv)  → $UV_PYTHON_INSTALL_DIR"
  if ! command -v uv >/dev/null 2>&1; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
  fi
  uv python install 3.10 >/dev/null
  TELEOP_PY="$(uv python find 3.10)"
  echo "  🟢 $TELEOP_PY"
fi

# ── ⑤ teleop ──────────────────────────────────────────────────────────────
if want 5 && [ -z "$SKIP_TELEOP" ]; then
  step "⑤ teleop 최소 환경 (setup_teleop.sh)"
  export PATH="$HOME/.local/bin:$PATH"
  TELEOP_PY="${TELEOP_PY:-$(uv python find 3.10)}" bash "$G1/teleop/setup_teleop.sh"
fi

# ── ⑥ CLAUDE.md ───────────────────────────────────────────────────────────
if want 6; then
  step "⑥ piene_ws/CLAUDE.md"
  src="$HERE/CLAUDE.robot.md"; dst="$PIENE_WS/CLAUDE.md"
  if [ ! -f "$src" ]; then echo "  ⚠ $src 없음 — 건너뜀"
  elif [ ! -f "$dst" ]; then cp "$src" "$dst"; echo "  🟢 생성 $dst"
  elif diff -q "$src" "$dst" >/dev/null; then echo "  이미 같음"
  else echo "  ⚠ $dst 가 repo 원본과 다르다 — 덮어쓰지 않음. 로봇에서 고쳤다면 com1 으로 옮겨 커밋할 것:"; diff "$src" "$dst" | head -20 || true; fi
fi

# ── ⑦ 자기검증 ────────────────────────────────────────────────────────────
if want 7; then
  step "⑦ 자기검증"
  ok=1
  [ -x "$G1/build/g1_ctrl" ] && echo "  🟢 g1_ctrl" || { echo "  🔴 g1_ctrl 없음"; ok=0; }
  if [ -z "$SKIP_TELEOP" ]; then
    PYV="$REPO/.venv-teleop/bin/python"
    if [ -x "$PYV" ] && "$PYV" - <<'PYCHK'
import sys, torch, general_motion_retargeting
assert sys.version_info[:2] == (3, 10), sys.version
assert torch.version.cuda is None, torch.version.cuda
from general_motion_retargeting import GeneralMotionRetargeting as G
G("xrobot", "unitree_g1")
PYCHK
    then echo "  🟢 teleop venv 3.10 · torch cpu · GMR 스모크"; else echo "  🔴 teleop 자기검증 실패"; ok=0; fi
    mb=$(( $(du -sm "$REPO/.venv-teleop" 2>/dev/null | cut -f1) + $(du -sm "$REPO/.gmr" 2>/dev/null | cut -f1) ))
    [ "$mb" -le 1500 ] && echo "  🟢 teleop 환경 ${mb} MB" || { echo "  🔴 teleop 환경 ${mb} MB > 1500"; ok=0; }
  fi
  echo "  repo 전체: $(du -sh "$REPO" | cut -f1)   (.git $(du -sh "$REPO/.git" | cut -f1))"
  [ "$ok" = 1 ] || die "자기검증 실패 — 위 🔴 를 고친 뒤 재실행 (멱등)"
  echo
  echo "🟢 bootstrap 완료. 다음: (com1) robot.sh verify → 로거 install.sh → 실기는 rules/RUNBOOK_real_robot_mode1.md"
fi
