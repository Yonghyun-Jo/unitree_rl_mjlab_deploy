#!/usr/bin/env bash
# setup_teleop.sh — VR teleop 브릿지(vr_teleop_bridge.py)용 «최소» 파이썬 환경을 이 repo 안에 만든다.
#
# 어디서 도나: 브릿지를 돌리는 머신. 현재 실기 토폴로지는 **온보드 Jetson** 이다
#   (노트북 PICO publisher → 온보드 bridge --transport udp → /dev/shm → g1_ctrl).
#   개발용으로 com1/노트북 리눅스에서도 같은 스크립트를 쓴다.
#
# 왜 «최소» 인가 (2026-09-01 실측): pip 기본값으로 두면 aarch64 에서 torch 2.13+CUDA 13 (4.5 GB) 을
#   받는데 Jetson 은 CUDA 11.4 라 한 바이트도 안 쓰인다. GMR 은 smplx 경유로 torch 를 import 만 한다
#   → CPU wheel 이면 충분. GMR assets 1.2 GB 중 필요한 건 unitree_g1 53 MB → sparse checkout.
#   결과: 6.6 GB → < 1.5 GB.
#
# 계약:
#   TELEOP_PY   인터프리터 (>=3.10). 없으면 uv 관리 3.10 → python3 순으로 찾는다.
#   GMR_COMMIT  기본 bb1bbe4 (2026-04-02, 실기 검증 커밋). 빈 문자열이면 upstream HEAD.
#   XRT_SRC     xrobotoolkit pybind 소스. glibc < 2.34 머신에선 시도하지 않는다(JetPack 5 는 로드 불가).
#   DRY_RUN=1   실행 대신 «[dry] 명령» 출력 (테스트용).  RESET_GMR=1  기존 .gmr 을 버리고 다시 받는다.
#   산출물: <repo>/.venv-teleop, <repo>/.gmr  (둘 다 gitignore)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
VENV="$REPO_ROOT/.venv-teleop"
GMR_DIR="$REPO_ROOT/.gmr"
REPO_STL="$REPO_ROOT/src/assets/robots/unitree_g1/xmls/assets"
GMR_REPO="https://github.com/YanjieZe/GMR.git"
GMR_COMMIT="${GMR_COMMIT-bb1bbe4}"
TORCH_SPEC="torch==2.13.0"
TORCH_INDEX="https://download.pytorch.org/whl/cpu"
SPARSE_DIRS="general_motion_retargeting assets/unitree_g1 scripts third_party"
MAX_MB=1500
DRY_RUN="${DRY_RUN:-}"

log() { echo "[setup_teleop] $*"; }
run() { if [ -n "$DRY_RUN" ]; then echo "[dry] $*"; else "$@"; fi; }

# ── 0) 인터프리터 ──────────────────────────────────────────────────────────
pick_python() {
  if [ -n "${TELEOP_PY:-}" ]; then echo "$TELEOP_PY"; return; fi
  if command -v uv >/dev/null 2>&1; then
    local p; p="$(uv python find 3.10 2>/dev/null || true)"; [ -n "$p" ] && { echo "$p"; return; }
  fi
  command -v python3 || true
}
BASE_PY="$(pick_python)"
[ -n "$BASE_PY" ] || { log "ERROR: python 을 못 찾았다. TELEOP_PY=<3.10 경로> 로 주거나 uv 로 3.10 을 설치할 것"; exit 1; }
if [ -z "$DRY_RUN" ]; then
  "$BASE_PY" -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' \
    || { log "ERROR: $BASE_PY 는 3.10 미만. GMR 은 >=3.10 (Jetson 시스템 3.8 은 불가 → uv python install 3.10)"; exit 1; }
fi
log "repo=$REPO_ROOT  python=$BASE_PY"

# ── 1) venv ────────────────────────────────────────────────────────────────
if [ ! -x "$VENV/bin/python" ]; then
  if command -v uv >/dev/null 2>&1; then run uv venv --seed --python "$BASE_PY" "$VENV"
  else run "$BASE_PY" -m venv "$VENV"; fi
fi
PY="$VENV/bin/python"
run "$PY" -m pip install -q --upgrade pip

# ── 2) torch CPU «먼저» — 그래야 GMR(smplx) 이 CUDA 판을 끌어오지 않는다 ────
run "$PY" -m pip install "$TORCH_SPEC" --index-url "$TORCH_INDEX"

# ── 3) GMR sparse + pinned ─────────────────────────────────────────────────
if [ -n "${RESET_GMR:-}" ] && [ -d "$GMR_DIR" ]; then run rm -rf "$GMR_DIR"; fi
if [ ! -d "$GMR_DIR/.git" ]; then
  log "cloning GMR (sparse: $SPARSE_DIRS, commit=${GMR_COMMIT:-HEAD}) ..."
  run mkdir -p "$GMR_DIR"
  run git -C "$GMR_DIR" init -q
  run git -C "$GMR_DIR" remote add origin "$GMR_REPO"
  run git -C "$GMR_DIR" sparse-checkout init --cone
  run git -C "$GMR_DIR" sparse-checkout set $SPARSE_DIRS
  if [ -n "$GMR_COMMIT" ]; then
    run git -C "$GMR_DIR" fetch -q --depth 1 --filter=blob:none origin "$GMR_COMMIT"
  else
    run git -C "$GMR_DIR" fetch -q --depth 1 --filter=blob:none origin HEAD
  fi
  run git -C "$GMR_DIR" checkout -q FETCH_HEAD
else
  have="$(git -C "$GMR_DIR" rev-parse --short HEAD 2>/dev/null || echo '?')"
  if [ -n "$GMR_COMMIT" ] && [ -z "$DRY_RUN" ] && [ "$have" != "$GMR_COMMIT" ]; then
    log "WARN: 기존 .gmr HEAD=$have ≠ 핀 $GMR_COMMIT. 맞추려면 RESET_GMR=1 로 재실행 (지금은 그대로 진행)"
  fi
fi
# g1 메시 복구 (upstream GMR 에는 G1 STL 이 없다 → 이 repo 의 STL 복사)
MESH_DIR="$GMR_DIR/assets/unitree_g1/meshes"
run mkdir -p "$MESH_DIR"
if [ -z "$DRY_RUN" ]; then find "$MESH_DIR" -type l -delete 2>/dev/null || true; fi
run cp -f "$REPO_STL"/*.STL "$MESH_DIR"/
run "$PY" -m pip install -e "$GMR_DIR"
run "$PY" -m pip install -r "$SCRIPT_DIR/requirements.txt"

# ── 4) xrt (선택) — glibc < 2.34 에선 시도하지 않는다 ─────────────────────
glibc="$(ldd --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+$' || echo 0)"
glibc_ok=$("$BASE_PY" -c "import sys; print(1 if tuple(map(int,'$glibc'.split('.')))>=(2,34) else 0)" 2>/dev/null || echo 0)
XRT_SRC="${XRT_SRC:-}"
if [ -n "$XRT_SRC" ] && [ -d "$XRT_SRC" ]; then
  if [ "$glibc_ok" != "1" ]; then
    log "SKIP xrt: glibc $glibc < 2.34 — 이 머신에선 PC-Service/xrt 로드 불가 (rules/RUNBOOK_onboard_pico_teleop.md). udp/zmq 만."
  else
    (
      log "installing xrobotoolkit_sdk from $XRT_SRC ..."
      ARCH="$(uname -m)"
      run "$PY" -m pip install -q cmake pybind11 setuptools
      export CMAKE_PREFIX_PATH="$("$PY" -m pybind11 --cmakedir 2>/dev/null || true)"
      if [ "$ARCH" = "aarch64" ]; then NATIVE="$XRT_SRC/lib/aarch64/libPXREARobotSDK.so"; else NATIVE="$XRT_SRC/lib/libPXREARobotSDK.so"; fi
      if [ "$ARCH" = "aarch64" ] && ! file "$NATIVE" 2>/dev/null | grep -q ELF; then
        T="$XRT_SRC/tmp"; run mkdir -p "$T"
        [ -d "$T/XRoboToolkit-PC-Service" ] || run git clone -b orin https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git "$T/XRoboToolkit-PC-Service"
        ( cd "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK" && run bash build.sh )
        run mkdir -p "$XRT_SRC/lib/aarch64" "$XRT_SRC/include/aarch64"
        run cp "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h" "$XRT_SRC/include/aarch64/"
        run cp -r "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann" "$XRT_SRC/include/aarch64/nlohmann/"
        run cp "$T/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so" "$XRT_SRC/lib/aarch64/"
        run rm -rf "$T"
      fi
      if file "$NATIVE" 2>/dev/null | grep -q ELF; then
        run "$PY" -m pip install --no-build-isolation -e "$XRT_SRC/"
        run "$PY" -c "import xrobotoolkit_sdk; print('[setup_teleop] OK: xrobotoolkit_sdk imports')"
      else
        log "xrt 스킵: $NATIVE 가 ELF 가 아니다 (x86 은 git lfs pull 필요). udp/zmq 는 동작."
      fi
    ) || log "WARN: xrobotoolkit_sdk 설치 실패 — 스킵 (udp/zmq 는 동작)"
  fi
else
  log "SKIP xrt (XRT_SRC 없음); 브릿지는 udp/zmq 모드."
fi

# ── 5) 자기검증 — 통과 못 하면 «설치됐다» 고 말하지 않는다 ────────────────
if [ -n "$DRY_RUN" ]; then log "dry-run 끝 (자기검증 생략)"; exit 0; fi
"$PY" - <<'PYCHK'
import torch, general_motion_retargeting as gmr
assert torch.version.cuda is None, "CUDA torch 가 들어왔다 (%s) — 4.5 GB 낭비. TORCH_INDEX 순서 확인" % torch.version.cuda
from general_motion_retargeting import GeneralMotionRetargeting as G
G("xrobot", "unitree_g1")
print("[setup_teleop] OK: torch %s (cpu), xrobot->unitree_g1 retargeter built" % torch.__version__)
PYCHK
used=$(( $(du -sm "$VENV" | cut -f1) + $(du -sm "$GMR_DIR" | cut -f1) ))
[ "$used" -le "$MAX_MB" ] || { log "ERROR: 환경이 ${used} MB > ${MAX_MB} MB — 최소화가 안 됐다 (pip list | grep -i nvidia 확인)"; exit 1; }
log "OK: .venv-teleop + .gmr = ${used} MB  (glibc $glibc → xrt local $( [ "$glibc_ok" = 1 ] && echo 가능 || echo 불가 ))"
log "run: $PY $SCRIPT_DIR/vr_teleop_bridge.py --transport udp --arm-estop --grip-enable --mode 1"
