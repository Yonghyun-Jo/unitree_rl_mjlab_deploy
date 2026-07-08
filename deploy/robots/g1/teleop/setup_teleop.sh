#!/usr/bin/env bash
# setup_teleop.sh — reproduce the LAPTOP-side VR teleop env (GMR retarget) for
# vr_teleop_bridge.py, self-contained from THIS repo.
#
# Run on the machine that runs the bridge (laptop / a Linux PC near com1) — NOT on the
# G1 onboard. The robot only runs the C++ g1_ctrl; GMR (~15ms IK) must stay off the
# 50Hz control host, so it streams a 276-byte ref into /dev/shm and the robot consumes it.
#
# Does: (1) venv <repo>/.venv-teleop, (2) clone GMR into <repo>/.gmr, (3) repair GMR's
# g1 mesh files (upstream GMR ships no G1 STLs) by copying THIS repo's committed STLs,
# (4) pip install GMR (editable) + bridge deps, (5) smoke-test the retargeter.
#
#   Usage:  bash deploy/robots/g1/teleop/setup_teleop.sh
#   Run:    .venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --mode 1
#   (on com1 the existing `gmr` conda env already works; this is for a fresh machine.)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"    # teleop -> g1 -> robots -> deploy -> repo
VENV="$REPO_ROOT/.venv-teleop"
GMR_DIR="$REPO_ROOT/.gmr"
REPO_STL="$REPO_ROOT/src/assets/robots/unitree_g1/xmls/assets"

GMR_REPO="https://github.com/YanjieZe/GMR.git"
# reference_code/GMR (the working snapshot on com1) is not a git repo, so we track upstream
# default branch. If retarget behavior drifts, set GMR_COMMIT to a known-good SHA to pin.
GMR_COMMIT=""

echo "[setup_teleop] repo=$REPO_ROOT"
command -v python3 >/dev/null || { echo "ERROR: python3 없음 (need >=3.10)"; exit 1; }
command -v git     >/dev/null || { echo "ERROR: git 없음"; exit 1; }

# 1) venv
if [ ! -d "$VENV" ]; then python3 -m venv "$VENV"; fi
PY="$VENV/bin/python"
"$PY" -m pip install -q --upgrade pip

# 2) clone GMR
if [ ! -d "$GMR_DIR/.git" ]; then
  echo "[setup_teleop] cloning GMR ..."
  git clone "$GMR_REPO" "$GMR_DIR"
fi
[ -n "$GMR_COMMIT" ] && git -C "$GMR_DIR" checkout -q "$GMR_COMMIT" || true

# 3) repair GMR g1 meshes: upstream GMR ships NO g1 STLs (empty/dangling). Copy the
#    byte-identical STLs committed in THIS repo -> GMR's meshes dir (de-symlink).
MESH_DIR="$GMR_DIR/general_motion_retargeting/../assets/unitree_g1/meshes"
[ -d "$GMR_DIR/assets/unitree_g1" ] && MESH_DIR="$GMR_DIR/assets/unitree_g1/meshes"
mkdir -p "$MESH_DIR"
find "$MESH_DIR" -type l -delete 2>/dev/null || true       # drop dangling symlinks
cp -f "$REPO_STL"/*.STL "$MESH_DIR"/
echo "[setup_teleop] copied $(ls "$MESH_DIR"/*.STL 2>/dev/null | wc -l) STLs -> $MESH_DIR"

# 4) install GMR (editable) + bridge deps
"$PY" -m pip install -e "$GMR_DIR"
"$PY" -m pip install -r "$SCRIPT_DIR/requirements.txt"

# 5) smoke test: build the xrobot->unitree_g1 retargeter (loads the mocap model + meshes)
echo "[setup_teleop] smoke test ..."
"$PY" - <<'PY'
from general_motion_retargeting import GeneralMotionRetargeting as G
G("xrobot", "unitree_g1")
print("[setup_teleop] OK: xrobot->unitree_g1 retargeter built (meshes resolve).")
PY

echo "[setup_teleop] done. run the bridge:"
echo "  $PY $SCRIPT_DIR/vr_teleop_bridge.py --mode 1"
