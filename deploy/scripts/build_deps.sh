#!/usr/bin/env bash
# build_deps.sh — populate <repo>/.deps with unitree_sdk2 + bundled CycloneDDS.
#
# g1_ctrl (deploy/robots/g1) and unitree_mujoco (simulate/) link unitree_sdk2 +
# ddsc/ddscxx from the repo-local .deps prefix (deploy/robots/g1/CMakeLists.txt:
# DEPS_PREFIX = <repo>/.deps). .deps/ is gitignored, so a FRESH CLONE must run this
# ONCE before building. Arch-agnostic: builds for THIS machine (x86_64 dev PC or
# aarch64 Jetson onboard) — unitree_sdk2 ships a prebuilt libunitree_sdk2.a + CycloneDDS
# for both arches (lib/<arch>, thirdparty/lib/<arch>), and install copies the right ones.
#
#   Usage:  bash deploy/scripts/build_deps.sh
#   Then:   cd deploy/robots/g1 && mkdir -p build && cd build && cmake .. && make -j4
#
# System deps first (apt): cmake build-essential libboost-all-dev libyaml-cpp-dev \
#                          zlib1g-dev libfmt-dev libeigen3-dev
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"        # deploy/scripts -> repo
DEPS="$REPO_ROOT/.deps"
SDK_SRC="$DEPS/src/unitree_sdk2"

SDK_REPO="https://github.com/unitreerobotics/unitree_sdk2.git"
SDK_COMMIT="e02dc701548fc81a5fa06e6734e98540329a2377"   # pinned known-good (deploy 검증본)

echo "[build_deps] repo=$REPO_ROOT"
echo "[build_deps] prefix=$DEPS  arch=$(uname -m)"

command -v cmake >/dev/null || { echo "ERROR: cmake 없음 (sudo apt install cmake build-essential)"; exit 1; }
command -v git   >/dev/null || { echo "ERROR: git 없음"; exit 1; }

mkdir -p "$DEPS/src"

# 1) fetch unitree_sdk2 at the pinned commit
if [ ! -d "$SDK_SRC/.git" ]; then
  echo "[build_deps] cloning unitree_sdk2 ..."
  git clone "$SDK_REPO" "$SDK_SRC"
fi
git -C "$SDK_SRC" checkout -q "$SDK_COMMIT" 2>/dev/null || {
  git -C "$SDK_SRC" fetch origin && git -C "$SDK_SRC" checkout -q "$SDK_COMMIT"; }

# 2) configure (no examples) + build + install into the .deps prefix.
#    unitree_sdk2 ships a PREBUILT libunitree_sdk2.a + CycloneDDS for the current arch;
#    install copies libunitree_sdk2.a + headers + ddsc/ddscxx into .deps/{lib,include}.
cmake -S "$SDK_SRC" -B "$SDK_SRC/build" \
      -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$DEPS"
cmake --build "$SDK_SRC/build" -j"$(nproc)"
cmake --install "$SDK_SRC/build"

echo "[build_deps] done. .deps/lib:"
ls "$DEPS/lib" 2>/dev/null | sed 's/^/  /'
echo "[build_deps] next: cd $REPO_ROOT/deploy/robots/g1 && mkdir -p build && cd build && cmake .. && make -j4"
