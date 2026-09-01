# 온보드 bootstrap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 새 G1 온보드에서 `git clone` 2개 + `bootstrap.sh` + `install.sh` 만으로 center_g1 과 같은 상태(제어기·VR teleop 브릿지·로거)를 만들고, teleop 환경을 6.6 GB → < 1.5 GB 로 줄인다.

**Architecture:** 브릿지 코드는 제어기 repo(`deploy/robots/g1/teleop/`)에 그대로 둔다(`/dev/shm` VrRef 계약이 C++ 과 같은 커밋에 있어야 한다). «모듈»은 **환경 생성 스크립트**다 — `setup_teleop.sh` 가 torch CPU 선설치 + GMR sparse/pinned 로 최소 환경을 만들고, `deploy/onboard/bootstrap.sh` 가 제어기 빌드→python→teleop→CLAUDE→자기검증을 멱등 7단계로 묶는다. 로거는 `install.sh` 가 SDK 를 자기 venv 에 직접 넣어 다른 사람 워크스페이스 의존을 끊는다.

**Tech Stack:** bash(멱등 스크립트), uv(python 3.10 관리), pip(`torch==2.13.0+cpu`), git sparse-checkout, systemd user unit, pytest(로거), CycloneDDS 0.10.2 + unitree_sdk2_python.

**Spec:** `docs/superpowers/specs/2026-09-01-onboard-bootstrap-design.md`

## Global Constraints

- 🅑 공용 base(`deploy/include/**`) 수정 금지. 이 계획은 `deploy/onboard/**`, `deploy/robots/g1/teleop/**`, `rules/**`, `piene_g1_logger/**` 만 만진다.
- 로봇에서 편집 금지. 모든 변경은 com1 → commit → push → 로봇 `robot.sh sync-code` 로 간다. 로봇 쪽 조회·실행은 `~/piene_automation/robot_bridge/robot.sh sh '<명령>'`.
- `git commit -a` 금지 — 각 태스크의 파일만 `git add`. 워킹트리에 **다른 창의 미커밋 변경**이 있다(` D README_zh.md`, ` M …/260829_v3_ms8_place2_30k/ONNX_META.json`, ` M rules/RUNBOOK_g1_from_scratch.md`, `?? disabled`). 절대 스테이징하지 않는다. `rules/RUNBOOK_g1_from_scratch.md` 는 이 계획에서 건드리지 않는다.
- 커밋 메시지에 AI 흔적(Co-Authored-By 등) 금지. 한국어, 기존 스타일(`영역: 무엇 — 왜`).
- 스크립트는 **sudo 를 실행하지 않는다**(공용 기계). 필요한 apt 명령은 출력만.
- 핀: GMR `bb1bbe4` · `torch==2.13.0` from `https://download.pytorch.org/whl/cpu` · `cyclonedds==0.10.2` · teleop python 3.10 · 로거 python 3.8(시스템).
- g1_ctrl 은 항상 `-DCMAKE_BUILD_TYPE=Release`.
- 브랜치: 작업 전 `git branch --show-current` 가 `smooth_mode_switch`(deploy) / `main`(logger) 인지 확인. 다르면 멈추고 사용자에게.
- com1 에는 shellcheck 가 없다 → 정적 검사는 `bash -n`.

## File Structure

```
unitree_rl_mjlab (smooth_mode_switch)
  deploy/robots/g1/teleop/setup_teleop.sh           수정: 최소 환경(torch cpu 선설치·GMR sparse+pin·인터프리터 선택·DRY_RUN·자기검증)
  deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh   신규: DRY_RUN 출력 순서·핀 검사 (bash, 의존 없음)
  deploy/onboard/bootstrap.sh                        신규: 7단계 멱등 부트스트랩 (+ --check / --only / --skip-teleop)
  deploy/onboard/README.md                           신규: 새 로봇 첫날 순서 (robot_bridge.md §5-2 와 같은 내용, repo 안 사본)
  deploy/onboard/CLAUDE.robot.md                     신규: 로봇 piene_ws/CLAUDE.md 원본
  deploy/robots/g1/teleop/tools/pico_sniff.py        신규(로봇에서 회수)
  deploy/robots/g1/teleop/legacy/vr_relay_send.py    신규(회수)
  deploy/robots/g1/teleop/legacy/vr_relay_recv.py    신규(회수)
  deploy/robots/g1/teleop/legacy/README.md           신규: 왜 legacy·언제 되살리나
  rules/RUNBOOK_onboard_pico_teleop.md               신규(회수 + 머리말)
  deploy/robots/g1/teleop/README.md                  수정: §C 머리에 ⛔ 한 줄, 점검용에 pico_sniff 한 줄
piene_g1_logger (main)
  systemd/g1-logger.service.in                       이름 변경+템플릿화 (__REPO__)
  install.sh                                         재작성: 렌더·SDK·자기검증 (+ --render-only)
  tests/test_install_render.py                       신규
  pyproject.toml / .gitignore / README.md / DEPLOY.md  수정
Obsidian (커밋 안 함)
  claude_rules/procedures/robot_bridge.md §5-2       새 순서
  5. AI_workspace/unitree_rl_mjlab/legacy/           robot_state_logger.py · onboard_teleop_setup.sh · sim2sim_setup_x86.sh 사본
```

Part A(Task 1–6, deploy repo) 와 Part B(Task 7–9, logger repo) 는 서로 독립이다. Task 10 은 둘 다 끝난 뒤.

---

## Part A — unitree_rl_mjlab

### Task 1: `setup_teleop.sh` 를 최소 환경 생성기로 바꾼다

**Files:**
- Modify: `deploy/robots/g1/teleop/setup_teleop.sh` (전체 교체)
- Test: `deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh` (신규)

**Interfaces:**
- Consumes: 없음
- Produces: 환경변수 계약 — `TELEOP_PY`(인터프리터 경로) · `GMR_COMMIT`(기본 `bb1bbe4`) · `XRT_SRC` · `DRY_RUN=1`(명령을 실행하지 않고 `[dry] …` 로 출력) · `RESET_GMR=1`(기존 `.gmr` 을 지우고 다시 clone). 산출물 위치는 그대로 `<repo>/.venv-teleop`, `<repo>/.gmr`. 종료 코드 0 = 자기검증 통과. Task 2 가 `TELEOP_PY=<uv 3.10> bash setup_teleop.sh` 로 부른다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh`:
```bash
#!/usr/bin/env bash
# DRY_RUN 출력으로 «순서와 핀» 을 검사한다. 네트워크·설치 없음.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(DRY_RUN=1 TELEOP_PY=/usr/bin/python3 bash "$HERE/../setup_teleop.sh" 2>&1)" || { echo "$OUT"; echo "FAIL: exit != 0"; exit 1; }
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
```

- [ ] **Step 2: 실패 확인**

Run: `bash deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh`
Expected: FAIL — 현재 스크립트는 `DRY_RUN` 을 모르므로 실제 venv 를 만들려 하거나 `whl/cpu` 가 없어 실패한다. (실제 설치가 시작되면 Ctrl-C.) `TELEOP_PY` 도 무시된다.

- [ ] **Step 3: 스크립트 전체 교체**

`deploy/robots/g1/teleop/setup_teleop.sh` 를 아래로 바꾼다 (기존 xrt 블록의 로직은 유지하되 `run` 헬퍼로 감싼다):
```bash
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
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `bash -n deploy/robots/g1/teleop/setup_teleop.sh && bash deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh`
Expected: `OK: setup_teleop dry-run order/pins`

- [ ] **Step 5: (선택, 10분) com1 x86 에서 실제 최소 환경 1회**

Run: `cp -r ~/unitree_rl_mjlab /tmp/claude-1000/-home-piene-unitree-rl-mjlab/09119067-fa77-4aca-9524-37339daa7a98/scratchpad/repo_copy 2>/dev/null; cd …/repo_copy && GMR_COMMIT=bb1bbe4 bash deploy/robots/g1/teleop/setup_teleop.sh`
Expected: 마지막 두 줄 `OK: torch 2.13.0+cpu (cpu) …` / `OK: .venv-teleop + .gmr = <1500 MB`. (x86 CPU wheel 도 같은 인덱스에 있다.) 끝나면 복사본 삭제.

- [ ] **Step 6: Commit**

```bash
git add deploy/robots/g1/teleop/setup_teleop.sh deploy/robots/g1/teleop/tests/test_setup_teleop_dryrun.sh
git commit -m "teleop(setup): 최소 환경 — torch cpu 선설치·GMR sparse+bb1bbe4 핀·glibc 게이트·자기검증 (6.6 GB → <1.5 GB)"
```

---

### Task 2: `deploy/onboard/bootstrap.sh` + `README.md`

**Files:**
- Create: `deploy/onboard/bootstrap.sh`
- Create: `deploy/onboard/README.md`

**Interfaces:**
- Consumes: Task 1 의 `setup_teleop.sh` (`TELEOP_PY`, 종료 코드), `deploy/scripts/build_deps.sh`, `deploy/onboard/CLAUDE.robot.md`(Task 3 — 없으면 ⑥ 을 건너뛰고 경고).
- Produces: `bootstrap.sh [--check] [--only <1-7>] [--skip-teleop] [--ws <dir>]`. 종료 코드 0 = 7단계 전부 통과. 종료 2 = 사전점검 실패(아키텍처·apt·github). Task 6 이 로봇에서 실행한다.

- [ ] **Step 1: 실패하는 테스트 (com1 은 x86 → `--check` 가 2 로 끝나야 한다)**

Run: `bash deploy/onboard/bootstrap.sh --check; echo "exit=$?"`
Expected(현재): `No such file` — 스크립트가 없다.

- [ ] **Step 2: 스크립트 작성**

`deploy/onboard/bootstrap.sh`:
```bash
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
```

`deploy/onboard/README.md`:
```markdown
# deploy/onboard — 새 G1 온보드에 «같은 상태» 를 만드는 곳

로봇은 공용 기계다. 내 것은 `~/dyros_ws/piene_ws/` 안에만 만든다 (규칙 원문: `claude_rules/projects/unitree_rl_mjlab.md`).

## 새 로봇 첫날 (com1 에서, 노트북이 로봇 옆에 있을 때)
```bash
cd ~/piene_automation
./robot_bridge/robot.sh check                       # 3단 🟢
./robot_bridge/robot.sh register <이름> "<위치>"      # 지문으로 정체 고정 (IP 는 두 로봇이 같다)
# clone 2개 — private repo 라 com1 키를 «빌려서» (ControlPath=none 필수: 마스터 연결은 -A 없이 열려 있다)
ssh -A -o ControlPath=none g1 'mkdir -p ~/dyros_ws/piene_ws && cd ~/dyros_ws/piene_ws \
  && git clone --depth 1 -b smooth_mode_switch git@github.com:Yonghyun-Jo/unitree_rl_mjlab_deploy.git \
  && git clone --depth 1 git@github.com:Yonghyun-Jo/piene_g1_logger.git'
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && bash deploy/onboard/bootstrap.sh --check'
#   apt 가 모자라면 사람이 로봇에서 sudo apt install … 후 다시
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && nohup bash deploy/onboard/bootstrap.sh > /tmp/bootstrap.log 2>&1 &'
#   10~15 분. tail: robot.sh sh 'tail -5 /tmp/bootstrap.log'
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/piene_g1_logger && bash install.sh'
./robot_bridge/robot.sh verify                      # 게이트 (setcap 은 robot.sh deploy 가 하거나 사람이)
```
첫날 기록할 것: `bootstrap ①` 의 glibc 판정(xrt local 가능 여부) · 손/머리 장착물/판 rev(memory `ask-hardware-config-before-sim2real-diagnosis`).

## 파일
- `bootstrap.sh` — 7단계 멱등. sudo 안 부름.
- `CLAUDE.robot.md` — 로봇 `piene_ws/CLAUDE.md` 의 원본. 로봇에서 고치지 말고 여기서 고쳐 push.
- 정책 슬롯 무게는 git 이 아니라 `deploy/scripts/policy_slot.py push` (rsync).
```

- [ ] **Step 3: 테스트**

Run: `bash -n deploy/onboard/bootstrap.sh && bash deploy/onboard/bootstrap.sh --check; echo "exit=$?"`
Expected: `🔴 aarch64 전용 (현재 x86_64) …` 그리고 `exit=2`.
Run: `bash deploy/onboard/bootstrap.sh --only 6 --ws /tmp/claude-1000/-home-piene-unitree-rl-mjlab/09119067-fa77-4aca-9524-37339daa7a98/scratchpad/ws_test; ls …/ws_test`
Expected: Task 3 전이면 `⚠ … CLAUDE.robot.md 없음 — 건너뜀`, Task 3 후면 `🟢 생성 …/ws_test/CLAUDE.md`.

- [ ] **Step 4: Commit**

```bash
git add deploy/onboard/bootstrap.sh deploy/onboard/README.md
git commit -m "onboard(bootstrap): 새 G1 에서 clone 뒤 명령 하나로 .deps→g1_ctrl(Release)→uv 3.10→teleop→CLAUDE→자기검증 (멱등, sudo 미실행)"
```

---

### Task 3: `CLAUDE.robot.md` — 로봇 규칙의 원본을 repo 로

**Files:**
- Create: `deploy/onboard/CLAUDE.robot.md`

**Interfaces:**
- Produces: bootstrap ⑥ 이 `$PIENE_WS/CLAUDE.md` 로 복사하는 파일.

- [ ] **Step 1: 로봇에서 현재 내용을 가져온다 (편집은 com1 에서)**

Run:
```bash
cd ~/unitree_rl_mjlab
~/piene_automation/robot_bridge/robot.sh sh 'cat ~/dyros_ws/piene_ws/CLAUDE.md' > /tmp/claude-1000/-home-piene-unitree-rl-mjlab/09119067-fa77-4aca-9524-37339daa7a98/scratchpad/claude_robot_raw.md
wc -l /tmp/claude-1000/-home-piene-unitree-rl-mjlab/09119067-fa77-4aca-9524-37339daa7a98/scratchpad/claude_robot_raw.md   # 약 60 줄, "# piene_ws — 실로봇 온보드 (Jetson)" 로 시작해야 한다
```

- [ ] **Step 2: 머리말을 붙여 repo 파일로**

```bash
{
cat <<'HDR'
<!-- 원본: unitree_rl_mjlab_deploy/deploy/onboard/CLAUDE.robot.md — bootstrap.sh ⑥ 이 piene_ws/CLAUDE.md 로 복사한다.
     로봇에서 이 파일을 고치지 않는다. com1 에서 고쳐 push → 로봇 pull → bootstrap --only 6. -->
HDR
sed -e 's|@unitree_rl_mjlab_deploy/CLAUDE.md|(공통 규칙은 repo 의 rules/ 와 com1 의 claude_rules/projects/unitree_rl_mjlab.md 에 있다)|' \
    -e 's|저장소는 `unitree_rl_mjlab_deploy/` 하나뿐이다|저장소는 `unitree_rl_mjlab_deploy/` 와 `piene_g1_logger/` 둘이다|' \
    /tmp/claude-1000/-home-piene-unitree-rl-mjlab/09119067-fa77-4aca-9524-37339daa7a98/scratchpad/claude_robot_raw.md
} > deploy/onboard/CLAUDE.robot.md
grep -c "piene_ws" deploy/onboard/CLAUDE.robot.md   # ≥ 3
```
(로봇의 `unitree_rl_mjlab_deploy/CLAUDE.md` 는 untracked 이고 «git 으로 추적된다» 고 잘못 적혀 있다 — Task 6 에서 로봇에서 삭제한다. 루트 `CLAUDE.md` 는 com1 에서 Obsidian 심링크라 repo 에 넣지 않는다.)

- [ ] **Step 3: 테스트** — `bash deploy/onboard/bootstrap.sh --only 6 --ws …/scratchpad/ws_test && diff deploy/onboard/CLAUDE.robot.md …/scratchpad/ws_test/CLAUDE.md && echo SAME`
Expected: `🟢 생성` 뒤 `SAME`.

- [ ] **Step 4: Commit**

```bash
git add deploy/onboard/CLAUDE.robot.md
git commit -m "onboard: 로봇 piene_ws/CLAUDE.md 의 원본을 repo 에 둔다 (bootstrap ⑥ 이 복사)"
```

---

### Task 4: 고아 파일 구출 — RUNBOOK · pico_sniff · vr_relay(legacy)

**Files:**
- Create: `rules/RUNBOOK_onboard_pico_teleop.md`, `deploy/robots/g1/teleop/tools/pico_sniff.py`, `deploy/robots/g1/teleop/legacy/vr_relay_send.py`, `deploy/robots/g1/teleop/legacy/vr_relay_recv.py`, `deploy/robots/g1/teleop/legacy/README.md`
- Modify: `deploy/robots/g1/teleop/README.md:55` (§C 머리), `:83` 앞(점검용 한 줄)

- [ ] **Step 1: 로봇에서 복사**

```bash
cd ~/unitree_rl_mjlab && mkdir -p deploy/robots/g1/teleop/tools deploy/robots/g1/teleop/legacy
R=g1:~/dyros_ws/piene_ws
scp -q $R/pico_sniff.py deploy/robots/g1/teleop/tools/
scp -q $R/vr_relay_send.py $R/vr_relay_recv.py deploy/robots/g1/teleop/legacy/
scp -q $R/RUNBOOK_onboard_pico_teleop.md rules/
md5sum deploy/robots/g1/teleop/tools/pico_sniff.py; ~/piene_automation/robot_bridge/robot.sh sh 'md5sum ~/dyros_ws/piene_ws/pico_sniff.py'   # 같아야 한다
```

- [ ] **Step 2: RUNBOOK 머리말**

`rules/RUNBOOK_onboard_pico_teleop.md` 맨 위(첫 줄 `# 온보드 PICO 텔레옵 런북 …` 앞)에 삽입:
```markdown
> **2026-09-01 이관 메모** — 이 문서는 2026-07-14 에 로봇 온보드에만 있던 것을 repo 로 옮긴 것이다.
> **JetPack 6(Ubuntu 22.04, glibc ≥ 2.34) 로봇에서만 아래 절차가 유효하다.** center_g1(JetPack 5.1.1) 은
> 아래 ⛔ 대로 불가라서 현재 실기 토폴로지는 [RUNBOOK_laptop_pico_teleop.md](RUNBOOK_laptop_pico_teleop.md) 의
> 노트북 publisher → 온보드 bridge `--transport udp --arm-estop` 이다. 새 로봇의 판정은 `deploy/onboard/bootstrap.sh --check` 가 glibc 로 알려준다.
> 그리고 ⛔ 의 «`.deb` 를 설치하지 말 것» 은 사후 결론이다 — center_g1 에는 `/opt/apps/roboticsservice/` 가 **이미 설치돼 있다**(무해, 안 뜸). «되돌리기」절 참조.

```

- [ ] **Step 3: legacy README**

`deploy/robots/g1/teleop/legacy/README.md`:
```markdown
# teleop/legacy — 지금은 안 쓰지만 «되살릴 조건» 이 분명한 것

## vr_relay_send.py / vr_relay_recv.py (2026-07-10)
com1 에서 도는 `vr_teleop_bridge.py --transport local`(xrt 직접 읽기 + 하드 E-stop) 이 자기 `/dev/shm/g1_vr_ref` 에
쓴 276 B VrRef 프레임을 ZMQ(:5557, CONFLATE) 로 Jetson 의 `/dev/shm/g1_vr_ref` 에 그대로 재현한다.
200 ms 무수신 → `valid=0` 을 써서 g1_ctrl 이 클립으로 복귀. 종료 시에도 `valid=0`.

**왜 legacy 인가**: 하드 E-stop 이 `--transport local` 전용이던 시절의 우회(«structure B»). 지금은 네트워크
transport 도 `--arm-estop` 으로 무장되므로 브릿지를 온보드에서 `--transport udp` 로 돌린다.

**되살릴 조건**: PC-Service/xrt 가 도는 **리눅스 PC(glibc ≥ 2.34)** 가 로봇 옆에 있고, 브릿지를 로봇이 아니라
그 PC 에서 돌리고 싶을 때. 이 두 파일은 `vr_shm.py` 의 레이아웃(`<iIii` + 65f = 276 B, magic 0x6702)을 그대로
따른다 — `vr_shm.py`/C++ `struct VrRef` 가 바뀌면 여기도 같이 바꿔야 한다.
```

- [ ] **Step 4: teleop README 두 곳**

`deploy/robots/g1/teleop/README.md` 55행 `## C. 온보드 로컬 텔레옵 …` 바로 아래에:
```markdown
> ⛔ **JetPack 5(glibc 2.31) Jetson 에서는 불가** — PC-Service/xrt 가 glibc 2.34 를 요구해 로드 자체가 안 된다
> (`rules/RUNBOOK_onboard_pico_teleop.md`). center_g1 이 그렇다. 실기는 노트북 publisher → 온보드 bridge
> `--transport udp --arm-estop` (`rules/RUNBOOK_laptop_pico_teleop.md`). 판정은 `deploy/onboard/bootstrap.sh --check`.
```
83행 `## 자립성 메모` 앞에 절 추가:
```markdown
## 점검 도구
- `tools/pico_sniff.py` — PICO 앱/publisher 가 실제로 무엇을 어디로 쏘는지 (UDP+TCP :5556 동시 리슨, pico_wire 806 B 해석). Hop-2 진단.
- `legacy/` — 안 쓰지만 되살릴 조건이 분명한 것 (`legacy/README.md`).

```

- [ ] **Step 5: 테스트** — `python3 -m py_compile deploy/robots/g1/teleop/tools/pico_sniff.py deploy/robots/g1/teleop/legacy/vr_relay_*.py && echo compiled` · `grep -c "JetPack 6" rules/RUNBOOK_onboard_pico_teleop.md` ≥ 1 · `grep -n "⛔" deploy/robots/g1/teleop/README.md` 가 56행 근처.

- [ ] **Step 6: Commit**

```bash
git add rules/RUNBOOK_onboard_pico_teleop.md deploy/robots/g1/teleop/tools/pico_sniff.py deploy/robots/g1/teleop/legacy/ deploy/robots/g1/teleop/README.md
git commit -m "teleop(rescue): 로봇에만 있던 온보드 PICO 런북(⛔ glibc 2.34)·pico_sniff·vr_relay 를 repo 로 — 로봇이 죽어도 남게"
```

---

### Task 5: 나머지 3개는 볼트 legacy 사본만 (커밋 없음)

**Files:**
- Create(볼트): `~/Documents/PIENE/5. AI_workspace/unitree_rl_mjlab/legacy/{robot_state_logger.py,onboard_teleop_setup.sh,sim2sim_setup_x86.sh,README.md}`

- [ ] **Step 1**
```bash
V="$HOME/Documents/PIENE/5. AI_workspace/unitree_rl_mjlab/legacy"; mkdir -p "$V"
scp -q g1:~/dyros_ws/piene_ws/{robot_state_logger.py,onboard_teleop_setup.sh,sim2sim_setup_x86.sh} "$V/"
cat > "$V/README.md" <<'TXT'
# legacy — 로봇 온보드(piene_ws)에만 있다가 2026-09-01 에 걷어낸 것. 대체물이 있어 repo 에는 넣지 않았다.
- robot_state_logger.py (07-14): lowstate/lowcmd CSV + 콘솔 셧다운 경고(모터 mode→0, tau>60 Nm, 기울기). → piene_g1_logger 가 대체. **콘솔 경고만** 아직 g1_logger 에 없는 기능.
- onboard_teleop_setup.sh (07-14): miniforge 경로로 온보드 xrt. 한 번도 안 돌았음. → deploy/onboard/bootstrap.sh
- sim2sim_setup_x86.sh (07-10): com1 x86 빌드, 브랜치 wose_obs 하드체크. → rules/RUNBOOK_g1_from_scratch.md Phase 4
TXT
ls -la "$V"
```
(볼트는 Obsidian Sync — 커밋하지 않는다.)

---

### Task 6: push → center_g1 에서 재현 검증 → 로봇 정리

**Files:** 없음(로봇 조작). 실기 세션 밖에서, 사용자와 시각을 맞춘 뒤.

**Interfaces:**
- Consumes: Task 1–4 커밋이 `origin/smooth_mode_switch` 에 있어야 한다. `robot.sh sync-code` 는 origin 에서 당긴다.

- [ ] **Step 1: 커밋된 트리만으로 빌드되는지 (push 전 규칙)**
```bash
cd ~/unitree_rl_mjlab && git status --short | grep -v '^??' | grep -vE 'README_zh|ONNX_META|RUNBOOK_g1_from_scratch' ; echo "(위에 내 파일이 남아 있으면 안 된다)"
git push origin smooth_mode_switch
```

- [ ] **Step 2: 로봇 코드 동기화 + 기존 환경 백업**
```bash
cd ~/piene_automation
./robot_bridge/robot.sh sync-code unitree_rl_mjlab_deploy
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && mv .venv-teleop .venv-teleop.bak_260901 && mv .gmr .gmr.bak_260901 && df -h / | tail -1'
```

- [ ] **Step 3: bootstrap (백그라운드, 10~15 분)**
```bash
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && bash deploy/onboard/bootstrap.sh --check'
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && nohup bash deploy/onboard/bootstrap.sh > /tmp/bootstrap_260901.log 2>&1 & echo started'
# 폴링:  ./robot_bridge/robot.sh sh 'tail -3 /tmp/bootstrap_260901.log'
```
Expected(끝): `🟢 teleop 환경 <1500 MB` · `🟢 bootstrap 완료`. ①이 `glibc 2.31 < 2.34 → xrt 불가` 를 찍어야 한다.

- [ ] **Step 4: 게이트 + 브릿지 스모크**
```bash
./robot_bridge/robot.sh verify
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && timeout 8 .venv-teleop/bin/python deploy/robots/g1/teleop/vr_teleop_bridge.py --transport udp --arm-estop --grip-enable --mode 1 2>&1 | head -20; rm -f /dev/shm/g1_estop'
```
Expected: 브릿지 시작 로그에 `hard E-stop: ARMED` (g1_ctrl 없이도 시작 로그까지는 나온다; 8초 후 timeout 정상). ⚠ g1_ctrl 은 띄우지 않는다.

- [ ] **Step 5: 로봇 정리 (백업·고아·죽은 것)**
```bash
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws && rm -rf unitree_rl_mjlab_deploy/.venv-teleop.bak_260901 unitree_rl_mjlab_deploy/.gmr.bak_260901 unitree_rl_mjlab_deploy/.xrt-pybind unitree_rl_mjlab_deploy/CLAUDE.md .vscode \
  && rm -f pico_sniff.py vr_relay_send.py vr_relay_recv.py RUNBOOK_onboard_pico_teleop.md robot_state_logger.py onboard_teleop_setup.sh sim2sim_setup_x86.sh RUNBOOK_real_robot_mode1.md SYSTEM_OVERVIEW.md \
  && ls -la && du -sh unitree_rl_mjlab_deploy'
```
Expected: piene_ws 에 `CLAUDE.md g1_logs piene_g1_logger unitree_rl_mjlab_deploy` 만. deploy repo `du` ≈ 1.6–2.0 G (git 446 M + .deps 225 M + src 225 M + teleop < 1.5 G 중 실제 ~0.8 G).
⚠ `~/g1logs`(80 M, 07-14 유물) 삭제는 사용자에게 한 번 묻고.

- [ ] **Step 6: 기록** — 볼트 `designs/260901_onboard_piene_ws_portability.md` 끝에 «§7 재현 결과: bootstrap 소요 시간 · 최종 MB · glibc 판정» 3줄 append. (실험 노트 규칙: `experiments/260901_onboard_bootstrap.md` 는 Task 10 에서 한 번에.)

---

## Part B — piene_g1_logger

### Task 7: 유닛 템플릿 + `install.sh --render-only`

**Files:**
- Rename: `systemd/g1-logger.service` → `systemd/g1-logger.service.in`
- Modify: `install.sh` (전체 교체 — SDK 부분은 Task 8 에서 채우지만 이 태스크에서 함수 자리를 만든다)
- Test: `tests/test_install_render.py`

**Interfaces:**
- Produces: `bash install.sh --render-only` → 렌더된 유닛을 stdout 에 출력하고 종료 0 (파일 안 씀, pip/systemctl 안 함). 유닛의 `WorkingDirectory`/`ExecStart` 는 **체크아웃 절대경로**. Task 8 이 `install_sdk()` 함수를 채운다.

- [ ] **Step 1: 실패하는 테스트**

`tests/test_install_render.py`:
```python
"""install.sh 가 유닛을 «체크아웃 경로» 로 렌더하는지. drop-in/PYTHONPATH 로 남의 워크스페이스를 빌리지 않는지."""
import os, subprocess, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]

def render():
    r = subprocess.run(["bash", str(ROOT / "install.sh"), "--render-only"],
                       cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout

def test_unit_uses_checkout_path():
    out = render()
    assert f"WorkingDirectory={ROOT}" in out
    assert f"ExecStart={ROOT}/.venv/bin/python -m g1_logger.main --config {ROOT}/config/g1_logger.yaml" in out

def test_unit_has_no_home_relative_or_pythonpath():
    out = render()
    assert "%h/piene_g1_logger" not in out
    assert "PYTHONPATH" not in out
    assert "garry_ws" not in out

def test_template_has_placeholder():
    tpl = (ROOT / "systemd" / "g1-logger.service.in").read_text()
    assert "__REPO__" in tpl and "%h/piene_g1_logger" not in tpl
```

- [ ] **Step 2: 실패 확인** — `cd ~/piene_g1_logger && .venv/bin/python -m pytest tests/test_install_render.py -q 2>/dev/null || python3 -m pytest tests/test_install_render.py -q`
Expected: FAIL (`--render-only` 를 모르고 실제 venv 를 만들려 함 / 템플릿 파일 없음).

- [ ] **Step 3: 템플릿**
```bash
cd ~/piene_g1_logger && git mv systemd/g1-logger.service systemd/g1-logger.service.in
```
`systemd/g1-logger.service.in` 내용:
```ini
# systemd/g1-logger.service.in — install.sh 가 __REPO__ 를 체크아웃 절대경로로 치환해
# ~/.config/systemd/user/g1-logger.service 로 쓴다. 손으로 복사하지 않는다.
# SDK(unitree_sdk2py·cyclonedds)는 .venv 안에 있다 — drop-in/PYTHONPATH 로 밖을 빌리지 않는다.
[Unit]
Description=G1 real-robot logger (DDS -> CSV + WebSocket)
After=network.target

[Service]
Type=simple
WorkingDirectory=__REPO__
ExecStart=__REPO__/.venv/bin/python -m g1_logger.main --config __REPO__/config/g1_logger.yaml
Restart=on-failure
RestartSec=3
Nice=5

[Install]
WantedBy=default.target
```

- [ ] **Step 4: install.sh 골격**
```bash
#!/usr/bin/env bash
# install.sh — 온보드(Jetson) 로거 설치. 멱등. clone 후:  bash install.sh
#   ① venv(시스템 python3=3.8) + requirements + pytest   ② SDK 를 «이 venv 에» (cyclonedds 0.10.2 + unitree_sdk2py)
#   ③ 자기검증(import + pytest) — 실패하면 서비스를 켜지 않는다   ④ 유닛 렌더(체크아웃 경로) + enable --now + linger
# 옵션: --render-only  유닛만 stdout 에 출력 (테스트용, 아무것도 안 만든다)
#       --no-service   ①②③ 만 (서비스 등록 생략)
# 2026-08-13 사고: SDK 가 venv 에 없어 서비스가 일주일 crash-loop. 2026-09-01: SDK 가 남의 워크스페이스(garry_ws)를
# drop-in PYTHONPATH 로 빌리고 있었다 → 새 로봇에선 조용히 죽는다. 그래서 SDK 를 venv 에 직접 넣고 끝에 검증한다.
set -euo pipefail
cd "$(dirname "$0")"
REPO="$(pwd)"
UNIT_SRC="$REPO/systemd/g1-logger.service.in"
UNIT_DST="$HOME/.config/systemd/user/g1-logger.service"
render_unit() { sed "s|__REPO__|$REPO|g" "$UNIT_SRC"; }
case "${1:-}" in
  --render-only) render_unit; exit 0 ;;
esac
NO_SERVICE=""; [ "${1:-}" = "--no-service" ] && NO_SERVICE=1

echo "== ① venv =="
[ -x .venv/bin/python ] || python3 -m venv .venv
./.venv/bin/pip install -q -U pip
./.venv/bin/pip install -q -r requirements.txt pytest

echo "== ② SDK → .venv =="
install_sdk   # Task 8 에서 정의 (여기서는 함수가 없으면 실패한다 — 의도)

echo "== ③ 자기검증 =="
./.venv/bin/python -c "import unitree_sdk2py, cyclonedds; print('  sdk', unitree_sdk2py.__file__); print('  dds', cyclonedds.__file__)"
./.venv/bin/python -m pytest -q tests/ || { echo "🔴 pytest 실패 — 서비스를 켜지 않는다"; exit 1; }

[ -z "$NO_SERVICE" ] || { echo "== --no-service: 여기까지 =="; exit 0; }
echo "== ④ 서비스 =="
raw_log_dir=$(grep '^log_dir' config/g1_logger.yaml | awk '{print $2}'); mkdir -p "${raw_log_dir/#\~/$HOME}"
mkdir -p "$(dirname "$UNIT_DST")"
if [ -d "$UNIT_DST.d" ]; then echo "  기존 drop-in 제거 (내용 백업 → /tmp/g1-logger.service.d.bak):"; cat "$UNIT_DST.d"/*; cp -r "$UNIT_DST.d" /tmp/g1-logger.service.d.bak; rm -rf "$UNIT_DST.d"; fi
render_unit > "$UNIT_DST"
systemctl --user daemon-reload
systemctl --user enable --now g1-logger.service
loginctl enable-linger "$USER" || true
sleep 3; systemctl --user --no-pager --lines=5 status g1-logger.service || true
echo "🟢 설치 완료. 60 초 뒤 $(echo "${raw_log_dir/#\~/$HOME}") 에 realrobot_*.csv 가 생겨야 한다."
```

- [ ] **Step 5: 테스트 통과** — `python3 -m pytest tests/test_install_render.py -q` → 3 passed. (`--render-only` 는 `install_sdk` 에 닿기 전에 종료하므로 통과한다.)

- [ ] **Step 6: Commit**
```bash
git add systemd/g1-logger.service.in install.sh tests/test_install_render.py
git commit -m "install: 유닛을 체크아웃 경로로 렌더한다(%h 하드코딩·drop-in 제거) + --render-only 테스트"
```

---

### Task 8: SDK 를 venv 에 — `install_sdk()` + 메타데이터/문서

**Files:**
- Modify: `install.sh` (`install_sdk` 정의를 `render_unit` 아래에 추가)
- Modify: `pyproject.toml` (`requires-python = ">=3.8"`), `.gitignore` (`.sdk/`), `README.md` «온보드 배포» 2번, `DEPLOY.md` §1.3

**Interfaces:**
- Consumes: 로봇의 공장 CycloneDDS 빌드 `~/cyclonedds/install`(libddsc.so). 다른 로봇에서 다르면 `CYCLONEDDS_HOME` 으로 준다.
- Produces: `.venv` 안에 `cyclonedds==0.10.2`, `unitree_sdk2py`(editable, `.sdk/unitree_sdk2_python`).

- [ ] **Step 1: 실패하는 테스트 (render 테스트에 «SDK 계약» 검사를 더한다)**

`tests/test_install_render.py` 에 추가:
```python
def test_install_sdk_contract():
    src = (ROOT / "install.sh").read_text()
    assert "install_sdk()" in src, "install_sdk 함수가 없다"
    assert "cyclonedds==0.10.2" in src
    assert "unitreerobotics/unitree_sdk2_python" in src
    assert 'CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"' in src
    assert "garry_ws" not in src
```
Run: `python3 -m pytest tests/test_install_render.py -q` → 1 failed (`install_sdk 함수가 없다`).

- [ ] **Step 2: 함수 추가** — `install.sh` 의 `render_unit()` 정의 바로 아래:
```bash
SDK_DIR="$REPO/.sdk/unitree_sdk2_python"
SDK_REPO="https://github.com/unitreerobotics/unitree_sdk2_python.git"
install_sdk() {
  # cyclonedds 파이썬은 C 라이브러리(libddsc.so)에 링크해 빌드된다. 공장 Jetson 은 ~/cyclonedds/install 에 있다.
  CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"
  [ -f "$CYCLONEDDS_HOME/lib/libddsc.so" ] || {
    echo "🔴 $CYCLONEDDS_HOME/lib/libddsc.so 없음. 후보: ls -d ~/cyclonedds*/install  → CYCLONEDDS_HOME=<경로> bash install.sh"; exit 1; }
  export CYCLONEDDS_HOME
  if ! ./.venv/bin/python -c "import cyclonedds" 2>/dev/null; then
    echo "  cyclonedds==0.10.2 빌드 (CYCLONEDDS_HOME=$CYCLONEDDS_HOME, 2~3 분)"
    ./.venv/bin/pip install -q "cyclonedds==0.10.2"
  fi
  if [ ! -d "$SDK_DIR/.git" ]; then
    mkdir -p "$(dirname "$SDK_DIR")"
    git clone -q --depth 1 "$SDK_REPO" "$SDK_DIR"
  fi
  ./.venv/bin/python -c "import unitree_sdk2py" 2>/dev/null || ./.venv/bin/pip install -q -e "$SDK_DIR"
  echo "  🟢 sdk=$SDK_DIR  dds=$CYCLONEDDS_HOME"
}
```
`pyproject.toml`: `requires-python = ">=3.8"`. `.gitignore` 에 `.sdk/` 한 줄.
`README.md` «온보드 배포 (Jetson)» 2번 항목 아래 문장 «unitree_sdk2py 가 온보드에 이미 설치돼 있지 않으면 …» 을 «`install.sh` 가 `.sdk/` 에 SDK 를 clone 해 `.venv` 에 넣는다. CycloneDDS C 라이브러리 위치가 `~/cyclonedds/install` 이 아니면 `CYCLONEDDS_HOME=<경로> bash install.sh`.» 로 교체. 1번 clone 경로는 `~/dyros_ws/piene_ws/piene_g1_logger`.
`DEPLOY.md` §1.3 제목을 «### 1.3 SDK 는 install.sh 가 `.venv` 에 넣는다 (2026-09-01)» 로, 본문을 위 README 문장 + «예전엔 systemd drop-in `PYTHONPATH` 로 다른 워크스페이스의 SDK 를 빌렸다 — 새 로봇에서 조용히 죽는 원인이라 폐지» 로 교체.

- [ ] **Step 3: 테스트** — `python3 -m pytest tests/ -q` → 전부 passed (기존 21 + 4). `bash -n install.sh`.

- [ ] **Step 4: Commit**
```bash
git add install.sh pyproject.toml .gitignore README.md DEPLOY.md tests/test_install_render.py
git commit -m "install: SDK(cyclonedds 0.10.2 + unitree_sdk2py) 를 자기 venv 에 — garry_ws drop-in 의존 제거, 실패하면 서비스를 켜지 않는다"
git push origin main
```

---

### Task 9: center_g1 로거 재설치 (실기 세션 밖, ~2 분 중단)

- [ ] **Step 1**
```bash
cd ~/piene_automation
./robot_bridge/robot.sh sync-code piene_g1_logger
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/piene_g1_logger && bash install.sh 2>&1 | tail -25'
```
Expected: `🟢 sdk=…/.sdk/unitree_sdk2_python dds=/home/unitree/cyclonedds/install` · pytest passed · `기존 drop-in 제거` · `Active: active (running)`.

- [ ] **Step 2: 검증**
```bash
./robot_bridge/robot.sh sh 'env -u PYTHONPATH ~/dyros_ws/piene_ws/piene_g1_logger/.venv/bin/python -c "import unitree_sdk2py; print(unitree_sdk2py.__file__)"; systemctl --user is-active g1-logger; ls ~/.config/systemd/user/; sleep 65; ls -t ~/dyros_ws/piene_ws/g1_logs | head -2'
./robot_bridge/robot.sh pull
```
Expected: `__file__` 이 `piene_ws/piene_g1_logger/.sdk/...` (garry_ws 아님) · `active` · `g1-logger.service.d` 없음 · 새 CSV · com1 회수.

---

## Part C

### Task 10: 절차 문서 + 실험 노트

- [ ] **Step 1: `claude_rules/procedures/robot_bridge.md` §5-2** 를 `deploy/onboard/README.md` 의 «새 로봇 첫날» 블록과 같은 순서로 교체 (rsync 문장·«`config/g1_logger.yaml` 은 로봇에서 손으로 고쳤으므로」문장 삭제 — 이제 repo 값이 실측값이다). §5-1 표에 «SDK 는 `.venv` 안(`install.sh`) — 다른 사람 워크스페이스를 빌리지 않는다» 한 줄.
- [ ] **Step 2: 실험 노트** `5. AI_workspace/unitree_rl_mjlab/experiments/260901_onboard_bootstrap.md` (형식: date/project/branch/tags → 변경 요약·동기·수정 파일·핵심 구현·결과(Task 6·9 실측: 소요 시간·MB·glibc)·다음 단계). `python3 "5. AI_workspace/_tools/gen_index.py"`.
- [ ] **Step 3: memory** `robot-telemetry-already-built-not-deployed.md` 에 «SDK 는 venv 안(2026-09-01)» 한 줄 갱신.

---

## Self-Review (작성 후 확인)
- 스펙 §3-1 ①~⑦ → Task 2 · §3-2 → Task 1 · §3-3 → Task 3 · §3-4 → Task 4·5·6(Step 5) · §3-5 → Task 7·8·9 · §3-6 → Task 10 · §6 테스트 1~6 → 각 Task 의 테스트 단계 + Task 6·9. 누락 없음.
- `RUNBOOK_g1_from_scratch.md` 는 다른 창이 수정 중이라 의도적으로 제외(README 로 대신).
- 이름 일치: `TELEOP_PY`·`GMR_COMMIT`·`DRY_RUN`·`RESET_GMR`(Task 1) ↔ Task 2 사용 / `install_sdk`·`render_unit`·`--render-only`·`__REPO__`(Task 7) ↔ Task 8 테스트 문자열 일치.
