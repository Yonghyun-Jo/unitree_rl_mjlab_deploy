#!/usr/bin/env bash
# Launch the masked-loco controller + browser GUI together from ONE terminal.
#   sim2sim:    ./run_g1_with_gui.sh           (network=lo; start unitree_mujoco first)
#   real robot: ./run_g1_with_gui.sh enp5s0    (your robot ethernet iface)
#
# The viser GUI runs in the background (writes /dev/shm/g1_masked_gui); g1_ctrl runs in the
# foreground so its terminal keyboard still works as a backup. GUI + g1_ctrl MUST be on the
# same host (shared /dev/shm) — true for the standard tethered control-PC deploy. The browser
# itself may be remote (open <thisHost>:8080). The GUI is killed when g1_ctrl exits.
set -u
NET="${1:-lo}"
HERE="$(cd "$(dirname "$0")" && pwd)"
G1DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$G1DIR/../../.." && pwd)"
UV="${UV:-$HOME/.local/bin/uv}"

if [[ ! -x "$G1DIR/build/g1_ctrl" ]]; then
  echo "[run] g1_ctrl not built. Run: (cd $G1DIR && mkdir -p build && cd build && cmake .. && make -j4)"; exit 1
fi

# g1_ctrl 은 키보드 텔레옵(1/2/3, WASD)을 위해 tty 를 -icanon -echo 로 바꾸는데, Ctrl-C 로 죽으면
# 되돌리지 못하고 셸이 raw 로 남는다(g1_ctrl 자체에도 핸들러를 넣었지만, 그쪽이 못 도는 경로
# — SIGKILL, 구버전 바이너리 — 까지 여기서 덮는다). tty 가 아니면 stty 가 실패하므로 조용히 skip.
STTY_SAVE="$(stty -g 2>/dev/null || true)"

echo "[run] starting browser GUI (viser) -> /dev/shm/g1_masked_gui ..."
( cd "$REPO" && "$UV" run --with viser python "$HERE/masked_gui.py" ) &
GUI_PID=$!
cleanup() {
  kill "$GUI_PID" 2>/dev/null
  rm -f /dev/shm/g1_masked_gui
  if [[ -n "$STTY_SAVE" ]]; then
    stty "$STTY_SAVE" 2>/dev/null || stty sane 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
sleep 1

echo "[run] starting g1_ctrl --network=$NET  (terminal keyboard: 1/2/3, WASD/QE, p=stop)"
"$G1DIR/build/g1_ctrl" --network="$NET"
