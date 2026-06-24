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

echo "[run] starting browser GUI (viser) -> /dev/shm/g1_masked_gui ..."
( cd "$REPO" && "$UV" run --with viser python "$HERE/masked_gui.py" ) &
GUI_PID=$!
cleanup() { kill "$GUI_PID" 2>/dev/null; rm -f /dev/shm/g1_masked_gui; }
trap cleanup EXIT INT TERM
sleep 1

echo "[run] starting g1_ctrl --network=$NET  (terminal keyboard: 1/2/3, WASD/QE, p=stop)"
"$G1DIR/build/g1_ctrl" --network="$NET"
