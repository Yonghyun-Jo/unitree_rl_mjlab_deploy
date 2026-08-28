#!/usr/bin/env bash
# Launch the masked-loco controller + browser GUI together from ONE terminal.
#   sim2sim:    ./run_g1_with_gui.sh           (network=lo; start unitree_mujoco first)
#   real robot: ./run_g1_with_gui.sh eth0       (your robot ethernet iface)
#
#   정책 갈아끼우기 (하루에 여러 후보를 시험할 때):
#     ./run_g1_with_gui.sh eth0 --policy v1          ← 오늘의 별칭 (ACTIVE.yaml)
#     ./run_g1_with_gui.sh eth0 --policy 260828_v1_settle55k   ← 실제 슬롯 이름도 된다
#   별칭은 config/policy/ACTIVE.yaml 이 정한다 (policy_slot.py activate 로 바꾼다).
#   🔴 기록(trial 마커)에는 **별칭이 아니라 실제 슬롯 이름**이 남는다 — v1 은 날마다
#      다른 것을 가리키므로 별칭으로 기록하면 나중에 못 읽는다.
#   주지 않으면 config.yaml 의 policy_dir.
#   🔴 파일을 안 고치고, pull 도 재빌드도 없다 — 후보 슬롯을 아침에 한 번 push 해 두면 끝.
#      슬롯이 지금 바이너리보다 새 기능을 요구하면 g1_ctrl 이 기동을 거부하고 알려준다
#      (deploy.yaml 의 requires: ↔ DeployFeatures.h). 재빌드 여부를 사람이 판단하지 않는다.
#     ./run_g1_with_gui.sh --list          있는 슬롯 목록만 보고 끝
#
# The viser GUI runs in the background (writes /dev/shm/g1_masked_gui); g1_ctrl runs in the
# foreground so its terminal keyboard still works as a backup. GUI + g1_ctrl MUST be on the
# same host (shared /dev/shm) — true for the standard tethered control-PC deploy. The browser
# itself may be remote (open <thisHost>:8080). The GUI is killed when g1_ctrl exits.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
G1DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$G1DIR/../../.." && pwd)"
UV="${UV:-$HOME/.local/bin/uv}"
SLOTDIR="$G1DIR/config/policy/mimic_masked"
ACTIVE="$G1DIR/config/policy/ACTIVE.yaml"

# 별칭 -> 실제 슬롯 이름. ACTIVE.yaml 은 «키: 값» 한 줄짜리 평평한 형식이라 sed 로 읽는다
# (로봇에 pyyaml 이 있다고 가정하지 않는다). 없으면 조용히 빈 문자열.
resolve_alias() {
  [[ -f "$ACTIVE" ]] || return 0
  sed -n "s/^[[:space:]]*$1[[:space:]]*:[[:space:]]*\([^#[:space:]]*\).*/\1/p" "$ACTIVE" | head -1
}

list_slots() {
  local aliases=""
  if [[ -f "$ACTIVE" ]]; then
    echo "오늘의 별칭 ($(sed -n 's/^day:[[:space:]]*//p' "$ACTIVE" | head -1)):"
    sed -n 's/^\([a-z][a-z0-9]*\)[[:space:]]*:[[:space:]]*\([^#[:space:]]\+\).*/  \1 -> \2/p' "$ACTIVE" | grep -v '^  day ->'
    echo
  fi
  echo "슬롯 ($SLOTDIR):"
  for d in "$SLOTDIR"/*/; do
    n="$(basename "$d")"
    [[ -f "$d/params/deploy.yaml" ]] || continue
    if [[ -e "$d/exported/policy.onnx" ]]; then st="활성"; else st="보관 (policy_slot.py restore $n)"; fi
    req="$(sed -n '/^requires:/,/^[^ #-]/p' "$d/params/deploy.yaml" 2>/dev/null | grep -c '^  - ' || true)"
    printf "  %-46s  requires %s개  %s\n" "$n" "${req:-0}" "$st"
  done
}

NET="lo"; POLICY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --policy) POLICY="${2:-}"; shift 2 ;;
    --policy=*) POLICY="${1#*=}"; shift ;;
    --list) list_slots; exit 0 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) NET="$1"; shift ;;
  esac
done

if [[ -n "$POLICY" ]]; then
  # 실제 슬롯이 아니면 별칭으로 본다. 실제 이름이 먼저 이긴다(별칭과 슬롯 이름이 겹쳐도 안전).
  if [[ ! -d "$SLOTDIR/$POLICY" ]]; then
    RESOLVED="$(resolve_alias "$POLICY")"
    if [[ -n "$RESOLVED" ]]; then
      echo "[run] 별칭 $POLICY -> $RESOLVED   (ACTIVE.yaml)"
      POLICY="$RESOLVED"
    fi
  fi
  if [[ ! -d "$SLOTDIR/$POLICY" ]]; then
    echo "[run] 🔴 그런 슬롯도 별칭도 없다: $POLICY"; echo; list_slots; exit 1
  fi
  if [[ ! -e "$SLOTDIR/$POLICY/exported/policy.onnx" ]]; then
    echo "[run] 🔴 슬롯 '$POLICY' 은 «보관» 상태다 — 가중치가 이 기계에 없다."
    echo "[run]    com1 에서:  deploy/scripts/policy_slot.py restore $POLICY && policy_slot.py push $POLICY"
    exit 1
  fi
  export G1_POLICY_SLOT="$POLICY"
  echo "[run] policy slot = $POLICY   (config.yaml 대신 이것을 쓴다)"
fi

if [[ ! -x "$G1DIR/build/g1_ctrl" ]]; then
  echo "[run] g1_ctrl not built. Run: (cd $G1DIR && mkdir -p build && cd build && cmake .. && make -j4)"; exit 1
fi

# g1_ctrl 은 키보드 텔레옵(1/2/3, WASD)을 위해 tty 를 -icanon -echo 로 바꾸는데, Ctrl-C 로 죽으면
# 되돌리지 못하고 셸이 raw 로 남는다(g1_ctrl 자체에도 핸들러를 넣었지만, 그쪽이 못 도는 경로
# — SIGKILL, 구버전 바이너리 — 까지 여기서 덮는다). tty 가 아니면 stty 가 실패하므로 조용히 skip.
STTY_SAVE="$(stty -g 2>/dev/null || true)"

echo "[run] starting browser GUI (viser) -> /dev/shm/g1_masked_gui ..."
# 🔴 setsid 로 «자기 프로세스 그룹» 에 띄운다. uv 가 python 을 감싸기 때문에 자식(uv)만 죽이면
#    손자(viser python)가 남는다 — 하루에 여러 번 돌리면 그만큼 쌓이고, 남은 GUI 가 /dev/shm 을
#    계속 써서 다음 실행과 싸운다. 실측으로 매 실행 2개씩 샜다(robot.sh verify 가 지적).
setsid bash -c 'cd "$1" && exec "$2" run --with viser python "$3"' _ "$REPO" "$UV" "$HERE/masked_gui.py" &
GUI_PID=$!
cleanup() {
  # 그룹 전체(-PID)를 죽인다. 그래야 손자까지 간다.
  kill -TERM -"$GUI_PID" 2>/dev/null || kill "$GUI_PID" 2>/dev/null
  sleep 0.3
  kill -KILL -"$GUI_PID" 2>/dev/null || true
  rm -f /dev/shm/g1_masked_gui
  if [[ -n "$STTY_SAVE" ]]; then
    stty "$STTY_SAVE" 2>/dev/null || stty sane 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
sleep 1

# 🔴 «이 시험이 어느 정책이었나» 를 로그 옆에 남긴다.
#    안 남기면 나중에 CSV·그래프만 보고 어느 슬롯이었는지 알 방법이 없다 — 하루에 후보를
#    여러 개 돌리면 특히. 대시보드(real_robot)가 실험 시작시각으로 이 마커를 찾아 붙인다.
#    CSV 와 같은 디렉터리라 robot.sh pull 이 같이 회수한다.
LOGDIR="${G1_LOG_DIR:-$HOME/dyros_ws/piene_ws/g1_logs}"
if mkdir -p "$LOGDIR" 2>/dev/null; then
  MARK="$LOGDIR/trial_$(date +%Y%m%d_%H%M%S).txt"
  {
    echo "slot=${POLICY:-$(grep -oP 'mimic_masked/\K[^/]+' "$G1DIR/config/config.yaml" | head -1)}"
    echo "net=$NET"
    echo "started=$(date -Is)"
    echo "host=$(hostname)"
    echo "git=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  } > "$MARK" 2>/dev/null && echo "[run] trial marker -> $MARK"
fi

echo "[run] starting g1_ctrl --network=$NET  (terminal keyboard: 1/2/3, WASD/QE, p=stop)"
"$G1DIR/build/g1_ctrl" --network="$NET"
