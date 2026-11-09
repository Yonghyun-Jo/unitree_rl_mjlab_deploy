#!/usr/bin/env bash
# G1 아케이드 기동(2B-link) — g1_ctrl 아케이드 경로(G1_ARCADE=1)를 tmux 세션 g1_arcade 의 창 ctrl 에 띄우고,
# --relay 면 중계 v2(arcade_relay2.py, 포트 8711)를 창 relay 에 띄운다.
#
#   sim2sim(com1):  ./run_g1_arcade.sh lo   --bundle <묶음 폴더> --slot <walk 슬롯> [--relay]   (unitree_mujoco 를 먼저 정확히 1 개)
#   실기(로봇):     ./run_g1_arcade.sh eth0 --bundle <묶음 폴더> --slot 261009_v1_schedobs_stepw12_27k5 --relay
#   그 다음:  tmux attach -t g1_arcade → 창 ctrl 에서 f → m (FSM 키는 사람이 친다) · 브라우저 ?mode=real2&relay=ws://<bind>:8711
#   멈추기:   창 ctrl 에서 p(Passive) → Ctrl-C. 끝난 창은 남겨 둔다(로그를 읽게) — 정리는 tmux kill-session -t g1_arcade
#
# 옵션
#   --relay              중계 v2 를 창 relay 에 띄운다. 인터프리터 = 환경 변수 G1_ARCADE_RELAY_PY
#                        (기본 ~/dyros_ws/piene_ws/piene_g1_logger/.venv/bin/python — 로봇 logger venv: websockets · msgpack)
#   --relay-bind <addr>  중계 bind 주소(기본: lo = 127.0.0.1 · 그 밖 = 192.168.3.2 — 노트북 브라우저가 닿는 로봇 주소)
#   --state-csv <path>   G1_STATE_CSV 를 이 경로로(기본 = <로그 폴더>/arcade_state_<시각>.csv — 쓰는 동안 .part, g1_ctrl 이 끝나면
#                        .csv 로 바꾼다: 로봇 로그 자동 회수는 *.csv 를 가져가며 지우므로 쓰는 중인 파일은 .csv 가 아니어야 한다)
#   로그 폴더 = G1_LOG_DIR(기본 ~/dyros_ws/piene_ws/g1_logs): trial_<시각>.txt(어느 슬롯 · 묶음이었나 — run_g1_with_gui.sh 와 같은
#   마커) · arcade_ctl_<시각>.log(창 ctrl 출력) · arcade_relay_<시각>.log(창 relay 출력).
#   창 ctrl 로 넘기는 환경 변수(있을 때만, 없으면 창에서 지움): G1_ARCADE_FAULT(시험 주입 — lo 에서만, 실기면 거부) ·
#   G1_ARCADE_STATUS_SHM · G1_ARCADE_NAMES(중계에도 같은 경로를 준다) · G1_SAFETY_CSV.
#   (tmux 창은 이 스크립트가 아니라 tmux 서버의 환경을 받는다 — 그래서 필요한 것만 이 스크립트가 직접 넘긴다.)
#
# 거부(아무것도 띄우지 않고 끝 코드 1): 묶음 파일 md5 ≠ manifest · --slot ≠ 묶음 처음 role(walk) 꾸러미의 슬롯 · 슬롯
#   exported/policy.onnx 가 없거나 md5 ≠ 그 꾸러미 skill.json onnx_md5 · g1_ctrl 빌드 없음 · 아케이드 없이 빌드(G1_ARCADE_DIR 없음) ·
#   g1_ctrl 이 이미 돎(pgrep -x g1_ctrl) · lo 인데 unitree_mujoco 가 정확히 1 개가 아님 · tmux 세션 g1_arcade 가 이미 있음 ·
#   --relay 인데 인터프리터가 없거나(websockets · msgpack import 실패) 포트 8711 을 누가 쓰는 중.
# 기동 뒤 사람이 볼 기동 로그 다섯 줄: [slot] · [arcade] 묶음(bundle=… manifest md5 …) · [safety] pos_clamp ON(+ qd_guard ON 확인) ·
#   [g1_controller] CMAKE_BUILD_TYPE = Release(build/CMakeCache.txt) · [arcade] mlockall= 판정. 하나라도 없거나 틀리면 🔴 · 끝 코드 1 ·
#   중계는 띄우지 않는다(g1_ctrl 은 그대로 떠 있다 — 사람이 보고 끈다).
# 🔴 이 스크립트는 로봇을 움직이지 않는다(g1_ctrl 은 Passive 로 뜨고 f · m 은 사람이 친다). 남의 프로세스는 죽이지 않는다(거부만).
set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="$HERE/$(basename "${BASH_SOURCE[0]}")"
G1DIR="$(cd "$HERE/.." && pwd)"
SESSION=g1_arcade
RELAY_PORT=8711
PASS_VARS=(G1_ARCADE_FAULT G1_ARCADE_STATUS_SHM G1_ARCADE_NAMES G1_SAFETY_CSV)

wait_flag() {  # wait_flag <파일> — 기동 신호(launcher 가 창 옵션 · pipe-pane 을 건 뒤 만든다)를 10 s 까지 기다린다
  local i
  for ((i = 0; i < 100; i++)); do [[ -e "$1" ]] && { rm -f "$1"; return 0; }; sleep 0.1; done
  return 1
}

# ── 창 안에서 도는 쪽(내부 — 사람이 부르지 않는다) ─────────────────────────────────────────────────────
if [[ "${1:-}" == "__ctrl" ]]; then
  # __ctrl <신호 파일> <net> <묶음> <슬롯> <state csv> <part 0|1> <g1_ctrl> [KEY=VALUE ...]
  GO="$2" NET="$3" BUNDLE="$4" SLOT="$5" CSV="$6" PART="$7" G1CTRL="$8"
  shift 8
  wait_flag "$GO" || { echo "[run] 🔴 기동 신호가 10 s 안에 안 왔다 — g1_ctrl 을 띄우지 않는다"; exit 1; }
  for v in "${PASS_VARS[@]}"; do unset "$v"; done
  for kv in "$@"; do export "${kv?}"; done   # KEY=VALUE 를 그대로 export
  export G1_ARCADE=1 G1_ARCADE_BUNDLE="$BUNDLE" G1_POLICY_SLOT="$SLOT" G1_STATE_CSV="$CSV"
  echo "[run] g1_ctrl --network=$NET · G1_ARCADE=1 · G1_ARCADE_BUNDLE=$BUNDLE · G1_POLICY_SLOT=$SLOT · G1_STATE_CSV=$CSV${*:+ · $*}"
  # Ctrl-C · 창 닫힘(HUP)은 g1_ctrl 이 받아 끝나고, 이 셸은 남아 tty 복구 · .part 이름 바꾸기를 한다.
  trap ':' INT HUP
  STTY_SAVE="$(stty -g 2>/dev/null || true)"
  "$G1CTRL" --network="$NET"
  RC=$?
  if [[ -n "$STTY_SAVE" ]]; then stty "$STTY_SAVE" 2>/dev/null || stty sane 2>/dev/null; fi
  if [[ "$PART" == 1 ]]; then
    if [[ -s "$CSV" ]]; then mv -f "$CSV" "${CSV%.part}" && echo "[run] G1_STATE_CSV -> ${CSV%.part} ($(wc -l < "${CSV%.part}") 줄)"
    else echo "[run] ⚠ G1_STATE_CSV 가 비었다 ($CSV)"; fi
  fi
  echo "[run] g1_ctrl 끝 (rc=$RC) — 창은 남겨 둔다. 정리: tmux kill-session -t $SESSION"
  exit "$RC"
fi
if [[ "${1:-}" == "__relay" ]]; then
  # __relay <신호 파일> <python> <relay 인자 ...>
  GO="$2"; shift 2
  wait_flag "$GO" || { echo "[run] 🔴 기동 신호가 10 s 안에 안 왔다 — 중계를 띄우지 않는다"; exit 1; }
  exec "$@"
fi

# ── 기동(사람이 부르는 쪽) ──────────────────────────────────────────────────────────────────────────
die() { echo "[run] 🔴 $*" >&2; exit 1; }
usage() { sed -n '2,9p' "$SELF"; exit 2; }

NET=""; BUNDLE=""; SLOT=""; RELAY=0; RELAY_BIND=""; STATE_CSV=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bundle) BUNDLE="${2:-}"; shift 2 ;;
    --slot) SLOT="${2:-}"; shift 2 ;;
    --relay) RELAY=1; shift ;;
    --relay-bind) RELAY_BIND="${2:-}"; shift 2 ;;
    --state-csv) STATE_CSV="${2:-}"; shift 2 ;;
    -h|--help) awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$SELF"; exit 0 ;;
    -*) echo "모르는 옵션: $1" >&2; usage ;;
    *) [[ -z "$NET" ]] || { echo "인터페이스를 두 번 줬다: $NET · $1" >&2; usage; }; NET="$1"; shift ;;
  esac
done
[[ -n "$NET" && -n "$BUNDLE" && -n "$SLOT" ]] || usage
[[ "$NET" == lo || -d "/sys/class/net/$NET" ]] || die "네트워크 인터페이스 $NET 이 없다(sim2sim = lo · 실기 = 로봇 NIC, 보통 eth0)"
command -v tmux >/dev/null || die "tmux 가 없다"
command -v python3 >/dev/null || die "python3 가 없다(묶음 md5 대조에 쓴다)"
if [[ -n "${G1_ARCADE_FAULT:-}" && "$NET" != lo ]]; then
  die "G1_ARCADE_FAULT=$G1_ARCADE_FAULT 가 설정돼 있다 — 시험 주입은 sim2sim(lo)에서만. 실기에서는 지우고 다시"
fi

# 빌드: config 는 /proc/self/exe 의 부모(build)의 ../config 에서 읽는다(param.h) — 이 자리의 바이너리만 쓴다
G1CTRL="$G1DIR/build/g1_ctrl"
CACHE="$G1DIR/build/CMakeCache.txt"
[[ -x "$G1CTRL" ]] || die "g1_ctrl 빌드가 없다: $G1CTRL (cmake .. -DCMAKE_BUILD_TYPE=Release -DG1_ARCADE_DIR=<g1_arcade> && make)"
[[ -f "$CACHE" ]] || die "$CACHE 가 없다 — 빌드 종류 · 아케이드 여부를 확인할 수 없다"
BUILD_TYPE="$(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' "$CACHE" | head -1)"
ARC_DIR="$(sed -n 's/^G1_ARCADE_DIR:[A-Z]*=//p' "$CACHE" | head -1)"
[[ -n "$ARC_DIR" ]] || die "이 g1_ctrl 은 아케이드 없이 빌드됐다(CMakeCache 에 G1_ARCADE_DIR 없음) — g1_ctrl 이 G1_ARCADE=1 기동을 거부한다"

# 묶음 · 슬롯(python3 표준 라이브러리만 — 로봇 3.8)
[[ -f "$BUNDLE/manifest.json" ]] || die "묶음이 없다: $BUNDLE/manifest.json"
BUNDLE="$(cd "$BUNDLE" && pwd)"
CHECK="$(python3 - "$BUNDLE" "$SLOT" "$G1DIR" <<'PYEOF'
import hashlib, json, os, sys
b, slot, g1 = sys.argv[1:4]
def md5(p):
    h = hashlib.md5()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()
m = json.load(open(os.path.join(b, "manifest.json")))
bad = [r for r, want in m["files"].items() if not os.path.isfile(os.path.join(b, r)) or md5(os.path.join(b, r)) != want]
if bad:
    print("ERR 묶음 파일 md5 가 manifest 와 다르다(또는 없다): %s" % ", ".join(bad)); sys.exit(0)
d = json.load(open(os.path.join(b, "data/demo_set.json")))
first = [s for s in d["skills"] if s["role"] == d["default"]]
if not first:
    print("ERR 묶음 demo_set 에 처음 role '%s' 꾸러미가 없다" % d["default"]); sys.exit(0)
sk = json.load(open(os.path.join(b, "skills", first[0]["id"], "skill.json")))
want_slot = sk.get("slot") or ""
if want_slot != slot:
    print("ERR --slot %s ≠ 묶음 처음 role '%s' 꾸러미 %s 의 슬롯 '%s' — g1_ctrl 안전층(pos_clamp · qd 임계)이 그 슬롯 것이어야 한다"
          % (slot, d["default"], first[0]["id"], want_slot)); sys.exit(0)
onnx = os.path.join(g1, "config/policy/mimic_masked", slot, "exported/policy.onnx")
if not os.path.isfile(onnx):
    print("ERR 슬롯 '%s' 의 가중치가 이 기계에 없다(%s) — «보관» 슬롯이면 com1 에서 policy_slot.py restore 뒤 전송" % (slot, onnx)); sys.exit(0)
got, want = md5(onnx), sk.get("onnx_md5") or ""
if got != want:
    print("ERR 슬롯 '%s' policy.onnx md5 %s ≠ 꾸러미 %s skill.json onnx_md5 %s — 다른 가중치" % (slot, got, first[0]["id"], want)); sys.exit(0)
roles = ",".join(s["role"] for s in d["skills"])
print("OK %s %s %d %s %s %s" % (md5(os.path.join(b, "manifest.json")), m.get("g1_arcade_commit", "?"), len(m["files"]), roles,
                              "robot" if m.get("robot") else "sim", "dirty" if m.get("dirty") else "clean"))
PYEOF
)" || die "묶음 검사 실행 실패"
case "$CHECK" in
  OK\ *) read -r _ MANIFEST_MD5 B_COMMIT B_NFILES B_ROLES B_KIND B_DIRTY <<< "$CHECK" ;;
  ERR\ *) die "${CHECK#ERR }" ;;
  *) die "묶음 검사 결과를 못 읽었다: $CHECK" ;;
esac

# 프로세스 · 세션 관문(«정책 무관 즉시 발산» 의 1순위 원인 — 두 제어기 · 섞인 lowstate)
if pgrep -x g1_ctrl >/dev/null; then
  die "g1_ctrl 이 이미 떠 있다(PID $(pgrep -x g1_ctrl | tr '\n' ' ')) — 두 제어기는 정책과 무관하게 발산한다. 그것을 먼저 끝낼 것"
fi
if [[ "$NET" == lo ]]; then
  NSIM="$(pgrep -x unitree_mujoco | wc -l)"
  [[ "$NSIM" -eq 1 ]] || die "unitree_mujoco 가 ${NSIM} 개다(PID $(pgrep -x unitree_mujoco | tr '\n' ' ')) — sim2sim(lo)은 정확히 1 개(0 = 시뮬 먼저 · 2+ = lowstate 가 섞인다)"
fi
tmux has-session -t "=$SESSION" 2>/dev/null && die "tmux 세션 $SESSION 이 이미 있다 — 끝난 것이면 tmux kill-session -t $SESSION 뒤 다시"

# 중계 준비(띄우기 전에 다 확인)
RELAY_PY="${G1_ARCADE_RELAY_PY:-$HOME/dyros_ws/piene_ws/piene_g1_logger/.venv/bin/python}"
RELAY_ARGS=()
if [[ $RELAY -eq 1 ]]; then
  [[ -n "$RELAY_BIND" ]] || { if [[ "$NET" == lo ]]; then RELAY_BIND=127.0.0.1; else RELAY_BIND=192.168.3.2; fi; }
  command -v "$RELAY_PY" >/dev/null || die "중계 인터프리터가 없다: $RELAY_PY (G1_ARCADE_RELAY_PY 로 바꾼다)"
  "$RELAY_PY" -c 'import websockets, msgpack' 2>/dev/null || die "$RELAY_PY 에 websockets · msgpack 이 없다"
  [[ -f "$HERE/arcade_relay2.py" ]] || die "중계 v2 가 없다: $HERE/arcade_relay2.py"
  if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$RELAY_PORT\$"; then
    die "포트 $RELAY_PORT 를 누가 쓰고 있다(ss -ltnp 로 확인) — 중계 v2 가 이미 떠 있으면 그것을 먼저 끝낼 것"
  fi
  RELAY_ARGS=(--bind "$RELAY_BIND" --port "$RELAY_PORT")
  [[ -n "${G1_ARCADE_STATUS_SHM:-}" ]] && RELAY_ARGS+=(--status-shm "$G1_ARCADE_STATUS_SHM")
  [[ -n "${G1_ARCADE_NAMES:-}" ]] && RELAY_ARGS+=(--names "$G1_ARCADE_NAMES")
elif [[ -n "$RELAY_BIND" ]]; then
  die "--relay-bind 는 --relay 와 같이"
fi

# 로그 폴더 · 마커
LOGDIR="${G1_LOG_DIR:-$HOME/dyros_ws/piene_ws/g1_logs}"
mkdir -p "$LOGDIR" || die "로그 폴더 $LOGDIR 를 못 만든다"
LOGDIR="$(cd "$LOGDIR" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
CTL_LOG="$LOGDIR/arcade_ctl_$STAMP.log"
RELAY_LOG="$LOGDIR/arcade_relay_$STAMP.log"
if [[ -n "$STATE_CSV" ]]; then
  mkdir -p "$(dirname "$STATE_CSV")" || die "G1_STATE_CSV 폴더를 못 만든다: $STATE_CSV"
  STATE_CSV="$(cd "$(dirname "$STATE_CSV")" && pwd)/$(basename "$STATE_CSV")"; PART=0
else
  STATE_CSV="$LOGDIR/arcade_state_$STAMP.csv.part"; PART=1
fi
PASS=()
for v in "${PASS_VARS[@]}"; do [[ -n "${!v:-}" ]] && PASS+=("$v=${!v}"); done
REPO="$(cd "$G1DIR/../../.." && pwd)"
MARK="$LOGDIR/trial_$STAMP.txt"
{
  echo "slot=$SLOT"
  echo "net=$NET"
  echo "started=$(date -Is)"
  echo "host=$(hostname)"
  echo "git=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "arcade=1"
  echo "arcade_bundle=$BUNDLE"
  echo "arcade_manifest_md5=$MANIFEST_MD5"
  echo "arcade_g1_arcade=$ARC_DIR"
  echo "state_csv=${STATE_CSV%.part}"
  echo "relay=$([[ $RELAY -eq 1 ]] && echo "ws://$RELAY_BIND:$RELAY_PORT" || echo off)"
  if [[ ${#PASS[@]} -gt 0 ]]; then echo "env=${PASS[*]}"; fi
} > "$MARK" || die "마커 $MARK 를 못 쓴다"
echo "[run] 묶음 $BUNDLE · manifest md5 $MANIFEST_MD5 · g1_arcade $B_COMMIT · 파일 $B_NFILES · $B_KIND · $B_DIRTY · role [$B_ROLES]"
echo "[run] 슬롯 $SLOT(가중치 md5 = 꾸러미 onnx_md5) · g1_ctrl $G1CTRL · 빌드 $BUILD_TYPE · core $ARC_DIR"
echo "[run] 마커 $MARK · 창 ctrl 출력 $CTL_LOG"

# ── 창 ctrl ─────────────────────────────────────────────────────────────────────────────────────
RUNTMP="$(mktemp -d /tmp/g1_arcade_run.XXXXXX)" || die "임시 폴더를 못 만든다"
trap 'rm -rf "$RUNTMP"' EXIT
CMD="$(printf '%q ' bash "$SELF" __ctrl "$RUNTMP/go_ctrl" "$NET" "$BUNDLE" "$SLOT" "$STATE_CSV" "$PART" "$G1CTRL" "${PASS[@]}")"
tmux new-session -d -s "$SESSION" -n ctrl -x 220 -y 50 "$CMD" || die "tmux 세션 $SESSION 을 못 만들었다"
# 창이 바로 끝나도(기동 거부) 출력이 남게 — 기동 신호 전에 건다
tmux set-option -w -t "$SESSION:ctrl" remain-on-exit on >/dev/null || die "remain-on-exit 를 못 걸었다(세션이 이미 끝났나?)"
tmux pipe-pane -t "$SESSION:ctrl" -o "cat >> $(printf '%q' "$CTL_LOG")" || die "pipe-pane 을 못 걸었다"
touch "$RUNTMP/go_ctrl"

clean_log() { sed -e 's/\x1b\[[0-9;?]*[A-Za-z]//g' -e 's/\r//g' "$1" 2>/dev/null; }
msg_of() { sed -E 's/^\[[0-9: .-]+\] \[[a-z]+\] //'; }   # spdlog 머리(시각 · 등급)를 뗀다
pane_alive() { [[ "$(tmux list-panes -t "$SESSION:$1" -F '#{pane_dead}' 2>/dev/null)" == 0 ]]; }

ALIVE=1
for ((i = 0; i < 600; i++)); do   # 60 s — ORT 세션 · 예열(로봇은 com1 보다 느리다)
  L="$(clean_log "$CTL_LOG")"
  grep -qF '[arcade] mlockall=' <<< "$L" && break
  grep -qF '기동 거부' <<< "$L" && break
  if ! pane_alive ctrl; then ALIVE=0; break; fi
  sleep 0.1
done
sleep 0.3
L="$(clean_log "$CTL_LOG")"
pane_alive ctrl || ALIVE=0

L_SLOT="$(grep -m1 -F "[slot] G1_POLICY_SLOT='$SLOT'" <<< "$L" | msg_of)"
L_ARC="$(grep -m1 -F "[arcade] bundle=$BUNDLE " <<< "$L" | msg_of)"
L_SAFE="$(grep -m1 -E '\[safety\] pos_clamp ON' <<< "$L" | msg_of)"
L_QD="$(grep -m1 -E '\[safety\] qd_guard ON' <<< "$L" | msg_of)"
L_MLOCK="$(grep -m1 -F '[arcade] mlockall=' <<< "$L" | msg_of)"
L_BUILD="[g1_controller] CMAKE_BUILD_TYPE = ${BUILD_TYPE:-(없음)}"
BAD=0
line() {  # line <좋은가 0|1> <표시할 줄> <없을 때 설명>
  if [[ "$1" == 1 ]]; then echo "  ✅ $2"; else echo "  🔴 $3"; BAD=$((BAD + 1)); fi
}
echo "[run] ── 기동 로그 다섯 줄 (창 $SESSION:ctrl · 전체 $CTL_LOG) ──"
line "$([[ -n "$L_SLOT" ]] && echo 1 || echo 0)" "$L_SLOT" "[slot] G1_POLICY_SLOT='$SLOT' 줄이 없다"
line "$([[ -n "$L_ARC" && "$L_ARC" == *"manifest md5 $MANIFEST_MD5"* ]] && echo 1 || echo 0)" "$L_ARC" \
  "[arcade] bundle=$BUNDLE … manifest md5 $MANIFEST_MD5 줄이 없다${L_ARC:+ (있는 줄: $L_ARC)}"
line "$([[ -n "$L_SAFE" && -n "$L_QD" ]] && echo 1 || echo 0)" "$L_SAFE" \
  "[safety] pos_clamp ON · qd_guard ON 이 다 있어야 한다(pos_clamp: ${L_SAFE:-없음} · qd_guard: ${L_QD:-없음})"
line "$([[ "$BUILD_TYPE" == Release ]] && echo 1 || echo 0)" "$L_BUILD" "$L_BUILD — Release 가 아니면 sim2sim · 실기가 학습 시간 예산을 대표하지 못한다"
line "$([[ -n "$L_MLOCK" ]] && echo 1 || echo 0)" "$L_MLOCK" "[arcade] mlockall= 판정 줄이 없다"
L_RT="$(grep -m1 -E '\[rt\].*SCHED_FIFO.*실패' <<< "$L" | msg_of)"
if [[ -n "$L_RT" ]]; then echo "  ⚠ 참고: $L_RT"; fi
L_FAULT="$(grep -m1 -F '시험 주입 켜짐' <<< "$L" | msg_of)"
if [[ -n "$L_FAULT" ]]; then echo "  ⚠ 참고: $L_FAULT"; fi
if [[ $ALIVE -ne 1 ]] || grep -qF '기동 거부' <<< "$L"; then
  echo "[run] 🔴 g1_ctrl 이 기동을 거부했거나 끝났다:"
  grep -E '기동 거부|\[critical\]|\[error\]' <<< "$L" | msg_of | head -8 | sed 's/^/    /'
  echo "[run]    창 출력: tmux attach -t $SESSION (끝난 창은 남아 있다) · 정리: tmux kill-session -t $SESSION"
  exit 1
fi
if [[ $BAD -ne 0 ]]; then
  echo "[run] 🔴 기동 로그 다섯 줄 중 $BAD 개가 없거나 틀리다 — 실기 운용 금지. 중계는 띄우지 않았다."
  echo "[run]    g1_ctrl 은 떠 있다(Passive). 끝내기: tmux attach -t $SESSION → Ctrl-C · 정리: tmux kill-session -t $SESSION"
  exit 1
fi

# ── 창 relay ────────────────────────────────────────────────────────────────────────────────────
if [[ $RELAY -eq 1 ]]; then
  RCMD="$(printf '%q ' bash "$SELF" __relay "$RUNTMP/go_relay" "$RELAY_PY" "$HERE/arcade_relay2.py" "${RELAY_ARGS[@]}")"
  tmux new-window -d -t "$SESSION:" -n relay "$RCMD" || die "창 relay 를 못 만들었다"
  tmux set-option -w -t "$SESSION:relay" remain-on-exit on >/dev/null || die "창 relay 에 remain-on-exit 를 못 걸었다"
  tmux pipe-pane -t "$SESSION:relay" -o "cat >> $(printf '%q' "$RELAY_LOG")" || die "창 relay 에 pipe-pane 을 못 걸었다"
  touch "$RUNTMP/go_relay"
  UP=0
  for ((i = 0; i < 100; i++)); do   # 10 s
    if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$RELAY_PORT\$"; then UP=1; break; fi
    pane_alive relay || break
    sleep 0.1
  done
  if [[ $UP -ne 1 ]]; then
    echo "[run] 🔴 중계 v2 가 포트 $RELAY_PORT 를 열지 못했다 — 창 relay 출력:"
    clean_log "$RELAY_LOG" | tail -8 | sed 's/^/    /'
    echo "[run]    g1_ctrl 은 떠 있다(Passive). 정리: tmux kill-session -t $SESSION"
    exit 1
  fi
  echo "[run] 중계 v2 ws://$RELAY_BIND:$RELAY_PORT (창 relay · 출력 $RELAY_LOG)"
fi
echo "[run] 🟢 떴다. 다음: tmux attach -t $SESSION → 창 ctrl 에서 f → m (사람이 친다)$([[ $RELAY -eq 1 ]] && echo " · 브라우저 ?mode=real2&relay=ws://$RELAY_BIND:$RELAY_PORT")"
echo "[run]    멈추기: 창 ctrl 에서 p → Ctrl-C · 정리: tmux kill-session -t $SESSION"
exit 0
