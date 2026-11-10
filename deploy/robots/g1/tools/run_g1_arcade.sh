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
#   로그 폴더 = G1_LOG_DIR(기본 ~/dyros_ws/piene_ws/g1_logs): arcade_ctl_<시각>.log(창 ctrl 출력) · arcade_relay_<시각>.log(창 relay 출력)
#   · trial_<시각>.txt(어느 슬롯 · 묶음이었나 — run_g1_with_gui.sh 와 같은 마커, 대시보드 · 회수가 실험 ↔ 슬롯을 이 파일로 잇는다).
#   🔴 마커는 모든 관문(기동 전 검사 · 기동 로그 판정 · 중계)을 통과한 뒤에만 쓴다 — 거부된 기동은 마커를 남기지 않는다.
#   창 ctrl 로 넘기는 환경 변수(있을 때만, 없으면 창에서 지움): G1_ARCADE_FAULT(시험 주입 — lo 에서만, 실기면 거부) ·
#   G1_ARCADE_STATUS_SHM · G1_ARCADE_NAMES(중계에도 같은 경로를 준다) · G1_SAFETY_CSV.
#   (tmux 창은 이 스크립트가 아니라 tmux 서버의 환경을 받는다 — 그래서 필요한 것만 이 스크립트가 직접 넘긴다.)
#   🔴 창 ctrl 의 G1_* 는 허용 목록만 남긴다: 이 스크립트가 넣는 G1_ARCADE · G1_ARCADE_BUNDLE · G1_POLICY_SLOT · G1_STATE_CSV +
#   위 넷 중 넘긴 것. 그 밖 G1_*(실험용 덮어쓰기 G1_IMU_CAL_DEG · G1_FOOTZ_SRC · G1_OBS_CSV · G1_NO_RT · G1_DIAG_LOAD …)는 tmux
#   서버 환경에 남아 있어도 지우고, 지운 이름을 창 ctrl 첫 줄 «[run] 창 ctrl 환경 G1_* …» 에 보인다(기동 판정이 다시 찍는다).
#
# 거부(아무것도 띄우지 않고 끝 코드 1): 묶음 파일 md5 ≠ manifest · --slot ≠ 묶음 처음 role(walk) 꾸러미의 슬롯 · 슬롯
#   exported/policy.onnx 가 없거나 md5 ≠ 그 꾸러미 skill.json onnx_md5 · g1_ctrl 빌드 없음 · 아케이드 없이 빌드(G1_ARCADE_DIR 없음) ·
#   빌드가 낡음(CMakeCache.txt · 빌드 정보 헤더 · 컴파일되는 소스(g1 main.cpp · src · include · cmake · deploy/include · 아케이드
#   core · host) 중 g1_ctrl 보다 새것이 있음) · g1_ctrl 이 이미 돎(pgrep -x g1_ctrl) · lo 인데 unitree_mujoco 가 정확히 1 개가 아님 ·
#   tmux 세션 g1_arcade 가 이미 있음 · --relay 인데 인터프리터가 없거나(websockets · msgpack import 실패) 포트 8711 을 누가 쓰는 중.
# 기동 로그 다섯 줄 — ①②③⑤ 는 g1_ctrl 실행 로그에서 본 줄, ④ 는 빌드 캐시에서 읽은 줄(실행 로그가 아니다):
#   ① [slot] G1_POLICY_SLOT='<슬롯>' ② [arcade] bundle=<묶음> … manifest md5 <md5> ③ Mimic_Masked 의 [safety] pos_clamp ON · qd_guard ON
#   (값이 ON 이어야 함) ④ [g1_controller] CMAKE_BUILD_TYPE = Release(build/CMakeCache.txt — 위 «낡음» 검사로 캐시 · 소스가 이 바이너리와
#   맞는지 묶음, 바이너리 안의 빌드 커밋 문자열도 보인다) ⑤ [arcade] mlockall=ok — 실기(lo 가 아님)는 ok 가 아니면 🔴, lo 는 ⚠.
#   + [rt] FSM 1kHz -> SCHED_FIFO 성공 줄 — 실기는 없거나 실패면 🔴(감시기의 «느린 틱 하나 ≈ 80 ms» 예산은 로봇 FIFO 전제), lo 는 ⚠ 참고.
#   🔴 가 하나라도 있으면 끝 코드 1 · 중계 안 띄움 · 마커 안 씀(g1_ctrl 은 Passive 로 떠 있다 — 사람이 보고 끈다).
# 중계는 «이 창에서 띄운 그 pid» 가 :8711 LISTEN 소켓을 쥐었을 때만 떴다고 본다(/proc/net/tcp 의 inode ↔ /proc/<pid>/fd).
# 판정만 다시(시험용 · 아무것도 띄우지 않음): ./run_g1_arcade.sh __judge <net> <창 ctrl 로그> <슬롯> <묶음 폴더> <manifest md5>
# 🔴 이 스크립트는 로봇을 움직이지 않는다(g1_ctrl 은 Passive 로 뜨고 f · m 은 사람이 친다). 남의 프로세스는 죽이지 않는다(거부만).
set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="$HERE/$(basename "${BASH_SOURCE[0]}")"
G1DIR="$(cd "$HERE/.." && pwd)"
SESSION=g1_arcade
RELAY_PORT=8711
PASS_VARS=(G1_ARCADE_FAULT G1_ARCADE_STATUS_SHM G1_ARCADE_NAMES G1_SAFETY_CSV)
G1CTRL="$G1DIR/build/g1_ctrl"
CACHE="$G1DIR/build/CMakeCache.txt"
BINFO="$G1DIR/build/generated/g1_build_info.h"

wait_flag() {  # wait_flag <파일> — 기동 신호(launcher 가 창 옵션 · pipe-pane 을 건 뒤 만든다)를 10 s 까지 기다린다
  local i
  for ((i = 0; i < 100; i++)); do [[ -e "$1" ]] && { rm -f "$1"; return 0; }; sleep 0.1; done
  return 1
}
clean_log() { sed -e 's/\x1b\[[0-9;?]*[A-Za-z]//g' -e 's/\r//g' "$1" 2>/dev/null; }
msg_of() { sed -E 's/^\[[0-9: .-]+\] \[[a-z]+\] //'; }   # spdlog 머리(시각 · 등급)를 뗀다

# 빌드 검사 — BUILD_TYPE · ARC_DIR · BUILD_STALE(낡음 이유, 없으면 빈 값) · BUILD_NOTE(빌드 커밋 · dirty — 보여 주기만)
check_build() {
  local newer c d head
  BUILD_TYPE="$(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' "$CACHE" 2>/dev/null | head -1)"
  ARC_DIR="$(sed -n 's/^G1_ARCADE_DIR:[A-Z]*=//p' "$CACHE" 2>/dev/null | head -1)"
  BUILD_STALE=""
  if [[ "$CACHE" -nt "$G1CTRL" ]]; then BUILD_STALE="CMakeCache.txt 가 g1_ctrl 보다 새것(다시 구성한 뒤 make 안 함)"
  elif [[ -f "$BINFO" && "$BINFO" -nt "$G1CTRL" ]]; then BUILD_STALE="빌드 정보 헤더가 g1_ctrl 보다 새것(빌드가 끝나지 않음?)"
  else
    newer="$(find "$G1DIR/main.cpp" "$G1DIR/CMakeLists.txt" "$G1DIR/src" "$G1DIR/include" "$G1DIR/cmake" "$G1DIR/../../include" \
               "$ARC_DIR/core/CMakeLists.txt" "$ARC_DIR/core/src" "$ARC_DIR/core/include" "$ARC_DIR/core/vendor" \
               "$ARC_DIR/host/CMakeLists.txt" "$ARC_DIR/host/src" "$ARC_DIR/host/include" \
               -type f -newer "$G1CTRL" 2>/dev/null | head -3 | tr '\n' ' ')"
    [[ -z "$newer" ]] || BUILD_STALE="g1_ctrl 보다 새 소스: $newer"
  fi
  c="$(sed -n 's/^#define G1_BUILD_COMMIT "\(.*\)"/\1/p' "$BINFO" 2>/dev/null)"
  d="$(sed -n 's/^#define G1_BUILD_DIRTY \(.*\)/\1/p' "$BINFO" 2>/dev/null)"
  head="$(git -C "$G1DIR" rev-parse --short=12 HEAD 2>/dev/null || echo '?')"
  if [[ -z "$c" ]]; then BUILD_NOTE="빌드 커밋 모름(헤더 없음)"
  elif ! grep -aqF "$c" "$G1CTRL"; then BUILD_NOTE="빌드 커밋 $c 이 바이너리 안에 없다"; BUILD_STALE="${BUILD_STALE:-빌드 정보 헤더 커밋 $c 이 g1_ctrl 안에 없다}"
  else BUILD_NOTE="바이너리 안 빌드 커밋 $c · dirty $d · 지금 HEAD $head$([[ "$c" == "$head" && "$d" == 0 ]] || echo ' ⚠ 다름(컴파일 소스는 바이너리보다 새것 없음)')"
  fi
}

# 기동 로그 판정 — judge_startup <net> <로그(색 · CR 뗀 것)> <슬롯> <묶음> <manifest md5>. BAD = 🔴 수(⚠ 는 안 셈)
judge_startup() {
  local net="$1" L="$2" slot="$3" bundle="$4" md5="$5" seg l_slot l_arc l_pc l_qd l_rl l_ml l_rt l_fault real=0
  [[ "$net" != lo ]] && real=1
  BAD=0
  ok() { echo "  ✅ $*"; }
  bad() { echo "  🔴 $*"; BAD=$((BAD + 1)); }
  warn() { echo "  ⚠ $*"; }
  l_slot="$(grep -m1 -F "[slot] G1_POLICY_SLOT='$slot'" <<< "$L" | msg_of)"
  # Mimic_Masked 의 안전 줄 = 그 [slot] 줄 뒤 · [arcade] bundle= 줄 앞(다른 Mimic 상태도 [safety] 줄을 찍는다)
  seg="$(awk -v s="[slot] G1_POLICY_SLOT='$slot'" 'index($0, s) { on = 1; next } on && index($0, "[arcade] bundle=") { exit } on' <<< "$L")"
  l_pc="$(grep -m1 -F '[safety] pos_clamp ' <<< "$seg" | msg_of)"
  l_qd="$(grep -m1 -F '[safety] qd_guard ' <<< "$seg" | msg_of)"
  l_rl="$(grep -m1 -F '[safety] rate_limit ' <<< "$seg" | msg_of)"
  l_arc="$(grep -m1 -F "[arcade] bundle=$bundle " <<< "$L" | msg_of)"
  l_ml="$(grep -m1 -F '[arcade] mlockall=' <<< "$L" | msg_of)"
  l_rt="$(grep -m1 -F '[rt] FSM 1kHz' <<< "$L" | msg_of)"
  l_fault="$(grep -m1 -F '시험 주입 켜짐' <<< "$L" | msg_of)"
  l_env="$(grep -m1 -F '[run] 창 ctrl 환경 G1_* — ' <<< "$L")"
  echo "[run] ── 기동 로그 다섯 줄 — ①②③⑤ = g1_ctrl 실행 로그에서 본 줄 · ④ = 빌드 캐시에서 읽은 줄(실행 로그 아님) ──"
  if [[ -n "$l_env" ]]; then echo "  · ${l_env#\[run\] }"
  else bad "창 ctrl 환경 정리 줄(«[run] 창 ctrl 환경 G1_* — …»)이 없다 — 허용 목록 밖 G1_* 가 g1_ctrl 에 남았는지 모른다"; fi
  if [[ -n "$l_slot" ]]; then ok "① $l_slot"; else bad "① [slot] G1_POLICY_SLOT='$slot' 줄이 없다"; fi
  if [[ -n "$l_arc" && "$l_arc" == *"manifest md5 $md5"* ]]; then ok "② $l_arc"
  else bad "② [arcade] bundle=$bundle … manifest md5 $md5 줄이 없다${l_arc:+ (있는 줄: $l_arc)}"; fi
  if [[ "$l_pc" == "[safety] pos_clamp ON"* ]]; then ok "③ $l_pc"; else bad "③ Mimic_Masked [safety] pos_clamp 가 ON 이 아니다: ${l_pc:-줄 없음}"; fi
  if [[ "$l_qd" == "[safety] qd_guard ON"* ]]; then ok "③ $l_qd"; else bad "③ Mimic_Masked [safety] qd_guard 가 ON 이 아니다: ${l_qd:-줄 없음}"; fi
  if [[ "$BUILD_TYPE" == Release && -z "$BUILD_STALE" ]]; then
    ok "④ (빌드 캐시 — 실행 로그 아님) [g1_controller] CMAKE_BUILD_TYPE = Release · build/CMakeCache.txt · 캐시 · 컴파일 소스가 g1_ctrl 보다 새것 없음 · $BUILD_NOTE"
  else bad "④ (빌드 캐시) [g1_controller] CMAKE_BUILD_TYPE = ${BUILD_TYPE:-(없음)}${BUILD_STALE:+ · 낡음: $BUILD_STALE} — Release · 최신 빌드여야 한다"; fi
  if [[ "$l_ml" == "[arcade] mlockall=ok ("* ]]; then ok "⑤ $l_ml"
  elif [[ -z "$l_ml" ]]; then bad "⑤ [arcade] mlockall= 판정 줄이 없다"
  elif [[ $real -eq 1 ]]; then bad "⑤ $l_ml — 실기는 mlockall=ok 여야 한다(memlock 한도 · CAP_IPC_LOCK 확인: setcap cap_sys_nice,cap_ipc_lock+ep g1_ctrl)"
  else warn "⑤ $l_ml — lo(sim2sim)라 판정 밖, 실기에서는 🔴"; fi
  if [[ "$l_rt" == "[rt] FSM 1kHz -> SCHED_FIFO prio "* && "$l_rt" != *실패* ]]; then ok "$l_rt"
  elif [[ $real -eq 1 ]]; then bad "${l_rt:-[rt] FSM 1kHz 줄이 없다} — 실기는 SCHED_FIFO 여야 한다(감시기 «느린 틱 하나 ≈ 80 ms» 예산의 전제 · setcap cap_sys_nice)"
  else warn "참고(lo): ${l_rt:-[rt] FSM 1kHz 줄이 없다}"; fi
  if [[ -n "$l_rl" ]]; then echo "  · 참고(판정 밖): $l_rl"; fi
  if [[ -n "$l_fault" ]]; then warn "참고: $l_fault"; fi
}

# 중계 확인 — listener_owned_by <pid> <port>: 그 pid 의 fd 가 <port> LISTEN 소켓(/proc/net/tcp · tcp6 의 inode)을 쥐었나
listener_owned_by() {
  local pid="$1" hex ino fd
  hex="$(printf '%04X' "$2")"
  while read -r ino; do
    for fd in /proc/"$pid"/fd/*; do
      [[ "$(readlink "$fd" 2>/dev/null)" == "socket:[$ino]" ]] && return 0
    done
  done < <(awk -v p=":$hex" 'substr($2, length($2) - 4) == p && $4 == "0A" { print $10 }' /proc/net/tcp /proc/net/tcp6 2>/dev/null)
  return 1
}
port_listening() {  # port_listening <port> — 누구든 LISTEN 중인가
  awk -v p=":$(printf '%04X' "$1")" 'substr($2, length($2) - 4) == p && $4 == "0A" { f = 1 } END { exit !f }' /proc/net/tcp /proc/net/tcp6 2>/dev/null
}

# ── 창 안에서 도는 쪽(내부 — 사람이 부르지 않는다) ─────────────────────────────────────────────────────
if [[ "${1:-}" == "__ctrl" ]]; then
  # __ctrl <신호 파일> <net> <묶음> <슬롯> <state csv> <part 0|1> <g1_ctrl> [KEY=VALUE ...]
  GO="$2" NET="$3" BUNDLE="$4" SLOT="$5" CSV="$6" PART="$7" G1BIN="$8"
  shift 8
  wait_flag "$GO" || { echo "[run] 🔴 기동 신호가 10 s 안에 안 왔다 — g1_ctrl 을 띄우지 않는다"; exit 1; }
  # 허용 목록 = 아래 export 넷(KEEP) + launcher 가 KEY=VALUE 로 넘긴 PASS_VARS. 넘긴 KEY 가 PASS_VARS 밖이면 띄우지 않는다.
  KEEP=(G1_ARCADE G1_ARCADE_BUNDLE G1_POLICY_SLOT G1_STATE_CSV)
  PASSED=()
  for kv in "$@"; do
    k="${kv%%=*}"
    if [[ "$kv" != *=* || " ${PASS_VARS[*]} " != *" $k "* ]]; then
      echo "[run] 🔴 창 ctrl 로 넘길 수 없는 값 '$kv' — 넘길 수 있는 것: ${PASS_VARS[*]}. g1_ctrl 을 띄우지 않는다"; exit 1
    fi
    PASSED+=("$k")
  done
  GONE=()
  while IFS= read -r v; do   # 창 환경(= tmux 서버 환경)의 G1_* 를 모두 지우고, 다시 넣지 않을 이름만 모은다
    unset "$v"
    [[ " ${KEEP[*]} ${PASSED[*]} " == *" $v "* ]] || GONE+=("$v")
  done < <(compgen -e | grep -E '^G1_' | sort)
  for kv in "$@"; do export "${kv?}"; done   # KEY=VALUE 를 그대로 export
  export G1_ARCADE=1 G1_ARCADE_BUNDLE="$BUNDLE" G1_POLICY_SLOT="$SLOT" G1_STATE_CSV="$CSV"
  echo "[run] 창 ctrl 환경 G1_* — 허용 목록(${KEEP[*]}${PASSED[*]:+ ${PASSED[*]}}) 밖 지움: ${GONE[*]:-없음}"
  echo "[run] g1_ctrl --network=$NET · G1_ARCADE=1 · G1_ARCADE_BUNDLE=$BUNDLE · G1_POLICY_SLOT=$SLOT · G1_STATE_CSV=$CSV${*:+ · $*}"
  # Ctrl-C · 창 닫힘(HUP)은 g1_ctrl 이 받아 끝나고, 이 셸은 남아 tty 복구 · .part 이름 바꾸기를 한다.
  trap ':' INT HUP
  STTY_SAVE="$(stty -g 2>/dev/null || true)"
  "$G1BIN" --network="$NET"
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
  # __relay <신호 파일> <pid 파일> <python> <relay 인자 ...> — exec 라 이 셸의 pid($$) = 중계 python 의 pid
  GO="$2" PIDF="$3"; shift 3
  wait_flag "$GO" || { echo "[run] 🔴 기동 신호가 10 s 안에 안 왔다 — 중계를 띄우지 않는다"; exit 1; }
  echo "$$" > "$PIDF"
  exec "$@"
fi
if [[ "${1:-}" == "__judge" ]]; then
  # __judge <net> <창 ctrl 로그> <슬롯> <묶음 폴더> <manifest md5> — 저장된 로그를 다시 판정만 한다(시험용, 아무것도 띄우지 않음)
  [[ $# -eq 6 ]] || { echo "쓰는 법: $0 __judge <net> <로그> <슬롯> <묶음> <manifest md5>" >&2; exit 2; }
  check_build
  judge_startup "$2" "$(clean_log "$3")" "$4" "$(cd "$5" && pwd)" "$6"
  echo "[judge] net=$2 · 🔴 $BAD 개"
  [[ $BAD -eq 0 ]]; exit $?
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
[[ $RELAY -eq 1 || -z "$RELAY_BIND" ]] || die "--relay-bind 는 --relay 와 같이"

# 빌드: config 는 /proc/self/exe 의 부모(build)의 ../config 에서 읽는다(param.h) — 이 자리의 바이너리만 쓴다
[[ -x "$G1CTRL" ]] || die "g1_ctrl 빌드가 없다: $G1CTRL (cmake .. -DCMAKE_BUILD_TYPE=Release -DG1_ARCADE_DIR=<g1_arcade> && make)"
[[ -f "$CACHE" ]] || die "$CACHE 가 없다 — 빌드 종류 · 아케이드 여부를 확인할 수 없다"
check_build
[[ -n "$ARC_DIR" ]] || die "이 g1_ctrl 은 아케이드 없이 빌드됐다(CMakeCache 에 G1_ARCADE_DIR 없음) — g1_ctrl 이 G1_ARCADE=1 기동을 거부한다"
[[ -z "$BUILD_STALE" ]] || die "g1_ctrl 빌드가 낡았다 — $BUILD_STALE. (cd $G1DIR/build && make -j6) 뒤 다시"

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
  port_listening "$RELAY_PORT" && die "포트 $RELAY_PORT 를 누가 쓰고 있다(ss -ltnp 로 확인) — 중계 v2 가 이미 떠 있으면 그것을 먼저 끝낼 것"
  RELAY_ARGS=(--bind "$RELAY_BIND" --port "$RELAY_PORT")
  [[ -n "${G1_ARCADE_STATUS_SHM:-}" ]] && RELAY_ARGS+=(--status-shm "$G1_ARCADE_STATUS_SHM")
  [[ -n "${G1_ARCADE_NAMES:-}" ]] && RELAY_ARGS+=(--names "$G1_ARCADE_NAMES")
fi

# 로그 폴더(마커는 관문을 다 통과한 뒤 맨 끝에 쓴다)
LOGDIR="${G1_LOG_DIR:-$HOME/dyros_ws/piene_ws/g1_logs}"
mkdir -p "$LOGDIR" || die "로그 폴더 $LOGDIR 를 못 만든다"
LOGDIR="$(cd "$LOGDIR" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"; STARTED="$(date -Is)"   # 마커 이름 · started= = g1_ctrl 을 띄운 시각(마커를 쓰는 시각이 아니다)
CTL_LOG="$LOGDIR/arcade_ctl_$STAMP.log"
RELAY_LOG="$LOGDIR/arcade_relay_$STAMP.log"
MARK="$LOGDIR/trial_$STAMP.txt"
if [[ -n "$STATE_CSV" ]]; then
  mkdir -p "$(dirname "$STATE_CSV")" || die "G1_STATE_CSV 폴더를 못 만든다: $STATE_CSV"
  STATE_CSV="$(cd "$(dirname "$STATE_CSV")" && pwd)/$(basename "$STATE_CSV")"; PART=0
else
  STATE_CSV="$LOGDIR/arcade_state_$STAMP.csv.part"; PART=1
fi
PASS=()
for v in "${PASS_VARS[@]}"; do [[ -n "${!v:-}" ]] && PASS+=("$v=${!v}"); done
REPO="$(cd "$G1DIR/../../.." && pwd)"
echo "[run] 묶음 $BUNDLE · manifest md5 $MANIFEST_MD5 · g1_arcade $B_COMMIT · 파일 $B_NFILES · $B_KIND · $B_DIRTY · role [$B_ROLES]"
echo "[run] 슬롯 $SLOT(가중치 md5 = 꾸러미 onnx_md5) · g1_ctrl $G1CTRL · 빌드 $BUILD_TYPE · core $ARC_DIR · $BUILD_NOTE"
echo "[run] 창 ctrl 출력 $CTL_LOG"

# ── 창 ctrl ─────────────────────────────────────────────────────────────────────────────────────
RUNTMP="$(mktemp -d /tmp/g1_arcade_run.XXXXXX)" || die "임시 폴더를 못 만든다"
trap 'rm -rf "$RUNTMP"' EXIT
CMD="$(printf '%q ' bash "$SELF" __ctrl "$RUNTMP/go_ctrl" "$NET" "$BUNDLE" "$SLOT" "$STATE_CSV" "$PART" "$G1CTRL" "${PASS[@]}")"
tmux new-session -d -s "$SESSION" -n ctrl -x 220 -y 50 "$CMD" || die "tmux 세션 $SESSION 을 못 만들었다"
# 창이 바로 끝나도(기동 거부) 출력이 남게 — 기동 신호 전에 건다
tmux set-option -w -t "$SESSION:ctrl" remain-on-exit on >/dev/null || die "remain-on-exit 를 못 걸었다(세션이 이미 끝났나?)"
tmux pipe-pane -t "$SESSION:ctrl" -o "cat >> $(printf '%q' "$CTL_LOG")" || die "pipe-pane 을 못 걸었다"
touch "$RUNTMP/go_ctrl"

pane_alive() { [[ "$(tmux list-panes -t "$SESSION:$1" -F '#{pane_dead}' 2>/dev/null)" == 0 ]]; }
ALIVE=1
for ((i = 0; i < 600; i++)); do   # 60 s — ORT 세션 · 예열(로봇은 com1 보다 느리다). 끝 표지 = FSM 루프의 [rt] FSM 1kHz 줄(아케이드 줄들 뒤)
  L="$(clean_log "$CTL_LOG")"
  grep -qF '[rt] FSM 1kHz' <<< "$L" && break
  grep -qF '기동 거부' <<< "$L" && break
  if ! pane_alive ctrl; then ALIVE=0; break; fi
  sleep 0.1
done
sleep 0.3
L="$(clean_log "$CTL_LOG")"
pane_alive ctrl || ALIVE=0

if [[ $ALIVE -ne 1 ]] || grep -qF '기동 거부' <<< "$L"; then
  judge_startup "$NET" "$L" "$SLOT" "$BUNDLE" "$MANIFEST_MD5"
  echo "[run] 🔴 g1_ctrl 이 기동을 거부했거나 끝났다(마커 안 씀):"
  grep -E '기동 거부|\[critical\]|\[error\]' <<< "$L" | msg_of | head -8 | sed 's/^/    /'
  echo "[run]    창 출력: tmux attach -t $SESSION (끝난 창은 남아 있다) · 정리: tmux kill-session -t $SESSION"
  exit 1
fi
judge_startup "$NET" "$L" "$SLOT" "$BUNDLE" "$MANIFEST_MD5"
if [[ $BAD -ne 0 ]]; then
  echo "[run] 🔴 기동 로그 판정 🔴 $BAD 개 — $([[ "$NET" != lo ]] && echo "실기 운용 금지, g1_ctrl 을 끌 것. ")중계 안 띄움 · 마커 안 씀."
  echo "[run]    g1_ctrl 은 떠 있다(Passive). 끝내기: tmux attach -t $SESSION → Ctrl-C · 정리: tmux kill-session -t $SESSION"
  exit 1
fi

# ── 창 relay ────────────────────────────────────────────────────────────────────────────────────
if [[ $RELAY -eq 1 ]]; then
  RCMD="$(printf '%q ' bash "$SELF" __relay "$RUNTMP/go_relay" "$RUNTMP/relay.pid" "$RELAY_PY" "$HERE/arcade_relay2.py" "${RELAY_ARGS[@]}")"
  tmux new-window -d -t "$SESSION:" -n relay "$RCMD" || die "창 relay 를 못 만들었다"
  tmux set-option -w -t "$SESSION:relay" remain-on-exit on >/dev/null || die "창 relay 에 remain-on-exit 를 못 걸었다"
  tmux pipe-pane -t "$SESSION:relay" -o "cat >> $(printf '%q' "$RELAY_LOG")" || die "창 relay 에 pipe-pane 을 못 걸었다"
  touch "$RUNTMP/go_relay"
  UP=0; RPID=""
  for ((i = 0; i < 100; i++)); do   # 10 s — 이 창에서 띄운 pid 가 :8711 LISTEN 을 쥘 때까지
    [[ -z "$RPID" && -s "$RUNTMP/relay.pid" ]] && RPID="$(cat "$RUNTMP/relay.pid")"
    if [[ -n "$RPID" && -d "/proc/$RPID" ]] && tr '\0' ' ' < "/proc/$RPID/cmdline" 2>/dev/null | grep -qF arcade_relay2.py \
       && listener_owned_by "$RPID" "$RELAY_PORT"; then UP=1; break; fi
    pane_alive relay || break
    sleep 0.1
  done
  if [[ $UP -ne 1 ]]; then
    echo "[run] 🔴 이 창의 중계 v2(pid ${RPID:-모름})가 :$RELAY_PORT LISTEN 을 쥐지 못했다(마커 안 씀) — 창 relay 출력:"
    clean_log "$RELAY_LOG" | tail -8 | sed 's/^/    /'
    echo "[run]    g1_ctrl 은 떠 있다(Passive). 정리: tmux kill-session -t $SESSION"
    exit 1
  fi
  echo "[run] 중계 v2 pid $RPID 가 ws://$RELAY_BIND:$RELAY_PORT LISTEN 을 쥠(창 relay · 출력 $RELAY_LOG)"
fi

# ── 마커(관문을 다 통과한 뒤에만 — run_g1_with_gui.sh 형식 + 아케이드 칸) ─────────────────────────────────────
{
  echo "slot=$SLOT"
  echo "net=$NET"
  echo "started=$STARTED"
  echo "host=$(hostname)"
  echo "git=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "arcade=1"
  echo "arcade_bundle=$BUNDLE"
  echo "arcade_manifest_md5=$MANIFEST_MD5"
  echo "arcade_g1_arcade=$ARC_DIR"
  echo "state_csv=${STATE_CSV%.part}"
  echo "relay=$([[ $RELAY -eq 1 ]] && echo "ws://$RELAY_BIND:$RELAY_PORT" || echo off)"
  if [[ ${#PASS[@]} -gt 0 ]]; then echo "env=${PASS[*]}"; fi
} > "$MARK.tmp" && mv -f "$MARK.tmp" "$MARK" || echo "[run] ⚠ 마커 $MARK 를 못 썼다 — 대시보드가 이 실험의 슬롯을 모른다"
echo "[run] 🟢 떴다(마커 $MARK). 다음: tmux attach -t $SESSION → 창 ctrl 에서 f → m (사람이 친다)$([[ $RELAY -eq 1 ]] && echo " · 브라우저 ?mode=real2&relay=ws://$RELAY_BIND:$RELAY_PORT")"
echo "[run]    m 뒤 창 ctrl 의 [rt] 정책 50Hz -> SCHED_FIFO 줄도 볼 것(FSM 과 같은 권한 — 실기에서 «실패» 면 p)"
echo "[run]    멈추기: 창 ctrl 에서 p → Ctrl-C · 정리: tmux kill-session -t $SESSION"
exit 0
