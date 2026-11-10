#!/usr/bin/env bash
# run_g1_arcade.sh 창 ctrl 환경 — 허용 목록 밖 G1_* 는 g1_ctrl 에 닿지 않는다(최종 검토 · 실험용 덮어쓰기 누설).
# 창 ctrl 이 도는 그 코드(__ctrl)를 tmux 없이 직접 부르고, g1_ctrl 자리에 «받은 환경을 파일로 쓰는» 가짜를 넣는다.
# 끝 코드 0 = 통과. 아무것도 띄우지 않고 로봇 · 시뮬에 닿지 않는다.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
RUN="$PWD/tools/run_g1_arcade.sh"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
fail=0
no() { echo "FAIL: $*"; fail=1; }

cat > "$T/fake_g1_ctrl" <<'EOF'
#!/usr/bin/env bash
env | grep -E '^G1_' | sort > "$FAKE_ENV_OUT"
echo "args $*" >> "$FAKE_ENV_OUT"
EOF
chmod +x "$T/fake_g1_ctrl"

ctrl() {  # ctrl <출력 이름> [KEY=VALUE ...] — 더럽힌 환경으로 __ctrl 한 번(기동 신호는 미리 만든다)
  local name="$1"; shift
  touch "$T/go_$name"
  FAKE_ENV_OUT="$T/env_$name" \
  G1_IMU_CAL_DEG=3 G1_FOOTZ_SRC=fk G1_OBS_CSV=/tmp/x.csv G1_NO_RT=1 G1_DIAG_LOAD=5 G1_ARCADE_FAULT=stall \
  G1_ARCADE_BUNDLE=/stale/bundle G1_POLICY_SLOT=stale_slot \
    bash "$RUN" __ctrl "$T/go_$name" lo /b/bundle walk_slot "$T/state.csv" 0 "$T/fake_g1_ctrl" "$@" > "$T/out_$name" 2>&1
}

# ① 넘긴 것(G1_ARCADE_STATUS_SHM)만 · 스크립트 넷만 남는다
ctrl a G1_ARCADE_STATUS_SHM=/g1_arcade_status_t || no "① 끝 코드 $?: $(cat "$T/out_a")"
want="G1_ARCADE=1
G1_ARCADE_BUNDLE=/b/bundle
G1_ARCADE_STATUS_SHM=/g1_arcade_status_t
G1_POLICY_SLOT=walk_slot
G1_STATE_CSV=$T/state.csv
args --network=lo"
[[ "$(cat "$T/env_a" 2>/dev/null)" == "$want" ]] || no "① g1_ctrl 이 받은 G1_* 가 다르다:
$(cat "$T/env_a" 2>/dev/null)"
line="$(grep -F '[run] 창 ctrl 환경 G1_* — ' "$T/out_a")"
for v in G1_ARCADE_FAULT G1_DIAG_LOAD G1_FOOTZ_SRC G1_IMU_CAL_DEG G1_NO_RT G1_OBS_CSV; do
  [[ "$line" == *" $v"* ]] || no "① 지운 이름 줄에 $v 가 없다: $line"
done
for v in G1_ARCADE_BUNDLE G1_POLICY_SLOT G1_ARCADE_STATUS_SHM; do   # 다시 넣은 것은 «지움» 목록에 없다
  [[ "${line#*지움:}" != *"$v"* ]] || no "① 다시 넣은 $v 가 지운 목록에 있다: $line"
done

# ② 시험 주입(G1_ARCADE_FAULT)은 launcher 가 넘길 때만 남는다
ctrl b G1_ARCADE_FAULT=stall_ms:200 || no "② 끝 코드 $?"
grep -qx 'G1_ARCADE_FAULT=stall_ms:200' "$T/env_b" || no "② 넘긴 G1_ARCADE_FAULT 가 없다"
! grep -q 'G1_IMU_CAL_DEG' "$T/env_b" || no "② G1_IMU_CAL_DEG 가 남았다"

# ③ PASS_VARS 밖의 KEY=VALUE 는 거부 — g1_ctrl 을 부르지 않는다
if ctrl c G1_IMU_CAL_DEG=3; then no "③ 허용 목록 밖 KEY 를 받아들였다"; fi
[[ ! -e "$T/env_c" ]] || no "③ 거부했는데 g1_ctrl 이 불렸다"
grep -qF '넘길 수 없는 값' "$T/out_c" || no "③ 거부 문구 없음: $(cat "$T/out_c")"

# ④ 더럽힌 것이 없으면 «없음»
touch "$T/go_d"
env -i PATH="$PATH" HOME="$HOME" FAKE_ENV_OUT="$T/env_d" \
  bash "$RUN" __ctrl "$T/go_d" lo /b/bundle walk_slot "$T/state.csv" 0 "$T/fake_g1_ctrl" > "$T/out_d" 2>&1 || no "④ 끝 코드 $?"
grep -qF '밖 지움: 없음' "$T/out_d" || no "④ «없음» 줄이 없다: $(cat "$T/out_d")"

# ⑤ 기동 판정(저장된 로그를 다시 판정하는 __judge)이 그 줄을 기동 출력에 다시 찍고, 줄이 없으면 🔴 로 센다
#    (다른 판정 줄 ①~⑤ 는 이 가짜 로그에 없어 🔴 가 여럿 — 여기서는 환경 줄만 본다)
cp "$T/out_a" "$T/log_with"
grep -vF '[run] 창 ctrl 환경 G1_*' "$T/out_a" > "$T/log_without"
bash "$RUN" __judge lo "$T/log_with" walk_slot "$T" 0 > "$T/judge_with" 2>&1
bash "$RUN" __judge lo "$T/log_without" walk_slot "$T" 0 > "$T/judge_without" 2>&1
grep -qF '· 창 ctrl 환경 G1_* — 허용 목록' "$T/judge_with" || no "⑤ 판정 출력에 환경 줄이 없다: $(head -5 "$T/judge_with")"
! grep -qF '🔴 창 ctrl 환경 정리 줄' "$T/judge_with" || no "⑤ 줄이 있는데 🔴"
grep -qF '🔴 창 ctrl 환경 정리 줄' "$T/judge_without" || no "⑤ 줄이 없는데 🔴 가 아니다"
n_with="$(sed -n 's/.*🔴 \([0-9]*\) 개.*/\1/p' "$T/judge_with")"; n_without="$(sed -n 's/.*🔴 \([0-9]*\) 개.*/\1/p' "$T/judge_without")"
[[ -n "$n_with" && "$n_without" == "$((n_with + 1))" ]] || no "⑤ 환경 줄 없음이 🔴 하나로 세지지 않았다($n_with → $n_without)"

[[ $fail -eq 0 ]] && echo "ok: 허용 목록 밖 G1_* 6 개 지움 · 넘긴 것만 남음 · PASS_VARS 밖 거부 · 깨끗한 환경 «없음»"
exit $fail
