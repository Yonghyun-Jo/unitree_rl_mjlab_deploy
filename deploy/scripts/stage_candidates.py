#!/usr/bin/env python3
"""stage_candidates.py — «이거 실기에서 하겠다» 를 한 명령으로 v1/v2/… 슬롯까지.

# 무엇을 푸나
실기 시험 전날/아침에 후보가 여럿 생긴다 («8 cm 판», «6 cm 판», «발배치 강화 판»…).
그때마다 사람이 export → 슬롯 폴더 → deploy.yaml 한 줄 → META → ACTIVE.yaml → 옛 슬롯 치우기 →
커밋 → 로봇 전송을 손으로 했고, 그 사이 한 단계가 빠지면 «로봇 앞에서» 드러났다
(min_swing 누락 · requires 누락 · 별칭이 옛 슬롯을 가리킴 · 가중치 없는 슬롯).

이 스크립트는 그 전부를 **후보 파일 하나**로 한다. 같은 파일로 다시 돌리면 이미 있는 슬롯은
건너뛰고(멱등) 별칭만 다시 맞춘다 — 학습이 늦게 끝난 후보를 v3 로 «추가» 하는 것도 같은 명령.

# 후보 파일 (YAML)
    day: 260829                      # 별칭 묶음 이름. 슬롯 이름의 접두 = <day>_<alias>_<label>
    base:      logs/rsl_rl/g1_student/<base_run>/model_<it>.pt        # flow base (mjlab 상대경로)
    manifest:  motions/g1_flow_specialists_v2.yaml
    mode2_ckpt: logs/rsl_rl/g1_student/<run>/model_<it>.pt
    mode3_ckpt: logs/rsl_rl/g1_student/<run>/model_<it>.pt
    sampling_steps: 6
    template_slot: 260828_v1b_minswing6   # deploy.yaml · npz 를 여기서 가져온다 (requires 포함)
    candidates:
      - alias: v1
        label: ms8_noslip1_30k
        mode1_ckpt: logs/rsl_rl/g1_student/<run>/model_29999.pt
        min_swing: 0.08                 # 배포 mode1.min_swing. 0 이면 키를 뺀다
        note: "무슨 실험인지 한 문장"
      - alias: v2
        ...

# 무엇을 하나 (순서)
  ① 후보마다 mjlab 에서 ONNX export (이미 있으면 건너뜀) — parity 실패면 그 후보는 중단
  ② 슬롯 생성: exported/policy.onnx · params/deploy.yaml(템플릿 + mode1 min_swing) · params/*.npz
     (그날 첫 슬롯만 실사본, 나머지는 상대 심링크 — 저장 규칙) · ONNX_META.json
  ③ 학습측 FOOT_GEN 과 gait 패리티 대조 (체크포인트의 launch 커밋에서 읽음) — 다르면 경고 + META 기록
  ④ ACTIVE.yaml = 이 파일의 별칭 전부 (day 교체)
  ⑤ 직전 ACTIVE 에 있었지만 이번에 없는 슬롯은 **보관소로** (policy_slot archive) — «옛 정책 치우기»
  ⑥ check · index · git add(정체만) · commit
  ⑦ --robot: 가중치 rsync(policy_slot push) + robot.sh deploy. 로봇이 안 닿으면 명령만 찍는다

# 지키는 것
  - 커밋에는 정체(deploy.yaml · ONNX_META.json · ACTIVE.yaml · POLICY_INDEX.md)만 (.gitignore 가 무게를 막는다)
  - 학습은 절대 돌리지 않는다. export 만.
  - 로봇을 «움직이는» 명령은 없다. 전송·빌드까지.

# 사용
    python3 deploy/scripts/stage_candidates.py candidates/260829.yaml            # 슬롯+별칭+커밋
    python3 deploy/scripts/stage_candidates.py candidates/260829.yaml --robot    # + 로봇 전송·빌드
    python3 deploy/scripts/stage_candidates.py candidates/260829.yaml --dry-run
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
POLICY = os.path.join(REPO, "deploy/robots/g1/config/policy")
SLOTDIR = os.path.join(POLICY, "mimic_masked")
ACTIVE = os.path.join(POLICY, "ACTIVE.yaml")
INDEX = os.path.join(POLICY, "POLICY_INDEX.md")
SLOT_PY = os.path.join(HERE, "policy_slot.py")
MJLAB = os.environ.get("MJLAB_WS", os.path.expanduser("~/mjlab1.4/mjlab_g1_motion"))
UV = os.environ.get("UV_BIN", os.path.expanduser("~/.local/bin/uv"))
ROBOT_SH = os.path.expanduser("~/piene_automation/robot_bridge/robot.sh")
CFG_REL = "src/mjlab_g1_motion/tasks/stage4_mode1_env_cfg.py"

try:
    import yaml
except ImportError:                                  # 배포 venv 에 pyyaml 이 없을 수 있다
    yaml = None


# ── 작은 도구 ────────────────────────────────────────────────────────────
def log(msg):
    print(msg, flush=True)


def die(msg, rc=1):
    log("🔴 " + msg)
    sys.exit(rc)


def sh(cmd, cwd=None, check=True, capture=False):
    r = subprocess.run(cmd, cwd=cwd, text=True,
                       stdout=subprocess.PIPE if capture else None,
                       stderr=subprocess.STDOUT if capture else None)
    if check and r.returncode != 0:
        if capture:
            sys.stdout.write(r.stdout or "")
        raise RuntimeError("명령 실패 (%d): %s" % (r.returncode, " ".join(cmd)))
    return r


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_yaml(path):
    if yaml is None:
        die("pyyaml 이 없다 — miniconda python 으로 실행하거나 pip install pyyaml")
    return yaml.safe_load(open(path, encoding="utf-8"))


def read_active():
    out = {}
    if not os.path.exists(ACTIVE):
        return out
    for line in open(ACTIVE, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = [x.strip() for x in line.split(":", 1)]
        if k and v and k != "day":
            out[k] = v
    return out


def run_dir_of(ckpt_rel):
    return os.path.dirname(os.path.join(MJLAB, ckpt_rel))


def training_commit(ckpt_rel):
    p = os.path.join(run_dir_of(ckpt_rel), "training_meta.json")
    if os.path.exists(p):
        try:
            return json.load(open(p, encoding="utf-8")).get("commit") or ""
        except Exception:
            pass
    return ""


def foot_gen_at(commit):
    """학습 launch 커밋의 stage4_mode1_env_cfg.FOOT_GEN 에서 gait 패리티에 필요한 값만 읽는다."""
    if not commit:
        return {}
    r = subprocess.run(["git", "show", "%s:%s" % (commit, CFG_REL)], cwd=MJLAB,
                       text=True, capture_output=True)
    if r.returncode != 0:
        return {}
    m = re.search(r"FOOT_GEN\s*=\s*dict\((.*?)\)\n", r.stdout, re.S)
    if not m:
        return {}
    body = m.group(1)
    out = {}
    for key in ("cadence_scale", "settle_eff", "settle_phase", "turn_k", "stance_z"):
        mm = re.search(r"\b%s\s*=\s*([0-9.]+)" % key, body)
        if mm:
            out[key] = float(mm.group(1))
    for key in ("settle_steps", "min_swing"):
        mm = re.search(r"\b%s\s*=\s*\{([^}]*)\}" % key, body)
        if mm:
            d = {}
            for kv in mm.group(1).split(","):
                if ":" in kv:
                    k, v = kv.split(":", 1)
                    d[int(k)] = float(v)
            out[key] = d
    return out


# ── ① export ─────────────────────────────────────────────────────────────
def export_onnx(cfg, cand, slot, dry):
    out_rel = os.path.join("exported", slot, "policy.onnx")
    out_abs = os.path.join(MJLAB, out_rel)
    cmd = [UV, "run", "--no-sync", "python", "-m",
           "mjlab_g1_motion.scripts.tools.export_multihead_onnx",
           "--flow-base-checkpoint", cfg["base"], "--manifest", cfg["manifest"],
           "--mode1-ckpt", cand["mode1_ckpt"], "--mode2-ckpt", cfg["mode2_ckpt"],
           "--mode3-ckpt", cfg["mode3_ckpt"], "--sampling-steps", str(cfg.get("sampling_steps", 6)),
           "--out", out_rel]
    export_cmd = "cd %s && %s" % (MJLAB, " ".join(cmd))
    if os.path.exists(out_abs):
        log("   export 건너뜀 — 이미 있다: %s" % out_rel)
        return out_abs, export_cmd, {}
    log("   export → %s" % out_rel)
    if dry:
        log("   (dry-run) " + export_cmd)
        return out_abs, export_cmd, {}
    r = sh(cmd, cwd=MJLAB, check=False, capture=True)
    txt = r.stdout or ""
    tail = "\n".join(txt.strip().splitlines()[-25:])
    if r.returncode != 0 or not os.path.exists(out_abs):
        log(tail)
        raise RuntimeError("export 실패: %s" % slot)
    v = {}
    m = re.search(r"onnxruntime vs torch\s+max\|Δ\|\s*=\s*([0-9.e+-]+)\s*\((\w+), tol=([0-9.e+-]+)\)", txt)
    if m:
        v["onnx_vs_torch"] = "max|Δ| = %s (tol %s) %s" % (m.group(1), m.group(3), m.group(2))
        if m.group(2) != "OK":
            log(tail)
            raise RuntimeError("parity 실패: %s" % slot)
    m = re.search(r"\[export\] (그래프 검증 OK — .*)", txt)
    if m:
        v["graph"] = m.group(1)
    m = re.search(r"\[export\] (obs 계약 .*OK)", txt)
    if m:
        v["obs_contract_sum"] = m.group(1)
    m = re.search(r"\[export\] (mask slice .*OK)", txt)
    if m:
        v["mask_slice"] = m.group(1)
    log("   " + (v.get("onnx_vs_torch") or "parity 줄을 못 찾음"))
    return out_abs, export_cmd, v


def onnx_metadata(path):
    code = ("import onnx,json,sys;m=onnx.load(sys.argv[1],load_external_data=False);"
            "print(json.dumps({p.key:p.value for p in m.metadata_props}))")
    r = sh([UV, "run", "--no-sync", "python", "-c", code, path], cwd=MJLAB, check=False, capture=True)
    try:
        return json.loads((r.stdout or "").strip().splitlines()[-1])
    except Exception:
        return {}


# ── ② 슬롯 생성 ──────────────────────────────────────────────────────────
_MS_BLOCK = re.compile(r"^  # ── 최소 스윙 클리어런스.*?^  mode1: \{[^\n]*\n", re.S | re.M)
_MODE1_LINE = re.compile(r"^  mode1: \{([^}]*)\}[^\n]*\n", re.M)


def render_deploy_yaml(tmpl_text, cand, slot, cfg, fg_train):
    ck = cand["mode1_ckpt"]
    run = os.path.basename(os.path.dirname(ck))
    it = re.sub(r"\D", "", os.path.basename(ck)) or "?"
    commit = training_commit(ck)
    ms = float(cand.get("min_swing", 0.0) or 0.0)
    ms_train = (fg_train.get("min_swing") or {}).get(1, 0.0)

    # 헤더: «이 슬롯의 mode1 은 무엇인가» 한 줄
    lines = tmpl_text.split("\n")
    for i, l in enumerate(lines[:12]):
        if l.startswith("#   mode1 "):
            lines[i] = ("#   mode1 %s / model_%s  <- 이 슬롯에서 바뀐 것은 이것 하나 (launch commit %s)"
                        % (run, it, commit[:7] or "?"))
            if i + 1 < len(lines) and re.match(r"^#\s{9}\(", lines[i + 1]):
                del lines[i + 1]
            break
    text = "\n".join(lines)
    text = re.sub(r"^(\s*#\s*mode1 head = ).*$",
                  lambda m: m.group(1) + "%s (launch commit %s)" % (run, commit[:7] or "?"),
                  text, count=1, flags=re.M)
    # gait 블록 머리의 계보 주석 («mode1 = <run> @<it>» + «->» 이어지는 줄들) 도 이 슬롯 기준으로
    fg_s = ", ".join("%s %s" % (k, (v if not isinstance(v, dict) else "{%s}" % ", ".join(
        "%d: %g" % (a, b) for a, b in v.items()))) for k, v in fg_train.items()) or "(launch 커밋을 못 읽음)"
    text = re.sub(r"^#   mode1 = .*\n(?:#\s{10}->.*\n|#\s{13}.*\n)*",
                  "#   mode1 = %s @%s (launch commit %s)\n#          -> 그 커밋의 stage4_mode1_env_cfg.FOOT_GEN: %s\n"
                  % (run, it, commit[:7] or "?", fg_s),
                  text, count=1, flags=re.M)

    # mode1 줄 재구성 (템플릿의 나머지 키는 그대로, min_swing 만 후보값)
    m = _MODE1_LINE.search(text)
    if not m:
        raise RuntimeError("템플릿 deploy.yaml 에 `  mode1: {...}` 줄이 없다")
    kv = [x.strip() for x in m.group(1).split(",") if x.strip() and not x.strip().startswith("min_swing")]
    if ms > 0:
        kv.append("min_swing: %g" % ms)
    mode1_line = "  mode1: {%s}\n" % ", ".join(kv)

    if ms > 0:
        parity = ("학습 FOOT_GEN.min_swing[1] = %g -> 동일 (train ≡ deploy)" % ms_train
                  if abs(ms_train - ms) < 1e-9 else
                  "⚠ 학습 FOOT_GEN.min_swing[1] = %g 인데 배포는 %g — 의도한 A/B 가 아니면 사고" % (ms_train, ms))
    else:
        parity = ("학습 FOOT_GEN.min_swing[1] = %g 인데 배포는 없음 — 확인할 것" % ms_train
                  if ms_train > 0 else "학습·배포 모두 min_swing 없음")
    block = (
        "  # ── 최소 스윙 클리어런스 (stage_candidates.py 가 후보 파일로 생성, %s) ──────────\n"
        "  # 슬롯 %s · mode1 = %s/model_%s\n"
        "  # 배포 min_swing %s · %s\n"
        "  # 표 진폭이 min_swing 보다 작으면 «궤적의 골을 고정한 채» 모양 그대로 키운다. 정착 걸음도\n"
        "  # 같은 LUT 조회 경로라 같이 커진다(골 불변). 학습과 같은 값이면 OOD 가 아니다.\n"
        % (_dt.date.today().isoformat(), slot, run, it, ("%g" % ms if ms > 0 else "없음"), parity)
    ) + mode1_line
    text, n = _MS_BLOCK.subn(lambda _m: block, text, count=1)
    if n == 0:                                        # 템플릿에 블록이 없으면 mode1 줄만 교체
        text = _MODE1_LINE.sub(lambda _m: block, text, count=1)
    return text, parity


def npz_list(params_dir):
    return sorted(f for f in os.listdir(params_dir) if f.endswith(".npz"))


def place_weights(slot_root, onnx_src, tmpl_params, first_slot, slot, dry):
    """exported/policy.onnx + params/*.npz. 그날 첫 슬롯만 실사본, 나머지는 상대 심링크."""
    exp = os.path.join(slot_root, "exported")
    par = os.path.join(slot_root, "params")
    os.makedirs(exp, exist_ok=True)
    os.makedirs(par, exist_ok=True)
    dst = os.path.join(exp, "policy.onnx")
    if not os.path.lexists(dst):
        if not dry:
            shutil.copy2(onnx_src, dst)
        log("   exported/policy.onnx  ← %s (%.1f MB)" % (os.path.relpath(onnx_src, MJLAB),
                                                          os.path.getsize(onnx_src) / 1048576 if not dry else 0))
    for f in npz_list(tmpl_params):
        d = os.path.join(par, f)
        if os.path.lexists(d):
            continue
        if first_slot is None or first_slot == slot:
            src = os.path.realpath(os.path.join(tmpl_params, f))     # 템플릿이 심링크여도 실체를 복사
            if not dry:
                shutil.copy2(src, d)
            log("   params/%s  ← 실사본" % f)
        else:
            rel = os.path.join("..", "..", first_slot, "params", f)
            if not dry:
                os.symlink(rel, d)
            log("   params/%s  → %s (심링크)" % (f, rel))


def write_meta(slot_root, slot, cand, cfg, day, export_cmd, verify, onnx_meta, fg_train, parity_note,
               requires, onnx_path, dry):
    ck = cand["mode1_ckpt"]
    run = os.path.basename(os.path.dirname(ck))
    commit = training_commit(ck)
    meta = {
        "exported_at": _dt.date.today().isoformat(),
        "slot": slot,
        "trial": cand["alias"],
        "trial_day": "20%s-%s-%s" % (day[:2], day[2:4], day[4:6]),
        "note": cand.get("note", ""),
        "staged_by": "deploy/scripts/stage_candidates.py",
        "requires": requires,
        "sampling_steps": onnx_meta.get("sampling_steps", str(cfg.get("sampling_steps", 6))),
        "actor_hidden_dims": onnx_meta.get("actor_hidden_dims", ""),
        "obs_dim": onnx_meta.get("obs_dim", ""),
        "obs_contract_version": onnx_meta.get("obs_contract_version", ""),
        "obs_contract": onnx_meta.get("obs_contract", ""),
        "flow_base_checkpoint": cfg["base"],
        "mode1_ckpt": ck,
        "mode1_run": run,
        "mode1_launch_commit": commit,
        "mode2_ckpt": cfg["mode2_ckpt"],
        "mode3_ckpt": cfg["mode3_ckpt"],
        "manifest": cfg["manifest"],
        "deploy_min_swing": float(cand.get("min_swing", 0.0) or 0.0),
        "gait_parity": {
            "근거": "체크포인트 run 의 training_meta.json commit 에서 stage4_mode1_env_cfg.FOOT_GEN 을 읽어 대조",
            "train_FOOT_GEN": {k: (v if not isinstance(v, dict) else {str(a): b for a, b in v.items()})
                               for k, v in fg_train.items()},
            "min_swing": parity_note,
        },
        "verify": dict(verify, onnx_md5=(md5(onnx_path) if (not dry and os.path.exists(onnx_path)) else ""),
                       sim2sim_walk="미실행 — 사용자가 확인", real_robot="미실행"),
        "export_cmd": export_cmd,
        "deployed": {"status": "pending — stage_candidates --robot 또는 policy_slot push"},
    }
    p = os.path.join(slot_root, "ONNX_META.json")
    if not dry:
        json.dump(meta, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    log("   ONNX_META.json 기록")
    return meta


def requires_of(deploy_yaml_text):
    m = re.search(r"^requires:\n((?:  - .*\n)+)", deploy_yaml_text, re.M)
    if not m:
        return []
    return [re.sub(r"\s*#.*", "", l).strip("- ").strip() for l in m.group(1).splitlines()]


# ── 메인 ─────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("candidates", help="후보 YAML")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-archive", action="store_true", help="직전 ACTIVE 슬롯을 보관소로 안 보낸다")
    ap.add_argument("--no-commit", action="store_true")
    ap.add_argument("--robot", action="store_true", help="가중치 rsync + robot.sh deploy 까지")
    ap.add_argument("--rerender-yaml", action="store_true",
                    help="이미 있는 슬롯의 params/deploy.yaml 을 템플릿에서 다시 그린다(생성물이라 안전)")
    a = ap.parse_args()
    dry = a.dry_run

    cfg = load_yaml(a.candidates)
    day = str(cfg.get("day") or _dt.date.today().strftime("%y%m%d"))
    cands = cfg.get("candidates") or []
    if not cands:
        die("candidates 가 비었다")
    for k in ("base", "manifest", "mode2_ckpt", "mode3_ckpt", "template_slot"):
        if not cfg.get(k):
            die("후보 파일에 %s 가 없다" % k)
    tmpl_root = os.path.join(SLOTDIR, cfg["template_slot"])
    tmpl_yaml = os.path.join(tmpl_root, "params", "deploy.yaml")
    if not os.path.exists(tmpl_yaml):
        die("템플릿 슬롯의 deploy.yaml 이 없다: %s" % tmpl_yaml)
    tmpl_text = open(tmpl_yaml, encoding="utf-8").read()
    if not npz_list(os.path.join(tmpl_root, "params")):
        die("템플릿 슬롯에 npz 가 없다 — policy_slot.py restore %s 먼저" % cfg["template_slot"])
    for c in cands:
        for k in ("alias", "label", "mode1_ckpt"):
            if not c.get(k):
                die("후보에 %s 가 없다: %s" % (k, c))
        if not os.path.exists(os.path.join(MJLAB, c["mode1_ckpt"])):
            die("체크포인트가 없다: %s" % c["mode1_ckpt"])

    r = subprocess.run(["git", "branch", "--show-current"], cwd=REPO, text=True, capture_output=True)
    log("배포 repo 브랜치: %s   day %s   후보 %d 개" % (r.stdout.strip(), day, len(cands)))
    prev_active = read_active()

    made = []
    first_slot = None
    for c in cands:
        slot = "%s_%s_%s" % (day, c["alias"], c["label"])
        root = os.path.join(SLOTDIR, slot)
        log("\n■ %s  (%s)" % (slot, c["mode1_ckpt"]))
        if os.path.exists(os.path.join(root, "ONNX_META.json")) and os.path.exists(os.path.join(root, "exported", "policy.onnx")):
            if a.rerender_yaml:
                text, parity_note = render_deploy_yaml(tmpl_text, c, slot, cfg,
                                                       foot_gen_at(training_commit(c["mode1_ckpt"])))
                if not dry:
                    open(os.path.join(root, "params", "deploy.yaml"), "w", encoding="utf-8").write(text)
                log("   이미 있다 — deploy.yaml 만 다시 그림  %s" % parity_note)
            else:
                log("   이미 있다 — 건너뜀 (별칭만 갱신)")
            made.append((c["alias"], slot))
            if first_slot is None:
                first_slot = slot
            continue
        fg_train = foot_gen_at(training_commit(c["mode1_ckpt"]))
        try:
            onnx_abs, export_cmd, verify = export_onnx(cfg, c, slot, dry)
        except RuntimeError as e:
            log("🔴 %s — 이 후보는 건너뛴다" % e)
            continue
        if first_slot is None:
            first_slot = slot
        if not dry:
            os.makedirs(root, exist_ok=True)
        place_weights(root, onnx_abs, os.path.join(tmpl_root, "params"), first_slot, slot, dry)
        text, parity_note = render_deploy_yaml(tmpl_text, c, slot, cfg, fg_train)
        if not dry:
            os.makedirs(os.path.join(root, "params"), exist_ok=True)
            open(os.path.join(root, "params", "deploy.yaml"), "w", encoding="utf-8").write(text)
        log("   params/deploy.yaml  (템플릿 %s, min_swing %s)  %s"
            % (cfg["template_slot"], c.get("min_swing", 0), parity_note))
        onnx_meta = onnx_metadata(onnx_abs) if (not dry and os.path.exists(onnx_abs)) else {}
        write_meta(root, slot, c, cfg, day, export_cmd, verify, onnx_meta, fg_train, parity_note,
                   requires_of(text), os.path.join(root, "exported", "policy.onnx"), dry)
        made.append((c["alias"], slot))

    if not made:
        die("만들어진 슬롯이 없다")

    # ④ 별칭
    log("\n■ ACTIVE.yaml  (day %s)" % day)
    assign = ["%s=%s" % (al, s) for al, s in made]
    if not dry:
        sh([sys.executable, SLOT_PY, "activate", "--day", day] + assign, cwd=REPO)
    else:
        log("   (dry-run) activate " + " ".join(assign))

    # ⑤ 옛 활성 슬롯 → 보관소
    new_set = {s for _, s in made}
    old = sorted(set(prev_active.values()) - new_set)
    if old and not a.no_archive:
        log("\n■ 옛 활성 슬롯 보관 (%d 개) — 가중치는 보관소로, 정체는 git 에 남는다" % len(old))
        for s in old:
            if s == cfg["template_slot"] and first_slot and not dry:
                pass                      # 템플릿의 npz 는 이미 «실사본» 으로 복사했으므로 보관해도 된다
            if not dry:
                sh([sys.executable, SLOT_PY, "archive", s], cwd=REPO, check=False)
            else:
                log("   (dry-run) archive %s" % s)
    elif old:
        log("\n■ 옛 활성 슬롯 %d 개는 --no-archive 로 남긴다: %s" % (len(old), ", ".join(old)))

    # ⑥ check · index · commit
    if not dry:
        log("\n■ check")
        sh([sys.executable, SLOT_PY, "check"], cwd=REPO, check=False)
        sh([sys.executable, SLOT_PY, "index"], cwd=REPO, check=False)
        if not a.no_commit:
            files = [os.path.relpath(ACTIVE, REPO), os.path.relpath(INDEX, REPO)]
            for _, s in made:
                for rel in ("params/deploy.yaml", "ONNX_META.json"):
                    files.append(os.path.relpath(os.path.join(SLOTDIR, s, rel), REPO))
            for s in old:
                files.append(os.path.relpath(os.path.join(SLOTDIR, s), REPO))   # 보관으로 빠진 무게(untracked 라 no-op)
            sh(["git", "add", "--"] + [f for f in files if os.path.exists(os.path.join(REPO, f))], cwd=REPO)
            st = subprocess.run(["git", "status", "--porcelain", "--", POLICY], cwd=REPO,
                                text=True, capture_output=True).stdout.strip()
            if st:
                msg = "deploy(%s): 실기 후보 %s" % (day, ", ".join("%s=%s" % (al, s) for al, s in made))
                by_alias = {c["alias"]: c for c in cands}
                body = "\n".join("- %s: %s (min_swing %s) %s" % (al, s, by_alias[al].get("min_swing", 0),
                                                                 by_alias[al].get("note", ""))
                                 for al, s in made)
                if old and not a.no_archive:
                    body += "\n\n보관(가중치→보관소, 정체는 git): " + ", ".join(old)
                sh(["git", "commit", "-q", "-m", msg, "-m", body], cwd=REPO)
                log("   커밋: %s" % msg)
            else:
                log("   커밋할 변경 없음")

    # ⑦ 로봇
    log("\n■ 로봇")
    push_cmd = "python3 deploy/scripts/policy_slot.py push"
    dep_cmd = "%s deploy" % ROBOT_SH
    if a.robot and not dry:
        r = subprocess.run([ROBOT_SH, "check"], text=True, capture_output=True, timeout=60)
        if r.returncode != 0:
            log("   로봇이 안 닿는다 (노트북이 로봇 옆에 있어야 한다). 나중에:")
            log("     cd %s && git push origin %s && %s && %s" % (REPO, "$(git branch --show-current)", push_cmd, dep_cmd))
        else:
            sh(["git", "push", "origin", subprocess.run(["git", "branch", "--show-current"], cwd=REPO,
                                                         text=True, capture_output=True).stdout.strip()], cwd=REPO)
            sh([sys.executable, SLOT_PY, "push"], cwd=REPO, check=False)
            sh([ROBOT_SH, "deploy"], check=False)
    else:
        log("   (전송 생략) 로봇 옆에서:  cd %s && git push origin $(git branch --show-current) && %s && %s"
            % (REPO, push_cmd, dep_cmd))

    log("\n■ 로봇 앞에서")
    for al, s in made:
        log("   ./deploy/robots/g1/tools/run_g1_with_gui.sh eth0 --policy %s     # %s" % (al, s))
    return 0


if __name__ == "__main__":
    sys.exit(main())
