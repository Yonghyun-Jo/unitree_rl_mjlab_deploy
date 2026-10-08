#!/usr/bin/env python3
"""check_slot_parity.py — 배포 슬롯이 «학습과 같은 입력·같은 작동» 인지 항목별 표로 판정한다 (Tier A, 파일만).

# 왜 있나
기동 시 obs 계약(ONNX metadata ↔ deploy.yaml)은 «구조» — 항 이름·순서·차원·history — 만 본다.
값의 뜻(scale/clip, 기본 자세, action scale·offset, PD gain, 관절 순서, step_dt, 명령 범위, 발-z 생성
조건)은 아무 데서도 대조하지 않았다. exporter 는 그 몫을 verify-deploy-obs-parity 스킬에 넘겼는데
(mjlab_g1_motion export_multihead_onnx.py `_obs_contract` docstring) 그 스킬은 IsaacLab/cc.cpp 용이라
mjlab→g1_ctrl 경로에서는 한 번도 돌지 않았다. 사람이 «맞겠지» 로 넘기던 것을 기계가 표로 판정한다.

# 학습 쪽 진실은 «그 체크포인트를 학습한 commit» 에서 읽는다 — HEAD 가 아니다
각 run 의 training_meta.json commit 을 `git archive` 로 풀어 **그 시점 코드를 import** 한다(학습 env cfg 를
그대로 지어 본다. sim·GPU 는 안 쓴다). launch 때 미커밋 변경이 parity 관련 파일을 건드렸으면 그 diff 까지
얹는다. 결과는 commit 별 JSON 으로 캐시한다:
    ${G1_PARITY_CACHE:-~/.cache/g1_slot_parity}/truth/      (repo 밖이라 git 과 무관. 지워도 다시 만든다)

# 판정 — 이미 있는 판정 함수를 다시 짜지 않고 부른다
  policy_slot.onnx_md5_status · stage_candidates.export_mismatch / foot_gen_at ·
  gen_obs_block.block / DEPLOY_NAME · gen_gait_lut_header(--check, load_at/table_block)
행 판정 = PASS · FAIL · KNOWN(parity_known.yaml 에 사유와 함께 적힌 «알고 배포하는» 불일치) · WARN · SKIP.
🔴 필수 행에서 «읽지 못함» 은 FAIL 이다. 못 읽은 것을 통과로 치면 계측기가 안 움직여도 초록불이 된다.
슬롯 판정 = FAIL 이 하나라도 있으면 FAIL, 아니면 KNOWN 이 있으면 KNOWN, 아니면 PASS (WARN·SKIP 은 안 바꾼다).

# 출력
  <slot>/PARITY.md   사람이 읽는 표
  <slot>/PARITY.json 게이트가 읽는다 — policy_slot.py push · robot.sh verify(verify_deploy.py)가
                     «판정 PASS/KNOWN + ONNX md5·deploy.yaml sha256 가 판정 때와 같은가» 를 본다.
  내용이 같으면(시각만 다르면) 파일을 다시 쓰지 않는다 — 다시 돌릴 때마다 git diff 가 생기지 않게.

# 사용
    python3 deploy/scripts/check_slot_parity.py <slot>        # 슬롯 이름 또는 슬롯 폴더 경로
    python3 deploy/scripts/check_slot_parity.py --active      # ACTIVE.yaml 의 별칭 전부
      --no-write   표만 찍고 PARITY.* 는 안 쓴다
      --brief      화면에는 PASS 가 아닌 행만 (stage_candidates 가 쓴다. 파일은 늘 전체 표)
      --known P    허용목록 파일 (기본 deploy/robots/g1/config/policy/parity_known.yaml)
    종료코드: 0 = 전부 PASS/KNOWN · 1 = FAIL 이 있다
  yaml·onnx 가 없는 파이썬으로 부르면 mjlab venv 파이썬으로 스스로 다시 뜬다(conda 변수는 뺀다).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import importlib.util
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
G1 = os.path.join(REPO, "deploy/robots/g1")
POLICY = os.path.join(G1, "config/policy")
SLOTDIR = os.path.join(POLICY, "mimic_masked")
KNOWN_DEFAULT = os.path.join(POLICY, "parity_known.yaml")
CACHE = os.environ.get("G1_PARITY_CACHE", os.path.expanduser("~/.cache/g1_slot_parity"))
TOOL_VERSION = 1
TRUTH_VERSION = 1          # _extract_truth 의 출력 형식. 바꾸면 올린다(캐시가 같이 무효가 된다)

# 판정에 쓴 «배포 쪽» 파일 (repo 상대). 판정 뒤에 이것이 바뀌면 PARITY.json 의 근거가 낡은 것이다 —
# 게이트(policy_slot·verify_deploy)가 WARN 으로 알린다. ONNX·deploy.yaml 은 따로(하드) 본다.
CPP_FILES = {
    "state_mimic": "deploy/robots/g1/src/State_Mimic.cpp",
    "observations": "deploy/include/isaaclab/envs/mdp/observations/observations.h",
    "mode_table": "deploy/robots/g1/include/ModeTable.h",
    "features": "deploy/robots/g1/include/DeployFeatures.h",
    "loco": "deploy/robots/g1/include/MaskedLocoController.h",
    "gait_lut": "deploy/robots/g1/include/GaitLut.h",
    "sim_config": "simulate/config.yaml",
    "g1_config": "deploy/robots/g1/config/config.yaml",
}
CONTEXT_FILES = tuple(v for k, v in CPP_FILES.items() if k != "g1_config")   # config.yaml 은 날마다 바뀐다

# launch 때 미커밋 변경이 이 경로를 건드렸으면 «그 시점 코드» 는 commit 만으로 복원되지 않는다 → diff 를 얹는다.
TRIGGER_PREFIXES = (
    "src/mjlab_g1_motion/assets/",
    "src/mjlab_g1_motion/tasks/mimic_env_cfg.py",
    "src/mjlab_g1_motion/tasks/student_mimic_env_cfg.py",
    "src/mjlab_g1_motion/tasks/teacher_env_cfg.py",
    "src/mjlab_g1_motion/tasks/stage4_",
    "src/mjlab_g1_motion/tasks/g1_mimic_env.py",
    "src/mjlab_g1_motion/tasks/mdp/observations.py",
    "src/mjlab_g1_motion/tasks/mdp/loco_controller.py",
    "src/mjlab_g1_motion/tasks/mdp/gait_lut",
    "src/mjlab_g1_motion/tasks/mdp/foot_gen.py",
)
# 회전 비대칭(turn_asym)을 학습에 넣은 commit (2026-08-17). GaitLut.h·deploy.yaml 주석이 같은 근거를 댄다.
# 이 commit 이후의 lut head 는 회전 중 바깥발을 더 드는 발-z 명령으로 학습됐다.
TURN_ASYM_COMMIT = "17995ba"

V_PASS, V_FAIL, V_KNOWN, V_WARN, V_SKIP = "PASS", "FAIL", "KNOWN", "WARN", "SKIP"
_CLEAN_ENV_DROP = ("LD_LIBRARY_PATH", "CONDA_PREFIX", "CONDA_DEFAULT_ENV", "CONDA_SHLVL", "PYTHONPATH")


# ── 작은 도구 ────────────────────────────────────────────────────────────
def _clean_env(extra=None):
    """conda 가 켜진 셸의 LD_LIBRARY_PATH 는 uv venv 의 torch 를 깬다 — 자식 파이썬에는 안 넘긴다.
    CUDA_VISIBLE_DEVICES="" : 학습이 GPU 를 쓰는 중일 수 있다. 진실 추출은 cfg 를 지을 뿐이라 GPU 가 필요 없다."""
    env = {k: v for k, v in os.environ.items() if k not in _CLEAN_ENV_DROP}
    env["CUDA_VISIBLE_DEVICES"] = ""
    env.update(extra or {})
    return env


def _hash_file(path, algo):
    h = hashlib.new(algo)
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git(repo, *args, check=False):
    r = subprocess.run(["git", "-C", repo] + list(args), capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError("git %s 실패: %s" % (" ".join(args), r.stderr.strip()[:200]))
    return r


def _short(c):
    return (c or "")[:7] or "?"


def _fmt(v):
    """표 칸에 들어갈 짧은 값."""
    if isinstance(v, float):
        return "%g" % v
    if isinstance(v, (list, tuple)):
        if len(v) > 6 and len(set(map(lambda x: round(x, 6) if isinstance(x, float) else x, v))) == 1:
            return "[%s]×%d" % (_fmt(v[0]), len(v))
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    if v is None:
        return "없음"
    return str(v)


def _fmt_vec(v, n=6):
    """관절 29개 벡터를 표 칸에 맞게 — 앞 n 개 + 개수. 다른 관절은 비고 칸에 이름으로 적는다."""
    v = [round(float(x), 4) for x in v]
    return "%d값 [%s%s]" % (len(v), ", ".join(_fmt(x) for x in v[:n]), ", …" if len(v) > n else "")


def _same(a, b, tol=1e-6):
    """허용오차 비교 — 숫자는 tol, 리스트는 원소별, 그 밖은 ==."""
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b or a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    return a == b


def _rel_close(a, b, rtol):
    """배포 yaml 은 소수 3~4 자리로 반올림돼 있다 — 상대오차로 본다(실측 최대 2.8e-5)."""
    if a is None or b is None:
        return False
    return abs(float(a) - float(b)) <= rtol * max(abs(float(a)), abs(float(b)), 1e-12)


# ── 학습 진실 추출 (자식 프로세스: mjlab venv 파이썬 + 그 commit 의 src) ─────────────
def _extract_truth(src, out):
    """`git archive` 로 푼 그 commit 의 src 를 import 해서 학습 env 가 «실제로 쓰는 값» 을 JSON 으로 남긴다.

    값을 다시 계산하지 않는다 — 학습 cfg 빌더(unitree_g1_teacher_env_cfg)가 짓는 cfg 객체와 mjlab 자신의
    이름 해석 함수(resolve_expr · filter_exp · resolve_matching_names_values)로 읽는다. 그래야 학습과 같은
    규칙으로 «관절별 값» 이 나온다. 모션 파일은 env 를 만들 때만 읽혀서 cfg 짓기에는 필요 없다."""
    import inspect
    import importlib as _il
    t0 = time.time()
    sys.path.insert(0, src)
    import mjlab_g1_motion
    here = os.path.realpath(mjlab_g1_motion.__file__)
    if not here.startswith(os.path.realpath(src) + os.sep):
        raise RuntimeError("archive 가 아니라 설치본을 import 했다: %s" % here)   # 계측기 자기검증
    from mjlab_g1_motion.assets import robot_spec as R
    from mjlab_g1_motion.tasks.mimic_env_cfg import unitree_g1_teacher_env_cfg
    from mjlab_g1_motion.tasks.student_mimic_env_cfg import G1StudentObservations
    from mjlab.managers.observation_manager import ObservationTermCfg
    from mjlab.utils.string import resolve_expr, filter_exp
    from mjlab.utils.lab_api.string import resolve_matching_names_values

    def js(v):                                   # JSON 으로 갈 수 있게 (tuple → list, dict 키 → str)
        if isinstance(v, dict):
            return {str(k): js(x) for k, x in v.items()}
        if isinstance(v, (list, tuple)):
            return [js(x) for x in v]
        if isinstance(v, (int, float, str, bool)) or v is None:
            return v
        return repr(v)

    J = tuple(R.JOINT_ORDER)
    cfg = unitree_g1_teacher_env_cfg("/nonexistent_parity_probe.npz", num_envs=1)
    robot = cfg.scene.entities["robot"]
    default = [float(x) for x in resolve_expr(robot.init_state.joint_pos, J, 0.0)]
    kp, kd, owners = [None] * len(J), [None] * len(J), [0] * len(J)
    for a in robot.articulation.actuators:
        for n in filter_exp(a.target_names_expr, J):
            i = J.index(n)
            kp[i], kd[i] = float(a.stiffness), float(a.damping)
            owners[i] += 1
    act = cfg.actions["joint_pos"]

    def per_joint(field, fill):
        if isinstance(field, dict):
            vals = [fill] * len(J)
            idx, _, vs = resolve_matching_names_values(field, J)
            for i, v in zip(idx, vs):
                vals[i] = float(v)
            return vals
        return [float(field)] * len(J)

    scale = per_joint(act.scale, 1.0)
    use_default_offset = bool(getattr(act, "use_default_offset", False))
    offset = default if use_default_offset else per_joint(getattr(act, "offset", 0.0), 0.0)

    P = G1StudentObservations.StudentPolicyCfg
    terms = []
    for k, v in vars(P).items():
        if isinstance(v, ObservationTermCfg):
            n = v.noise
            terms.append(dict(
                name=k, func=getattr(v.func, "__name__", str(v.func)), func_module=getattr(v.func, "__module__", ""),
                scale=js(v.scale), clip=js(v.clip), history=getattr(v, "history_length", None),
                noise=None if n is None else [type(n).__name__, getattr(n, "n_min", None), getattr(n, "n_max", None)]))

    res = dict(truth_version=TRUTH_VERSION, J=list(J), default=default, kp=kp, kd=kd, owners=owners, scale=scale,
               use_default_offset=use_default_offset, offset=offset, act_clip=js(getattr(act, "clip", None)),
               act_type=type(act).__name__, decimation=cfg.decimation, timestep=cfg.sim.mujoco.timestep,
               step_dt=cfg.decimation * cfg.sim.mujoco.timestep,
               obs_terms=terms, obs_history=getattr(P, "HISTORY_LENGTH", None),
               obs_corruption=getattr(P, "ENABLE_CORRUPTION", None))

    from mjlab_g1_motion.tasks import g1_mimic_env as E
    try:
        import torch
        mu, ml = E._mode_to_masks(torch.tensor([1, 2, 3]))
        res["mode_masks"] = {str(m): [float(a), float(b)] for m, a, b in zip([1, 2, 3], mu.tolist(), ml.tolist())}
    except Exception as e:                                    # noqa: BLE001 — 그 commit 에 없으면 «모름»
        res["mode_masks"] = None
        res["mode_masks_error"] = repr(e)[:200]
    try:
        sig = inspect.signature(E.G1MimicEnv.enable_foot_gen)
        res["foot_gen_defaults"] = js({k: p.default for k, p in sig.parameters.items()
                                      if p.default is not inspect.Parameter.empty})
    except Exception as e:                                    # noqa: BLE001
        res["foot_gen_defaults"] = None
        res["foot_gen_defaults_error"] = repr(e)[:200]
    st = {}
    for m in (1, 2, 3):
        try:
            M = _il.import_module("mjlab_g1_motion.tasks.stage4_mode%d_env_cfg" % m)
            st[str(m)] = dict(present=True, has_foot_gen=hasattr(M, "FOOT_GEN"),
                              FOOT_GEN=js(getattr(M, "FOOT_GEN", None)), CMD_BASE_VEL=js(getattr(M, "CMD_BASE_VEL", None)))
        except Exception as e:                                # noqa: BLE001 — base commit 에는 없을 수 있다
            st[str(m)] = dict(present=False, error=repr(e)[:200])
    res["stage4"] = st
    from mjlab.envs.mdp import observations as MO
    res["last_action_src"] = inspect.getsource(MO.last_action)
    res["imported_from"] = here
    res["elapsed_s"] = round(time.time() - t0, 2)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False)


def _truth_sig():
    import inspect
    return hashlib.sha1(("%d|%s" % (TRUTH_VERSION, inspect.getsource(_extract_truth))).encode()).hexdigest()[:12]


def _mjlab_python(mjlab):
    """학습 코드를 import 할 파이썬. 지금 파이썬이 mjlab·torch 를 import 할 수 있으면 그것, 아니면 그 워크트리의 venv."""
    if importlib.util.find_spec("mjlab") is not None and importlib.util.find_spec("torch") is not None:
        return sys.executable
    py = os.path.join(mjlab, ".venv", "bin", "python")
    return py if os.path.exists(py) else None


def _lib_sig(mjlab):
    """mjlab 라이브러리(전 워크트리 공유 path dep)의 판 — 바뀌면 cfg 해석이 바뀔 수 있어 캐시 키에 넣는다."""
    lib = os.path.join(os.path.dirname(os.path.abspath(mjlab)), "mjlab")
    r = _git(lib, "rev-parse", "HEAD")
    return r.stdout.strip()[:12] if r.returncode == 0 else "nolib"


_TRUTH_MEM = {}


def _is_trigger(path):
    return any(path.startswith(p) for p in TRIGGER_PREFIXES)


def commit_truth(mjlab, run_dir):
    """run 의 training_meta.json 이 가리키는 그 시점 코드의 «학습 진실». (truth | None, info)."""
    info = {"run": os.path.basename(run_dir.rstrip("/")), "commit": None, "problems": [], "notes": []}
    tm_p = os.path.join(run_dir, "training_meta.json")
    if not os.path.exists(tm_p):
        info["problems"].append("training_meta.json 없음 — 이 head 가 어느 코드로 학습됐는지 알 수 없다")
        return None, info
    try:
        tm = json.load(open(tm_p, encoding="utf-8"))
    except Exception as e:                                    # noqa: BLE001
        info["problems"].append("training_meta.json 을 못 읽음: %s" % e)
        return None, info
    commit = (tm.get("commit") or "").strip()
    info["commit"] = commit
    info["branch"] = tm.get("branch")
    if not commit:
        info["problems"].append("training_meta.json 에 commit 이 없다")
        return None, info
    if _git(mjlab, "cat-file", "-e", commit + "^{commit}").returncode != 0:
        info["problems"].append("commit %s 이 mjlab git 에 없다" % _short(commit))
        return None, info
    diff = tm.get("diff") or ""
    files = re.findall(r"^diff --git a/(\S+)", diff, re.M)
    if tm.get("diff_too_large"):
        st = [ln[3:].strip() for ln in (tm.get("status_porcelain") or "").splitlines() if ln and not ln.startswith("??")]
        trig = [f for f in st if _is_trigger(f)]
        if trig:
            info["problems"].append("launch 미커밋 diff 가 기록되지 않았는데(diff_too_large) parity 파일이 바뀌어 있었다: %s"
                                    % ", ".join(trig[:4]))
            return None, info
        if st:
            info["notes"].append("launch 미커밋 변경 %d 파일(기록 없음) — parity 무관 파일만" % len(st))
    trig = [f for f in files if _is_trigger(f)]
    if files and not trig:
        info["notes"].append("launch 미커밋 변경 %d 파일 — parity 무관 파일만" % len(files))
    if trig:
        info["notes"].append("launch 미커밋 변경 반영: %s" % ", ".join(trig))
    info["applied_trigger_files"] = trig
    key = "%s_%s_%s_%s" % (commit[:12], hashlib.sha1(diff.encode()).hexdigest()[:8] if trig else "clean",
                           _truth_sig(), _lib_sig(mjlab))
    if key in _TRUTH_MEM:
        return _TRUTH_MEM[key], info
    cache_p = os.path.join(CACHE, "truth", key + ".json")
    if os.path.exists(cache_p):
        try:
            t = json.load(open(cache_p, encoding="utf-8"))
            _TRUTH_MEM[key] = t                       # 캐시 여부는 표에 안 적는다 — 같은 판정이 실행마다 달라 보이면 안 된다
            return t, info
        except Exception:                                     # noqa: BLE001 — 깨진 캐시는 다시 만든다
            pass
    py = _mjlab_python(mjlab)
    if py is None:
        info["problems"].append("mjlab 을 import 할 파이썬이 없다 (%s/.venv 없음)" % mjlab)
        return None, info
    os.makedirs(os.path.join(CACHE, "work"), exist_ok=True)
    work = tempfile.mkdtemp(prefix="truth_", dir=os.path.join(CACHE, "work"))
    try:
        ar = subprocess.run(["git", "-C", mjlab, "archive", "--format=tar", commit, "src/mjlab_g1_motion"],
                            capture_output=True)
        if ar.returncode != 0:
            info["problems"].append("git archive 실패: %s" % ar.stderr.decode(errors="replace").strip()[:200])
            return None, info
        with tarfile.open(fileobj=io.BytesIO(ar.stdout)) as tf:
            # 우리 repo 의 git archive 다. src 안에 절대경로 심링크(_wandb_upload_gate.py)가 있어 "data" 필터는 거부한다
            # — "tar" 필터는 경로 탈출만 막고 심링크는 그대로 둔다(그 파일은 진실 추출에서 import 하지 않는다).
            try:
                tf.extractall(work, filter="tar")
            except TypeError:                                 # filter 인자가 없는 옛 파이썬
                tf.extractall(work)
        if trig:
            dp = os.path.join(work, "launch.diff")
            open(dp, "w", encoding="utf-8").write(diff)
            ap = subprocess.run(["git", "apply", "-p1", "--include=src/*", dp], cwd=work,
                                capture_output=True, text=True)
            if ap.returncode != 0:
                info["problems"].append("launch 미커밋 diff 를 그 commit 에 얹지 못했다: %s" % ap.stderr.strip()[:200])
                return None, info
        out = os.path.join(work, "truth.json")
        r = subprocess.run([py, os.path.abspath(__file__), "--_truth", os.path.join(work, "src"), out],
                           capture_output=True, text=True, env=_clean_env(), cwd=work, timeout=600)
        if r.returncode != 0 or not os.path.exists(out):
            tail = (r.stderr or r.stdout or "").strip().splitlines()[-3:]
            info["problems"].append("그 commit 의 학습 cfg 를 못 지었다: %s" % " / ".join(tail)[:300])
            return None, info
        t = json.load(open(out, encoding="utf-8"))
        t["commit"] = commit
        os.makedirs(os.path.dirname(cache_p), exist_ok=True)
        with open(cache_p, "w", encoding="utf-8") as f:
            json.dump(t, f, ensure_ascii=False)
        _TRUTH_MEM[key] = t
        return t, info
    finally:
        shutil.rmtree(work, ignore_errors=True)


# ── 배포 쪽 사실 ─────────────────────────────────────────────────────────
def _read(rel):
    p = os.path.join(REPO, rel)
    return open(p, encoding="utf-8").read() if os.path.exists(p) else None


def _strip_cpp_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def cpp_facts():
    """C++ 소스에서 판정에 쓰는 상수·표를 읽는다. 못 읽은 칸은 None 으로 남겨 해당 행이 FAIL 하게 한다."""
    f = {"problems": []}
    sm = _read(CPP_FILES["state_mimic"])
    ob = _read(CPP_FILES["observations"])
    mt = _read(CPP_FILES["mode_table"])
    df = _read(CPP_FILES["features"])
    lc = _read(CPP_FILES["loco"])
    regs = set()
    for t in (ob, sm):
        if t:
            regs |= set(re.findall(r"REGISTER_OBSERVATION\((\w+)\)", t))
    f["registered"] = regs
    caps = {}
    if sm:
        for k in ("VX_MAX_FWD", "VX_MAX_BWD", "KB_MAXVY", "KB_MAXW"):
            m = re.search(r"static constexpr float %s\s*=\s*([0-9.]+)f?;" % k, sm)
            caps[k] = float(m.group(1)) if m else None
        m = re.search(r'n\["source"\]\s*\?\s*n\["source"\]\.as<std::string>\(\)\s*:\s*"(\w+)"', sm)
        f["gait_source_default"] = m.group(1) if m else None
        m = re.search(r"REGISTER_OBSERVATION\(command_mask\)\s*\{(.*?)\n\}", sm, re.S)
        f["command_mask_body"] = m.group(1) if m else None
    f["caps"] = caps
    if ob:
        m = re.search(r"REGISTER_OBSERVATION\(last_action\)\s*\{(.*?)\n\}", ob, re.S)
        f["last_action_body"] = m.group(1) if m else None
    rows = {}
    if mt:
        for m in re.finditer(r'\{(\d+),\s*"(\w+)",\s*\'(.)\',\s*\{\{([^}]*)\}\},\s*'
                             r'(true|false),\s*(true|false),\s*(true|false),\s*(true|false),\s*(true|false),\s*'
                             r'(true|false),\s*(true|false),\s*RefSource::(\w+),\s*FootZ::(\w+),[^"]*"(\w+)"\}', mt):
            g = m.groups()
            rows[int(g[0])] = dict(name=g[1], bits=[float(x.strip().rstrip("f")) for x in g[3].split(",")],
                                   track_upper=g[4] == "true", track_lower=g[5] == "true",
                                   base_vel_live=g[6] == "true", foot_z=g[12], gait_key=g[13])
    f["mode_rows"] = rows
    known = None
    if df:
        m = re.search(r"static const std::vector<std::string> k = \{(.*?)\};", df, re.S)
        if m:
            known = re.findall(r'"([A-Za-z0-9_]+)"', _strip_cpp_comments(m.group(1)))
    f["features_known"] = known
    mg, ctl = {}, {}
    if lc:
        m = re.search(r"struct ModeGait \{(.*?)\n  \};", lc, re.S)
        if m:
            body = _strip_cpp_comments(m.group(1))
            for k, v in re.findall(r"\b(?:int|float|bool)\s+(\w+)\s*=\s*([^;]+);", body):
                v = v.strip().rstrip("f")
                mg[k] = (v == "true") if v in ("true", "false") else float(v)
            rest = _strip_cpp_comments(lc[m.end():])
            for k in ("walk_max", "run_min", "settle_eff", "settle_phase", "turn_k", "period_steps",
                      "stance_z", "height_scale"):
                mm = re.search(r"\b%s\s*=\s*([0-9.]+)f?\s*[,;]" % k, rest)
                if mm:
                    ctl[k] = float(mm.group(1))
    f["mode_gait_defaults"] = mg
    f["loco_defaults"] = ctl
    return f


def scene_order():
    """sim2sim 이 쓰는 MuJoCo 장면의 모터·관절센서 순서. 브리지가 motor i ↔ ctrl[i]·sensordata[i] 로 잇는다."""
    import xml.etree.ElementTree as ET
    sc = _read(CPP_FILES["sim_config"]) or ""
    m = re.search(r'^robot_scene:\s*"([^"]+)"', sc, re.M)
    if not m:
        return None, "simulate/config.yaml 에 robot_scene 이 없다"
    rel = m.group(1)
    p = os.path.join(REPO, rel)
    if not os.path.exists(p):
        return None, "장면 파일이 없다: %s" % rel
    root = ET.parse(p).getroot()
    act = [e.get("joint") for e in root.iter() if e.tag in ("motor", "position", "general") and e.get("joint")]
    jp = [e.get("joint") for e in root.iter("jointpos")]
    jv = [e.get("joint") for e in root.iter("jointvel")]
    return {"scene": rel, "actuators": act, "jointpos": jp, "jointvel": jv}, None


def onnx_facts(path):
    import onnx
    from onnx import numpy_helper
    m = onnx.load(path, load_external_data=False)
    md = {p.key: p.value for p in m.metadata_props}
    shp = lambda vi: [d.dim_value if d.dim_value else (d.dim_param or "?") for d in vi.type.tensor_type.shape.dim]  # noqa: E731
    ins = [(i.name, shp(i)) for i in m.graph.input]
    outs = [(o.name, shp(o)) for o in m.graph.output]
    inits = {i.name: i for i in m.graph.initializer}
    consts = {}
    for n in m.graph.node:
        if n.op_type == "Constant":
            for a in n.attribute:
                if a.name == "value":
                    consts[n.output[0]] = numpy_helper.to_array(a.t)

    def val(name):
        if name in inits:
            return numpy_helper.to_array(inits[name])
        return consts.get(name)
    slices = []
    in_names = {i[0] for i in ins}
    for n in m.graph.node:
        if n.op_type == "Slice" and n.input and n.input[0] in in_names:
            vs = [val(x) for x in n.input[1:4]]
            if all(v is not None for v in vs):
                slices.append(dict(start=int(vs[0].ravel()[0]), end=int(vs[1].ravel()[0]), axis=int(vs[2].ravel()[0])))
    return dict(meta=md, inputs=ins, outputs=outs, slices=slices)


# ── 판정 ────────────────────────────────────────────────────────────────
class Report:
    def __init__(self, slot):
        self.slot = slot
        self.rows = []
        self._n = {}
        self.evaluated = set()

    def add(self, group, rid, item, train, deploy, verdict, note="", mismatches=None):
        g = group.split(" ", 1)[0]
        self._n[g] = self._n.get(g, 0) + 1
        self.rows.append(dict(no="%s.%d" % (g, self._n[g]), group=group, id=rid, item=item,
                              train=train, deploy=deploy, verdict=verdict, note=note,
                              mismatches=mismatches or []))
        self.evaluated.add(rid)

    def verdict(self):
        vs = [r["verdict"] for r in self.rows]
        if V_FAIL in vs:
            return V_FAIL
        return V_KNOWN if V_KNOWN in vs else V_PASS

    def counts(self):
        c = {}
        for r in self.rows:
            c[r["verdict"]] = c.get(r["verdict"], 0) + 1
        return c


def load_known(path):
    """허용목록. 못 읽으면 (None, 이유) — 그때는 알려진 불일치도 전부 FAIL 로 본다."""
    if not os.path.exists(path):
        return [], "허용목록 파일이 없다 (%s) — 모든 불일치를 FAIL 로 본다" % os.path.relpath(path, REPO)
    try:
        import yaml
        d = yaml.safe_load(open(path, encoding="utf-8")) or {}
        ents = d.get("known") or []
        for e in ents:
            for k in ("id", "row", "key", "train", "deploy", "reason"):
                if k not in e:
                    return None, "허용목록 항목에 %s 가 없다: %s" % (k, e.get("id", e))
        return ents, None
    except Exception as e:                                    # noqa: BLE001
        return None, "허용목록을 못 읽음: %s" % e


def judge_mismatches(rid, mismatches, known, used):
    """불일치 [(key, 학습값, 배포값)] → (verdict, note). 허용목록에 «같은 값으로» 적힌 것만 KNOWN."""
    if not mismatches:
        return V_PASS, ""
    notes, covered = [], 0
    for key, tv, dv in mismatches:
        hit = None
        for e in (known or []):
            if e["row"] == rid and str(e["key"]) == str(key):
                hit = e
                break
        if hit is not None and _same(hit["train"], tv) and _same(hit["deploy"], dv):
            covered += 1
            used.add(hit["id"])
            notes.append("%s: 학습 %s ≠ 배포 %s — %s (%s)" % (key, _fmt(tv), _fmt(dv), hit["id"],
                                                          str(hit.get("decision", "")).strip()))
        elif hit is not None:
            used.add(hit["id"])
            notes.append("%s: 학습 %s ≠ 배포 %s — 허용목록 %s 는 학습 %s / 배포 %s 로 적혀 있다(값이 달라졌다)"
                         % (key, _fmt(tv), _fmt(dv), hit["id"], _fmt(hit["train"]), _fmt(hit["deploy"])))
        else:
            notes.append("%s: 학습 %s ≠ 배포 %s" % (key, _fmt(tv), _fmt(dv)))
    v = V_KNOWN if covered == len(mismatches) else V_FAIL
    return v, " · ".join(notes)


def _mjlab_of(meta):
    """export_cmd 의 `cd <mjlab> &&` — 체크포인트 경로가 상대경로인 기준. 없으면 stage_candidates 와 같은 기본값."""
    m = re.search(r"cd\s+(\S+)\s+&&", str(meta.get("export_cmd") or ""))
    if m and os.path.isdir(m.group(1)):
        return m.group(1)
    return os.environ.get("MJLAB_WS", os.path.expanduser("~/mjlab1.4/mjlab_g1_motion"))


def _ckpt_abs(mjlab, ck):
    if not ck or str(ck).strip().lower() == "none":
        return None
    return ck if os.path.isabs(ck) else os.path.join(mjlab, ck)


def check_slot(slot_root, known_path=KNOWN_DEFAULT):
    """슬롯 하나 → 보고서(dict). 파일은 쓰지 않는다(write_outputs 가 쓴다)."""
    import yaml
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import policy_slot
    import stage_candidates as sc
    import gen_obs_block as gob
    import gen_gait_lut_header as glh

    slot_root = os.path.abspath(slot_root.rstrip("/"))
    slot = os.path.basename(slot_root)
    rep = Report(slot)
    t0 = time.time()
    meta_p = os.path.join(slot_root, "ONNX_META.json")
    dy_p = os.path.join(slot_root, "params", "deploy.yaml")
    onnx_p = os.path.join(slot_root, "exported", "policy.onnx")
    try:
        meta = json.load(open(meta_p, encoding="utf-8"))
    except Exception as e:                                    # noqa: BLE001
        meta = None
        rep.add("1 출처·동일성", "id.meta", "ONNX_META.json", "-", "-", V_FAIL, "못 읽음: %s" % e)
    try:
        dep = yaml.safe_load(open(dy_p, encoding="utf-8"))
    except Exception as e:                                    # noqa: BLE001
        dep = None
        rep.add("1 출처·동일성", "id.deploy_yaml", "params/deploy.yaml", "-", "-", V_FAIL, "못 읽음: %s" % e)
    ox = None
    if os.path.exists(onnx_p):
        try:
            ox = onnx_facts(onnx_p)
        except Exception as e:                                # noqa: BLE001
            rep.add("1 출처·동일성", "id.onnx", "ONNX 열기", "-", "-", V_FAIL, "못 읽음: %s" % e)
    else:
        rep.add("1 출처·동일성", "id.onnx", "ONNX", "-", "exported/policy.onnx 없음", V_FAIL,
                "가중치가 없다 — `policy_slot.py restore %s`" % slot)
    if meta is None or dep is None or ox is None:
        return _finish(rep, slot_root, meta or {}, {}, t0, None)

    known, kerr = load_known(known_path)
    used = set()
    mjlab = _mjlab_of(meta)

    # ── 1 출처·동일성 ────────────────────────────────────────────
    G = "1 출처·동일성"
    h, want, okm = policy_slot.onnx_md5_status(slot, os.path.dirname(slot_root))
    if want is None:
        rep.add(G, "id.onnx_md5", "ONNX md5 == META 기록", "-", h[:12], V_FAIL, "ONNX_META.json 에 verify.onnx_md5 가 없다")
    else:
        rep.add(G, "id.onnx_md5", "ONNX md5 == META 기록", want[:12], h[:12], V_PASS if okm else V_FAIL,
                "" if okm else "슬롯의 ONNX 가 기록된 것과 다르다")
    md = ox["meta"]
    cfgc = {"base": meta.get("flow_base_checkpoint"), "sampling_steps": meta.get("sampling_steps", 6),
            "mode2_ckpt": meta.get("mode2_ckpt"), "mode3_ckpt": meta.get("mode3_ckpt")}
    why = sc.export_mismatch(md, cfgc, {"mode1_ckpt": meta.get("mode1_ckpt")})
    for k in ("obs_contract", "obs_dim"):
        if str(md.get(k, "")) != str(meta.get(k, "")):
            why.append("%s: ONNX %s ≠ META %s" % (k, md.get(k), meta.get(k)))
    if os.path.realpath(_ckpt_abs(mjlab, str(md.get("manifest") or "")) or "") != \
            os.path.realpath(_ckpt_abs(mjlab, str(meta.get("manifest") or "")) or ""):
        why.append("manifest: ONNX %s ≠ META %s" % (md.get("manifest"), meta.get("manifest")))
    rep.add(G, "id.onnx_meta", "ONNX 메타(base·head·steps·계약) == ONNX_META.json", "ONNX metadata", "ONNX_META.json",
            V_FAIL if why else V_PASS, " · ".join(why))
    contract = []
    for item in str(md.get("obs_contract") or "").split(","):
        if item.strip():
            n, d_, hh = item.strip().split(":")
            contract.append((n, int(d_), int(hh)))
    c_sum = sum(d_ * hh for _, d_, hh in contract)
    ins, outs = ox["inputs"], ox["outputs"]
    io_ok = (len(ins) == 1 and ins[0][1][-1] == c_sum and len(outs) == 1)
    rep.add(G, "id.onnx_io", "ONNX 입출력", "계약 합 %d" % c_sum,
            "%s → %s" % (", ".join("%s%s" % i for i in ins), ", ".join("%s%s" % o for o in outs)),
            V_PASS if io_ok else V_FAIL, "" if io_ok else "입력 하나(obs, 폭=계약 합)·출력 하나여야 한다")

    heads = {"base": meta.get("flow_base_checkpoint"), "1": meta.get("mode1_ckpt"),
             "2": meta.get("mode2_ckpt"), "3": meta.get("mode3_ckpt")}
    truths, infos, commits = {}, {}, {}
    for hk in ("base", "1", "2", "3"):
        label = "base" if hk == "base" else "mode%s" % hk
        ck = _ckpt_abs(mjlab, heads[hk])
        if ck is None:
            rep.add(G, "src.%s" % label, "%s 출처" % label, "-", "none", V_SKIP, "이 head 는 비어 있다(bare base 로 돈다)")
            continue
        run_dir = os.path.dirname(ck)
        t, info = commit_truth(mjlab, run_dir)
        infos[label] = info
        commits[label] = info.get("commit")
        notes = list(info["notes"])
        probs = list(info["problems"])
        if hk != "base":
            ta_p = os.path.join(run_dir, "train_args.json")
            if os.path.exists(ta_p):
                try:
                    ta = json.load(open(ta_p, encoding="utf-8"))
                    if ta.get("cmd_mode") is not None and int(ta["cmd_mode"]) != int(hk):
                        probs.append("이 체크포인트는 cmd_mode %s 로 학습됐는데 mode%s 칸에 들어 있다" % (ta["cmd_mode"], hk))
                    tm_ = ta.get("manifest")
                    if tm_ and os.path.realpath(_ckpt_abs(mjlab, tm_)) != \
                            os.path.realpath(_ckpt_abs(mjlab, str(meta.get("manifest") or "")) or ""):
                        notes.append("⚠ 학습 manifest %s ≠ export manifest %s" % (tm_, meta.get("manifest")))
                except Exception as e:                        # noqa: BLE001
                    notes.append("train_args.json 을 못 읽음: %s" % e)
            else:
                notes.append("train_args.json 없음 (2026-08-13 이전 run)")
        v = V_FAIL if (probs or t is None) else (V_WARN if any(n.startswith("⚠") for n in notes) else V_PASS)
        if t is not None:
            truths[label] = t
        rep.add(G, "src.%s" % label, "%s 학습 원장" % label,
                "%s @%s" % (info["run"][:40], _short(info.get("commit"))), "ONNX_META %s" % label, v,
                " · ".join(probs + notes))

    def sig_obs(t):
        return json.dumps([t.get("obs_terms"), t.get("obs_history"), t.get("obs_corruption")], sort_keys=True)

    def sig_act(t):
        return json.dumps([t.get("J"), [round(x, 9) for x in t.get("default") or []],
                           [round(x or 0, 9) for x in t.get("kp") or []], [round(x or 0, 9) for x in t.get("kd") or []],
                           [round(x, 9) for x in t.get("scale") or []], t.get("step_dt"),
                           t.get("use_default_offset")], sort_keys=True)
    labels = [lab for lab, hk in (("base", "base"), ("mode1", "1"), ("mode2", "2"), ("mode3", "3"))
              if _ckpt_abs(mjlab, heads[hk])]
    missing = [k for k in labels if k not in truths]
    for rid, item, sig in (("id.obs_same", "base·head 의 obs 그룹이 같은가", sig_obs),
                           ("id.act_same", "base·head 의 작동 상수(관절 순서·기본자세·PD·scale·dt)가 같은가", sig_act)):
        if missing:
            rep.add(G, rid, item, "-", "-", V_FAIL, "학습 진실을 못 읽은 체크포인트: %s" % ", ".join(missing))
            continue
        groups = {}
        for k in labels:
            groups.setdefault(sig(truths[k]), []).append(k)
        same = len(groups) == 1
        rep.add(G, rid, item, " · ".join("%s@%s" % (k, _short(commits.get(k))) for k in labels),
                "하나의 obs 벡터·하나의 작동을 공유", V_PASS if same else V_FAIL,
                "" if same else "갈림: " + " | ".join(",".join(v) for v in groups.values()))
    ec = meta.get("export_commit")
    if not ec or not ec.get("head"):
        rep.add(G, "id.export_commit", "export 시점 mjlab commit", "-", "기록 없음", V_SKIP,
                (ec or {}).get("note") or "이 기록을 남기기 전에 만든 슬롯 — 판정은 launch commit 진실로 한다")
    else:
        dirty = [f for f in (ec.get("dirty") or []) if _is_trigger(f)]
        rep.add(G, "id.export_commit", "export 시점 mjlab commit", "-",
                "%s%s" % (_short(ec["head"]), " (미커밋 변경 있음)" if ec.get("dirty") else ""),
                V_WARN if dirty else V_PASS, ("parity 파일이 미커밋 상태로 export 됐다: %s" % ", ".join(dirty)) if dirty else "")

    base_t = truths.get("base") or next(iter(truths.values()), None)
    cf = cpp_facts()
    obs_d = (dep or {}).get("observations") or {}

    # ── 2 obs 배치 ──────────────────────────────────────────────
    G = "2 obs 배치"
    try:
        want_blk = yaml.safe_load("\n".join(gob.block(onnx_p)))["observations"]
        gerr = None
    except SystemExit as e:
        want_blk, gerr = {}, "gen_obs_block 이 거부: %s" % e
    except Exception as e:                                    # noqa: BLE001
        want_blk, gerr = {}, "gen_obs_block 실패: %s" % e
    want_items = list(want_blk.items())
    dep_items = [(k, v) for k, v in obs_d.items() if isinstance(v, dict)]
    t_terms = (base_t or {}).get("obs_terms") or []
    t_hist = (base_t or {}).get("obs_history")
    for i in range(max(len(contract), len(dep_items))):
        c = contract[i] if i < len(contract) else None
        dn, dv = dep_items[i] if i < len(dep_items) else (None, None)
        probs, info = [], []
        if gerr:
            probs.append(gerr)
        if c is None:
            probs.append("ONNX 계약에 없는 배포 항")
        if dn is None:
            probs.append("배포 deploy.yaml 에 없는 학습 항")
        tt = t_terms[i] if i < len(t_terms) else None
        if base_t is None:
            probs.append("학습 cfg 를 못 읽었다")
        elif tt is None or (c and tt["name"] != c[0]):
            probs.append("학습 cfg 의 %d번째 항(%s) ≠ ONNX 계약(%s)" % (i + 1, tt and tt["name"], c and c[0]))
        if c and dv is not None:
            if dv.get("train_term") != c[0]:
                probs.append("train_term %s ≠ 계약 %s" % (dv.get("train_term"), c[0]))
            sc_ = dv.get("scale") or []
            if len(sc_) != c[1]:
                probs.append("scale 길이 %d ≠ 계약 차원 %d" % (len(sc_), c[1]))
            if int(dv.get("history_length", 1)) != c[2]:
                probs.append("history %s ≠ 계약 %d" % (dv.get("history_length"), c[2]))
            if i < len(want_items):
                wn, wv = want_items[i]
                if wn != dn or wv.get("params") != dv.get("params"):
                    probs.append("배포 항 %s%s ≠ 대응표 %s%s" % (dn, dv.get("params"), wn, wv.get("params")))
            else:
                probs.append("대응표(gen_obs_block)에 없는 위치")
            if dn not in cf["registered"]:
                probs.append("C++ REGISTER_OBSERVATION(%s) 없음" % dn)
            if tt is not None:
                ts = tt.get("scale")
                exp = [1.0] * c[1] if ts is None else ([float(ts)] * c[1] if isinstance(ts, (int, float)) else ts)
                if not _same([float(x) for x in sc_], exp, 1e-9):
                    probs.append("scale 학습 %s ≠ 배포 %s" % (_fmt(exp), _fmt(sc_)))
                tc = tt.get("clip")
                if not _same(tc, dv.get("clip")):
                    probs.append("clip 학습 %s ≠ 배포 %s" % (_fmt(tc), _fmt(dv.get("clip"))))
                th = t_hist if t_hist else tt.get("history")
                if th and int(th) != c[2]:
                    probs.append("학습 history %s ≠ 계약 %d" % (th, c[2]))
                if tt.get("noise"):
                    info.append("학습 노이즈 %s(%s, %s) — 배포엔 없음(정상)" % tuple(tt["noise"]))
                if tt.get("func") != dn:
                    info.append("학습 함수 %s ↔ 배포 항 %s (대응표로 잇는다)" % (tt.get("func"), dn))
        train_s = ("%s ← %s · %d×%d · scale %s · clip %s" % (c[0], (tt or {}).get("func", "?"), c[1], c[2],
                   _fmt((tt or {}).get("scale")) if tt and tt.get("scale") is not None else "없음(=1)",
                   _fmt((tt or {}).get("clip")))) if c else "-"
        dep_s = ("%s · train_term=%s · %d×%s · scale %s · clip %s" % (
            dn, dv.get("train_term"), len(dv.get("scale") or []), dv.get("history_length"),
            _fmt(dv.get("scale")), _fmt(dv.get("clip")))) if dv is not None else "-"
        rep.add(G, "obs.%s" % (c[0] if c else dn), "%d. %s" % (i + 1, c[0] if c else dn), train_s, dep_s,
                V_FAIL if probs else V_PASS, " · ".join(probs + info))
    d_sum = sum(len((v or {}).get("scale") or []) * int((v or {}).get("history_length", 1)) for _, v in dep_items)
    ins_w = ins[0][1][-1] if ins else None
    ok_sum = (d_sum == c_sum == ins_w == int(meta.get("obs_dim") or -1))
    rep.add(G, "obs.sum", "합 = ONNX 입력 = META obs_dim", "계약 %d" % c_sum,
            "deploy %d · ONNX %s · META %s" % (d_sum, ins_w, meta.get("obs_dim")), V_PASS if ok_sum else V_FAIL)
    gym = bool(obs_d.get("use_gym_history"))
    rep.add(G, "obs.layout", "history 배치", "mjlab: 항마다 history(오래된→최신)를 펴서 항 순서로 잇는다",
            "use_gym_history %s → %s" % ("true" if gym else "미설정", "frame-major" if gym else "term-major"),
            V_FAIL if gym else V_PASS, "use_gym_history: true 는 frame-major 라 학습과 순서가 다르다" if gym else "")
    mask_i = next((i for i, c in enumerate(contract) if c[0] == "mask"), None)
    if mask_i is None:
        rep.add(G, "obs.mask_slice", "head 선택 mask 위치", "계약에 mask 항 없음", "-", V_SKIP, "단일 head ONNX")
    else:
        off = 0
        for n, (_dn, dv) in enumerate(dep_items):
            if n == mask_i:
                break
            off += len(dv.get("scale") or []) * int(dv.get("history_length", 1))
        md_dim, md_h = contract[mask_i][1], contract[mask_i][2]
        dep_slice = (off + (md_h - 1) * md_dim - d_sum, off + md_h * md_dim - d_sum)
        gs = [(s["start"], s["end"]) for s in ox["slices"] if s["axis"] in (1, -1) and s["end"] - s["start"] == md_dim]
        ok_ = bool(gs) and all(g == dep_slice for g in gs)
        rep.add(G, "obs.mask_slice", "head 선택 mask 위치 (ONNX 그래프가 실제로 자르는 곳)",
                "그래프 Slice %s" % (", ".join("[%d:%d]" % g for g in gs) or "못 찾음"),
                "배포 배치의 최신 mask = [%d:%d]" % dep_slice, V_PASS if ok_ else V_FAIL,
                "" if ok_ else "그래프가 자르는 곳과 배포 배치의 mask 위치가 다르다")

    # ── 3 작동 ──────────────────────────────────────────────────
    G = "3 작동"
    J = (base_t or {}).get("J")
    sc_o, serr = scene_order()
    jmap = [int(x) for x in ((dep or {}).get("joint_ids_map") or [])]
    if J is None or sc_o is None:
        rep.add(G, "act.joint_order", "관절 순서", "-", "-", V_FAIL, serr or "학습 JOINT_ORDER 를 못 읽었다")
    else:
        probs = []
        acts = sc_o["actuators"]
        try:
            mapped = [acts[k] for k in jmap]
        except IndexError:
            mapped = None
        if mapped != list(J):
            probs.append("joint_ids_map 으로 이은 장면 모터 순서가 학습 JOINT_ORDER 와 다르다")
        if sc_o["jointpos"] != acts or sc_o["jointvel"] != acts:
            probs.append("장면의 jointpos/jointvel 센서 순서가 모터 순서와 다르다(브리지는 같은 i 로 읽는다)")
        own = (base_t or {}).get("owners") or []
        if any(o != 1 for o in own):
            probs.append("학습 actuator 그룹이 관절마다 정확히 하나가 아니다: %s"
                         % [J[i] for i, o in enumerate(own) if o != 1][:4])
        rep.add(G, "act.joint_order", "관절 순서", "JOINT_ORDER %d (%s … %s)" % (len(J), J[0], J[-1]),
                "joint_ids_map %s · 장면 %s" % ("항등" if jmap == list(range(len(jmap))) else "치환", sc_o["scene"]),
                V_FAIL if probs else V_PASS,
                " · ".join(probs) or "실기 SDK 모터 순서 = 이 장면 순서라는 전제(유니트리 G1 29dof 정의)")
    act_d = (((dep or {}).get("actions") or {}).get("JointPositionAction") or {})

    def vec_row(rid, item, tv, dv, rtol=None, atol=None, note=""):
        if tv is None or dv is None or len(tv) != len(dv):
            rep.add(G, rid, item, _fmt(tv), _fmt(dv), V_FAIL, "못 읽었거나 길이가 다르다")
            return
        if rtol is not None:
            bad = [i for i, (a, b) in enumerate(zip(tv, dv)) if not _rel_close(a, b, rtol)]
            err = max((abs(a - b) / max(abs(a), abs(b), 1e-12) for a, b in zip(tv, dv)), default=0.0)
            how = "최대 상대오차 %.1e (허용 %g)" % (err, rtol)
        else:
            bad = [i for i, (a, b) in enumerate(zip(tv, dv)) if abs(a - b) > atol]
            err = max((abs(a - b) for a, b in zip(tv, dv)), default=0.0)
            how = "최대 |Δ| %.1e (허용 %g)" % (err, atol)
        names = [J[i] for i in bad[:4]] if J else bad[:4]
        rep.add(G, rid, item, _fmt_vec(tv), _fmt_vec(dv),
                V_FAIL if bad else V_PASS, how + ((" · 다른 관절: %s" % ", ".join(names)) if bad else "") +
                ((" · " + note) if note else ""))
    T = base_t or {}
    vec_row("act.default_pos", "기본 자세 default_joint_pos", T.get("default"), (dep or {}).get("default_joint_pos"), atol=1e-6)
    vec_row("act.offset", "action offset", T.get("offset"), act_d.get("offset"), atol=1e-6,
            note="학습 use_default_offset=%s" % T.get("use_default_offset"))
    vec_row("act.scale", "action scale", T.get("scale"), act_d.get("scale"), rtol=1e-3)
    vec_row("act.kp", "PD stiffness", T.get("kp"), (dep or {}).get("stiffness"), rtol=1e-3)
    vec_row("act.kd", "PD damping", T.get("kd"), (dep or {}).get("damping"), rtol=1e-3)
    sdt, ddt = T.get("step_dt"), (dep or {}).get("step_dt")
    rep.add(G, "act.step_dt", "정책 주기 step_dt", "%s × %s = %s" % (T.get("timestep"), T.get("decimation"), sdt),
            _fmt(ddt), V_PASS if (sdt is not None and ddt is not None and abs(float(sdt) - float(ddt)) < 1e-9) else V_FAIL)
    clip = act_d.get("clip")
    dflt = (dep or {}).get("default_joint_pos") or []
    sf = (dep or {}).get("safety") or {}
    if not clip or len(clip) != len(dflt):
        rep.add(G, "act.clip", "action clip (배포 전용 기계한계)", "학습 clip %s" % _fmt(T.get("act_clip")), _fmt(clip),
                V_FAIL, "clip 이 없거나 길이가 다르다")
    else:
        out_ = [i for i, (c_, d_) in enumerate(zip(clip, dflt)) if not (c_[0] <= d_ <= c_[1])]
        eq = (sf.get("pos_min") == [c_[0] for c_ in clip] and sf.get("pos_max") == [c_[1] for c_ in clip])
        v = V_FAIL if out_ else (V_PASS if eq else V_WARN)
        rep.add(G, "act.clip", "action clip (배포 전용 기계한계)", "학습 clip %s (%s)" % (_fmt(T.get("act_clip")),
                T.get("act_type")), "29쌍 · 기본자세 포함 %s · safety pos_min/max 와 %s" % (
                "예" if not out_ else "아니오", "같다" if eq else "다르다"), v,
                ("기본자세가 clip 밖: %s" % out_) if out_ else ("" if eq else "L1 1차(clip)와 2차(pos_min/max)가 다르다"))
    lab = cf.get("last_action_body") or ""
    raw_ok = "action_manager->action()" in lab and "processed" not in lab
    t_raw = "raw_action" in (T.get("last_action_src") or "")
    rep.add(G, "act.last_action", "last_action = 정책 raw 출력", "mjlab last_action → %s" % ("raw_action" if t_raw else "?"),
            "observations.h last_action → %s" % ("action_manager->action()" if raw_ok else "?"),
            V_PASS if (raw_ok and t_raw) else V_FAIL, "" if (raw_ok and t_raw) else "scale·offset 을 거친 값을 넣으면 학습과 다르다")

    # ── 4 명령·gait ─────────────────────────────────────────────
    G = "4 명령·gait"
    rows_mt = cf.get("mode_rows") or {}
    head_modes = [m for m in (1, 2, 3) if _ckpt_abs(mjlab, heads[str(m)])]
    mask_dim = contract[mask_i][1] if mask_i is not None else None
    if mask_dim is None:
        rep.add(G, "cmd.mask", "모드 → mask 비트", "-", "-", V_SKIP, "mask 항 없음")
    elif mask_dim != 2:
        rep.add(G, "cmd.mask", "모드 → mask 비트", "mask %d칸 (계약 v2)" % mask_dim, "-", V_SKIP,
                "계약 v2 mode_mask 대조는 gen_mode_table_header --check 몫")
    else:
        mism, probs = [], []
        body = cf.get("command_mask_body") or ""
        if "b[0]" not in body or "b[1]" not in body:
            probs.append("C++ command_mask 가 표의 앞 2칸(b[0], b[1])을 안 낸다")
        for m in head_modes:
            t = truths.get("mode%d" % m)
            tv = (t or {}).get("mode_masks", {}) or {}
            tv = tv.get(str(m)) if tv else None
            dv = (rows_mt.get(m) or {}).get("bits", [None, None])[:2]
            if tv is None or None in dv:
                probs.append("mode%d: 학습 또는 ModeTable 을 못 읽음" % m)
            elif not _same(tv, dv):
                mism.append(("mode%d" % m, tv, dv))
        v, note = judge_mismatches("cmd.mask", mism, known, used)
        rep.add(G, "cmd.mask", "모드 → [mask_upper, mask_lower]",
                " · ".join("mode%d %s" % (m, _fmt(((truths.get("mode%d" % m) or {}).get("mode_masks") or {}).get(str(m))))
                           for m in head_modes),
                " · ".join("mode%d %s" % (m, _fmt((rows_mt.get(m) or {}).get("bits", [])[:2])) for m in head_modes),
                V_FAIL if probs else v, " · ".join(probs + ([note] if note else [])))
    caps = cf.get("caps") or {}
    for m in (1, 2):
        if m not in head_modes or not (rows_mt.get(m) or {}).get("base_vel_live"):
            continue
        rid = "cmd.mode%d" % m
        t = truths.get("mode%d" % m)
        cb = (((t or {}).get("stage4") or {}).get(str(m)) or {}).get("CMD_BASE_VEL")
        if not cb or None in caps.values() or not caps:
            rep.add(G, rid, "mode%d 명령 범위 (조이스틱 끝 = 학습 범위의 끝)" % m, _fmt(cb), _fmt(caps), V_FAIL,
                    "학습 CMD_BASE_VEL 또는 C++ 상한을 못 읽음")
            continue
        dvs = {"vx": [-caps["VX_MAX_BWD"], caps["VX_MAX_FWD"]], "vy": [-caps["KB_MAXVY"], caps["KB_MAXVY"]],
               "wz": [-caps["KB_MAXW"], caps["KB_MAXW"]]}
        tvs = {"vx": cb.get("vx_range"), "vy": cb.get("vy_range"), "wz": cb.get("wz_range")}
        mism = [(k, tvs[k], dvs[k]) for k in ("vx", "vy", "wz") if not _same(tvs[k], dvs[k])]
        v, note = judge_mismatches(rid, mism, known, used)
        rep.add(G, rid, "mode%d 명령 범위 (조이스틱 끝 = 학습 범위의 끝)" % m,
                "stage4_mode%d CMD_BASE_VEL vx %s vy %s wz %s @%s" % (m, _fmt(tvs["vx"]), _fmt(tvs["vy"]), _fmt(tvs["wz"]),
                                                                  _short((t or {}).get("commit"))),
                "C++ vx %s vy %s wz %s (모드 공통)" % (_fmt(dvs["vx"]), _fmt(dvs["vy"]), _fmt(dvs["wz"])), v, note)

    # 발-z 생성 조건: 학습 = 그 head commit 의 enable_foot_gen 기본값 위에 stage4_modeN.FOOT_GEN.
    #                배포 = deploy.yaml gait: 위에 C++ 기본값(MaskedLocoController.h·load_gait_cfg).
    hdr = glh.HEADER.read_text() if glh.HEADER.exists() else ""
    hdr_data = glh._data_only(hdr[hdr.index(glh.BEGIN):hdr.index(glh.END) + len(glh.END)]) \
        if (glh.BEGIN in hdr and glh.END in hdr) else ""
    gd = (dep or {}).get("gait") or {}
    mgd, lcd = cf.get("mode_gait_defaults") or {}, cf.get("loco_defaults") or {}
    if 1 in head_modes and truths.get("mode1"):
        fg_imp = (((truths["mode1"].get("stage4") or {}).get("1") or {}).get("FOOT_GEN")) or {}
        fg_sc = sc.foot_gen_at(truths["mode1"]["commit"])
        diff_ = []
        for k, v in fg_sc.items():
            iv = fg_imp.get(k)
            if isinstance(v, dict):
                iv = {int(a): b for a, b in (iv or {}).items()} if isinstance(iv, dict) else iv
            if not _same(v, iv) and not (isinstance(v, dict) and isinstance(iv, dict) and
                                         set(v) == set(iv) and all(_same(v[a], iv[a]) for a in v)):
                diff_.append("%s: foot_gen_at %s ≠ import %s" % (k, _fmt(v), _fmt(iv)))
        ok_ = bool(fg_sc) and not diff_
        rep.add(G, "gait.reader", "FOOT_GEN 판독기 두 벌이 같은 값을 읽는가 (계측기 자기검증)",
                "import (그 commit 의 cfg 모듈)", "stage_candidates.foot_gen_at (META gait_parity 를 쓴 판독기)",
                V_PASS if ok_ else V_FAIL, " · ".join(diff_) if diff_ else ("" if fg_sc else "foot_gen_at 이 아무것도 못 읽었다"))
    for m in (1, 2):
        if m not in head_modes:
            continue
        row = rows_mt.get(m) or {}
        if row.get("foot_z") != "Gen":
            continue
        rid = "gait.mode%d" % m
        t = truths.get("mode%d" % m)
        st4 = ((t or {}).get("stage4") or {}).get(str(m)) or {}
        if t is None or not st4.get("present") or not mgd or not lcd or cf.get("gait_source_default") is None:
            rep.add(G, rid, "mode%d 발-z 생성 조건" % m, "-", "-", V_FAIL, "학습 FOOT_GEN 또는 C++ 기본값을 못 읽음")
            continue
        fg, dfl = st4.get("FOOT_GEN") or {}, t.get("foot_gen_defaults") or {}

        def teff(key, off):
            """그 commit 에 그 키가 없으면 «그 기능 없이» 학습된 것 = 꺼진 값. 모드별 dict 는 없는 모드 = 꺼짐."""
            v = fg[key] if key in fg else (dfl[key] if key in dfl else off)
            if isinstance(v, dict):
                return v.get(str(m), off)
            return v
        tr = dict(source=teff("foot_source", "quintic"), cadence=float(teff("cadence_scale", 1.0)),
                  settle_steps=int(teff("settle_steps", 0)), min_swing=float(teff("min_swing", 0.0)),
                  stand_deadzone=float(teff("stand_deadzone", 0.0)), settle_eff=float(teff("settle_eff", 0.15)),
                  settle_phase=float(teff("settle_phase", 0.07)), walk_max=float(teff("walk_max", 1.2)),
                  run_min=float(teff("run_min", 1.7)), turn_k=float(teff("turn_k", 0.3)),
                  height_scale=float(teff("height_scale", 1.0)), stance_z=float(teff("stance_z", 0.066)),
                  period_steps=int(teff("period_steps", 43)))
        n = gd.get(row.get("gait_key") or "mode%d" % m) or {}
        de = dict(source=n.get("source", cf["gait_source_default"]), table=int(n.get("table", mgd.get("table", 1))),
                  cadence=float(n.get("cadence", mgd.get("cadence", 1.0))),
                  turn_asym=bool(n.get("turn_asym", mgd.get("turn_asym", False))),
                  settle_steps=int(n.get("settle_steps", mgd.get("settle_steps", 0))),
                  min_swing=float(n.get("min_swing", mgd.get("min_swing", 0.0))),
                  stand_deadzone=float(n.get("stand_deadzone", mgd.get("stand_deadzone", 0.0))),
                  height_scale=float(n.get("height_scale", mgd.get("height_scale", 1.0))),
                  stance_z=float(n.get("stance_z", mgd.get("stance_z", 0.066))),
                  walk_max=float(gd.get("walk_max", lcd.get("walk_max", 1.2))),
                  run_min=float(gd.get("run_min", lcd.get("run_min", 1.7))),
                  settle_eff=float(gd.get("settle_eff", lcd.get("settle_eff", 0.15))),
                  settle_phase=float(gd.get("settle_phase", lcd.get("settle_phase", 0.07))),
                  turn_k=float(lcd.get("turn_k", 0.3)), period_steps=int(lcd.get("period_steps", 43)))
        if de["table"] not in (1, 2):
            de["table"] = 1                                   # C++ 도 없는 판번호는 1 로 되돌린다
        probs, mism = [], []
        keys = ["source", "cadence", "settle_steps", "min_swing", "stand_deadzone", "turn_k"]
        if tr["source"] == "lut" or de["source"] == "lut":
            keys += ["walk_max", "run_min"]
            if any(f.startswith("src/mjlab_g1_motion/tasks/mdp/gait_lut")
                   for f in (infos.get("mode%d" % m) or {}).get("applied_trigger_files") or []):
                probs.append("launch 미커밋 diff 가 발-z 표를 건드렸다 — 표 판별 불가")
            try:
                hit = [tag for tag in ("V1", "V2")
                       if glh._data_only(glh.table_block(tag, glh.load_at(t["commit"]), "")) in hdr_data]
            except Exception as e:                            # noqa: BLE001
                hit = []
                probs.append("그 commit 의 발-z 표를 못 읽음: %s" % repr(e)[:120])
            tr["table"] = int(hit[0][1]) if len(hit) == 1 else None
            if tr["table"] is None and not probs:
                probs.append("그 commit 의 발-z 표가 GaitLut.h 의 V1/V2 어느 것과도 같지 않다")
            r_anc = _git(mjlab, "merge-base", "--is-ancestor", TURN_ASYM_COMMIT, t["commit"])
            tr["turn_asym"] = (tr["source"] == "lut") and r_anc.returncode == 0
            if r_anc.returncode not in (0, 1):
                probs.append("turn_asym 도입 commit(%s) 조상 판정 실패" % TURN_ASYM_COMMIT)
            keys += ["table", "turn_asym"]
        else:
            keys += ["height_scale", "stance_z", "period_steps"]
        if tr["settle_steps"] > 0 or de["settle_steps"] > 0:
            keys += ["settle_eff", "settle_phase"]
        for k in keys:
            if tr.get(k) is None:
                continue
            if not _same(tr[k], de[k]):
                mism.append((k, tr[k], de[k]))
        v, note = judge_mismatches(rid, mism, known, used)
        short = lambda d: " ".join("%s=%s" % (k, _fmt(d.get(k))) for k in keys)  # noqa: E731
        rep.add(G, rid, "mode%d 발-z 생성 조건 (foot_z obs 의 값)" % m,
                "FOOT_GEN@%s: %s" % (_short(t.get("commit")), short(tr)), "deploy gait: %s" % short(de),
                V_FAIL if probs else v, " · ".join(probs + ([note] if note else [])))
    if 3 in head_modes:
        t = truths.get("mode3")
        st4 = ((t or {}).get("stage4") or {}).get("3") or {}
        row3 = rows_mt.get(3) or {}
        reqs = (dep or {}).get("requires") or []
        if t is None or not st4.get("present") or not row3:
            rep.add(G, "gait.mode3_src", "mode3 발-z 원천", "-", "-", V_FAIL, "학습 stage4_mode3 또는 ModeTable 을 못 읽음")
        else:
            t_src = "Gen" if st4.get("has_foot_gen") else "Ref"
            ok_ = (row3.get("foot_z") == t_src) and (t_src != "Ref" or "ref_foot_height_ref" in reqs)
            rep.add(G, "gait.mode3_src", "mode3 발-z 원천",
                    "stage4_mode3 FOOT_GEN %s → %s" % ("있음" if t_src == "Gen" else "없음", "생성기" if t_src == "Gen"
                                                      else "레퍼런스 발 world-z"),
                    "ModeTable mode3 foot_z=%s · requires ref_foot_height_ref %s" % (
                        row3.get("foot_z"), "있음" if "ref_foot_height_ref" in reqs else "없음"),
                    V_PASS if ok_ else V_FAIL, "VR 이 발 높이를 안 줄 때는 두 발 접지(stance)로 대신한다(설계)" if ok_ else
                    "배포가 학습과 다른 원천에서 발-z 를 만든다")
    r = subprocess.run([sys.executable, os.path.join(HERE, "gen_gait_lut_header.py"), "--check"],
                       capture_output=True, text=True, cwd=REPO)
    msg = re.sub(r"^[^0-9A-Za-z가-힣(]+", "", " ".join((r.stdout + r.stderr).strip().splitlines()[:1]))
    rep.add(G, "gait.lut_header", "GaitLut.h 배열 == 학습 gait_lut_data", "gait_lut_data.py (V1=d94c7d9~1 · V2=워킹트리)",
            "GaitLut.h GENERATED 구역", V_PASS if r.returncode == 0 else V_FAIL, "gen_gait_lut_header --check: %s" % msg)
    try:
        g1cfg = yaml.safe_load(_read(CPP_FILES["g1_config"]) or "") or {}
        ic = g1cfg.get("imu_cal") or {}
        vals = (float(ic.get("pitch_deg", 0.0)), float(ic.get("roll_deg", 0.0)))
        rep.add(G, "cfg.imu_cal", "IMU 보정 (projected_gravity 를 바꾼다)", "학습: 보정 없음",
                "config.yaml imu_cal pitch %g° roll %g°" % vals, V_PASS if vals == (0.0, 0.0) else V_WARN,
                "" if vals == (0.0, 0.0) else "실기에서만 걸린다(sim 은 무시) — 의도한 보정인지 확인")
    except Exception as e:                                    # noqa: BLE001
        rep.add(G, "cfg.imu_cal", "IMU 보정", "-", "-", V_FAIL, "config.yaml 을 못 읽음: %s" % e)
    reqs = (dep or {}).get("requires") or []
    kn = cf.get("features_known")
    miss = [x for x in reqs if kn is not None and x not in kn]
    rep.add(G, "cfg.requires", "requires ⊆ 이 소스가 아는 기능", "deploy.yaml requires %d개" % len(reqs),
            "DeployFeatures.h known() %s개" % ("?" if kn is None else len(kn)),
            V_FAIL if (kn is None or miss) else V_PASS,
            ("C++ known() 을 못 읽음" if kn is None else ("모르는 기능: %s" % ", ".join(miss) if miss else "")))
    rep.add(G, "tierb.stamp", "추적 패리티 (sim2sim obs 덤프 vs 학습 함수)", "-", "-", V_SKIP, "Tier B 미도입")

    # ── 허용목록 위생 ──
    if kerr:
        rep.add("5 허용목록", "known.file", "parity_known.yaml", "-", "-", V_FAIL if known is None else V_WARN, kerr)
    for e in (known or []):
        if e["id"] not in used and e["row"] in rep.evaluated:
            rep.add("5 허용목록", "known.%s" % e["id"], "허용목록 %s 가 낡았다" % e["id"],
                    "적힌 학습 %s" % _fmt(e["train"]), "적힌 배포 %s" % _fmt(e["deploy"]), V_WARN,
                    "그 불일치가 지금은 없다(고쳐졌다) — parity_known.yaml 에서 지울 것")
    return _finish(rep, slot_root, meta, commits, t0, mjlab, known=[e for e in (known or []) if e["id"] in used])


def _finish(rep, slot_root, meta, commits, t0, mjlab, known=None):
    dy_p = os.path.join(slot_root, "params", "deploy.yaml")
    onnx_p = os.path.join(slot_root, "exported", "policy.onnx")
    fresh = {"onnx_md5": _hash_file(onnx_p, "md5") if os.path.exists(onnx_p) else None,
             "deploy_yaml_sha256": _hash_file(dy_p, "sha256") if os.path.exists(dy_p) else None}
    ctx = {rel: _hash_file(os.path.join(REPO, rel), "sha256") for rel in CONTEXT_FILES
           if os.path.exists(os.path.join(REPO, rel))}
    if os.path.exists(KNOWN_DEFAULT):                         # 허용목록을 고치면 KNOWN 판정의 근거가 바뀐다
        ctx[os.path.relpath(KNOWN_DEFAULT, REPO)] = _hash_file(KNOWN_DEFAULT, "sha256")
    head = _git(REPO, "rev-parse", "--short", "HEAD").stdout.strip()
    return {
        "tool": "deploy/scripts/check_slot_parity.py", "tool_version": TOOL_VERSION,
        "slot": rep.slot, "checked_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "verdict": rep.verdict(), "counts": rep.counts(),
        "fresh": fresh, "context_sha256": ctx,
        "commits": commits, "mjlab": mjlab, "deploy_repo_head": head,
        "known_used": [{"id": e["id"], "row": e["row"], "key": e["key"], "reason": e.get("reason", ""),
                        "decision": e.get("decision", ""), "where": e.get("where", "")} for e in (known or [])],
        "elapsed_s": round(time.time() - t0, 2),
        "rows": [{k: r[k] for k in ("no", "group", "id", "item", "train", "deploy", "verdict", "note")} for r in rep.rows],
    }


def render_md(rep, brief=False):
    """brief = 화면용 — PASS 행은 빼고 셈만 남긴다(전체 표는 PARITY.md)."""
    def cell(s):
        return str(s).replace("|", "\\|").replace("\n", " ")
    c = rep["counts"]
    order = (V_PASS, V_KNOWN, V_WARN, V_SKIP, V_FAIL)
    L = ["# 패리티 — `%s`" % rep["slot"], "",
         "판정 **%s** (%s) · %s · %s v%s" % (rep["verdict"], " · ".join("%s %d" % (k, c.get(k, 0)) for k in order),
                                          rep["checked_at"], os.path.basename(rep["tool"]), rep["tool_version"]),
         "",
         "- 학습 원장(각 run 의 training_meta.json commit): %s" % (
             " · ".join("%s `%s`" % (k, _short(v)) for k, v in (rep.get("commits") or {}).items()) or "-"),
         "- 배포 (판정 당시): deploy repo `%s` · ONNX md5 `%s` · deploy.yaml sha256 `%s`" % (
             rep["deploy_repo_head"], (rep["fresh"]["onnx_md5"] or "-")[:12],
             (rep["fresh"]["deploy_yaml_sha256"] or "-")[:12]),
         "- 이 표가 유효한 조건: ONNX md5·deploy.yaml sha256 가 위와 같을 것 (policy_slot push · robot.sh verify 가 본다)",
         "", "| # | 항목 | 학습 (원장) | 배포 | 판정 | 비고 |", "|---|---|---|---|---|---|"]
    shown = [r for r in rep["rows"] if not (brief and r["verdict"] == V_PASS)]
    for r in shown:
        L.append("| %s | %s | %s | %s | **%s** | %s |" % (r["no"], cell(r["item"]), cell(r["train"]), cell(r["deploy"]),
                                                       r["verdict"], cell(r["note"])))
    if brief and len(shown) < len(rep["rows"]):
        L.append("| … | PASS %d 행 생략 — 전체는 PARITY.md | | | | |" % (len(rep["rows"]) - len(shown)))
    if rep.get("known_used"):
        L += ["", "## KNOWN — 학습과 다르다는 것을 알고 배포하는 항목 (`parity_known.yaml`)", ""]
        for e in rep["known_used"]:
            L.append("- **%s** `%s` · %s — %s 근거 위치: %s. **%s**" % (
                e["id"], e["row"], e["key"], str(e["reason"]).strip().rstrip(".") + ".", e.get("where", ""),
                str(e.get("decision", "")).strip()))
    L.append("")
    return "\n".join(L)


def write_outputs(slot_root, rep):
    """PARITY.json·PARITY.md. 시각만 다르고 내용이 같으면 다시 쓰지 않는다(재실행마다 git diff 가 생기지 않게)."""
    pj = os.path.join(slot_root, "PARITY.json")
    pm = os.path.join(slot_root, "PARITY.md")
    # 실행마다 달라지는 칸은 빼고 비교한다: 시각·걸린 시간·deploy repo HEAD(커밋할 때마다 바뀐다 — 넣으면
    # «PARITY 커밋 → HEAD 변경 → 다시 쓰기 → 또 커밋» 이 끝없이 돈다). 판정 근거가 바뀐 것만 새로 쓴다.
    strip = lambda d: {k: v for k, v in d.items() if k not in ("checked_at", "elapsed_s", "deploy_repo_head")}  # noqa: E731
    if os.path.exists(pj) and os.path.exists(pm):
        try:
            old = json.load(open(pj, encoding="utf-8"))
            if strip(old) == strip(json.loads(json.dumps(rep, ensure_ascii=False))):
                return False
        except Exception:                                     # noqa: BLE001
            pass
    with open(pj, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
        f.write("\n")
    with open(pm, "w", encoding="utf-8") as f:
        f.write(render_md(rep))
    return True


def _resolve_slot(arg):
    if os.path.isdir(arg):
        return os.path.abspath(arg)
    p = os.path.join(SLOTDIR, arg)
    if os.path.isdir(p):
        return p
    raise SystemExit("그런 슬롯이 없다: %s" % arg)


def _ensure_deps(argv):
    """yaml·onnx 가 없는 파이썬이면 mjlab venv 파이썬으로 다시 뜬다 — 사람이 어느 파이썬으로 불러도 같은 결과."""
    try:
        import yaml  # noqa: F401
        import onnx  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_G1_PARITY_REEXEC"):
        raise SystemExit("yaml·onnx 를 import 할 파이썬이 없다 (mjlab venv 도 안 된다)")
    py = os.path.join(os.environ.get("MJLAB_WS", os.path.expanduser("~/mjlab1.4/mjlab_g1_motion")), ".venv/bin/python")
    if not os.path.exists(py):
        raise SystemExit("yaml·onnx 가 없고 mjlab venv 도 없다: %s" % py)
    env = _clean_env({"_G1_PARITY_REEXEC": "1"})
    env.pop("CUDA_VISIBLE_DEVICES", None)
    os.execve(py, [py, os.path.abspath(__file__)] + argv, env)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "--_truth":                        # 자식 프로세스: 그 commit 의 학습 진실 추출
        _extract_truth(argv[1], argv[2])
        return 0
    _ensure_deps(argv)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("slot", nargs="?", help="슬롯 이름 또는 슬롯 폴더 경로")
    ap.add_argument("--active", action="store_true", help="ACTIVE.yaml 의 별칭 전부")
    ap.add_argument("--known", default=KNOWN_DEFAULT, help="허용목록 (기본 config/policy/parity_known.yaml)")
    ap.add_argument("--no-write", action="store_true", help="PARITY.* 를 쓰지 않는다")
    ap.add_argument("--brief", action="store_true", help="화면에는 PASS 가 아닌 행만 (파일은 전체)")
    a = ap.parse_args(argv)
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import policy_slot
    targets = []
    if a.active:
        act = policy_slot.read_active()
        if not act:
            print("ACTIVE.yaml 이 비었다")
            return 1
        targets = [(al, os.path.join(SLOTDIR, s)) for al, s in sorted(act.items(), key=lambda kv: kv[0])]
    elif a.slot:
        targets = [("", _resolve_slot(a.slot))]
    else:
        ap.print_help()
        return 1
    t_all = time.time()
    summary = []
    for alias, root in targets:
        try:
            rep = check_slot(root, a.known)
        except Exception as e:                                # noqa: BLE001 — 도구가 죽어도 «통과» 로 남지 않게
            import traceback
            traceback.print_exc()
            rr = Report(os.path.basename(root.rstrip("/")))
            rr.add("0 도구", "tool.error", "판정 도구 오류", "-", "-", V_FAIL, repr(e)[:300])
            rep = _finish(rr, root, {}, {}, time.time(), None)
        print(render_md(rep, brief=a.brief))
        wrote = False
        if not a.no_write:
            wrote = write_outputs(root, rep)
        summary.append((alias, rep["slot"], rep["verdict"], rep["counts"], rep["elapsed_s"], wrote))
    print("── 요약 (%.1f s) ──" % (time.time() - t_all))
    for alias, slot, v, c, el, wrote in summary:
        print("  %-4s %-44s %-5s %s  %.1fs%s" % (alias or "-", slot, v,
                                                " ".join("%s %d" % (k, n) for k, n in sorted(c.items())), el,
                                                "" if a.no_write else ("  (PARITY.* 갱신)" if wrote else "  (내용 같음 — 안 씀)")))
    return 1 if any(v == V_FAIL for _, _, v, _, _, _ in summary) else 0


if __name__ == "__main__":
    sys.exit(main())
