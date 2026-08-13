# LAFAN Motion Player Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 학습에 쓴 LAFAN 모션 클립을 배포 정책의 모션 참조로 주입해 sim2sim·실로봇에서 안전하게 재생하는 SSH 터미널 플레이어를 만든다.

**Architecture:** 3겹. ① Resolver = 배포된 정책 슬롯의 `ONNX_META.json` → 매니페스트를 따라가 "이 정책이 학습한 클립 + 유효 모드"를 산출. ② Playlist = 클립 + 구간 + 모드 + 속도를 `PlayItem` 으로 조립. ③ Publisher = 램프인 → 재생 → 램프아웃 상태기계를 50 Hz 절대시각으로 기존 `/dev/shm/g1_vr_ref` 계약에 송출. **배포 C++는 수정하지 않는다.**

**Tech Stack:** Python 3.11 (`/home/piene/unitree_rl_mjlab/.venv`), numpy, PyYAML. 기존 `deploy/robots/g1/teleop/vr_shm.py`, `estop_shm.py` 재사용.

**Spec:** `docs/superpowers/specs/2026-08-13-lafan-motion-player-design.md`

## Global Constraints

- **배포 C++ 무수정.** `deploy/robots/g1/src/`, `include/`, `main.cpp` 를 건드리지 않는다. 재컴파일 없음.
- **기존 `teleop/vr_replay.py` 무수정.** 배선 검증용으로 남긴다.
- **테스트는 stdlib-only self-check 스크립트.** 이 repo의 `.venv` 에 pytest가 없고, 기존 `deploy/robots/g1/teleop/tests/test_estop_shm.py` 가 `main()` + `assert` + `sys.exit(main())` 스타일이다. **이 관례를 따르고 pytest 의존성을 추가하지 않는다.** (spec §14는 "pytest"라 적었으나 repo 관례가 우선 — 검증 내용은 동일.)
- **실행 인터프리터**: `/home/piene/unitree_rl_mjlab/.venv/bin/python`. 모든 테스트 명령은 이 경로를 쓴다.
- **shm 계약 불변**: `/dev/shm/g1_vr_ref`, magic `0x6702`, 276 B, 포맷 `<iIii` + `f`×65. `vr_shm.write()` 를 호출해서 쓰고 직접 struct를 짜지 않는다.
- **`cmd_mode` 는 2 또는 3만.** mode1 재생은 거부한다 (참조가 마스킹돼 정책에 도달하지 않음). mode 4 이상은 C++ `g_poll_vr` 가 거부하므로 보내지 않는다.
- **하드코딩 금지 3종**: `fps` 는 npz의 `fps` 키에서, `standby` pose 는 `deploy.yaml` 의 `default_joint_pos` 에서, 관절한계는 `deploy.yaml` 의 `safety.pos_min/pos_max` 에서 읽는다.
- **base_vel 배포 캡**: `vx ±3.0`, `vy ±1.5`, `wz ±2.0` (C++ `KB_MAXVX/KB_MAXVY/KB_MAXW` 와 동일).
- **커밋 메시지에 AI 흔적 금지.** `Co-Authored-By` 등 트레일러를 붙이지 않는다.
- **작업 전 branch 확인**: `git branch --show-current` 가 `smooth_mode_switch` 인지 확인. 다르면 멈추고 사용자에게 확인.

---

## File Structure

```
deploy/robots/g1/teleop/motion_player/
  __init__.py       빈 패키지 마커
  resolver.py       ONNX_META → manifest → PolicyContext        (Task 1)
  clips.py          npz 로딩 + 프리플라이트 검사                  (Task 2)
  frames.py         참조 프레임 생성 (램프·속도·base_vel) 순수함수  (Task 3)
  publisher.py      상태기계 + 50 Hz 송출 + 중단 경로             (Task 4)
  playlist.py       presets.yaml + CLI 문법 → PlayItem           (Task 5)
  cli.py            화면 + 키 입력 + dry-run                     (Task 6)
  presets.yaml      프리셋 구간 + slot_overrides + motion_root
  tests/
    __init__.py
    test_resolver.py   test_clips.py   test_frames.py
    test_publisher.py  test_playlist.py
```

spec §15 대비 `clips.py` / `frames.py` 를 분리했다. 프리플라이트(파일 I/O + 검증)와 프레임 생성(순수 수학)은 책임이 다르고, 순수 함수를 분리해야 로봇 없이 테스트할 수 있다.

---

## Task 1: Resolver — 정책이 학습한 클립 목록

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/__init__.py`
- Create: `deploy/robots/g1/teleop/motion_player/resolver.py`
- Test: `deploy/robots/g1/teleop/motion_player/tests/__init__.py`
- Test: `deploy/robots/g1/teleop/motion_player/tests/test_resolver.py`

**Interfaces:**
- Consumes: 없음 (첫 태스크)
- Produces:
  - `ClipInfo(name: str, path: Path, modes: tuple[int, ...])`
  - `PolicyContext(slot: str, policy_dir: Path, manifest_path: Path | None, clips: list[ClipInfo], valid_modes: set[int], deployable: bool | None, warnings: list[str])`
  - `strip_manifest_annotation(raw: str) -> str`
  - `load_manifest(path: Path) -> list[ClipInfo]`
  - `resolve(config_path: Path, motion_root: Path, slot_overrides: dict[str, str], fsm_name: str = "Mimic_Masked") -> PolicyContext`

### 배경 (구현자가 알아야 할 것)

`deploy/robots/g1/config/config.yaml` 의 `FSM.Mimic_Masked.policy_dir` 이 현재 배포된 정책 슬롯을 가리킨다 (예: `config/policy/mimic_masked/gmt_multihead_cwc_scratch/`). 그 슬롯 디렉터리의 `ONNX_META.json` 이 어떤 매니페스트로 학습했는지와 모드별 head 유무를 담는다.

**실제 데이터의 함정 두 가지 — 둘 다 테스트로 잠근다:**

1. `gmt_multihead_cwc_scratch/ONNX_META.json` 의 `manifest` 값은
   `"motions/g1_flow_specialists.yaml (24클립, mode3_cop_slip@ab39e4e 시점)"` 이다.
   **뒤에 사람이 붙인 괄호 주석이 있다.**
2. 그 주석을 떼도 `g1_flow_specialists.yaml` 은 **존재하지 않는다**
   (`_deploy24.yaml` / `_full.yaml` / `_v2.yaml` 만 있음).
   → **추측해서 24클립짜리를 고르지 않는다.** 클립 0개 + 경고로 안전 실패하고,
   사람이 `presets.yaml` 의 `slot_overrides` 로 명시하게 한다.

`v2_mode3_steps6/ONNX_META.json` 은 `manifest: "motions/g1_flow_specialists_v2.yaml"` (주석 없음, 파일 존재) 이고 `mode1_ckpt`/`mode2_ckpt` 가 `"none"` 이다 → `valid_modes == {3}`.

매니페스트 스키마:

```yaml
root_path: /home/piene/mjlab1.4/mjlab_g1_motion
motions:
  - file: /home/piene/mjlab1.4/mjlab_g1_motion/COLMOv2/walk/walk1_subject2.npz
    teacher: ...
    weight: 1.0
    modes: [1, 2, 3]
```

`file` 은 절대경로일 수도, `root_path` 기준 상대경로일 수도 있다. `modes` 가 없으면 `(1, 2, 3)` 으로 본다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/motion_player/tests/__init__.py` 를 빈 파일로 만들고,
`deploy/robots/g1/teleop/motion_player/tests/test_resolver.py`:

```python
"""test_resolver.py — Resolver 자체 검증 (stdlib + yaml only, 로봇/시뮬 불필요)."""
import json
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import resolver  # noqa: E402


def test_strip_annotation():
    f = resolver.strip_manifest_annotation
    assert f("motions/a.yaml") == "motions/a.yaml"
    # 실제 gmt_multihead_cwc_scratch 의 값
    assert f("motions/g1_flow_specialists.yaml (24클립, mode3_cop_slip@ab39e4e 시점)") \
        == "motions/g1_flow_specialists.yaml"
    assert f("  motions/b.yaml  ") == "motions/b.yaml"
    print("  ok strip_annotation")


def _write_manifest(root: Path, name: str, entries) -> Path:
    p = root / name
    lines = [f"root_path: {root}", "motions:"]
    for f, modes in entries:
        lines.append(f"  - file: {f}")
        lines.append(f"    modes: {list(modes)}")
    p.write_text("\n".join(lines) + "\n")
    return p


def test_load_manifest_absolute_and_relative():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "walk").mkdir()
        abs_npz = root / "walk" / "walk1_subject1.npz"
        abs_npz.write_bytes(b"")
        rel_npz = root / "walk" / "walk1_subject2.npz"
        rel_npz.write_bytes(b"")
        mf = _write_manifest(root, "m.yaml",
                             [(str(abs_npz), [1, 2, 3]), ("walk/walk1_subject2.npz", [2, 3])])
        clips = resolver.load_manifest(mf)
        assert [c.name for c in clips] == ["walk1_subject1", "walk1_subject2"], clips
        assert clips[0].path == abs_npz
        assert clips[1].path == rel_npz, clips[1].path
        assert clips[0].modes == (1, 2, 3)
        assert clips[1].modes == (2, 3)
    print("  ok load_manifest")


def _slot(root: Path, name: str, meta: dict | None) -> Path:
    d = root / "config" / "policy" / "mimic_masked" / name
    (d / "params").mkdir(parents=True)
    if meta is not None:
        (d / "ONNX_META.json").write_text(json.dumps(meta))
    return d


def _config(root: Path, slot_rel: str) -> Path:
    p = root / "config" / "config.yaml"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("FSM:\n  Mimic_Masked:\n    policy_dir: %s\n" % slot_rel)
    return p


def test_resolve_valid_modes_from_meta():
    """modeN_ckpt == 'none' 인 모드는 valid_modes 에서 빠진다 (v2_mode3_steps6 케이스)."""
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        mroot = root / "mjlab"
        (mroot / "motions").mkdir(parents=True)
        (mroot / "walk").mkdir()
        npz = mroot / "walk" / "walk1_subject1.npz"
        npz.write_bytes(b"")
        _write_manifest(mroot, "motions/v2.yaml", [(str(npz), [1, 2, 3])])
        _slot(root, "v2_mode3_steps6", {
            "manifest": "motions/v2.yaml",
            "mode1_ckpt": "none", "mode2_ckpt": "none", "mode3_ckpt": "x.pt",
            "deployable_on_orin_nx": False,
        })
        cfg = _config(root, "config/policy/mimic_masked/v2_mode3_steps6/")
        ctx = resolver.resolve(cfg, motion_root=mroot, slot_overrides={})
        assert ctx.slot == "v2_mode3_steps6", ctx.slot
        assert ctx.valid_modes == {3}, ctx.valid_modes
        assert ctx.deployable is False
        assert [c.name for c in ctx.clips] == ["walk1_subject1"]
    print("  ok resolve_valid_modes")


def test_resolve_missing_manifest_fails_safe():
    """매니페스트가 존재하지 않으면 클립 0개 + 경고. 절대 추측하지 않는다."""
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        mroot = root / "mjlab"
        (mroot / "motions").mkdir(parents=True)
        (mroot / "motions" / "g1_flow_specialists_deploy24.yaml").write_text("motions: []\n")
        _slot(root, "gmt_multihead_cwc_scratch", {
            "manifest": "motions/g1_flow_specialists.yaml (24클립, mode3_cop_slip@ab39e4e 시점)",
            "mode1_ckpt": "a.pt", "mode2_ckpt": "b.pt", "mode3_ckpt": "c.pt",
        })
        cfg = _config(root, "config/policy/mimic_masked/gmt_multihead_cwc_scratch/")
        ctx = resolver.resolve(cfg, motion_root=mroot, slot_overrides={})
        assert ctx.clips == [], ctx.clips
        assert any("g1_flow_specialists.yaml" in w for w in ctx.warnings), ctx.warnings
    print("  ok missing_manifest_fails_safe")


def test_resolve_slot_override_wins():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        mroot = root / "mjlab"
        (mroot / "motions").mkdir(parents=True)
        (mroot / "walk").mkdir()
        npz = mroot / "walk" / "walk2_subject1.npz"
        npz.write_bytes(b"")
        _write_manifest(mroot, "motions/deploy24.yaml", [(str(npz), [1, 2, 3])])
        _slot(root, "gmt_multihead_cwc_scratch", {
            "manifest": "motions/g1_flow_specialists.yaml (24클립, ...)",
            "mode3_ckpt": "c.pt",
        })
        cfg = _config(root, "config/policy/mimic_masked/gmt_multihead_cwc_scratch/")
        ctx = resolver.resolve(cfg, motion_root=mroot,
                               slot_overrides={"gmt_multihead_cwc_scratch": "motions/deploy24.yaml"})
        assert [c.name for c in ctx.clips] == ["walk2_subject1"], ctx.clips
    print("  ok slot_override")


def test_resolve_no_meta_fails_safe():
    """ONNX_META.json 이 없는 구 슬롯(gmt_multihead_v0)은 안전 실패한다."""
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        mroot = root / "mjlab"
        mroot.mkdir()
        _slot(root, "gmt_multihead_v0", None)
        cfg = _config(root, "config/policy/mimic_masked/gmt_multihead_v0/")
        ctx = resolver.resolve(cfg, motion_root=mroot, slot_overrides={})
        assert ctx.clips == []
        assert ctx.valid_modes == set(), ctx.valid_modes
        assert any("ONNX_META" in w for w in ctx.warnings), ctx.warnings
    print("  ok no_meta_fails_safe")


def test_resolve_missing_npz_dropped_with_warning():
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        mroot = root / "mjlab"
        (mroot / "motions").mkdir(parents=True)
        _write_manifest(mroot, "motions/m.yaml", [(str(mroot / "gone.npz"), [3])])
        _slot(root, "s", {"manifest": "motions/m.yaml", "mode3_ckpt": "c.pt"})
        cfg = _config(root, "config/policy/mimic_masked/s/")
        ctx = resolver.resolve(cfg, motion_root=mroot, slot_overrides={})
        assert ctx.clips == []
        assert any("gone.npz" in w for w in ctx.warnings), ctx.warnings
    print("  ok missing_npz_dropped")


def main() -> int:
    test_strip_annotation()
    test_load_manifest_absolute_and_relative()
    test_resolve_valid_modes_from_meta()
    test_resolve_missing_manifest_fails_safe()
    test_resolve_slot_override_wins()
    test_resolve_no_meta_fails_safe()
    test_resolve_missing_npz_dropped_with_warning()
    print("test_resolver: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_resolver.py
```
Expected: FAIL — `ModuleNotFoundError: No module named 'motion_player'`

- [ ] **Step 3: 최소 구현을 쓴다**

`deploy/robots/g1/teleop/motion_player/__init__.py` 는 빈 파일.

`deploy/robots/g1/teleop/motion_player/resolver.py`:

```python
"""Resolver — 배포된 정책 슬롯이 학습한 클립 목록과 유효 모드를 알아낸다.

config.yaml 의 policy_dir -> <slot>/ONNX_META.json -> manifest YAML 을 따라간다.
새 리타겟 세대(COLMOv2/v3)·새 슬롯·클립 증가가 전부 여기서 흡수되므로,
플레이어의 나머지 코드는 클립 목록을 하드코딩하지 않는다.

안전 원칙: 알아낼 수 없으면 **추측하지 않고 빈 목록 + 경고**로 실패한다.
학습하지 않은 클립을 재생하면 OOD -> 실로봇에서 낙상이다.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# ONNX_META 의 manifest 값 뒤에 사람이 붙인 괄호 주석을 떼기 위한 패턴.
# 실제 예: "motions/g1_flow_specialists.yaml (24클립, mode3_cop_slip@ab39e4e 시점)"
_MANIFEST_ANNOT = re.compile(r"\s*\(.*\)\s*$")


@dataclass
class ClipInfo:
    name: str                    # npz 파일 stem (예: "walk1_subject1")
    path: Path                   # 절대경로로 해석된 npz
    modes: tuple[int, ...]       # 이 클립이 학습된 모드


@dataclass
class PolicyContext:
    slot: str
    policy_dir: Path
    manifest_path: Path | None
    clips: list[ClipInfo]
    valid_modes: set[int]        # ONNX_META 의 modeN_ckpt != "none" 인 N
    deployable: bool | None      # deployable_on_orin_nx (없으면 None)
    warnings: list[str] = field(default_factory=list)


def strip_manifest_annotation(raw: str) -> str:
    """ONNX_META 의 manifest 값에서 경로만 남긴다."""
    return _MANIFEST_ANNOT.sub("", raw.strip())


def load_manifest(path: Path) -> list[ClipInfo]:
    """매니페스트 YAML -> ClipInfo 목록. file 은 절대경로 또는 root_path 기준 상대경로."""
    doc = yaml.safe_load(path.read_text()) or {}
    root = Path(doc.get("root_path", path.parent.parent))
    clips: list[ClipInfo] = []
    for entry in doc.get("motions") or []:
        f = Path(entry["file"])
        if not f.is_absolute():
            f = root / f
        modes = tuple(int(m) for m in entry.get("modes", (1, 2, 3)))
        clips.append(ClipInfo(name=f.stem, path=f, modes=modes))
    return clips


def _read_policy_dir(config_path: Path, fsm_name: str) -> Path:
    doc = yaml.safe_load(config_path.read_text()) or {}
    rel = doc["FSM"][fsm_name]["policy_dir"]
    # config.yaml 의 policy_dir 은 proj_dir(= deploy/robots/g1) 기준 상대경로다.
    return (config_path.parent.parent / rel).resolve()


def resolve(config_path: Path, motion_root: Path,
            slot_overrides: dict[str, str],
            fsm_name: str = "Mimic_Masked") -> PolicyContext:
    """현재 배포 정책 슬롯의 PolicyContext 를 만든다."""
    policy_dir = _read_policy_dir(Path(config_path), fsm_name)
    slot = policy_dir.name
    warnings: list[str] = []

    meta_path = policy_dir / "ONNX_META.json"
    meta: dict = {}
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text())
    else:
        warnings.append(f"{slot}: ONNX_META.json 없음 — 매니페스트/유효모드 미상")

    valid_modes = {n for n in (1, 2, 3)
                   if str(meta.get(f"mode{n}_ckpt", "none")).strip().lower() != "none"}

    raw = slot_overrides.get(slot) or meta.get("manifest")
    manifest_path: Path | None = None
    clips: list[ClipInfo] = []
    if raw:
        rel = strip_manifest_annotation(str(raw))
        cand = Path(rel)
        if not cand.is_absolute():
            cand = Path(motion_root) / cand
        if cand.is_file():
            manifest_path = cand
            for c in load_manifest(cand):
                if c.path.is_file():
                    clips.append(c)
                else:
                    warnings.append(f"클립 파일 없음 — 목록에서 제외: {c.path}")
        else:
            warnings.append(
                f"{slot}: 매니페스트를 찾을 수 없음 ({rel}). "
                f"presets.yaml 의 slot_overrides 에 실제 경로를 지정하세요.")
    elif meta_path.is_file():
        warnings.append(f"{slot}: ONNX_META 에 manifest 항목 없음")

    if meta.get("deployable_on_orin_nx") is False:
        warnings.append(f"{slot}: 실기 배포 불가 슬롯 (deployable_on_orin_nx=false)")

    return PolicyContext(slot=slot, policy_dir=policy_dir, manifest_path=manifest_path,
                         clips=clips, valid_modes=valid_modes,
                         deployable=meta.get("deployable_on_orin_nx"), warnings=warnings)
```

- [ ] **Step 4: 통과하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_resolver.py
```
Expected: PASS — 마지막 줄에 `test_resolver: ALL PASS`

- [ ] **Step 5: 실제 repo에 대고 한 번 돌려본다 (수동 확인)**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python -c "
import sys; sys.path.insert(0, 'deploy/robots/g1/teleop')
from pathlib import Path
from motion_player import resolver
ctx = resolver.resolve(Path('deploy/robots/g1/config/config.yaml'),
                       motion_root=Path('/home/piene/mjlab1.4/mjlab_g1_motion'),
                       slot_overrides={})
print('slot:', ctx.slot, '| clips:', len(ctx.clips), '| valid_modes:', ctx.valid_modes)
for w in ctx.warnings: print('  WARN', w)
"
```
Expected: `slot: gmt_multihead_cwc_scratch | clips: 0 | valid_modes: {1, 2, 3}` +
`WARN ... 매니페스트를 찾을 수 없음 (motions/g1_flow_specialists.yaml)`.
**이게 정상이다** — 안전 실패가 설계대로 동작한 것. Task 5에서 `slot_overrides` 로 채운다.

- [ ] **Step 6: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/__init__.py \
        deploy/robots/g1/teleop/motion_player/resolver.py \
        deploy/robots/g1/teleop/motion_player/tests/
git commit -m "motion_player: 정책 슬롯 -> 학습 클립 목록 resolver

ONNX_META.json 의 manifest 를 따라가 클립+유효모드를 산출한다. 매니페스트를
못 찾거나 ONNX_META 가 없으면 추측하지 않고 빈 목록+경고로 안전 실패한다
(학습 안 한 클립 재생 = OOD = 낙상)."
```

---

## Task 2: 클립 로딩과 프리플라이트

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/clips.py`
- Test: `deploy/robots/g1/teleop/motion_player/tests/test_clips.py`

**Interfaces:**
- Consumes: `resolver.ClipInfo`
- Produces:
  - `DeployProfile(standby: np.ndarray, pos_min: np.ndarray | None, pos_max: np.ndarray | None)`
  - `load_deploy_profile(policy_dir: Path) -> DeployProfile`
  - `ClipData(name, fps, n_frames, duration, joint_pos, joint_vel, root_quat, pelvis_lin_vel_w, pelvis_ang_vel_w)`
  - `load_clip(info: ClipInfo) -> ClipData`
  - `PreflightResult(ok: bool, reasons: list[str], f_start: int, f_end: int, entry_err: float, entry_joint: int, ramp_in_s: float)`
  - `preflight(clip: ClipData, span: tuple[float, float], mode: int, allowed_modes: tuple[int, ...], profile: DeployProfile) -> PreflightResult`
  - `RAMP_RATE_RAD_S = 0.35`, `RAMP_IN_MIN_S = 1.5`

### 배경

npz 키: `joint_pos[T,29] f32`, `joint_vel[T,29] f32`, `body_pos_w[T,B,3]`, `body_quat_w[T,B,4] wxyz`, `body_lin_vel_w[T,B,3]`, `body_ang_vel_w[T,B,3]`, `fps[1] f64`. **body 0 = pelvis.**

`deploy.yaml` 에서 읽을 것: `default_joint_pos`(29) = standby, `safety.pos_min`/`safety.pos_max`(각 29). `safety` 블록이 없으면 `actions.JointPositionAction.clip`(29×2)로 폴백하고, 그것도 없으면 한계 검사를 건너뛰되 경고한다.

**램프인 시간 산정**: `entry_err = max|clip.joint_pos[f_start] − standby|`,
`ramp_in_s = max(RAMP_IN_MIN_S, entry_err / RAMP_RATE_RAD_S)`.
실측 entry_err 는 0.40~0.64 rad → 1.5~1.9 s 가 나온다. C++ 크로스페이드(1.0 s)보다 항상 길다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/motion_player/tests/test_clips.py`:

```python
"""test_clips.py — 클립 로딩/프리플라이트 자체 검증 (합성 npz, 로봇 불필요)."""
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import clips, resolver  # noqa: E402

STANDBY = np.zeros(29, dtype=np.float32)


def _make_npz(path: Path, T=100, fps=50.0, offset=0.5, nan_at=None):
    jp = np.full((T, 29), offset, dtype=np.float32)
    jv = np.full((T, 29), 0.1, dtype=np.float32)
    if nan_at is not None:
        jp[nan_at, 3] = np.nan
    quat = np.zeros((T, 30, 4), dtype=np.float32)
    quat[:, :, 0] = 1.0
    np.savez(path, joint_pos=jp, joint_vel=jv,
             body_pos_w=np.zeros((T, 30, 3), dtype=np.float32),
             body_quat_w=quat,
             body_lin_vel_w=np.full((T, 30, 3), 0.2, dtype=np.float32),
             body_ang_vel_w=np.full((T, 30, 3), 0.3, dtype=np.float32),
             fps=np.array([fps]))


def _profile(lo=-2.0, hi=2.0):
    return clips.DeployProfile(standby=STANDBY.copy(),
                               pos_min=np.full(29, lo, dtype=np.float32),
                               pos_max=np.full(29, hi, dtype=np.float32))


def _load(d: Path, **kw) -> clips.ClipData:
    p = d / "walk1_subject1.npz"
    _make_npz(p, **kw)
    return clips.load_clip(resolver.ClipInfo(name="walk1_subject1", path=p, modes=(1, 2, 3)))


def test_load_clip_reads_fps_and_pelvis():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=120, fps=30.0)
        assert c.fps == 30.0, c.fps          # 50 하드코딩 금지
        assert c.n_frames == 120
        assert abs(c.duration - 4.0) < 1e-6, c.duration
        assert c.joint_pos.shape == (120, 29)
        assert c.pelvis_lin_vel_w.shape == (120, 3)   # body 0 만 뽑아 옴
        assert c.root_quat.shape == (120, 4)
    print("  ok load_clip")


def test_preflight_ok_and_ramp_time():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.7)       # entry_err = 0.7 rad
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert r.ok, r.reasons
        assert r.f_start == 0 and r.f_end == 50, (r.f_start, r.f_end)
        assert abs(r.entry_err - 0.7) < 1e-5, r.entry_err
        assert abs(r.ramp_in_s - 2.0) < 1e-5, r.ramp_in_s   # 0.7/0.35 = 2.0 > 1.5
    print("  ok preflight_ok")


def test_preflight_ramp_floor():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.1)       # 0.1/0.35 = 0.29 -> 바닥값 1.5 로
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert abs(r.ramp_in_s - 1.5) < 1e-9, r.ramp_in_s
    print("  ok ramp_floor")


def test_preflight_rejects_mode1():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d))
        r = clips.preflight(c, (0.0, 1.0), mode=1, allowed_modes=(1, 2, 3), profile=_profile())
        assert not r.ok
        assert any("mode1" in s for s in r.reasons), r.reasons
    print("  ok reject_mode1")


def test_preflight_rejects_mode_not_allowed():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d))
        r = clips.preflight(c, (0.0, 1.0), mode=2, allowed_modes=(3,), profile=_profile())
        assert not r.ok
        assert any("mode2" in s for s in r.reasons), r.reasons
    print("  ok reject_mode_not_allowed")


def test_preflight_rejects_bad_span():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=100, fps=50.0)      # duration = 2.0 s
        for span in [(-1.0, 1.0), (0.0, 5.0), (1.5, 1.5), (1.5, 0.5)]:
            r = clips.preflight(c, span, mode=3, allowed_modes=(1, 2, 3), profile=_profile())
            assert not r.ok, span
            assert any("구간" in s for s in r.reasons), (span, r.reasons)
    print("  ok reject_bad_span")


def test_preflight_rejects_nan_in_span():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), T=100, nan_at=60)
        r = clips.preflight(c, (0.0, 0.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert r.ok, r.reasons                       # 0~25 프레임에는 NaN 없음
        r = clips.preflight(c, (1.0, 1.5), mode=3, allowed_modes=(1, 2, 3), profile=_profile())
        assert not r.ok                              # 50~75 프레임에 NaN 있음
        assert any("NaN" in s for s in r.reasons), r.reasons
    print("  ok reject_nan")


def test_preflight_rejects_joint_limit():
    with tempfile.TemporaryDirectory() as d:
        c = _load(Path(d), offset=0.5)
        r = clips.preflight(c, (0.0, 1.0), mode=3, allowed_modes=(1, 2, 3),
                            profile=_profile(lo=-0.2, hi=0.2))
        assert not r.ok
        assert any("관절한계" in s for s in r.reasons), r.reasons
    print("  ok reject_joint_limit")


def test_load_deploy_profile_from_real_yaml():
    """실제 배포 yaml 을 읽어 standby/한계가 29개로 나오는지."""
    here = Path(__file__).resolve().parents[4]   # -> deploy/robots/g1
    slot = here / "config" / "policy" / "mimic_masked" / "gmt_multihead_cwc_scratch"
    if not (slot / "params" / "deploy.yaml").is_file():
        print("  skip load_deploy_profile (슬롯 없음)")
        return
    p = clips.load_deploy_profile(slot)
    assert p.standby.shape == (29,), p.standby.shape
    assert abs(float(p.standby[3]) - 0.669) < 1e-6, p.standby[3]   # KNEES_BENT 무릎
    assert p.pos_min is not None and p.pos_min.shape == (29,)
    print("  ok load_deploy_profile")


def main() -> int:
    test_load_clip_reads_fps_and_pelvis()
    test_preflight_ok_and_ramp_time()
    test_preflight_ramp_floor()
    test_preflight_rejects_mode1()
    test_preflight_rejects_mode_not_allowed()
    test_preflight_rejects_bad_span()
    test_preflight_rejects_nan_in_span()
    test_preflight_rejects_joint_limit()
    test_load_deploy_profile_from_real_yaml()
    print("test_clips: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_clips.py
```
Expected: FAIL — `ImportError: cannot import name 'clips'`

- [ ] **Step 3: 최소 구현을 쓴다**

`deploy/robots/g1/teleop/motion_player/clips.py`:

```python
"""클립 npz 로딩 + 재생 전 프리플라이트 검사.

프리플라이트는 "이 구간을 이 모드로 재생해도 되는가"를 판정한다. 하나라도 걸리면
재생하지 않는다 — 실로봇에서 되돌릴 수 없는 것은 낙상뿐이다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .resolver import ClipInfo

# 램프인 속도 상한과 바닥값. 실측 entry_err 0.40~0.64 rad -> 1.5~1.9 s.
# C++ 모드전환 크로스페이드(switch_blend_steps: 50 = 1.0 s)보다 항상 길어야 한다.
RAMP_RATE_RAD_S = 0.35
RAMP_IN_MIN_S = 1.5


@dataclass
class DeployProfile:
    standby: np.ndarray                  # [29] deploy.yaml default_joint_pos
    pos_min: np.ndarray | None           # [29] 또는 None(검사 생략)
    pos_max: np.ndarray | None


@dataclass
class ClipData:
    name: str
    fps: float
    n_frames: int
    duration: float
    joint_pos: np.ndarray                # [T,29]
    joint_vel: np.ndarray                # [T,29]
    root_quat: np.ndarray                # [T,4] pelvis wxyz
    pelvis_lin_vel_w: np.ndarray         # [T,3]
    pelvis_ang_vel_w: np.ndarray         # [T,3]


@dataclass
class PreflightResult:
    ok: bool
    reasons: list[str] = field(default_factory=list)
    f_start: int = 0
    f_end: int = 0
    entry_err: float = 0.0
    entry_joint: int = -1
    ramp_in_s: float = RAMP_IN_MIN_S


def load_deploy_profile(policy_dir: Path) -> DeployProfile:
    """deploy.yaml 에서 standby pose 와 관절한계를 읽는다. 하드코딩 금지."""
    doc = yaml.safe_load((Path(policy_dir) / "params" / "deploy.yaml").read_text())
    standby = np.asarray(doc["default_joint_pos"], dtype=np.float32)
    lo = hi = None
    safety = doc.get("safety") or {}
    if isinstance(safety.get("pos_min"), list) and isinstance(safety.get("pos_max"), list):
        lo = np.asarray(safety["pos_min"], dtype=np.float32)
        hi = np.asarray(safety["pos_max"], dtype=np.float32)
    else:
        clip = ((doc.get("actions") or {}).get("JointPositionAction") or {}).get("clip")
        if isinstance(clip, list) and len(clip) == len(standby):
            arr = np.asarray(clip, dtype=np.float32)
            lo, hi = arr[:, 0], arr[:, 1]
    if lo is None or lo.shape != standby.shape:
        lo = hi = None
    return DeployProfile(standby=standby, pos_min=lo, pos_max=hi)


def load_clip(info: ClipInfo) -> ClipData:
    """npz -> ClipData. fps 는 파일에서 읽는다(50 하드코딩 금지). pelvis = body 0."""
    d = np.load(info.path)
    jp = np.asarray(d["joint_pos"], dtype=np.float32)
    jv = np.asarray(d["joint_vel"], dtype=np.float32)
    quat = np.asarray(d["body_quat_w"][:, 0], dtype=np.float32)      # pelvis wxyz
    lin = np.asarray(d["body_lin_vel_w"][:, 0], dtype=np.float32)
    ang = np.asarray(d["body_ang_vel_w"][:, 0], dtype=np.float32)
    fps = float(np.asarray(d["fps"]).reshape(-1)[0]) if "fps" in d else 50.0
    n = int(len(jp))
    return ClipData(name=info.name, fps=fps, n_frames=n, duration=n / fps,
                    joint_pos=jp, joint_vel=jv, root_quat=quat,
                    pelvis_lin_vel_w=lin, pelvis_ang_vel_w=ang)


def preflight(clip: ClipData, span: tuple[float, float], mode: int,
              allowed_modes: tuple[int, ...], profile: DeployProfile) -> PreflightResult:
    """재생 전 전수 검사. 통과해야만 송출한다."""
    reasons: list[str] = []

    if mode == 1:
        reasons.append("mode1 재생 불가 — 참조가 상·하체 전부 마스킹돼 정책에 도달하지 않는다 "
                       "(masked_joint_command/masked_root_ori_b 가 0). mode2 또는 mode3 을 쓰세요.")
    elif mode not in (2, 3):
        reasons.append(f"mode{mode} 는 VR 채널로 보낼 수 없다 (g_poll_vr 이 1~3만 수용)")
    elif mode not in allowed_modes:
        reasons.append(f"mode{mode} 는 이 클립/정책에서 유효하지 않다 (유효: {sorted(allowed_modes)})")

    t0, t1 = float(span[0]), float(span[1])
    span_ok = (0.0 <= t0 < t1 <= clip.duration)
    if not span_ok:
        reasons.append(f"구간이 잘못됨: [{t0:.2f}, {t1:.2f}] — 클립 길이 {clip.duration:.2f}s")
        return PreflightResult(ok=False, reasons=reasons)

    f_start = int(round(t0 * clip.fps))
    f_end = min(int(round(t1 * clip.fps)), clip.n_frames)
    if f_end - f_start < 2:
        reasons.append(f"구간이 너무 짧음: {f_end - f_start} 프레임")
        return PreflightResult(ok=False, reasons=reasons)

    seg_p = clip.joint_pos[f_start:f_end]
    seg_v = clip.joint_vel[f_start:f_end]
    if not (np.isfinite(seg_p).all() and np.isfinite(seg_v).all()
            and np.isfinite(clip.root_quat[f_start:f_end]).all()):
        reasons.append("구간에 NaN/Inf 가 있다")

    if profile.pos_min is not None:
        under = seg_p < profile.pos_min
        over = seg_p > profile.pos_max
        if under.any() or over.any():
            j = int(np.argmax((under | over).any(axis=0)))
            reasons.append(f"관절한계 위반: joint[{j}] "
                           f"(범위 [{profile.pos_min[j]:.3f}, {profile.pos_max[j]:.3f}])")

    diff = np.abs(clip.joint_pos[f_start] - profile.standby)
    entry_joint = int(np.argmax(diff))
    entry_err = float(diff[entry_joint])
    ramp_in_s = max(RAMP_IN_MIN_S, entry_err / RAMP_RATE_RAD_S)

    return PreflightResult(ok=not reasons, reasons=reasons, f_start=f_start, f_end=f_end,
                           entry_err=entry_err, entry_joint=entry_joint, ramp_in_s=ramp_in_s)
```

- [ ] **Step 4: 통과하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_clips.py
```
Expected: PASS — `test_clips: ALL PASS`

- [ ] **Step 5: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/clips.py \
        deploy/robots/g1/teleop/motion_player/tests/test_clips.py
git commit -m "motion_player: 클립 로딩 + 재생 전 프리플라이트

fps/standby/관절한계를 전부 파일에서 읽는다(하드코딩 금지). mode1 요청,
범위 밖 구간, NaN, 관절한계 위반을 거부하고 진입 자세차로 램프인 시간을 산정한다."
```

---

## Task 3: 참조 프레임 생성 (순수 함수)

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/frames.py`
- Test: `deploy/robots/g1/teleop/motion_player/tests/test_frames.py`

**Interfaces:**
- Consumes: `clips.ClipData`, `clips.DeployProfile`
- Produces:
  - `RefFrame(cmd_mode: int, valid: int, base_vel: tuple[float,float,float], root_quat: tuple[float,float,float,float], dof_pos: np.ndarray, dof_vel: np.ndarray)`
  - `smoothstep(s: float) -> float`
  - `quat_slerp(a, b, s) -> np.ndarray`
  - `yaw_local_base_vel(quat_wxyz, lin_vel_w, ang_vel_w) -> tuple[float,float,float]`
  - `clamp_base_vel(bv) -> tuple[float,float,float]`
  - `ramp_frame(clip, profile, f_anchor, s, mode, base_vel, direction) -> RefFrame`
  - `play_frame(clip, f_idx, speed, mode, base_vel_kind, manual_bv) -> RefFrame`
  - `CAP_VX = 3.0`, `CAP_VY = 1.5`, `CAP_WZ = 2.0`

### 배경

- **RAMP_IN**: `s: 0→1`. `dof_pos = standby + smoothstep(s)·(clip[f₀] − standby)`,
  `dof_vel = s·clip_vel[f₀]`, `root_quat = slerp(identity, clip_quat[f₀], s)`.
- **RAMP_OUT**: 같은 함수를 `direction="out"` 으로. `s: 0→1` 이면 clip → standby.
- **속도 스케일**: `speed` 를 바꾸면 **`dof_vel` 도 같은 배율로 곱한다.** 안 그러면 위치는
  느린데 속도 피드포워드만 빨라 참조가 모순된다.
- **`base_vel` from clip**: pelvis 세계좌표 속도를 yaw-local로 회전.
  `yaw = atan2(2(wz+xy), 1−2(y²+z²))`, `vx = cos·vwx + sin·vwy`, `vy = −sin·vwx + cos·vwy`, `wz = ω_w[2]`.
- mode3 에서는 C++ `MaskedLocoController` 가 `base_vel` 을 0으로 덮으므로 무엇을 보내도 무해하지만,
  **혼란을 막기 위해 mode3 이면 `(0,0,0)` 을 보낸다.**

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/motion_player/tests/test_frames.py`:

```python
"""test_frames.py — 참조 프레임 생성 순수함수 검증 (로봇/파일 불필요)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import clips, frames  # noqa: E402


def _clip(T=100, fps=50.0, pos=0.8, vel=1.2):
    quat = np.zeros((T, 4), dtype=np.float32)
    quat[:, 0] = 1.0
    return clips.ClipData(
        name="c", fps=fps, n_frames=T, duration=T / fps,
        joint_pos=np.full((T, 29), pos, dtype=np.float32),
        joint_vel=np.full((T, 29), vel, dtype=np.float32),
        root_quat=quat,
        pelvis_lin_vel_w=np.zeros((T, 3), dtype=np.float32),
        pelvis_ang_vel_w=np.zeros((T, 3), dtype=np.float32))


def _profile(standby=0.0):
    return clips.DeployProfile(standby=np.full(29, standby, dtype=np.float32),
                               pos_min=None, pos_max=None)


def test_smoothstep_endpoints():
    assert frames.smoothstep(0.0) == 0.0
    assert frames.smoothstep(1.0) == 1.0
    assert abs(frames.smoothstep(0.5) - 0.5) < 1e-9
    assert frames.smoothstep(-3.0) == 0.0 and frames.smoothstep(9.0) == 1.0  # clamp
    print("  ok smoothstep")


def test_ramp_in_endpoints_are_continuous():
    """s=0 이면 정확히 standby, s=1 이면 정확히 clip[f0]. 경계에서 점프가 없어야 한다."""
    c, p = _clip(pos=0.8, vel=1.2), _profile(standby=0.0)
    a = frames.ramp_frame(c, p, f_anchor=0, s=0.0, mode=3, base_vel=(0, 0, 0), direction="in")
    b = frames.ramp_frame(c, p, f_anchor=0, s=1.0, mode=3, base_vel=(0, 0, 0), direction="in")
    assert np.allclose(a.dof_pos, 0.0), a.dof_pos[:3]
    assert np.allclose(a.dof_vel, 0.0), a.dof_vel[:3]
    assert np.allclose(b.dof_pos, 0.8), b.dof_pos[:3]
    assert np.allclose(b.dof_vel, 1.2), b.dof_vel[:3]
    print("  ok ramp_in_endpoints")


def test_ramp_out_endpoints():
    c, p = _clip(pos=0.8, vel=1.2), _profile(standby=0.0)
    a = frames.ramp_frame(c, p, f_anchor=5, s=0.0, mode=3, base_vel=(0, 0, 0), direction="out")
    b = frames.ramp_frame(c, p, f_anchor=5, s=1.0, mode=3, base_vel=(0, 0, 0), direction="out")
    assert np.allclose(a.dof_pos, 0.8) and np.allclose(a.dof_vel, 1.2)
    assert np.allclose(b.dof_pos, 0.0) and np.allclose(b.dof_vel, 0.0)
    print("  ok ramp_out_endpoints")


def test_ramp_is_monotonic():
    c, p = _clip(pos=1.0), _profile(standby=0.0)
    prev = -1.0
    for i in range(21):
        f = frames.ramp_frame(c, p, 0, i / 20.0, 3, (0, 0, 0), "in")
        cur = float(f.dof_pos[0])
        assert cur >= prev - 1e-9, (i, cur, prev)
        prev = cur
    print("  ok ramp_monotonic")


def test_speed_scales_velocity():
    """속도를 늦추면 qd_ref 도 같은 배율로 늦춰져야 한다."""
    c = _clip(vel=2.0)
    full = frames.play_frame(c, 10, speed=1.0, mode=3, base_vel_kind="zero", manual_bv=(0, 0, 0))
    half = frames.play_frame(c, 10, speed=0.5, mode=3, base_vel_kind="zero", manual_bv=(0, 0, 0))
    assert np.allclose(full.dof_vel, 2.0)
    assert np.allclose(half.dof_vel, 1.0), half.dof_vel[:3]
    assert np.allclose(full.dof_pos, half.dof_pos)      # 위치는 프레임이 같으면 같다
    print("  ok speed_scales_velocity")


def test_yaw_local_base_vel_identity():
    """yaw=0 이면 세계좌표 그대로."""
    vx, vy, wz = frames.yaw_local_base_vel((1.0, 0.0, 0.0, 0.0), (1.0, 2.0, 0.0), (0.0, 0.0, 0.5))
    assert abs(vx - 1.0) < 1e-6 and abs(vy - 2.0) < 1e-6 and abs(wz - 0.5) < 1e-6
    print("  ok base_vel_identity")


def test_yaw_local_base_vel_90deg():
    """yaw=+90deg 로봇이 세계 +x 로 1 m/s 이동 -> 몸 기준 vy = -1."""
    h = math.sqrt(0.5)
    q = (h, 0.0, 0.0, h)                    # wxyz, yaw = +pi/2
    vx, vy, wz = frames.yaw_local_base_vel(q, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert abs(vx - 0.0) < 1e-6, vx
    assert abs(vy + 1.0) < 1e-6, vy
    print("  ok base_vel_90deg")


def test_clamp_base_vel():
    assert frames.clamp_base_vel((99.0, -99.0, 99.0)) == (3.0, -1.5, 2.0)
    print("  ok clamp_base_vel")


def test_mode3_zeroes_base_vel():
    """mode3 은 C++ 가 어차피 0으로 덮는다. 혼란 방지를 위해 우리도 0을 보낸다."""
    c = _clip()
    f = frames.play_frame(c, 0, speed=1.0, mode=3, base_vel_kind="manual", manual_bv=(1.0, 1.0, 1.0))
    assert f.base_vel == (0.0, 0.0, 0.0), f.base_vel
    f2 = frames.play_frame(c, 0, speed=1.0, mode=2, base_vel_kind="manual", manual_bv=(1.0, 1.0, 1.0))
    assert f2.base_vel == (1.0, 1.0, 1.0), f2.base_vel
    print("  ok mode3_zeroes_base_vel")


def test_quat_slerp_endpoints():
    a = np.array([1.0, 0.0, 0.0, 0.0])
    b = np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
    assert np.allclose(frames.quat_slerp(a, b, 0.0), a)
    assert np.allclose(frames.quat_slerp(a, b, 1.0), b)
    mid = frames.quat_slerp(a, b, 0.5)
    assert abs(np.linalg.norm(mid) - 1.0) < 1e-6, mid
    print("  ok quat_slerp")


def main() -> int:
    test_smoothstep_endpoints()
    test_ramp_in_endpoints_are_continuous()
    test_ramp_out_endpoints()
    test_ramp_is_monotonic()
    test_speed_scales_velocity()
    test_yaw_local_base_vel_identity()
    test_yaw_local_base_vel_90deg()
    test_clamp_base_vel()
    test_mode3_zeroes_base_vel()
    test_quat_slerp_endpoints()
    print("test_frames: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_frames.py
```
Expected: FAIL — `ImportError: cannot import name 'frames'`

- [ ] **Step 3: 최소 구현을 쓴다**

`deploy/robots/g1/teleop/motion_player/frames.py`:

```python
"""참조 프레임 생성 — 순수 함수만. 파일도 shm도 시간도 건드리지 않는다.

여기서 만든 RefFrame 을 publisher 가 vr_shm.write() 로 내보낸다.
순수하게 유지하는 이유: 로봇/시뮬 없이 램프 연속성·속도 스케일·좌표변환을 테스트하기 위해서다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .clips import ClipData, DeployProfile

# 배포 base_vel 캡 — C++ KB_MAXVX / KB_MAXVY / KB_MAXW 와 동일해야 한다.
CAP_VX = 3.0
CAP_VY = 1.5
CAP_WZ = 2.0

_IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class RefFrame:
    cmd_mode: int
    valid: int
    base_vel: tuple[float, float, float]
    root_quat: tuple[float, float, float, float]     # wxyz
    dof_pos: np.ndarray                              # [29]
    dof_vel: np.ndarray                              # [29]


def smoothstep(s: float) -> float:
    """3s^2 - 2s^3. 양 끝에서 1차 도함수가 0이라 진입/이탈이 부드럽다."""
    s = min(1.0, max(0.0, float(s)))
    return s * s * (3.0 - 2.0 * s)


def quat_slerp(a, b, s: float) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    s = min(1.0, max(0.0, float(s)))
    dot = float(np.dot(a, b))
    if dot < 0.0:                 # 최단 경로
        b, dot = -b, -dot
    if dot > 0.9995:              # 거의 같으면 선형보간 후 정규화
        out = a + s * (b - a)
        return out / np.linalg.norm(out)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_t = math.sin(theta)
    return (math.sin((1.0 - s) * theta) / sin_t) * a + (math.sin(s * theta) / sin_t) * b


def yaw_local_base_vel(quat_wxyz, lin_vel_w, ang_vel_w) -> tuple[float, float, float]:
    """pelvis 세계좌표 속도 -> yaw-local [vx, vy, wz].

    학습 base_vel 분포가 "clip pelvis velocity" 라 이 값이 in-distribution 이다
    (deploy/robots/g1/src/State_Mimic.cpp:187-188 주석).
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    vwx, vwy = float(lin_vel_w[0]), float(lin_vel_w[1])
    return (c * vwx + s * vwy, -s * vwx + c * vwy, float(ang_vel_w[2]))


def clamp_base_vel(bv) -> tuple[float, float, float]:
    return (min(CAP_VX, max(-CAP_VX, float(bv[0]))),
            min(CAP_VY, max(-CAP_VY, float(bv[1]))),
            min(CAP_WZ, max(-CAP_WZ, float(bv[2]))))


def _resolve_base_vel(clip: ClipData, f_idx: int, mode: int,
                      kind: str, manual_bv) -> tuple[float, float, float]:
    # mode3 은 C++ MaskedLocoController 가 base_vel 을 0 으로 덮는다(cmd_mode>=3).
    # 혼란을 막기 위해 우리도 0 을 보낸다.
    if mode >= 3:
        return (0.0, 0.0, 0.0)
    if kind == "clip":
        return clamp_base_vel(yaw_local_base_vel(clip.root_quat[f_idx],
                                                 clip.pelvis_lin_vel_w[f_idx],
                                                 clip.pelvis_ang_vel_w[f_idx]))
    if kind == "manual":
        return clamp_base_vel(manual_bv)
    return (0.0, 0.0, 0.0)


def ramp_frame(clip: ClipData, profile: DeployProfile, f_anchor: int, s: float,
               mode: int, base_vel, direction: str) -> RefFrame:
    """standby <-> clip[f_anchor] 사이를 smoothstep 으로 잇는다.

    direction="in":  s=0 -> standby, s=1 -> clip[f_anchor]
    direction="out": s=0 -> clip[f_anchor], s=1 -> standby
    """
    a = smoothstep(s)
    if direction == "out":
        a = 1.0 - a
    target_p = clip.joint_pos[f_anchor].astype(np.float64)
    target_v = clip.joint_vel[f_anchor].astype(np.float64)
    standby = profile.standby.astype(np.float64)
    dof_pos = standby + a * (target_p - standby)
    dof_vel = a * target_v
    quat = quat_slerp(_IDENTITY_QUAT, clip.root_quat[f_anchor].astype(np.float64), a)
    return RefFrame(cmd_mode=mode, valid=1,
                    base_vel=clamp_base_vel(base_vel) if mode < 3 else (0.0, 0.0, 0.0),
                    root_quat=tuple(float(v) for v in quat),
                    dof_pos=dof_pos.astype(np.float32),
                    dof_vel=dof_vel.astype(np.float32))


def play_frame(clip: ClipData, f_idx: int, speed: float, mode: int,
               base_vel_kind: str, manual_bv) -> RefFrame:
    """재생 중 한 프레임. ⚠ speed 를 바꾸면 dof_vel 도 같은 배율로 곱한다."""
    f_idx = int(min(max(f_idx, 0), clip.n_frames - 1))
    bv = _resolve_base_vel(clip, f_idx, mode, base_vel_kind, manual_bv)
    return RefFrame(cmd_mode=mode, valid=1, base_vel=bv,
                    root_quat=tuple(float(v) for v in clip.root_quat[f_idx]),
                    dof_pos=clip.joint_pos[f_idx].astype(np.float32),
                    dof_vel=(clip.joint_vel[f_idx] * float(speed)).astype(np.float32))
```

- [ ] **Step 4: 통과하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_frames.py
```
Expected: PASS — `test_frames: ALL PASS`

- [ ] **Step 5: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/frames.py \
        deploy/robots/g1/teleop/motion_player/tests/test_frames.py
git commit -m "motion_player: 참조 프레임 생성 순수함수

smoothstep 램프(양 끝 도함수 0), quat slerp, 클립 pelvis 속도의 yaw-local 변환,
배포 캡 clamp. speed 변경 시 dof_vel 도 같이 스케일해 위치/속도 참조 모순을 막는다."
```

---

## Task 4: Publisher — 상태기계와 송출

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/publisher.py`
- Test: `deploy/robots/g1/teleop/motion_player/tests/test_publisher.py`

**Interfaces:**
- Consumes: `frames.RefFrame`, `clips.ClipData`, `clips.DeployProfile`, `clips.PreflightResult`
- Produces:
  - `PlaybackSpec(clip, profile, f_start, f_end, mode, speed, base_vel_kind, manual_bv, ramp_in_s, ramp_out_s)`
  - `RAMP_OUT_S = 1.5`, `ABORT_RAMP_OUT_S = 0.8`, `RELEASE_HOLD_S = 0.5`
  - `plan_frames(spec, abort_at: float | None = None) -> Iterator[tuple[float, RefFrame]]`
    — (재생 시작 후 경과 벽시계 초, 프레임). 시간을 읽지 않는 **순수 생성기**.
  - `Publisher(writer=vr_shm.write)` with `run(spec, on_tick=None, should_abort=None) -> str`

### 배경

**왜 순수 생성기(`plan_frames`)와 실행기(`Publisher.run`)를 나누나**: 시간·shm 을 만지는 코드와
"무엇을 언제 보낼지" 를 분리해야 로봇 없이 전체 시퀀스를 검증할 수 있다.

**RELEASE 가 중요한 이유**: C++ 의 heading 재앵커와 1 초 크로스페이드는 **모드가 바뀔 때만**
발동한다 (`State_Mimic.cpp:596-612`). 그래서 재생이 끝나면 `cmd_mode=1` 패킷을 보내 모드 전환을
일으키고, 0.5 초 유지 후 `valid=0` 을 보낸 뒤 shm 파일을 지운다. 이렇게 해야 다음 재생이
`mode1 → mode2/3` 전환으로 시작해 재앵커를 다시 받는다.

**`valid=0` 만 보내고 끝내면 안 되는 이유**: C++ `clear_vr()` 은 q_ref 를 standby 로 **계단
스냅**한다 (`include/State_Mimic.h:156-163`). 그래서 RAMP_OUT 으로 이미 standby 에 도달한
뒤에만 `valid=0` 을 보낸다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/motion_player/tests/test_publisher.py`:

```python
"""test_publisher.py — 송출 시퀀스/중단경로 검증 (shm 대신 가짜 writer)."""
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import vr_shm  # noqa: E402
from motion_player import clips, publisher  # noqa: E402


def _spec(speed=1.0, mode=3, ramp_in=1.0, ramp_out=1.0, T=200, fps=50.0):
    quat = np.zeros((T, 4), dtype=np.float32)
    quat[:, 0] = 1.0
    clip = clips.ClipData(name="c", fps=fps, n_frames=T, duration=T / fps,
                          joint_pos=np.full((T, 29), 0.8, dtype=np.float32),
                          joint_vel=np.full((T, 29), 1.0, dtype=np.float32),
                          root_quat=quat,
                          pelvis_lin_vel_w=np.zeros((T, 3), dtype=np.float32),
                          pelvis_ang_vel_w=np.zeros((T, 3), dtype=np.float32))
    profile = clips.DeployProfile(standby=np.zeros(29, dtype=np.float32),
                                  pos_min=None, pos_max=None)
    return publisher.PlaybackSpec(clip=clip, profile=profile, f_start=0, f_end=100,
                                  mode=mode, speed=speed, base_vel_kind="zero",
                                  manual_bv=(0.0, 0.0, 0.0),
                                  ramp_in_s=ramp_in, ramp_out_s=ramp_out)


def test_plan_starts_at_standby_and_ends_at_standby():
    seq = list(publisher.plan_frames(_spec()))
    assert np.allclose(seq[0][1].dof_pos, 0.0), seq[0][1].dof_pos[:3]
    assert seq[0][1].cmd_mode == 3 and seq[0][1].valid == 1
    # 마지막 재생 프레임(RELEASE 직전)은 standby 로 돌아와 있어야 한다
    play_frames = [f for _, f in seq if f.valid == 1 and f.cmd_mode == 3]
    assert np.allclose(play_frames[-1].dof_pos, 0.0, atol=1e-5), play_frames[-1].dof_pos[:3]
    print("  ok starts_and_ends_at_standby")


def test_plan_release_switches_to_mode1_then_invalidates():
    seq = [f for _, f in publisher.plan_frames(_spec())]
    modes = [f.cmd_mode for f in seq]
    assert 1 in modes, "RELEASE 에서 mode1 패킷이 없다 — C++ 크로스페이드가 안 걸린다"
    first_m1 = modes.index(1)
    assert all(m == 3 for m in modes[:first_m1]), modes[:first_m1][:5]
    assert seq[-1].valid == 0, "마지막 패킷은 valid=0 이어야 한다"
    assert all(f.valid == 1 for f in seq[:-1])
    print("  ok release_sequence")


def test_plan_reaches_clip_pose_during_play():
    seq = [f for _, f in publisher.plan_frames(_spec())]
    assert any(np.allclose(f.dof_pos, 0.8) for f in seq), "재생 구간에서 클립 자세에 도달 못함"
    print("  ok reaches_clip_pose")


def test_plan_frame_spacing_is_control_period():
    seq = list(publisher.plan_frames(_spec(fps=50.0)))
    dts = [seq[i + 1][0] - seq[i][0] for i in range(len(seq) - 1)]
    assert all(abs(d - 0.02) < 1e-9 for d in dts), sorted(set(round(d, 6) for d in dts))
    print("  ok frame_spacing")


def test_speed_half_doubles_wall_clock():
    """속도 0.5배 -> PLAY 구간 벽시계가 정확히 2배.

    ⚠ (cmd_mode==3, valid==1) 프레임 수를 세면 안 된다 — RAMP_IN/RAMP_OUT 프레임도 같은
    태그를 달고 있고 speed 와 무관하게 길이가 고정이라, PLAY 가 정확히 2배(100->200)여도
    총합은 202->302 = 1.495x 로 희석된다. 시간 차이로 재야 성질이 그대로 드러난다.
    """
    full = list(publisher.plan_frames(_spec(speed=1.0)))
    half = list(publisher.plan_frames(_spec(speed=0.5)))
    play_wall_full = 100 / 50.0 / 1.0     # (f_end-f_start)/fps/speed
    play_wall_half = 100 / 50.0 / 0.5
    assert abs((half[-1][0] - full[-1][0]) - (play_wall_half - play_wall_full)) < 1e-6, \
        (full[-1][0], half[-1][0])
    assert len(half) - len(full) == 100, (len(full), len(half))
    print("  ok speed_half_doubles_wall_clock")


def test_abort_shortens_and_still_ends_at_standby():
    seq = list(publisher.plan_frames(_spec(ramp_in=1.0, ramp_out=1.0), abort_at=1.5))
    assert seq[-1][1].valid == 0
    play_frames = [f for _, f in seq if f.valid == 1 and f.cmd_mode == 3]
    assert np.allclose(play_frames[-1].dof_pos, 0.0, atol=1e-5), play_frames[-1].dof_pos[:3]
    normal = list(publisher.plan_frames(_spec(ramp_in=1.0, ramp_out=1.0)))
    assert seq[-1][0] < normal[-1][0], (seq[-1][0], normal[-1][0])
    print("  ok abort_path")


def test_publisher_writes_bytes_matching_vr_shm_contract():
    """송출 패킷이 vr_shm 계약(276B, magic 0x6702)과 바이트 일치하는지."""
    captured = []

    def fake_write(seq, valid, cmd_mode, base_vel, root_quat, dof_pos, dof_vel):
        captured.append(struct.pack(vr_shm.FMT, vr_shm.MAGIC, int(seq), int(valid),
                                    int(cmd_mode), *base_vel, *root_quat, *dof_pos, *dof_vel))

    p = publisher.Publisher(writer=fake_write, sleeper=lambda _t: None, clock=_FakeClock())
    result = p.run(_spec(ramp_in=0.1, ramp_out=0.1))
    assert result == "completed", result
    assert captured, "아무것도 송출하지 않았다"
    for buf in captured:
        assert len(buf) == 276, len(buf)
        magic, _seq, _v, _m = struct.unpack("<iIii", buf[:16])
        assert magic == 0x6702, hex(magic)
    seqs = [struct.unpack("<iIii", b[:16])[1] for b in captured]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), "seq 가 단조증가하지 않는다"
    print("  ok shm_contract")


class _FakeClock:
    """벽시계 대신 쓰는 가짜 시계 — 호출할 때마다 20 ms 씩 흐른다."""
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        v = self.t
        self.t += 0.02
        return v


def test_publisher_should_abort_hook():
    calls = {"n": 0}

    def should_abort():
        calls["n"] += 1
        return calls["n"] > 10

    p = publisher.Publisher(writer=lambda *a, **k: None, sleeper=lambda _t: None,
                            clock=_FakeClock())
    result = p.run(_spec(ramp_in=0.1, ramp_out=0.1), should_abort=should_abort)
    assert result == "aborted", result
    print("  ok should_abort_hook")


def main() -> int:
    test_plan_starts_at_standby_and_ends_at_standby()
    test_plan_release_switches_to_mode1_then_invalidates()
    test_plan_reaches_clip_pose_during_play()
    test_plan_frame_spacing_is_control_period()
    test_speed_half_doubles_wall_clock()
    test_abort_shortens_and_still_ends_at_standby()
    test_publisher_writes_bytes_matching_vr_shm_contract()
    test_publisher_should_abort_hook()
    print("test_publisher: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_publisher.py
```
Expected: FAIL — `ImportError: cannot import name 'publisher'`

- [ ] **Step 3: 최소 구현을 쓴다**

`deploy/robots/g1/teleop/motion_player/publisher.py`:

```python
"""Publisher — 램프인/재생/램프아웃/해제 상태기계와 50 Hz 송출.

시퀀스 계산(plan_frames, 순수)과 실행(Publisher.run, 시간·shm)을 나눈다.
로봇 없이 전체 시퀀스를 검증하기 위해서다.

⚠ 종료 프로토콜을 지키는 것이 이 모듈의 존재 이유다:
  - C++ clear_vr() 은 q_ref 를 standby 로 계단 스냅한다 -> RAMP_OUT 이 먼저 끝나야 valid=0.
  - C++ 재앵커/크로스페이드는 모드 전환에만 발동한다 -> 끝에 cmd_mode=1 패킷을 반드시 보낸다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterator

import numpy as np

from .clips import ClipData, DeployProfile
from .frames import RefFrame, play_frame, ramp_frame

RAMP_OUT_S = 1.5
ABORT_RAMP_OUT_S = 0.8
RELEASE_HOLD_S = 0.5
CONTROL_DT = 0.02          # 50 Hz — 정책 step_dt 와 동일


@dataclass
class PlaybackSpec:
    clip: ClipData
    profile: DeployProfile
    f_start: int
    f_end: int
    mode: int
    speed: float
    base_vel_kind: str          # "clip" | "zero" | "manual"
    manual_bv: tuple[float, float, float]
    ramp_in_s: float
    ramp_out_s: float = RAMP_OUT_S


def _standby_frame(spec: PlaybackSpec, cmd_mode: int, valid: int) -> RefFrame:
    return RefFrame(cmd_mode=cmd_mode, valid=valid, base_vel=(0.0, 0.0, 0.0),
                    root_quat=(1.0, 0.0, 0.0, 0.0),
                    dof_pos=spec.profile.standby.astype(np.float32),
                    dof_vel=np.zeros(29, dtype=np.float32))


def plan_frames(spec: PlaybackSpec, abort_at: float | None = None
                ) -> Iterator[tuple[float, RefFrame]]:
    """(경과 벽시계 초, 프레임) 시퀀스. 시간을 읽지 않는 순수 생성기.

    abort_at 이 주어지면 그 시각에 재생을 끊고 짧은 램프아웃으로 넘어간다.
    """
    t = 0.0

    # --- RAMP_IN: standby -> clip[f_start] ---
    n_in = max(1, int(round(spec.ramp_in_s / CONTROL_DT)))
    for i in range(n_in + 1):
        yield t, ramp_frame(spec.clip, spec.profile, spec.f_start, i / n_in,
                            spec.mode, (0.0, 0.0, 0.0), "in")
        t += CONTROL_DT

    # --- PLAY: 클립 시간축을 speed 로 훑는다 ---
    play_start = t
    n_clip = spec.f_end - spec.f_start
    play_wall_s = (n_clip / spec.clip.fps) / max(1e-6, spec.speed)
    n_play = max(1, int(round(play_wall_s / CONTROL_DT)))
    f_last = spec.f_start
    for i in range(n_play):
        if abort_at is not None and t >= abort_at:
            break
        clip_elapsed = (i * CONTROL_DT) * spec.speed
        f_last = min(spec.f_end - 1, spec.f_start + int(round(clip_elapsed * spec.clip.fps)))
        yield t, play_frame(spec.clip, f_last, spec.speed, spec.mode,
                            spec.base_vel_kind, spec.manual_bv)
        t += CONTROL_DT
    aborted = abort_at is not None and t < play_start + play_wall_s - 1e-9

    # --- RAMP_OUT: clip[f_last] -> standby ---
    out_s = ABORT_RAMP_OUT_S if aborted else spec.ramp_out_s
    n_out = max(1, int(round(out_s / CONTROL_DT)))
    for i in range(n_out + 1):
        yield t, ramp_frame(spec.clip, spec.profile, f_last, i / n_out,
                            spec.mode, (0.0, 0.0, 0.0), "out")
        t += CONTROL_DT

    # --- RELEASE: mode1 로 전환(재앵커/크로스페이드 유발) -> 유지 -> valid=0 ---
    for _ in range(max(1, int(round(RELEASE_HOLD_S / CONTROL_DT)))):
        yield t, _standby_frame(spec, cmd_mode=1, valid=1)
        t += CONTROL_DT
    yield t, _standby_frame(spec, cmd_mode=1, valid=0)


class Publisher:
    """plan_frames 를 실제 시간축에 태워 shm 으로 내보낸다."""

    def __init__(self, writer: Callable | None = None,
                 sleeper: Callable[[float], None] | None = None,
                 clock: Callable[[], float] | None = None):
        if writer is None:
            import vr_shm                        # teleop/ 에 있음 (sys.path 로 들어옴)
            writer = vr_shm.write
        self._write = writer
        self._sleep = sleeper if sleeper is not None else time.sleep
        self._clock = clock if clock is not None else time.perf_counter
        self._seq = 0

    def run(self, spec: PlaybackSpec,
            on_tick: Callable[[float, RefFrame], None] | None = None,
            should_abort: Callable[[], bool] | None = None) -> str:
        """재생. 반환값 "completed" | "aborted".

        타이밍은 sleep 누적이 아니라 절대시각 데드라인이다. 밀리면 프레임을 떨어뜨리고
        시간축을 지킨다 (참조가 느려지는 것보다 낫다).
        """
        start = self._clock()
        aborted = False
        gen = plan_frames(spec)
        pending: list[tuple[float, RefFrame]] = []
        for t_rel, frame in gen:
            if not aborted and should_abort is not None and should_abort():
                aborted = True
                now = self._clock() - start
                pending = list(plan_frames(spec, abort_at=max(now, 0.0)))
                pending = [(tt, ff) for tt, ff in pending if tt >= now]
                break
            self._emit(t_rel, frame, start, on_tick)
        for t_rel, frame in pending:
            self._emit(t_rel, frame, start, on_tick)
        return "aborted" if aborted else "completed"

    def _emit(self, t_rel: float, frame: RefFrame, start: float, on_tick) -> None:
        deadline = start + t_rel
        lag = self._clock() - deadline
        if lag < -1e-6:
            self._sleep(-lag)
        elif lag > CONTROL_DT:
            return                        # 밀렸으면 이 프레임은 버리고 시간축을 지킨다
        self._seq += 1
        self._write(self._seq, frame.valid, frame.cmd_mode, list(frame.base_vel),
                    list(frame.root_quat), frame.dof_pos.tolist(), frame.dof_vel.tolist())
        if on_tick is not None:
            on_tick(t_rel, frame)
```

- [ ] **Step 4: 통과하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_publisher.py
```
Expected: PASS — `test_publisher: ALL PASS`

- [ ] **Step 5: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/publisher.py \
        deploy/robots/g1/teleop/motion_player/tests/test_publisher.py
git commit -m "motion_player: 램프인/재생/램프아웃/해제 상태기계와 50Hz 송출

순수 생성기(plan_frames)와 실행기(Publisher)를 분리해 로봇 없이 전체 시퀀스를 검증한다.
종료 시 RAMP_OUT 으로 standby 에 도달한 뒤에만 valid=0 을 보내고(clear_vr 계단 스냅 회피),
cmd_mode=1 패킷으로 모드 전환을 일으켜 C++ 재앵커/크로스페이드를 받는다."
```

---

## Task 5: Playlist — 프리셋과 CLI 문법

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/playlist.py`
- Create: `deploy/robots/g1/teleop/motion_player/presets.yaml`
- Test: `deploy/robots/g1/teleop/motion_player/tests/test_playlist.py`

**Interfaces:**
- Consumes: `resolver.ClipInfo`
- Produces:
  - `PlayItem(clip_name: str, span: tuple[float, float] | None, mode: int, base_vel: str, speed: float, label: str)`
  - `PlayerConfig(motion_root: Path, slot_overrides: dict[str, str], presets: list[PlayItem])`
  - `load_config(path: Path) -> PlayerConfig`
  - `presets_for(cfg: PlayerConfig, clip_name: str) -> list[PlayItem]`
  - `parse_command(text: str, clips: list[ClipInfo], cfg: PlayerConfig, default_mode: int) -> tuple[str, object]`

### CLI 문법

`parse_command` 는 `(kind, payload)` 를 돌려준다. `kind` 는 `"play"` | `"list"` | `"quit"` | `"error"`.

```
1a              -> ("play", PlayItem)     클립 #1 의 프리셋 a
1 40 15         -> ("play", PlayItem)     클립 #1, 40 초부터 15 초 (span = (40, 55))
1 40 15 x0.5    -> ("play", PlayItem)     0.5 배속
1 40 15 m2      -> ("play", PlayItem)     mode2 강제
1 40 15 x0.5 m2 -> ("play", PlayItem)     둘 다
l               -> ("list", None)
q               -> ("quit", None)
그 외            -> ("error", "사람이 읽을 사유")
```

`m1` 은 파싱 단계에서 거부한다(사유 문자열에 이유를 담는다). 인덱스는 **1-기반**.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`deploy/robots/g1/teleop/motion_player/tests/test_playlist.py`:

```python
"""test_playlist.py — presets.yaml 로드 + CLI 문법 파싱 검증."""
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import playlist, resolver  # noqa: E402

CLIPS = [resolver.ClipInfo(name="walk1_subject1", path=Path("/x/walk1_subject1.npz"),
                           modes=(1, 2, 3)),
         resolver.ClipInfo(name="jumps1_subject1", path=Path("/x/jumps1_subject1.npz"),
                           modes=(2, 3))]

YAML = """
motion_root: /home/piene/mjlab1.4/mjlab_g1_motion
slot_overrides:
  gmt_multihead_cwc_scratch: motions/g1_flow_specialists_deploy24.yaml
presets:
  - clip: walk1_subject1
    label: "직진 보행"
    span: [12.0, 27.0]
    mode: 2
    base_vel: clip
    speed: 1.0
  - clip: walk1_subject1
    label: "회전"
    span: [88.0, 103.0]
    mode: 3
    base_vel: zero
    speed: 0.5
"""


def _cfg():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "presets.yaml"
        p.write_text(YAML)
        return playlist.load_config(p)


def test_load_config():
    cfg = _cfg()
    assert str(cfg.motion_root) == "/home/piene/mjlab1.4/mjlab_g1_motion"
    assert cfg.slot_overrides["gmt_multihead_cwc_scratch"] \
        == "motions/g1_flow_specialists_deploy24.yaml"
    assert len(cfg.presets) == 2
    assert cfg.presets[0].span == (12.0, 27.0)
    assert cfg.presets[0].mode == 2 and cfg.presets[0].base_vel == "clip"
    print("  ok load_config")


def test_presets_for():
    cfg = _cfg()
    assert len(playlist.presets_for(cfg, "walk1_subject1")) == 2
    assert playlist.presets_for(cfg, "jumps1_subject1") == []
    print("  ok presets_for")


def test_parse_preset_letter():
    cfg = _cfg()
    kind, item = playlist.parse_command("1a", CLIPS, cfg, default_mode=3)
    assert kind == "play", (kind, item)
    assert item.clip_name == "walk1_subject1" and item.span == (12.0, 27.0)
    assert item.mode == 2 and item.speed == 1.0
    kind, item = playlist.parse_command("1b", CLIPS, cfg, default_mode=3)
    assert item.span == (88.0, 103.0) and item.speed == 0.5 and item.mode == 3
    print("  ok parse_preset_letter")


def test_parse_arbitrary_span():
    cfg = _cfg()
    kind, item = playlist.parse_command("1 40 15", CLIPS, cfg, default_mode=3)
    assert kind == "play"
    assert item.span == (40.0, 55.0), item.span
    assert item.mode == 3 and item.speed == 1.0
    print("  ok parse_arbitrary_span")


def test_parse_speed_and_mode_flags():
    cfg = _cfg()
    _, item = playlist.parse_command("1 40 15 x0.5", CLIPS, cfg, default_mode=3)
    assert item.speed == 0.5 and item.mode == 3
    _, item = playlist.parse_command("1 40 15 m2", CLIPS, cfg, default_mode=3)
    assert item.mode == 2 and item.speed == 1.0
    _, item = playlist.parse_command("1 40 15 x0.25 m2", CLIPS, cfg, default_mode=3)
    assert item.speed == 0.25 and item.mode == 2
    print("  ok parse_flags")


def test_parse_rejects_mode1_with_reason():
    cfg = _cfg()
    kind, msg = playlist.parse_command("1 40 15 m1", CLIPS, cfg, default_mode=3)
    assert kind == "error", (kind, msg)
    assert "mode1" in msg and "마스킹" in msg, msg
    print("  ok reject_mode1")


def test_parse_rejects_bad_input():
    cfg = _cfg()
    for text in ["", "99 0 5", "1 40", "1a9", "1 abc 5", "0 0 5", "1 40 15 x0"]:
        kind, msg = playlist.parse_command(text, CLIPS, cfg, default_mode=3)
        assert kind == "error", (text, kind)
        assert isinstance(msg, str) and msg, text
    print("  ok reject_bad_input")


def test_parse_list_and_quit():
    cfg = _cfg()
    assert playlist.parse_command("l", CLIPS, cfg, 3)[0] == "list"
    assert playlist.parse_command("q", CLIPS, cfg, 3)[0] == "quit"
    print("  ok list_quit")


def main() -> int:
    test_load_config()
    test_presets_for()
    test_parse_preset_letter()
    test_parse_arbitrary_span()
    test_parse_speed_and_mode_flags()
    test_parse_rejects_mode1_with_reason()
    test_parse_rejects_bad_input()
    test_parse_list_and_quit()
    print("test_playlist: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: 실패하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_playlist.py
```
Expected: FAIL — `ImportError: cannot import name 'playlist'`

- [ ] **Step 3: 최소 구현을 쓴다**

`deploy/robots/g1/teleop/motion_player/presets.yaml`:

```yaml
# LAFAN Motion Player 설정.
#
# motion_root    : 매니페스트/클립 경로의 기준 (mjlab_g1_motion 워크스페이스 루트)
# slot_overrides : ONNX_META 의 manifest 를 신뢰할 수 없는 슬롯에 실제 경로를 지정.
#                  gmt_multihead_cwc_scratch 의 ONNX_META 는
#                  "motions/g1_flow_specialists.yaml (24클립, ...)" 라고 적혀 있으나
#                  그 파일은 존재하지 않는다. 24클립의 실체는 _deploy24.yaml 이다.
#                  ⚠ 이 매핑은 사람이 확인한 사실이다. 추측으로 늘리지 말 것.
# presets        : 자주 쓰는 재생 구간.

motion_root: /home/piene/mjlab1.4/mjlab_g1_motion

slot_overrides:
  gmt_multihead_cwc_scratch: motions/g1_flow_specialists_deploy24.yaml
  gmt_multihead_v0: motions/g1_flow_specialists_deploy24.yaml

presets: []
```

`deploy/robots/g1/teleop/motion_player/playlist.py`:

```python
"""Playlist — presets.yaml 로드와 CLI 문법 파싱. 파일도 shm 도 만지지 않는다(설정 읽기 제외)."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .resolver import ClipInfo

_MODE1_REASON = (
    "mode1 재생 불가 — 참조가 상·하체 전부 마스킹돼 정책에 도달하지 않는다 "
    "(masked_joint_command / masked_root_ori_b 가 0). mode2 또는 mode3 을 쓰세요.")


@dataclass
class PlayItem:
    clip_name: str
    span: tuple[float, float] | None      # None = 클립 전체 (권장하지 않음)
    mode: int
    base_vel: str                         # "clip" | "zero" | "manual"
    speed: float
    label: str = ""


@dataclass
class PlayerConfig:
    motion_root: Path
    slot_overrides: dict[str, str] = field(default_factory=dict)
    presets: list[PlayItem] = field(default_factory=list)


def load_config(path: Path) -> PlayerConfig:
    doc = yaml.safe_load(Path(path).read_text()) or {}
    presets = []
    for e in doc.get("presets") or []:
        span = e.get("span")
        presets.append(PlayItem(
            clip_name=e["clip"],
            span=(float(span[0]), float(span[1])) if span else None,
            mode=int(e.get("mode", 3)),
            base_vel=str(e.get("base_vel", "zero")),
            speed=float(e.get("speed", 1.0)),
            label=str(e.get("label", ""))))
    return PlayerConfig(motion_root=Path(doc.get("motion_root", ".")),
                        slot_overrides=dict(doc.get("slot_overrides") or {}),
                        presets=presets)


def presets_for(cfg: PlayerConfig, clip_name: str) -> list[PlayItem]:
    return [p for p in cfg.presets if p.clip_name == clip_name]


_PRESET_RE = re.compile(r"^(\d+)([a-z])$")


def parse_command(text: str, clips: list[ClipInfo], cfg: PlayerConfig,
                  default_mode: int) -> tuple[str, object]:
    """CLI 한 줄 -> (kind, payload). kind: play | list | quit | error."""
    t = (text or "").strip()
    if not t:
        return "error", "빈 입력"
    if t in ("q", "quit"):
        return "quit", None
    if t in ("l", "list"):
        return "list", None

    m = _PRESET_RE.match(t)
    if m:
        idx, letter = int(m.group(1)), m.group(2)
        if not (1 <= idx <= len(clips)):
            return "error", f"클립 번호 {idx} 가 범위를 벗어남 (1~{len(clips)})"
        name = clips[idx - 1].name
        ps = presets_for(cfg, name)
        k = ord(letter) - ord("a")
        if not (0 <= k < len(ps)):
            return "error", f"{name} 에 프리셋 '{letter}' 가 없음 (프리셋 {len(ps)}개)"
        return "play", ps[k]

    parts = t.split()
    if len(parts) < 3:
        return "error", "형식: <번호> <시작초> <길이초> [x<속도>] [m<모드>]  (예: 1 40 15 x0.5 m2)"
    try:
        idx = int(parts[0])
        start = float(parts[1])
        dur = float(parts[2])
    except ValueError:
        return "error", "번호/시작초/길이초는 숫자여야 함"
    if not (1 <= idx <= len(clips)):
        return "error", f"클립 번호 {idx} 가 범위를 벗어남 (1~{len(clips)})"
    if start < 0 or dur <= 0:
        return "error", "시작초는 0 이상, 길이초는 0 보다 커야 함"

    speed, mode = 1.0, default_mode
    for extra in parts[3:]:
        if extra.startswith("x"):
            try:
                speed = float(extra[1:])
            except ValueError:
                return "error", f"속도 형식이 잘못됨: {extra} (예: x0.5)"
            if speed <= 0:
                return "error", "속도는 0 보다 커야 함"
        elif extra.startswith("m"):
            try:
                mode = int(extra[1:])
            except ValueError:
                return "error", f"모드 형식이 잘못됨: {extra} (예: m2)"
        else:
            return "error", f"알 수 없는 옵션: {extra}"

    if mode == 1:
        return "error", _MODE1_REASON
    if mode not in (2, 3):
        return "error", f"mode{mode} 는 VR 채널로 보낼 수 없다 (g_poll_vr 이 1~3만 수용)"

    return "play", PlayItem(clip_name=clips[idx - 1].name, span=(start, start + dur),
                            mode=mode, base_vel="clip" if mode == 2 else "zero", speed=speed)
```

- [ ] **Step 4: 통과하는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_playlist.py
```
Expected: PASS — `test_playlist: ALL PASS`

- [ ] **Step 5: Resolver 가 override 로 실제 클립을 찾는지 확인한다 (수동)**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python -c "
import sys; sys.path.insert(0, 'deploy/robots/g1/teleop')
from pathlib import Path
from motion_player import resolver, playlist
cfg = playlist.load_config(Path('deploy/robots/g1/teleop/motion_player/presets.yaml'))
ctx = resolver.resolve(Path('deploy/robots/g1/config/config.yaml'),
                       motion_root=cfg.motion_root, slot_overrides=cfg.slot_overrides)
print('slot:', ctx.slot, '| clips:', len(ctx.clips), '| valid_modes:', sorted(ctx.valid_modes))
for w in ctx.warnings: print('  WARN', w)
"
```
Expected: `clips: 24`, `valid_modes: [1, 2, 3]`, 경고 없음.
**24가 아니면 멈추고 사용자에게 보고한다** — 매핑 가정이 틀린 것이다.

- [ ] **Step 6: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/playlist.py \
        deploy/robots/g1/teleop/motion_player/presets.yaml \
        deploy/robots/g1/teleop/motion_player/tests/test_playlist.py
git commit -m "motion_player: presets.yaml 과 CLI 문법 파싱

slot_overrides 로 ONNX_META 의 깨진 manifest 경로를 사람이 확인한 실제 경로로 잇는다.
mode1 요청은 파싱 단계에서 이유와 함께 거부한다."
```

---

## Task 6: CLI — 화면, 확인 프롬프트, dry-run, 런북

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/cli.py`
- Modify: `deploy/robots/g1/teleop/README.md` (파일 끝에 절 추가)
- Modify: `rules/SYSTEM_OVERVIEW.md` (§6 실행 명령어 요약에 한 줄 추가)

**Interfaces:**
- Consumes: `resolver.resolve`, `playlist.load_config/parse_command/presets_for`, `clips.load_clip/load_deploy_profile/preflight`, `publisher.Publisher/PlaybackSpec`
- Produces: 실행 진입점 `python -m motion_player.cli` / `python deploy/robots/g1/teleop/motion_player/cli.py`

### 배경

이 태스크는 **사람이 쓰는 물건**을 만든다. 단위 테스트 대신 `--dry-run` 이 검증 수단이다.
`--dry-run` 은 shm에 아무것도 쓰지 않고 계획된 시퀀스를 요약 출력한다 (spec §13 검증 1단계).

터미널 키 입력은 `termios`/`tty` raw 모드를 쓴다. **반드시 `try/finally` 로 원복**한다 —
이 repo에는 Ctrl-C 후 터미널이 raw로 남는 사고 이력이 있다 (커밋 `3b79d12`).

- [ ] **Step 1: CLI 를 구현한다**

`deploy/robots/g1/teleop/motion_player/cli.py`:

```python
#!/usr/bin/env python3
"""LAFAN Motion Player — 배포 정책의 모션 참조로 클립을 재생하는 터미널 플레이어.

사용 (com1 에서 unitree_mujoco + g1_ctrl --network=lo 가 떠 있는 상태):
    .venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py
    .venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run

⚠ 재기동 전 `pkill -x g1_ctrl` — orphan 컨트롤러 중복은 즉시 낙상으로 나타나 policy 문제로 오진된다.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import termios
import tty
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))          # teleop/ (vr_shm, estop_shm)
sys.path.insert(0, str(_HERE.parent.parent))   # motion_player 패키지 부모

from motion_player import clips as clips_mod          # noqa: E402
from motion_player import playlist as playlist_mod    # noqa: E402
from motion_player import publisher as pub_mod        # noqa: E402
from motion_player import resolver as resolver_mod    # noqa: E402

_G1_DIR = _HERE.parent.parent                  # deploy/robots/g1
_CONFIG = _G1_DIR / "config" / "config.yaml"
_PRESETS = _HERE / "presets.yaml"


def _render_list(ctx, cfg) -> None:
    dep = {True: "실기 OK", False: "실기 배포 불가", None: "실기 여부 미상"}[ctx.deployable]
    modes = ",".join(str(m) for m in sorted(ctx.valid_modes)) or "없음"
    print(f"\nLAFAN Player — policy: {ctx.slot}   (modes {modes} · {dep})")
    if ctx.manifest_path:
        print(f"manifest: {ctx.manifest_path.name}  ·  {len(ctx.clips)} clips")
    for w in ctx.warnings:
        print(f"  ⚠ {w}")
    if not ctx.clips:
        print("\n재생할 클립이 없습니다. presets.yaml 의 slot_overrides 를 확인하세요.\n")
        return
    print(f"\n  {'#':>3}  {'clip':<24} {'length':>9}  {'modes':<7} presets")
    for i, c in enumerate(ctx.clips, 1):
        ps = playlist_mod.presets_for(cfg, c.name)
        tag = "  ".join(f"[{chr(ord('a') + k)}] {p.label or ''} "
                        f"{p.span[0]:.0f}–{p.span[1]:.0f}s" for k, p in enumerate(ps)) or "—"
        try:
            d = clips_mod.load_clip(c)
            length = f"{d.duration:.1f}s"
        except Exception:
            length = "?"
        print(f"  {i:>3}  {c.name:<24} {length:>9}  "
              f"{','.join(str(m) for m in c.modes):<7} {tag}")
    print("\n> <번호><프리셋문자>   예: 1a")
    print("> <번호> <시작초> <길이초> [x속도] [m모드]   예: 1 40 15 x0.5 m2")
    print("> l · q   목록 · 종료\n")


def _confirm(clip, pre, item, dry: bool) -> bool:
    wall = (pre.f_end - pre.f_start) / clip.fps / item.speed
    clip_s = (pre.f_end - pre.f_start) / clip.fps
    print(f"\n▶ {clip.name}   {item.span[0]:.1f}–{item.span[1]:.1f}s "
          f"({clip_s:.1f}s, ×{item.speed:.2f} → 벽시계 {wall:.1f}s)   "
          f"mode{item.mode} · base_vel={item.base_vel}")
    print(f"  진입 자세차 {pre.entry_err:.2f} rad (joint[{pre.entry_joint}]) "
          f"→ 램프인 {pre.ramp_in_s:.1f}s")
    if not pre.ok:
        for r in pre.reasons:
            print(f"  ✗ {r}")
        print("  재생하지 않습니다.\n")
        return False
    print("  관절한계 위반 없음 · NaN 없음 · 매니페스트 ✓")
    if dry:
        print("  [dry-run] 송출하지 않습니다.\n")
        return False
    ans = input("  [Enter] 재생   [그 외] 취소 > ")
    return ans == ""


class _KeyWatcher:
    """재생 중 Space(중단) / x(E-stop) 을 논블로킹으로 본다. 터미널 상태를 반드시 원복한다."""

    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else -1
        self.saved = None
        self.abort = False
        self.estop = False

    def __enter__(self):
        if self.enabled:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            os.set_blocking(self.fd, False)
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            os.set_blocking(self.fd, True)
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def poll(self) -> bool:
        if not self.enabled:
            return self.abort
        try:
            ch = os.read(self.fd, 1)
        except (BlockingIOError, OSError):
            return self.abort
        if ch in (b" ", b"\x03"):
            self.abort = True
        elif ch == b"x":
            self.estop = True
            self.abort = True
        return self.abort


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LAFAN Motion Player")
    ap.add_argument("--dry-run", action="store_true",
                    help="shm 에 쓰지 않고 계획만 출력")
    ap.add_argument("--arm-estop", action="store_true",
                    help="E-stop 하트비트 무장 (플레이어 사망 시 Passive). 기본 미무장")
    ap.add_argument("--presets", default=str(_PRESETS))
    args = ap.parse_args(argv)

    cfg = playlist_mod.load_config(Path(args.presets))
    ctx = resolver_mod.resolve(_CONFIG, motion_root=cfg.motion_root,
                               slot_overrides=cfg.slot_overrides)
    profile = clips_mod.load_deploy_profile(ctx.policy_dir)
    default_mode = 2 if 2 in ctx.valid_modes else (3 if 3 in ctx.valid_modes else 3)

    estop = None
    if args.arm_estop and not args.dry_run:
        import estop_shm
        estop = estop_shm

    _render_list(ctx, cfg)
    seq_estop = 0
    try:
        while True:
            try:
                text = input("> ")
            except EOFError:
                break
            kind, payload = playlist_mod.parse_command(text, ctx.clips, cfg, default_mode)
            if kind == "quit":
                break
            if kind == "list":
                _render_list(ctx, cfg)
                continue
            if kind == "error":
                print(f"  ✗ {payload}")
                continue

            item = payload
            info = next(c for c in ctx.clips if c.name == item.clip_name)
            clip = clips_mod.load_clip(info)
            allowed = tuple(sorted(set(info.modes) & (ctx.valid_modes or set(info.modes))))
            pre = clips_mod.preflight(clip, item.span, item.mode, allowed, profile)
            if not _confirm(clip, pre, item, args.dry_run):
                continue

            spec = pub_mod.PlaybackSpec(
                clip=clip, profile=profile, f_start=pre.f_start, f_end=pre.f_end,
                mode=item.mode, speed=item.speed, base_vel_kind=item.base_vel,
                manual_bv=(0.0, 0.0, 0.0), ramp_in_s=pre.ramp_in_s)

            with _KeyWatcher(enabled=True) as keys:
                def on_tick(t_rel, frame):
                    nonlocal seq_estop
                    if estop is not None:
                        seq_estop += 1
                        estop.write(seq_estop, 1 if keys.estop else 0)
                    print(f"\r  ▶ {clip.name} {t_rel:6.2f}s  mode{frame.cmd_mode}  "
                          f"Space=중단 x=E-stop ", end="", flush=True)

                p = pub_mod.Publisher()
                result = p.run(spec, on_tick=on_tick, should_abort=keys.poll)
            print(f"\n  {result}"
                  + ("  (E-stop 발동 — g1_ctrl 이 Passive 로 갔습니다. f 로 재기립)"
                     if keys.estop else ""))
    finally:
        if estop is not None:
            estop.clear()
    print("bye")
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    sys.exit(main())
```

- [ ] **Step 2: dry-run 이 도는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
printf 'l\nq\n' | .venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run
```
Expected: 슬롯 이름 + `24 clips` + 클립 표가 출력되고 `bye` 로 끝난다. 경고 줄이 없어야 한다.

- [ ] **Step 3: dry-run 재생 계획이 나오는지 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
printf '1 40 15 x0.5 m2\nq\n' | .venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run
```
Expected: `▶ <clip> 40.0–55.0s (15.0s, ×0.50 → 벽시계 30.0s) mode2 · base_vel=clip`,
진입 자세차와 램프인 초, `[dry-run] 송출하지 않습니다.`

- [ ] **Step 4: mode1 거부를 확인한다**

```bash
cd /home/piene/unitree_rl_mjlab
printf '1 40 15 m1\nq\n' | .venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run
```
Expected: `✗ mode1 재생 불가 — 참조가 상·하체 전부 마스킹돼 ...`

- [ ] **Step 5: 전체 테스트를 한 번에 돌린다**

```bash
cd /home/piene/unitree_rl_mjlab
for t in resolver clips frames publisher playlist; do
  .venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_$t.py || exit 1
done
echo "ALL MOTION_PLAYER TESTS PASS"
```
Expected: 5개 전부 `ALL PASS` 후 `ALL MOTION_PLAYER TESTS PASS`

- [ ] **Step 6: README 에 운영 절을 추가한다**

`deploy/robots/g1/teleop/README.md` **파일 맨 끝**에 append:

```markdown

## LAFAN Motion Player (`motion_player/`)

학습에 쓴 LAFAN 클립을 배포 정책의 모션 참조로 재생한다. PICO 없이 동작한다.

```bash
# com1: unitree_mujoco + g1_ctrl 이 떠 있어야 한다. 재기동 전 pkill -x g1_ctrl 필수.
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py --dry-run   # 계획만
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py            # 실제 송출
```

- **mode2** = 상체만 클립, 다리는 자율 보행 (`base_vel: clip` 이면 클립대로 이동). **실기 첫 시도용.**
- **mode3** = 전신 클립. base_vel 은 C++ 가 0 으로 덮는다.
- **mode1 은 재생 불가** — 참조가 마스킹돼 정책에 도달하지 않는다.
- 재생 중: `Space` 중단(램프아웃 0.8 s), `x` E-stop(즉시 Passive, 복구는 `f` 재기립).
- 클립 목록은 배포 정책의 `ONNX_META.json` → 매니페스트를 따라 자동으로 정해진다.
  슬롯이 바뀌면 목록도 따라 바뀐다. 매니페스트를 못 찾으면 `presets.yaml` 의
  `slot_overrides` 에 실제 경로를 적는다.

테스트: `for t in resolver clips frames publisher playlist; do .venv/bin/python deploy/robots/g1/teleop/motion_player/tests/test_$t.py; done`
```

- [ ] **Step 7: SYSTEM_OVERVIEW 실행 요약에 한 줄 추가한다**

`rules/SYSTEM_OVERVIEW.md` §6 의 `# ── 점검용 ──` 블록에서 아래 줄을

```
python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2 # PICO 없이 VR경로 검증
```

이렇게 바꾼다 (기존 줄은 남기고 아래에 한 줄 추가):

```
python deploy/robots/g1/teleop/vr_replay.py <motion.npz> --mode 2 # PICO 없이 VR경로 검증
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py     # LAFAN 클립 재생(구간/속도/mode2·3)
```

- [ ] **Step 8: 커밋**

```bash
git add deploy/robots/g1/teleop/motion_player/cli.py \
        deploy/robots/g1/teleop/README.md \
        rules/SYSTEM_OVERVIEW.md
git commit -m "motion_player: 터미널 CLI + dry-run + 운영 문서

클립 목록/확인 프롬프트/재생 중 Space·x 키. 터미널 raw 모드는 finally 로 반드시 원복한다.
dry-run 이 shm 없이 계획을 검증하는 1단계 검증 수단이다."
```

---

## Task 7: sim2sim 검증과 O1 계측

**Files:**
- Create: `deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py`
- Modify: `docs/superpowers/specs/2026-08-13-lafan-motion-player-design.md` (§12 O1 에 결과 기록)

**Interfaces:**
- Consumes: 완성된 `cli.py`
- Produces: O1(mode2 yaw 드리프트) 판정과 spec 갱신

### 배경

spec §12 O1: mode2 에서 다리는 `base_vel` 로 걷고 상체 참조는 클립 pelvis 자세다. 로봇 실제
yaw 와 클립 yaw 가 벌어지면 `masked_root_ori_b` 오차가 누적된다. **추측하지 않고 잰다.**

로봇 실제 yaw 는 `g1_ctrl` 내부에 있어 Python 에서 직접 못 읽는다. 대신 **MuJoCo sim 쪽에서**
읽는다: `simulate/` 의 unitree_mujoco 가 DDS 로 `rt/lowstate` 를 내보내므로 `unitree_sdk2py`
로 IMU quaternion 을 구독한다. 이 의존성이 없으면 **대안**: sim GUI 화면을 60 s 녹화해 육안
판정하고 그 사실을 spec 에 명시한다 (없는 계측을 지어내지 않는다).

- [ ] **Step 1: 사전 정리 후 sim2sim 을 띄운다 (사용자 수행)**

```bash
pkill -x g1_ctrl; pkill -x unitree_mujoco
pgrep -x g1_ctrl                      # 비어 있어야 한다
cd /home/piene/unitree_rl_mjlab
./simulate/build/unitree_mujoco &      # 터미널 1
cd deploy/robots/g1/build && ./g1_ctrl --network=lo    # 터미널 2
# g1_ctrl 터미널에서 f (FixStand) -> m (Mimic_Masked)
```

- [ ] **Step 2: mode2 ×0.5, base_vel=zero 로 첫 재생**

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/cli.py
# > 1 40 15 x0.5 m2      (walk 클립, base_vel 은 mode2 기본이 clip 이므로
#                          zero 로 보려면 presets.yaml 에 base_vel: zero 프리셋을 두고 1a 로 실행)
```
확인: 진입에서 관절이 튀지 않는가 · 재생 중 상체가 클립을 따라가는가 · 종료 후 서 있는가.

- [ ] **Step 3: mode2 ×0.5, base_vel=clip 으로 재생**

`presets.yaml` 의 `presets:` 에 아래를 넣고 `1a` 로 실행:

```yaml
presets:
  - clip: walk1_subject1
    label: "직진 보행"
    span: [40.0, 55.0]
    mode: 2
    base_vel: clip
    speed: 0.5
```

확인: 클립이 걷는 대로 로봇도 이동하는가.

- [ ] **Step 4: mode3 ×0.5 → ×1.0 으로 올린다**

```
> 1 40 15 x0.5 m3
> 1 40 15 x1.0 m3
```
확인: 전신 재생 중 낙상이 없는가. 낙상하면 **속도를 더 낮춰 재시도하고 그 사실을 기록**한다.

- [ ] **Step 5: O1 계측 — mode2 60 초 재생 중 yaw 추이**

`deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py`:

```python
#!/usr/bin/env python3
"""yaw_probe.py — sim 로봇 pelvis yaw 를 주기적으로 찍어 mode2 재생의 yaw 드리프트를 잰다.

unitree_sdk2py 로 rt/lowstate 의 IMU quaternion 을 구독한다. 없으면 그 사실을 알리고 종료한다
(없는 계측을 지어내지 않는다).

    .venv/bin/python deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py --seconds 90
"""
from __future__ import annotations

import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--hz", type=float, default=5.0)
    args = ap.parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError:
        print("unitree_sdk2py 없음 — 이 계측은 수행할 수 없습니다.")
        print("대안: sim GUI 를 60s 녹화해 육안 판정하고 그 사실을 spec §12 O1 에 명시하세요.")
        return 2

    ChannelFactoryInitialize(0, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()

    t0 = time.perf_counter()
    print("t_s,yaw_rad")
    while time.perf_counter() - t0 < args.seconds:
        msg = sub.Read(200)
        if msg is not None:
            w, x, y, z = msg.imu_state.quaternion
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            print(f"{time.perf_counter() - t0:.2f},{yaw:.4f}", flush=True)
        time.sleep(1.0 / args.hz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

실행:

```bash
cd /home/piene/unitree_rl_mjlab
.venv/bin/python deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py --seconds 90 \
  > /tmp/claude-1000/yaw_mode2.csv &
# 다른 터미널에서 60s 이상 mode2 재생 (예: 1 40 60 x1.0 m2)
```

- [ ] **Step 6: 중단 경로를 전수 확인한다**

각각 재생 중에 실행하고, **관절 목표가 튀지 않는지**와 로봇이 서 있는지를 본다.

```bash
# 1) Space — 램프아웃 0.8s 후 mode1 복귀
# 2) Ctrl-C — 같은 경로
# 3) 강제 종료: 다른 터미널에서
pkill -9 -f motion_player/cli.py     # -> C++ VR stale 0.5s -> standby 스냅
# 4) E-stop: 재생 중 x  (--arm-estop 으로 띄운 세션에서) -> Passive. 복구는 g1_ctrl 에서 f
```

- [ ] **Step 7: 결과를 spec §12 에 기록하고 커밋**

`docs/superpowers/specs/2026-08-13-lafan-motion-player-design.md` 의 §12 O1 항목 아래에
**측정 결과와 결론**을 append 한다. 형식:

```markdown
**O1 측정 결과 (YYYY-MM-DD, sim2sim)**
- 조건: `<clip>` `<span>` ×`<speed>` mode2, base_vel=`<kind>`, 지속 `<N>` s
- yaw 오차: 시작 `<a>` rad → 종료 `<b>` rad (최대 `<c>` rad)
- 판정: `<드리프트 허용 / 대응 필요>` — 근거 `<한 줄>`
- 채택한 대응: `<없음 / 주기적 mode 왕복 / 구간 길이 상한 N초>`
```

`unitree_sdk2py` 가 없어 육안 판정했다면 **그 사실을 그대로 적는다.**

```bash
git add docs/superpowers/specs/2026-08-13-lafan-motion-player-design.md \
        deploy/robots/g1/teleop/motion_player/tools/yaw_probe.py \
        deploy/robots/g1/teleop/motion_player/presets.yaml
git commit -m "motion_player: sim2sim 검증 결과와 yaw 계측 도구

spec O1(mode2 yaw 드리프트)을 측정해 결론을 기록. 프리셋에 검증한 구간을 남긴다."
```

- [ ] **Step 8: 실로봇 전 체크리스트를 사용자에게 보고한다**

구현자는 **실로봇을 직접 돌리지 않는다.** 아래를 보고하고 멈춘다.

```
실로봇 재생 전 확인:
  1. sim2sim mode2 / mode3 전부 통과했는가
  2. 중단 경로 4가지(Space / Ctrl-C / kill -9 / E-stop) 전부 확인했는가
  3. O1 yaw 판정이 spec 에 기록됐는가
  4. 배포 슬롯이 deployable_on_orin_nx=true 인가 (v2_mode3_steps6 은 false)
  5. 첫 시도는 mode2 · walk · 짧은 구간 · ×0.5 · 사람이 하드 E-stop 옆에서 대기
```

---

## Self-Review

**1. Spec coverage**

| spec 요구 | 태스크 |
|---|---|
| R1 정책이 학습한 클립만 노출 | Task 1 |
| R2 임의 구간 + 속도, 원본 무수정 | Task 2, 5 |
| R3 프리셋 | Task 5 |
| R4 mode2/3 지원, mode1 거부 | Task 2(preflight), 5(parse) |
| R5 램프 진입/이탈 | Task 3, 4 |
| R6 사람 abort | Task 4(should_abort), 6(_KeyWatcher) |
| R7 C++ 무수정 | Global Constraints — 어느 태스크도 C++ 를 건드리지 않음 |
| R8 확장 흡수 | Task 1(Resolver) + Task 5(slot_overrides) |
| §5 Resolver 규칙 1~7 | Task 1 (7개 테스트로 전부 잠금) |
| §6 Playlist 필드/문법 | Task 5 |
| §7 상태기계 6단계 | Task 4 |
| §7.2 프리플라이트 7항목 | Task 2 |
| §7.3 확인 프롬프트 | Task 6 `_confirm` |
| §8 CLI 화면 | Task 6 `_render_list` |
| §9 안전 계층 0~2 신규 | Task 2, 3, 4, 6 |
| §9 E-stop 기본 미무장 + `--arm-estop` | Task 6 |
| §11 확장 시나리오 | Task 1 설계로 충족 |
| §12 O1 yaw 계측 | Task 7 |
| §13 검증 순서 6단계 | Task 6 Step 2~4(dry-run), Task 7 |
| §14 테스트 8종 | Task 1~5 테스트 (pytest → stdlib 로 형식만 변경) |
| §15 파일 배치 | File Structure (`clips.py`/`frames.py` 분리 명시) |

빠진 요구 없음.

**2. Placeholder scan** — "TBD"/"적절히"/"비슷하게" 없음. 모든 코드 스텝에 실제 코드가 있다.
Task 7 은 사람이 로봇 앞에서 하는 절차라 코드 대신 실행 명령과 기록 형식을 명시했다.

**3. Type consistency** — 태스크 간 이름 대조 완료:
`ClipInfo(name/path/modes)` (T1) → T2 `load_clip` 인자 / T5 `parse_command` 인자로 동일 사용.
`ClipData` 필드(`fps/n_frames/duration/joint_pos/joint_vel/root_quat/pelvis_lin_vel_w/pelvis_ang_vel_w`)
가 T2 정의 → T3 `_clip()` 테스트 → T4 `_spec()` 테스트에서 동일.
`DeployProfile(standby/pos_min/pos_max)` T2 → T3, T4 동일.
`PreflightResult(f_start/f_end/entry_err/entry_joint/ramp_in_s)` T2 → T6 `_confirm` 동일.
`RefFrame(cmd_mode/valid/base_vel/root_quat/dof_pos/dof_vel)` T3 → T4 동일.
`PlaybackSpec` 필드 T4 정의 → T6 생성 동일.
`PlayItem(clip_name/span/mode/base_vel/speed/label)` T5 → T6 사용 동일.
`vr_shm.write(seq, valid, cmd_mode, base_vel, root_quat, dof_pos, dof_vel)` — 기존 시그니처와 일치.
