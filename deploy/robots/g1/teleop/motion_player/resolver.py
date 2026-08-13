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
