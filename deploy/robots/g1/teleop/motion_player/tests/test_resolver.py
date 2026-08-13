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
