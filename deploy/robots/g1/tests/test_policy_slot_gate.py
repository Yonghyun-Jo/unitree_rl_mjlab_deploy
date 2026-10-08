#!/usr/bin/env python3
"""policy_slot.py 의 push 안전 게이트(controller Ruling 31) — `deployable_real: false` 슬롯은
로봇으로 못 나간다. 같은 자리의 두 번째 게이트: 학습↔배포 패리티 표(PARITY.json)가 PASS/KNOWN 이고
«지금 파일» 에 대한 것이어야 나간다 (뒤쪽 test_parity_*).

# 왜 이 테스트가 있나
개발용 슬롯(예: `260922_dev_v5_m45`)의 ONNX_META.json 은 `deployable_real: false` 를 갖지만,
그 필드를 읽는 코드가 없으면 `policy_slot.py activate v1=<dev 슬롯> && policy_slot.py push`
한 줄로 개발 정책이 실기로 나간다(`cmd_push` 가 그때까지 `has_weights()` 만 봤다).

게이트는 `policy_slot.undeployable_slots()` 하나로 모아 `cmd_push` 가 rsync 를 시작하기
«전에» 슬롯 전부를 검사한다(부분 push 금지 — 하나라도 걸리면 아무것도 안 보낸다). 이
테스트는 rsync 를 전혀 부르지 않고, 임시 슬롯 트리를 만들어 그 판정 함수만 직접 검증한다.

실행:  python3 deploy/robots/g1/tests/test_policy_slot_gate.py
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent          # deploy/robots/g1
REPO = G1.parent.parent.parent                        # unitree_rl_mjlab
sys.path.insert(0, str(REPO / "deploy/scripts"))


def _make_slot(root: Path, name: str, meta: dict | None) -> None:
    """슬롯 폴더 하나 + (있으면) ONNX_META.json 만 있는 최소 트리. 가중치는 안 만든다 —
    이 게이트는 가중치 유무와 무관하게 META 만 본다."""
    d = root / name
    d.mkdir(parents=True, exist_ok=True)
    if meta is not None:
        (d / "ONNX_META.json").write_text(json.dumps(meta), encoding="utf-8")


def test_deployable_real_false_is_blocked():
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        _make_slot(root, "dev_slot", {"deployable_real": False, "note": "개발용 — 실기 금지"})
        bad = policy_slot.undeployable_slots(["dev_slot"], slot_dir=str(root))
        assert bad == [("dev_slot", "개발용 — 실기 금지")], bad


def test_true_and_missing_field_pass():
    """`deployable_real: true` 와, 필드 자체가 없는 슬롯(지금까지의 모든 실기 슬롯)은 통과한다."""
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        _make_slot(root, "real_true", {"deployable_real": True})
        _make_slot(root, "real_no_field", {"note": "옛 슬롯 — 필드 자체가 없음"})
        _make_slot(root, "real_no_meta", None)          # ONNX_META.json 조차 없음
        bad = policy_slot.undeployable_slots(
            ["real_true", "real_no_field", "real_no_meta"], slot_dir=str(root))
        assert bad == [], bad


def test_mixed_batch_flags_only_the_dev_slot():
    """실기 slot + dev slot 을 같이 push 할 때 걸리는 건 dev 슬롯 하나뿐이어야 한다
    (호출부인 cmd_push 는 bad 가 하나라도 있으면 전부를 막는다 — 그건 cmd_push 몫)."""
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        _make_slot(root, "ok_slot", {"deployable_real": True})
        _make_slot(root, "dev_slot", {"deployable_real": False, "note": "실기 금지"})
        bad = policy_slot.undeployable_slots(["ok_slot", "dev_slot"], slot_dir=str(root))
        assert [s for s, _ in bad] == ["dev_slot"], bad


def test_falsy_but_not_bool_false_does_not_trip():
    """0 처럼 falsy 하지만 진짜 `False` 가 아닌 값과 헷갈리지 않는다 — 게이트는 `is False`."""
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        _make_slot(root, "weird_zero", {"deployable_real": 0})
        bad = policy_slot.undeployable_slots(["weird_zero"], slot_dir=str(root))
        assert bad == [], bad


def test_cmd_push_default_slot_dir_matches_real_slotdir():
    """`undeployable_slots` 의 기본 `slot_dir` 이 실제 policy_slot.SLOTDIR (cmd_push 가 쓰는 것)과
    같은지 — 이 상수가 달라지면 이 테스트 파일의 임시 트리 검증이 실제 게이트와 딴 것을 본다."""
    import policy_slot
    import inspect
    default = inspect.signature(policy_slot.undeployable_slots).parameters["slot_dir"].default
    assert default == policy_slot.SLOTDIR


# ── 패리티 게이트 (PARITY.json) ────────────────────────────────────────────
# check_slot_parity.py 가 쓴 판정이 «지금 슬롯 파일» 에 대한 PASS/KNOWN 일 때만 push 한다.
# 판정 뒤에 ONNX·deploy.yaml 이 바뀌면 그 판정은 다른 파일에 대한 것이다 → STALE 로 막는다.
import hashlib  # noqa: E402


def _h(path: Path, algo: str) -> str:
    return hashlib.new(algo, path.read_bytes()).hexdigest()


def _make_parity_slot(root: Path, name: str, verdict="PASS", known_ids=(), context=None):
    d = root / name
    (d / "exported").mkdir(parents=True, exist_ok=True)
    (d / "params").mkdir(parents=True, exist_ok=True)
    (d / "exported/policy.onnx").write_bytes(b"onnx-bytes-" + name.encode())
    (d / "params/deploy.yaml").write_text("step_dt: 0.02\n", encoding="utf-8")
    rep = {"verdict": verdict,
           "fresh": {"onnx_md5": _h(d / "exported/policy.onnx", "md5"),
                     "deploy_yaml_sha256": _h(d / "params/deploy.yaml", "sha256")},
           "context_sha256": context or {},
           "known_used": [{"id": k} for k in known_ids]}
    (d / "PARITY.json").write_text(json.dumps(rep), encoding="utf-8")
    return d


def test_parity_missing_and_fail_block_push():
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        _make_parity_slot(root, "ok_pass")
        _make_parity_slot(root, "ok_known", verdict="KNOWN", known_ids=("K1", "K2"))
        _make_parity_slot(root, "bad_fail", verdict="FAIL")
        d = _make_parity_slot(root, "bad_missing")
        (d / "PARITY.json").unlink()
        blocked = policy_slot.parity_blocked_slots(["ok_pass", "ok_known", "bad_fail", "bad_missing"],
                                                   slot_dir=str(root))
        assert [(s, st) for s, st, _ in blocked] == [("bad_fail", "FAIL"), ("bad_missing", "MISSING")], blocked
        st, detail, _ = policy_slot.parity_status("ok_known", slot_dir=str(root))
        assert st == "KNOWN" and "K1,K2" in detail, (st, detail)


def test_parity_stale_after_onnx_or_yaml_change():
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        a = _make_parity_slot(root, "onnx_changed")
        (a / "exported/policy.onnx").write_bytes(b"retrained")
        b = _make_parity_slot(root, "yaml_changed")
        (b / "params/deploy.yaml").write_text("step_dt: 0.02\n# 한 줄 더\n", encoding="utf-8")
        for s in ("onnx_changed", "yaml_changed"):
            st, detail, _ = policy_slot.parity_status(s, slot_dir=str(root))
            assert st == "STALE", (s, st, detail)


def test_parity_context_change_only_warns():
    """판정에 쓴 C++ 파일이 바뀌면 판정은 유효(통과)하되 경고 목록에 오른다 — push 는 막지 않는다."""
    import policy_slot
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        repo = root / "repo"
        (repo / "deploy").mkdir(parents=True)
        f = repo / "deploy/State_Mimic.cpp"
        f.write_text("int a = 1;\n", encoding="utf-8")
        _make_parity_slot(root, "ctx", context={"deploy/State_Mimic.cpp": _h(f, "sha256")})
        f.write_text("int a = 2;\n", encoding="utf-8")
        st, _, warns = policy_slot.parity_status("ctx", slot_dir=str(root), repo=str(repo))
        assert st == "PASS" and warns == ["deploy/State_Mimic.cpp"], (st, warns)
        assert policy_slot.parity_blocked_slots(["ctx"], slot_dir=str(root)) == []


def test_parity_gate_default_slot_dir_matches_real_slotdir():
    import policy_slot
    import inspect
    default = inspect.signature(policy_slot.parity_blocked_slots).parameters["slot_dir"].default
    assert default == policy_slot.SLOTDIR


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            fn()
            print(f"  ok   {name}")
        except Exception as e:                        # noqa: BLE001
            fails += 1
            print(f"  FAIL {name}: {e}")
    print(("모두 통과" if not fails else f"{fails}개 실패"))
    sys.exit(1 if fails else 0)
