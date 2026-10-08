#!/usr/bin/env python3
"""check_slot_parity.py 변이시험 — 판정이 «실제로 빨개지는가».

# 왜 있나
안 움직이는 계측기는 없는 계측기보다 나쁘다 — 초록불을 믿게 만든다. 고장을 일부러 심은 슬롯에서
그 행이 FAIL 하는 것을 본 판정만 믿는다. 기준선(지금 ACTIVE 슬롯들)은 PASS + K1·K2 KNOWN 이어야 한다.

# 진짜 슬롯은 건드리지 않는다
슬롯의 정체(deploy.yaml · ONNX_META.json)를 임시 폴더로 복사하고 ONNX 는 심링크로 «읽기만» 한다.
PARITY.* 도 임시 폴더에만 쓴다. 마지막 시험이 진짜 슬롯들의 PARITY.* 가 그대로인지 확인한다.

실행 (학습 cfg 를 import 하므로 mjlab venv 파이썬, conda 변수는 뺀다):
  env -u LD_LIBRARY_PATH -u CONDA_PREFIX ~/mjlab1.4/mjlab_g1_motion/.venv/bin/python \\
      deploy/robots/g1/tests/test_slot_parity.py
학습 원장(mjlab logs·git)이나 활성 슬롯 ONNX 가 없는 머신(로봇·깨끗한 clone)이면 «skip» 을 찍고 0 으로 끝난다.
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent          # deploy/robots/g1
REPO = G1.parent.parent.parent                        # unitree_rl_mjlab
sys.path.insert(0, str(REPO / "deploy/scripts"))

import yaml                                           # noqa: E402

import check_slot_parity as csp                       # noqa: E402
import policy_slot                                    # noqa: E402


def _active_slots():
    out = []
    for alias, s in sorted(policy_slot.read_active().items()):
        root = Path(policy_slot.SLOTDIR) / s
        if (root / "exported/policy.onnx").exists() and (root / "ONNX_META.json").exists():
            out.append(root)
    return out


def _skip_reason():
    slots = _active_slots()
    if not slots:
        return "가중치가 있는 ACTIVE 슬롯이 없다"
    meta = json.load(open(slots[0] / "ONNX_META.json", encoding="utf-8"))
    mj = csp._mjlab_of(meta)
    ck = csp._ckpt_abs(mj, meta.get("mode1_ckpt"))
    if not ck or not os.path.exists(os.path.join(os.path.dirname(ck), "training_meta.json")):
        return "학습 원장(run 의 training_meta.json)이 이 머신에 없다"
    return None


def _clone(src_root: Path, td: str) -> Path:
    dst = Path(td) / src_root.name
    (dst / "params").mkdir(parents=True)
    (dst / "exported").mkdir()
    shutil.copy2(src_root / "params/deploy.yaml", dst / "params/deploy.yaml")
    shutil.copy2(src_root / "ONNX_META.json", dst / "ONNX_META.json")
    os.symlink(os.path.realpath(src_root / "exported/policy.onnx"), dst / "exported/policy.onnx")
    return dst


def _edit_yaml(root: Path, fn):
    p = root / "params/deploy.yaml"
    d = yaml.safe_load(open(p, encoding="utf-8"))
    fn(d)
    with open(p, "w", encoding="utf-8") as f:
        yaml.safe_dump(d, f, sort_keys=False, allow_unicode=True)


def _rows(rep):
    return {r["id"]: r for r in rep["rows"]}


def _expect_fail(rep, rid):
    r = _rows(rep).get(rid)
    assert r is not None, "행 %s 가 없다" % rid
    assert r["verdict"] == "FAIL", "%s 가 FAIL 이어야 하는데 %s — %s" % (rid, r["verdict"], r["note"])
    assert rep["verdict"] == "FAIL", "슬롯 판정이 FAIL 이어야 한다: %s" % rep["verdict"]


def _base():
    return _active_slots()[0]


# ── 기준선 ───────────────────────────────────────────────────────────────
def test_baseline_active_slots_known_k1_k2():
    """지금 ACTIVE 슬롯 전부: FAIL 0, KNOWN 은 정확히 K1(gait.mode2)·K2(cmd.mode2)."""
    for src in _active_slots():
        with tempfile.TemporaryDirectory() as td:
            rep = csp.check_slot(str(_clone(src, td)))
            fails = [(r["id"], r["note"]) for r in rep["rows"] if r["verdict"] == "FAIL"]
            assert not fails, "%s: FAIL %s" % (src.name, fails)
            known = {r["id"] for r in rep["rows"] if r["verdict"] == "KNOWN"}
            assert known == {"gait.mode2", "cmd.mode2"}, "%s: KNOWN 행 %s" % (src.name, known)
            assert {e["id"] for e in rep["known_used"]} == {"K1", "K2"}, rep["known_used"]
            assert rep["verdict"] == "KNOWN", rep["verdict"]
            # 계측기 자기검증 행이 실제로 돌았다 (안 돌면 «없음» 이 아니라 FAIL 이어야 한다)
            assert _rows(rep)["gait.reader"]["verdict"] == "PASS"
            assert _rows(rep)["tierb.stamp"]["verdict"] == "SKIP" and "Tier B 미도입" in _rows(rep)["tierb.stamp"]["note"]


# ── 변이시험: 고장을 심으면 그 행이 FAIL ────────────────────────────────────
def test_swap_29_wide_terms_fails():
    """joint_pos_rel ↔ joint_vel_rel 교환 — 둘 다 29×10 이라 합(1640)은 그대로. 크기 검사로는 못 잡는 사고."""
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)

        def swap(d):
            obs = d["observations"]
            items = list(obs.items())
            i = [k for k, _ in items].index("joint_pos_rel")
            j = [k for k, _ in items].index("joint_vel_rel")
            items[i], items[j] = items[j], items[i]
            d["observations"] = dict(items)
        _edit_yaml(root, swap)
        rep = csp.check_slot(str(root))
        _expect_fail(rep, "obs.joint_pos")
        _expect_fail(rep, "obs.joint_vel")
        assert _rows(rep)["obs.sum"]["verdict"] == "PASS"       # 합은 그대로라는 것까지 확인


def test_kp_plus_1pct_fails():
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        _edit_yaml(root, lambda d: d["stiffness"].__setitem__(0, d["stiffness"][0] * 1.01))
        _expect_fail(csp.check_slot(str(root)), "act.kp")


def test_cadence_change_fails():
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        _edit_yaml(root, lambda d: d["gait"]["mode1"].__setitem__("cadence", 1.2))
        _expect_fail(csp.check_slot(str(root)), "gait.mode1")


def test_removed_train_term_fails():
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        _edit_yaml(root, lambda d: d["observations"]["base_vel_command"].pop("train_term"))
        _expect_fail(csp.check_slot(str(root)), "obs.base_vel")


def test_missing_training_meta_fails():
    """head 의 run 에 training_meta.json 이 없으면 «어느 코드로 학습됐는지 모름» = FAIL (통과로 치지 않는다)."""
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        meta = json.load(open(root / "ONNX_META.json", encoding="utf-8"))
        real_run = os.path.dirname(csp._ckpt_abs(csp._mjlab_of(meta), meta["mode1_ckpt"]))
        fake_run = Path(td) / "fake_run" / os.path.basename(real_run)
        fake_run.mkdir(parents=True)
        for f in ("train_args.json", "motion.txt"):                # training_meta.json 만 빼고 옮긴다
            if os.path.exists(os.path.join(real_run, f)):
                shutil.copy2(os.path.join(real_run, f), fake_run / f)
        meta["mode1_ckpt"] = str(fake_run / os.path.basename(meta["mode1_ckpt"]))
        json.dump(meta, open(root / "ONNX_META.json", "w", encoding="utf-8"), ensure_ascii=False)
        rep = csp.check_slot(str(root))
        _expect_fail(rep, "src.mode1")
        assert "training_meta.json 없음" in _rows(rep)["src.mode1"]["note"]


def test_unreadable_onnx_is_fail_not_pass():
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        os.remove(root / "exported/policy.onnx")
        rep = csp.check_slot(str(root))
        _expect_fail(rep, "id.onnx")


def test_known_values_must_match_and_stale_entries_warn():
    """허용목록은 «같은 값» 일 때만 KNOWN. 값이 다르면 FAIL, 사라진 불일치의 항목은 WARN(지울 것)."""
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        known = yaml.safe_load(open(csp.KNOWN_DEFAULT, encoding="utf-8"))
        for e in known["known"]:
            if e["id"] == "K2":
                e["deploy"] = [-1.5, 3.0]                          # 실제(+2.5)와 다르게 적힌 항목
        known["known"].append(dict(id="K9", row="cmd.mode1", key="vx", train=[-1.0, 1.0], deploy=[-1.0, 1.0],
                                   reason="시험용 — 지금은 없는 불일치"))
        kp = Path(td) / "known.yaml"
        yaml.safe_dump(known, open(kp, "w", encoding="utf-8"), allow_unicode=True, sort_keys=False)
        rep = csp.check_slot(str(root), known_path=str(kp))
        _expect_fail(rep, "cmd.mode2")
        assert _rows(rep)["gait.mode2"]["verdict"] == "KNOWN"
        assert _rows(rep)["known.K9"]["verdict"] == "WARN", _rows(rep).get("known.K9")


def test_outputs_written_only_to_given_slot_and_gate_sees_them():
    """PARITY.* 는 넘겨준 슬롯 폴더에만 쓰이고, 내용이 같으면 다시 안 쓴다. policy_slot 게이트가 그 파일을 읽는다."""
    with tempfile.TemporaryDirectory() as td:
        root = _clone(_base(), td)
        rep = csp.check_slot(str(root))
        assert csp.write_outputs(str(root), rep) is True
        assert (root / "PARITY.json").exists() and (root / "PARITY.md").exists()
        assert csp.write_outputs(str(root), csp.check_slot(str(root))) is False     # 시각만 다르다 → 안 씀
        st, detail, _ = policy_slot.parity_status(root.name, td)
        assert st == "KNOWN", (st, detail)
        _edit_yaml(root, lambda d: d.__setitem__("step_dt", 0.02))                 # 값은 같아도 파일이 바뀌었다
        st, detail, _ = policy_slot.parity_status(root.name, td)
        assert st == "STALE", (st, detail)


_REAL_BEFORE = {}


def _snapshot():
    out = {}
    for d in sorted(Path(policy_slot.SLOTDIR).iterdir()):
        for f in ("PARITY.json", "PARITY.md"):
            p = d / f
            out[str(p)] = (p.stat().st_mtime_ns, p.stat().st_size) if p.exists() else None
    return out


# 기준값은 모듈을 읽을 때 잡는다 — 스크립트 실행(run_unit_tests.sh)만 아래 __main__ 에서 잡으면
# pytest 로 돌릴 때 빈 dict 와 비교해 진짜 슬롯이 멀쩡해도 실패한다.
_REAL_BEFORE.update(_snapshot())


def test_zz_real_slots_untouched():
    """이 파일의 어떤 시험도 진짜 슬롯의 PARITY.* 를 만들거나 바꾸지 않았다."""
    assert _snapshot() == _REAL_BEFORE


if __name__ == "__main__":
    why = _skip_reason()
    if why:
        print("skip test_slot_parity: %s" % why)
        sys.exit(0)
    _REAL_BEFORE.update(_snapshot())
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
