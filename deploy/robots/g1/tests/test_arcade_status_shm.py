#!/usr/bin/env python3
"""arcade_status_shm.read_names() 시험 — 이름표 파일(g1_ctrl 이 기동 때 한 번 씀)을 읽는다. 없거나 깨지면 None.
  python -m pytest deploy/robots/g1/tests/test_arcade_status_shm.py
"""
import json
import sys
from pathlib import Path

G1 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(G1 / "tools"))
import arcade_status_shm  # noqa: E402


def test_read_names_roundtrip(tmp_path):
    names = {"proto": 2, "boot_id": "123456789", "slot": "s1", "bundle": "abc", "roles": ["walk", "jump"],
             "skills": ["a", "b"], "notices": {"0": "", "1": "busy: event ignored"}}
    p = tmp_path / "names.json"
    p.write_text(json.dumps(names))
    assert arcade_status_shm.read_names(str(p)) == names


def test_read_names_missing_is_none(tmp_path):
    assert arcade_status_shm.read_names(str(tmp_path / "nope.json")) is None


def test_read_names_broken_is_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json")
    assert arcade_status_shm.read_names(str(p)) is None
    p.write_text("[1, 2]")   # dict 가 아님
    assert arcade_status_shm.read_names(str(p)) is None
