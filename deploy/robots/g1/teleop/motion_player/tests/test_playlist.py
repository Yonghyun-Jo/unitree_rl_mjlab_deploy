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

YAML_NO_SPAN = """
motion_root: /home/piene/mjlab1.4/mjlab_g1_motion
slot_overrides: {}
presets:
  - clip: walk1_subject1
    label: "구간 없음"
    mode: 2
    base_vel: clip
    speed: 1.0
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


def test_load_config_missing_span_raises():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "presets.yaml"
        p.write_text(YAML_NO_SPAN)
        try:
            playlist.load_config(p)
            assert False, "should have raised ValueError"
        except ValueError as e:
            assert "구간 없음" in str(e), str(e)
    print("  ok missing_span_raises")


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
    test_load_config_missing_span_raises()
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
