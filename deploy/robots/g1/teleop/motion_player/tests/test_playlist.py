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

# IMPORTANT-6: COLMO/COLMOv2 처럼 서로 다른 npz 가 같은 stem("walk1_subject2")을 쓰는 경우.
DUP_CLIPS = [resolver.ClipInfo(name="walk1_subject1", path=Path("/x/colmo/walk1_subject1.npz"),
                               modes=(1, 2, 3)),
             resolver.ClipInfo(name="walk1_subject2", path=Path("/x/colmo/walk1_subject2.npz"),
                               modes=(1, 2, 3)),
             resolver.ClipInfo(name="walk1_subject2", path=Path("/x/colmov2/walk1_subject2.npz"),
                               modes=(1, 2, 3))]

DUP_YAML = """
motion_root: /home/piene/mjlab1.4/mjlab_g1_motion
slot_overrides: {}
presets:
  - clip: walk1_subject2
    label: "중복 이름"
    span: [10.0, 20.0]
    mode: 2
    base_vel: clip
    speed: 1.0
"""

YAML_MANUAL_OK = """
motion_root: /home/piene/mjlab1.4/mjlab_g1_motion
slot_overrides: {}
presets:
  - clip: walk1_subject1
    label: "수동 base_vel"
    span: [0.0, 5.0]
    mode: 2
    base_vel: manual
    base_vel_manual: [0.3, 0.0, 0.1]
    speed: 1.0
"""

YAML_MANUAL_MISSING_VEC = """
motion_root: /home/piene/mjlab1.4/mjlab_g1_motion
slot_overrides: {}
presets:
  - clip: walk1_subject1
    label: "수동인데 벡터 없음"
    span: [0.0, 5.0]
    mode: 2
    base_vel: manual
    speed: 1.0
"""


def _cfg_from(text: str):
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "presets.yaml"
        p.write_text(text)
        return playlist.load_config(p)


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
    assert item.clip_index == 0, item.clip_index
    kind, item = playlist.parse_command("1b", CLIPS, cfg, default_mode=3)
    assert item.span == (88.0, 103.0) and item.speed == 0.5 and item.mode == 3
    assert item.clip_index == 0, item.clip_index
    print("  ok parse_preset_letter")


def test_parse_arbitrary_span():
    cfg = _cfg()
    kind, item = playlist.parse_command("1 40 15", CLIPS, cfg, default_mode=3)
    assert kind == "play"
    assert item.span == (40.0, 55.0), item.span
    assert item.mode == 3 and item.speed == 1.0
    assert item.clip_index == 0, item.clip_index
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


def test_parse_arbitrary_span_mode2_defaults_base_vel_zero():
    """G10: spec §13 단계2 는 mode2 를 base_vel:zero 로 먼저 시도해야 한다 — clip 로 바로
    가면 그 단계를 CLI 로 밟을 방법이 없어진다."""
    cfg = _cfg()
    _, item = playlist.parse_command("1 40 15 m2", CLIPS, cfg, default_mode=3)
    assert item.mode == 2 and item.base_vel == "zero", item.base_vel
    print("  ok arbitrary_span_mode2_base_vel_zero")


def test_default_mode_for():
    assert playlist.default_mode_for({2, 3}) == 2
    assert playlist.default_mode_for({2}) == 2
    assert playlist.default_mode_for({3}) == 3
    assert playlist.default_mode_for(set()) == 2       # 미상(ONNX_META 없음) -> 저위험 기본값
    print("  ok default_mode_for")


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


def test_parse_arbitrary_span_resolves_correct_index_when_name_duplicated():
    """IMPORTANT-6: 이름이 중복돼도 임의구간 커맨드는 타이핑한 번호 그대로 인덱스를 못박는다."""
    cfg = _cfg_from(DUP_YAML)
    kind, item = playlist.parse_command("3 40 15", DUP_CLIPS, cfg, default_mode=3)
    assert kind == "play", (kind, item)
    assert item.clip_index == 2, item.clip_index      # DUP_CLIPS[2] = COLMOv2 walk1_subject2
    print("  ok arbitrary_span_correct_index_when_duplicated")


def test_parse_preset_letter_rejects_ambiguous_clip_name():
    """IMPORTANT-6: 프리셋은 이름으로만 저장되므로, 그 이름이 중복이면 어느 클립 기준인지
    모호하다 — 하나를 임의로 골라 재생하지 않고 명확한 에러로 거부해야 한다."""
    cfg = _cfg_from(DUP_YAML)
    kind, msg = playlist.parse_command("2a", DUP_CLIPS, cfg, default_mode=3)
    assert kind == "error", (kind, msg)
    assert "모호" in msg, msg
    kind, msg = playlist.parse_command("3a", DUP_CLIPS, cfg, default_mode=3)
    assert kind == "error", (kind, msg)
    assert "모호" in msg, msg
    print("  ok preset_letter_rejects_ambiguous_name")


def test_load_config_manual_base_vel():
    """G9: base_vel: manual 프리셋은 base_vel_manual 벡터를 PlayItem.manual_bv 로 읽어야 한다."""
    cfg = _cfg_from(YAML_MANUAL_OK)
    assert len(cfg.presets) == 1
    p = cfg.presets[0]
    assert p.base_vel == "manual"
    assert p.manual_bv == (0.3, 0.0, 0.1), p.manual_bv
    print("  ok load_config_manual_base_vel")


def test_load_config_manual_base_vel_without_vector_raises():
    """G9: base_vel: manual 인데 base_vel_manual 이 없으면 조용히 zero 로 동작하지 않고
    명확한 에러로 거부해야 한다."""
    try:
        _cfg_from(YAML_MANUAL_MISSING_VEC)
        assert False, "should have raised ValueError"
    except ValueError as e:
        assert "base_vel_manual" in str(e), str(e)
    print("  ok manual_base_vel_missing_vector_raises")


def main() -> int:
    test_load_config()
    test_load_config_missing_span_raises()
    test_presets_for()
    test_parse_preset_letter()
    test_parse_arbitrary_span()
    test_parse_speed_and_mode_flags()
    test_parse_arbitrary_span_mode2_defaults_base_vel_zero()
    test_default_mode_for()
    test_parse_rejects_mode1_with_reason()
    test_parse_rejects_bad_input()
    test_parse_list_and_quit()
    test_parse_arbitrary_span_resolves_correct_index_when_name_duplicated()
    test_parse_preset_letter_rejects_ambiguous_clip_name()
    test_load_config_manual_base_vel()
    test_load_config_manual_base_vel_without_vector_raises()
    print("test_playlist: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
