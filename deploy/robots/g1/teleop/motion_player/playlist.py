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
        if not span:
            label = str(e.get("label") or e.get("clip") or "")
            raise ValueError(f"preset '{label}' 에 span 이 없습니다 — 구간은 필수입니다")
        presets.append(PlayItem(
            clip_name=e["clip"],
            span=(float(span[0]), float(span[1])),
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
