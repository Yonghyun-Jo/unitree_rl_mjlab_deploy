"""Playlist — presets.yaml 로드와 CLI 문법 파싱. 파일도 shm 도 만지지 않는다(설정 읽기 제외)."""
from __future__ import annotations

import re
from dataclasses import dataclass, field, replace
from pathlib import Path

import yaml

from .resolver import ClipInfo

_MODE1_REASON = (
    "mode1 재생 불가 — 참조가 상·하체 전부 마스킹돼 정책에 도달하지 않는다 "
    "(masked_joint_command / masked_root_ori_b 가 0). mode2 또는 mode3 을 쓰세요.")


@dataclass
class PlayItem:
    clip_name: str
    span: tuple[float, float] | None      # 실제로는 필수 — load_config 이 부재를 거부한다
    mode: int
    base_vel: str                         # "clip" | "zero" | "manual"
    speed: float
    label: str = ""
    # IMPORTANT-6: 클립은 이름(stem)이 아니라 이 인덱스로 식별한다 — COLMO/COLMOv2 처럼
    # 서로 다른 npz 가 같은 stem 을 쓰면 이름만으로는 어느 클립인지 모호하다.
    clip_index: int | None = None
    # G9: base_vel == "manual" 일 때 쓸 고정 [vx, vy, wz]. base_vel != "manual" 이면 무시.
    manual_bv: tuple[float, float, float] = (0.0, 0.0, 0.0)


def default_mode_for(valid_modes: set[int]) -> int:
    """IMPORTANT-5: mode2 가 저위험 첫 시도(spec §10)다 — valid_modes 가 비어 미상이어도
    mode2 를 기본값으로 한다. mode3 만 확실히 알려진 경우에만 mode3 을 기본값으로 한다."""
    if not valid_modes or 2 in valid_modes:
        return 2
    return 3


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
        base_vel = str(e.get("base_vel", "zero"))
        manual_bv = (0.0, 0.0, 0.0)
        if base_vel == "manual":
            # G9: base_vel: manual 은 base_vel_manual: [vx, vy, wz] 이 반드시 같이 있어야
            # 한다 — 없으면 조용히 zero 로 동작하던 것(버그)을 명시적으로 거부한다.
            raw_mbv = e.get("base_vel_manual")
            if not raw_mbv or len(raw_mbv) != 3:
                label = str(e.get("label") or e.get("clip") or "")
                raise ValueError(
                    f"preset '{label}' 은 base_vel: manual 인데 "
                    f"base_vel_manual: [vx, vy, wz] 가 없습니다")
            manual_bv = (float(raw_mbv[0]), float(raw_mbv[1]), float(raw_mbv[2]))
        presets.append(PlayItem(
            clip_name=e["clip"],
            span=(float(span[0]), float(span[1])),
            mode=int(e.get("mode", 3)),
            base_vel=base_vel,
            speed=float(e.get("speed", 1.0)),
            label=str(e.get("label", "")),
            manual_bv=manual_bv))
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
        # IMPORTANT-6: 프리셋은 클립 이름으로만 저장돼 있다. 같은 이름의 클립이 목록에
        # 둘 이상이면(COLMO/COLMOv2 처럼) 이 프리셋의 구간이 어느 파일 기준인지 이름만
        # 으로는 알 수 없다 — 추측으로 하나를 고르지 않고 거부한다.
        dup = sum(1 for c in clips if c.name == name)
        if dup > 1:
            return "error", (
                f"'{name}' 이름이 클립 {dup}개에 중복됩니다 — 프리셋은 이름으로만 저장돼 "
                f"어느 클립 기준인지 모호합니다. '<번호> <시작초> <길이초>' 형식으로 "
                f"직접 구간을 지정하세요.")
        ps = presets_for(cfg, name)
        k = ord(letter) - ord("a")
        if not (0 <= k < len(ps)):
            return "error", f"{name} 에 프리셋 '{letter}' 가 없음 (프리셋 {len(ps)}개)"
        # 프리셋은 이름만 갖고 있으므로, 사용자가 실제로 타이핑한 번호(idx)를
        # clip_index 로 못박아 CLI 가 이름으로 재탐색하지 않게 한다.
        return "play", replace(ps[k], clip_index=idx - 1)

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

    # G10: 임의구간 커맨드는 mode2 라도 base_vel: zero 를 기본값으로 한다(spec §13 단계2 순서
    # — zero 를 먼저 검증하고 나서 clip 로 올라간다). base_vel: clip 이 필요하면 프리셋으로
    # 명시하세요.
    return "play", PlayItem(clip_name=clips[idx - 1].name, span=(start, start + dur),
                            mode=mode, base_vel="zero", speed=speed, clip_index=idx - 1)
