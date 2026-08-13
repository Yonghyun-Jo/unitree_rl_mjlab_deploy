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
sys.path.insert(0, str(_HERE.parent))          # teleop/ — vr_shm, estop_shm, motion_player 패키지

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
