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


def _render_list(ctx, cfg, duration_cache: dict) -> None:
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
        # IMPORTANT-8: load_clip 은 body_quat_w/lin_vel_w/ang_vel_w 까지 통째로 풀어 수십 MB
        # 를 디코딩한다 — 목록을 볼 때마다(특히 'l' 재입력마다) 24클립을 매번 다시 열 이유가
        # 없다. 파일 경로로 한 번 계산해두고 재사용한다. 이름(c.name)으로 키를 잡으면
        # COLMO/walk/walk1_subject2.npz 와 COLMOv2/walk/walk1_subject2.npz 처럼 stem 이
        # 같고 파일이 다른 클립이 서로의 duration 을 훔쳐 보여준다.
        if c.path not in duration_cache:
            try:
                duration_cache[c.path] = clips_mod.load_clip(c).duration
            except Exception:
                duration_cache[c.path] = None
        d = duration_cache[c.path]
        length = f"{d:.1f}s" if d is not None else "?"
        print(f"  {i:>3}  {c.name:<24} {length:>9}  "
              f"{','.join(str(m) for m in c.modes):<7} {tag}")
    print("\n> <번호><프리셋문자>   예: 1a")
    print("> <번호> <시작초> <길이초> [x속도] [m모드]   예: 1 40 15 x0.5 m2")
    print("> l · q   목록 · 종료\n")


def _confirm(clip, pre, item, modes_unknown: bool = False) -> bool:
    """프리플라이트 결과를 출력한다. 반환값: 이 구간을 재생 준비(PlaybackSpec 구성)해도 되는가.

    ⚠ dry-run 여부는 여기서 판단하지 않는다(G11) — dry-run 이라도 프리플라이트를 통과하면
    True 를 반환해 호출부가 PlaybackSpec/plan_frames 까지는 실제로 돌리게 한다. Publisher
    구성 여부(=실제 송출 여부)는 호출부(main)가 args.dry_run 으로 따로 가른다.
    """
    wall = (pre.f_end - pre.f_start) / clip.fps / item.speed
    clip_s = (pre.f_end - pre.f_start) / clip.fps
    print(f"\n▶ {clip.name}   {item.span[0]:.1f}–{item.span[1]:.1f}s "
          f"({clip_s:.1f}s, ×{item.speed:.2f} → 벽시계 {wall:.1f}s)   "
          f"mode{item.mode} · base_vel={item.base_vel}")
    if modes_unknown:
        print("  ⚠ 이 정책 슬롯은 모드별 head 유무를 알 수 없습니다 (ONNX_META.json 없음) — "
              "클립에 적힌 modes 를 그대로 신뢰합니다.")
    print(f"  진입 자세차 {pre.entry_err:.2f} rad (joint[{pre.entry_joint}]) "
          f"→ 램프인 {pre.ramp_in_s:.1f}s")
    if not pre.ok:
        for r in pre.reasons:
            print(f"  ✗ {r}")
        print("  재생하지 않습니다.\n")
        return False
    print("  관절한계 위반 없음 · NaN 없음 · 매니페스트 ✓")
    return True


class _KeyWatcher:
    """재생 중 Space(중단) / x(E-stop) 을 논블로킹으로 본다. 터미널 상태를 반드시 원복한다.

    Ctrl-C(SIGINT)도 여기서 가로챈다: cbreak 모드에서도 ISIG 는 켜져 있어서 Ctrl-C 가
    os.read() 로 바이트(\\x03)로 오는 게 아니라 KeyboardInterrupt 로 곧장 올라간다 —
    그러면 p.run() 이 통째로 끊겨 RAMP_OUT/RELEASE 를 건너뛰고 참조가 중간 자세에
    멈춘 채로 C++ 워치독(stale 감지)에 떠넘겨진다. 그래서 재생 중엔 SIGINT 핸들러를 걸어
    Space 와 같은 경로(abort 플래그)로 받는다. 두 번째 Ctrl-C 는 탈출구로 남겨 진짜 멈춘
    재생을 강제 종료할 수 있게 한다.

    IMPORTANT-7: SIGTERM 도 spec §9 상 SIGINT 와 같은 급의 "operator abort" 다 — 모듈
    스코프 핸들러(파일 맨 아래)는 sys.exit(0) 으로 즉시 종료하는데, 재생 중에 그게 걸리면
    p.run() 이 SystemExit 로 끊겨 RAMP_OUT/RELEASE 를 건너뛴다(SIGINT 미수정 시절과 동일한
    버그). 그래서 재생 중엔 SIGINT 와 동일하게 abort 플래그로 받고, __exit__ 에서 이전
    핸들러(모듈 스코프 핸들러)로 복원해 비재생 구간에서는 여전히 즉시 종료되게 한다.
    """

    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else -1
        self.saved = None
        self.saved_sigint = None
        self.saved_sigterm = None
        self.abort = False
        self.estop = False
        self._sigint_hits = 0

    def __enter__(self):
        if self.enabled:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            os.set_blocking(self.fd, False)
            self.saved_sigint = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, self._on_sigint)
            self.saved_sigterm = signal.getsignal(signal.SIGTERM)
            signal.signal(signal.SIGTERM, self._on_sigterm)
        return self

    def __exit__(self, *exc):
        if self.saved_sigint is not None:
            signal.signal(signal.SIGINT, self.saved_sigint)
        if self.saved_sigterm is not None:
            signal.signal(signal.SIGTERM, self.saved_sigterm)
        if self.saved is not None:
            os.set_blocking(self.fd, True)
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def _on_sigint(self, signum, frame):
        self._sigint_hits += 1
        if self._sigint_hits >= 2:
            # 탈출구: 두 번째 Ctrl-C 는 강제 종료. 기본 핸들러로 되돌리고 다시 던진다 —
            # with 문의 예외 경로로 __exit__ 가 호출되어 터미널/시그널은 그대로 원복된다.
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            raise KeyboardInterrupt
        self.abort = True

    def _on_sigterm(self, signum, frame):
        # SIGKILL/하드크래시만 4층(C++ stale 워치독)에 맡긴다(spec §9) — SIGTERM 은 여기서
        # 반드시 RAMP_OUT 경로를 타야 하므로 두 번째 신호를 기다리는 탈출구를 두지 않는다.
        self.abort = True

    def poll(self) -> bool:
        if not self.enabled:
            return self.abort
        try:
            ch = os.read(self.fd, 1)
        except (BlockingIOError, OSError):
            return self.abort
        if ch == b" ":
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
    # IMPORTANT-5: valid_modes 가 비어(미상) 있어도 mode2(저위험)를 기본값으로 한다 —
    # mode3(전신) 이 기본이 되면 근거가 가장 부족한 슬롯이 가장 공격적인 모드로 재생된다.
    default_mode = playlist_mod.default_mode_for(ctx.valid_modes)

    estop_mod = None
    if args.arm_estop and not args.dry_run:
        import estop_shm
        estop_mod = estop_shm

    duration_cache: dict = {}          # IMPORTANT-8: 클립 경로 -> duration. 'l' 재입력마다 재로딩 방지.
    _render_list(ctx, cfg, duration_cache)
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
                _render_list(ctx, cfg, duration_cache)
                continue
            if kind == "error":
                print(f"  ✗ {payload}")
                continue

            item = payload
            # IMPORTANT-6: clip_index 가 있으면(모든 정상 파싱 경로가 채운다) 이름 재탐색을
            # 하지 않는다 — COLMO/COLMOv2 처럼 이름이 같은 클립이 있으면 next(...)가 사용자가
            # 타이핑한 번호와 무관하게 목록의 첫 매치를 골라버린다.
            if item.clip_index is not None:
                info = ctx.clips[item.clip_index]
            else:
                info = next(c for c in ctx.clips if c.name == item.clip_name)
            try:
                clip = clips_mod.load_clip(info)
            except Exception as e:
                print(f"  ✗ {info.name} 로드 실패: {e}")
                continue
            modes_unknown = not ctx.valid_modes
            allowed = tuple(sorted(set(info.modes) & (ctx.valid_modes or set(info.modes))))
            pre = clips_mod.preflight(clip, item.span, item.mode, allowed, profile)
            if not _confirm(clip, pre, item, modes_unknown=modes_unknown):
                continue

            spec = pub_mod.PlaybackSpec(
                clip=clip, profile=profile, f_start=pre.f_start, f_end=pre.f_end,
                mode=item.mode, speed=item.speed, base_vel_kind=item.base_vel,
                manual_bv=item.manual_bv, ramp_in_s=pre.ramp_in_s)

            if args.dry_run:
                # G11: dry-run = "shm 에 쓰지 않고 프레임 계산만" — Publisher/vr_shm 은 이
                # 경로에서 절대 구성/임포트하지 않되, plan_frames 는 실제로 소비해 구간·
                # 속도·T_in 산정이 실제로 몇 프레임/몇 초짜리 재생이 되는지 검증한다.
                n = 0
                total_t = 0.0
                for t_rel, _frame in pub_mod.plan_frames(spec):
                    n += 1
                    total_t = t_rel
                print(f"  [dry-run] {n} 프레임 계산됨 · 총 {total_t:.2f}s (송출하지 않음)\n")
                continue

            ans = input("  [Enter] 재생   [그 외] 취소 > ")
            if ans != "":
                continue

            with _KeyWatcher(enabled=True) as keys:
                def on_tick(t_rel, frame):
                    nonlocal seq_estop
                    if estop_mod is not None:
                        seq_estop += 1
                        estop_mod.write(seq_estop, 1 if keys.estop else 0)
                    print(f"\r  ▶ {clip.name} {t_rel:6.2f}s  mode{frame.cmd_mode}  "
                          f"Space=중단 x=E-stop ", end="", flush=True)

                p = pub_mod.Publisher()
                try:
                    result = p.run(spec, on_tick=on_tick, should_abort=keys.poll)
                finally:
                    # IMPORTANT-4: 하트비트는 재생 구간에서만 나간다(위 on_tick) — 재생이
                    # 끝나는 즉시 무장 해제해서 "무장은 됐는데 하트비트가 안 나가는" 창을
                    # 만들지 않는다. 그 창이 열리면 C++ 워치독이 ~0.5s 후 죽은 writer 로
                    # 보고 강제 Passive 로 넘어가 다음 명령 입력을 기다리는 동안 로봇이
                    # 주저앉는다.
                    if estop_mod is not None:
                        estop_mod.clear()
            print(f"\n  {result}"
                  + ("  (E-stop 발동 — g1_ctrl 이 Passive 로 갔습니다. f 로 재기립)"
                     if keys.estop else ""))
    finally:
        if estop_mod is not None:
            estop_mod.clear()
    print("bye")
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    sys.exit(main())
