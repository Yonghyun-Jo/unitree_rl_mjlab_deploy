"""test_cli.py — cli.py 보조 로직 검증(SIGTERM 처리, 클립 목록 캐시). shm/robot 불필요.

⚠ 이 파일은 실제 /dev/shm/g1_vr_ref · g1_estop 을 절대 건드리지 않는다 — 여기서 쓰는
Publisher/estop_shm 은 전부 실제로 구성/호출되지 않는다(SIGTERM/캐시 로직만 검증).
"""
import contextlib
import io
import os
import signal
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from motion_player import cli as cli_mod            # noqa: E402
from motion_player import clips as clips_mod         # noqa: E402
from motion_player import playlist as playlist_mod   # noqa: E402
from motion_player import resolver as resolver_mod   # noqa: E402


def test_sigterm_handler_sets_abort_flag_not_exit():
    """IMPORTANT-7: _on_sigterm 은 (SIGINT 와 동일하게) sys.exit 를 던지지 않고 abort
    플래그만 세워야 한다 — 재생 중 SystemExit 로 끊기면 RAMP_OUT/RELEASE 를 건너뛴다."""
    kw = cli_mod._KeyWatcher(enabled=False)
    assert kw.abort is False
    kw._on_sigterm(signal.SIGTERM, None)     # SystemExit 을 던지면 이 줄에서 곧장 죽는다
    assert kw.abort is True
    print("  ok sigterm_handler_sets_abort")


def test_keywatcher_installs_and_restores_sigterm_via_real_tty():
    """SIGTERM 을 실제로 프로세스에 보내 _KeyWatcher 가 잡는지, __exit__ 후 이전
    핸들러(모듈 스코프의 sys.exit(0))로 정확히 복원되는지 실제 pty 로 검증한다."""
    import pty
    master_fd, slave_fd = pty.openpty()
    fake_stdin = os.fdopen(slave_fd, "rb", buffering=0)
    old_stdin = sys.stdin
    prev_sigterm = signal.getsignal(signal.SIGTERM)
    try:
        sys.stdin = fake_stdin
        with cli_mod._KeyWatcher(enabled=True) as keys:
            assert keys.enabled is True, "pty 는 isatty() 가 True 여야 한다"
            assert keys.abort is False
            os.kill(os.getpid(), signal.SIGTERM)
            for _ in range(50):                 # 시그널 전달 대기(드물게 지연될 수 있음)
                if keys.abort:
                    break
                time.sleep(0.01)
            assert keys.abort is True, "SIGTERM 이 abort 플래그를 세우지 않았다"
        assert signal.getsignal(signal.SIGTERM) == prev_sigterm, \
            "__exit__ 후 이전 SIGTERM 핸들러로 복원되지 않았다"
    finally:
        sys.stdin = old_stdin
        signal.signal(signal.SIGTERM, prev_sigterm)
        with contextlib.suppress(OSError):
            fake_stdin.close()
        with contextlib.suppress(OSError):
            os.close(master_fd)
    print("  ok keywatcher_sigterm_install_and_restore")


def _make_tiny_npz(path: Path, T: int = 10, fps: float = 50.0) -> None:
    jp = np.zeros((T, 29), dtype=np.float32)
    jv = np.zeros((T, 29), dtype=np.float32)
    quat = np.zeros((T, 2, 4), dtype=np.float32)
    quat[:, :, 0] = 1.0
    lin = np.zeros((T, 2, 3), dtype=np.float32)
    ang = np.zeros((T, 2, 3), dtype=np.float32)
    np.savez(path, joint_pos=jp, joint_vel=jv, body_quat_w=quat,
             body_lin_vel_w=lin, body_ang_vel_w=ang, fps=np.array([fps]))


def test_render_list_caches_clip_durations_across_calls():
    """IMPORTANT-8: 'l' 을 두 번 호출해도 load_clip 은 클립당 한 번만 불려야 한다.

    두 클립을 일부러 같은 이름(stem)·다른 경로·다른 길이로 만든다 — COLMO/walk/
    walk1_subject2.npz 와 COLMOv2/walk/walk1_subject2.npz 처럼 매니페스트가 이름이
    같은 클립을 섞을 수 있기 때문이다. 캐시 키가 c.name 이면 두 번째 클립이 첫 번째
    클립의 duration 을 그대로 보여줘도 이름이 다른 fixture 로는 절대 못 잡는다 — 그래서
    이름을 같게 고정해야 이 테스트가 실제로 그 회귀를 잡는다.
    """
    with tempfile.TemporaryDirectory() as d:
        droot = Path(d)
        sub1, sub2 = droot / "COLMO" / "walk", droot / "COLMOv2" / "walk"
        sub1.mkdir(parents=True)
        sub2.mkdir(parents=True)
        p1, p2 = sub1 / "walk1_subject2.npz", sub2 / "walk1_subject2.npz"
        _make_tiny_npz(p1, T=10, fps=50.0)     # duration = 0.2s
        _make_tiny_npz(p2, T=40, fps=50.0)     # duration = 0.8s — p1 과 뚜렷이 다르게
        clip_infos = [resolver_mod.ClipInfo(name="walk1_subject2", path=p1, modes=(1, 2, 3)),
                     resolver_mod.ClipInfo(name="walk1_subject2", path=p2, modes=(1, 2, 3))]
        ctx = resolver_mod.PolicyContext(slot="s", policy_dir=droot, manifest_path=None,
                                         clips=clip_infos, valid_modes={2, 3}, deployable=True,
                                         warnings=[])
        cfg = playlist_mod.PlayerConfig(motion_root=droot, slot_overrides={}, presets=[])

        calls = {"n": 0}
        orig_load_clip = clips_mod.load_clip

        def counting_load_clip(info):
            calls["n"] += 1
            return orig_load_clip(info)

        clips_mod.load_clip = counting_load_clip
        try:
            cache: dict = {}
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                cli_mod._render_list(ctx, cfg, cache)
                cli_mod._render_list(ctx, cfg, cache)
        finally:
            clips_mod.load_clip = orig_load_clip

        assert calls["n"] == len(clip_infos), calls["n"]    # 두 번째 호출은 캐시 히트라 재로딩 없음
        assert set(cache.keys()) == {p1, p2}, cache          # 이름이 아니라 경로로 키가 잡혀야 한다
        assert abs(cache[p1] - 0.2) < 1e-6, cache[p1]
        assert abs(cache[p2] - 0.8) < 1e-6, cache[p2]
        out = buf.getvalue()
        assert "0.2s" in out and "0.8s" in out, out          # 각 행이 자기 자신의 duration 을 보여야 한다
    print("  ok render_list_caches_durations")


def main() -> int:
    test_sigterm_handler_sets_abort_flag_not_exit()
    test_keywatcher_installs_and_restores_sigterm_via_real_tty()
    test_render_list_caches_clip_durations_across_calls()
    print("test_cli: ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
