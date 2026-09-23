#!/usr/bin/env python3
"""masked_gui.py 의 mode5 자세 버튼이 «전부 눌리는가» — 줄당 몇 개로 끊겼는지까지.

viser 의 button_group 은 한 줄로만 편다. 패널 폭을 넘으면 뒤쪽 버튼이 화면 밖으로 잘리고
스크롤 막대도 안 나와서, 운용자에겐 «없는 버튼» 이 된다 (2026-09-23: «드러누움(n)» 이 안 보였다).
그래서 여기선 GUI 를 띄우지 않고 viser 를 가짜로 바꿔 main() 을 한 번 돌려 다음을 본다:
  ① 키 있는 자세가 빠짐없이 · 한 번씩 나온다        (하나라도 빠지면 그 자세는 GUI 로 못 간다)
  ② 한 줄에 masked_gui.M5_BTNS_PER_ROW 개 이하      (한 줄로 되돌아가면 다시 잘린다)
  ③ 마지막 줄의 버튼을 눌러도 m5_preset = 표 index + 1 · 누름 번호가 는다  (줄을 나누며 번호가 밀리지 않았나)

🔴 /dev/shm 을 안 건드린다 — gui_shm.SHM_PATH 를 임시 파일로 돌린 뒤 실행한다(도는 로봇·제어기를 방해하지 않게).
"""
import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "..", "tools")
sys.path.insert(0, TOOLS)


class _Done(Exception):
    """main() 의 while True 를 빠져나오는 신호."""


def _unpack(gui_shm) -> dict:
    """제어기가 읽는 그대로 — shm 파일을 gui_shm.FMT 로 푼다 (자리 = write() 의 pack 순서)."""
    import struct
    v = struct.unpack(gui_shm.FMT, open(gui_shm.SHM_PATH, "rb").read())
    return {"magic": v[0], "seq": v[1], "mode_req": v[2], "m5_preset": v[9],
            "m5_press_seq": v[10], "clip_req": v[11]}


class _Handle:
    def __init__(self, label=None, options=None):
        self.label, self.options, self.content, self.value = label, options, "", None
        self.cbs = []

    def on_click(self, fn):
        self.cbs.append(fn)
        return fn

    on_update = on_trigger = on_click

    def click(self, option):
        """그 버튼을 누른 것처럼 — viser 는 누른 라벨을 target.value 로 준다."""
        self.value = option
        ev = types.SimpleNamespace(target=self)
        for fn in self.cbs:
            fn(ev)


class _Folder:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class _Gui:
    def __init__(self):
        self.groups = []            # [(label, options, handle)] — 낸 순서대로

    def add_folder(self, *a, **k):
        return _Folder()

    def add_markdown(self, *a, **k):
        return _Handle()

    def add_slider(self, *a, **k):
        return _Handle()

    def add_button(self, *a, **k):
        return _Handle()

    def add_command(self, *a, **k):
        return _Handle()

    def add_button_group(self, label, options, **k):
        h = _Handle(label, tuple(options))
        self.groups.append((label, tuple(options), h))
        return h


SERVERS = []


class _Server:
    def __init__(self, *a, **k):
        self.gui = _Gui()
        SERVERS.append(self)


def main() -> int:
    sys.modules["viser"] = types.SimpleNamespace(ViserServer=_Server)
    import masked_gui
    import gui_shm
    from mode5_presets_gen import PRESETS

    with tempfile.TemporaryDirectory() as tmp:
        gui_shm.SHM_PATH = os.path.join(tmp, "g1_masked_gui")   # 🔴 진짜 /dev/shm 을 안 건드린다
        masked_gui.time = types.SimpleNamespace(               # while True: time.sleep(1) 에서 빠져나온다
            sleep=lambda *_: (_ for _ in ()).throw(_Done()),
            time=lambda: 0.0)
        try:
            masked_gui.main()
        except _Done:
            pass
        else:
            print("FAIL: main() 이 끝까지 안 갔다 (while True 에 도달하지 못했다)")
            return 1

        if len(SERVERS) != 1:
            print(f"FAIL: ViserServer 를 {len(SERVERS)} 번 만들었다 (가짜 viser 가 안 꽂혔을 수 있다)")
            return 1
        g = SERVERS[0].gui
        # 🔴 상한은 여기 «고정» 한다 — masked_gui.M5_BTNS_PER_ROW 와 비교하면 그 값을 키우는 순간
        #    (= 버그를 되돌리는 순간) 테스트도 같이 늘어나 아무것도 막지 못한다.
        #    4 = 실측: 09-23 화면에서 이 라벨이 4개까지 들어가고 5번째부터 잘렸다. 지금 설정은 3(여유 한 칸).
        MAX_PER_ROW = 4
        per_row = masked_gui.M5_BTNS_PER_ROW
        keyed = [(index, name, key) for index, name, key, _n in PRESETS if key]

        # mode5 자세 그룹 = 자세 라벨을 담은 그룹 전부 (모드·클립 그룹과 섞이지 않게 라벨 내용으로 고른다)
        names = {name for _i, name, _k in keyed}
        rows = [(label, opts, h) for label, opts, h in g.groups
                if opts and any(o.split(" (")[0] in names for o in opts)]
        if not rows:
            print("FAIL: mode5 자세 버튼 그룹이 없다")
            return 1

        shown = [o for _l, opts, _h in rows for o in opts]
        if len(shown) != len(set(shown)):
            print(f"FAIL: 같은 자세가 두 번 나온다 — {shown}")
            return 1
        for index, name, key in keyed:
            if not any(o.startswith(f"{name} ({key})") for o in shown):
                print(f"FAIL: 자세 «{name} ({key})» 가 GUI 에 없다 — 운용자가 못 누른다")
                return 1
        if per_row > MAX_PER_ROW:
            print(f"FAIL: M5_BTNS_PER_ROW={per_row} > {MAX_PER_ROW} — 패널 폭을 넘어 뒤쪽 자세가 잘린다")
            return 1
        for label, opts, _h in rows:
            if len(opts) > MAX_PER_ROW:
                print(f"FAIL: 한 줄에 {len(opts)} 개 (상한 {MAX_PER_ROW}) — 패널을 넘으면 잘린다: {label!r} {opts}")
                return 1

        # ③ 마지막 줄의 마지막 버튼을 눌러 본다 — 줄을 나누며 번호가 밀렸으면 여기서 걸린다.
        last_label, last_opts, last_h = rows[-1]
        target = last_opts[-1]
        want_name = target.split(" (")[0]
        want_index = next(i for i, name, _k in keyed if name == want_name)
        # 🔴 state 를 보면 안 된다 — gui_shm.write 가 1회성 칸을 보낸 «뒤에» 0 으로 되돌린다.
        #    제어기가 실제로 읽는 것은 파일이므로 파일을 푼다(칸 순서 = gui_shm.FMT).
        before = _unpack(gui_shm)
        last_h.click(target)
        got = _unpack(gui_shm)
        if got["m5_preset"] != want_index + 1:
            print(f"FAIL: «{target}» 를 눌렀는데 shm m5_preset={got['m5_preset']} "
                  f"(표 index {want_index} → {want_index + 1} 이어야)")
            return 1
        if got["m5_press_seq"] == before["m5_press_seq"]:
            print("FAIL: 누름 번호(m5_press_seq)가 안 늘었다 — 같은 자세 재입력이 새 목표가 안 된다")
            return 1

    print(f"ok: 자세 {len(shown)}개 / {len(rows)}줄 (줄당 ≤ {per_row}), "
          f"마지막 버튼 «{target}» → m5_preset {got['m5_preset']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
