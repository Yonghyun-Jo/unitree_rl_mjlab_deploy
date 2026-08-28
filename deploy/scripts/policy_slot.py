#!/usr/bin/env python3
"""policy_slot.py — 배포 슬롯의 «무거운 부분» 을 git 밖에서 관리한다.

# 왜 갈랐나
슬롯 하나가 60 MB(ONNX 37 + npz 26)다. 그걸 git 에 넣으면 **히스토리에서 영원히 안 빠진다** —
워킹트리에서 지워도 `.git` 은 그대로고, 새로 clone 하는 쪽(로봇)은 매번 전부 받는다.
2026-08-28 하루에만 4개 슬롯 241 MB 가 들어갔고 그 시점 `.git` 736 MB 중 581 MB 가 정책 파일이었다.

그래서 한 슬롯을 둘로 나눈다:

    git  = 실험의 «정체»   deploy.yaml · ONNX_META.json  (수십 KB, 영구 보존)
    보관소 = 실험의 «무게»   exported/** · params/*.npz    (60 MB, git 밖)

슬롯을 보관소로 내보내도 **뭐였는지·무엇으로 판정됐는지는 git 에 그대로 남는다.**
지금까지 「어떤 실험인지 명확하지 않다」의 원인은 기록이 없어서가 아니라 기록과 가중치가
같이 사라졌기 때문이다.

# 로봇으로는 rsync
git 이 아니라 rsync 로 보낸다. `--partial --append-verify` 라 노트북 경유 WiFi 가 끊겨도
이어받는다(git 은 처음부터 다시 받는다). 방향은 여전히 com1 → 로봇 단방향이고, 무결성은
robot.sh verify 의 md5 대조가 이미 보고 있다.

# 명령
    policy_slot.py list                 슬롯 전체 (활성 / 보관 / 깨짐)
    policy_slot.py archive <slot>...    가중치를 보관소로, 워킹트리에서 제거
    policy_slot.py restore <slot>...    보관소에서 되살림
    policy_slot.py activate v1=<slot> [v2=<slot> ...]   오늘 무엇을 v1/v2 로 부를지
    policy_slot.py active               지금 별칭
    policy_slot.py push [<slot>...]     로봇으로 rsync (생략하면 활성 슬롯 전부)
    policy_slot.py index                POLICY_INDEX.md 재생성
    policy_slot.py check                무결성 (활성 슬롯에 가중치가 실제로 있는가)
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
POLICY = os.path.join(REPO, "deploy/robots/g1/config/policy")
SLOTDIR = os.path.join(POLICY, "mimic_masked")
ACTIVE = os.path.join(POLICY, "ACTIVE.yaml")
INDEX = os.path.join(POLICY, "POLICY_INDEX.md")

STORE = os.environ.get(
    "G1_POLICY_STORE",
    os.path.expanduser("~/experiments/_realrobot/center_g1/_policies"))
ROBOT = os.environ.get("G1_ROBOT_HOST", "g1")
ROBOT_WS = os.environ.get(
    "G1_ROBOT_WS", "~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy")

# 「무게」로 치는 것. 이 목록이 곧 .gitignore 와 rsync 대상의 정의다.
def weights_of(slot_root):
    """슬롯 안에서 git 밖으로 보내는 파일들 (상대경로)."""
    out = []
    exp = os.path.join(slot_root, "exported")
    if os.path.isdir(exp):
        for f in sorted(os.listdir(exp)):
            out.append(os.path.join("exported", f))
    par = os.path.join(slot_root, "params")
    if os.path.isdir(par):
        for f in sorted(os.listdir(par)):
            if f.endswith(".npz"):
                out.append(os.path.join("params", f))
    return out


def slot_names():
    if not os.path.isdir(SLOTDIR):
        return []
    return sorted(d for d in os.listdir(SLOTDIR)
                  if os.path.isdir(os.path.join(SLOTDIR, d)))


def has_weights(slot):
    """가중치가 워킹트리에 실제로 있는가 (심링크는 «닿는가» 로 본다)."""
    p = os.path.join(SLOTDIR, slot, "exported", "policy.onnx")
    return os.path.exists(p)          # os.path.exists 는 심링크를 따라간다


def in_store(slot):
    return os.path.exists(os.path.join(STORE, slot, "exported", "policy.onnx"))


def meta_of(slot):
    p = os.path.join(SLOTDIR, slot, "ONNX_META.json")
    if not os.path.exists(p):
        return {}
    try:
        return json.load(open(p, encoding="utf-8"))
    except Exception:
        return {}


def du_mb(path):
    tot = 0
    for root, _, files in os.walk(path):
        for f in files:
            fp = os.path.join(root, f)
            if os.path.islink(fp):
                continue
            try:
                tot += os.path.getsize(fp)
            except OSError:
                pass
    return tot / 1048576.0


def read_active():
    """일부러 아주 단순한 형식으로 읽는다 — 로봇의 bash 런처도 같은 파일을 sed 로 읽는다."""
    out = {}
    if not os.path.exists(ACTIVE):
        return out
    for line in open(ACTIVE, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = line.split(":", 1)
        k, v = k.strip(), v.strip()
        if k and v and k not in ("day",):
            out[k] = v
    return out


# ── 명령 ────────────────────────────────────────────────────────────────
def cmd_list(args):
    al = {v: k for k, v in read_active().items()}
    print("슬롯 (%s)" % SLOTDIR)
    print("  %-46s %-6s %-9s %8s  %s" % ("이름", "별칭", "상태", "MB", "출처"))
    for s in slot_names():
        root = os.path.join(SLOTDIR, s)
        here, store = has_weights(s), in_store(s)
        if here:
            state = "활성"
        elif store:
            state = "보관"
        else:
            state = "🔴 없음"
        m = meta_of(s)
        src = (m.get("mode1_ckpt") or m.get("note") or "")
        src = src.split("/")[-1] if "/" in src else src[:40]
        print("  %-46s %-6s %-9s %8.1f  %s"
              % (s, al.get(s, ""), state, du_mb(root), src))
    print("\n보관소 (%s)" % STORE)
    if os.path.isdir(STORE):
        arch = sorted(d for d in os.listdir(STORE)
                      if os.path.isdir(os.path.join(STORE, d)))
        print("  %d 개, %.1f MB" % (len(arch), du_mb(STORE)))
    else:
        print("  (없음)")


def cmd_archive(args):
    for s in args.slots:
        root = os.path.join(SLOTDIR, s)
        if not os.path.isdir(root):
            print("🔴 그런 슬롯이 없다: %s" % s); return 1
        if s in read_active().values():
            print("🔴 %s 는 지금 ACTIVE.yaml 에 걸려 있다 — 먼저 activate 에서 빼라" % s)
            return 1
        dst = os.path.join(STORE, s)
        moved = 0
        for rel in weights_of(root):
            src, d = os.path.join(root, rel), os.path.join(dst, rel)
            os.makedirs(os.path.dirname(d), exist_ok=True)
            if os.path.islink(src):
                link = os.readlink(src)
                if os.path.lexists(d):
                    os.remove(d)
                os.symlink(link, d)
                os.remove(src)
            else:
                if os.path.exists(d) and os.path.getsize(d) == os.path.getsize(src):
                    os.remove(src)          # 이미 같은 게 보관소에 있다
                else:
                    shutil.move(src, d)
            moved += 1
        # 정체(deploy.yaml · ONNX_META.json)는 «복사» 해 둔다 — 보관소만 봐도 뭔지 알게.
        for rel in ("ONNX_META.json", "params/deploy.yaml"):
            src = os.path.join(root, rel)
            if os.path.exists(src):
                d = os.path.join(dst, rel)
                os.makedirs(os.path.dirname(d), exist_ok=True)
                shutil.copy2(src, d)
        exp = os.path.join(root, "exported")
        if os.path.isdir(exp) and not os.listdir(exp):
            os.rmdir(exp)
        print("보관 %s — 파일 %d 개 → %s" % (s, moved, dst))
    return 0


def cmd_restore(args):
    for s in args.slots:
        src_root, dst_root = os.path.join(STORE, s), os.path.join(SLOTDIR, s)
        if not os.path.isdir(src_root):
            print("🔴 보관소에 없다: %s" % s); return 1
        if not os.path.isdir(dst_root):
            print("🔴 repo 에 슬롯 껍데기가 없다: %s (git 에서 되살려라)" % s); return 1
        n = 0
        for rel in weights_of(src_root):
            src, d = os.path.join(src_root, rel), os.path.join(dst_root, rel)
            os.makedirs(os.path.dirname(d), exist_ok=True)
            if os.path.lexists(d):
                continue
            if os.path.islink(src):
                os.symlink(os.readlink(src), d)
            else:
                shutil.copy2(src, d)
            n += 1
        print("복원 %s — 파일 %d 개" % (s, n))
    return 0


def cmd_activate(args):
    day = args.day
    pairs = []
    for a in args.assign:
        if "=" not in a:
            print("🔴 형식은 v1=<슬롯> 이다: %s" % a); return 1
        k, v = a.split("=", 1)
        if not os.path.isdir(os.path.join(SLOTDIR, v)):
            print("🔴 그런 슬롯이 없다: %s" % v); return 1
        if not has_weights(v):
            print("⚠  %s 는 가중치가 없다 — 먼저 restore 하라" % v)
        pairs.append((k.strip(), v.strip()))
    lines = [
        "# 오늘 무엇을 v1/v2/... 로 부를지. 런처가 `--policy v1` 을 여기로 푼다.",
        "#   ./tools/run_g1_with_gui.sh eth0 --policy v1",
        "# 🔴 기록(trial 마커·Streamlit)에는 **별칭이 아니라 실제 슬롯 이름**이 남는다 —",
        "#    v1 은 날마다 다른 것을 가리키므로 별칭으로 기록하면 나중에 못 읽는다.",
        "day: %s" % day,
    ]
    lines += ["%s: %s" % (k, v) for k, v in pairs]
    open(ACTIVE, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("ACTIVE.yaml — day %s" % day)
    for k, v in pairs:
        print("  %-4s -> %s" % (k, v))
    return 0


def cmd_active(args):
    if not os.path.exists(ACTIVE):
        print("(ACTIVE.yaml 없음)"); return 0
    sys.stdout.write(open(ACTIVE, encoding="utf-8").read())
    return 0


def cmd_push(args):
    slots = args.slots or sorted(set(read_active().values()))
    if not slots:
        print("🔴 보낼 슬롯이 없다 (ACTIVE.yaml 이 비었고 인자도 없다)"); return 1
    rc = 0
    for s in slots:
        root = os.path.join(SLOTDIR, s)
        if not has_weights(s):
            print("🔴 %s 에 가중치가 없다 — restore 부터" % s); rc = 1; continue
        remote = "%s:%s/deploy/robots/g1/config/policy/mimic_masked/%s/" % (
            ROBOT, ROBOT_WS, s)
        cmd = ["rsync", "-a", "--partial", "--append-verify", "--info=progress2",
               "--include=exported/***", "--include=params/",
               "--include=params/*.npz", "--include=*/",
               "--exclude=*", root + "/", remote]
        print("→ %s" % s)
        if args.dry_run:
            print("   " + " ".join(cmd)); continue
        r = subprocess.run(cmd)
        if r.returncode != 0:
            print("🔴 rsync 실패: %s" % s); rc = r.returncode
    return rc


def cmd_index(args):
    al = {v: k for k, v in read_active().items()}
    rows = []
    for s in slot_names():
        m = meta_of(s)
        here, store = has_weights(s), in_store(s)
        state = "활성" if here else ("보관" if store else "🔴 없음")
        v = m.get("verify") or {}
        if not isinstance(v, dict):      # 옛 슬롯은 verify 가 문자열 한 줄이다
            v = {"sim2sim_walk": str(v)}
        rows.append((s, al.get(s, ""), state, m, v))
    out = ["# 정책 슬롯 원장",
           "",
           "> 자동 생성 — `deploy/scripts/policy_slot.py index`. 손으로 고치지 말 것.",
           "",
           "**활성** = 가중치가 repo 워킹트리에 있다(로봇에 보낼 수 있다).",
           "**보관** = 가중치가 `%s` 에 있고 repo 엔 정체(`deploy.yaml`·`ONNX_META.json`)만 남았다."
           % STORE,
           "`restore` 로 언제든 되살린다. **기록은 어느 쪽이든 git 에 남아 있다.**",
           "",
           "| 슬롯 | 별칭 | 상태 | mode1 출처 | sim2sim | 실기 |",
           "|---|---|---|---|---|---|"]
    for s, alias, state, m, v in rows:
        ck = (m.get("mode1_ckpt") or "").split("/")
        ck = "/".join(ck[-2:]) if len(ck) > 1 else (ck[0] if ck else "")
        out.append("| `%s` | %s | %s | %s | %s | %s |" % (
            s, alias or "—", state, ck or "—",
            str(v.get("sim2sim_walk", "—"))[:60],
            str(v.get("real_robot", "—"))[:40]))
    out += ["", "## 왜 갈라 두나", "",
            "슬롯 하나가 60 MB 다. git 은 히스토리에서 그걸 **영원히 안 지운다** — 워킹트리에서",
            "지워도 새로 clone 하는 쪽은 전부 받는다. 그래서 «무게» 만 git 밖으로 내보내고",
            "«정체» 는 git 에 남긴다. 실험이 무엇이었는지는 슬롯을 치워도 사라지지 않는다.", ""]
    open(INDEX, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print("%s — 슬롯 %d 개" % (INDEX, len(rows)))
    return 0


def cmd_check(args):
    bad = 0
    act = read_active()
    if not act:
        print("⚠  ACTIVE.yaml 이 비었다 — `activate v1=<슬롯>` 으로 오늘 것을 지정하라")
    for alias, s in sorted(act.items()):
        root = os.path.join(SLOTDIR, s)
        if not os.path.isdir(root):
            print("🔴 %s -> %s : 슬롯이 없다" % (alias, s)); bad += 1; continue
        if not has_weights(s):
            print("🔴 %s -> %s : 가중치 없음 (restore 필요)" % (alias, s)); bad += 1; continue
        onnx = os.path.join(root, "exported", "policy.onnx")
        h = hashlib.md5(open(onnx, "rb").read()).hexdigest()
        want = (meta_of(s).get("verify", {}) or {}).get("onnx_md5")
        tag = "" if not want else (" md5 OK" if want.startswith(h[:12]) else
                                   " 🔴 md5 불일치 (기록 %s / 실제 %s)" % (want[:12], h[:12]))
        if want and not want.startswith(h[:12]):
            bad += 1
        print("🟢 %-4s -> %-44s %s%s" % (alias, s, h[:12], tag))
    # 깨진 심링크
    for s in slot_names():
        p = os.path.join(SLOTDIR, s, "exported", "policy.onnx")
        if os.path.islink(p) and not os.path.exists(p):
            print("🔴 %s : 심링크가 깨졌다 -> %s" % (s, os.readlink(p))); bad += 1
    print("\n%s" % ("🔴 문제 %d 건" % bad if bad else "🟢 이상 없음"))
    return 1 if bad else 0


def cmd_migrate_robot(args):
    """일회성 — 로봇을 «가중치가 git 에 있던» 상태에서 «git 밖» 으로 넘긴다.

    🔴 순서가 전부다. 그냥 pull 하면 git 이 «추적하던 파일이 커밋에서 사라졌다» 고 보고
       로봇 워킹트리에서 **바이너리를 지운다**. 그러면 241 MB 를 WiFi 로 다시 보내야 한다.
       그래서 pull 전에 빼두고 pull 뒤에 되돌린다 — 전송 0 MB.
       (보관소로 나간 옛 슬롯 10개는 일부러 안 빼둔다. 로봇에서 지워지는 게 맞다.)
    """
    keep = args.slots or sorted(set(read_active().values()))
    if not keep:
        print("🔴 남길 슬롯이 없다"); return 1
    root = "%s/deploy/robots/g1/config/policy/mimic_masked" % ROBOT_WS
    tar = "/tmp/policy_weights_migrate.tar"
    pats = " ".join("%s/exported %s/params/*.npz" % (k, k) for k in keep)

    def ssh(cmd, quiet=False):
        r = subprocess.run(["ssh", "-o", "ConnectTimeout=15", ROBOT, cmd],
                           capture_output=True, text=True)
        if not quiet:
            sys.stdout.write(r.stdout)
        if r.returncode != 0:
            sys.stderr.write(r.stderr)
        return r.returncode, r.stdout.strip()

    print("① 로봇에서 오늘 슬롯 %d 개의 가중치를 빼둔다" % len(keep))
    rc, _ = ssh("cd %s && tar cf %s %s && tar tf %s | wc -l" % (root, tar, pats, tar))
    if rc:
        print("🔴 tar 실패 — 중단한다 (아직 아무것도 안 바뀌었다)"); return 1

    print("② 코드 pull")
    r = subprocess.run([os.path.expanduser("~/piene_automation/robot_bridge/robot.sh"),
                        "sync-code", "unitree_rl_mjlab_deploy"])
    if r.returncode != 0:
        print("🔴 pull 실패 — 로봇에서 `tar xf %s` 로 되돌려라" % tar); return 1

    print("③ 되돌린다")
    rc, _ = ssh("cd %s && tar xf %s && echo 복원완료" % (root, tar))
    if rc:
        print("🔴 복원 실패 — 로봇에서 직접 `cd %s && tar xf %s`" % (root, tar)); return 1

    print("④ 확인")
    ssh("cd %s && for s in %s; do printf '  %%-30s %%s\\n' $s "
        "\"$(md5sum $s/exported/policy.onnx 2>/dev/null | cut -c1-12)\"; done; "
        "echo; du -sh . ; df -h / | tail -1" % (root, " ".join(keep)))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("list").set_defaults(fn=cmd_list)
    p = sub.add_parser("archive"); p.add_argument("slots", nargs="+"); p.set_defaults(fn=cmd_archive)
    p = sub.add_parser("restore"); p.add_argument("slots", nargs="+"); p.set_defaults(fn=cmd_restore)
    p = sub.add_parser("activate")
    p.add_argument("assign", nargs="+", metavar="v1=<슬롯>")
    p.add_argument("--day", default=__import__("datetime").date.today().strftime("%y%m%d"))
    p.set_defaults(fn=cmd_activate)
    sub.add_parser("active").set_defaults(fn=cmd_active)
    p = sub.add_parser("push"); p.add_argument("slots", nargs="*")
    p.add_argument("--dry-run", action="store_true"); p.set_defaults(fn=cmd_push)
    sub.add_parser("index").set_defaults(fn=cmd_index)
    sub.add_parser("check").set_defaults(fn=cmd_check)
    p = sub.add_parser("migrate-robot"); p.add_argument("slots", nargs="*")
    p.set_defaults(fn=cmd_migrate_robot)
    a = ap.parse_args()
    if not getattr(a, "fn", None):
        ap.print_help(); return 1
    return a.fn(a) or 0


if __name__ == "__main__":
    sys.exit(main())
