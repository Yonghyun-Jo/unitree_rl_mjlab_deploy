#!/usr/bin/env python3
"""imu_bias_fit.py — 실기 gait CSV 에서 IMU 장착 편향을 «몸 고정» 과 «바닥 경사(월드 고정)» 로 분리한다.

    .venv/bin/python deploy/scripts/imu_bias_fit.py <gait_*.csv ...>        # 정책 ON 구간(제어기 --dump)
    .venv/bin/python deploy/scripts/imu_bias_fit.py --selftest                # 부호 규약 자기검증
    uv run --no-project --with pyarrow --with numpy --with mujoco \
        python deploy/scripts/imu_bias_fit.py --onboard <realrobot_*.parquet ...>   # 정책 ON/OFF 대조

# 무엇을 재나
이 정책은 상태추정이 없어 IMU 중력이 유일한 «어디가 수직인가» 다. IMU 가 δ° 기울어 붙어 있으면
정책은 몸을 δ° 기운 채로 «수직» 으로 지킨다. 그 δ 를 로봇을 특별히 세우지 않고 **기존 로그**로 잰다:

  · 발 접촉 geom 은 ankle_roll 프레임 z=−0.025 → **발바닥 법선 = ankle_roll 의 +z**.
  · 양발 지지 정지 구간에서 관절각으로 FK → 두 발 법선의 합 n (골반 프레임).
  · IMU 중력 g = quat⁻¹·(0,0,−1) → up = −g.  n 과 up 의 pitch/roll 차이가 «측정치» 다.
  · 바닥이 수평이면 n 이 진짜 수직이고, 그 차이가 곧 IMU 편향이다.

# 바닥 경사와 어떻게 가르나
편향은 몸에 고정돼 방위를 따라 돌고, 바닥 경사는 월드에 고정돼 방위에 따라 사인파로 나타난다:
    pitch(ψ) = b_x + t_x cos ψ + t_y sin ψ
    roll (ψ) = b_y − t_x sin ψ + t_y cos ψ        (ψ = IMU yaw)
최소자승으로 (b_x, b_y, t_x, t_y) 를 푼다. **방위 폭이 180° 이상** 이어야 갈린다.

# 부호 — 🔴 여기서 나온 b 를 config.yaml `imu_cal.pitch_deg` 에 **그대로** 넣는다
모형: q_보고 = q_골반 · Ry(b).  ImuCal::set(b) 는 q_골반 = q_보고 · Ry(b)⁻¹ 로 되돌린다 — 같은 b 다.
`--selftest` 가 합성 데이터로 「측정 P = b」와 「ImuCal(b) 후 P → 0」을 잠근다(C++ 쪽은 test_imu_cal ②).
2026-09-01 노트는 «부호를 뒤집어 넣으라(−b)» 고 적었는데 **그게 틀렸다** — 실기 quat 에
ImuCal(−4.22) 를 걸면 잔차가 0 에 오고, ImuCal(+4.22) 는 두 배가 된다(2026-09-03 확인).

# 🔴 «몸-고정» ≠ «센서» — 정책이 만든 발 기울기도 몸을 따라 돈다
방위 분리는 «바닥 경사» 만 걸러낸다. 정책이 뒤꿈치로 서서 발끝이 들리면 그 기울기도 몸-고정이라
편향처럼 보인다. 그래서 **정책이 꺼진 구간(FixStand)** 과 비교해야 한다 → `--onboard` (온보드
parquet 을 kp 로 갈라 정책 ON/OFF 의 P 를 나란히 찍는다). 2026-09-03: 정책 ON −3.5~−4.2°,
FixStand −0.3 ± 1.7° → 편향이 아니라 자세일 가능성이 크다. config 는 0 으로 두었다.

# 방법의 계통오차
같은 계산을 **sim 로그**(IMU 정확)에 돌리면 0.15° 가 나온다(2026-09-01). 그 크기 이하는 못 믿는다.

# 불확실성
표본은 50 Hz 연속이라 서로 독립이 아니다. 그래서 «정지 에피소드» (연속 구간) 단위로 부트스트랩한다.
런별 적합도 같이 찍어 «한 런이 끌고 가는가» 를 본다.
"""
import argparse, csv, os, sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(REPO, "src/assets/robots/unitree_g1/xmls/g1.xml")


def num(r, k, dd=0.0):
    try:
        return float(r[k])
    except Exception:
        return dd


def quat_conj_rotate(q, v):
    """q⁻¹ · v · q  (q = [w,x,y,z] 단위 쿼터니언, v 3벡터) — 월드 벡터를 몸 프레임으로."""
    w, x, y, z = q

    def mul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                         w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])

    qc = np.array([w, -x, -y, -z])
    return mul(mul(qc, np.array([0.0, *v])), np.array([w, x, y, z]))[1:]


def extract(path, per_file, max_dz, verbose=True):
    """한 CSV → (pitch_off[deg], roll_off[deg], yaw[rad], episode_id) — 양발지지 정지 표본만."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    LF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link")
    RF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link")

    R = [r for r in csv.DictReader(open(path, encoding="utf-8", errors="ignore")) if r.get("kp_0")]
    kp = np.array([num(r, "kp_0") for r in R])
    eff = np.array([num(r, "eff") for r in R])
    bvx = np.array([num(r, "bv_x") for r in R])
    # 정책 ON(kp>0) · 보행 위상 정지(eff<0.01) · 전진 명령 0
    idx = np.where((kp > 0) & (eff < 0.01) & (np.abs(bvx) < 1e-6))[0]
    stride = max(1, len(idx) // per_file) if per_file > 0 else 1
    P, Rl, Y, E = [], [], [], []
    ep, last_i = 0, None
    for i in idx[::stride]:
        r = R[i]
        d.qpos[:] = 0
        d.qpos[3] = 1.0
        for j in range(29):
            d.qpos[7 + j] = num(r, "q_%d" % j)
        mujoco.mj_kinematics(m, d)
        if abs(d.xpos[LF][2] - d.xpos[RF][2]) > max_dz:      # 두 발 높이가 다르면 양발지지 아님
            continue
        n = d.xmat[LF].reshape(3, 3)[:, 2] + d.xmat[RF].reshape(3, 3)[:, 2]
        n /= np.linalg.norm(n)
        g = quat_conj_rotate([num(r, "quat_w"), num(r, "quat_x"), num(r, "quat_y"), num(r, "quat_z")],
                             [0, 0, -1.0])
        up = -g / np.linalg.norm(g)
        if last_i is not None and i - last_i > 50:              # 1 s(50 Hz) 이상 끊기면 새 에피소드
            ep += 1
        last_i = i
        P.append(np.degrees(np.arctan2(n[0], n[2]) - np.arctan2(up[0], up[2])))
        Rl.append(np.degrees(np.arctan2(n[1], n[2]) - np.arctan2(up[1], up[2])))
        Y.append(num(r, "rpy_y"))
        E.append(ep)
    if verbose:
        print("  %-28s 행 %6d · 정지 후보 %5d · 양발지지 표본 %4d · 에피소드 %2d"
              % (os.path.basename(path), len(R), len(idx), len(P), ep + 1))
    return np.array(P), np.array(Rl), np.array(Y), np.array(E)


def fit(P, Rl, Y):
    c, s = np.cos(Y), np.sin(Y)
    one, zero = np.ones_like(c), np.zeros_like(c)
    A = np.vstack([np.column_stack([one, zero, c, s]),
                   np.column_stack([zero, one, -s, c])])
    b = np.concatenate([P, Rl])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    pred = A @ sol
    ss_res = ((b - pred) ** 2).sum()
    ss_tot = ((b - b.mean()) ** 2).sum()
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return sol, r2, np.sqrt(ss_res / len(b))


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _Ry(deg):
    a = np.radians(deg) / 2
    return np.array([np.cos(a), 0.0, np.sin(a), 0.0])


def _P(q, n):
    up = -quat_conj_rotate(q, [0, 0, -1.0]); up = up / np.linalg.norm(up)
    return np.degrees(np.arctan2(n[0], n[2]) - np.arctan2(up[0], up[2]))


def selftest():
    """합성: 참 자세 θ 로 서고 발은 평평(n = 참 up). IMU 가 Ry(β) 로 돌아 붙음 → q_rep = q_true·Ry(β).
    잠그는 것: ① 측정 P == β  ② ImuCal::apply 와 같은 식 q_rep·Ry(β)⁻¹ 을 걸면 P → 0  ③ −β 면 두 배."""
    bad = 0
    for th in (0.0, 3.0, -2.0, 8.0):
        q_true = _Ry(th)
        n = -quat_conj_rotate(q_true, [0, 0, -1.0]); n = n / np.linalg.norm(n)
        for beta in (-4.22, 1.0, 4.22):
            q_rep = _qmul(q_true, _Ry(beta))
            P = _P(q_rep, n)
            P_ok = _P(_qmul(q_rep, np.array([1, -1, -1, -1]) * _Ry(beta)), n)      # ImuCal(beta)
            P_bad = _P(_qmul(q_rep, np.array([1, -1, -1, -1]) * _Ry(-beta)), n)    # ImuCal(-beta)
            ok = abs(P - beta) < 1e-6 and abs(P_ok) < 1e-6 and abs(abs(P_bad) - 2 * abs(beta)) < 1e-6
            bad += not ok
            print("  θ=%+4.1f β=%+5.2f  P=%+6.2f  ImuCal(β)→%+6.2f  ImuCal(−β)→%+6.2f  %s"
                  % (th, beta, P, P_ok, P_bad, "ok" if ok else "FAIL"))
    print("[selftest] %s" % ("ALL PASS — config 에는 fit 의 b 를 그대로 넣는다" if not bad else "%d FAIL" % bad))
    return 1 if bad else 0


def onboard(paths, stride_cap=400):
    """온보드 로거 parquet 을 kp_0 로 갈라(정책 ON = 40.179, FixStand = config FSM kp) 정지 구간의 P 를 찍는다.
    실행: uv run --no-project --with pyarrow --with numpy --with mujoco python deploy/scripts/imu_bias_fit.py --onboard <parquet...>"""
    import pyarrow.parquet as pq, mujoco
    m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
    LF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link")
    RF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link")
    print("%-34s %6s %6s %8s %8s %8s %8s %6s" % ("파일", "kp0", "n", "P[°]", "rpy_p", "ankL", "ankR", "yaw폭"))
    agg = {}
    for f in paths:
        t = pq.read_table(f); c = t.column_names
        if "kp_0" not in c or "quat_w" not in c:
            print(os.path.basename(f), "컬럼 없음"); continue
        A = {k: t.column(k).to_numpy() for k in
             ["kp_0", "quat_w", "quat_x", "quat_y", "quat_z", "rpy_p", "rpy_y"] + ["q_%d" % j for j in range(29)] + ["dq_%d" % j for j in range(29)]}
        dq = np.max(np.abs(np.stack([A["dq_%d" % j] for j in range(29)], 1)), 1)
        still = (dq < 0.05) & np.isfinite(A["kp_0"])
        for kpv in sorted(set(np.round(A["kp_0"][still]).astype(int))):
            if kpv == 0:
                continue                                   # passive = 매달림/앉음, 발이 안 실려 있다
            sel = np.where(still & (np.round(A["kp_0"]).astype(int) == kpv))[0]
            if len(sel) < 200:
                continue
            sel = sel[::max(1, len(sel) // stride_cap)]
            P, RP, AL, AR, Y = [], [], [], [], []
            for i in sel:
                d.qpos[:] = 0; d.qpos[3] = 1.0
                for j in range(29):
                    d.qpos[7 + j] = A["q_%d" % j][i]
                mujoco.mj_kinematics(m, d)
                if abs(d.xpos[LF][2] - d.xpos[RF][2]) > 0.008:
                    continue
                n = d.xmat[LF].reshape(3, 3)[:, 2] + d.xmat[RF].reshape(3, 3)[:, 2]; n /= np.linalg.norm(n)
                q = [A["quat_w"][i], A["quat_x"][i], A["quat_y"][i], A["quat_z"][i]]
                P.append(_P(q, n)); RP.append(np.degrees(A["rpy_p"][i]))
                AL.append(np.degrees(A["q_4"][i])); AR.append(np.degrees(A["q_10"][i])); Y.append(A["rpy_y"][i])
            if len(P) < 30:
                continue
            fallen = abs(np.mean(RP)) > 15.0                    # 넘어진/눕힌 블록 — 발이 안 실려 있다
            print("%-34s %6d %6d %+8.2f %+8.2f %+8.2f %+8.2f %6.0f%s"
                  % (os.path.basename(f)[:34], kpv, len(P), np.mean(P), np.mean(RP), np.mean(AL), np.mean(AR),
                     np.degrees(max(Y) - min(Y)), "  (제외: |pitch|>15°)" if fallen else ""))
            if not fallen:
                agg.setdefault(kpv, []).append(np.mean(P))
    print("\n  kp 별 블록 P (정책 ON = kp 40 · 그 밖 = FixStand 등 정책 OFF) — 중앙값 [사분위]:")
    for kpv, v in sorted(agg.items()):
        q1, med, q3 = np.percentile(v, [25, 50, 75])
        print("    kp %3d  블록 %3d  P 중앙값 %+6.2f°  [%+6.2f, %+6.2f]" % (kpv, len(v), med, q1, q3))
    print("  🔴 장착 편향이면 kp 와 무관하게 같은 값이어야 한다. 정책 ON 에서만 크면 «자세» 다.")
    return 0


def main():
    if "--selftest" in sys.argv:
        return selftest()
    if "--onboard" in sys.argv:
        k = sys.argv.index("--onboard")
        return onboard(sys.argv[k + 1:])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="실기 gait_*.csv (--dump 로 찍은 것)")
    ap.add_argument("--per-file", type=int, default=700, help="파일당 표본 상한(균등 서브샘플). 0=전부")
    ap.add_argument("--max-dz", type=float, default=0.008, help="양발 높이차 허용 [m]")
    ap.add_argument("--boot", type=int, default=1000, help="에피소드 부트스트랩 횟수")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    print("[표본 추출]")
    per = []
    for f in a.csv:
        per.append(extract(f, a.per_file, a.max_dz))
    P = np.concatenate([p[0] for p in per]); Rl = np.concatenate([p[1] for p in per])
    Y = np.concatenate([p[2] for p in per])
    # 에피소드 id 를 파일 간에 겹치지 않게
    E, off = [], 0
    for p in per:
        E.append(p[3] + off); off += p[3].max() + 1 if len(p[3]) else 0
    E = np.concatenate(E)
    if len(P) < 30:
        print("🔴 표본이 %d 개뿐이다 — 판정 불가" % len(P)); return 1
    span = np.degrees(Y.max() - Y.min())
    print("\n양발지지 정지 표본 %d 개 · 에피소드 %d 개 · 방위 폭 %.0f°%s"
          % (len(P), E.max() + 1, span, "" if span >= 180 else "   🔴 180° 미만 — 편향/경사 분리 불가"))

    sol, r2, rms = fit(P, Rl, Y)
    bx, by, tx, ty = sol
    print("\n  🔴 IMU 편향(몸에 고정)   pitch %+.2f°   roll %+.2f°   크기 %.2f°" % (bx, by, np.hypot(bx, by)))
    print("  🟡 바닥 경사(월드 고정)  x %+.2f°   y %+.2f°   크기 %.2f° (방위 %.0f°)"
          % (tx, ty, np.hypot(tx, ty), np.degrees(np.arctan2(ty, tx))))
    print("  적합도 R² = %.3f · 잔차 RMS %.2f°" % (r2, rms))

    # 런별 — 한 런이 끌고 가는가
    if len(per) > 1:
        print("\n  런별 적합 (각 런만으로):")
        for f, p in zip(a.csv, per):
            if len(p[0]) < 30:
                print("    %-28s 표본 %d — 생략" % (os.path.basename(f), len(p[0]))); continue
            s1, r21, rms1 = fit(p[0], p[1], p[2])
            sp = np.degrees(p[2].max() - p[2].min())
            print("    %-28s n=%4d 방위 %3.0f°  편향 pitch %+.2f° roll %+.2f°  경사 %.2f°  R² %.2f"
                  % (os.path.basename(f), len(p[0]), sp, s1[0], s1[1], np.hypot(s1[2], s1[3]), r21))

    # 에피소드 부트스트랩 — 연속 표본은 독립이 아니라서 에피소드째로 뽑는다
    rng = np.random.default_rng(a.seed)
    eps = np.unique(E)
    bs = []
    for _ in range(a.boot):
        pick = rng.choice(eps, size=len(eps), replace=True)
        sel = np.concatenate([np.where(E == e)[0] for e in pick])
        if np.degrees(Y[sel].max() - Y[sel].min()) < 120:
            continue
        s, _, _ = fit(P[sel], Rl[sel], Y[sel])
        bs.append(s[:2])
    if len(bs) >= 50:
        bs = np.array(bs)
        lo, hi = np.percentile(bs, [2.5, 97.5], axis=0)
        print("\n  에피소드 부트스트랩 95 %% (%d/%d 유효):  pitch [%+.2f, %+.2f]°   roll [%+.2f, %+.2f]°"
              % (len(bs), a.boot, lo[0], hi[0], lo[1], hi[1]))
    else:
        print("\n  부트스트랩 유효 표본 부족(%d) — 구간을 못 낸다" % len(bs))

    print("\n  방위 구간별 실측 pitch 성분 (편향이면 일정, 경사면 사인파)")
    yd = np.degrees(Y)
    for lo_ in range(-180, 180, 45):
        s2 = (yd >= lo_) & (yd < lo_ + 45)
        if s2.sum() < 25:
            continue
        print("    yaw %+4d~%+4d°  n=%4d   pitch %+6.2f°   roll %+6.2f°"
              % (lo_, lo_ + 45, s2.sum(), P[s2].mean(), Rl[s2].mean()))

    print("\n  ▶ config.yaml 에 «넣을 값» (fit 의 b 그대로 · 바닥 경사는 안 뺀다):")
    print("      imu_cal:\n        pitch_deg: %+.2f\n        roll_deg:  %+.2f" % (bx, by))
    print("    첫 실기는 하네스 + 절반부터:  G1_IMU_CAL_DEG=\"%.2f,%.2f\"" % (bx / 2, by / 2))
    print("    🔴 넣기 전에 --onboard 로 «정책 OFF(FixStand) 구간도 같은 값인가» 를 본다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
