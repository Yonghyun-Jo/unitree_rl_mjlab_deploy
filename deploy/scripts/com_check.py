#!/usr/bin/env python3
"""com_check.py — «실기 질량중심이 모델과 같은가» 를 발목 토크로 잰다 (정책 OFF 구간도 됨).

    uv run --no-project --with pyarrow --with numpy --with mujoco \\
        python deploy/scripts/com_check.py <realrobot_*.parquet ...>      # 온보드 로거 (FixStand 포함)
    .venv/bin/python deploy/scripts/com_check.py <gait_*.csv ...>         # 제어기 --dump (정책 ON)

# 원리
정지 균형에서 발목 pitch 토크 합 = M·g·(x_CoP − x_발목) 이고, 정적이면 CoP = CoM 의 발바닥 투영.
같은 관절각으로 모델(FK, xml 질량분포)의 CoM 을 **참 수직(IMU)** 으로 발바닥에 투영해 대조한다. **Δ = CoP(토크) − CoM(모델).**
sim 에서 Δ ≈ 0 인 것을 확인했다(v2: +3.23 vs +3.26 cm). 실기에서 Δ 가 한쪽 부호로 일관되면
«실기 질량분포가 모델과 다르다» 다. 2026-09-03: 모든 날·모든 상태에서 Δ = −1.5 ~ −3 cm (뒤).

# 조건
· 양발 수평(|zL−zR| < 8 mm) · 정지(|dq| < 0.05) · **발이 실려 있어야 한다** — 하네스가 무게를 나눠 들면
  CoP 크기가 틀어진다(부호는 안 바뀐다). FixStand 로 잴 땐 하네스를 완전히 느슨하게.
· 발목 관절 마찰(≈1.7 Nm)이 τ 추정을 깎는 몫 = 발목당 ~0.5 cm. 그 이하 차이는 못 믿는다.
· G1 발목은 평행기구라 τ 환산이 근사일 수 있다 — 크기의 불확실성이지 부호는 아니다.
"""
import sys, os, csv, numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
XML = os.path.join(REPO, "src/assets/robots/unitree_g1/xmls/g1.xml")


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _rot_inv(q, v):
    return _qmul(_qmul(np.array([q[0], -q[1], -q[2], -q[3]]), np.array([0.0, *v])), q)[1:]


def load(path):
    if path.endswith(".parquet"):
        import pyarrow.parquet as pq
        t = pq.read_table(path)
        return {k: t.column(k).to_numpy().astype(float) for k in t.column_names if k.startswith(("q_", "dq_", "tau_est_", "kp_", "bv_", "time", "quat_"))}
    R = [r for r in csv.DictReader(open(path, encoding="utf-8", errors="ignore")) if r.get("kp_0")]
    keys = [k for k in R[0] if k.startswith(("q_", "dq_", "tau_est_", "kp_", "bv_", "time", "quat_"))]
    return {k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in R]) for k in keys}


def main():
    import mujoco
    m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m)
    LF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link")
    RF = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link")
    M = m.body_subtreemass[1]; g = 9.81
    print("모델 총질량 %.2f kg (손·장착물 없음).  Δ = CoP(토크) − CoM(모델) [cm], − 면 실기 CoM 이 모델보다 뒤" % M)
    print("%-38s %5s %5s %8s %8s %8s %7s" % ("파일", "kp0", "n", "CoM모델", "CoP토크", "Δ", "발목각"))
    allD = []
    for f in sys.argv[1:]:
        A = load(f)
        if "tau_est_4" not in A or "q_4" not in A:
            print(os.path.basename(f), "컬럼 없음"); continue
        dq = np.nanmax(np.abs(np.stack([A["dq_%d" % j] for j in range(29)], 1)), 1)
        still = (dq < 0.05) & (A["kp_0"] > 0)
        if "bv_x" in A:
            still &= (np.abs(A["bv_x"]) < 1e-6) & (np.abs(A.get("bv_wz", A["bv_x"])) < 1e-6)
        for kpv in sorted(set(np.round(A["kp_0"][still]).astype(int))):
            sel = np.where(still & (np.round(A["kp_0"]).astype(int) == kpv))[0]
            if len(sel) < 100:
                continue
            sel = sel[::max(1, len(sel) // 300)]
            com, cop, ang = [], [], []
            for i in sel:
                d.qpos[:] = 0; d.qpos[3] = 1.0
                for j in range(29):
                    d.qpos[7 + j] = A["q_%d" % j][i]
                mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
                if abs(d.xpos[LF][2] - d.xpos[RF][2]) > 0.008:
                    continue
                # 🔴 수직은 발바닥 법선이 아니라 «참 수직»(IMU 중력) 이다. 밑창이 굴러 발끝이 들리면 발바닥
                #    평면 축으로 잰 CoM 은 h·tan(기울기) 만큼 틀린다(0.6 m × 2.9° ≈ 3 cm — 2026-09-03 에 이걸로
                #    «CoM 이 뒤» 를 3 cm 로 부풀렸다). IMU 는 폰 수평계로 −4.5° 일치 확인(2026-09-03).
                q = [A["quat_w"][i], A["quat_x"][i], A["quat_y"][i], A["quat_z"][i]]
                up = -_rot_inv(q, [0, 0, -1.0]); up /= np.linalg.norm(up)
                x = np.array([1.0, 0, 0]); x -= np.dot(x, up) * up; x /= np.linalg.norm(x)
                mid = (d.xpos[LF] + d.xpos[RF]) / 2
                com.append(np.dot(d.subtree_com[0] - mid, x) * 100)
                cop.append((A["tau_est_4"][i] + A["tau_est_10"][i]) / (M * g) * 100)
                ang.append(np.degrees(A["q_4"][i]))
            if len(com) < 30:
                continue
            D = np.mean(cop) - np.mean(com); allD.append(D)
            print("%-38s %5d %5d %+8.2f %+8.2f %+8.2f %+7.1f%s" % (os.path.basename(f)[:38], kpv, len(com), np.mean(com), np.mean(cop), D, np.mean(ang),
                  "   (깊이 앉음 — 하네스 의심)" if np.mean(ang) < -17 else ""))
    if allD:
        print("\n  블록 %d 개  Δ 중앙값 %+.2f cm  [%+.2f, %+.2f]" % (len(allD), np.median(allD), np.percentile(allD, 25), np.percentile(allD, 75)))
        print("  한쪽 부호로 일관 + |Δ| > 1 cm 면 질량분포 차이. 0.5 cm 이하는 관절 마찰 안이라 못 믿는다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
