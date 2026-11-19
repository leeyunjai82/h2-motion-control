"""사용: python tools/ik_reach.py [urdf]          (기본 robots/h2/H2.urdf)
       python tools/ik_reach.py h2 [urdf] [-]   (예전 형식도 그대로 — 셋째 인자(메시 폴더)는 쓰지 않음)

H2 양팔 IK 도달 범위 표 (오프라인, pinocchio DLS — casadi 불필요, 로봇 불필요).

robot_arm_ik.py 와 같은 축소 모델: 다리·허리·헤드 잠금, L_ee/R_ee = *_wrist_yaw_joint + x 0.05.
목표: 양손 대칭, 손 자세 = 단위 회전, pelvis 기준.

잡기 시퀀스 전체 확인은 tools/grab_reach.py (서버 IK · robot.yaml grab 값).
"""
import os
import sys
import numpy as np
import pinocchio as pin

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_URDF = os.path.join(REPO_ROOT, "robots", "h2", "H2.urdf")

LOCK_COMMON = [f"{s}_{j}_joint" for s in ("left", "right")
               for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
LOCK_COMMON += ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
H2_EXTRA_LOCK = ["head_pitch_joint", "head_yaw_joint"]   # robot.yaml ik.lock_joints 의 헤드


def build(urdf, mesh_dir, extra_lock):
    """축소 모델. mesh_dir 은 쓰지 않음 (기구학만 — 호출부 호환용 인자)."""
    full = pin.buildModelFromUrdf(urdf)       # 기구학만 (메시 불필요)
    lock = [full.getJointId(n) for n in LOCK_COMMON + extra_lock if full.existJointName(n)]
    m = pin.buildReducedModel(full, lock, np.zeros(full.nq))
    for side, j in (("L", "left_wrist_yaw_joint"), ("R", "right_wrist_yaw_joint")):
        m.addFrame(pin.Frame(f"{side}_ee", m.getJointId(j), pin.SE3(np.eye(3), np.array([0.05, 0, 0])),
                             pin.FrameType.OP_FRAME))
    return m


def solve(m, d, TL, TR, q0, iters=300):
    q = q0.copy()
    fl, fr = m.getFrameId("L_ee"), m.getFrameId("R_ee")
    for _ in range(iters):
        pin.framesForwardKinematics(m, d, q)
        eL = pin.log6(d.oMf[fl].actInv(TL)).vector
        eR = pin.log6(d.oMf[fr].actInv(TR)).vector
        err = np.concatenate([eL, eR])
        if np.linalg.norm(eL[:3]) < 1e-3 and np.linalg.norm(eR[:3]) < 1e-3 and \
           np.linalg.norm(eL[3:]) < 1e-2 and np.linalg.norm(eR[3:]) < 1e-2:
            break
        pin.computeJointJacobians(m, d, q)
        JL = pin.getFrameJacobian(m, d, fl, pin.LOCAL)
        JR = pin.getFrameJacobian(m, d, fr, pin.LOCAL)
        J = np.vstack([JL, JR])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(12), err)
        q = np.clip(pin.integrate(m, q, 0.5 * dq), m.lowerPositionLimit, m.upperPositionLimit)
    pin.framesForwardKinematics(m, d, q)
    pe = max(np.linalg.norm(d.oMf[fl].translation - TL.translation),
             np.linalg.norm(d.oMf[fr].translation - TR.translation))
    re = max(np.linalg.norm(pin.log3(d.oMf[fl].rotation.T @ TL.rotation)),
             np.linalg.norm(pin.log3(d.oMf[fr].rotation.T @ TR.rotation)))
    return q, pe, np.degrees(re)


def grid(name, urdf, mesh_dir, extra_lock, half_w):
    m = build(urdf, mesh_dir, extra_lock)
    d = m.createData()
    print(f"\n=== {name}  nq={m.nq}  joints={[m.names[i] for i in range(1, m.njoints)]}")
    xs = [0.20, 0.30, 0.40, 0.50]
    zs = [-0.30, -0.20, -0.10, 0.00, 0.10, 0.20, 0.30]
    print(f"양손 y=±{half_w:.2f} m, 단위 회전. 칸: 위치오차cm/자세오차deg  (✓ = 1cm·5deg 이내)")
    print("  z\\x  " + "".join(f"{x:>14.2f}" for x in xs))
    for z in zs:
        row = f"{z:+.2f}  "
        for x in xs:
            TL = pin.SE3(np.eye(3), np.array([x, +half_w, z]))
            TR = pin.SE3(np.eye(3), np.array([x, -half_w, z]))
            best = None
            for q0 in (np.zeros(m.nq), pin.neutral(m) + 0.3):
                q0 = np.clip(q0, m.lowerPositionLimit, m.upperPositionLimit)
                r = solve(m, d, TL, TR, q0)
                if best is None or r[1] + r[2] / 500 < best[1] + best[2] / 500:
                    best = r
            ok = best[1] < 0.01 and best[2] < 5
            row += f"{best[1]*100:6.1f}/{best[2]:4.0f}{'✓' if ok else ' ':>2} "
        print(row)


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0].lower() == "g1":
        sys.exit("G1 지원 제거됨 — 이 저장소는 H2 전용. 사용: python tools/ik_reach.py [robots/h2/H2.urdf]")
    if args and args[0].lower() == "h2":
        args = args[1:]                      # 예전 형식: h2 <urdf> <mesh_dir|->
    urdf = args[0] if args else DEFAULT_URDF
    if not os.path.isfile(urdf):
        sys.exit(f"URDF 없음: {urdf}\n사용: python tools/ik_reach.py [robots/h2/H2.urdf]")
    grid(f"H2 ({urdf})", urdf, None, H2_EXTRA_LOCK, 0.17)
