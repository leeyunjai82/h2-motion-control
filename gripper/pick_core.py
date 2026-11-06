"""
pick_core.py — 그리퍼 한 팔 잡기: 모델 · IK · 경로 계획 · arm_server 스트림 실행 (gripper/pick_web.py 가 씀)

  · 팔 IK 모델 = utils/hand_to_marker.py · common/joint_check.py 와 같은 축소 모델 (robot.yaml ik) +
    집게 가운데(TCP) 프레임 = 손목 yaw 관절 + gripper.yaml tcp_offset (계산값)
  · 팔 이동은 모두 arm_server /stream (관절 목표 스트리밍): 계획 단계에서 관절 경로를 미리 다 풀어 두고
    (직선 구간은 2 mm · 0.5° 마다 IK, 관절이 튀면 계획 실패), 실행은 그 경로를 50 Hz 로 보냄.
  · 잡기는 준비 자세(카메라 화면 밖, gripper.yaml ready_tcp_m)에서 시작해서 준비 자세로 끝남.
    기본 자세 ↔ 준비 자세는 따로 (관절 보간) — 확인 창에 그리퍼가 가장 앞으로 나가는 거리 · 가장 낮은 점을 보여 줌
    arm_server 가 관절마다 server_max_vel 을 넘는 변화를 자름. 송신이 끊기면 서버가 마지막 목표에서 멈춤.
  · 박스 쪽 코드(robot_server · arm_server 의 /hands)는 쓰지 않음
"""
import json
import math
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "common"))
import robot_env   # robot.yaml · URDF · 카메라 값
import gripper_config

import numpy as np
import pinocchio as pin

CFG = gripper_config.load()
P = CFG.get("pick") or {}
_need = ("mm_per_rad", "open_margin_mm", "squeeze_mm", "contact_min_mm", "pad_len_half_mm", "pad_h_half_mm", "tip_beyond_mm",
         "body_back_mm", "body_h_half_mm", "min_overlap_mm", "tip_clear_mm",
         "transit_clear_mm", "approach_m", "lift_m", "hold_s", "rpy_deg", "v_lin", "v_slow", "v_joint",
         "server_max_vel", "track_abort_deg", "tcp_bias_m", "ready_tcp_m", "ready_rpy_deg", "via_tcp_m", "via_rpy_deg",
         "body_front_x", "body_half_y", "body_z_min", "body_clear_m")
_miss = [k for k in _need if k not in P]
if _miss:
    sys.exit(f"[pick] gripper.yaml pick: 에 {_miss} 없음 — 기본값으로 대신하지 않음")
if not (0 < float(P["server_max_vel"]) <= 3.0) or not (0 < float(P["v_joint"]) <= float(P["server_max_vel"])):
    sys.exit("[pick] gripper.yaml pick: 0 < v_joint ≤ server_max_vel ≤ 3.0 이어야 함")

_tb = P["tcp_bias_m"]
if not (isinstance(_tb, dict) and all(isinstance(_tb.get(s), list) and len(_tb[s]) == 3 and
                                      all(isinstance(v, (int, float)) and abs(v) <= 0.05 for v in _tb[s]) for s in ("left", "right"))):
    sys.exit("[pick] gripper.yaml pick.tcp_bias_m: left/right 각각 [x, y, z] (각 |값| ≤ 0.05 m)")
TCP_BIAS = {s: np.array(_tb[s], dtype=float) for s in ("left", "right")}   # 실제 − 계산 (pelvis)

ARM_SERVER = "http://localhost:50022"
J = robot_env.JOINTS
ARM = [int(s) for s in J["arm"]]
SIDES = ("left", "right")
I3 = np.eye(3)
EE_OFF = np.array(robot_env.CFG["ik"]["ee_offset"], dtype=float)
TCP_OFF = np.array(CFG["tcp_offset"], dtype=float)
Q_LO, Q_HI = float(CFG["q_range"][0]), float(CFG["q_range"][1])
MM_PER_RAD = float(P["mm_per_rad"])

# 준비 자세 (gripper.yaml pick.ready_tcp_m · ready_rpy_deg — 왼손 기준, 오른손은 y 반대) — 카메라 화면 밖
_rt = [float(v) for v in P["ready_tcp_m"]]
READY_TCP = {"left": np.array([_rt[0], abs(_rt[1]), _rt[2]]), "right": np.array([_rt[0], -abs(_rt[1]), _rt[2]])}
_rr = [math.radians(float(v)) for v in P["ready_rpy_deg"]]
READY_R = pin.rpy.rpyToMatrix(*_rr)
_vt = [float(v) for v in P["via_tcp_m"]]
VIA_TCP = {"left": np.array([_vt[0], abs(_vt[1]), _vt[2]]), "right": np.array([_vt[0], -abs(_vt[1]), _vt[2]])}
VIA_R = pin.rpy.rpyToMatrix(*[math.radians(float(v)) for v in P["via_rpy_deg"]])
FRONT_X, FRONT_LOW = 0.25, 0.12   # 기본 ↔ 준비 이동: 몸 앞 FRONT_X 넘는 곳에서 그리퍼가 FRONT_LOW 아래로 가면 경유 자세를 거침
AT_READY_POS, AT_READY_ROT = 0.03, math.radians(10)

# 클릭 점 (박스 윗면 가운데) 허용 범위 (pelvis) — 깊이가 엉뚱하면 거부
CLICK_X, CLICK_Y_MAX, CLICK_Z = (0.25, 0.80), 0.45, (-0.50, 0.40)
MIDLINE_CROSS = 0.08        # 팔이 몸 가운데를 넘어 반대쪽으로 갈 수 있는 거리 [m]
MAX_FROM_READY = 0.45       # 준비 자세 집게 가운데 → 잡기 전 위치 최대 [m]
IK_POS_TOL, IK_ROT_TOL = 0.003, math.radians(2.0)
JUMP_MAX = 0.08             # 경로 샘플 사이 관절 변화 최대 [rad] — 넘으면 IK 해가 튄 것
STREAM_HZ = 50.0
MIN_SEG_S = 1.0

# 그리퍼 겉 점 (TCP 프레임, m) — 바닥 거리 확인용. 몸통: TCP − 0.123 ~ − body_back, 높이 ± body_h_half, 폭 ± 39 mm
#   손가락: 몸통 앞면 ~ 손가락 끝, 높이 ± pad_h_half, 폭 ± 45 mm (벌렸을 때 손가락 바깥 — 어림)
_mm = lambda k: float(P[k]) / 1000.0
ENVELOPE = np.array([[x, y, z] for x in (-0.123, -_mm("body_back_mm")) for y in (-0.039, 0.039)
                     for z in (-_mm("body_h_half_mm"), _mm("body_h_half_mm"))] +
                    [[x, y, z] for x in (-_mm("body_back_mm"), 0.0, _mm("tip_beyond_mm")) for y in (-0.045, 0.045)
                     for z in (-_mm("pad_h_half_mm"), _mm("pad_h_half_mm"))])


# ---------------- 그리퍼 q ↔ mm ----------------
def gap_mm(q):
    return float(q) * MM_PER_RAD


def q_for_gap(mm):
    return min(Q_HI, max(Q_LO, float(mm) / MM_PER_RAD))


# ---------------- 모델 ----------------
class Model:
    def __init__(self):
        full = pin.buildModelFromUrdf(robot_env.URDF_PATH)
        lock = [full.getJointId(n) for n in robot_env.CFG["ik"]["lock_joints"] if full.existJointName(n)]
        m = pin.buildReducedModel(full, lock, np.zeros(full.nq))
        self.fid = {}
        for side, jn in zip(SIDES, robot_env.CFG["ik"]["ee_joints"]):
            jid = m.getJointId(jn)
            m.addFrame(pin.Frame(f"{side}_tcp", jid, pin.SE3(I3, TCP_OFF), pin.FrameType.OP_FRAME))
            self.fid[side] = m.getFrameId(f"{side}_tcp")
        self.q_slot = [ARM.index(int(J["map"][m.names[j]])) for j in range(1, m.njoints)]
        self.qname = {m.joints[j].idx_q: m.names[j] for j in range(1, m.njoints)}
        self.jid = {s: {n: m.getJointId(f"{s}_{n}_joint") for n in ("elbow", "wrist_roll", "wrist_yaw")} for s in SIDES}
        self.idx = {s: [m.joints[j].idx_q for j in range(1, m.njoints) if m.names[j].startswith(s + "_")] for s in SIDES}
        if m.nq != 14 or sorted(self.q_slot) != list(range(14)) or any(len(v) != 7 for v in self.idx.values()):
            sys.exit(f"[pick] 축소 모델이 팔 14 축이 아님 (nq {m.nq})")
        self.m, self.d = m, m.createData()
        self.lo, self.hi = m.lowerPositionLimit.copy(), m.upperPositionLimit.copy()

    def to_model(self, q_ctrl):
        return np.asarray(q_ctrl, dtype=float)[self.q_slot]

    def to_ctrl(self, qm):
        q = np.zeros(14)
        q[self.q_slot] = qm
        return q

    def tcp(self, qm, side):
        pin.framesForwardKinematics(self.m, self.d, qm)
        return self.d.oMf[self.fid[side]].copy()

    def ik(self, qm0, side, p, R, iters=300):
        """그 팔 7 축만 (감쇠 최소제곱 + 관절 한계). → (qm, 위치 오차 m, 회전 오차 rad)"""
        m, d, f, ix = self.m, self.d, self.fid[side], self.idx[side]
        qm = qm0.copy()
        for _ in range(iters):
            pin.framesForwardKinematics(m, d, qm)
            pin.computeJointJacobians(m, d, qm)
            M = d.oMf[f]
            e = np.r_[p - M.translation, pin.log3(R @ M.rotation.T)]
            if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
                break
            Jf = pin.getFrameJacobian(m, d, f, pin.LOCAL_WORLD_ALIGNED)[:, ix]
            dq = Jf.T @ np.linalg.solve(Jf @ Jf.T + 1e-4 * np.eye(6), e)
            qm[ix] = np.clip(qm[ix] + np.clip(dq, -0.1, 0.1), self.lo[ix], self.hi[ix])
        M = self.tcp(qm, side)
        return qm, float(np.linalg.norm(p - M.translation)), float(np.linalg.norm(pin.log3(R @ M.rotation.T)))

    def body_gap(self, qm, side):
        """몸 앞 영역(|y| < body_half_y, z > body_z_min) 에 있는 팔꿈치 · 손목 · 그리퍼 겉 점의 x − 몸통 앞면 x [m]. 없으면 None."""
        pin.forwardKinematics(self.m, self.d, qm)
        pts = [self.d.oMi[self.jid[side][n]].translation for n in ("elbow", "wrist_roll", "wrist_yaw")]
        T = self.tcp(qm, side)
        pts += list(ENVELOPE @ T.rotation.T + T.translation)
        xs = [p[0] for p in pts if abs(p[1]) < float(P["body_half_y"]) and p[2] > float(P["body_z_min"])]
        return (min(xs) - float(P["body_front_x"])) if xs else None

    def lowest(self, qm, side):
        """그리퍼 겉 점 중 가장 낮은 z (pelvis)."""
        M = self.tcp(qm, side)
        return float(np.min((ENVELOPE @ M.rotation.T + M.translation)[:, 2]))


def at_ready(mdl, qm, side):
    T = mdl.tcp(qm, side)
    return (np.linalg.norm(T.translation - READY_TCP[side]) < AT_READY_POS and
            np.linalg.norm(pin.log3(READY_R @ T.rotation.T)) < AT_READY_ROT)


def default_q_ctrl():
    if robot_env.CFG.get("default_arm_deg") is None:
        return None
    return np.radians(np.asarray(robot_env.CFG["default_arm_deg"], dtype=float))


def _sweep(mdl, side, segs):
    """관절 보간 구간들 → (몸 앞 FRONT_X 넘는 곳의 그리퍼 가장 낮은 z 또는 None, 가장 앞 x)."""
    low, mx = None, -9.0
    for sg in segs:
        for q in sg.qs:
            T = mdl.tcp(q, side)
            e = ENVELOPE @ T.rotation.T + T.translation
            mx = max(mx, float(e[:, 0].max()))
            if e[:, 0].max() > FRONT_X:
                lz = float(e[:, 2].min())
                low = lz if low is None else min(low, lz)
    return low, mx


def plan_pose(mdl, q_now_ctrl, side, which):
    """그 팔만 준비 자세 / 기본 자세로 (관절 보간, 다른 팔 그대로). 바로 가면 그리퍼가 몸 앞 낮은 곳을 지나는 경우
    경유 자세(gripper.yaml via_tcp_m)를 거침. → (구간 목록, 정보 dict) 또는 (None, 이유)."""
    q0 = mdl.to_model(q_now_ctrl)
    if which == "ready":
        if at_ready(mdl, q0, side):
            return None, "이미 준비 자세"
        q1, ep, er = mdl.ik(q0, side, READY_TCP[side], READY_R, iters=600)
        if ep > IK_POS_TOL or er > IK_ROT_TOL:
            return None, f"준비 자세 IK 실패 ({ep * 1000:.1f} mm)"
        name = "준비 자세로"
    else:
        dq = default_q_ctrl()
        if dq is None:
            return None, "robot.yaml default_arm_deg 없음"
        q1 = q0.copy()
        q1[mdl.idx[side]] = mdl.to_model(dq)[mdl.idx[side]]
        name = "기본 자세로"
    segs = [joint_seg(name, q0, q1, n=80)]
    low, mx = _sweep(mdl, side, segs)
    via = False
    if low is not None and low < FRONT_LOW:
        qv, ep, er = mdl.ik(q0 if which == "ready" else q1, side, VIA_TCP[side], VIA_R, iters=600)
        if ep > IK_POS_TOL or er > IK_ROT_TOL:
            return None, f"경유 자세 IK 실패 ({ep * 1000:.1f} mm) — 바로 가면 그리퍼가 몸 앞 z {low:.2f} 까지 내려감"
        segs = [joint_seg(f"{name} (경유 자세까지)", q0, qv, n=80), joint_seg(f"{name} (경유 자세에서)", qv, q1, n=80)]
        low, mx = _sweep(mdl, side, segs)
        via = True
    info = {"sec": round(sum(sg.sec for sg in segs), 1), "low_front_z": None if low is None else round(low, 3),
            "max_x": round(mx, 3), "via": via}
    return segs, info


def rpy_matrix(rpy_deg):
    r, p, y = (math.radians(float(v)) for v in rpy_deg)
    return pin.rpy.rpyToMatrix(r, p, y)


# ---------------- 경로 ----------------
SPEED = {"f": 1.0}      # 속도 배율 (웹에서 고름) — 계획할 때 v_lin · v_slow · v_joint 와 서버 상한에 곱함 (서버 상한은 3.0 rad/s 까지)
SPEED_MAX = 3.0


def set_speed(f):
    f = float(f)
    if not (0.25 <= f <= SPEED_MAX):
        raise ValueError(f"속도 배율 0.25–{SPEED_MAX}")
    SPEED["f"] = f


def seg_vmax():
    return min(3.0, float(P["server_max_vel"]) * SPEED["f"])


class Seg:
    """팔 한 구간: 관절 경로 (모델 순서 샘플 목록) + 시간."""
    def __init__(self, name, qs, sec, kind, clear_min):
        self.name, self.qs, self.sec, self.kind, self.clear_min = name, qs, sec, kind, clear_min
        self.vmax = seg_vmax()             # arm_server /stream 관절 속도 상한 (계획할 때 배율)


def seg_time(qs, length_m, v_lin, v_joint):
    """smoothstep 으로 샘플을 고르게 지나갈 때 (진행률 s 의 최고 속도 1.5 / T) 직선 속도와 '가장 빨리 움직이는 구간'의
    관절 속도가 상한을 넘지 않는 시간. (평균으로 정하면 손목이 몰려 움직이는 구간에서 서버가 속도를 자름)"""
    if len(qs) < 2:
        return MIN_SEG_S
    n = len(qs) - 1
    dq_ds = float(np.max(np.abs(np.diff(np.asarray(qs), axis=0)))) * n      # 샘플 사이 최대 관절 변화 / 진행률
    t = 1.5 * max(length_m / v_lin if v_lin else 0.0, dq_ds / v_joint)
    return max(MIN_SEG_S, t)


def joint_seg(name, q_from, q_to, n=40):
    qs = [q_from + (q_to - q_from) * (i / n) for i in range(n + 1)]
    return Seg(name, qs, seg_time(qs, 0.0, 0.0, float(P["v_joint"]) * SPEED["f"]), "joint", None)


JOINT_KO = {"shoulder_pitch": "어깨 pitch", "shoulder_roll": "어깨 roll", "shoulder_yaw": "어깨 yaw", "elbow": "팔꿈치",
            "wrist_roll": "손목 roll", "wrist_pitch": "손목 pitch", "wrist_yaw": "손목 yaw"}


def at_limits(mdl, qm, side, tol=math.radians(1.5)):
    out = []
    for i in mdl.idx[side]:
        if qm[i] - mdl.lo[i] < tol or mdl.hi[i] - qm[i] < tol:
            key = mdl.qname[i].replace(side + "_", "").replace("_joint", "")
            out.append(f"{JOINT_KO.get(key, key)} 한계 {math.degrees(mdl.lo[i]):.0f}~{math.degrees(mdl.hi[i]):.0f}°")
    return ", ".join(out)


def cart_seg(mdl, name, side, q_from, T_to, v_lin):
    """지금 q 의 집게 가운데 → T_to 직선 (위치 선형 · 회전 slerp), 2 mm · 0.5° 마다 IK. 실패 시 (None, 이유)."""
    T0 = mdl.tcp(q_from, side)
    dp = T_to.translation - T0.translation
    w = pin.log3(T_to.rotation @ T0.rotation.T)
    n = max(2, int(math.ceil(max(np.linalg.norm(dp) / 0.002, np.linalg.norm(w) / math.radians(0.5)))))
    qs, q = [q_from.copy()], q_from.copy()
    for i in range(1, n + 1):
        s = i / n
        p = T0.translation + s * dp
        R = pin.exp3(s * w) @ T0.rotation
        q2, ep, er = mdl.ik(q, side, p, R, iters=150)
        if ep > IK_POS_TOL or er > IK_ROT_TOL or np.max(np.abs(q2 - q)) > JUMP_MAX:
            lim = at_limits(mdl, q2, side)
            what = (f"닿지 않음 ({ep * 1000:.0f} mm · {math.degrees(er):.1f}°)" if ep > IK_POS_TOL or er > IK_ROT_TOL
                    else f"관절이 튐 ({math.degrees(np.max(np.abs(q2 - q))):.1f}°)")
            return None, (f"{name}: {s * 100:.0f}% 지점에서 {what}" + (f" — {lim}" if lim else "") +
                          " → RPY(pitch)·박스 위치를 바꿔 보세요")
        qs.append(q2)
        q = q2
    f = SPEED["f"]
    return Seg(name, qs, seg_time(qs, float(np.linalg.norm(dp)), v_lin * f, float(P["v_joint"]) * f), "cart", None), None


STEP_KO = {"@open": "그리퍼 벌림", "@close": "그리퍼 닫기", "@hold": "들고 있기"}


def plan_pick(mdl, q_now_ctrl, side, top, width_mm, height_mm, rpy_deg, bin_pt=None, bin_h_mm=None):
    """박스 윗면 가운데 top (pelvis, m) · 잡는 폭(집게 방향 = 로봇 좌우) · 높이 → 계획 dict. 실패면 {'ok': False, 'reasons': [...]}.
    bin_pt (통 안쪽을 클릭한 점, xy 만 씀) · bin_h_mm (테이블에서 통 윗면 테두리까지) 를 주면 제자리 대신 통에 넣음:
    들기(박스 아랫면이 테두리 3 cm 위) → 통 위로 → 박스 아랫면이 테두리 1 cm 위까지 내려 → 벌림 → 위로 → 준비 자세."""
    reasons, notes = [], []
    top = np.asarray(top, dtype=float)
    width_mm, height_mm = float(width_mm), float(height_mm)
    height = height_mm / 1000.0
    R = rpy_matrix(rpy_deg)
    if not (CLICK_X[0] <= top[0] <= CLICK_X[1]) or abs(top[1]) > CLICK_Y_MAX or not (CLICK_Z[0] <= top[2] <= CLICK_Z[1]):
        reasons.append(f"클릭 점 {np.round(top, 3).tolist()} 이 작업 범위 밖 (x {CLICK_X}, |y| ≤ {CLICK_Y_MAX}, z {CLICK_Z}) — 깊이 확인")
    if (side == "right" and top[1] > MIDLINE_CROSS) or (side == "left" and top[1] < -MIDLINE_CROSS):
        reasons.append(f"{'오른' if side == 'right' else '왼'}팔로 몸 반대쪽 {abs(top[1]) * 100:.0f} cm — 팔끼리·몸통 충돌 위험, 다른 손을 고르세요")
    floor_z = top[2] - height
    low_off = float(np.min((ENVELOPE @ R.T)[:, 2]))            # 그리퍼 가장 낮은 점 − TCP 높이 (이 자세에서)
    pad_v = (abs(R[2, 0]) * P["pad_len_half_mm"] + abs(R[2, 2]) * P["pad_h_half_mm"]) / 1000.0   # 패드 위아래 반폭
    z_c = top[2] - height / 2                                    # 박스 가운데 높이
    z_min = floor_z + P["tip_clear_mm"] / 1000.0 - low_off       # 그리퍼가 바닥에 안 닿는 가장 낮은 TCP
    z_g = max(z_c, z_min)
    overlap = min(top[2], z_g + pad_v) - max(floor_z, z_g - pad_v)
    if overlap * 1000 < P["min_overlap_mm"]:
        reasons.append(f"이 자세(RPY {list(rpy_deg)})로는 그리퍼 몸통이 바닥에 닿아 박스를 {max(0.0, overlap) * 1000:.0f} mm 만 쥠"
                       f" (최소 {P['min_overlap_mm']} mm) — pitch 를 키우거나 더 높은 박스 (높이 {height_mm:.0f} mm)")
    open_mm = min(width_mm + P["open_margin_mm"], gap_mm(Q_HI))
    side_room = (open_mm - width_mm) / 2
    if side_room < 8:
        reasons.append(f"폭 {width_mm:.0f} mm — 벌림 최대 {gap_mm(Q_HI):.0f} mm 에서 한쪽 여유 {side_room:.1f} mm (8 mm 이상 필요)")
    close_mm = max(width_mm - P["squeeze_mm"], gap_mm(Q_LO))
    grasp_p = np.array([top[0], top[1], z_g])
    pre_p = grasp_p + np.array([0.0, 0.0, P["approach_m"]])      # 위에서 수직으로 내려감
    lift_p = grasp_p + np.array([0.0, 0.0, P["lift_m"]])
    place_p = grasp_p + np.array([0.0, 0.0, 0.003])
    bias = TCP_BIAS[side]                                        # 보내는 목표 = 원하는 자리 − 치우침
    cmd = lambda p: np.asarray(p, dtype=float) - bias
    T = lambda p, Rm=R: pin.SE3(Rm, np.asarray(p, dtype=float))
    out = {"ok": False, "side": side, "top": top.round(4).tolist(), "width_mm": width_mm, "height_mm": height_mm, "rpy_deg": list(map(float, rpy_deg)),
           "floor_z": round(floor_z, 4), "overlap_mm": round(overlap * 1000, 1), "grasp": grasp_p.round(4).tolist(), "pre": pre_p.round(4).tolist(),
           "open_mm": round(open_mm, 1), "close_mm": round(close_mm, 1), "q_open": round(q_for_gap(open_mm), 3),
           "q_close": round(q_for_gap(close_mm), 3), "reasons": reasons, "notes": notes}
    if reasons:
        return out
    if bias.any():
        notes.append(f"{'왼' if side == 'left' else '오른'}손 치우침 보정 {np.round(bias * 1000).astype(int).tolist()} mm (gripper.yaml tcp_bias_m)")
    if np.linalg.norm(pre_p - READY_TCP[side]) > MAX_FROM_READY:
        reasons.append(f"준비 자세에서 {np.linalg.norm(pre_p - READY_TCP[side]) * 100:.0f} cm — {MAX_FROM_READY * 100:.0f} cm 이내로 박스를 가까이")
        return out

    q0 = mdl.to_model(q_now_ctrl)
    if not at_ready(mdl, q0, side):
        reasons.append(f"{'왼' if side == 'left' else '오른'}팔이 준비 자세가 아님 — 먼저 [준비 자세로]")
        return out
    q_ready = q0
    clear_bin = None
    if bin_pt is not None:
        bin_pt = np.asarray(bin_pt, dtype=float)
        rim_z = floor_z + float(bin_h_mm) / 1000.0
        d_b = z_g - floor_z                                           # 잡은 자리 TCP ↔ 박스 아랫면
        carry_z = max(lift_p[2], rim_z + 0.03 + d_b, rim_z + 0.03 - low_off)
        rel_z = max(rim_z + 0.01 + d_b, rim_z + 0.015 - low_off)
        bxy = bin_pt[:2]
        if not (CLICK_X[0] <= bxy[0] <= CLICK_X[1]) or abs(bxy[1]) > CLICK_Y_MAX:
            reasons.append(f"통 클릭 점 {np.round(bxy, 3).tolist()} 이 작업 범위 밖")
        if (side == "right" and bxy[1] > MIDLINE_CROSS) or (side == "left" and bxy[1] < -MIDLINE_CROSS):
            reasons.append(f"통이 {'오른' if side == 'right' else '왼'}팔 반대쪽 {abs(bxy[1]) * 100:.0f} cm — 다른 손을 고르거나 통을 옮기세요")
        if np.linalg.norm(bxy - top[:2]) < 0.06:
            reasons.append("통과 박스가 6 cm 안 — 통 가운데를 다시 클릭")
        if reasons:
            return out
        lift_p = np.array([grasp_p[0], grasp_p[1], carry_z])
        over_p = np.array([bxy[0], bxy[1], carry_z])
        rel_p = np.array([bxy[0], bxy[1], rel_z])
        clear_bin = rim_z + 0.012
        out.update(bin=bin_pt.round(4).tolist(), rim_z=round(rim_z, 4), release=rel_p.round(4).tolist())
        steps = [("접근 위치로", T(cmd(pre_p)), P["v_lin"], "transit"),
                 ("내려가기", T(cmd(grasp_p)), P["v_slow"], "grasp"),
                 ("@close", None, None, None),
                 ("들기", T(cmd(lift_p)), P["v_slow"], "grasp"),
                 ("통 위로", T(cmd(over_p)), P["v_lin"], "bin"),
                 ("통에 넣는 높이로", T(cmd(rel_p)), P["v_slow"], "bin"),
                 ("@open", None, None, None),
                 ("위로", T(cmd(over_p)), P["v_slow"], "bin"),
                 ("준비 자세로 복귀", T(READY_TCP[side], READY_R), P["v_lin"], "transit")]
    else:
      steps = [("접근 위치로", T(cmd(pre_p)), P["v_lin"], "transit"),
             ("내려가기", T(cmd(grasp_p)), P["v_slow"], "grasp"),
             ("@close", None, None, None),
             ("들기", T(cmd(lift_p)), P["v_slow"], "grasp"),
             ("@hold", None, None, None),
             ("내려놓기", T(cmd(place_p)), P["v_slow"], "grasp"),
             ("@open", None, None, None),
             ("빼기", T(cmd(pre_p)), P["v_slow"], "grasp"),
             ("준비 자세로 복귀", T(READY_TCP[side], READY_R), P["v_lin"], "transit")]
    q = q_ready
    seq = [("@open", None)]
    for name, Tt, v, kind in steps:
        if name.startswith("@"):
            seq.append((name, None))
            continue
        sg, why = cart_seg(mdl, name, side, q, Tt, v)
        if sg is None:
            reasons.append(why)
            return out
        sg.clear_min = (floor_z + (P["tip_clear_mm"] - 2) / 1000.0 if kind == "grasp" else
                        clear_bin if kind == "bin" else floor_z + P["transit_clear_mm"] / 1000.0)
        seq.append(("arm", sg))
        q = sg.qs[-1]
    # 바닥 거리 (모든 샘플 · 그리퍼 겉 점)
    for kind, sg in seq:
        if kind != "arm":
            continue
        cmin = sg.clear_min if sg.clear_min is not None else floor_z + P["transit_clear_mm"] / 1000.0
        gaps = [g for g in (mdl.body_gap(qq, side) for qq in sg.qs[::3]) if g is not None]
        if gaps and min(gaps) < float(P["body_clear_m"]):
            reasons.append(f"{sg.name}: 팔 · 그리퍼가 몸통 앞면에 {max(0.0, min(gaps)) * 100:.0f} cm 까지 (최소 {float(P['body_clear_m']) * 100:.0f} cm)"
                           " — 통 · 박스를 몸에서 더 멀리 (통 가운데 pelvis 앞 0.40 m 이상)")
        low = min(mdl.lowest(qq, side) for qq in sg.qs)
        if low < cmin - 1e-4:
            ref, what = ((out["rim_z"], "통 테두리") if sg.clear_min == clear_bin and clear_bin is not None else (floor_z, "바닥(박스 아랫면)"))
            reasons.append(f"{sg.name}: 그리퍼가 {what} z {ref:.3f} 에 {(low - ref) * 1000:.0f} mm 까지 내려감"
                           f" (최소 {(cmin - ref) * 1000:.0f} mm)")
    if reasons:
        return out
    out.update(ok=True, seq=seq, q_start=q0, q_ready=q_ready,
               total_s=round(sum(sg.sec for k, sg in seq if k == "arm") + (0 if bin_pt is not None else P["hold_s"]) + 3 * 2.0, 1),
               steps=[(STEP_KO.get(k, k) if k != "arm" else sg.name, None if k != "arm" else round(sg.sec, 1)) for k, sg in seq])
    if side_room < 12:
        notes.append(f"한쪽 여유 {side_room:.1f} mm — 위치 오차가 크면 집게가 박스에 닿을 수 있음")
    return out


def plan_home(mdl, q_now_ctrl, side, floor_z):
    """멈춘 자리 → 위로 (바닥 + 15 cm 까지) → 준비 자세."""
    q = mdl.to_model(q_now_ctrl)
    T_now = mdl.tcp(q, side)
    seq = []
    up_z = max(T_now.translation[2], (floor_z if floor_z is not None else T_now.translation[2]) + 0.15)
    if up_z - T_now.translation[2] > 0.005:
        sg, why = cart_seg(mdl, "위로", side, q, pin.SE3(T_now.rotation, T_now.translation + [0, 0, up_z - T_now.translation[2]]), P["v_slow"])
        if sg is None:
            return None, why
        seq.append(("arm", sg))
        q = sg.qs[-1]
    sg, why = cart_seg(mdl, "준비 자세로", side, q, pin.SE3(READY_R, READY_TCP[side]), P["v_lin"])
    if sg is None:
        return None, why
    seq.append(("arm", sg))
    return seq, None


# ---------------- arm_server ----------------
def call(path, body=None, timeout=5.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(ARM_SERVER + path, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def arm_ready_check():
    """arm_server 상태 확인 → (ok, 메시지, 지금 목표 q 제어 순서 rad)."""
    try:
        st = call("/status", timeout=2.0)
    except Exception as e:  # noqa: BLE001
        return False, f"arm_server(50022) 응답 없음 ({e})", None
    if not st.get("ready") or st.get("mode") != "hold" or float(st.get("weight", 0)) < 0.99:
        return False, f"arm_server 가 hold · weight 1 아님 ({st})", None
    if st.get("busy") or (st.get("stream") or {}).get("active"):
        return False, f"arm_server 가 다른 동작 중 ({st})", None
    pose = call("/pose")
    return True, "ok", np.radians(np.asarray(pose["arm_target_deg"], dtype=float))


LAST_CMD = {"q": None, "t": 0.0}    # 마지막으로 arm_server 에 보낸 팔 14 축 목표 (제어 순서, rad) — 녹화의 action


def run_seg(mdl, sg, stop_evt, on_tick=None):
    """구간 하나를 arm_server /stream 으로. → (ok, 메시지). stop_evt 가 켜지면 그 자리에서 멈춤."""
    b = call("/stream/begin", {"max_vel": float(getattr(sg, "vmax", P["server_max_vel"]))})
    tok = b["token"]
    start_ctrl = np.radians(np.asarray(b["target_deg"], dtype=float))
    LAST_CMD.update(q=start_ctrl.copy(), t=time.time())
    first_ctrl = mdl.to_ctrl(sg.qs[0])
    off = float(np.max(np.abs(start_ctrl - first_ctrl)))
    n = len(sg.qs) - 1
    msg, ok, n_clip = "완료", True, 0
    try:
        if off > math.radians(3.0):
            return False, f"{sg.name}: 팔 자세가 계획과 {math.degrees(off):.1f}° 다름 — 다시 계획하세요"
        t0 = time.time()
        while True:
            if stop_evt.is_set():
                return False, f"{sg.name}: 정지 — 그 자리에서 멈춤"
            t = time.time() - t0
            s = min(max(t / sg.sec, 0.0), 1.0)
            f = s * s * (3 - 2 * s) * n
            i = min(int(f), n - 1) if n > 0 else 0
            qm = sg.qs[i] + (sg.qs[min(i + 1, n)] - sg.qs[i]) * (f - i) if n > 0 else sg.qs[0]
            r = call("/stream/cmd", {"token": tok, "deg": np.degrees(mdl.to_ctrl(qm)).tolist()}, timeout=1.0)
            LAST_CMD.update(q=np.radians(np.asarray(r["target_deg"], dtype=float)), t=time.time())   # 서버가 자른 뒤 실제로 보낸 값
            n_clip += int(bool(r.get("vel_clip")))
            lag = float(np.max(np.abs(np.asarray(r["target_deg"]) - np.asarray(r["meas_deg"]))))
            if lag > float(P["track_abort_deg"]):
                return False, f"{sg.name}: 명령과 실측이 {lag:.1f}° 차이 — 걸렸거나 부딪힘, 그 자리에서 멈춤"
            if on_tick:
                on_tick(s, lag)
            if t >= sg.sec + 0.3:
                break
            time.sleep(1.0 / STREAM_HZ)
        if n_clip:
            msg = f"완료 (서버 속도 자름 {n_clip} 회)"
        return ok, msg
    finally:
        try:
            call("/stream/end", {"token": tok}, timeout=2.0)
        except Exception:  # noqa: BLE001 — 서버가 2 초 뒤 세션을 닫음, 팔은 마지막 목표에서 멈춤
            pass
