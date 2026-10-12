#!/usr/bin/env python3
"""
hand_to_marker.py — 카메라로 잰 마커 위치로 손을 보내 카메라→IK 좌표 변환을 실물로 확인

  준비: 로봇 FSM 703(stand), arm_server 실행 중 (ROBOT_CHECK=1 ROBOT=h2 ./start_editor.sh real), hold·weight 1
        카메라를 쓰는 다른 프로그램(rs_stream 등)은 꺼 둘 것 — 이 도구가 카메라를 직접 연다.
  실행: ROBOT_CHECK=1 ROBOT=h2 python utils/hand_to_marker.py --id 0              # 마커 위 10 cm
        ROBOT_CHECK=1 ROBOT=h2 python utils/hand_to_marker.py --id 0 --above 0.05  # 5 cm 위
        ROBOT_CHECK=1 ROBOT=h2 python utils/hand_to_marker.py --fake 0.45,-0.10,0.30 # 카메라 없이 IK 좌표 직접 (시험용)

  순서
   1) 마커 3D (depth, cam_marker_check 와 같은 방식) → robot.yaml camera 로 torso → IK(pelvis) 좌표
   1') 손이 목표에서 멀면(> 30 cm) 먼저 준비 자세로: 양손 IK (0.30, ±0.20, 0.20), 손바닥 마주보기 — 'yes' 확인, 4 초
   2) 손 선택: 마커가 오른쪽(y<0)이면 오른손, 왼쪽이면 왼손 (--hand 로 지정 가능). 다른 손은 지금 자세 유지
   3) 목표 = 마커 중심 바로 위 --above [m] (기본 0.10). 손 자세 = 손바닥 마주보기 (잡기와 같은 단위 회전)
   4) 범위·이동 거리 확인 → 'yes' 입력해야 이동 (3 초, arm_server /hands)
   5) 도착 후: 목표 vs 실제(관절각 FK) 오차 출력 → 실물에서 손 기준점과 마커 중심의 차이를 줄자로 재기
   6) Enter → 준비 자세 → 처음 자세로 복귀

  손 기준점 = 손목 yaw 관절에서 손 쪽으로 5 cm (robot.yaml ik.ee_offset, 잡기에서 쓰는 점과 같음)
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT 미지정이면 종료

import numpy as np

ARM_SERVER = "http://localhost:50022"
J = robot_env.JOINTS
ARM = [int(s) for s in J["arm"]]
CX, CY, CZ, CP = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH
P2T = np.array(robot_env.CFG["frames"]["pelvis_to_torso"], dtype=float)
MOVE_SEC = 3.0
# 안전 범위 (IK = pelvis 기준) — 몸통·다리 쪽이나 너무 먼 곳으로 보내지 않는다
X_RANGE = (0.25, 0.60)
Y_MAX = 0.35
Z_RANGE = (0.05, 0.60)
MAX_STEP = 0.30            # 손 위치에서 목표까지 한 번에 최대 [m] (준비 자세가 아닐 때)
MAX_STEP_READY = 0.40      # 준비 자세(알려진 안전 자세)에서 목표까지 최대 [m]
READY_L = np.array([0.30, 0.20, 0.20])   # 준비 자세 (IK 좌표) — 양손 앞으로, 손바닥 마주보기(단위 회전, 잡기와 같음)
READY_R = np.array([0.30, -0.20, 0.20])
READY_MAX = 0.50           # 지금 손에서 준비 자세까지 최대 [m]


def arm(path, body=None, timeout=10.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(ARM_SERVER + path, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def ik_model():
    """robot_arm_ik.py / joint_check.py 와 같은 축소 모델 → (model, data, fl, fr, q_slot)."""
    import pinocchio as pin
    full = pin.buildModelFromUrdf(robot_env.URDF_PATH)
    lock = [full.getJointId(n) for n in robot_env.CFG["ik"]["lock_joints"] if full.existJointName(n)]
    m = pin.buildReducedModel(full, lock, np.zeros(full.nq))
    off = np.array(robot_env.CFG["ik"]["ee_offset"], dtype=float)
    for nm, jn in zip(("L_ee", "R_ee"), robot_env.CFG["ik"]["ee_joints"]):
        m.addFrame(pin.Frame(nm, m.getJointId(jn), pin.SE3(np.eye(3), off), pin.FrameType.OP_FRAME))
    q_slot = [ARM.index(int(J["map"][m.names[j]])) for j in range(1, m.njoints)]
    return m, m.createData(), m.getFrameId("L_ee"), m.getFrameId("R_ee"), q_slot


def hands_now(k):
    import pinocchio as pin
    m, d, fl, fr, q_slot = k
    q = np.asarray(arm("/pose")["arm_rad"], dtype=float)[q_slot]
    pin.framesForwardKinematics(m, d, q)
    return d.oMf[fl].translation.copy(), d.oMf[fl].rotation.copy(), d.oMf[fr].translation.copy(), d.oMf[fr].rotation.copy()


def quat_wxyz(R):
    import pinocchio as pin
    qq = pin.Quaternion(R)
    return [float(qq.w), float(qq.x), float(qq.y), float(qq.z)]


def camera_to_torso(c):
    cx, cy, cz = c
    cos_p, sin_p = math.cos(CP), math.sin(CP)
    return np.array([(-cy * sin_p + cz * cos_p) + CX, -cx + CY, -(cy * cos_p + cz * sin_p) + CZ])


def marker_ik(mid, sec):
    """마커 중심 → IK(pelvis) 좌표 (depth 7×7 중앙값, 컬러 정렬). 못 찾으면 None."""
    import cv2
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, *robot_env.CAMERA_RES, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, *robot_env.CAMERA_RES, rs.format.z16, 30)
    prof = pipe.start(cfg)
    try:
        align = rs.align(rs.stream.color)
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        scale = prof.get_device().first_depth_sensor().get_depth_scale()
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                      cv2.aruco.DetectorParameters())
        pts = []
        t_end = time.time() + sec
        while time.time() < t_end:
            fs = align.process(pipe.wait_for_frames(1000))
            cf, df = fs.get_color_frame(), fs.get_depth_frame()
            if not cf or not df:
                continue
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(np.asanyarray(cf.get_data()), cv2.COLOR_BGR2GRAY))
            if ids is None:
                continue
            for c, i in zip(corners, ids.flatten()):
                if int(i) != mid:
                    continue
                u, v = c.reshape(4, 2).mean(axis=0)
                dimg = np.asanyarray(df.get_data())
                ui, vi = int(round(u)), int(round(v))
                w = dimg[max(0, vi - 3):vi + 4, max(0, ui - 3):ui + 4].astype(float)
                w = w[w > 0]
                if len(w):
                    pts.append(rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(np.median(w)) * scale))
        if not pts:
            return None, 0
        return camera_to_torso(np.median(np.array(pts), axis=0)) + P2T, len(pts)
    finally:
        pipe.stop()


def move(pl, rl, pr, rr, sec):
    arm("/hands", {"left_xyz": list(map(float, pl)), "right_xyz": list(map(float, pr)),
                   "left_quat": quat_wxyz(rl), "right_quat": quat_wxyz(rr), "duration": sec}, timeout=sec + 20)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0, help="마커 ID (기본 0)")
    ap.add_argument("--above", type=float, default=0.10, help="마커 위 높이 [m] (기본 0.10, 최소 0.03)")
    ap.add_argument("--hand", choices=("left", "right"), default=None, help="기본: 마커 y 부호로 선택")
    ap.add_argument("--sec", type=float, default=2.0, help="마커 측정 시간 [s]")
    ap.add_argument("--fake", default=None, help="카메라 없이 마커 IK 좌표 직접 'x,y,z' (시험용)")
    a = ap.parse_args()
    if a.above < 0.03:
        sys.exit("❌ --above 는 0.03 m 이상")

    try:
        st = arm("/status", timeout=2.0)
    except Exception as e:  # noqa: BLE001
        sys.exit(f"❌ arm_server(50022) 응답 없음 ({e}) — 다른 터미널에서 'ROBOT_CHECK=1 ROBOT=h2 ./start_editor.sh real' 먼저")
    if st.get("mode") != "hold" or float(st.get("weight", 0)) < 0.99:
        sys.exit(f"❌ arm_server 가 hold·weight 1 아님 ({st}) — /check 에서 [제어권 잡기]")

    k = ik_model()
    pl0, rl0, pr0, rr0 = hands_now(k)
    print(f"[hand] 지금 손 (IK 좌표) L {np.round(pl0, 3).tolist()}  R {np.round(pr0, 3).tolist()}")

    if a.fake:
        mk = np.array([float(v) for v in a.fake.split(",")])
        print(f"[hand] --fake 마커 IK {np.round(mk, 3).tolist()}")
    else:
        mk, n = marker_ik(a.id, a.sec)
        if mk is None:
            sys.exit(f"❌ 마커 ID {a.id} (또는 깊이) 안 보임")
        print(f"[hand] 마커 ID {a.id} (표본 {n})  IK(pelvis) x {mk[0]:.3f} y {mk[1]:+.3f} z {mk[2]:+.3f}"
              f"  |  torso z {mk[2] - P2T[2]:+.3f}")

    hand = a.hand or ("right" if mk[1] < 0 else "left")
    tgt = mk + np.array([0.0, 0.0, a.above])
    I3 = np.eye(3)                       # 손바닥 마주보기 (robot_server 잡기와 같은 단위 회전)
    cur = pr0 if hand == "right" else pl0
    step = float(np.linalg.norm(tgt - cur))
    used_ready = False
    at_ready = (np.linalg.norm(pl0 - READY_L) < 0.03 and np.linalg.norm(pr0 - READY_R) < 0.03)
    if step > MAX_STEP and not (at_ready and step <= MAX_STEP_READY):
        dl, dr = float(np.linalg.norm(READY_L - pl0)), float(np.linalg.norm(READY_R - pr0))
        print(f"[hand] 목표가 지금 손에서 {step * 100:.0f} cm — 먼저 준비 자세로: L {READY_L.tolist()} R {READY_R.tolist()}"
              f" (이동 L {dl * 100:.0f} / R {dr * 100:.0f} cm, 4 초, 손바닥 마주보기)")
        if max(dl, dr) > READY_MAX:
            sys.exit(f"❌ 준비 자세까지 {max(dl, dr) * 100:.0f} cm > {READY_MAX * 100:.0f} cm — 팔을 먼저 기본 자세로")
        if input("  준비 자세로 이동하려면 yes 입력: ").strip() != "yes":
            print("취소")
            return
        move(READY_L, I3, READY_R, I3, 4.0)
        time.sleep(0.5)
        used_ready = True
        cur = READY_R if hand == "right" else READY_L
        step = float(np.linalg.norm(tgt - cur))
    print(f"[hand] {hand} 손 → 마커 위 {a.above * 100:.0f} cm: 목표 {np.round(tgt, 3).tolist()}  (지금 손에서 {step * 100:.1f} cm)")

    bad = []
    if not (X_RANGE[0] <= tgt[0] <= X_RANGE[1]):
        bad.append(f"x {tgt[0]:.3f} 범위 {X_RANGE} 밖")
    if abs(tgt[1]) > Y_MAX:
        bad.append(f"|y| {abs(tgt[1]):.3f} > {Y_MAX}")
    if not (Z_RANGE[0] <= tgt[2] <= Z_RANGE[1]):
        bad.append(f"z {tgt[2]:.3f} 범위 {Z_RANGE} 밖 (테이블이 너무 낮거나 높음)")
    lim = MAX_STEP_READY if (used_ready or at_ready) else MAX_STEP
    if step > lim:
        bad.append(f"이동 {step * 100:.0f} cm > {lim * 100:.0f} cm (마커를 더 가까이)")
    if bad:
        sys.exit("❌ 이동 안 함: " + "; ".join(bad))

    if input("  이동하려면 yes 입력: ").strip() != "yes":
        print("취소")
        return
    base_l, base_r = (READY_L, READY_R) if used_ready else (pl0, pr0)
    rot_l, rot_r = (I3, I3) if used_ready else (rl0, rr0)
    if hand == "right":
        move(base_l, rot_l, tgt, I3, MOVE_SEC)
    else:
        move(tgt, I3, base_r, rot_r, MOVE_SEC)
    time.sleep(0.5)
    pl1, rl1, pr1, rr1 = hands_now(k)
    act = pr1 if hand == "right" else pl1
    e = act - tgt
    print(f"[hand] 도착 (관절각 FK) {np.round(act, 3).tolist()}  목표 대비 x {e[0] * 100:+.1f} y {e[1] * 100:+.1f} z {e[2] * 100:+.1f} cm"
          f"  (IK 추종 오차 — 크면 IK/게인 문제)")
    print(f"  실물 확인: 손 기준점(손목 yaw 관절에서 손 쪽 5 cm)이 마커 중심 바로 위 {a.above * 100:.0f} cm 에 있어야 함.")
    print("    줄자로 → 앞뒤 차이(로봇 앞쪽 +), 좌우 차이(로봇 왼쪽 +), 마커 면에서 높이 를 재서 알려 주세요.")
    input("  원래 자세로 돌아가려면 Enter ")
    if used_ready:
        move(READY_L, I3, READY_R, I3, MOVE_SEC)
        time.sleep(0.3)
    move(pl0, rl0, pr0, rr0, 4.0 if used_ready else MOVE_SEC)
    print("[hand] 복귀 완료")


if __name__ == "__main__":
    main()
