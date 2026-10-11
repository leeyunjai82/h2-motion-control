#!/usr/bin/env python3
"""
grab_reach.py — 박스 위치별로 잡기 시퀀스를 '서버와 같은 IK' 로 풀어 손이 끝까지 가는지 확인 (오프라인, 로봇 불필요)

  ROBOT=h2 python utils/grab_reach.py                       # 기본 격자 (place, 윗면 +0.15/+0.20/+0.25, x 0.35–0.60)
  ROBOT=h2 python utils/grab_reach.py --mode center         # 건네기
  ROBOT=h2 python utils/grab_reach.py --x 0.45 --y 0.05 --top 0.20 -v   # 한 점, 단계별 오차 출력
  ROBOT=h2 python utils/grab_reach.py --frac 0 0.25 --D 0.20        # grab.grab_inset_frac 값별 비교 (박스 깊이 D)
  ROBOT=h2 python utils/grab_reach.py --inset 0 0.03 0.05            # D 미측정 고정값(grab.grab_inset_x) 비교 (frac 0)
  ROBOT=h2 python utils/grab_reach.py --z-offset -0.064             # grab.z_offset 대신 이 값
  ROBOT=h2 python utils/grab_reach.py --yaw 0 10 20 25               # 비스듬한 박스 (위에서 볼 때 반시계 +, 박스 중심 기준 회전)
  ROBOT=h2 python utils/grab_reach.py --yaw 20 --no-follow           # grab.box_yaw_follow 끔 (손목이 박스 회전을 안 따라감)

  방법
   - robot_server.GrabController 를 그대로 써서 대기자세(ready) → grab_box → place/center 의 손 목표를 기록
     (박스 L/R/윗면 중심은 sim_server 와 같은 가정: 윗면 좌우 변 중점에서 안쪽 2 cm)
   - 각 이동을 arm_controller_wrapper.move_hands 처럼 보간(smoothstep, 25 Hz)하며 robot_arm_ik.ArmIK.solve_ik
     (casadi, 서버와 같은 비용·관절 한계·스무딩 필터) 로 풀고, 이동 끝의 관절각 FK 와 목표의 차이를 잰다
   - 시작 팔 자세 = robot.yaml default_arm_deg
  판정: 모든 단계에서 위치 오차 < 2 cm, 손 자세 오차 < 10° 이면 O
  좌표: IK(pelvis) 기준. 박스 윗면 top 은 pelvis 위 높이 [m] (H2 pelvis ≈ 바닥 1.01 m → top 0.20 = 바닥 1.21 m)
"""
import argparse
import os
import sys
import time

os.environ.setdefault("ROBOT_SIM", "1")       # 오프라인 — DDS·실기 경로 사용 안 함
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "common"))
import robot_env   # noqa: E402

import numpy as np   # noqa: E402
import pinocchio as pin   # noqa: E402

POS_OK_CM = 2.0
ROT_OK_DEG = 10.0
HZ = 25


def build():
    import robot_server as rsv
    from ctrl.robot_arm_ik import ArmIK
    rsv.time.sleep = lambda *_a, **_k: None          # 시퀀스 안의 대기 생략 (기록만)
    return rsv, ArmIK()


def box_cam(rsv, x, y, top, W, yaw_deg=0.0):
    """IK(pelvis) 기준 박스 → grab_box 입력(카메라 좌표 L, R, 윗면 중심). sim_server 와 같은 가정.
    yaw_deg: 박스 회전 (위에서 볼 때 반시계 +) — L/R = 회전한 좌우 변 중점에서 안쪽 2 cm."""
    inset = 0.02
    p2t = np.array(rsv.PELVIS_TO_TORSO)
    cx_, cy_, cz_, cp = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH
    c, s = np.cos(cp), np.sin(cp)

    def to_cam(P):
        X, Y, Z = np.asarray(P, float) - p2t
        cy_r, cz_r = cz_ - Z, X - cx_
        return [float(cy_ - Y), float(c * cy_r - s * cz_r), float(s * cy_r + c * cz_r)]
    ya = np.radians(yaw_deg)
    hw = W / 2 - inset
    return (to_cam((x - np.sin(ya) * hw, y + np.cos(ya) * hw, top)),
            to_cam((x + np.sin(ya) * hw, y - np.cos(ya) * hw, top)), to_cam((x, y, top)))


WRIST = None   # (roll, pitch, yaw) deg — 박스 페이지 Wrist RPY 와 같은 값 (양손 같게)


def record(rsv, mode, x, y, top, W, H, D=None, yaw=0.0):
    rec = []
    g = rsv.GrabController(arm=None, speak=lambda *_a, **_k: None, robot_available=False)
    if WRIST is not None:
        for side in ("left", "right"):
            g.wrist_params[side] = {"roll": WRIST[0], "pitch": WRIST[1], "yaw": WRIST[2]}
    g.redetect = None
    g.handover_direction = mode

    def mv(L, R, dur, msg="", left_rot=None, right_rot=None):
        rec.append((msg, np.array(L, float), np.array(R, float), left_rot, right_rot, float(dur)))
        return True
    g._move = mv
    g.ready()
    L, R, C = box_cam(rsv, x, y, top, W, yaw)
    g.grab_box(L, R, box_h_m=H, top_center_cam=C, box_d_m=D)
    return rec


def run(ik, rec, q0, verbose=False):
    m, d = ik.reduced_robot.model, ik.reduced_robot.data
    fl, fr = ik.L_hand_id, ik.R_hand_id
    q = q0.copy()
    ik.init_data = q0.copy()
    if hasattr(ik, "smooth_filter"):
        for _ in range(ik.smooth_filter._window_size if hasattr(ik.smooth_filter, "_window_size") else 4):
            ik.smooth_filter.add_data(q0.copy())
    worst = (0.0, 0.0, "")
    rows = []
    for msg, tL, tR, rL, rR, dur in rec:
        pin.framesForwardKinematics(m, d, q)
        sLp, sRp = d.oMf[fl].translation.copy(), d.oMf[fr].translation.copy()
        sLr, sRr = pin.Quaternion(d.oMf[fl].rotation), pin.Quaternion(d.oMf[fr].rotation)
        rL = rL if rL is not None else pin.Quaternion(1, 0, 0, 0)
        rR = rR if rR is not None else pin.Quaternion(1, 0, 0, 0)
        n = max(1, int(dur * HZ))
        for i in range(n + 1):
            t = (i / n) ** 2 * (3 - 2 * i / n)
            q, _ = ik.solve_ik(pin.SE3(sLr.slerp(t, rL), sLp + t * (tL - sLp)).homogeneous,
                               pin.SE3(sRr.slerp(t, rR), sRp + t * (tR - sRp)).homogeneous, q, None)
        pin.framesForwardKinematics(m, d, q)
        eL = np.linalg.norm(d.oMf[fl].translation - tL) * 100
        eR = np.linalg.norm(d.oMf[fr].translation - tR) * 100
        aL = np.degrees(np.linalg.norm(pin.log3(rL.toRotationMatrix().T @ d.oMf[fl].rotation)))
        aR = np.degrees(np.linalg.norm(pin.log3(rR.toRotationMatrix().T @ d.oMf[fr].rotation)))
        e, a = max(eL, eR), max(aL, aR)
        rows.append((msg, e, a, tL, tR))
        if e > worst[0] or (e == worst[0] and a > worst[1]):
            worst = (e, a, msg)
        if verbose:
            print(f"   {msg:28s} 목표 L {np.round(tL, 3)} R {np.round(tR, 3)}   오차 {e:5.1f} cm  {a:5.1f}°")
    ok = all(e < POS_OK_CM and a < ROT_OK_DEG for _, e, a, _, _ in rows)
    return ok, worst, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("place", "center"), default="place")
    ap.add_argument("--x", type=float, nargs="*", default=[0.35, 0.40, 0.45, 0.50, 0.55, 0.60], help="박스 중심 x (IK)")
    ap.add_argument("--y", type=float, nargs="*", default=[0.0, 0.05], help="박스 중심 y (대칭이라 + 만)")
    ap.add_argument("--top", type=float, nargs="*", default=[0.15, 0.20, 0.25], help="박스 윗면 z (pelvis 위)")
    ap.add_argument("--W", type=float, default=0.28, help="박스 폭(손 사이 방향) [m]")
    ap.add_argument("--H", type=float, default=0.18, help="박스 높이 [m] (잡는 높이 = 윗면 − H/2 + z_offset)")
    ap.add_argument("--D", type=float, default=0.20, help="박스 깊이(좌/우 옆면 길이) [m] — grab_inset_frac 에 쓰임. 0 = 미측정")
    ap.add_argument("--wrist", default=None, help="손목 RPY deg 'roll,pitch,yaw' (박스 페이지 Wrist RPY 와 같음, 양손 같게)")
    ap.add_argument("--frac", type=float, nargs="*", default=None, help="grab.grab_inset_frac 대신 이 값들로 (여러 개면 각각 표)")
    ap.add_argument("--inset", type=float, nargs="*", default=None, help="grab.grab_inset_x 대신 이 값들로 [m], frac 0 (여러 개면 각각 표)")
    ap.add_argument("--z-offset", type=float, default=None, help="grab.z_offset 대신 이 값 [m]")
    ap.add_argument("--yaw", type=float, nargs="*", default=[0.0], help="박스 회전 [deg] (위에서 볼 때 반시계 +), 여러 개면 각각 표")
    ap.add_argument("--no-follow", action="store_true", help="grab.box_yaw_follow 끔 (손목이 박스 회전을 안 따라감)")
    ap.add_argument("-v", action="store_true", help="단계별 오차")
    a = ap.parse_args()
    global WRIST
    if a.wrist:
        WRIST = tuple(float(v) for v in a.wrist.split(","))

    t0 = time.time()
    rsv, ik = build()
    if a.z_offset is not None:
        rsv.GRAB_Z_OFFSET = a.z_offset
    if a.no_follow:
        rsv.BOX_YAW_FOLLOW = False
    q0 = np.radians(np.array(robot_env.CFG["default_arm_deg"], dtype=float))
    print(f"[reach] ROBOT={robot_env.ROBOT} mode={a.mode} 박스 W {a.W} H {a.H}  z_offset {rsv.GRAB_Z_OFFSET} "
          f"handover_x {rsv.HANDOVER_X}  wrist {WRIST or '0,0,0'}  판정 < {POS_OK_CM:.0f} cm / {ROT_OK_DEG:.0f}°  (IK 준비 {time.time() - t0:.1f}s)")
    D = a.D if a.D > 0 else None
    if a.frac is not None:
        variants = [(f, rsv.GRAB_INSET_X) for f in a.frac]
    elif a.inset is not None:
        variants = [(0.0, v) for v in a.inset]
    else:
        variants = [(rsv.GRAB_INSET_FRAC, rsv.GRAB_INSET_X)]
    for frac, fixed in variants:
        rsv.GRAB_INSET_FRAC, rsv.GRAB_INSET_X = frac, fixed
        inset = rsv.grab_inset(D)
        for top, yaw in [(t, w) for t in a.top for w in a.yaw]:
            print(f"\n박스 윗면 pelvis+{top:.2f} m  (H2 바닥 기준 약 {1.01 + top:.2f} m)   "
                  f"손 x = 박스 중심 − {inset:.3f}  (frac {frac}, 고정 {fixed}, D {D})   박스 회전 {yaw:+.0f}° "
                  f"(손목 따라감 {'예' if rsv.BOX_YAW_FOLLOW else '아니오'})")
            print("   y\\x  " + "  ".join(f"{x:5.2f}" for x in a.x))
            for y in a.y:
                cells = []
                for x in a.x:
                    rec = record(rsv, a.mode, x, y, top, a.W, a.H, D, yaw)
                    if not any(r[0].startswith("⑥ 잡기") for r in rec):     # 잡기 전에 끝남 (회전 한계 등 — 서버가 거부)
                        cells.append(" 거부")
                        continue
                    ok, worst, rows = run(ik, rec, q0, verbose=False)
                    cells.append(" O   " if ok else f"{worst[0]:4.0f}c")
                    if a.v:
                        print(f"  x {x} y {y} top {top} yaw {yaw:+.0f}: {'O' if ok else 'X'}  최대 {worst[0]:.1f} cm {worst[1]:.0f}° @ {worst[2]}")
                        run(ik, record(rsv, a.mode, x, y, top, a.W, a.H, D, yaw), q0, verbose=True)
                print(f"  {y:+.2f} " + "  ".join(cells))
    print("\n  O = 모든 단계 위치 < 2 cm · 자세 < 10°.  숫자 = 가장 크게 빗나간 단계의 위치 오차 [cm] (-v 로 단계 확인)"
          "  거부 = 서버가 잡지 않음 (박스 회전 > grab.max_box_yaw_deg)")
    print(f"  ({time.time() - t0:.0f}s)")


if __name__ == "__main__":
    main()
