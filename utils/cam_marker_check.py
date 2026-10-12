#!/usr/bin/env python3
"""
cam_marker_check.py — ArUco 마커로 카메라 장착값(robot.yaml camera) 확인 (카메라만 사용, 로봇 명령 없음)

  마커: DICT_4X4_50, 한 변 45 mm (--size 로 변경)
  rs_stream.py 등 카메라를 쓰는 프로그램은 먼저 끌 것.  IMU 까지 쓰려면 sudo (또는 RealSense udev 규칙).

  ROBOT_CHECK=1 ROBOT=h2 python utils/cam_marker_check.py --id 0 --watch           # 계속 출력
  ROBOT_CHECK=1 ROBOT=h2 python utils/cam_marker_check.py --id 0 --points 0.45,0.55 # 2점 보정

  마커 3D 위치는 두 방법으로 구한다.
    depth : 마커 중심 픽셀의 D435i 깊이(컬러에 정렬, 7×7 중앙값) → 컬러 내부 파라미터로 역투영
            ← detect_box(박스 인식)가 실제로 쓰는 방식. 보정은 이 값 기준.
    pnp   : 마커 네 모서리 + 마커 크기로 solvePnP — 마커가 작고 멀거나 비스듬하면 거리 오차가 큼 (참고용)

  출력
   1) 렌즈 기준 수평 좌표 (robot.yaml pitch): 앞 / 좌 / 아래  ← 줄자와 비교
      아래 = 렌즈 높이 − 테이블 윗면 높이
   1') 같은 순간 D435i IMU 숙임각으로 계산한 앞 / 아래 (IMU 를 열 수 있을 때)
   2) 마커 면 방향으로 추정한 숙임각 (pnp, 참고용)
   3) torso_link / IK(pelvis) 좌표 — 잡기에서 쓰는 값 (robot_server camera_to_torso 와 같은 식)

  --points: 같은 테이블 면, 몸통 중심(torso_link x 0)에서 앞 거리를 아는 위치들에 마커를 차례로 놓고 측정
    → '모든 점의 높이가 같다' 로 숙임각, '앞 거리가 맞다' 로 camera.x 를 계산 (테이블 높이는 몰라도 됨)
    → robot.yaml / IMU 숙임각을 썼을 때의 결과도 함께 출력
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT 미지정이면 종료 — robot.yaml camera / frames 사용

import cv2
import numpy as np
import pyrealsense2 as rs

CX, CY, CZ, CP = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH
P2T = np.array(robot_env.CFG["frames"]["pelvis_to_torso"], dtype=float)


def camera_to_torso(c, pitch=CP):
    """robot_server.py camera_to_torso 와 같은 식 (카메라 광학 좌표 x 오른쪽, y 아래, z 앞)."""
    cx, cy, cz = c
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cy_r = cy * cos_p + cz * sin_p
    cz_r = -cy * sin_p + cz * cos_p
    return np.array([cz_r + CX, -cx + CY, -cy_r + CZ])


def fwd_down(t, p):
    """카메라 좌표 t → 렌즈 기준 (앞, 아래) [m], 숙임각 p [rad]."""
    return t[2] * math.cos(p) - t[1] * math.sin(p), t[2] * math.sin(p) + t[1] * math.cos(p)


def start_pipe():
    """컬러 + 깊이 + (가능하면) 가속도. IMU 권한이 없으면 컬러 + 깊이만."""
    for with_imu in (True, False):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, *robot_env.CAMERA_RES, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, *robot_env.CAMERA_RES, rs.format.z16, 30)
        if with_imu:
            cfg.enable_stream(rs.stream.accel)
        try:
            return pipe, pipe.start(cfg), with_imu
        except RuntimeError as e:
            if not with_imu:
                raise
            print(f"[marker] ⚠️ IMU 스트림 열기 실패 → 컬러+깊이만 사용 ({e})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=0.045, help="마커 한 변 [m] (기본 0.045)")
    ap.add_argument("--id", type=int, default=None, help="이 ID 만 사용 (기본: 화면에서 가장 큰 마커)")
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--points", default=None, help="2점 이상 보정: 몸통 중심에서 마커까지 앞 거리 [m], 예: 0.45,0.55")
    a = ap.parse_args()

    pipe, prof, has_imu = start_pipe()
    align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    dscale = prof.get_device().first_depth_sensor().get_depth_scale()
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
    dist = np.array(intr.coeffs[:5], dtype=np.float64)
    yk = robot_env.CAMERA_K
    print(f"[marker] 카메라 K: fx {intr.fx:.1f} fy {intr.fy:.1f} ppx {intr.ppx:.1f} ppy {intr.ppy:.1f} "
          f"(robot.yaml intrinsics {yk[0]:.1f} / {yk[1]:.1f} / {yk[2]:.1f} / {yk[3]:.1f})")
    print(f"[marker] robot.yaml camera: x {CX} y {CY} z {CZ} pitch {math.degrees(CP):.1f}°  "
          f"pelvis_to_torso {P2T.tolist()}  마커 {a.size * 1000:.0f} mm  IMU {'사용' if has_imu else '없음'}")

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    h = a.size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    seen = set()

    def pick(corners, ids):
        """--id 가 있으면 그 ID, 없으면 화면에서 가장 큰(= 가장 가까운) 마커. 처음 보는 ID 는 알려 준다."""
        best, best_a = (None, None), -1.0
        for c, i in zip(corners, ids.flatten()):
            if int(i) not in seen:
                seen.add(int(i))
                print(f"  (화면에 마커 ID {int(i)} 보임)")
            if a.id is not None and i != a.id:
                continue
            area = cv2.contourArea(c.reshape(4, 2).astype(np.float32))
            if area > best_a:
                best, best_a = (c, i), area
        return best

    def measure(sec):
        """sec 동안 측정 → dict(depth 3D 중앙값, pnp 3D, 법선, IMU 숙임각, id, 개수) 또는 None."""
        tds, tps, ns, acc = [], [], [], []
        mid = None
        t_end = time.time() + sec
        while time.time() < t_end:
            fs = pipe.wait_for_frames(1000)
            if has_imu:
                af = fs.first_or_default(rs.stream.accel)
                if af:
                    d_ = af.as_motion_frame().get_motion_data()
                    acc.append((d_.x, d_.y, d_.z))
            fs = align.process(fs)
            cf, df = fs.get_color_frame(), fs.get_depth_frame()
            if not cf or not df:
                continue
            img = np.asanyarray(cf.get_data())
            corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            if ids is None:
                continue
            c, i = pick(corners, ids)
            if c is None:
                continue
            c4 = c.reshape(4, 2).astype(np.float64)
            mid = int(i)
            # depth: 마커 중심 7×7 중앙값 → 컬러 내부 파라미터로 역투영
            u, v = c4.mean(axis=0)
            dimg = np.asanyarray(df.get_data())
            ui, vi = int(round(u)), int(round(v))
            win = dimg[max(0, vi - 3):vi + 4, max(0, ui - 3):ui + 4].astype(float)
            win = win[win > 0]
            if len(win):
                z = float(np.median(win)) * dscale
                tds.append(rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], z))
            ok, rvec, tvec = cv2.solvePnP(obj, c4, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                tps.append(tvec.ravel())
                ns.append(cv2.Rodrigues(rvec)[0][:, 2])
        if not tds and not tps:
            return None
        r = {"id": mid, "n": len(tds), "td": np.median(np.array(tds), axis=0) if tds else None,
             "tp": np.median(np.array(tps), axis=0) if tps else None, "ip": None, "n_vec": None}
        if ns:
            n = np.median(np.array(ns), axis=0)
            n /= np.linalg.norm(n)
            r["n_vec"] = -n if n[1] > 0 else n
        if acc:
            g = np.mean(np.array(acc), axis=0)
            r["ip"] = math.asin(min(1.0, abs(g[2]) / float(np.linalg.norm(g))))
        return r

    def report(r):
        print(f"  ID {r['id']}  depth 표본 {r['n']}")
        for name, tag in (("td", "depth"), ("tp", "pnp  ")):
            t = r[name]
            if t is None:
                print(f"    [{tag}] 없음")
                continue
            f, d = fwd_down(t, CP)
            tor = camera_to_torso(t)
            ik = tor + P2T
            print(f"    [{tag}] 거리 {np.linalg.norm(t):.3f}  1) 앞 {f:.3f} 좌 {-t[0]:+.3f} 아래 {d:.3f}"
                  f"  3) torso x {tor[0]:.3f} y {tor[1]:+.3f} z {tor[2]:+.3f} | IK x {ik[0]:.3f} z {ik[2]:+.3f}")
            if name == "td" and r["ip"] is not None:
                f2, d2 = fwd_down(t, r["ip"])
                print(f"    1') IMU 숙임각 {math.degrees(r['ip']):.1f}° 로: 앞 {f2:.3f} 아래 {d2:.3f} → torso x {f2 + CX:.3f}")
        if r["n_vec"] is not None:
            n = r["n_vec"]
            print(f"    2) 마커 면 숙임각(pnp, 참고) {math.degrees(math.atan2(-n[2], -n[1])):.1f}°")

    if a.points:
        xs = [float(v) for v in a.points.split(",")]
        meas = []
        try:
            for i, xt in enumerate(xs):
                input(f"\n[{i + 1}/{len(xs)}] 마커를 몸통 중심에서 앞 {xt * 100:.0f} cm (같은 테이블 면) 에 놓고 Enter ")
                r = measure(a.sec)
                if r is None or r["td"] is None:
                    print("  마커(또는 깊이) 안 보임 — 중단")
                    return
                report(r)
                meas.append((xt, r))
        finally:
            pipe.stop()
        print("\n== 2점 보정 결과 (depth 기준) ==")
        ts_ = [m[1]["td"] for m in meas]
        grid = np.radians(np.arange(10.0, 70.0, 0.05))
        p_fit = float(grid[int(np.argmin([np.std([fwd_down(t, p)[1] for t in ts_]) for p in grid]))])
        ips = [m[1]["ip"] for m in meas]
        cands = [("높이 일치 조건 숙임각", p_fit)]
        if all(v is not None for v in ips):
            cands.append(("IMU 숙임각(평균)", float(np.mean(ips))))
        cands.append(("robot.yaml 숙임각", CP))
        for label, p in cands:
            fd = [fwd_down(t, p) for t in ts_]
            cx = [xt - f for (xt, _), (f, _) in zip(meas, fd)]
            downs = [d for _, d in fd]
            print(f"  {label} {math.degrees(p):5.1f}°: camera.x 필요값 " + " / ".join(f"{v:.3f}" for v in cx) +
                  f" (평균 {np.mean(cx):.3f}),  렌즈→테이블 아래 " + " / ".join(f"{v:.3f}" for v in downs) +
                  f" (차이 {max(downs) - min(downs):.3f})")
        sep = [float(np.linalg.norm(np.array(ts_[k + 1]) - np.array(ts_[k]))) for k in range(len(ts_) - 1)]
        print("  측정된 이웃 점 사이 거리 " + " / ".join(f"{v:.3f}" for v in sep) +
              " m  (실제 " + " / ".join(f"{xs[k + 1] - xs[k]:.3f}" for k in range(len(xs) - 1)) + " m — 깊이 축척 확인)")
        print("  · 같은 숙임각에서 camera.x 필요값이 점마다 같고, 아래 차이가 1 cm 안이면 그 숙임각이 맞음")
        return

    try:
        while True:
            r = measure(1.0 if a.watch else a.sec)
            if r is None:
                print("  마커 안 보임")
            else:
                report(r)
            if not a.watch:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
