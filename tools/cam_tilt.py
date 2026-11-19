#!/usr/bin/env python3
"""
cam_tilt.py — D435i 내장 IMU(가속도)로 카메라가 아래로 숙인 각 측정 → robot.yaml camera.pitch_deg 확인용

  rs_stream.py(start_grab_sim real-cam / start_robot) 가 카메라를 쓰고 있으면 먼저 끌 것 (장치 동시 사용 불가).
  conda activate tv (또는 source activate_tv.sh)
  python tools/cam_tilt.py            # 3초 평균, 1회
  python tools/cam_tilt.py --watch    # 계속 출력 (장착 각도 맞추면서 볼 때), Ctrl+C 종료

계산
  RealSense 좌표 (깊이/컬러 광학 좌표와 같은 방향): x 오른쪽, y 아래, z 앞(광학축)
  정지 상태 가속도 a 는 중력 방향 성분만 → 광학축(z)과 수평면 사이 각 = asin(|a_z| / |a|)
  카메라를 더 숙였을 때 값이 커지면 정상 (부호 규약은 장치 펌웨어마다 다를 수 있어 크기만 사용)

주의
  측정값은 '중력 기준' 각도. robot.yaml pitch_deg 는 'torso_link 기준' 각도 —
  로봇 몸통이 똑바로 서 있을 때(허리 0, 기립) 재야 두 값이 같다. 몸통이 기울어 있으면 그만큼 차이남.
  좌우 기울기(roll)도 같이 출력 — 0 에 가까워야 함 (camera_to_torso 는 pitch 만 반영).
"""
import argparse
import math
import time

import numpy as np
import pyrealsense2 as rs


def read_accel(pipe, sec):
    acc = []
    t_end = time.time() + sec
    while time.time() < t_end:
        fs = pipe.wait_for_frames(1000)
        for f in fs:
            if f.is_motion_frame() and f.get_profile().stream_type() == rs.stream.accel:
                d = f.as_motion_frame().get_motion_data()
                acc.append((d.x, d.y, d.z))
    return np.array(acc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=3.0, help="평균 낼 시간 [s]")
    ap.add_argument("--watch", action="store_true", help="계속 측정")
    a = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.accel)
    prof = pipe.start(cfg)
    dev = prof.get_device()
    print(f"[cam_tilt] {dev.get_info(rs.camera_info.name)}  S/N {dev.get_info(rs.camera_info.serial_number)}")
    try:
        while True:
            A = read_accel(pipe, 1.0 if a.watch else a.sec)
            if len(A) == 0:
                print("[cam_tilt] ❌ 가속도 데이터 없음 (D435i 가 맞는지, 다른 프로그램이 쓰는지 확인)")
                return
            g = A.mean(axis=0)
            n = float(np.linalg.norm(g))
            pitch = math.degrees(math.asin(min(1.0, abs(g[2]) / n)))
            roll = math.degrees(math.atan2(g[0], abs(g[1])))
            print(f"  아래로 숙인 각 {pitch:5.1f}°   좌우 기울기 {roll:+5.1f}°   "
                  f"(a = {g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f} m/s², |a| {n:.2f}, n={len(A)}, 흔들림 σ {A.std(axis=0).max():.3f})")
            if not a.watch:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
