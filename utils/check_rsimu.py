#!/usr/bin/env python3
"""
check_rsimu.py — D435i IMU + 로봇 LowState IMU 를 같이 읽어 '몸통(torso) 기준 카메라 숙임각' 측정
                 (robot.yaml camera.pitch_deg 가 뜻하는 값 — 로봇이 앞뒤로 기울어 서 있어도 그만큼 빼서 구함)

  start_robot.sh / rs_stream.py 등 카메라를 쓰는 프로그램은 먼저 끌 것 (D435i 동시 사용 불가).
  D435i IMU 권한 오류가 나면 sudo (또는 RealSense udev 규칙).
  ROBOT=h2 python utils/check_rsimu.py              # 실시간 (0.2 s 마다), Ctrl+C 종료
  ROBOT=h2 python utils/check_rsimu.py --sec 5      # 5 초 평균 + 권장 pitch_deg 출력
  ROBOT=h2 python utils/check_rsimu.py --sec 5 eno1 # DDS 네트워크 인터페이스 지정
  ROBOT=h2 python utils/check_rsimu.py --save       # 5 초 재고 camera.pitch_deg 를 robots/h2/calib_local.yaml 에 저장
                                                    #   (git 에 안 올라감, 다음 실행부터 모든 서버가 이 값 — start_robot.sh 다시 시작)

저장 (--save) — 거치대를 다시 조이거나 건드릴 때마다 (2026-10-11 다시 조인 뒤 46.0 → 42.5)
  저장 안 함: 흔들림(표준편차) > 1.0° 또는 표본 < 200 (로봇이 움직이는 중 · 접촉 불량)
  경고만   : 이전 저장값보다 1° 넘게 바뀜 → 거치대가 움직였거나 덜 조였을 수 있음 (볼트 확인 후 다시 재기 권장)
  파일에 남는 것: camera.pitch_deg (0.1° 반올림), 측정 시각·평균·표준편차·표본 수, 최근 10 회 이력
  sudo 로 실행해도 파일 소유자는 원래 사용자 (SUDO_UID)

계산
  cam_abs      = D435i 가속도로 구한 광학축 숙임 (중력 기준)        = atan2(−a_z, √(a_x² + a_y²))
  torso_abs    = 로봇 IMU pitch (LowState imu_state.rpy[1]) + 허리 pitch 관절각 (robot.yaml joints.map waist_pitch_joint)
  cam_to_torso = cam_abs − torso_abs   ← robot.yaml camera.pitch_deg 와 비교할 값
  비교 대상: robot.yaml camera.pitch_deg (실장착 보정값)

부호 확인 (H2 는 미확인): 거치대에서 로봇 몸통을 손으로 살짝 앞뒤로 흔들 때
  cam_abs · torso_abs 는 같이 변하고 cam_to_torso 는 거의 그대로면 부호가 맞음.
  cam_to_torso 가 두 배로 흔들리면 로봇 IMU pitch 부호가 반대 (확인 필요 → 알려줄 것).
H2 LowState imu_state 가 pelvis IMU 인지 torso IMU 인지는 확인 필요 — 허리 pitch 가 ≈0 이면 결과 차이 없음.
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # noqa: E402  (ROBOT 필수, H2 SDK 경로 설정 — unitree_sdk2py 보다 먼저)

import numpy as np   # noqa: E402
import pyrealsense2 as rs   # noqa: E402
from unitree_sdk2py.core.channel import ChannelSubscriber   # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState   # noqa: E402

CAMERA_PITCH_CFG = robot_env.CAMERA_PITCH                                   # 지금 쓰는 camera.pitch_deg [rad] (calib_local.yaml 이 있으면 그 값)
SAVE_MAX_STD_DEG = 1.0
SAVE_MIN_N = 200
DRIFT_WARN_DEG = 1.0
WAIST_PITCH_SLOT = int(robot_env.JOINTS["map"]["waist_pitch_joint"])        # H2 13 (xr_teleoperate 817fb00)

pelvis_pitch = None
pelvis_roll = 0.0
waist_pitch = 0.0


def start_robot_imu(interface=None):
    robot_env.dds_init(interface)
    sub = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub.Init()

    def _read():
        global pelvis_pitch, pelvis_roll, waist_pitch
        while True:
            msg = sub.Read()
            if msg is not None:
                pelvis_roll = float(msg.imu_state.rpy[0])
                pelvis_pitch = float(msg.imu_state.rpy[1])
                waist_pitch = float(msg.motor_state[WAIST_PITCH_SLOT].q)
            time.sleep(0.01)

    t = threading.Thread(target=_read, daemon=True)
    t.start()
    print(f"[Robot IMU] rt/lowstate 구독 (ROBOT={robot_env.ROBOT}, 허리 pitch 슬롯 {WAIST_PITCH_SLOT})")


def start_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    prof = pipeline.start(config)
    dev = prof.get_device()
    print(f"[D435i IMU] {dev.get_info(rs.camera_info.name)}  S/N {dev.get_info(rs.camera_info.serial_number)}")
    return pipeline


def get_accel_pitch(accel):
    ax, ay, az = accel.x, accel.y, accel.z
    return float(np.arctan2(-az, np.sqrt(ax**2 + ay**2)))  # p4


def sample(pipeline):
    """(cam_abs, pelvis_pitch, waist_pitch, pelvis_roll) 한 번 — 로봇 IMU 를 아직 못 받았으면 None."""
    frames = pipeline.wait_for_frames()
    accel_frame = frames.first_or_default(rs.stream.accel)
    if not accel_frame or pelvis_pitch is None:
        return None
    accel = accel_frame.as_motion_frame().get_motion_data()
    return get_accel_pitch(accel), pelvis_pitch, waist_pitch, pelvis_roll


def save_calib(c2t, cam_abs, torso_abs, n, sec):
    """camera.pitch_deg 를 calib_local.yaml 에 저장 (기존 파일의 다른 camera 키·이력 유지)."""
    import yaml
    path = robot_env.LOCAL_CALIB_PATH
    mean, std = float(c2t.mean()), float(c2t.std())
    if n < SAVE_MIN_N or std > SAVE_MAX_STD_DEG:
        print(f"\n  ❌ 저장 안 함 — 표본 {n} (최소 {SAVE_MIN_N}) · 흔들림 ±{std:.2f}° (최대 {SAVE_MAX_STD_DEG}°). "
              f"로봇이 가만히 서 있는지, 거치대가 덜렁거리지 않는지 확인 후 다시")
        return False
    old = {}
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            old = yaml.safe_load(f) or {}
    cam = dict(old.get("camera") or {})
    prev = cam.get("pitch_deg")
    new = round(mean, 1)
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    cam["pitch_deg"] = new
    measured = {"time": now, "tool": f"check_rsimu.py --sec {sec:g}", "cam_to_torso_deg": round(mean, 2),
                "std_deg": round(std, 2), "n": int(n), "cam_abs_deg": round(float(cam_abs.mean()), 2),
                "torso_abs_deg": round(float(torso_abs.mean()), 2),
                "robot_yaml_pitch_deg": float(robot_env.CAMERA_YAML["pitch_deg"])}
    hist = [{"time": now, "pitch_deg": new, "std_deg": round(std, 2)}] + list(old.get("history") or [])
    body = yaml.safe_dump({"camera": cam, "measured": measured, "history": hist[:10]},
                          allow_unicode=True, sort_keys=False, default_flow_style=False)
    head = ("# 이 PC(로봇) 전용 장착 보정 — git 에 안 올라감 (.gitignore calib_*.yaml)\n"
            "# utils/check_rsimu.py --save 가 씀. 있으면 robot.yaml camera 값을 덮어씀 (common/robot_env.py)\n")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(head + body)
    os.replace(tmp, path)
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid and gid:
        try:
            os.chown(path, int(uid), int(gid))
        except OSError:
            pass
    was = f"{prev}" if prev is not None else f"없음 (robot.yaml {float(robot_env.CAMERA_YAML['pitch_deg'])})"
    print(f"\n  ✅ 저장: camera.pitch_deg {new}  (이전 {was}) → {os.path.relpath(path, robot_env.REPO_ROOT)}")
    if prev is not None and abs(new - float(prev)) > DRIFT_WARN_DEG:
        print(f"  ⚠️ 이전 저장값보다 {new - float(prev):+.1f}° 바뀜 — 거치대가 움직였거나 덜 조였을 수 있음. 볼트 확인 후 한 번 더 재기 권장")
    print("  다음 실행부터 적용 — start_robot.sh 가 켜져 있었다면 다시 시작")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("interface", nargs="?", default=None, help="DDS 네트워크 인터페이스 (기본: robot_env 기본값)")
    ap.add_argument("--sec", type=float, default=0.0, help="이 시간 평균 후 요약 출력 (0 = 실시간 계속)")
    ap.add_argument("--save", action="store_true",
                    help="평균 camera.pitch_deg 를 robots/<ROBOT>/calib_local.yaml 에 저장 (--sec 없으면 5 초)")
    a = ap.parse_args()
    if a.save and a.sec <= 0:
        a.sec = 5.0

    start_robot_imu(a.interface)
    time.sleep(1.0)
    pipeline = start_camera()
    time.sleep(1.0)
    t_wait = time.time()
    while pelvis_pitch is None:
        if time.time() - t_wait > 5.0:
            print("[check_rsimu] ❌ rt/lowstate 수신 없음 (로봇 전원·네트워크·인터페이스 확인)")
            pipeline.stop()
            return
        time.sleep(0.1)

    try:
        if a.sec > 0:
            rows = []
            t_end = time.time() + a.sec
            while time.time() < t_end:
                s = sample(pipeline)
                if s is not None:
                    rows.append(s)
            if not rows:
                print("[check_rsimu] ❌ D435i 가속도 데이터 없음")
                return
            R = np.degrees(np.array(rows))
            cam_abs, pp, wp, pr = R[:, 0], R[:, 1], R[:, 2], R[:, 3]
            torso_abs = pp + wp
            c2t = cam_abs - torso_abs
            cfg = np.degrees(CAMERA_PITCH_CFG)
            print(f"\n=== {a.sec:.0f} 초 평균 (n={len(rows)}) — 평균 ± 표준편차 [deg] ===")
            print(f"  로봇 IMU pitch (pelvis_p) {pp.mean():+7.2f} ± {pp.std():.2f}    roll {pr.mean():+.2f}")
            print(f"  허리 pitch 관절 (waist_p)  {wp.mean():+7.2f} ± {wp.std():.2f}    (슬롯 {WAIST_PITCH_SLOT})")
            print(f"  몸통 기울기 (torso_abs)    {torso_abs.mean():+7.2f} ± {torso_abs.std():.2f}")
            print(f"  카메라 숙임 (cam_abs, 중력 기준) {cam_abs.mean():7.2f} ± {cam_abs.std():.2f}")
            print(f"  ▶ 몸통 기준 숙임 (cam_to_torso)  {c2t.mean():7.2f} ± {c2t.std():.2f}")
            print(f"  지금 camera.pitch_deg            {cfg:7.2f}   (차이 {c2t.mean() - cfg:+.2f}, 출처 {robot_env.CAMERA_SOURCE})")
            if robot_env.CAMERA_SOURCE != "robot.yaml":
                print(f"  robot.yaml camera.pitch_deg      {float(robot_env.CAMERA_YAML['pitch_deg']):7.2f}")
            if not a.save:
                print(f"\n  권장: camera.pitch_deg: {c2t.mean():.1f}   → 저장: --save")
                return
            save_calib(c2t, cam_abs, torso_abs, len(rows), a.sec)
            return

        print("\n=== 실시간 IMU 비교 (Ctrl+C 종료) ===")
        print(f"{'pelvis_p':>10} {'waist_p':>10} {'torso_abs':>10} {'cam_abs':>10} {'cam_to_torso':>14} {'yaml':>8} {'diff':>8}")
        while True:
            s = sample(pipeline)
            if s is None:
                continue
            cam_abs, pp, wp, _ = s
            torso_abs = pp + wp
            cam_to_torso = cam_abs - torso_abs
            diff = cam_to_torso - CAMERA_PITCH_CFG
            print(f"{np.degrees(pp):>10.2f} "
                  f"{np.degrees(wp):>10.2f} "
                  f"{np.degrees(torso_abs):>10.2f} "
                  f"{np.degrees(cam_abs):>10.2f} "
                  f"{np.degrees(cam_to_torso):>14.2f} "
                  f"{np.degrees(CAMERA_PITCH_CFG):>8.2f} "
                  f"{np.degrees(diff):>8.2f}°")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
