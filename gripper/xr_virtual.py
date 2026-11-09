"""
xr_virtual.py — Unitree xr_teleoperate 를 로봇 없이 (가상) 돌려 봄

  xr_teleoperate 의 Quest 입력 처리 (televuer · TeleVuerWrapper) 와 H2 팔 IK (H2_ArmIK) 를 **그대로** 쓰고,
  로봇 제어 (H2_ArmController · Dex1 그리퍼) 대신 IK 결과 관절값을 우리 3D 뷰어 (gripper/xr_viewer.py) 로 보냄.
  → 로봇 · 그리퍼에는 아무 명령도 안 보냄. xr_teleoperate 조작감을 URDF 가상 로봇으로 먼저 확인하는 용도.

  준비 (mini PC, 한 번):
    git clone --recurse-submodules https://github.com/unitreerobotics/xr_teleoperate ~/xr_teleoperate
    #  tv 환경에 xr_teleoperate README 의 의존성 (pinocchio 3 + casadi, meshcat, logging-mp, televuer=vuer) 이 있어야 함
    cd ~/xr_teleoperate/teleop/televuer && pip install -e .
    mkdir -p ~/.config/xr_teleoperate && cd ~/.config/xr_teleoperate && \\
      openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -keyout key.pem -out cert.pem -subj "/CN=localhost"

  실행:
    터미널 1:  python gripper/xr_viewer.py --table-z -1.0          # 뷰어 (http://localhost:50040/)
    터미널 2:  python gripper/xr_virtual.py --xr ~/xr_teleoperate   # Vuer 서버 (8012) + H2 IK → 뷰어
    Quest (USB): adb reverse tcp:8012 tcp:8012 → Quest 브라우저 https://localhost:8012/?ws=wss://localhost:8012
                 (자체 서명 경고 → 계속) → 'Enter VR' (pass-through)
  조작은 xr_teleoperate 와 같음: 헤드셋을 머리에 쓰고 컨트롤러를 쥔 손 = 로봇 손 (머리 기준 절대 위치), 트리거 = 그리퍼.
  다른 점 (안전): 그립(중지)을 누른 손만 따라감 (떼면 멈춤) · 관절 속도 상한 1 rad/s (원본은 버튼 없이 계속 · 30 rad/s)
"""
import argparse
import json
import os
import sys
import time
import threading
import urllib.request

import cv2
import numpy as np
import pinocchio as pin


def placeholder(msg):
    img = np.full((480, 640, 3), 40, np.uint8)
    for i, line in enumerate(("VIRTUAL (no robot command)", msg, "robot view: PC http://localhost:50040/")):
        cv2.putText(img, line, (20, 200 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)
    return img


def feed_video(tv, url):
    """잡기 웹 머리 카메라 MJPEG → Quest 화면 (immersive). 못 받으면 안내 글자 화면."""
    while True:
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                buf = b""
                while True:
                    chunk = r.read(16384)
                    if not chunk:
                        break
                    buf += chunk
                    last = None
                    while True:                       # 버퍼에 쌓인 프레임 중 가장 최신 것만
                        a = buf.find(b"\xff\xd8")
                        b = buf.find(b"\xff\xd9", a + 2) if a != -1 else -1
                        if a == -1 or b == -1:
                            break
                        last, buf = buf[a:b + 2], buf[b + 2:]
                    if last is not None:
                        img = cv2.imdecode(np.frombuffer(last, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            tv.render_to_xr(cv2.resize(img, (640, 480)))
                    if len(buf) > 4_000_000:
                        buf = b""
        except Exception as e:  # noqa: BLE001
            tv.render_to_xr(placeholder(f"no camera: {type(e).__name__}"))
            time.sleep(1.0)


def post(url, data, timeout=0.5):
    req = urllib.request.Request(url, data=json.dumps(data).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def get(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def main():
    ap = argparse.ArgumentParser(description="xr_teleoperate (televuer + H2_ArmIK) → 우리 3D 뷰어 (로봇 명령 없음)")
    ap.add_argument("--xr", default=os.path.expanduser("~/xr_teleoperate"), help="xr_teleoperate 저장소 경로")
    ap.add_argument("--viewer", default="http://localhost:50040", help="gripper/xr_viewer.py 주소")
    ap.add_argument("--hand", action="store_true", help="컨트롤러 대신 손 추적 입력")
    ap.add_argument("--hz", type=float, default=30.0, help="IK · 전송 주기 (xr_teleoperate 기본 30)")
    ap.add_argument("--joint-vmax", type=float, default=1.0, help="가상 팔 관절 속도 상한 [rad/s] (xr_teleoperate H2 기본 30)")
    ap.add_argument("--no-deadman", action="store_true", help="그립 안 눌러도 계속 따라감 (xr_teleoperate 원본 동작)")
    ap.add_argument("--video", default="http://localhost:50030/video", help="immersive 에서 Quest 에 보여 줄 영상 (잡기 웹 머리 카메라 MJPEG)")
    ap.add_argument("--display", choices=("pass-through", "immersive"), default="pass-through",
                    help="Vuer 화면 모드: pass-through = 실제 주변 (Vuer 왼쪽 아래 'pass-through' 버튼) · "
                         "immersive = G1 때처럼 'Virtual Reality' 버튼 (잡기 웹 머리 카메라 영상을 보여 줌)")
    a = ap.parse_args()

    xr = os.path.abspath(os.path.expanduser(a.xr))
    teleop = os.path.join(xr, "teleop")
    if not os.path.isfile(os.path.join(teleop, "robot_control", "robot_arm_ik.py")):
        sys.exit(f"[xr] {xr} 에 xr_teleoperate 가 없음 — git clone --recurse-submodules https://github.com/unitreerobotics/xr_teleoperate")
    sys.path.insert(0, xr)
    sys.path.insert(0, os.path.join(teleop, "televuer", "src"))
    os.chdir(teleop)   # H2_ArmIK 가 ../assets/h2/H2.urdf 를 상대 경로로 읽음

    try:
        from teleop.robot_control.robot_arm_ik import H2_ArmIK
    except ImportError as e:
        sys.exit(f"[xr] H2_ArmIK 를 못 불러옴 ({e}) — xr_teleoperate README 의 tv 환경 (pinocchio 3 + casadi, meshcat, logging-mp) 필요")
    try:
        from televuer import TeleVuerWrapper
    except ImportError as e:
        sys.exit(f"[xr] televuer 를 못 불러옴 ({e}) — cd {teleop}/televuer && pip install -e .")

    try:
        q0 = get(a.viewer + "/api/ext/q")
    except Exception as e:  # noqa: BLE001
        sys.exit(f"[xr] 뷰어 {a.viewer} 에 연결 안 됨 ({e}) — 먼저 python gripper/xr_viewer.py 실행")

    arm_ik = H2_ArmIK()
    m = arm_ik.reduced_robot.model
    names = [m.names[j] for j in range(1, m.njoints)]
    miss = [n for n in names if n not in q0]
    if miss:
        sys.exit(f"[xr] xr_teleoperate H2 모델 관절 이름이 뷰어와 다름: {miss}")
    q_last = np.array([q0[n] for n in names])   # 가상 팔 지금 자세에서 시작 (실제 로봇이면 실측 관절)
    arm_ik.init_data = q_last.copy()

    immersive = a.display == "immersive"
    # immersive 는 영상이 있어야 함 (televuer: zmq 또는 webrtc) — 로봇 영상 서버 대신 잡기 웹 머리 카메라 (D435i) 를 한 눈 영상으로 넣음
    tv = TeleVuerWrapper(use_hand_tracking=a.hand, binocular=not immersive, img_shape=(480, 640) if immersive else (480, 1280),
                         display_mode=a.display, zmq=immersive, webrtc=False, arm_reference_mode="head_yaw")
    if immersive:
        threading.Thread(target=feed_video, args=(tv, a.video), daemon=True).start()
    print("[xr] Vuer 서버 :8012 — Quest: adb reverse tcp:8012 tcp:8012 → https://localhost:8012/?ws=wss://localhost:8012", flush=True)
    print(f"[xr] 화면 모드 {a.display} — Vuer 에서 " + ("왼쪽 아래 'pass-through' 버튼" if a.display == "pass-through" else "'Virtual Reality' 버튼")
          + " 을 컨트롤러로 누름", flush=True)
    print("[xr] 로봇 명령 없음. IK 결과는 뷰어로만 보냄. Ctrl+C 로 끝", flush=True)

    dt = 1.0 / a.hz
    n_ok, t_print = 0, time.monotonic()
    try:
        while True:
            t0 = time.monotonic()
            d = tv.get_tele_data()
            if d.motion_data_ready:
                # 데드맨: 그립(중지) 누른 손만 그 손 자세를 목표로, 뗀 손은 지금 가상 손목 자세 그대로 (xr_teleoperate 원본은 버튼 없이 계속 따라감)
                pin.framesForwardKinematics(m, arm_ik.reduced_robot.data, q_last)
                now = {"left": arm_ik.reduced_robot.data.oMf[arm_ik.L_hand_id].homogeneous.copy(),
                       "right": arm_ik.reduced_robot.data.oMf[arm_ik.R_hand_id].homogeneous.copy()}
                press = {"left": a.no_deadman or float(d.left_ctrl_squeezeValue) > 0.5,
                         "right": a.no_deadman or float(d.right_ctrl_squeezeValue) > 0.5}
                tgt = {"left": d.left_wrist_pose if press["left"] else now["left"],
                       "right": d.right_wrist_pose if press["right"] else now["right"]}
                if press["left"] or press["right"]:
                    q_sol, _ = arm_ik.solve_ik(tgt["left"], tgt["right"], q_last, None)
                    dq = np.asarray(q_sol, dtype=float) - q_last
                    big = float(np.max(np.abs(dq)))
                    lim = a.joint_vmax * dt                 # 관절 속도 제한 (방향 유지, 같은 비율로 줄임)
                    q_last = q_last + (dq * (lim / big) if big > lim else dq)
                if a.hand:
                    grip = {"left": min(1.0, d.left_hand_pinchValue / 100.0), "right": min(1.0, d.right_hand_pinchValue / 100.0)}
                    grip = {k: 1.0 - v for k, v in grip.items()}   # 집게 벌림 (pinch 클수록 벌림 — xr_teleoperate 와 같은 방향인지 확인 필요)
                else:
                    # televuer: ctrl_triggerValue = 10 − 트리거 × 10 (10 = 놓음 · 0 = 끝까지) → 닫힘 비율
                    grip = {"left": 1.0 - d.left_ctrl_triggerValue / 10.0, "right": 1.0 - d.right_ctrl_triggerValue / 10.0}
                try:
                    post(a.viewer + "/api/ext", {"names": names, "q": q_last.tolist(), "grip": grip, "src": "xr_teleoperate (가상)"})
                    n_ok += 1
                except Exception as e:  # noqa: BLE001
                    print(f"[xr] 뷰어 전송 실패: {e}", flush=True)
            if time.monotonic() - t_print > 2.0:
                print(f"[xr] {'입력 받는 중' if d.motion_data_ready else 'Quest 입력 기다림'} · 보낸 수 {n_ok}", flush=True)
                t_print = time.monotonic()
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tv.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
