"""
xr_virtual.py — Unitree xr_teleoperate (Quest 입력 + H2 IK) → 가상 뷰어, 또는 (--real) 잡기 웹 → 실제 팔

  xr_teleoperate 의 Quest 입력 처리 (televuer · TeleVuerWrapper) 와 H2 팔 IK (H2_ArmIK) 를 그대로 쓰고,
  로봇 제어부 (H2_ArmController — 속도 제한 30 rad/s · 끌 때 0 자세로 감) 는 쓰지 않음.
   - 기본 (가상): IK 결과를 3D 뷰어 (gripper/xr_viewer.py, :50040) 로만 보냄. 로봇 명령 없음.
   - --real: 잡기 웹 (pick_web.py, :50030) 에서 [VR 시작] 을 누른 동안만 /api/xr/cmd 로 보냄 →
             잡기 웹이 안전 검사 후 arm_server · 그리퍼로 (gripper/xr_real.py). [VR 시작] 전 · [VR 끝] 뒤에는 아무것도 안 보냄.

  준비 (mini PC, 한 번):
    git clone --recurse-submodules https://github.com/unitreerobotics/xr_teleoperate ~/xr_teleoperate
    cd ~/xr_teleoperate/teleop/televuer && pip install -e .
    pip install 'vuer[all]==0.0.60' "params-proto>=2.13,<3"
    mkdir -p ~/.config/xr_teleoperate && cd ~/.config/xr_teleoperate && \\
      openssl req -x509 -nodes -days 3650 -newkey rsa:2048 -keyout key.pem -out cert.pem -subj "/CN=localhost"

  Quest (USB): adb reverse tcp:8012 tcp:8012 → https://localhost:8012/?ws=wss://localhost:8012 → 'Virtual Reality' (immersive)
  조작: 컨트롤러 그립(중지)을 누른 순간의 손 자리를 기준으로, 거기서 움직인 만큼 (위치 · 회전 1:1) 로봇 손이 움직임 (상대 매핑 — 로봇 자세에 손을 맞출 필요 없음).
        그립(중지)을 누른 손만 팔이 따라감 · 트리거 = 그리퍼 (당기면 닫힘 · 놓으면 벌림). Quest 화면 = 머리 카메라 + 손목 카메라 두 개 (잡기 웹 영상)
"""
import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import pinocchio as pin


# ---------------- 영상 (잡기 웹 MJPEG → Quest) ----------------
class Mjpeg:
    """MJPEG 주소 하나를 계속 읽어 가장 최신 프레임만 둠."""

    def __init__(self, url):
        self.url, self.img, self.t, self.err = url, None, 0.0, "아직"
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=3) as r:
                    buf = b""
                    while True:
                        chunk = r.read(16384)
                        if not chunk:
                            break
                        buf += chunk
                        last = None
                        while True:
                            a = buf.find(b"\xff\xd8")
                            b = buf.find(b"\xff\xd9", a + 2) if a != -1 else -1
                            if a == -1 or b == -1:
                                break
                            last, buf = buf[a:b + 2], buf[b + 2:]
                        if last is not None:
                            img = cv2.imdecode(np.frombuffer(last, np.uint8), cv2.IMREAD_COLOR)
                            if img is not None:
                                self.img, self.t, self.err = img, time.time(), ""
                        if len(buf) > 4_000_000:
                            buf = b""
            except Exception as e:  # noqa: BLE001
                self.err = type(e).__name__
                time.sleep(1.0)

    def fresh(self, age=1.0):
        return self.img if self.img is not None and time.time() - self.t < age else None


def compose(head, wl, wr, banner):
    """머리 카메라 640×480 + 아래 왼쪽 · 오른쪽에 손목 카메라 (220×165). 없으면 안내 글자."""
    if head is None:
        img = np.full((480, 640, 3), 40, np.uint8)
        cv2.putText(img, "no head camera (pick_web :50030)", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)
    else:
        img = cv2.resize(head, (640, 480))
    for w, x in ((wl, 0), (wr, 640 - 220)):
        if w is not None:
            img[480 - 165:480, x:x + 220] = cv2.resize(w, (220, 165))
            cv2.rectangle(img, (x, 480 - 165), (x + 219, 479), (255, 255, 255), 1)
    cv2.putText(img, banner, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return img


def feed_video(tv, base, status):
    head, wl, wr = Mjpeg(base + "/video"), Mjpeg(base + "/video_wrist/wrist_left"), Mjpeg(base + "/video_wrist/wrist_right")
    while True:
        tv.render_to_xr(compose(head.fresh(), wl.fresh(), wr.fresh(), status()))
        time.sleep(1.0 / 20)


def en(reason):
    """Quest 화면 글자 (OpenCV 글꼴은 한글 안 됨) — 멈춤 이유를 영어로."""
    keys = (("몸통", "BODY"), ("테이블", "TABLE"), ("다른 팔", "ARMS"), ("뒤로", "BACK"), ("한 번에", "IK JUMP"))
    return ",".join(e for k, e in keys if k in reason) or "LIMIT"


# ---------------- HTTP ----------------
def post(url, data, timeout=0.5):
    req = urllib.request.Request(url, data=json.dumps(data).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def get(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def main():
    ap = argparse.ArgumentParser(description="xr_teleoperate (televuer + H2_ArmIK) → 가상 뷰어 / --real 잡기 웹 (실제 팔)")
    ap.add_argument("--xr", default=os.path.expanduser("~/xr_teleoperate"), help="xr_teleoperate 저장소 경로")
    ap.add_argument("--viewer", default="http://localhost:50040", help="가상 뷰어 (gripper/xr_viewer.py) — --real 에서는 있으면 같이 보냄")
    ap.add_argument("--real", nargs="?", const="http://localhost:50030", default=None,
                    help="실제 팔: 잡기 웹 주소 (기본 http://localhost:50030). 잡기 웹 [VR 시작] 동안만 보냄")
    ap.add_argument("--pick", default="http://localhost:50030", help="영상 (머리 · 손목 카메라) 을 받을 잡기 웹 주소")
    ap.add_argument("--hand", action="store_true", help="컨트롤러 대신 손 추적 입력")
    ap.add_argument("--hz", type=float, default=30.0, help="IK · 전송 주기 (xr_teleoperate 기본 30)")
    ap.add_argument("--joint-vmax", type=float, default=None, help="관절 속도 상한 [rad/s] (가상 기본 1.0, --real 은 잡기 웹 세션 값)")
    ap.add_argument("--no-deadman", action="store_true", help="(가상만) 그립 안 눌러도 계속 따라감 — xr_teleoperate 원본 동작")
    ap.add_argument("--display", choices=("immersive", "pass-through"), default="immersive",
                    help="immersive = 'Virtual Reality' 버튼, 카메라 영상 · pass-through = 왼쪽 아래 'pass-through' 버튼, 실제 주변")
    a = ap.parse_args()
    real = a.real
    if real and a.no_deadman:
        sys.exit("[xr] --real 에서는 --no-deadman 을 쓸 수 없음")

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

    # 시작 자세: 가상 = 뷰어의 가상 팔 · 실제 = 잡기 웹 세션이 열릴 때 받음
    viewer_ok = True
    q0 = None
    try:
        q0 = get(a.viewer + "/api/ext/q")
    except Exception as e:  # noqa: BLE001
        viewer_ok = False
        if not real:
            sys.exit(f"[xr] 뷰어 {a.viewer} 에 연결 안 됨 ({e}) — 먼저 python gripper/xr_viewer.py 실행")
        print(f"[xr] 가상 뷰어 없음 — 실제 팔로만 보냄", flush=True)
    if real:
        try:
            get(real + "/api/xr/state")
        except Exception as e:  # noqa: BLE001
            sys.exit(f"[xr] 잡기 웹 {real} 에 연결 안 됨 ({e}) — start_gripper.sh 로 잡기 웹을 먼저")

    arm_ik = H2_ArmIK()
    m = arm_ik.reduced_robot.model
    names = [m.names[j] for j in range(1, m.njoints)]
    if q0 is not None:
        miss = [n for n in names if n not in q0]
        if miss:
            sys.exit(f"[xr] xr_teleoperate H2 모델 관절 이름이 다름: {miss}")
    q_last = np.array([q0[n] for n in names]) if q0 is not None else np.zeros(len(names))
    arm_ik.init_data = q_last.copy()
    filt0 = arm_ik.smooth_filter

    anchor = {"left": None, "right": None}    # 그립을 누른 순간 (사람 손목 자세, 로봇 손목 자세) — 손마다

    def reset_ik(q):
        anchor.update(left=None, right=None)
        arm_ik.init_data = q.copy()
        try:
            arm_ik.smooth_filter = type(filt0)(np.array([0.4, 0.3, 0.2, 0.1]), len(names))   # 지난 세션 해가 섞이지 않게
        except Exception:  # noqa: BLE001
            pass

    sess = {"active": False, "epoch": None, "msg": "VIRTUAL (no robot command)" if not real else "ROBOT OFF - press VR START on pick web",
            "vmax": a.joint_vmax or 1.0, "base": ""}
    immersive = a.display == "immersive"
    tv = TeleVuerWrapper(use_hand_tracking=a.hand, binocular=not immersive, img_shape=(480, 640) if immersive else (480, 1280),
                         display_mode=a.display, zmq=immersive, webrtc=False, arm_reference_mode="head_yaw")
    if immersive:
        threading.Thread(target=feed_video, args=(tv, a.pick, lambda: sess["msg"]), daemon=True).start()
    print("[xr] Vuer :8012 — Quest: adb reverse tcp:8012 tcp:8012 → https://localhost:8012/?ws=wss://localhost:8012 → "
          + ("'Virtual Reality'" if immersive else "왼쪽 아래 'pass-through'"), flush=True)
    print("[xr] " + ("실제 팔: 잡기 웹에서 [VR 시작] 을 누른 동안만 보냄" if real else "가상: 로봇 명령 없음, 뷰어로만"), flush=True)

    dt = 1.0 / a.hz
    n_ok, t_print, t_poll = 0, time.monotonic(), 0.0

    def step():
        nonlocal n_ok, t_print, t_poll, q_last
        t0 = time.monotonic()
        # --real: 세션 확인 (꺼져 있으면 0.3 초마다)
        if real and not sess["active"] and t0 - t_poll > 0.3:
            t_poll = t0
            try:
                s = get(real + "/api/xr/state", timeout=1.0)
                if s.get("active") and s.get("q0") and s.get("epoch") != sess["epoch"]:
                    q_last = np.array([s["q0"][n] for n in names])
                    reset_ik(q_last)
                    base = f"ROBOT ON {float(s['max_vel']):.2f} rad/s - hold GRIP to move"
                    sess.update(active=True, epoch=s["epoch"], vmax=a.joint_vmax or float(s["max_vel"]), base=base, msg=base)
                    print(f"[xr] 실제 팔 세션 #{s['epoch']} 시작 — 그립을 누른 손만 움직임", flush=True)
            except Exception:  # noqa: BLE001
                pass
        d = tv.get_tele_data()
        if real and sess["active"] and not d.motion_data_ready:
            # Quest 입력이 아직 없음 (VR 안 들어감 · 화면 꺼짐) — 그 자리 유지 명령만 보내 세션을 살려 둠 (팔은 안 움직임)
            try:
                post(real + "/api/xr/cmd", {"names": names, "q": q_last.tolist(), "grip": {}, "press": {}, "no_input": True}, timeout=1.0)
                sess["msg"] = "ROBOT ON - no Quest input (enter VR)"
            except urllib.error.HTTPError as e:
                sess.update(active=False, msg="ROBOT OFF - press VR START on pick web")
                print(f"[xr] 실제 팔 세션 끝 ({e.code})", flush=True)
            except Exception:  # noqa: BLE001
                pass
        sendable = d.motion_data_ready and (not real or sess["active"])
        if sendable:
            # 데드맨: 그립(중지) 누른 손만 그 손 자세를 목표로, 뗀 손은 지금 손목 자세 그대로
            pin.framesForwardKinematics(m, arm_ik.reduced_robot.data, q_last)
            now = {"left": arm_ik.reduced_robot.data.oMf[arm_ik.L_hand_id].homogeneous.copy(),
                   "right": arm_ik.reduced_robot.data.oMf[arm_ik.R_hand_id].homogeneous.copy()}
            press = {"left": a.no_deadman or float(d.left_ctrl_squeezeValue) > 0.5,
                     "right": a.no_deadman or float(d.right_ctrl_squeezeValue) > 0.5}
            # 상대 매핑: 그립을 누른 순간의 사람 손 ↔ 로봇 손을 기준으로, 그 뒤 사람 손이 움직인 만큼 (위치 · 회전 1:1) 만 로봇 손이 움직임
            #   → 사람은 편한 자리에서 그립을 누르면 됨 (로봇 자세에 손을 맞출 필요 없음). 그립을 놓았다 다시 누르면 기준을 새로 잡음
            human = {"left": np.asarray(d.left_wrist_pose, dtype=float), "right": np.asarray(d.right_wrist_pose, dtype=float)}
            tgt = {}
            for sd in ("left", "right"):
                if not press[sd]:
                    anchor[sd] = None
                    tgt[sd] = now[sd]
                    continue
                if anchor[sd] is None:
                    anchor[sd] = (human[sd].copy(), now[sd].copy())
                h0, r0 = anchor[sd]
                t = np.eye(4)
                t[:3, :3] = human[sd][:3, :3] @ h0[:3, :3].T @ r0[:3, :3]
                t[:3, 3] = r0[:3, 3] + (human[sd][:3, 3] - h0[:3, 3])
                tgt[sd] = t
            q_cmd = q_last
            if press["left"] or press["right"]:
                q_sol, _ = arm_ik.solve_ik(tgt["left"], tgt["right"], q_last, None)
                dq = np.asarray(q_sol, dtype=float) - q_last
                big = float(np.max(np.abs(dq)))
                lim = sess["vmax"] * dt                 # 관절 속도 제한 (방향 유지, 같은 비율로 줄임)
                q_cmd = q_last + (dq * (lim / big) if big > lim else dq)
            if a.hand:
                grip = {"left": 1.0 - min(1.0, d.left_hand_pinchValue / 100.0), "right": 1.0 - min(1.0, d.right_hand_pinchValue / 100.0)}
            else:
                # televuer: ctrl_triggerValue = 10 − 트리거 × 10 (10 = 놓음 · 0 = 끝까지) → 닫힘 비율 0~1
                grip = {"left": 1.0 - d.left_ctrl_triggerValue / 10.0, "right": 1.0 - d.right_ctrl_triggerValue / 10.0}
            if real:
                try:
                    res = post(real + "/api/xr/cmd", {"names": names, "q": q_cmd.tolist(), "grip": grip, "press": press}, timeout=1.0)
                    q_last = np.array([res["q"][n] for n in names])     # 잡기 웹이 받아들인 (안전 검사 · 서버 속도 제한 후) 목표에서 이어감
                    n_ok += 1
                    h = res.get("hands") or {}
                    stops = [f"{k[0].upper()}:{en(v.get('reason', ''))}" for k, v in h.items() if v.get("level") == "stop"]
                    sess["msg"] = ("STOP: " + " / ".join(stops))[:60] if stops else sess["base"]
                except urllib.error.HTTPError as e:
                    sess.update(active=False, msg="ROBOT OFF - press VR START on pick web")
                    print(f"[xr] 실제 팔 세션 끝 ({e.code}: {e.read().decode(errors='ignore')[:120]})", flush=True)
                except Exception as e:  # noqa: BLE001
                    print(f"[xr] 잡기 웹 전송 실패: {e}", flush=True)
            else:
                q_last = q_cmd
                n_ok += 1
            if viewer_ok:
                try:
                    post(a.viewer + "/api/ext", {"names": names, "q": q_last.tolist(), "grip": grip,
                                                 "src": "실제 팔 목표" if real else "xr_teleoperate (가상)"})
                except Exception:  # noqa: BLE001
                    pass
        if time.monotonic() - t_print > 2.0:
            st = "Quest 입력 기다림" if not d.motion_data_ready else (
                "입력 받는 중" if not real else ("실제 팔 세션 중" if sess["active"] else "잡기 웹 [VR 시작] 기다림"))
            print(f"[xr] {st} · 보낸 수 {n_ok}", flush=True)
            t_print = time.monotonic()

    last_err = ""
    try:
        while True:
            t_loop = time.monotonic()
            try:
                step()
            except KeyboardInterrupt:
                raise
            except Exception as e:  # noqa: BLE001 — 한 번 실패해도 멈추지 않음 (실제 팔은 잡기 웹 1.5 초 · arm_server 0.2 초 뒤 멈춤)
                msg = f"{type(e).__name__}: {e}"
                if msg != last_err:
                    import traceback
                    print(f"[xr] ⚠ 오류 (계속 돎): {msg}\n{traceback.format_exc(limit=4)}", flush=True)
                    last_err = msg
            time.sleep(max(0.0, dt - (time.monotonic() - t_loop)))
    except KeyboardInterrupt:
        pass
    finally:
        if real and sess["active"]:
            try:
                post(real + "/api/xr/stop", {}, timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        try:
            tv.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
