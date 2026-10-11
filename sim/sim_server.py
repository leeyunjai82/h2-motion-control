#!/usr/bin/env python3
"""
sim_server.py — 시뮬레이터: 가상 박스 + 가짜 detect_box (포트 50010) + 조작 화면

  ROBOT=h2 ROBOT_SIM=1 python sim/sim_server.py     (보통은 ./start_grab_sim.sh 가 띄운다)
  → http://localhost:50010/

하는 일
  · 가상 박스를 pelvis 기준 좌표(x 앞, y 왼쪽, 윗면 z)에 둔다.
  · fake_robot 의 rt/lowstate(도메인 1)로 현재 허리 자세를 읽어 torso_link 위치를 URDF FK 로 구하고,
    robot.yaml camera 장착값으로 박스를 D435i 카메라 좌표로 바꾼다
    (robot_server.camera_to_torso 의 역변환 — 허리 yaw 를 돌리면 카메라 좌표도 같이 바뀐다).
  · detect_box 와 같은 /pose, /status, /video_feed 를 제공 → robot_server 의 잡기 시퀀스
    (재검출 → 접근 → 잡기 → 들기 → 놓기/건네기) 를 코드 수정 없이 그대로 돌린다.
  · 카메라 화면 시야(640×480, detect_box 의 K) 밖이면 found=False — 장착 위치 검토용.
  · 손 위치 오차: robot_server 의 IK 목표 vs 현재 팔 관절로 계산한 손끝(L_ee/R_ee) — IK 도달 여부 확인용.

없는 것: 실제 영상 인식(YOLO), 접촉/물리, 박스가 손에 따라 움직이는 것, 보행.
"""
import json
import os
import re
import sys
import threading
import time
import urllib.request

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "common"))
import robot_env

if not robot_env.SIM:
    print("[sim_server] ❌ ROBOT_SIM=1 에서만 실행 (start_grab_sim.sh)")
    sys.exit(2)

import cv2
import pinocchio as pin
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# SIM_CAMERA=real (start_grab_sim.sh real-cam): 카메라·인식은 실물 (rs_stream + detect_box:50010), 로봇만 가상.
#   이 서버는 가상 박스/가짜 detect_box 를 끄고 조작 화면만 50012 에서 제공한다.
REAL_CAM = os.environ.get("SIM_CAMERA", "").strip() == "real"
PORT = 50012 if REAL_CAM else 50010   # 가상 카메라일 때는 detect_box 자리(50010)를 대신한다
DETECT = "http://localhost:50010"
ROBOT_SERVER = "http://localhost:50000"
ARM_SERVER = "http://localhost:50022"
IMG_W, IMG_H = 640, 480
INSET = 0.02                       # L/R = 윗면 좌우 변 중점에서 안쪽 2 cm (box_estimator 와 동일)

# ---- 카메라 내부 파라미터: detect_box 와 같은 값 (robot.yaml camera.intrinsics) ----
FX, FY, PPX, PPY = robot_env.CAMERA_K

# ---- 로봇 모델 ----
J = robot_env.JOINTS
N = int(J["motor_slots"])
ARM = [int(i) for i in J["arm"]]
WAIST = [int(i) for i in J["waist"]]
PELVIS_TO_TORSO = np.array(robot_env.CFG["frames"]["pelvis_to_torso"], dtype=float)
CX, CY, CZ, CP = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH

FULL = pin.buildModelFromUrdf(robot_env.URDF_PATH)
FULL_D = FULL.createData()
TORSO_FID = FULL.getFrameId("torso_link")
SLOT_TO_IQ = {int(s): FULL.joints[FULL.getJointId(n)].idx_q for n, s in J["map"].items() if FULL.existJointName(n)}

# IK 축소 모델 (robot_arm_ik.py 와 같은 구성) — 손끝 위치 계산용
_lock = [FULL.getJointId(n) for n in robot_env.CFG["ik"]["lock_joints"] if FULL.existJointName(n)]
RED = pin.buildReducedModel(FULL, _lock, np.zeros(FULL.nq))
_off = np.array(robot_env.CFG["ik"]["ee_offset"], dtype=float)
for nm, jn in zip(("L_ee", "R_ee"), robot_env.CFG["ik"]["ee_joints"]):
    RED.addFrame(pin.Frame(nm, RED.getJointId(jn), pin.SE3(np.eye(3), _off), pin.FrameType.OP_FRAME))
RED_D = RED.createData()
FL, FR = RED.getFrameId("L_ee"), RED.getFrameId("R_ee")
# 축소 모델 q 순서 → 슬롯 (팔 14축)
RED_SLOTS = []
for jid in range(1, RED.njoints):
    nm = RED.names[jid]
    RED_SLOTS.append(int(J["map"][nm]))

# ---- 상태 ----
state = {"q": np.zeros(N), "t": 0.0}
# 기본 = 테이블 110 cm · 박스 15 cm (윗면 pelvis + 0.24), 카메라 46.0° 에서 윗면 전체가 보이는 x (38–61 cm) · auto_zone 안
box = {"present": True, "x": 0.40, "y": round(robot_env.CAMERA_Y, 3), "top": 0.24, "W": 0.28, "D": 0.20, "H": 0.15}
# ↑ y = camera.y (RGB 렌즈 바로 앞) — 렌즈가 왼쪽 3.25 cm 라 y 0 이면 28 cm 박스 오른쪽 끝이 화면 밖 (테이블 110 · 박스 15)
OVERLAY = {"seg": bool((robot_env.CFG.get("vision") or {}).get("show_seg", False))}   # detect_box 와 같은 화면 옵션
lock = threading.Lock()


def lowstate_loop():
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    while True:
        m = sub.Read(1.0)
        if m is None:
            continue
        q = np.array([m.motor_state[i].q for i in range(N)], dtype=float)
        with lock:
            state["q"], state["t"] = q, time.time()


def torso_pose(q):
    """pelvis 기준 torso_link (R, t) — 현재 허리 각 반영."""
    qf = np.zeros(FULL.nq)
    for s, iq in SLOT_TO_IQ.items():
        qf[iq] = q[s]
    pin.framesForwardKinematics(FULL, FULL_D, qf)
    M = FULL_D.oMf[TORSO_FID]
    return M.rotation.copy(), M.translation.copy()


def torso_to_camera(p):
    """robot_server.camera_to_torso 의 역변환."""
    X, Y, Z = p
    cx = CY - Y
    cy_r = CZ - Z
    cz_r = X - CX
    c, s = np.cos(CP), np.sin(CP)
    return np.array([cx, c * cy_r - s * cz_r, s * cy_r + c * cz_r])


def camera_to_torso(cam):
    """robot_server.camera_to_torso 와 같은 식 (검증용)."""
    cx, cy, cz = cam
    c, s = np.cos(CP), np.sin(CP)
    cy_r = cy * c + cz * s
    cz_r = -cy * s + cz * c
    return np.array([cz_r + CX, -cx + CY, -cy_r + CZ])


def project(cam):
    if cam[2] <= 0.05:
        return None
    return FX * cam[0] / cam[2] + PPX, FY * cam[1] / cam[2] + PPY


def box_view():
    """현재 자세에서 박스의 카메라 좌표 / 화면 좌표 / 보이는지."""
    with lock:
        q = state["q"].copy()
        b = dict(box)
    R, t = torso_pose(q)
    to_t = lambda P: R.T @ (np.asarray(P, float) - t)
    x, y, top, W, D = b["x"], b["y"], b["top"], b["W"], b["D"]
    pts = {"L": (x, y + W / 2 - INSET, top), "R": (x, y - W / 2 + INSET, top), "C": (x, y, top)}
    corners = [(x + D / 2, y + W / 2, top), (x + D / 2, y - W / 2, top),
               (x - D / 2, y - W / 2, top), (x - D / 2, y + W / 2, top)]
    cam = {k: torso_to_camera(to_t(P)) for k, P in pts.items()}
    cams_c = [torso_to_camera(to_t(P)) for P in corners]
    uv = [project(c) for c in cams_c]
    visible = b["present"] and all(p is not None and 0 <= p[0] < IMG_W and 0 <= p[1] < IMG_H for p in uv)
    return {"cam": cam, "uv": uv, "uv_LR": [project(cam["L"]), project(cam["R"])], "visible": visible,
            "torso_C": to_t(pts["C"]), "waist_deg": np.degrees(q[WAIST]).tolist(), "box": b}


def hands_ik_frame(q):
    qa = np.array([q[s] for s in RED_SLOTS])
    pin.framesForwardKinematics(RED, RED_D, qa)
    return RED_D.oMf[FL].translation.copy(), RED_D.oMf[FR].translation.copy()


def http_json(url, data=None, method=None, timeout=1.0):
    req = urllib.request.Request(url, data=None if data is None else json.dumps(data).encode(),
                                 method=method or ("POST" if data is not None else "GET"),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


app = FastAPI(title="Sim (fake detect_box)")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


# ---------- detect_box 호환 ----------
def pose():
    v = box_view()
    if not v["visible"]:
        return {"found": False, "n": 0}
    return {"found": True, "type": "cardboard", "n": 10,
            "L": v["cam"]["L"].tolist(), "R": v["cam"]["R"].tolist(),
            "top_center": v["cam"]["C"].tolist(), "box_h": float(v["box"]["H"]), "box_d": float(v["box"]["D"]),
            "method": "sim"}


def status():
    v = box_view()
    out = {"found": v["visible"], "frames": 10 if v["visible"] else 0, "n": 10 if v["visible"] else 0,
           "auto_enabled": False, "auto_in_zone": False, "sim": True, "show_seg": OVERLAY["seg"]}
    if v["visible"]:
        tc = camera_to_torso(v["cam"]["C"])
        out["torso"] = {"x": round(float(tc[0]), 3), "y": round(float(tc[1]), 3), "z": round(float(tc[2]), 3)}
        out["box_h_cm"] = round(float(v["box"]["H"]) * 100, 1)
        out["box_d_cm"] = round(float(v["box"]["D"]) * 100, 1)
    return out


def set_auto_mode(enabled: bool = False):
    return {"ok": True, "enabled": False, "note": "시뮬: 자동 잡기 없음 — 시뮬 화면의 [잡기] 사용"}


def reset_window():
    return {"ok": True}


def set_overlay(seg: bool = None):
    if seg is not None:
        OVERLAY["seg"] = bool(seg)
    return {"success": True, "show_seg": OVERLAY["seg"]}


def render_frame():
    v = box_view()
    img = np.full((IMG_H, IMG_W, 3), 60, np.uint8)
    cv2.putText(img, f"SIM D435i  pitch {robot_env.CAMERA_PITCH_DEG:.1f}deg", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    uv = v["uv"]
    if v["box"]["present"] and all(p is not None for p in uv):
        pts = np.array([[int(p[0]), int(p[1])] for p in uv], np.int32)
        cv2.fillPoly(img, [pts], (60, 120, 170) if v["visible"] else (60, 60, 140))   # 가상 박스 (실물 화면에 해당)
        if OVERLAY["seg"]:
            cv2.polylines(img, [pts], True, (255, 255, 255), 2)
        tc = project(v["cam"]["C"])
        marks = [(lab, (int(p[0]), int(p[1]))) for lab, p in (("L", v["uv_LR"][0]), ("T", tc), ("R", v["uv_LR"][1]))
                 if p is not None]
        col = (255, 0, 255) if not OVERLAY["seg"] else (0, 255, 255)
        for lab, c in marks:
            if lab == "T" and OVERLAY["seg"]:
                continue
            cv2.circle(img, c, 6, col, -1)
            cv2.putText(img, lab, (c[0] + 8, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    cv2.putText(img, "FOUND" if v["visible"] else "NOT VISIBLE", (10, IMG_H - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 220, 80) if v["visible"] else (80, 80, 255), 2)
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


def video_feed():
    def gen():
        while True:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + render_frame() + b"\r\n"
            time.sleep(0.1)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


if not REAL_CAM:   # 가짜 detect_box (가상 카메라)
    app.get("/pose")(pose)
    app.get("/status")(status)
    app.get("/set_auto_mode")(set_auto_mode)
    app.post("/reset_window")(reset_window)
    app.get("/set_overlay")(set_overlay)
    app.get("/video_feed")(video_feed)


def current_pose():
    """잡기에 쓸 인식값 — 실물 카메라면 detect_box /pose, 아니면 가상 박스."""
    if REAL_CAM:
        try:
            return http_json(f"{DETECT}/pose", timeout=1.0)
        except Exception as e:
            return {"found": False, "error": f"detect_box 응답 없음: {e}"}
    return pose()


# ---------- 시뮬 조작 ----------
@app.get("/sim/state")
def sim_state():
    with lock:
        q = state["q"].copy()
        age = time.time() - state["t"]
    hl, hr = hands_ik_frame(q)
    if REAL_CAM:
        v = {"box": None, "visible": False, "torso_C": np.zeros(3), "waist_deg": np.degrees(q[WAIST]).tolist()}
        d = current_pose()
        if d.get("found") and d.get("top_center"):
            v["visible"] = True
            v["torso_C"] = camera_to_torso(d["top_center"])
            v["box"] = {"H": float(d.get("box_h") or 0.065)}
        else:
            v["box"] = {"H": 0.065}
    else:
        v = box_view()
    mid_z = float(v["torso_C"][2] + PELVIS_TO_TORSO[2] - v["box"]["H"] / 2)   # 박스 옆면 중간 높이 (IK 좌표)
    top_z = float(v["torso_C"][2] + PELVIS_TO_TORSO[2])
    out = {"robot": robot_env.ROBOT, "real_cam": REAL_CAM, "lowstate_age_s": round(age, 2), "box": v["box"], "visible": v["visible"],
           "waist_deg": [round(a, 1) for a in v["waist_deg"]],
           "box_torso": [round(float(a), 3) for a in v["torso_C"]],
           # 박스를 IK(손) 좌표로 — 손 높이와 비교 (torso + pelvis_to_torso, 허리 0 축소모델 기준)
           "box_ik_top": [round(float(a), 3) for a in (v["torso_C"] + PELVIS_TO_TORSO)],
           "camera": {"x": CX, "y": CY, "z": CZ, "pitch_deg": robot_env.CAMERA_PITCH_DEG,
                      "placeholder": bool(robot_env.CAMERA.get("placeholder", False))},
           "hands_ik": {"L": hl.round(3).tolist(), "R": hr.round(3).tolist()},
           # 손끝(L_ee) 높이 − 박스 옆면 중간 / 윗면 [cm] — 잡기·접근 높이 확인용
           "hand_vs_box_cm": {"mid": [round((hl[2] - mid_z) * 100, 1), round((hr[2] - mid_z) * 100, 1)],
                              "top": [round((hl[2] - top_z) * 100, 1), round((hr[2] - top_z) * 100, 1)]}}
    try:
        out["grab"] = http_json(f"{ROBOT_SERVER}/grab_status", timeout=0.3)
        viz = http_json(f"{ROBOT_SERVER}/viz", timeout=0.3)
        if viz.get("targets"):
            tL = np.array(viz["targets"]["L"]) + PELVIS_TO_TORSO      # /viz 는 torso 기준 → IK(pelvis) 기준
            tR = np.array(viz["targets"]["R"]) + PELVIS_TO_TORSO
            out["targets_ik"] = {"L": tL.round(3).tolist(), "R": tR.round(3).tolist()}
            out["hand_err_cm"] = [round(float(np.linalg.norm(tL - hl)) * 100, 1),
                                  round(float(np.linalg.norm(tR - hr)) * 100, 1)]
    except Exception as e:
        out["robot_server_err"] = str(e)
    try:
        out["arm"] = http_json(f"{ARM_SERVER}/status", timeout=0.3)
    except Exception as e:
        out["arm_err"] = str(e)
    return out


@app.post("/sim/box")
def sim_box(body: dict):
    with lock:
        for k in ("x", "y", "top", "W", "D", "H"):
            if k in body:
                box[k] = float(body[k])
        if "present" in body:
            box["present"] = bool(body["present"])
        return {"ok": True, "box": dict(box)}


@app.post("/sim/grab")
def sim_grab():
    """robot_server 를 box 모드로 두고 현재 인식값으로 /grab_at (detect_box 자동 잡기와 같은 요청)."""
    p = current_pose()
    if not p.get("found"):
        return JSONResponse({"ok": False, "reason": p.get("error") or "박스 인식 없음 — 카메라 시야/박스 위치 확인"})
    try:
        http_json(f"{ROBOT_SERVER}/set_mode?mode=box", data={}, timeout=3.0)
        time.sleep(2.5)    # set_mode box 의 대기 자세 이동
        r = http_json(f"{ROBOT_SERVER}/grab_at", data={"type": "cardboard", "L": p["L"], "R": p["R"],
                                                        "top_center": p["top_center"], "box_h": p["box_h"]},
                      timeout=3.0)
        return r
    except Exception as e:
        return JSONResponse({"ok": False, "reason": str(e)})


@app.post("/sim/handover")
def sim_handover(direction: str = "center"):
    try:
        return http_json(f"{ROBOT_SERVER}/set_handover_direction?direction={direction}", timeout=1.0)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": str(e)})


@app.post("/sim/stop")
def sim_stop():
    try:
        return http_json(f"{ROBOT_SERVER}/stop", data={}, timeout=3.0)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": str(e)})


@app.post("/sim/home")
def sim_home():
    try:
        http_json(f"{ROBOT_SERVER}/set_mode?mode=none", data={}, timeout=3.0)
        return http_json(f"{ROBOT_SERVER}/home", data={}, timeout=10.0)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": str(e)})


@app.get("/", response_class=HTMLResponse)
def index():
    return (HTML.replace("__ROBOT__", robot_env.ROBOT.upper()).replace("__REALCAM__", "1" if REAL_CAM else "0")
            .replace("__CAMTITLE__", "실물 D435i — detect_box :50010 (YOLO)" if REAL_CAM
                     else "가상 D435i 화면 (detect_box 대체 /video_feed)"))


HTML = r"""<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>__ROBOT__ Sim</title>
<style>
:root{--bg:#14171c;--card:#1d2229;--fg:#e6e9ee;--dim:#8a94a3;--ok:#4cc38a;--bad:#ef6b6b;--acc:#5aa9ff}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
.top{padding:10px 16px;border-bottom:1px solid #2a313b}.top b{font-size:16px}.warn{color:#f5b84d;margin-left:12px}
.wrap{display:grid;grid-template-columns:360px 1fr;gap:12px;padding:12px}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
.card{background:var(--card);border-radius:8px;padding:12px;margin-bottom:12px}
.card h3{margin:0 0 8px;font-size:14px;color:var(--dim)}
label{display:grid;grid-template-columns:120px 1fr 56px;align-items:center;gap:6px;margin:4px 0}
input[type=range]{width:100%}button{background:#2b3440;color:var(--fg);border:0;border-radius:6px;padding:8px 10px;margin:3px;cursor:pointer}
button.go{background:#1f6f4a}button.st{background:#7a2e2e}
table{width:100%;border-collapse:collapse}td{padding:3px 4px;border-bottom:1px solid #2a313b}td:first-child{color:var(--dim);width:42%}
.ok{color:var(--ok)}.bad{color:var(--bad)}img{width:100%;border-radius:6px;background:#000}
iframe{width:100%;height:560px;border:0;border-radius:6px;background:#000}
</style></head><body>
<div class="top"><b>__ROBOT__ 시뮬레이터</b><span class="warn">ROBOT_SIM=1 · DDS 도메인 1 (실기와 분리) · 물리/접촉 없음 · 로봇 가상</span></div>
<div class="wrap"><div>
 <div class="card" id="boxcard"><h3>가상 박스 (pelvis 기준, m)</h3>
  <label>앞 x<input type="range" id="x" min="0.15" max="0.80" step="0.01"><span id="vx"></span></label>
  <label>좌우 y (왼 +)<input type="range" id="y" min="-0.50" max="0.50" step="0.01"><span id="vy"></span></label>
  <label>윗면 높이 top<input type="range" id="top" min="-0.30" max="0.50" step="0.01"><span id="vtop"></span></label>
  <label>폭 W (손 사이)<input type="range" id="W" min="0.10" max="0.45" step="0.01"><span id="vW"></span></label>
  <label>깊이 D<input type="range" id="D" min="0.05" max="0.40" step="0.01"><span id="vD"></span></label>
  <label>높이 H<input type="range" id="H" min="0.03" max="0.30" step="0.01"><span id="vH"></span></label>
  <div><button onclick="pres(true)">박스 놓기</button><button onclick="pres(false)">박스 치우기</button></div>
 </div>
 <div class="card"><h3>동작</h3>
  <button class="go" onclick="post('/sim/grab')">잡기 → 건네기</button>
  <button onclick="post('/sim/handover?direction=center')">건네기 정면</button>
  <button onclick="post('/sim/handover?direction=left')">좌</button>
  <button onclick="post('/sim/handover?direction=right')">우</button><br>
  <button class="st" onclick="post('/sim/stop')">정지</button><button onclick="post('/sim/home')">홈</button>
  <div id="msg" style="color:var(--dim);margin-top:6px"></div>
 </div>
 <div class="card"><h3>상태</h3><table id="st"></table></div>
</div><div>
 <div class="card"><h3>__CAMTITLE__</h3><img id="cam"></div>
 <div class="card"><h3>3D (dashboard :50003)</h3><iframe id="dash"></iframe></div>
</div></div>
<script>
const host=location.hostname;document.getElementById('dash').src=`http://${host}:50003/dashboard`;
const REALCAM=__REALCAM__;
document.getElementById('cam').src=REALCAM?`http://${host}:50010/video_feed`:'/video_feed';
if(REALCAM)document.getElementById('boxcard').style.display='none';
const K=['x','y','top','W','D','H'];let inited=false,tmr=null;
function send(){const b={};K.forEach(k=>b[k]=+document.getElementById(k).value);
  fetch('/sim/box',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});}
K.forEach(k=>document.getElementById(k).addEventListener('input',e=>{
  document.getElementById('v'+k).textContent=(+e.target.value).toFixed(2);clearTimeout(tmr);tmr=setTimeout(send,80);}));
function pres(p){fetch('/sim/box',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({present:p})});}
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));
  document.getElementById('msg').textContent=u+' → '+JSON.stringify(d);}
const row=(k,v,c)=>`<tr><td>${k}</td><td class="${c||''}">${v}</td></tr>`;
async function poll(){try{const d=await(await fetch('/sim/state')).json();
  if(!inited&&!d.real_cam){K.forEach(k=>{const el=document.getElementById(k);el.value=d.box[k];
    document.getElementById('v'+k).textContent=(+d.box[k]).toFixed(2);});inited=true;}
  const g=d.grab||{},e=d.hand_err_cm;let h='';
  h+=row('로봇',d.robot+(d.lowstate_age_s<0.5?'':' (lowstate 끊김)'),d.lowstate_age_s<0.5?'ok':'bad');
  h+=row('arm_server',d.arm?`${d.arm.mode} · weight ${(+d.arm.weight).toFixed(2)}`:(d.arm_err||'-'),d.arm?'':'bad');
  h+=row('잡기 단계',g.busy?`${g.stage_idx+1}/${(g.stages||[]).length} ${g.stage}`:(g.mode?`대기 (mode ${g.mode})`:'-'));
  h+=row(d.real_cam?'박스 인식 (detect_box)':'카메라 시야',d.visible?(d.real_cam?'인식됨':'보임'):(d.real_cam?'없음':'안 보임'),d.visible?'ok':'bad');
  h+=row('박스 (torso 기준)',d.box_torso.join(', '));
  h+=row('허리 yaw/roll/pitch°',d.waist_deg.join(' / '));
  h+=row('손 목표 오차 L/R',e?`${e[0]} / ${e[1]} cm`:'-',e?(Math.max(...e)>2?'bad':'ok'):'');
  h+=row('손끝 L (IK 기준)',d.hands_ik.L.join(', '));h+=row('손끝 R',d.hands_ik.R.join(', '));
  h+=row('손 높이 − 박스 옆면 중간 L/R',`${d.hand_vs_box_cm.mid[0]} / ${d.hand_vs_box_cm.mid[1]} cm`);
  h+=row('손 높이 − 박스 윗면 L/R',`${d.hand_vs_box_cm.top[0]} / ${d.hand_vs_box_cm.top[1]} cm`);
  h+=row('카메라 장착',`x ${d.camera.x} y ${d.camera.y} z ${d.camera.z} · ${d.camera.pitch_deg}°`+(d.camera.placeholder?' (임시값)':''),d.camera.placeholder?'bad':'');
  if(d.robot_server_err)h+=row('robot_server',d.robot_server_err,'bad');
  document.getElementById('st').innerHTML=h;}catch(e){}}
poll();setInterval(poll,300);
</script></body></html>"""


if __name__ == "__main__":
    robot_env.dds_init()
    threading.Thread(target=lowstate_loop, daemon=True).start()
    # 역변환 자기 검증 (camera_to_torso ∘ torso_to_camera = 항등)
    _p = np.array([0.4, 0.1, -0.2])
    assert np.allclose(camera_to_torso(torso_to_camera(_p)), _p), "카메라 역변환 오류"
    print(f"[sim_server] ROBOT={robot_env.ROBOT}  http://0.0.0.0:{PORT}/  (K fx={FX} fy={FY})")
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
