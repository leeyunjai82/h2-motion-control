"""
vr_teleop.py — Quest 3 (WebXR) 컨트롤러 → H2 팔 IK.  1단계: 읽기 전용

  로봇(arm_server · Dex1-1)에는 아무 명령도 보내지 않음. 가상 팔(pinocchio 모델)만 손을 따라 움직이고,
  따라가는지 · 몸통/테이블/다른 팔 거리 · 관절 한계를 모니터 웹에 보여 줌. 2단계(실제 팔)는 같은 계산 결과를 arm_server 로 보냄.

  실행 (mini PC):
    python gripper/vr_teleop.py                 # 모니터  http://localhost:50040/
  Quest 3 (USB, 개발자 모드):
    adb reverse tcp:50040 tcp:50040            # Quest 브라우저에서 http://localhost:50040/vr  (localhost 라 HTTPS 필요 없음)
  Quest 3 (Wi-Fi): WebXR 는 HTTPS 필요 → --https vr.crt vr.key (자체 서명, 브라우저 경고 수락 — 동작 확인 필요)

  조작: 그립(중지) 버튼을 누르고 있는 동안만 따라감 — 누른 순간의 컨트롤러 · 그리퍼 위치 기준 상대 이동.
        떼면 그 자리 유지. 트리거 = 그리퍼 닫기. B/Y = 방향 맞춤 (지금 보는 방향 = 로봇 앞).
  좌표: WebXR (x 오른쪽, y 위, −z 앞) → pelvis (x 앞, y 왼쪽, z 위). 조작자가 로봇과 같은 방향을 보고 '방향 맞춤'.
"""
import argparse
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pick_core as pc      # 모델 · IK · 그리퍼 겉 점 · 몸통 거리 (arm_server 호출 함수는 쓰지 않음)
from pick_core import P

import numpy as np
import pinocchio as pin
import uvicorn
import xml.etree.ElementTree as ET
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, Response

READ_ONLY = True            # 1단계 — 이 파일에는 arm_server / 그리퍼로 보내는 코드가 없음

V = pc.CFG.get("vr") or {}
_need = ("port", "scale", "v_max", "w_max_deg", "joint_vmax", "ik_iters", "posture_gain", "follow_mm",
         "follow_deg", "unreach_mm", "arm_gap_m", "table_z_m", "workspace", "timeout_s")
_miss = [k for k in _need if k not in V]
if _miss:
    sys.exit(f"[vr] gripper.yaml vr: 에 {_miss} 없음 — 기본값으로 대신하지 않음")
if not (0 < float(V["joint_vmax"]) <= float(P["server_max_vel"])):
    sys.exit("[vr] gripper.yaml vr.joint_vmax 는 0 보다 크고 pick.server_max_vel 이하여야 함")

# WebXR → pelvis 축 바꿈 (행: pelvis x, y, z 가 WebXR 의 어느 축인지). det = +1
M_XR = np.array([[0.0, 0.0, -1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0]])
SIDES = ("right", "left")
JOINT_SHORT = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
NEAR_LIMIT = math.radians(5.0)


def quat_to_R(q):
    x, y, z, w = (float(v) for v in q)
    return pin.Quaternion(w, x, y, z).normalized().toRotationMatrix()


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rpy_deg(R):
    return [round(math.degrees(v), 1) for v in pin.rpy.matrixToRpy(R)]


def r3(v, n=3):
    return [round(float(x), n) for x in v]


class Hand:
    def __init__(self, side):
        self.side = side
        self.engaged = False
        self.c0 = None              # 그립 누른 순간 컨트롤러 (p, R) — 방향 맞춘 WebXR
        self.t0 = None              # 그 순간 가상 TCP (p, R) — pelvis
        self.tgt = None             # 속도 제한한 목표 TCP (p, R)
        self.trig = 0.0
        self.st = {"state": "대기", "reason": "", "level": "idle"}
        self.haptic = 0.0


class Teleop:
    def __init__(self, table_z, scale, v_max):
        self.mdl = pc.Model()
        self.table_z = float(table_z)
        self.scale, self.v_max = float(scale), float(v_max)
        self.w_max = math.radians(float(V["w_max_deg"]))
        self.jv = float(V["joint_vmax"])
        self.ws = V["workspace"]
        self.q_rest = self._ready_q()
        self.q = self.q_rest.copy()
        self.hands = {s: Hand(s) for s in SIDES}
        self.heading = 0.0          # 방향 맞춤 [rad] (WebXR y 축 둘레)
        self.lock = threading.Lock()
        self.last_msg = 0.0
        self.last_t = None
        self.rate = 0.0
        self.rtt_ms = None
        self.n_msg = 0
        self.n_sock = 0             # 열린 WebSocket 수 (Quest 와 3D 보기 가짜 컨트롤러가 동시에 보내면 섞임)
        m = self.mdl.m
        self.sh_jid = {s: m.getJointId(f"{s}_shoulder_pitch_joint") for s in pc.SIDES}
        self.gap_max = pc.gap_mm(pc.Q_HI)

    # ---------- 시작 자세 ----------
    def _ready_q(self):
        q = pc.default_q_ctrl()
        if q is None:
            sys.exit("[vr] robot.yaml default_arm_deg 없음")
        for side in SIDES:
            segs, info = pc.plan_pose(self.mdl, q, side, "ready")
            if segs is None:
                if info != "이미 준비 자세":
                    sys.exit(f"[vr] {side} 준비 자세 계산 실패: {info}")
                continue
            q = self.mdl.to_ctrl(segs[-1].qs[-1])
        return self.mdl.to_model(q)

    def reset(self):
        with self.lock:
            self.q = self.q_rest.copy()
            for h in self.hands.values():
                h.engaged, h.c0, h.t0, h.tgt = False, None, None, None
                h.st = {"state": "대기", "reason": "준비 자세로 리셋", "level": "idle"}

    # ---------- IK (위치 우선 → 남는 자유도로 자세 → 그다음 준비 자세 유지) ----------
    def ik(self, q0, side, p, R):
        """작업 우선순위 감쇠 최소제곱: 위치는 반드시, 자세는 위치를 해치지 않는 만큼만 (손목 한계에 걸리면 자세만 덜 따라감)."""
        mdl = self.mdl
        m, d, f, ix = mdl.m, mdl.d, mdl.fid[side], mdl.idx[side]
        kp = float(V["posture_gain"])
        n = len(ix)
        I = np.eye(n)
        qm = q0.copy()
        for _ in range(int(V["ik_iters"])):
            pin.framesForwardKinematics(m, d, qm)
            pin.computeJointJacobians(m, d, qm)
            Mf = d.oMf[f]
            e_p = p - Mf.translation
            e_r = pin.log3(R @ Mf.rotation.T)
            J = pin.getFrameJacobian(m, d, f, pin.LOCAL_WORLD_ALIGNED)[:, ix]
            Jp, Jr = J[:3], J[3:]
            Jp_i = Jp.T @ np.linalg.inv(Jp @ Jp.T + 1e-4 * np.eye(3))
            Np = I - Jp_i @ Jp
            dq = Jp_i @ e_p
            Jr_n = Jr @ Np
            Jr_i = Jr_n.T @ np.linalg.inv(Jr_n @ Jr_n.T + 1e-2 * np.eye(3))
            dq = dq + Jr_i @ (e_r - Jr @ dq)
            Nall = Np - Jr_i @ Jr_n
            dq = dq + Nall @ (kp * (self.q_rest[ix] - qm[ix]))
            qm[ix] = np.clip(qm[ix] + np.clip(dq, -0.1, 0.1), mdl.lo[ix], mdl.hi[ix])
        T = mdl.tcp(qm, side)
        return qm, float(np.linalg.norm(p - T.translation)), float(np.linalg.norm(pin.log3(R @ T.rotation.T)))

    # ---------- 안전 검사 ----------
    def arm_pts(self, qm, side):
        mdl = self.mdl
        pin.forwardKinematics(mdl.m, mdl.d, qm)
        pts = [mdl.d.oMi[self.sh_jid[side]].translation.copy()]
        pts += [mdl.d.oMi[mdl.jid[side][n]].translation.copy() for n in ("elbow", "wrist_roll", "wrist_yaw")]
        pts.append(mdl.tcp(qm, side).translation.copy())
        return pts

    def arm_gap(self, qm):
        a = self.arm_pts(qm, "left")[1:]
        b = self.arm_pts(qm, "right")[1:]
        return min(float(np.linalg.norm(x - y)) for x in a for y in b)

    def check(self, qm, side):
        """→ (이유 목록, 값 dict). 이유가 있으면 그 자세는 쓰지 않음."""
        mdl, why = self.mdl, []
        g = mdl.body_gap(qm, side)
        low = mdl.lowest(qm, side) - self.table_z
        ag = self.arm_gap(qm)
        if g is not None and g < float(P["body_clear_m"]):
            why.append(f"몸통 {g * 100:.0f} cm (최소 {float(P['body_clear_m']) * 100:.0f})")
        if low * 1000 < float(P["tip_clear_mm"]):
            why.append(f"테이블 {low * 1000:.0f} mm (최소 {float(P['tip_clear_mm']):.0f})")
        if ag < float(V["arm_gap_m"]):
            why.append(f"다른 팔 {ag * 100:.0f} cm (최소 {float(V['arm_gap_m']) * 100:.0f})")
        return why, {"body_gap_cm": None if g is None else round(g * 100, 1), "table_mm": round(low * 1000),
                     "arm_gap_cm": round(ag * 100, 1)}

    def clamp_ws(self, p):
        q = p.copy()
        q[0] = min(max(q[0], self.ws["x"][0]), self.ws["x"][1])
        q[1] = min(max(q[1], -self.ws["y_abs"]), self.ws["y_abs"])
        q[2] = min(max(q[2], self.ws["z"][0]), self.ws["z"][1])
        return q, bool(np.linalg.norm(q - p) > 1e-6)

    # ---------- 한 tick ----------
    def update(self, msg):
        now = time.monotonic()
        with self.lock:
            dt = 0.02 if self.last_t is None else min(max(now - self.last_t, 0.0), 0.1)
            if self.last_t is not None and now > self.last_t:
                self.rate = 0.9 * self.rate + 0.1 * (1.0 / max(now - self.last_t, 1e-3))
            self.last_t, self.last_msg = now, now
            self.n_msg += 1
            if msg.get("rtt") is not None:
                self.rtt_ms = float(msg["rtt"])
            head = msg.get("head")
            rq = msg.get("recenter_q") if msg.get("recenter") else None   # B/Y 를 누른 컨트롤러가 가리키는 방향 (목에 걸었을 때 정확)
            if rq or (head and (msg.get("recenter") or self.n_msg == 1)):
                f = quat_to_R(rq or head["q"]) @ np.array([0.0, 0.0, -1.0])
                self.heading = math.atan2(-f[0], -f[2])
                for h in self.hands.values():          # 기준이 바뀌었으니 그립을 다시 잡아야 함
                    h.engaged = False
            H = rot_y(-self.heading)
            hands = msg.get("hands") or {}
            for side in SIDES:
                self._hand(self.hands[side], hands.get(side), H, dt)
            return {"t": msg.get("t"), "haptic": {s: self.hands[s].haptic for s in SIDES},
                    "level": {s: self.hands[s].st["level"] for s in SIDES}}

    def _hand(self, h, c, H, dt):
        mdl, side = self.mdl, h.side
        h.haptic = 0.0
        T = mdl.tcp(self.q, side)
        if not c or c.get("p") is None:
            h.engaged, h.tgt = False, None
            h.st = {"state": "컨트롤러 없음", "reason": "", "level": "idle"}
            return
        h.trig = float(c.get("trig") or 0.0)
        grip = float(c.get("grip") or 0.0) > 0.5
        p_c = H @ np.asarray(c["p"], dtype=float)
        R_c = H @ quat_to_R(c["q"])
        if not grip:
            h.engaged, h.tgt = False, None
            h.st = {"state": "대기", "reason": "그립 버튼을 누르면 따라감", "level": "idle"}
            return
        if not h.engaged:
            h.engaged = True
            h.c0 = (p_c, R_c)
            h.t0 = (T.translation.copy(), T.rotation.copy())
            h.tgt = (T.translation.copy(), T.rotation.copy())
        # 원하는 목표 = 그립 순간 TCP + (컨트롤러 상대 이동, pelvis 축)
        p_des = h.t0[0] + self.scale * (M_XR @ (p_c - h.c0[0]))
        R_des = (M_XR @ (R_c @ h.c0[1].T) @ M_XR.T) @ h.t0[1]
        p_des, clipped_ws = self.clamp_ws(p_des)
        # 목표 속도 제한
        step = p_des - h.tgt[0]
        n = float(np.linalg.norm(step))
        lim_p = self.v_max * dt
        slowed = n > lim_p + 1e-9
        p_t = h.tgt[0] + (step * (lim_p / n) if slowed else step)
        w = pin.log3(R_des @ h.tgt[1].T)
        a = float(np.linalg.norm(w))
        lim_r = self.w_max * dt
        if a > lim_r + 1e-9:
            w, slowed = w * (lim_r / a), True
        R_t = pin.exp3(w) @ h.tgt[1]
        prev_tgt = h.tgt
        h.tgt = (p_t, R_t)
        # IK → 관절 속도 제한 → 안전 검사. 자세를 다 못 맞추면 위치를 지키는 만큼만 돌림 (지금 자세 → 목표 자세 사이로 줄여 가며)
        w_gap = pin.log3(R_t @ T.rotation.T)
        for frac in (1.0, 0.5, 0.25, 0.0):
            R_try = pin.exp3(frac * w_gap) @ T.rotation
            qik, ep, er = self.ik(self.q, side, p_t, R_try)
            if ep * 1000 <= float(V["follow_mm"]):
                break
        ix = mdl.idx[side]
        reasons = []
        if ep * 1000 > float(V["unreach_mm"]):
            reasons.append(f"닿지 않음 (IK {ep * 1000:.0f} mm)")
        q_new = self.q.copy()
        lim_q = self.jv * dt
        dq = qik[ix] - self.q[ix]
        big = float(np.max(np.abs(dq)))
        if big > lim_q:                 # 관절별로 자르면 움직이는 방향이 틀어져 위치가 밀림 — 전체를 같은 비율로 줄임
            dq = dq * (lim_q / big)
        q_new[ix] = self.q[ix] + dq
        why, vals = self.check(q_new, side)
        reasons += why
        if reasons:
            h.haptic = 0.6
            h.tgt = prev_tgt            # 목표가 막힌 쪽으로 계속 달아나지 않게 — 손을 되돌리면 바로 다시 따라감
            h.st = {"state": "멈춤", "reason": " · ".join(reasons), "level": "stop"}
            h.st.update(vals)
            return
        self.q = q_new
        Ta = mdl.tcp(self.q, side)
        e_p = float(np.linalg.norm(p_t - Ta.translation)) * 1000
        e_r = math.degrees(float(np.linalg.norm(pin.log3(R_t @ Ta.rotation.T))))
        notes = []
        if clipped_ws:
            notes.append("작업 범위 끝")
        if slowed:
            notes.append("속도 제한")
        if e_p <= float(V["follow_mm"]) and e_r <= float(V["follow_deg"]):
            state, level = "따라감", "ok"
        elif ep * 1000 <= float(V["follow_mm"]) and e_p > float(V["follow_mm"]):
            state, level = "따라가는 중 (관절 속도)", "ok"
        elif e_p <= float(V["follow_mm"]):
            state, level = f"자세 일부만 ({e_r:.0f}° 못 따라감)", "warn"
            h.haptic = 0.2
        else:
            state, level = f"위치 {e_p:.0f} mm 못 따라감", "warn"
            h.haptic = 0.3
        h.st = {"state": state, "reason": " · ".join(notes), "level": level, "err_mm": round(e_p, 1),
                "err_deg": round(e_r, 1)}
        h.st.update(vals)

    # ---------- 모니터 ----------
    def state(self):
        with self.lock:
            mdl = self.mdl
            out = {"read_only": READ_ONLY, "n_sock": self.n_sock, "rate_hz": round(self.rate, 1), "rtt_ms": self.rtt_ms,
                   "age_s": None if not self.last_msg else round(time.monotonic() - self.last_msg, 2),
                   "heading_deg": round(math.degrees(self.heading), 1), "table_z": self.table_z,
                   "v_max": self.v_max, "scale": self.scale,
                   "body": {"front_x": float(P["body_front_x"]), "half_y": float(P["body_half_y"]),
                            "z_min": float(P["body_z_min"])}, "hands": {}}
            if self.last_msg and time.monotonic() - self.last_msg > float(V["timeout_s"]):
                for h in self.hands.values():
                    if h.engaged or h.st["state"] not in ("연결 끊김",):
                        h.engaged, h.tgt = False, None
                        h.st = {"state": "연결 끊김", "reason": f"{float(V['timeout_s'])} 초 넘게 값 없음", "level": "idle"}
            for side in SIDES:
                h = self.hands[side]
                T = mdl.tcp(self.q, side)
                ix = mdl.idx[side]
                joints = []
                for k, i in enumerate(ix):
                    nm = mdl.qname[i].replace(f"{side}_", "").replace("_joint", "")
                    v, lo, hi = float(self.q[i]), float(mdl.lo[i]), float(mdl.hi[i])
                    joints.append({"name": nm, "deg": round(math.degrees(v), 1), "lo": round(math.degrees(lo), 1),
                                   "hi": round(math.degrees(hi), 1), "near": bool(v - lo < NEAR_LIMIT or hi - v < NEAR_LIMIT)})
                out["hands"][side] = {
                    "engaged": h.engaged, **h.st,
                    "tcp": r3(T.translation), "tcp_rpy": rpy_deg(T.rotation),
                    "target": None if h.tgt is None else r3(h.tgt[0]),
                    "target_rpy": None if h.tgt is None else rpy_deg(h.tgt[1]),
                    "gripper_mm": round(self.gap_max * (1.0 - h.trig), 1),
                    "pts": [r3(p) for p in self.arm_pts(self.q, side)],
                    "joints": joints}
            return out


class View:
    """3D 보기 (URDF 메시) — 가상 팔 관절을 전체 모델에 넣어 링크 자세를 계산. 팔 말고 다른 관절은 0."""

    def __init__(self, t):
        self.t = t
        env = pc.robot_env
        self.full = pin.buildModelFromUrdf(env.URDF_PATH)
        self.fd = self.full.createData()
        rm = t.mdl.m
        self.qmap = [(self.full.joints[self.full.getJointId(rm.names[j])].idx_q, rm.joints[j].idx_q) for j in range(1, rm.njoints)]
        self.mesh_dir = env.MESH_DIR
        files = {f.lower(): f for f in os.listdir(self.mesh_dir)}
        self.visuals, self.fids = [], {}
        for link in ET.parse(env.URDF_PATH).getroot().findall("link"):
            name = link.get("name")
            for vis in link.findall("visual"):
                mesh = vis.find("geometry/mesh")
                if mesh is None:
                    continue
                fn = files.get(os.path.basename(mesh.get("filename", "")).lower())
                if fn is None or not self.full.existFrame(name, pin.BODY):
                    continue
                o = vis.find("origin")
                xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
                Tm = np.eye(4)
                Tm[:3, :3], Tm[:3, 3] = pin.rpy.rpyToMatrix(*rpy), xyz
                sc = [float(v) for v in mesh.get("scale", "1 1 1").split()]
                col = vis.find("material/color")
                rgba = [float(v) for v in col.get("rgba").split()] if col is not None and col.get("rgba") else [0.75, 0.75, 0.78, 1.0]
                self.visuals.append({"link": name, "mesh": fn, "origin": Tm.T.flatten().round(6).tolist(), "scale": sc, "rgba": rgba})
                self.fids[name] = self.full.getFrameId(name, pin.BODY)
        self.mesh_path = {v["mesh"]: os.path.join(self.mesh_dir, v["mesh"]) for v in self.visuals}
        self.mesh_bytes = {}        # 메모리에서 만든 메시 (어댑터)
        self.dex1 = self._dex1()
        if self.dex1:       # H2 URDF 의 wrist_yaw 메시에는 기본 다섯 손가락 손이 붙어 있음 — 어댑터 시작보다 앞쪽 삼각형은 안 그림
            for v in self.visuals:
                if v["link"].endswith("_wrist_yaw_link"):
                    v["cut_x"] = self.dex1["wrist_face"]

    def _dex1(self):
        """Dex1-1 (Unitree 공식 URDF) — 몸체 + 손가락 2 (직선 관절). 손목 wrist_yaw 기준 장착 자세. 없으면 None (상자만 그림)."""
        dv = V.get("dex1_view") or {}
        path = os.path.join(HERE, str(dv.get("urdf", "")))
        if not dv or not os.path.isfile(path):
            print(f"[vr] Dex1 URDF 없음 ({path}) — 3D 보기에서 그리퍼를 상자로만 그림", flush=True)
            return None
        mdir = os.path.dirname(path)
        root = ET.parse(path).getroot()
        links = {}
        for link in root.findall("link"):
            vis = link.find("visual")
            mesh = None if vis is None else vis.find("geometry/mesh")
            if mesh is None:
                continue
            fn = os.path.basename(mesh.get("filename", ""))
            fp = os.path.join(mdir, "meshes", fn)
            if not os.path.isfile(fp):
                continue                      # Camera.STL 등 안 가져온 메시
            col = vis.find("material/color")
            key = "dex1_" + fn
            self.mesh_path[key] = fp
            links[link.get("name")] = {"mesh": key, "rgba": [float(v) for v in col.get("rgba").split()] if col is not None else [0.8, 0.8, 0.85, 1]}
        base = next((j.find("parent").get("link") for j in root.findall("joint")), None)
        fingers = []
        for j in root.findall("joint"):
            c = j.find("child").get("link")
            if j.get("type") != "prismatic" or c not in links or j.find("parent").get("link") != base:
                continue
            fingers.append({**links[c], "xyz": [float(v) for v in j.find("origin").get("xyz").split()],
                            "axis": [float(v) for v in j.find("axis").get("xyz").split()],
                            "upper": float(j.find("limit").get("upper"))})
        if base not in links or len(fingers) != 2:
            print("[vr] Dex1 URDF 구조가 예상과 다름 (몸체 + 직선 손가락 2) — 상자로만 그림", flush=True)
            return None
        Tm = np.eye(4)
        Tm[:3, :3] = pin.rpy.rpyToMatrix(*[math.radians(float(v)) for v in dv["mount_rpy_deg"]])
        Tm[0, 3] = float(pc.TCP_OFF[0]) - float(dv["origin_to_tcp_m"])
        out = {"base": links[base], "fingers": fingers, "mount": Tm.T.flatten().round(6).tolist(),
               "adapter": [float(v) for v in dv["adapter_m"]], "adapter_mesh": None, "wrist_face": float(dv["wrist_face_m"])}
        stl = self._adapter_stl(dv.get("adapter_stl"))
        if stl is not None:
            self.mesh_bytes["adapter"] = stl
            out["adapter_mesh"] = "adapter"
        return out

    def _adapter_stl(self, a):
        """사용자 어댑터 STL (mm, 원본 좌표) → 손목 wrist_yaw 기준 (m) 이진 STL. 파일이 없으면 None (원통으로 그림)."""
        if not a:
            return None
        path = os.path.join(HERE, str(a["path"]))
        if not os.path.isfile(path):
            print(f"[vr] 어댑터 STL 없음 ({path}) — 원통으로 그림", flush=True)
            return None
        raw = open(path, "rb").read()
        n = int.from_bytes(raw[80:84], "little") if len(raw) >= 84 else 0
        if len(raw) == 84 + n * 50:
            v = np.frombuffer(raw[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))["v"].reshape(-1, 3).astype(float)
        else:
            import re
            v = np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", raw), dtype=float)
        if len(v) < 3 or len(v) % 3:
            print(f"[vr] 어댑터 STL 을 못 읽음 ({path}) — 원통으로 그림", flush=True)
            return None
        R = np.asarray(a["rot"], dtype=float)
        w = (v - np.asarray(a["origin_mm"], dtype=float)) @ R.T * float(a["scale"]) + np.asarray(a["at_m"], dtype=float)
        tri = w.reshape(-1, 3, 3)
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
        rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
        rec["n"], rec["v"] = nrm, tri.reshape(-1, 9)
        lo, hi = w.min(axis=0), w.max(axis=0)
        print(f"[vr] 어댑터 STL {len(tri)} 삼각형 — 손목 기준 x {lo[0] * 1000:.1f}~{hi[0] * 1000:.1f} mm · "
              f"y ±{max(abs(lo[1]), hi[1]) * 1000:.1f} · z ±{max(abs(lo[2]), hi[2]) * 1000:.1f}", flush=True)
        return b"\0" * 80 + len(tri).to_bytes(4, "little") + rec.tobytes()

    def info(self):
        e = pc.ENVELOPE
        return {"visuals": self.visuals, "dex1": self.dex1, "table_z": self.t.table_z,
                "body": {"front_x": float(P["body_front_x"]), "half_y": float(P["body_half_y"]), "z_min": float(P["body_z_min"]),
                         "clear": float(P["body_clear_m"])},
                "envelope": {"min": e.min(axis=0).round(4).tolist(), "max": e.max(axis=0).round(4).tolist()}}

    def poses(self):
        t = self.t
        with t.lock:
            qf = np.zeros(self.full.nq)
            for fi, ri in self.qmap:
                qf[fi] = t.q[ri]
            pin.framesForwardKinematics(self.full, self.fd, qf)
            links = {n: self.fd.oMf[f].homogeneous.T.flatten().round(5).tolist() for n, f in self.fids.items()}
            hands = {}
            for s in SIDES:
                h = t.hands[s]
                T = t.mdl.tcp(t.q, s)
                hands[s] = {"tcp": T.homogeneous.T.flatten().round(5).tolist(), "level": h.st["level"],
                            "open": round(min(1.0, max(0.0, 1.0 - h.trig)), 3),     # 벌림 비율 (0 닫힘 ~ 1 최대) — 손가락 그림용
                            "target": None if h.tgt is None else r3(h.tgt[0]), "state": h.st["state"], "reason": h.st.get("reason", "")}
            return {"links": links, "hands": hands}


def make_app(t):
    app = FastAPI()
    view = View(t)
    here = HERE

    def page(name):
        with open(os.path.join(here, name), encoding="utf-8") as f:
            return HTMLResponse(f.read())

    @app.get("/")
    def monitor():
        return page("vr_monitor.html")

    @app.get("/vr")
    def vr():
        return page("vr.html")

    @app.get("/vrurdf")
    def vr_urdf_page():
        return page("vr_urdf.html")

    @app.get("/view")
    def view_page():
        return page("vr_view.html")

    @app.get("/api/view/info")
    def api_view_info():
        return view.info()

    @app.get("/api/view/poses")
    def api_view_poses():
        return view.poses()

    @app.get("/api/mesh/{name}")
    def api_mesh(name: str):
        if name in view.mesh_bytes:
            return Response(view.mesh_bytes[name], media_type="application/octet-stream")
        if name not in view.mesh_path:
            raise HTTPException(404, "mesh 없음")
        return FileResponse(view.mesh_path[name], media_type="application/octet-stream")

    @app.get("/vendor/three.min.js")
    def three_js():
        f = os.path.join(pc.robot_env.VENDOR_DIR, "three.min.js")
        if not os.path.exists(f):
            raise HTTPException(404, f"{f} 없음 (common/dashboard.py 와 같은 파일)")
        return FileResponse(f, media_type="application/javascript")

    @app.get("/api/state")
    def api_state():
        return t.state()

    @app.post("/api/reset")
    def api_reset():
        t.reset()
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        t.n_sock += 1
        try:
            while True:
                msg = await sock.receive_json()
                await sock.send_json(t.update(msg))
        except WebSocketDisconnect:
            pass
        finally:
            t.n_sock -= 1
            with t.lock:
                for h in t.hands.values():
                    h.engaged, h.tgt = False, None
                    h.st = {"state": "연결 끊김", "reason": "", "level": "idle"}

    return app


def main():
    ap = argparse.ArgumentParser(description="Quest 3 WebXR → H2 팔 IK (1단계 읽기 전용, 로봇 명령 없음)")
    ap.add_argument("--port", type=int, default=int(V["port"]))
    ap.add_argument("--table-z", type=float, default=float(V["table_z_m"]), help="테이블 높이 [m, pelvis]")
    ap.add_argument("--scale", type=float, default=float(V["scale"]))
    ap.add_argument("--v-max", type=float, default=float(V["v_max"]), help="목표 TCP 최고 속도 [m/s]")
    ap.add_argument("--https", nargs=2, metavar=("CERT", "KEY"), help="Wi-Fi 로 쓸 때 (WebXR 는 localhost 아니면 HTTPS 필요)")
    a = ap.parse_args()
    if not (0 < a.v_max <= 0.3) or not (0.2 <= a.scale <= 3.0):
        sys.exit("[vr] --v-max 0~0.3 m/s, --scale 0.2~3.0")
    t = Teleop(a.table_z, a.scale, a.v_max)
    print(f"[vr] 읽기 전용 — 로봇에 명령 안 보냄. 모니터 http://localhost:{a.port}/  ·  Quest: adb reverse tcp:{a.port} tcp:{a.port} 후 "
          f"http://localhost:{a.port}/vrurdf (URDF 가상 로봇) 또는 /vr", flush=True)
    kw = {"ssl_certfile": a.https[0], "ssl_keyfile": a.https[1]} if a.https else {}
    uvicorn.run(make_app(t), host="0.0.0.0", port=a.port, log_level="warning", **kw)


if __name__ == "__main__":
    main()
