"""
xr_viewer.py — 가상 H2 3D 뷰어 (URDF + Dex1-1 메시) · 로봇 명령 없음

  gripper/xr_virtual.py (Unitree xr_teleoperate 의 Quest 입력 + H2 IK) 가 보내는 팔 관절값을 받아 가상 로봇으로 그리고,
  우리 안전 검사 (몸통 · 테이블 · 다른 팔 · 뒤 한계) 를 "실제였으면 멈춤" 으로 표시만 함.

  python gripper/xr_viewer.py [--table-z -1.0]      # http://localhost:50040/
"""
import argparse
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pick_core as pc      # 모델 · 그리퍼 겉 점 · 몸통 거리 (arm_server 호출 함수는 쓰지 않음)
from pick_core import P

import numpy as np
import pinocchio as pin
import uvicorn
import xml.etree.ElementTree as ET
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, HTMLResponse, Response

V = pc.CFG.get("xr_view") or {}
_need = ("port", "table_z_m", "table_x_min_m", "arm_gap_m", "back_limit_m", "timeout_s", "dex1_view")
_miss = [k for k in _need if k not in V]
if _miss:
    sys.exit(f"[xr_view] gripper.yaml xr_view: 에 {_miss} 없음 — 기본값으로 대신하지 않음")

SIDES = ("right", "left")
NEAR_LIMIT = math.radians(5.0)


def rpy_deg(R):
    return [round(math.degrees(v), 1) for v in pin.rpy.matrixToRpy(R)]


def r3(v, n=3):
    return [round(float(x), n) for x in v]


class Hand:
    def __init__(self, side):
        self.side = side
        self.trig = 0.0             # 그리퍼 닫힘 비율 (0 벌림 ~ 1 닫힘)
        self.latch = None
        self.tgt = None
        self.st = {"state": "대기", "reason": "xr_virtual.py 입력 기다림", "level": "idle"}


class Viewer:
    """가상 팔 상태 — 외부 IK 결과를 그대로 받음. 시작 자세 = 로봇 켰을 때 (robot.yaml default_arm_deg)."""

    def __init__(self, table_z):
        self.mdl = pc.Model()
        self.table_z = float(table_z)
        q = pc.default_q_ctrl()
        if q is None:
            sys.exit("[xr_view] robot.yaml default_arm_deg 없음")
        self.q_start = self.mdl.to_model(q)
        self.q = self.q_start.copy()
        self.hands = {s: Hand(s) for s in SIDES}
        self.lock = threading.Lock()
        self.last_msg, self.last_t, self.rate = 0.0, None, 0.0
        m = self.mdl.m
        self.sh_jid = {s: m.getJointId(f"{s}_shoulder_pitch_joint") for s in pc.SIDES}
        self.gap_max = pc.gap_mm(pc.Q_HI)

    def reset(self):
        with self.lock:
            self.q = self.q_start.copy()

    # ---------- 안전 검사 (표시용) ----------
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
        T = mdl.tcp(qm, side)
        pts = pc.ENVELOPE @ T.rotation.T + T.translation
        front = pts[pts[:, 0] > float(V["table_x_min_m"])]       # 테이블은 몸 앞 이 x 부터 — 옆으로 내린 팔은 검사 안 함
        low = None if len(front) == 0 else float(front[:, 2].min()) - self.table_z
        ag = self.arm_gap(qm)
        back = min(float(min(p[0] for p in self.arm_pts(qm, side)[1:])), float(pts[:, 0].min()))   # 팔꿈치 · 손목 · 그리퍼 가장 뒤
        if back < float(V["back_limit_m"]):
            why.append(f"뒤로 {back * 100:.0f} cm (한계 {float(V['back_limit_m']) * 100:.0f})")
        if g is not None and g < float(P["body_clear_m"]):
            why.append(f"몸통 {g * 100:.0f} cm (최소 {float(P['body_clear_m']) * 100:.0f})")
        if low is not None and low * 1000 < float(P["tip_clear_mm"]):
            why.append(f"테이블 {low * 1000:.0f} mm (최소 {float(P['tip_clear_mm']):.0f})")
        if ag < float(V["arm_gap_m"]):
            why.append(f"다른 팔 {ag * 100:.0f} cm (최소 {float(V['arm_gap_m']) * 100:.0f})")
        return why, {"body_gap_cm": None if g is None else round(g * 100, 1), "table_mm": None if low is None else round(low * 1000),
                     "arm_gap_cm": round(ag * 100, 1), "back_cm": round(back * 100, 1)}


    # ---------- 외부 IK (gripper/xr_virtual.py) ----------
    def arm_q_named(self):
        with self.lock:
            return {self.mdl.qname[i]: float(self.q[i]) for s in SIDES for i in self.mdl.idx[s]}

    def set_external(self, names, q, grip, src):
        """외부 IK 결과 (관절 이름 → rad) 를 가상 팔에 그대로 넣음. 안전 검사는 표시만 (실제였다면 멈췄을지)."""
        now = time.monotonic()
        with self.lock:
            if self.last_t is not None and now > self.last_t:
                self.rate = 0.9 * self.rate + 0.1 * (1.0 / max(now - self.last_t, 1e-3))
            self.last_t, self.last_msg = now, now
            byname = {n: i for i, n in self.mdl.qname.items()}
            qn = self.q.copy()
            for n, v in zip(names, q):
                i = byname.get(n)
                if i is not None:
                    qn[i] = min(max(float(v), float(self.mdl.lo[i])), float(self.mdl.hi[i]))
            self.q = qn
            for side in SIDES:
                h = self.hands[side]
                h.trig = min(max(float((grip or {}).get(side, 0.0)), 0.0), 1.0)
                why, vals = self.check(self.q, side)
                h.st = {"state": src, "reason": ("실제였으면 멈춤: " + " · ".join(why)) if why else "",
                        "level": "stop" if why else "ok"}
                h.st.update(vals)


    def state(self):
        with self.lock:
            mdl = self.mdl
            age = None if not self.last_msg else round(time.monotonic() - self.last_msg, 2)
            out = {"rate_hz": round(self.rate, 1), "age_s": age, "table_z": self.table_z, "hands": {}}
            stale = age is not None and age > float(V["timeout_s"])
            for side in SIDES:
                h = self.hands[side]
                T = mdl.tcp(self.q, side)
                joints = []
                for i in mdl.idx[side]:
                    nm = mdl.qname[i].replace(f"{side}_", "").replace("_joint", "")
                    v, lo, hi = float(self.q[i]), float(mdl.lo[i]), float(mdl.hi[i])
                    joints.append({"name": nm, "deg": round(math.degrees(v), 1), "lo": round(math.degrees(lo), 1),
                                   "hi": round(math.degrees(hi), 1), "near": bool(v - lo < NEAR_LIMIT or hi - v < NEAR_LIMIT)})
                st = dict(h.st)
                if stale:
                    st.update({"state": "입력 끊김", "reason": f"{age} 초 전 마지막", "level": "idle"})
                out["hands"][side] = {**st, "tcp": r3(T.translation), "tcp_rpy": rpy_deg(T.rotation),
                                      "gripper_mm": round(self.gap_max * (1.0 - h.trig), 1), "joints": joints}
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
            print(f"[xr_view] Dex1 URDF 없음 ({path}) — 3D 보기에서 그리퍼를 상자로만 그림", flush=True)
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
            print("[xr_view] Dex1 URDF 구조가 예상과 다름 (몸체 + 직선 손가락 2) — 상자로만 그림", flush=True)
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
            print(f"[xr_view] 어댑터 STL 없음 ({path}) — 원통으로 그림", flush=True)
            return None
        raw = open(path, "rb").read()
        n = int.from_bytes(raw[80:84], "little") if len(raw) >= 84 else 0
        if len(raw) == 84 + n * 50:
            v = np.frombuffer(raw[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))["v"].reshape(-1, 3).astype(float)
        else:
            import re
            v = np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", raw), dtype=float)
        if len(v) < 3 or len(v) % 3:
            print(f"[xr_view] 어댑터 STL 을 못 읽음 ({path}) — 원통으로 그림", flush=True)
            return None
        R = np.asarray(a["rot"], dtype=float)
        w = (v - np.asarray(a["origin_mm"], dtype=float)) @ R.T * float(a["scale"]) + np.asarray(a["at_m"], dtype=float)
        tri = w.reshape(-1, 3, 3)
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
        rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
        rec["n"], rec["v"] = nrm, tri.reshape(-1, 9)
        lo, hi = w.min(axis=0), w.max(axis=0)
        print(f"[xr_view] 어댑터 STL {len(tri)} 삼각형 — 손목 기준 x {lo[0] * 1000:.1f}~{hi[0] * 1000:.1f} mm · "
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

    @app.get("/")
    def index():
        with open(os.path.join(HERE, "xr_view.html"), encoding="utf-8") as f:
            return HTMLResponse(f.read())

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

    @app.get("/api/ext/q")
    def api_ext_q():
        return t.arm_q_named()

    @app.post("/api/ext")
    async def api_ext(req: Request):
        d = await req.json()
        t.set_external(d.get("names") or [], d.get("q") or [], d.get("grip") or {}, str(d.get("src") or "외부 IK"))
        return {"ok": True}

    @app.post("/api/reset")
    def api_reset():
        t.reset()
        return {"ok": True}

    return app


def main():
    ap = argparse.ArgumentParser(description="가상 H2 3D 뷰어 (xr_virtual.py 입력, 로봇 명령 없음)")
    ap.add_argument("--port", type=int, default=int(V["port"]))
    ap.add_argument("--table-z", type=float, default=float(V["table_z_m"]), help="테이블 높이 [m, pelvis] (-1.0 = 테이블 없음)")
    a = ap.parse_args()
    t = Viewer(a.table_z)
    print(f"[xr_view] 로봇 명령 없음 — 뷰어 http://localhost:{a.port}/  (입력: python gripper/xr_virtual.py)", flush=True)
    uvicorn.run(make_app(t), host="0.0.0.0", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
