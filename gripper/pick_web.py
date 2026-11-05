#!/usr/bin/env python3
"""
pick_web.py — 그리퍼 한 팔로 작은 박스 잡기 웹 (http://<미니 PC>:50030)

  화면: D435i 컬러 스트림 · 손(왼/오른) · 박스 크기 [mm] · 집게 자세 RPY [deg] → 박스 윗면 가운데를 클릭 → 계획 →
        [실행] → (단계별이면) [다음 단계] 를 누를 때마다 한 단계씩. [정지] 는 언제나 — 팔은 그 자리, 그리퍼는 그 상태 유지.
  잡기 순서 (준비 자세에서 시작): 그리퍼 벌림 → 접근 위치 (잡는 자리 바로 위 10 cm) → 내려가기 → 닫기
             (박스에 막혔는지 확인, 아니면 놓침 → 벌리고 멈춤) → 들기 8 cm → 2 초 → 내려놓기 → 벌림 → 빼기 → 준비 자세
  [준비 자세로] / [기본 자세로]: 선택한 팔만 관절 보간. 준비 자세 = 그리퍼가 카메라 화면 밖 (gripper.yaml ready_tcp_m),
             이 이동 중 그리퍼는 몸 앞 0.25 m 밖으로 안 나감 (계산) — 확인 창에 가장 앞 · 가장 낮은 점
  화면 점: 노란 = 클릭, 하늘 = 계획한 잡는 자리, 초록/분홍 = 지금 왼/오른 집게 가운데 (관절각 FK → 카메라로 투영)

준비 (박스 쪽 robot_server · rs_stream 은 띄우지 않음 — 이 웹이 카메라를 직접 엶)
  1) 로봇 거치대 + FSM 703:          ROBOT_CHECK=1 ./start_fsm.sh stand
  2) 한 창에서 모두:                  ./start_gripper.sh   (그리퍼 서비스 → arm_server → 이 웹, Ctrl+C 로 다 끔)
     로봇 없이:                       ./start_gripper.sh --sim
  이 웹만 따로: source activate_tv.sh && python gripper/pick_web.py enp45s0   (그리퍼 서비스 · arm_server 는 떠 있어야 함)

  끝낼 때: 박스를 잡고 있으면 먼저 내려놓기. 이 웹을 끄면 그리퍼 송신이 멈춰 힘이 풀림.
"""
import argparse
import collections
import datetime
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pick_core as pc
from pick_core import robot_env, P

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel
from typing import List, Optional

PORT = 50030
DEPTH_WIN, DEPTH_FRAMES = 11, 10     # 클릭 점 깊이: 11 × 11 픽셀 × 최근 10 프레임 (≈ 0.33 초) 중앙값
DEPTH_SPREAD_WARN = 10.0             # 깊이 흔들림이 이보다 크면 [mm] 계획에 경고
LOG_DIR = os.path.join(os.path.dirname(HERE), "logs")
CX, CY, CZ, CP = robot_env.CAMERA_X, robot_env.CAMERA_Y, robot_env.CAMERA_Z, robot_env.CAMERA_PITCH
P2T = np.array(robot_env.CFG["frames"]["pelvis_to_torso"], dtype=float)
W, H = robot_env.CAMERA_RES
_cp, _sp = math.cos(CP), math.sin(CP)
A_CT = np.array([[0.0, -_sp, _cp], [-1.0, 0.0, 0.0], [0.0, -_cp, -_sp]])   # 카메라 → torso 회전 (hand_to_marker 와 같은 식)
B_CT = np.array([CX, CY, CZ])


def cam_to_pelvis(c):
    return A_CT @ np.asarray(c, dtype=float) + B_CT + P2T


def pelvis_to_pixel(p, K):
    c = A_CT.T @ (np.asarray(p, dtype=float) - B_CT - P2T)
    if c[2] <= 0.05:
        return None
    fx, fy, ppx, ppy = K
    return int(round(fx * c[0] / c[2] + ppx)), int(round(fy * c[1] / c[2] + ppy))


# ---------------- 카메라 ----------------
class RealCamera:
    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        prof = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.K = (self.intr.fx, self.intr.fy, self.intr.ppx, self.intr.ppy)
        self.lk = threading.Lock()
        self.color, self.depth, self.t = None, None, 0.0
        self.depths = collections.deque(maxlen=DEPTH_FRAMES)   # 최근 깊이 프레임 (클릭 점 중앙값용)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                fs = self.align.process(self.pipe.wait_for_frames(2000))
                cf, df = fs.get_color_frame(), fs.get_depth_frame()
                if not cf or not df:
                    continue
                with self.lk:
                    self.color = np.asanyarray(cf.get_data()).copy()
                    self.depth = np.asanyarray(df.get_data()).copy()
                    self.depths.append(self.depth)
                    self.t = time.time()
            except Exception as e:  # noqa: BLE001
                print(f"[cam] 프레임 오류: {e}")
                time.sleep(0.5)

    def frame(self):
        with self.lk:
            return None if self.color is None else self.color.copy(), self.t

    def point(self, u, v):
        """픽셀 → (pelvis 좌표, 깊이 흔들림 mm). 깊이 = 클릭 점 주변 DEPTH_WIN × DEPTH_WIN 픽셀 × 최근 DEPTH_FRAMES 프레임의
        0 아닌 값 중앙값 (한 프레임 7×7 은 검은 박스 윗면 · 가장자리에서 흔들림). 흔들림 = 그 값들의 10–90 % 폭."""
        h = DEPTH_WIN // 2
        ui, vi = int(round(u)), int(round(v))
        with self.lk:
            frames = list(self.depths)
        if not frames:
            return None
        w = np.concatenate([f[max(0, vi - h):vi + h + 1, max(0, ui - h):ui + h + 1].ravel() for f in frames]).astype(float)
        w = w[w > 0]
        if len(w) < 20:
            return None
        spread = float(np.percentile(w, 90) - np.percentile(w, 10)) * self.scale * 1000.0
        c = self.rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], float(np.median(w)) * self.scale)
        return cam_to_pelvis(c), spread


class FakeCamera:
    """로봇 없이 화면·계획 시험용 — 회색 화면, 클릭은 높이 top_z 인 수평면과 만나는 점."""
    def __init__(self, top_z):
        self.K = tuple(float(v) for v in robot_env.CAMERA_K)
        self.top_z = top_z
        img = np.full((H, W, 3), 90, np.uint8)
        for gx in np.arange(0.25, 0.85, 0.05):
            for gy in np.arange(-0.45, 0.46, 0.05):
                px = pelvis_to_pixel([gx, gy, top_z], self.K)
                if px and 0 <= px[0] < W and 0 <= px[1] < H:
                    cv2.circle(img, px, 1, (160, 160, 160), -1)
        cv2.putText(img, f"SIM camera: click = plane z {top_z:+.2f} m (pelvis), grid 5 cm", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        self.img = img

    def frame(self):
        return self.img.copy(), time.time()

    def point(self, u, v):
        fx, fy, ppx, ppy = self.K
        d = np.array([(u - ppx) / fx, (v - ppy) / fy, 1.0])
        a = A_CT @ d
        if abs(a[2]) < 1e-6:
            return None
        t = (self.top_z - B_CT[2] - P2T[2]) / a[2]
        return None if t <= 0 else (cam_to_pelvis(t * d), 0.0)


# ---------------- 그리퍼 ----------------
class RealGripper:
    def __init__(self):
        from dex1_gripper import Dex1Gripper
        self.g = Dex1Gripper(pc.CFG).start()

    def set(self, side, q):
        return not self.g.set(**{side: float(q)})

    def state(self, side):
        s = self.g.snapshot()[side]
        return s if s.get("ok") else None

    def snapshot(self):
        return self.g.snapshot()


class FakeGripper:
    def __init__(self):
        self.q = {"left": 0.3, "right": 0.3}
        self.goal = dict(self.q)
        self.block_mm = None           # 닫을 때 이 간격에서 막힘 (가짜 박스)
        self.t = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            time.sleep(0.01)
            for s in self.q:
                d = self.goal[s] - self.q[s]
                nq = self.q[s] + max(-0.03, min(0.03, d))
                if self.block_mm is not None and d < 0 and pc.gap_mm(nq) < self.block_mm:
                    nq = max(nq, self.block_mm / pc.MM_PER_RAD)
                self.q[s] = nq

    def set(self, side, q):
        self.goal[side] = float(q)
        return True

    def state(self, side):
        return {"q": round(self.q[side], 4), "dq": 0.0, "ok": True, "goal": self.goal[side]}

    def snapshot(self):
        return {s: {"q": round(self.q[s], 4), "goal": round(self.goal[s], 4), "ok": True} for s in self.q}


# ---------------- 실행기 ----------------
class Runner:
    def __init__(self, mdl, cam, grip, sim):
        self.mdl, self.cam, self.grip, self.sim = mdl, cam, grip, sim
        self.lk = threading.Lock()
        self.plan, self.plan_id, self.click, self.bin_click = None, 0, None, None
        self.phase, self.stage, self.waiting = "idle", "", False
        self.log = []
        self.stop_evt, self.next_evt = threading.Event(), threading.Event()
        self.th = None
        self.ctx = None                # {"side", "floor_z"} — 복귀에 씀
        self.arm_q = None              # 지금 관절 (제어 순서, rad) — 표시용
        self.arm_msg = ""
        threading.Thread(target=self._arm_monitor, daemon=True).start()

    def _arm_monitor(self):
        while True:
            try:
                pose = pc.call("/pose", timeout=1.0)
                self.arm_q = np.radians(np.asarray(pose["arm_deg"], dtype=float))
                self.arm_msg = "ok"
            except Exception as e:  # noqa: BLE001
                self.arm_q, self.arm_msg = None, f"arm_server 응답 없음 ({type(e).__name__})"
            time.sleep(0.2)

    def say(self, msg):
        line = f"{datetime.datetime.now().strftime('%H:%M:%S')} {msg}"
        print(f"[pick] {msg}")
        with self.lk:
            self.log = (self.log + [line])[-60:]
        self._jsonl({"event": msg})

    def _jsonl(self, rec):
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            rec = dict(rec, t=time.time(), plan_id=self.plan_id)
            with open(os.path.join(LOG_DIR, f"pick_{datetime.date.today():%Y%m%d}.jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def busy(self):
        return self.th is not None and self.th.is_alive()

    # ----- 계획 -----
    def make_plan(self, u, v, side, width_mm, height_mm, rpy, bin_uv=None, bin_h_mm=None):
        if self.busy():
            raise HTTPException(409, "실행 중 — 정지 후 다시")
        ok, msg, q_now = pc.arm_ready_check()
        if not ok:
            raise HTTPException(409, msg)
        got = self.cam.point(u, v)
        if got is None:
            raise HTTPException(400, "그 자리 깊이 없음 — 박스 윗면 가운데를 다시 클릭")
        top, spread = got
        bin_pt = None
        if bin_uv is not None:
            gb = self.cam.point(*bin_uv)
            if gb is None:
                raise HTTPException(400, "통 자리 깊이 없음 — 통 안쪽 가운데를 다시 클릭")
            bin_pt = gb[0]
        plan = pc.plan_pick(self.mdl, q_now, side, top, width_mm, height_mm, rpy, bin_pt=bin_pt, bin_h_mm=bin_h_mm)
        plan["depth_spread_mm"] = round(spread, 1)
        if spread > DEPTH_SPREAD_WARN:
            plan.setdefault("notes", []).append(f"클릭 점 깊이 흔들림 {spread:.0f} mm — 윗면 가장자리이거나 깊이가 불안정 (가운데를 다시 클릭)")
        with self.lk:
            self.plan_id += 1
            self.plan = plan
            self.click = (u, v)
            self.bin_click = bin_uv
            self.phase = "planned" if plan["ok"] else "idle"
        self._jsonl({"plan": self.summary(plan), "click_px": [u, v], "bin_px": bin_uv})
        self.say(f"계획 #{self.plan_id} {side} 폭 {width_mm:.0f} · 높이 {height_mm:.0f} mm RPY {rpy}"
                 + (f" → 통 (높이 {bin_h_mm:.0f} mm)" if bin_uv is not None else " → 제자리") + ": " + ("OK" if plan["ok"] else "불가 — " + "; ".join(plan["reasons"])))
        return self.summary(plan)

    @staticmethod
    def summary(plan):
        if plan is None:
            return None
        keys = ("ok", "side", "top", "depth_spread_mm", "width_mm", "height_mm", "rpy_deg", "floor_z", "overlap_mm", "grasp", "pre", "open_mm", "close_mm",
                "q_open", "q_close", "reasons", "notes", "total_s", "steps", "at_ready", "bin", "rim_z", "release")
        return {k: plan.get(k) for k in keys if k in plan}

    # ----- 실행 -----
    def start(self, plan_id, step_mode):
        with self.lk:
            if self.busy():
                raise HTTPException(409, "이미 실행 중")
            if self.plan is None or not self.plan.get("ok") or plan_id != self.plan_id:
                raise HTTPException(409, "실행할 계획 없음 (또는 화면의 계획이 오래됨) — 다시 클릭")
            plan = self.plan
            self.plan = dict(plan, ok=False)      # 같은 계획을 두 번 실행하지 않음
        ok, msg, q_now = pc.arm_ready_check()
        if not ok:
            raise HTTPException(409, msg)
        if np.max(np.abs(self.mdl.to_model(q_now) - plan["q_start"])) > math.radians(3.0):
            raise HTTPException(409, "계획 뒤에 팔이 움직임 — 다시 클릭해 계획")
        self.stop_evt.clear()
        self.next_evt.clear()
        self.ctx = {"side": plan["side"], "floor_z": plan["floor_z"]}
        self.th = threading.Thread(target=self._run, args=(plan, step_mode), daemon=True)
        self.th.start()

    def _wait_next(self, name):
        with self.lk:
            self.stage, self.waiting = f"다음: {name}", True
        while not self.next_evt.wait(0.1):
            if self.stop_evt.is_set():
                return False
        self.next_evt.clear()
        with self.lk:
            self.waiting = False
        return True

    def _grip_to(self, side, q, timeout=4.0, settle=0.4):
        if not self.grip.set(side, q):
            return None, "그리퍼 상태 없음 — dex1_1_service 확인"
        t0, t_still, last = time.time(), None, None
        while time.time() - t0 < timeout:
            if self.stop_evt.is_set():
                return None, "정지"
            st = self.grip.state(side)
            if st is None:
                return None, "그리퍼 상태 끊김"
            qn = st["q"]
            if last is not None and abs(qn - last) < 0.005 and time.time() - t0 > 0.5:
                t_still = t_still or time.time()
                if time.time() - t_still >= settle:
                    break
            else:
                t_still = None
            last = qn
            time.sleep(0.05)
        st = self.grip.state(side)
        return (None, "그리퍼 상태 끊김") if st is None else (st["q"], None)

    def _run(self, plan, step_mode):
        side = plan["side"]
        with self.lk:
            self.phase = "running"
        self.say(f"실행 #{self.plan_id} ({'단계별' if step_mode else '연속'}) — 약 {plan['total_s']} 초")
        result = "done"
        try:
            for kind, sg in plan["seq"]:
                name = sg.name if kind == "arm" else {"@open": "그리퍼 벌림", "@close": "그리퍼 닫기",
                                                     "@hold": f"{P['hold_s']} 초 들고 있기"}[kind]
                if step_mode and not self._wait_next(name):
                    result = "stopped"
                    break
                with self.lk:
                    self.stage = name
                if kind == "arm":
                    ok, msg = pc.run_seg(self.mdl, sg, self.stop_evt)
                    self.say(f"{name}: {msg}" + (self._arrival(side, sg) if ok else ""))
                    if not ok:
                        result = "stopped"
                        break
                elif kind == "@open":
                    q, err = self._grip_to(side, plan["q_open"])
                    if err:
                        self.say(f"{name}: {err}")
                        result = "stopped"
                        break
                    self.say(f"{name}: q {q:.2f} (≈ {pc.gap_mm(q):.0f} mm)")
                elif kind == "@close":
                    if self.sim and isinstance(self.grip, FakeGripper):
                        self.grip.block_mm = plan["width_mm"] - 1.0
                    q, err = self._grip_to(side, plan["q_close"])
                    if err:
                        self.say(f"{name}: {err}")
                        result = "stopped"
                        break
                    gap = pc.gap_mm(q)
                    held = gap >= plan["close_mm"] + P["contact_min_mm"]
                    self.say(f"{name}: q {q:.2f} (≈ {gap:.0f} mm, 목표 {plan['close_mm']:.0f} mm) → " +
                             ("박스에 막힘 = 잡음" if held else "목표까지 닫힘 = 놓친 것으로 봄"))
                    if not held:
                        self._grip_to(side, plan["q_open"])
                        result = "missed"
                        break
                elif kind == "@hold":
                    t_end = time.time() + float(P["hold_s"])
                    while time.time() < t_end and not self.stop_evt.is_set():
                        time.sleep(0.05)
                    if self.stop_evt.is_set():
                        result = "stopped"
                        break
                    st = self.grip.state(side)
                    self.say(f"{name}: 끝 (그리퍼 q {st['q']:.2f} ≈ {pc.gap_mm(st['q']):.0f} mm)" if st else f"{name}: 끝")
        except Exception as e:  # noqa: BLE001
            self.say(f"오류: {type(e).__name__}: {e} — 팔은 마지막 목표에서 멈춤")
            result = "error"
        if self.sim and isinstance(self.grip, FakeGripper):
            self.grip.block_mm = None
        with self.lk:
            self.phase = {"done": "done", "missed": "stopped"}.get(result, result)
            self.stage, self.waiting = ("끝 (준비 자세)" if result == "done" else "멈춤 — [복귀] 로 위로 → 준비 자세"), False
        self.say({"done": "잡기 순서 끝", "missed": "놓침 — 그리퍼를 벌림. [복귀] 를 누르세요",
                  "stopped": "멈춤 (팔은 그 자리, 그리퍼는 그 상태 유지)", "error": "오류로 멈춤"}[result])
        self._jsonl({"result": result})

    def _arrival(self, side, sg):
        """도착 0.4 초 뒤: 보낸 관절(경로 끝) 의 집게 가운데 vs 실측 관절의 집게 가운데 [mm] — 팔 추종 오차."""
        try:
            time.sleep(0.4)
            pose = pc.call("/pose", timeout=1.0)
            qm = self.mdl.to_model(np.radians(np.asarray(pose["arm_deg"], dtype=float)))
            want = self.mdl.tcp(sg.qs[-1], side).translation
            got = self.mdl.tcp(qm, side).translation
            e = (got - want) * 1000
            jd = float(np.degrees(np.max(np.abs(qm[self.mdl.idx[side]] - sg.qs[-1][self.mdl.idx[side]]))))
            self._jsonl({"arrival": sg.name, "side": side, "err_mm": np.round(e, 1).tolist(), "joint_max_deg": round(jd, 2)})
            return f" — 도착 오차 (실측 − 보냄) 앞뒤 {e[0]:+.0f} · 좌우 {e[1]:+.0f} · 높이 {e[2]:+.0f} mm, 관절 최대 {jd:.1f}°"
        except Exception as ex:  # noqa: BLE001
            return f" — 도착 오차 못 읽음 ({type(ex).__name__})"

    def home(self):
        if self.busy():
            raise HTTPException(409, "실행 중 — 먼저 정지")
        if not self.ctx:
            raise HTTPException(409, "복귀할 실행 기록 없음")
        ok, msg, q_now = pc.arm_ready_check()
        if not ok:
            raise HTTPException(409, msg)
        seq, why = pc.plan_home(self.mdl, q_now, self.ctx["side"], self.ctx["floor_z"])
        if seq is None:
            raise HTTPException(409, f"복귀 계획 불가: {why}")
        self.stop_evt.clear()

        def _go():
            with self.lk:
                self.phase = "running"
            res = "done"
            for _, sg in seq:
                with self.lk:
                    self.stage = sg.name
                ok2, m2 = pc.run_seg(self.mdl, sg, self.stop_evt)
                self.say(f"복귀 {sg.name}: {m2}")
                if not ok2:
                    res = "stopped"
                    break
            with self.lk:
                self.phase, self.stage = res, ("복귀 끝" if res == "done" else "복귀 중 멈춤")
            if res == "done":
                self.ctx = None
        self.th = threading.Thread(target=_go, daemon=True)
        self.th.start()

    def pose(self, side, which, confirm):
        """준비 자세 / 기본 자세 (그 팔만). confirm False 면 계산만 해서 확인 창 정보를 돌려줌."""
        if self.busy():
            raise HTTPException(409, "실행 중")
        ok, msg, q_now = pc.arm_ready_check()
        if not ok:
            raise HTTPException(409, msg)
        segs, info = pc.plan_pose(self.mdl, q_now, side, which)
        if segs is None:
            raise HTTPException(409, info)
        if not confirm:
            return info
        self.stop_evt.clear()
        with self.lk:
            self.plan = None             # 자세가 바뀌므로 이전 계획은 버림

        def _go():
            ok2 = True
            for sg in segs:
                with self.lk:
                    self.phase, self.stage = "running", sg.name
                ok2, m2 = pc.run_seg(self.mdl, sg, self.stop_evt)
                self.say(f"{side} {sg.name}: {m2}")
                if not ok2:
                    break
            with self.lk:
                self.phase, self.stage = ("done" if ok2 else "stopped"), (segs[-1].name + " 끝" if ok2 else "자세 이동 중 멈춤")
        self.th = threading.Thread(target=_go, daemon=True)
        self.th.start()
        return info

    def gripper_manual(self, side, action, size_mm):
        if self.busy():
            raise HTTPException(409, "실행 중")
        q = {"open": pc.q_for_gap(size_mm + P["open_margin_mm"]), "close": pc.q_for_gap(10.0)}[action]
        if not self.grip.set(side, q):
            raise HTTPException(409, "그리퍼 상태 없음 — dex1_1_service 확인")
        self.say(f"그리퍼 {side} → q {q:.2f} (≈ {pc.gap_mm(q):.0f} mm)")

    # ----- 화면 -----
    def overlay(self, img):
        K = self.cam.K
        with self.lk:
            plan, click, bclick = self.plan, self.click, self.bin_click
        if click:
            cv2.drawMarker(img, (int(click[0]), int(click[1])), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        if bclick:
            cv2.drawMarker(img, (int(bclick[0]), int(bclick[1])), (0, 140, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
        if plan and plan.get("grasp"):
            for key, col in (("grasp", (255, 200, 0)), ("pre", (255, 255, 120)), ("release", (0, 140, 255))):
                if not plan.get(key):
                    continue
                px = pelvis_to_pixel(plan[key], K)
                if px:
                    cv2.circle(img, px, 6, col, 2)
        q = self.arm_q
        if q is not None:
            qm = self.mdl.to_model(q)
            for s, col in (("left", (0, 220, 0)), ("right", (220, 0, 220))):
                px = pelvis_to_pixel(self.mdl.tcp(qm, s).translation + pc.TCP_BIAS[s], K)   # 치우침 보정한 실제 추정
                if px:
                    cv2.circle(img, px, 5, col, -1)
        return img


# ---------------- 웹 ----------------
class PlanReq(BaseModel):
    u: float
    v: float
    side: str
    width_mm: float
    height_mm: float
    rpy_deg: List[float]
    bin_u: Optional[float] = None       # 통 안쪽 가운데 클릭 (없으면 제자리 내려놓기)
    bin_v: Optional[float] = None
    bin_h_mm: Optional[float] = None    # 테이블 → 통 테두리 높이


class RunReq(BaseModel):
    plan_id: int
    step_mode: bool = True


class PoseReq(BaseModel):
    side: str
    which: str
    confirm: bool = False


class GripReq(BaseModel):
    side: str
    action: str
    size_mm: float = 50.0


def make_app(r: Runner):
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    def index():
        return PAGE.replace("__W__", str(W)).replace("__H__", str(H)).replace(
            "__RPY__", json.dumps(P["rpy_deg"])).replace("__SIM__", "true" if r.sim else "false")

    @app.get("/video")
    def video():
        def gen():
            last = 0.0
            while True:
                img, t = r.cam.frame()
                if img is None or t == last and not r.sim:
                    time.sleep(0.02)
                    continue
                last = t
                ok, jpg = cv2.imencode(".jpg", r.overlay(img), [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
                time.sleep(1 / 15)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/api/plan")
    def api_plan(q: PlanReq):
        if q.side not in pc.SIDES or len(q.rpy_deg) != 3 or not (10 <= q.width_mm <= 75) or not (15 <= q.height_mm <= 150):
            raise HTTPException(400, "side left/right, rpy 3 개, 폭 10–75 mm, 높이 15–150 mm")
        bin_uv = None
        if q.bin_u is not None:
            if q.bin_v is None or q.bin_h_mm is None or not (10 <= q.bin_h_mm <= 250):
                raise HTTPException(400, "통: 클릭 점 · 높이 10–250 mm")
            bin_uv = (q.bin_u, q.bin_v)
        plan = r.make_plan(q.u, q.v, q.side, q.width_mm, q.height_mm, q.rpy_deg, bin_uv, q.bin_h_mm)
        return {"plan_id": r.plan_id, "plan": plan}

    @app.post("/api/run")
    def api_run(q: RunReq):
        r.start(q.plan_id, q.step_mode)
        return {"ok": True}

    @app.post("/api/next")
    def api_next():
        r.next_evt.set()
        return {"ok": True}

    @app.post("/api/stop")
    def api_stop():
        r.stop_evt.set()
        r.say("정지 요청")
        return {"ok": True}

    @app.post("/api/home")
    def api_home():
        r.home()
        return {"ok": True}

    @app.post("/api/pose")
    def api_pose(q: PoseReq):
        if q.side not in pc.SIDES or q.which not in ("ready", "default"):
            raise HTTPException(400, "side left/right, which ready/default")
        return r.pose(q.side, q.which, q.confirm)

    @app.post("/api/gripper")
    def api_gripper(q: GripReq):
        if q.side not in pc.SIDES or q.action not in ("open", "close"):
            raise HTTPException(400, "side left/right, action open/close")
        r.gripper_manual(q.side, q.action, q.size_mm)
        return {"ok": True}

    @app.get("/api/state")
    def api_state():
        with r.lk:
            st = {"phase": r.phase, "stage": r.stage, "waiting": r.waiting, "log": r.log[-25:],
                  "plan_id": r.plan_id, "plan": r.summary(r.plan), "can_home": bool(r.ctx) and not r.busy(),
                  "busy": r.busy()}
        g = r.grip.snapshot()
        st["gripper"] = {}
        for s in pc.SIDES:
            d = g.get(s) or {}
            st["gripper"][s] = None if d.get("q") is None else {"q": d["q"], "mm": round(pc.gap_mm(d["q"]), 1), "ok": d.get("ok")}
        st["arm"] = r.arm_msg
        if r.arm_q is not None:
            qm = r.mdl.to_model(r.arm_q)
            st["tcp"] = {s: np.round(r.mdl.tcp(qm, s).translation + pc.TCP_BIAS[s], 3).tolist() for s in pc.SIDES}
        return st

    return app


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>그리퍼 잡기</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#ddd}
.wrap{display:flex;flex-wrap:wrap;gap:12px;padding:12px}
#img{max-width:100%;cursor:crosshair;border:1px solid #444;display:block}
.panel{min-width:320px;max-width:440px;flex:1}
fieldset{border:1px solid #444;margin:0 0 10px;padding:8px}
label{margin-right:10px}input[type=number]{width:70px;background:#222;color:#eee;border:1px solid #555}
button{padding:8px 12px;margin:3px 2px;background:#2a2a2a;color:#eee;border:1px solid #666;border-radius:4px;cursor:pointer}
button:disabled{opacity:.35;cursor:default}#stop{background:#a00;border-color:#f44;font-weight:bold;width:100%;font-size:18px}
#next{background:#064;border-color:#3c8}pre{white-space:pre-wrap;font-size:12px;background:#1a1a1a;padding:6px;max-height:300px;overflow:auto}
.bad{color:#f88}.ok{color:#8f8}.sim{background:#553;padding:4px 8px}
</style></head><body>
<div class="wrap"><div><img id="img" src="/video" width="__W__" height="__H__"><div id="simnote"></div>
<div style="font-size:12px;margin-top:4px">노란 + 클릭 · 하늘 ○ 계획한 잡는 자리(진한)/접근 위치 · 초록 ● 왼 집게 · 분홍 ● 오른 집게 (FK 투영)</div></div>
<div class="panel">
<button id="stop">■ 정지</button>
<fieldset><legend>박스 · 손 · 자세</legend>
<label><input type="radio" name="side" value="left">왼손</label><label><input type="radio" name="side" value="right" checked>오른손</label><br>
잡는 폭 <input id="size" type="number" value="50" min="10" max="75"> mm (로봇 좌우 방향) · 높이 <input id="height" type="number" value="50" min="15" max="150"> mm<br>
놓는 곳 <label><input type="radio" name="dest" value="bin" checked>통</label><label><input type="radio" name="dest" value="same">제자리</label>
· 통 높이 <input id="binh" type="number" value="80" min="10" max="250"> mm (테이블 → 테두리)<br>
RPY <input id="r" type="number" step="5"><input id="p" type="number" step="5"><input id="y" type="number" step="5"> deg
<div id="clickhint" style="font-size:13px;color:#ff0">① 박스 윗면 가운데를 클릭</div>
<div style="font-size:12px">0,0,0 = 손이 앞 · 집게 좌우, pitch + = 앞으로 숙임 (손목 pitch ±33° 라 90 은 거의 안 닿음). 박스 윗면 가운데를 클릭하면 계획 (접근은 위에서 수직)</div>
</fieldset>
<fieldset><legend>실행</legend>
<label><input type="checkbox" id="step" checked>단계별 (매 단계 [다음 단계])</label><br>
<button id="run" disabled>실행</button><button id="next" disabled>다음 단계 ▶</button><button id="home" disabled>복귀 (위로→준비)</button><br>
선택한 팔: <button id="pready">준비 자세로</button><button id="pdef">기본 자세로</button><br>
그리퍼 (선택한 손): <button id="gopen">벌림</button><button id="gclose">닫힘</button>
</fieldset>
<div id="status"></div><pre id="plan"></pre><pre id="log"></pre></div></div>
<script>
const RPY=__RPY__, SIM=__SIM__; let planId=0, planOk=false;
['r','p','y'].forEach((k,i)=>document.getElementById(k).value=RPY[i]);
if(SIM) document.getElementById('simnote').innerHTML='<span class="sim">SIM — 가짜 카메라 · 가짜 그리퍼</span>';
const $=id=>document.getElementById(id);
const side=()=>document.querySelector('input[name=side]:checked').value;
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});
 const j=await r.json().catch(()=>({})); if(!r.ok) throw new Error(j.detail||r.status); return j;}
let boxUV=null;
const dest=()=>document.querySelector('input[name=dest]:checked').value;
const hint=()=>{$('clickhint').textContent=boxUV?'② 통 안쪽 가운데를 클릭 (다시 하려면 놓는 곳을 바꿨다 되돌리기)':'① 박스 윗면 가운데를 클릭';};
document.querySelectorAll('input[name=dest]').forEach(x=>x.onchange=()=>{boxUV=null;hint();});
$('img').onclick=async e=>{const im=$('img'),rc=im.getBoundingClientRect();
 const u=(e.clientX-rc.left)*__W__/rc.width, v=(e.clientY-rc.top)*__H__/rc.height;
 let body={u,v,side:side(),width_mm:+$('size').value,height_mm:+$('height').value,rpy_deg:[+$('r').value,+$('p').value,+$('y').value]};
 if(dest()==='bin'){ if(!boxUV){boxUV=[u,v];hint();$('plan').textContent='박스 클릭 저장 — 이제 통 안쪽 가운데를 클릭';return;}
   body.u=boxUV[0];body.v=boxUV[1];body.bin_u=u;body.bin_v=v;body.bin_h_mm=+$('binh').value;boxUV=null;hint();}
 $('plan').textContent='계획 중…';
 try{const j=await post('/api/plan',body);
  planId=j.plan_id; const p=j.plan; planOk=p.ok; showPlan(p);}catch(err){planOk=false;$('plan').innerHTML='<span class="bad">'+err.message+'</span>';}};
function f3(a){return a?a.map(x=>x.toFixed(3)).join(', '):'-'}
function showPlan(p){let s=(p.ok?'<span class="ok">계획 OK</span>':'<span class="bad">계획 불가</span>')+` #${planId} ${p.side} 폭 ${p.width_mm} · 높이 ${p.height_mm} mm RPY ${p.rpy_deg}\n`;
 s+=`윗면 가운데 (pelvis) ${f3(p.top)} · 깊이 흔들림 ${p.depth_spread_mm} mm\n잡는 자리 ${f3(p.grasp)} · 접근 ${f3(p.pre)}\n바닥 z ${p.floor_z} · 패드↔박스 겹침 ${p.overlap_mm} mm\n`;
 if(p.bin) s+=`통 (클릭) ${f3(p.bin)} · 테두리 z ${p.rim_z} · 놓는 자리 ${f3(p.release)}\n`;
 s+=`그리퍼 벌림 ${p.open_mm} mm (q ${p.q_open}) · 닫기 목표 ${p.close_mm} mm (q ${p.q_close})\n`;
 (p.reasons||[]).forEach(x=>s+='<span class="bad">✗ '+x+'</span>\n'); (p.notes||[]).forEach(x=>s+='⚠ '+x+'\n');
 if(p.steps) s+='순서: '+p.steps.map(x=>x[1]?`${x[0]}(${x[1]}s)`:x[0]).join(' → ')+`\n합계 약 ${p.total_s} 초`;
 $('plan').innerHTML=s;}
$('run').onclick=async()=>{if(!confirm('팔이 움직입니다. 비상정지를 손에 쥐었습니까?'))return;
 try{await post('/api/run',{plan_id:planId,step_mode:$('step').checked});planOk=false;}catch(err){alert(err.message);}};
$('next').onclick=()=>post('/api/next').catch(e=>alert(e.message));
$('stop').onclick=()=>post('/api/stop').catch(e=>alert(e.message));
$('home').onclick=async()=>{if(!confirm('복귀: 위로 → 준비 자세. 진행할까요?'))return; post('/api/home').catch(e=>alert(e.message));};
async function poseGo(which){try{const i=await post('/api/pose',{side:side(),which,confirm:false});
 const low=i.low_front_z===null?'없음':`${(i.low_front_z*100).toFixed(0)} cm (pelvis 기준)`;
 if(!confirm(`${side()==='left'?'왼':'오른'}팔 ${which==='ready'?'준비 자세':'기본 자세'}로 (${i.sec} 초, 관절 보간)\n`+
   `몸 앞 25 cm 넘는 곳에서 그리퍼 가장 낮은 점: ${low}\n가장 앞: ${(i.max_x*100).toFixed(0)} cm${i.via?' (경유 자세를 거침)':''}\n`+
   `테이블·박스가 이 경로에 없고, 비상정지를 손에 쥐었습니까?`))return;
 await post('/api/pose',{side:side(),which,confirm:true});}catch(e){alert(e.message);}}
$('pready').onclick=()=>poseGo('ready'); $('pdef').onclick=()=>poseGo('default');
$('gopen').onclick=()=>post('/api/gripper',{side:side(),action:'open',size_mm:+$('size').value}).catch(e=>alert(e.message));
$('gclose').onclick=()=>{if(confirm('집게를 닫습니다 (≈10 mm). 손가락을 빼세요.'))post('/api/gripper',{side:side(),action:'close'}).catch(e=>alert(e.message));};
async function poll(){try{const s=await (await fetch('/api/state')).json();
 const g=s.gripper||{}; const gs=k=>g[k]?`${g[k].q.toFixed(2)} rad ≈ ${g[k].mm} mm`:'상태 없음';
 $('status').innerHTML=`<b>${s.phase}</b> ${s.stage||''}<br>팔: ${s.arm}${s.tcp?` · 집게 가운데 L ${f3(s.tcp.left)} / R ${f3(s.tcp.right)}`:''}<br>그리퍼 L ${gs('left')} · R ${gs('right')}`;
 $('log').textContent=(s.log||[]).join('\n'); $('log').scrollTop=1e9;
 $('run').disabled=!(planOk&&s.plan_id===planId&&!s.busy); $('next').disabled=!s.waiting; $('home').disabled=!s.can_home;
 ['gopen','gclose','pready','pdef'].forEach(k=>$(k).disabled=s.busy);}catch(e){} setTimeout(poll,300);}
poll();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("iface", nargs="?", default=None, help="DDS 인터페이스 (그리퍼 토픽, 예: enp45s0)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--sim", action="store_true", help="가짜 카메라 · 가짜 그리퍼 (팔은 arm_server 를 그대로 씀)")
    ap.add_argument("--sim-top-z", type=float, default=0.0, help="--sim 클릭 평면 높이 [m, pelvis]")
    a = ap.parse_args()
    mdl = pc.Model()
    if a.sim:
        cam, grip = FakeCamera(a.sim_top_z), FakeGripper()
    else:
        robot_env.dds_init(a.iface)
        grip = RealGripper()
        cam = RealCamera()
    r = Runner(mdl, cam, grip, a.sim)
    print(f"[pick] http://0.0.0.0:{a.port}/  ({'SIM' if a.sim else 'real'}) — TCP {pc.TCP_OFF.tolist()} m, "
          f"집게 {pc.MM_PER_RAD} mm/rad (계산값)")
    uvicorn.run(make_app(r), host="0.0.0.0", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
