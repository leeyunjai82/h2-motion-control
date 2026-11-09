"""
xr_real.py — VR 조작 (xr_teleoperate H2 IK) → 실제 팔 · 그리퍼.  잡기 웹 (pick_web.py) 안에서 돔

  흐름: Quest → gripper/xr_virtual.py --real (televuer + H2_ArmIK, 30 Hz) → 잡기 웹 POST /api/xr/cmd
        → 여기서 안전 검사 → arm_server /stream (팔) · Dex1 (그리퍼). 녹화는 잡기 웹 녹화기 그대로 (state/action 16 + 카메라).

  시작은 사람이 잡기 웹 [VR 시작] 을 눌러야 함 (xr_virtual 은 세션이 열릴 때만 보냄).

  안전 (겹겹이):
   - 시작 조건: 실기 팔 (카메라만 · 다른 동작 중이면 거부) · arm_server hold · 테이블을 잼 · 카메라 숙임 · 485 보드 · 앞 에피소드 결정
   - 팔 속도: arm_server 스트림 max_vel (기본 gripper.yaml xr_view.real_max_vel 0.3 rad/s, 상한 1.0)
   - 데드맨: 그립(중지)을 누른 손만 팔이 움직임. 그리퍼는 트리거만 따름 (당기면 닫힘 · 놓으면 벌림)
   - 손별 검사: 몸통 · 테이블 (잰 평면) · 다른 팔 · 뒤 한계 → 걸리면 그 손은 이번 명령을 버리고 그 자리
   - 한 번에 크게 다른 목표 (관절 0.5 rad 넘게 — IK 가 튐) 는 버림
   - 명령 · 실측 차이 > pick.track_abort_deg → 세션 끝 (걸리거나 부딪힘)
   - 값이 안 오면: arm_server 가 0.2 초 뒤 그 자리 멈춤, 1.5 초 넘으면 여기서 세션 끝
   - [정지] · [VR 끝] · 오류 → 세션 끝 (팔은 마지막 목표에서 멈춤, 0 자세로 가지 않음)
"""
import math
import threading
import time
import urllib.error

import numpy as np
import pinocchio as pin
from fastapi import HTTPException

import pick_core as pc
from pick_core import P

X = pc.CFG.get("xr_view") or {}
_need = ("arm_gap_m", "back_limit_m", "table_x_min_m", "real_max_vel")
_miss = [k for k in _need if k not in X]
if _miss:
    raise SystemExit(f"[xr] gripper.yaml xr_view: 에 {_miss} 없음 — 기본값으로 대신하지 않음")
MAX_VEL_CAP = 1.0           # VR 실기 스트림 max_vel 상한 [rad/s]
JUMP_RAD = 0.5              # 한 명령에서 관절 목표가 이보다 크게 바뀌면 버림 (IK 튐)
IDLE_END_S = 1.5            # 이 시간 넘게 명령이 없으면 세션 끝
SQUEEZE_Q = float(P["squeeze_mm"]) / pc.MM_PER_RAD   # 그리퍼를 지금 손가락 위치보다 이만큼만 더 조임 (≈ 8 mm) — 물건에 막혀도
                                                      #   완전 닫힘까지 억지로 조이지 않음 (계속 힘이 걸리면 과열). 자동 잡기의 squeeze_mm 와 같음


class XRTeleop:
    def __init__(self, runner):
        self.r = runner
        self.mdl = pc.Model()            # 따로 만든 모델 — 녹화 · 계획 스레드와 pinocchio Data 를 같이 쓰지 않음 (검사 값이 섞이지 않게)
        self.lk = threading.Lock()
        self.active = False
        self.tok = None
        self.epoch = 0
        self.q = None                  # 마지막으로 arm_server 가 받은 목표 (제어 순서, rad)
        self.q0_named = {}
        self.max_vel = float(X["real_max_vel"])
        self.t_last = 0.0
        self.n = 0
        self.msg = "꺼짐"
        self.hands = {s: {"state": "대기", "reason": "", "level": "idle"} for s in pc.SIDES}
        self.grip_last = {s: None for s in pc.SIDES}
        m = self.mdl.m
        self.sh_jid = {s: m.getJointId(f"{s}_shoulder_pitch_joint") for s in pc.SIDES}
        self.byname = {n: i for i, n in self.mdl.qname.items()}
        threading.Thread(target=self._watchdog, daemon=True).start()

    # ---------- 안전 검사 ----------
    def _arm_pts(self, qm, side):
        mdl = self.mdl
        pin.forwardKinematics(mdl.m, mdl.d, qm)
        pts = [mdl.d.oMi[mdl.jid[side][n]].translation.copy() for n in ("elbow", "wrist_roll", "wrist_yaw")]
        pts.append(mdl.tcp(qm, side).translation.copy())
        return pts

    def check(self, qm, side):
        """그 손이 이 자세로 가도 되는지 → 이유 목록 (비면 OK)."""
        mdl, why = self.mdl, []
        g = mdl.body_gap(qm, side)
        if g is not None and g < float(P["body_clear_m"]):
            why.append(f"몸통 {g * 100:.0f} cm")
        T = mdl.tcp(qm, side)
        env = pc.ENVELOPE @ T.rotation.T + T.translation
        plane = None if getattr(self, "no_table", False) else self.r.table_plane
        front = env[env[:, 0] > float(X["table_x_min_m"])]
        if plane is not None and len(front):
            zt = plane[0] + plane[1] * front[:, 0] + plane[2] * front[:, 1]
            low = float(np.min(front[:, 2] - zt))
            if low * 1000 < float(P["tip_clear_mm"]):
                why.append(f"테이블 {low * 1000:.0f} mm")
        a = self._arm_pts(qm, "left")
        b = self._arm_pts(qm, "right")
        ag = min(float(np.linalg.norm(x - y)) for x in a for y in b)
        if ag < float(X["arm_gap_m"]):
            why.append(f"다른 팔 {ag * 100:.0f} cm")
        back = min(min(float(p[0]) for p in self._arm_pts(qm, side)), float(env[:, 0].min()))
        if back < float(X["back_limit_m"]):
            why.append(f"뒤로 {back * 100:.0f} cm")
        return why

    # ---------- 세션 ----------
    def start(self, max_vel, cam_pitch_max, no_table=False):
        r = self.r
        with self.lk:
            if self.active:
                raise HTTPException(409, "이미 VR 조작 중")
            if getattr(r, "cam_only", False):
                raise HTTPException(409, "카메라만 모드 — 팔 없음 (start_gripper.sh 로 실행)")
            if r.busy():
                raise HTTPException(409, "다른 동작 중")
            if r.rec.pending is not None:
                raise HTTPException(409, "앞 에피소드를 [저장] 또는 [버리기] 먼저")
            if r.table_plane is None and not no_table:
                raise HTTPException(409, "테이블을 먼저 재세요 ([테이블 점 추가]) — 또는 [테이블 검사 끔] 체크 (테이블 없이 시험)")
            if not (0.0 < float(max_vel) <= MAX_VEL_CAP):
                raise HTTPException(400, f"속도 0 ~ {MAX_VEL_CAP} rad/s")
            ok, msg, _ = pc.arm_ready_check()
            if not ok:
                raise HTTPException(409, msg)
            cp = r.cam_pitch.read() if r.cam_pitch else None
            if cp is not None and abs(cp["diff"]) > cam_pitch_max:
                raise HTTPException(409, f"카메라 숙임 기준과 {cp['diff']:+.2f}° — 거치대를 맞추세요")
            if not r.sim:
                import glob
                if not glob.glob("/dev/ttyUSB*"):
                    raise HTTPException(409, "485–USB 보드 포트(/dev/ttyUSB*) 없음")
            try:
                b = pc.call("/stream/begin", {"max_vel": float(max_vel)})
            except urllib.error.HTTPError as e:
                raise HTTPException(409, f"arm_server 가 스트림을 안 엶 ({e.code}: {e.read().decode(errors='ignore')[:120]})")
            except Exception as e:  # noqa: BLE001
                raise HTTPException(409, f"arm_server 응답 없음 ({type(e).__name__})")
            self.tok = b["token"]
            self.q = np.radians(np.asarray(b["target_deg"], dtype=float))
            self.max_vel = float(max_vel)
            pc.LAST_CMD.update(q=self.q.copy(), t=time.time())
            qm = self.mdl.to_model(self.q)
            self.q0_named = {self.mdl.qname[i]: float(qm[i]) for s in pc.SIDES for i in self.mdl.idx[s]}
            self.grip_last = {s: None for s in pc.SIDES}
            self.used = set()              # 실제로 움직인 손 — 녹화 meta side (학습 손 고르기에 씀)
            self.hands = {s: {"state": "대기 (그립을 누르면 따라감)", "reason": "", "level": "idle"} for s in pc.SIDES}
            self.n, self.t_last = 0, time.time()
            self.epoch += 1
            self.active = True
            self.no_table = bool(no_table)
            self.msg = f"VR 조작 중 (관절 {self.max_vel:.2f} rad/s" + (", 테이블 검사 끔)" if self.no_table else ")")
        with r.lk:
            r.phase, r.stage, r.waiting = "vr", "VR 조작", False
        r.stage_idx = 1
        if r.rec_on:
            try:
                r.rec.start(r.rec_name, r.rec_cameras(), r.rec_state_names(), r.rec_sample,
                            {"side": "both", "mode": "vr", "stages": ["시작", "VR 조작"], "table_plane": r.table_plane,
                             "max_vel": self.max_vel})
                r.say(f"녹화 시작 → {r.rec_name} (VR)")
            except (ValueError, RuntimeError) as e:
                self.stop(f"녹화 시작 못 함: {e}")
                raise HTTPException(409, f"녹화 시작 못 함: {e}")
        r.say(f"VR 조작 시작 — 관절 {self.max_vel:.2f} rad/s, 그립을 누른 손만 움직임" + (" · ⚠ 테이블 검사 끔" if self.no_table else ""))
        return {"ok": True, "epoch": self.epoch, "q": self.q0_named}

    def stop(self, why="VR 끝"):
        r = self.r
        with self.lk:
            if not self.active:
                return False
            self.active = False
            tok, self.tok = self.tok, None
            self.msg = f"꺼짐 — {why}"
        try:
            pc.call("/stream/end", {"token": tok}, timeout=2.0)
        except Exception:  # noqa: BLE001 — arm_server 가 2 초 뒤 세션을 닫음, 팔은 마지막 목표에서 멈춤
            pass
        with r.lk:
            r.phase, r.stage = "idle", f"VR 끝 ({why})"
        r.say(f"VR 조작 끝 — {why} (팔은 마지막 목표에서 멈춤)")
        if r.rec.active:
            used = getattr(self, "used", set())
            side = next(iter(used)) if len(used) == 1 else "both"     # 한 손만 썼으면 그 손 (학습 웹 손 고르기)
            pnd = r.rec.finish(True, "vr", {"why": why, "n_cmd": self.n, "side": side, "hands_used": sorted(used)})
            r.say(f"녹화 끝 ({pnd['n']} 프레임) — [저장] 또는 [버리기] 를 고르세요")
        return True

    def _watchdog(self):
        while True:
            time.sleep(0.2)
            if self.active and time.time() - self.t_last > IDLE_END_S:
                self.stop(f"VR 입력이 {IDLE_END_S} 초 넘게 없음")

    # ---------- 한 명령 ----------
    def cmd(self, names, q, grip, press, no_input=False):
        r, mdl = self.r, self.mdl
        with self.lk:
            if not self.active:
                raise HTTPException(409, f"VR 세션 없음 ({self.msg})")
            if no_input:                       # Quest 입력 없음 — 팔 · 그리퍼는 그대로, 세션만 유지
                for side in pc.SIDES:
                    self.hands[side] = {"state": "Quest 입력 없음", "reason": "Quest 에서 'Virtual Reality' 를 누르세요", "level": "idle"}
                press, grip = {}, {}
            qm_now = mdl.to_model(self.q)
            qm_in = qm_now.copy()
            for n, v in zip(names, q):
                i = self.byname.get(n)
                if i is not None and np.isfinite(v):
                    qm_in[i] = float(v)
            cand = qm_now.copy()
            for side in (() if no_input else pc.SIDES):
                ix = mdl.idx[side]
                if not press.get(side):
                    self.hands[side] = {"state": "대기 (그립 안 누름)", "reason": "", "level": "idle"}
                    continue
                jump = float(np.max(np.abs(qm_in[ix] - qm_now[ix])))
                if jump > JUMP_RAD:
                    self.hands[side] = {"state": "멈춤", "reason": f"목표가 한 번에 {math.degrees(jump):.0f}° 바뀜 (IK 튐)", "level": "stop"}
                    continue
                trial = cand.copy()
                trial[ix] = np.clip(qm_in[ix], mdl.lo[ix], mdl.hi[ix])
                why = self.check(trial, side)
                if why:
                    self.hands[side] = {"state": "멈춤", "reason": " · ".join(why), "level": "stop"}
                    continue
                cand = trial
                self.used.add(side)
                self.hands[side] = {"state": "따라감", "reason": "", "level": "ok"}
            tok = self.tok
        try:
            res = pc.call("/stream/cmd", {"token": tok, "deg": np.degrees(mdl.to_ctrl(cand)).tolist()}, timeout=1.0)
        except urllib.error.HTTPError as e:
            if self.tok == tok:                # 그 사이 새 세션이 열렸으면 그 세션은 건드리지 않음
                self.stop(f"arm_server 가 명령 거부 ({e.code})")
            raise HTTPException(409, "arm_server 세션 끝 — 잡기 웹에서 [VR 시작] 다시")
        except Exception as e:  # noqa: BLE001
            if self.tok == tok:
                self.stop(f"arm_server 응답 없음 ({type(e).__name__})")
            raise HTTPException(409, "arm_server 응답 없음")
        tgt = np.radians(np.asarray(res["target_deg"], dtype=float))
        lag = float(np.max(np.abs(np.asarray(res["target_deg"]) - np.asarray(res["meas_deg"]))))
        with self.lk:
            if not self.active or self.tok != tok:  # 보내는 사이 [정지] · [VR 끝] — 그리퍼 명령도 안 보냄
                raise HTTPException(409, f"VR 세션 없음 ({self.msg})")
            self.q = tgt
            self.n += 1
            self.t_last = time.time()
        pc.LAST_CMD.update(q=tgt.copy(), t=time.time())
        if lag > float(P["track_abort_deg"]):
            self.stop(f"명령과 실측이 {lag:.1f}° 차이 — 걸렸거나 부딪힘")
            raise HTTPException(409, "명령 · 실측 차이 — 세션 끝")
        # 그리퍼: 트리거만 따름 (그립과 상관없이 — 당기면 닫힘, 놓으면 벌림). Quest 입력이 없으면 그대로
        for side in pc.SIDES:
            if grip.get(side) is not None:
                self._grip(side, grip[side])
        qm = mdl.to_model(tgt)
        return {"ok": True, "hands": self.hands, "vel_clip": bool(res.get("vel_clip")), "lag_deg": round(lag, 1),
                "q": {mdl.qname[i]: float(qm[i]) for s in pc.SIDES for i in mdl.idx[s]}}

    def grip_target(self, closing, q_meas):
        """트리거 닫힘 비율 · 지금 손가락 q → 보낼 목표 q. 닫는 쪽은 지금 위치보다 SQUEEZE_Q 까지만 (물건에 막히면 거기서 멈춤)."""
        want = pc.q_for_gap(pc.gap_mm(pc.Q_HI) * (1.0 - min(max(float(closing), 0.0), 1.0)))
        if q_meas is None:
            return None                     # 그리퍼 상태를 모르면 안 보냄
        return min(max(max(want, float(q_meas) - SQUEEZE_Q), pc.Q_LO), pc.Q_HI)

    def _grip(self, side, closing):
        st = self.r.grip.state(side)
        gq = self.grip_target(closing, None if st is None else st.get("q"))
        if gq is None:
            return
        if self.grip_last[side] is None or abs(gq - self.grip_last[side]) > 0.03:
            if self.r.grip.set(side, gq):
                self.grip_last[side] = gq

    def state(self):
        with self.lk:
            age = None if not self.active else round(time.time() - self.t_last, 2)
            return {"active": self.active, "epoch": self.epoch, "msg": self.msg, "max_vel": self.max_vel, "n": self.n,
                    "age_s": age, "hands": self.hands, "q0": self.q0_named if self.active else None}
