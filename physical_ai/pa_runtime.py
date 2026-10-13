"""
pa_runtime.py — policy_server 가 쓰는 부품 (카메라 · DDS 상태 · 손 위치 FK · OpenVINO 정책 · 안전 필터)

  CameraReader   : MJPEG(/video_feed) 최신 프레임 (JPEG 바이트 + 수신 시각)
  ArmWatch       : DDS rt/lowstate(실측 q·온도·tau_est) + rt/arm_sdk(지령 q·weight) 구독 — 읽기 전용, 송신 없음
  HandFK         : 팔 14축 → 왼손·오른손 끝 위치 (IK 좌표 = pelvis 기준, robot_server/sim_server 와 같은 축소 모델)
  OVPolicy       : 학습 PC 에서 받은 OpenVINO IR (policy.xml/.bin + meta.json) 추론
  TemporalEnsembler : ACT 시간 앙상블 (매 스텝 추론, 겹치는 예측을 지수 가중 평균)
  Safety         : 관절 범위 · 손 작업 공간 · 손 사이 거리 · 추종 오차 · 토크 검사 (온도는 표시만 — 정지 안 함)

데이터 형식 (episode.npz) 은 physical_ai/README.md 참고. 관절 순서는 robot.yaml joints.arm (= arm_server 14축) 그대로.
"""
import json
import os
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "common"))
import robot_env   # noqa: E402 — ROBOT 검사 + robot.yaml

J = robot_env.JOINTS
N_MOTORS = int(J["motor_slots"])
ARM_SLOTS = [int(i) for i in J["arm"]]
WEIGHT_SLOT = int(J["weight_slot"])
_SLOT_NAME = {int(v): k for k, v in (J.get("map") or {}).items()}
JOINT_NAMES = [_SLOT_NAME.get(s, f"slot{s}") for s in ARM_SLOTS]     # URDF 관절 이름 (14)
MODEL_FORMAT = "h2-act-v1"


# =====================================================================
# 카메라 (MJPEG)
# =====================================================================
class CameraReader:
    """multipart MJPEG 스트림에서 JPEG 를 SOI(FFD8)~EOI(FFD9) 로 잘라 최신 1장만 보관. 끊기면 1초 뒤 다시 연결."""

    def __init__(self, url):
        self.url = url
        self.jpeg = None
        self.t = 0.0
        self.count = 0
        self.err = ""
        self.fps = 0.0
        self._cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    self.err = ""
                    buf = b""
                    n0, t0 = self.count, time.time()
                    while True:
                        chunk = r.read1(65536)
                        if not chunk:
                            raise IOError("스트림 끝")
                        buf += chunk
                        while True:
                            a = buf.find(b"\xff\xd8")
                            if a < 0:
                                buf = buf[-1:]
                                break
                            b = buf.find(b"\xff\xd9", a + 2)
                            if b < 0:
                                buf = buf[a:]
                                break
                            jpg = buf[a:b + 2]
                            buf = buf[b + 2:]
                            with self._cond:
                                self.jpeg, self.t = jpg, time.time()
                                self.count += 1
                                self._cond.notify_all()
                        if len(buf) > 4_000_000:          # 경계를 못 찾는 이상한 스트림
                            buf = b""
                        dt = time.time() - t0
                        if dt >= 2.0:
                            self.fps = (self.count - n0) / dt
                            n0, t0 = self.count, time.time()
            except Exception as e:     # noqa: BLE001 — 끊김은 재연결
                self.err = f"{type(e).__name__}: {e}"
                self.fps = 0.0
                time.sleep(1.0)

    def latest(self):
        with self._cond:
            return self.jpeg, self.t, self.count

    def wait_new(self, last_count, timeout=1.0):
        with self._cond:
            self._cond.wait_for(lambda: self.count != last_count, timeout=timeout)
            return self.jpeg, self.t, self.count

    def age(self):
        return time.time() - self.t if self.t else float("inf")


# =====================================================================
# DDS 상태 (읽기 전용)
# =====================================================================
class ArmWatch:
    """rt/lowstate · rt/arm_sdk 구독. arm_server 가 rt/arm_sdk 를 250 Hz 로 보내므로 지령 q 를 그대로 받는다."""

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        robot_env.dds_init()
        self._lk = threading.Lock()
        self.q = np.zeros(14)
        self.dq = np.zeros(14)
        self.temp = np.zeros(14)
        self.tau = np.zeros(14)
        self.t_state = 0.0
        self.cmd_q = np.zeros(14)
        self.weight = 0.0
        self.t_cmd = 0.0
        self._sub_s = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub_s.Init()
        self._sub_c = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self._sub_c.Init()
        threading.Thread(target=self._loop_state, daemon=True).start()
        threading.Thread(target=self._loop_cmd, daemon=True).start()

    def _loop_state(self):
        while True:
            m = self._sub_s.Read(1.0)
            if m is None:
                continue
            ms = m.motor_state
            q = np.array([ms[s].q for s in ARM_SLOTS], dtype=float)
            dq = np.array([ms[s].dq for s in ARM_SLOTS], dtype=float)
            try:   # temperature[2] = [외부, 권선] (robots/h2/FACTS.md) — 큰 값
                tp = np.array([float(max(ms[s].temperature)) for s in ARM_SLOTS])
            except Exception:   # noqa: BLE001
                tp = np.zeros(14)
            tau = np.array([float(ms[s].tau_est) for s in ARM_SLOTS])
            with self._lk:
                self.q, self.dq, self.temp, self.tau, self.t_state = q, dq, tp, tau, time.time()

    def _loop_cmd(self):
        while True:
            m = self._sub_c.Read(1.0)
            if m is None:
                continue
            mc = m.motor_cmd
            q = np.array([mc[s].q for s in ARM_SLOTS], dtype=float)
            w = float(mc[WEIGHT_SLOT].q)
            with self._lk:
                self.cmd_q, self.weight, self.t_cmd = q, w, time.time()

    def snapshot(self):
        with self._lk:
            return {"q": self.q.copy(), "dq": self.dq.copy(), "temp": self.temp.copy(), "tau": self.tau.copy(),
                    "t_state": self.t_state, "cmd_q": self.cmd_q.copy(), "weight": self.weight, "t_cmd": self.t_cmd}


# =====================================================================
# 손 위치 FK · URDF 한계
# =====================================================================
class HandFK:
    """IK 와 같은 축소 모델 (robot.yaml ik.lock_joints, ee_joints + ee_offset) — 손끝 위치 [m], IK 좌표."""

    def __init__(self):
        import pinocchio as pin
        self.pin = pin
        full = pin.buildModelFromUrdf(robot_env.URDF_PATH)
        ik = robot_env.CFG["ik"]
        lock = [full.getJointId(n) for n in ik["lock_joints"] if full.existJointName(n)]
        red = pin.buildReducedModel(full, lock, np.zeros(full.nq))
        off = np.array(ik["ee_offset"], dtype=float)
        for nm, jn in zip(("L_ee", "R_ee"), ik["ee_joints"]):
            red.addFrame(pin.Frame(nm, red.getJointId(jn), pin.SE3(np.eye(3), off), pin.FrameType.OP_FRAME))
        self.model, self.data = red, red.createData()
        self.fl, self.fr = red.getFrameId("L_ee"), red.getFrameId("R_ee")
        # 축소 모델 q 순서 → ARM_SLOTS 안 위치
        red_slots = [int(J["map"][red.names[j]]) for j in range(1, red.njoints)]
        self.perm = [ARM_SLOTS.index(s) for s in red_slots]
        # URDF 한계 / 최대 토크 (ARM_SLOTS 순서)
        lo, hi, eff = [], [], []
        for nm in JOINT_NAMES:
            jid = full.getJointId(nm)
            iq, iv = full.joints[jid].idx_q, full.joints[jid].idx_v
            lo.append(full.lowerPositionLimit[iq]); hi.append(full.upperPositionLimit[iq])
            eff.append(full.effortLimit[iv])
        self.lo, self.hi, self.effort = np.array(lo), np.array(hi), np.array(eff)
        self._lk = threading.Lock()

    def hands(self, q14):
        """q14 (ARM_SLOTS 순서, rad) → (6,) [Lx, Ly, Lz, Rx, Ry, Rz]."""
        q = np.asarray(q14, dtype=float)[self.perm]
        with self._lk:
            self.pin.framesForwardKinematics(self.model, self.data, q)
            return np.concatenate([self.data.oMf[self.fl].translation, self.data.oMf[self.fr].translation]).copy()


# =====================================================================
# OpenVINO 정책
# =====================================================================
def read_model_meta(model_dir):
    with open(os.path.join(model_dir, "meta.json"), encoding="utf-8") as f:
        return json.load(f)


def check_model_meta(meta):
    """로봇과 맞는 모델인지 — 형식, 관절 이름·순서, 차원. 문제 목록 반환 (빈 목록 = OK)."""
    errs = []
    if meta.get("format") != MODEL_FORMAT:
        errs.append(f"format {meta.get('format')} ≠ {MODEL_FORMAT}")
    if list(meta.get("joint_names") or []) != JOINT_NAMES:
        errs.append("joint_names 가 이 로봇 팔 14축 순서와 다름 (robot.yaml joints.arm)")
    if int(meta.get("state_dim", 0)) != 14 or int(meta.get("action_dim", 0)) != 14:
        errs.append("state_dim/action_dim 은 14")
    for k in ("chunk_size", "image", "stats", "fps"):
        if k not in meta:
            errs.append(f"meta.{k} 없음")
    return errs


class OVPolicy:
    """입력 image [1,3,H,W] float 0~1 RGB, state [1,14] rad → 출력 action [1,chunk,14] rad.
    정규화(이미지 ImageNet · 상태/행동 평균·표준편차)는 모델 안에 들어 있다 (trainer/act_model.py ACTExport)."""

    def __init__(self, model_dir, device="CPU"):
        import openvino as ov
        self.dir = model_dir
        self.meta = read_model_meta(model_dir)
        errs = check_model_meta(self.meta)
        if errs:
            raise ValueError("; ".join(errs))
        self.H, self.W = int(self.meta["image"]["h"]), int(self.meta["image"]["w"])
        self.chunk = int(self.meta["chunk_size"])
        core = ov.Core()
        model = core.read_model(os.path.join(model_dir, "policy.xml"))
        cfg = {"PERFORMANCE_HINT": "LATENCY"}
        if device == "CPU":     # 일부 CPU 는 기본이 bf16 → 학습 PC 검사(f32)와 같게. GPU/NPU 는 장치 기본(f16)
            cfg["INFERENCE_PRECISION_HINT"] = "f32"
        t0 = time.time()
        self.compiled = core.compile_model(model, device, cfg)
        self.compile_s = time.time() - t0
        try:
            self.exec_devices = list(self.compiled.get_property("EXECUTION_DEVICES"))
        except Exception:   # noqa: BLE001
            self.exec_devices = [device]
        self.device = device
        self.req = self.compiled.create_infer_request()
        names = set().union(*(p.get_names() for p in self.compiled.inputs))
        if not {"image", "state"} <= names:
            raise ValueError(f"모델 입력 이름 {sorted(names)} — image, state 필요")
        self.out = self.compiled.output(0)
        self.last_ms = 0.0

    def preprocess(self, jpeg):
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("JPEG 디코딩 실패")
        if img.shape[0] != self.H or img.shape[1] != self.W:
            img = cv2.resize(img, (self.W, self.H), interpolation=cv2.INTER_AREA)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.ascontiguousarray(img.transpose(2, 0, 1)[None])

    def infer(self, jpeg, state):
        x = self.preprocess(jpeg)
        s = np.asarray(state, dtype=np.float32).reshape(1, 14)
        t0 = time.perf_counter()
        res = self.req.infer({"image": x, "state": s})
        self.last_ms = (time.perf_counter() - t0) * 1000.0
        a = np.asarray(res[self.out], dtype=np.float64).reshape(self.chunk, 14)
        return a


class TemporalEnsembler:
    """ACT 시간 앙상블 (LeRobot ACTTemporalEnsembler 와 같은 식). w_i = exp(-coeff·i), 먼저 나온 예측이 i=0."""

    def __init__(self, chunk, coeff=0.01):
        self.chunk = chunk
        self.w = np.exp(-coeff * np.arange(chunk))
        self.wc = np.cumsum(self.w)
        self.reset()

    def reset(self):
        self.acts = None      # (L, D)
        self.cnt = None       # (L,)

    def update(self, actions):
        a = np.asarray(actions, dtype=float)
        if self.acts is None:
            self.acts = a.copy()
            self.cnt = np.ones(len(a), dtype=int)
        else:
            c = self.cnt
            self.acts = self.acts * self.wc[c - 1][:, None]
            self.acts = self.acts + a[:-1] * self.w[c][:, None]
            self.acts = self.acts / self.wc[c][:, None]
            self.cnt = np.minimum(c + 1, self.chunk)     # 큐 길이 chunk−1 이라 c ≤ chunk−1 → w[c] 범위 안
            self.acts = np.vstack([self.acts, a[-1:]])
            self.cnt = np.concatenate([self.cnt, [1]])
        out = self.acts[0].copy()
        self.acts, self.cnt = self.acts[1:], self.cnt[1:]
        return out


# =====================================================================
# 안전 필터
# =====================================================================
DEFAULT_SAFETY = {
    "max_vel": 1.5,            # rad/s — arm_server 스트림 세션 속도 제한 (데모 최대 속도는 모델 meta stats.max_vel 로 표시)
    "margin_deg": 5.0,         # 데이터셋 지령 범위 ± 이만큼 (URDF 한계 안으로 다시 자름)
    "ws_margin_m": 0.05,       # 데이터셋 손 위치 범위(박스) ± 이만큼
    "hand_gap_margin_m": 0.03, # 데이터셋 두 손 최소 거리 − 이만큼 보다 가까우면 거부
    "max_reject": 10,          # 연속 거부(작업 공간 밖·손 너무 가까움) 횟수 → 정지 (30 Hz 기준 0.33 s)
    "track_err_deg": 20.0,     # 목표−실측 차이가 이보다 큰 상태가
    "track_err_sec": 0.5,      #   이만큼 이어지면 정지 (막힘·충돌 의심)
    "torque_ratio": 0.9,       # |tau_est| / URDF effort 가 이보다 큰 상태가
    "torque_sec": 0.3,         #   이만큼 이어지면 정지
    "max_run_s": 60.0,         # 한 번 실행 최대 시간
    "frame_stale_s": 0.3,      # 카메라 프레임이 이보다 오래되면 정지
    "state_stale_s": 0.1,      # rt/lowstate 가 이보다 오래되면 정지
}


def cmd_hand_stats(fk, actions):
    """지령(action) 관절각들 → 그 지령 손 위치(FK) 범위·두 손 거리 범위. actions: (N,14) 들의 목록.
    안전 필터는 정책의 '지령' 을 검사하므로 기준도 데모의 '지령' 손 위치여야 함 — 실측(ee)은 박스에 막혀 지령보다 바깥
    (조이기: 지령 손이 박스 면보다 L/R 2 cm + squeeze 3 cm 안쪽 → 실측 두 손 거리보다 약 10 cm 가까움)."""
    lo, hi, gmin, gmax, n = np.full(6, np.inf), np.full(6, -np.inf), np.inf, -np.inf, 0
    for A in actions:
        for q in np.asarray(A, dtype=float):
            h = fk.hands(q)
            lo, hi = np.minimum(lo, h), np.maximum(hi, h)
            g = float(np.linalg.norm(h[:3] - h[3:]))
            gmin, gmax, n = min(gmin, g), max(gmax, g), n + 1
    if not n:
        return None
    return {"ee_cmd_min": lo.tolist(), "ee_cmd_max": hi.tolist(), "hand_gap_cmd_min": gmin, "hand_gap_cmd_max": gmax,
            "frames": n}


class Safety:
    """정책 출력 a (14, rad) → 안전한 목표 또는 거부. stop 조건은 check_* 가 사유 문자열을 돌려준다.
    손 작업 공간·두 손 거리는 데모 '지령' 손 위치 기준 (meta stats 의 ee_cmd_* 또는 로봇 PC 데이터셋에서 계산한 cmd_stats).
    둘 다 없으면 실측(ee) 기준 — 조일 때 지령이 실측보다 안쪽이라 거부될 수 있음 (2026-10-13 실기: 0.236 < 0.292 m 로 정지)."""

    def __init__(self, meta, fk: HandFK, cfg, cmd_stats=None):
        self.cfg = dict(DEFAULT_SAFETY)
        self.cfg.update({k: float(v) for k, v in (cfg or {}).items() if k in DEFAULT_SAFETY})
        st = dict(meta["stats"])
        cs = cmd_stats if cmd_stats else (st if "ee_cmd_min" in st else None)
        if cs:
            st["ee_min"], st["ee_max"], st["hand_gap_min"] = cs["ee_cmd_min"], cs["ee_cmd_max"], cs["hand_gap_cmd_min"]
            self.hand_ref = "지령"
        else:
            self.hand_ref = "실측 (지령 기준 없음 — 조일 때 거부될 수 있음)"
        m = np.radians(self.cfg["margin_deg"])
        lo = np.asarray(st["action_min"], dtype=float) - m
        hi = np.asarray(st["action_max"], dtype=float) + m
        self.lo = np.maximum(lo, fk.lo)
        self.hi = np.minimum(hi, fk.hi)
        wm = self.cfg["ws_margin_m"]
        self.ws_lo = np.asarray(st["ee_min"], dtype=float) - wm
        self.ws_hi = np.asarray(st["ee_max"], dtype=float) + wm
        self.gap_min = float(st["hand_gap_min"]) - self.cfg["hand_gap_margin_m"]
        self.fk = fk
        self.reset()

    def reset(self):
        self.n_clip = 0          # 관절 범위로 자른 횟수 (누적)
        self.n_reject = 0        # 거부 (누적)
        self.reject_run = 0      # 연속 거부
        self.t_track = None
        self.t_torque = None
        self.last_reason = ""

    def filter(self, a, prev):
        """a: 정책 목표, prev: 지난번 보낸 목표 → (보낼 목표, 정지 사유 또는 None)."""
        q = np.clip(a, self.lo, self.hi)
        if np.any(np.abs(q - a) > 1e-6):
            self.n_clip += 1
        h = self.fk.hands(q)
        why = None
        if np.any(h < self.ws_lo) or np.any(h > self.ws_hi):
            k = int(np.argmax(np.maximum(self.ws_lo - h, h - self.ws_hi)))
            why = f"손 작업 공간 밖 ({'LR'[k // 3]}{'xyz'[k % 3]} {h[k]:+.3f} m)"
        elif np.linalg.norm(h[:3] - h[3:]) < self.gap_min:
            why = f"두 손 너무 가까움 ({np.linalg.norm(h[:3] - h[3:]):.3f} < {self.gap_min:.3f} m)"
        if why:
            self.n_reject += 1
            self.reject_run += 1
            self.last_reason = why
            if self.reject_run >= self.cfg["max_reject"]:
                return prev, f"연속 거부 {self.reject_run}회 — {why}"
            return prev, None
        self.reject_run = 0
        return q, None

    def check_robot(self, snap, target, now):
        """실측 상태로 정지 조건 확인 → 사유 또는 None."""
        c = self.cfg
        if now - snap["t_state"] > c["state_stale_s"]:
            return f"rt/lowstate 끊김 ({now - snap['t_state']:.2f}s)"
        # 온도로는 멈추지 않음 (사용자 2026-10-13) — 근거 있는 값이 없음: H2 팔 모터 보호 온도 자료 없음 (확인 필요).
        #   예전 70 °C 는 실측 최고 61 °C 에 여유를 둔 임의값이었음. 온도는 웹 DDS 칩·모터 최고로 표시만
        if target is not None:
            err = np.abs(np.asarray(target) - snap["q"])
            if np.degrees(err.max()) > c["track_err_deg"]:
                self.t_track = self.t_track or now
                if now - self.t_track > c["track_err_sec"]:
                    k = int(err.argmax())
                    return f"추종 오차 {np.degrees(err[k]):.1f}° ({JOINT_NAMES[k]}) — 막힘/충돌 의심"
            else:
                self.t_track = None
        eff = np.where(self.fk.effort > 0, self.fk.effort, np.inf)
        ratio = np.abs(snap["tau"]) / eff
        if ratio.max() > c["torque_ratio"]:
            self.t_torque = self.t_torque or now
            if now - self.t_torque > c["torque_sec"]:
                k = int(ratio.argmax())
                return f"토크 {snap['tau'][k]:+.1f} Nm = 최대의 {ratio[k] * 100:.0f}% ({JOINT_NAMES[k]})"
        else:
            self.t_torque = None
        return None
