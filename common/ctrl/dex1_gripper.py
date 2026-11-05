"""
dex1_gripper.py — Dex1-1 그리퍼 2개를 DDS 로 계속 제어 (arm_server 안에서, end_effector: dex1 일 때만)

  dex1_1_service (미니 PC, C++) 가 485 ↔ DDS rt/dex1/{left,right}/{cmd,state} 를 맡고,
  이 모듈은 실제 손(left/right) 기준으로 목표 q 를 받아 rate Hz 로 끊김 없이 보낸다.
  서비스는 명령이 끊기면 BRAKE · kp = kd = 0 (힘 풀림) → 잡은 물체를 놓치지 않게 계속 송신.

  · 손 ↔ 토픽: robot.yaml gripper.topics (이 로봇은 ID 0 그리퍼가 왼손 → rt/dex1/right)
  · q [rad]: 0 = 닫힘 (보정 기준), 클수록 벌어짐. 목표는 gripper.q_range 로 자름
  · 처음 상태를 받기 전에는 아무것도 보내지 않고, 받으면 그 자리를 목표로 잡음 (켜자마자 움직이지 않음)
  · 목표로 갈 때 명령 q 를 max_vel [rad/s] 로 조금씩 옮김 (0.6 rad ≈ 1 cm, xr_teleoperate 주석)
  · release(): 송신을 멈춤 → 서비스가 시간 초과로 힘을 풂

  g = Dex1Gripper(robot_env.GRIPPER).start()
  g.set(left=2.7, right=None)      # 오른손은 그대로
  g.snapshot()                     # {"left": {"q", "dq", "tau", "cmd", "goal", "age_s", "ok"}, ...}
"""
import threading
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

SIDES = ("left", "right")
STATE_STALE_S = 0.5          # 이보다 오래 상태가 안 오면 ok False (표시용 — 송신은 계속)


class Dex1Gripper:
    def __init__(self, cfg):
        cfg = dict(cfg or {})
        self.topics = dict(cfg.get("topics") or {"left": "rt/dex1/left", "right": "rt/dex1/right"})
        lo, hi = (cfg.get("q_range") or [0.0, 5.0])[:2]
        self.q_lo, self.q_hi = float(lo), float(hi)
        self.kp = float(cfg.get("kp", 5.0))
        self.kd = float(cfg.get("kd", 0.05))
        self.max_vel = float(cfg.get("max_vel", 3.0))
        self.rate = float(cfg.get("rate", 200.0))
        self._lk = threading.Lock()
        self._st = {s: None for s in SIDES}          # (t, q, dq, tau)
        self._cmd = {s: None for s in SIDES}         # 지금 보내는 명령 q
        self._goal = {s: None for s in SIDES}        # 목표 q
        self._vel = {s: self.max_vel for s in SIDES}
        self._sending = True
        self._pubs, self._subs = {}, []
        self._th = None

    # ---------- 시작 ----------
    def start(self):
        for s in SIDES:
            base = self.topics[s]
            sub = ChannelSubscriber(f"{base}/state", MotorStates_)
            sub.Init(self._handler(s), 10)
            self._subs.append(sub)
            pub = ChannelPublisher(f"{base}/cmd", MotorCmds_)
            pub.Init()
            self._pubs[s] = pub
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        print(f"[gripper] Dex1-1 — 왼손 {self.topics['left']} · 오른손 {self.topics['right']}, "
              f"q {self.q_lo}–{self.q_hi} rad, kp {self.kp} kd {self.kd}, {self.max_vel} rad/s, {self.rate:.0f} Hz")
        return self

    def _handler(self, side):
        def f(msg):
            if not msg.states:
                return
            st = msg.states[0]
            with self._lk:
                self._st[side] = (time.time(), float(st.q), float(st.dq), float(st.tau_est))
                if self._goal[side] is None:            # 처음 받은 상태 = 그 자리를 붙잡음
                    q0 = self._clip(st.q)
                    self._goal[side] = q0
                    self._cmd[side] = q0
                    print(f"[gripper] {side} 첫 상태 q {st.q:.3f} — 그 자리 유지")
        return f

    def _clip(self, q):
        return min(self.q_hi, max(self.q_lo, float(q)))

    # ---------- 송신 ----------
    def _loop(self):
        dt = 1.0 / self.rate
        t_prev = time.time()
        while True:
            time.sleep(dt)
            now = time.time()
            step_dt = min(now - t_prev, 0.05)
            t_prev = now
            with self._lk:
                if not self._sending:
                    continue
                out = {}
                for s in SIDES:
                    if self._goal[s] is None:
                        continue
                    c, g = self._cmd[s], self._goal[s]
                    step = self._vel[s] * step_dt
                    c = g if abs(g - c) <= step else c + (step if g > c else -step)
                    self._cmd[s] = c
                    out[s] = c
            for s, q in out.items():
                m = unitree_go_msg_dds__MotorCmd_()
                m.q, m.dq, m.tau, m.kp, m.kd = q, 0.0, 0.0, self.kp, self.kd
                try:
                    self._pubs[s].Write(MotorCmds_(cmds=[m]))
                except Exception as e:  # noqa: BLE001 — 송신 실패는 다음 주기에 다시
                    print(f"[gripper] {s} 송신 실패: {e}")

    # ---------- 명령 ----------
    def set(self, left=None, right=None, max_vel=None):
        """손별 목표 q [rad] (None = 그대로). 상태를 아직 못 받은 손은 무시하고 그 손 이름을 돌려줌."""
        skipped = []
        with self._lk:
            for s, q in (("left", left), ("right", right)):
                if q is None:
                    continue
                if self._goal[s] is None:
                    skipped.append(s)
                    continue
                self._goal[s] = self._clip(q)
                self._vel[s] = min(self.max_vel, float(max_vel)) if max_vel else self.max_vel
            self._sending = True
        return skipped

    def hold(self):
        """움직이던 것을 멈추고 지금 명령 자리를 유지 (힘은 유지 — 잡은 물체를 놓지 않음)."""
        with self._lk:
            for s in SIDES:
                if self._cmd[s] is not None:
                    self._goal[s] = self._cmd[s]
            self._sending = True

    def release(self):
        """송신을 멈춤 → 서비스가 시간 초과로 BRAKE · kp = kd = 0 (힘 풀림). set()/hold() 하면 다시 송신."""
        with self._lk:
            self._sending = False

    # ---------- 조회 ----------
    def snapshot(self):
        now = time.time()
        with self._lk:
            out = {"sending": self._sending, "q_range": [self.q_lo, self.q_hi]}
            for s in SIDES:
                st = self._st[s]
                d = {"cmd": None if self._cmd[s] is None else round(self._cmd[s], 4),
                     "goal": None if self._goal[s] is None else round(self._goal[s], 4)}
                if st is None:
                    d.update(ok=False, q=None, dq=None, tau=None, age_s=None)
                else:
                    t, q, dq, tau = st
                    d.update(ok=(now - t) < STATE_STALE_S, q=round(q, 4), dq=round(dq, 4),
                             tau=round(tau, 4), age_s=round(now - t, 3))
                out[s] = d
            return out
