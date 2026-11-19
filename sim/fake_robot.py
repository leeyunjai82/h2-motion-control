#!/usr/bin/env python3
"""
fake_robot.py — 로봇 없이 서버 스택을 시험하기 위한 기구학 가짜 로봇 (DDS)

  ROBOT=h2 ROBOT_SIM=1 python sim/fake_robot.py      (보통은 ./start_grab_sim.sh 가 띄운다)

동작
  · DDS 도메인 1 (robot_env.DDS_DOMAIN) 전용. ROBOT_SIM=1 이 아니면 실행 거부
    → 실기 네트워크(도메인 0)에 가짜 rt/lowstate 가 나가는 일이 없다.
  · rt/arm_sdk (unitree_hg LowCmd_) 구독:
      weight = motor_cmd[weight_slot].q (0~1)
      팔/허리/헤드 슬롯 목표 = (1 - weight) · 보행 자세 + weight · 명령 q   (실기 arm_sdk 혼합과 같은 개념)
  · 관절은 1차 지연(τ 40 ms) + 속도 제한 6 rad/s + URDF 관절 한계로 목표를 따라간다.
    (동역학·접촉·균형은 없음 — IK 도달 여부, 시퀀스 순서, 허리 yaw, weight 램프를 보는 용도)
  · rt/lowstate 200 Hz 발행. mode_machine = robot.yaml identity.mode_machine (없으면 0),
    모터가 있는 슬롯은 temperature 30 / vol 48 로 채운다 (tools/check_robot_id.py 로 확인 가능).
  · 보행(LocoClient RPC) 은 흉내 내지 않는다 — 시뮬 범위는 제자리 팔/허리 동작.
"""
import os
import sys
import threading
import time

import numpy as np

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env   # ROBOT 검사 + robot.yaml

if not robot_env.SIM:
    print("[fake_robot] ❌ ROBOT_SIM=1 이 아니면 실행하지 않습니다 (실기 DDS 도메인 0 에 가짜 lowstate 금지)")
    sys.exit(2)

import pinocchio as pin
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

DT = 0.002            # 내부 적분 500 Hz
PUB_EVERY = 0.005     # lowstate 200 Hz
TAU = 0.04            # 1차 지연 [s]
VMAX = 6.0            # 속도 제한 [rad/s]

J = robot_env.JOINTS
N = int(J["motor_slots"])
ARM = [int(i) for i in J["arm"]]
WAIST = [int(i) for i in J["waist"]]
HEAD = [int(i) for i in (J.get("head") or [])]
WEIGHT = int(J["weight_slot"])
CTRL = ARM + WAIST + HEAD                     # arm_sdk 로 움직이는 슬롯
LIVE = sorted(set(int(v) for v in J["map"].values()))   # 모터가 있는 슬롯

# URDF 관절 한계 (슬롯별)
_model = pin.buildModelFromUrdf(robot_env.URDF_PATH)
LO = np.full(N, -np.inf)
HI = np.full(N, np.inf)
for name, slot in J["map"].items():
    if _model.existJointName(name):
        jid = _model.getJointId(name)
        iq = _model.joints[jid].idx_q
        LO[int(slot)] = _model.lowerPositionLimit[iq]
        HI[int(slot)] = _model.upperPositionLimit[iq]

# 보행 제어기가 잡고 있는 자세 (weight=0 일 때) — 팔 = default_arm_deg, 나머지 0
LOCO_POSE = np.zeros(N)
LOCO_POSE[ARM] = np.radians(np.array(robot_env.CFG["default_arm_deg"], dtype=float))

q = LOCO_POSE.copy()
dq = np.zeros(N)
cmd_q = LOCO_POSE.copy()
weight = 0.0
last_cmd_t = 0.0
lock = threading.Lock()
n_cmd = 0


def on_arm_sdk(msg: LowCmd_):
    global weight, last_cmd_t, n_cmd
    with lock:
        weight = float(np.clip(msg.motor_cmd[WEIGHT].q, 0.0, 1.0))
        for i in CTRL:
            cmd_q[i] = float(msg.motor_cmd[i].q)
        last_cmd_t = time.time()
        n_cmd += 1


def step():
    global q
    with lock:
        w = weight if (time.time() - last_cmd_t) < 0.5 else 0.0   # 명령이 끊기면 보행 제어기로 복귀
        target = LOCO_POSE.copy()
        target[CTRL] = (1.0 - w) * LOCO_POSE[CTRL] + w * cmd_q[CTRL]
    target = np.clip(target, LO, HI)
    v = np.clip((target - q) / TAU, -VMAX, VMAX)
    q_new = np.clip(q + v * DT, LO, HI)
    dq[:] = (q_new - q) / DT
    q = q_new


def main():
    robot_env.dds_init()
    pub = ChannelPublisher("rt/lowstate", LowState_)
    pub.Init()
    sub = ChannelSubscriber("rt/arm_sdk", LowCmd_)
    sub.Init(on_arm_sdk, 10)

    mm = (robot_env.CFG.get("identity") or {}).get("mode_machine")
    mode_machine = int(mm) if mm is not None else 0
    st = unitree_hg_msg_dds__LowState_()
    st.mode_machine = mode_machine
    for i in LIVE:
        st.motor_state[i].temperature = [30, 30]
        st.motor_state[i].vol = 48.0

    print(f"[fake_robot] ROBOT={robot_env.ROBOT}  DDS 도메인 {robot_env.DDS_DOMAIN}  "
          f"mode_machine={mode_machine}  팔 {ARM[0]}~{ARM[-1]} 허리 {WAIST} 헤드 {HEAD} weight 슬롯 {WEIGHT}")
    t_pub = 0.0
    t_log = time.time()
    tick = 0
    while True:
        t0 = time.time()
        step()
        if t0 - t_pub >= PUB_EVERY:
            for i in range(N):
                st.motor_state[i].q = float(q[i])
                st.motor_state[i].dq = float(dq[i])
            st.tick = tick
            pub.Write(st)
            tick += 1
            t_pub = t0
        if t0 - t_log >= 5.0:
            with lock:
                w, n = weight, n_cmd
            print(f"[fake_robot] weight={w:.2f}  arm_sdk 수신 {n}  허리(deg)={np.round(np.degrees(q[WAIST]), 1).tolist()}")
            t_log = t0
        time.sleep(max(0.0, DT - (time.time() - t0)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
