#!/usr/bin/env python3
"""
move_test.py — Dex1-1 그리퍼 움직임 시험 (gripper/) (rt/dex1/*/cmd 송신 — 천천히 벌렸다 닫음)

준비
  미니 PC 에서 dex1_1_service 실행 중:  sudo ./dex1_1_gripper_server -n <유선 인터페이스>
  그리퍼 사이에 손가락·물건 없게. 닫는 힘은 kp 로 정해짐 (기본 kp 5.0 · kd 0.05 = xr_teleoperate 값)

사용
  source activate_tv.sh
  python gripper/move_test.py enp45s0                    # 양손: 현재 → 반쯤(2.7 rad) 벌림 → 닫힘 근처(0.3 rad)
  python gripper/move_test.py enp45s0 --side left        # 왼손만 (gripper/gripper.yaml topics 기준 실제 손)
  python gripper/move_test.py enp45s0 --open 5.0 --close 0.3 --speed 0.6

동작
  · 상태(rt/dex1/*/state)를 3 초 안에 못 받으면 아무것도 보내지 않고 끝냄
  · 목표 q 는 gripper.yaml q_range (실측 0 ~ 5.05 rad) 로 자름 (0 = 닫힘 — 보정 기준)
  · speed [rad/s] 로 목표를 조금씩 옮김 (0.6 rad ≈ 1 cm, xr_teleoperate 주석) — 200 Hz 송신
  · 끝나거나 Ctrl+C 면 송신을 멈춤 → 서비스가 시간 초과로 BRAKE · kp = kd = 0 (힘 풀림, 그 자리)
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # H2 SDK 경로 + DDS 초기화 (unitree_sdk2py 보다 먼저) — 설정은 안 바꿈
import gripper_config   # gripper/gripper.yaml

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

CFG = gripper_config.load()
Q_MIN, Q_MAX = float(CFG["q_range"][0]), float(CFG["q_range"][1])   # 실측 범위 (gripper.yaml q_range)
RATE = 200.0


def main():
    ap = argparse.ArgumentParser(description="Dex1-1 그리퍼 움직임 시험 (천천히 벌렸다 닫음)")
    ap.add_argument("iface", nargs="?", default=None, help="DDS 네트워크 인터페이스 (예: enp45s0)")
    ap.add_argument("--side", choices=("left", "right", "both"), default="both", help="실제 손 (기본 both)")
    ap.add_argument("--open", type=float, default=2.7, help="벌릴 q [rad] (기본 2.7 = 반쯤)")
    ap.add_argument("--close", type=float, default=0.3, help="닫을 q [rad] (기본 0.3 — 끝까지 닫지 않음)")
    ap.add_argument("--speed", type=float, default=1.0, help="목표 이동 속도 [rad/s] (기본 1.0)")
    ap.add_argument("--hold", type=float, default=1.0, help="각 자리에서 머무는 시간 [s]")
    ap.add_argument("--kp", type=float, default=5.0)
    ap.add_argument("--kd", type=float, default=0.05)
    a = ap.parse_args()

    clip = lambda v: min(Q_MAX, max(Q_MIN, float(v)))
    q_open, q_close = clip(a.open), clip(a.close)
    speed = max(0.1, min(3.0, a.speed))
    sides = ("left", "right") if a.side == "both" else (a.side,)
    topics = CFG["topics"]
    for s in sides:
        print(f"[dex1] {s}: {topics[s]}/cmd · /state  (gripper/gripper.yaml)")

    robot_env.dds_init(a.iface)
    state, lock = {}, threading.Lock()

    def handler(side):
        def f(msg):
            if msg.states:
                with lock:
                    state[side] = (time.time(), msg.states[0].q)
        return f

    subs, pubs = [], {}
    for s in sides:
        sub = ChannelSubscriber(f"{topics[s]}/state", MotorStates_)
        sub.Init(handler(s), 10)
        subs.append(sub)
        pub = ChannelPublisher(f"{topics[s]}/cmd", MotorCmds_)
        pub.Init()
        pubs[s] = pub

    t_end = time.time() + 3.0
    while time.time() < t_end and len(state) < len(sides):
        time.sleep(0.05)
    missing = [s for s in sides if s not in state]
    if missing:
        print(f"[dex1] 상태 없음: {missing} — 아무것도 보내지 않고 끝냄 (서비스 · -n · 전원 · 485 · gripper/gripper.yaml topics 확인)")
        return

    with lock:
        target = {s: state[s][1] for s in sides}
    print("[dex1] 시작 q:", {s: round(v, 3) for s, v in target.items()})
    plan = [("벌림", q_open), ("닫힘", q_close)]

    def send(goal):
        for s in sides:
            c = unitree_go_msg_dds__MotorCmd_()
            c.q, c.dq, c.tau, c.kp, c.kd = float(goal[s]), 0.0, 0.0, a.kp, a.kd
            pubs[s].Write(MotorCmds_(cmds=[c]))

    dt = 1.0 / RATE
    try:
        for name, goal_q in plan:
            print(f"[dex1] → {name} {goal_q:.2f} rad")
            t_hold = None
            t_print = 0.0
            while True:
                done = True
                for s in sides:
                    d = goal_q - target[s]
                    step = speed * dt
                    target[s] += max(-step, min(step, d))
                    done &= abs(goal_q - target[s]) < 1e-6
                send(target)
                now = time.time()
                if now - t_print > 0.25:
                    with lock:
                        meas = {s: round(state[s][1], 3) for s in sides if s in state}
                    print(f"   목표 {round(next(iter(target.values())), 3):6.3f}  실측 {meas}", flush=True)
                    t_print = now
                if done:
                    t_hold = t_hold or now
                    if now - t_hold >= a.hold:
                        break
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[dex1] Ctrl+C — 송신 멈춤")
    with lock:
        print("[dex1] 끝 q:", {s: round(state[s][1], 3) for s in sides if s in state})
    print("[dex1] 송신 멈춤 → 서비스 시간 초과 뒤 BRAKE (힘 풀림)")


if __name__ == "__main__":
    main()
