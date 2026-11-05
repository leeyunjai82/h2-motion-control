#!/usr/bin/env python3
"""
check_dex1.py — Dex1-1 그리퍼 상태 확인 (gripper/) (rt/dex1/*/state 읽기 전용, 명령 송신 없음)

준비
  미니 PC 에서 dex1_1_service 실행 중:  sudo ./dex1_1_gripper_server -n <유선 인터페이스>
  (명령이 없으면 서비스가 모터를 BRAKE · kp = kd = 0 으로 둠 → 손으로 천천히 여닫을 수 있음, 저항 있음)

사용
  source activate_tv.sh
  python gripper/check_dex1.py enp45s0          # 20 초 동안 0.2 초마다 출력
  python gripper/check_dex1.py enp45s0 --sec 60

보는 것
  · 손별 q [rad] · dq · tau_est · 온도 — 손은 gripper/gripper.yaml topics 기준 (실제 손 ↔ 서비스 토픽)
  · 보정 확인: 손으로 끝까지 닫으면 q ≈ 0, 끝까지 벌리면 q 가 커짐
    (xr_teleoperate 는 0–5.4 rad 를 씀 — 이 로봇 그리퍼의 실제 최대값은 이 출력으로 확인)
  · 상태가 안 오면: 서비스 실행 · -n 인터페이스 · 그리퍼 전원 · 485 연결 확인
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # H2 SDK 경로 + DDS 초기화 (unitree_sdk2py 보다 먼저) — 설정은 안 바꿈
import gripper_config   # gripper/gripper.yaml

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_


def main():
    ap = argparse.ArgumentParser(description="Dex1-1 그리퍼 상태 확인 (읽기 전용)")
    ap.add_argument("iface", nargs="?", default=None, help="DDS 네트워크 인터페이스 (예: enp45s0) — 없으면 자동")
    ap.add_argument("--sec", type=float, default=20.0, help="확인 시간 [s] (기본 20)")
    a = ap.parse_args()

    topics = gripper_config.load()["topics"]
    print(f"[dex1] 손 ↔ 토픽: 왼손 {topics['left']}/state · 오른손 {topics['right']}/state  (gripper/gripper.yaml)")

    robot_env.dds_init(a.iface)
    last, lock = {}, threading.Lock()

    def handler(side):
        def f(msg):
            if not msg.states:
                return
            s = msg.states[0]
            with lock:
                last[side] = (time.time(), s.q, s.dq, s.tau_est, s.temperature, s.lost)
        return f

    subs = []
    for side in ("left", "right"):
        sub = ChannelSubscriber(f"{topics[side]}/state", MotorStates_)
        sub.Init(handler(side), 10)
        subs.append(sub)

    seen = {"left": False, "right": False}
    qmin = {"left": None, "right": None}
    qmax = {"left": None, "right": None}
    t0 = time.time()
    while time.time() - t0 < a.sec:
        time.sleep(0.2)
        now = time.time()
        parts = []
        with lock:
            snap = dict(last)
        for side, name in (("left", "왼손"), ("right", "오른손")):
            if side not in snap:
                parts.append(f"{name}: 상태 없음")
                continue
            t, q, dq, tau, temp, lost = snap[side]
            seen[side] = True
            qmin[side] = q if qmin[side] is None else min(qmin[side], q)
            qmax[side] = q if qmax[side] is None else max(qmax[side], q)
            age = now - t
            parts.append(f"{name}: q {q:6.3f}  dq {dq:6.2f}  tau {tau:5.2f}  {temp:3d}°C"
                         + (f"  ({age:.1f}s 전)" if age > 0.5 else ""))
        print(" | ".join(parts), flush=True)

    print()
    for side, name in (("left", "왼손"), ("right", "오른손")):
        if seen[side]:
            print(f"[dex1] {name}: 받은 q 범위 {qmin[side]:.3f} ~ {qmax[side]:.3f} rad")
        else:
            print(f"[dex1] {name}: 상태 없음 — 서비스 실행 · -n 인터페이스 · 그리퍼 전원 · 485 연결 · gripper/gripper.yaml topics 확인")


if __name__ == "__main__":
    main()
