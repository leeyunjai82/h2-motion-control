#!/usr/bin/env python3
"""
check_robot_id.py — 연결된 로봇 식별값 확인 (rt/lowstate 읽기 전용, 지령 송신 없음)

목적
  ROBOT 설정과 실제 연결된 로봇이 다를 때 실행을 거부하는 안전장치에 쓸 값을 실기에서 잰다.
  unitree_hg LowState_.motor_state 는 G1/H2 모두 35칸 고정 배열이라
  배열 길이로는 관절 수를 알 수 없다 → mode_machine 과 '살아있는 모터 슬롯'을 같이 본다.

사용
  conda activate tv
  python tools/check_robot_id.py            # 인터페이스 자동
  python tools/check_robot_id.py enp3s0     # DDS 인터페이스 지정

출력
  · version / mode_pr / mode_machine
  · 슬롯별 temperature / vol / motorstate / q  — 모터가 실제 붙은 슬롯은 온도·전압이 0 이 아님
    (이 판정 기준 자체도 실기 출력으로 확인할 것)
"""
import sys
import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

N_SAMPLES = 20      # 몇 번 읽어서 mode_machine 이 일정한지 확인


def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    sub = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub.Init()

    print("[check] rt/lowstate 수신 대기 (최대 10초)...")
    msg = None
    t_end = time.time() + 10.0
    while time.time() < t_end:
        msg = sub.Read(0.5)
        if msg is not None:
            break
    if msg is None:
        print("[check] ❌ 수신 없음 — 네트워크(192.168.123.x) / 인터페이스 / 로봇 전원 확인")
        sys.exit(1)

    machines = set()
    for _ in range(N_SAMPLES):
        m = sub.Read(0.5)
        if m is not None:
            machines.add(int(m.mode_machine))
            msg = m
        time.sleep(0.02)

    print(f"\nversion      : {list(msg.version)}")
    print(f"mode_pr      : {msg.mode_pr}")
    print(f"mode_machine : {msg.mode_machine}   ({N_SAMPLES}회 관측값: {sorted(machines)})")
    print(f"motor_state  : {len(msg.motor_state)} 슬롯 (메시지 고정 길이)")

    print("\nidx  mode     q(rad)   temp0 temp1    vol   motorstate")
    alive = []
    for i, s in enumerate(msg.motor_state):
        t0, t1 = int(s.temperature[0]), int(s.temperature[1])
        live = (t0 != 0 or t1 != 0 or float(s.vol) > 0.0)
        if live:
            alive.append(i)
        print(f"{i:3d}  {int(s.mode):4d}  {float(s.q):+8.4f}  {t0:5d} {t1:5d}  {float(s.vol):6.2f}  "
              f"{int(s.motorstate):10d}{'' if live else '   (응답 없음?)'}")

    print(f"\n온도/전압이 0 이 아닌 슬롯: {len(alive)}개 → {alive}")
    print("※ 이 값을 robots/<robot>/ 설정의 기준값으로 쓰기 전에, 로봇별로 반복 측정해 일정한지 확인할 것")


if __name__ == "__main__":
    main()
