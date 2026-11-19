#!/usr/bin/env python3
"""
robot_state.py — 팔 명령이 로봇에 먹는지 진단 (읽기 전용: 조회 API + DDS 구독만, 명령 송신 없음)

사용 (arm_server 가 떠 있는 상태에서 다른 터미널):
  ROBOT=h2 source activate_tv.sh
  ROBOT_CHECK=1 ROBOT=h2 python tools/robot_state.py

출력
  1) LocoClient 조회: GetFsmId(7001) / GetFsmMode(7002) / GetArmSdkStatus(7007) / GetAvailableFsmIds(7008)
     — 실기 응답 여부 자체도 확인 대상 (FACTS.md "실기 응답 여부는 확인 필요")
  2) rt/arm_sdk 구독: arm_server 가 보내는 명령이 DDS 에 실제로 나가는지, 주기, weight(슬롯 weight_slot) 값
  3) 팔·허리 슬롯별  명령 q (rt/arm_sdk)  vs  실측 q (rt/lowstate)  [deg]
     명령과 실측 차이가 계속 크면 → 로봇이 arm_sdk 명령을 받아들이지 않는 상태
"""
import os
import sys
import time

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env   # ROBOT 미지정 / robot.yaml 없음이면 종료, 로봇별 SDK 경로 설정

import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState

J = robot_env.JOINTS
WEIGHT_SLOT = int(J["weight_slot"])
SLOTS = [int(s) for s in J["waist"]] + [int(s) for s in J["arm"]]
NAME = {int(v): k.replace("_joint", "") for k, v in J["map"].items()}
LISTEN_SEC = 3.0


def loco_queries():
    print("== 1) LocoClient 조회 (읽기 전용) ==")
    try:
        LocoClient = robot_env.loco_client_class()
    except ImportError as e:
        print(f"  LocoClient 로드 실패: {e}")
        return
    c = LocoClient()
    c.SetTimeout(3.0)
    c.Init()
    for name in ("GetFsmId", "GetFsmMode", "GetArmSdkStatus", "GetAvailableFsmIds"):
        fn = getattr(c, name, None)
        if fn is None:
            print(f"  {name:20s}: (이 SDK 에 없음)")
            continue
        try:
            print(f"  {name:20s}: {fn()}")
        except Exception as e:  # noqa: BLE001 — 조회 실패도 진단 정보
            print(f"  {name:20s}: 오류 {e!r}")
    print("  (반환값 첫 번째 = code. 0 이 아니면 로봇이 이 API 에 응답하지 않은 것)")


def main():
    robot_env.dds_init()
    print(f"[robot_state] ROBOT={robot_env.ROBOT}  DDS 도메인 {robot_env.DDS_DOMAIN}  weight 슬롯 {WEIGHT_SLOT}\n")

    loco_queries()

    cmd = {"n": 0, "msg": None}

    def on_cmd(m):
        cmd["n"] += 1
        cmd["msg"] = m

    sub_cmd = ChannelSubscriber("rt/arm_sdk", hg_LowCmd)
    sub_cmd.Init(on_cmd, 10)
    sub_st = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub_st.Init()

    print(f"\n== 2) rt/arm_sdk 구독 {LISTEN_SEC:.0f}초 ==")
    time.sleep(LISTEN_SEC)
    n, m = cmd["n"], cmd["msg"]
    if m is None:
        print("  ❌ rt/arm_sdk 수신 0건 — arm_server 가 명령을 내보내지 않음 (arm_server 로그 앞부분 확인)")
    else:
        print(f"  수신 {n}건 ({n / LISTEN_SEC:.0f} Hz),  weight = motor_cmd[{WEIGHT_SLOT}].q = {m.motor_cmd[WEIGHT_SLOT].q:.3f},"
              f"  LowCmd mode_pr = {m.mode_pr}, mode_machine = {m.mode_machine}")

    st = None
    t_end = time.time() + 2.0
    while st is None and time.time() < t_end:
        st = sub_st.Read(0.5)
    if st is None:
        print("  ❌ rt/lowstate 수신 없음")
        return
    print(f"  lowstate mode_machine = {st.mode_machine}")

    print("\n== 3) 명령 vs 실측 [deg] ==")
    print(" slot  name                         mode   kp     kd    명령 q   실측 q    차이")
    for s in SLOTS:
        qs = np.degrees(float(st.motor_state[s].q))
        if m is not None:
            c = m.motor_cmd[s]
            qc = np.degrees(float(c.q))
            print(f" {s:4d}  {NAME.get(s, ''):28s} {int(c.mode):3d}  {float(c.kp):6.1f} {float(c.kd):5.1f}"
                  f"  {qc:+7.1f}  {qs:+7.1f}  {qc - qs:+6.1f}")
        else:
            print(f" {s:4d}  {NAME.get(s, ''):28s}    -      -     -        -    {qs:+7.1f}")


if __name__ == "__main__":
    main()
