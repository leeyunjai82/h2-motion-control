#!/usr/bin/env python3
"""
arm_sdk_test.py — rt/arm_sdk 로 팔 관절 하나만 조금 움직여 보고, 로봇이 따라오는지 판정 (단독 실행)

  LowCmd.mode_machine 은 robot.yaml lowcmd.mode_machine (H2: lowstate = xr_teleoperate 방식).
  arm_server / simulator 는 꺼 둔 상태에서 실행 (rt/arm_sdk 를 두 곳에서 보내면 안 됨).

  ROBOT_CHECK=1 ROBOT=h2 ./start_fsm.sh no-bal        # FSM 4 (FixStand)
  ROBOT=h2 source activate_tv.sh
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py               # 이 repo(robot_arm.py) 방식, 슬롯 15 +5°
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --mm 0        # LowCmd.mode_machine 0 (이전 방식) 비교
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --style sdk   # 공식 SDK 예제 구성
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --allow-fsm 601  # 리모컨 운동제어 모드 등 목록 밖 FSM
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --enable      # EnableArmSDK → 시험 → DisableArmSDK (공식 예제 절차)
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --enable --style sdk --slot 30 --deg 10   # 머리 yaw (29 = pitch)
  ROBOT_CHECK=1 ROBOT=h2 python utils/arm_sdk_test.py --enable --style official --slot 30 --deg 10   # 공식 예제와 똑같이 (머리만 움직임)
  안전: 시험 슬롯 외 팔 슬롯이 시작 대비 8°, 허리가 3° 넘게 움직이면 즉시 weight 반납

  --style sdk  : unitree_sdk2_python h2_arm_sdk_dds_example.py 와 같은 구성
                 팔 14 슬롯 kp 80 / kd 1.5 + 머리 29/30 kp 30 / kd 1.0, mode 는 건드리지 않음(0).
                 (허리는 현재각 유지 robot.yaml kp_waist / kd_waist 추가)
  --style official : 공식 예제와 메시지 구성까지 똑같이 — sdk 와의 차이 3 가지를 없앰
                 ① 허리 12–14 명령 안 함 (kp 0, 예제와 같음) ② LowCmd mode_machine 0 · mode_pr 0 (예제 기본값, --mm 으로 바꿀 수 있음)
                 ③ weight 를 첫 주기에 바로 1.0 (예제 Stage 1 과 같음, 목표 = 현재각이라 팔이 튀지 않음)
                 팔은 0 자세로 보내지 않고 현재각 유지 (예제 Stage 1 의 0 자세는 2026-10-07 실기에서 팔이 몸에 부딪힘)
                 --enable 이면 EnableArmSDK 전후로 내장 팔 서비스(g1_arm_example)·ai_sport 상태도 출력
  --style ours : robot_arm.py 와 같은 구성 — init_slots(0–30) 전부 mode 1 + robot.yaml gains, q = 현재각

순서 (50 Hz, SDK 예제와 같은 주기)
  1) weight 0 → 1 (1.0 s), 모든 목표 = 시작 시 실측각   (official: 첫 주기 1.0 → 0.5 s 유지)
  2) 시험 슬롯만 +deg (1.0 s) → 유지 (1.5 s) → 원위치 (1.0 s)
  3) weight 1 → 0 (1.0 s)   — Ctrl+C 시에도 weight 를 0 으로 내리고 끝냄
판정: 유지 구간 끝 실측이 시작 대비 deg 의 절반 이상 움직였으면 "따라옴"
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # ROBOT / robot.yaml / 로봇별 SDK 경로

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

J = robot_env.JOINTS
G = robot_env.CFG["gains"]
ARM = [int(s) for s in J["arm"]]
WAIST = [int(s) for s in J["waist"]]
HEAD = [int(s) for s in (J.get("head") or [])]
INIT = [int(s) for s in J["init_slots"]]
WRIST = {int(s) for s in J["wrist"]}
WEAK = {int(s) for s in J["weak"]}
WEIGHT = int(J["weight_slot"])
NAME = {int(v): k.replace("_joint", "") for k, v in J["map"].items()}
ARM_SDK_FSM = {4, 703}     # SDK master 814556d h2_arm_sdk_dds_example.py ARM_SDK_SUPPORTED_FSM_IDS
DT = 0.02                  # SDK 예제 control_dt_
MAX_DEG = 10.0
WATCH_DEG = 8.0            # 시험 슬롯 외 팔 슬롯이 시작 대비 이만큼 움직이면 즉시 weight 반납 (예상 밖 움직임)
WATCH_WAIST_DEG = 3.0      # 허리는 더 작게 (official 은 허리 kp 0 을 보냄 — 실기에서는 허리에 arm_sdk 가 안 닿지만 만일을 위해)
WATCH_SERVICES = ("g1_arm_example", "ai_sport")   # 내장 팔 서비스(이름이 G1 그대로) · 운동제어 — robot_services.py 목록 기준


def print_services(tag):
    """EnableArmSDK 가 내장 팔 서비스를 끄는지 보려고 상태만 읽음 (robot_state RPC, 명령 없음)."""
    try:
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        rsc = RobotStateClient()
        rsc.SetTimeout(3.0)
        rsc.Init()
        code, lst = rsc.ServiceList()
        if code != 0 or lst is None:
            print(f"[test] 서비스 상태 ({tag}): ServiceList 실패 code {code}")
            return
        m = {x.name: x.status for x in lst}
        txt = ", ".join(f"{n} {({0: '켜짐', 1: '꺼짐'}).get(m.get(n), m.get(n))}" for n in WATCH_SERVICES)
        print(f"[test] 서비스 상태 ({tag}): {txt}")
    except Exception as e:   # 조회 실패는 시험에 영향 없음
        print(f"[test] 서비스 상태 ({tag}) 조회 실패: {e}")


def ours_gain(s):
    """robot_arm.py 와 같은 슬롯별 게인."""
    if s in HEAD:
        return G["kp_head"], G["kd_head"]
    if s in WRIST:
        return G["kp_wrist"], G["kd_wrist"]
    if s in WAIST:
        return G["kp_waist"], G["kd_waist"]
    if s in WEAK:
        return G["kp_low"], G["kd_low"]
    return G["kp_high"], G["kd_high"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slot", type=int, default=ARM[0],
                    help=f"팔 슬롯 {ARM[0]}~{ARM[-1]}, 허리 {WAIST} 또는 머리 {HEAD} (기본 {ARM[0]})")
    ap.add_argument("--deg", type=float, default=5.0, help=f"움직일 각도 (|deg| ≤ {MAX_DEG})")
    ap.add_argument("--style", choices=("sdk", "official", "ours"), default="ours")
    ap.add_argument("--allow-fsm", type=int, action="append", default=[],
                    help="지원 목록 {4, 703} 밖의 FSM 도 허용 (예: 리모컨으로 들어간 운동제어 모드의 FSM ID — robot_state.py 로 확인)")
    ap.add_argument("--enable", action="store_true",
                    help="시작 전 EnableArmSDK(7109), 끝나면 DisableArmSDK — 공식 예제와 같은 절차 (결과 코드 0 아니면 중단)")
    ap.add_argument("--mm", default=None, help='LowCmd.mode_machine: "lowstate" 또는 숫자 (기본: robot.yaml lowcmd.mode_machine)')
    a = ap.parse_args()
    if a.slot not in ARM and a.slot not in WAIST and a.slot not in HEAD:
        sys.exit(f"❌ --slot 은 팔 {ARM}, 허리 {WAIST}, 머리 {HEAD} 만")
    if a.slot in WAIST and abs(a.deg) > 5.0:
        sys.exit("❌ 허리는 |deg| ≤ 5 (넘어짐 위험)")
    if not (0 < abs(a.deg) <= MAX_DEG):
        sys.exit(f"❌ --deg 는 0 < |deg| ≤ {MAX_DEG}")

    robot_env.dds_init()
    st = {"msg": None}
    sub = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub.Init(lambda m: st.__setitem__("msg", m), 10)
    t_end = time.time() + 3.0
    while st["msg"] is None and time.time() < t_end:
        time.sleep(0.05)
    if st["msg"] is None:
        sys.exit("❌ rt/lowstate 수신 없음 — 명령하지 않음")
    ok, why = robot_env.check_identity(st["msg"])
    print(why)
    if not ok:
        sys.exit(3)

    # FSM / arm sdk 상태 (실기만 — 시뮬 fake_robot 에는 조회 API 없음)
    if not robot_env.SIM:
        c = robot_env.loco_client_class()()
        c.SetTimeout(3.0)
        c.Init()
        code, fsm = c.GetFsmId()
        print(f"[test] GetFsmId = ({code}, {fsm}),  GetArmSdkStatus = {c.GetArmSdkStatus()}")
        if code != 0 or fsm not in (ARM_SDK_FSM | set(a.allow_fsm)):
            sys.exit(f"❌ FSM {fsm} — arm_sdk 지원 FSM {sorted(ARM_SDK_FSM)} 에서만 실행 "
                     f"(ROBOT_CHECK=1 ROBOT={robot_env.ROBOT} ./start_fsm.sh no-bal)")
    loco = c if not robot_env.SIM else None
    if a.enable and loco is not None:
        print_services("EnableArmSDK 전")
        ret = loco.EnableArmSDK()
        print(f"[test] EnableArmSDK → {ret}  (GetArmSdkStatus = {loco.GetArmSdkStatus()})")
        print_services("EnableArmSDK 후")
        if ret != 0:
            sys.exit("❌ EnableArmSDK 실패 — 명령하지 않음 (SDK 814556d 필요: ROBOT=h2 source activate_tv.sh)")

    pub = ChannelPublisher("rt/arm_sdk", hg_LowCmd)
    pub.Init()
    crc = CRC()
    cmd = unitree_hg_msg_dds__LowCmd_()
    # LowCmd 헤더 — robot_arm.py 와 같은 규칙 (robot.yaml lowcmd.mode_machine), --mm 으로 바꿔 비교 가능
    _mm = a.mm if a.mm is not None else 0 if a.style == "official" else (robot_env.CFG.get("lowcmd") or {}).get("mode_machine", 0)
    cmd.mode_pr = 0
    cmd.mode_machine = int(st["msg"].mode_machine) if _mm == "lowstate" else int(_mm)

    q_all = np.array([float(st["msg"].motor_state[i].q) for i in range(len(st["msg"].motor_state))])
    q0 = float(q_all[a.slot])
    if a.style == "official":
        slots = ARM + HEAD          # 허리 없음, mode 0 (예제와 같음)
        for s in slots:
            kp, kd = (80.0, 1.5) if s in ARM else (30.0, 1.0)
            cmd.motor_cmd[s].kp, cmd.motor_cmd[s].kd = kp, kd
    elif a.style == "sdk":
        slots = ARM + HEAD + WAIST
        for s in slots:
            kp, kd = (80.0, 1.5) if s in ARM else (30.0, 1.0) if s in HEAD else (G["kp_waist"], G["kd_waist"])
            cmd.motor_cmd[s].kp, cmd.motor_cmd[s].kd = kp, kd
    else:
        slots = INIT
        for s in slots:
            cmd.motor_cmd[s].mode = 1
            cmd.motor_cmd[s].kp, cmd.motor_cmd[s].kd = ours_gain(s)
    for s in slots:
        cmd.motor_cmd[s].q, cmd.motor_cmd[s].dq, cmd.motor_cmd[s].tau = float(q_all[s]), 0.0, 0.0

    print(f"[test] LowCmd mode_machine = {cmd.mode_machine} ({_mm})")
    print(f"[test] style={a.style}  슬롯 {a.slot} ({NAME.get(a.slot, '')})  시작 {np.degrees(q0):+.1f}° → "
          f"{np.degrees(q0) + a.deg:+.1f}°  (명령 슬롯 {len(slots)}개)")

    target = q0 + np.radians(a.deg)
    phases = [("weight 0→1", 1.0), ("이동", 1.0), ("유지", 1.5), ("복귀", 1.0), ("weight 1→0", 1.0)]
    if a.style == "official":
        phases[0] = ("weight 1", 0.5)      # 예제처럼 첫 주기부터 1.0
    weight = 0.0
    held = None

    def send(w, q_slot):
        cmd.motor_cmd[WEIGHT].q = w
        cmd.motor_cmd[a.slot].q = q_slot
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    watch = [s_ for s_ in ARM + WAIST if s_ != a.slot]
    q_watch0 = {s_: float(q_all[s_]) for s_ in watch}

    def check_watch():
        m = st["msg"]
        for s_ in watch:
            d = np.degrees(float(m.motor_state[s_].q) - q_watch0[s_])
            if abs(d) > (WATCH_WAIST_DEG if s_ in WAIST else WATCH_DEG):
                raise RuntimeError(f"슬롯 {s_} ({NAME.get(s_, '')}) 가 시작 대비 {d:+.1f}° 움직임 — 예상 밖")

    try:
        ms0 = st["msg"].motor_state[a.slot]
        print(f"[test] 슬롯 {a.slot} 시작 상태: mode {ms0.mode}, 온도 {list(ms0.temperature)}, motorstate {ms0.motorstate}, "
              f"tau {ms0.tau_est:+.2f} Nm")
        if a.slot in HEAD:
            kp_s = float(cmd.motor_cmd[a.slot].kp)
            print(f"[test] 명령이 모터에 닿으면 유지 구간 토크 ≈ kp {kp_s:.0f} × {abs(np.radians(a.deg)):.3f} rad "
                  f"= {kp_s * abs(np.radians(a.deg)):.1f} Nm 근처까지 오름 (안 닿으면 0 근처)")
        for name, dur in phases:
            n = int(round(dur / DT))
            tau_pk, dq_pk = 0.0, 0.0
            for i in range(1, n + 1):
                r = i / n
                q_slot = q0
                if name == "weight 0→1":
                    weight = r
                elif name == "weight 1":
                    weight = 1.0
                elif name == "이동":
                    q_slot = q0 + (target - q0) * r
                elif name == "유지":
                    q_slot = target
                elif name == "복귀":
                    q_slot = target + (q0 - target) * r
                elif name == "weight 1→0":
                    weight = 1.0 - r
                send(weight, q_slot)
                time.sleep(DT)
                check_watch()
                ms = st["msg"].motor_state[a.slot]
                tau_pk, dq_pk = max(tau_pk, abs(ms.tau_est)), max(dq_pk, abs(ms.dq))
            meas = float(st["msg"].motor_state[a.slot].q)
            if name == "유지":
                held = meas
            # 토크가 안 오르면 명령이 모터에 안 닿는 것 (FSM 이 그 관절을 arm_sdk 에 안 넘김), 오르는데 안 움직이면 막힘·다른 제어와 충돌
            print(f"  {name:10s} 끝: 명령 {np.degrees(q_slot):+6.1f}°  실측 {np.degrees(meas):+6.1f}°  weight {weight:.2f}"
                  f"   |tau| 최대 {tau_pk:5.2f} Nm  |dq| 최대 {dq_pk:5.2f} rad/s")
    except (KeyboardInterrupt, RuntimeError) as e:
        print(f"\n[test] 중단 ({e or 'Ctrl+C'}) — weight 를 0 으로 내림")
        for i in range(25):
            weight = max(0.0, weight - 0.04)
            send(weight, float(cmd.motor_cmd[a.slot].q))
            time.sleep(DT)
        send(0.0, float(cmd.motor_cmd[a.slot].q))
        return
    finally:
        if a.enable and loco is not None:
            print(f"[test] DisableArmSDK → {loco.DisableArmSDK()}")

    moved = np.degrees(held - q0)
    verdict = "✓ 따라옴" if abs(moved) >= abs(a.deg) * 0.5 and np.sign(moved) == np.sign(a.deg) else "✗ 안 따라옴"
    print(f"\n[test] 결과: 명령 {a.deg:+.1f}° / 실측 변화 {moved:+.1f}°  →  {verdict}  (style={a.style})")


if __name__ == "__main__":
    main()
