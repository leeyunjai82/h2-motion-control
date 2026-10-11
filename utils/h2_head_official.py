#!/usr/bin/env python3
"""
h2_head_official.py — 공식 H2 arm_sdk 예제를 거의 그대로 돌려 머리 29/30 이 움직이는지 확인 (단독 실행)

  원본: unitree_sdk2_python 814556d example/h2/high_level/h2_arm_sdk_dds_example.py
  원본과 같은 것: DDS 초기화, LocoClient 순서(SetTimeout 5 → GetFsmId → EnableArmSDK → … → DisableArmSDK),
                 EnableArmSDK 뒤에 publisher/subscriber 생성, 첫 lowstate 까지 1 초 단위 대기, RecurrentThread 50 Hz,
                 LowCmd 기본값(mode 0 · mode_pr 0 · mode_machine 0), 팔 kp 80 / kd 1.5, 머리 kp 30 / kd 1.0,
                 weight motor_cmd[31].q 를 Stage 1 첫 주기부터 1.0, 머리 궤적(Stage 1 → 0, Stage 2 → pitch +15° · yaw +20°,
                 Stage 3 → 0, Stage 4 weight 3 초 동안 1 → 0), 각 3 초.
  원본과 다른 것: ① 팔 14 개 목표 = 시작 때 실측각 유지 (원본 Stage 1 의 0 자세는 2026-10-07 실기에서 팔이 몸에 부딪힘)
                 ② 0.5 초마다 머리 명령·실측·토크 출력, 끝에 요약 ③ 팔이 8° / 허리가 3° 넘게 움직이면 바로 Stage 4 (weight 내림)
                 ④ SDK 경로를 robot_env 로 잡음 (tv 환경 기본 SDK f559291 에는 unitree_sdk2py.h2 가 없음)

  arm_server(start_robot.sh) 는 꺼 둘 것 — rt/arm_sdk 를 두 곳에서 보내면 안 됨. 로봇은 FSM 4 또는 703.
  ROBOT=h2 source activate_tv.sh
  ROBOT=h2 python utils/h2_head_official.py [네트워크 인터페이스]

판정: 머리 명령이 모터에 닿으면 Stage 2 끝에서 실측이 목표 쪽으로 움직이고 토크가 수 Nm 까지 오름
      (kp 30 × 15° ≈ 7.9 Nm). 실측 변화 0 · 토크 0 근처면 펌웨어가 arm_sdk 의 머리 슬롯을 받지 않는 것.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # noqa: E402  SDK 814556d 경로 (unitree_sdk2py.h2)

import numpy as np   # noqa: E402

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber   # noqa: E402
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_   # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_   # noqa: E402
from unitree_sdk2py.h2.loco.h2_loco_client import LocoClient   # noqa: E402
from unitree_sdk2py.utils.crc import CRC   # noqa: E402
from unitree_sdk2py.utils.thread import RecurrentThread   # noqa: E402


SMALL_HEAD_YAW = np.deg2rad(20.0)
SMALL_HEAD_PITCH = np.deg2rad(15.0)

ARM_SDK_SUPPORTED_FSM_IDS = {4, 703}

ARM_WATCH_DEG = 8.0
WAIST_WATCH_DEG = 3.0


class H2JointIndex:
    # 공식 문서 「关节电机顺序」(2026-07-30): 허리 12 roll / 13 pitch / 14 yaw (원본 예제의 12 yaw 순서는 틀림 — 명령은 안 보냄)
    WaistRoll = 12
    WaistPitch = 13
    WaistYaw = 14

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    # Head
    HeadPitch = 29
    HeadYaw = 30

    kNotUsedJoint = 31  # NOTE: Weight


ARM_JOINTS = list(range(H2JointIndex.LeftShoulderPitch, H2JointIndex.RightWristYaw + 1))
HEAD_JOINTS = [H2JointIndex.HeadPitch, H2JointIndex.HeadYaw]
WAIST_JOINTS = [H2JointIndex.WaistRoll, H2JointIndex.WaistPitch, H2JointIndex.WaistYaw]


class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 3.0
        self.kp = 80.0
        self.kd = 1.5
        self.head_kp = 30.0
        self.head_kd = 1.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        self.head_target = {H2JointIndex.HeadPitch: SMALL_HEAD_PITCH, H2JointIndex.HeadYaw: SMALL_HEAD_YAW}
        self.upper_body_joints = ARM_JOINTS + HEAD_JOINTS
        self.head_joints = set(HEAD_JOINTS)

        self.q_init = None          # 첫 lowstate 의 실측각 (팔 유지 목표, 감시 기준)
        self.abort = ""
        self.next_log = 0.0
        self.stat = {j: {"dq_max": 0.0, "tau_max": 0.0} for j in HEAD_JOINTS}

    def Init(self):
        # create publisher
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while not self.first_update_low_state:
            time.sleep(1)

        m = self.low_state
        self.q_init = np.array([float(m.motor_state[i].q) for i in range(len(m.motor_state))])
        for j in HEAD_JOINTS:
            ms = m.motor_state[j]
            print(f"[start] 슬롯 {j}: q {np.degrees(ms.q):+6.1f}°  mode {ms.mode}  온도 {list(ms.temperature)}  "
                  f"motorstate {ms.motorstate}  tau {ms.tau_est:+.2f} Nm")
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if not self.first_update_low_state:
            self.first_update_low_state = True

    def SetJointCommand(self, joint, q):
        command = self.low_cmd.motor_cmd[joint]
        command.tau = 0.0
        command.q = q
        command.dq = 0.0
        if joint in self.head_joints:
            command.kp = self.head_kp
            command.kd = self.head_kd
        else:
            command.kp = self.kp
            command.kd = self.kd

    def Watch(self):
        """팔·허리가 예상 밖으로 움직이면 이유를 돌려줌 (빈 문자열 = 정상)."""
        m = self.low_state
        for j in ARM_JOINTS + WAIST_JOINTS:
            d = np.degrees(float(m.motor_state[j].q) - self.q_init[j])
            if abs(d) > (WAIST_WATCH_DEG if j in WAIST_JOINTS else ARM_WATCH_DEG):
                return f"슬롯 {j} 가 시작 대비 {d:+.1f}° 움직임"
        return ""

    def Log(self, stage):
        m = self.low_state
        for j in HEAD_JOINTS:
            ms = m.motor_state[j]
            s = self.stat[j]
            s["dq_max"] = max(s["dq_max"], abs(float(ms.q) - self.q_init[j]))
            s["tau_max"] = max(s["tau_max"], abs(float(ms.tau_est)))
        if self.time_ >= self.next_log:
            self.next_log = self.time_ + 0.5
            txt = "  ".join(
                f"{'pitch' if j == H2JointIndex.HeadPitch else 'yaw'} 명령 {np.degrees(self.low_cmd.motor_cmd[j].q):+6.1f}° "
                f"실측 {np.degrees(m.motor_state[j].q):+6.1f}° tau {m.motor_state[j].tau_est:+5.2f}"
                for j in HEAD_JOINTS)
            print(f"[{self.time_:5.2f}s {stage}] weight {self.low_cmd.motor_cmd[H2JointIndex.kNotUsedJoint].q:.2f}  {txt}")

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        self.time_ += self.control_dt_

        if not self.abort and self.time_ < self.duration_ * 6:
            self.abort = self.Watch()
            if self.abort:
                print(f"[중단] {self.abort} — Stage 4 로 (weight 내림)")
                self.time_ = self.duration_ * 6 + self.control_dt_

        if self.time_ < self.duration_:
            stage = "Stage 1"
            self.low_cmd.motor_cmd[H2JointIndex.kNotUsedJoint].q = 1.0

            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            for joint in ARM_JOINTS:
                self.SetJointCommand(joint, float(self.q_init[joint]))
            for joint in HEAD_JOINTS:
                q = (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.SetJointCommand(joint, q)

        elif self.time_ < self.duration_ * 3:
            stage = "Stage 2"
            ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
            for joint in ARM_JOINTS:
                self.SetJointCommand(joint, float(self.q_init[joint]))
            for joint in HEAD_JOINTS:
                q = ratio * self.head_target[joint] + (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.SetJointCommand(joint, q)

        elif self.time_ < self.duration_ * 6:
            stage = "Stage 3"
            ratio = np.clip((self.time_ - self.duration_ * 3) / (self.duration_ * 3), 0.0, 1.0)
            for joint in ARM_JOINTS:
                self.SetJointCommand(joint, float(self.q_init[joint]))
            for joint in HEAD_JOINTS:
                q = (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.SetJointCommand(joint, q)

        elif self.time_ < self.duration_ * 7:
            stage = "Stage 4"
            # [Stage 4]: release arm_sdk
            ratio = np.clip((self.time_ - self.duration_ * 6) / self.duration_, 0.0, 1.0)
            self.low_cmd.motor_cmd[H2JointIndex.kNotUsedJoint].q = 1.0 - ratio

        else:
            stage = "끝"
            self.done = True

        self.Log(stage)
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

    def Summary(self):
        print("\n[요약] 머리 (시작 대비 최대 변화 / 최대 |tau|)")
        for j in HEAD_JOINTS:
            s = self.stat[j]
            name = "pitch 29" if j == H2JointIndex.HeadPitch else "yaw 30"
            print(f"  {name}: 최대 변화 {np.degrees(s['dq_max']):5.1f}°   최대 |tau| {s['tau_max']:5.2f} Nm")
        moved = max(np.degrees(self.stat[j]["dq_max"]) for j in HEAD_JOINTS)
        if self.abort:
            print(f"  판정 보류 — 중단됨 ({self.abort})")
        elif moved >= 5.0:
            print("  판정: ✓ 머리가 arm_sdk 명령을 따라옴")
        else:
            print("  판정: ✗ 머리 안 따라옴 — 공식 예제 구성 그대로도 명령이 모터에 안 닿음 (펌웨어 쪽, 유니트리 문의)")


if __name__ == "__main__":
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    print("         (arm_server / start_robot.sh 가 꺼져 있어야 함 — rt/arm_sdk 를 두 곳에서 보내면 안 됨)")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    loco_client = LocoClient()
    loco_client.SetTimeout(5.0)
    loco_client.Init()

    ret, fsm_id = loco_client.GetFsmId()
    if ret != 0:
        print(f"Failed to get H2 FSM ID, error code: {ret}")
        sys.exit(1)
    if fsm_id not in ARM_SDK_SUPPORTED_FSM_IDS:
        print(
            f"Current FSM {fsm_id} does not accept rt/arm_sdk commands. "
            f"Switch to one of {sorted(ARM_SDK_SUPPORTED_FSM_IDS)} first."
        )
        sys.exit(1)
    print(f"[info] FSM {fsm_id}")

    ret = loco_client.EnableArmSDK()
    if ret != 0:
        print(f"Failed to enable the external Arm SDK, error code: {ret}")
        sys.exit(1)
    print(f"[info] EnableArmSDK → {ret}")

    exit_code = 0
    custom = None
    try:
        custom = Custom()
        custom.Init()
        custom.Start()

        while True:
            time.sleep(1)
            if custom.done:
                print("Done!")
                break
    except KeyboardInterrupt:
        print("Interrupted; releasing Arm SDK control.")
        if custom is not None and not custom.done:
            # 원본은 바로 Disable — 여기서는 Stage 4 처음(weight 1.0)부터 3 초 동안 내린 뒤 Disable
            custom.abort = custom.abort or "Ctrl+C"
            custom.time_ = max(custom.time_, custom.duration_ * 6)
            t_end = time.time() + custom.duration_ + 0.5
            while not custom.done and time.time() < t_end:
                time.sleep(0.05)
    finally:
        ret = loco_client.DisableArmSDK()
        if ret != 0:
            print(f"Failed to disable the external Arm SDK, error code: {ret}")
            exit_code = 1
        if custom is not None and custom.q_init is not None:
            custom.Summary()

    sys.exit(exit_code)
