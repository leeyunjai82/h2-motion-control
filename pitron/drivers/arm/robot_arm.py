import numpy as np
import threading
import time
from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import ( LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

import logging
logging.basicConfig(level=logging.INFO)
logger_mp = logging.getLogger(__name__)

kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

# ---- 로봇별 값 (robots/<ROBOT>/robot.yaml joints / gains) ----
from pitron.core import robot_env
_J = robot_env.JOINTS
_G = robot_env.CFG["gains"]
G1_29_Num_Motors = int(_J["motor_slots"])           # 이름은 기존 호환용 (H2 도 같은 값을 쓴다)
ARM_SLOTS    = [int(i) for i in _J["arm"]]          # 팔 14축, IK q 순서
WAIST_SLOTS  = [int(i) for i in _J["waist"]]        # yaw, roll, pitch
HEAD_SLOTS   = {int(i) for i in (_J.get("head") or [])}
WEIGHT_SLOT  = int(_J["weight_slot"])               # arm_sdk weight (motor_cmd[WEIGHT_SLOT].q)


def _arm_urdf_limits():
    """ARM_SLOTS 순서의 URDF 관절 한계 (lo, hi) [rad] — robot.yaml joints.map(URDF 이름 → 슬롯) 로 찾음. 못 읽으면 None."""
    try:
        import pinocchio as pin
        m = pin.buildModelFromUrdf(robot_env.URDF_PATH)
        name_of = {int(v): k for k, v in (_J.get("map") or {}).items()}
        idx = [m.joints[m.getJointId(name_of[s])].idx_q for s in ARM_SLOTS]
        return np.asarray(m.lowerPositionLimit)[idx].copy(), np.asarray(m.upperPositionLimit)[idx].copy()
    except Exception as e:      # noqa: BLE001 — 한계를 못 읽어도 제어는 계속 (경고만)
        logging.getLogger(__name__).warning(f"URDF 팔 관절 한계를 못 읽음 — 팔 목표를 자르지 않음: {e}")
        return None


# 팔 목표를 URDF 관절 한계 안으로 자름 (모든 팔 명령 공통 — IK 는 원래 한계 안, 관절 직접 명령·모션 파일 대비)
ARM_LIMITS = _arm_urdf_limits()
INIT_SLOTS   = [int(i) for i in _J["init_slots"]]   # 시작 시 mode/kp/kd/q 설정 슬롯
WRIST_SLOTS  = {int(i) for i in _J["wrist"]}
WEAK_SLOTS   = {int(i) for i in _J["weak"]}
# 허리를 시작 시 현재각으로만 잡고 이후 명령하지 않음 (xr_teleoperate H2_ArmController 와 같음, H2 robot.yaml true)
WAIST_HOLD   = bool(_J.get("waist_hold_initial", False))
ARM_VEL_LIMIT = float(_G.get("arm_velocity_limit", 20.0))   # 팔 관절 속도 제한 [rad/s] (H2 robot.yaml 30 = 공식 기본)
# 머리 [pitch, yaw] 슬롯 (robot.yaml joints.map head_pitch_joint / head_yaw_joint). 703 에서는 arm_sdk 로 안 움직임 (FACTS.md).
# 명령은 joints.head_range_deg 안으로 자르고, 송신 루프가 gains.head_velocity_limit 속도로 목표까지 옮김
HEAD_ORDER   = [int(_J["map"][k]) for k in ("head_pitch_joint", "head_yaw_joint") if k in (_J.get("map") or {})]
_HR          = _J.get("head_range_deg") or {}
HEAD_RANGE   = (np.radians([_HR["pitch"], _HR["yaw"]]) if HEAD_ORDER and "pitch" in _HR and "yaw" in _HR else None)
HEAD_VEL_LIMIT = float(_G.get("head_velocity_limit", 1.0))   # [rad/s]

class MotorState:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0

class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data

class ArmController:
    """양팔(+허리) arm_sdk/lowcmd 송신 — 로봇별 값은 robots/<ROBOT>/robot.yaml (joints, gains, lowcmd, sdk).
    G1 = xr_teleoperate G1_29_ArmController, H2 = xr_teleoperate 817fb00 H2_ArmController 와 같은 슬롯·게인·헤더
    (+ H2 EnableArmSDK — 실기에서 이것 없이는 팔이 안 움직임)."""
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger_mp.info(f"ArmController({robot_env.ROBOT}) 초기화 중...")

        # 제어 타겟 초기화
        self.q_target = np.zeros(14)        # 양팔 14축
        self.tauff_target = np.zeros(14)    # 양팔 토크 피드포워드
        self.waist_q_target = np.zeros(3)   # 허리 3축 (Yaw, Roll, Pitch)

        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode

        # 게인 설정 (robot.yaml gains)
        self.kp_high = float(_G["kp_high"])
        self.kd_high = float(_G["kd_high"])
        self.kp_low = float(_G["kp_low"])
        self.kd_low = float(_G["kd_low"])
        self.kp_wrist = float(_G["kp_wrist"])
        self.kd_wrist = float(_G["kd_wrist"])
        self.kp_waist = float(_G["kp_waist"])
        self.kd_waist = float(_G["kd_waist"])
        self.kp_head = _G.get("kp_head")
        self.kd_head = _G.get("kd_head")

        self.arm_velocity_limit = ARM_VEL_LIMIT
        self.control_dt = 1.0 / 250.0 # 250Hz

        # arm_sdk 가중치 (0.0=loco 소유 / 1.0=arm_sdk 소유). motion_mode 에서만 의미.
        # FSM 501(Regular)에서는 weight=1 유지 상태로도 보행 가능(실기 검증).
        self.arm_weight = 1.0 if motion_mode else 0.0

        self._speed_gradual_max = False
        self._gradual_start_time = None

        # IMU 데이터 (subscribe 루프에서 갱신)
        self.imu_rpy   = np.zeros(3)  # [roll, pitch, yaw] rad
        self.imu_accel = np.zeros(3)  # [x, y, z] m/s²
        self.imu_gyro  = np.zeros(3)  # [x, y, z] rad/s

        # DDS 초기화
        if self.simulation_mode:
            ChannelFactoryInitialize(1)
        else:
            robot_env.dds_init()

        # Publisher / Subscriber 설정
        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)

        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self._last_lowstate_msg = None

        # 수신 스레드 시작
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # 데이터 수신 대기
        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("DDS 데이터 수신 대기 중...")
        logger_mp.info("DDS 연결 성공.")

        # 연결된 로봇 확인 (robot.yaml identity) — 다르면 송신 스레드를 띄우기 전에 중단
        ok, why = robot_env.check_identity(self._last_lowstate_msg)
        logger_mp.warning(why) if ok else logger_mp.error(why)
        if not ok:
            raise RuntimeError(why)

        # 메시지 객체 생성
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        # LowCmd.mode_machine — robot.yaml lowcmd.mode_machine
        #   0 / "lowstate" = 수신한 rt/lowstate.mode_machine 을 그대로 (xr_teleoperate H2_ArmController 방식, H2 robot.yaml)
        _mm = (robot_env.CFG.get("lowcmd") or {}).get("mode_machine", 0)
        self.msg.mode_machine = int(self._last_lowstate_msg.mode_machine) if _mm == "lowstate" else int(_mm)
        logger_mp.info(f"LowCmd mode_machine = {self.msg.mode_machine} (robot.yaml lowcmd.mode_machine: {_mm})")

        # 현재 상태 읽기 및 초기 타겟 설정
        current_all_q = self.get_current_motor_q()
        self.q_target = self.get_current_dual_arm_q()
        self.waist_q_target = current_all_q[WAIST_SLOTS]   # robot.yaml joints.waist (H2: 14, 12, 13 = yaw, roll, pitch)
        self.head_q_target = current_all_q[HEAD_ORDER] if HEAD_ORDER else np.zeros(0)   # [pitch, yaw] — 시작 각도 유지
        self.head_q_cmd = self.head_q_target.copy()       # 속도 제한을 거친 실제 송신값

        logger_mp.info("모든 관절 고정 설정 중 (팔/허리 제외)...")
        arm_indices = set(ARM_SLOTS)
        waist_indices = set(WAIST_SLOTS)

        for id in INIT_SLOTS:
            self.msg.motor_cmd[id].mode = 1
            if id in HEAD_SLOTS:
                self.msg.motor_cmd[id].kp = float(self.kp_head)
                self.msg.motor_cmd[id].kd = float(self.kd_head)
            elif id in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            elif id in waist_indices:
                self.msg.motor_cmd[id].kp = self.kp_waist
                self.msg.motor_cmd[id].kd = self.kd_waist
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high

            self.msg.motor_cmd[id].q = current_all_q[id]

        logger_mp.info("관절 고정 완료.")

        # arm_sdk 활성화 (robot.yaml sdk.enable_arm_sdk — H2: EnableArmSDK 7109 없이는 rt/arm_sdk 를 반영하지 않음, 실기 2026-10-07)
        self._loco = None
        if self.motion_mode and not self.simulation_mode and not robot_env.SIM \
                and (robot_env.CFG.get("sdk") or {}).get("enable_arm_sdk"):
            self._loco = robot_env.loco_client_class()()
            self._loco.SetTimeout(5.0)
            self._loco.Init()
            ret = self._loco.EnableArmSDK()
            logger_mp.warning(f"EnableArmSDK → {ret}")
            if ret != 0:
                raise RuntimeError(f"EnableArmSDK 실패 (code {ret}) — 송신하지 않음")

        # 송신 스레드 시작
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info(f"ArmController({robot_env.ROBOT}) 초기화 완료!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self._last_lowstate_msg = msg   # 로봇 확인용 (mode_machine)
                # 관절 상태 저장
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)

                # IMU 데이터 저장 (pelvis IMU)
                self.imu_rpy   = np.array(msg.imu_state.rpy)
                self.imu_accel = np.array(msg.imu_state.accelerometer)
                self.imu_gyro  = np.array(msg.imu_state.gyroscope)

            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self):
        err_streak = 0
        while True:
            start_time = time.time()

            # weight 매 주기 반영 (hold/release 램프가 이 값을 바꾼다)
            if self.motion_mode:
                self.msg.motor_cmd[WEIGHT_SLOT].q = self.arm_weight

            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target
                waist_q_target   = self.waist_q_target
                head_q_target    = self.head_q_target

            # 1. 팔 제어 업데이트
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)

            for idx, id in enumerate(ARM_SLOTS):
                self.msg.motor_cmd[id].q   = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq  = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # 2. 허리 제어 업데이트 (robot.yaml joints.waist — H2 는 시작 각도 유지)
            for i, joint_idx in enumerate(WAIST_SLOTS):
                self.msg.motor_cmd[joint_idx].q   = waist_q_target[i]
                self.msg.motor_cmd[joint_idx].dq  = 0
                self.msg.motor_cmd[joint_idx].tau = 0

            # 3. 머리 (H2 29/30) — 목표까지 head_velocity_limit 로 이동
            if HEAD_ORDER:
                step = HEAD_VEL_LIMIT * self.control_dt
                self.head_q_cmd = self.head_q_cmd + np.clip(head_q_target - self.head_q_cmd, -step, step)
                for i, joint_idx in enumerate(HEAD_ORDER):
                    self.msg.motor_cmd[joint_idx].q   = self.head_q_cmd[i]
                    self.msg.motor_cmd[joint_idx].dq  = 0
                    self.msg.motor_cmd[joint_idx].tau = 0

            # 4. CRC 계산 및 전송 (예외 시에도 루프 유지 - 송신 중단은 낙상 위험)
            try:
                self.msg.crc = self.crc.Crc(self.msg)
                self.lowcmd_publisher.Write(self.msg)
                err_streak = 0
            except Exception as e:
                err_streak += 1
                logger_mp.error(f"arm_sdk 송신 실패({err_streak}): {e}")
                if self.motion_mode and err_streak >= 250:  # 약 1초 연속 실패
                    logger_mp.error("송신 연속 실패 - weight 비상 반납 시도")
                    self.arm_weight = max(0.0, self.arm_weight - 0.02)

            # 속도 점진적 증가 처리
            if self._speed_gradual_max:
                t_elapsed = start_time - self._gradual_start_time
                _top = max(30.0, ARM_VEL_LIMIT)          # H2: 30 그대로 (공식은 점진 증가 없음)
                self.arm_velocity_limit = ARM_VEL_LIMIT + ((_top - ARM_VEL_LIMIT) * min(1.0, t_elapsed / 5.0))

            current_time = time.time()
            sleep_time = max(0, (self.control_dt - (current_time - start_time)))
            time.sleep(sleep_time)

    # ==================== 제어 메서드 ====================

    def ctrl_dual_arm(self, q_target, tauff_target):
        if ARM_LIMITS is not None:
            q = np.clip(np.asarray(q_target, dtype=float), ARM_LIMITS[0], ARM_LIMITS[1])
            over = np.abs(q - np.asarray(q_target, dtype=float))
            if over.max() > np.radians(0.5) and time.time() - getattr(self, "_limit_warn_t", 0.0) > 2.0:
                self._limit_warn_t = time.time()
                k = int(over.argmax())
                logger_mp.warning(f"팔 목표가 URDF 관절 한계 밖 — 잘라서 보냄 (슬롯 {ARM_SLOTS[k]}: "
                                  f"{np.degrees(q_target[k]):.1f}° → {np.degrees(q[k]):.1f}°)")
            q_target = q
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def ctrl_waist(self, q_target):
        """허리 관절(Yaw, Roll, Pitch) 목표 각도 설정"""
        if len(q_target) != 3:
            return
        if WAIST_HOLD:      # robot.yaml joints.waist_hold_initial — 시작 시 각도 유지, 명령 무시 (H2 공식과 같음)
            if not getattr(self, "_waist_hold_logged", False):
                logger_mp.warning("허리 명령 무시 (robot.yaml joints.waist_hold_initial: 시작 시 각도 유지)")
                self._waist_hold_logged = True
            return
        with self.ctrl_lock:
            self.waist_q_target = np.array(q_target)

    def has_head(self):
        """머리 명령 가능 여부 (robot.yaml joints.map 에 머리 관절 + joints.head_range_deg)."""
        return HEAD_RANGE is not None

    def ctrl_head(self, q_target):
        """머리 [pitch, yaw] 목표 [rad] (pitch + = 숙임, yaw + = 왼쪽). head_range_deg 로 자른 값을 돌려줌, 머리 없으면 None.
        실제 이동은 송신 루프가 head_velocity_limit 로 천천히."""
        if HEAD_RANGE is None:
            return None
        q = np.clip(np.asarray(q_target, dtype=float), HEAD_RANGE[:, 0], HEAD_RANGE[:, 1])
        with self.ctrl_lock:
            self.head_q_target = q
        return q.copy()

    def get_head_target(self):
        with self.ctrl_lock:
            return self.head_q_target.copy()

    # ==================== arm_sdk 제어권 (hold/release) ====================

    def disable_arm_sdk(self):
        """EnableArmSDK 를 호출했으면 DisableArmSDK (weight 를 0 으로 내린 뒤 호출할 것)."""
        if getattr(self, "_loco", None) is None:
            return None
        ret = self._loco.DisableArmSDK()
        logger_mp.warning(f"DisableArmSDK → {ret}")
        self._loco = None
        return ret

    def get_weight(self):
        return float(self.arm_weight)

    def sync_targets_to_current(self):
        """팔/허리 타겟을 현재 실측각으로 1회 동기화 (hold 진입 시 튐 방지).
        주의: 매 주기 추종은 금지 - 복원토크가 사라져 굽는다. 1회만."""
        all_q = self.get_current_motor_q()
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q()
            self.tauff_target = np.zeros(14)
            if not WAIST_HOLD:
                self.waist_q_target = all_q[WAIST_SLOTS].copy()
            if HEAD_ORDER:
                self.head_q_target = all_q[HEAD_ORDER].copy()
                self.head_q_cmd = self.head_q_target.copy()

    def ramp_weight(self, dst, duration=2.0):
        """weight 를 현재값에서 dst 까지 duration 초 동안 선형 램프 (블로킹)."""
        dst = float(np.clip(dst, 0.0, 1.0))
        src_w = float(self.arm_weight)
        n = max(1, int(duration / 0.02))
        for i in range(1, n + 1):
            self.arm_weight = src_w + (dst - src_w) * i / n
            time.sleep(0.02)
        self.arm_weight = dst

    # ==================== 상태 조회 메서드 ====================

    def get_current_motor_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in range(G1_29_Num_Motors)])

    def get_current_dual_arm_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in ARM_SLOTS])

    def get_current_dual_arm_dq(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in ARM_SLOTS])

    # ==================== IMU 조회 메서드 ====================

    def get_imu_rpy(self):
        """pelvis IMU RPY [roll, pitch, yaw] (rad)"""
        return self.imu_rpy.copy()

    def get_imu_roll(self):
        """pelvis IMU roll (rad)"""
        return float(self.imu_rpy[0])

    def get_imu_pitch(self):
        """pelvis IMU pitch (rad)"""
        return float(self.imu_rpy[1])

    def get_imu_yaw(self):
        """pelvis IMU yaw (rad)"""
        return float(self.imu_rpy[2])

    def get_imu_accel(self):
        """pelvis IMU accelerometer [x, y, z] (m/s²)"""
        return self.imu_accel.copy()

    def get_imu_gyro(self):
        """pelvis IMU gyroscope [x, y, z] (rad/s)"""
        return self.imu_gyro.copy()

    def get_waist_q(self):
        """현재 허리 관절각 [yaw, roll, pitch] (rad)"""
        q = self.get_current_motor_q()
        return q[WAIST_SLOTS].copy()

    def get_head_q(self):
        """현재 머리 관절각 [pitch, yaw] (rad), 머리 없으면 빈 배열"""
        q = self.get_current_motor_q()
        return q[HEAD_ORDER].copy() if HEAD_ORDER else np.zeros(0)

    # ==================== 유틸리티 ====================

    def ctrl_dual_arm_go_home(self):
        logger_mp.info("양팔 홈 포지션 이동 시작...")
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
            if not WAIST_HOLD:
                self.waist_q_target = np.zeros(3)

        time.sleep(2.0)

        if self.motion_mode:
            self.ramp_weight(0.0, 1.0)
        logger_mp.info("홈 이동 완료 및 제어권 반납.")

    def speed_gradual_max(self, t=5.0):
        self._gradual_start_time = time.time()
        self._speed_gradual_max = True

    def _Is_weak_motor(self, motor_index):
        return int(motor_index) in WEAK_SLOTS          # robot.yaml joints.weak

    def _Is_wrist_motor(self, motor_index):
        return int(motor_index) in WRIST_SLOTS         # robot.yaml joints.wrist



# 예전 이름 (호환용) — 내용은 위 ArmController
G1_29_ArmController = ArmController

# ---- 예전 G1 관절 번호 enum (arm_controller_wrapper import 호환용 — 제어 코드는 위 robot.yaml 값만 사용) ----
class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28

class G1_29_JointIndex(IntEnum):
    kLeftHipPitch   = 0
    kLeftHipRoll    = 1
    kLeftHipYaw     = 2
    kLeftKnee       = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll  = 5
    kRightHipPitch  = 6
    kRightHipRoll   = 7
    kRightHipYaw    = 8
    kRightKnee      = 9
    kRightAnklePitch = 10
    kRightAnkleRoll  = 11
    kWaistYaw        = 12
    kWaistRoll       = 13
    kWaistPitch      = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28
    kNotUsedJoint0      = 29


