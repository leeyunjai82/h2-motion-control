"""
arm_server.py — H2 팔/허리 전용 서버 (포트 50022)

rt/arm_sdk 를 이 프로세스가 단독 점유한다. 팔·허리를 움직이는 모든
프로그램(robot_server, 외부 프로젝트, SLAM 개발자)은 반드시 이 서버의
HTTP API 를 통해서만 지령한다. arm_sdk 직접 송신 프로세스는 이것 하나.

기능
  - hold / release        : arm_sdk weight 램프 (제어권 점유/반납)
  - /joints               : 팔 14축 관절각 보간 (deg)
  - /hands                : IK 좌표 이동 (torso 기준 xyz + 쿼터니언/RPY)
  - /waist                : 허리 3축 (deg)
  - /head  (GET/POST)     : 머리 [pitch, yaw] (deg, H2 29/30) — 목표만 바꾸고 바로 응답 (이동은 송신 루프가 속도 제한)
  - /park                 : 기본 자세 복귀 (허리 중립 + DEFAULT_ARM_DEG)
  - /freeze /stop_motion  : 현재 자세 동결 / 보간 중단 (스트림 세션도 끝냄)
  - /stream/begin|cmd|end : 팔 14축 관절 목표 스트리밍 (physical_ai 정책 추론 30 Hz 용).
                            보간 없이 목표를 바로 바꾸되 세션 속도 제한(max_vel) 안에서만 움직이고,
                            중력 보상 토크(IK 와 같은 rnea)를 같이 보낸다. 세션 동안 팔 잠금(_arm_lock)을 쥐어
                            다른 팔 동작은 409. 0.2 초 지령이 없으면 마지막 목표에서 멈춰 있고(hold),
                            2 초 지령이 없으면 세션을 끝내고 잠금을 푼다.
  - /pose /status         : 실측·타겟·weight 조회
  - 동작 엔드포인트는 완료까지 블로킹 응답 (시퀀스 작성 용이).
    동시에 두 동작 요청이 오면 뒤엣것은 409.

실행
  python arm_server.py          # robot_env.dds_init() — robot_server와 동일
"""

import os
import sys
import json
import time
import uuid
import asyncio
import threading
import numpy as np
from typing import List, Optional

import uvicorn
import pinocchio as pin
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from contextlib import asynccontextmanager

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import robot_env   # ROBOT 미지정/미지원이면 여기서 종료

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import ArmControllerWrapper
from ctrl.robot_arm import ARM_LIMITS

PORT = 50022

# 기본 자세 (robot_server 와 같은 값 — robot.yaml default_arm_deg 한 곳에서 읽음)
# None = 미측정 (실기 모터 번호 확인 모드의 H2 등) → release 는 팔을 움직이지 않고 반납, park 는 거부
DEFAULT_ARM_DEG = ([float(v) for v in robot_env.CFG["default_arm_deg"]]
                   if robot_env.CFG.get("default_arm_deg") is not None else None)
PARK_WAIST_DEG = [0.0, 0.0, 0.0]   # yaw, roll, pitch

arm: Optional[ArmControllerWrapper] = None
ARM_MODE = "hold"                  # hold | release (기동 시 weight=1)
_switching = False
# 팔과 허리는 병렬 지령 가능 (robot_server 모션 러너가 joints+waist 동시 실행).
# 같은 그룹 내 동시 동작만 409. hold/release/park 는 둘 다 잡는다.
_arm_lock   = threading.Lock()     # joints / hands / stream 세션
_waist_lock = threading.Lock()     # waist

# ---- 관절 스트리밍 (/stream/*) ----
STREAM_HOLD_SEC = 0.2              # 지령이 이만큼 없으면 마지막 목표에서 멈춤 (hold 표시·로그)
STREAM_END_SEC  = 2.0              # 지령이 이만큼 없으면 세션 종료 + 팔 잠금 해제
STREAM_VEL_CAP  = 3.0              # 세션 max_vel 상한 [rad/s] (송신 루프 속도 제한 robot.yaml gains.arm_velocity_limit 와 별개로 더 낮게)
STREAM_DT_MAX   = 0.1              # 한 지령에서 속도 제한에 쓰는 시간 상한 [s] — 멈췄다 다시 와도 한 번에 크게 안 움직임
_stream = {"token": None, "q": None, "t_last": 0.0, "t_begin": 0.0, "max_vel": 1.0,
           "held": False, "n": 0, "n_vel_clip": 0, "why_end": ""}
_stream_mx = threading.Lock()      # _stream 갱신 + 중력 계산 직렬화
_stream_pin = None                 # (model, data) — IK 축소 모델 (q 순서 = 팔 14축 = ctrl_dual_arm 순서)


# ---------------- Pydantic ----------------
class JointsReq(BaseModel):
    deg: List[float]               # 14개 (좌7+우7)
    duration: float = 2.0

class WaistReq(BaseModel):
    yaw: float = 0.0               # deg
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.5

class HeadReq(BaseModel):
    pitch: float = 0.0             # deg, + = 숙임
    yaw: float = 0.0               # deg, + = 왼쪽

class HandsReq(BaseModel):
    left_xyz: List[float]
    right_xyz: List[float]
    left_quat: Optional[List[float]] = None    # [w,x,y,z]
    right_quat: Optional[List[float]] = None
    left_rpy: Optional[List[float]] = None     # deg (quat 없을 때)
    right_rpy: Optional[List[float]] = None
    duration: float = 2.0
    frequency: int = 100

class RampReq(BaseModel):
    duration: float = 2.0

class ParkReq(BaseModel):
    duration: float = 2.0
    arm_deg: Optional[List[float]] = None      # 미지정 시 DEFAULT_ARM_DEG

class StreamBeginReq(BaseModel):
    max_vel: float = 1.0           # rad/s — 관절마다 지령 사이 이동량 = max_vel × 경과시간 (STREAM_VEL_CAP 로 잘림)

class StreamCmdReq(BaseModel):
    token: str
    deg: List[float]               # 14개 (좌7+우7), 팔 목표

class StreamEndReq(BaseModel):
    token: Optional[str] = None    # None = 지금 세션 강제 종료


def _rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    return pin.Quaternion(cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                          cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy).normalized()


def _quat_from(quat, rpy):
    if quat:
        return pin.Quaternion(quat[0], quat[1], quat[2], quat[3]).normalized()
    if rpy and any(v != 0 for v in rpy):
        return _rpy_to_quat(*rpy)
    return None


def _acquire_or_409(*locks):
    got = []
    for lk in locks:
        if lk.acquire(blocking=False):
            got.append(lk)
        else:
            for g in got:
                g.release()
            raise HTTPException(409, "다른 동작 진행 중")
    return got


async def _run_blocking(locks, fn, *args):
    """동작을 executor 에서 완료까지 실행. locks 는 진입 전에 잡혀 있어야 함."""
    loop = asyncio.get_running_loop()
    try:
        return await loop.run_in_executor(None, fn, *args)
    finally:
        for lk in locks:
            lk.release()


# ---------------- Lifespan ----------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm
    print("[arm_server] 시작")
    robot_env.dds_init()
    arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
    arm.start()
    threading.Thread(target=_stream_watchdog, daemon=True).start()
    print(f"[arm_server] 준비 완료 (weight=1, hold)  http://localhost:{PORT}/")
    yield
    # 종료: 자세 유지, 제어권만 천천히 반납
    print("[arm_server] 종료 — 제어권 반납")
    try:
        if arm and arm.arm_ctrl and arm.arm_ctrl.motion_mode:
            w = arm.arm_ctrl.get_weight()
            if w > 0.0:
                arm.arm_ctrl.ramp_weight(0.0, 1.0)
            arm.arm_ctrl.disable_arm_sdk()     # robot.yaml sdk.enable_arm_sdk 로봇만 (H2)
    except Exception as e:
        print(f"[arm_server] 반납 실패: {e}")
    os._exit(0)


app = FastAPI(title="H2 Arm Server", lifespan=lifespan)


# ---------------- 조회 ----------------
@app.get("/status")
async def status():
    if not arm or not arm.arm_ctrl:
        return {"ready": False}
    return {"ready": True, "mode": ARM_MODE, "switching": _switching,
            "weight": arm.arm_ctrl.get_weight(),
            "busy": _arm_lock.locked() or _waist_lock.locked(),
            "stream": _stream_info()}


@app.get("/pose")
async def pose():
    """실측/타겟 관절각. 각도는 deg, *_rad 는 rad."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    ctrl = arm.arm_ctrl
    meas = np.asarray(ctrl.get_current_dual_arm_q(), dtype=float)
    waist = np.asarray(ctrl.get_waist_q(), dtype=float)
    with ctrl.ctrl_lock:
        tgt = np.asarray(ctrl.q_target, dtype=float).copy()
        wt = np.asarray(getattr(ctrl, "waist_q_target", np.zeros(3)),
                        dtype=float).copy()
    return {"arm_deg": np.degrees(meas).round(3).tolist(),
            "arm_target_deg": np.degrees(tgt).round(3).tolist(),
            "waist_deg": np.degrees(waist).round(3).tolist(),
            "waist_target_deg": np.degrees(wt).round(3).tolist(),
            "arm_rad": meas.round(5).tolist(),
            "waist_rad": waist.round(5).tolist()}


# ---------------- 제어권 ----------------
def _do_hold(duration):
    global ARM_MODE, _switching
    try:
        arm.arm_ctrl.sync_targets_to_current()
        arm.arm_ctrl.ramp_weight(1.0, duration)
        ARM_MODE = "hold"
        print("[ARM] hold 완료")
    finally:
        _switching = False


def _do_release(duration, arm_deg):
    global ARM_MODE, _switching
    try:
        # 기본 자세로 보간 후 반납 — 인계 순간 튐 최소화
        arm.move_waist_smooth(yaw=PARK_WAIST_DEG[0], roll=PARK_WAIST_DEG[1],
                              pitch=PARK_WAIST_DEG[2], duration=duration)
        if arm_deg is not None:
            arm.move_joints_smooth(arm_deg, duration)
        else:
            print("[ARM] 기본 팔 자세 미측정 — 팔은 현재 자세 그대로 반납")
        arm.arm_ctrl.ramp_weight(0.0, duration)
        ARM_MODE = "release"
        print("[ARM] release 완료")
    finally:
        _switching = False


@app.post("/hold")
async def hold(req: RampReq = RampReq()):
    global _switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if ARM_MODE == "hold" and not _switching:
        return {"ok": True, "mode": ARM_MODE}
    locks = _acquire_or_409(_arm_lock, _waist_lock)
    _switching = True
    await _run_blocking(locks, _do_hold, req.duration)
    return {"ok": True, "mode": ARM_MODE}


@app.post("/release")
async def release(req: ParkReq = ParkReq()):
    global _switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if ARM_MODE == "release" and not _switching:
        return {"ok": True, "mode": ARM_MODE}
    locks = _acquire_or_409(_arm_lock, _waist_lock)
    _switching = True
    await _run_blocking(locks, _do_release, req.duration,
                        req.arm_deg or DEFAULT_ARM_DEG)
    return {"ok": True, "mode": ARM_MODE}


# ---------------- 동작 ----------------
@app.post("/joints")
async def joints(req: JointsReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    if len(req.deg) != 14:
        raise HTTPException(400, "deg 는 14개")
    locks = _acquire_or_409(_arm_lock)
    await _run_blocking(locks, arm.move_joints_smooth, list(req.deg), req.duration)
    return {"ok": True}


@app.post("/waist")
async def waist(req: WaistReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    locks = _acquire_or_409(_waist_lock)

    def _do():
        arm.move_waist_smooth(yaw=req.yaw, roll=req.roll,
                              pitch=req.pitch, duration=req.duration)
    await _run_blocking(locks, _do)
    return {"ok": True}


def _head_state(target=None):
    c = arm.arm_ctrl
    t = c.get_head_target() if target is None else target
    return {"ok": True, "target": [round(float(v), 2) for v in np.degrees(t)],
            "meas": [round(float(v), 2) for v in np.degrees(c.get_head_q())]}


@app.get("/head")
async def head_get():
    """머리 [pitch, yaw] 목표·실측 (deg)."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if not arm.arm_ctrl.has_head():
        raise HTTPException(409, "머리 없음 (robot.yaml joints.head_range_deg)")
    return _head_state()


@app.post("/head")
async def head(req: HeadReq):
    """머리 목표 (deg). robot.yaml joints.head_range_deg 로 잘림, 블로킹 없음 (703 에서는 머리가 안 움직임 — FACTS.md)."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    q = arm.arm_ctrl.ctrl_head(np.radians([req.pitch, req.yaw]))
    if q is None:
        raise HTTPException(409, "머리 없음 (robot.yaml joints.head_range_deg)")
    return _head_state(q)


@app.post("/hands")
async def hands(req: HandsReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    l_rot = _quat_from(req.left_quat, req.left_rpy)
    r_rot = _quat_from(req.right_quat, req.right_rpy)
    locks = _acquire_or_409(_arm_lock)

    def _do():
        arm.move_hands(list(req.left_xyz), list(req.right_xyz),
                       l_rot, r_rot, req.duration, req.frequency)
    await _run_blocking(locks, _do)
    return {"ok": True}


@app.post("/park")
async def park(req: ParkReq = ParkReq()):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    if (req.arm_deg or DEFAULT_ARM_DEG) is None:
        raise HTTPException(409, "기본 팔 자세 미측정 (robot.yaml default_arm_deg) — park 불가")
    locks = _acquire_or_409(_arm_lock, _waist_lock)

    def _do():
        arm.move_waist_smooth(yaw=PARK_WAIST_DEG[0], roll=PARK_WAIST_DEG[1],
                              pitch=PARK_WAIST_DEG[2], duration=req.duration)
        arm.move_joints_smooth(req.arm_deg or DEFAULT_ARM_DEG, req.duration)
    await _run_blocking(locks, _do)
    return {"ok": True}


@app.post("/stop_motion")
async def stop_motion():
    """진행 중 보간 중단 (자세는 그 자리에 멈춤). 언제든 호출 가능."""
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    _stream_end("stop_motion")
    try:
        arm.stop_motion()
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)})
    return {"ok": True}


@app.post("/freeze")
async def freeze():
    """보간 중단 + 팔·허리·머리 타겟을 현재 실측각으로 동결."""
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    _stream_end("freeze")
    try:
        arm.stop_motion()
    except Exception:
        pass
    try:
        arm.arm_ctrl.sync_targets_to_current()   # 팔 = 현재각, 토크 0, 허리 = 현재각 (robot.yaml joints.waist 슬롯, 허리 유지 로봇은 그대로)
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)})
    return {"ok": True}


# ---------------- 관절 스트리밍 (physical_ai 정책 추론) ----------------
def _stream_info():
    with _stream_mx:
        if _stream["token"] is None:
            return {"active": False, "why_end": _stream["why_end"]}
        now = time.time()
        return {"active": True, "held": _stream["held"], "n": _stream["n"],
                "n_vel_clip": _stream["n_vel_clip"], "max_vel": _stream["max_vel"],
                "idle_s": round(now - _stream["t_last"], 3), "age_s": round(now - _stream["t_begin"], 1)}


def _stream_end(why):
    """스트림 세션 종료 + 팔 잠금 해제 (세션이 없으면 아무것도 안 함). 목표는 마지막 값 그대로 유지."""
    with _stream_mx:
        if _stream["token"] is None:
            return False
        _stream.update(token=None, q=None, held=False, why_end=why)
        _arm_lock.release()      # begin 에서 잡은 잠금 (threading.Lock 은 다른 스레드에서 풀 수 있음)
    print(f"[STREAM] 종료 — {why}")
    return True


def _stream_watchdog():
    while True:
        time.sleep(0.05)
        end_why = None
        with _stream_mx:
            if _stream["token"] is None:
                continue
            idle = time.time() - _stream["t_last"]
            if idle > STREAM_END_SEC:
                end_why = f"{STREAM_END_SEC:.0f}초 지령 없음"
            elif idle > STREAM_HOLD_SEC and not _stream["held"]:
                _stream["held"] = True     # 목표는 마지막 값 그대로 → 그 자세에서 멈춰 있음
                print(f"[STREAM] {STREAM_HOLD_SEC:.1f}초 지령 없음 — 마지막 목표에서 정지 (hold)")
        if end_why:
            _stream_end(end_why)


def _gravity_tau(q):
    """IK 와 같은 중력 보상 토크 (rnea, 속도·가속도 0). q = 팔 14축 [rad]."""
    global _stream_pin
    if _stream_pin is None:
        m = arm.arm_ik.reduced_robot.model
        _stream_pin = (m, m.createData())
    m, d = _stream_pin
    z = np.zeros(m.nv)
    return np.asarray(pin.rnea(m, d, q, z, z), dtype=float).copy()


@app.post("/stream/begin")
async def stream_begin(req: StreamBeginReq = StreamBeginReq()):
    """스트림 세션 시작 — 팔 잠금을 잡는다 (다른 팔 동작 중이면 409). 첫 목표 기준 = 지금 팔 목표."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if ARM_MODE != "hold" or _switching:
        raise HTTPException(409, f"팔 제어권 {ARM_MODE}{' (전환 중)' if _switching else ''} — hold 에서만 스트림")
    if not (0.0 < req.max_vel <= STREAM_VEL_CAP):
        raise HTTPException(400, f"max_vel 은 0 초과 {STREAM_VEL_CAP} rad/s 이하")
    _acquire_or_409(_arm_lock)
    arm.stop_motion()            # 혹시 남은 보간 루프 끊기 (잠금을 잡았으니 보통 없음)
    with arm.arm_ctrl.ctrl_lock:
        q0 = np.asarray(arm.arm_ctrl.q_target, dtype=float).copy()
    tok = uuid.uuid4().hex
    now = time.time()
    with _stream_mx:
        _stream.update(token=tok, q=q0, t_last=now, t_begin=now, max_vel=float(req.max_vel),
                       held=False, n=0, n_vel_clip=0, why_end="")
    print(f"[STREAM] 시작 (max_vel {req.max_vel:.2f} rad/s)")
    return {"ok": True, "token": tok, "target_deg": np.degrees(q0).round(3).tolist(),
            "meas_deg": np.degrees(arm.arm_ctrl.get_current_dual_arm_q()).round(3).tolist()}


@app.post("/stream/cmd")
def stream_cmd(req: StreamCmdReq):      # def — 짧은 계산, 스레드풀에서
    """팔 목표 1회. 지난 목표에서 max_vel × 경과시간(≤0.1 s) 만큼만 움직이고 URDF 한계로 자른다.
    응답: 실제 보낸 목표(target_deg) · 실측(meas_deg) · 속도 제한에 걸렸는지(vel_clip)."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if len(req.deg) != 14:
        raise HTTPException(400, "deg 는 14개")
    want = np.radians(np.asarray(req.deg, dtype=float))
    if not np.all(np.isfinite(want)):
        raise HTTPException(400, "deg 에 nan/inf")
    with _stream_mx:
        if _stream["token"] is None or req.token != _stream["token"]:
            raise HTTPException(409, f"스트림 세션 없음 ({_stream['why_end'] or '시작 안 함'})")
        now = time.time()
        dt = min(max(now - _stream["t_last"], 0.0), STREAM_DT_MAX)
        step = _stream["max_vel"] * dt
        q_prev = _stream["q"]
        q = np.clip(want, q_prev - step, q_prev + step)
        vel_clip = bool(np.any(np.abs(q - want) > 1e-6))
        if ARM_LIMITS is not None:
            q = np.clip(q, ARM_LIMITS[0], ARM_LIMITS[1])
        tau = _gravity_tau(q)
        arm.arm_ctrl.ctrl_dual_arm(q, tau)
        if _stream["held"]:
            print(f"[STREAM] 지령 다시 수신 ({now - _stream['t_last']:.2f}s 만에)")
        _stream.update(q=q, t_last=now, held=False, n=_stream["n"] + 1,
                       n_vel_clip=_stream["n_vel_clip"] + int(vel_clip))
    return {"ok": True, "vel_clip": vel_clip,
            "target_deg": np.degrees(q).round(3).tolist(),
            "meas_deg": np.degrees(arm.arm_ctrl.get_current_dual_arm_q()).round(3).tolist()}


@app.post("/stream/end")
async def stream_end(req: StreamEndReq = StreamEndReq()):
    """스트림 세션 종료 (팔은 마지막 목표에서 멈춤). token 없이 부르면 지금 세션을 강제 종료."""
    with _stream_mx:
        cur = _stream["token"]
    if cur is None:
        return {"ok": True, "ended": False}
    if req.token is not None and req.token != cur:
        raise HTTPException(409, "다른 스트림 세션")
    return {"ok": True, "ended": _stream_end("end 요청" if req.token else "강제 종료 요청")}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)
