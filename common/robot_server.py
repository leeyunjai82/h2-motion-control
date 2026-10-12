#!/usr/bin/env python3
# Version: 1.19
# Changes:
#   1.19 - place(제자리 내려놓기) 손 벌림 0.10 → grab.place_open (0.15) — 손을 들 때 박스가 따라 올라옴 (사용자 2026-10-12)
#   1.18 - 1.17 다시 잡기 되돌림 (사용자 2026-10-11: 당기기 전에 다시 잡는 건 원한 동작이 아님 → 원래대로).
#          건네기 놓을 때 손 벌림 0.10 → grab.handover_open (0.15) 은 유지 — 박스를 확실히 놓게 (사용자)
#   1.16 - 왼손 y 보정(grab.left_hand_y_offset)은 ④ 접근·⑤ 하강에만 — 넓게 내려가고, ⑥ 잡기부터는 양손 같게 조임
#          (사용자 2026-10-11: 박스가 풀림 · '잡을 때부터 더 조이면 됨'. 이전엔 잡은 뒤에도 왼손이 3 cm 덜 조였음)
#   1.15 - TTS 멘트는 영어만 (사용자 2026-10-11) — 한/영 연동(GET /set_tts_lang)·한국어 멘트 제거, robot.yaml tts.voice 한 개
#   1.14 - 비스듬한 박스: L/R 점으로 박스 회전각을 구해 접근·하강·잡기·당기기 동안 양손 손목 yaw 에 더함 (손바닥이 옆면과 평행).
#          ⑥' 대칭 정렬에서 손목을 기본값으로 되돌리며 박스를 반듯하게 펴고, 손 간격은 잡은 두 점 사이 거리 그대로 (돌려도 더 조이지 않게).
#          |회전| > grab.max_box_yaw_deg 면 잡지 않음. 박스 잡기 시작 멘트 추가 ("Let me pick up the box.")
#          접근점을 옆면 바깥 방향 대신 순수 좌우(±y)로 — 비스듬하면 먼 쪽 손이 도달 한계를 넘던 것 (조이기 방향은 옆면 법선 그대로)
#          TTS 멘트 한/영 — 제어 화면 한/EN 버튼을 따라감 (GET /set_tts_lang), 음성 번호는 robot.yaml tts.voice
#   1.13 - /system 에 camera (지금 쓰는 숙임각과 출처) — robots/h2/calib_local.yaml (utils/check_rsimu.py --save) 가 있으면 그 값
#   1.12 - grab.squeeze: 잡을 때 양손을 박스 안쪽으로 더 미는 거리 (위치 제어라 이만큼이 쥐는 힘) — 잡기~들기~건네기 유지,
#          손 벌림에서 풀림. 이전에는 왼손 보정 +3 cm 때문에 양손 합쳐 1 cm 만 조여 건네는 중 박스가 미끄러짐 (사용자 2026-10-09)
#   1.11 - Center(건네기) 는 옆에서 잡았어도 양손 가운데를 몸 정면(y 0)으로 옮겨 건넴 (Place 는 원래 자리 그대로)
#          손목 기본값을 손별로 (robot.yaml grab.wrist_rpy_deg: {left: [...], right: [...]} 또는 [r, p, y] 양손 같게)
#   1.10 - GET /system (로봇 FSM · 서버 상태, 제어 화면 System 카드), arm_server 가 늦게 떠도 3초마다 다시 연결,
#          Box 선택 때 팔이 안 움직일 상황(arm 미연결 · release · FSM≠4/703)이면 /set_mode 가 warn
#   1.9 - H2 전용: 보행(/loco)·마커 추종(/follow)·마커 잡기·허리 yaw 정렬·좌우 건네기 삭제
#   1.8 - /viz 에 비교용 다른 박스 추정 방식(other) + 차이(cmp) 추가 — 표시 전용, 잡기는 detect_box BOX_METHOD 값
#   1.7 - 잡기 단계별 소요 시간 기록 + GET /grab_history (최근 5회) — 표시용
#   1.6 - 잡기 진행 단계(stage) 노출(/status, /grab_status), 3D 시각화용 GET /viz (박스·손 목표)
#   1.5 - 잡은 뒤 건네기까지 단계 사이 대기 축소(1.3s→0.5s), box 받음 대기 3초→2초 (HANDOVER_HOLD_SEC)
#   1.4 - arm 제어를 arm_server(50022) HTTP로 분리, Box Size 엔드포인트 제거(사용처 없음) (arm_sdk 단독 점유는 arm_server)
#   1.3 - 마커 추종 정면(법선) 경유점 접근 — 옆에서 와도 마커 정면으로 돌아 들어감
#   1.2 - grab_box가 L/R 실제좌표 직접 사용(기울어진 박스 양손 정확)
#   1.1 - handover 허리 yaw 회전 각도비례 감속(90도시 느리게), reset 1.5초
#   1.0 - box 놓기 5초→3초, 미수령 시 약간 내려놓기(타임아웃)
#   0.9 - TTS 멘트 선물 컨셉 제거, 잡기 위주로 변경
#   0.8 - handover 받음 처리 분리 (marker=가림감지 / box=고정5초)
#   0.7 - park 자세 [0.0,±0.28,-0.38] 차렷에 가깝게
#   0.6 - WAIST_BASE_PITCH 상수 (현재 -3.0)
#   0.5 - set_mode 시 ready/park 자세, marker_x_axis reshape 방어
#   0.4 - align 후 재감지(redetect) 추가, _run_grab 트레이스백
#   0.3 - 종료 시 팔 자세 유지(제어권만 반납), 포트 50000
#   0.2 - viewer 제거(dashboard 담당), robot_web.html 분리
#   0.1 - run_motion + grab_core 통합 초기본
"""
robot_server.py — H2 잡기 제어 서버 (포트 50000)

  · 관절/IK 모션 실행 (/run, /run_ik, /motions/run)
  · 컨트롤 웹 UI (robot_web.html, 3D 뷰어는 dashboard.py)
  · GrabController — 박스 잡기 시퀀스 (대기 → 접근 → 하강 → 잡기 → 당기기 → 들기 → 놓기/건네기 → 복귀)
  · POST /grab_at      — 인식(detect_box :50010)이 좌표 주면 잡기 실행
  · GET  /active_mode  — 현재 모드 (인식 파일이 폴링)
  · POST /set_mode     — 웹에서 box/none 전환
  · GET  /grab_status  — busy 등

팔 명령은 arm_server(:50022, rt/arm_sdk 단독 점유)를 HTTP 로 거친다.
H2: 보행 없음, 허리 고정(arm_sdk 로 안 움직임) — 건네기는 정면(center) / 제자리(place) 만.
"""

import os
import sys
import json
import time
import asyncio
import threading
import numpy as np
from collections import deque
from pathlib import Path
from contextlib import asynccontextmanager
from typing import List, Optional

import uvicorn
import pinocchio as pin
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse, Response, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

import robot_env   # robots/h2/robot.yaml (H2 전용)

# ===== 경로 (robots/h2/) =====
MOTIONS_DIR = Path(robot_env.MOTIONS_DIR)
MOTIONS_DIR.mkdir(exist_ok=True)
ASSETS_DIR  = robot_env.ROBOT_DIR
URDF_PATH   = robot_env.URDF_PATH
MESH_DIR    = robot_env.MESH_DIR
VENDOR_DIR  = robot_env.VENDOR_DIR

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import GLOBAL_TO_INTERNAL
from ctrl.arm_http import ArmHttpClient
from ctrl.hw_usage import HwUsage
from ctrl.sys_watch import SysWatch




# ==========================================
# 카메라 → torso 좌표 변환 (ik_box와 동일 상수)
# ==========================================
# D435i 장착값 — robots/h2/robot.yaml camera (torso_link 기준)
CAMERA_X          = robot_env.CAMERA_X
CAMERA_Y          = robot_env.CAMERA_Y
CAMERA_Z          = robot_env.CAMERA_Z
CAMERA_PITCH_URDF = robot_env.CAMERA_PITCH


def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cx_r =  cx
    cy_r =  cy * cos_p + cz * sin_p
    cz_r = -cy * sin_p + cz * cos_p
    return (float(cz_r + CAMERA_X),
            float(-cx_r + CAMERA_Y),
            float(-cy_r + CAMERA_Z))


# ==========================================
# 잡기 파라미터 — robots/h2/robot.yaml grab
# ==========================================
APPROACH_EXTRA = float(robot_env.CFG["grab"]["approach_extra"])   # 접근 때 박스 옆면에서 바깥으로 더 벌림 [m]
GRAB_Z_OFFSET  = float(robot_env.CFG["grab"]["z_offset"])          # grab_z = 박스 윗면 − H/2 + z_offset (실기 보정값)
GRAB_X_OFFSET  = float(robot_env.CFG["grab"].get("grab_x_offset", 0.0))
# ↑ 좁혀 잡을 때 손 x 를 박스 중심에서 이만큼 옮김. H2 0 (박스 옆면 가운데를 그대로 잡음)
GRAB_INSET_FRAC = float(robot_env.CFG["grab"].get("grab_inset_frac", 0.0))
GRAB_INSET_X = float(robot_env.CFG["grab"].get("grab_inset_x", 0.0))
GRAB_INSET_MARGIN = 0.03     # 손 x 가 박스 앞면(몸쪽 면)에서 최소 이만큼 안쪽
# ↑ 접근·하강·잡기·제자리 놓기 손 x 를 모두 박스 중심보다 몸쪽으로.
#   grab_inset_frac: 윗면 앞뒤 길이 D(인식된 윗면 좌/우 변 길이) 의 비율. 0.25 = 윗면 중심과 몸쪽 끝변의 가운데.
#   grab_inset_x: D 를 못 받았을 때 고정값 [m].
#   하강 전부터 같은 x 라 좁힐 때 손이 앞뒤로 움직이지 않음. 팔을 덜 뻗어 손이 더 내려감.


def grab_inset(box_d):
    """손 x 를 박스 중심보다 몸쪽으로 옮길 거리 [m]. D 를 알면 frac·D (앞면에서 GRAB_INSET_MARGIN 이상 안쪽), 모르면 고정값."""
    if GRAB_INSET_FRAC > 0 and box_d and 0.05 < box_d < 0.80:
        return min(GRAB_INSET_FRAC * box_d, max(0.0, box_d / 2 - GRAB_INSET_MARGIN))
    return GRAB_INSET_X
PULL_X = float(robot_env.CFG["grab"].get("pull_x", 0.0))
# ↑ 좁혀 잡은 '뒤' 그 높이 그대로 x 로 끌어당김 [m] (− = 몸쪽). 팔을 덜 뻗은 자세에서 들기 위해
ALIGN_TO_HANDS = bool(robot_env.CFG["grab"].get("align_to_hands", True))
# ↑ 잡은 뒤 '대칭 정렬'·들기·놓기 기준을 실제 잡은 손 위치로 (박스 중심 추정과 L/R 점이 어긋나도 손이 튀지 않게)
HANDOVER_X     = float(robot_env.CFG["grab"]["handover_x"])   # 건네기 손 x (IK 좌표)
READY_XYZ      = [float(v) for v in robot_env.CFG["grab"]["ready_xyz"]]
# ↑ Box 버튼(대기 자세)·잡기 끝 복귀(⑪ Home) 왼손 [x, y, z] (IK 좌표, 오른손은 y 반대)
LIFT_ABOVE     = float(robot_env.CFG["grab"]["lift_above"])
# ↑ 들기 높이 = 박스 윗면 + lift_above [m] — 몸쪽으로 당겨 높이 들면 어깨가 카메라 거치대 요크에 닿음
def _wrist_cfg():
    """grab.wrist_rpy_deg — [roll, pitch, yaw] (양손 같게) 또는 {left: [...], right: [...]} [deg]."""
    w = robot_env.CFG["grab"].get("wrist_rpy_deg", [0.0, 0.0, 0.0])
    if isinstance(w, dict):
        return {s: [float(v) for v in w.get(s, [0.0, 0.0, 0.0])] for s in ("left", "right")}
    w = [float(v) for v in w]
    return {"left": w, "right": list(w)}
WRIST_RPY_DEG  = _wrist_cfg()   # {"left": [r, p, y], "right": [r, p, y]}
# ↑ 손목 RPY 기본값 [roll, pitch, yaw] deg, 양손 같게. 웹 Wrist RPY 로 바꾸면 그 값
LEFT_HAND_Y_OFFSET = float(robot_env.CFG["grab"].get("left_hand_y_offset", 0.0))   # 왼손 y 보정 [m] (+ = 바깥/왼쪽) — ④ 접근·⑤ 하강만
GRAB_SQUEEZE = float(robot_env.CFG["grab"].get("squeeze", 0.0))   # 잡을 때 양손을 박스 안쪽으로 더 [m] (각 손)
TTS_VOICE = int((robot_env.CFG.get("tts") or {}).get("voice", 6))   # TTS 서버 음성 번호 (멘트는 영어)
BOX_YAW_FOLLOW = bool(robot_env.CFG["grab"].get("box_yaw_follow", True))     # 비스듬한 박스: 손목 yaw 를 박스 회전만큼 더함
MAX_BOX_YAW_DEG = float(robot_env.CFG["grab"].get("max_box_yaw_deg", 25.0))  # 이보다 더 돌아 있으면 잡지 않음 [deg]
HANDOVER_OPEN = float(robot_env.CFG["grab"].get("handover_open", 0.15))      # 건네기 놓을 때 각 손을 바깥으로 [m]
PLACE_OPEN = float(robot_env.CFG["grab"].get("place_open", 0.15))            # 제자리 내려놓은 뒤 각 손을 바깥으로 [m]
WAIST_BASE_PITCH = float(robot_env.CFG["grab"]["waist_base_pitch_deg"])   # Home(pose=zero) 허리 pitch (H2 허리는 arm_sdk 로 안 움직임)

# 잡은 뒤 → 건네기 구간 타이밍 (s)
STEP_PAUSE        = 0.1   # 대칭정렬/들기/건네기 동작 사이 정지
HANDOVER_HOLD_SEC = 2.0   # center: 건넨 뒤 손 벌리기까지 대기
PLACE_HOLD_SEC    = 1.5   # place: 들어 올린 채 보여 주는 시간
PLACE_Z_CLEAR     = 0.005 # place: 내려놓을 때 잡았던 높이보다 이만큼 위에서 놓기 (테이블 누름 방지)
HANDOVER_MODES = ("center", "place")   # 허리 고정이라 정면 건네기 / 제자리 내려놓기만
DEFAULT_HANDOVER = str(robot_env.CFG["grab"].get("default_handover", "place"))
assert DEFAULT_HANDOVER in HANDOVER_MODES, f"robot.yaml grab.default_handover: {DEFAULT_HANDOVER}"


# ==========================================
# 잡기 진행 단계 (웹 진행 표시 / 3D 시각화용)
# ==========================================
GRAB_STAGES = ["재검출", "위쪽 접근", "측면 하강", "잡기",
               "들기", "건네기", "받기 대기", "놓기", "복귀"]

# IK 목표 좌표계 = pelvis 기준(허리 0 가정 축소모델). 카메라 좌표는 torso_link 기준.
# /viz 는 모두 torso_link 기준으로 내보낸다 (dashboard 가 torso_link 에 붙여 그림).
PELVIS_TO_TORSO = tuple(float(v) for v in robot_env.CFG["frames"]["pelvis_to_torso"])   # robot.yaml frames.pelvis_to_torso


def cam_to_ik(cx, cy, cz):
    """잡기용: 카메라 좌표 → IK 목표 좌표 (torso + pelvis_to_torso, H2 는 차이 z 12.3 cm)."""
    x, y, z = camera_to_torso(cx, cy, cz)
    return (x + PELVIS_TO_TORSO[0], y + PELVIS_TO_TORSO[1], z + PELVIS_TO_TORSO[2])


def ik_to_torso(p):
    return [float(p[0] - PELVIS_TO_TORSO[0]), float(p[1] - PELVIS_TO_TORSO[1]),
            float(p[2] - PELVIS_TO_TORSO[2])]


# ==========================================
# GrabController
# ==========================================
class GrabController:
    """잡기 시퀀스 실행기.

    robot_server가 arm, speak, wrist_params, handover 설정을 주입.
    """
    def __init__(self, arm=None, speak=None,
                 robot_available=False):
        self.arm  = arm
        self.speak = speak or (lambda t: print(f"[TTS-DUMMY] {t}"))
        self.robot_available = robot_available

        # 손목 RPY
        self.wrist_params = {s: dict(zip(('roll', 'pitch', 'yaw'), WRIST_RPY_DEG[s])) for s in ('left', 'right')}
        # handover 방향
        self.handover_direction = DEFAULT_HANDOVER   # center (정면 건네기) | place (제자리 내려놓기)

        # TTS 멘트 (선물 컨셉 제거, 잡기 위주, 영어)
        self.MSGS = {"start":    "Let me pick up the box.",            # 잡기 시작 (재검출·회전 확인 뒤, 팔이 움직이기 직전)
                     "picked":   "I got it.",                          # 잡고 몸쪽으로 당긴 뒤
                     "handover": "Here you go. Please take the box.",  # Center: 가운데로 내민 뒤
                     "received": "Nicely done!",                       # Center: 2 초 뒤 (받았는지 확인 안 함)
                     "timeout":  "No one? I will put it down.",        # (지금은 안 쓰임 — 받음 확인이 없음)
                     "home":     "Bring me another box.",              # Center: 대기 자세 복귀 뒤
                     "placed":   "I put it back."}                     # Place: 내려놓고 손 뺀 뒤

        self.HOME_LEFT  = [READY_XYZ[0],  READY_XYZ[1], READY_XYZ[2]]
        self.HOME_RIGHT = [READY_XYZ[0], -READY_XYZ[1], READY_XYZ[2]]

        # 재감지 콜백 (robot_server가 주입, detect_box /pose) — None이면 재감지 안 함
        self.redetect = None
        # 진행 표시 / 시각화
        self.stage = None                  # GRAB_STAGES 중 하나 (None=대기)
        self._stage_log = []               # [(단계, 시작시각)] — 이번 잡기
        self.history = deque(maxlen=5)     # 최근 잡기 기록 (단계별 소요 시간)
        self.targets = None                # 마지막 IK 목표 {"L":[xyz],"R":[xyz]} (IK=pelvis 기준)

    def _stage(self, name):
        if name not in GRAB_STAGES:
            return
        self.stage = name
        self._stage_log.append((name, time.time()))
        print(f"[STAGE] {GRAB_STAGES.index(name)+1}/{len(GRAB_STAGES)} {name}")

    def begin_log(self):
        self._stage_log = []

    def end_log(self):
        """이번 잡기의 단계별 소요 시간을 history 에 남긴다. 복귀까지 갔으면 완료."""
        log, end = self._stage_log, time.time()
        if not log:
            return
        stages = [[n, round((log[i + 1][1] if i + 1 < len(log) else end) - t, 2)]
                  for i, (n, t) in enumerate(log)]
        self.history.appendleft({"end": round(end, 1), "total": round(end - log[0][1], 2),
                                 "ok": log[-1][0] == "복귀", "stages": stages})
        print(f"[GRAB] 소요 {end - log[0][1]:.1f}s — " +
              ", ".join(f"{n} {d:.1f}" for n, d in stages))

    def _say(self, key):
        self.speak(self.MSGS[key])

    # ---- 로봇 저수준 래퍼 ----
    def _rpy_to_quat(self, roll_deg, pitch_deg, yaw_deg):
        r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        yq= cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        return pin.Quaternion(w, x, yq, z).normalized()

    def _move(self, left_xyz, right_xyz, duration, msg="",
              left_rot=None, right_rot=None):
        print(f"[IK] {msg}  L:{[f'{v:.3f}' for v in left_xyz]}  "
              f"R:{[f'{v:.3f}' for v in right_xyz]}")
        self.targets = {"L": [float(v) for v in left_xyz], "R": [float(v) for v in right_xyz]}
        if not self.robot_available or self.arm is None:
            time.sleep(duration)
            return True
        try:
            self.arm.move_hands(left_xyz, right_xyz,
                                left_rot, right_rot, duration, 100)
            return True
        except Exception as e:
            print(f"[IK] 오류: {e}")
            return False

    def _wrist_quats(self, extra_yaw_deg=0.0):
        """손목 자세 쿼터니언 (왼, 오른). extra_yaw_deg: 양손 yaw 에 더할 값 (박스 회전 — 몸통 z 축 기준이라 roll·pitch 는 손 기준 그대로)."""
        lp = self.wrist_params['left']
        rp = self.wrist_params['right']
        return (self._rpy_to_quat(lp['roll'], lp['pitch'], lp['yaw'] + extra_yaw_deg),
                self._rpy_to_quat(rp['roll'], rp['pitch'], rp['yaw'] + extra_yaw_deg))

    # ---- 공통 후반부: 대칭→들기→handover→복귀 ----
    def _finish_sequence(self, grab_x_base, grp_off_L, grp_off_R,
                         grab_z, lift_z, l_rot, r_rot, yc=0.0):
        # yc: 양손 중심 y (align_to_hands = 실제 잡은 손 중심, 아니면 0)
        self._say("picked")

        sym_L = [grab_x_base, yc + grp_off_L, grab_z]
        sym_R = [grab_x_base, yc - grp_off_R, grab_z]
        self._stage("들기")
        if not self._move(sym_L, sym_R, 1.5, "⑥' 대칭 정렬", l_rot, r_rot):
            return
        time.sleep(STEP_PAUSE)

        ll = [grab_x_base, yc + grp_off_L, lift_z]
        rl = [grab_x_base, yc - grp_off_R, lift_z]
        if not self._move(ll, rl, 1.5, "⑦ 들기", l_rot, r_rot):
            return
        time.sleep(STEP_PAUSE)

        if self.handover_direction == "place":
            self._place_back(grab_x_base, grp_off_L, grp_off_R, grab_z, lift_z, l_rot, r_rot, yc)
            return

        # 건네기는 항상 몸 정면 가운데 (y 0) — 옆에서 잡았어도 양손 가운데를 0 으로 옮김 (허리 고정이라 팔로만)
        if abs(yc) > 0.005:
            print(f"[HANDOVER] 잡은 자리 y {yc*100:+.1f} cm → 가운데(y 0)로 건넴")
        yc = 0.0
        self._stage("건네기")
        hl = [HANDOVER_X, yc + grp_off_L, lift_z]
        hr = [HANDOVER_X, yc - grp_off_R, lift_z]
        if not self._move(hl, hr, 1.5, "⑧ 건네기", l_rot, r_rot):
            return
        time.sleep(STEP_PAUSE)
        self._stage("받기 대기")
        self._say("handover")

        # 받음 처리 — 박스는 받아도 계속 보이므로 고정 대기 후 놓기
        print(f"[HANDOVER] 박스 — {HANDOVER_HOLD_SEC:.0f}초 대기 후 놓기")
        time.sleep(HANDOVER_HOLD_SEC)
        received = True

        self._say("received" if received else "timeout")
        self._stage("놓기")

        if received:
            # 받음 — 그 높이에서 손 벌려 놓기
            open_L = [HANDOVER_X, yc + grp_off_L + HANDOVER_OPEN, lift_z]
            open_R = [HANDOVER_X, yc - grp_off_R - HANDOVER_OPEN, lift_z]
            self._move(open_L, open_R, 1.0, "⑩ 손 벌림 (놓기)", l_rot, r_rot)
        else:
            # 못 받음 — 약간 내려서 살포시 놓고 손 벌림
            down_z = lift_z - 0.12
            dl = [HANDOVER_X, yc + grp_off_L, down_z]
            dr = [HANDOVER_X, yc - grp_off_R, down_z]
            self._move(dl, dr, 1.2, "⑩ 내려놓기", l_rot, r_rot)
            time.sleep(0.2)
            open_L = [HANDOVER_X, yc + grp_off_L + HANDOVER_OPEN, down_z]
            open_R = [HANDOVER_X, yc - grp_off_R - HANDOVER_OPEN, down_z]
            self._move(open_L, open_R, 1.0, "⑩' 손 벌림 (놓기)", l_rot, r_rot)
        time.sleep(0.3)

        self._stage("복귀")
        print("[HANDOVER] ⑪ 복귀")
        self._move(self.HOME_LEFT, self.HOME_RIGHT, 2.0, "⑪ Home")
        self._say("home")

    # ---- place: 들었던 자리에 다시 내려놓기 (사람 없이 반복 시연) ----
    def _place_back(self, grab_x_base, grp_off_L, grp_off_R, grab_z, lift_z, l_rot, r_rot, yc=0.0):
        # 잡기에서 박스를 몸쪽으로 (GRAB_X_OFFSET + PULL_X) 만큼 당겼으므로, 원래 자리로 다시 밀어 놓는다
        #   px = 처음 잡은 손 x (박스 중심 − GRAB_INSET_X) → 박스가 원래 자리에 놓임
        #   → 다음 회에도 같은 자리에서 인식·잡기 (반복 시연). 들어 올린 높이에서 앞으로 → 내려놓기
        px = grab_x_base - GRAB_X_OFFSET - PULL_X
        print(f"[PLACE] 들어 올린 채 {PLACE_HOLD_SEC:.1f}초")
        time.sleep(PLACE_HOLD_SEC)
        self._stage("놓기")
        fl = [px, yc + grp_off_L, lift_z]
        fr = [px, yc - grp_off_R, lift_z]
        if not self._move(fl, fr, 1.5, "⑨ 원래 자리 위로", l_rot, r_rot):
            return
        down_z = grab_z + PLACE_Z_CLEAR
        dl = [px, yc + grp_off_L, down_z]
        dr = [px, yc - grp_off_R, down_z]
        if not self._move(dl, dr, 1.5, "⑨' 제자리 내려놓기", l_rot, r_rot):
            return
        time.sleep(0.3)
        ol = [px, yc + grp_off_L + PLACE_OPEN, down_z]
        orr = [px, yc - grp_off_R - PLACE_OPEN, down_z]
        if not self._move(ol, orr, 1.0, "⑩ 손 벌림", l_rot, r_rot):
            return
        ul = [px, yc + grp_off_L + PLACE_OPEN, lift_z]
        ur = [px, yc - grp_off_R - PLACE_OPEN, lift_z]
        if not self._move(ul, ur, 1.0, "⑩' 손 위로 (박스에서 떨어지기)", l_rot, r_rot):
            return
        self._say("placed")
        time.sleep(0.3)
        self._stage("복귀")
        print("[PLACE] ⑪ 복귀")
        self._move(self.HOME_LEFT, self.HOME_RIGHT, 2.0, "⑪ Home")

    # ---- 대기 자세 (모드 선택 시) ----
    def ready(self):
        """잡을 준비 — 팔을 작업 대기 자세(HOME)로 들어 올림."""
        print("[READY] 대기 자세로")
        l_rot, r_rot = self._wrist_quats()
        self._move(self.HOME_LEFT, self.HOME_RIGHT, 2.0, "READY 대기자세", l_rot, r_rot)

    def park(self):
        """대기 해제 — 팔을 기동 시점 자세로 되돌린다.

        기동 자세를 캡처해두고 그 각도로 복귀하므로, 부팅 직후와 OFF 후의
        팔 위치가 같아진다. 캡처 실패 시에만 기존 IK 차렷 자세로 간다.
        """
        if BOOT_ARM_DEG is not None and self.robot_available and self.arm is not None:
            print("[PARK] 팔 내림 — 기동 자세로 복귀")
            try:
                self.arm.move_joints_smooth(BOOT_ARM_DEG, 2.0)
                return
            except Exception as e:
                print(f"[PARK] 기동 자세 복귀 실패: {e} — IK 폴백")
        print("[PARK] 팔 내림 (IK)")
        self._move([0.0, 0.28, -0.38], [0.0, -0.28, -0.38], 2.0, "PARK 팔내림")

    # ---- box(cardboard) 잡기 ----
    def grab_box(self, L_cam, R_cam, box_h_m=None, top_center_cam=None, box_d_m=None):
        """박스: L/R(윗면 좌우 변 중심, 안쪽 2cm) 직접 사용.

        L_cam, R_cam: 카메라 좌표 grip 점
        box_h_m: 측정된 박스 높이 (잡는 높이 결정용)
        top_center_cam: 윗면 중심
        box_d_m: 측정된 윗면 앞뒤 길이 = 윗면 좌/우 변 길이 (잡는 x 결정용, grab_inset)
        """
        print("[GRAB-BOX] 시작")

        Lx, Ly, Lz = cam_to_ik(L_cam[0], L_cam[1], L_cam[2])
        Rx, Ry, Rz = cam_to_ik(R_cam[0], R_cam[1], R_cam[2])

        # 중심 (grab_x_base)
        if top_center_cam is not None:
            cx, cy, cz = cam_to_ik(top_center_cam[0],
                                          top_center_cam[1],
                                          top_center_cam[2])
        else:
            cx, cy, cz = (Lx+Rx)/2, (Ly+Ry)/2, (Lz+Rz)/2

        # 재감지 — 0.6 s 뒤 detect_box /pose 를 한 번 더 읽어 최신 박스 위치로
        self._stage("재검출")
        if self.redetect is not None:
            time.sleep(0.6)
            d = self.redetect()
            if d and d.get("L") and d.get("R"):
                L_cam, R_cam = d["L"], d["R"]
                if d.get("box_h"): box_h_m = d["box_h"]
                if d.get("box_d"): box_d_m = d["box_d"]
                if d.get("top_center"): top_center_cam = d["top_center"]
                Lx, Ly, Lz = cam_to_ik(L_cam[0], L_cam[1], L_cam[2])
                Rx, Ry, Rz = cam_to_ik(R_cam[0], R_cam[1], R_cam[2])
                if top_center_cam is not None:
                    cx, cy, cz = cam_to_ik(*top_center_cam)
                else:
                    cx, cy, cz = (Lx+Rx)/2, (Ly+Ry)/2, (Lz+Rz)/2
                print(f"[GRAB-BOX] 재감지 center=[{cx:.3f},{cy:.3f},{cz:.3f}]")
            else:
                print("[GRAB-BOX] 재감지 실패 — 원래 좌표 사용")

        # 손 x 를 박스 중심보다 몸쪽으로 (grab_inset) — L/R/중심을 같이 옮겨 접근·하강·잡기·놓기가 모두 같은 x
        inset = grab_inset(box_d_m)
        if inset:
            Lx -= inset; Rx -= inset; cx -= inset
            d_txt = f"D {box_d_m*100:.0f} cm" if box_d_m else "D 미측정 → 고정값"
            print(f"[GRAB-BOX] 손 x 박스 중심보다 {inset*100:.1f} cm 몸쪽 ({d_txt}, L x {Lx:.3f}, R x {Rx:.3f})")

        # 잡는 높이: 윗면(=L/R z)에서 박스 H 절반 내려 옆면 중간
        h = box_h_m if box_h_m else 0.065
        top_z = (Lz + Rz) / 2
        grab_z  = top_z - h / 2 + GRAB_Z_OFFSET
        above_z = top_z + 0.10
        lift_z  = top_z + LIFT_ABOVE

        # 박스 회전 (위에서 볼 때 반시계 + = 오른쪽 변이 더 앞): 돌지 않은 박스면 L→R = (0, −W) → 0°
        box_yaw = float(np.degrees(np.arctan2(Rx - Lx, Ly - Ry)))
        if abs(box_yaw) > MAX_BOX_YAW_DEG:
            print(f"[GRAB-BOX] ❌ 박스가 {box_yaw:+.1f}° 돌아 있음 (한계 ±{MAX_BOX_YAW_DEG:.0f}°) — 잡지 않음. 박스를 반듯하게 놓아 주세요")
            return
        yaw_add = box_yaw if BOX_YAW_FOLLOW else 0.0
        l_rot, r_rot = self._wrist_quats()               # 기본 자세 — ⑥' 대칭 정렬부터 (박스를 반듯하게 편 뒤)
        gl_rot, gr_rot = self._wrist_quats(yaw_add)      # 잡기까지 — 손바닥을 박스 옆면과 평행하게
        print(f"[GRAB-BOX] 박스 회전 {box_yaw:+.1f}° → 손목 yaw {'+' if yaw_add >= 0 else ''}{yaw_add:.1f}° (접근~당기기), "
              f"대칭 정렬에서 기본값으로")
        self._say("start")

        # === L/R 실제 좌표를 직접 손 목표로 사용 (기울어진 박스 대응) ===
        # 왼손 = L점, 오른손 = R점 (각 변의 실제 위치)
        # 접근: L/R에서 바깥으로 더 벌려 위에서 내려옴
        #   바깥 방향 = 중심(cx,cy)에서 L/R로 향하는 단위벡터
        def outward(px, py):
            dx, dy = px - cx, py - cy
            n = (dx*dx + dy*dy) ** 0.5
            return (dx/n, dy/n) if n > 1e-6 else (0.0, 0.0)
        oLx, oLy = outward(Lx, Ly)
        oRx, oRy = outward(Rx, Ry)

        # 접근점 (L/R 에서 옆으로만 +APPROACH_EXTRA, x 는 L/R 그대로)
        #   1.14: 박스 옆면 바깥 방향(oL/oR) 대신 순수 좌우(±y) — 비스듬한 박스에서 먼 쪽 손이 옆면 방향으로 더 앞으로 나가
        #         도달 한계를 넘던 것(20° · x 0.40 에서 앞쪽 손 x 0.49 → ⑤ 측면 하강 4 cm 부족)을 막음. 반듯한 박스면 거의 같음
        appL = [Lx, Ly + APPROACH_EXTRA + LEFT_HAND_Y_OFFSET, above_z]
        appR = [Rx, Ry - APPROACH_EXTRA, above_z]
        self._stage("위쪽 접근")
        if not self._move(appL, appR, 1.5, "④ 위쪽 접근", gl_rot, gr_rot): return
        time.sleep(0.2)

        # 하강 (같은 XY, grab_z로)
        appL[2] = grab_z; appR[2] = grab_z
        self._stage("측면 하강")
        if not self._move(appL, appR, 1.0, "⑤ 측면 하강", gl_rot, gr_rot): return
        time.sleep(0.2)

        # 잡기 — 실제 L/R 점 + X는 몸쪽으로 당김(GRAB_X_OFFSET)
        #   + 조이기: L/R 에서 박스 안쪽(중심 쪽)으로 GRAB_SQUEEZE — 팔은 위치 제어라 목표가 박스 안으로 들어간 만큼 쥠.
        #     이후 대칭 정렬·들기·건네기·제자리 놓기는 이 손 간격 그대로 (align_to_hands), 손 벌림에서 풀림
        gripL = [Lx + GRAB_X_OFFSET - oLx*GRAB_SQUEEZE, Ly - oLy*GRAB_SQUEEZE, grab_z]     # 왼손 보정 없음 — 양손 같게 조임
        gripR = [Rx + GRAB_X_OFFSET - oRx*GRAB_SQUEEZE, Ry - oRy*GRAB_SQUEEZE, grab_z]
        if GRAB_SQUEEZE:
            print(f"[GRAB-BOX] 조이기 각 손 {GRAB_SQUEEZE*100:.1f} cm 안쪽 — 손 간격 {abs(gripL[1] - gripR[1])*100:.1f} cm "
                  f"(L/R 점 간격 {abs(Ly - Ry)*100:.1f} cm)")
        self._stage("잡기")
        if not self._move(gripL, gripR, 2.5, "⑥ 잡기", gl_rot, gr_rot): return
        time.sleep(1.0)

        # 끌어당기기 — 잡은 뒤 테이블 위에서 몸쪽으로 (팔이 덜 뻗은 자세에서 들기 위해)
        if PULL_X != 0.0:
            pullL = [gripL[0] + PULL_X, gripL[1], grab_z]
            pullR = [gripR[0] + PULL_X, gripR[1], grab_z]
            if not self._move(pullL, pullR, 1.5, "⑥'' 끌어당기기", gl_rot, gr_rot): return
            time.sleep(0.2)

        # 대칭 정렬용 파라미터: 잡은 뒤 양손을 평행/대칭으로 정리
        if ALIGN_TO_HANDS:
            # 실제로 잡은 손 위치 기준 — 박스 중심 추정(cx, cy)과 L/R 점이 어긋나도 잡은 손이 튀지 않게.
            #   x = 양손 x 평균 (+당긴 거리), y = 양손 중심 기준 대칭
            grab_x_base = (gripL[0] + gripR[0]) / 2 + PULL_X
            yL = gripL[1]
            yc = (yL + gripR[1]) / 2
            grp_off_L = yL - yc
            grp_off_R = yc - gripR[1]
            if yaw_add:
                # 돌아 있던 박스를 ⑥' 에서 반듯하게 펴도 쥔 폭 그대로 — y 차이(W·cos) 대신 두 손 사이 거리(W)의 절반
                #   (y 차이로 두면 펴는 동안 각 손이 W(1−cos)/2 더 조임: 28 cm · 25° 면 1.3 cm)
                grp_off_L = grp_off_R = 0.5 * float(np.hypot(gripL[0] - gripR[0], yL - gripR[1]))
        else:
            grab_x_base = cx + GRAB_X_OFFSET + PULL_X
            grp_off_L = abs(Ly - cy)
            grp_off_R = abs(Ry - cy)
            yc = 0.0
        self._finish_sequence(grab_x_base, grp_off_L, grp_off_R,
                              grab_z, lift_z, l_rot, r_rot, yc)



try:
    from ctrl.mandro3 import HandController, motions as hand_motions
    HAND_AVAILABLE = True
except ImportError:
    HAND_AVAILABLE = False

try:
    from ctrl.text_to_speech import TextToSpeech
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False



# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmHttpClient]        = None   # arm_server(50022) 클라이언트
hand: Optional[object]               = None
tts:  Optional[object]               = None
grab: Optional[GrabController]       = None

is_running = False
STOP_FLAG  = False

# 잡기 모드 게이트
ACTIVE_MODE = "none"          # "none" | "box"
grab_busy   = False
grab_lock   = threading.Lock()

# arm_sdk 제어권 상태 (기동 시 motion_mode=True 로 weight=1 이므로 hold)
#   hold    : arm_sdk 가 팔 점유 (weight 1). 잡기/IK 가능.
#   release : arm_sdk 반납 (weight 0) — 팔은 로봇 FSM(703) 이 잡음. 팔 지령 무효.
ARM_MODE = "hold"             # "hold" | "release"
_arm_switching = False

# ---- 기본 자세 (Home / Stop / Grab Mode OFF 공통) ----
# 팔 각도는 아래 DEFAULT_ARM_DEG. 허리는 중립.
PARK_WAIST_DEG = [0.0, 0.0, 0.0]   # yaw, roll, pitch


def _park_arm(duration=2.0):
    """기본 자세로 복귀 — 허리 중립 + 팔 BOOT_ARM_DEG. IK 미사용.

    Home / Stop / Grab Mode OFF 가 모두 이 자세를 쓴다.
    """
    if not arm:
        return False
    arm.move_waist_smooth(yaw=PARK_WAIST_DEG[0], roll=PARK_WAIST_DEG[1],
                          pitch=PARK_WAIST_DEG[2], duration=duration)
    arm.move_joints_smooth(BOOT_ARM_DEG, duration)
    return True


# ---- 기본 자세 팔 각도 (Home / Stop / Grab OFF 공통) ----
# 실측 기본 자세 (robot.yaml default_arm_deg).
# 순서: shoulder P/R/Y, elbow, wrist R/P/Y  → 좌 7 + 우 7
DEFAULT_ARM_DEG = [float(v) for v in robot_env.CFG["default_arm_deg"]]   # robot.yaml default_arm_deg

# True  : 기동 시점 실측각을 캡처해 그 자세로 복귀 (재기동 자세에 따라 달라짐)
# False : 위 DEFAULT_ARM_DEG 고정 (항상 같은 자세 — 권장)
USE_BOOT_CAPTURE = False

BOOT_ARM_DEG = list(DEFAULT_ARM_DEG)

def _freeze_arm():
    """팔·허리를 현재 실측 자세로 고정한다. (arm_server /freeze)"""
    if not arm:
        return
    try:
        arm.freeze()
    except Exception as e:
        print(f"[STOP] freeze 실패: {e}")


# ==========================================
# Pydantic
# ==========================================
class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class HandMotionData(BaseModel):
    hand: str
    motion: str

class MotionFrame(BaseModel):
    duration: float
    pose: Optional[PoseData] = None
    hand_motion: Optional[HandMotionData] = None

class IKMotionFrame(BaseModel):
    duration: float
    left_xyz: Optional[List[float]] = None
    right_xyz: Optional[List[float]] = None
    left_rpy: Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    hand_motion: Optional[HandMotionData] = None

# 잡기 요청 (인식 파일 → robot_server)
class GrabRequest(BaseModel):
    type: str = "cardboard"                # 박스만 ("cardboard")
    L:    Optional[List[float]] = None     # box 왼쪽 grip (카메라)
    R:    Optional[List[float]] = None     # box 오른쪽 grip
    top_center: Optional[List[float]] = None
    box_h: Optional[float] = None          # box 높이 (m)
    box_d: Optional[float] = None          # box 윗면 앞뒤 길이 = 윗면 좌/우 변 길이 (m)


# ==========================================
# 헬퍼 (run_motion 동일)
# ==========================================
def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, y, z).normalized()

def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    left_rot  = rpy_to_quaternion(*left_rpy)  if left_rpy  and any(v != 0 for v in left_rpy)  else None
    right_rot = rpy_to_quaternion(*right_rpy) if right_rpy and any(v != 0 for v in right_rpy) else None
    arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)

def execute_hand_motion_sync(h: str, motion: str):
    if hand:
        hand.send_motion(motion, selector=h)



def _stop_and_park(duration=1.5):
    """진행 중 보간을 끊고 기본 자세로 복귀한다."""
    if not arm:
        return
    try:
        arm.stop_motion()          # 진행 중 보간 중단
        time.sleep(0.05)
    except Exception:
        pass
    try:
        _park_arm(duration)
    except Exception as e:
        print(f"[STOP] 자세 복귀 실패: {e}")


def _connect_arm(quiet=False):
    """arm_server(50022) 연결 + park 복귀 자세 결정 → ArmHttpClient 또는 None."""
    global BOOT_ARM_DEG
    try:
        a = ArmHttpClient()
    except Exception as e:
        if not quiet:
            print(f"⚠️ Arm 실패: {e} — 3초마다 다시 연결 시도")
        return None
    if USE_BOOT_CAPTURE:
        try:
            BOOT_ARM_DEG = [round(float(v), 3)
                            for v in np.degrees(a.arm_ctrl.get_current_dual_arm_q())]
            print(f"✅ park 자세: 기동 실측 캡처 {BOOT_ARM_DEG}")
        except Exception as e:
            BOOT_ARM_DEG = list(DEFAULT_ARM_DEG)
            print(f"⚠️ 캡처 실패({e}) — 기본 자세 사용")
    else:
        print(f"✅ park 자세: 고정값 사용")
        try:
            cur = np.degrees(a.arm_ctrl.get_current_dual_arm_q())
            diff = float(np.max(np.abs(cur - np.array(BOOT_ARM_DEG))))
            print(f"   현재 팔과의 최대 편차 {diff:.1f}도")
        except Exception:
            pass
    print("✅ Arm 초기화 (arm_sdk: hold)")
    return a


def _arm_watch():
    """기동 때 arm_server 가 안 떠 있었으면(start_robot.sh 순서·로봇 재부팅) 붙을 때까지 3초마다 다시 연결."""
    global arm
    while True:
        time.sleep(3.0)
        if arm is not None or grab is None:
            continue
        a = _connect_arm(quiet=True)
        if a is not None:
            arm = a
            grab.arm, grab.robot_available = a, True
            print("✅ arm_server 다시 연결 — 잡기 가능")


def arm_warnings():
    """Box 를 눌러도 팔이 안 움직일 이유 목록 (없으면 [])."""
    w = []
    if arm is None:
        w.append("arm_server 미연결 — 팔이 안 움직임 (arm_server 로그 확인, 3초마다 다시 연결 시도)")
    elif _sync_arm_mode().get("mode", ARM_MODE) != "hold":
        w.append("제어권 반납(release) 상태 — 팔이 안 움직임 (arm_server 재시작 또는 POST /arm_hold)")
    f = SYS.fsm_warning()
    if f:
        w.append(f)
    return w


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm, hand, tts, grab, ACTIVE_MODE, BOOT_ARM_DEG

    print("[robot_server] 시작")
    HW.start()                     # CPU·GPU·NPU 사용률 (제어 화면 위 칩, GET /hw)
    robot_env.dds_init()

    arm = _connect_arm()          # 실패하면 None — _arm_watch 가 3초마다 다시 시도

    if HAND_AVAILABLE:
        try:
            hand = HandController('/dev/ttyACM0')
            print("✅ 손 초기화")
        except Exception as e:
            print(f"⚠️ 손 실패: {e}")

    if TTS_AVAILABLE:
        try:
            tts = TextToSpeech(verbose=False)
            print("✅ TTS 초기화")
        except Exception as e:
            print(f"⚠️ TTS 실패: {e}")

    # 잡기 컨트롤러 (arm 주입)
    def _speak(text):
        if tts:
            tts.speak(text, voice=TTS_VOICE, lang="en")
        else:
            print(f"[TTS-DUMMY] {text}")
    grab = GrabController(arm=arm, speak=_speak,
                          robot_available=(arm is not None))

    # 재감지: detect_box /pose 를 다시 읽음
    def _redetect():
        import urllib.request as _u
        try:
            raw = _u.urlopen("http://localhost:50010/pose", timeout=1.0).read()
            d = json.loads(raw)
            return d if d.get("found") else None
        except Exception as e:
            print(f"[REDETECT] 실패: {e}")
            return None
    grab.redetect = _redetect
    print("✅ GrabController 준비")
    threading.Thread(target=_arm_watch, daemon=True, name="arm_watch").start()
    SYS.start()                    # 로봇 FSM · 서버 상태 (제어 화면 System 카드, GET /system)

    print("[robot_server] 준비 완료  http://localhost:50000/")
    yield

    # ==========================================
    # 안전 종료 시퀀스 (팔 자세 유지, 제어권만 반납)
    # ==========================================
    print("[shutdown] 종료 시퀀스 시작")
    t_shutdown = time.time()

    # ACTIVE_MODE 자동 트리거 방지를 위해 none으로 전환
    ACTIVE_MODE = "none"

    # 잡기 진행 중이면 잠깐 대기
    busy_deadline = time.time() + 3.0
    while grab_busy and time.time() < busy_deadline:
        time.sleep(0.1)
    if grab_busy:
        print("[shutdown] grab 진행 중이지만 시간 초과 — 강제 진행")

    # weight 반납은 arm_server 책임 — robot_server 는 팔을 건드리지 않는다

    time.sleep(0.3)
    print(f"[robot_server] 종료 (총 {time.time()-t_shutdown:.2f}초)")
    os._exit(0)


HW = HwUsage(period=1.0)          # 이 PC 의 CPU / GPU / NPU 사용률 (ctrl/hw_usage.py)
SYS = SysWatch(robot_env.CFG.get("fsm"), period=2.0, sim=robot_env.SIM)   # 로봇 FSM · 서버 상태 (ctrl/sys_watch.py)

app = FastAPI(title="H2 Robot Server", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


# ==========================================
# 모션 실행 (run_motion 동일)
# ==========================================
async def _execute_frames(frames: List[MotionFrame]):
    global is_running, STOP_FLAG
    is_running = True; STOP_FLAG = False
    loop = asyncio.get_running_loop()
    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG: break
            print(f"[Runner] 프레임 {i+1}/{len(frames)}")
            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(None, execute_hand_motion_sync,
                                                    frame.hand_motion.hand, frame.hand_motion.motion)
            if frame.pose and frame.pose.targets and arm:
                # HTTP 스냅샷 (ctrl_lock 은 호환용 더미)
                with arm.arm_ctrl.ctrl_lock:
                    arm_targets = np.degrees(arm.arm_ctrl.q_target.copy())
                try:
                    with arm.arm_ctrl.ctrl_lock:
                        waist_targets = np.degrees(getattr(arm.arm_ctrl,'waist_q_target',np.zeros(3)).copy())
                except:
                    waist_targets = np.zeros(3)
                has_waist = False
                for t in frame.pose.targets:
                    if 0 <= t.motor_index <= 2:
                        waist_targets[t.motor_index] = t.target_degree; has_waist = True
                    elif 15 <= t.motor_index <= 28:
                        arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree
                tasks = [loop.run_in_executor(None, arm.move_joints_smooth, arm_targets.tolist(), frame.duration)]
                if has_waist:
                    tasks.append(loop.run_in_executor(None, arm.move_waist_smooth,
                        float(waist_targets[0]),float(waist_targets[1]),float(waist_targets[2]),frame.duration))
                await asyncio.gather(*tasks)
            else:
                await asyncio.sleep(frame.duration)   # 손만 / 대기 (보행 프레임은 H2 에서 무시)
            if hand_future: await hand_future
    finally:
        is_running = False

async def _execute_ik_frames(frames: List[IKMotionFrame]):
    global is_running, STOP_FLAG
    is_running = True; STOP_FLAG = False
    loop = asyncio.get_running_loop()
    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG: break
            print(f"[IK Runner] 프레임 {i+1}/{len(frames)}")
            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(None, execute_hand_motion_sync,
                                                    frame.hand_motion.hand, frame.hand_motion.motion)
            if frame.left_xyz and frame.right_xyz and arm:
                lr = frame.left_rpy or [0.0,0.0,0.0]
                rr = frame.right_rpy or [0.0,0.0,0.0]
                await loop.run_in_executor(None, move_hands_with_rotation,
                    frame.left_xyz, frame.right_xyz, lr, rr, frame.duration, 100)
            else:
                await asyncio.sleep(frame.duration)   # 손만 / 대기 (보행 프레임은 H2 에서 무시)
            if hand_future: await hand_future
    finally:
        is_running = False

# ==========================================
# 잡기 — 모드 게이트 + grab_at
# ==========================================
_pose_thread: Optional[threading.Thread] = None   # 모드 전환 때 띄운 대기/park 자세 이동 스레드


def _run_grab(req: GrabRequest):
    """별도 스레드에서 잡기 시퀀스 실행."""
    global grab_busy
    # 모드 전환 직후 대기 자세 이동이 아직 진행 중이면 끝날 때까지 기다림 (겹치면 arm_server 가 409 로 거부해 잡기가 중단됨)
    t = _pose_thread
    if t is not None and t.is_alive():
        print("[GRAB] 대기 자세 이동 중 — 끝난 뒤 시작")
        t.join(timeout=8.0)
    grab.begin_log()
    try:
        if req.type == "cardboard":
            grab.grab_box(req.L, req.R, box_h_m=req.box_h,
                          top_center_cam=req.top_center, box_d_m=req.box_d)
        else:
            print(f"[GRAB] 알 수 없는 type: {req.type}")
    except Exception:
        import traceback
        print("[GRAB] 예외 발생:")
        traceback.print_exc()
    finally:
        grab.end_log()
        with grab_lock:
            grab_busy = False
        grab.stage = None
        print("[GRAB] 완료")


@app.post("/grab_at", summary="인식 파일이 좌표 주면 잡기 실행")
async def grab_at(req: GrabRequest):
    global grab_busy
    # 모드 게이트
    if ACTIVE_MODE != "box":
        return JSONResponse({"ok": False, "reason": f"mode={ACTIVE_MODE}"})
    if req.type != "cardboard":
        return JSONResponse({"ok": False, "reason": f"type={req.type} (박스만)"})
    # 중복 방지
    with grab_lock:
        if grab_busy or is_running:
            return JSONResponse({"ok": False, "reason": "busy"})
        grab_busy = True
    threading.Thread(target=_run_grab, args=(req,), daemon=True).start()
    return JSONResponse({"ok": True, "type": req.type})


@app.get("/active_mode", summary="현재 잡기 모드 (인식 파일이 폴링)")
async def get_active_mode():
    return {"mode": ACTIVE_MODE, "busy": grab_busy, "is_running": is_running}


@app.post("/set_mode", summary="잡기 모드 전환 (none/box)")
async def set_mode(mode: str):
    global ACTIVE_MODE
    if mode not in ("none", "box"):
        return JSONResponse({"ok": False, "error": f"invalid: {mode} (none / box)"})
    prev = ACTIVE_MODE
    ACTIVE_MODE = mode
    print(f"[MODE] {prev} → {mode}")

    # 모드 전환 시 대기 자세 (잡기 중이 아닐 때만)
    global _pose_thread
    if not grab_busy and not is_running and grab is not None:
        def _pose():
            if mode == "box":
                grab.ready()      # 팔 들어 대기
            else:
                grab.park()       # 팔 내림
        _pose_thread = threading.Thread(target=_pose, daemon=True)
        _pose_thread.start()

    warn = await asyncio.get_running_loop().run_in_executor(None, arm_warnings) if mode == "box" else []
    for w in warn:
        print(f"[MODE] ⚠️ {w}")
    return {"ok": True, "mode": ACTIVE_MODE, "warn": warn}


def _stage_info():
    st = grab.stage if grab else None
    return {"stage": st, "stage_idx": GRAB_STAGES.index(st) if st in GRAB_STAGES else -1,
            "stages": GRAB_STAGES}


@app.get("/grab_status")
async def grab_status():
    return {"mode": ACTIVE_MODE, "busy": grab_busy, "is_running": is_running, **_stage_info()}


@app.get("/grab_history", summary="최근 잡기 5회 단계별 소요 시간 (s)")
async def grab_history():
    return {"stages": GRAB_STAGES, "history": list(grab.history) if grab else []}


@app.get("/viz", summary="3D 시각화용 — 인식 박스 + 손 목표 (torso_link 기준, m)")
def viz():
    """dashboard 3D 뷰어가 폴링. 박스는 detect_box /pose(카메라 좌표)를 torso 로 변환,
    손 목표는 마지막 IK 목표(pelvis 기준)를 torso 기준으로 변환."""
    import urllib.request as _u
    box = other = cmp = None
    if ACTIVE_MODE == "box":
        try:
            d = json.loads(_u.urlopen("http://localhost:50010/pose", timeout=0.3).read())
            if d.get("found") and d.get("L") and d.get("R"):
                box = {"L": list(camera_to_torso(*d["L"])), "R": list(camera_to_torso(*d["R"])),
                       "top": list(camera_to_torso(*d["top_center"])) if d.get("top_center") else None,
                       "h": d.get("box_h"), "method": d.get("method", "legacy")}
                o = d.get("other")   # 비교용: 잡기에 안 쓰는 다른 추정 방식 (표시 전용)
                if o and o.get("L") and o.get("R"):
                    other = {"L": list(camera_to_torso(*o["L"])), "R": list(camera_to_torso(*o["R"])),
                             "top": list(camera_to_torso(*o["top_center"])) if o.get("top_center") else None,
                             "h": o.get("box_h"), "method": o.get("method")}
                cmp = d.get("cmp")
        except Exception:
            box = other = cmp = None
    tg = None
    if grab and grab.targets and grab_busy:
        tg = {"L": ik_to_torso(grab.targets["L"]), "R": ik_to_torso(grab.targets["R"])}
    return {"mode": ACTIVE_MODE, "box": box, "other": other, "cmp": cmp, "targets": tg, **_stage_info()}


@app.get("/set_wrist")
async def set_wrist(l_roll: float=0, l_pitch: float=0, l_yaw: float=0,
                    r_roll: float=0, r_pitch: float=0, r_yaw: float=0):
    grab.wrist_params = {
        'left':  {'roll': l_roll, 'pitch': l_pitch, 'yaw': l_yaw},
        'right': {'roll': r_roll, 'pitch': r_pitch, 'yaw': r_yaw},
    }
    print(f"[WRIST] {grab.wrist_params}")
    return {"success": True, "wrist_params": grab.wrist_params}


@app.get("/set_handover_direction")
async def set_handover_direction(direction: str = "center"):
    if direction not in HANDOVER_MODES:
        return JSONResponse({"success": False, "error": f"invalid: {direction} (center / place)"})
    grab.handover_direction = direction
    print(f"[HANDOVER] {direction}")
    return {"success": True, "direction": direction}


@app.get("/grab_manual")
async def grab_manual():
    """수동 잡기: 현재 모드의 인식 파일 /pose를 GET해서 잡기."""
    import urllib.request
    if ACTIVE_MODE == "none":
        return JSONResponse({"ok": False, "reason": "mode is none"})
    url = "http://localhost:50010/pose"
    try:
        raw = urllib.request.urlopen(url, timeout=1.0).read()
        d = json.loads(raw)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"detect fetch 실패: {e}"})
    if not d.get("found"):
        return JSONResponse({"ok": False, "reason": "검출 없음"})
    req = GrabRequest(**{k: d.get(k) for k in
                         ("type","L","R","top_center","box_h","box_d")
                         if k in d})
    return await grab_at(req)


# ==========================================
# API: 상태 / 모션
# ==========================================
@app.get("/status")
async def status():
    return {"is_running": is_running, "arm_ready": arm is not None,
            "hand_ready": hand is not None,
            "tts_ready": tts is not None, "active_mode": ACTIVE_MODE,
            "grab_busy": grab_busy, **_stage_info()}

def camera_status():
    """System 카드 Camera 줄 — 지금 쓰는 숙임각과 출처 (실기에서 이 PC 실측이 없으면 warn)."""
    deg = robot_env.CAMERA_PITCH_DEG
    yaml_deg = float(robot_env.CAMERA_YAML["pitch_deg"])
    if robot_env.SIM:
        return {"state": "ok", "text": f"{deg:.1f}° (sim)", "why": "시뮬 — 장착 보정 불필요"}
    if robot_env.CAMERA_SOURCE != "robot.yaml":
        when = str((robot_env.LOCAL_CALIB.get("measured") or {}).get("time", "?"))
        return {"state": "ok", "text": f"{deg:.1f}° ({when[5:16]})",
                "why": f"calib_local.yaml 실측 {when} (robot.yaml {yaml_deg:.1f}°). 거치대를 다시 조였으면 check_rsimu --save"}
    return {"state": "warn", "text": f"{deg:.1f}° (robot.yaml)",
            "why": "이 PC 에서 잰 값 없음 — start_robot.sh 끄고 sudo ROBOT=h2 python utils/check_rsimu.py --save"}


@app.get("/system", summary="로봇 FSM · 서버 상태 (2초마다 갱신) + 팔이 안 움직일 이유(warn) + 카메라 숙임각 출처")
def system():                     # def — arm_server 조회가 블로킹이라 스레드풀에서
    return {**SYS.latest(), "warn": arm_warnings(), "camera": camera_status()}

@app.get("/hw", summary="이 PC 의 CPU / GPU / NPU 사용률 [%] (1초마다 갱신, 못 읽으면 pct null + why)")
async def hw_usage():
    return HW.latest()

@app.get("/motions")
async def list_motions():
    files = sorted([f.name for f in MOTIONS_DIR.glob("*.json")])
    return {"motions": files, "directory": str(MOTIONS_DIR)}

@app.post("/motions/run/{filename}")
async def run_motion_by_name(filename: str):
    if is_running or grab_busy:
        raise HTTPException(409, "동작 중")
    filepath = MOTIONS_DIR / filename
    if not filepath.exists():
        raise HTTPException(404, f"파일 없음: {filename}")
    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not data:
        raise HTTPException(400, "빈 모션")
    first = data[0]
    is_ik = "left_xyz" in first or "right_xyz" in first
    if is_ik:
        asyncio.create_task(_execute_ik_frames([IKMotionFrame(**f) for f in data]))
    else:
        asyncio.create_task(_execute_frames([MotionFrame(**f) for f in data]))
    return {"status": "started", "frames": len(data), "format": "ik" if is_ik else "joint"}

@app.post("/run")
async def run_motion(frames: List[MotionFrame]):
    if is_running or grab_busy: raise HTTPException(409, "동작 중")
    if not frames: raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames)}

@app.post("/run_ik")
async def run_ik_motion(frames: List[IKMotionFrame]):
    if is_running or grab_busy: raise HTTPException(409, "동작 중")
    if not frames: raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames)}

@app.post("/run_file", summary="관절값 모션 파일 업로드 후 실행")
async def run_motion_file(file: UploadFile = File(...)):
    if is_running or grab_busy:
        raise HTTPException(409, "동작 중")
    try:
        data   = json.loads(await file.read())
        frames = [MotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}


@app.post("/run_ik_file", summary="IK 모션 파일 업로드 후 실행")
async def run_ik_motion_file(file: UploadFile = File(...)):
    if is_running or grab_busy:
        raise HTTPException(409, "동작 중")
    try:
        data   = json.loads(await file.read())
        frames = [IKMotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}

@app.post("/stop", summary="정지 — 모션 중단 + 기본 자세 복귀")
async def stop_motion():
    """모션·잡기 보간 중단 + 기본 자세 복귀 (팔 BOOT_ARM_DEG)."""
    global STOP_FLAG
    STOP_FLAG = True
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, _stop_and_park)
    return {"status": "stopped", "arm": "home"}

@app.post("/home", summary="자세 복귀 — 기본은 팔 내림. pose=zero면 관절 0도")
async def go_home(pose: str = "park"):
    global STOP_FLAG
    STOP_FLAG = True
    await asyncio.sleep(0.1)
    if not arm:
        return {"status": "skipped", "reason": "arm 미초기화"}
    loop = asyncio.get_running_loop()
    if pose == "zero":
        # 구 동작: 관절 전부 0도 = 앞으로 나란히
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 2.0),
            loop.run_in_executor(None, arm.move_waist_smooth, 0.0, 0.0, WAIST_BASE_PITCH, 2.0))
    else:
        await loop.run_in_executor(None, _park_arm, 2.0)
    return {"status": "home", "pose": pose}

@app.get("/boot_pose", summary="기동 시점 팔 자세 (park 복귀 목표)")
async def get_boot_pose():
    return {"ok": True, "arm_deg": BOOT_ARM_DEG,
            "source": "capture" if USE_BOOT_CAPTURE else "fixed"}


@app.post("/boot_pose/recapture", summary="현재 팔 자세를 park 복귀 목표로 재설정")
async def recapture_boot_pose():
    global BOOT_ARM_DEG
    if not arm or not getattr(arm, "arm_ctrl", None):
        raise HTTPException(503, "Arm 미초기화")
    if grab_busy or is_running:
        raise HTTPException(409, "동작 중")
    BOOT_ARM_DEG = [round(float(v), 3)
                    for v in np.degrees(arm.arm_ctrl.get_current_dual_arm_q())]
    print(f"[BOOT_POSE] 재설정: {BOOT_ARM_DEG}")
    return {"ok": True, "arm_deg": BOOT_ARM_DEG}


# ==========================================
# arm_sdk 제어권 토글 (hold / release)
# ==========================================
def _do_arm_release():
    """arm_server /release — 기본자세 보간 후 weight 1->0."""
    global ARM_MODE, _arm_switching
    try:
        arm.release(2.0, BOOT_ARM_DEG)
        ARM_MODE = "release"
        print("[ARM] release 완료 — arm_sdk weight 0")
    except Exception as e:
        print(f"[ARM] release 실패: {e}")
        _sync_arm_mode()
    finally:
        _arm_switching = False

def _do_arm_hold():
    """arm_server /hold — 실측각 동기화 후 weight 0->1."""
    global ARM_MODE, _arm_switching
    try:
        arm.hold(2.0)
        ARM_MODE = "hold"
        print("[ARM] hold 완료 — arm_sdk 가 팔 점유")
    except Exception as e:
        print(f"[ARM] hold 실패: {e}")
        _sync_arm_mode()
    finally:
        _arm_switching = False

def _sync_arm_mode():
    """arm_server 실제 상태로 ARM_MODE 동기화 (단일 진실원 = arm_server)."""
    global ARM_MODE
    try:
        st = arm.status()
        ARM_MODE = st.get("mode", ARM_MODE)
        return st
    except Exception:
        return {}


@app.get("/arm_mode", summary="arm_sdk 제어권 상태 (arm_server 기준)")
async def arm_mode():
    st = _sync_arm_mode() if arm else {}
    return {"mode": ARM_MODE, "weight": st.get("weight"),
            "switching": _arm_switching or st.get("switching", False)}

@app.post("/arm_release", summary="제어권 반납 — arm_sdk weight 0 (팔은 로봇 FSM 이 잡음)")
async def arm_release():
    global _arm_switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if grab_busy or is_running:
        raise HTTPException(409, "동작 중 - 정지 후 전환")
    if _arm_switching:
        raise HTTPException(409, "전환 중")
    if ARM_MODE == "release":
        return {"ok": True, "mode": ARM_MODE}
    _arm_switching = True
    asyncio.get_running_loop().run_in_executor(None, _do_arm_release)
    return {"ok": True, "mode": "release", "switching": True}

@app.post("/arm_hold", summary="제어권 점유 — 잡기/IK 모드")
async def arm_hold():
    global _arm_switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if grab_busy or is_running:
        raise HTTPException(409, "동작 중 - 정지 후 전환")
    if _arm_switching:
        raise HTTPException(409, "전환 중")
    if ARM_MODE == "hold":
        return {"ok": True, "mode": ARM_MODE}
    _arm_switching = True
    asyncio.get_running_loop().run_in_executor(None, _do_arm_hold)
    return {"ok": True, "mode": "hold", "switching": True}


# ==========================================
# 웹 UI — robot_web.html 읽어 viewer 코드 삽입
# ==========================================
WEB_HTML_PATH = os.path.join(current_dir, "robot_web.html")

@app.get("/i18n.js", include_in_schema=False)
async def i18n_js():
    return FileResponse(os.path.join(current_dir, "i18n.js"), media_type="application/javascript")


def _ui_inject():
    """페이지에 서버 상태 전달 (window.UI) + 모션 파일이 없으면 Motions 카드 숨김."""
    has_motions = any(MOTIONS_DIR.glob("*.json"))
    ui = {"robot": robot_env.ROBOT, "motions": has_motions,
          "handover": grab.handover_direction if grab else DEFAULT_HANDOVER,
          "wrist": grab.wrist_params if grab else None}
    css = "" if has_motions else "#card-motions{display:none!important}"
    return f"<script>window.UI={json.dumps(ui, ensure_ascii=False)};</script><style>{css}</style>"


@app.get("/", include_in_schema=False)
async def index():
    html = open(WEB_HTML_PATH, encoding="utf-8").read()
    return HTMLResponse(html.replace("</head>", _ui_inject() + "</head>", 1))


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50000, timeout_graceful_shutdown=2)
