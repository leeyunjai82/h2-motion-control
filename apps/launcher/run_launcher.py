#!/usr/bin/env python3
"""
apps/launcher/run_launcher.py — 홈: H2 자세(FSM) + 기능 (박스 · 에디터 · 모방학습) 시작 · 정지 · 종료 (포트 80)

  ./launcher.sh             (start_fsm.sh 와 같이 sudo 로 tv 환경 python 실행)
  → http://<robot-ip>/   (포트 80 — root 실행 필요, launcher.sh 의 sudo)
  sudo <tv python> apps/launcher/run_launcher.py [robot]   (robot 생략 시 h2 — 이 저장소는 H2 전용)

자세 — FSM 단계별 버튼 (LocoClient.SetFsmId 직접 호출, 연속 시퀀스 없음)
  FSM 번호·이름·버튼 라벨은 robots/h2/robot.yaml fsm (damp/lock/run, names, labels) 에서 만든다.
  사람이 로봇 상태를 보고 한 단계씩 누른다. init_fsm.py 의 stand = 1 → 4 → 703.
  버튼을 누르면 5초 카운트다운 후 전송 (취소 가능, 대기 중 Damp 는 눌러서 교체 가능).
  전송 직전에 허용 조건을 다시 검사한다.
  현재 FSM(GetFsmId)에서 갈 수 있는 단계만 허용 (H2 값):
  · 1   Damp                 : 항상 (서 있으면 넘어짐 — 경고창)
  · 4   FixStand             : FSM 1 에서 / 703 에서(Robot 정지 상태만)
  · 703 PhaseWalk (balance)  : FSM 4 에서 — 제자리 밸런스 서기 (H2 는 보행 없음, arm_sdk 동작)
  · Sit 버튼 없음 (H2 는 거치대에 건 채 운용 — 끝낼 때 Damp, 사용자 2026-10-09). 터미널 ./start_fsm.sh sit 은 그대로
  · FSM 조회 불가 시 순서 제한 없음 (Robot 실행 중 4 차단만 유지) — 순서는 사람이 지킨다
  · Robot 시작은 FSM 703 에서만

  ./start_fsm.sh / tools/init_fsm.py 는 수정하지 않았다 — 터미널에서 따로 쓸 수 있다.
  단, 둘을 동시에 쓰면 서로의 진행을 모른다. 한쪽만 쓸 것.

기능 (apps/*/app.yaml 을 읽어 카드로 그림 — 새 기능은 app.yaml 하나 추가)
  · 한 번에 하나만 실행 (팔 · D435i 는 프로세스 하나만). 실기 모드(real)는 FSM 703 에서만 시작
  · 정지 : 각 start 스크립트에 SIGTERM (= Ctrl+C 와 같은 종료 순서)
  · 종료 : 정지 → FixStand → Damp (각 단계 카운트다운 · 취소 가능). 강제로 꺼졌으면 자동 Damp 안 함
  · 미니 PC 학습 : 런처(root)가 GPU 최대 클럭을 제한하고 끝나면 되돌림 (sudo 비밀번호 대신)

권한
  · launcher 는 root 로 돈다 (launcher.sh 의 sudo). FSM RPC 도 root 로 호출 (start_fsm.sh 와 동일).
  · start_robot.sh 는 sudo 를 실행한 원래 사용자(SUDO_USER)로 내려서 실행한다.
    root 로 띄우면 logs/ 등 파일 소유자가 root 가 되어 수동 실행 시 쓰기 실패.
주의
  · 웹 버튼은 비상정지가 아니다. 물리 E-STOP / 리모컨을 항상 손에 둘 것.
"""

import html
import json
import os
import pwd
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque

# 로봇 선택 — launcher.sh 가 sudo 뒤에서 인자로 넘긴다 (sudo 가 환경변수를 지우므로).
#   sudo <tv python> apps/launcher/run_launcher.py <robot>      (H2 전용 — 인자가 없거나 비어 있으면 h2)
# os.environ 에 넣어 두면 start_robot.sh(_user_env) 에도 그대로 전달된다.
os.environ["ROBOT"] = (sys.argv[1].strip() if len(sys.argv) > 1 else "") or "h2"
import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env   # ROBOT 미지정/미지원이면 여기서 종료
from pitron import ports  # noqa: E402
from pitron import paths

import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, FileResponse

PORT = ports.PORTS["launcher"]                   # 1024 미만 → root 필요 (launcher.sh 가 sudo 로 실행)
ROOT = paths.REPO_ROOT
LOG_DIR = paths.LOGS_DIR
ROBOT_STOP_WAIT = 20.0      # start_robot.sh 종료 시퀀스(최대 8초 + sweep) 여유

# sudo 로 실행됐으면 원래 사용자 정보 (start_robot.sh 를 이 사용자로 실행)
SUDO_USER = os.environ.get("SUDO_USER") if os.geteuid() == 0 else None
USER_PW = pwd.getpwnam(SUDO_USER) if SUDO_USER else None


def _own(path):
    """root 로 만든 파일/폴더를 원래 사용자 소유로."""
    if USER_PW:
        try:
            os.chown(path, USER_PW.pw_uid, USER_PW.pw_gid)
        except OSError:
            pass


os.makedirs(LOG_DIR, exist_ok=True)
_own(LOG_DIR)


def _as_user():
    """자식 프로세스를 원래 사용자 권한으로 (그룹 포함 — video/plugdev 등 장치 접근)."""
    os.initgroups(USER_PW.pw_name, USER_PW.pw_gid)
    os.setgid(USER_PW.pw_gid)
    os.setuid(USER_PW.pw_uid)


def _user_env():
    env = dict(os.environ)
    env.update(HOME=USER_PW.pw_dir, USER=USER_PW.pw_name, LOGNAME=USER_PW.pw_name)
    for k in ("SUDO_USER", "SUDO_UID", "SUDO_GID", "SUDO_COMMAND"):
        env.pop(k, None)
    return env


def _logfile(name):
    path = os.path.join(LOG_DIR, f"launcher_{name}_{time.strftime('%Y%m%d')}.log")
    f = open(path, "a", encoding="utf-8")
    _own(path)
    return f


# ==========================================
# FSM (자세) — LocoClient 직접 호출, 단계별
# ==========================================
# FSM ID — robots/h2/robot.yaml fsm (H2: 1 Damp / 4 FixStand / 703 PhaseWalk(제자리 밸런스)). Sit 은 launcher 에 없음
#   화면 버튼·안내 문구·JS 상수는 아래 값으로 만든다 (_render_html).
_F = robot_env.FSM
FSM_NAME = {int(k): str(v) for k, v in _F["names"].items()}
FSM_BAL = {int(k): True for k in _F["balance"]}   # 밸런스 제어 여부 (나머지 없음)
STANDING = {int(k) for k in _F["standing"]}
FSM_DAMP, FSM_LOCK, FSM_RUN = int(_F["damp"]), int(_F["lock"]), int(_F["run"])
_LBL = _F["labels"]
STEPS = (FSM_DAMP, FSM_LOCK, FSM_RUN)


def _check_fsm_cfg():
    """robot.yaml fsm 이 launcher 단계(damp → lock → run)와 맞는지 — 어긋나면 기동 거부."""
    errs = []
    for key, fid in (("damp", FSM_DAMP), ("lock", FSM_LOCK), ("run", FSM_RUN)):
        if fid not in FSM_NAME:
            errs.append(f"fsm.{key}={fid} 가 fsm.names 에 없음")
    if len(set(STEPS)) != len(STEPS):
        errs.append(f"damp/lock/run 중복 {STEPS}")
    if FSM_LOCK not in STANDING or FSM_RUN not in STANDING:
        errs.append(f"lock({FSM_LOCK})·run({FSM_RUN}) 은 fsm.standing {sorted(STANDING)} 에 있어야 함")
    if FSM_DAMP in STANDING:
        errs.append(f"damp({FSM_DAMP}) 는 fsm.standing 에 없어야 함")
    if FSM_RUN not in FSM_BAL:
        errs.append(f"run({FSM_RUN}) 은 fsm.balance {sorted(FSM_BAL)} 에 있어야 함 (arm_sdk 밸런스 서기)")
    for key in ("lock_button", "run_button", "run_enter"):
        if not str((_LBL or {}).get(key) or "").strip():
            errs.append(f"fsm.labels.{key} 없음")
    if errs:
        raise SystemExit(f"[launcher] ❌ robots/{robot_env.ROBOT}/robot.yaml fsm 설정 오류 — " + "; ".join(errs))


_check_fsm_cfg()
POLL_SEC = 1.0
DELAY_SEC = 5.0                   # 버튼 → 전송 지연
API_GET_FSM_ID = 7001             # ROBOT_API_ID_LOCO_GET_FSM_ID (h2_loco_api.py)
ROBOT_BUSY = "Robot 서버 실행 중 — 먼저 [Robot 정지]"


def allowed(target, cur, robot_running):
    """(허용 여부, 거부 사유). UI 버튼 활성화와 서버 검사에 같은 규칙을 쓴다."""
    if target == FSM_DAMP:
        return True, ""
    if cur is None:
        # FSM 조회 불가(펌웨어/SDK 미지원) — 순서 제한 없이 허용, Robot 실행 중 차단만 유지
        if robot_running and target == FSM_LOCK:
            return False, ROBOT_BUSY
        return True, ""
    if target == FSM_LOCK:
        if cur == FSM_DAMP:
            return True, ""
        if cur == FSM_RUN:
            return (False, ROBOT_BUSY) if robot_running else (True, "")
        return False, f"{FSM_LOCK} 는 FSM {FSM_DAMP}({FSM_NAME[FSM_DAMP]}) 또는 {FSM_RUN} 에서만 (현재 {cur})"
    if target == FSM_RUN:
        return (True, "") if cur == FSM_LOCK else (False, f"{FSM_RUN} 은 FSM {FSM_LOCK}({_LBL['lock_button']}) 에서만 (현재 {cur})")
    return False, "알 수 없는 단계"


class FsmCtl:
    def __init__(self):
        self.client = None                     # 명령용 (timeout 10s, init_fsm.py 와 동일)
        self.qclient = None                    # 조회용 (timeout 1s — 폴링이 명령을 오래 막지 않게)
        self.read_lock = threading.Lock()
        self.init_err = None
        self._dds_inited = False
        self.call_lock = threading.Lock()      # 명령 RPC 직렬화
        self.cur = None                        # 마지막으로 읽은 FSM id (None=미확인)
        self.cur_err = None
        self.busy = None                       # 대기/전송 중인 FSM id
        self.fire_at = 0.0                     # 전송 예정 시각
        self.cancel = threading.Event()
        self.last_sent = None                  # 마지막으로 성공 전송한 FSM id (조회 불가 시 참고)
        self.result = None                     # (ok, msg)
        self.lines = deque(maxlen=300)
        self.logf = _logfile("fsm")
        self.lock = threading.Lock()

    def log(self, msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        self.lines.append(line)
        print(f"[fsm] {msg}")
        self.logf.write(f"{time.strftime('%Y-%m-%d')} {line}\n")
        self.logf.flush()

    # ---- DDS / LocoClient (지연 초기화 — 로봇 미연결이어도 웹은 뜬다) ----
    def _ensure_client(self):
        if self.client is not None:
            return True
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            LocoClient = robot_env.loco_client_class()   # robot.yaml sdk.loco_module
            if not self._dds_inited:          # 프로세스당 1회만
                robot_env.dds_init()
                self._dds_inited = True
            # 연결된 로봇 확인 (robot.yaml identity.mode_machine) — 다르면 FSM 명령을 보내지 않는다
            if (robot_env.CFG.get("identity") or {}).get("mode_machine") is not None:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState
                sub = ChannelSubscriber("rt/lowstate", hg_LowState)
                sub.Init()
                msg = sub.Read(2.0)
                if msg is None:
                    raise RuntimeError("rt/lowstate 수신 없음 — 로봇 확인 불가, FSM 명령 거부")
                ok, why = robot_env.check_identity(msg)
                self.log(why)
                if not ok:
                    raise RuntimeError(why)
            c = LocoClient()
            c.Init()
            c.SetTimeout(10.0)            # init_fsm.py 와 동일
            q = LocoClient()
            q.Init()
            q.SetTimeout(1.0)
            self.client, self.qclient = c, q
            self.init_err = None
            return True
        except Exception as e:
            self.init_err = str(e)
            return False

    @staticmethod
    def _get_fsm_id(c):
        """GetFsmId — SDK 에 메서드가 없으면(2026-06 이전 unitree_sdk2py) 같은 RPC(7001)를 직접 호출.
        구버전도 Init() 에서 7001 을 등록하므로 _Call 로 부를 수 있다 (만약을 위해 재등록)."""
        if hasattr(c, "GetFsmId"):
            return c.GetFsmId()
        c._RegistApi(API_GET_FSM_ID, 0)
        code, data = c._Call(API_GET_FSM_ID, json.dumps({}))
        if code != 0:
            return code, None
        return code, json.loads(data).get("data")

    def read_fsm(self):
        """현재 FSM id 읽기 → id 또는 None. 결과는 self.cur 에 저장."""
        if not self._ensure_client():
            self.cur, self.cur_err = None, f"LocoClient 초기화 실패: {self.init_err}"
            return None
        with self.read_lock:
            try:
                code, data = self._get_fsm_id(self.qclient)
            except Exception as e:
                code, data = -1, str(e)
        if code != 0 or data is None:
            self.cur, self.cur_err = None, f"FSM 조회 실패 (code={code})"
            return None
        self.cur, self.cur_err = int(data), None
        return self.cur

    def set(self, target, robot_running_fn):
        """검사 후 DELAY_SEC 카운트다운 → 전송 (별도 스레드). 대기 중 Damp 는 교체 허용."""
        if target not in STEPS:
            raise HTTPException(400, f"FSM 은 {STEPS} 중 하나")
        with self.lock:
            if self.busy is not None:
                if target == FSM_DAMP and self.busy != FSM_DAMP and time.time() < self.fire_at:
                    self.log(f"FSM {self.busy} 대기 취소 — Damp 로 교체")
                    self.cancel.set()
                else:
                    raise HTTPException(409, f"FSM {self.busy} 대기/전송 중")
        if self.cancel.is_set():               # 교체: 이전 스레드가 빠질 때까지
            t0 = time.time()
            while self.busy is not None and time.time() - t0 < 2:
                time.sleep(0.02)
        if not self._ensure_client():
            raise HTTPException(503, f"LocoClient 초기화 실패: {self.init_err}")
        ok, why = allowed(target, self.read_fsm(), robot_running_fn())
        if not ok:
            self.log(f"FSM {target} 거부 — {why}")
            raise HTTPException(409, why)
        with self.lock:
            if self.busy is not None:
                raise HTTPException(409, f"FSM {self.busy} 대기/전송 중")
            self.cancel.clear()
            self.busy, self.fire_at, self.result = target, time.time() + DELAY_SEC, None
        self.log(f"FSM {target} {FSM_NAME.get(target, '')} — {DELAY_SEC:.0f}초 후 전송")
        threading.Thread(target=self._fire, args=(target, robot_running_fn), daemon=True).start()

    def cancel_pending(self):
        with self.lock:
            if self.busy is None or time.time() >= self.fire_at:
                return False
            self.cancel.set()
        return True

    def _fire(self, target, robot_running_fn):
        try:
            if self.cancel.wait(DELAY_SEC):
                self.log(f"FSM {target} 취소됨")
                self.result = (False, f"FSM {target} 취소됨")
                return
            with self.lock:
                self.fire_at = 0.0            # 이제부터 취소 불가
            cur = self.read_fsm()
            ok, why = allowed(target, cur, robot_running_fn())   # 대기 중 상태가 바뀌었을 수 있다
            if not ok:
                self.log(f"FSM {target} 전송 직전 거부 — {why}")
                self.result = (False, f"FSM {target} 거부: {why}")
                return
            self.log(f"SetFsmId({target}) … (현재 {cur})")
            with self.call_lock:
                code = self.client.SetFsmId(target)
            now = self.read_fsm()
            self.log(f"SetFsmId({target}) → code={code}, 현재 FSM = {now} ({FSM_NAME.get(now, '?')})")
            if code != 0:
                self.result = (False, f"FSM {target} 실패 (code={code})")
                return
            self.last_sent = target
            self.result = (True, f"FSM {target} {FSM_NAME.get(target, '')} 전송 완료")
        except Exception as e:
            self.log(f"FSM {target} 오류: {e}")
            self.result = (False, f"FSM {target} 오류: {e}")
        finally:
            with self.lock:
                self.busy, self.fire_at = None, 0.0

    def poll_loop(self):
        while True:
            if self.busy is None:
                self.read_fsm()
            time.sleep(POLL_SEC)

    def state(self, robot_running):
        return {"cur": self.cur, "cur_name": FSM_NAME.get(self.cur, "") if self.cur is not None else "",
                "cur_bal": FSM_BAL.get(self.cur, False),
                "last_sent": self.last_sent,
                "cur_err": self.cur_err, "busy": self.busy, "result": self.result,
                "remain": round(max(0.0, self.fire_at - time.time()), 1) if self.fire_at else 0,
                "allowed": {str(t): allowed(t, self.cur, robot_running)[0] for t in STEPS},
                "log": list(self.lines)[-120:]}


# ==========================================
# start_robot.sh (자식 프로세스)
# ==========================================
class Job:
    """자식 프로세스 1개 + 출력 버퍼."""

    def __init__(self, name):
        self.name = name
        self.proc = None
        self.label = ""
        self.started = 0.0
        self.ended = 0.0
        self.rc = None
        self.lines = deque(maxlen=300)
        self.lock = threading.Lock()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, args, label, as_user=False, env_extra=None, keep_lines=False):
        with self.lock:
            if self.running():
                raise HTTPException(409, f"{self.name} 실행 중: {self.label}")
            if not keep_lines:
                self.lines.clear()
            self.label, self.rc = label, None
            self.started, self.ended = time.time(), 0.0
            logf = _logfile(self.name)
            drop = as_user and USER_PW is not None
            logf.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} {label} =====\n")
            self.proc = subprocess.Popen(
                args, cwd=ROOT, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True, text=True, bufsize=1,
                preexec_fn=_as_user if drop else None,
                env={**(_user_env() if drop else os.environ), **(env_extra or {})})
            threading.Thread(target=self._pump, args=(self.proc, logf), daemon=True).start()

    def _pump(self, proc, logf):
        for line in proc.stdout:
            line = line.rstrip("\n")
            self.lines.append(f"[{time.strftime('%H:%M:%S')}] {line}")
            logf.write(line + "\n")
            logf.flush()
        proc.wait()
        self.rc, self.ended = proc.returncode, time.time()
        self.lines.append(f"[{time.strftime('%H:%M:%S')}] --- 종료 (rc={self.rc}) ---")
        logf.close()

    def state(self):
        # rc 는 프로세스에서 직접 — 고아 자식이 stdout 파이프를 잡고 있으면 _pump 가 끝나지 않을 수 있다
        rc = self.proc.poll() if self.proc is not None else None
        if rc is not None and not self.ended:
            self.ended = time.time()
        return {"running": self.running(), "label": self.label, "rc": rc,
                "elapsed": round((time.time() if self.running() else (self.ended or time.time()))
                                 - self.started, 1) if self.started else 0,
                "log": list(self.lines)[-120:]}


fsm = FsmCtl()
job = Job("app")            # 한 번에 하나만 (팔 · D435i 를 프로세스 하나만 쓸 수 있음)
CUR_MODE = {"m": None}      # 지금 (또는 마지막) 실행한 모드


# ==========================================
# 앱 (apps/*/app.yaml) — 홈 카드 · 시작 명령
# ==========================================
def load_apps():
    import yaml
    out, modes = [], {}
    for d in sorted(os.listdir(os.path.join(ROOT, "apps"))):
        f = os.path.join(ROOT, "apps", d, "app.yaml")
        if not os.path.isfile(f):
            continue
        with open(f, encoding="utf-8") as fh:
            a = yaml.safe_load(fh) or {}
        a["id"] = d
        for m in a.get("modes") or []:
            if m["id"] in modes:
                raise SystemExit(f"[launcher] 모드 id 중복: {m['id']} ({f})")
            cmd = list(m["cmd"])
            if not os.path.isfile(os.path.join(ROOT, cmd[0])):
                raise SystemExit(f"[launcher] {f}: 시작 파일 없음 {cmd[0]}")
            if m.get("open") and m["open"].split(":", 1)[0] not in ports.PORTS:
                raise SystemExit(f"[launcher] {f}: open 포트 이름 {m['open']} 이 config/ports.env 에 없음")
            m.update(app=d, app_title=a.get("title", d), cmd=cmd, real=bool(m.get("real")))
            modes[m["id"]] = m
        out.append(a)
    out.sort(key=lambda a: a.get("order", 99))
    return out, modes


APPS, MODES = load_apps()


def local_cfg():
    import yaml
    f = os.path.join(ROOT, "config", "local.yaml")
    if not os.path.isfile(f):
        return {}
    with open(f, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def busy():
    """실행 중이거나 시작 준비 중 (그리퍼 서비스 · 카메라 확인)."""
    return job.running() or PREP["on"]


def real_running():
    m = CUR_MODE["m"]
    return busy() and m is not None and m["real"]


# ==========================================
# 그리퍼 실기 (app.yaml gripper: true) — root 가 필요한 일은 런처가 함 (웹에서는 sudo 비밀번호를 물을 곳이 없음)
#   ① Dex1-1 그리퍼 서비스 (모터 두 개를 찾을 때까지 최대 3 번 — start_gripper.sh 와 같은 방식)
#   ② 카메라 숙임 확인 (tools/check_rsimu.py 3 초, root) → 결과 파일을 start_gripper.sh 에 넘김 (판단은 스크립트)
#   ③ start_gripper.sh (사용자 권한, GRIPPER_BY_LAUNCHER=1) — 끝나면 그리퍼 서비스를 마지막에 끔
# ==========================================
PREP = {"on": False, "cancel": False}
DEX1 = {"proc": None}


def robot_iface():
    return str(local_cfg().get("dds_interface") or "enp45s0")      # start_gripper.sh 기본값과 같음


def dex1_bin():
    home = USER_PW.pw_dir if USER_PW else os.path.expanduser("~")
    return str(local_cfg().get("dex1_bin") or os.path.join(home, "project", "dex1_1_service", "bin", "dex1_1_gripper_server"))


def _say(msg):
    job.lines.append(f"[{time.strftime('%H:%M:%S')}] [launcher] {msg}")


def _kill(p, wait=8):
    if p is None or p.poll() is not None:
        return
    p.terminate()
    try:
        p.wait(wait)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait(3)


def dex1_start(iface):
    b = dex1_bin()
    if not os.access(b, os.X_OK):
        raise RuntimeError(f"그리퍼 서비스 실행 파일 없음: {b} (다른 위치면 config/local.yaml dex1_bin)")
    if subprocess.run(["pgrep", "-f", "dex1_1_gripper_server"], capture_output=True).returncode == 0:
        raise RuntimeError("그리퍼 서비스가 이미 떠 있음 (터미널?) — 끄고 다시: sudo pkill -f dex1_1_gripper_server")
    logp = os.path.join(LOG_DIR, f"dex1_service_{time.strftime('%Y%m%d')}.log")
    for attempt in (1, 2, 3):
        if PREP["cancel"]:
            raise RuntimeError("취소")
        _say(f"그리퍼 서비스 {b} -n {iface} … (시도 {attempt}/3)")
        lines = []
        p = subprocess.Popen([b, "-n", iface], cwd=os.path.dirname(b), stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)

        def pump(p=p, lines=lines):
            with open(logp, "a", encoding="utf-8") as f:
                _own(logp)
                for ln in p.stdout:
                    lines.append(ln)
                    f.write(f"[{time.strftime('%H:%M:%S')}] {ln}")
                    f.flush()
        threading.Thread(target=pump, daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < 15 and p.poll() is None and not any("Gripper Server started" in x for x in lines):
            time.sleep(0.3)
        time.sleep(0.5)
        found = sorted(set(re.findall(r"Motor ID: ([01])", "".join(lines))))
        _say(f"그리퍼 서비스 찾은 모터: {found or '없음'}")
        if found == ["0", "1"] and p.poll() is None:
            DEX1["proc"] = p
            return
        _kill(p, 5)
        if attempt < 3:
            _say("둘 다 못 찾음 — 2 초 뒤 다시")
            time.sleep(2)
    raise RuntimeError(f"3 번 다 그리퍼 둘 다 못 찾음 (ID 0 = 오른손, ID 1 = 왼손) — 빠진 손 전원 · 485 선 확인 (로그 {logp})")


def dex1_stop():
    p = DEX1["proc"]
    if p is not None and p.poll() is None:
        _say("그리퍼 서비스 끔")
        _kill(p, 8)
    DEX1["proc"] = None


def cam_check(iface):
    _say("카메라 숙임 확인 (check_rsimu 3 초 — 로봇을 건드리지 마세요) …")
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "check_rsimu.py"), "--sec", "3", iface],
                       cwd=ROOT, capture_output=True, text=True, timeout=90)
    path = os.path.join(LOG_DIR, "launcher_cam_check.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write(r.stdout + r.stderr)
    _own(path)
    return path


def _prep_and_start(m, cmd, env):
    try:
        iface = robot_iface()
        dex1_start(iface)
        if PREP["cancel"]:
            raise RuntimeError("취소")
        if "--no-cam-check" not in cmd:
            env["CAM_CHECK_FILE"] = cam_check(iface)
        if PREP["cancel"]:
            raise RuntimeError("취소")
        env["GRIPPER_BY_LAUNCHER"] = "1"
        job.start(["bash"] + cmd + [iface], f"{m['app_title']} · {m['label']}", as_user=True, env_extra=env, keep_lines=True)
        threading.Thread(target=_watch_end, args=(job.proc, m), daemon=True).start()
    except Exception as e:  # noqa: BLE001
        _say(f"❌ 시작 못 함: {e}")
        dex1_stop()
    finally:
        PREP.update(on=False, cancel=False)


# ---- 미니 PC 학습: GPU 최대 클럭 제한 (Arc B390 2500 MHz 연속 부하에서 PC 멈춤 — apps/trainer/start_trainer_xpu.sh) ----
XPU = {"file": None, "old": None}


def xpu_cap(on):
    import glob
    if on:
        fs = sorted(glob.glob("/sys/class/drm/card*/device/tile0/gt0/freq0/max_freq"))
        if not fs:
            job.lines.append("[launcher] ⚠️ GPU 클럭 파일 없음 — 제한 없이 학습 (Intel xe 드라이버가 아님?)")
            return
        f, mhz = fs[0], int(os.environ.get("XPU_MAX_MHZ", "1200"))
        old = open(f).read().strip()
        with open(f, "w") as fh:
            fh.write(str(mhz))
        XPU.update(file=f, old=old, pre=set(_pgrep("apps/trainer/train_act.py")))
        job.lines.append(f"[launcher] GPU 최대 클럭 {old} → {open(f).read().strip()} MHz (학습 끝나면 되돌림)")
    elif XPU["file"]:
        mine = [p for p in _pgrep("apps/trainer/train_act.py") if p not in XPU.get("pre", set())]   # 런처가 띄운 학습만
        for sig, wait in ((signal.SIGTERM, 3), (signal.SIGKILL, 0)):    # 클럭을 풀기 전에 학습부터 끔
            for p in mine:
                try:
                    os.kill(p, sig)
                except ProcessLookupError:
                    pass
            time.sleep(wait)
        with open(XPU["file"], "w") as fh:
            fh.write(XPU["old"])
        job.lines.append(f"[launcher] GPU 최대 클럭 되돌림 → {XPU['old']} MHz")
        XPU.update(file=None, old=None)


def _watch_end(p, m):
    """실행이 끝나면 (정지 · 오류 · 스스로 종료) 뒷정리 — 학습이면 클럭 복원."""
    p.wait()
    if m.get("xpu_cap"):
        xpu_cap(False)
    if m.get("gripper"):
        dex1_stop()          # 잡기 웹 · arm_server 가 먼저 끝난 뒤 (start_gripper.sh 와 같은 순서)


# ==========================================
# 이미 떠 있는 것 (터미널에서 띄운 것 · 남은 프로그램) — 찾아서 보여 주고, 동의하면 Ctrl+C 와 같은 방식으로 끈 뒤 시작
# ==========================================
WATCH_PORTS = {p: n for n, p in ports.PORTS.items() if n != "launcher"}


def _ppid(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            return int(f.read().rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return None


def _cmd(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _descends(pid, top):
    for _ in range(40):
        if pid is None or pid <= 1:
            return False
        if pid == top:
            return True
        pid = _ppid(pid)
    return False


def _script_root(pid):
    """그 프로세스를 띄운 start 스크립트 중 가장 위 (터미널 Ctrl+C 대상). 없으면 자기 자신."""
    best, cur, me = pid, pid, os.getpid()
    for _ in range(40):
        pp = _ppid(cur)
        if pp is None or pp <= 1 or pp == me:
            break
        if re.search(r"start_[a-z_]+\.sh", _cmd(pp)):
            best = pp
        cur = pp
    return best


def _pgrep(pat):
    r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
    return [int(x) for x in r.stdout.split() if int(x) != os.getpid()]


def _short(cmd):
    m = re.search(r"(start_[a-z_]+\.sh|[A-Za-z0-9_]+\.py|dex1_1_gripper_server)(.*)", cmd)
    return (m.group(1) + m.group(2)[:40]) if m else cmd[:60]


def _listeners():
    """듣고 있는 TCP 포트 → pid 집합 (/proc 에서 직접 — ss 가 없어도 됨, root 라 모든 프로세스가 보임)."""
    inode_port = {}
    for f in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(f) as fh:
                next(fh)
                for ln in fh:
                    c = ln.split()
                    if len(c) > 9 and c[3] == "0A":                     # LISTEN
                        inode_port[c[9]] = int(c[1].rsplit(":", 1)[1], 16)
        except OSError:
            pass
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            for fd in os.listdir(f"/proc/{d}/fd"):
                t = os.readlink(f"/proc/{d}/fd/{fd}")
                if t.startswith("socket:["):
                    port = inode_port.get(t[8:-1])
                    if port is not None:
                        out.setdefault(port, set()).add(int(d))
        except OSError:
            continue
    return out


def find_conflicts():
    """→ [{pid, label, what: [...], notes: [...], own}] — 런처가 띄운 것(own)도 포함."""
    own = job.proc.pid if job.running() else None
    out = {}

    def add(pid, what, note=None):
        if own is not None and _descends(pid, own):
            e = out.setdefault(-1, {"pid": own, "label": job.label, "what": [], "notes": [], "own": True})
        else:
            t = _script_root(pid)
            e = out.setdefault(t, {"pid": t, "label": _short(_cmd(t)), "what": [], "notes": [], "own": False})
        if what not in e["what"]:
            e["what"].append(what)
        if note and note not in e["notes"]:
            e["notes"].append(note)

    for port, pids in _listeners().items():
        if port in WATCH_PORTS:
            for pid in pids:
                add(pid, f"{WATCH_PORTS[port]} :{port}",
                    "팔이 기본 자세로 돌아간 뒤 제어권을 반납함 (팔이 움직임)" if WATCH_PORTS[port] == "arm_server" else None)
    for pid in _pgrep("apps/trainer/train_act.py"):
        add(pid, "학습 (train_act)", "학습이 중단됨 — 마지막 체크포인트 (5000 step 마다) 에서 이어 학습 가능")
    for pid in _pgrep("dex1_1_gripper_server"):
        if not (DEX1["proc"] is not None and DEX1["proc"].pid == pid):
            add(pid, "그리퍼 서비스", "그리퍼 힘이 풀림 — 잡고 있는 물건은 먼저 내려놓기")
    if own is not None and -1 not in out:
        out[-1] = {"pid": own, "label": job.label, "what": ["런처가 띄운 것"], "notes": [], "own": True}
    return list(out.values())


def stop_conflicts(cs):
    """Ctrl+C 와 같게: 스크립트면 그 프로세스 그룹에 SIGINT (각 스크립트의 정상 종료 순서), 아니면 SIGTERM. 45 초 뒤에도 남으면 강제."""
    for c in cs:
        if c["own"]:
            _say(f"정지: {c['label']}")
            stop_job(in_prep=True)
            time.sleep(1.0)            # 그 실행의 뒷정리 (그리퍼 서비스 · 클럭 복원) 가 끝나게
            continue
        pid = c["pid"]
        _say(f"정지: {c['label']} (pid {pid}) — {', '.join(c['what'])}")
        try:
            pg = os.getpgid(pid)
            if ".sh" in _cmd(pid) and pg != os.getpgid(0):
                os.killpg(pg, signal.SIGINT)
            else:
                os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    t0 = time.time()
    while time.time() - t0 < 45:
        if not [c for c in find_conflicts() if not c["own"]]:
            _say("떠 있던 것 정지 완료")
            return True
        time.sleep(1)
    left = [c for c in find_conflicts() if not c["own"]]
    for c in left:
        _say(f"⚠ 45 초 안에 안 꺼짐 — 강제 종료: {c['label']} (pid {c['pid']})")
        try:
            os.kill(c["pid"], signal.SIGKILL)
        except ProcessLookupError:
            pass
    time.sleep(2)
    left = [c for c in find_conflicts() if not c["own"]]
    if left:
        _say("❌ 아직 남아 있음: " + ", ".join(c["label"] for c in left))
        return False
    return True


def _launch(m):
    """실제 시작 (검사 · 정지가 끝난 뒤)."""
    if m.get("gripper") and m["real"]:
        _prep_and_start(m, [os.path.join(ROOT, m["cmd"][0])] + m["cmd"][1:], {})
        return
    if m.get("xpu_cap"):
        xpu_cap(True)
    job.start(["bash", os.path.join(ROOT, m["cmd"][0])] + m["cmd"][1:], f"{m['app_title']} · {m['label']}",
              as_user=True, env_extra={"XPU_CAP_BY_LAUNCHER": "1"} if m.get("xpu_cap") else None, keep_lines=True)
    if m["real"] and fsm.cur is None:
        _say(f"⚠️ FSM 확인 불가({fsm.cur_err}) — {FSM_RUN} 상태인지 직접 확인할 것")
    threading.Thread(target=_watch_end, args=(job.proc, m), daemon=True).start()


def _replace_and_launch(m, cs):
    try:
        if cs and not stop_conflicts(cs):
            _say("❌ 떠 있던 것을 다 끄지 못해 시작 안 함")
            return
        if PREP["cancel"]:
            _say("취소")
            return
        if m["real"]:
            cur = fsm.read_fsm()
            if cur is not None and cur != FSM_RUN:
                _say(f"❌ FSM {cur} — {FSM_RUN} 에서만 시작 (정지 중 FSM 이 바뀜?)")
                return
        _launch(m)
    except Exception as e:  # noqa: BLE001
        _say(f"❌ 시작 못 함: {e}")
    finally:
        if not (m.get("gripper") and m["real"]):
            PREP.update(on=False, cancel=False)


def start_mode(mid, replace=False):
    m = MODES.get(mid)
    if m is None:
        raise HTTPException(404, f"모드 없음: {mid}")
    if m.get("todo"):
        raise HTTPException(409, f"아직 웹에서 시작 못 함 — {m['todo']}")
    if PREP["on"]:
        raise HTTPException(409, "시작 준비 중 — 기다리거나 정지")
    if SEQ["busy"]:
        raise HTTPException(409, "종료 진행 중")
    if m["real"]:
        if fsm.busy is not None:
            raise HTTPException(409, f"FSM {fsm.busy} 대기/전송 중")
        cur = fsm.read_fsm()
        if cur is not None and cur != FSM_RUN:
            raise HTTPException(409, f"FSM {cur} ({FSM_NAME.get(cur, '')}) — {FSM_DAMP} → {FSM_LOCK} → {FSM_RUN} 로 {_LBL['run_enter']} 진입 후 시작")
        if m.get("gripper") and USER_PW is None:
            raise HTTPException(409, "런처가 sudo 로 실행되지 않음 — ./launcher.sh 로 띄울 것")
    cs = find_conflicts()
    if cs and not replace:
        raise HTTPException(409, {"conflicts": cs, "msg": "이미 떠 있는 것이 있음 — 끄고 시작할지 확인"})
    CUR_MODE["m"] = m
    SEQ.update(msg="", ok=None)
    job.lines.clear()
    job.label, job.rc, job.started, job.ended = f"{m['app_title']} · {m['label']} (준비 중)", None, time.time(), 0.0
    PREP.update(on=True, cancel=False)
    threading.Thread(target=_replace_and_launch, args=(m, cs), daemon=True).start()


def stop_job(in_prep=False):
    """bash 에만 SIGTERM → 각 start 스크립트의 trap cleanup (Ctrl+C 와 같은 종료 순서). → 정상 종료 여부"""
    if PREP["on"] and not in_prep:
        PREP["cancel"] = True
        for _ in range(200):
            if not PREP["on"]:
                break
            time.sleep(0.3)
    p = job.proc
    if p is None or p.poll() is not None:
        dex1_stop()
        return True
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(ROBOT_STOP_WAIT)
        return True
    except subprocess.TimeoutExpired:
        job.lines.append("[launcher] 종료 지연 — 프로세스 그룹 SIGKILL (팔 제어권 반납이 생략됐을 수 있음)")
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return False


# ==========================================
# 종료: 실행 중인 것 정지 → FSM 703 → FixStand(4) → Damp(1)  (시작 1 → 4 → 703 의 역순)
# ==========================================
SEQ = {"busy": False, "msg": "", "ok": None}


def _wait_fsm(target):
    fsm.set(target, real_running)
    while fsm.busy is not None:
        time.sleep(0.2)
    return bool(fsm.result and fsm.result[0])


def _shutdown():
    def say(msg, ok=None):
        SEQ.update(msg=msg, ok=ok)
        fsm.log(f"[종료] {msg}")
    try:
        if busy():
            say("실행 중인 것 정지 중 (팔 기본 자세 · 제어권 반납) …")
            if not stop_job():
                say("정지가 늦어 강제로 끔 — 팔 반납이 생략됐을 수 있어 자동 Damp 안 함. 로봇을 확인하고 Damp 를 직접 누르세요", False)
                return
        cur = fsm.read_fsm()
        if cur is None:
            say(f"FSM 확인 불가 ({fsm.cur_err}) — 로봇 미연결이면 그대로 끝. 연결돼 있으면 Damp 를 직접 누르세요", True)
            return
        if cur == FSM_DAMP:
            say("이미 Damp — 끝", True)
            return
        if cur != FSM_LOCK:
            say(f"FSM {cur} → {FSM_LOCK} ({_LBL['lock_button']}) — {DELAY_SEC:.0f}초 후")
            if not _wait_fsm(FSM_LOCK):
                say(f"{FSM_LOCK} 전송 실패 · 취소 — 멈춤 (Damp 는 직접)", False)
                return
            time.sleep(2.0)
        say(f"FSM {FSM_LOCK} → {FSM_DAMP} (Damp) — {DELAY_SEC:.0f}초 후")
        if not _wait_fsm(FSM_DAMP):
            say(f"{FSM_DAMP} 전송 실패 · 취소 — 멈춤", False)
            return
        say("종료 완료 — Damp", True)
    finally:
        SEQ["busy"] = False


@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=fsm.poll_loop, daemon=True).start()
    yield
    # launcher 종료(Ctrl+C) 시 로봇 서버도 정상 종료 시퀀스로 정지.
    # 남겨두면 다음 start_robot.sh 의 sweep 이 SIGKILL 로 죽여 arm 제어권 반납이 생략된다.
    if busy():
        print("[launcher] 종료 — 실행 중인 것 정지 중...")
        stop_job()
    dex1_stop()


app = FastAPI(title="pitron launcher", lifespan=lifespan)


@app.post("/fsm/{target}")
def run_fsm(target: int):
    if SEQ["busy"] and target != FSM_DAMP:
        raise HTTPException(409, "종료 진행 중 — Damp 만 가능")
    fsm.set(target, real_running)
    return {"ok": True, "delay": DELAY_SEC}


@app.post("/fsm_cancel")
def fsm_cancel():
    return {"ok": fsm.cancel_pending()}


@app.get("/apps")
def apps_list():
    url = str(local_cfg().get("trainer_url") or "").strip()
    out = []
    for a in APPS:
        b = {k: a.get(k) for k in ("id", "title", "group", "desc")}
        b["modes"] = []
        for m in a.get("modes") or []:
            o = m.get("open")
            b["modes"].append({**{k: m.get(k) for k in ("id", "label", "real", "step", "todo", "gripper")},
                               "open": {"port": ports.PORTS[o.split(":", 1)[0]], "path": o.split(":", 1)[1]} if o else None})
        b["links"] = [{"step": l.get("step"), "label": l["label"], "url": url} for l in a.get("links") or []
                      if l.get("url_from") == "trainer_url" and url]
        out.append(b)
    return out


@app.post("/app/{mid}/start")
def app_start(mid: str, replace: bool = False):
    start_mode(mid, replace)
    return {"ok": True}


@app.post("/app/stop")
def app_stop():
    if not busy():
        return {"ok": True, "already": True}
    threading.Thread(target=stop_job, daemon=True).start()
    return {"ok": True}


@app.post("/shutdown")
def shutdown():
    if SEQ["busy"]:
        raise HTTPException(409, "종료 진행 중")
    SEQ.update(busy=True, msg="시작", ok=None)
    threading.Thread(target=_shutdown, daemon=True).start()
    return {"ok": True}


@app.get("/status")
def status():
    m = CUR_MODE["m"]
    return {"fsm": fsm.state(real_running()),
            "job": {**job.state(), "running": busy(), "prep": PREP["on"], "mode": m["id"] if m else None,
                    "real": bool(m and m["real"])},
            "seq": dict(SEQ)}


@app.get("/", response_class=HTMLResponse)
def index():
    return ports.fill(HTML)


@app.get("/i18n.js", include_in_schema=False)
def i18n_js():
    return FileResponse(paths.I18N_JS, media_type="application/javascript")


HTML = r"""<!DOCTYPE html>
<html lang="ko"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>@@TITLE@@</title>
<style>
:root{--bg:#0e1116;--panel:#161b22;--panel2:#1c232d;--line:#2a3340;--ink:#c9d4e0;--dim:#6b7785;
  --accent:#00b7eb;--accent2:#7fd8f5;--ok:#3ddc97;--warn:#ff6b6b;--amber:#ffb454}   /* 메인 = 회사 메인 컬러 (제어 웹과 같음), 상태 OK 는 --ok */
*{box-sizing:border-box}
html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.45 ui-monospace,Menlo,monospace;
  display:flex;flex-direction:column;min-height:640px}
.top{display:flex;align-items:center;gap:12px;padding:12px 18px;border-bottom:1px solid var(--line);box-shadow:inset 0 -1px 0 rgba(0,183,235,.45)}
.top b{color:var(--accent)} .top .r{margin-left:auto;color:var(--dim);font-size:11px}
.wrap{flex:1;min-height:0;display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden;display:flex;flex-direction:column;min-height:0}
.h{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim);box-shadow:inset 3px 0 0 var(--accent);padding:9px 13px;
  border-bottom:1px solid var(--line);background:var(--panel2);display:flex;justify-content:space-between}
.h code{text-transform:none}
.b{padding:13px;display:flex;flex-direction:column;gap:10px;flex:1;min-height:0}
.row{display:flex;gap:8px}.row>*{flex:1}
button{font:inherit;font-size:14px;font-weight:700;padding:16px 8px;border-radius:8px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--ink)}
button small{display:block;font-size:10px;font-weight:400;color:var(--dim);margin-top:3px}
button:disabled{opacity:.3;cursor:not-allowed}
button .no{display:block;font-size:20px;margin-bottom:2px}
button.pend{box-shadow:0 0 0 2px var(--amber) inset;opacity:1!important}
button.next:not(:disabled){box-shadow:0 0 0 2px var(--accent) inset;animation:nx 1.6s infinite}
@keyframes nx{50%{box-shadow:0 0 0 2px transparent inset}}
.go{border-color:#0b5a70;color:var(--accent)} .go:hover:not(:disabled){background:#082c38}
.st{border-color:#5a2b2b;color:var(--warn)} .st:hover:not(:disabled){background:#2e1515}
.ok{border-color:#0b5a70;color:var(--accent2)} .ok:hover:not(:disabled){background:#082c38}
.fsmnow{display:flex;align-items:baseline;gap:12px;padding:12px 14px;border-radius:8px;border:1px solid var(--line);background:#0a0d12}
.fsmnow .k{font-size:11px;color:var(--dim);white-space:nowrap}
.fsmnow .v{white-space:nowrap}
.fsmnow .bal:empty{display:none}
.fsmnow .v{font-size:22px;font-weight:700}
.fsmnow .n{font-size:13px;color:var(--dim)}
.fsmnow .bal{margin-left:auto;font-size:11px;padding:2px 8px;border-radius:999px;border:1px solid var(--line)}
.fsmnow .bal.on{color:var(--ok);border-color:#1f5a43}.fsmnow .bal.off{color:var(--amber);border-color:#6a4a1f}
.fsmnow.bal .v{color:var(--ok)} .fsmnow.std .v{color:var(--accent)} .fsmnow.low .v{color:var(--amber)}
.fsmnow.unk .v{color:var(--warn);font-size:14px}
.state{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:8px;border:1px solid var(--line);background:var(--panel2)}
.state::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--dim);flex:none}
.state.run{border-color:#6a4a1f;color:var(--amber)}.state.run::before{background:var(--amber);animation:bl 1s infinite}
.state.on{border-color:#1f5a43;color:var(--ok)}.state.on::before{background:var(--ok)}
.state.err{border-color:#5a2b2b;color:var(--warn)}.state.err::before{background:var(--warn)}
@keyframes bl{50%{opacity:.25}}
pre{margin:0;flex:1;min-height:120px;overflow:auto;background:#0a0d12;border:1px solid var(--line);
  border-radius:8px;padding:10px;font-size:11.5px;white-space:pre-wrap;word-break:break-all}
.note{font-size:11px;color:var(--dim)}
.links a{color:var(--accent2);margin-right:12px}
.col{display:flex;flex-direction:column;gap:14px;min-height:0}
.card.grow{flex:1}
.app{border:1px solid var(--line);border-radius:8px;padding:10px 12px;background:#0a0d12;display:flex;flex-direction:column;gap:8px}
.app .t{display:flex;align-items:baseline;gap:10px}.app .t b{font-size:15px;color:var(--ink)}.app .t span{font-size:11px;color:var(--dim)}
.app .t a{margin-left:auto}
.app .steps{display:grid;grid-template-columns:auto 1fr;gap:8px 12px;align-items:center}
.app .sn{font-size:12px;color:var(--accent2);white-space:nowrap}
.app .row button{padding:10px 8px;font-size:13px}
button.cur{box-shadow:0 0 0 2px var(--ok) inset;color:var(--ok)}
a.open{color:var(--accent2);font-size:12px;text-decoration:none;white-space:nowrap}
a.open.off{color:var(--dim);pointer-events:none}
.shut{margin-top:2px}
/* 큰 화면: 버튼 키움 */
@media(min-height:900px){button{padding:22px 8px;font-size:15px}button .no{font-size:24px}}
@media(max-width:900px){body{height:auto}.wrap{grid-template-columns:1fr}pre{min-height:240px}}
.modal{position:fixed;inset:0;background:#000b;display:none;align-items:center;justify-content:center;z-index:9}
.modal.show{display:flex}
.mbox{width:min(460px,92vw);background:var(--panel);border:2px solid var(--warn);border-radius:12px;overflow:hidden}
.mbox .mt{background:#2e1515;color:var(--warn);font-size:16px;font-weight:700;padding:12px 16px}
.mbox .mm{padding:16px;font-size:13.5px;line-height:1.7;white-space:pre-line}
.mbox .mm .big{display:block;color:var(--warn);font-weight:700;font-size:15px;margin-bottom:8px}
.mbox .mb{display:flex;gap:8px;padding:0 16px 16px}.mbox .mb button{flex:1;padding:12px}
.mbox .run{background:#5a1f1f;border-color:var(--warn);color:#fff}
</style></head><body>
<script src="/i18n.js"></script>
<div class="top"><b>pitron<span style="color:var(--ink)">·launcher</span></b><span class="r">:80 · 웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요</span><span id="lang-slot" style="margin-left:12px"></span></div>
<div class="wrap">
  <div class="card">
    <div class="h"><span>자세 (FSM)</span><span></span></div>
    <div class="b">
      <div class="fsmnow unk" id="fsmnow"><span class="k">현재 FSM</span><span class="v" id="fsm-v">확인 중</span><span class="n" id="fsm-n"></span><span class="bal" id="fsm-b"></span></div>
      <div class="note">일어서기: <b>@@STAND_SEQ@@</b> 순서로, 로봇이 자리 잡은 걸 보고 다음 단계를 누르세요</div>
@@FSM_BUTTONS@@
      <button class="st shut" id="b-shut" onclick="shutdownAll()">종료<small>실행 중인 것 정지 → @@LOCK@@ → @@DAMP@@ (시작의 역순)</small></button>
      <div class="row" style="align-items:stretch">
        <div class="state" id="fsm-s" style="flex:3">대기</div>
        <button class="st" id="b-cancel" onclick="fsmCancel()" style="flex:1;padding:8px;display:none">취소</button>
      </div>
      <pre id="fsm-log"></pre>
    </div>
  </div>
  <div class="col">
    <div class="card">
      <div class="h"><span>기능</span><span>한 번에 하나만 · 실기는 FSM @@RUN@@ 에서</span></div>
      <div class="b" id="apps">불러오는 중…</div>
    </div>
    <div class="card grow">
      <div class="h"><span>실행 중</span><span id="job-mode"></span></div>
      <div class="b">
        <div class="row" style="align-items:stretch">
          <div class="state" id="rb-s" style="flex:3">정지됨</div>
          <button class="st" id="b-stop" onclick="appStop()" style="flex:1;padding:8px">정지<small>Ctrl+C 와 같음</small></button>
        </div>
        <pre id="rb-log"></pre>
      </div>
    </div>
  </div>
</div>
<div class="modal" id="modal"><div class="mbox">
  <div class="mt" id="m-t">⚠️ 경고</div><div class="mm" id="m-m"></div>
  <div class="mb"><button id="m-no">취소</button><button class="run" id="m-yes">실행</button></div>
</div></div>
<script>
// FSM 번호·이름 — apps/launcher/run_launcher.py 가 robot.yaml fsm 으로 채움
const FSM=@@FSM_JS@@;
const host=location.hostname;
// 경고 모달 — Promise<bool>
function warn(title,big,body){return new Promise(res=>{
  const m=document.getElementById('modal');
  document.getElementById('m-t').textContent='⚠️ '+title;
  document.getElementById('m-m').innerHTML=(big?`<span class="big">${big}</span>`:'')+body;
  const done=v=>{m.classList.remove('show');document.removeEventListener('keydown',esc);res(v);};
  const esc=e=>{if(e.key==='Escape')done(false);};
  document.getElementById('m-no').onclick=()=>done(false);
  document.getElementById('m-yes').onclick=()=>done(true);
  document.addEventListener('keydown',esc);
  m.classList.add('show');document.getElementById('m-no').focus();});}
// ---- 기능 카드 (apps/*/app.yaml) ----
let APPS=[];
const STEP={collect:'① 수집',train:'② 학습',infer:'③ 추론'};
const openUrl=o=>`http://${host}:${o.port}${o.path}`;
const esc=t=>String(t??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
function modeBtn(m){return `<button class="${m.real?'go':'ok'}" id="m-${m.id}" data-todo="${m.todo?1:0}" onclick="appStart('${m.id}')"`+
  `${m.todo?` title="${esc(m.todo)}"`:''}>${esc(m.label)}<small>${m.todo?'준비 중 — '+esc(m.todo):m.real?'실제 로봇 · FSM '+FSM.run:'로봇 없이'}</small></button>`;}
const openA=m=>m.open?`<a class="open off" data-open="${m.id}" href="${openUrl(m.open)}" target="_blank">열기 →</a>`:'';
function renderApps(){
  const home=APPS.filter(a=>a.group!=='imitation'),im=APPS.filter(a=>a.group==='imitation');
  let h=home.map(a=>`<div class="app"><div class="t"><b>${esc(a.title)}</b><span>${esc(a.desc)}</span>${openA(a.modes[0])}</div>`+
    `<div class="row">${a.modes.map(modeBtn).join('')}</div></div>`).join('');
  if(im.length){
    const ms=im.flatMap(a=>a.modes),ls=im.flatMap(a=>a.links||[]);
    h+=`<div class="app"><div class="t"><b>모방학습</b><span>VR 원격 조작 수집 → ACT 학습 → 정책 실행</span></div><div class="steps">`+
      Object.entries(STEP).map(([k,n])=>{const sm=ms.filter(m=>m.step===k),sl=ls.filter(l=>l.step===k);
        return `<div class="sn">${n}</div><div class="row">${sm.map(modeBtn).join('')}`+
          `${sl.map(l=>`<button class="ok" onclick="window.open('${esc(l.url)}','_blank')">${esc(l.label)}<small>링크</small></button>`).join('')}`+
          `<div style="flex:none;display:flex;flex-direction:column;justify-content:center;gap:4px">${sm.map(openA).join('')}</div></div>`;}).join('')+`</div></div>`;}
  document.getElementById('apps').innerHTML=h;}
fetch('/apps').then(r=>r.json()).then(d=>{APPS=d;renderApps();poll();}).catch(e=>{document.getElementById('apps').textContent='기능 목록 못 읽음: '+e;});
let CUR=null;
// Damp 만 경고 후 실행, lock / run 은 바로 실행
async function confirmFsm(t){
  const standing=FSM.standing.includes(CUR);
  if(t===FSM.damp)return warn(`${FSM.names[t]} (FSM ${t})`,
    standing?'지금 서 있습니다 — 힘이 빠져 넘어집니다!':
    CUR===null?'현재 상태 확인 불가 — 서 있다면 힘이 빠져 넘어집니다!':'모터 힘이 빠집니다',
    '· 로봇을 사람이 받치고 있거나 스탠드에 묶여 있습니까?\n· 주변에 사람/장애물이 없습니까?');
  return true;}
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));
  if(!r.ok)throw new Error(d.detail||r.status);return d;}
async function fsm(t){if(!await confirmFsm(t))return;
  try{await post('/fsm/'+t);}catch(e){alert(e.message);}poll();}
async function fsmCancel(){try{await post('/fsm_cancel');}catch(e){}poll();}
async function appStart(id){const m=APPS.flatMap(a=>a.modes).find(x=>x.id===id);if(!m)return;
  const msg=m.real?`${m.label} — 실제 로봇이 움직입니다.\n\n· 거치대 · FSM ${FSM.run} 확인\n· 주변을 비우고 E-STOP 을 손에`+
    (m.gripper?`\n· 그리퍼 서비스를 켜고 카메라 숙임을 3 초 잽니다 — 그동안 로봇을 건드리지 마세요`:''):`${m.label} — 로봇 없이 (가상)`;
  if(!confirm(msg))return;
  let r=await fetch('/app/'+id+'/start',{method:'POST'}),d=await r.json().catch(()=>({}));
  if(r.status===409&&d.detail&&d.detail.conflicts){
    const cs=d.detail.conflicts;
    const body=cs.map(c=>`· ${esc(c.label)}${c.own?' (홈에서 실행)':' (터미널 · 남은 프로그램)'} — ${esc(c.what.join(', '))}`+
      c.notes.map(n=>`\n   ⚠ ${esc(n)}`).join('')).join('\n')+'\n\nCtrl+C 와 같은 방식으로 정상 종료한 뒤 시작합니다 (45 초 안에 안 꺼지면 강제).';
    if(!await warn('이미 떠 있는 것',`${cs.length} 개를 끄고 '${esc(m.label)}' 을 시작할까요?`,body))return;
    r=await fetch('/app/'+id+'/start?replace=true',{method:'POST'});d=await r.json().catch(()=>({}));}
  if(!r.ok)alert(typeof d.detail==='string'?d.detail:JSON.stringify(d.detail));
  poll();}
async function appStop(){if(!confirm('실행 중인 것을 정지합니다 (팔 기본 자세 · 제어권 반납 후 종료).'))return;
  try{await post('/app/stop');}catch(e){alert(e.message);}poll();}
async function shutdownAll(){
  if(!await warn('종료','실행 중인 것을 정지한 뒤 '+FSM.lock+' → '+FSM.damp+' (Damp) 로 갑니다',
    '· 로봇이 거치대에 걸려 있습니까? Damp 하면 힘이 빠집니다\n· 각 단계는 '+5+'초 카운트다운 — 취소하면 그 자리에서 멈춤\n· 로봇 미연결(가상만 썼으면) FSM 명령은 보내지 않음'))return;
  try{await post('/shutdown');}catch(e){alert(e.message);}poll();}
function fill(pre,lines){const atBottom=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-20;
  pre.textContent=lines.join('\n');if(atBottom)pre.scrollTop=pre.scrollHeight;}
function setState(id,cls,txt){const el=document.getElementById(id);el.className='state'+(cls?' '+cls:'');el.textContent=txt;}
async function poll(){try{const d=await(await fetch('/status')).json();
  const f=d.fsm,r=d.robot;
  // 현재 FSM
  const nw=document.getElementById('fsmnow');
  if(f.cur===null){nw.className='fsmnow unk';document.getElementById('fsm-v').textContent='확인 불가';
    document.getElementById('fsm-n').textContent=(f.cur_err||'')+' — 순서 제한 없음'+
      (f.last_sent!==null?` · 마지막 전송 ${f.last_sent}`:'');}
  else{nw.className='fsmnow '+(f.cur_bal?'bal':FSM.standing.includes(f.cur)?'std':'low');
    document.getElementById('fsm-v').textContent=f.cur;
    document.getElementById('fsm-n').textContent=f.cur_name;}
  const fb=document.getElementById('fsm-b');
  if(f.cur===null){fb.textContent='';fb.className='bal';}
  else{fb.textContent=f.cur_bal?'밸런스 제어':'밸런스 없음';fb.className='bal '+(f.cur_bal?'on':'off');}
  const ref=f.cur!==null?f.cur:f.last_sent;          // 조회 불가면 마지막 전송 기준
  // 버튼: 갈 수 있는 단계만 활성(조회 불가면 제한 없음), 다음 단계 강조 (damp → lock → run)
  CUR=f.cur;
  const next=ref!==null?FSM.next[ref]:undefined;
  FSM.steps.forEach(t=>{const b=document.getElementById('b-'+t);
    // 대기 중엔 다른 버튼 잠금 — 단 Damp 는 대기 중 교체 가능
    const pend=f.busy!==null&&f.remain>0;
    b.disabled=(f.busy!==null&&!(t===FSM.damp&&pend&&f.busy!==FSM.damp))||!f.allowed[t];
    b.classList.toggle('next',t===next&&f.busy===null);
    b.classList.toggle('pend',t===f.busy);});
  const cb=document.getElementById('b-cancel');
  cb.style.display=(f.busy!==null&&f.remain>0)?'':'none';
  if(f.busy!==null&&f.remain>0)setState('fsm-s','run',`FSM ${f.busy} — ${Math.ceil(f.remain)}초 후 전송`);
  else if(f.busy!==null)setState('fsm-s','run',`FSM ${f.busy} 전송 중…`);
  else if(f.result)setState('fsm-s',f.result[0]?'on':'err',f.result[1]);
  else setState('fsm-s','','대기');
  fill(document.getElementById('fsm-log'),f.log);
  // 실행 중 · 기능 버튼
  const j=d.job,sq=d.seq;
  document.querySelectorAll('#apps button[id^="m-"]').forEach(b=>{const id=b.id.slice(2),m=APPS.flatMap(a=>a.modes).find(x=>x.id===id);
    b.disabled=j.running||sq.busy||b.dataset.todo==='1'||(m&&m.real&&(f.busy!==null||(f.cur!==null&&f.cur!==FSM.run)));
    b.classList.toggle('cur',j.running&&j.mode===id);});
  document.querySelectorAll('#apps a.open').forEach(a=>a.classList.toggle('off',!(j.running&&j.mode===a.dataset.open)));
  document.getElementById('b-stop').disabled=!j.running||sq.busy;
  document.getElementById('b-shut').disabled=sq.busy;
  document.getElementById('job-mode').textContent=j.label||'';
  if(sq.busy)setState('rb-s','run','종료: '+sq.msg);
  else if(j.prep)setState('rb-s','run',`${j.label} — 떠 있던 것 정지 · 시작 준비 중`);
  else if(j.running)setState('rb-s','on',`${j.label} — 실행 중 (${Math.floor(j.elapsed/60)}분 ${Math.floor(j.elapsed%60)}초)`);
  else if(sq.msg&&sq.ok!==null)setState('rb-s',sq.ok?'':'err','종료: '+sq.msg);
  else if(j.label)setState('rb-s',j.rc===0?'':'err',`${j.label} — 정지됨 (rc=${j.rc})`);
  else setState('rb-s','','정지됨');
  fill(document.getElementById('rb-log'),j.log);
}catch(e){setState('fsm-s','err','launcher 연결 끊김');}}
poll();setInterval(poll,500);
</script></body></html>"""



def _fsm_button(fid, cls, label, note):
    return (f'<button class="{cls}" id="b-{fid}" onclick="fsm({fid})"><span class="no">{fid}</span>'
            f'{html.escape(str(label))}<small>{html.escape(note)}</small></button>')


def _bal_txt(fid):
    return "밸런스" if fid in FSM_BAL else "밸런스 없음"


def fsm_buttons():
    """FSM 단계 버튼 [(id, class, 라벨, 설명)] — robot.yaml fsm (names / labels / damp·lock·run)."""
    return [
        (FSM_DAMP, "st", FSM_NAME[FSM_DAMP], "힘 빼기 · 항상 가능"),
        (FSM_LOCK, "go", _LBL["lock_button"], f"{_bal_txt(FSM_LOCK)} · {FSM_DAMP} / {FSM_RUN} 에서"),
        (FSM_RUN, "go", _LBL["run_button"],
         f"{_bal_txt(FSM_RUN)} · arm_sdk · {FSM_LOCK} 에서"),
    ]


def _render_html(h):
    """HTML 의 @@...@@ 자리에 robot.yaml fsm 값으로 만든 버튼·문구·JS 상수를 넣는다."""
    btn = {b[0]: _fsm_button(*b) for b in fsm_buttons()}
    rows = f'      <div class="row">\n        {btn[FSM_DAMP]}\n        {btn[FSM_LOCK]}\n        {btn[FSM_RUN]}\n      </div>'
    js = {"damp": FSM_DAMP, "lock": FSM_LOCK, "run": FSM_RUN,
          "steps": list(STEPS), "standing": sorted(STANDING),
          "next": {str(FSM_DAMP): FSM_LOCK, str(FSM_LOCK): FSM_RUN},
          "names": {str(k): v for k, v in sorted(FSM_NAME.items())}}
    reps = [
        ("@@TITLE@@", "pitron launcher", 1),
        ("@@STAND_SEQ@@", f"{FSM_DAMP} → {FSM_LOCK} → {FSM_RUN}", 1),
        ("@@FSM_BUTTONS@@", rows, 1),
        ("@@RUN@@", str(FSM_RUN), 1),
        ("@@LOCK@@", str(FSM_LOCK), 1),
        ("@@DAMP@@", str(FSM_DAMP), 1),
        ("@@FSM_JS@@", json.dumps(js, ensure_ascii=False).replace("</", "<\\/"), 1),
    ]
    for old, new, n in reps:
        if h.count(old) != n:
            raise SystemExit(f"[launcher] HTML 자리표시 '{old}' 개수 {h.count(old)} ≠ {n}")
        h = h.replace(old, new)
    if "@@" in h:
        raise SystemExit("[launcher] HTML 에 채우지 않은 자리표시가 남음")
    return h


HTML = _render_html(HTML)


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("[launcher] ⚠️ root 아님 — ./launcher.sh (sudo) 로 실행 권장")
    print(f"[launcher] ROBOT={robot_env.ROBOT}  http://0.0.0.0:{PORT}/  (start_robot.sh 실행 사용자: {SUDO_USER or os.environ.get('USER')})")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
