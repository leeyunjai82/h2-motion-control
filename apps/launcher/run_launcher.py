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


def _brand():
    b = local_cfg().get("brand") or {}
    return {"name": str(b.get("name") or "pitron"), "sub": str(b.get("sub") or "Physical AI · 모방학습")}


@app.get("/menu")
def menu():
    import yaml
    with open(os.path.join(ROOT, "apps", "launcher", "menu.yaml"), encoding="utf-8") as f:
        m = yaml.safe_load(f) or {}
    for sec in m.get("sections") or []:
        for it in sec.get("items") or []:
            it["modes"] = [{"id": i, "label": MODES[i]["label"], "real": MODES[i]["real"], "app": MODES[i]["app_title"]}
                           for i in it.get("modes") or [] if i in MODES]
            for t in it.get("tabs") or []:
                n, path = t["page"].split(":", 1)
                t.update(port=ports.PORTS[n], path=path)
    url = str(local_cfg().get("trainer_url") or "").strip()
    return {"brand": _brand(), "sections": m.get("sections") or [], "trainer_url": url}


def _read_json(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


@app.get("/summary")
def summary():
    """대시보드: 데이터셋 · 모델 · 학습 작업 (파일만 읽음)."""
    ds_dir = paths.data_dir("datasets", legacy="gripper/data")
    pol_dir = paths.data_dir("policies", legacy="gripper/models")
    tr_dir = paths.data_dir("trainer", legacy="trainer/data")
    out = {"datasets": [], "models": [], "jobs": []}
    if os.path.isdir(ds_dir):
        for n in sorted(os.listdir(ds_dir)):
            ep = os.path.join(ds_dir, n, "episodes")
            if not os.path.isdir(ep):
                continue
            ok = tot = 0
            last = ""
            for e in os.listdir(ep):
                m = _read_json(os.path.join(ep, e, "meta.json"))
                if m is None:
                    continue
                tot += 1
                ok += bool(m.get("ok"))
                last = max(last, str(m.get("created") or ""))
            out["datasets"].append({"name": n, "ok": ok, "total": tot, "last": last})
    if os.path.isdir(pol_dir):
        for n in sorted(os.listdir(pol_dir)):
            m = _read_json(os.path.join(pol_dir, n, "meta.json"))
            if m:
                out["models"].append({"name": n, "dataset": m.get("dataset"), "steps": m.get("steps"),
                                      "val_err_deg": m.get("val_err_deg"), "created": m.get("exported") or m.get("created")})
    jd = os.path.join(tr_dir, "jobs")
    if os.path.isdir(jd):
        for n in sorted(os.listdir(jd), reverse=True)[:8]:
            j = _read_json(os.path.join(jd, n, "job.json"))
            if j:
                out["jobs"].append({k: j.get(k) for k in ("id", "dataset", "status", "step", "total", "val_err_deg", "created")})
    out["datasets"].sort(key=lambda d: d["last"], reverse=True)
    out["models"].sort(key=lambda d: str(d["created"] or ""), reverse=True)
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
  --accent:#00b7eb;--accent2:#7fd8f5;--deep:#08465a;--tint:rgba(0,183,235,.14);--ok:#3ddc97;--warn:#ff6b6b;--amber:#ffb454;--r:10px}
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{background:var(--bg);color:var(--ink);font:13px/1.45 'SF Mono',ui-monospace,'Cascadia Code',Menlo,monospace;
  display:flex;flex-direction:column;overflow:hidden;background-image:radial-gradient(circle at 20% 0%,#16202c 0%,var(--bg) 55%)}
a{color:var(--accent2)}
/* 위쪽 막대 */
.top{display:flex;align-items:center;gap:10px;padding:10px 16px;border-bottom:1px solid var(--line);
  background:linear-gradient(90deg,#11161d,#0e1116);box-shadow:inset 0 -1px 0 rgba(0,183,235,.45);flex:none}
.brand{font-weight:700;font-size:16px;color:var(--accent);letter-spacing:.5px;cursor:pointer}.brand small{color:var(--dim);font-weight:400;font-size:11px;margin-left:8px}
.chip{display:inline-flex;align-items:center;gap:6px;font-size:11.5px;padding:4px 11px;border-radius:999px;border:1px solid var(--line);
  background:var(--panel2);color:var(--dim);white-space:nowrap;cursor:pointer}
.chip b{color:var(--ink)}.chip::before{content:"";width:7px;height:7px;border-radius:50%;background:currentColor}
.chip.ok{color:var(--ok);border-color:#1f5a43}.chip.std{color:var(--accent);border-color:#0b5a70}.chip.low{color:var(--amber);border-color:#6a4a1f}
.chip.err{color:var(--warn);border-color:#5a2b2b}.chip.run{color:var(--amber);border-color:#6a4a1f}
.top .r{margin-left:auto;display:flex;gap:8px;align-items:center}
.estop{font-size:10.5px;color:var(--dim);margin-right:6px}
/* 본문 */
.main{flex:1;min-height:0;display:flex}
.side{width:230px;flex:none;border-right:1px solid var(--line);padding:12px 10px;overflow-y:auto;background:#0c1015}
.side .sec{font-size:10px;letter-spacing:1.5px;color:var(--dim);text-transform:uppercase;margin:14px 8px 6px}
.side .it{display:flex;align-items:center;gap:8px;padding:9px 10px;border-radius:8px;cursor:pointer;color:var(--ink);font-size:13px}
.side .it:hover{background:var(--panel2)}.side .it.on{background:var(--tint);color:var(--accent);box-shadow:inset 3px 0 0 var(--accent)}
.side .it .dot{margin-left:auto;width:7px;height:7px;border-radius:50%;background:transparent}.side .it .dot.on{background:var(--ok);box-shadow:0 0 6px var(--ok)}
.content{flex:1;min-width:0;min-height:0;display:flex;flex-direction:column;overflow:hidden}
.view{display:none;flex:1;min-height:0;flex-direction:column;overflow:auto;padding:16px}.view.on{display:flex}
.vh{display:flex;align-items:baseline;gap:12px;margin-bottom:12px;flex-wrap:wrap}.vh h2{margin:0;font-size:18px}.vh .d{color:var(--dim);font-size:12px}
.vh .tabs{display:flex;gap:4px;margin-left:auto}
.tab{font:inherit;font-size:12px;padding:5px 12px;border-radius:999px;border:1px solid var(--line);background:var(--panel2);color:var(--dim);cursor:pointer}
.tab.on{color:#04121f;background:var(--accent);border-color:var(--accent);font-weight:700}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--r);overflow:hidden;display:flex;flex-direction:column}
.card .h{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim);box-shadow:inset 3px 0 0 var(--accent);padding:9px 13px;
  border-bottom:1px solid var(--line);background:var(--panel2);display:flex;justify-content:space-between;gap:8px}
.card .b{padding:13px;display:flex;flex-direction:column;gap:10px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}
button{font:inherit;font-size:13px;font-weight:700;padding:12px 10px;border-radius:8px;cursor:pointer;background:var(--panel2);border:1px solid var(--line);color:var(--ink)}
button small{display:block;font-size:10.5px;font-weight:400;color:var(--dim);margin-top:3px}
button:disabled{opacity:.35;cursor:not-allowed}
.go{border-color:#0b5a70;color:var(--accent)}.go:hover:not(:disabled){background:#082c38}
.st{border-color:#5a2b2b;color:var(--warn)}.st:hover:not(:disabled){background:#2e1515}
.ok2{border-color:#1f5a43;color:var(--ok)}.ok2:hover:not(:disabled){background:#0f2a22}
.sm{padding:6px 12px;font-size:12px}
button .no{display:block;font-size:20px;margin-bottom:2px}
button.pend{box-shadow:0 0 0 2px var(--amber) inset;opacity:1!important}
button.next:not(:disabled){box-shadow:0 0 0 2px var(--accent) inset;animation:nx 1.6s infinite}
@keyframes nx{50%{box-shadow:0 0 0 2px transparent inset}}
.row{display:flex;gap:8px;flex-wrap:wrap}.row>*{flex:1}
.state{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:8px;border:1px solid var(--line);background:var(--panel2)}
.state::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--dim);flex:none}
.state.run{border-color:#6a4a1f;color:var(--amber)}.state.run::before{background:var(--amber);animation:bl 1s infinite}
.state.on{border-color:#1f5a43;color:var(--ok)}.state.on::before{background:var(--ok)}
.state.err{border-color:#5a2b2b;color:var(--warn)}.state.err::before{background:var(--warn)}
@keyframes bl{50%{opacity:.25}}
pre{margin:0;min-height:120px;max-height:40vh;overflow:auto;background:#0a0d12;border:1px solid var(--line);border-radius:8px;padding:10px;font-size:11.5px;white-space:pre-wrap;word-break:break-all}
.note{font-size:11.5px;color:var(--dim)}
.fsmnow{display:flex;align-items:baseline;gap:12px;padding:12px 14px;border-radius:8px;border:1px solid var(--line);background:#0a0d12}
.fsmnow .k{font-size:11px;color:var(--dim)}.fsmnow .v{font-size:22px;font-weight:700}.fsmnow .n{font-size:13px;color:var(--dim)}
.fsmnow.bal .v{color:var(--ok)}.fsmnow.std .v{color:var(--accent)}.fsmnow.low .v{color:var(--amber)}.fsmnow.unk .v{color:var(--warn);font-size:14px}
/* 대시보드 */
.steps{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}
.stp{background:var(--panel);border:1px solid var(--line);border-radius:var(--r);padding:12px 14px;cursor:pointer;min-height:118px;display:flex;flex-direction:column;gap:4px}
.stp:hover{border-color:var(--accent)}.stp .t{font-size:14px;font-weight:700}.stp .v{font-size:24px;font-weight:700;color:#fff}.stp .s{font-size:11px;color:var(--dim)}
.stp.run{border-color:#1f5a43;box-shadow:inset 0 3px 0 var(--ok)}
.tbl{width:100%;border-collapse:collapse;font-size:12px}.tbl th{font-weight:400;color:var(--dim);font-size:10.5px;text-align:left;padding:0 6px 5px;border-bottom:1px solid var(--line)}
.tbl td{padding:6px;border-bottom:1px solid var(--line);white-space:nowrap}
/* 기능 화면 */
.frame{flex:1;min-height:0;border:1px solid var(--line);border-radius:var(--r);overflow:hidden;background:#05080c;display:none}
.frame.on{display:block}.frame iframe{width:100%;height:100%;border:0;display:block}
.startp{display:none;flex-direction:column;gap:12px;max-width:900px}.startp.on{display:flex}
.modal{position:fixed;inset:0;background:#000b;display:none;align-items:center;justify-content:center;z-index:9}.modal.show{display:flex}
.mbox{width:min(520px,92vw);background:var(--panel);border:2px solid var(--warn);border-radius:12px;overflow:hidden}
.mbox .mt{background:#2e1515;color:var(--warn);font-size:16px;font-weight:700;padding:12px 16px}
.mbox .mm{padding:16px;font-size:13px;line-height:1.7;white-space:pre-line}.mbox .mm .big{display:block;color:var(--warn);font-weight:700;font-size:14.5px;margin-bottom:8px}
.mbox .mb{display:flex;gap:8px;padding:0 16px 16px}.mbox .mb button{flex:1}.mbox .run{background:#5a1f1f;border-color:var(--warn);color:#fff}
@media(max-width:1100px){.steps{grid-template-columns:repeat(2,1fr)}.side{width:180px}}
</style></head><body>
<script src="/i18n.js"></script>
<div class="top">
  <span class="brand" id="brand" onclick="go('home')">pitron<small id="brand-sub"></small></span>
  <span class="chip" id="chip-fsm" onclick="go('robot')" title="로봇 자세 (FSM)">FSM <b>-</b></span>
  <span class="chip" id="chip-job" onclick="go('home')">실행 <b>없음</b></span>
  <div class="r">
    <span class="estop">웹 버튼은 비상정지가 아님 — E-STOP · 리모컨을 손에</span>
    <button class="st sm" id="b-stop" onclick="appStop()">정지</button>
    <button class="st sm" id="b-shut" onclick="shutdownAll()" title="실행 중인 것 정지 → @@LOCK@@ → @@DAMP@@">종료</button>
    <span id="lang-slot"></span>
  </div>
</div>
<div class="main">
  <nav class="side" id="side"></nav>
  <section class="content" id="content">
    <!-- 홈 -->
    <div class="view" id="v-home">
      <div class="vh"><h2>홈</h2><span class="d" id="home-d">로봇 준비 → 데이터 수집 → 학습 → 추론</span></div>
      <div class="steps" id="steps"></div>
      <div class="grid" style="margin-top:12px">
        <div class="card"><div class="h"><span>로봇</span><a href="#robot" onclick="go('robot');return false">자세 →</a></div>
          <div class="b"><div class="fsmnow unk" id="fsmnow-h"><span class="k">FSM</span><span class="v" id="fsm-v-h">확인 중</span><span class="n" id="fsm-n-h"></span></div>
          <div class="note">일어서기 <b>@@STAND_SEQ@@</b> · 끝낼 때 [종료] (실행 중인 것 정지 → @@LOCK@@ → @@DAMP@@)</div></div></div>
        <div class="card"><div class="h"><span>실행 중</span><span id="job-mode"></span></div>
          <div class="b"><div class="state" id="rb-s">정지됨</div><pre id="rb-log"></pre></div></div>
      </div>
      <div class="grid" style="margin-top:12px">
        <div class="card"><div class="h"><span>데이터셋</span><span>data/datasets</span></div><div class="b"><table class="tbl" id="t-ds"></table></div></div>
        <div class="card"><div class="h"><span>모델</span><span>data/policies</span></div><div class="b"><table class="tbl" id="t-md"></table></div></div>
        <div class="card"><div class="h"><span>학습 작업 (이 PC)</span><span>data/trainer</span></div><div class="b"><table class="tbl" id="t-jb"></table></div></div>
      </div>
    </div>
    <!-- 로봇 -->
    <div class="view" id="v-robot">
      <div class="vh"><h2>로봇 (자세)</h2><span class="d">로봇을 보면서 한 단계씩 — 시작 @@STAND_SEQ@@, 끝낼 때 [종료]</span></div>
      <div class="card" style="max-width:900px"><div class="b">
        <div class="fsmnow unk" id="fsmnow"><span class="k">현재 FSM</span><span class="v" id="fsm-v">확인 중</span><span class="n" id="fsm-n"></span></div>
@@FSM_BUTTONS@@
        <button class="st" onclick="shutdownAll()">종료<small>실행 중인 것 정지 → @@LOCK@@ → @@DAMP@@ (시작의 역순, 각 단계 카운트다운 · 취소 가능)</small></button>
        <div class="row" style="align-items:stretch"><div class="state" id="fsm-s" style="flex:3">대기</div>
          <button class="st sm" id="b-cancel" onclick="fsmCancel()" style="flex:1;display:none">취소</button></div>
        <pre id="fsm-log"></pre>
        <div class="note">실기 기능은 FSM @@RUN@@ 에서만 시작</div>
      </div></div>
    </div>
    <!-- 기능 화면 (메뉴 항목마다 하나, JS 가 만듦) -->
  </section>
</div>
<div class="modal" id="modal"><div class="mbox"><div class="mt" id="m-t">⚠️</div><div class="mm" id="m-m"></div>
  <div class="mb"><button id="m-no">취소</button><button class="run" id="m-yes">실행</button></div></div></div>
<script>
const FSM=@@FSM_JS@@;
const host=location.hostname,$=id=>document.getElementById(id);
const esc=t=>String(t??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const pageUrl=t=>`http://${host}:${t.port}${t.path}`;
let MENU=null,ITEMS={},CUR=null,ST=null,VIEW='home',TAB={};
function warn(title,big,body,yes){return new Promise(res=>{const m=$('modal');$('m-t').textContent='⚠️ '+title;
  $('m-m').innerHTML=(big?`<span class="big">${big}</span>`:'')+body;$('m-yes').textContent=yes||'실행';
  const done=v=>{m.classList.remove('show');document.removeEventListener('keydown',ek);res(v);};const ek=e=>{if(e.key==='Escape')done(false);};
  $('m-no').onclick=()=>done(false);$('m-yes').onclick=()=>done(true);document.addEventListener('keydown',ek);m.classList.add('show');$('m-no').focus();});}
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));if(!r.ok)throw new Error(typeof d.detail==='string'?d.detail:JSON.stringify(d.detail));return d;}

// ---------- 메뉴 · 화면 ----------
function buildMenu(){
  $('brand').firstChild.textContent=MENU.brand.name;$('brand-sub').textContent=MENU.brand.sub;document.title=MENU.brand.name;
  let h=`<div class="it on" data-v="home" onclick="go('home')">▣ 홈</div>`;
  for(const s of MENU.sections){h+=`<div class="sec">${esc(s.title)}</div>`;
    for(const it of s.items){ITEMS[it.id]=it;h+=`<div class="it" data-v="${it.id}" onclick="go('${it.id}')">${esc(it.title)}<span class="dot" id="dot-${it.id}"></span></div>`;
      if(it.id==='robot')continue;
      const v=document.createElement('div');v.className='view';v.id='v-'+it.id;
      v.innerHTML=`<div class="vh"><h2>${esc(it.title)}</h2><span class="d">${esc(it.desc||'')}</span><div class="tabs" id="tabs-${it.id}"></div></div>
        <div class="startp" id="sp-${it.id}"></div><div class="frame" id="fr-${it.id}"></div>`;
      $('content').appendChild(v);}}
  $('side').innerHTML=h;
  const st=[['collect','① 데이터 수집'],['dataset','② 데이터셋'],['train','③ 학습'],['model','④ 모델'],['infer','⑤ 추론']];
  $('steps').innerHTML=st.map(([id,t])=>`<div class="stp" id="stp-${id}" onclick="go('${id}')"><span class="t">${t}</span><span class="v" id="sv-${id}">-</span><span class="s" id="ss-${id}"></span></div>`).join('');}
function go(v){VIEW=v;location.hash=v==='home'?'':v;
  document.querySelectorAll('.side .it').forEach(e=>e.classList.toggle('on',e.dataset.v===v));
  document.querySelectorAll('.view').forEach(e=>e.classList.toggle('on',e.id==='v-'+v));render();}
function running(){return ST&&ST.job.running?ST.job.mode:null;}
function renderItem(it){
  const run=running(),prep=ST&&ST.job.prep,mine=it.modes.some(m=>m.id===run),tabs=it.tabs||[];
  const ti=Math.min(TAB[it.id]||0,Math.max(0,tabs.length-1));
  $('tabs-'+it.id).innerHTML=(tabs.length>1?tabs.map((t,i)=>`<button class="tab ${i===ti?'on':''}" onclick="TAB['${it.id}']=${i};render(true)">${esc(t.title)}</button>`).join(''):'')+
    (mine&&!prep&&tabs.length?`<button class="tab" onclick="window.open('${pageUrl(tabs[ti])}','_blank')">새 창 ↗</button>`:'');
  const fr=$('fr-'+it.id),sp=$('sp-'+it.id);
  if(mine&&!prep&&tabs.length){
    const url=pageUrl(tabs[ti]);
    if(fr.dataset.url!==url||!fr.querySelector('iframe')){
      if(fr.dataset.probe===url)return;fr.dataset.probe=url;                 // 화면이 뜰 때까지 확인 후 넣음
      fetch(url,{mode:'no-cors',cache:'no-store'}).then(()=>{fr.innerHTML=`<iframe src="${url}"></iframe>`;fr.dataset.url=url;fr.classList.add('on');sp.classList.remove('on');})
        .catch(()=>{}).finally(()=>{fr.dataset.probe='';});
      startPanel(it,sp,true);return;}
    fr.classList.add('on');sp.classList.remove('on');return;}
  fr.classList.remove('on');fr.innerHTML='';fr.dataset.url='';startPanel(it,sp,false);}
function startPanel(it,sp,loading){
  sp.classList.add('on');const run=running(),j=ST?ST.job:null,other=run&&!it.modes.some(m=>m.id===run);
  const fsmOk=m=>!m.real||!ST||ST.fsm.cur===null||ST.fsm.cur===FSM.run;
  let h=`<div class="card"><div class="h"><span>${loading?'시작 중':'시작'}</span><span>${other?'지금 실행 중: '+esc(j.label)+' — 시작하면 정지하고 전환':''}</span></div><div class="b">`;
  if(!it.modes.length)h+=`<div class="note">이 화면은 실행이 필요 없음</div>`;
  else h+=`<div class="row">`+it.modes.map(m=>`<button class="${m.real?'go':'ok2'}" ${loading||(ST&&ST.seq.busy)||!fsmOk(m)?'disabled':''} onclick="appStart('${m.id}')">${esc(m.label)}`+
    `<small>${m.real?'실제 로봇 · FSM '+FSM.run+(fsmOk(m)?'':' 필요 (로봇 메뉴)'):'로봇 없이'} · ${esc(m.app)}</small></button>`).join('')+`</div>`;
  if(it.remote)h+=`<div class="note" style="line-height:1.8">5090 에서 학습: ② 데이터셋 화면의 <b>[Dataset zip]</b> 받기 → 5090 학습 웹에 올려 학습 → 받은 <b>model zip</b> 을 ④ 모델 에서 올리기`+
    (MENU.trainer_url?` · <a href="${esc(MENU.trainer_url)}" target="_blank">5090 학습 웹 열기 ↗</a>`:` · (5090 주소는 config/local.yaml trainer_url)`)+`</div>`;
  if(j&&(j.prep||loading||(j.label&&!j.running)))h+=`<pre>${esc((j.log||[]).slice(-40).join('\n'))}</pre>`;
  h+=`</div></div>`;if(sp.dataset.h!==h){sp.innerHTML=h;sp.dataset.h=h;}}
function render(force){if(!MENU||!ST)return;if(ITEMS[VIEW]&&VIEW!=='robot')renderItem(ITEMS[VIEW]);}

// ---------- 시작 · 정지 · 종료 ----------
async function appStart(id){const all=Object.values(ITEMS).flatMap(i=>i.modes),m=all.find(x=>x.id===id);if(!m)return;
  const ok=await warn(m.label,m.real?'실제 로봇이 움직입니다':'로봇 없이 (가상)',m.real?'· 거치대 · FSM '+FSM.run+' 확인\n· 주변을 비우고 E-STOP 을 손에\n· 그리퍼는 서비스를 켜고 카메라 숙임을 3 초 잽니다 — 로봇을 건드리지 마세요':'· 가짜 로봇 (DDS 도메인 1) — 실기와 섞이지 않음','시작');
  if(!ok)return;
  let r=await fetch('/app/'+id+'/start',{method:'POST'}),d=await r.json().catch(()=>({}));
  if(r.status===409&&d.detail&&d.detail.conflicts){const cs=d.detail.conflicts;
    const body=cs.map(c=>`· ${esc(c.label)}${c.own?' (지금 실행 중)':' (터미널 · 남은 프로그램)'} — ${esc(c.what.join(', '))}`+c.notes.map(n=>`\n   ⚠ ${esc(n)}`).join('')).join('\n')+
      '\n\nCtrl+C 와 같은 방식으로 정상 종료한 뒤 시작합니다 (45 초 안에 안 꺼지면 강제).';
    if(!await warn('이미 떠 있는 것',`${cs.length} 개를 끄고 '${esc(m.label)}' 을 시작할까요?`,body,'끄고 시작'))return;
    r=await fetch('/app/'+id+'/start?replace=true',{method:'POST'});d=await r.json().catch(()=>({}));}
  if(!r.ok)alert(typeof d.detail==='string'?d.detail:JSON.stringify(d.detail));poll();}
async function appStop(){if(!await warn('정지','실행 중인 것을 정지합니다','팔은 기본 자세로 돌아간 뒤 제어권을 반납합니다 (실기).','정지'))return;
  try{await post('/app/stop');}catch(e){alert(e.message);}poll();}
async function shutdownAll(){
  if(!await warn('종료','실행 중인 것을 정지한 뒤 '+FSM.lock+' → '+FSM.damp+' (Damp)',
    '· 로봇이 거치대에 걸려 있습니까? Damp 하면 힘이 빠집니다\n· 각 단계는 카운트다운 — 취소하면 그 자리에서 멈춤\n· 로봇이 연결 안 됐으면 FSM 명령은 보내지 않음','종료'))return;
  try{await post('/shutdown');}catch(e){alert(e.message);}poll();}
async function fsm(t){const standing=FSM.standing.includes(ST&&ST.fsm.cur);
  if(t===FSM.damp&&!await warn(`${FSM.names[t]} (FSM ${t})`,standing?'지금 서 있습니다 — 힘이 빠져 넘어집니다!':'모터 힘이 빠집니다',
    '· 로봇을 사람이 받치고 있거나 거치대에 묶여 있습니까?\n· 주변에 사람/장애물이 없습니까?'))return;
  try{await post('/fsm/'+t);}catch(e){alert(e.message);}poll();}
async function fsmCancel(){try{await post('/fsm_cancel');}catch(e){}poll();}

// ---------- 상태 ----------
function setState(id,cls,txt){const el=$(id);el.className='state'+(cls?' '+cls:'');el.textContent=txt;}
function fill(pre,lines){const b=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-20;pre.textContent=lines.join('\n');if(b)pre.scrollTop=pre.scrollHeight;}
function fsmBox(f,box,v,n){
  if(f.cur===null){$(box).className='fsmnow unk';$(v).textContent='확인 불가';$(n).textContent=f.cur_err||'';}
  else{$(box).className='fsmnow '+(f.cur_bal?'bal':FSM.standing.includes(f.cur)?'std':'low');$(v).textContent=f.cur;$(n).textContent=f.cur_name+(f.cur_bal?' · 밸런스':'');}}
async function poll(){try{const d=await(await fetch('/status')).json();ST=d;const f=d.fsm,j=d.job,sq=d.seq;
  // 위쪽 막대
  const cf=$('chip-fsm');cf.className='chip '+(f.cur===null?'err':f.cur_bal?'ok':FSM.standing.includes(f.cur)?'std':'low');
  cf.innerHTML=`FSM <b>${f.cur===null?'확인 불가':f.cur+' '+esc(f.cur_name)}</b>`+(f.busy!==null?` → ${f.busy} (${Math.ceil(f.remain)}s)`:'');
  const cj=$('chip-job');cj.className='chip '+(j.prep?'run':j.running?'ok':'');cj.innerHTML=`실행 <b>${j.running?esc(j.label):'없음'}</b>`;
  $('b-stop').disabled=!j.running||sq.busy;$('b-shut').disabled=sq.busy;
  // 메뉴 점
  for(const id in ITEMS){const d2=$('dot-'+id);if(d2)d2.className='dot'+(j.running&&ITEMS[id].modes.some(m=>m.id===j.mode)?' on':'');}
  // 홈 · 로봇
  fsmBox(f,'fsmnow-h','fsm-v-h','fsm-n-h');fsmBox(f,'fsmnow','fsm-v','fsm-n');
  $('job-mode').textContent=j.label||'';
  if(sq.busy)setState('rb-s','run','종료: '+sq.msg);
  else if(j.prep)setState('rb-s','run',`${j.label} — 시작 준비 중`);
  else if(j.running)setState('rb-s','on',`${j.label} — 실행 중 (${Math.floor(j.elapsed/60)}분 ${Math.floor(j.elapsed%60)}초)`);
  else if(sq.msg&&sq.ok!==null)setState('rb-s',sq.ok?'':'err','종료: '+sq.msg);
  else if(j.label)setState('rb-s',j.rc===0?'':'err',`${j.label} — 정지됨 (rc=${j.rc})`);
  else setState('rb-s','','정지됨');
  fill($('rb-log'),j.log);
  const ref=f.cur!==null?f.cur:f.last_sent,next=ref!==null?FSM.next[ref]:undefined,pend=f.busy!==null&&f.remain>0;
  FSM.steps.forEach(t=>{const b=$('b-'+t);if(!b)return;b.disabled=(f.busy!==null&&!(t===FSM.damp&&pend&&f.busy!==FSM.damp))||!f.allowed[t];
    b.classList.toggle('next',t===next&&f.busy===null);b.classList.toggle('pend',t===f.busy);});
  $('b-cancel').style.display=pend?'':'none';
  if(pend)setState('fsm-s','run',`FSM ${f.busy} — ${Math.ceil(f.remain)}초 후 전송`);else if(f.busy!==null)setState('fsm-s','run',`FSM ${f.busy} 전송 중…`);
  else if(f.result)setState('fsm-s',f.result[0]?'on':'err',f.result[1]);else setState('fsm-s','','대기');
  fill($('fsm-log'),f.log);
  for(const id of ['collect','dataset','train','model','infer']){const e=$('stp-'+id);if(e&&ITEMS[id])e.classList.toggle('run',j.running&&ITEMS[id].modes.some(m=>m.id===j.mode));}
  render();
}catch(e){$('chip-fsm').className='chip err';$('chip-fsm').innerHTML='FSM <b>런처 연결 끊김</b>';}}
async function pollSummary(){try{const s=await(await fetch('/summary')).json(),d0=s.datasets[0],m0=s.models[0],j0=s.jobs[0];
  const f1=v=>v==null?'-':(+v).toFixed(2)+'°';
  $('sv-collect').textContent=d0?d0.ok:'-';$('ss-collect').textContent=d0?`최근 ${d0.name} · 성공 에피소드`:'데이터셋 없음';
  $('sv-dataset').textContent=s.datasets.length;$('ss-dataset').textContent='데이터셋 · 에피소드 '+s.datasets.reduce((a,d)=>a+d.ok,0)+' 개';
  $('sv-train').textContent=j0?(j0.status==='running'?Math.round(100*(j0.step||0)/(j0.total||1))+'%':j0.status):'-';
  $('ss-train').textContent=j0?`${j0.dataset} · val ${f1(j0.val_err_deg)}`:'이 PC 학습 작업 없음';
  $('sv-model').textContent=s.models.length;$('ss-model').textContent=m0?`최근 ${m0.name} · val ${f1(m0.val_err_deg)}`:'모델 없음';
  $('sv-infer').textContent=m0?'준비':'-';$('ss-infer').textContent=m0?'모델을 골라 실행':'모델을 먼저 (④)';
  $('t-ds').innerHTML='<tr><th>이름</th><th>성공/전체</th><th>최근</th></tr>'+s.datasets.map(d=>`<tr><td>${esc(d.name)}</td><td>${d.ok}/${d.total}</td><td>${esc((d.last||'').replace('T',' ').slice(0,16))}</td></tr>`).join('');
  $('t-md').innerHTML='<tr><th>이름</th><th>데이터셋</th><th>step</th><th>val</th></tr>'+s.models.map(m=>`<tr><td>${esc(m.name)}</td><td>${esc(m.dataset)}</td><td>${m.steps??'-'}</td><td>${f1(m.val_err_deg)}</td></tr>`).join('');
  $('t-jb').innerHTML='<tr><th>작업</th><th>데이터셋</th><th>상태</th><th>val</th></tr>'+s.jobs.map(j=>`<tr><td>${esc(j.id)}</td><td>${esc(j.dataset)}</td><td>${esc(j.status)} ${j.status==='running'?Math.round(100*(j.step||0)/(j.total||1))+'%':''}</td><td>${f1(j.val_err_deg)}</td></tr>`).join('');
}catch(e){}}
fetch('/menu').then(r=>r.json()).then(m=>{MENU=m;buildMenu();go(location.hash.slice(1)||'home');poll();pollSummary();});
setInterval(poll,700);setInterval(pollSummary,5000);
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
        if h.count(old) < 1:
            raise SystemExit(f"[launcher] HTML 자리표시 '{old}' 없음")
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
