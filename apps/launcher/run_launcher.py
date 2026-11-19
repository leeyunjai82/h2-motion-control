#!/usr/bin/env python3
"""
apps/launcher/run_launcher.py — H2 자세(FSM) 제어 + start_robot.sh 실행 웹 (포트 80)

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

로봇 서버
  · Robot 시작 / 정지 : ./start_robot.sh 실행 / SIGTERM(= Ctrl+C 와 같은 종료 시퀀스)

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
from pitron import paths

import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, FileResponse

PORT = 80                   # 1024 미만 → root 필요 (launcher.sh 가 sudo 로 실행)
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

    def start(self, args, label, as_user=False):
        with self.lock:
            if self.running():
                raise HTTPException(409, f"{self.name} 실행 중: {self.label}")
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
                env=_user_env() if drop else None)
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
robot = Job("robot")


def _stop_robot():
    p = robot.proc
    if p is None or p.poll() is not None:
        return
    # bash 에만 SIGTERM → start_robot.sh 의 trap cleanup (Ctrl+C 와 동일 시퀀스)
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(ROBOT_STOP_WAIT)
    except subprocess.TimeoutExpired:
        robot.lines.append("[launcher] 종료 지연 — 프로세스 그룹 SIGKILL")
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=fsm.poll_loop, daemon=True).start()
    yield
    # launcher 종료(Ctrl+C) 시 로봇 서버도 정상 종료 시퀀스로 정지.
    # 남겨두면 다음 start_robot.sh 의 sweep 이 SIGKILL 로 죽여 arm 제어권 반납이 생략된다.
    if robot.running():
        print("[launcher] 종료 — start_robot.sh 정지 중...")
        _stop_robot()


app = FastAPI(title="pitron launcher", lifespan=lifespan)


@app.post("/fsm/{target}")
def run_fsm(target: int):
    fsm.set(target, robot.running)
    return {"ok": True, "delay": DELAY_SEC}


@app.post("/fsm_cancel")
def fsm_cancel():
    return {"ok": fsm.cancel_pending()}


@app.post("/robot/start")
def robot_start():
    if fsm.busy is not None:
        raise HTTPException(409, f"FSM {fsm.busy} 대기/전송 중")
    cur = fsm.read_fsm()
    if cur is not None and cur != FSM_RUN:
        raise HTTPException(409, f"FSM {cur} ({FSM_NAME.get(cur, '')}) — {FSM_DAMP} → {FSM_LOCK} → {FSM_RUN} 로 {_LBL['run_enter']} 진입 후 시작")
    robot.start(["bash", os.path.join(ROOT, "start_robot.sh")], "start_robot.sh", as_user=True)
    if cur is None:
        robot.lines.append(f"[launcher] ⚠️ FSM 확인 불가({fsm.cur_err}) — {FSM_RUN} 상태인지 직접 확인할 것")
    return {"ok": True}


@app.post("/robot/stop")
def robot_stop():
    if not robot.running():
        return {"ok": True, "already": True}
    threading.Thread(target=_stop_robot, daemon=True).start()
    return {"ok": True}


@app.get("/status")
def status():
    return {"fsm": fsm.state(robot.running()), "robot": robot.state()}


@app.get("/", response_class=HTMLResponse)
def index():
    return HTML


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
      <div class="row" style="align-items:stretch">
        <div class="state" id="fsm-s" style="flex:3">대기</div>
        <button class="st" id="b-cancel" onclick="fsmCancel()" style="flex:1;padding:8px;display:none">취소</button>
      </div>
      <pre id="fsm-log"></pre>
    </div>
  </div>
  <div class="card">
    <div class="h"><span>로봇 서버 <code>start_robot.sh</code></span><span></span></div>
    <div class="b">
      <div class="row">
        <button class="go" id="b-rstart" onclick="robotStart()">Robot 시작<small>FSM @@RUN@@ 에서만 · 6개 서버</small></button>
        <button class="st" id="b-rstop" onclick="robotStop()">Robot 정지<small>Ctrl+C 와 동일</small></button>
      </div>
      <div class="state" id="rb-s">정지됨</div>
      <pre id="rb-log"></pre>
      <div class="links note" id="links"></div>
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
document.getElementById('links').innerHTML=
  `<a href="http://${host}:50000/" target="_blank">Control :50000</a>`+
  `<a href="http://${host}:50003/dashboard" target="_blank">Dashboard :50003</a>`;
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
async function robotStart(){if(!confirm('start_robot.sh 를 실행합니다.\n\n· arm_server 가 기동하면서 팔/허리 제어권(weight=1)을 잡습니다.'))return;
  try{await post('/robot/start');}catch(e){alert(e.message);}poll();}
async function robotStop(){if(!confirm('로봇 서버를 정지합니다 (팔 제어권 반납 후 종료).'))return;
  try{await post('/robot/stop');}catch(e){alert(e.message);}poll();}
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
  // 로봇 서버
  document.getElementById('b-rstart').disabled=r.running||f.busy!==null||(f.cur!==null&&f.cur!==FSM.run);
  document.getElementById('b-rstart').classList.toggle('next',f.cur===FSM.run&&!r.running);
  document.getElementById('b-rstop').disabled=!r.running;
  if(r.running)setState('rb-s','on',`실행 중 (${Math.floor(r.elapsed/60)}분 ${Math.floor(r.elapsed%60)}초)`);
  else if(r.label)setState('rb-s',r.rc===0?'':'err',`정지됨 (rc=${r.rc})`);
  else setState('rb-s','','정지됨');
  fill(document.getElementById('rb-log'),r.log);
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
