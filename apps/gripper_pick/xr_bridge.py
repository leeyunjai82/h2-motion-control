"""
xr_bridge.py — Quest 연결 (xr_virtual.py --real: televuer · Vuer :8012 + H2 IK) 을 잡기 웹이 켜고 · 끄고 · 다시 켬 + adb 로 Quest 페이지 열기

  연결 프로그램은 잡기 웹의 자식 프로세스. 1 초마다 /api/xr/hello 로 상태 (Quest 입력이 들어오는지) 를 보냄.
  Quest 쪽: USB 연결 + 개발자 모드 · USB 디버깅 허용 → [Quest 페이지 열기] = adb reverse tcp:8012 + 브라우저로 https://localhost:8012 열기
"""
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron import ports  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VUER_PORT = int(ports.PORTS.get("xr_vuer", 8012))
DISPLAYS = ("pass-through", "immersive")
HELLO_STALE_S = 3.0          # 연결 프로그램 신호가 이만큼 없으면 '응답 없음'


def _port_open(port):
    s = socket.socket()
    s.settimeout(0.3)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def _pids(pattern):
    try:
        out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True, timeout=3).stdout
        return [int(x) for x in out.split() if int(x) != os.getpid()]
    except (OSError, subprocess.SubprocessError, ValueError):
        return []


class XRBridge:
    def __init__(self, xr_dir=None, log=print):
        self.xr_dir = xr_dir or os.environ.get("XR_DIR") or os.path.expanduser("~/xr_teleoperate")
        self.log = log
        self.lk = threading.Lock()
        self.proc = None
        self.display = "pass-through"
        self.since = None
        self.lines = deque(maxlen=40)
        self.hello = {"t": 0.0}
        self.err = ""
        self.quest_msg = ""

    # ----- 연결 프로그램 -----
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, display=None):
        with self.lk:
            if display:
                if display not in DISPLAYS:
                    raise ValueError(f"화면 방식: {DISPLAYS}")
                self.display = display
            if self.running():
                return self.status()
            if not os.path.isdir(os.path.join(self.xr_dir, "teleop")):
                raise RuntimeError(f"xr_teleoperate 없음: {self.xr_dir} (XR_DIR 로 지정)")
            for pid in _pids("apps/gripper_pick/xr_virtual.py"):      # 앞에 띄운 것이 남아 8012 를 잡고 있으면 Quest 가 옛 것에 붙음
                self._term(pid)
            if _port_open(VUER_PORT):
                raise RuntimeError(f"{VUER_PORT} 포트를 다른 프로그램이 쓰는 중 — 그 프로그램을 끄고 다시 연결")
            self.err, self.hello = "", {"t": 0.0}
            self.lines.clear()
            self.proc = subprocess.Popen([sys.executable, "-u", os.path.join(HERE, "xr_virtual.py"), "--xr", self.xr_dir, "--real",
                                          "--display", self.display], cwd=_ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, bufsize=1, start_new_session=True)
            self.since = time.time()
            threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        self.log(f"Quest 연결 켬 (Vuer :{VUER_PORT}, {self.display})")
        return self.status()

    def _pump(self, p):
        for ln in p.stdout:
            ln = ln.rstrip()
            if ln:
                self.lines.append(time.strftime("%H:%M:%S ") + ln)
                print(ln, flush=True)                  # 잡기 웹 로그 (pick_web_*.log) 에도 남김
        rc = p.wait()
        if self.proc is p and rc not in (0, -signal.SIGINT, -signal.SIGTERM):
            self.err = f"연결 프로그램이 끝남 (rc {rc}) — " + (self.lines[-1][9:] if self.lines else "")

    @staticmethod
    def _term(pid, wait=5.0):
        try:
            os.kill(pid, signal.SIGINT)          # xr_virtual 은 Ctrl+C 처럼 끝냄 (팔 세션 정리 · Vuer 닫기)
        except ProcessLookupError:
            return
        t = time.time()
        while time.time() - t < wait:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def stop(self):
        with self.lk:
            p, self.proc = self.proc, None
            if p is not None and p.poll() is None:
                self._term(p.pid)
            self.since, self.hello = None, {"t": 0.0}
        self.log("Quest 연결 끔")
        return self.status()

    def restart(self, display=None):
        self.stop()
        time.sleep(1.0)                          # Vuer 포트가 풀릴 시간
        return self.start(display)

    def on_hello(self, d):
        self.hello = {**(d or {}), "t": time.time()}

    # ----- Quest (adb) -----
    @staticmethod
    def adb_state():
        if not shutil.which("adb"):
            return "adb 없음"
        try:
            r = subprocess.run(["adb", "get-state"], capture_output=True, text=True, timeout=4)
        except subprocess.TimeoutExpired:
            return "응답 없음"
        if r.returncode == 0:
            return r.stdout.strip() or "?"
        err = (r.stderr or "").lower()
        return "권한 허용 필요" if "unauthorized" in err else "연결 안 됨"

    def quest_open(self):
        """USB 로 연결된 Quest 에서 Vuer 페이지 열기 (adb reverse + 브라우저)."""
        st = self.adb_state()
        if st != "device":
            self.quest_msg = f"Quest 가 adb 에 안 잡힘 ({st}) — USB 연결 · 헤드셋 안에서 'USB 디버깅 허용'"
            raise RuntimeError(self.quest_msg)
        if not self.running():
            raise RuntimeError("먼저 [연결] — Quest 연결 프로그램이 꺼져 있음")
        url = f"https://localhost:{VUER_PORT}/?ws=wss://localhost:{VUER_PORT}"
        cmds = [["adb", "shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.prox_close"],
                ["adb", "reverse", f"tcp:{VUER_PORT}", f"tcp:{VUER_PORT}"],
                ["adb", "shell", "am", "start", "-a", "android.intent.action.VIEW", "-d", url, "com.oculus.browser"]]
        for i, c in enumerate(cmds):                 # 0 = 화면 켜 두기 (실패해도 계속) · 1 = 포트 · 2 = 브라우저 (둘은 꼭 성공)
            r = subprocess.run(c, capture_output=True, text=True, timeout=10)
            if i > 0 and r.returncode != 0:
                self.quest_msg = f"{' '.join(c[:3])} 실패: {(r.stderr or r.stdout).strip()[:160]}"
                raise RuntimeError(self.quest_msg)
        self.quest_msg = "Quest 페이지 열음 — 헤드셋에서 " + ("'pass-through'" if self.display == "pass-through" else "'Virtual Reality'") + " 누르기"
        self.log(self.quest_msg)
        return self.quest_msg

    # ----- 상태 -----
    def status(self, with_adb=False):
        run = self.running()
        age = time.time() - self.hello["t"] if self.hello.get("t") else None
        alive = run and age is not None and age < HELLO_STALE_S
        out = {"running": run, "display": self.display, "since": self.since, "vuer": _port_open(VUER_PORT) if run else False,
               "alive": alive, "input": bool(alive and self.hello.get("input")), "sent": self.hello.get("n"),
               "hello_age_s": None if age is None else round(age, 1), "err": self.err, "quest_msg": self.quest_msg,
               "log": list(self.lines)[-8:], "xr_dir": self.xr_dir, "port": VUER_PORT}
        if with_adb:
            out["adb"] = self.adb_state()
        return out
