"""
lerobot_env.py — lerobot 가상환경 찾기 (표준 라이브러리만). 수집 v3 변환 · 학습 · 추론 프로세스가 이 python 으로 돎.

  config/local.yaml
    lerobot_python: /home/<사용자>/miniconda3/envs/lerobot/bin/python   (없으면 이 기본 경로)
    lerobot_repo_prefix: local                                            (데이터셋 repo_id 앞부분)
"""
import os
import pwd

from pitron import paths


def local_cfg():
    p = os.path.join(paths.REPO_ROOT, "config", "local.yaml")
    if not os.path.isfile(p):
        return {}
    try:
        import yaml
        with open(p, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001 — yaml 없는 환경: 간단히 'key: value' 만 읽음
        out = {}
        with open(p, encoding="utf-8") as f:
            for ln in f:
                if ":" in ln and not ln.startswith((" ", "#")):
                    k, v = ln.split(":", 1)
                    out[k.strip()] = v.split("#", 1)[0].strip().strip("'\"")
        return out


def _home():
    u = os.environ.get("SUDO_USER")
    return pwd.getpwnam(u).pw_dir if u else os.path.expanduser("~")


def python():
    return str(local_cfg().get("lerobot_python") or os.path.join(_home(), "miniconda3", "envs", "lerobot", "bin", "python"))


def repo_prefix():
    return str(local_cfg().get("lerobot_repo_prefix") or "local")


def missing_msg(py=None):
    return (f"lerobot 가상환경 없음: {py or python()} — 한 번 설치: apps/trainer/install_lerobot.sh "
            "(다른 위치면 config/local.yaml lerobot_python)")


class SyncWorker:
    """수집 데이터셋 → lerobot/ (LeRobot v3) 를 백그라운드에서 맞춤. 한 번에 하나, nice 10 (제어 루프보다 뒤).
    want(name) 로 요청 → state(name) = {state: idle|queued|running|ok|error|no_lerobot, msg, done, total}"""
    def __init__(self, data_dir, log=print):
        import threading
        self.data_dir, self.log = data_dir, log
        self.lk = threading.Lock()
        self.cv = threading.Condition(self.lk)
        self.queue, self.st = [], {}
        threading.Thread(target=self._loop, daemon=True).start()

    def want(self, name):
        with self.cv:
            if name not in self.queue:
                self.queue.append(name)
                if self.st.get(name, {}).get("state") != "running":
                    self.st.setdefault(name, {})["state"] = "queued"
            self.cv.notify()

    def state(self, name):
        with self.lk:
            return dict(self.st.get(name) or {"state": "idle"})

    def busy(self, name):
        with self.lk:
            return name in self.queue or (self.st.get(name) or {}).get("state") == "running"

    def _set(self, name, **kw):
        with self.lk:
            self.st.setdefault(name, {}).update(kw)

    def _loop(self):
        import subprocess
        while True:
            with self.cv:
                while not self.queue:
                    self.cv.wait()
                name = self.queue.pop(0)          # 도는 동안 다시 요청되면 끝난 뒤 한 번 더 (그 사이 바뀐 것 반영)
            try:
                py = python()
                if not os.path.isfile(py):
                    self._set(name, state="no_lerobot", msg=missing_msg(py))
                else:
                    self._set(name, state="running", msg="", done=0, total=0)
                    p = subprocess.Popen([py, "-u", "-m", "pitron.learning.lerobot_sync", os.path.join(self.data_dir, name)],
                                         cwd=paths.REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         env={**os.environ, "HF_HUB_OFFLINE": "1"}, preexec_fn=lambda: os.nice(10))
                    tail = []
                    for ln in p.stdout:
                        ln = ln.rstrip()
                        if ln.startswith("@@"):
                            try:
                                import json
                                j = json.loads(ln[2:])
                                if j.get("stage") == "episode":
                                    self._set(name, done=j["done"], total=j["total"])
                            except (ValueError, KeyError):
                                pass
                        elif ln:
                            tail = (tail + [ln])[-6:]
                    rc = p.wait()
                    if rc == 0:
                        self._set(name, state="ok", msg="")
                    else:
                        msg = next((t for t in reversed(tail) if "❌" in t or "Error" in t), tail[-1] if tail else f"rc {rc}")
                        self._set(name, state="error", msg=msg[-300:])
                        self.log(f"[lerobot] {name} 변환 실패: {msg[-300:]}")
            except Exception as e:  # noqa: BLE001 — 한 번 실패해도 작업 스레드는 계속 (다음 요청이 '대기' 로 멈추지 않게)
                self._set(name, state="error", msg=f"{type(e).__name__}: {e}"[-300:])
                self.log(f"[lerobot] {name} 변환 오류: {e}")
