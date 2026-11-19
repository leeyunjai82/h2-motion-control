"""
sys_watch.py — 제어 화면 System 카드용: 로봇 FSM + 각 서버 상태 (period 초마다, 백그라운드 스레드 하나)

  FSM    : LocoClient.GetFsmId (RPC 7001, 읽기 전용 — 명령 보내지 않음). 이름은 robot.yaml fsm.names
           팔(arm_sdk)이 움직이는 FSM = robot.yaml fsm.lock(4) · fsm.run(703) (SDK 814556d 예제, 실기 703 ✓ · 601 ✗)
  서버   : localhost HTTP (0.5 초 타임아웃)
           arm_server :50022 /status (ready·mode·weight) · rs_stream :50001 / · detect_box :50010 /status
           head_track :50013 /status (RGB 수신 오류) · dashboard :50003 /api/status
  state  : ok(초록) · warn(노랑) · err(빨강) · off(회색 — 선택 서버가 안 떠 있음)

  sw = SysWatch(fsm_cfg, sim=robot_env.SIM).start()   # robot_env.dds_init() 뒤에. 시뮬이면 rs_stream 도 선택 서버
  sw.latest()  → {"fsm": {"id": 703, "name": "PhaseWalk", "arm_ok": True, "state": "ok", "text": "703 PhaseWalk"},
                  "services": [{"name": "arm_server", "port": 50022, "state": "ok", "text": "hold · w 1.00"}, …], "t": …}
  sw.fsm_warning() → Box 를 눌러도 팔이 안 움직일 FSM 이면 안내 문자열, 아니면 ""
"""
import json
import threading
import time
import urllib.request

# (이름, 포트, 경로, 필수 여부) — 필수 서버가 응답 없으면 빨강, 선택 서버면 회색
SERVICES = (
    ("arm_server", 50022, "/status", True),
    ("rs_stream", 50001, "/", True),
    ("detect_box", 50010, "/status", True),
    ("head_track", 50013, "/status", False),
    ("dashboard", 50003, "/api/status", False),
)


def _get(port, path, timeout=0.5):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as r:
        body = r.read(200000)
    try:
        return json.loads(body)
    except ValueError:
        return {}


def _arm(d):
    if not d.get("ready"):
        return "err", "준비 안 됨", "arm_server 가 떠 있지만 팔 미초기화 — arm_server 로그 확인"
    mode, w = d.get("mode") or "-", d.get("weight")
    text = f"{mode} · w {w:.2f}" if isinstance(w, (int, float)) else str(mode)
    if mode != "hold" or (isinstance(w, (int, float)) and w < 0.99):
        return "warn", text, "제어권 반납(release) 상태 — 팔 명령이 먹지 않음"
    return "ok", text, "arm_sdk 제어권 점유 (hold)"


def _head(d):
    errs = [f"{k}: {v['error']}" for k, v in d.items() if isinstance(v, dict) and v.get("error")]
    if errs:
        return "warn", "RGB 없음", " / ".join(errs)
    fps = [v.get("fps") for v in d.values() if isinstance(v, dict) and v.get("fps") is not None]
    return "ok", (f"{min(fps):.0f} fps" if fps else "OK"), "머리 카메라 RGB 수신 중"


def _detect(d):
    if "auto_enabled" in d:
        return "ok", "자동 ON" if d.get("auto_enabled") else "OK", "박스 인식"
    return "ok", "OK", "박스 인식"


def _dash(d):
    return ("ok", "OK", "3D 뷰어 · rt/lowstate 수신") if d.get("connected") else \
           ("warn", "로봇 없음", "dashboard 는 떠 있지만 rt/lowstate 미수신")


CHECK = {"arm_server": _arm, "head_track": _head, "detect_box": _detect, "dashboard": _dash}


class SysWatch:
    def __init__(self, fsm_cfg=None, period=2.0, sim=False):
        fsm_cfg = fsm_cfg or {}
        self.sim = bool(sim)                     # start_grab_sim.sh 는 rs_stream 을 안 띄움 (가상 카메라)
        self.names = {int(k): str(v) for k, v in (fsm_cfg.get("names") or {}).items()}
        self.arm_fsm = {int(v) for v in (fsm_cfg.get("lock"), fsm_cfg.get("run")) if v is not None}
        self.run_fsm = fsm_cfg.get("run")
        self.period = float(period)
        self.loco = None
        self.loco_err = ""
        self.lock = threading.Lock()
        self.state = {"fsm": {"id": None, "state": "off", "text": "-", "why": "재는 중"},
                      "services": [], "t": None}
        self._th = None

    # ---- FSM ----
    def _loco(self):
        if self.loco is None:
            from pitron.core import robot_env
            c = robot_env.loco_client_class()()
            c.SetTimeout(1.0)
            c.Init()
            self.loco = c
        return self.loco

    def _fsm(self):
        if self.sim:
            return {"id": None, "state": "off", "text": "시뮬", "why": "start_grab_sim.sh — 가상 로봇에는 FSM 없음"}
        try:
            code, fid = self._loco().GetFsmId()
        except Exception as e:                   # noqa: BLE001 — SDK·DDS 오류는 화면에 이유로
            return {"id": None, "state": "err", "text": "조회 실패", "why": f"GetFsmId: {e}"}
        if code != 0 or fid is None:
            return {"id": None, "state": "err", "text": "응답 없음",
                    "why": f"GetFsmId code {code} — 로봇 전원·네트워크 확인"}
        fid = int(fid)
        name = self.names.get(fid, "?")
        arm_ok = fid in self.arm_fsm
        why = "팔(arm_sdk) 동작 FSM" if arm_ok else \
              f"이 FSM 에서는 팔이 안 움직임 — 팔이 움직이는 FSM: {self.run_fsm} (./start_fsm.sh stand)"
        return {"id": fid, "name": name, "arm_ok": arm_ok, "state": "ok" if arm_ok else "warn",
                "text": f"{fid} {name}", "why": why}

    # ---- 서버 ----
    def _services(self):
        out = []
        for name, port, path, required in SERVICES:
            try:
                d = _get(port, path)
                st, text, why = CHECK.get(name, lambda _d: ("ok", "OK", ""))(d)
            except Exception as e:               # noqa: BLE001
                req = required and not (self.sim and name == "rs_stream")
                st, text, why = ("err" if req else "off"), "응답 없음", f":{port}{path} {e}"
            out.append({"name": name, "port": port, "state": st, "text": text, "why": why})
        return out

    def sample(self):
        out = {"fsm": self._fsm(), "services": self._services(), "t": time.time()}
        with self.lock:
            self.state = out
        return out

    def _loop(self):
        while True:
            t0 = time.time()
            try:
                self.sample()
            except Exception as e:               # noqa: BLE001 — 상태 표시 때문에 서버가 죽으면 안 됨
                print(f"[sys_watch] 실패: {e}")
            time.sleep(max(0.2, self.period - (time.time() - t0)))

    def start(self):
        if self._th is None:
            self._th = threading.Thread(target=self._loop, daemon=True, name="sys_watch")
            self._th.start()
        return self

    def latest(self):
        with self.lock:
            return dict(self.state)

    def fsm_warning(self):
        f = self.latest().get("fsm") or {}
        if f.get("id") is not None and not f.get("arm_ok"):
            return f"로봇 FSM {f.get('text')} — 팔이 움직이는 FSM: {self.run_fsm} (./start_fsm.sh stand)"
        return ""
