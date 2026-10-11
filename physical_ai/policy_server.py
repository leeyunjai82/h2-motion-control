#!/usr/bin/env python3
# Version: 1.0
"""
policy_server.py — 피지컬 AI (ACT 모방학습) 로봇 PC 웹 서버 (포트 50030)

  ./physical_ai/start_policy.sh      (start_robot.sh 또는 start_grab_sim.sh 가 먼저 떠 있어야 함)
  → http://<로봇 PC>:50030/

하는 일
  1) 수집   robot_server 의 잡기(스크립트 시연)가 도는 동안 30 Hz 로 에피소드 기록
            D435i RGB(rs_stream /video_feed) + 팔 14축 실측(rt/lowstate) + 지령(rt/arm_sdk) + 손 위치(FK) + 잡기 단계
            · 잡기 연동: robot_server /grab_status busy 가 켜지면 시작, 꺼지면 끝 (/grab_history ok 로 성공 판정)
            · 자동 반복: /set_mode box → 박스 보이면 /grab_manual → 끝나면 쉬는 동안 사람이 박스를 옮김 → 반복
            · 수동: 시작/정지 버튼 (잡기와 무관하게 그 시간 동안 기록)
  2) 데이터셋 에피소드 목록·삭제, zip 다운로드 (학습 PC 의 trainer 웹에 업로드)
  3) 모델   trainer 가 만든 model zip 업로드 → 선택 → OpenVINO 컴파일 (CPU / GPU / NPU)
  4) 추론   30 Hz: 카메라 + 실측 → 정책 → 시간 앙상블 → 안전 필터 → arm_server /stream/cmd
            시작 조건: 팔 hold · 잡기 모드 OFF · 잡기/모션 안 도는 중 · 카메라·rt/lowstate 살아 있음
            정지: Stop 버튼 / Esc / 안전 필터 / 최대 시간. 정지하면 팔은 그 자리에서 멈춘다 (박스를 든 채일 수 있음)

기존 파일은 바꾸지 않는다. 팔 지령은 arm_server /stream/* 만, 잡기는 robot_server 기존 API 만 쓴다.
데이터 형식은 physical_ai/README.md.
"""
import json
import os
import re
import shutil
import sys
import threading
import time
import urllib.error
import urllib.request
import zipfile
from collections import deque
from typing import Optional

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pa_runtime as rt          # noqa: E402 — robot_env 도 여기서 (ROBOT 검사)
from pa_runtime import robot_env  # noqa: E402

import uvicorn                    # noqa: E402
from fastapi import FastAPI, File, HTTPException, UploadFile   # noqa: E402
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse   # noqa: E402
from starlette.background import BackgroundTask   # noqa: E402

PORT = 50030
FPS = 30
ARM = "http://localhost:50022"
ROBOT = "http://localhost:50000"
DETECT = "http://localhost:50010"          # detect_box (시뮬 가상 카메라면 sim_server 가 같은 자리)
SIM = robot_env.SIM
REAL_CAM = os.environ.get("SIM_CAMERA", "").strip() == "real"
SIM_VIRTUAL = SIM and not REAL_CAM         # 가상 박스 (sim_server /sim/box 로 위치 바꿀 수 있음)
CAMERA_URL = os.environ.get("PA_CAMERA_URL") or (
    "http://localhost:50010/video_feed" if SIM_VIRTUAL else "http://localhost:50001/video_feed")
DATA = os.environ.get("PA_DATA_DIR") or os.path.join(HERE, "data")
DS_DIR = os.path.join(DATA, "datasets")
MD_DIR = os.path.join(DATA, "models")
TMP_DIR = os.path.join(DATA, "tmp")
SETTINGS_PATH = os.path.join(DATA, "settings.json")
MAX_EP_SEC = 120.0                         # 한 에피소드 최대 길이 (넘으면 끊고 실패 처리)
LIFT_STAGE = "들기"                         # record until=lift 일 때 이 단계가 끝나면 기록 끝
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")
for d in (DS_DIR, MD_DIR, TMP_DIR):
    os.makedirs(d, exist_ok=True)


# =====================================================================
# 설정
# =====================================================================
DEFAULTS = {
    "dataset": "box_grab",
    "record": {"image": "320x240",      # 저장 해상도 (원본 640x480 / 320x240 = 용량 1/4)
               "until": "all",          # all = 복귀까지 / lift = 들기까지
               "save_failed": False,    # 실패한 잡기도 저장
               "link": False},          # 잡기 연동 기록
    "auto": {"count": 10, "pause_s": 8.0, "handover": "place", "wait_box_s": 20.0,
             "mode_off_after": True,
             "sim_random": True,        # 시뮬(가상 박스)만: 매 회 박스 위치 무작위
             "sim_x": [0.38, 0.44], "sim_y": [0.02, 0.045]},
    "policy": {"model": None, "device": "CPU", "mode": "ensemble", "coeff": 0.01, "n_action_steps": 20},
    "safety": dict(rt.DEFAULT_SAFETY),
}
_settings_lk = threading.Lock()


def _merge(base, over):
    out = json.loads(json.dumps(base))
    for k, v in (over or {}).items():
        if isinstance(out.get(k), dict) and isinstance(v, dict):
            out[k].update(v)
        elif k in out:
            out[k] = v
    return out


def load_settings():
    try:
        with open(SETTINGS_PATH, encoding="utf-8") as f:
            return _merge(DEFAULTS, json.load(f))
    except Exception:   # noqa: BLE001 — 없거나 깨지면 기본값
        return json.loads(json.dumps(DEFAULTS))


SET = load_settings()


def save_settings():
    with _settings_lk:
        tmp = SETTINGS_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(SET, f, ensure_ascii=False, indent=1)
        os.replace(tmp, SETTINGS_PATH)


# =====================================================================
# HTTP 도우미
# =====================================================================
def http_json(url, data=None, timeout=1.0, method=None):
    """→ (status, json). 네트워크 오류는 예외. 4xx/5xx 는 status 와 detail 을 돌려준다."""
    body = None if data is None else json.dumps(data).encode()
    req = urllib.request.Request(url, data=body, method=method or ("POST" if data is not None else "GET"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        try:
            d = json.loads(e.read() or b"{}")
        except Exception:   # noqa: BLE001
            d = {}
        return e.code, d


def try_json(url, data=None, timeout=1.0, method=None):
    try:
        return http_json(url, data, timeout, method)
    except Exception as e:   # noqa: BLE001
        return 0, {"error": f"{type(e).__name__}: {e}"}


# =====================================================================
# 공용 부품
# =====================================================================
cam = rt.CameraReader(CAMERA_URL)
watch = None
watch_err = ""
try:
    watch = rt.ArmWatch()
except Exception as e:   # noqa: BLE001
    watch_err = f"DDS 구독 실패: {e}"
    print(f"[policy] ⚠ {watch_err}")
fk = rt.HandFK()
STAGE_NAMES = []        # robot_server GRAB_STAGES (첫 /grab_status 에서 채움)


def log(msg):
    print(f"[policy] {msg}", flush=True)


# =====================================================================
# robot_server 잡기 상태 감시 (20 Hz)
# =====================================================================
class GrabMonitor:
    def __init__(self):
        self.st = {}
        self.t_ok = 0.0
        self.err = ""
        self.busy = False
        self.on_rise = []       # 콜백 (busy False→True)
        self.on_fall = []       # 콜백 (busy True→False)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        global STAGE_NAMES
        while True:
            code, d = try_json(f"{ROBOT}/grab_status", timeout=0.5)
            if code == 200:
                self.st, self.t_ok, self.err = d, time.time(), ""
                if d.get("stages"):
                    STAGE_NAMES = list(d["stages"])
                b = bool(d.get("busy"))
                if b != self.busy:
                    self.busy = b
                    for cb in (self.on_rise if b else self.on_fall):
                        try:
                            cb(d)
                        except Exception as e:   # noqa: BLE001
                            log(f"grab 콜백 오류: {e}")
            else:
                self.err = d.get("error") or f"HTTP {code}"
                if self.busy and time.time() - self.t_ok > 3.0:   # 잡기 중 robot_server 가 사라짐
                    self.busy = False
                    for cb in self.on_fall:
                        try:
                            cb({"lost": True})
                        except Exception as e:   # noqa: BLE001
                            log(f"grab 콜백 오류: {e}")
            time.sleep(0.05)

    def alive(self):
        return time.time() - self.t_ok < 1.5

    def stage_idx(self):
        return int(self.st.get("stage_idx", -1)) if self.alive() else -1


grabmon = GrabMonitor()


# =====================================================================
# 데이터셋 / 에피소드
# =====================================================================
def ds_path(name):
    if not NAME_RE.match(name or ""):
        raise HTTPException(400, f"이름은 영문·숫자·_.- 1~64자: {name!r}")
    return os.path.join(DS_DIR, name)


def ensure_dataset(name):
    p = ds_path(name)
    os.makedirs(os.path.join(p, "episodes"), exist_ok=True)
    info = os.path.join(p, "info.json")
    if not os.path.exists(info):
        with open(info, "w", encoding="utf-8") as f:
            json.dump({"name": name, "created": time.strftime("%Y-%m-%d %H:%M:%S"), "robot": robot_env.ROBOT,
                       "fps": FPS, "joint_names": rt.JOINT_NAMES, "format": "h2-episodes-v1"},
                      f, ensure_ascii=False, indent=1)
    return p


def next_ep_id(p):
    ids = [int(m.group(1)) for n in os.listdir(os.path.join(p, "episodes"))
           if (m := re.match(r"^ep_(\d{6})$", n))]
    return f"ep_{(max(ids) + 1 if ids else 1):06d}"


def list_episodes(name):
    p = os.path.join(ds_path(name), "episodes")
    out = []
    if not os.path.isdir(p):
        return out
    for n in sorted(os.listdir(p)):
        mp = os.path.join(p, n, "meta.json")
        if not os.path.exists(mp):
            continue
        try:
            with open(mp, encoding="utf-8") as f:
                m = json.load(f)
            m["size_mb"] = round(os.path.getsize(os.path.join(p, n, "episode.npz")) / 1e6, 2)
            out.append(m)
        except Exception:   # noqa: BLE001 — 쓰는 중이거나 깨진 것
            continue
    return out


def list_datasets():
    out = []
    for n in sorted(os.listdir(DS_DIR)):
        if not os.path.isdir(os.path.join(DS_DIR, n)) or not NAME_RE.match(n):
            continue
        eps = list_episodes(n)
        out.append({"name": n, "episodes": len(eps), "frames": sum(e.get("n", 0) for e in eps),
                    "minutes": round(sum(e.get("duration", 0) for e in eps) / 60, 1),
                    "size_mb": round(sum(e.get("size_mb", 0) for e in eps), 1)})
    return out


class Recorder:
    """30 Hz 기록. start() → 매 틱 (이미지·실측·지령·손 위치·단계) → stop() 이 파일로 씀 (별도 스레드)."""

    def __init__(self):
        self.lk = threading.Lock()
        self.ep = None          # 기록 중 에피소드 (dict)
        self.last = None        # 마지막 저장 결과 (UI 표시)
        # 첫 호출이 느린 것들(JPEG 코덱·FK) 미리 — 안 하면 첫 에피소드 첫 틱이 0.3 s 늦음
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.imdecode(cv2.imencode(".jpg", cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA))[1], cv2.IMREAD_COLOR)
        fk.hands(np.zeros(14))
        threading.Thread(target=self._run, daemon=True).start()

    def active(self):
        return self.ep is not None

    def start(self, source, extra=None):
        with self.lk:
            if self.ep is not None:
                return None
            if watch is None:
                log("기록 안 함 — DDS 없음")
                return None
            w, h = [int(v) for v in SET["record"]["image"].split("x")]
            p = ensure_dataset(SET["dataset"])
            self.ep = {"id": next_ep_id(p), "dataset": SET["dataset"], "path": p, "source": source,
                       "t0": time.time(), "w": w, "h": h, "until": SET["record"]["until"],
                       "capturing": True, "lift_seen": False, "past_lift": False, "gap": 0.0,
                       "t": [], "state": [], "action": [], "ee": [], "stage": [], "weight": [],
                       "jpeg": [], "img_t": [], "extra": dict(extra or {}), "last_cam": -1, "abort": ""}
            log(f"기록 시작 {self.ep['dataset']}/{self.ep['id']} ({source})")
            return self.ep["id"]

    def _tick(self, ep):
        now = time.time()
        jpg, t_img, n_img = cam.latest()
        snap = watch.snapshot()
        # 잠깐 끊김은 그 틱만 건너뜀 (t 는 실제 시각으로 남음), 누적 1 초 넘으면 그 에피소드는 버림
        why = ("카메라" if jpg is None or now - t_img > 0.5 else
               "rt/lowstate·rt/arm_sdk" if now - snap["t_state"] > 0.2 or now - snap["t_cmd"] > 0.2 else "")
        if why:
            ep["gap"] += 1.0 / FPS
            if ep["gap"] > 1.0:
                ep["abort"] = f"{why} 끊김 누적 1초 넘음"
            return
        if n_img != ep["last_cam"]:          # 새 프레임만 변환 (카메라가 30 fps 보다 느리면 같은 프레임 재사용)
            if (ep["w"], ep["h"]) != (640, 480):   # 원본(640x480)이면 받은 JPEG 그대로 — 재압축 없음
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    return
                if (img.shape[1], img.shape[0]) != (ep["w"], ep["h"]):
                    img = cv2.resize(img, (ep["w"], ep["h"]), interpolation=cv2.INTER_AREA)
                jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
            ep["cur_jpg"], ep["last_cam"] = jpg, n_img
        si = grabmon.stage_idx()
        ep["t"].append(now - ep["t0"])
        ep["state"].append(snap["q"])
        ep["action"].append(snap["cmd_q"])
        ep["ee"].append(fk.hands(snap["q"]))
        ep["stage"].append(si)
        ep["weight"].append(snap["weight"])
        ep["jpeg"].append(ep["cur_jpg"])
        ep["img_t"].append(t_img - ep["t0"])

    def _run(self):
        period = 1.0 / FPS
        t_next = time.perf_counter()
        while True:
            t_next += period
            ep = self.ep
            if ep is not None and ep["capturing"]:
                try:
                    self._tick(ep)
                except Exception as e:   # noqa: BLE001
                    ep["abort"] = f"기록 오류: {e}"
                # 들기까지만: 들기 단계를 지나면 기록 멈춤 (에피소드는 잡기가 끝날 때 닫음)
                if ep["until"] == "lift" and STAGE_NAMES and LIFT_STAGE in STAGE_NAMES:
                    li = STAGE_NAMES.index(LIFT_STAGE)
                    si = ep["stage"][-1] if ep["stage"] else -1
                    if si == li:
                        ep["lift_seen"] = True
                    elif ep["lift_seen"] and si > li:
                        ep["capturing"], ep["past_lift"] = False, True
                if time.time() - ep["t0"] > MAX_EP_SEC:
                    ep["abort"] = f"{MAX_EP_SEC:.0f}초 넘음"
                if ep["abort"]:
                    self.stop(ok=False, why=ep["abort"])
            dt = t_next - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.perf_counter()

    def stop(self, ok=True, why=""):
        with self.lk:
            ep, self.ep = self.ep, None
        if ep is None:
            return None
        n = len(ep["t"])
        keep = n >= FPS and (ok or SET["record"]["save_failed"])
        res = {"id": ep["id"], "dataset": ep["dataset"], "n": n, "ok": bool(ok), "saved": keep,
               "why": why, "duration": round(ep["t"][-1], 2) if n else 0.0, "t": time.time()}
        self.last = res
        log(f"기록 끝 {ep['id']} — {n} 프레임, ok={ok}, {'저장' if keep else '버림'} {why}")
        if keep:
            threading.Thread(target=self._write, args=(ep, res), daemon=True).start()
        return res

    @staticmethod
    def _write(ep, res):
        d = os.path.join(ep["path"], "episodes", ep["id"])
        tmp = d + ".tmp"
        os.makedirs(tmp, exist_ok=True)
        jp = ep["jpeg"]
        off = np.zeros(len(jp) + 1, dtype=np.int64)
        off[1:] = np.cumsum([len(b) for b in jp])
        np.savez(os.path.join(tmp, "episode.npz"),
                 t=np.asarray(ep["t"], dtype=np.float64),
                 state=np.asarray(ep["state"], dtype=np.float32),
                 action=np.asarray(ep["action"], dtype=np.float32),
                 ee=np.asarray(ep["ee"], dtype=np.float32),
                 stage=np.asarray(ep["stage"], dtype=np.int8),
                 weight=np.asarray(ep["weight"], dtype=np.float32),
                 img_t=np.asarray(ep["img_t"], dtype=np.float64),
                 jpeg=np.frombuffer(b"".join(jp), dtype=np.uint8),
                 jpeg_off=off)
        with open(os.path.join(tmp, "thumb.jpg"), "wb") as f:
            f.write(jp[0])
        meta = {"format": "h2-episodes-v1", "id": ep["id"], "created": time.strftime("%Y-%m-%d %H:%M:%S",
                time.localtime(ep["t0"])), "fps": FPS, "n": res["n"], "duration": res["duration"], "ok": res["ok"],
                "why": res["why"], "source": ep["source"], "until": ep["until"],
                "image": {"w": ep["w"], "h": ep["h"]}, "joint_names": rt.JOINT_NAMES, "stage_names": STAGE_NAMES,
                "skipped_sec": round(ep["gap"], 2),
                "robot": robot_env.ROBOT, "sim": SIM, "camera_url": CAMERA_URL,
                "camera_pitch_deg": robot_env.CAMERA_PITCH_DEG, **ep["extra"]}
        with open(os.path.join(tmp, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        os.replace(tmp, d)


rec = Recorder()


def _hist_ok(t0):
    """robot_server /grab_history 에서 t0 이후 끝난 잡기의 ok."""
    code, d = try_json(f"{ROBOT}/grab_history", timeout=1.0)
    if code != 200:
        return False, "grab_history 없음"
    for h in d.get("history", []):
        if float(h.get("end", 0)) >= t0 - 1.0:
            return bool(h.get("ok")), "" if h.get("ok") else f"마지막 단계 {h['stages'][-1][0] if h.get('stages') else '?'}"
    return False, "grab_history 에 기록 없음"


def _on_grab_rise(st):
    if policy.running():
        return
    if SET["record"]["link"] or auto.running():
        extra = {"handover": None}
        if SIM_VIRTUAL:
            code, d = try_json(f"{DETECT}/sim/state", timeout=0.5)
            if code == 200:
                extra["sim_box"] = d.get("box")
        rec.start("auto" if auto.running() else "grab", extra)


def _on_grab_fall(st):
    ep = rec.ep
    if ep is None or ep["source"] not in ("grab", "auto"):
        return
    time.sleep(0.3)      # robot_server end_log 가 history 에 쓰는 시간
    if st.get("lost"):
        rec.stop(ok=False, why="robot_server 응답 없음")
        return
    ok, why = _hist_ok(ep["t0"])
    if ep["until"] == "lift" and ep["past_lift"]:
        ok, why = True, ""
    rec.stop(ok=ok, why=why)


grabmon.on_rise.append(_on_grab_rise)
grabmon.on_fall.append(_on_grab_fall)


# =====================================================================
# 자동 반복 수집
# =====================================================================
class AutoCollect:
    def __init__(self):
        self.th = None
        self.stop_ev = threading.Event()
        self.st = {"running": False, "i": 0, "n": 0, "ok": 0, "fail": 0, "phase": "", "msg": ""}

    def running(self):
        return self.th is not None and self.th.is_alive()

    def start(self):
        if self.running():
            raise HTTPException(409, "이미 자동 수집 중")
        if policy.running():
            raise HTTPException(409, "정책 실행 중")
        if rec.active():
            raise HTTPException(409, "기록 중 — 먼저 정지")
        self.stop_ev.clear()
        self.th = threading.Thread(target=self._run, args=(dict(SET["auto"]),), daemon=True)
        self.th.start()

    def stop(self):
        self.stop_ev.set()

    def _phase(self, p, msg=""):
        self.st["phase"], self.st["msg"] = p, msg
        log(f"[auto] {p} {msg}")

    def _wait(self, sec, label):
        t_end = time.time() + sec
        while time.time() < t_end:
            if self.stop_ev.is_set():
                return False
            self._phase(label, f"{t_end - time.time():.0f}초")
            time.sleep(0.25)
        return True

    def _sim_box(self, cfg):
        rng = np.random.default_rng()
        for _ in range(20):
            x = float(rng.uniform(*cfg["sim_x"]))
            y = float(rng.uniform(*cfg["sim_y"]))
            try_json(f"{DETECT}/sim/box", {"x": round(x, 3), "y": round(y, 3), "present": True})
            time.sleep(0.15)
            code, d = try_json(f"{DETECT}/pose", timeout=0.5)
            if code == 200 and d.get("found"):
                return {"x": round(x, 3), "y": round(y, 3)}
        return None

    def _run(self, cfg):
        n = int(cfg["count"])
        self.st.update(running=True, i=0, n=n, ok=0, fail=0)
        try:
            for i in range(n):
                if self.stop_ev.is_set():
                    break
                self.st["i"] = i + 1
                if i > 0 and not self._wait(float(cfg["pause_s"]), "박스 위치 바꾸세요"):
                    break
                if SIM_VIRTUAL and cfg.get("sim_random"):
                    if self._sim_box(cfg) is None:
                        self._phase("시뮬 박스", "보이는 위치를 못 찾음")
                code, _ = try_json(f"{ROBOT}/set_handover_direction?direction={cfg['handover']}", timeout=1.0)
                # 이전 회 잡기가 아직 돌면 끝까지 기다림 (detect_box 자동 잡기 등)
                while grabmon.busy and not self.stop_ev.is_set():
                    self._phase("이전 잡기 끝나기 기다림")
                    time.sleep(0.3)
                # 매 회 Box 대기 자세부터 — 잡기 끝 '⑪ Home' 은 손목 회전 없는 자세라 그대로 시작하면 1 회차와 시작 자세가 달라짐
                self._phase("Box 대기 자세")
                code, d = try_json(f"{ROBOT}/set_mode?mode=box", {}, timeout=3.0)
                if code != 200 or not d.get("ok"):
                    self._phase("중단", f"set_mode box 실패: {d}")
                    break
                if not self._wait(3.0, "대기 자세 이동"):
                    break
                try_json(f"{DETECT}/reset_window", {}, timeout=1.0)
                t_box = time.time() + float(cfg["wait_box_s"])
                found = False
                while time.time() < t_box and not self.stop_ev.is_set():
                    code, d = try_json(f"{DETECT}/pose", timeout=0.5)
                    if code == 200 and d.get("found") and int(d.get("n", 0)) >= 10:
                        found = True
                        break
                    self._phase("박스 인식 기다림", f"{t_box - time.time():.0f}초")
                    time.sleep(0.3)
                if self.stop_ev.is_set():
                    break
                if not found:
                    self.st["fail"] += 1
                    self._phase("건너뜀", "박스 안 보임")
                    continue
                self._phase("잡기", f"{i + 1}/{n}")
                code, d = try_json(f"{ROBOT}/grab_manual", timeout=3.0)
                t_rise = time.time() + 3.0
                while not grabmon.busy and time.time() < t_rise:
                    time.sleep(0.05)
                if not grabmon.busy:
                    self.st["fail"] += 1
                    self._phase("건너뜀", f"잡기 시작 안 됨: {d.get('reason') or d}")
                    continue
                t_end = time.time() + MAX_EP_SEC
                while grabmon.busy and time.time() < t_end:
                    if self.stop_ev.is_set():
                        try_json(f"{ROBOT}/stop", {}, timeout=3.0)
                        break
                    self._phase("잡기 중", str(grabmon.st.get("stage") or ""))
                    time.sleep(0.2)
                time.sleep(1.0)    # 기록 저장 판정 (_on_grab_fall)
                last = rec.last or {}
                if last.get("ok"):
                    self.st["ok"] += 1
                else:
                    self.st["fail"] += 1
            if cfg.get("mode_off_after"):
                try_json(f"{ROBOT}/set_mode?mode=none", {}, timeout=3.0)
        finally:
            self.st["running"] = False
            self._phase("끝" if not self.stop_ev.is_set() else "중지됨",
                        f"성공 {self.st['ok']} · 실패 {self.st['fail']}")


auto = AutoCollect()


# =====================================================================
# 모델
# =====================================================================
def md_path(name):
    if not NAME_RE.match(name or ""):
        raise HTTPException(400, f"모델 이름은 영문·숫자·_.- 1~64자: {name!r}")
    return os.path.join(MD_DIR, name)


def list_models():
    out = []
    for n in sorted(os.listdir(MD_DIR)):
        p = os.path.join(MD_DIR, n)
        if not os.path.isfile(os.path.join(p, "meta.json")):
            continue
        try:
            m = rt.read_model_meta(p)
        except Exception as e:   # noqa: BLE001
            out.append({"name": n, "error": str(e)})
            continue
        out.append({"name": n, "dataset": m.get("dataset"), "episodes": m.get("episodes"),
                    "frames": m.get("frames"), "steps": m.get("steps"), "loss": m.get("final_loss"),
                    "val_l1": m.get("val_l1"), "created": m.get("created"), "image": m.get("image"),
                    "chunk_size": m.get("chunk_size"), "export_diff": m.get("export_max_abs_diff"),
                    "errors": rt.check_model_meta(m), "warn": ov_warn(m),
                    "size_mb": round(sum(os.path.getsize(os.path.join(p, f)) for f in os.listdir(p)) / 1e6, 1)})
    return out


def _ov_ver(v):
    try:
        return tuple(int(x) for x in str(v).split("-")[0].split(".")[:2])
    except Exception:   # noqa: BLE001
        return None


def ov_warn(meta):
    """학습 PC OpenVINO 가 이 PC 보다 새 버전이면 IR 을 못 읽을 수 있음 → 경고 문자열."""
    try:
        import openvino as ov
        mine, theirs = _ov_ver(ov.__version__), _ov_ver(meta.get("openvino"))
        if mine and theirs and theirs > mine:
            return f"학습 PC OpenVINO {meta.get('openvino')} > 이 PC {ov.__version__} — 로드 실패하면 학습 PC 를 같은 버전으로"
    except Exception:   # noqa: BLE001
        pass
    return ""


def ov_devices():
    try:
        import openvino as ov
        return list(ov.Core().available_devices)
    except Exception as e:   # noqa: BLE001
        return [f"openvino 없음: {e}"]


# =====================================================================
# 정책 실행
# =====================================================================
def _smooth(x):
    return x * x * (3 - 2 * x)


class _Stop(Exception):
    """Stop 버튼 / Esc."""


class PolicyRunner:
    def __init__(self):
        self.pol: Optional[rt.OVPolicy] = None
        self.loading = ""
        self.load_err = ""
        self.th = None
        self.stop_ev = threading.Event()
        self.st = {"phase": "idle", "why": "", "steps": 0}
        self.history = deque(maxlen=20)
        self.safety = None

    def running(self):
        return self.th is not None and self.th.is_alive()

    # ---- 모델 로드 ----
    def load(self, name, device):
        if self.running():
            raise HTTPException(409, "정책 실행 중 — 먼저 정지")
        if self.loading:
            raise HTTPException(409, "다른 모델 로드 중")
        p = md_path(name)
        if not os.path.isfile(os.path.join(p, "policy.xml")):
            raise HTTPException(404, f"모델 없음: {name}")
        self.loading, self.load_err = f"{name} @ {device}", ""

        def _do():
            try:
                pol = rt.OVPolicy(p, device)
                # 첫 추론 (GPU/NPU 커널 준비) — 시간 측정용
                dummy = cv2.imencode(".jpg", np.zeros((pol.H, pol.W, 3), np.uint8))[1].tobytes()
                pol.infer(dummy, np.zeros(14))
                pol.infer(dummy, np.zeros(14))
                self.pol = pol
                pol.name = name
                SET["policy"].update(model=name, device=device)
                save_settings()
                log(f"모델 로드 {name} @ {device} (실행 {pol.exec_devices}, 컴파일 {pol.compile_s:.1f}s, "
                    f"추론 {pol.last_ms:.1f} ms)")
            except Exception as e:   # noqa: BLE001
                self.load_err = f"{type(e).__name__}: {e}"
                log(f"모델 로드 실패: {self.load_err}")
            finally:
                self.loading = ""
        threading.Thread(target=_do, daemon=True).start()

    # ---- 시작 조건 ----
    def preconditions(self):
        """[(이름, ok, 설명)]"""
        out = []
        out.append(("모델", self.pol is not None,
                    f"{self.pol.name} @ {','.join(self.pol.exec_devices)}" if self.pol else (self.load_err or "로드 안 됨")))
        code, a = try_json(f"{ARM}/status", timeout=0.5)
        arm_ok = code == 200 and a.get("ready") and a.get("mode") == "hold" and not a.get("switching")
        arm_free = code == 200 and not a.get("busy") and not (a.get("stream") or {}).get("active")
        out.append(("arm_server hold", bool(arm_ok), f"{a.get('mode', '-')} w={a.get('weight', '-')}" if code == 200
                    else a.get("error", f"HTTP {code}")))
        out.append(("팔 동작 없음", bool(arm_free), "busy" if code == 200 and a.get("busy") else ""))
        g = grabmon.st if grabmon.alive() else {}
        if grabmon.alive():
            out.append(("잡기 모드 OFF", g.get("mode") == "none", f"mode={g.get('mode')}"))
            out.append(("잡기/모션 안 도는 중", not g.get("busy") and not g.get("is_running"),
                        f"busy={g.get('busy')} run={g.get('is_running')}"))
        else:
            out.append(("robot_server", True, "응답 없음 — 충돌할 잡기 없음"))
        out.append(("카메라", cam.age() < 0.5, f"{cam.fps:.0f} fps" if cam.age() < 0.5 else (cam.err or "프레임 없음")))
        if watch is None:
            out.append(("rt/lowstate", False, watch_err))
        else:
            s = watch.snapshot()
            out.append(("rt/lowstate", time.time() - s["t_state"] < 0.2, f"{time.time() - s['t_state']:.2f}s 전"))
        out.append(("수집 안 하는 중", not rec.active() and not auto.running(), ""))
        return out

    def start(self):
        if self.running():
            raise HTTPException(409, "이미 실행 중")
        bad = [f"{n}: {d}" for n, ok, d in self.preconditions() if not ok]
        if bad:
            raise HTTPException(409, "시작 조건 안 맞음 — " + " / ".join(bad))
        self.stop_ev.clear()
        self.th = threading.Thread(target=self._run, args=(self.pol, dict(SET["policy"]), dict(SET["safety"])),
                                   daemon=True)
        self.th.start()

    def stop(self, why="Stop"):
        if self.running():
            self.st["why"] = why
            self.stop_ev.set()

    # ---- 실행 루프 ----
    def _cmd(self, tok, q):
        code, d = try_json(f"{ARM}/stream/cmd", {"token": tok, "deg": np.degrees(q).round(4).tolist()}, timeout=0.3)
        if code != 200:
            raise RuntimeError(f"arm_server /stream/cmd {code}: {d.get('detail') or d.get('error') or d}")
        return d

    def _run(self, pol, pcfg, scfg):
        meta = pol.meta
        safety = rt.Safety(meta, fk, scfg)
        self.safety = safety
        start_q = np.asarray(meta["start_pose"], dtype=float)
        st = {"phase": "start", "why": "", "steps": 0, "t0": time.time(), "model": pol.name, "infer_ms": 0.0,
              "infer_ms_avg": 0.0, "hz": 0.0, "n_vel_clip": 0, "n_clip": 0, "n_reject": 0, "reject_why": "",
              "target_deg": None, "meas_deg": None, "mode": pcfg["mode"]}
        self.st = st
        tok = None
        why = ""
        try:
            code, d = try_json(f"{ARM}/stream/begin", {"max_vel": float(safety.cfg["max_vel"])}, timeout=1.0)
            if code != 200:
                raise RuntimeError(f"stream/begin {code}: {d.get('detail') or d}")
            tok = d["token"]
            q_cur = np.radians(d["target_deg"])
            # 1) 시작 자세로 (데이터셋 첫 프레임 평균, 관절 공간 smoothstep — robot_server park 와 같은 경로를 반대로)
            dist = float(np.max(np.abs(start_q - q_cur)))
            T = float(np.clip(dist / 0.4, 1.5, 5.0)) if dist > np.radians(2) else 0.3
            st["phase"] = "시작 자세로"
            n = int(T * FPS)
            t_next = time.perf_counter()
            for k in range(1, n + 1):
                if self.stop_ev.is_set():
                    raise _Stop()
                r_why = safety.check_robot(watch.snapshot(), None, time.time())   # 온도·토크·끊김 (추종 오차 제외)
                if r_why:
                    raise RuntimeError(f"시작 자세 이동 중 — {r_why}")
                q = q_cur + _smooth(k / n) * (start_q - q_cur)
                r = self._cmd(tok, q)
                st["target_deg"], st["meas_deg"] = r["target_deg"], r["meas_deg"]
                t_next += 1.0 / FPS
                time.sleep(max(0.0, t_next - time.perf_counter()))
            for _ in range(int(0.5 * FPS)):          # 멈춘 뒤 0.5 초 안정
                if self.stop_ev.is_set():
                    raise _Stop()
                r = self._cmd(tok, start_q)
                time.sleep(1.0 / FPS)
            prev = np.radians(r["target_deg"])
            # 2) 정책
            st["phase"] = "정책 실행"
            ens = rt.TemporalEnsembler(pol.chunk, float(pcfg["coeff"]))
            queue = []
            n_steps = max(1, min(int(pcfg["n_action_steps"]), pol.chunk))
            t_run0 = time.time()
            t_next = time.perf_counter()
            t_last = time.perf_counter()
            ms_sum = 0.0
            while True:
                if self.stop_ev.is_set():
                    raise _Stop()
                now = time.time()
                if now - t_run0 > safety.cfg["max_run_s"]:
                    why = f"최대 시간 {safety.cfg['max_run_s']:.0f}s"
                    break
                jpg, t_img, _ = cam.latest()
                if jpg is None or now - t_img > safety.cfg["frame_stale_s"]:
                    why = f"카메라 끊김 ({now - t_img:.2f}s)"
                    break
                snap = watch.snapshot()
                r_why = safety.check_robot(snap, prev, now)
                if r_why:
                    why = r_why
                    break
                g = grabmon.st if grabmon.alive() else {}
                if g.get("busy") or g.get("is_running") or g.get("mode") not in (None, "none"):
                    why = f"robot_server 동작 시작됨 (mode={g.get('mode')}, busy={g.get('busy')})"
                    break
                if pcfg["mode"] == "ensemble":
                    a = ens.update(pol.infer(jpg, snap["q"]))
                    ms_sum += pol.last_ms
                else:
                    if not queue:
                        queue = list(pol.infer(jpg, snap["q"])[:n_steps])
                        ms_sum += pol.last_ms
                    a = queue.pop(0)
                q, s_why = safety.filter(a, prev)
                if s_why:
                    why = s_why
                    break
                r = self._cmd(tok, q)
                prev = np.radians(r["target_deg"])
                st["steps"] += 1
                st["n_vel_clip"] += int(r.get("vel_clip", False))
                st.update(target_deg=r["target_deg"], meas_deg=r["meas_deg"], infer_ms=round(pol.last_ms, 1),
                          infer_ms_avg=round(ms_sum / st["steps"], 1), n_clip=safety.n_clip,
                          n_reject=safety.n_reject, reject_why=safety.last_reason,
                          run_s=round(time.time() - t_run0, 1))
                tp = time.perf_counter()
                gap = tp - t_last                     # 지령 사이 간격 (30 Hz 면 0.033 s)
                st["hz"] = round(0.9 * st["hz"] + 0.1 / max(gap, 1e-3), 1)
                t_last = tp
                if gap > 0.3:
                    why = f"루프 지연 {gap:.2f}s (추론 {pol.last_ms:.0f} ms) — 장치를 GPU 로 바꾸거나 청크 방식으로"
                    break
                t_next = max(t_next + 1.0 / FPS, tp)  # 늦으면 밀린 만큼 쌓지 않음 (그만큼 Hz 가 떨어져 보임)
                time.sleep(max(0.0, t_next - time.perf_counter()))
        except _Stop:
            why = self.st.get("why") or "Stop"
        except Exception as e:   # noqa: BLE001
            why = f"오류: {e}"
        finally:
            if tok:
                try_json(f"{ARM}/stream/end", {"token": tok}, timeout=1.0)
            st["phase"] = "정지"
            st["why"] = why
            self.history.appendleft({"t": time.strftime("%H:%M:%S", time.localtime(st["t0"])), "model": pol.name,
                                     "dur": round(time.time() - st["t0"], 1), "steps": st["steps"], "why": why,
                                     "n_clip": st["n_clip"], "n_reject": st["n_reject"], "n_vel_clip": st["n_vel_clip"]})
            log(f"정책 정지 — {why} ({st['steps']} 스텝)")


policy = PolicyRunner()


# =====================================================================
# FastAPI
# =====================================================================
app = FastAPI(title="Physical AI — policy server")


@app.get("/", response_class=HTMLResponse)
def index():
    with open(os.path.join(HERE, "policy_web.html"), encoding="utf-8") as f:
        return f.read()


@app.get("/video_feed")
def video_feed():
    """기록·추론에 쓰는 바로 그 카메라 프레임 (15 fps 로 다시 보냄)."""
    def gen():
        last = -1
        while True:
            jpg, _, n = cam.wait_new(last, timeout=1.0)
            if jpg is not None and n != last:
                last = n
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            time.sleep(1 / 15)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


def _servers():
    code, a = try_json(f"{ARM}/status", timeout=0.3)
    s = watch.snapshot() if watch else None
    now = time.time()
    return {
        "arm": {"ok": code == 200 and bool(a.get("ready")), "mode": a.get("mode"), "weight": a.get("weight"),
                "busy": a.get("busy"), "stream": a.get("stream"), "err": a.get("error") if code != 200 else ""},
        "robot": {"ok": grabmon.alive(), "mode": grabmon.st.get("mode"), "busy": grabmon.st.get("busy"),
                  "is_running": grabmon.st.get("is_running"), "stage": grabmon.st.get("stage"),
                  "stage_idx": grabmon.st.get("stage_idx", -1), "stages": STAGE_NAMES, "err": grabmon.err},
        "camera": {"ok": cam.age() < 0.5, "fps": round(cam.fps, 1), "url": CAMERA_URL, "err": cam.err,
                   "age": round(min(cam.age(), 99), 2)},
        "dds": {"ok": bool(s) and now - s["t_state"] < 0.2, "state_age": round(now - s["t_state"], 3) if s else None,
                "cmd_age": round(now - s["t_cmd"], 3) if s else None, "weight": round(s["weight"], 2) if s else None,
                "temp_max": float(s["temp"].max()) if s else None, "err": watch_err},
        "sim": SIM, "sim_virtual": SIM_VIRTUAL, "robot_name": robot_env.ROBOT,
    }


@app.get("/api/state")
def api_state():
    ep = rec.ep
    rec_st = None
    if ep is not None:
        rec_st = {"id": ep["id"], "source": ep["source"], "frames": len(ep["t"]),
                  "sec": round(time.time() - ep["t0"], 1), "capturing": ep["capturing"]}
    pst = dict(policy.st)
    pst.pop("t0", None)
    sf = policy.safety
    return {"servers": _servers(), "settings": SET, "record": rec_st, "last_record": rec.last,
            "auto": auto.st, "auto_running": auto.running(),
            "policy": {"running": policy.running(), **pst, "loading": policy.loading, "load_err": policy.load_err,
                       "model": policy.pol.name if policy.pol else None,
                       "exec": policy.pol.exec_devices if policy.pol else [],
                       "range_deg": [np.degrees(sf.lo).round(1).tolist(), np.degrees(sf.hi).round(1).tolist()]
                       if sf else None,
                       "history": list(policy.history)},
            "joint_names": rt.JOINT_NAMES}


@app.get("/api/preconditions")
def api_pre():
    return [{"name": n, "ok": bool(ok), "detail": d} for n, ok, d in policy.preconditions()]


@app.post("/api/settings")
def api_settings(body: dict):
    for sec, vals in body.items():
        if sec == "dataset":
            ensure_dataset(vals)
            SET["dataset"] = vals
        elif sec in SET and isinstance(SET[sec], dict) and isinstance(vals, dict):
            for k, v in vals.items():
                if k in SET[sec]:
                    SET[sec][k] = v
    if SET["record"]["image"] not in ("640x480", "320x240"):
        SET["record"]["image"] = "320x240"
    if SET["record"]["until"] not in ("all", "lift"):
        SET["record"]["until"] = "all"
    save_settings()
    return {"ok": True, "settings": SET}


# ---- 데이터셋 ----
@app.get("/api/datasets")
def api_datasets():
    return {"current": SET["dataset"], "datasets": list_datasets()}


@app.get("/api/datasets/{name}/episodes")
def api_episodes(name: str):
    return list_episodes(name)


@app.delete("/api/datasets/{name}/episodes/{ep}")
def api_ep_delete(name: str, ep: str):
    if not re.match(r"^ep_\d{6}$", ep):
        raise HTTPException(400, "에피소드 id")
    p = os.path.join(ds_path(name), "episodes", ep)
    if not os.path.isdir(p):
        raise HTTPException(404, "없음")
    shutil.rmtree(p)
    log(f"에피소드 삭제 {name}/{ep}")
    return {"ok": True}


@app.delete("/api/datasets/{name}")
def api_ds_delete(name: str):
    p = ds_path(name)
    if not os.path.isdir(p):
        raise HTTPException(404, "없음")
    if rec.active() and rec.ep["dataset"] == name:
        raise HTTPException(409, "기록 중인 데이터셋")
    shutil.rmtree(p)
    log(f"데이터셋 삭제 {name}")
    return {"ok": True}


@app.get("/api/datasets/{name}/episodes/{ep}/thumb.jpg")
def api_thumb(name: str, ep: str):
    if not re.match(r"^ep_\d{6}$", ep):
        raise HTTPException(400, "에피소드 id")
    p = os.path.join(ds_path(name), "episodes", ep, "thumb.jpg")
    if not os.path.exists(p):
        raise HTTPException(404, "없음")
    return FileResponse(p, media_type="image/jpeg")


@app.get("/api/datasets/{name}/zip")
def api_zip(name: str):
    p = ds_path(name)
    if not os.path.isdir(p):
        raise HTTPException(404, "없음")
    eps = list_episodes(name)
    if not eps:
        raise HTTPException(409, "에피소드 없음")
    out = os.path.join(TMP_DIR, f"{name}_{int(time.time())}.zip")
    with zipfile.ZipFile(out, "w", zipfile.ZIP_STORED, allowZip64=True) as z:   # npz·jpg 는 이미 압축됨
        z.write(os.path.join(p, "info.json"), f"{name}/info.json")
        for e in eps:
            for fn in ("episode.npz", "meta.json", "thumb.jpg"):
                fp = os.path.join(p, "episodes", e["id"], fn)
                if os.path.exists(fp):
                    z.write(fp, f"{name}/episodes/{e['id']}/{fn}")
    return FileResponse(out, media_type="application/zip", filename=f"{name}.zip",
                        background=BackgroundTask(os.remove, out))


# ---- 기록 ----
@app.post("/api/record/start")
def api_rec_start():
    if policy.running() or auto.running():
        raise HTTPException(409, "정책 실행 / 자동 수집 중")
    if watch is None:
        raise HTTPException(503, watch_err)
    i = rec.start("manual")
    if i is None:
        raise HTTPException(409, "이미 기록 중")
    return {"ok": True, "id": i}


@app.post("/api/record/stop")
def api_rec_stop(ok: bool = True):
    r = rec.stop(ok=ok, why="" if ok else "수동 실패 표시")
    return {"ok": True, "result": r}


# ---- 자동 수집 ----
@app.post("/api/auto/start")
def api_auto_start():
    if watch is None:
        raise HTTPException(503, watch_err)
    if not grabmon.alive():
        raise HTTPException(503, "robot_server 응답 없음")
    auto.start()
    return {"ok": True}


@app.post("/api/auto/stop")
def api_auto_stop():
    auto.stop()
    return {"ok": True}


# ---- 모델 ----
@app.get("/api/models")
def api_models():
    return {"models": list_models(), "devices": ov_devices(), "current": policy.pol.name if policy.pol else None}


@app.post("/api/models")
def api_model_upload(file: UploadFile = File(...)):
    tmp = os.path.join(TMP_DIR, f"up_{int(time.time() * 1000)}.zip")
    with open(tmp, "wb") as f:
        shutil.copyfileobj(file.file, f, 1 << 20)
    try:
        with zipfile.ZipFile(tmp) as z:
            names = [i.filename for i in z.infolist() if not i.is_dir()]
            for n in names:
                if n.startswith("/") or ".." in n.replace("\\", "/").split("/"):
                    raise HTTPException(400, f"zip 안 경로 이상: {n}")
            metas = [n for n in names if n.endswith("meta.json")]
            if len(metas) != 1:
                raise HTTPException(400, "zip 안에 meta.json 이 하나 있어야 함 (trainer 가 만든 model zip)")
            base = metas[0][:-len("meta.json")]
            for need in ("policy.xml", "policy.bin"):
                if base + need not in names:
                    raise HTTPException(400, f"zip 안에 {need} 없음")
            meta = json.loads(z.read(metas[0]))
            errs = rt.check_model_meta(meta)
            if errs:
                raise HTTPException(400, "이 로봇과 안 맞는 모델: " + "; ".join(errs))
            stem = re.sub(r"[^A-Za-z0-9_.\-]", "_", str(meta.get("name") or os.path.splitext(file.filename)[0]))[:56]
            name, k = stem, 2
            while os.path.exists(os.path.join(MD_DIR, name)):
                name, k = f"{stem}_{k}", k + 1
            dst = os.path.join(MD_DIR, name)
            os.makedirs(dst)
            for n in names:
                if n.startswith(base) and "/" not in n[len(base):]:
                    with z.open(n) as src, open(os.path.join(dst, n[len(base):]), "wb") as out:
                        shutil.copyfileobj(src, out, 1 << 20)
    except zipfile.BadZipFile:
        raise HTTPException(400, "zip 아님")
    finally:
        os.remove(tmp)
    log(f"모델 업로드 {name}")
    return {"ok": True, "name": name}


@app.delete("/api/models/{name}")
def api_model_delete(name: str):
    p = md_path(name)
    if not os.path.isdir(p):
        raise HTTPException(404, "없음")
    if policy.pol and policy.pol.name == name:
        if policy.running():
            raise HTTPException(409, "실행 중인 모델")
        policy.pol = None
    shutil.rmtree(p)
    return {"ok": True}


@app.post("/api/models/{name}/load")
def api_model_load(name: str, device: str = "CPU"):
    policy.load(name, device)
    return {"ok": True}


# ---- 정책 ----
@app.post("/api/policy/start")
def api_policy_start():
    policy.start()
    return {"ok": True}


@app.post("/api/policy/stop")
def api_policy_stop():
    policy.stop("Stop 버튼")
    return {"ok": True}


def _autoload():
    name, dev = SET["policy"].get("model"), SET["policy"].get("device", "CPU")
    if name and os.path.isfile(os.path.join(MD_DIR, name, "policy.xml")):
        try:
            policy.load(name, dev)
        except Exception as e:   # noqa: BLE001
            log(f"모델 자동 로드 실패: {e}")


if __name__ == "__main__":
    ensure_dataset(SET["dataset"])
    _autoload()
    log(f"시작 — http://localhost:{PORT}/  카메라 {CAMERA_URL}  데이터 {DATA}  (ROBOT={robot_env.ROBOT}"
        f"{', SIM' if SIM else ''})")
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
