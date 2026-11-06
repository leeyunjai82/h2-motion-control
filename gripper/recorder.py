"""
recorder.py — 그리퍼 잡기 웹(pick_web)의 잡기 1 회 = ACT 에피소드 1 개 녹화 (30 Hz)

  <data>/<dataset>/info.json                        format, cameras{이름: {w,h}}, state_names, fps, 만든 때
  <data>/<dataset>/episodes/ep_000001/episode.npz   t(N,) · state(N,16) · action(N,16) · tcp(N,6) · stage(N,) ·
                                                    jpeg_<카메라>(M,) uint8 (JPEG 이어 붙임) · jpeg_off_<카메라>(N+1,)
  <data>/<dataset>/episodes/ep_000001/meta.json     format, ok, result, side, fps, n, cameras, state_names, stages, plan, close

  state  = [왼팔 7 실측 rad, 왼 그리퍼 q 실측, 오른팔 7 실측, 오른 그리퍼 q 실측]   (한 손 학습은 학습기에서 골라 씀)
  action = [왼팔 7 지령,      왼 그리퍼 목표,   오른팔 7 지령,  오른 그리퍼 목표]   (지령 = arm_server /stream 으로 보낸 값)
  tcp    = 왼 · 오른 집게 가운데 (실측 관절 FK, pelvis) — 학습 안 씀, 안전 필터 · 분석용
  카메라 = body (D435i 컬러, 반 해상도) + 손목 카메라 (gripper.yaml wrist_cams, 있는 것만). 데이터셋 안에서는 같은 구성만 허용
  npz 는 pickle 없이 (학습기가 allow_pickle=False 로 엶)
"""
import datetime
import json
import os
import re
import threading
import time
import zipfile

import cv2
import numpy as np

EP_FORMAT = "h2-gripper-episodes-v1"
FPS = 30.0
JPEG_Q = 90
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")   # .gitignore
NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,40}$")


# ---------------- 손목 카메라 (USB UVC, OpenCV) ----------------
SIZES = ((320, 240), (640, 480), (800, 600), (640, 360))   # 손목 카메라 시도 순서 (size 를 주면 그 크기 먼저) — 그 크기 · 25 fps 이상으로 연 첫 값


class WristCams:
    """gripper.yaml wrist_cams {이름: 장치 경로 또는 번호} — 각 카메라를 스레드로 계속 읽어 최신 프레임만 둠.
    못 연 카메라는 빠짐 (err 에 이유). MJPEG 로 SIZES 를 차례로 시도 — 장치가 지원 안 하는 크기를 받으면 가장 가까운
    크기(이 PC 의 USB_Camera 는 3264×2448 15 fps 일 수 있음)로 열려서, 실제로 열린 크기 · fps 를 보고 고름."""
    def __init__(self, cfg, fps=30, size=None):
        self.lk = threading.Lock()
        self.frames, self.caps, self.err, self.size = {}, {}, {}, {}
        for name, dev in (cfg or {}).items():
            if dev in (None, ""):
                continue
            cap = cv2.VideoCapture(int(dev) if str(dev).isdigit() else str(dev), cv2.CAP_V4L2)
            if not cap.isOpened():
                self.err[name] = f"못 엶: {dev}"
                continue
            chosen = None
            for w, h in ([tuple(size)] if size else []) + [sz for sz in SIZES if not size or sz != tuple(size)]:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                cap.set(cv2.CAP_PROP_FPS, fps)
                gw, gh, gf = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), cap.get(cv2.CAP_PROP_FPS)
                if (gw, gh) == (w, h) and gf >= 25:
                    chosen = (gw, gh, gf)
                    break
            if chosen is None:
                self.err[name] = f"{dev}: {SIZES} 중 25 fps 로 열리는 크기 없음 (지금 {gw}×{gh} {gf:.0f} fps)"
                cap.release()
                continue
            print(f"[wrist] {name}: {chosen[0]}×{chosen[1]} {chosen[2]:.0f} fps MJPG ({dev})")
            self.caps[name] = cap
            self.size[name] = (chosen[0], chosen[1])
            threading.Thread(target=self._loop, args=(name, cap), daemon=True).start()

    def _loop(self, name, cap):
        while True:
            ok, img = cap.read()
            if ok:
                with self.lk:
                    self.frames[name] = (img, time.time())
            else:
                self.err[name] = "읽기 실패"
                time.sleep(0.2)

    def names(self):
        return sorted(self.caps)

    def frame(self, name):
        with self.lk:
            f = self.frames.get(name)
        return (None, 0.0) if f is None else (f[0].copy(), f[1])


class FakeWristCams:
    """--sim: 가짜 손목 카메라 (번호가 바뀌는 회색 화면)."""
    def __init__(self, names=("wrist_left", "wrist_right")):
        self._names = list(names)
        self.err, self.size, self.n = {}, {n: (640, 480) for n in names}, 0

    def names(self):
        return list(self._names)

    def frame(self, name):
        self.n += 1
        img = np.full((480, 640, 3), 70, np.uint8)
        cv2.putText(img, f"SIM {name} {self.n}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (230, 230, 230), 2)
        return img, time.time()


# ---------------- 녹화 ----------------
class Recorder:
    """start(...) → 30 Hz 로 sample() 을 불러 쌓음 → stop(ok, ...) 에서 저장 (ok 아니어도 저장, meta.ok 로 구분).
    sample() 이 돌려줄 것: dict(state(16), action(16), tcp(6), stage int, images{카메라: BGR})."""
    def __init__(self, data_dir=DATA_DIR):
        self.data_dir = data_dir
        self.lk = threading.Lock()
        self.active = False
        self.th = None
        self.n = 0
        self.last = None          # 마지막 저장 결과 (표시용)

    # ----- 데이터셋 -----
    def ds_dir(self, name):
        if not NAME_RE.match(name or ""):
            raise ValueError("데이터셋 이름은 영문 · 숫자 · _ - 만 (1–40 자)")
        return os.path.join(self.data_dir, name)

    def info(self, name):
        p = os.path.join(self.ds_dir(name), "info.json")
        if not os.path.isfile(p):
            return None
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    def summary(self, name):
        d = os.path.join(self.ds_dir(name), "episodes")
        n_ok = n_all = 0
        if os.path.isdir(d):
            for e in sorted(os.listdir(d)):
                mp = os.path.join(d, e, "meta.json")
                if os.path.isfile(mp):
                    n_all += 1
                    try:
                        with open(mp, encoding="utf-8") as f:
                            n_ok += bool(json.load(f).get("ok"))
                    except (OSError, json.JSONDecodeError):
                        pass
        inf = self.info(name)
        return {"name": name, "episodes": n_all, "ok": n_ok, "cameras": (inf or {}).get("cameras"),
                "size_mb": round(sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(self.ds_dir(name)) for f in fs) / 1e6, 1)
                if os.path.isdir(self.ds_dir(name)) else 0.0}

    def datasets(self):
        if not os.path.isdir(self.data_dir):
            return []
        return [self.summary(n) for n in sorted(os.listdir(self.data_dir)) if NAME_RE.match(n) and os.path.isdir(os.path.join(self.data_dir, n))]

    def check_cameras(self, name, cameras):
        """데이터셋 카메라 구성과 같은지. 새 데이터셋이면 None (처음 에피소드가 정함)."""
        inf = self.info(name)
        if inf is None:
            return None
        want = inf.get("cameras") or {}
        if sorted(want) != sorted(cameras):
            return f"데이터셋 '{name}' 카메라 {sorted(want)} ≠ 지금 {sorted(cameras)} — 다른 데이터셋 이름을 쓰거나 카메라를 맞추세요"
        for k, v in cameras.items():
            if [want[k]["w"], want[k]["h"]] != [v["w"], v["h"]]:
                return f"카메라 {k} 크기 {want[k]['w']}×{want[k]['h']} ≠ 지금 {v['w']}×{v['h']}"
        return None

    # ----- 녹화 -----
    def start(self, name, cameras, state_names, sample_fn, meta):
        """cameras {이름: {w, h}} = 저장 크기. sample_fn() → dict 또는 None (그 틱 건너뜀)."""
        with self.lk:
            if self.active:
                raise RuntimeError("이미 녹화 중")
            err = self.check_cameras(name, cameras)
            if err:
                raise ValueError(err)
            self.active = True
            self.n = 0
        self.buf = {"t": [], "state": [], "action": [], "tcp": [], "stage": [], "jpg": {k: [] for k in cameras}}
        self.name, self.cameras, self.state_names, self.meta0 = name, cameras, list(state_names), dict(meta)
        self.t0 = time.time()
        self.stop_evt = threading.Event()
        self.th = threading.Thread(target=self._loop, args=(sample_fn,), daemon=True)
        self.th.start()

    def _loop(self, sample_fn):
        dt = 1.0 / FPS
        nxt = time.time()
        enc = [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q]
        while not self.stop_evt.is_set():
            nxt += dt
            try:
                s = sample_fn()
            except Exception as e:  # noqa: BLE001
                print(f"[rec] 샘플 오류: {type(e).__name__}: {e}")
                s = None
            if s is not None and all(s["images"].get(k) is not None for k in self.cameras):
                jp = {}
                for k, c in self.cameras.items():
                    img = s["images"][k]
                    if img.shape[1] != c["w"] or img.shape[0] != c["h"]:
                        img = cv2.resize(img, (c["w"], c["h"]), interpolation=cv2.INTER_AREA)
                    ok, b = cv2.imencode(".jpg", img, enc)
                    jp[k] = b.tobytes() if ok else None
                if all(v is not None for v in jp.values()):
                    self.buf["t"].append(time.time() - self.t0)
                    self.buf["state"].append(np.asarray(s["state"], np.float32))
                    self.buf["action"].append(np.asarray(s["action"], np.float32))
                    self.buf["tcp"].append(np.asarray(s["tcp"], np.float32))
                    self.buf["stage"].append(int(s["stage"]))
                    for k in self.cameras:
                        self.buf["jpg"][k].append(jp[k])
                    self.n += 1
            time.sleep(max(0.0, nxt - time.time()))
            if time.time() - nxt > 0.5:       # 많이 밀리면 다시 맞춤 (프레임을 몰아 찍지 않음)
                nxt = time.time()

    def stop(self, ok, result, extra=None, keep=True):
        """녹화 끝 → 저장 (keep False 면 버림). → 저장 정보 dict 또는 None."""
        with self.lk:
            if not self.active:
                return None
            self.active = False
        self.stop_evt.set()
        self.th.join(timeout=3.0)
        n = len(self.buf["t"])
        if not keep or n < 2:
            self.last = {"saved": False, "n": n, "why": "버림" if not keep else "프레임 부족"}
            return self.last
        root = self.ds_dir(self.name)
        ep_root = os.path.join(root, "episodes")
        os.makedirs(ep_root, exist_ok=True)
        if self.info(self.name) is None:
            with open(os.path.join(root, "info.json"), "w", encoding="utf-8") as f:
                json.dump({"format": EP_FORMAT, "fps": FPS, "cameras": self.cameras, "state_names": self.state_names,
                           "created": datetime.datetime.now().isoformat(timespec="seconds")}, f, ensure_ascii=False, indent=1)
        ids = [int(m.group(1)) for m in (re.match(r"ep_(\d+)$", e) for e in os.listdir(ep_root)) if m]
        ep = f"ep_{(max(ids) + 1 if ids else 1):06d}"
        d = os.path.join(ep_root, ep)
        os.makedirs(d)
        arrs = {"t": np.asarray(self.buf["t"], np.float64), "state": np.stack(self.buf["state"]),
                "action": np.stack(self.buf["action"]), "tcp": np.stack(self.buf["tcp"]),
                "stage": np.asarray(self.buf["stage"], np.int16)}
        for k in self.cameras:
            bl = self.buf["jpg"][k]
            off = np.zeros(len(bl) + 1, np.int64)
            off[1:] = np.cumsum([len(b) for b in bl])
            arrs[f"jpeg_{k}"] = np.frombuffer(b"".join(bl), np.uint8)
            arrs[f"jpeg_off_{k}"] = off
        np.savez(os.path.join(d, "episode.npz"), **arrs)
        meta = {"format": EP_FORMAT, "ok": bool(ok), "result": result, "fps": FPS, "n": n,
                "duration_s": round(float(arrs["t"][-1]), 2), "cameras": self.cameras, "state_names": self.state_names,
                "created": datetime.datetime.now().isoformat(timespec="seconds"), **self.meta0, **(extra or {})}
        with open(os.path.join(d, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        self.last = {"saved": True, "episode": ep, "n": n, "ok": bool(ok), "mb": round(sum(a.nbytes for a in arrs.values()) / 1e6, 1)}
        return self.last

    # ----- 내보내기 -----
    def make_zip(self, name, only_ok=False):
        """데이터셋 → zip 파일 (data/<이름>.zip, 메모리에 다 올리지 않음). 학습 PC 웹(trainer)에 올리는 파일. → 경로"""
        root = self.ds_dir(name)
        if not os.path.isdir(os.path.join(root, "episodes")):
            raise FileNotFoundError(name)
        path = os.path.join(self.data_dir, f"{name}{'_ok' if only_ok else ''}.zip")
        tmp = path + ".tmp"
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_STORED) as z:     # npz · jpeg 는 이미 압축됨
            z.write(os.path.join(root, "info.json"), f"{name}/info.json")
            for e in sorted(os.listdir(os.path.join(root, "episodes"))):
                d = os.path.join(root, "episodes", e)
                mp = os.path.join(d, "meta.json")
                if not os.path.isfile(mp):
                    continue
                if only_ok:
                    with open(mp, encoding="utf-8") as f:
                        if not json.load(f).get("ok"):
                            continue
                for fn in ("meta.json", "episode.npz"):
                    z.write(os.path.join(d, fn), f"{name}/episodes/{e}/{fn}")
        os.replace(tmp, path)
        return path
