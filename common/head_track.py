"""
head_track.py — H2 머리 카메라 인식 서버 (포트 50013, 보기 전용)
  왼눈  : 얼굴 + 나이·성별 (robot.yaml head_track.mode, age_gender) — OpenVINO Open Model Zoo face-detection-retail-0004
          + age-gender-recognition-retail-0013 (NPU). mode person / auto 면 person-detection-0201 도
  오른눈: 사람 행동 (robot.yaml head_track.actions) — Open Model Zoo person-detection-action-recognition-0006 (NPU),
          6종을 3개로 합쳐 표시: sit (앉음·쓰기·책상에 엎드림) / raise hand / stand (서 있음·뒤돌아봄) (사용자 2026-10-12)
          actions.enabled false 면 objects (YOLO — POLAR 자세 polar11s / COCO 사물 yolo11s)
  전체 : 두 눈 화면 전체를 작게 나란히 + 인식 박스 (제어 화면 Head Vision 카드 기본, robot.yaml head_track.view.height)
  스트립: 두 눈에서 인식된 것만 잘라 한 줄로 (Head Vision 카드의 CROP 버튼)
  (가슴 D435i 는 박스 인식 — detect_box, 이 서버와 무관)

  ROBOT=h2 python common/head_track.py               # → http://<pc-ip>:50013/   (start_robot.sh 가 H2 에서 띄움)
  ROBOT=h2 python common/head_track.py --no-objects  # 오른눈 인식 끔

  머리 추종(머리 돌리기)은 뺐음 — 실기 2026-10-09: 703 에서 rt/arm_sdk 머리 29/30 명령이 반영 안 됨 (robots/h2/FACTS.md).
  펌웨어가 머리를 arm_sdk 에 넘겨주면 커밋 8dddfe4 의 head_track.py (--drive, /track·/look·/home) 를 되살리면 됨.

  준비: 앱에서 video_hub 끔 + 'Stereo patch PC1' 켬, RGB 수신 IP = 이 PC
        (python utils/check_head_cam.py --set-ip → 앱에서 서비스 재시작). gstreamer 설치는 common/ctrl/head_cam.py 참고.
        check_head_cam.py 와 동시에 켜지 말 것 (같은 UDP 포트)
  설정: robot.yaml head_track (fps 5 — 눈별 인식 횟수, device, objects, strip)
  화면·상태·로그 문자열은 영어 (사용자 2026-10-09)

  API
    GET  /             화면 (스트립 + 왼눈·오른눈 영상 + 상태)
    GET  /pair         MJPEG — 왼눈·오른눈 전체 화면을 높이 view.height 로 줄여 나란히 + 인식 박스, /pair.jpg = 한 장
    GET  /strip        MJPEG — 인식된 것만 잘라 한 줄 (왼눈 초록·주황, 오른눈 파랑), /strip.jpg = 한 장
    GET  /video_feed   MJPEG 왼눈 (얼굴·사람 박스)
    GET  /video_right  MJPEG 오른눈 (행동 / YOLO 박스)
    GET  /detections   JSON 지금 인식 목록
    GET  /status       {left, right} — fps, 인식 ms, 모델, device(설정) · exec(OpenVINO 실제 실행 장치), error
  CORS * (제어 화면 :50000 이 /status·/pair·/strip 을 읽음)
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_env   # noqa: E402  ROBOT 필수
from ctrl.head_cam import PC1, RGB_PORTS, RgbRx, my_ip, web_urls   # noqa: E402
from ctrl.hw_usage import exec_devices, yolo_devices   # noqa: E402
from ctrl.ov_detect import ACTIONS_0006, OvActions, OvAgeGender, OvSSD   # noqa: E402

import uvicorn   # noqa: E402
from fastapi import FastAPI, HTTPException   # noqa: E402
from fastapi.middleware.cors import CORSMiddleware   # noqa: E402
from fastapi.responses import HTMLResponse, Response, StreamingResponse   # noqa: E402

C = dict(robot_env.CFG.get("head_track") or {})
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FRESH_S = 1.0                      # 스트립에 쓸 인식 결과의 최대 나이 [s]


def cfg(key, default):
    v = C.get(key, default)
    return default if v is None else v


def wait_rgb(port, who):
    """머리 카메라 RGB 가 올 때까지 기다렸다가 수신기 반환 (서비스가 나중에 켜져도 됨). who.err 에 상태."""
    while True:
        rx = RgbRx(port)
        if rx.probe(6.0):
            rx.start()
            who.err = ""
            print(f"[head_track] head camera RGB {rx.size[0]}x{rx.size[1]} (UDP {port})")
            return rx
        who.err = (f"no head camera RGB (UDP {port}) - robot side: utils/head_cam_on.py (start_robot.sh runs it; "
                   f"or app: video_hub off, Stereo patch PC1 on), receiver IP = {my_ip(PC1)}")


def jpeg(img, q=80):
    ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return j.tobytes() if ok else None


def put_info(v, text):
    cv2.putText(v, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


class EyeLoop(threading.Thread):
    """한 눈의 수신 → (fps 만큼) 인식 → 그림 반복. 하위 클래스가 process 를 채움."""

    def __init__(self, port, name):
        super().__init__(daemon=True)
        self.port, self.name = port, name
        self.lock = threading.Lock()
        self.jpeg = None
        self.last = None                   # (BGR, 인식 목록, 받은 시각)
        self.fps = 0.0
        self.det_ms = 0.0
        self.err = "waiting for head camera RGB"

    def process(self, img, now):
        raise NotImplementedError

    def run(self):
        rx = wait_rgb(self.port, self)
        period = 1.0 / max(0.5, float(cfg("fps", 5.0)))
        last_t, last_proc, t_fps, n_fps = 0.0, 0.0, time.time(), 0
        while True:
            f = rx.latest()
            now = time.time()
            if f is None or f[1] == last_t or now - last_proc < period:
                if last_t and now - last_t > 1.0:
                    self.err, self.fps = f"head camera RGB lost (UDP {self.port})", 0.0
                time.sleep(0.005)
                continue
            if self.err.startswith("head camera RGB lost"):
                self.err = ""
            img, t = f
            last_t, last_proc = t, now
            t0 = time.time()
            dets, view = self.process(img, now)
            self.det_ms = (time.time() - t0) * 1000
            j = jpeg(view)
            with self.lock:
                self.last = (img, dets, now)
                if j is not None:
                    self.jpeg = j
            n_fps += 1
            if now - t_fps >= 2.0:
                self.fps, t_fps, n_fps = n_fps / (now - t_fps), now, 0


class Faces(EyeLoop):
    """왼눈: 얼굴 (mode auto 면 얼굴이 person_after_s 동안 없을 때 사람)."""

    def __init__(self, port, mode, device, age_gender=False):
        super().__init__(port, "left")
        self.mode, self.device = mode, device
        self.face = OvSSD("face", device, cfg("face_conf", 0.6)) if mode in ("face", "auto") else None
        self.person = OvSSD("person", device, cfg("person_conf", 0.5)) if mode in ("person", "auto") else None
        self.ag = OvAgeGender(device) if age_gender and self.face is not None else None
        self.tracks = []                   # 나이·성별 시간 평균용 [{box, age, male, t}]
        self.last_face = 0.0

    def models(self):
        return [m for m in (self.face, self.person, self.ag) if m is not None]

    def _age_gender(self, img, box, now):
        """얼굴 하나의 (나이, 남자 확률) — 앞 프레임의 같은 얼굴(IoU 0.3 넘음)과 지수 평균 0.7/0.3
        (5 fps 에서 Intel 데모의 30 fps × 0.05 와 비슷하게 1 초 남짓)."""
        r = self.ag(img, box)
        if r is None:
            return None
        age, male = r
        best, best_iou = None, 0.3
        for t in self.tracks:
            iou = box_iou(t["box"], box)
            if iou > best_iou:
                best, best_iou = t, iou
        if best is None:
            best = {"age": age, "male": male}
            self.tracks.append(best)
        else:
            best["age"] = 0.7 * best["age"] + 0.3 * age
            best["male"] = 0.7 * best["male"] + 0.3 * male
        best["box"], best["t"] = box, now
        return best["age"], best["male"]

    def process(self, img, now):
        faces = self.face(img) if self.face is not None else []
        if faces:
            self.last_face = now
        persons = []
        if self.person is not None and (self.mode == "person" or
                                        (not faces and now - self.last_face > float(cfg("person_after_s", 1.0)))):
            persons = self.person(img)
        v = img.copy()
        for b in persons:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 160, 255), 2)
        for b in faces:
            cv2.rectangle(v, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 220, 120), 2)
        put_info(v, f"L  {self.fps:.1f} fps  det {self.det_ms:.0f} ms")
        dets = []
        for b in faces:
            d = dict(label="face", conf=b[4], box=b[:4])
            ag = self._age_gender(img, b[:4], now) if self.ag is not None else None
            if ag is not None:
                d.update(age=round(ag[0]), male=round(ag[1], 2),
                         text=f"{'male' if ag[1] >= 0.5 else 'female'} {round(ag[0])}")
                cv2.putText(v, d["text"], (int(b[0]), max(12, int(b[1]) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 220, 120), 1, cv2.LINE_AA)
            dets.append(d)
        self.tracks = [t for t in self.tracks if now - t["t"] < 1.0]
        dets += [dict(label="person", conf=b[4], box=b[:4]) for b in persons]
        return dets, v

    def status(self):
        ex = []
        for m in self.models():
            ex += [d for d in exec_devices(m.compiled) if d not in ex]
        return {"mode": self.mode, "fps": round(self.fps, 1), "det_ms": round(self.det_ms, 1),
                "models": [m.name for m in self.models()], "device": self.device,
                "exec": ex or sorted({m.device for m in self.models()}), "error": self.err}


def box_iou(a, b):
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - iw * ih
    return iw * ih / u if u > 0 else 0.0


class Actions(EyeLoop):
    """오른눈: 사람 행동 (OMZ person-detection-action-recognition-0006). 6종 확률을 groups 대로 더해 가장 큰 묶음을 표시.
    합친 확률이 act_conf 보다 낮거나, edge_person 이면 화면 왼쪽·오른쪽 끝에 닿은(몸이 잘린) 사람은 행동 없이 'person'
    (샘플 사진에서 잘린 사람에게 엉뚱한 행동이 붙음 — 2026-10-12)."""

    def __init__(self, port, ac):
        super().__init__(port, "right")
        self.title = str(ac.get("title", "actions"))
        self.device = str(ac.get("device", "CPU"))
        self.det = OvActions(self.device, float(ac.get("det_conf", 0.4)), str(ac.get("resize", "letterbox")))
        self.model_name = self.det.name
        self.act_conf = float(ac.get("act_conf", 0.6))
        self.edge_person = bool(ac.get("edge_person", True))
        self.max = int(ac.get("max", 6))
        groups = ac.get("groups") or {"sit": ["sitting", "writing", "lie_on_desk"], "raise hand": ["raising_hand"],
                                      "stand": ["standing", "turned_around"]}
        bad = [a for g in groups.values() for a in g if a not in ACTIONS_0006]
        if bad:
            raise ValueError(f"head_track.actions.groups: unknown action {bad} (use {ACTIONS_0006})")
        self.groups = [(str(name), [ACTIONS_0006.index(a) for a in acts]) for name, acts in groups.items()]

    def process(self, img, now):
        H, W = img.shape[:2]
        dets = []
        for x0, y0, x1, y1, c, prob in self.det(img)[:self.max]:
            p, name = max((float(prob[ix].sum()), n) for n, ix in self.groups)
            cut = self.edge_person and (x0 <= 2 or x1 >= W - 2)
            if p < self.act_conf or cut:
                name, p = "person", c
            dets.append(dict(label=name, conf=p, det=round(c, 3), box=(x0, y0, x1, y1), text=f"{name} {p:.2f}",
                             actions={a: round(float(v), 3) for a, v in zip(ACTIONS_0006, prob)}))
        v = img.copy()
        for d in dets:
            col = DET_COLOR.get(d["label"], (255, 150, 40))
            x0, y0, x1, y1 = (int(t) for t in d["box"])
            cv2.rectangle(v, (x0, y0), (x1, y1), col, 2)
            cv2.putText(v, d["text"], (x0 + 2, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        put_info(v, f"R  {self.fps:.1f} fps  det {self.det_ms:.0f} ms")
        return dets, v

    def status(self):
        return {"fps": round(self.fps, 1), "det_ms": round(self.det_ms, 1), "models": [self.det.name], "title": self.title,
                "device": self.device, "exec": exec_devices(self.det.compiled) or [self.det.device],
                "groups": {n: [ACTIONS_0006[i] for i in ix] for n, ix in self.groups}, "error": self.err}


class Objects(EyeLoop):
    """오른눈: YOLO 검출 (ultralytics, OpenVINO) — 자세(POLAR) 또는 COCO 사물. device 는 ultralytics 형식 (intel:npu / intel:gpu / intel:cpu)."""

    def __init__(self, port, model, device, conf, imgsz, exclude, title="objects"):
        super().__init__(port, "right")
        self.title = title
        from ultralytics import YOLO
        self.model = YOLO(model, task="detect")
        self.model_name = os.path.basename(model.rstrip("/"))
        self.device, self.conf, self.imgsz = device, conf, imgsz
        self.exclude = set(exclude)
        names = self.model.names
        self.keep = None if not self.exclude else [i for i, n in names.items() if n not in self.exclude]

    def process(self, img, now):
        r = self.model(img, conf=self.conf, imgsz=self.imgsz, device=self.device, classes=self.keep, verbose=False)[0]
        dets = []
        if r.boxes is not None and len(r.boxes):
            for (x0, y0, x1, y1), c, k in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(), r.boxes.cls.cpu().numpy()):
                dets.append(dict(label=r.names[int(k)], conf=float(c), box=(float(x0), float(y0), float(x1), float(y1))))
        v = img.copy()
        for d in dets:
            x0, y0, x1, y1 = (int(t) for t in d["box"])
            cv2.rectangle(v, (x0, y0), (x1, y1), (255, 150, 40), 2)
            cv2.putText(v, f"{d['label']} {d['conf']:.2f}", (x0 + 2, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 150, 40), 1, cv2.LINE_AA)
        put_info(v, f"R  {self.fps:.1f} fps  det {self.det_ms:.0f} ms")
        return dets, v

    def status(self):
        # ultralytics 는 없는 장치를 조용히 AUTO 로 바꿈 → exec 가 실제 장치 (첫 추론 전에는 [])
        return {"fps": round(self.fps, 1), "det_ms": round(self.det_ms, 1), "models": [self.model_name], "title": self.title,
                "device": self.device, "exec": yolo_devices(self.model), "error": self.err}


def crop_tile(img, box, label, color, h):
    """인식 박스를 10 % 넓혀 잘라 높이 h 로 + 아래 이름표."""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = box
    mx, my = 0.1 * (x1 - x0), 0.1 * (y1 - y0)
    x0, y0 = int(max(0, x0 - mx)), int(max(0, y0 - my))
    x1, y1 = int(min(W, x1 + mx)), int(min(H, y1 + my))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    c = img[y0:y1, x0:x1]
    cw = int(np.clip(round(c.shape[1] * h / c.shape[0]), 0.6 * h, 1.8 * h))
    c = cv2.resize(c, (cw, h))
    w = max(cw, cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0] + 6)   # 이름표가 잘리지 않을 폭
    if w > cw:                                                                     # 좁은 그림은 가운데 두고 양옆 여백
        pad = np.full((h, w, 3), 24, np.uint8)
        pad[:, (w - cw) // 2:(w - cw) // 2 + cw] = c
        c = pad
    bar = np.full((18, w, 3), 24, np.uint8)
    bar[:3] = color
    cv2.putText(bar, label, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1, cv2.LINE_AA)
    return np.vstack([c, bar])


DET_COLOR = {"face": (0, 220, 120), "person": (0, 160, 255),               # BGR. 그 밖(YOLO 자세·사물) = (255, 150, 40)
             "stand": (255, 150, 40), "raise hand": (0, 200, 255), "sit": (200, 110, 220)}
LEFT_TITLE = {"face": "face", "person": "person", "auto": "face / person"}


def eye_panel(eye, title, h):
    """한 눈 전체 화면을 높이 h 로 줄이고 인식 박스·이름을 그 크기에서 다시 그림 (원본 크기 글씨는 줄이면 안 읽힘)."""
    with eye.lock:
        last = eye.last
    w = int(round(h * 544 / 448))                   # 머리 카메라 눈별 544×448 (robots/h2/FACTS.md) — 영상 없을 때 자리
    if last is None or time.time() - last[2] > 2.0:
        out = np.full((h, w, 3), 18, np.uint8)
        cv2.putText(out, title, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        msg = "no head-cam stream" if last is None or "RGB" in (eye.err or "") else "no update"
        cv2.putText(out, msg, (6, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 150, 230), 1, cv2.LINE_AA)
        return out
    img, dets, _ = last
    s = h / img.shape[0]
    out = cv2.resize(img, (int(round(img.shape[1] * s)), h), interpolation=cv2.INTER_AREA)
    for d in dets:
        c = DET_COLOR.get(d["label"], (255, 150, 40))
        x0, y0, x1, y1 = (int(round(v * s)) for v in d["box"])
        cv2.rectangle(out, (x0, y0), (x1, y1), c, 2)
        cv2.putText(out, d.get("text") or f"{d['label']} {d['conf']:.2f}", (x0 + 2, max(11, y0 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, c, 1, cv2.LINE_AA)
    label = f"{title}  {eye.fps:.1f} fps"
    tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
    out[2:20, 2:tw + 10] = (out[2:20, 2:tw + 10] * 0.35).astype(np.uint8)          # 글씨 뒤 어둡게
    cv2.putText(out, label, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1, cv2.LINE_AA)
    return out


def build_pair(left, right):
    """두 눈 전체 화면을 작게 나란히 (왼눈 | 오른눈). 오른눈 인식이 꺼져 있으면 왼눈만."""
    h = int((cfg("view", {}) or {}).get("height", 200))
    panels = [eye_panel(left, f"L  {LEFT_TITLE.get(left.mode, left.mode)}", h)]
    if right is not None:
        panels += [np.full((h, 4, 3), 14, np.uint8), eye_panel(right, f"R  {right.title}", h)]
    return np.hstack(panels)


def build_strip(left, right):
    """두 눈에서 지금 인식된 것만 잘라 한 줄 (왼눈 → 오른눈). 없으면 안내 한 줄."""
    s = cfg("strip", {}) or {}
    h, gap = int(s.get("height", 72)), 4
    tiles, now, seen = [], time.time(), False
    for eye, maxn in ((left, int(s.get("max_left", 3))), (right, int((cfg("objects", {}) or {}).get("max", 6)))):
        if eye is None:
            continue
        with eye.lock:
            last = eye.last
        if last is None or now - last[2] > FRESH_S:
            continue
        seen = True
        img, dets, _ = last
        for d in sorted(dets, key=lambda d: -d["conf"])[:maxn]:
            color = DET_COLOR.get(d["label"], (255, 150, 40))
            t = crop_tile(img, d["box"], d.get("text") or f"{d['label']} {d['conf']:.2f}", color, h)
            if t is not None:
                tiles.append(t)
    if not tiles:                       # 영상은 오는데 인식 없음 / 영상 자체가 안 옴 (앱 video_hub 끔·Stereo patch PC1 켬 확인)
        msg = ["no detection"] if seen else ["no head-cam stream", "app: video_hub off, Stereo patch PC1 on"]
        out = np.full((h + 18, 330, 3), 18, np.uint8)
        y = (h + 18) // 2 + 5 - 10 * (len(msg) - 1)
        for i, m in enumerate(msg):
            cv2.putText(out, m, (10, y + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55 if i == 0 else 0.42,
                        (130, 130, 130) if seen else (90, 150, 230), 1, cv2.LINE_AA)
        return out
    sep = np.full((h + 18, gap, 3), 14, np.uint8)
    parts = []
    for t in tiles:
        parts += [t, sep]
    return np.hstack(parts[:-1])


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H2 Head Vision</title><style>body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
img{max-width:100%;border:1px solid #444}.row{display:flex;gap:8px;flex-wrap:wrap}.row img{width:calc(50% - 6px);min-width:280px}
pre{background:#222;padding:8px;white-space:pre-wrap}</style></head><body>
<h3>H2 Head Vision &mdash; left: __LEFT__, right: __RIGHT__</h3>
<div><img src="/pair" style="height:200px;width:auto"> <img src="/strip" style="height:96px;width:auto"></div>
<div class="row"><img src="/video_feed"><img src="/video_right"></div><pre id="s"></pre>
<script>setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('s').textContent=JSON.stringify(j,null,1)}),1000)</script>
</body></html>"""


def mjpeg(get, period=0.05):
    def gen():
        last = None
        while True:
            j = get()
            if j is not None and j is not last:
                last = j
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n"
            time.sleep(period)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


def make_app(left, right, strip, pair):
    app = FastAPI(title="H2 head vision")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["GET"], allow_headers=["*"])

    def latest(eye):
        if eye is None:
            return None
        with eye.lock:
            return eye.jpeg

    @app.get("/", response_class=HTMLResponse)
    def index():
        return PAGE.replace("__LEFT__", LEFT_TITLE.get(left.mode, left.mode)).replace(
            "__RIGHT__", right.title if right else "off")

    @app.get("/status")
    def status():
        return {"left": left.status(), "right": right.status() if right else None}

    @app.get("/detections")
    def detections():
        out, now = [], time.time()
        for eye in (left, right):
            if eye is None:
                continue
            with eye.lock:
                last = eye.last
            if last is not None and now - last[2] <= FRESH_S:
                out += [dict(eye=eye.name, label=d["label"], conf=round(d["conf"], 3), box=[round(v) for v in d["box"]],
                             **{k: d[k] for k in ("text", "age", "male", "det", "actions") if k in d})
                        for d in last[1]]
        return out

    @app.get("/pair")
    def pair_feed():
        return mjpeg(lambda: pair["jpeg"], 0.1)

    @app.get("/pair.jpg")
    def pair_jpg():
        j = pair["jpeg"]
        if j is None:
            raise HTTPException(503, "not ready")
        return Response(j, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/strip")
    def strip_feed():
        return mjpeg(lambda: strip["jpeg"], 0.1)

    @app.get("/strip.jpg")
    def strip_jpg():
        j = strip["jpeg"]
        if j is None:
            raise HTTPException(503, "not ready")
        return Response(j, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/video_feed")
    def video_feed():
        return mjpeg(lambda: latest(left))

    @app.get("/video_right")
    def video_right():
        return mjpeg(lambda: latest(right))

    return app


def main():
    ap = argparse.ArgumentParser(description="H2 head camera vision (left: face/person, right: YOLO detect — posture/objects)")
    ap.add_argument("--mode", choices=("face", "person", "auto"), default=cfg("mode", "auto"))
    ap.add_argument("--device", default=cfg("device", "CPU"), help="face/person OpenVINO device CPU / GPU / NPU / AUTO")
    ap.add_argument("--no-objects", action="store_true", help="disable right-eye detection")
    ap.add_argument("--port", type=int, default=int(cfg("port", 50013)))
    ap.add_argument("--face-conf", type=float, default=None, help="override robot.yaml head_track.face_conf")
    ap.add_argument("--person-conf", type=float, default=None, help="override robot.yaml head_track.person_conf")
    a = ap.parse_args()
    if a.face_conf is not None:
        C["face_conf"] = a.face_conf
    if a.person_conf is not None:
        C["person_conf"] = a.person_conf

    left = Faces(RGB_PORTS["left"], a.mode, a.device, bool((cfg("age_gender", {}) or {}).get("enabled", False)))
    right = None
    ac = cfg("actions", {}) or {}
    oc = cfg("objects", {}) or {}
    if a.no_objects:
        pass
    elif ac.get("enabled", False):
        right = Actions(RGB_PORTS["right"], ac)
    elif oc.get("enabled", True):
        path = os.path.join(MODEL_DIR, oc.get("model", "polar11s_openvino_model"))
        if os.path.exists(path):
            right = Objects(RGB_PORTS["right"], path, oc.get("device", "intel:cpu"), float(oc.get("conf", 0.45)),
                            int(oc.get("imgsz", 640)), oc.get("exclude", []), str(oc.get("title", "objects")))
        else:
            print(f"[head_track] WARNING object model not found ({path}) - right eye off")
    print(f"[head_track] left {[m.name + '@' + m.device for m in left.models()]}"
          + (f", right {right.model_name}@{right.device}" if right else ", right off")
          + f", {cfg('fps', 5.0)} fps ->\n    {web_urls(a.port, PC1)}")

    strip = {"jpeg": None}
    pair = {"jpeg": None}

    def strip_loop():
        while True:
            strip["jpeg"] = jpeg(build_strip(left, right), 85)
            pair["jpeg"] = jpeg(build_pair(left, right), 85)
            time.sleep(0.2)
    left.start()
    if right is not None:
        right.start()
    threading.Thread(target=strip_loop, daemon=True).start()
    uvicorn.run(make_app(left, right, strip, pair), host="0.0.0.0", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
