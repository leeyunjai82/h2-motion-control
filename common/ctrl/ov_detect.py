"""
ov_detect.py — OpenVINO Open Model Zoo 모델 (얼굴 / 사람 검출, 나이·성별, 사람 행동)

  모델 (common/models/omz, Intel Open Model Zoo 2023.0 FP16, Apache-2.0)
    face       : face-detection-retail-0004    입력 1×3×300×300 BGR
    person     : person-detection-0201          입력 1×3×384×384 BGR (retail-0013 은 위에서 내려보는 매장 카메라용 — 눈높이 사람을 놓침)
      SSD 출력 1×1×200×7 = [image_id, label, conf, x_min, y_min, x_max, y_max] (0–1), image_id −1 이후는 무효
    age_gender : age-gender-recognition-retail-0013   얼굴 62×62 BGR → 나이/100 (fc3_a), [여, 남] (prob). 18~75세로 학습
    actions    : person-detection-action-recognition-0006   1×400×680×3 BGR → 사람 박스 + 행동 6종
                 (sitting, writing, raising_hand, standing, turned_around, lie_on_desk — 교실 장면으로 학습)

  det = OvSSD("face", device="NPU", conf=0.6)   # 안 되면 HETERO:NPU,CPU → CPU 로 자동 (det.device = 실제 장치)
  boxes = det(bgr)        # [(x0, y0, x1, y1, conf), ...] 원본 픽셀, conf 내림차순
  컴파일 결과는 ~/.cache/h2-motion-control/openvino 에 캐시 (NPU 첫 컴파일이 느려서)
"""
import os

import cv2
import numpy as np

try:
    import openvino as ov                     # 2023.1 이후
    _Core = ov.Core
except (ImportError, AttributeError):
    from openvino.runtime import Core as _Core   # 2022.x

MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "omz")
MODELS = {"face": "face-detection-retail-0004", "person": "person-detection-0201",
          "age_gender": "age-gender-recognition-retail-0013", "actions": "person-detection-action-recognition-0006"}
CACHE_DIR = os.path.expanduser("~/.cache/h2-motion-control/openvino")


def compile_with_fallback(core, model, device):
    """device 로 컴파일, 실패하면 HETERO:device,CPU (안 되는 층만 CPU) → CPU. 반환 (compiled, 실제 장치)."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    order = [device] if device in ("CPU", "AUTO") or device.startswith("HETERO") else [device, f"HETERO:{device},CPU", "CPU"]
    err = None
    for d in order:
        try:
            return core.compile_model(model, d, {"CACHE_DIR": CACHE_DIR}), d
        except Exception as e:      # noqa: BLE001 — 장치 없음·지원 안 되는 층 등
            err = e
            why = " ".join(ln.strip() for ln in str(e).splitlines() if ln.strip() and ".cpp:" not in ln)[:200]
            print(f"[ov_detect] {d} compile failed -> next device: {why}")
    raise err


def load(name, device):
    """MODELS 이름 또는 .xml 경로 → (compiled, 실제 장치, 모델 이름)."""
    xml = name if name.endswith(".xml") else os.path.join(MODEL_DIR, MODELS.get(name, name) + ".xml")
    if not os.path.exists(xml):
        raise FileNotFoundError(f"model not found: {xml}")
    core = _Core()
    compiled, device = compile_with_fallback(core, core.read_model(xml), device)
    return compiled, device, os.path.splitext(os.path.basename(xml))[0]


class OvSSD:
    def __init__(self, name, device="CPU", conf=0.5):
        self.compiled, device, self.name = load(name, device)
        self.out = self.compiled.output(0)
        _, _, self.h, self.w = [int(d) for d in self.compiled.input(0).shape]
        self.conf = float(conf)
        self.device = device

    def __call__(self, bgr):
        H, W = bgr.shape[:2]
        blob = cv2.resize(bgr, (self.w, self.h)).transpose(2, 0, 1)[None].astype(np.float32)
        det = np.asarray(self.compiled([blob])[self.out]).reshape(-1, 7)
        end = np.where(det[:, 0] < 0)[0]
        if len(end):
            det = det[:end[0]]
        det = det[det[:, 2] >= self.conf]
        out = []
        for _, _, c, x0, y0, x1, y1 in det[np.argsort(-det[:, 2])]:
            x0, x1 = np.clip([x0, x1], 0, 1) * W
            y0, y1 = np.clip([y0, y1], 0, 1) * H
            if x1 - x0 >= 2 and y1 - y0 >= 2:
                out.append((float(x0), float(y0), float(x1), float(y1), float(c)))
        return out


class OvAgeGender:
    """얼굴 박스 → (나이, 남자 확률). Intel interactive_face_detection_demo 와 같게 얼굴 박스를 1.2 배 정사각형으로 넓혀
    자르고(화면 밖은 잘림) 62×62 BGR 0~255 로 넣음."""
    ENLARGE = 1.2

    def __init__(self, device="CPU"):
        self.compiled, self.device, self.name = load("age_gender", device)
        self.age_out = self.compiled.output("fc3_a")
        self.gender_out = self.compiled.output("prob")

    def __call__(self, bgr, box):
        H, W = bgr.shape[:2]
        x0, y0, x1, y1 = box[:4]
        cx, cy, s = (x0 + x1) / 2, (y0 + y1) / 2, self.ENLARGE * max(x1 - x0, y1 - y0)
        a0, b0 = int(max(0, cx - s / 2)), int(max(0, cy - s / 2))
        a1, b1 = int(min(W, cx + s / 2)), int(min(H, cy + s / 2))
        if a1 - a0 < 16 or b1 - b0 < 16:
            return None
        blob = cv2.resize(bgr[b0:b1, a0:a1], (62, 62)).transpose(2, 0, 1)[None].astype(np.float32)
        r = self.compiled([blob])
        return float(np.asarray(r[self.age_out]).reshape(-1)[0] * 100), float(np.asarray(r[self.gender_out]).reshape(-1)[1])


ACTIONS_0006 = ["sitting", "writing", "raising_hand", "standing", "turned_around", "lie_on_desk"]


class OvActions:
    """person-detection-action-recognition-0006 → [(x0, y0, x1, y1, 사람 conf, 행동 확률 (6,)), ...] 원본 픽셀.

    풀기는 Intel smart_classroom_demo (action_detector.cpp, 2023.0) 와 같음: 사전박스 2 헤드(보폭 8: 앵커 1 / 16: 앵커 4),
    분산 [0.1, 0.1, 0.2, 0.2], loc = [dy, dx, dh, dw], 행동 = softmax(16 × logit), Soft-NMS (sigma 0.6).
    단 행동 출력은 IR 모양 [1, H, W, 6] 그대로 (칸, 행동) 순서로 읽음 — 데모 C++ 처럼 행동이 앞인 순서로 읽으면
    모델 문서 예시 그림(앉음 4 · 손 듦 1)과 안 맞음 (2026-10-12 확인).
    resize: letterbox = 비율 유지 + 오른쪽·아래 여백 (머리 카메라 544×448 에서 서 있는 사람 검출이 더 좋음) / stretch = 데모처럼 늘림."""
    HEADS = [(8, [(26.17863728, 58.670372)]),
             (16, [(35.36, 81.829632), (45.8114572, 107.651852), (63.31491832, 142.595732), (93.5070856, 201.107692)])]
    VAR = (0.1, 0.1, 0.2, 0.2)
    SCALE = 16.0
    SIGMA = 0.6

    def __init__(self, device="CPU", conf=0.4, resize="letterbox"):
        self.compiled, self.device, self.name = load("actions", device)
        _, self.ih, self.iw, _ = [int(d) for d in self.compiled.input(0).shape]
        self.loc = self.compiled.output("ActionNet/out_detection_loc")
        self.det = self.compiled.output("ActionNet/out_detection_conf")
        self.acts = [self.compiled.output("ActionNet/action_heads/out_head_1_anchor_1")] + \
                    [self.compiled.output(f"ActionNet/action_heads/out_head_2_anchor_{k}") for k in range(1, 5)]
        self.conf = float(conf)
        self.letterbox = resize != "stretch"

    def __call__(self, bgr):
        H, W = bgr.shape[:2]
        if self.letterbox:
            s = min(self.iw / W, self.ih / H)
            x = np.zeros((self.ih, self.iw, 3), np.uint8)
            x[:int(round(H * s)), :int(round(W * s))] = cv2.resize(bgr, (int(round(W * s)), int(round(H * s))))
            fw, fh = self.iw / s, self.ih / s          # 정규화 좌표 → 원본 픽셀
        else:
            x, fw, fh = cv2.resize(bgr, (self.iw, self.ih)), W, H
        r = self.compiled([x[None].astype(np.float32)])
        loc = np.asarray(r[self.loc])[0]
        conf = np.asarray(r[self.det])[0][:, 1]
        acts = [np.asarray(r[o])[0] for o in self.acts]
        cand, start, g = [], 0, 0
        for step, anchors in self.HEADS:
            bh, bw, nc = acts[g].shape
            na = len(anchors)
            n = bh * bw * na
            for hp in np.where(conf[start:start + n] >= self.conf)[0]:
                p, aid, pos = start + hp, hp % na, hp // na
                row, col = divmod(pos, bw)
                lg = acts[g + aid].reshape(-1, nc)[pos] * self.SCALE
                e = np.exp(lg - lg.max())
                pw, ph = anchors[aid][0] / self.iw, anchors[aid][1] / self.ih
                dy, dx, dh, dw = loc[p]
                cx = self.VAR[0] * dx * pw + (col + 0.5) * step / self.iw
                cy = self.VAR[1] * dy * ph + (row + 0.5) * step / self.ih
                w, h = np.exp(self.VAR[2] * dw) * pw, np.exp(self.VAR[3] * dh) * ph
                cand.append(((cx - w / 2) * fw, (cy - h / 2) * fh, (cx + w / 2) * fw, (cy + h / 2) * fh,
                             float(conf[p]), e / e.sum()))
            start += n
            g += na
        if not cand:
            return []
        b = np.array([c[:4] for c in cand])
        sc = np.array([c[4] for c in cand])
        area = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
        out = []
        while sc.max() >= self.conf:                     # Soft-NMS (데모와 같음)
            i = int(sc.argmax())
            sc[i] = 0.0
            x0, y0, x1, y1 = b[i]
            out.append((float(max(0, x0)), float(max(0, y0)), float(min(W, x1)), float(min(H, y1)), cand[i][4], cand[i][5]))
            iw = np.clip(np.minimum(x1, b[:, 2]) - np.maximum(x0, b[:, 0]), 0, None)
            ih = np.clip(np.minimum(y1, b[:, 3]) - np.maximum(y0, b[:, 1]), 0, None)
            iou = iw * ih / (area[i] + area - iw * ih + 1e-9)
            sc = sc * np.exp(-iou ** 2 / self.SIGMA)
        return out
