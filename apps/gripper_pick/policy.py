"""
policy.py — lerobot 정책 실행 (잡기 웹 pick_web 이 씀): 모델 폴더 → 추론 프로세스 (lerobot 가상환경) → 30 Hz → 팔 /stream + 그리퍼

  모델 = lerobot pretrained_model 폴더 (config.json · model.safetensors · policy_pre/postprocessor …) + h2_model.json (있으면)
    정책 종류 상관없음 (act · diffusion · smolvla · pi0 …) — 추론은 pitron/learning/lerobot_infer.py 가 lerobot 표준대로
    ACT 는 OpenVINO 로 (CPU · GPU · NPU), 나머지는 torch (CPU · Intel GPU xpu)
  저장: data/policies/<이름>/ (git 제외). 이 PC 에서 학습하면 자동으로 들어옴, 다른 PC 모델은 zip 으로 올림
  데이터셋은 안 씀 — 모델과 이 로봇의 약속은 state 16 (왼팔 7 · 왼 그리퍼 · 오른팔 7 · 오른 그리퍼) · action 16 · 카메라 이름

  한 틱 (30 Hz, 제어 루프)
    추론 요청이 없으면 지금 관측으로 하나 보냄 (영상은 모델 입력 크기로 camfit — 맞춤 방식은 h2_model.json fit, 없으면 화면에서 고름)
    답이 왔으면 그 action 으로 안전 필터 → 팔 14 축 /stream/cmd · 그리퍼 목표
    답이 아직이면 (chunk 를 새로 계산 중) 마지막 목표를 다시 보냄 = 그 자리에 멈춰 기다림 (arm_server 세션 유지)
  멈춤 (팔은 그 자리, 그리퍼는 그 상태 유지)
    · 정책 출력이 모델의 학습 범위 (action min/max) 를 range_tol 넘게 벗어남 — 범위 안은 그대로, 약간 넘는 건 범위로 자름
    · 집게 가운데 (지령 관절 FK) 가 작업 범위 밖 — config/local.yaml policy_tcp_box (없으면 이 검사 안 함)
    · 그리퍼 가장 낮은 점이 테이블 평면 + floor_clear 아래 (테이블을 쟀을 때)
    · 지령 ↔ 실측 관절 차이 > track_abort_deg (arm_server 응답)
    · 추론을 stall_s 넘게 기다림 · 최대 시간 · [정지]
  한 회 끝: min_run_s 지난 뒤 지령이 준비 자세 (양손) 에 hold_end_s 머묾 — 수집도 준비 자세에서 시작해 준비 자세로 끝남
"""
import json
import math
import os
import re
import shutil
import subprocess
import threading
import time
import zipfile

import cv2
import numpy as np

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron import paths  # noqa: E402
from pitron.learning import camfit, lerobot_env  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = paths.data_dir("policies", legacy="gripper/models")   # 모델 저장소 (git 제외)
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,80}$")
FILE_RE = re.compile(r"^[A-Za-z0-9_.\-]+\.(json|safetensors)$")
FPS = 30.0
STATE_DIM = 16
# 모델 state · action 16 → 팔 14 (제어 순서) · 그리퍼: (손, 팔 14 자리, action 팔 7 자리, action 그리퍼 자리)
PARTS = (("left", slice(0, 7), slice(0, 7), 7), ("right", slice(7, 14), slice(8, 15), 15))

DEFAULTS = {
    "max_s": 90.0,           # 최대 실행 시간 [s]
    "stall_s": 15.0,         # 추론 답을 이만큼 못 받으면 멈춤 [s] (그동안 팔은 제자리)
    "range_tol": 0.15,       # 학습 범위 밖 허용 [rad] — 넘으면 멈춤 (안쪽은 범위로 자름)
    "floor_clear": 0.005,    # 그리퍼 ↔ 테이블 최소 [m]
    "track_abort_deg": 15.0,
    "server_max_vel": 1.0,   # arm_server /stream 관절 속도 상한 [rad/s]
    "hold_end_s": 1.0,       # 준비 자세에 이만큼 머물면 한 회 끝
    "min_run_s": 8.0,        # 시작 직후는 끝 판정 안 함 (시작도 준비 자세라서)
    "repeat_gap_s": 2.0,     # 반복 실행: 한 회가 정상으로 끝난 뒤 다음 회까지 쉬는 시간 [s]
    "repeat_max": 20,        # 반복 실행: 최대 회수 (안전)
}
END_OK = "준비 자세 도착"   # 이 이유로 끝난 회만 다음 회로 이어감 (안전 필터 · 정지 · 오류 · 최대 시간은 멈춤)


# ---------------- 모델 파일 ----------------
def _read(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def model_info(name):
    d = os.path.join(MODEL_DIR, name)
    cfg = _read(os.path.join(d, "config.json"))
    if not cfg or not os.path.isfile(os.path.join(d, "model.safetensors")):
        return None
    h2 = _read(os.path.join(d, "h2_model.json")) or {}
    feats = cfg.get("input_features") or {}
    cams = {k.split(".", 2)[-1]: f"{v['shape'][2]}×{v['shape'][1]}" for k, v in feats.items()
            if k.startswith("observation.images.") and len(v.get("shape", [])) == 3}
    return {"name": name, "policy": cfg.get("type"), "cameras": cams,
            "state_dim": ((feats.get("observation.state") or {}).get("shape") or [None])[0],
            "action_dim": (((cfg.get("output_features") or {}).get("action") or {}).get("shape") or [None])[0],
            "dataset": h2.get("dataset"), "episodes": h2.get("dataset_episodes"), "step": h2.get("step"), "task": h2.get("task"),
            "fit": h2.get("fit"), "trained": h2.get("trained"), "has_h2": bool(h2)}


def models():
    if not os.path.isdir(MODEL_DIR):
        return []
    return [m for m in (model_info(n) for n in sorted(os.listdir(MODEL_DIR)) if NAME_RE.match(n)) if m]


def install_zip(zpath, filename="model.zip"):
    """model zip (<이름>/config.json · model.safetensors · … — 학습 웹 zip 또는 lerobot pretrained_model 폴더 zip) → MODEL_DIR/<이름>"""
    with zipfile.ZipFile(zpath) as z:
        files = [i for i in z.infolist() if not i.is_dir()]
        roots = {i.filename.split("/", 1)[0] for i in files if "/" in i.filename}
        if len(roots) != 1 or any("/" not in i.filename for i in files):
            raise ValueError("zip 안에 폴더 하나 (<모델 이름>/config.json …) 여야 함")
        root = roots.pop()
        name = root
        if root in ("pretrained_model", "last") or not NAME_RE.match(root):   # lerobot 체크포인트 폴더를 그대로 묶은 zip
            name = re.sub(r"[^A-Za-z0-9_.\-]", "_", os.path.splitext(os.path.basename(filename))[0])[:60] or "model"
        keep = []
        for i in files:
            rel = i.filename[len(root) + 1:]
            if "/" in rel or ".." in rel:
                continue                                    # 하위 폴더 (openvino 캐시 등) 는 안 받음
            if FILE_RE.match(rel):
                keep.append((i, rel))
        have = {r for _, r in keep}
        if not {"config.json", "model.safetensors"} <= have:
            raise ValueError("lerobot 모델이 아님 (config.json · model.safetensors 없음)")
        cfg = json.loads(z.read(f"{root}/config.json"))
        if not cfg.get("type"):
            raise ValueError("config.json 에 정책 type 없음")
        dst = os.path.join(MODEL_DIR, name)
        tmp = dst + ".tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        for i, rel in keep:
            with z.open(i) as src, open(os.path.join(tmp, rel), "wb") as out:
                shutil.copyfileobj(src, out)
        shutil.rmtree(dst, ignore_errors=True)
        os.replace(tmp, dst)
    return name


# ---------------- 추론 프로세스 ----------------
class Infer:
    """pitron/learning/lerobot_infer.py 를 lerobot 가상환경에서 띄우고 stdin/stdout 으로 주고받음."""

    def __init__(self, name, device, task="", fit=None):
        d = os.path.join(MODEL_DIR, name)
        cfg = _read(os.path.join(d, "config.json")) or {}
        self.h2 = _read(os.path.join(d, "h2_model.json")) or {}
        py = lerobot_env.python()
        if not os.path.isfile(py):
            raise RuntimeError(lerobot_env.missing_msg(py))
        ptype = cfg.get("type")
        if ptype == "act":
            args = ["--backend", "openvino", "--ov-device", device]
        elif device == "NPU":
            raise RuntimeError(f"NPU 는 OpenVINO (ACT) 만 — 이 모델은 {ptype}: CPU 또는 GPU")
        else:
            args = ["--backend", "torch", "--device", "xpu" if device == "GPU" else "cpu"]
        self.task = task or self.h2.get("task") or ""
        self.fit = fit or self.h2.get("fit") or "stretch"
        self.name, self.device, self.ptype = name, device, ptype
        self.lk = threading.Lock()
        self.proc = subprocess.Popen([py, "-u", "-m", "pitron.learning.lerobot_infer", "--model", d, "--task", self.task] + args,
                                     cwd=_ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     env={**os.environ, "HF_HUB_OFFLINE": "1"})
        self.err_tail = []
        threading.Thread(target=self._stderr, daemon=True).start()
        info = self._read()
        if not info.get("ok"):
            self.close()
            raise RuntimeError(info.get("err") or "추론 프로세스 시작 실패")
        self.info = info
        if info["state_dim"] != STATE_DIM or info["action_dim"] != STATE_DIM:
            self.close()
            raise RuntimeError(f"이 로봇은 state · action 16 — 모델은 state {info['state_dim']} · action {info['action_dim']}")
        self.cams = info["cams"]
        self.a_min = np.asarray(info["a_min"]) if info.get("a_min") else None
        self.a_max = np.asarray(info["a_max"]) if info.get("a_max") else None

    def _stderr(self):
        for ln in self.proc.stderr:
            ln = ln.decode(errors="replace").rstrip()
            if ln:
                self.err_tail = (self.err_tail + [ln])[-20:]

    def _read(self):
        while True:
            ln = self.proc.stdout.readline()
            if not ln:
                tail = next((t for t in reversed(self.err_tail) if "Error" in t), self.err_tail[-1] if self.err_tail else "")
                return {"ok": False, "err": f"추론 프로세스 끝남 {tail}"[-400:]}
            ln = ln.decode(errors="replace")
            if ln.startswith("@@"):
                return json.loads(ln[2:])

    def reset(self):
        with self.lk:
            self.proc.stdin.write(b'{"op":"reset"}\n')
            self.proc.stdin.flush()
            return self._read()

    def act(self, state, rgb_list):
        """state (16,) · 모델 카메라 순서의 RGB uint8 → (action (16,), ms)"""
        with self.lk:
            self.proc.stdin.write((json.dumps({"op": "act", "state": [float(x) for x in state]}) + "\n").encode())
            for im in rgb_list:
                self.proc.stdin.write(np.ascontiguousarray(im, np.uint8).tobytes())
            self.proc.stdin.flush()
            r = self._read()
        if not r.get("ok"):
            raise RuntimeError(r.get("err") or "추론 실패")
        return np.asarray(r["action"], dtype=float), r.get("ms")

    def prep(self, name, bgr):
        """수집과 같게: camfit (모델 입력 크기 · 맞춤 방식) → JPEG q90 한 번 → RGB."""
        c = next(c for c in self.cams if c["name"] == name)
        bgr = camfit.fit(bgr, c["w"], c["h"], self.fit)
        ok, b = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            bgr = cv2.imdecode(b, cv2.IMREAD_COLOR)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def close(self):
        try:
            self.proc.stdin.write(b'{"op":"quit"}\n')
            self.proc.stdin.flush()
            self.proc.wait(3)
        except Exception:  # noqa: BLE001
            self.proc.kill()


class PolicyStop(Exception):
    """정책 실행을 멈추는 이유 (안전 필터 · 정지 · 끝)."""


class PolicyRunner:
    """pick_web Runner 와 붙어 동작: r.cam · r.wrist · r.grip · r.mdl · r.cam_pitch(실측) · r.table_plane · r.say."""

    def __init__(self, r, pc):
        self.r, self.pc = r, pc
        self.policy = None
        self.cfg = dict(DEFAULTS)
        self.th = None
        self.stop_evt = threading.Event()
        self.state = {"running": False, "step": 0, "infer_ms": None, "why_end": "", "phase": ""}
        box = lerobot_env.local_cfg().get("policy_tcp_box") or {}
        self.tcp_box = {s: (np.asarray(box[s]["min"], float), np.asarray(box[s]["max"], float))
                        for s in ("left", "right") if isinstance(box.get(s), dict)}

    def load(self, name, device, task="", fit=None):
        if self.running():
            raise RuntimeError("정책 실행 중")
        if fit and fit not in camfit.MODES:
            raise ValueError(f"맞춤 방식 {fit}")
        if self.policy is not None:
            self.policy.close()
        self.policy = None
        self.policy = Infer(name, device, task, fit)
        return self.info()

    def info(self):
        p = self.policy
        if p is None:
            return None
        i = p.info
        return {"name": p.name, "device": p.device, "policy": p.ptype, "backend": i.get("backend"),
                "cameras": [c["name"] for c in p.cams], "sizes": {c["name"]: f"{c['w']}×{c['h']}" for c in p.cams},
                "fit": p.fit, "task": p.task, "chunk": i.get("n_action_steps"), "load_s": i.get("load_s"),
                "infer_ms": i.get("infer_ms"), "ov_diff": i.get("ov_diff"), "episodes": p.h2.get("dataset_episodes"),
                "steps": p.h2.get("step"), "dataset": p.h2.get("dataset"), "has_h2": bool(p.h2),
                "tcp_box": bool(self.tcp_box)}

    def running(self):
        return self.th is not None and self.th.is_alive()

    # ----- 시작 조건 -----
    def check(self):
        """→ 문제 목록 (빈 목록이면 시작 가능)."""
        r, p, bad = self.r, self.policy, []
        if p is None:
            return ["모델을 불러오지 않음"]
        if p.proc.poll() is not None:
            return ["추론 프로세스가 끝남 — 모델을 다시 불러오세요"]
        if r.busy() or self.running():
            bad.append("다른 동작 중")
        if r.rec.active or r.rec.pending is not None:
            bad.append("녹화 중 · 저장 결정 대기")
        names = r.wrist.names() if r.wrist else []
        miss = [c["name"] for c in p.cams if c["name"] != "body" and c["name"] not in names]
        if miss:
            bad.append(f"카메라 없음: {miss}")
        sn = p.h2.get("state_names")
        if sn and list(sn) != list(r.rec_state_names()):
            bad.append("모델 관절 순서가 이 로봇과 다름 (h2_model.json state_names)")
        q = self.meas_arm()
        if q is None:
            bad.append("팔 실측 없음 (rt/lowstate · arm_server)")
        if self.grip_q() is None:
            bad.append("그리퍼 상태 없음")
        cp = r.cam_pitch.read() if r.cam_pitch else None
        if cp is not None and abs(cp["diff"]) > 1.0:
            bad.append(f"카메라 숙임 기준과 {cp['diff']:+.2f}° — 거치대를 맞추세요")
        if q is not None:
            qm = r.mdl.to_model(q)
            off = [("왼팔" if s == "left" else "오른팔") for s in self.pc.SIDES if not self.pc.at_ready(r.mdl, qm, s)]
            if off:
                bad.append(f"{' · '.join(off)} 이 준비 자세가 아님 — 먼저 [준비 자세로] (수집과 같은 시작)")
        return bad

    def warnings(self):
        """시작은 되지만 알아 둘 것."""
        p, w = self.policy, []
        if p is None:
            return w
        if not p.h2.get("state_names"):
            w.append("관절 이름 정보 없음 (h2_model.json) — 이 로봇 데이터셋으로 학습한 모델인지 확인")
        if not self.tcp_box:
            w.append("작업 범위 (policy_tcp_box) 설정 없음 — 집게 위치 범위 검사 안 함")
        if p.a_min is None:
            w.append("모델에 action 범위 통계 없음 — 범위 검사 안 함")
        return w

    def meas_arm(self):
        q = self.r.cam_pitch.arm_now() if self.r.cam_pitch else None
        return q if q is not None else self.r.arm_q

    def grip_q(self):
        out = {}
        for side, *_ in PARTS:
            g = self.r.grip.state(side)
            if g is None or g.get("q") is None:
                return None
            out[side] = g["q"]
        return out

    @staticmethod
    def state16(q14, gq):
        """팔 14 (제어 순서) + {손: 그리퍼 q} → state 16 (녹화 순서와 같음)"""
        return np.r_[q14[0:7], gq["left"], q14[7:14], gq["right"]]

    # ----- 실행 -----
    def start(self, repeat=False, run_no=1):
        """repeat: 한 회가 '준비 자세 도착' 으로 끝나면 repeat_gap_s 쉬고 다음 회를 자동으로 (시작 조건을 매번 다시 검사).
        안전 필터 · [정지] · 오류 · 최대 시간으로 끝나면 반복도 멈춤. 최대 repeat_max 회."""
        bad = self.check()
        if bad:
            raise RuntimeError("; ".join(bad))
        if run_no == 1:
            self.user_stop = False
        self.repeat = bool(repeat)
        self.stop_evt.clear()
        self.state = {"running": True, "step": 0, "infer_ms": None, "why_end": "", "phase": "정책", "wait_s": 0.0,
                      "run_no": run_no, "repeat": self.repeat}
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def stop(self):
        self.user_stop = True            # 반복 중이면 다음 회도 안 함
        self.repeat = False
        self.stop_evt.set()
        if self.state.get("waiting_next"):
            self.state.update(waiting_next=False, phase="", why_end=f"{self.state.get('why_end', '')} · 반복 멈춤 (정지)")

    def _next_run(self, run_no):
        r = self.r
        t_end = time.time() + float(self.cfg["repeat_gap_s"])
        while time.time() < t_end:
            if self.user_stop:
                return
            time.sleep(0.1)
        if self.user_stop:
            return
        try:
            self.start(repeat=True, run_no=run_no)
            r.say(f"정책 반복 — {run_no} 회째 시작")
        except RuntimeError as e:
            self.state.update(why_end=f"{self.state.get('why_end', '')} · 반복 멈춤: {e}")
            r.say(f"정책 반복 멈춤 — 시작 조건: {e}")

    def _observe(self):
        """→ (state 16, 모델 카메라 순서 RGB 목록) 또는 None"""
        r, p = self.r, self.policy
        q, gq = self.meas_arm(), self.grip_q()
        if q is None or gq is None:
            return None
        imgs = []
        for c in p.cams:
            img = r.cam.frame()[0] if c["name"] == "body" else r.wrist.frame(c["name"])[0]
            if img is None:
                return None
            imgs.append(p.prep(c["name"], img))
        return self.state16(np.asarray(q, float), gq), imgs

    def _ask(self, box):
        """추론 한 번 (스레드) → box['a'] 또는 box['err']"""
        try:
            obs = self._observe()
            if obs is None:
                box["err"] = "팔 · 그리퍼 실측 또는 카메라 영상 끊김"
                return
            box["a"], ms = self.policy.act(*obs)
            self.state["infer_ms"] = ms
        except Exception as e:  # noqa: BLE001
            box["err"] = f"추론 오류 {type(e).__name__}: {e}"

    def _run(self):
        r, pc, p, cfg = self.r, self.pc, self.policy, self.cfg
        why = "끝"
        tok = None
        try:
            p.reset()
            b = pc.call("/stream/begin", {"max_vel": float(cfg["server_max_vel"])})
            tok = b["token"]
            target = np.radians(np.asarray(b["target_deg"], dtype=float))      # 팔 14 축
            pc.LAST_CMD.update(q=target.copy(), t=time.time())
            r.say(f"정책 시작 {p.name} ({p.ptype}, {p.info.get('backend')} {p.device}, 첫 추론 {p.info.get('infer_ms')} ms)")
            t_start = time.time()
            t_near = None
            nxt = time.time()
            box, th, t_wait = None, None, None
            while True:
                nxt += 1 / FPS
                if self.stop_evt.is_set():
                    raise PolicyStop("정지")
                if time.time() - t_start > cfg["max_s"]:
                    raise PolicyStop(f"최대 시간 {cfg['max_s']:.0f} 초")
                if th is None:                                  # 요청 하나 보냄 (한 틱에 최대 하나)
                    box = {}
                    th = threading.Thread(target=self._ask, args=(box,), daemon=True)
                    th.start()
                    t_wait = time.time()
                    th.join(max(0.0, nxt - time.time() - 0.003))
                a = None
                if not th.is_alive():
                    if box.get("err"):
                        raise PolicyStop(box["err"])
                    a, th = box["a"], None
                    self.state["wait_s"] = 0.0
                else:
                    w = time.time() - t_wait
                    self.state["wait_s"] = round(w, 1)
                    if w > cfg["stall_s"]:
                        raise PolicyStop(f"추론 답을 {w:.0f} 초 못 받음")
                if a is None:                                   # 계산 중 — 마지막 목표 그대로 다시 보냄 (제자리)
                    rr = pc.call("/stream/cmd", {"token": tok, "deg": np.degrees(target).tolist()}, timeout=1.0)
                    time.sleep(max(0.0, nxt - time.time()))
                    continue
                # 안전 필터
                if p.a_min is not None:
                    over = np.maximum(p.a_min - a, a - p.a_max)
                    if np.max(over) > cfg["range_tol"]:
                        k = int(np.argmax(over))
                        raise PolicyStop(f"정책 출력이 학습 범위 밖 ({r.rec_state_names()[k]} {math.degrees(over[k]):.0f}° 넘음)")
                    a = np.clip(a, p.a_min, p.a_max)
                cmd = target.copy()
                for _, csl, asl, _ in PARTS:
                    cmd[csl] = a[asl]
                qm = r.mdl.to_model(cmd)
                for side, *_ in PARTS:                          # 손마다: 작업 범위 · 테이블
                    T = r.mdl.tcp(qm, side)
                    if side in self.tcp_box:
                        lo, hi = self.tcp_box[side]
                        if np.any(T.translation < lo) or np.any(T.translation > hi):
                            raise PolicyStop(f"{side} 집게 가운데 {np.round(T.translation, 3).tolist()} 가 작업 범위 밖")
                    if r.table_plane is not None:
                        a0, bb, cc = r.table_plane
                        tz = a0 + bb * T.translation[0] + cc * T.translation[1]
                        low = r.mdl.lowest(qm, side)
                        if low < tz + cfg["floor_clear"]:
                            raise PolicyStop(f"{side} 그리퍼가 테이블에 {max(0.0, low - tz) * 1000:.0f} mm 까지 — 멈춤")
                rr = pc.call("/stream/cmd", {"token": tok, "deg": np.degrees(cmd).tolist()}, timeout=1.0)
                target = np.radians(np.asarray(rr["target_deg"], dtype=float))
                pc.LAST_CMD.update(q=target.copy(), t=time.time())
                lag = float(np.max(np.abs(np.asarray(rr["target_deg"]) - np.asarray(rr["meas_deg"]))))
                if lag > cfg["track_abort_deg"]:
                    raise PolicyStop(f"지령 ↔ 실측 {lag:.0f}° — 걸렸거나 부딪힘")
                for side, _, _, gi in PARTS:
                    r.grip.set(side, float(np.clip(a[gi], pc.Q_LO, pc.Q_HI)))
                self.state["step"] += 1
                if time.time() - t_start > cfg["min_run_s"]:  # 끝 판정: 지령이 양손 준비 자세에 머묾
                    near = all(pc.at_ready(r.mdl, qm, s) for s in pc.SIDES)
                    t_near = (t_near or time.time()) if near else None
                    if t_near and time.time() - t_near > cfg["hold_end_s"]:
                        raise PolicyStop(f"{END_OK} — 한 회 끝")
                time.sleep(max(0.0, nxt - time.time()))
                if time.time() - nxt > 0.3:
                    nxt = time.time()
        except PolicyStop as e:
            why = str(e)
        except Exception as e:  # noqa: BLE001
            why = f"오류 {type(e).__name__}: {e}"
        finally:
            if tok:
                try:
                    pc.call("/stream/end", {"token": tok}, timeout=2.0)
                except Exception:  # noqa: BLE001
                    pass
            self.stop_evt.set()
            run_no = int(self.state.get("run_no", 1))
            self.state.update(running=False, why_end=why, phase="", wait_s=0.0)
            r.say(f"정책 끝 — {why} (팔은 그 자리, 그리퍼는 그 상태)")
            if getattr(self, "repeat", False) and not getattr(self, "user_stop", False):
                if why.startswith(END_OK) and run_no < int(self.cfg["repeat_max"]):
                    self.state.update(phase=f"다음 회 대기 ({self.cfg['repeat_gap_s']:.0f} 초)", waiting_next=True)
                    threading.Thread(target=self._next_run, args=(run_no + 1,), daemon=True).start()
                else:
                    self.state.update(why_end=why + (" · 반복 멈춤" if not why.startswith(END_OK) else f" · 반복 {run_no} 회 끝 (최대)"))
