"""
policy.py — 그리퍼 ACT 정책 실행 (잡기 웹 pick_web 이 씀): model zip → OpenVINO → 30 Hz 추론 → 팔 /stream + 그리퍼

  model zip (trainer, format h2-gripper-act-v1): <이름>/policy.xml · policy.bin · meta.json
    입력 state [1,8] (side 손 7 관절 + 그리퍼 q, rad) · image_<카메라> [1,3,H,W] 0~1 RGB → action [1,chunk,8] rad
    양손 모델 (side both, 오른손 · 왼손 에피소드를 한 데이터셋으로 학습): state · action 16 = [왼팔 7, 왼 그리퍼, 오른팔 7, 오른 그리퍼]
  저장: gripper/models/<이름>/ (.gitignore)

  한 틱 (30 Hz)
    영상: 녹화 때와 같게 — 카메라 크기로 줄임 (INTER_AREA) → JPEG q90 로 한 번 눌렀다 풂 (학습 데이터와 같은 화질) → RGB 0~1
    state: rt/lowstate 팔 실측 7 (side) + 그리퍼 q 실측
    추론 → chunk 예측을 쌓아 시점이 같은 예측끼리 가중 평균 (ACT temporal ensemble, w = exp(−k·i), i = 0 이 가장 오래된 예측)
    안전 필터 → 팔 14 축 목표 (다른 팔은 시작 때 목표 그대로) /stream/cmd · 그리퍼 목표 set
  멈춤 (팔은 그 자리, 그리퍼는 그 상태 유지)
    · 정책 출력이 학습 데이터 범위(meta stats action_min/max)를 range_tol 넘게 벗어남 — 범위 안은 그대로, 약간 넘는 건 범위로 자름
    · 집게 가운데(지령 관절 FK)가 학습 데이터 범위(tcp_min/max) ± tcp_tol 밖
    · 그리퍼 가장 낮은 점이 테이블 평면 + floor_clear 아래 (테이블을 쟀을 때)
    · 지령 ↔ 실측 관절 차이 > track_abort_deg (arm_server 응답)
    · 최대 시간 · 끝 자세(meta end_pose) 근처에 hold_end_s 머묾 (= 한 회 끝) · [정지]
"""
import json
import math
import os
import re
import shutil
import threading
import time
import zipfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "models")
MODEL_FORMAT = "h2-gripper-act-v1"
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,80}$")
FPS = 30.0

DEFAULTS = {
    "ensemble_k": 0.01,      # temporal ensemble 가중 (ACT 논문 값)
    "max_s": 90.0,           # 최대 실행 시간 [s]
    "range_tol": 0.15,       # 학습 범위 밖 허용 [rad] — 넘으면 멈춤 (안쪽은 범위로 자름)
    "tcp_tol": 0.05,         # 집게 가운데 학습 범위 밖 허용 [m]
    "floor_clear": 0.005,    # 그리퍼 ↔ 테이블 최소 [m]
    "track_abort_deg": 15.0,
    "server_max_vel": 1.0,   # arm_server /stream 관절 속도 상한 [rad/s]
    "end_tol_deg": 4.0,      # 끝 자세(end_pose) 와 이 안에 hold_end_s 머물면 끝
    "hold_end_s": 1.0,
    "min_run_s": 8.0,        # 시작 직후는 끝 판정 안 함 (시작 자세 = 끝 자세 근처라서)
    "start_tol_deg": 3.0,    # 시작 전 팔이 start_pose 와 이보다 다르면 먼저 천천히 맞춤
    "repeat_gap_s": 2.0,     # 반복 실행: 한 회가 정상으로 끝난 뒤 다음 회까지 쉬는 시간 [s]
    "repeat_max": 20,        # 반복 실행: 최대 회수 (안전)
}
END_OK = "끝 자세 도착"     # 이 이유로 끝난 회만 다음 회로 이어감 (안전 필터 · 정지 · 오류 · 최대 시간은 멈춤)


# ---------------- 모델 파일 ----------------
def models():
    out = []
    if not os.path.isdir(MODEL_DIR):
        return out
    for n in sorted(os.listdir(MODEL_DIR)):
        mp = os.path.join(MODEL_DIR, n, "meta.json")
        if not os.path.isfile(mp):
            continue
        try:
            with open(mp, encoding="utf-8") as f:
                m = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        out.append({"name": n, "format": m.get("format"), "side": m.get("side"), "dataset": m.get("dataset"),
                    "episodes": m.get("episodes"), "steps": m.get("steps"), "val_err_deg": m.get("val_err_deg"),
                    "cameras": [c["name"] for c in (m.get("cameras") or [])], "created": m.get("created"),
                    "openvino": m.get("openvino")})
    return out


def install_zip(zpath):
    """model zip → MODEL_DIR/<이름>. 정해진 파일만 풀고 format 확인. → 이름"""
    with zipfile.ZipFile(zpath) as z:
        files = [i for i in z.infolist() if not i.is_dir()]
        roots = {i.filename.split("/", 1)[0] for i in files}
        if len(roots) != 1:
            raise ValueError("zip 안에 폴더 하나 (<모델 이름>/policy.xml …) 여야 함 — 학습 PC 웹에서 받은 zip 그대로")
        root = roots.pop()
        if not NAME_RE.match(root):
            raise ValueError(f"모델 이름 이상: {root}")
        want = {f"{root}/policy.xml", f"{root}/policy.bin", f"{root}/meta.json"}
        have = {i.filename for i in files}
        if not want <= have:
            raise ValueError(f"zip 에 {sorted(want - have)} 없음")
        meta = json.loads(z.read(f"{root}/meta.json"))
        if meta.get("format") != MODEL_FORMAT:
            raise ValueError(f"그리퍼 모델 아님 (format {meta.get('format')})")
        dst = os.path.join(MODEL_DIR, root)
        tmp = dst + ".tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        for fn in ("policy.xml", "policy.bin", "meta.json"):
            with z.open(f"{root}/{fn}") as src, open(os.path.join(tmp, fn), "wb") as out:
                shutil.copyfileobj(src, out)
        shutil.rmtree(dst, ignore_errors=True)
        os.replace(tmp, dst)
    return root


# ---------------- 정책 ----------------
class Policy:
    """OpenVINO 로 컴파일된 그리퍼 ACT."""

    def __init__(self, name, device="CPU"):
        import openvino as ov
        d = os.path.join(MODEL_DIR, name)
        with open(os.path.join(d, "meta.json"), encoding="utf-8") as f:
            self.meta = json.load(f)
        if self.meta.get("format") != MODEL_FORMAT:
            raise ValueError(f"그리퍼 모델 아님: {self.meta.get('format')}")
        self.name, self.device = name, device
        self.side = self.meta["side"]
        # 손별 (팔 14 축 중 자리, 출력 action 중 팔 7 · 그리퍼 자리, tcp 3 자리)
        #   한 손 모델: state/action 8 = [팔 7, 그리퍼]. 양손 모델 (both): 16 = [왼팔 7, 왼 그리퍼, 오른팔 7, 오른 그리퍼], tcp 6
        if self.side == "left":
            self.parts = [("left", slice(0, 7), slice(0, 7), 7, slice(0, 3))]
        elif self.side == "right":
            self.parts = [("right", slice(7, 14), slice(0, 7), 7, slice(0, 3))]
        elif self.side == "both":
            self.parts = [("left", slice(0, 7), slice(0, 7), 7, slice(0, 3)),
                          ("right", slice(7, 14), slice(8, 15), 15, slice(3, 6))]
        else:
            raise ValueError(f"모델 side 이상: {self.side}")
        self.cams = self.meta["cameras"]
        self.chunk = int(self.meta["chunk_size"])
        self.dim = int(self.meta["action_dim"])
        st = self.meta["stats"]
        self.a_min, self.a_max = np.asarray(st["action_min"]), np.asarray(st["action_max"])
        self.tcp_min = np.asarray(st["tcp_min"]) if "tcp_min" in st else None
        self.tcp_max = np.asarray(st["tcp_max"]) if "tcp_max" in st else None
        self.start_pose = np.asarray(self.meta["start_pose"], dtype=float)
        self.end_pose = np.asarray(self.meta["end_pose"], dtype=float)
        core = ov.Core()
        t0 = time.time()
        self.comp = core.compile_model(os.path.join(d, "policy.xml"), device)
        self.compile_s = time.time() - t0
        self.out = self.comp.output(0)
        self.req = self.comp.create_infer_request()
        # 시험 추론 (시간)
        feed = self.blank_feed()
        t0 = time.time()
        for _ in range(3):
            self.req.infer(feed)
        self.infer_ms = (time.time() - t0) / 3 * 1000

    def blank_feed(self):
        f = {"state": self.start_pose[None].astype(np.float32)}
        for c in self.cams:
            f[f"image_{c['name']}"] = np.zeros((1, 3, int(c["h"]), int(c["w"])), np.float32)
        return f

    def prep(self, name, bgr):
        """녹화와 같은 화질: 카메라 크기로 줄임 → JPEG q90 → RGB 0~1 NCHW."""
        c = next(c for c in self.cams if c["name"] == name)
        if bgr.shape[1] != c["w"] or bgr.shape[0] != c["h"]:
            bgr = cv2.resize(bgr, (int(c["w"]), int(c["h"])), interpolation=cv2.INTER_AREA)
        ok, b = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            bgr = cv2.imdecode(b, cv2.IMREAD_COLOR)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0

    def infer(self, state, images):
        feed = {"state": np.asarray(state, np.float32)[None]}
        for c in self.cams:
            feed[f"image_{c['name']}"] = self.prep(c["name"], images[c["name"]])
        self.req.infer(feed)
        return np.asarray(self.req.get_output_tensor(0).data)[0].copy()     # (chunk, dim)


class Ensemble:
    """ACT temporal ensemble (추론과 제어를 따로): 예측은 '관측한 틱' t0 와 함께 add, 제어 틱 T 에서는 T 를 가리키는
    예측들 (t0 ≤ T < t0 + chunk) 을 exp(−k·i) 로 평균 (i = 0 이 가장 오래됨). 추론이 33 ms 보다 느려도 제어는 30 Hz."""

    def __init__(self, chunk, k):
        self.chunk, self.k, self.preds = chunk, k, []     # (t0, chunk 예측)
        self.lk = threading.Lock()

    def add(self, t0, pred):
        with self.lk:
            self.preds.append((t0, pred))

    def get(self, T):
        with self.lk:
            self.preds = [(t0, p) for t0, p in self.preds if T - t0 < self.chunk]
            use = [p[T - t0] for t0, p in self.preds if 0 <= T - t0 < self.chunk]
        if not use:
            return None
        acts = np.stack(use)
        w = np.exp(-self.k * np.arange(len(acts)))
        return (acts * w[:, None]).sum(0) / w.sum()


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

    def load(self, name, device):
        if self.running():
            raise RuntimeError("정책 실행 중")
        self.policy = None
        self.policy = Policy(name, device)
        return self.info()

    def info(self):
        p = self.policy
        if p is None:
            return None
        return {"name": p.name, "device": p.device, "side": p.side, "cameras": [c["name"] for c in p.cams],
                "chunk": p.chunk, "compile_s": round(p.compile_s, 2), "infer_ms": round(p.infer_ms, 1),
                "episodes": p.meta.get("episodes"), "steps": p.meta.get("steps"), "val_err_deg": p.meta.get("val_err_deg")}

    def running(self):
        return self.th is not None and self.th.is_alive()

    # ----- 시작 조건 -----
    def check(self):
        """→ 문제 목록 (빈 목록이면 시작 가능)."""
        r, p, bad = self.r, self.policy, []
        if p is None:
            return ["모델을 불러오지 않음"]
        if r.busy() or self.running():
            bad.append("다른 동작 중")
        if r.rec.active or r.rec.pending is not None:
            bad.append("녹화 중 · 저장 결정 대기")
        names = r.wrist.names() if r.wrist else []
        miss = [c["name"] for c in p.cams if c["name"] != "body" and c["name"] not in names]
        if miss:
            bad.append(f"카메라 없음: {miss}")
        q = self.meas_arm()
        if q is None:
            bad.append("팔 실측 없음 (rt/lowstate · arm_server)")
        gq = self.grip_q()
        if gq is None:
            bad.append("그리퍼 상태 없음")
        cp = r.cam_pitch.read() if r.cam_pitch else None
        if cp is not None and abs(cp["diff"]) > 1.0:
            bad.append(f"카메라 숙임 기준과 {cp['diff']:+.2f}° — 거치대를 맞추세요")
        if q is not None and gq is not None:
            st = self.side_state(q, gq)
            d = max(np.degrees(np.max(np.abs(st[asl] - p.start_pose[asl]))) for _, _, asl, _, _ in p.parts)
            if d > 25.0:
                bad.append(f"팔이 시작 자세(학습 데이터 첫 자세)와 {d:.0f}° 다름 — 먼저 [준비 자세로]")
        return bad

    def meas_arm(self):
        q = self.r.cam_pitch.arm_now() if self.r.cam_pitch else None
        return q if q is not None else self.r.arm_q

    def grip_q(self):
        """→ {손: 그리퍼 q 실측} (모델이 쓰는 손 전부) 또는 None"""
        out = {}
        for side, *_ in self.policy.parts:
            g = self.r.grip.state(side)
            if g is None or g.get("q") is None:
                return None
            out[side] = g["q"]
        return out

    def side_state(self, q14, gq):
        """팔 14 (제어 순서) + {손: 그리퍼 q} → 모델 state (한 손 8 / 양손 16, 녹화 순서와 같음)"""
        return np.concatenate([np.r_[q14[csl], gq[side]] for side, csl, _, _, _ in self.policy.parts])

    # ----- 실행 -----
    def start(self, repeat=False, run_no=1):
        """repeat: 한 회가 '끝 자세 도착' 으로 끝나면 repeat_gap_s 쉬고 다음 회를 자동으로 (시작 조건을 매번 다시 검사).
        안전 필터 · [정지] · 오류 · 최대 시간으로 끝나면 반복도 멈춤. 최대 repeat_max 회."""
        bad = self.check()
        if bad:
            raise RuntimeError("; ".join(bad))
        if run_no == 1:
            self.user_stop = False
        self.repeat = bool(repeat)
        self.stop_evt.clear()
        self.state = {"running": True, "step": 0, "infer_ms": None, "why_end": "", "phase": "시작 자세로",
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
        """반복: 쉬었다가 다음 회. 그 사이 [정지] · 시작 조건 실패면 멈춤."""
        r, cfg = self.r, self.cfg
        t_end = time.time() + float(cfg["repeat_gap_s"])
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

    def _infer_loop(self, ens):
        """최신 관측 → 추론 → ens.add(관측한 틱, 예측). 제어 루프와 따로 (추론이 느려도 제어는 30 Hz)."""
        r, p = self.r, self.policy
        try:
            while not self.stop_evt.is_set():
                q = self.meas_arm()
                gq = self.grip_q()
                img, _ = r.cam.frame()
                images = {"body": img}
                for c in p.cams:
                    if c["name"] != "body":
                        images[c["name"]] = r.wrist.frame(c["name"])[0]
                if q is None or gq is None or any(v is None for v in images.values()):
                    self.inf_err = "팔 · 그리퍼 실측 또는 카메라 영상 끊김"
                    return
                t_obs = self.T
                t0 = time.time()
                pred = p.infer(self.side_state(q, gq), images)
                self.state["infer_ms"] = round((time.time() - t0) * 1000, 1)
                ens.add(t_obs, pred)
        except Exception as e:  # noqa: BLE001
            self.inf_err = f"추론 오류 {type(e).__name__}: {e}"

    def _run(self):
        r, pc, p, cfg = self.r, self.pc, self.policy, self.cfg
        why = "끝"
        tok = None
        try:
            b = pc.call("/stream/begin", {"max_vel": float(cfg["server_max_vel"])})
            tok = b["token"]
            target = np.radians(np.asarray(b["target_deg"], dtype=float))      # 14 축, 다른 팔은 이 값 유지
            pc.LAST_CMD.update(q=target.copy(), t=time.time())
            r.say(f"정책 시작 {p.name} ({p.side}, {p.device}, 추론 {p.infer_ms:.0f} ms)")
            # 1) 시작 자세로 천천히 (학습 데이터 첫 지령 평균) — 모델이 쓰는 팔 전부
            q_start = target.copy()
            for _, csl, asl, _, _ in p.parts:
                q_start[csl] = p.start_pose[asl]
            d = float(np.max(np.abs(target - q_start)))
            if math.degrees(d) > cfg["start_tol_deg"]:
                sec = max(2.0, 1.5 * d / 0.3)
                q0 = target.copy()
                t0 = time.time()
                while True:
                    if self.stop_evt.is_set():
                        raise PolicyStop("정지")
                    s = min((time.time() - t0) / sec, 1.0)
                    s = s * s * (3 - 2 * s)
                    q = q0 + s * (q_start - q0)
                    rr = pc.call("/stream/cmd", {"token": tok, "deg": np.degrees(q).tolist()}, timeout=1.0)
                    pc.LAST_CMD.update(q=np.radians(np.asarray(rr["target_deg"])), t=time.time())
                    if s >= 1.0:
                        break
                    time.sleep(1 / FPS)
                target = np.radians(np.asarray(rr["target_deg"], dtype=float))
            for side, _, _, gi, _ in p.parts:
                r.grip.set(side, float(p.start_pose[gi]))
            # 2) 정책 — 추론 스레드가 최신 관측으로 계속 추론해 ens 에 넣고, 이 루프는 30 Hz 로 ens 에서 꺼내 보냄
            self.state["phase"] = "정책"
            ens = Ensemble(p.chunk, float(cfg["ensemble_k"]))
            self.T = 0
            self.inf_err = None
            inf_th = threading.Thread(target=self._infer_loop, args=(ens,), daemon=True)
            inf_th.start()
            t_start = time.time()
            t_near = None
            nxt = time.time()
            a = None
            while True:
                nxt += 1 / FPS
                if self.stop_evt.is_set():
                    raise PolicyStop("정지")
                if self.inf_err:
                    raise PolicyStop(self.inf_err)
                if time.time() - t_start > cfg["max_s"]:
                    raise PolicyStop(f"최대 시간 {cfg['max_s']:.0f} 초")
                a_new = ens.get(self.T)
                self.T += 1
                if a_new is None:
                    if a is None:                     # 첫 예측 전 — 지금 목표 유지
                        time.sleep(max(0.0, nxt - time.time()))
                        if time.time() - t_start > 5.0:
                            raise PolicyStop("5 초 동안 첫 예측 없음 (추론이 너무 느림)")
                        continue
                    raise PolicyStop("예측이 끊김 (추론이 chunk 보다 느림)")
                a = a_new
                # 안전 필터
                over = np.maximum(p.a_min - a, a - p.a_max)
                if np.max(over) > cfg["range_tol"]:
                    k = int(np.argmax(over))
                    raise PolicyStop(f"정책 출력이 학습 범위 밖 ({p.meta['joint_names'][k]} {math.degrees(over[k]):.0f}° 넘음)")
                a = np.clip(a, p.a_min, p.a_max)
                cmd = target.copy()
                for _, csl, asl, _, _ in p.parts:
                    cmd[csl] = a[asl]
                qm = r.mdl.to_model(cmd)
                for side, _, _, _, tsl in p.parts:        # 손마다: 학습 범위 · 테이블
                    T = r.mdl.tcp(qm, side)
                    if p.tcp_min is not None:
                        tcp = T.translation
                        if np.any(tcp < p.tcp_min[tsl] - cfg["tcp_tol"]) or np.any(tcp > p.tcp_max[tsl] + cfg["tcp_tol"]):
                            raise PolicyStop(f"{side} 집게 가운데 {np.round(tcp, 3).tolist()} 가 학습 범위 ± {cfg['tcp_tol'] * 100:.0f} cm 밖")
                    if r.table_plane is not None:
                        a0, bb, cc = r.table_plane
                        tz = a0 + bb * T.translation[0] + cc * T.translation[1]
                        low = r.mdl.lowest(qm, side)
                        if low < tz + cfg["floor_clear"]:
                            raise PolicyStop(f"{side} 그리퍼가 테이블에 {max(0.0, low - tz) * 1000:.0f} mm 까지 — 멈춤")
                rr = pc.call("/stream/cmd", {"token": tok, "deg": np.degrees(cmd).tolist()}, timeout=1.0)
                pc.LAST_CMD.update(q=np.radians(np.asarray(rr["target_deg"])), t=time.time())
                lag = float(np.max(np.abs(np.asarray(rr["target_deg"]) - np.asarray(rr["meas_deg"]))))
                if lag > cfg["track_abort_deg"]:
                    raise PolicyStop(f"지령 ↔ 실측 {lag:.0f}° — 걸렸거나 부딪힘")
                for side, _, _, gi, _ in p.parts:
                    r.grip.set(side, float(np.clip(a[gi], pc.Q_LO, pc.Q_HI)))
                self.state["step"] += 1
                # 끝 판정: 끝 자세 근처에 머묾
                if time.time() - t_start > cfg["min_run_s"]:
                    near = max(math.degrees(np.max(np.abs(a[asl] - p.end_pose[asl]))) for _, _, asl, _, _ in p.parts) < cfg["end_tol_deg"]
                    t_near = (t_near or time.time()) if near else None
                    if t_near and time.time() - t_near > cfg["hold_end_s"]:
                        raise PolicyStop("끝 자세 도착 — 한 회 끝")
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
            self.stop_evt.set()          # 추론 스레드도 끝냄
            run_no = int(self.state.get("run_no", 1))
            self.state.update(running=False, why_end=why, phase="")
            r.say(f"정책 끝 — {why} (팔은 그 자리, 그리퍼는 그 상태)")
            if getattr(self, "repeat", False) and not getattr(self, "user_stop", False):
                if why.startswith(END_OK) and run_no < int(self.cfg["repeat_max"]):
                    self.state.update(phase=f"다음 회 대기 ({self.cfg['repeat_gap_s']:.0f} 초)", waiting_next=True)
                    threading.Thread(target=self._next_run, args=(run_no + 1,), daemon=True).start()
                else:
                    self.state.update(why_end=why + (" · 반복 멈춤" if not why.startswith(END_OK) else f" · 반복 {run_no} 회 끝 (최대)"))
