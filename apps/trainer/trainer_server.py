#!/usr/bin/env python3
# Version: 2.0
"""
trainer_server.py — LeRobot 학습 웹 (포트 50100). 미니 PC · 학습 PC (5090) 같은 코드.

  ./apps/trainer/start_trainer.sh        → http://<PC>:50100/   (접근 키 없음 — LAN 전용)

흐름 (데이터셋 · 학습 · 모델은 서로 독립)
  데이터셋  LeRobot v3.0 — 이 PC 에서 수집한 것 (data/datasets/<이름>/lerobot) 또는 올린 zip (v3, 예전 형식 zip 은 v3 로 바꿔 넣음)
  학습      lerobot-train (lerobot 가상환경, 정책 종류는 lerobot 에 있는 것 아무거나) — 한 번에 하나, 나머지는 큐
  모델      마지막 체크포인트 pretrained_model + h2_model.json → model zip (다운로드) · 이 PC 가 수집 PC 면 data/policies/<이름> 에도 저장

보안: LAN 전용. 업로드 zip 은 경로 검사 후 정해진 파일만 풂.
"""
import fcntl
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import sys
import threading
import time
import zipfile

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron import paths  # noqa: E402
from pitron import ports  # noqa: E402
from pitron.learning import lerobot_env  # noqa: E402

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("TRAINER_PORT", ports.PORTS["trainer"]))
DATA = os.environ.get("TRAINER_DATA") or paths.data_dir("trainer", legacy="trainer/data")   # data/trainer (git 제외)
DS_DIR = os.path.join(DATA, "datasets")          # 올린 데이터셋 (LeRobot v3 폴더)
JOB_DIR = os.path.join(DATA, "jobs")
UP_DIR = os.path.join(DATA, "uploads")
for d in (DS_DIR, JOB_DIR, UP_DIR):
    os.makedirs(d, exist_ok=True)
# 같은 PC 에서 수집한 데이터셋 (data/datasets/<이름>/lerobot) — 업로드 없이 바로 학습. 없으면 (5090) 무시
COLLECT_DIR = os.environ.get("TRAINER_COLLECT") or paths.data_dir("datasets", legacy="gripper/data")
if not os.path.isdir(COLLECT_DIR):
    COLLECT_DIR = None
MODEL_STORE = paths.data_dir("policies", legacy="gripper/models")
LEROBOT_PY = os.environ.get("LEROBOT_PY") or lerobot_env.train_python()     # 학습 환경 lerobot_train (XPU · CUDA)
NAME_RE = re.compile(r"^(?!\.+$)[A-Za-z0-9_.\-]{1,64}$")          # "." · ".." 같은 점만 있는 이름은 안 됨
V3_FILE_RE = re.compile(r"^(meta/[A-Za-z0-9_./\-]+\.(json|jsonl|parquet)|data/chunk-\d+/file-\d+\.parquet|"
                        r"videos/[A-Za-z0-9_.\-]+/chunk-\d+/file-\d+\.mp4)$")
OLD_FILE_RE = re.compile(r"^(info\.json|episodes/ep_\d{6}/(episode\.npz|meta\.json))$")
HUB_POLICIES = {"smolvla", "pi0", "pi05", "pi0_fast", "groot", "xvla", "wall_x"}   # 사전학습 바탕 모델을 Hub 에서 받는 정책
DEFAULT_TRAIN = {
    "name": "", "policy": "act", "pretrained": "", "steps": 40000, "batch_size": 8,
    "device": os.environ.get("TRAINER_DEVICE", "auto"), "save_freq": 5000,
    "num_workers": int(os.environ.get("TRAINER_NUM_WORKERS", "4")), "image_aug": False, "seed": 1000, "extra": "",
}
TRAIN_THREADS = os.environ.get("TRAINER_THREADS")      # 학습 프로세스 CPU 연산 스레드 수 (없으면 torch 기본 = 코어 수)


def log(msg):
    print(f"[trainer] {msg}", flush=True)


# =====================================================================
# lerobot 환경 (서버는 lerobot 을 import 하지 않음 — 한 번 물어봄)
# =====================================================================
PROBE = {"done": False, "ok": False, "err": "", "torch": None, "lerobot": None, "cuda": None, "xpu": None, "policies": []}
PROBE_CODE = r"""
import json, torch, lerobot, lerobot.policies  # 정책 설정 등록
from lerobot.configs.policies import PreTrainedConfig
x = getattr(torch, "xpu", None)
print("@@" + json.dumps({"torch": torch.__version__, "lerobot": lerobot.__version__,
  "cuda": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
  "xpu": x.get_device_name(0) if x is not None and x.is_available() else None,
  "policies": sorted(PreTrainedConfig.get_known_choices())}))
"""


def probe():
    if not os.path.isfile(LEROBOT_PY):
        PROBE.update(done=True, ok=False, err=lerobot_env.missing_msg(LEROBOT_PY, train=True))
        return
    try:
        out = subprocess.run([LEROBOT_PY, "-c", PROBE_CODE], capture_output=True, text=True, timeout=180)
        ln = next((x for x in out.stdout.splitlines() if x.startswith("@@")), None)
        if ln is None:
            PROBE.update(done=True, ok=False, err=(out.stderr.strip().splitlines() or ["?"])[-1][-300:])
            return
        PROBE.update(json.loads(ln[2:]), done=True, ok=True, err="")
        log(f"lerobot {PROBE['lerobot']} · torch {PROBE['torch']} · CUDA {PROBE['cuda']} · XPU {PROBE['xpu']}")
    except Exception as e:  # noqa: BLE001
        PROBE.update(done=True, ok=False, err=f"{type(e).__name__}: {e}")


def best_device():
    return "cuda" if PROBE.get("cuda") else "xpu" if PROBE.get("xpu") else "cpu"


# =====================================================================
# 데이터셋 (LeRobot v3)
# =====================================================================
def ds_root(name):
    """올린 데이터셋 → 없으면 이 PC 에서 수집한 데이터셋의 lerobot/."""
    p = os.path.join(DS_DIR, name)
    if os.path.isfile(os.path.join(p, "meta", "info.json")) or COLLECT_DIR is None:
        return p
    return os.path.join(COLLECT_DIR, name, "lerobot")


def _read(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def ds_summary(name):
    root = ds_root(name)
    info = _read(os.path.join(root, "meta", "info.json")) or {}
    h2 = _read(os.path.join(root, "meta", "h2.json")) or {}
    feats = info.get("features", {})
    cams = {k.split(".", 2)[-1]: f"{v['shape'][1]}×{v['shape'][0]}" for k, v in feats.items()
            if v.get("dtype") in ("video", "image") and len(v.get("shape", [])) == 3}
    size = sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(root) for f in fs) if os.path.isdir(root) else 0
    fps = info.get("fps") or 30
    return {"name": name, "version": info.get("codebase_version"), "episodes": info.get("total_episodes", 0),
            "frames": info.get("total_frames", 0), "minutes": round(info.get("total_frames", 0) / fps / 60, 1), "fps": fps,
            "cameras": cams, "state_dim": (feats.get("observation.state", {}).get("shape") or [None])[0],
            "task": h2.get("task"), "fit": h2.get("fit"), "robot": info.get("robot_type"),
            "size_mb": round(size / 1e6, 1), "source": "업로드" if root.startswith(DS_DIR + os.sep) else "수집",
            "updated": time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(root))) if os.path.isdir(root) else ""}


def ds_names():
    names = {n for n in os.listdir(DS_DIR) if NAME_RE.match(n) and os.path.isfile(os.path.join(DS_DIR, n, "meta", "info.json"))}
    if COLLECT_DIR:
        names |= {n for n in os.listdir(COLLECT_DIR) if NAME_RE.match(n)
                  and os.path.isfile(os.path.join(COLLECT_DIR, n, "lerobot", "meta", "info.json"))}
    return sorted(names)


def _free_name(stem):
    stem = re.sub(r"[^A-Za-z0-9_.\-]", "_", stem)[:56].strip(".") or "dataset"
    name, k = stem, 2
    while os.path.exists(os.path.join(DS_DIR, name)):
        name, k = f"{stem}_{k}", k + 1
    return name


def extract_dataset(zpath, filename):
    """zip (<이름>/meta/info.json … = LeRobot v3, 또는 <이름>/info.json … = 예전 수집 형식) → DS_DIR/<이름> (v3)."""
    try:
        z = zipfile.ZipFile(zpath)
    except zipfile.BadZipFile:
        raise HTTPException(400, "zip 아님")
    with z:
        files = [i for i in z.infolist() if not i.is_dir()]
        roots = {i.filename.split("/", 1)[0] for i in files}
        if len(roots) != 1:
            raise HTTPException(400, "zip 안에 폴더 하나 (<데이터셋 이름>/...) 여야 함 — 수집 화면에서 받은 zip 그대로 올리세요")
        root = roots.pop()
        rels = []
        for i in files:
            n = i.filename
            if n.startswith("/") or ".." in n.split("/"):
                raise HTTPException(400, f"zip 안 경로 이상: {n}")
            rels.append((i, n[len(root) + 1:]))
        v3 = any(r == "meta/info.json" for _, r in rels)
        old = any(r == "info.json" for _, r in rels)
        if not v3 and not old:
            raise HTTPException(400, "LeRobot v3 데이터셋 (meta/info.json) 이 아님")
        keep = [(i, r) for i, r in rels if (V3_FILE_RE if v3 else OLD_FILE_RE).match(r)]
        need = sum(i.file_size for i, _ in keep)
        free = shutil.disk_usage(DS_DIR).free
        if need * (2 if old else 1) > free - 2e9:
            raise HTTPException(507, f"디스크 부족 (필요 {need / 1e9:.1f} GB, 남음 {free / 1e9:.1f} GB)")
        name = _free_name(root[:-len("_lerobot")] if root.endswith("_lerobot") else root)
        tmp = tempfile.mkdtemp(prefix=f".{name}.", dir=UP_DIR)          # 올리기마다 따로 (동시에 올려도 안 겹침)
        for i, rel in keep:
            dst = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with z.open(i) as src, open(dst, "wb") as out:
                shutil.copyfileobj(src, out, 1 << 20)
    try:
        if v3:
            inf = _read(os.path.join(tmp, "meta", "info.json")) or {}
            if inf.get("codebase_version") != "v3.0":
                raise HTTPException(400, f"LeRobot 데이터셋 {inf.get('codebase_version')} — v3.0 만 (lerobot 0.4.x)")
            os.replace(tmp, os.path.join(DS_DIR, name))
        else:                                                    # 예전 형식 → v3 (lerobot 환경에서 변환)
            if not os.path.isfile(LEROBOT_PY):
                raise HTTPException(409, lerobot_env.missing_msg(LEROBOT_PY, train=True))
            log(f"예전 형식 데이터셋 → LeRobot v3 변환: {name}")
            r = subprocess.run([LEROBOT_PY, "-u", "-m", "pitron.learning.lerobot_sync", tmp, "--out", os.path.join(DS_DIR, name),
                                "--repo-id", f"local/{name}"], cwd=_ROOT, capture_output=True, text=True,
                               env={**os.environ, "HF_HUB_OFFLINE": "1"})
            if r.returncode != 0:
                tail = (r.stdout + r.stderr).strip().splitlines()
                raise HTTPException(400, "v3 변환 실패: " + next((t for t in reversed(tail) if "❌" in t), tail[-1] if tail else "?")[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    s = ds_summary(name)
    log(f"데이터셋 올림 {name}: {s['episodes']} ep, {s['frames']} 프레임, {s['size_mb']} MB ({filename})")
    return s


# =====================================================================
# 모델 (lerobot pretrained_model + h2_model.json)
# =====================================================================
def safetensors_ok(path):
    """가중치 파일이 끝까지 써졌는지 (헤더의 마지막 offset = 파일 크기) — 저장 도중 멈춘 체크포인트를 거름."""
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            n = int.from_bytes(f.read(8), "little")
            if n <= 0 or n > size:
                return False
            h = json.loads(f.read(n))
        end = max((v["data_offsets"][1] for k, v in h.items() if k != "__metadata__"), default=0)
        return size == 8 + n + end
    except (OSError, ValueError, KeyError, TypeError):
        return False


def last_checkpoint(out):
    """out/checkpoints/<step>/pretrained_model 중 끝까지 저장된 마지막 → (경로, step) 또는 (None, 0)."""
    ck = os.path.join(out, "checkpoints")
    steps = sorted((int(n), n) for n in os.listdir(ck) if n.isdigit()) if os.path.isdir(ck) else []
    for st, n in reversed(steps):
        p = os.path.join(ck, n, "pretrained_model")
        if (os.path.isfile(os.path.join(p, "config.json")) and _read(os.path.join(p, "config.json"))
                and safetensors_ok(os.path.join(p, "model.safetensors"))):
            return p, st
    return None, 0


def package(j):
    """마지막 체크포인트 → JOB_DIR/<id>/model/<이름>/ + <이름>.zip (+ 수집 PC 면 모델 저장소). → 이름"""
    d = os.path.join(JOB_DIR, j["id"])
    src, step = last_checkpoint(os.path.join(d, "out"))
    if src is None:
        raise RuntimeError("저장된 체크포인트 없음 (저장 주기 전에 끝남)")
    name = j["params"]["name"]
    mdir = os.path.join(d, "model", name)
    shutil.rmtree(mdir, ignore_errors=True)
    shutil.copytree(src, mdir)
    cfg = _read(os.path.join(mdir, "config.json")) or {}
    root = ds_root(j["dataset"])
    h2 = _read(os.path.join(root, "meta", "h2.json")) or {}
    meta = {"format": "h2-lerobot-model-v1", "name": name, "policy": cfg.get("type"), "step": step,
            "steps_planned": j["params"]["steps"], "dataset": j["dataset"], "dataset_episodes": ds_summary(j["dataset"])["episodes"],
            "task": h2.get("task"), "fit": h2.get("fit"), "cameras": h2.get("cameras"), "state_names": h2.get("state_names"),
            "fps": h2.get("fps"), "device": j.get("device"), "lerobot": PROBE.get("lerobot"), "torch": PROBE.get("torch"),
            "loss": j.get("loss"), "trained": time.strftime("%Y-%m-%dT%H:%M:%S"), "job": j["id"]}
    with open(os.path.join(mdir, "h2_model.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    zp = os.path.join(d, f"{name}.zip")
    with zipfile.ZipFile(zp + ".tmp", "w", zipfile.ZIP_STORED) as z:
        for fn in sorted(os.listdir(mdir)):
            z.write(os.path.join(mdir, fn), f"{name}/{fn}")
    os.replace(zp + ".tmp", zp)
    j["zip"], j["model_step"] = os.path.basename(zp), step
    if COLLECT_DIR:                                              # 수집 PC (미니 PC) → 추론에서 바로 고름
        dst = os.path.join(MODEL_STORE, name)
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(mdir, dst)
        j["saved_model"] = name
        log(f"모델 저장소에 저장: {dst}")
    return name


# =====================================================================
# 작업 (큐 + lerobot-train 서브프로세스)
# =====================================================================
TQDM_RE = re.compile(r"(\d+)/(\d+) \[([\d:]+)<([\d:?]+)(?:,\s*([\d.]+)(s/step|step/s|s/it|it/s))?")
LOSS_RE = re.compile(r"\bloss:([\d.eE+\-]+)")
CKPT_RE = re.compile(r"Checkpoint policy after step (\d+)")


def _sec(hms):
    try:
        p = [int(x) for x in hms.split(":")]
    except ValueError:
        return None
    s = 0
    for x in p:
        s = s * 60 + x
    return s


class Jobs:
    def __init__(self):
        self.lk = threading.Lock()
        self.jobs = {}
        self.proc = None
        for n in sorted(os.listdir(JOB_DIR)):
            j = _read(os.path.join(JOB_DIR, n, "job.json"))
            if not j or "params" not in j or "policy" not in j["params"]:
                continue                                         # 예전 ACT 학습기 작업은 목록에서 뺌 (폴더는 그대로)
            if j["status"] in ("running", "canceling", "finishing"):
                j["status"], j["error"] = "interrupted", "서버가 학습 중에 꺼짐"
            self.jobs[j["id"]] = j
            self._save(j)
        threading.Thread(target=self._worker, daemon=True).start()

    def _save(self, j):
        d = os.path.join(JOB_DIR, j["id"])
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "job.json.tmp"), "w", encoding="utf-8") as f:
            json.dump(j, f, ensure_ascii=False)
        os.replace(os.path.join(d, "job.json.tmp"), os.path.join(d, "job.json"))

    def add(self, dataset, params):
        if not NAME_RE.match(dataset or "") or not os.path.isfile(os.path.join(ds_root(dataset), "meta", "info.json")):
            raise HTTPException(404, f"데이터셋 없음 (LeRobot v3): {dataset}")
        tc = dict(DEFAULT_TRAIN)
        for k, v in (params or {}).items():
            if k in DEFAULT_TRAIN and v not in (None, ""):
                tc[k] = bool(v) if isinstance(DEFAULT_TRAIN[k], bool) else type(DEFAULT_TRAIN[k])(v)
        if PROBE["policies"] and not tc["pretrained"] and tc["policy"] not in PROBE["policies"]:
            raise HTTPException(400, f"정책 종류 {tc['policy']} — lerobot 에 있는 것: {', '.join(PROBE['policies'])}")
        if tc["device"] not in ("auto", "cuda", "xpu", "cpu"):
            raise HTTPException(400, "학습 장치: auto · cuda · xpu · cpu")
        try:
            shlex.split(tc["extra"])
        except ValueError as e:
            raise HTTPException(400, f"추가 옵션 따옴표 이상: {e}")
        if tc["steps"] < 1 or tc["batch_size"] < 1 or tc["save_freq"] < 1:
            raise HTTPException(400, "step · batch · 저장 주기는 1 이상")
        jid = time.strftime("j%m%d_%H%M%S")
        while jid in self.jobs:
            jid += "x"
        tc["name"] = re.sub(r"[^A-Za-z0-9_.\-]", "_", tc["name"] or "")[:60].strip(".") or f"{tc['policy']}_{dataset}_{time.strftime('%m%d_%H%M')}"
        j = {"id": jid, "dataset": dataset, "params": tc, "status": "queued", "created": time.time(),
             "started": None, "finished": None, "step": 0, "total": tc["steps"], "loss": None, "it_s": None, "eta_s": None,
             "ckpt_step": 0, "zip": None, "model_step": 0, "saved_model": None, "curve": [], "error": "", "device": None}
        with self.lk:
            self.jobs[jid] = j
            self._save(j)
        log(f"작업 추가 {jid} ({dataset}, {tc['policy']}, {tc['steps']} step)")
        return j

    def _worker(self):
        while True:
            time.sleep(1.0)
            with self.lk:
                q = sorted((j for j in self.jobs.values() if j["status"] == "queued"), key=lambda j: j["created"])
                if not q or self.proc is not None or not PROBE["done"]:
                    continue
                j = q[0]
                j["status"], j["started"] = "running", time.time()
            try:
                self._run(j)
            except Exception as e:   # noqa: BLE001
                j["status"], j["error"] = "failed", f"{type(e).__name__}: {e}"
            finally:
                j["finished"] = time.time()
                self.proc = None
                with self.lk:
                    self._save(j)
                log(f"작업 {j['id']} → {j['status']} {j['error']}")

    def cmd(self, j):
        p = j["params"]
        dev = best_device() if p["device"] == "auto" else p["device"]
        j["device"] = dev
        out = os.path.join(JOB_DIR, j["id"], "out")
        shutil.rmtree(out, ignore_errors=True)                     # lerobot-train 은 out 이 없어야 새로 시작
        c = [LEROBOT_PY, "-u", "-m", "lerobot.scripts.lerobot_train",
             f"--dataset.repo_id=local/{j['dataset']}", f"--dataset.root={ds_root(j['dataset'])}",
             f"--dataset.image_transforms.enable={'true' if p['image_aug'] else 'false'}"]
        if p["pretrained"]:                                         # 이어 학습 · 미세 조정: 모델 저장소 이름 또는 Hub id
            local = os.path.join(MODEL_STORE, p["pretrained"])
            c.append(f"--policy.path={local if os.path.isfile(os.path.join(local, 'config.json')) else p['pretrained']}")
        else:
            c.append(f"--policy.type={p['policy']}")
        c += [f"--policy.device={dev}", "--policy.push_to_hub=false", f"--batch_size={p['batch_size']}", f"--steps={p['steps']}",
              f"--save_freq={min(p['save_freq'], p['steps'])}", f"--log_freq={max(1, min(50, p['steps'] // 20))}",
              f"--num_workers={p['num_workers']}", f"--seed={p['seed']}", f"--output_dir={out}", f"--job_name={p['name']}",
              "--wandb.enable=false"]
        return c + shlex.split(p["extra"])

    def _run(self, j):
        d = os.path.join(JOB_DIR, j["id"])
        os.makedirs(d, exist_ok=True)
        cmd = self.cmd(j)
        logf = open(os.path.join(d, "train.log"), "a", encoding="utf-8")
        logf.write("$ " + " ".join(shlex.quote(x) for x in cmd) + "\n")
        p = j["params"]
        hub = bool(p["pretrained"] and not os.path.isdir(os.path.join(MODEL_STORE, p["pretrained"]))) or p["policy"] in HUB_POLICIES
        env = {**os.environ, "PYTHONUNBUFFERED": "1", "TQDM_MININTERVAL": "2"}
        if not hub:
            env["HF_HUB_OFFLINE"] = "1"                         # 데이터셋 · 모델 모두 로컬 — 인터넷 안 씀
        if TRAIN_THREADS:
            env.update(OMP_NUM_THREADS=TRAIN_THREADS, MKL_NUM_THREADS=TRAIN_THREADS)
        # 이 PC 에서 수집한 데이터셋이면 학습하는 동안 공유 잠금 — 그동안 수집 쪽 v3 변환 (lerobot_sync, 배타 잠금) 은 학습이 끝날 때까지 기다림
        lock = None
        if COLLECT_DIR and not ds_root(j["dataset"]).startswith(DS_DIR + os.sep):
            lock = open(os.path.join(COLLECT_DIR, j["dataset"], ".lerobot.lock"), "a")
            fcntl.flock(lock, fcntl.LOCK_SH)
        # nice 15: 학습이 CPU 를 다 써도 SSH · 이 웹 · 다른 프로그램이 먼저 돌게
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                         start_new_session=True, cwd=_ROOT, env=env, preexec_fn=lambda: os.nice(15))
            self._pump(j, logf)
        finally:
            if lock is not None:
                fcntl.flock(lock, fcntl.LOCK_UN)
                lock.close()

    def _pump(self, j, logf):
        last_save, last_tq, tail = 0.0, 0.0, []
        for line in self.proc.stdout:                  # text 모드 → tqdm 의 \r 도 줄 바꿈으로 들어옴
            line = line.rstrip()
            if not line:
                continue
            # 진행 막대와 로그가 한 줄에 붙어 나올 수 있음 ("Training: 25%|…| 1/4 […]INFO … loss:…") → 나눠서 둘 다 봄
            k = line.find("INFO ") if "Training" in line else -1
            bar, rest = (line[:k], line[k:]) if k > 0 else ((line, "") if "Training" in line else ("", line))
            m = TQDM_RE.search(bar) if bar else None
            if m:
                j["step"], j["total"] = int(m.group(1)), int(m.group(2))
                j["eta_s"] = _sec(m.group(4)) if "?" not in m.group(4) else None
                if m.group(5):
                    v = float(m.group(5))
                    j["it_s"] = v if m.group(6) in ("step/s", "it/s") else (1.0 / v if v else None)
                if time.time() - last_tq > 30:                  # 진행 막대는 30 초마다 한 줄만 로그에
                    logf.write(bar[-160:] + "\n")
                    last_tq = time.time()
            elif bar:
                rest = line
            if rest:
                lm = LOSS_RE.search(rest)
                sm = re.search(r"\bstep:(\S+)", rest)
                if lm and sm:
                    j["loss"] = float(lm.group(1))
                    j["curve"].append([j["step"], j["loss"]])
                    if len(j["curve"]) > 1500:
                        j["curve"] = j["curve"][:-500:2] + j["curve"][-500:]
                cm = CKPT_RE.search(rest)
                if cm:
                    j["ckpt_step"] = int(cm.group(1))
                logf.write(rest + "\n")
                tail = (tail + [rest])[-30:]
            logf.flush()
            if time.time() - last_save > 2.0:
                with self.lk:
                    self._save(j)
                last_save = time.time()
        rc = self.proc.wait()
        logf.write(f"[exit {rc}]\n")
        logf.close()
        if j["status"] == "canceling":
            j["status"] = "canceled"
            return
        if rc != 0 and j["status"] != "finishing":
            j["status"] = "failed"
            j["error"] = next((ln for ln in reversed(tail) if re.search(r"Error|error|❌", ln)), tail[-1] if tail else f"exit {rc}")[-400:]
            return
        try:
            package(j)
            j["status"] = "done"
        except Exception as e:  # noqa: BLE001
            j["status"], j["error"] = "failed", f"모델 만들기 실패: {e}"

    def _stop(self, jid, status):
        with self.lk:
            j = self.jobs.get(jid)
            if not j:
                raise HTTPException(404, "작업 없음")
            if j["status"] == "queued" and status == "canceling":
                j["status"] = "canceled"
                self._save(j)
                return j
            if j["status"] != "running" or self.proc is None:
                raise HTTPException(409, f"상태 {j['status']}")
            if status == "finishing" and not j.get("ckpt_step"):
                raise HTTPException(409, "아직 저장된 체크포인트 없음 — 저장 주기까지 기다리거나 취소")
            j["status"] = status
            os.killpg(self.proc.pid, signal.SIGTERM)
        return j

    def cancel(self, jid):
        return self._stop(jid, "canceling")

    def finish(self, jid):
        """지금 마치기 = 학습을 멈추고 마지막 체크포인트로 모델을 만듦."""
        return self._stop(jid, "finishing")

    def delete(self, jid):
        with self.lk:
            j = self.jobs.get(jid)
            if not j:
                raise HTTPException(404, "작업 없음")
            if j["status"] in ("running", "canceling", "finishing"):
                raise HTTPException(409, "실행 중 — 먼저 취소")
            del self.jobs[jid]
        shutil.rmtree(os.path.join(JOB_DIR, jid), ignore_errors=True)


jobs = Jobs()
threading.Thread(target=probe, daemon=True).start()


# =====================================================================
# FastAPI
# =====================================================================
app = FastAPI(title="pitron trainer")


@app.get("/", response_class=HTMLResponse)
def index():
    with open(os.path.join(HERE, "trainer_web.html"), encoding="utf-8") as f:
        return ports.with_home(f.read())


_info_cache = {"t": 0, "v": None}


@app.get("/api/info")
def api_info():
    if time.time() - _info_cache["t"] < 2.0 and _info_cache["v"]:
        return _info_cache["v"]
    gpu = None
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu,temperature.gpu",
                              "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=3).stdout
        n, mu, mt, u, t = [x.strip() for x in out.strip().splitlines()[0].split(",")]
        gpu = {"name": n, "mem_used": int(mu), "mem_total": int(mt), "util": int(u), "temp": int(t)}
    except Exception:   # noqa: BLE001
        if PROBE.get("xpu"):
            gpu = {"name": PROBE["xpu"], "xpu": True}
    du = shutil.disk_usage(DATA)
    v = {"gpu": gpu, "lerobot_py": LEROBOT_PY, "probe": PROBE, "device_auto": best_device() if PROBE["done"] else None,
         "python": sys.version.split()[0], "disk_free_gb": round(du.free / 1e9, 1), "defaults": DEFAULT_TRAIN,
         "collect_pc": COLLECT_DIR is not None, "models": sorted(n for n in os.listdir(MODEL_STORE)
                                                                  if os.path.isfile(os.path.join(MODEL_STORE, n, "config.json")))
         if os.path.isdir(MODEL_STORE) else [],
         "queue": {"running": sum(j["status"] == "running" for j in jobs.jobs.values()),
                   "queued": sum(j["status"] == "queued" for j in jobs.jobs.values())}}
    _info_cache.update(t=time.time(), v=v)
    return v


@app.get("/api/datasets")
def api_datasets():
    return [ds_summary(n) for n in ds_names()]


@app.put("/api/datasets/upload")
async def api_ds_upload(request: Request, filename: str = "dataset.zip"):
    tmp = os.path.join(UP_DIR, f"up_{int(time.time() * 1000)}.zip")
    n = 0
    try:
        with open(tmp, "wb") as f:
            async for chunk in request.stream():
                f.write(chunk)
                n += len(chunk)
        if n == 0:
            raise HTTPException(400, "빈 파일")
        return await run_in_threadpool(extract_dataset, tmp, filename)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


@app.delete("/api/datasets/{name}")
def api_ds_delete(name: str):
    if not NAME_RE.match(name):
        raise HTTPException(400, "이름")
    if any(j["dataset"] == name and j["status"] in ("queued", "running") for j in jobs.jobs.values()):
        raise HTTPException(409, "이 데이터셋으로 학습 대기/중인 작업이 있음")
    p = os.path.join(DS_DIR, name)
    if not os.path.isdir(p):
        if COLLECT_DIR and os.path.isdir(os.path.join(COLLECT_DIR, name)):
            raise HTTPException(409, "이 PC 에서 수집한 데이터셋 — 지우기는 수집 화면에서")
        raise HTTPException(404, "없음")
    shutil.rmtree(p)
    return {"ok": True}


def _brief(j):
    return {k: v for k, v in j.items() if k != "curve"}


@app.get("/api/jobs")
def api_jobs():
    return sorted((_brief(j) for j in jobs.jobs.values()), key=lambda j: -j["created"])


@app.post("/api/jobs")
def api_job_add(body: dict):
    return _brief(jobs.add(body.get("dataset"), body.get("params")))


@app.get("/api/jobs/{jid}")
def api_job(jid: str):
    j = jobs.jobs.get(jid)
    if not j:
        raise HTTPException(404, "작업 없음")
    return j


@app.get("/api/jobs/{jid}/log", response_class=PlainTextResponse)
def api_job_log(jid: str, tail: int = 300):
    if jid not in jobs.jobs:
        raise HTTPException(404, "작업 없음")
    p = os.path.join(JOB_DIR, jid, "train.log")
    if not os.path.exists(p):
        return ""
    with open(p, encoding="utf-8", errors="replace") as f:
        return "".join(f.readlines()[-max(1, min(tail, 5000)):])


@app.post("/api/jobs/{jid}/cancel")
def api_job_cancel(jid: str):
    return _brief(jobs.cancel(jid))


@app.post("/api/jobs/{jid}/finish")
def api_job_finish(jid: str):
    return _brief(jobs.finish(jid))


@app.delete("/api/jobs/{jid}")
def api_job_delete(jid: str):
    jobs.delete(jid)
    return {"ok": True}


@app.get("/api/jobs/{jid}/model.zip")
def api_job_zip(jid: str):
    j = jobs.jobs.get(jid)
    if not j or not j.get("zip"):
        raise HTTPException(404, "model zip 없음")
    p = os.path.join(JOB_DIR, jid, os.path.basename(j["zip"]))
    if not os.path.exists(p):
        raise HTTPException(404, "파일 없음")
    return FileResponse(p, media_type="application/zip", filename=os.path.basename(p))


if __name__ == "__main__":
    log(f"시작 — http://0.0.0.0:{PORT}/   데이터 {DATA}   lerobot python {LEROBOT_PY}")
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
