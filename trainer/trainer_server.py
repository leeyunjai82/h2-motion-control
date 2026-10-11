#!/usr/bin/env python3
# Version: 1.0
"""
trainer_server.py — 학습 PC (RTX 5090, Ubuntu) ACT 학습 웹 (포트 50100)

  ./trainer/start_trainer.sh        → http://<학습 PC>:50100/   (처음 띄울 때 접근 키를 출력 — 웹에서 한 번 입력)

흐름
  1) 로봇 웹(50030)에서 받은 데이터셋 zip 업로드 → data/datasets/<이름>/ 에 풀고 검사
  2) 학습 설정 → 큐에 넣음 (한 번에 하나씩, GPU 하나)
  3) train_act.py 서브프로세스: 학습 → OpenVINO IR 변환 → model zip  (진행·loss·로그는 웹에 실시간)
  4) model zip 다운로드 → 로봇 웹 Models 에 업로드

보안: LAN 전용. /api/* 는 접근 키(X-Key 헤더 또는 ?key=) 필요. 키 = 환경변수 TRAINER_KEY 또는 data/key.txt (자동 생성).
업로드 zip 은 경로 검사 후 정해진 파일(info.json, episode.npz, meta.json, thumb.jpg)만 풀고, npz 는 pickle 없이 연다.
"""
import json
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
import zipfile
from importlib import metadata

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, PlainTextResponse

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("TRAINER_PORT", "50100"))
DATA = os.environ.get("TRAINER_DATA") or os.path.join(HERE, "data")
DS_DIR = os.path.join(DATA, "datasets")
JOB_DIR = os.path.join(DATA, "jobs")
UP_DIR = os.path.join(DATA, "uploads")
for d in (DS_DIR, JOB_DIR, UP_DIR):
    os.makedirs(d, exist_ok=True)
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")
EP_FILE_RE = re.compile(r"^episodes/ep_\d{6}/(episode\.npz|meta\.json|thumb\.jpg)$")
sys.path.insert(0, HERE)
from train_act import DEFAULT_TRAIN   # noqa: E402 — 학습 설정 기본값 (torch import 는 train_act 안에서 하므로 여기서도 로드됨)


def _key():
    k = os.environ.get("TRAINER_KEY", "").strip()
    if k:
        return k
    p = os.path.join(DATA, "key.txt")
    if os.path.exists(p):
        return open(p).read().strip()
    k = secrets.token_urlsafe(6)
    with open(p, "w") as f:
        f.write(k)
    os.chmod(p, 0o600)
    return k


KEY = _key()


def log(msg):
    print(f"[trainer] {msg}", flush=True)


# =====================================================================
# 데이터셋
# =====================================================================
def ds_summary(name):
    p = os.path.join(DS_DIR, name)
    info = {}
    try:
        with open(os.path.join(p, "info.json"), encoding="utf-8") as f:
            info = json.load(f)
    except Exception:   # noqa: BLE001
        pass
    eps, ok, frames, dur, size, img = 0, 0, 0, 0.0, 0, None
    ep_dir = os.path.join(p, "episodes")
    for n in sorted(os.listdir(ep_dir)) if os.path.isdir(ep_dir) else []:
        mp = os.path.join(ep_dir, n, "meta.json")
        if not os.path.exists(mp):
            continue
        try:
            with open(mp, encoding="utf-8") as f:
                m = json.load(f)
        except Exception:   # noqa: BLE001
            continue
        eps += 1
        if m.get("ok"):
            ok += 1
            frames += int(m.get("n", 0))
            dur += float(m.get("duration", 0))
        img = img or m.get("image")
        npz = os.path.join(ep_dir, n, "episode.npz")
        size += os.path.getsize(npz) if os.path.exists(npz) else 0
    return {"name": name, "episodes": eps, "ok": ok, "frames": frames, "minutes": round(dur / 60, 1),
            "size_mb": round(size / 1e6, 1), "image": img, "robot": info.get("robot"),
            "created": info.get("created"), "uploaded": time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(p)))}


def extract_dataset(zpath, filename):
    """로봇 웹 zip (<이름>/info.json, <이름>/episodes/ep_xxxxxx/...) → DS_DIR/<이름>. 이름이 있으면 _2, _3 …"""
    try:
        z = zipfile.ZipFile(zpath)
    except zipfile.BadZipFile:
        raise HTTPException(400, "zip 아님")
    with z:
        files = [i for i in z.infolist() if not i.is_dir()]
        roots = {i.filename.split("/", 1)[0] for i in files}
        if len(roots) != 1:
            raise HTTPException(400, "zip 안에 폴더 하나 (<데이터셋 이름>/...) 여야 함 — 로봇 웹에서 받은 zip 그대로 올리세요")
        root = roots.pop()
        keep = []
        for i in files:
            n = i.filename
            if n.startswith("/") or ".." in n.split("/"):
                raise HTTPException(400, f"zip 안 경로 이상: {n}")
            rel = n[len(root) + 1:]
            if rel == "info.json" or EP_FILE_RE.match(rel):
                keep.append((i, rel))
        if not any(r == "info.json" for _, r in keep):
            raise HTTPException(400, "info.json 없음 — 로봇 웹(policy_server) 데이터셋 zip 이 아님")
        need = sum(i.file_size for i, _ in keep)
        free = shutil.disk_usage(DS_DIR).free
        if need > free - 2e9:
            raise HTTPException(507, f"디스크 부족 (필요 {need / 1e9:.1f} GB, 남음 {free / 1e9:.1f} GB)")
        stem = re.sub(r"[^A-Za-z0-9_.\-]", "_", root)[:56] or "dataset"
        name, k = stem, 2
        while os.path.exists(os.path.join(DS_DIR, name)):
            name, k = f"{stem}_{k}", k + 1
        tmp = os.path.join(DS_DIR, f".{name}.tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        for i, rel in keep:
            dst = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with z.open(i) as src, open(dst, "wb") as out:
                shutil.copyfileobj(src, out, 1 << 20)
    os.replace(tmp, os.path.join(DS_DIR, name))
    s = ds_summary(name)
    if s["ok"] == 0:
        shutil.rmtree(os.path.join(DS_DIR, name))
        raise HTTPException(400, "성공 에피소드가 없음")
    log(f"데이터셋 업로드 {name}: {s['ok']}/{s['episodes']} ep, {s['frames']} 프레임, {s['size_mb']} MB ({filename})")
    return s


# =====================================================================
# 작업 (큐 + 서브프로세스)
# =====================================================================
class Jobs:
    def __init__(self):
        self.lk = threading.Lock()
        self.jobs = {}
        self.proc = None
        self.cur = None
        for n in sorted(os.listdir(JOB_DIR)):
            jp = os.path.join(JOB_DIR, n, "job.json")
            if os.path.exists(jp):
                try:
                    with open(jp, encoding="utf-8") as f:
                        j = json.load(f)
                except Exception:   # noqa: BLE001
                    continue
                if j["status"] == "running":
                    j["status"], j["error"] = "interrupted", "서버가 학습 중에 꺼짐 (ckpt_last.pt 가 있으면 이어 학습 가능)"
                self.jobs[j["id"]] = j
                self._save(j)
        threading.Thread(target=self._worker, daemon=True).start()

    def _save(self, j):
        d = os.path.join(JOB_DIR, j["id"])
        os.makedirs(d, exist_ok=True)
        tmp = os.path.join(d, "job.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(j, f, ensure_ascii=False)
        os.replace(tmp, os.path.join(d, "job.json"))

    def add(self, dataset, params):
        if not NAME_RE.match(dataset or "") or not os.path.isdir(os.path.join(DS_DIR, dataset)):
            raise HTTPException(404, f"데이터셋 없음: {dataset}")
        tc = dict(DEFAULT_TRAIN)
        for k, v in (params or {}).items():
            if k in DEFAULT_TRAIN and v not in (None, ""):
                tc[k] = type(DEFAULT_TRAIN[k])(v) if not isinstance(DEFAULT_TRAIN[k], bool) else bool(v)
        jid = time.strftime("j%m%d_%H%M%S")
        while jid in self.jobs:
            jid += "x"
        tc["name"] = re.sub(r"[^A-Za-z0-9_.\-]", "_", tc["name"] or f"act_{dataset}_{time.strftime('%m%d_%H%M')}")[:60]
        j = {"id": jid, "dataset": dataset, "params": tc, "status": "queued", "created": time.time(),
             "started": None, "finished": None, "phase": "", "msg": "", "step": 0, "total": tc["steps"],
             "loss": None, "val_l1": None, "val_err_deg": None, "it_s": None, "eta_s": None, "zip": None,
             "curve": [], "vcurve": [], "error": "", "export_diff_deg": None}
        with self.lk:
            self.jobs[jid] = j
            self._save(j)
        log(f"작업 추가 {jid} ({dataset}, {tc['steps']} step)")
        return j

    def _worker(self):
        while True:
            time.sleep(1.0)
            with self.lk:
                q = sorted((j for j in self.jobs.values() if j["status"] == "queued"), key=lambda j: j["created"])
                if not q or self.proc is not None:
                    continue
                j = q[0]
                j["status"], j["started"] = "running", time.time()
                self.cur = j
            try:
                self._run(j)
            except Exception as e:   # noqa: BLE001
                j["status"], j["error"] = "failed", f"{type(e).__name__}: {e}"
            finally:
                j["finished"] = time.time()
                self.proc, self.cur = None, None
                with self.lk:
                    self._save(j)
                log(f"작업 {j['id']} → {j['status']} {j['error']}")

    def _run(self, j):
        d = os.path.join(JOB_DIR, j["id"])
        os.makedirs(d, exist_ok=True)
        cfgp = os.path.join(d, "config.json")
        with open(cfgp, "w", encoding="utf-8") as f:
            json.dump(j["params"], f, ensure_ascii=False, indent=1)
        cmd = [sys.executable, "-u", os.path.join(HERE, "train_act.py"), "--dataset", os.path.join(DS_DIR, j["dataset"]),
               "--out", d, "--config", cfgp]
        logf = open(os.path.join(d, "train.log"), "a", encoding="utf-8")
        logf.write(f"$ {' '.join(cmd)}\n")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     start_new_session=True, cwd=HERE)
        last_save = 0.0
        tail = []
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            if line.startswith("@@"):
                try:
                    p = json.loads(line[2:])
                except Exception:   # noqa: BLE001
                    continue
                self._progress(j, p)
            else:
                logf.write(line + "\n")
                logf.flush()
                tail = (tail + [line])[-20:]
            if time.time() - last_save > 2.0:
                with self.lk:
                    self._save(j)
                last_save = time.time()
        rc = self.proc.wait()
        logf.write(f"[exit {rc}]\n")
        logf.close()
        if j["status"] == "canceling":
            j["status"] = "canceled"
        elif rc == 0 and j.get("zip"):
            j["status"] = "done"
        else:
            j["status"] = "failed"
            j["error"] = next((ln for ln in reversed(tail) if ln.strip() and not ln.startswith("  ")), f"exit {rc}")

    @staticmethod
    def _progress(j, p):
        for k in ("phase", "msg", "step", "total", "loss", "it_s", "eta_s", "val_l1", "val_err_deg", "zip",
                  "export_diff_deg"):
            if k in p:
                j[k] = p[k]
        if "loss" in p:
            j["curve"].append([p["step"], p["loss"], p.get("l1"), p.get("kl")])
            if len(j["curve"]) > 1500:                 # 오래된 점을 솎아 냄
                j["curve"] = j["curve"][:-500:2] + j["curve"][-500:]
        if "val_l1" in p:
            j["vcurve"].append([p["step"], p["val_l1"], p.get("val_err_deg")])

    def cancel(self, jid):
        with self.lk:
            j = self.jobs.get(jid)
            if not j:
                raise HTTPException(404, "작업 없음")
            if j["status"] == "queued":
                j["status"] = "canceled"
                self._save(j)
                return j
            if j["status"] != "running" or self.proc is None:
                raise HTTPException(409, f"상태 {j['status']}")
            j["status"] = "canceling"
            os.killpg(self.proc.pid, signal.SIGTERM)
        return j

    def finish(self, jid):
        with self.lk:
            j = self.jobs.get(jid)
            if not j or j["status"] != "running" or self.proc is None:
                raise HTTPException(409, "학습 중인 작업이 아님")
            os.kill(self.proc.pid, signal.SIGUSR1)
        return j

    def delete(self, jid):
        with self.lk:
            j = self.jobs.get(jid)
            if not j:
                raise HTTPException(404, "작업 없음")
            if j["status"] in ("running", "canceling"):
                raise HTTPException(409, "실행 중 — 먼저 취소")
            del self.jobs[jid]
        shutil.rmtree(os.path.join(JOB_DIR, jid), ignore_errors=True)


jobs = Jobs()


# =====================================================================
# FastAPI
# =====================================================================
app = FastAPI(title="ACT trainer")


@app.middleware("http")
async def auth(request: Request, call_next):
    if request.url.path.startswith("/api/"):
        k = request.headers.get("x-key") or request.query_params.get("key")
        if not k or not secrets.compare_digest(k, KEY):
            return JSONResponse({"detail": "접근 키 필요"}, status_code=401)
    return await call_next(request)


@app.get("/", response_class=HTMLResponse)
def index():
    with open(os.path.join(HERE, "trainer_web.html"), encoding="utf-8") as f:
        return f.read()


def _ver(pkg):
    try:
        return metadata.version(pkg)
    except Exception:   # noqa: BLE001
        return None


_info_cache = {"t": 0, "v": None}


@app.get("/api/info")
def api_info():
    if time.time() - _info_cache["t"] < 2.0 and _info_cache["v"]:
        return _info_cache["v"]
    gpus = []
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu,temperature.gpu,driver_version",
                              "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=3).stdout
        for ln in out.strip().splitlines():
            n, mu, mt, u, t, drv = [x.strip() for x in ln.split(",")]
            gpus.append({"name": n, "mem_used": int(mu), "mem_total": int(mt), "util": int(u), "temp": int(t), "driver": drv})
    except Exception as e:   # noqa: BLE001
        gpus = [{"error": f"nvidia-smi: {e}"}]
    du = shutil.disk_usage(DATA)
    v = {"gpus": gpus, "torch": _ver("torch"), "torchvision": _ver("torchvision"), "openvino": _ver("openvino"),
         "python": sys.version.split()[0], "disk_free_gb": round(du.free / 1e9, 1), "defaults": DEFAULT_TRAIN,
         "queue": {"running": sum(j["status"] == "running" for j in jobs.jobs.values()),
                   "queued": sum(j["status"] == "queued" for j in jobs.jobs.values())}}
    _info_cache.update(t=time.time(), v=v)
    return v


@app.get("/api/datasets")
def api_datasets():
    return [ds_summary(n) for n in sorted(os.listdir(DS_DIR)) if NAME_RE.match(n) and os.path.isdir(os.path.join(DS_DIR, n))]


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
        raise HTTPException(404, "없음")
    shutil.rmtree(p)
    return {"ok": True}


def _brief(j):
    return {k: v for k, v in j.items() if k not in ("curve", "vcurve")}


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
    log(f"시작 — http://0.0.0.0:{PORT}/   데이터 {DATA}")
    log(f"접근 키: {KEY}   (웹에서 처음 한 번 입력. 바꾸려면 {os.path.join(DATA, 'key.txt')} 삭제 후 재시작 또는 TRAINER_KEY=...)")
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
