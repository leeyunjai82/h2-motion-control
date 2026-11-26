#!/usr/bin/env python3
"""
act_viewer.py — LeRobot v3 데이터셋 · lerobot 모델 보기 (전시용, 로봇 · GPU · lerobot 필요 없음)

  Windows: apps/act_viewer/start_viewer.bat 더블클릭 — 처음 한 번 apps/act_viewer/.venv 가상환경을 만들고 requirements.txt 설치 (Python 3.9+)
  홈 화면 (런처): [데이터 보기] — 런처가 lerobot 환경 python 으로 늘 띄워 둠 (따로 실행할 필요 없음)
  Linux  : python3 -m venv apps/act_viewer/.venv && apps/act_viewer/.venv/bin/pip install -r apps/act_viewer/requirements.txt
           apps/act_viewer/.venv/bin/python apps/act_viewer/act_viewer.py [--data <폴더>] [--port 8800] [--no-browser]

  이 PC 의 데이터셋 · 모델은 zip 없이 바로 보임: data/datasets/<이름>/lerobot · data/policies/<이름> (홈 화면 → 데이터 보기)
  data/viewer/ 에 zip 을 넣어 두면 켤 때 자동으로 읽음 (화면에 끌어다 놓거나 [zip 열기] 하면 이 폴더에 저장)
    데이터셋 zip : 수집 화면 · 홈의 [LeRobot v3 zip]           — <이름>/meta/info.json (codebase_version v3.0)
    모델 zip     : 학습 웹 (:50100) model zip (lerobot 모델)     — <이름>/config.json · model.safetensors (정책 종류 상관없음)
  parquet 는 pyarrow 로 읽고, 영상 (mp4) 은 캐시 폴더 (data/viewer/.cache) 에 꺼내 브라우저 <video> 로 보여 줌. 모델은 실행 안 함
  3D 로봇 = robots/h2/H2.urdf + 메시, 손 = robots/h2/hands/dex1 (Unitree Dex1-1) + 어댑터 (apps/gripper_pick/gripper.yaml xr_view.dex1_view)
"""
import argparse
import json
import mimetypes
import os
import re
import shutil
import struct
import sys
import threading
import webbrowser
import zipfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(HERE, "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron import paths, ports  # noqa: E402 — 표준 라이브러리만
REPO = os.path.dirname(os.path.dirname(HERE))
PAGE = os.path.join(HERE, "act_viewer.html")
THREE = os.path.join(REPO, "web", "vendor", "three.min.js")
URDF = os.path.join(REPO, "robots", "h2", "H2.urdf")
DEX1_YAML = os.path.join(REPO, "robots", "h2", "hands", "dex1", "dex1.yaml")
GRIP_YAML = os.path.join(REPO, "apps", "gripper_pick", "gripper.yaml")
MESH_DIRS = {"h2": os.path.join(REPO, "robots", "h2", "meshes"),
             "dex1": os.path.join(REPO, "robots", "h2", "hands", "dex1", "meshes"),
             "adapter": os.path.join(REPO, "robots", "h2", "hands", "dex1")}
CHUNK = 1 << 20
NAME_RE = re.compile(r"^[^/\\:*?\"<>|]{1,120}\.zip$", re.I)
VIDEO_RE = re.compile(r"^videos/[A-Za-z0-9_.\-]+/chunk-\d+/file-\d+\.mp4$")
LOCAL_RE = re.compile(r"^(datasets|uploaded|policies)/([A-Za-z0-9_.\-]{1,64})$")     # 이 PC 의 데이터셋 · 올린 데이터셋 · 모델 (zip 아님)
DS_DIR = paths.data_dir("datasets", legacy="gripper/data")
UP_DIR = os.path.join(os.environ.get("TRAINER_DATA") or paths.data_dir("trainer", legacy="trainer/data"), "datasets")   # 학습용으로 올린 것
MODEL_DIR = paths.data_dir("policies", legacy="gripper/models")


def local_dir(fid):
    """'datasets/<이름>' → data/datasets/<이름>/lerobot · 'uploaded/<이름>' → data/trainer/datasets/<이름> ·
    'policies/<이름>' → data/policies/<이름> (없으면 None)."""
    m = LOCAL_RE.match(fid or "")
    if not m or m.group(2).strip(".") == "":
        return None
    kind, n = m.groups()
    d = {"datasets": os.path.join(DS_DIR, n, "lerobot"), "uploaded": os.path.join(UP_DIR, n), "policies": os.path.join(MODEL_DIR, n)}[kind]
    return d if os.path.isdir(d) else None


def local_items():
    out = []
    if os.path.isdir(DS_DIR):
        out += [f"datasets/{n}" for n in sorted(os.listdir(DS_DIR)) if not n.startswith(".") and os.path.isfile(os.path.join(DS_DIR, n, "lerobot", "meta", "info.json"))]
    if os.path.isdir(UP_DIR):
        out += [f"uploaded/{n}" for n in sorted(os.listdir(UP_DIR)) if not n.startswith(".") and os.path.isfile(os.path.join(UP_DIR, n, "meta", "info.json"))]
    if os.path.isdir(MODEL_DIR):
        out += [f"policies/{n}" for n in sorted(os.listdir(MODEL_DIR))
                if not n.startswith(".") and os.path.isfile(os.path.join(MODEL_DIR, n, "config.json")) and os.path.isfile(os.path.join(MODEL_DIR, n, "model.safetensors"))]
    return [x for x in out if LOCAL_RE.match(x)]


class DirZip:
    """폴더를 zip 처럼 (namelist · open · getinfo) — 이름은 <root>/<폴더 안 상대 경로>."""

    def __init__(self, folder, root):
        self.folder, self.root = folder, root

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def namelist(self):
        out = []
        for r, ds, fs in os.walk(self.folder):
            ds[:] = [d for d in ds if not d.startswith(".") and d not in ("openvino",)]
            for f in fs:
                if not f.startswith("."):
                    out.append(f"{self.root}/" + os.path.relpath(os.path.join(r, f), self.folder).replace(os.sep, "/"))
        return out

    def _path(self, name):
        rel = name.split("/", 1)[1] if name.startswith(self.root + "/") else None
        if not rel or ".." in rel.split("/"):
            raise KeyError(name)
        return os.path.join(self.folder, *rel.split("/"))

    def open(self, name):
        return open(self._path(name), "rb")

    def getinfo(self, name):
        class I:
            file_size = os.path.getsize(self._path(name))
        return I


# ---------------- 지우기 ----------------
def _get_json(port_name, path):
    """다른 화면 상태 (안 떠 있으면 None)."""
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{ports.PORTS[port_name]}{path}", timeout=1.0) as r:
            return json.loads(r.read())
    except Exception:  # noqa: BLE001
        return None


def _busy(kind, name):
    """지우면 안 되는 이유 (쓰는 중이면 문구, 아니면 None)."""
    st = _get_json("pick_web", "/api/state") or {}
    if kind == "datasets":
        rec = st.get("rec") or {}
        if rec.get("name") == name and (rec.get("on") or rec.get("active")):
            return "수집 화면이 이 데이터셋에 녹화 중 (또는 수집 켬) — 수집 화면에서 끄고 지우세요"
    if kind == "policies":
        pol = st.get("policy") or {}
        if (pol.get("loaded") or {}).get("name") == name:
            return "추론 화면에 이 모델이 불러져 있음 — 추론 화면을 끄고 지우세요"
    if kind in ("datasets", "uploaded"):
        jobs = _get_json("trainer", "/api/jobs") or []
        if any(j.get("dataset") == name and j.get("status") in ("queued", "running", "canceling", "finishing") for j in jobs):
            return "이 데이터셋으로 학습 대기 · 중인 작업이 있음 — 학습 화면에서 먼저 취소"
    return None


def delete_item(fid, data_dir):
    """데이터셋 · 모델 · zip 지우기 (되돌릴 수 없음). → 지운 경로"""
    m = LOCAL_RE.match(fid or "")
    if m:
        kind, name = m.groups()
        base = {"datasets": DS_DIR, "uploaded": UP_DIR, "policies": MODEL_DIR}[kind]
        target = os.path.join(base, name)                  # 수집 데이터셋은 원본 녹화 + lerobot/ 통째로
    else:
        name = os.path.basename(fid or "")
        if not NAME_RE.match(name):
            raise FileNotFoundError(fid)
        kind, base, target = "zip", data_dir, os.path.join(data_dir, name)
    if not os.path.exists(target) or os.path.dirname(os.path.realpath(target)) != os.path.realpath(base):
        raise FileNotFoundError(fid)
    why = _busy(kind, name)
    if why:
        raise PermissionError(why)
    if kind == "zip":
        os.remove(target)
        for c in os.listdir(os.path.join(data_dir, ".cache")) if os.path.isdir(os.path.join(data_dir, ".cache")) else []:
            if c.startswith(os.path.splitext(name)[0] + "_"):
                shutil.rmtree(os.path.join(data_dir, ".cache", c), ignore_errors=True)
        return target
    lk = None
    if kind == "datasets":                                 # v3 변환 (배타) · 학습 (공유) 이 잡는 잠금 — 쓰는 중이면 못 지움
        import fcntl
        lk = open(os.path.join(target, ".lerobot.lock"), "a")
        try:
            fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lk.close()
            raise PermissionError("LeRobot v3 변환 또는 학습이 이 데이터셋을 쓰는 중 — 끝난 뒤 지우세요")
    try:
        trash = os.path.join(base, f".deleting_{name}_{os.getpid()}")
        os.replace(target, trash)                          # 먼저 이름을 바꿔 목록에서 바로 빠지게
        shutil.rmtree(trash, ignore_errors=True)
    finally:
        if lk:
            lk.close()
    with _lk:
        for k in [k for k in _cache if k[0] == fid]:
            del _cache[k]
    return target


def _open(src):
    """src = zip 경로 또는 'datasets/<이름>' · 'policies/<이름>'."""
    d = local_dir(src)
    if d is not None:
        return DirZip(d, LOCAL_RE.match(src).group(2))
    return zipfile.ZipFile(src)


def _stamp(src):
    d = local_dir(src)
    if d is None:
        return os.path.getmtime(src)
    ps = [os.path.join(d, p) for p in ("meta/info.json", "meta/h2.json", "config.json", "model.safetensors", "h2_model.json")]
    return max([os.path.getmtime(p) for p in ps if os.path.isfile(p)] + [os.path.getmtime(d)])


def _size(src):
    d = local_dir(src)
    if d is None:
        return os.path.getsize(src)
    def keep(r):                                       # 맨 위 . (숨김) · openvino (추론 캐시) 폴더는 뺌
        top = os.path.relpath(r, d).split(os.sep)[0]
        return top == "." or not top.startswith((".", "openvino"))
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(d) if keep(r) for f in fs if not f.startswith("."))


def robot_info():
    """3D 로봇에 필요한 것: H2 URDF 글자 + Dex1 URDF 글자 + 장착 · 그리퍼 값 (gripper.yaml). 못 읽으면 그 항목 None."""
    out = {"urdf": open(URDF, encoding="utf-8").read(), "dex1": None, "grip": None, "warn": []}
    try:
        import yaml
    except ImportError:
        out["warn"].append("PyYAML 없음 (apps/act_viewer/requirements.txt 설치) — 손은 그리퍼 없이 그림")
        return out
    with open(GRIP_YAML, encoding="utf-8") as f:
        g = yaml.safe_load(f) or {}
    with open(DEX1_YAML, encoding="utf-8") as f:
        g.update(yaml.safe_load(f) or {})
    dv = ((g.get("xr_view") or {}).get("dex1_view")) or {}
    out["grip"] = {"q_hi": float(g["q_range"][1]), "mm_per_rad": float(g["pick"]["mm_per_rad"])}
    durdf = os.path.join(REPO, str(dv.get("urdf", "")))
    if dv and os.path.isfile(durdf):
        a = dv.get("adapter_stl") or {}
        apath = os.path.join(REPO, str(a.get("path", "")))
        out["dex1"] = {"urdf": open(durdf, encoding="utf-8").read(),
                       "origin_x": float(g["tcp_offset"][0]) - float(dv["origin_to_tcp_m"]),
                       "mount_rpy_deg": [float(v) for v in dv["mount_rpy_deg"]],
                       "wrist_face": float(dv["wrist_face_m"]), "adapter": [float(v) for v in dv["adapter_m"]],
                       "adapter_stl": ({"file": os.path.basename(apath), "scale": float(a["scale"]),
                                        "origin_mm": [float(v) for v in a["origin_mm"]], "at_m": [float(v) for v in a["at_m"]],
                                        "rot": [[float(v) for v in r] for r in a["rot"]]}
                                       if a and os.path.isfile(apath) else None)}
    else:
        out["warn"].append(f"Dex1 URDF 없음 ({durdf}) — 손은 그리퍼 없이 그림")
    return out


# ---------------- zip 읽기 ----------------
_lk = threading.Lock()
_cache = {}          # (경로, mtime) → 요약


def _root(z):
    roots = {n.split("/", 1)[0] for n in z.namelist() if "/" in n}
    return roots.pop() if len(roots) == 1 else None


def _json(z, name):
    with z.open(name) as f:
        return json.loads(f.read().decode("utf-8").replace("NaN", "null"))


def _parquet(z, name, columns=None):
    import pyarrow.parquet as pq
    with z.open(name) as f:
        return pq.read_table(f, columns=columns)


def _safetensors_params(z, name):
    """safetensors 헤더만 읽어 파라미터 수 (torch 없이)."""
    with z.open(name) as f:
        n = struct.unpack("<Q", f.read(8))[0]
        if n > 64 << 20:
            return None
        h = json.loads(f.read(n))
    tot = 0
    for k, v in h.items():
        if k != "__metadata__":
            c = 1
            for s in v.get("shape", []):
                c *= int(s)
            tot += c
    return tot


def summarize(path):
    """zip → {kind: dataset|model|unknown, ...} (zip 이 바뀌지 않으면 캐시)."""
    key = (path, _stamp(path))
    with _lk:
        if key in _cache:
            return _cache[key]
    out = {"file": path if local_dir(path) else os.path.basename(path), "size": _size(path), "local": bool(local_dir(path))}
    try:
        with _open(path) as z:
            root, names = _root(z), set(z.namelist())
            if root and f"{root}/meta/info.json" in names:
                out.update(dataset(z, root, names))
            elif root and f"{root}/config.json" in names and f"{root}/model.safetensors" in names:
                out.update(model(z, root, names))
            elif root and f"{root}/info.json" in names:
                out.update(kind="unknown", name=root, err="예전 수집 형식 — LeRobot v3 로 바꿔서 넣으세요 (python tools/export_lerobot.py <zip> --out <폴더>)")
            else:
                out.update(kind="unknown", name=root or out["file"], err="LeRobot v3 데이터셋 · lerobot 모델 zip 이 아님")
    except Exception as e:  # noqa: BLE001
        out.update(kind="unknown", name=out["file"], err=f"{type(e).__name__}: {e}")
    with _lk:
        _cache[key] = out
    return out


def dataset(z, root, names):
    info = _json(z, f"{root}/meta/info.json")
    if info.get("codebase_version") != "v3.0":
        return {"kind": "unknown", "name": root, "err": f"LeRobot 데이터셋 {info.get('codebase_version')} — v3.0 만"}
    h2 = _json(z, f"{root}/meta/h2.json") if f"{root}/meta/h2.json" in names else {}
    feats = info.get("features", {})
    vkeys = [k for k, v in feats.items() if v.get("dtype") == "video"]
    cams = {k.split(".", 2)[-1]: {"key": k, "w": int(feats[k]["shape"][1]), "h": int(feats[k]["shape"][0]),
                                  "codec": (feats[k].get("info") or {}).get("video.codec")} for k in vkeys}
    cols = ["episode_index", "length", "dataset_from_index", "dataset_to_index", "data/chunk_index", "data/file_index"]
    for k in vkeys:
        cols += [f"videos/{k}/chunk_index", f"videos/{k}/file_index", f"videos/{k}/from_timestamp", f"videos/{k}/to_timestamp"]
    rows = []
    for n in sorted(x for x in names if x.startswith(f"{root}/meta/episodes/") and x.endswith(".parquet")):
        t = _parquet(z, n)
        rows += t.select([c for c in cols if c in t.column_names] + (["tasks"] if "tasks" in t.column_names else [])).to_pylist()
    rows.sort(key=lambda r: r["episode_index"])
    h2e = {int(e["index"]): e for e in h2.get("episodes", [])}
    eps = []
    for r in rows:
        i = int(r["episode_index"])
        e = h2e.get(i, {})
        eps.append({"index": i, "n": int(r["length"]), "from": int(r["dataset_from_index"]), "to": int(r["dataset_to_index"]),
                    "data": [int(r.get("data/chunk_index") or 0), int(r.get("data/file_index") or 0)],
                    "video": {k: [int(r[f"videos/{k}/chunk_index"]), int(r[f"videos/{k}/file_index"]),
                                  float(r[f"videos/{k}/from_timestamp"])] for k in vkeys if f"videos/{k}/chunk_index" in r},
                    "tasks": r.get("tasks"), "side": e.get("side"), "result": e.get("result"), "raw": e.get("raw"),
                    "created": e.get("created"), "stages": e.get("stages")})
    sn = (feats.get("observation.state") or {}).get("names") or h2.get("state_names") or []
    return {"kind": "dataset", "name": root, "fps": info.get("fps"), "robot": info.get("robot_type"), "cameras": cams,
            "state_names": sn, "has_tcp": "tcp" in feats, "has_stage": "stage" in feats, "task": h2.get("task"), "fit": h2.get("fit"),
            "total_frames": info.get("total_frames"), "episodes": eps, "chunks_size": info.get("chunks_size"),
            "data_path": info.get("data_path"), "video_path": info.get("video_path"), "updated": h2.get("updated")}


def model(z, root, names):
    cfg = _json(z, f"{root}/config.json")
    h2 = _json(z, f"{root}/h2_model.json") if f"{root}/h2_model.json" in names else {}
    tc = _json(z, f"{root}/train_config.json") if f"{root}/train_config.json" in names else {}
    feats = cfg.get("input_features") or {}
    cams = {k.split(".", 2)[-1]: {"w": int(v["shape"][2]), "h": int(v["shape"][1])} for k, v in feats.items()
            if k.startswith("observation.images.") and len(v.get("shape", [])) == 3}
    keep = ("chunk_size", "n_action_steps", "n_obs_steps", "horizon", "dim_model", "n_heads", "n_encoder_layers", "n_decoder_layers",
            "vision_backbone", "use_vae", "kl_weight", "temporal_ensemble_coeff", "num_inference_steps", "noise_scheduler_type",
            "optimizer_lr", "vlm_model_name")
    info = z.getinfo(f"{root}/model.safetensors")
    return {"kind": "model", "name": h2.get("name") or root, "policy": cfg.get("type"), "cameras": cams,
            "state_dim": ((feats.get("observation.state") or {}).get("shape") or [None])[0],
            "action_dim": (((cfg.get("output_features") or {}).get("action") or {}).get("shape") or [None])[0],
            "config": {k: cfg[k] for k in keep if k in cfg}, "h2": h2,
            "train": {"steps": tc.get("steps"), "batch_size": tc.get("batch_size"), "seed": tc.get("seed"),
                      "dataset": (tc.get("dataset") or {}).get("repo_id")},
            "params": _safetensors_params(z, f"{root}/model.safetensors"), "weights_mb": round(info.file_size / 1e6, 1)}


def episode(path, ei):
    """에피소드 ei → state · action (· tcp · stage) · timestamp (프레임 순서)."""
    import pyarrow.compute as pc
    s = summarize(path)
    e = next((x for x in s.get("episodes", []) if x["index"] == ei), None)
    if e is None:
        raise KeyError(f"에피소드 {ei} 없음")
    fn = s["data_path"].format(chunk_index=e["data"][0], file_index=e["data"][1])
    with _open(path) as z:
        cols = ["episode_index", "timestamp", "observation.state", "action"] + (["tcp"] if s["has_tcp"] else []) + (["stage"] if s["has_stage"] else [])
        t = _parquet(z, f"{s['name']}/{fn}", columns=cols)
    t = t.filter(pc.equal(t["episode_index"], ei))
    out = {"index": ei, "n": t.num_rows, "t": t["timestamp"].to_pylist()}
    for k, col in (("state", "observation.state"), ("action", "action"), ("tcp", "tcp")):
        if col in t.column_names:
            out[k] = [round(float(v), 5) for row in t[col].to_pylist() for v in row]
    if "stage" in t.column_names:
        out["stage"] = [int(v[0]) if isinstance(v, list) else int(v) for v in t["stage"].to_pylist()]
    out["D"] = len(out["state"]) // max(1, out["n"])
    return out


def video_file(data_dir, path, key, chunk, file):
    """zip 안 mp4 → 캐시 폴더 파일 경로 (한 번만 꺼냄)."""
    s = summarize(path)
    rel = s["video_path"].format(video_key=key, chunk_index=int(chunk), file_index=int(file))
    if not VIDEO_RE.match(rel):
        raise KeyError(rel)
    d = local_dir(path)
    if d is not None:                                  # 이 PC 의 데이터셋: 파일 그대로
        return os.path.join(d, *rel.split("/"))
    cdir = os.path.join(data_dir, ".cache", f"{os.path.splitext(s['file'])[0]}_{int(os.path.getmtime(path))}")
    dst = os.path.join(cdir, rel)
    if not os.path.isfile(dst):
        with _lk:
            if not os.path.isfile(dst):
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                with zipfile.ZipFile(path) as z, z.open(f"{s['name']}/{rel}") as src, open(dst + ".tmp", "wb") as out:
                    shutil.copyfileobj(src, out, CHUNK)
                os.replace(dst + ".tmp", dst)
    return dst


class Handler(BaseHTTPRequestHandler):
    data_dir = paths.data_dir("viewer", legacy="viewer/data")

    def log_message(self, fmt, *args):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8", code)

    def _file(self, path, ctype=None):
        """파일 (Range 지원 — 브라우저 <video> 가 필요한 부분만 읽음)"""
        if not os.path.isfile(path):
            return self.send_error(HTTPStatus.NOT_FOUND)
        size = os.path.getsize(path)
        ctype = ctype or mimetypes.guess_type(path)[0] or "application/octet-stream"
        m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range", "") or "")
        if m and (m.group(1) or m.group(2)):
            if m.group(1):
                start = int(m.group(1))
                end = min(int(m.group(2)) if m.group(2) else size - 1, size - 1)
            else:
                start, end = max(0, size - int(m.group(2))), size - 1
            if start > end or start >= size:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
            self.send_response(HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        else:
            start, end = 0, size - 1
            self.send_response(HTTPStatus.OK)
        n = end - start + 1
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(n))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Cache-Control", "max-age=3600")
        self.end_headers()
        with open(path, "rb") as f:
            f.seek(start)
            while n > 0:
                b = f.read(min(CHUNK, n))
                if not b:
                    break
                self.wfile.write(b)
                n -= len(b)

    def _zip(self, q):
        fid = (q.get("f") or [""])[0]
        if local_dir(fid):
            return fid
        name = os.path.basename(fid)
        p = os.path.join(self.data_dir, name)
        if not NAME_RE.match(name) or not os.path.isfile(p):
            raise FileNotFoundError(name)
        return p

    def do_GET(self):
        u = urlparse(self.path)
        p, q = unquote(u.path), parse_qs(u.query)
        try:
            if p in ("/", "/index.html"):
                return self._file(PAGE, "text/html; charset=utf-8")
            if p == "/vendor/three.min.js":
                return self._file(THREE, "application/javascript")
            if p in ("/web/robot3d.js", "/web/i18n.js"):
                return self._file(os.path.join(paths.WEB_DIR, os.path.basename(p)), "application/javascript")
            if p == "/api/robot":
                return self._json(robot_info())
            if p == "/api/files":
                d = self.data_dir
                fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".zip")) if os.path.isdir(d) else []
                return self._json({"dir": d, "files": [summarize(x) for x in local_items()] + [summarize(os.path.join(d, f)) for f in fs]})
            if p == "/api/episode":
                return self._json(episode(self._zip(q), int((q.get("i") or ["0"])[0])))
            if p == "/api/video":
                f = video_file(self.data_dir, self._zip(q), (q.get("key") or [""])[0], (q.get("c") or ["0"])[0], (q.get("n") or ["0"])[0])
                return self._file(f, "video/mp4")
            if p == "/api/mesh":
                d = MESH_DIRS.get((q.get("d") or [""])[0])
                name = os.path.basename((q.get("f") or [""])[0])
                if not d or not name:
                    return self.send_error(HTTPStatus.NOT_FOUND)
                path = os.path.join(d, name)
                if not os.path.isfile(path):          # URDF 와 대소문자가 다른 파일 (Windows 외)
                    hit = [f for f in os.listdir(d) if f.lower() == name.lower()]
                    path = os.path.join(d, hit[0]) if hit else path
                return self._file(path, "application/octet-stream")
            self.send_error(HTTPStatus.NOT_FOUND)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        except (FileNotFoundError, KeyError, ValueError) as e:
            self._json({"detail": f"{type(e).__name__}: {e}"}, 404)

    def do_DELETE(self):
        """데이터셋 · 모델 · zip 지우기 (?f=datasets/<이름> · uploaded/<이름> · policies/<이름> · <zip 이름>)."""
        u = urlparse(self.path)
        if unquote(u.path) != "/api/item":
            return self.send_error(HTTPStatus.NOT_FOUND)
        fid = (parse_qs(u.query).get("f") or [""])[0]
        try:
            self._json({"deleted": os.path.basename(delete_item(fid, self.data_dir))})
        except FileNotFoundError:
            self._json({"detail": f"없음: {fid}"}, 404)
        except PermissionError as e:
            self._json({"detail": str(e)}, 409)

    def do_PUT(self):
        """화면에 끌어다 놓은 zip → data 폴더에 저장."""
        u = urlparse(self.path)
        if unquote(u.path) != "/api/upload":
            return self.send_error(HTTPStatus.NOT_FOUND)
        name = os.path.basename(unquote((parse_qs(u.query).get("name") or [""])[0]))
        n = int(self.headers.get("Content-Length") or 0)
        if not NAME_RE.match(name) or n <= 0:
            return self._json({"detail": "zip 파일 이름 · 크기"}, 400)
        dst = os.path.join(self.data_dir, name)
        with open(dst + ".tmp", "wb") as f:
            while n > 0:
                b = self.rfile.read(min(CHUNK, n))
                if not b:
                    break
                f.write(b)
                n -= len(b)
        os.replace(dst + ".tmp", dst)
        self._json(summarize(dst))


def main():
    ap = argparse.ArgumentParser(description="LeRobot v3 데이터셋 · lerobot 모델 보기")
    ap.add_argument("--data", default=Handler.data_dir, help="zip 을 둔 폴더 (기본 data/viewer)")
    ap.add_argument("--port", type=int, default=ports.PORTS["act_viewer"])
    ap.add_argument("--host", default="127.0.0.1", help="다른 PC 에서 보려면 0.0.0.0")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.data, exist_ok=True)
    Handler.data_dir = os.path.abspath(a.data)
    for need in (PAGE, THREE, URDF):
        if not os.path.isfile(need):
            sys.exit(f"❌ 파일 없음: {need} (저장소를 통째로 clone 했는지 확인)")
    try:
        import pyarrow  # noqa: F401
    except ImportError:
        sys.exit("❌ pyarrow 없음 — apps/act_viewer/requirements.txt 설치 (start_viewer.bat 은 .venv 를 지우고 다시 실행)")
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    url = f"http://127.0.0.1:{a.port}/"
    zips = [f for f in os.listdir(a.data) if f.lower().endswith(".zip")]
    print(f"[act_viewer] {url}   (끄기: Ctrl+C)")
    print(f"[act_viewer] zip 폴더: {Handler.data_dir} — {len(zips)} 개 {zips[:6]}")
    if not a.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
