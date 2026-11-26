"""
dataset_store.py — 데이터셋 zip 받기 (학습 웹 · 런처 공용). 표준 라이브러리만.

  LeRobot v3 zip = <이름>/meta/info.json · data/… · videos/…  (수집 화면 · 홈의 [LeRobot v3 zip])
  예전 수집 형식 zip = <이름>/info.json · episodes/ep_*/…        (v3 변환은 학습 웹이 lerobot 환경에서)
"""
import json
import os
import re
import shutil
import tempfile
import zipfile

from pitron import paths

V3_FILE_RE = re.compile(r"^(meta/[A-Za-z0-9_./\-]+\.(json|jsonl|parquet)|data/chunk-\d+/file-\d+\.parquet|"
                        r"videos/[A-Za-z0-9_.\-]+/chunk-\d+/file-\d+\.mp4)$")
OLD_FILE_RE = re.compile(r"^(info\.json|episodes/ep_\d{6}/(episode\.npz|meta\.json))$")
UPLOADED_DIR = os.path.join(os.environ.get("TRAINER_DATA") or paths.data_dir("trainer", legacy="trainer/data"), "datasets")
COLLECT_DIR = paths.data_dir("datasets", legacy="gripper/data")


class DiskFull(OSError):
    pass


def free_name(stem, dirs):
    """dirs 어디에도 없는 이름 (수집 데이터셋과 같은 이름이면 올린 것이 가려 버리므로 둘 다 봄)."""
    stem = re.sub(r"[^A-Za-z0-9_.\-]", "_", stem)[:56].strip(".") or "dataset"
    name, k = stem, 2
    while any(os.path.exists(os.path.join(d, name)) for d in dirs if d):
        name, k = f"{stem}_{k}", k + 1
    return name


def unpack(zpath, up_dir, name_dirs):
    """zip → 임시 폴더 (필요한 파일만). → (이름, 임시 폴더, v3 여부). 형식 이상 = ValueError · 디스크 부족 = DiskFull"""
    try:
        z = zipfile.ZipFile(zpath)
    except zipfile.BadZipFile:
        raise ValueError("zip 아님")
    with z:
        files = [i for i in z.infolist() if not i.is_dir()]
        roots = {i.filename.split("/", 1)[0] for i in files}
        if len(roots) != 1:
            raise ValueError("zip 안에 폴더 하나 (<데이터셋 이름>/...) 여야 함 — 수집 화면에서 받은 zip 그대로 올리세요")
        root = roots.pop()
        rels = []
        for i in files:
            n = i.filename
            if n.startswith("/") or ".." in n.split("/"):
                raise ValueError(f"zip 안 경로 이상: {n}")
            rels.append((i, n[len(root) + 1:]))
        v3 = any(r == "meta/info.json" for _, r in rels)
        old = any(r == "info.json" for _, r in rels)
        if not v3 and not old:
            raise ValueError("LeRobot v3 데이터셋 (meta/info.json) 이 아님")
        keep = [(i, r) for i, r in rels if (V3_FILE_RE if v3 else OLD_FILE_RE).match(r)]
        need = sum(i.file_size for i, _ in keep)
        os.makedirs(up_dir, exist_ok=True)
        free = shutil.disk_usage(up_dir).free
        if need * (2 if old else 1) > free - 2e9:
            raise DiskFull(f"디스크 부족 (필요 {need / 1e9:.1f} GB, 남음 {free / 1e9:.1f} GB)")
        name = free_name(root[:-len("_lerobot")] if root.endswith("_lerobot") else root, name_dirs)
        tmp = tempfile.mkdtemp(prefix=f".{name}.", dir=up_dir)          # 올리기마다 따로 (동시에 올려도 안 겹침)
        try:
            for i, rel in keep:
                dst = os.path.join(tmp, rel)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                with z.open(i) as src, open(dst, "wb") as out:
                    shutil.copyfileobj(src, out, 1 << 20)
        except Exception:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
    return name, tmp, v3


def finish_v3(tmp, ds_dir, name):
    """임시 폴더 (v3) → ds_dir/<이름>. v3.0 이 아니면 ValueError."""
    try:
        with open(os.path.join(tmp, "meta", "info.json"), encoding="utf-8") as f:
            inf = json.load(f)
    except (OSError, ValueError):
        inf = {}
    if inf.get("codebase_version") != "v3.0":
        raise ValueError(f"LeRobot 데이터셋 {inf.get('codebase_version')} — v3.0 만 (lerobot 0.4.x)")
    os.makedirs(ds_dir, exist_ok=True)
    os.replace(tmp, os.path.join(ds_dir, name))
    return name
