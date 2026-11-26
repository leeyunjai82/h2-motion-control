"""
model_store.py — 모델 저장소 (data/policies/<이름>) 에 model zip 넣기. 표준 라이브러리만 (런처 · 잡기 웹 공용).

  model zip = <이름>/config.json · model.safetensors · (policy_*processor*.json/safetensors · train_config.json · h2_model.json)
    학습 웹 (:50100) 의 model zip, 또는 lerobot pretrained_model 폴더를 그대로 묶은 zip
"""
import json
import os
import re
import shutil
import zipfile

from pitron import paths

MODEL_DIR = paths.data_dir("policies", legacy="gripper/models")   # 모델 저장소 (git 제외)
NAME_RE = re.compile(r"^(?!\.+$)[A-Za-z0-9_.\-]{1,80}$")          # "." · ".." 같은 점만 있는 이름은 안 됨
FILE_RE = re.compile(r"^[A-Za-z0-9_.\-]+\.(json|safetensors)$")


def install_zip(zpath, filename="model.zip", model_dir=None):
    """model zip → <model_dir>/<이름> (같은 이름이 있으면 바꿈). → 이름"""
    model_dir = model_dir or MODEL_DIR
    os.makedirs(model_dir, exist_ok=True)
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
        dst = os.path.join(model_dir, name)
        if not NAME_RE.match(name) or os.path.dirname(os.path.realpath(dst)) != os.path.realpath(model_dir):
            raise ValueError(f"모델 이름 이상: {name}")
        tmp = dst + ".tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        for i, rel in keep:
            with z.open(i) as src, open(os.path.join(tmp, rel), "wb") as out:
                shutil.copyfileobj(src, out)
        shutil.rmtree(dst, ignore_errors=True)
        os.replace(tmp, dst)
    return name
