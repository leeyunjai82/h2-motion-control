#!/usr/bin/env python3
"""
lerobot_sync.py — 수집 원본 (h2-gripper-episodes-v1) → LeRobot 데이터셋 v3.0.  lerobot 가상환경의 python 으로 돎 (lerobot 0.4.x).

  수집 PC (pick_web 이 에피소드 저장 · 지우기 뒤 자동으로 부름):
    python -m pitron.learning.lerobot_sync data/datasets/pick_1124
      → data/datasets/pick_1124/lerobot/   새 성공 에피소드만 이어 붙임. 지운 에피소드 · 작업 설명 바뀜 → 처음부터 다시 만듦
  아무 폴더로 내보내기 (예전 데이터셋 폴더 · zip):
    python -m pitron.learning.lerobot_sync old_dataset.zip --out ~/lerobot_data/old --task "pick up the cube"

  바꾸는 것
    state(16)  → observation.state   (이름 = info.json state_names)
    action(16) → action
    카메라 JPEG → observation.images.<카메라> (영상, 기본 h264 — 브라우저 · 뷰어에서 바로 재생)
    tcp(6)  → tcp   · stage → stage    (observation · action 이 아니라 학습 입력에는 안 들어감, 분석용)
    timestamp = 프레임 번호 / fps
  성공 에피소드만 넣음 (--all 이면 실패도). 에피소드별 손 · 결과 · 원본 이름은 meta/h2.json.
  진행은 '@@{json}' 줄 (pick_web 이 읽음).
"""
import argparse
import datetime
import fcntl
import json
import os
import shutil
import sys
import tempfile
import zipfile

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from pitron.learning import formats  # noqa: E402

H2_META = "h2.json"
H2_FORMAT = "h2-lerobot-v1"
DEFAULT_TASK = "pick up the object"


def say(**kw):
    print("@@" + json.dumps(kw, ensure_ascii=False), flush=True)


def open_raw(src, tmp):
    """폴더 또는 zip → 원본 데이터셋 폴더 (info.json 있는 곳)."""
    if zipfile.is_zipfile(src):
        with zipfile.ZipFile(src) as z:
            z.extractall(tmp)
        src = tmp
    if os.path.isfile(os.path.join(src, "info.json")):
        return src
    subs = [os.path.join(src, d) for d in os.listdir(src) if os.path.isfile(os.path.join(src, d, "info.json"))]
    if len(subs) != 1:
        sys.exit(f"❌ 원본 데이터셋을 못 찾음 (info.json): {src}")
    return subs[0]


def raw_episodes(root, include_fail=False):
    ep_root = os.path.join(root, "episodes")
    out = []
    for e in sorted(os.listdir(ep_root)) if os.path.isdir(ep_root) else []:
        mp = os.path.join(ep_root, e, "meta.json")
        if formats.EPISODE_RE.match(e) and os.path.isfile(mp) and os.path.isfile(os.path.join(ep_root, e, "episode.npz")):
            with open(mp, encoding="utf-8") as f:
                m = json.load(f)
            if include_fail or m.get("ok"):
                out.append((e, m))
    return out


def features_of(info):
    names = list(info["state_names"])
    return {
        "observation.state": {"dtype": "float32", "shape": (len(names),), "names": names},
        "action": {"dtype": "float32", "shape": (len(names),), "names": names},
        **{f"observation.images.{c}": {"dtype": "video", "shape": (int(v["h"]), int(v["w"]), 3),
                                       "names": ["height", "width", "channels"]} for c, v in info["cameras"].items()},
        "tcp": {"dtype": "float32", "shape": (6,), "names": ["left_x", "left_y", "left_z", "right_x", "right_y", "right_z"]},
        "stage": {"dtype": "int64", "shape": (1,), "names": None},
    }


def add_episode(ds, root, e, info, task):
    z = np.load(os.path.join(root, "episodes", e, "episode.npz"), allow_pickle=False)
    st, ac = z["state"].astype(np.float32), z["action"].astype(np.float32)
    n_names = len(info["state_names"])
    if st.shape[1] != n_names or ac.shape != st.shape:
        raise ValueError(f"{e}: state {st.shape} · action {ac.shape} ≠ 이름 {n_names} 개")
    tcp = z["tcp"].astype(np.float32) if "tcp" in z.files else np.zeros((len(st), 6), np.float32)
    stage = z["stage"].astype(np.int64) if "stage" in z.files else np.zeros(len(st), np.int64)
    cams = info["cameras"]
    jpg = {c: (z[f"jpeg_{c}"], z[f"jpeg_off_{c}"]) for c in cams}
    for i in range(st.shape[0]):
        fr = {"observation.state": st[i], "action": ac[i], "tcp": tcp[i], "stage": np.array([stage[i]], np.int64), "task": task}
        for c, (buf, off) in jpg.items():
            bgr = cv2.imdecode(buf[int(off[i]):int(off[i + 1])], cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError(f"{e}: {c} 프레임 {i} JPEG 깨짐")
            if bgr.shape[1] != int(cams[c]["w"]) or bgr.shape[0] != int(cams[c]["h"]):
                raise ValueError(f"{e}: {c} 프레임 크기 {bgr.shape[1]}×{bgr.shape[0]} ≠ 데이터셋 {cams[c]['w']}×{cams[c]['h']}")
            fr[f"observation.images.{c}"] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        ds.add_frame(fr)
    ds.save_episode()
    return st.shape[0]


def ep_row(idx, e, m, n):
    return {"index": idx, "raw": e, "side": m.get("side"), "ok": bool(m.get("ok")), "result": m.get("result"),
            "frames": int(n), "created": m.get("created"), "stages": m.get("stages")}


def write_h2(out, h2):
    p = os.path.join(out, "meta", H2_META)
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(h2, f, ensure_ascii=False, indent=1)
    os.replace(p + ".tmp", p)


def read_h2(out):
    p = os.path.join(out, "meta", H2_META)
    if not os.path.isfile(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def build(root, out, repo_id, task, eps, info, vcodec, robot_type, append_from=None):
    """eps [(e, meta)] 를 out 에 넣음. append_from = 기존 h2 (이어 붙이기) 또는 None (새로 만듦)."""
    from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    if CODEBASE_VERSION != "v3.0":
        sys.exit(f"❌ 이 lerobot 은 데이터셋 {CODEBASE_VERSION} — v3.0 (lerobot 0.4.x) 필요")
    fps = int(round(float(info.get("fps", formats.FPS))))
    if append_from is None:
        ds = LeRobotDataset.create(repo_id, fps=fps, features=features_of(info), root=out, robot_type=robot_type,
                                   use_videos=True, vcodec=vcodec)
        rows = []
    else:
        ds = LeRobotDataset(repo_id, root=out, vcodec=vcodec)
        rows = list(append_from["episodes"])
    for k, (e, m) in enumerate(eps):
        n = add_episode(ds, root, e, info, task)
        rows.append(ep_row(len(rows), e, m, n))
        say(stage="episode", done=k + 1, total=len(eps), ep=e, frames=n)
    ds.finalize()
    shutil.rmtree(os.path.join(out, "images"), ignore_errors=True)     # 영상 인코딩 전 임시 프레임 폴더 (빈 폴더만 남음)
    h2 ={"format": H2_FORMAT, "source_format": info.get("format"), "task": task, "fps": fps,
          "cameras": info["cameras"], "fit": info.get("fit", "stretch"), "state_names": info["state_names"],
          "robot_type": robot_type, "repo_id": repo_id, "episodes": rows,
          "updated": datetime.datetime.now().isoformat(timespec="seconds")}
    write_h2(out, h2)
    chk = LeRobotDataset(repo_id, root=out)
    if chk.num_episodes != len(rows) or chk.num_frames != sum(r["frames"] for r in rows):
        sys.exit(f"❌ 개수가 다름 — 에피소드 {chk.num_episodes}/{len(rows)} · 프레임 {chk.num_frames}/{sum(r['frames'] for r in rows)}")
    return h2


def sync(ds_dir, vcodec="h264", robot_type="unitree_h2_dex1", repo_prefix="local"):
    """수집 데이터셋 폴더 → <폴더>/lerobot (이어 붙이기 또는 다시 만들기)."""
    ds_dir = os.path.abspath(ds_dir)
    with open(os.path.join(ds_dir, "info.json"), encoding="utf-8") as f:
        info = json.load(f)
    if info.get("format") != formats.DATASET_FORMAT:
        sys.exit(f"❌ format {info.get('format')} ≠ {formats.DATASET_FORMAT}")
    task = str(info.get("task") or DEFAULT_TASK)
    out = os.path.join(ds_dir, "lerobot")
    repo_id = f"{repo_prefix}/{os.path.basename(ds_dir)}"
    lock = open(os.path.join(ds_dir, ".lerobot.lock"), "w")
    fcntl.flock(lock, fcntl.LOCK_EX)                     # 한 데이터셋에 한 프로세스만
    try:
        eps = raw_episodes(ds_dir)
        h2 = read_h2(out) if os.path.isdir(out) else None
        have = [r["raw"] for r in (h2 or {}).get("episodes", [])]
        want = [e for e, _ in eps]
        if h2 and h2.get("task") == task and want[:len(have)] == have and os.path.isfile(os.path.join(out, "meta", "info.json")):
            new = eps[len(have):]
            if not new:
                say(stage="done", episodes=len(have), mode="same")
                return
            say(stage="start", mode="append", new=len(new), total=len(want))
            build(ds_dir, out, repo_id, task, new, info, vcodec, robot_type, append_from=h2)
            say(stage="done", episodes=len(want), mode="append")
            return
        if not eps:                                      # 성공 에피소드 없음 → v3 없음
            shutil.rmtree(out, ignore_errors=True)
            say(stage="done", episodes=0, mode="empty")
            return
        tmp = out + ".tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        say(stage="start", mode="rebuild", new=len(eps), total=len(eps))
        build(ds_dir, tmp, repo_id, task, eps, info, vcodec, robot_type)
        shutil.rmtree(out, ignore_errors=True)
        os.replace(tmp, out)
        say(stage="done", episodes=len(eps), mode="rebuild")
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


def main():
    ap = argparse.ArgumentParser(description="수집 원본 → LeRobot v3.0")
    ap.add_argument("src", help="수집 데이터셋 폴더 (또는 --out 과 함께 예전 데이터셋 zip)")
    ap.add_argument("--out", help="새 폴더로 내보내기 (없어야 함). 없으면 <src>/lerobot 를 맞춤")
    ap.add_argument("--repo-id", help="예: local/pick_1124 (기본 <prefix>/<이름>)")
    ap.add_argument("--repo-prefix", default="local")
    ap.add_argument("--task", help="작업 설명 (기본 info.json task)")
    ap.add_argument("--all", action="store_true", help="--out: 실패 에피소드도 포함")
    ap.add_argument("--vcodec", default="h264", help="영상 코덱 (h264 · libsvtav1)")
    ap.add_argument("--robot-type", default="unitree_h2_dex1")
    a = ap.parse_args()
    if not a.out:
        sync(a.src, a.vcodec, a.robot_type, a.repo_prefix)
        return
    out = os.path.abspath(os.path.expanduser(a.out))
    if os.path.exists(out):
        sys.exit(f"❌ 이미 있음: {out} (지우거나 다른 폴더)")
    with tempfile.TemporaryDirectory() as tmp:
        root = open_raw(os.path.abspath(os.path.expanduser(a.src)), tmp)
        with open(os.path.join(root, "info.json"), encoding="utf-8") as f:
            info = json.load(f)
        if info.get("format") != formats.DATASET_FORMAT:
            sys.exit(f"❌ format {info.get('format')} ≠ {formats.DATASET_FORMAT}")
        eps = raw_episodes(root, a.all)
        if not eps:
            sys.exit("❌ 넣을 에피소드 없음 (성공 에피소드 0 — 실패까지 넣으려면 --all)")
        task = a.task or str(info.get("task") or DEFAULT_TASK)
        h2 = build(root, out, a.repo_id or f"{a.repo_prefix}/{os.path.basename(out)}", task, eps, info, a.vcodec, a.robot_type)
    print(f"[lerobot] ✓ {out} — 에피소드 {len(h2['episodes'])} · 프레임 {sum(r['frames'] for r in h2['episodes'])}")


if __name__ == "__main__":
    main()
