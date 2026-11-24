#!/usr/bin/env python3
"""
export_lerobot.py — 우리 그리퍼 데이터셋 (h2-gripper-episodes-v1) → LeRobot 데이터셋 v3.0

  lerobot 0.4.x 가 깔린 환경에서 (예: 5090 의 lerobot 가상환경 — 이 저장소의 tv · act 환경에는 lerobot 없음):
    python tools/export_lerobot.py data/datasets/pick_20261110 --out ~/lerobot_data/pick_20261110 \
        --repo-id circulus/pick_20261110 --task "pick up the cube and put it in the bin"
    입력은 데이터셋 폴더 또는 수집 화면에서 받은 데이터셋 zip.  옵션: --all (실패 에피소드 포함) · --vcodec h264 · --image (영상 대신 프레임 이미지)

  바꾸는 것
    state(16)  → observation.state   (이름 = info.json state_names: 왼팔 7 · 왼 그리퍼 q · 오른팔 7 · 오른 그리퍼 q)
    action(16) → action
    카메라 JPEG → observation.images.<카메라> (MP4 영상, RGB)
    timestamp  = 프레임 번호 / fps (LeRobot 이 동기 검사에 씀 — 녹화 시각 t 는 30 Hz 기준과 몇 ms 차이 날 수 있음)
  안 넣는 것: tcp · stage (ACT 학습에 안 씀)
  끝에 LeRobotDataset 으로 다시 열어 에피소드 · 프레임 수를 확인함
"""
import argparse
import json
import os
import sys
import tempfile
import zipfile

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pitron.learning import formats  # noqa: E402 — 표준 라이브러리만


def open_dataset(src, tmp):
    """폴더 또는 zip → 데이터셋 폴더 (info.json 이 있는 곳)."""
    if zipfile.is_zipfile(src):
        with zipfile.ZipFile(src) as z:
            z.extractall(tmp)
        src = tmp
    if os.path.isfile(os.path.join(src, "info.json")):
        return src
    subs = [os.path.join(src, d) for d in os.listdir(src) if os.path.isfile(os.path.join(src, d, "info.json"))]
    if len(subs) != 1:
        sys.exit(f"❌ 데이터셋을 못 찾음 (info.json): {src}")
    return subs[0]


def main():
    ap = argparse.ArgumentParser(description="h2-gripper-episodes-v1 → LeRobot v3.0")
    ap.add_argument("src", help="데이터셋 폴더 또는 데이터셋 zip")
    ap.add_argument("--out", required=True, help="LeRobot 데이터셋을 만들 폴더 (없어야 함)")
    ap.add_argument("--repo-id", required=True, help="예: circulus/pick_20261110 (Hub 에 올리지 않아도 이름으로 씀)")
    ap.add_argument("--task", required=True, help="작업 설명 문장 (모든 에피소드에 같은 문장)")
    ap.add_argument("--all", action="store_true", help="실패 에피소드도 포함 (기본: 성공만 — 학습과 같음)")
    ap.add_argument("--vcodec", default="libsvtav1", help="영상 코덱 (LeRobot 기본 libsvtav1, 안 되면 h264)")
    ap.add_argument("--image", action="store_true", help="영상 대신 프레임 이미지로 저장")
    ap.add_argument("--robot-type", default="unitree_h2_dex1")
    a = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    if CODEBASE_VERSION != "v3.0":
        sys.exit(f"❌ 이 lerobot 은 데이터셋 {CODEBASE_VERSION} — v3.0 (lerobot 0.4.x) 필요")
    if os.path.exists(a.out):
        sys.exit(f"❌ 이미 있음: {a.out} (지우거나 다른 폴더)")

    with tempfile.TemporaryDirectory() as tmp:
        root = open_dataset(os.path.abspath(os.path.expanduser(a.src)), tmp)
        with open(os.path.join(root, "info.json"), encoding="utf-8") as f:
            info = json.load(f)
        if info.get("format") != formats.DATASET_FORMAT:
            sys.exit(f"❌ format {info.get('format')} ≠ {formats.DATASET_FORMAT}")
        cams = info["cameras"]                                   # {이름: {w, h}}
        names = list(info["state_names"])
        fps = int(round(float(info.get("fps", formats.FPS))))
        ep_root = os.path.join(root, "episodes")
        eps = []
        for e in sorted(os.listdir(ep_root)):
            mp = os.path.join(ep_root, e, "meta.json")
            if formats.EPISODE_RE.match(e) and os.path.isfile(mp):
                with open(mp, encoding="utf-8") as f:
                    m = json.load(f)
                if a.all or m.get("ok"):
                    eps.append((e, m))
        if not eps:
            sys.exit("❌ 넣을 에피소드 없음 (성공 에피소드 0 — 실패까지 넣으려면 --all)")

        vis = "image" if a.image else "video"
        features = {
            "observation.state": {"dtype": "float32", "shape": (len(names),), "names": names},
            "action": {"dtype": "float32", "shape": (len(names),), "names": names},
            **{f"observation.images.{c}": {"dtype": vis, "shape": (int(v["h"]), int(v["w"]), 3),
                                           "names": ["height", "width", "channels"]} for c, v in cams.items()},
        }
        print(f"[export] {os.path.basename(root)} → {a.out}  ({len(eps)} 에피소드, {fps} fps, 카메라 {list(cams)}, {vis} {a.vcodec if vis == 'video' else ''})")
        ds = LeRobotDataset.create(a.repo_id, fps=fps, features=features, root=a.out, robot_type=a.robot_type,
                                   use_videos=not a.image, vcodec=a.vcodec)
        n_frames = 0
        for k, (e, m) in enumerate(eps):
            z = np.load(os.path.join(ep_root, e, "episode.npz"), allow_pickle=False)
            st, ac = z["state"].astype(np.float32), z["action"].astype(np.float32)
            if st.shape[1] != len(names) or ac.shape != st.shape:
                sys.exit(f"❌ {e}: state {st.shape} · action {ac.shape} ≠ 이름 {len(names)} 개")
            jpg = {c: (z[f"jpeg_{c}"], z[f"jpeg_off_{c}"]) for c in cams}
            for i in range(st.shape[0]):
                fr = {"observation.state": st[i], "action": ac[i], "task": a.task}
                for c, (buf, off) in jpg.items():
                    bgr = cv2.imdecode(buf[int(off[i]):int(off[i + 1])], cv2.IMREAD_COLOR)
                    if bgr is None:
                        sys.exit(f"❌ {e}: {c} 프레임 {i} JPEG 깨짐")
                    fr[f"observation.images.{c}"] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                ds.add_frame(fr)
            ds.save_episode()
            n_frames += st.shape[0]
            print(f"  {k + 1}/{len(eps)} {e}  {st.shape[0]} 프레임  (side {m.get('side')})", flush=True)
        ds.finalize()

    chk = LeRobotDataset(a.repo_id, root=a.out)
    print(f"[export] ✓ 완료 — 다시 열어 확인: 에피소드 {chk.num_episodes} · 프레임 {chk.num_frames} (넣은 것 {len(eps)} · {n_frames})")
    if chk.num_episodes != len(eps) or chk.num_frames != n_frames:
        sys.exit("❌ 개수가 다름 — 로그 확인")


if __name__ == "__main__":
    main()
