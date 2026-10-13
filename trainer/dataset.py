"""
dataset.py — 로봇 PC(physical_ai/policy_server.py)가 기록한 에피소드 → ACT 학습 샘플

  <dataset>/info.json
  <dataset>/episodes/ep_000001/episode.npz   t, state(N,14), action(N,14), ee(N,6), stage, weight, img_t,
                                             jpeg(M,) uint8 (JPEG 를 이어 붙인 것), jpeg_off(N+1,)
  <dataset>/episodes/ep_000001/meta.json     ok, fps, image{w,h}, joint_names, ...

샘플 (프레임 t):  image t, state t, action t … t+chunk−1 (끝을 넘으면 마지막 값으로 채우고 is_pad=True)
npz 는 allow_pickle=False 로만 연다 (업로드 파일에서 코드 실행 방지).
"""
import json
import os

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

EP_FORMAT = "h2-episodes-v1"


def load_episodes(root, only_ok=True):
    """→ (episodes 목록, info). 각 episode: dict(id, state, action, ee, jpeg, off, n, meta)."""
    with open(os.path.join(root, "info.json"), encoding="utf-8") as f:
        info = json.load(f)
    eps = []
    ep_dir = os.path.join(root, "episodes")
    for n in sorted(os.listdir(ep_dir)):
        d = os.path.join(ep_dir, n)
        mp, npz = os.path.join(d, "meta.json"), os.path.join(d, "episode.npz")
        if not (os.path.isfile(mp) and os.path.isfile(npz)):
            continue
        with open(mp, encoding="utf-8") as f:
            meta = json.load(f)
        if meta.get("format") != EP_FORMAT:
            raise ValueError(f"{n}: format {meta.get('format')} ≠ {EP_FORMAT}")
        if only_ok and not meta.get("ok"):
            continue
        with np.load(npz, allow_pickle=False) as z:
            e = {"id": n, "meta": meta, "state": z["state"].astype(np.float32), "action": z["action"].astype(np.float32),
                 "ee": z["ee"].astype(np.float32), "jpeg": z["jpeg"], "off": z["jpeg_off"].astype(np.int64),
                 "stage": z["stage"]}
            if "ee_cmd" in z:          # 지령 손 위치 (로봇 PC policy_server 1.4 부터) — 로봇 안전 필터 기준
                e["ee_cmd"] = z["ee_cmd"].astype(np.float32)
        lens = (len(e["state"]), len(e["action"]), len(e["ee"]), len(e["stage"]), len(e["off"]) - 1)
        e["n"] = min(lens)
        if len(set(lens)) != 1:
            # 로봇 PC policy_server 1.1 이하: 기록 끝에서 마지막 프레임이 일부 배열에만 들어간 채 저장될 수 있었음 (1.2 에서 고침).
            #   앞 n 프레임은 같은 틱들이라 짧은 쪽에 맞춰 씀
            print(f"[dataset] {n}: 배열 길이 다름 (state, action, ee, stage, jpeg) = {lens} → 앞 {e['n']} 프레임만 씀", flush=True)
            for k in ("state", "action", "ee", "ee_cmd", "stage"):
                if k in e:
                    e[k] = e[k][:e["n"]]
            e["off"] = e["off"][:e["n"] + 1]
        if e["n"] < 2 or e["action"].shape != e["state"].shape:
            print(f"[dataset] {n}: 프레임 {e['n']} 개 — 건너뜀", flush=True)
            continue
        eps.append(e)
    if not eps:
        raise ValueError("쓸 에피소드 없음 (성공 에피소드 0)")
    names = [tuple(e["meta"].get("joint_names") or []) for e in eps]
    if len(set(names)) != 1:
        raise ValueError("에피소드마다 joint_names 가 다름")
    return eps, info


def compute_stats(eps, fps):
    """정규화 통계(학습 에피소드) + 로봇 안전 필터용 범위 (모든 에피소드)."""
    S = np.concatenate([e["state"] for e in eps])
    A = np.concatenate([e["action"] for e in eps])
    EE = np.concatenate([e["ee"] for e in eps])
    gap = np.linalg.norm(EE[:, :3] - EE[:, 3:], axis=1)
    vel = np.concatenate([np.abs(np.diff(e["action"], axis=0)) * fps for e in eps])
    cmd = {}
    if all("ee_cmd" in e for e in eps):    # 지령 손 위치 — 로봇 안전 필터가 정책 지령을 검사하는 기준 (실측 ee 는 박스에 막혀 바깥)
        EC = np.concatenate([e["ee_cmd"] for e in eps])
        gc = np.linalg.norm(EC[:, :3] - EC[:, 3:], axis=1)
        cmd = {"ee_cmd_min": EC.min(0).tolist(), "ee_cmd_max": EC.max(0).tolist(),
               "hand_gap_cmd_min": float(gc.min()), "hand_gap_cmd_max": float(gc.max())}
    return {**cmd,
        "state_mean": S.mean(0).tolist(), "state_std": np.maximum(S.std(0), 1e-3).tolist(),
        "action_mean": A.mean(0).tolist(), "action_std": np.maximum(A.std(0), 1e-3).tolist(),
        "action_min": A.min(0).tolist(), "action_max": A.max(0).tolist(),
        "state_min": S.min(0).tolist(), "state_max": S.max(0).tolist(),
        "ee_min": EE.min(0).tolist(), "ee_max": EE.max(0).tolist(),
        "hand_gap_min": float(gap.min()), "hand_gap_max": float(gap.max()),
        "max_vel": vel.max(0).tolist(),
    }


def decode(jpeg, off, i, h, w):
    img = cv2.imdecode(jpeg[off[i]:off[i + 1]], cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("JPEG 디코딩 실패")
    if img.shape[0] != h or img.shape[1] != w:
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)   # 로봇 pa_runtime.OVPolicy.preprocess 와 같음
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


class EpisodeDataset(Dataset):
    """이미지는 uint8 (3,H,W), 정규화·증강은 학습 루프(GPU)에서."""

    def __init__(self, eps, chunk, h, w, stats):
        self.eps, self.chunk, self.h, self.w = eps, chunk, h, w
        self.index = [(k, t) for k, e in enumerate(eps) for t in range(e["n"])]
        self.s_mean = np.asarray(stats["state_mean"], np.float32)
        self.s_std = np.asarray(stats["state_std"], np.float32)
        self.a_mean = np.asarray(stats["action_mean"], np.float32)
        self.a_std = np.asarray(stats["action_std"], np.float32)

    def __len__(self):
        return len(self.index)

    def __getitem__(self, i):
        k, t = self.index[i]
        e = self.eps[k]
        img = decode(e["jpeg"], e["off"], t, self.h, self.w)
        idx = np.arange(t, t + self.chunk)
        pad = idx >= e["n"]
        act = e["action"][np.minimum(idx, e["n"] - 1)]
        return {"image": torch.from_numpy(np.ascontiguousarray(img.transpose(2, 0, 1))),
                "state": torch.from_numpy((e["state"][t] - self.s_mean) / self.s_std),
                "action": torch.from_numpy((act - self.a_mean) / self.a_std),
                "is_pad": torch.from_numpy(pad)}
