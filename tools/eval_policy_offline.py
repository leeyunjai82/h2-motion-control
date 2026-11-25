#!/usr/bin/env python3
"""
eval_policy_offline.py — 로봇 없이 모델 확인: 수집한 실제 영상 · 관절값을 모델에 넣고, 모델 출력 ↔ 사람이 한 동작 비교.

  python tools/eval_policy_offline.py                                   모델 목록
  python tools/eval_policy_offline.py <모델>                             학습한 데이터셋 (h2_model.json) 에서 에피소드 5 개
  python tools/eval_policy_offline.py <모델> --dataset <이름> --episodes all --device CPU

  추론은 실기와 같은 길 (pitron.learning.lerobot_infer 프로세스 · ACT 는 OpenVINO). 에피소드마다 reset 뒤 프레임 순서대로.
  실기와 다른 점: 관절 입력이 로봇 실측이 아니라 녹화값 (모델 출력이 팔을 움직이지 않음 — open-loop)
  결과
    · 관절 오차: 모델 출력 ↔ 녹화 action (팔 14 관절 [deg] · 그리퍼 2 [q])
    · 실기 안전 필터 미리 보기: 학습 범위 (range_tol) · 한 틱 점프 (jump_rad) — 실기였으면 멈췄을 프레임 수 · 처음 프레임
    · data/logs/policy_eval_<모델>_<시각>/ep_<번호>.png (회색 = 녹화, 하늘색 = 모델, 빨강 = 실기라면 멈춤) · summary.json
  lerobot 가상환경 (수집 · 추론) 에서 돎 — 다른 python 으로 실행하면 그 환경으로 다시 실행함.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, ROOT)

try:
    import lerobot  # noqa: F401
except ImportError:
    from pitron.learning import lerobot_env
    py = lerobot_env.python()
    if not os.path.isfile(py) or os.environ.get("H2_EVAL_REEXEC"):           # 다시 실행해도 lerobot 이 없으면 끝 (무한 반복 막음)
        sys.exit(f"❌ lerobot 없음 — {lerobot_env.missing_msg(py)}")
    os.environ["H2_EVAL_REEXEC"] = "1"
    os.execv(py, [py, os.path.abspath(__file__)] + sys.argv[1:])

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from pitron import paths  # noqa: E402

MODEL_DIR = paths.data_dir("policies", legacy="gripper/models")
DS_DIR = paths.data_dir("datasets", legacy="gripper/data")
# 실기 안전 필터와 같은 기준 (apps/gripper_pick/policy.py DEFAULTS · PARTS)
RANGE_TOL, JUMP_RAD = 0.15, 0.15
ARM = list(range(0, 7)) + list(range(8, 15))        # action 16 = 왼팔 7 · 왼 그리퍼 · 오른팔 7 · 오른 그리퍼
GRIP = [7, 15]


def read_json(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def list_models():
    ms = sorted(n for n in (os.listdir(MODEL_DIR) if os.path.isdir(MODEL_DIR) else [])
                if os.path.isfile(os.path.join(MODEL_DIR, n, "config.json")))
    print(f"lerobot 모델 ({MODEL_DIR}): {len(ms)} 개")
    for n in ms:
        h2 = read_json(os.path.join(MODEL_DIR, n, "h2_model.json")) or {}
        print(f"  {n:40s} {h2.get('policy') or '-':10s} 데이터셋 {h2.get('dataset') or '?'}  step {h2.get('step') or '?'}")
    if ms:
        print(f"\n실행 예:  python tools/eval_policy_offline.py {ms[-1]}")


class Infer:
    """lerobot_infer 프로세스 (실기와 같은 주고받기)."""

    def __init__(self, model_dir, device, task, n_steps=0, backend="auto"):
        cfg = read_json(os.path.join(model_dir, "config.json")) or {}
        if backend == "torch":
            if device != "CPU":
                raise SystemExit("❌ torch 추론은 CPU 로만 (실기와 같음) — --device CPU")
            args = ["--backend", "torch", "--device", "cpu"]
        elif cfg.get("type") == "act":
            args = ["--backend", "openvino", "--ov-device", device]
        elif device != "CPU":
            raise SystemExit(f"❌ ACT 외 정책 ({cfg.get('type')}) 은 CPU 로만 (실기와 같음) — --device CPU")
        else:
            args = ["--backend", "torch", "--device", "cpu"]
        self.p = subprocess.Popen([sys.executable, "-u", "-m", "pitron.learning.lerobot_infer", "--model", model_dir,
                                   "--task", task, "--n-action-steps", str(n_steps)] + args, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, env={**os.environ, "HF_HUB_OFFLINE": "1"})
        self.info = self._reply()
        if not self.info.get("ok"):
            raise SystemExit(f"❌ 모델 불러오기 실패: {self.info.get('err')}")

    def _reply(self):
        while True:
            ln = self.p.stdout.readline()
            if not ln:
                raise SystemExit("❌ 추론 프로세스가 끝남")
            ln = ln.decode("utf-8", "replace").strip()
            if ln.startswith("@@"):
                return json.loads(ln[2:])

    def reset(self):
        self.p.stdin.write(b'{"op": "reset"}\n')
        self.p.stdin.flush()
        self._reply()

    def act(self, state, imgs):
        self.p.stdin.write((json.dumps({"op": "act", "state": [float(x) for x in state]}) + "\n").encode())
        for im in imgs:
            self.p.stdin.write(np.ascontiguousarray(im, np.uint8).tobytes())
        self.p.stdin.flush()
        r = self._reply()
        if not r.get("ok"):
            raise SystemExit(f"❌ 추론 오류: {r.get('err')}")
        return np.asarray(r["action"], np.float64), r["ms"]

    def close(self):
        try:
            self.p.stdin.write(b'{"op": "quit"}\n')
            self.p.stdin.flush()
            self.p.wait(timeout=5)
        except Exception:  # noqa: BLE001
            self.p.kill()


def plot(path, title, rec, pred, stop_frames, names):
    """4 × 4 칸: 관절마다 녹화 (회색) · 모델 (하늘색) · 실기라면 멈춤 (빨강 세로줄)."""
    W, H, cols, rows, pad = 1600, 1000, 4, 4, 28
    img = np.full((H, W, 3), (22, 17, 14), np.uint8)
    cv2.putText(img, title, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    cw, ch = W // cols, (H - pad) // rows
    n = len(rec)
    for k in range(rec.shape[1]):
        x0, y0 = (k % cols) * cw, pad + (k // cols) * ch
        x1, y1, x2, y2 = x0 + 8, y0 + 20, x0 + cw - 8, y0 + ch - 8
        cv2.rectangle(img, (x1, y1), (x2, y2), (64, 51, 42), 1)
        unit = 1.0 if k in GRIP else 180.0 / math.pi
        r, p = rec[:, k] * unit, pred[:, k] * unit
        lo, hi = float(min(r.min(), p.min())), float(max(r.max(), p.max()))
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1

        def pts(v):
            xs = x1 + (np.arange(n) / max(1, n - 1)) * (x2 - x1)
            ys = y2 - (v - lo) / (hi - lo) * (y2 - y1)
            return np.stack([xs, ys], 1).astype(np.int32).reshape(-1, 1, 2)

        for f in stop_frames:
            xf = int(x1 + f / max(1, n - 1) * (x2 - x1))
            cv2.line(img, (xf, y1), (xf, y2), (60, 60, 200), 1)
        cv2.polylines(img, [pts(r)], False, (150, 150, 150), 1, cv2.LINE_AA)
        cv2.polylines(img, [pts(p)], False, (235, 183, 0), 1, cv2.LINE_AA)
        err = float(np.mean(np.abs(p - r)))
        lab = f"{names[k][:26]}  err {err:.1f}{'' if k in GRIP else ' deg'}"
        cv2.putText(img, lab, (x1 + 2, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser(description="수집한 실제 영상으로 모델 출력 ↔ 녹화 동작 비교 (로봇 없이)")
    ap.add_argument("model", nargs="?")
    ap.add_argument("--dataset", help="데이터셋 이름 (기본: 모델을 학습한 데이터셋, h2_model.json)")
    ap.add_argument("--episodes", default="5", help="개수 (고르게) · all · 번호 목록 (예 0,3,7)")
    ap.add_argument("--device", default="CPU", help="OpenVINO 장치 CPU · GPU · NPU (ACT) — ACT 외는 CPU")
    ap.add_argument("--backend", default="auto", choices=("auto", "torch"),
                    help="auto = 실기와 같음 (ACT 는 OpenVINO) · torch = lerobot 그대로 (OpenVINO 없이, 비교용)")
    ap.add_argument("--n-action-steps", type=int, default=0, help="chunk 에서 실행할 동작 수 (0 = 모델 설정 그대로) — 예 20: 0.67 초마다 다시 계산")
    a = ap.parse_args()
    if not a.model:
        list_models()
        return 0
    md = os.path.join(MODEL_DIR, a.model)
    if not os.path.isfile(os.path.join(md, "config.json")):
        list_models()
        return f"\n❌ 모델 없음: {a.model}"
    h2m = read_json(os.path.join(md, "h2_model.json")) or {}
    ds_name = a.dataset or h2m.get("dataset")
    if not ds_name:
        return "❌ 데이터셋을 모름 (h2_model.json 없음) — --dataset <이름>"
    root = os.path.join(DS_DIR, ds_name, "lerobot")
    if not os.path.isfile(os.path.join(root, "meta", "info.json")):
        return f"❌ LeRobot v3 데이터셋 없음: {root}"
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata, LeRobotDataset
    meta = LeRobotDatasetMetadata(f"local/{ds_name}", root=root)
    total = meta.total_episodes
    if a.episodes == "all":
        eps = list(range(total))
    elif "," in a.episodes:
        eps = [int(x) for x in a.episodes.split(",") if x.strip()]
    else:
        k = max(1, min(int(a.episodes), total))
        eps = sorted({round(i * (total - 1) / max(1, k - 1)) for i in range(k)}) if k > 1 else [0]
    h2d = read_json(os.path.join(root, "meta", "h2.json")) or {}
    names = h2d.get("state_names") or [f"j{i}" for i in range(16)]
    if h2m.get("state_names") and h2m["state_names"] != names:
        return "❌ 모델 관절 순서 ≠ 데이터셋 관절 순서 (h2_model.json state_names)"
    task = str(h2d.get("task") or h2m.get("task") or "")

    print(f"[평가] 모델 {a.model} · 데이터셋 {ds_name} (에피소드 {total} 중 {len(eps)}) · {a.device}", flush=True)
    inf = Infer(md, a.device, task, a.n_action_steps, a.backend)
    info = inf.info
    print(f"  불러옴: {info['type']} · {info['backend']} · chunk {info.get('chunk_size')} 중 {info.get('n_action_steps')} 개씩 실행 · 첫 추론 {info['infer_ms']} ms · 카메라 "
          + ", ".join(f"{c['name']} {c['w']}×{c['h']}" for c in info["cams"]), flush=True)
    amin = np.asarray(info["a_min"]) if info.get("a_min") else None
    amax = np.asarray(info["a_max"]) if info.get("a_max") else None
    out = os.path.join(paths.DATA_DIR, "logs", f"policy_eval_{a.model}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out, exist_ok=True)
    rows = []
    try:
        for e in eps:
            ds = LeRobotDataset(f"local/{ds_name}", root=root, episodes=[e])
            missing = [c["name"] for c in info["cams"] if f"observation.images.{c['name']}" not in ds.meta.features]
            if missing:
                return f"❌ 데이터셋에 없는 카메라: {missing}"
            inf.reset()
            rec, pred, ms = [], [], []
            stop_range, stop_jump = [], []
            target = None
            jumps = []
            why = {}                                                            # 멈춤 원인 관절: (종류, 이름) → [회수, 최대]
            for i in range(len(ds)):
                x = ds[i]
                imgs = []
                for c in info["cams"]:
                    t = x[f"observation.images.{c['name']}"]                    # float CHW 0–1
                    im = (t.permute(1, 2, 0).numpy() * 255.0 + 0.5).clip(0, 255).astype(np.uint8)
                    if im.shape[:2] != (c["h"], c["w"]):
                        return f"❌ 영상 크기 {im.shape[1]}×{im.shape[0]} ≠ 모델 {c['w']}×{c['h']} ({c['name']})"
                    imgs.append(im)
                state = x["observation.state"].numpy().astype(np.float64)
                act, t_ms = inf.act(state, imgs)
                ms.append(t_ms)
                rec.append(x["action"].numpy().astype(np.float64))
                pred.append(act)
                if not np.all(np.isfinite(act)):
                    stop_range.append(i)
                    continue
                a_cl = act
                if amin is not None:                                            # 실기: 범위 밖 → 멈춤, 안쪽은 자름
                    over = np.maximum(amin - act, act - amax)
                    if float(np.max(over)) > RANGE_TOL:
                        stop_range.append(i)
                        k = int(np.argmax(over))
                        v = float(over[k]) if k in GRIP else math.degrees(over[k])
                        w = why.setdefault(("범위", names[k] + (" (q)" if k in GRIP else "")), [0, 0.0])
                        w[0], w[1] = w[0] + 1, max(w[1], v)
                    a_cl = np.clip(act, amin, amax)
                cmd = a_cl[ARM]
                if target is None:
                    target = state[ARM]                                         # 실기: 시작 = 실측 (준비 자세)
                jd = np.abs(cmd - target)
                jumps.append(float(np.max(jd)))
                if jumps[-1] > JUMP_RAD:                                        # 실기: 한 틱 점프 → 멈춤
                    stop_jump.append(i)
                    k = ARM[int(np.argmax(jd))]
                    w = why.setdefault(("점프", names[k]), [0, 0.0])
                    w[0], w[1] = w[0] + 1, max(w[1], math.degrees(float(np.max(jd))))
                target = cmd
            rec, pred = np.asarray(rec), np.asarray(pred)
            err = np.abs(pred - rec)
            arm_deg = np.degrees(err[:, ARM])
            row = {"episode": e, "frames": len(rec), "arm_mean_deg": round(float(arm_deg.mean()), 2),
                   "arm_p95_deg": round(float(np.percentile(arm_deg, 95)), 2), "arm_max_deg": round(float(arm_deg.max()), 1),
                   "grip_mean_q": round(float(err[:, GRIP].mean()), 3),
                   "stop_range": len(stop_range), "stop_jump": len(stop_jump),
                   "jump_max_deg": round(math.degrees(max(jumps)), 1) if jumps else None,
                   "stop_frames": sorted(set(stop_range + stop_jump))[:12],
                   "stop_joints": [{"kind": kd, "joint": jn, "count": c, "max": round(m, 2)} for (kd, jn), (c, m) in why.items()],
                   "first_stop_s": round(min(stop_range + stop_jump) / 30.0, 1) if (stop_range or stop_jump) else None,
                   "infer_ms_mean": round(float(np.mean(ms)), 1), "infer_ms_max": round(float(np.max(ms)), 1)}
            rows.append(row)
            stops = sorted(set(stop_range + stop_jump))
            plot(os.path.join(out, f"ep_{e:03d}.png"),
                 f"{a.model} | {ds_name} ep {e} | gray = recorded, cyan = model, red = real robot would stop", rec, pred, stops, names)
            fs = "없음" if row["first_stop_s"] is None else (f"{row['first_stop_s']} s (범위 {row['stop_range']} · 점프 {row['stop_jump']} 프레임,"
                                                             f" 프레임 {', '.join(map(str, row['stop_frames']))})")
            print(f"  ep {e:3d}  {row['frames']:5d} 프레임  팔 오차 평균 {row['arm_mean_deg']:5.2f}° · 95% {row['arm_p95_deg']:5.1f}° · 최대 {row['arm_max_deg']:5.1f}°"
                  f"  그리퍼 {row['grip_mean_q']:.3f}  추론 평균 {row['infer_ms_mean']} · 최대 {row['infer_ms_max']} ms"
                  f"  한 틱 최대 {row['jump_max_deg']}°\n           실기라면 멈춤: {fs}", flush=True)
            if why:
                print("           원인 관절: " + " · ".join(f"{kd} {jn} ×{c} (최대 {m:.2f}{'' if jn.endswith('(q)') else '°'}{' 넘음' if kd == '범위' else ''})"
                                                     for (kd, jn), (c, m) in sorted(why.items(), key=lambda x: -x[1][0])), flush=True)
    finally:
        inf.close()
    s = {"model": a.model, "dataset": ds_name, "device": a.device, "backend": info["backend"],
         "chunk_size": info.get("chunk_size"), "n_action_steps": info.get("n_action_steps"),
         "range_tol": RANGE_TOL, "jump_rad": JUMP_RAD, "episodes": rows,
         "arm_mean_deg": round(float(np.mean([r["arm_mean_deg"] for r in rows])), 2),
         "episodes_with_stop": sum(r["first_stop_s"] is not None for r in rows)}
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(s, f, ensure_ascii=False, indent=1)
    print(f"\n팔 오차 평균 {s['arm_mean_deg']}° · 실기라면 멈췄을 에피소드 {s['episodes_with_stop']} / {len(rows)}")
    print("  (open-loop: 관절 입력이 녹화값 — 실기에서는 모델 출력이 팔을 움직이고 그 실측이 다시 들어감)")
    print(f"결과: {out}  (ep_*.png · summary.json)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
