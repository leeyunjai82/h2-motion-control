#!/usr/bin/env python3
"""
train_act.py — ACT 학습 → OpenVINO IR 변환 → 로봇용 model zip

  python train_act.py --dataset data/datasets/box_grab --out data/jobs/j001 [--config cfg.json] [--steps 40000]
  python train_act.py --out data/jobs/j001 --export-only            # 저장된 ckpt_last.pt 로 변환만
  python train_act.py --dataset ... --out ... --resume data/jobs/j000/ckpt_last.pt   # 이어 학습

trainer_server.py 가 서브프로세스로 부른다. 진행 상황은 '@@{json}' 줄로 stdout 에 쓴다.
SIGUSR1 → 지금까지 학습한 것으로 바로 변환·마무리 (웹의 '지금 마치기').

결과 (--out 안)
  ckpt_last.pt            가중치·옵티마이저·통계 (이어 학습용)
  model/policy.xml|.bin   OpenVINO IR (FP16 가중치) — 입력 image [1,3,H,W] 0~1 RGB, state [1,14] rad → action [1,chunk,14] rad
  model/meta.json         관절 순서·통계·안전 범위·시작 자세 (로봇 policy_server 가 검사)
  <name>.zip              로봇 웹에 올릴 파일
"""
import argparse
import json
import math
import os
import signal
import sys
import time
import zipfile

import numpy as np
import torch
import torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from act_model import ACT, ACTConfig, ACTExport, IMAGENET_MEAN, IMAGENET_STD   # noqa: E402
from dataset import EpisodeDataset, compute_stats, load_episodes                 # noqa: E402

MODEL_FORMAT = "h2-act-v1"
DEFAULT_TRAIN = {
    "name": "", "steps": 40000, "batch_size": 8, "lr": 1e-5, "lr_backbone": 1e-5, "weight_decay": 1e-4,
    "grad_clip": 10.0, "image": "auto", "chunk_size": 100, "kl_weight": 10.0, "dim_model": 512,
    "dim_feedforward": 3200, "n_heads": 8, "n_encoder_layers": 4, "augment": True, "val_frac": 0.1,
    "seed": 1, "num_workers": 8, "amp": True, "eval_every": 2000, "ckpt_every": 5000, "log_every": 50,
    "pretrained_backbone": True, "only_ok": True,
}
_finish_now = False


def emit(**kw):
    print("@@" + json.dumps(kw, ensure_ascii=False), flush=True)


def log(msg):
    print(f"[train] {msg}", flush=True)


def _on_usr1(signum, frame):
    global _finish_now
    _finish_now = True
    log("SIGUSR1 — 지금까지 학습한 것으로 마무리")


def augment_batch(img_u8, aug, gen=None):
    """uint8 (B,3,H,W) → 정규화된 float. aug: 이동 ±2 %, 밝기·대비·채도 ±20 % (조명·거치대 미세 변화 대비)."""
    x = img_u8.float() / 255.0
    if aug:
        B = x.shape[0]
        dev = x.device
        t = (torch.rand(B, 2, device=dev) * 2 - 1) * 0.04          # affine_grid 좌표는 −1~1 → 0.04 = 폭의 2 %
        theta = torch.zeros(B, 2, 3, device=dev)
        theta[:, 0, 0] = theta[:, 1, 1] = 1.0
        theta[:, :, 2] = t
        grid = F.affine_grid(theta, list(x.shape), align_corners=False)
        x = F.grid_sample(x, grid, mode="bilinear", padding_mode="border", align_corners=False)
        r = lambda: (1 + (torch.rand(B, 1, 1, 1, device=dev) * 2 - 1) * 0.2)
        m = x.mean(dim=(1, 2, 3), keepdim=True)
        x = (x - m) * r() + m
        x = x * r()
        g = x.mean(dim=1, keepdim=True)
        x = (x - g) * r() + g
        x = x.clamp(0, 1)
    mean = torch.tensor(IMAGENET_MEAN, device=x.device).view(1, 3, 1, 1)
    std = torch.tensor(IMAGENET_STD, device=x.device).view(1, 3, 1, 1)
    return (x - mean) / std


def split_episodes(eps, val_frac, seed):
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(eps))
    n_val = int(round(len(eps) * val_frac)) if len(eps) >= 5 else 0
    val = [eps[i] for i in sorted(order[:n_val])]
    train = [eps[i] for i in sorted(order[n_val:])]
    return train, val


def _worker_init(_):
    import cv2
    cv2.setNumThreads(1)


@torch.no_grad()
def evaluate(model, loader, a_std, dev, max_batches=20):
    model.eval()
    l1s, errs = [], []
    for k, b in enumerate(loader):
        if k >= max_batches:
            break
        img = augment_batch(b["image"].to(dev), False)
        st, act, pad = b["state"].to(dev), b["action"].to(dev), b["is_pad"].to(dev)
        pred, _ = model(img, st)              # 추론과 같음 (VAE 없이 latent 0)
        m = (~pad).unsqueeze(-1).float()
        l1s.append(((pred - act).abs() * m).sum().item() / max(m.sum().item() * act.shape[-1], 1))
        errs.append((((pred - act).abs() * a_std) * m).sum().item() / max(m.sum().item() * act.shape[-1], 1))
    model.train()
    return (float(np.mean(l1s)), float(np.degrees(np.mean(errs)))) if l1s else (None, None)


def export(out_dir, ckpt_path, emit_progress=True):
    """ckpt → OpenVINO IR + meta.json + zip. torch 결과와 OpenVINO(CPU) 결과 차이를 meta 에 기록."""
    import openvino as ov
    if emit_progress:
        emit(phase="export", msg="OpenVINO 변환")
    ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    cfg = ACTConfig(**{**ck["config"], "pretrained_backbone": False})
    act = ACT(cfg)
    act.load_state_dict(ck["model"])
    act.eval()
    wrap = ACTExport(act, ck["stats"]).eval()
    H, W, S = cfg.image_h, cfg.image_w, cfg.state_dim
    rng = np.random.default_rng(0)
    ex_img = torch.from_numpy(rng.random((1, 3, H, W), dtype=np.float32))
    ex_st = torch.tensor(np.asarray(ck["stats"]["state_mean"], np.float32)[None])
    with torch.no_grad():
        ref = wrap(ex_img, ex_st).numpy()
        ovm = ov.convert_model(wrap, example_input=(ex_img, ex_st))
    ovm.inputs[0].get_tensor().set_names({"image"})
    ovm.inputs[1].get_tensor().set_names({"state"})
    ovm.outputs[0].get_tensor().set_names({"action"})
    mdir = os.path.join(out_dir, "model")
    os.makedirs(mdir, exist_ok=True)
    xml = os.path.join(mdir, "policy.xml")
    ov.save_model(ovm, xml, compress_to_fp16=True)
    # 변환 검사는 f32 로 (CPU 기본 정밀도가 bf16 인 CPU 도 있음 — 그 차이는 변환 오류가 아님)
    comp = ov.Core().compile_model(xml, "CPU", {"INFERENCE_PRECISION_HINT": "f32"})
    got = comp({"image": ex_img.numpy(), "state": ex_st.numpy()})[comp.output(0)]
    diff = float(np.abs(got - ref).max())
    log(f"OpenVINO(CPU f32, FP16 가중치) vs torch 최대 차이 {diff:.2e} rad ({np.degrees(diff):.3f}°)")
    meta = dict(ck["meta"])
    meta.update(export_max_abs_diff=diff, export_max_abs_diff_deg=float(np.degrees(diff)),
                openvino=ov.__version__, exported=time.strftime("%Y-%m-%d %H:%M:%S"),
                inputs={"image": f"[1,3,{H},{W}] float32 0~1 RGB", "state": f"[1,{S}] float32 rad"},
                outputs={"action": f"[1,{cfg.chunk_size},{cfg.action_dim}] float32 rad"})
    with open(os.path.join(mdir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    name = meta["name"]
    zpath = os.path.join(out_dir, f"{name}.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for fn in ("policy.xml", "policy.bin", "meta.json"):
            z.write(os.path.join(mdir, fn), f"{name}/{fn}")
    log(f"model zip → {zpath} ({os.path.getsize(zpath) / 1e6:.1f} MB)")
    if emit_progress:
        emit(phase="done", zip=os.path.basename(zpath), export_diff_deg=float(np.degrees(diff)))
    return zpath


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset")
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", help="학습 설정 json 파일 (DEFAULT_TRAIN 키)")
    ap.add_argument("--steps", type=int)
    ap.add_argument("--resume")
    ap.add_argument("--export-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ckpt_path = os.path.join(a.out, "ckpt_last.pt")
    if a.export_only:
        export(a.out, ckpt_path)
        return

    tc = dict(DEFAULT_TRAIN)
    if a.config:
        with open(a.config, encoding="utf-8") as f:
            tc.update({k: v for k, v in json.load(f).items() if k in DEFAULT_TRAIN})
    if a.steps:
        tc["steps"] = a.steps
    signal.signal(signal.SIGUSR1, _on_usr1)
    torch.manual_seed(int(tc["seed"]))
    np.random.seed(int(tc["seed"]))

    emit(phase="load", msg="에피소드 읽는 중")
    eps, info = load_episodes(a.dataset, only_ok=bool(tc["only_ok"]))
    fps = int(eps[0]["meta"].get("fps", 30))
    joint_names = list(eps[0]["meta"]["joint_names"])
    if tc["image"] == "auto":
        W, H = int(eps[0]["meta"]["image"]["w"]), int(eps[0]["meta"]["image"]["h"])
    else:
        W, H = [int(v) for v in str(tc["image"]).lower().split("x")]
    train_eps, val_eps = split_episodes(eps, float(tc["val_frac"]), int(tc["seed"]))
    stats = compute_stats(train_eps, fps)
    rng_stats = compute_stats(eps, fps)            # 안전 범위는 모든 에피소드 기준
    for k in ("action_min", "action_max", "state_min", "state_max", "ee_min", "ee_max", "hand_gap_min",
              "hand_gap_max", "max_vel", "ee_cmd_min", "ee_cmd_max", "hand_gap_cmd_min", "hand_gap_cmd_max"):
        if k in rng_stats:
            stats[k] = rng_stats[k]
    start_pose = np.mean([e["action"][0] for e in eps], axis=0).tolist()
    end_pose = np.mean([e["action"][-1] for e in eps], axis=0).tolist()     # 로봇이 '한 회 끝' 판정에 씀
    n_frames = sum(e["n"] for e in eps)
    log(f"에피소드 {len(eps)} (학습 {len(train_eps)} · 검증 {len(val_eps)}), 프레임 {n_frames}, "
        f"{fps} fps, 이미지 {W}x{H}")

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dev_name = torch.cuda.get_device_name(0) if dev.type == "cuda" else "CPU"
    if dev.type != "cuda":
        log("⚠ CUDA 없음 — CPU 학습 (매우 느림)")
    if dev.type == "cuda":
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    cfg = ACTConfig(state_dim=14, action_dim=14, image_h=H, image_w=W, chunk_size=int(tc["chunk_size"]),
                    dim_model=int(tc["dim_model"]), dim_feedforward=int(tc["dim_feedforward"]),
                    n_heads=int(tc["n_heads"]), n_encoder_layers=int(tc["n_encoder_layers"]),
                    kl_weight=float(tc["kl_weight"]), pretrained_backbone=bool(tc["pretrained_backbone"]))
    model = ACT(cfg).to(dev)
    log(f"ACT {sum(p.numel() for p in model.parameters()) / 1e6:.1f} M 파라미터, 백본 {model.backbone_note}, "
        f"특징맵 {model.feat_hw}, 장치 {dev_name}")
    bb = [p for n, p in model.named_parameters() if n.startswith("backbone.") and p.requires_grad]
    rest = [p for n, p in model.named_parameters() if not n.startswith("backbone.") and p.requires_grad]
    opt = torch.optim.AdamW([{"params": rest, "lr": float(tc["lr"])},
                             {"params": bb, "lr": float(tc["lr_backbone"])}],
                            lr=float(tc["lr"]), weight_decay=float(tc["weight_decay"]))
    step = 0
    if a.resume:
        ck = torch.load(a.resume, map_location="cpu", weights_only=False)
        if ck["config"]["image_h"] != H or ck["config"]["image_w"] != W or ck["config"]["chunk_size"] != cfg.chunk_size:
            raise SystemExit("이어 학습: 이미지 크기·chunk 가 같아야 함")
        model.load_state_dict(ck["model"])
        opt.load_state_dict(ck["opt"])
        stats = ck["stats"]                          # 같은 정규화를 써야 이어짐
        step = int(ck["step"])
        log(f"이어 학습 — {a.resume} step {step} 부터")

    ds = EpisodeDataset(train_eps, cfg.chunk_size, H, W, stats)
    nw = int(tc["num_workers"])
    loader = torch.utils.data.DataLoader(ds, batch_size=int(tc["batch_size"]), shuffle=True, drop_last=True,
                                         num_workers=nw, pin_memory=dev.type == "cuda",
                                         persistent_workers=nw > 0, worker_init_fn=_worker_init)
    vloader = None
    if val_eps:
        vds = EpisodeDataset(val_eps, cfg.chunk_size, H, W, stats)
        vloader = torch.utils.data.DataLoader(vds, batch_size=int(tc["batch_size"]), shuffle=True,
                                              num_workers=min(nw, 2), worker_init_fn=_worker_init)
    a_std = torch.tensor(stats["action_std"], device=dev)
    use_amp = bool(tc["amp"]) and dev.type == "cuda"
    amp_dtype = torch.bfloat16 if (use_amp and torch.cuda.is_bf16_supported()) else torch.float16
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp and amp_dtype == torch.float16)

    name = tc["name"] or f"act_{os.path.basename(os.path.normpath(a.dataset))}_{time.strftime('%m%d_%H%M')}"
    base_meta = {"format": MODEL_FORMAT, "name": name, "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                 "dataset": info.get("name") or os.path.basename(os.path.normpath(a.dataset)),
                 "robot": info.get("robot"), "episodes": len(eps), "val_episodes": len(val_eps),
                 "frames": n_frames, "fps": fps, "joint_names": joint_names, "state_dim": 14, "action_dim": 14,
                 "chunk_size": cfg.chunk_size, "image": {"h": H, "w": W}, "stats": stats, "start_pose": start_pose,
                 "end_pose": end_pose,
                 "config": cfg.to_dict(), "train": tc, "torch": torch.__version__, "device": dev_name}

    def save(final_loss, val):
        tmp = ckpt_path + ".tmp"
        meta = dict(base_meta, steps=step, final_loss=final_loss, val_l1=val[0], val_err_deg=val[1])
        torch.save({"model": model.state_dict(), "opt": opt.state_dict(), "config": cfg.to_dict(),
                    "stats": stats, "step": step, "meta": meta}, tmp)
        os.replace(tmp, ckpt_path)

    total = int(tc["steps"])
    emit(phase="train", step=step, total=total, msg="학습 시작")
    model.train()
    t0, s0 = time.time(), step
    run = {"loss": 0.0, "l1": 0.0, "kl": 0.0, "n": 0}
    last_loss, val = float("nan"), (None, None)
    it = iter(loader)
    while step < total and not _finish_now:
        try:
            b = next(it)
        except StopIteration:
            it = iter(loader)
            b = next(it)
        img = augment_batch(b["image"].to(dev, non_blocking=True), bool(tc["augment"]))
        st = b["state"].to(dev, non_blocking=True)
        act = b["action"].to(dev, non_blocking=True)
        pad = b["is_pad"].to(dev, non_blocking=True)
        with torch.autocast(device_type=dev.type, dtype=amp_dtype, enabled=use_amp):
            loss, l1, kl = model.loss(img, st, act, pad)
        opt.zero_grad(set_to_none=True)
        scaler.scale(loss).backward()
        scaler.unscale_(opt)
        torch.nn.utils.clip_grad_norm_(model.parameters(), float(tc["grad_clip"]))
        scaler.step(opt)
        scaler.update()
        step += 1
        if not math.isfinite(loss.item()):
            raise SystemExit(f"loss 가 nan/inf (step {step}) — lr 을 낮추거나 amp 끄기")
        run["loss"] += loss.item(); run["l1"] += l1.item(); run["kl"] += kl.item(); run["n"] += 1
        if step % int(tc["log_every"]) == 0 or step == total:
            n = max(run["n"], 1)
            last_loss = run["loss"] / n
            it_s = (step - s0) / max(time.time() - t0, 1e-6)
            emit(phase="train", step=step, total=total, loss=round(last_loss, 5), l1=round(run["l1"] / n, 5),
                 kl=round(run["kl"] / n, 5), it_s=round(it_s, 2), eta_s=int((total - step) / max(it_s, 1e-6)))
            run = {"loss": 0.0, "l1": 0.0, "kl": 0.0, "n": 0}
        if vloader is not None and (step % int(tc["eval_every"]) == 0 or step == total):
            val = evaluate(model, vloader, a_std, dev)
            emit(phase="train", step=step, total=total, val_l1=round(val[0], 5), val_err_deg=round(val[1], 3))
        if step % int(tc["ckpt_every"]) == 0:
            save(last_loss, val)
    if vloader is not None and val[0] is None:
        val = evaluate(model, vloader, a_std, dev)
    save(last_loss, val)
    log(f"학습 끝 step {step}, loss {last_loss:.4f}, val L1 {val[0]}, val 오차 {val[1]}°")
    del loader, vloader
    export(a.out, ckpt_path)


if __name__ == "__main__":
    main()
