#!/usr/bin/env python3
"""
xpu_check.py — 학습과 같은 ACT 모델을 가짜 입력으로 한 단계씩 돌려, 어느 단계에서 멈추는지 / 얼마나 걸리는지 찍음
  (데이터 로더 · 학습 서버 없이. 단계마다 화면 + logs/xpu_check_<시각>.log 에 바로 저장 — 멈춰도 마지막 단계가 남음)

  conda env act_xpu 의 python 으로:
    ~/miniconda3/envs/act_xpu/bin/python trainer/xpu_check.py                  # Intel GPU, 배치 8, 카메라 3 대 (학습과 같은 크기)
    ~/miniconda3/envs/act_xpu/bin/python trainer/xpu_check.py --device cpu     # 비교용
    옵션: --batch 8 --iters 10 --no-pretrained (ImageNet 가중치 안 받음)

  단계: ① 모델 만들기 (CPU) ② 장치로 올리기 ③ 앞으로 계산 (no_grad) ④ 앞으로 + 역전파 ⑤ 가중치 갱신 ⑥ 반복 N 번 (step 당 시간)
  2026-11-10: 이 PC (Panther Lake B390) 에서 학습이 첫 50 step 전에 멈춤 (GPU 는 쉬는 상태, python 1 개 CPU 100 %) → 어느 단계인지 찾기
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

LOG = os.path.join(os.path.dirname(HERE), "logs", f"xpu_check_{time.strftime('%Y%m%d_%H%M%S')}.log")
os.makedirs(os.path.dirname(LOG), exist_ok=True)
_T0 = time.time()


def say(msg):
    line = f"[{time.strftime('%H:%M:%S')} +{time.time() - _T0:6.1f}s] {msg}"
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()
        os.fsync(f.fileno())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="xpu", choices=("xpu", "cpu", "cuda"))
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--no-pretrained", action="store_true")
    a = ap.parse_args()
    say(f"로그 → {LOG}")

    say("import torch ...")
    import torch
    from act_model import ACT, ACTConfig
    from train_act import DEFAULT_TRAIN
    say(f"torch {torch.__version__}")
    if a.device == "xpu":
        x = getattr(torch, "xpu", None)
        if x is None or not x.is_available():
            say("❌ torch.xpu 못 씀"); sys.exit(1)
        say(f"XPU 장치: {x.get_device_name(0)}")
    dev = torch.device(a.device)

    def sync():
        if dev.type == "xpu":
            torch.xpu.synchronize()
        elif dev.type == "cuda":
            torch.cuda.synchronize()

    tc = DEFAULT_TRAIN
    cams = [{"name": "body", "h": 240, "w": 424}, {"name": "wrist_left", "h": 240, "w": 320}, {"name": "wrist_right", "h": 240, "w": 320}]
    SD, B = 16, a.batch
    say(f"① 모델 만들기 (CPU) — 카메라 {[(c['name'], c['w'], c['h']) for c in cams]}, state {SD}, chunk {tc['chunk_size']}, 배치 {B}")
    cfg = ACTConfig(state_dim=SD, action_dim=SD, image_h=240, image_w=320, chunk_size=int(tc["chunk_size"]), cameras=cams,
                    dim_model=int(tc["dim_model"]), dim_feedforward=int(tc["dim_feedforward"]), n_heads=int(tc["n_heads"]),
                    n_encoder_layers=int(tc["n_encoder_layers"]), kl_weight=float(tc["kl_weight"]),
                    pretrained_backbone=not a.no_pretrained)
    model = ACT(cfg)
    say(f"   완료 — {sum(p.numel() for p in model.parameters()) / 1e6:.1f} M 파라미터, 백본 {model.backbone_note}")

    say(f"② 장치로 올리기 ({dev}) ...")
    model = model.to(dev)
    sync()
    say("   완료")

    g = torch.Generator().manual_seed(0)
    imgs = [torch.randn(B, 3, c["h"], c["w"], generator=g).to(dev) for c in cams]
    st = torch.randn(B, SD, generator=g).to(dev)
    act = torch.randn(B, cfg.chunk_size, SD, generator=g).to(dev)
    pad = torch.zeros(B, cfg.chunk_size, dtype=torch.bool).to(dev)
    sync()
    say("   입력 올림")

    say("③ 앞으로 계산 (no_grad, 첫 실행 — 커널 준비가 여기서 오래 걸릴 수 있음) ...")
    model.eval()
    t = time.time()
    with torch.no_grad():
        out, _ = model(imgs, st)
    sync()
    say(f"   완료 {time.time() - t:.2f}s — 출력 {tuple(out.shape)}")

    say("④ 앞으로 + 역전파 (첫 실행) ...")
    model.train()
    opt = torch.optim.AdamW(model.parameters(), lr=1e-5, weight_decay=1e-4)
    t = time.time()
    loss, l1, kl = model.loss(imgs, st, act, pad)
    sync()
    say(f"   앞으로 {time.time() - t:.2f}s — loss {loss.item():.4f}")
    t = time.time()
    loss.backward()
    sync()
    say(f"   역전파 {time.time() - t:.2f}s")

    say("⑤ 가중치 갱신 (AdamW 첫 실행) ...")
    t = time.time()
    torch.nn.utils.clip_grad_norm_(model.parameters(), float(tc["grad_clip"]))
    opt.step()
    opt.zero_grad(set_to_none=True)
    sync()
    say(f"   완료 {time.time() - t:.2f}s")

    say(f"⑥ 반복 {a.iters} 번 ...")
    ts = []
    for i in range(a.iters):
        t = time.time()
        loss, l1, kl = model.loss(imgs, st, act, pad)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), float(tc["grad_clip"]))
        opt.step()
        opt.zero_grad(set_to_none=True)
        sync()
        ts.append(time.time() - t)
        say(f"   {i + 1}/{a.iters}: {ts[-1]:.2f}s  loss {loss.item():.4f}")
    say(f"✓ 끝 — step 당 평균 {sum(ts) / len(ts):.2f}s (처음 1 번 빼면 {sum(ts[1:]) / max(1, len(ts) - 1):.2f}s)")


if __name__ == "__main__":
    main()
