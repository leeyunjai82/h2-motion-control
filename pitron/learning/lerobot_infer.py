#!/usr/bin/env python3
"""
lerobot_infer.py — lerobot 모델 (pretrained_model 폴더) 추론 프로세스. lerobot 가상환경의 python 으로 돎 (pick_web 이 띄움).

  python -m pitron.learning.lerobot_infer --model data/policies/<이름> [--device cpu|cuda|xpu] [--backend torch|openvino --ov-device GPU]

  정책 종류와 상관없이 lerobot 표준 그대로: config.json type → 정책 클래스 · pre/post processor (정규화) · select_action
  (action chunk 큐 · temporal ensemble 등은 그 정책이 학습한 설정 그대로). 데이터셋은 안 씀.
  openvino: ACT 만 — 정책 안의 model(batch) 만 OpenVINO 로 바꿈 (큐 · 정규화는 lerobot 그대로). 변환 결과는 <모델>/openvino/ 에 캐시

  주고받기 (stdin / stdout, 한 번에 하나)
    시작   → @@{"ok": true, type, cams: [{name, h, w}], state_dim, action_dim, chunk_size, n_action_steps, a_min, a_max, ...}
    요청   {"op": "act", "state": [..]} + 줄바꿈 + 카메라 순서대로 RGB uint8 (h·w·3 바이트씩)  → @@{"action": [..], "ms": ..}
           {"op": "reset"}  → @@{"ok": true}     (한 회 시작 전: 정책 큐 비움)
           {"op": "quit"}
  다른 출력 (lerobot · torch 로그) 은 stderr 로 보냄.
"""
import argparse
import json
import os
import sys
import time

PROTO = None                                     # 주고받기 전용 (진짜 stdout) — main() 에서 엶


def send(obj):
    PROTO.write("@@" + json.dumps(obj, ensure_ascii=False) + "\n")
    PROTO.flush()


def load_stats(model_dir):
    """postprocessor (unnormalizer) safetensors 의 action min/max (학습 데이터 범위)."""
    from safetensors.numpy import load_file
    for fn in sorted(os.listdir(model_dir)):
        if fn.endswith(".safetensors") and "processor" in fn:
            d = load_file(os.path.join(model_dir, fn))
            if "action.min" in d and "action.max" in d:
                return d["action.min"].tolist(), d["action.max"].tolist()
    return None, None


def ov_swap(pol, model_dir, ov_device, example):
    """ACT: pol.model (nn.Module) 을 OpenVINO 로 바꿈. example = 전처리된 batch (dict)."""
    import openvino as ov
    import torch
    from lerobot.utils.constants import OBS_IMAGES
    keys = ["observation.state"] + ([k for k in ["observation.environment_state"] if k in example]) + list(pol.config.image_features)
    inner = pol.model

    class Wrap(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, *xs):
            b = dict(zip(keys, xs))
            b[OBS_IMAGES] = [b[k] for k in pol.config.image_features]
            return self.m(b)[0]

    w = Wrap(inner).eval()
    st = os.stat(os.path.join(model_dir, "model.safetensors"))
    stamp = f"{st.st_size}-{int(st.st_mtime)}"
    xml = os.path.join(model_dir, "openvino", "policy.xml")
    stamp_p = os.path.join(model_dir, "openvino", "stamp.txt")
    if not (os.path.isfile(xml) and os.path.isfile(stamp_p) and open(stamp_p).read().strip() == stamp):
        print(f"[infer] OpenVINO 변환 → {xml}", flush=True)
        os.makedirs(os.path.dirname(xml), exist_ok=True)
        with torch.no_grad():
            m = ov.convert_model(w, example_input=tuple(example[k] for k in keys))
        ov.save_model(m, xml, compress_to_fp16=False)
        with open(stamp_p, "w") as f:
            f.write(stamp)
    core = ov.Core()
    model = core.read_model(xml)
    # 입력 크기 고정 (batch 1 · 모델 입력 크기) — 실제 입력은 늘 이 크기. NPU 는 고정된 모델만 받음 ('negative shape dim bound -1', 2026-11-25)
    model.reshape({inp: ov.PartialShape(list(example[k].shape)) for inp, k in zip(model.inputs, keys)})
    if ov_device != "CPU":
        core.set_property({"CACHE_DIR": os.path.join(model_dir, "openvino", "cache")})   # 컴파일 결과 캐시 (두 번째부터 빨리 뜸)
    comp = core.compile_model(model, ov_device, {"INFERENCE_PRECISION_HINT": "f32"} if ov_device == "CPU" else {})
    req = comp.create_infer_request()
    with torch.no_grad():
        ref = w(*[example[k] for k in keys]).numpy()
    req.infer([example[k].numpy() for k in keys])
    diff = float(abs(req.get_output_tensor(0).data - ref).max())

    class OVModel(torch.nn.Module):
        def forward(self, batch):
            req.infer([batch[k].detach().cpu().numpy() for k in keys])
            return torch.from_numpy(req.get_output_tensor(0).data.copy()), (None, None)

    pol.model = OVModel()
    return diff


def main():
    global PROTO
    PROTO = os.fdopen(os.dup(1), "w", buffering=1)
    os.dup2(2, 1)                                # 라이브러리 print 는 stderr 로
    sys.stdout = sys.stderr
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--device", default="cpu", help="torch 장치 cpu · cuda · xpu")
    ap.add_argument("--backend", default="torch", choices=("torch", "openvino"))
    ap.add_argument("--ov-device", default="CPU", help="OpenVINO 장치 CPU · GPU · NPU · AUTO")
    ap.add_argument("--task", default="", help="언어 조건 정책 (smolvla · pi0 …) 의 작업 설명")
    ap.add_argument("--n-action-steps", type=int, default=0, help="chunk 에서 실행할 동작 수 (0 = 모델 설정 그대로). 작을수록 자주 다시 계산")
    ap.add_argument("--temporal-ensemble", type=float, default=0.0,
                    help="ACT temporal ensemble 계수 (0 = 끔, ACT 논문 0.01) — 매 틱 다시 계산해 chunk 들을 겹쳐 평균 (한 틱 안에 계산이 끝나야 함)")
    a = ap.parse_args()
    t0 = time.time()
    try:
        import numpy as np
        import torch
        import lerobot.policies  # noqa: F401 — 정책 설정 등록
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        md = os.path.abspath(a.model)
        cfg = PreTrainedConfig.from_pretrained(md)
        cfg.device = a.device
        if a.temporal_ensemble > 0:                  # lerobot ACT 공식 옵션 (ACTTemporalEnsembler) — n_action_steps 1 고정
            if cfg.type != "act" or not hasattr(cfg, "temporal_ensemble_coeff"):
                raise ValueError(f"temporal ensemble 은 ACT 만 — 이 모델은 {cfg.type}")
            if a.n_action_steps > 0:
                raise ValueError("temporal ensemble 과 n_action_steps 는 같이 못 씀 (1 고정)")
            cfg.temporal_ensemble_coeff, cfg.n_action_steps = a.temporal_ensemble, 1
        if a.n_action_steps > 0:
            if not hasattr(cfg, "n_action_steps"):
                raise ValueError(f"{cfg.type} 는 n_action_steps 설정이 없음")
            if getattr(cfg, "temporal_ensemble_coeff", None) is not None:
                raise ValueError("temporal ensemble 모델은 n_action_steps 1 고정")
            if a.n_action_steps > getattr(cfg, "chunk_size", a.n_action_steps):
                raise ValueError(f"n_action_steps {a.n_action_steps} > chunk_size {cfg.chunk_size}")
            cfg.n_action_steps = a.n_action_steps
        pol = get_policy_class(cfg.type).from_pretrained(md, config=cfg)
        pol.eval()
        pol.reset()
        pre, post = make_pre_post_processors(pol.config, pretrained_path=md,
                                             preprocessor_overrides={"device_processor": {"device": a.device}})
        feats = pol.config.input_features
        cams = [{"name": k.split(".", 2)[-1], "key": k, "c": int(v.shape[0]), "h": int(v.shape[1]), "w": int(v.shape[2])}
                for k, v in feats.items() if k.startswith("observation.images.")]
        state_dim = int(feats["observation.state"].shape[0]) if "observation.state" in feats else 0
        act_dim = int(pol.config.output_features["action"].shape[0])
        others = [k for k in feats if k != "observation.state" and not k.startswith("observation.images.")]
        if others:
            raise ValueError(f"이 로봇에 없는 입력: {others}")
        task = a.task

        def obs_of(state, imgs):
            o = {"observation.state": torch.as_tensor(np.asarray(state, np.float32))}
            for c, im in zip(cams, imgs):
                o[c["key"]] = torch.from_numpy(im).permute(2, 0, 1).float() / 255.0
            o["task"] = task
            return o

        blank = obs_of(np.zeros(state_dim, np.float32), [np.zeros((c["h"], c["w"], 3), np.uint8) for c in cams])
        ov_diff = None
        if a.backend == "openvino":
            if cfg.type != "act":
                raise ValueError(f"OpenVINO 는 ACT 만 — 이 모델은 {cfg.type} (torch 로)")
            with torch.no_grad():
                ov_diff = ov_swap(pol, md, a.ov_device, pre(dict(blank)))
        ms = []
        with torch.no_grad():
            for _ in range(3):
                pol.reset()
                t = time.time()
                post(pol.select_action(pre(dict(blank))))
                ms.append((time.time() - t) * 1000)
        pol.reset()
        amin, amax = load_stats(md)
        send({"ok": True, "type": cfg.type, "cams": [{k: c[k] for k in ("name", "h", "w")} for c in cams],
              "state_dim": state_dim, "action_dim": act_dim, "chunk_size": getattr(cfg, "chunk_size", None),
              "n_action_steps": getattr(cfg, "n_action_steps", None),
              "temporal_ensemble": getattr(cfg, "temporal_ensemble_coeff", None), "a_min": amin, "a_max": amax,
              "backend": a.backend, "device": a.ov_device if a.backend == "openvino" else a.device,
              "load_s": round(time.time() - t0, 1), "infer_ms": round(min(ms), 1), "ov_diff": ov_diff,
              "torch": torch.__version__})
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        send({"ok": False, "err": f"{type(e).__name__}: {e}"})
        return 1

    inp = sys.stdin.buffer
    sizes = [c["h"] * c["w"] * 3 for c in cams]
    while True:
        line = inp.readline()
        if not line:
            return 0
        try:
            req = json.loads(line)
        except ValueError:
            send({"ok": False, "err": "요청 json 이상"})
            continue
        op = req.get("op")
        if op == "quit":
            return 0
        if op == "reset":
            pol.reset()
            send({"ok": True})
            continue
        if op != "act":
            send({"ok": False, "err": f"op {op}"})
            continue
        imgs = []
        for c, n in zip(cams, sizes):
            b = inp.read(n)
            if len(b) != n:
                return 1
            imgs.append(np.frombuffer(b, np.uint8).reshape(c["h"], c["w"], 3))
        try:
            t = time.time()
            with torch.no_grad():
                act = post(pol.select_action(pre(obs_of(req["state"], imgs))))
            send({"ok": True, "action": act.detach().cpu().numpy().reshape(-1).tolist(), "ms": round((time.time() - t) * 1000, 1)})
        except Exception as e:  # noqa: BLE001
            send({"ok": False, "err": f"{type(e).__name__}: {e}"})


if __name__ == "__main__":
    sys.exit(main())
