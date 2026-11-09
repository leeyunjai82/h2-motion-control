#!/usr/bin/env bash
# 학습 PC: ACT 학습 웹 → http://<학습 PC>:50100/
# 사용: ./trainer/start_trainer.sh        (처음이면 먼저 ./trainer/install_trainer.sh <openvino 버전>)
#   python 찾는 순서: TRAINER_PY=<python 경로> → conda env TRAINER_ENV → act (CUDA) → act_xpu (Intel GPU) → python3
#   데이터 위치 바꾸기: TRAINER_DATA=/mnt/ssd/trainer ./trainer/start_trainer.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${TRAINER_PY:-}"
if [ -z "$PY" ]; then
  # env 이름: TRAINER_ENV 를 주면 그것만, 아니면 act (5090 · CUDA) → act_xpu (Intel GPU, install_trainer_xpu.sh) 순서
  for env in ${TRAINER_ENV:-act act_xpu}; do
    for base in "${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
      [ -n "$base" ] && [ -x "$base/envs/$env/bin/python" ] && { PY="$base/envs/$env/bin/python"; break 2; }
    done
  done
fi
[ -z "$PY" ] && PY="$(command -v python3)"
echo "[trainer] python: $PY"
if ! "$PY" -c "import torch, torchvision, openvino, fastapi, uvicorn, cv2" 2>/dev/null; then
  echo "❌ 패키지 없음 — 먼저 설치: ./trainer/install_trainer.sh <로봇 PC 와 같은 openvino 버전>" >&2
  "$PY" -c "import torch, torchvision, openvino, fastapi, uvicorn, cv2"
  exit 1
fi
"$PY" - <<'PY'
import torch
x = getattr(torch, "xpu", None)
if torch.cuda.is_available():
    dev = f"CUDA {torch.cuda.get_device_name(0)} {torch.cuda.get_arch_list()}"
elif x is not None and x.is_available():
    dev = f"Intel XPU {x.get_device_name(0)}"
else:
    dev = "GPU 없음 (CUDA · Intel XPU) — CPU 학습 (매우 느림)"
print(f"[trainer] torch {torch.__version__} | 학습 장치: {dev}")
PY
LOGD="${TRAINER_DATA:-$HERE/data}/logs"
mkdir -p "$LOGD"
"$PY" -u "$HERE/trainer_server.py" 2>&1 | tee -a "$LOGD/trainer_$(date +%Y%m%d).log"
