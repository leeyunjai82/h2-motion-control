#!/usr/bin/env bash
# 학습 PC: ACT 학습 웹 → http://<학습 PC>:50100/   (처음 띄우면 접근 키를 출력 — 웹에서 한 번 입력)
# 사용: ./trainer/start_trainer.sh
#   python 찾는 순서: TRAINER_PY=<python 경로> → conda env TRAINER_ENV(기본 act) → python3
#   데이터 위치 바꾸기: TRAINER_DATA=/mnt/ssd/trainer ./trainer/start_trainer.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${TRAINER_PY:-}"
if [ -z "$PY" ]; then
  for base in "${CONDA_PREFIX_1:-}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
    [ -n "$base" ] && [ -x "$base/envs/${TRAINER_ENV:-act}/bin/python" ] && { PY="$base/envs/${TRAINER_ENV:-act}/bin/python"; break; }
  done
fi
[ -z "$PY" ] && PY="$(command -v python3)"
echo "[trainer] python: $PY"
if ! "$PY" -c "import torch, torchvision, openvino, fastapi, uvicorn, cv2" 2>/dev/null; then
  echo "❌ 패키지 없음 — trainer/README.md 설치 순서대로 (torch cu128 → requirements.txt → openvino)" >&2
  "$PY" -c "import torch, torchvision, openvino, fastapi, uvicorn, cv2"
  exit 1
fi
"$PY" -c "import torch; ok=torch.cuda.is_available(); print('[trainer] torch', torch.__version__, '| CUDA', ok, '|', torch.cuda.get_device_name(0) if ok else 'GPU 없음 — CPU 학습(매우 느림)', '|', torch.cuda.get_arch_list() if ok else '')"
LOGD="${TRAINER_DATA:-$HERE/data}/logs"
mkdir -p "$LOGD"
"$PY" -u "$HERE/trainer_server.py" 2>&1 | tee -a "$LOGD/trainer_$(date +%Y%m%d).log"
