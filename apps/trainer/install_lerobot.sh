#!/usr/bin/env bash
# lerobot 가상환경 (수집 v3 변환 · 학습 · 추론 공용) — conda env 'lerobot' (로봇 tv 환경과 별개)
#   ./apps/trainer/install_lerobot.sh            장치 자동 (nvidia-smi → cuda, Intel GPU → xpu, 없으면 cpu)
#   ./apps/trainer/install_lerobot.sh xpu        미니 PC (Intel Arc)
#   ./apps/trainer/install_lerobot.sh cuda       학습 PC (RTX 5090, CUDA 12.8 이상 드라이버)
#   env 이름 · 위치 바꾸기: LEROBOT_ENV=이름 · 다른 곳이면 config/local.yaml lerobot_python
#
# 버전: lerobot 0.4.4 (데이터셋 v3.0) — torch <2.11 을 요구함 → torch 2.10.0 · torchvision 0.25.0
#   ⚠ 미니 PC 의 예전 학습 환경 act_xpu 는 torch 2.11.0+xpu — lerobot 과 같이 못 씀 (그래서 env 를 따로 만듦)
#   ⚠ Arc B390 (Panther Lake) 가 torch 2.10.0+xpu 에서 잡히는지는 확인 필요 — ⑤ 에서 처음 확인됨
#   ⚠ Intel GPU 학습은 런처가 GPU 최대 클럭을 1200 MHz 로 제한한 상태에서만 (제한 없이 돌리면 PC 멈춤, 2026-11-10)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
ENV_NAME="${LEROBOT_ENV:-lerobot}"
PYVER="3.10"
LEROBOT_VER="0.4.4"
TORCH_PKGS="${TORCH_PKGS:-torch==2.10.0 torchvision==0.25.0}"
step() { echo; echo "==== $* ===="; }
die() { echo "❌ $*" >&2; exit 1; }

DEV="${1:-auto}"
if [ "$DEV" = "auto" ]; then
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then DEV=cuda
  elif ls /dev/dri/renderD* >/dev/null 2>&1 && lspci 2>/dev/null | grep -i -E "vga|display" | grep -qi intel; then DEV=xpu
  else DEV=cpu; fi
fi
case "$DEV" in
  cuda) TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}" ;;
  xpu)  TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/xpu}" ;;
  cpu)  TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cpu}" ;;
  *) die "장치: auto · cuda · xpu · cpu" ;;
esac
echo "장치: $DEV  ($TORCH_PKGS from $TORCH_INDEX)"

# ---------------------------------------------------------------- ① conda
step "① conda"
CONDA_BASE=""
for base in "${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
  [ -n "$base" ] && [ -x "$base/bin/conda" ] && { CONDA_BASE="$base"; break; }
done
[ -n "$CONDA_BASE" ] || die "conda 없음 — Miniforge 설치 후 다시"
CONDA="$CONDA_BASE/bin/conda"
echo "conda: $CONDA_BASE"

# ---------------------------------------------------------------- ② env
step "② conda env '$ENV_NAME' (python $PYVER, ffmpeg)"
PY="$CONDA_BASE/envs/$ENV_NAME/bin/python"
if [ -x "$PY" ]; then
  echo "이미 있음 — 그 안에 설치/갱신: $PY"
else
  # ffmpeg: 영상 디코딩 (torchcodec) 이 시스템 · env 의 ffmpeg 라이브러리를 씀
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge --override-channels "python=$PYVER" pip "ffmpeg<8"
fi
[ -x "$PY" ] || die "env python 없음: $PY"
"$PY" -m pip install --upgrade pip

# ---------------------------------------------------------------- ③ torch
step "③ $TORCH_PKGS ($DEV)"
# shellcheck disable=SC2086
"$PY" -m pip install $TORCH_PKGS --index-url "$TORCH_INDEX"

# ---------------------------------------------------------------- ④ lerobot + 웹 + OpenVINO
step "④ lerobot==$LEROBOT_VER · fastapi · uvicorn · openvino"
"$PY" -m pip install "lerobot==$LEROBOT_VER" fastapi "uvicorn>=0.29" pyyaml openvino
"$PY" - <<'PY'
import torch, lerobot
print(f"torch {torch.__version__} · lerobot {lerobot.__version__}")
if not torch.__version__.startswith("2.10"):
    raise SystemExit("❌ lerobot 설치가 torch 를 바꿨음 (" + torch.__version__ + ") — TORCH_PKGS 확인")
PY

# ---------------------------------------------------------------- ⑤ 장치 확인
step "⑤ 장치 확인 ($DEV)"
DEV="$DEV" "$PY" - <<'PY'
import os, sys, time, torch
dev = os.environ["DEV"]
if dev == "cuda":
    if not torch.cuda.is_available():
        sys.exit("❌ torch.cuda.is_available() = False — nvidia-smi · 드라이버 확인")
    print("장치:", torch.cuda.get_device_name(0), torch.cuda.get_arch_list())
elif dev == "xpu":
    x = getattr(torch, "xpu", None)
    if x is None or not x.is_available():
        sys.exit("❌ torch.xpu.is_available() = False — Level Zero 드라이버 (libze-intel-gpu1) · render 그룹 확인. CPU 학습은 됨 (느림)")
    print("장치:", x.get_device_name(0))
if dev != "cpu":
    a = torch.randn(2048, 2048, device=dev); b = torch.randn(2048, 2048, device=dev)
    (a @ b).sum().item()
    sync = torch.cuda.synchronize if dev == "cuda" else torch.xpu.synchronize
    sync(); t = time.time()
    for _ in range(20):
        c = a @ b
    sync(); dt = (time.time() - t) / 20
    print(f"✓ {dev} 연산 OK — 2048² 행렬곱 {dt * 1000:.1f} ms")
from lerobot.datasets.video_utils import get_safe_default_codec
print("영상 디코더:", get_safe_default_codec())
PY

# ---------------------------------------------------------------- ⑥ config
step "⑥ config/local.yaml lerobot_python"
if [ "$PY" != "$HOME/miniconda3/envs/lerobot/bin/python" ]; then
  echo "기본 위치가 아님 → config/local.yaml 에 넣으세요:"
  echo "  lerobot_python: $PY"
else
  echo "기본 위치 ($PY) — 설정 필요 없음"
fi
echo
echo "✓ 끝. 학습 웹: $HERE/start_trainer.sh   (미니 PC 는 홈 화면 → 학습)"
