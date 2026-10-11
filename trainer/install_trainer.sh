#!/usr/bin/env bash
# 학습 PC (Ubuntu, RTX 5090) — trainer 실행 환경 설치. 처음 한 번만.
#
# 사용: ./trainer/install_trainer.sh <openvino 버전>
#   openvino 버전은 로봇 PC 와 같게 (학습 PC 가 더 새 버전이면 로봇이 모델을 못 읽을 수 있음)
#     로봇 PC 에서 확인: source activate_tv.sh && python -c "import openvino; print(openvino.__version__)"
#     예) ./trainer/install_trainer.sh 2026.4.1
#   환경 이름 바꾸기: TRAINER_ENV=act2 ./trainer/install_trainer.sh 2026.4.1      (기본 act — start_trainer.sh 와 같은 이름)
#   torch 는 시험한 조합으로 고정: torch 2.11.0 + torchvision 0.26.0 (cu128 저장소의 마지막 판, 2026-10 확인)
#     바꾸려면: TORCH_PKGS="torch==2.14.1 torchvision==0.29.1" TORCH_INDEX=https://download.pytorch.org/whl/cu130 ...
#     (cu130 은 nvidia-smi CUDA Version 13.0 이상 드라이버 필요)
#
# 하는 일: ① 드라이버 CUDA 버전 확인(12.8 이상) ② conda 찾기 ③ conda env 만들기(python 3.11, conda-forge)
#          ④ torch·torchvision(cu128, 버전 고정) ⑤ requirements.txt + openvino==<버전> ⑥ GPU·sm_120·openvino 확인
#          ⑦ ResNet18 ImageNet 가중치 미리 받기 (학습 때 인터넷 없어도 되게)
# 이미 있는 env 는 지우지 않고 그 안에 설치/갱신한다.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
OV="${1:-}"
ENV_NAME="${TRAINER_ENV:-act}"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"
TORCH_PKGS="${TORCH_PKGS:-torch==2.11.0 torchvision==0.26.0}"   # 컨테이너에서 같은 판(CPU)으로 학습·OpenVINO 변환 확인
PYVER="3.11"

die() { echo "❌ $*" >&2; exit 1; }
step() { echo; echo "=== $* ==="; }

if [ -z "$OV" ]; then
  sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
  die "openvino 버전을 주세요 (로봇 PC 와 같은 버전)"
fi
echo "$OV" | grep -Eq '^[0-9]{4}\.[0-9]+(\.[0-9]+)?$' || die "openvino 버전 형식이 이상함: $OV (예: 2026.4.1)"

# ---------------------------------------------------------------- ① 드라이버
step "① NVIDIA 드라이버"
command -v nvidia-smi >/dev/null || die "nvidia-smi 없음 — NVIDIA 드라이버부터 설치 (Ubuntu: 소프트웨어 & 업데이트 → 추가 드라이버)"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader || die "nvidia-smi 실행 실패 — 드라이버 확인"
CUDA_VER="$(nvidia-smi | sed -n 's/.*CUDA Version: *\([0-9][0-9]*\.[0-9][0-9]*\).*/\1/p' | head -1)"
[ -n "$CUDA_VER" ] || die "nvidia-smi 출력에서 CUDA Version 을 못 읽음"
echo "드라이버가 지원하는 CUDA: $CUDA_VER"
awk -v v="$CUDA_VER" 'BEGIN{split(v,a,"."); exit !((a[1]>12)||(a[1]==12&&a[2]>=8))}' \
  || die "CUDA $CUDA_VER < 12.8 — RTX 50 시리즈(sm_120)는 CUDA 12.8 이상 빌드 torch 필요. 드라이버를 올리세요"

# ---------------------------------------------------------------- ② conda
step "② conda"
CONDA_BASE=""
for base in "${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
  [ -n "$base" ] && [ -x "$base/bin/conda" ] && { CONDA_BASE="$base"; break; }
done
if [ -z "$CONDA_BASE" ]; then
  cat >&2 <<'EOF'
❌ conda 없음 — Miniforge 설치 후 다시 실행:
   wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
   bash Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
EOF
  exit 1
fi
CONDA="$CONDA_BASE/bin/conda"
echo "conda: $CONDA_BASE"

# ---------------------------------------------------------------- ③ env
step "③ conda env '$ENV_NAME' (python $PYVER)"
PY="$CONDA_BASE/envs/$ENV_NAME/bin/python"
if [ -x "$PY" ]; then
  echo "이미 있음 — 그 안에 설치/갱신: $PY"
else
  # conda-forge 만 사용 (defaults 채널 약관 동의 프롬프트 피함)
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge --override-channels "python=$PYVER" pip
fi
[ -x "$PY" ] || die "env python 없음: $PY"
"$PY" -m pip install --upgrade pip

# ---------------------------------------------------------------- ④ torch
step "④ $TORCH_PKGS ($TORCH_INDEX)"
# shellcheck disable=SC2086 — TORCH_PKGS 는 공백으로 나뉜 패키지 목록
"$PY" -m pip install $TORCH_PKGS --index-url "$TORCH_INDEX"

# ---------------------------------------------------------------- ⑤ 나머지
step "⑤ requirements.txt + openvino==$OV"
"$PY" -m pip install -r "$HERE/requirements.txt" "openvino==$OV"

# ---------------------------------------------------------------- ⑥ 확인
step "⑥ 확인"
"$PY" - "$OV" <<'EOF'
import sys
import torch, torchvision, openvino, cv2, fastapi, uvicorn
want_ov = sys.argv[1]
print(f"torch {torch.__version__} (CUDA 빌드 {torch.version.cuda}) | torchvision {torchvision.__version__} | "
      f"openvino {openvino.__version__} | opencv {cv2.__version__}")
if not openvino.__version__.startswith(want_ov):
    sys.exit(f"❌ openvino {openvino.__version__} ≠ 요청 {want_ov}")
if not torch.cuda.is_available():
    sys.exit("❌ torch.cuda.is_available() = False — 드라이버 / torch CUDA 빌드 확인")
name = torch.cuda.get_device_name(0)
cap = torch.cuda.get_device_capability(0)
arch = f"sm_{cap[0]}{cap[1]}"
archs = torch.cuda.get_arch_list()
print(f"GPU {name} ({arch}) | torch 지원 {archs}")
if arch not in archs:
    sys.exit(f"❌ 이 torch 는 {arch} 를 지원 안 함 — CUDA 12.8 이상 빌드 torch 필요 (TORCH_INDEX)")
x = torch.randn(2048, 2048, device="cuda")
y = (x @ x).sum().item()           # 실제 GPU 연산 한 번
print("GPU 연산 OK")
EOF

# ---------------------------------------------------------------- ⑦ 백본 가중치
step "⑦ ResNet18 ImageNet 가중치 미리 받기"
"$PY" -c "import torchvision; torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1); print('OK (~/.cache/torch)')" \
  || echo "⚠ 못 받음 — 학습 때 다시 시도하고, 그래도 안 되면 무작위 초기화로 학습 (로그에 경고)"

echo
echo "✓ 설치 끝 — 실행: ./trainer/start_trainer.sh   (env: $ENV_NAME)"
