#!/usr/bin/env bash
# 학습 환경 — Intel GPU (내장 그래픽 포함, PyTorch XPU) 용. 로봇 PC 에서 5090 없이 학습해 볼 때.
#   ./trainer/install_trainer_xpu.sh <openvino 버전>      (로봇 PC: source activate_tv.sh && python -c "import openvino; print(openvino.__version__)")
#
# 로봇이 쓰는 tv 환경은 건드리지 않음 — conda env 'act_xpu' 를 따로 만듦 (이름 바꾸기: TRAINER_ENV=이름).
# 하는 일: ① Intel GPU 장치 · Level Zero 드라이버 확인 ② conda ③ env ④ torch (XPU 빌드) ⑤ requirements + openvino
#          ⑥ torch.xpu 로 GPU 인식 · 연산 확인 ⑦ ResNet18 가중치 미리 받기
# ⚠ 이 PC 의 GPU (lspci: Intel Device b082) 가 PyTorch XPU 에서 잡히는지는 ⑥ 에서 처음 확인됨 (2026-11-09 아직 확인 전)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OV="${1:-}"
[ -n "$OV" ] || { echo "사용: $0 <openvino 버전 — 로봇 tv 환경과 같게>" >&2; exit 1; }
ENV_NAME="${TRAINER_ENV:-act_xpu}"
PYVER="3.11"
TORCH_PKGS="${TORCH_PKGS:-torch torchvision}"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/xpu}"
step() { echo; echo "==== $* ===="; }
die() { echo "❌ $*" >&2; exit 1; }

# ---------------------------------------------------------------- ① Intel GPU
step "① Intel GPU 장치 · 드라이버"
lspci | grep -i -E "vga|display" || true
ls /dev/dri/renderD* >/dev/null 2>&1 || die "/dev/dri/renderD* 없음 — Intel 그래픽 커널 드라이버가 안 올라옴"
id -nG | grep -q -w render || echo "⚠ 이 계정이 render 그룹이 아님 — 안 되면: sudo usermod -aG render \$USER 후 다시 로그인"
if ls /usr/lib/x86_64-linux-gnu/libze_intel_gpu.so* >/dev/null 2>&1 || ldconfig -p | grep -q libze_intel_gpu; then
  echo "Level Zero GPU 드라이버 있음"
else
  cat >&2 <<'EOF'
⚠ Level Zero GPU 드라이버 (libze_intel_gpu) 가 안 보임 — torch.xpu 가 GPU 를 못 잡을 수 있음.
   Ubuntu 24.04 이상 예 (Intel 클라이언트 GPU 안내 기준, 확인 필요):
     sudo add-apt-repository -y ppa:kobuk-team/intel-graphics
     sudo apt install -y libze-intel-gpu1 libze1 intel-opencl-icd clinfo
   설치 후 이 스크립트를 다시 실행. (계속 진행은 함 — ⑥ 에서 결과 확인)
EOF
fi

# ---------------------------------------------------------------- ② conda
step "② conda"
CONDA_BASE=""
for base in "${CONDA_EXE:+$(dirname "$(dirname "$CONDA_EXE")")}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
  [ -n "$base" ] && [ -x "$base/bin/conda" ] && { CONDA_BASE="$base"; break; }
done
[ -n "$CONDA_BASE" ] || die "conda 없음"
CONDA="$CONDA_BASE/bin/conda"
echo "conda: $CONDA_BASE"

# ---------------------------------------------------------------- ③ env
step "③ conda env '$ENV_NAME' (python $PYVER) — tv 와 별개"
PY="$CONDA_BASE/envs/$ENV_NAME/bin/python"
if [ -x "$PY" ]; then
  echo "이미 있음 — 그 안에 설치/갱신: $PY"
else
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge --override-channels "python=$PYVER" pip
fi
[ -x "$PY" ] || die "env python 없음: $PY"
"$PY" -m pip install --upgrade pip

# ---------------------------------------------------------------- ④ torch (XPU)
step "④ $TORCH_PKGS ($TORCH_INDEX)"
# shellcheck disable=SC2086
"$PY" -m pip install $TORCH_PKGS --index-url "$TORCH_INDEX"

# ---------------------------------------------------------------- ⑤ 나머지
step "⑤ requirements.txt + openvino==$OV"
"$PY" -m pip install -r "$HERE/requirements.txt" "openvino==$OV"

# ---------------------------------------------------------------- ⑥ GPU 확인
step "⑥ torch.xpu 확인"
"$PY" - <<'PY'
import sys, time, torch
print(f"torch {torch.__version__}")
x = getattr(torch, "xpu", None)
if x is None or not x.is_available():
    sys.exit("❌ torch.xpu.is_available() = False — ① 의 Level Zero 드라이버 · render 그룹 확인 (CPU 로는 학습 가능하나 느림)")
print("장치:", x.get_device_name(0))
a = torch.randn(2048, 2048, device="xpu"); b = torch.randn(2048, 2048, device="xpu")
(a @ b).sum().item(); x.synchronize()
t = time.time()
for _ in range(20):
    c = a @ b
x.synchronize()
dt = (time.time() - t) / 20
print(f"✓ XPU 연산 OK — 2048² 행렬곱 {dt * 1000:.1f} ms ≈ {2 * 2048 ** 3 / dt / 1e12:.2f} TFLOPS (fp32)")
PY

# ---------------------------------------------------------------- ⑦ ResNet18 가중치
step "⑦ ResNet18 가중치"
"$PY" -c "import torchvision; torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.DEFAULT); print('✓ ResNet18 가중치 OK')" \
  || echo "⚠ 가중치를 못 받음 — 첫 학습 때 다시 시도"

echo
echo "✓ 끝. 학습:  $PY $HERE/train_act.py --dataset <데이터셋 폴더> --out <출력 폴더> --device xpu --steps 50   (속도 재기)"
