#!/usr/bin/env bash
# env_snapshot.sh — tv 가상환경 패키지 + 이 PC 의 드라이버·장치 정보를 파일로 남김 (root 불필요, 읽기만 함)
#   잘 돌아갈 때 / 가상환경을 지우기 전에 실행하고 커밋해 둔다 → 복원 절차는 REBUILD_ENV.md
#
#   ./tools/env_snapshot.sh                 # → env_snapshot/<호스트명>/
#   ./tools/env_snapshot.sh <저장 폴더>      # 예: 다시 만든 뒤 비교용 /tmp/after
#
# 만드는 파일
#   conda_explicit.txt   conda 패키지 (URL·md5 고정)          → conda create -n tv --override-channels -c conda-forge --file conda_explicit.txt
#   pip_torch.txt        PyTorch CPU 판 (+cpu)                ┐ 한 번에: pip install -r pip_torch.txt -r pip_pkgs.txt \
#   pip_pkgs.txt         나머지 pip 패키지 name==version       ┘            --extra-index-url https://download.pytorch.org/whl/cpu
#                        (torch 만 먼저 깔면 torchvision 이 numpy 2.x 를 끌어옴 → 같이 깔아야 numpy 1.24.4 유지)
#   pip_vcs.txt          git 에서 받은 패키지 (unitree_sdk2py)  → pip install -r pip_vcs.txt --src "$CONDA_PREFIX/src"
#   pip_freeze_all.txt   pip freeze 원문 (참고)
#   activate.d/          tv 활성화 훅 중 직접 만든 것 (INSTALL 11단계 threads.sh 등, conda 패키지 것은 제외)
#   system.txt           OS·커널·CPU, GPU/NPU 커널 드라이버·펌웨어, apt 패키지 버전(GStreamer·Intel GPU/NPU 런타임),
#                        OpenVINO 장치, RealSense, 저장소 커밋, third_party SDK, 캐시
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/robot_env.sh" || exit 1
ENV_DIR="$CONDA_BASE/envs/tv"
CONDA="$CONDA_BASE/bin/conda"
[ -x "$TV_PY" ] || { echo "❌ tv 환경 python 없음: $TV_PY (이미 지웠으면 스냅샷을 만들 수 없음)" >&2; exit 1; }
OUT="${1:-$ROOT/env_snapshot/$(hostname)}"
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"
echo "[snapshot] tv = $ENV_DIR → $OUT"

# ---------- 1. conda 패키지 ----------
if "$CONDA" list -p "$ENV_DIR" --explicit --md5 > "$OUT/conda_explicit.txt"; then
  echo "[snapshot] conda_explicit.txt  $(grep -c '^http' "$OUT/conda_explicit.txt") 개"
else
  echo "❌ conda list --explicit 실패" >&2
fi

# ---------- 2. pip 패키지 (conda 가 깐 것은 빼고) + 직접 만든 activate.d 훅 ----------
"$TV_PY" - "$OUT" "$CONDA" "$ENV_DIR" <<'PY'
import glob, json, os, re, shutil, subprocess, sys

out, conda, prefix = sys.argv[1:4]
norm = lambda s: re.sub(r"[-_.]+", "-", s).lower()

# conda list 의 channel 'pypi' = pip 로 깐 패키지
try:
    cl = json.loads(subprocess.check_output([conda, "list", "-p", prefix, "--json"]))
    pypi = {norm(e["name"]) for e in cl if e.get("channel") == "pypi"}
except Exception as e:                       # noqa: BLE001
    print(f"[snapshot] ⚠️ conda list --json 실패 ({e}) → pip freeze 의 name==version 을 전부 pip 패키지로 봄")
    pypi = None

freeze = subprocess.check_output([sys.executable, "-m", "pip", "freeze"], text=True)
with open(os.path.join(out, "pip_freeze_all.txt"), "w") as f:
    f.write(freeze)

TORCH = {"torch", "torchvision", "torchaudio"}
torch, pkgs, vcs, by_conda, warn = [], [], [], [], []
for s in (ln.strip() for ln in freeze.splitlines()):
    if not s or s.startswith("#"):
        continue
    if s.startswith("-e ") or " @ git+" in s or " @ http" in s:
        vcs.append(s)
        if s.startswith("-e ") and not s.startswith("-e git+"):
            warn.append(f"git 원격 없는 editable 설치 → 복원 불가, 직접 다시 설치: {s}")
        continue
    if " @ " in s:                           # name @ file:///… — conda 패키지 또는 로컬 휠
        name = norm(s.split(" @ ")[0])
        if pypi is not None and name in pypi:
            warn.append(f"로컬 파일에서 pip 로 설치됨 → 복원 시 직접 설치: {s}")
        else:
            by_conda.append(s)
        continue
    name = norm(re.split(r"[=<>!~ ;]", s, maxsplit=1)[0])
    if pypi is not None and name not in pypi:
        by_conda.append(s)
        continue
    (torch if name in TORCH else pkgs).append(s)

for t in torch:
    if "+cpu" not in t:
        warn.append(f"PyTorch 가 CPU 판(+cpu)이 아님 — CUDA 판이면 nvidia-* 수 GB 가 같이 깔림: {t}")


def write(fn, head, lines):
    with open(os.path.join(out, fn), "w") as f:
        f.write("".join(f"# {h}\n" for h in head) + "".join(l + "\n" for l in lines))


BOTH = "pip install -r pip_torch.txt -r pip_pkgs.txt --extra-index-url https://download.pytorch.org/whl/cpu"
write("pip_torch.txt", ["PyTorch CPU 판 — pip_pkgs.txt 와 한 번에 (따로 깔면 numpy 2.x 가 들어옴)", BOTH], torch)
write("pip_pkgs.txt", ["나머지 pip 패키지 (conda 가 깐 것 제외) — pip_torch.txt 와 한 번에", BOTH], pkgs)
write("pip_vcs.txt", ["git 에서 받은 패키지 — 위 두 파일 다음에",
                      'pip install -r pip_vcs.txt --src "$CONDA_PREFIX/src"'], vcs)

# activate.d — conda 패키지가 깐 파일(conda-meta/*.json files)은 빼고 직접 만든 것만
owned = set()
for meta in glob.glob(os.path.join(prefix, "conda-meta", "*.json")):
    try:
        owned.update(json.load(open(meta)).get("files", []))
    except (OSError, ValueError):
        pass
hooks = []
for d in ("activate.d", "deactivate.d"):
    for p in sorted(glob.glob(os.path.join(prefix, "etc", "conda", d, "*"))):
        rel = os.path.relpath(p, prefix)
        if rel in owned or not os.path.isfile(p):
            continue
        os.makedirs(os.path.join(out, d), exist_ok=True)
        shutil.copy2(p, os.path.join(out, d, os.path.basename(p)))
        hooks.append(rel)

print(f"[snapshot] pip: torch {len(torch)} · 일반 {len(pkgs)} · git {len(vcs)}  (conda 가 깐 것 {len(by_conda)} 개는 conda_explicit.txt)")
print(f"[snapshot] 활성화 훅: {', '.join(hooks) or '없음 (INSTALL 11단계 threads.sh 가 없으면 IK 가 느려짐)'}")
for w in warn:
    print(f"[snapshot] ⚠️ {w}")
PY

# ---------- 3. 시스템 정보 ----------
sec() { printf '\n===== %s =====\n' "$1"; }
{
  echo "# env_snapshot  $(date '+%Y-%m-%d %H:%M:%S %z')  host=$(hostname)"

  sec "OS / 커널 / CPU / 메모리"
  ( . /etc/os-release 2>/dev/null && echo "os      : ${PRETTY_NAME:-?}" )
  echo "kernel  : $(uname -r)"
  echo "cpu     : $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ *//')  ($(nproc) 스레드)"
  echo "mem     : $(awk '/MemTotal/ {printf "%.1f GB", $2/1048576}' /proc/meminfo)"

  sec "conda / python"
  echo "CONDA_BASE : $CONDA_BASE"
  echo "TV_PY      : $TV_PY   (sudoers NOPASSWD 경로와 같아야 함)"
  "$CONDA" --version 2>&1
  "$TV_PY" -V 2>&1
  "$TV_PY" -m pip --version 2>&1

  sec "저장소"
  git -C "$ROOT" log -1 --format='commit  : %H  %cd  %s' --date=short 2>&1
  echo "branch  : $(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>&1)"
  echo "third_party: $(ls "$ROOT/third_party" 2>/dev/null | tr '\n' ' ')"

  sec "GPU / NPU 커널 드라이버 (사용률을 읽는 파일 — pitron/core/hw_usage.py)"
  for c in /sys/class/drm/card[0-9]* /sys/class/accel/accel[0-9]*; do
    case "$c" in *-*) continue ;; esac
    [ -e "$c/device" ] || continue
    echo "$(basename "$c"): driver=$(basename "$(readlink -f "$c/device/driver")") pci=$(basename "$(readlink -f "$c/device")") id=$(cat "$c/device/vendor" 2>/dev/null):$(cat "$c/device/device" 2>/dev/null)"
  done
  for m in xe i915 intel_vpu; do
    if [ -d "/sys/module/$m" ]; then
      echo "module $m: 로드됨 $(cat /sys/module/$m/version 2>/dev/null) $(modinfo -F filename "$m" 2>/dev/null)"
    fi
  done
  ls -l /dev/accel /dev/dri 2>&1 | sed 's/^/  /'
  echo "NPU 펌웨어: $(ls /lib/firmware/intel/vpu/ 2>/dev/null | tr '\n' ' ')"
  (cd "$ROOT" && timeout 20 "$TV_PY" pitron/core/hw_usage.py --sec 0 2>&1)

  sec "apt 패키지 (GStreamer · Intel GPU/NPU 런타임 · 커널 · 펌웨어)"
  dpkg-query -W -f='${db:Status-Abbrev}\t${Package}\t${Version}\n' \
      'gstreamer1.0-*' 'libgstreamer1.0-*' 'libgstreamer-plugins-*' \
      'intel-*' 'libze*' 'level-zero*' 'libigc*' 'libigdgmm*' 'ocl-icd*' 'clinfo' \
      'linux-firmware' 'linux-image-*' 'libgl1' 'libglib2.0-0*' 'mawk' 'curl' 2>/dev/null \
    | awk -F'\t' '$1 ~ /^ii/ {printf "%-40s %s\n", $2, $3}'
  echo "gst-launch-1.0: $(gst-launch-1.0 --version 2>/dev/null | head -1 || true)"
  [ -e /etc/udev/rules.d/99-realsense-libusb.rules ] && echo "udev: 99-realsense-libusb.rules 있음" || echo "udev: 99-realsense-libusb.rules 없음"

  sec "OpenVINO 장치"
  timeout 90 "$TV_PY" - <<'PY' 2>&1
import re
import openvino as ov
core = ov.Core()
print("openvino", ov.__version__)
print("devices ", core.available_devices)
for d in core.available_devices:
    try:
        props = core.get_property(d, "SUPPORTED_PROPERTIES")
    except Exception as e:                  # noqa: BLE001
        print(f"  {d}: 속성 못 읽음 ({e})")
        continue
    for p in props:
        if not re.search(r"NAME|VERSION|ARCHITECTURE|DEVICE_ID|DEVICE_TYPE|UARCH|GOPS|MEMORY", str(p)):
            continue
        try:
            print(f"  {d:4s} {p:32s} {core.get_property(d, p)}")
        except Exception:                   # noqa: BLE001
            pass
PY

  sec "RealSense"
  timeout 20 "$TV_PY" - <<'PY' 2>&1
import pyrealsense2 as rs
devs = rs.context().devices
print(f"pyrealsense2 {getattr(rs, '__version__', '-')}  장치 {len(devs)} 개")
for d in devs:
    g = lambda k: d.get_info(k) if d.supports(k) else "-"
    print(f"  {g(rs.camera_info.name)}  S/N {g(rs.camera_info.serial_number)}  FW {g(rs.camera_info.firmware_version)}  USB {g(rs.camera_info.usb_type_descriptor)}")
PY

  sec "핵심 패키지 import"
  (cd "$ROOT" && timeout 60 "$TV_PY" -c "
from pitron.core import robot_env
import numpy, torch, cv2, pinocchio, casadi, openvino, ultralytics, unitree_sdk2py, os
from pinocchio import casadi as _c
print('numpy', numpy.__version__, '| torch', torch.__version__, '| cv2', cv2.__version__, '| pinocchio', pinocchio.__version__,
      '| casadi', casadi.__version__, '| openvino', openvino.__version__, '| ultralytics', ultralytics.__version__)
print('unitree_sdk2py', os.path.dirname(unitree_sdk2py.__file__))
print('OMP_NUM_THREADS', os.environ.get('OMP_NUM_THREADS'))
" 2>&1)

  sec "캐시"
  du -sh "$HOME/.cache/h2-motion-control/openvino" 2>/dev/null || echo "OpenVINO 컴파일 캐시 없음"
  ls -l "$ROOT"/robots/h2/*.pkl 2>/dev/null || echo "IK 모델 캐시(*.pkl) 없음"
} > "$OUT/system.txt" 2>&1
echo "[snapshot] system.txt"

echo
echo "완료 → $OUT"
case "$OUT" in
  "$ROOT"/*) echo "커밋:  git add ${OUT#$ROOT/} && git commit -m \"가상환경 스냅샷 $(hostname) $(date +%F)\"" ;;
esac
