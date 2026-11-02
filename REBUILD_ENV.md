# 가상환경(tv) 다시 만들기

`tv` conda 환경을 지우고 다시 만드는 절차입니다. 처음 설치는 [INSTALL.md](./INSTALL.md), 사용법은 [README.md](./README.md).

| 상황 | 방법 |
| --- | --- |
| 같은 PC, 잘 되던 버전 그대로 | **1 → 2 → 3-A** (스냅샷에서 복원) |
| 스냅샷이 없음 / 버전을 새로 | **2 → 3-B** (INSTALL 5·7·11단계를 다시) |
| 새 PC | INSTALL 전체 + 아래 [5. 가상환경 밖](#5-가상환경-밖--새-pc-일-때만) |

- 패키지 하나만 꼬였으면 전체를 다시 만들기 전에 그 패키지만 고정값으로 다시 설치해 보세요.
  예: numpy 2.x 가 들어왔다 → `pip install numpy==1.24.4`
- 경로·명령은 `~/project/h2-motion-control`, Miniconda `~/miniconda3`, 환경 이름 `tv` 기준입니다.
- 명령 검증: 2026-10-09 개발 환경 (Ubuntu 24.04, conda 26.7.1). 실기 PC 에서는 확인 필요.

---

## 1. 지우기 전에 — 스냅샷 (잘 될 때 해 두기)

```bash
cd ~/project/h2-motion-control
./utils/env_snapshot.sh                 # → env_snapshot/<호스트명>/  (root 불필요, 읽기만 함)
git add env_snapshot/ && git commit -m "가상환경 스냅샷 $(hostname) $(date +%F)" && git push
```

| 파일 | 내용 |
| --- | --- |
| `conda_explicit.txt` | conda 패키지 (URL·md5 고정) — pinocchio, casadi, scikit-sparse, python 등 |
| `pip_torch.txt` | PyTorch CPU 판 (`+cpu`) |
| `pip_pkgs.txt` | 나머지 pip 패키지 `name==version` (conda 가 깐 것 제외) |
| `pip_vcs.txt` | git 에서 받은 패키지 (`unitree_sdk2py` f559291) |
| `pip_freeze_all.txt` | pip freeze 원문 (참고) |
| `activate.d/` | 직접 만든 활성화 훅 (INSTALL 11단계 `threads.sh`) |
| `system.txt` | OS·커널·CPU, GPU/NPU 드라이버·펌웨어, apt 패키지 버전(GStreamer·Intel GPU/NPU 런타임), OpenVINO 장치, RealSense, 저장소 커밋, 캐시 |

- 실행 끝에 `⚠️` 가 나오면 읽어 볼 것: PyTorch 가 CPU 판이 아님, 활성화 훅 없음(IK 가 느려짐), git 원격 없는 설치 등.
- 지금 잘 돌아가는 상태에서 한 번 해 두면 다음에 바로 3-A 로 복원할 수 있습니다.

---

## 2. 지우기

로봇은 `./start_fsm.sh damp` (또는 전원 끔), 실행 중인 `start_*.sh` / `launcher.sh` 는 `Ctrl+C` 로 먼저 끕니다.

```bash
cd ~/project/h2-motion-control
source $HOME/miniconda3/bin/activate                                  # base (tv 가 켜져 있으면 못 지움)
conda remove -y --all -n tv --override-channels -c conda-forge
ls $HOME/miniconda3/envs/                                             # tv 가 없어야 정상
```

- `conda env remove -n tv` 는 conda 26.7.1 에서 `CondaToSNonInteractiveError` (기본 채널 약관) 로 실패합니다 — 위 명령을 쓰세요.
- `third_party/` (H2 SDK 814556d) 와 `common/models/` 는 가상환경 밖이라 그대로 둡니다.
- 버전이 바뀔 때만 지울 캐시 (안 지워도 동작에는 문제 없음 — 첫 실행이 느려질 뿐):

  | 캐시 | 언제 지움 |
  | --- | --- |
  | `~/.cache/h2-motion-control/openvino` (OpenVINO 컴파일 결과) | OpenVINO 버전이 바뀔 때 — 첫 실행 때 다시 컴파일 |
  | `robots/h2/h2_xr817fb00_model_cache.pkl` (IK 모델, `robot.yaml ik_cache`) | pinocchio 버전이 바뀔 때 — `pickle.load` 오류가 나면 반드시 |

---

## 3-A. 스냅샷에서 그대로 복원

```bash
cd ~/project/h2-motion-control
SNAP=env_snapshot/$(hostname)            # 다른 PC 의 스냅샷을 쓰려면 그 폴더

source $HOME/miniconda3/bin/activate
conda create -y -n tv --override-channels -c conda-forge --file $SNAP/conda_explicit.txt
conda activate tv

# PyTorch 와 나머지를 한 번에 (따로 깔면 torchvision 이 numpy 2.x 를 끌어옴)
pip install -r $SNAP/pip_torch.txt -r $SNAP/pip_pkgs.txt --extra-index-url https://download.pytorch.org/whl/cpu
pip install -r $SNAP/pip_vcs.txt --src "$CONDA_PREFIX/src"

# 활성화 훅 (BLAS 스레드 1개 고정)
for d in activate.d deactivate.d; do
  [ -d $SNAP/$d ] && mkdir -p "$CONDA_PREFIX/etc/conda/$d" && cp $SNAP/$d/* "$CONDA_PREFIX/etc/conda/$d/"
done
conda deactivate && conda deactivate
```

- `--override-channels -c conda-forge` 를 빼면 explicit 파일이어도 `CondaToSNonInteractiveError` 로 실패합니다 (실측).
- `pip_torch.txt` 만 먼저 설치하면 torchvision 이 numpy 2.2.6 을 끌어옵니다 (실측) → 반드시 위처럼 한 줄로.
- 검증 (개발 환경): 스냅샷 → 새 환경 → conda 패키지 목록 동일, `pip freeze` 동일, `pip check` 이상 없음,
  `numpy 1.24.4 | torch 2.4.1+cpu | pinocchio 3.1.0` import 확인.

그다음 [4. 확인](#4-확인).

---

## 3-B. 처음부터 (INSTALL 5 · 7 · 11 단계)

스냅샷이 없거나 버전을 새로 받을 때. INSTALL.md 와 같은 명령입니다.

```bash
cd ~/project/h2-motion-control
source $HOME/miniconda3/bin/activate

# INSTALL 5
conda create -y -n tv --override-channels -c conda-forge \
    python=3.10 pinocchio=3.1.0 numpy=1.24.4 scikit-sparse=0.4.16 git
conda activate tv

# INSTALL 7
pip install --upgrade pip
pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt --src "$CONDA_PREFIX/src"
pip install meshcat==0.3.2 logging-mp openvino

# INSTALL 11 — BLAS 스레드 1개 고정
mkdir -p "$CONDA_PREFIX/etc/conda/activate.d"
cat > "$CONDA_PREFIX/etc/conda/activate.d/threads.sh" << 'EOF'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
EOF
conda deactivate && conda deactivate
```

- `openvino`, `logging-mp` 는 버전을 고정하지 않아 최신이 들어옵니다. 예전 버전으로 맞추려면 예전 스냅샷의
  `pip_pkgs.txt` 에서 해당 줄(`openvino==…`, `openvino-telemetry==…`, `logging-mp==…`)을 보고 `pip install openvino==<버전>`.
- 다 되면 [1. 스냅샷](#1-지우기-전에--스냅샷-잘-될-때-해-두기) 을 새로 떠서 커밋해 두세요.

---

## 4. 확인

```bash
cd ~/project/h2-motion-control
source activate_tv.sh                     # tv + H2 SDK(third_party) — 처음이면 SDK 를 받음

python -c "import numpy, torch, cv2, pinocchio, sksparse, unitree_sdk2py, openvino; from pinocchio import casadi; print('numpy', numpy.__version__, '| torch', torch.__version__)"
#   numpy 1.24.4 | torch 2.4.1+cpu
echo $OMP_NUM_THREADS                     # 1
python utils/get_dev.py | head -3         # Available devices 에 CPU, GPU, NPU
python common/ctrl/hw_usage.py --sec 3    # CPU / GPU / NPU 숫자 (못 읽으면 이유)
sudo -n "$TV_PY" -c "print('sudo ok')"    # 비밀번호를 묻지 않아야 정상 (환경 이름 tv 그대로면 sudoers 수정 불필요)

./start_grab_sim.sh                            # 가상 잡기 — http://<pc-ip>:50000/ 에서 Box → Grab Now 가 끝까지 가면 정상
```

스냅샷과 비교 (다르면 그 줄이 바뀐 패키지):

```bash
./utils/env_snapshot.sh /tmp/after
diff env_snapshot/$(hostname)/pip_pkgs.txt /tmp/after/pip_pkgs.txt && echo "pip 같음"
diff env_snapshot/$(hostname)/conda_explicit.txt /tmp/after/conda_explicit.txt && echo "conda 같음"
```

IK 속도 확인은 INSTALL 13단계 (6) — 수 ms 면 정상, 수십 ms 면 활성화 훅(BLAS 스레드)이 빠진 것.

---

## 5. 가상환경 밖 — 새 PC 일 때만

같은 PC 에서 tv 만 다시 만들 때는 아래를 건드릴 필요가 없습니다.

| 항목 | 새 PC 에서 |
| --- | --- |
| Miniconda (`~/miniconda3`, `conda init` 안 함) | INSTALL 4 |
| RealSense udev 규칙 | INSTALL 3 |
| sudoers `/etc/sudoers.d/h2-motion` (`…/envs/tv/bin/python`) | INSTALL 8 |
| 네트워크 `192.168.123.0/24` | INSTALL 9 |
| USB 시리얼 (`dialout`, 그리퍼 485–USB 보드 등) | INSTALL 10 |
| GStreamer (머리 카메라 RGB 디코딩) | `sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav` |
| 방화벽 (머리 카메라 RGB UDP 5004/5006) | ufw 를 켰으면 `sudo ufw allow from 192.168.123.0/24` |
| Intel GPU / NPU 드라이버 (OpenVINO GPU·NPU) | 패키지 이름·버전 확인 필요 — 예전 PC 의 `system.txt` (커널, `intel-*` / `libze*` apt 목록, NPU 펌웨어)와 맞추고 `python utils/get_dev.py` 로 GPU·NPU 가 보이는지 확인 |
| H2 SDK `third_party/unitree_sdk2_python-814556d` | `source activate_tv.sh` 가 자동으로 받음 |

---

## 하지 말 것

- `conda init` — `.bashrc` 가 바뀜. 스크립트(`activate_tv.sh`)가 필요할 때만 활성화합니다.
- 버전 없이 `pip install torch` — CUDA 판(nvidia-* 수 GB)이 들어옴. 항상 `--index-url …/whl/cpu` 또는 스냅샷.
- `pip install -U <패키지>` — 의존성이 따라 올라가 numpy 2.x 가 들어올 수 있음 (OpenCV 4.10 은 numpy 1.x 전용).
- `pip install pin` — `pinocchio.casadi` 가 없음. pinocchio 는 conda-forge 로만.
- 환경 이름을 `tv` 말고 다른 것으로 — `robot_env.sh` 의 `TV_PY` 와 sudoers 경로가 `envs/tv` 고정입니다.
- base 환경에 패키지 설치.
