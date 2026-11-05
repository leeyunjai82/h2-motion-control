# 설치 가이드 — H2 Motion Control · Ubuntu 24.04 LTS

외장 Intel mini PC(x86_64)에 **Ubuntu 24.04 LTS** 를 기준으로 설치하는 절차입니다. 순서대로 진행하세요.

- 사용법 : [`README.md`](./README.md)
- H2 실측·근거 : [`robots/h2/FACTS.md`](./robots/h2/FACTS.md)
- 가상환경(tv) 다시 만들기 / 스냅샷 : [`REBUILD_ENV.md`](./REBUILD_ENV.md)

> "확인 필요" 표시는 이 문서 작성 시점에 실기/운영 PC 에서 검증하지 못한 항목입니다. 설치하면서 결과를 기록해 주세요.

---

## 1. 시스템 요구사항

| 항목 | 권장 |
| --- | --- |
| OS | **Ubuntu 24.04 LTS** (x86_64) |
| Python | 3.10 (Miniconda `tv` 환경) |
| 메모리 / 디스크 | 8 GB 이상 / 10 GB 이상 여유 |
| 네트워크 | 로봇과 같은 LAN `192.168.123.0/24` (유선 권장) |
| 카메라 | Intel RealSense D435i (USB 3.0 포트) |
| 권한 | sudo 가능한 계정 |

설치 경로 예: `$HOME/project/h2-motion-control/`.
스크립트는 계정명을 하드코딩하지 않습니다 (`$HOME`, 스크립트 위치 기준).

---

## 2. OS 기본 패키지 (시스템 — 최소)

Python 쪽은 전부 `tv` 가상환경에 넣고, 시스템에는 꼭 필요한 것만 설치합니다.

```bash
sudo apt update
sudo apt install -y wget curl
```

- `curl` : `start_grab_sim.sh` / `start_editor.sh` 가 서버 응답 확인에 사용 (가상환경 활성화 전에 호출).
- 빌드 도구(build-essential, libssl-dev, libsuitesparse-dev 등)는 **필요 없습니다** — cyclonedds 는 pip 바이너리 휠,
  scikit-sparse 는 conda-forge 패키지로 설치합니다.
- 머리 카메라(`head_track`, `utils/check_head_cam.py`) RGB 디코딩 — 없으면 `start_robot.sh` 가 head_track 을 건너뜀:
  `sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav`
- OpenCV 가 `libGL.so.1` 을 찾지 못하면 (서버판 Ubuntu 등): `sudo apt install -y libgl1 libglib2.0-0`
- 로그 타임스탬프(`awk strftime`) : 24.04 기본 `mawk 1.3.4 20240123` 에서 동작 확인.
  mawk 는 파이프 입력을 블록 단위로 모아 읽어 로그가 늦게 기록되므로 `robot_env.sh` 의 `stamp()` 가 `-W interactive` 로 줄 단위 처리.

---

## 3. Intel RealSense (D435i) — pip `pyrealsense2` 만

코드는 Python `pyrealsense2` 만 씁니다. `requirements.txt` 의 `pyrealsense2==2.55.1.6486` 이 7단계에서 **`tv` 가상환경 안에 설치**되며
(PyPI 바이너리 휠 `cp310 manylinux1_x86_64`, librealsense 라이브러리 포함), **apt 로 RealSense SDK 를 깔 필요는 없습니다.**

카메라는 USB 3.0 포트에 연결하고, 7단계 후 확인:

```bash
source $HOME/miniconda3/bin/activate tv
python -c "import pyrealsense2 as rs; d=rs.context().devices; print([x.get_info(rs.camera_info.name)+' '+x.get_info(rs.camera_info.usb_type_descriptor) for x in d])"
# ['Intel RealSense D435I 3.2'] 처럼 나오면 정상 (3.x = USB3)
conda deactivate
```

장치가 안 잡히거나 권한 오류가 나면 (일반 계정 USB 접근) — 같은 버전(v2.55.1)의 udev 규칙 파일만 시스템에 넣습니다:

```bash
sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.55.1/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# 카메라 USB 를 뺐다 다시 꽂기
```

- `librealsense2-dkms` (커널 패치) 는 설치하지 않습니다 — 공식 문서상 HWE 커널 5.15/5.19/6.5 만 지원, 24.04 커널(6.8 이상) 비대상.
- `realsense-viewer` 같은 GUI 도구가 필요할 때만 apt 저장소(librealsense.realsenseai.com)의 `librealsense2-utils` 를 설치합니다 (선택).

---

## 4. Miniconda — `.bashrc` 를 건드리지 않고 설치

로봇을 돌리는 계정 하나에만 설치합니다. **`conda init` 을 하지 않으므로** 다른 계정과 이 계정의 평소 셸에는 영향이 없습니다.

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3     # -b: 묻지 않음, .bashrc 수정 없음
rm Miniconda3-latest-Linux-x86_64.sh
$HOME/miniconda3/bin/conda --version
```

> `conda init` 을 실행하지 마세요. 필요할 때만 활성화합니다 (아래 5단계 · "평소 사용").
> 다른 위치(예: `/opt/miniconda3`)에 설치했다면 `export CONDA_BASE=/opt/miniconda3` — 스크립트가 이 값을 씁니다.

---

## 5. `tv` 가상환경 생성 (conda-forge 만 사용)

```bash
source $HOME/miniconda3/bin/activate          # 이 터미널에서만 conda 사용
conda create -y -n tv --override-channels -c conda-forge \
    python=3.10 pinocchio=3.1.0 numpy=1.24.4 scikit-sparse=0.4.16 git
conda activate tv
```

- `--override-channels -c conda-forge` : Anaconda 기본 채널을 쓰지 않으므로 약관(ToS) 승인 절차가 없습니다 (conda 26.7.1 확인).
- `pinocchio=3.1.0` : xr_teleoperate 공식 절차와 같은 버전. **pip 의 `pin` 패키지는 `pinocchio.casadi` 가 없어 쓰면 안 됩니다.**
- `numpy=1.24.4` : requirements.txt 와 같은 값 (pip 가 numpy 를 다시 바꾸지 않게). OpenCV 4.10 때문에 2.x 금지.
- `scikit-sparse` : conda-forge 바이너리 → 시스템 `libsuitesparse-dev` 불필요.
- `git` : 가상환경 안의 git (unitree_sdk2py 를 GitHub 에서 받을 때 사용).

---

## 6. 프로젝트 받기

```bash
mkdir -p $HOME/project && cd $HOME/project
git clone https://github.com/leeyunjai82/h2-motion-control.git      # tv 활성화 상태 → 가상환경의 git
cd h2-motion-control
source activate_tv.sh          # 처음 한 번: H2 용 unitree_sdk2py(robot.yaml sdk.commit 814556d)를 third_party/ 에 받음
```

---

## 7. Python 패키지 (전부 `tv` 안)

```bash
# (tv 활성화 상태, 프로젝트 폴더에서)
pip install --upgrade pip

# PyTorch 는 CPU 판 (Intel mini PC — CUDA 판은 수 GB 의 nvidia 패키지가 같이 깔림)
pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cpu

# 나머지 — unitree_sdk2py 소스는 가상환경 안($CONDA_PREFIX/src)에 받는다
pip install -r requirements.txt --src "$CONDA_PREFIX/src"

# requirements.txt 에 없는 것
pip install meshcat==0.3.2 logging-mp openvino
```

확인:
```bash
python -c "import numpy, torch, cv2, pinocchio, sksparse, unitree_sdk2py, openvino; from pinocchio import casadi; print('numpy', numpy.__version__, '| torch', torch.__version__)"
# numpy 1.24.4 | torch 2.4.1+cpu 가 나와야 정상
```

- 검증 (2026-10, Ubuntu 24.04): 위 순서로 설치 후 `./start_grab_sim.sh` 잡기 시퀀스 완료.
  운영 mini PC 실설치에서 `numpy 1.24.4 | torch 2.4.1+cpu` 확인 (CPU 판 torch 정상).
- `openvino`, `logging-mp` 는 버전을 고정하지 않았습니다 — 운영 mini PC 의 `pip freeze` 값과 맞추는 것을 권장 (확인 필요).
- `unitree_sdk2py`: requirements 의 고정 커밋 `f559291` 은 tv 환경 기본값이고 `unitree_sdk2py.h2` 가 없습니다.
  H2 는 `robots/h2/robot.yaml sdk.commit` (814556d) 을 `third_party/` 에 받아 `common/robot_env.py` 가 그쪽을 먼저 씁니다
  (`activate_tv.sh` / `start_*.sh` 가 처음 한 번 자동으로 받음).

### 평소 사용 (conda 는 필요할 때만)

- `start_*.sh` / `launcher.sh` 는 스스로 `tv` 를 활성화합니다 → **그냥 실행하면 됩니다.**
- 터미널에서 직접 python 을 쓸 때만:
  ```bash
  source $HOME/miniconda3/bin/activate tv      # 또는 프로젝트 폴더에서: source activate_tv.sh
  ...
  conda deactivate                             # 끝나면
  ```

---

## 8. sudo 설정 (`start_fsm.sh`)

`start_fsm.sh` 는 tv 환경 python 을 `sudo` 로 실행합니다. 비밀번호 없이 쓰려면 NOPASSWD 규칙을 추가합니다.

```bash
sudo visudo -f /etc/sudoers.d/h2-motion
```
```
<사용자> ALL=(root) NOPASSWD: /home/<사용자>/miniconda3/envs/tv/bin/python
```

- 경로는 스크립트가 실제 실행하는 python 과 **정확히 같아야** 합니다. 확인:
  ```bash
  source robot_env.sh && echo "$TV_PY"
  sudo -n "$TV_PY" -c "print('ok')"     # 비밀번호를 묻지 않아야 정상
  ```
- 로봇 이름(h2)은 sudo 뒤에서 **명령 인자**로 넘기므로(`init_fsm.py <mode> h2`, `run_launcher.py h2`) sudoers 에 `SETENV` 를 줄 필요가 없습니다.
- `launcher.sh` 는 시작 시 `sudo -v` 로 비밀번호를 한 번 묻습니다 (NOPASSWD 불필요).

---

## 9. 네트워크

로봇 대역 `192.168.123.0/24` (H2 PC1 `192.168.123.161`). PC 유선 LAN 을 같은 대역으로 설정합니다.

```bash
nmcli connection show                     # 유선 연결 이름 확인
sudo nmcli connection modify "<연결이름>" ipv4.addresses 192.168.123.222/24 ipv4.method manual
sudo nmcli connection up "<연결이름>"
ping 192.168.123.161
```

- 방화벽: Ubuntu 기본 `ufw` 는 비활성입니다 (`sudo ufw status`). 켜져 있다면 로봇 대역을 허용하세요: `sudo ufw allow from 192.168.123.0/24`.
- DDS 인터페이스: 서버들은 `ChannelFactoryInitialize(0)` 로 인터페이스를 자동 선택합니다. Wi-Fi 와 유선이 같이 켜진 PC 에서 로봇 데이터가 안 들어오면 Wi-Fi 를 끄고 확인하세요 (인터페이스 지정 옵션은 3단계에서 공통 설정으로 추가 예정).

---

## 10. Dex1-1 그리퍼 (`dex1_1_service`, 그리퍼를 달았을 때만)

구성: 그리퍼 —485→ 485–USB 보드 (UART 4 포트) —USB-C→ 미니 PC 의 **`dex1_1_service`** (C++, 시리얼 ↔ DDS) —DDS `rt/dex1/{left,right}/{cmd,state}`→ 이 저장소 (Python).
그리퍼 전원은 H2 [2] 또는 [11] (24 V, GXT30(2)) 의 +/− 두 핀 (CAN 두 핀은 비움, 꽂기 전에 멀티미터로 +24 V · 극성 확인).
2026-11-05 이 미니 PC (Ubuntu 24.04.4) 에서 아래 순서로 설치 · 보정 · 움직임 확인.

### 10-1. 설치 (로봇 꺼도 됨)

```bash
sudo apt update
sudo apt install -y git cmake g++ build-essential \
    libserialport-dev libspdlog-dev libboost-all-dev libyaml-cpp-dev libfmt-dev libeigen3-dev
sudo usermod -aG dialout $USER                       # 다시 로그인 후 적용

# C++ unitree_sdk2 (→ /usr/local, CycloneDDS libddsc · libddscxx 같이 설치)
mkdir -p ~/project && cd ~/project
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install

# dex1_1_service — 실행 파일은 bin/ 에 생김
cd ~/project
git clone https://github.com/unitreerobotics/dex1_1_service.git
cd dex1_1_service && mkdir build && cd build
cmake .. -DCMAKE_EXE_LINKER_FLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib"
make -j$(nproc)
ldd ../bin/dex1_1_gripper_server | grep -i dds       # /usr/local/lib/libddsc.so.0 · libddscxx.so.0 만 (0debian 없어야 함)
```

- **링커 옵션이 꼭 필요**: 시스템에 Ubuntu 패키지 CycloneDDS (`/lib/x86_64-linux-gnu/libddsc.so.0debian`) 가 있으면 빌드가 그쪽 `ddsc` 를 붙여
  unitree_sdk2 의 `libddscxx` 와 버전이 섞임 → 서비스가 모터를 찾은 직후 토픽을 만들다 `free(): invalid pointer` 로 죽음.
- 로봇 쪽 유선 인터페이스 이름: `ip -br addr | grep 192.168.123` (예: `enp45s0`). 서비스 · 보정 · 시험 모두 **`-n <이름>`** 필수 —
  기본값 `eth0` 이면 `does not match an available interface` 로 바로 멈춤.

### 10-2. 보정 (로봇 켜서 그리퍼 전원)

보정 = 지금 자리를 닫힘 q 0 으로. **전원 한 번에 한 번만** 됨 (`only one successful change is allowed each power-up`).

1. 서비스가 떠 있으면 끔 (`Ctrl+C`) — 같은 시리얼 포트를 씀
2. 그리퍼 24 V 전원을 뽑고, 전원 없는 상태에서 집게를 **천천히** 끝까지 닫음 (전원이 있으면 모터 쪽 저항이 커서 뻑뻑함 — 세게 비틀지 말 것)
3. 닫힌 채로 전원을 다시 꽂고 바로:
   ```bash
   cd ~/project/dex1_1_service/bin
   sudo ./dex1_1_gripper_server -c -n enp45s0
   ```
   ID 0 · ID 1 차례마다 집게가 맞닿아 있는지 보고 `s` + Enter → 둘 다 `Calibration successful!`. 비어 있는 포트의 `does not reply` 경고는 정상
4. 끝나면 전원을 한 번 더 껐다 켜고 서비스를 띄움

덜 닫힌 채로 보정하면 그 자리가 0 이 되어, 펴진 상태에서 q 가 음수로 나오거나 벌리라는 명령에 안 움직임 → 위 순서로 다시.

### 10-3. 실행 · 확인

```bash
cd ~/project/dex1_1_service/bin
sudo ./dex1_1_gripper_server -n enp45s0              # 'Dex1-1 Gripper Server started.' — 이 창은 그대로

# 다른 터미널
cd ~/project/h2-motion-control && source activate_tv.sh
python gripper/check_dex1.py enp45s0 --sec 15          # 읽기만 — 손으로 여닫아 q 확인 (닫힘 ≈ 0, 끝까지 벌림 ≈ 5.3)
python gripper/move_test.py enp45s0 --side left   # 천천히 2.7 → 0.3 rad (left / right / 기본 양손)
```

- 서비스는 모터 ID 0 → `rt/dex1/right`, ID 1 → `rt/dex1/left` 로 내보냄. 이 로봇은 토픽 이름에 맞춰 **ID 0 을 오른손, ID 1 을 왼손**에 닮
  (`gripper/gripper.yaml topics`) → 이 저장소 도구 · 공식 도구(`test_dex1_1_gripper_server -l/-r`, xr_teleoperate) 모두 같은 손.
  달기 전에 그리퍼 하나만 전원을 넣고 `gripper/check_dex1.py` 로 어느 쪽 q 가 변하는지 보고 ID 를 확인할 것.
- 명령이 끊기면 서비스가 BRAKE · kp = kd = 0 (힘 풀림). 잡은 물체를 놓칠 수 있으니 실제 사용 코드는 명령을 끊김 없이 보낼 것.
- 서비스가 Python 쪽이 붙는 순간 `ddsi_xt_type_init_impl with invalid type object` 뒤 세그멘테이션 오류로 죽은 적이 한 번 있음 (재현 안 됨).
  다시 나오면 `sudo gdb -batch -ex run -ex bt --args ./dex1_1_gripper_server -n enp45s0 2>&1 | tee /tmp/dex1_gdb.log` 로 스택을 남길 것.
- `ModemManager` 가 새로 꽂힌 ttyACM 장치를 잡으면: `sudo systemctl disable --now ModemManager` (모뎀 안 쓰는 PC 에서만).
- mandro 손은 2026-11-02 부터 안 씀 (코드 삭제) — 이 저장소는 시리얼 포트를 직접 열지 않음.

---

## 11. 성능: BLAS 스레드 1개 고정 (중요)

양팔 IK(pinocchio + CasADi)는 작은 선형계를 풉니다. 멀티스레드 OpenBLAS 에서는 동기화 오버헤드가 커서 IK 1회가 ~1.5 ms → ~40 ms 로 느려질 수 있습니다.

```bash
mkdir -p $HOME/miniconda3/envs/tv/etc/conda/activate.d
cat > $HOME/miniconda3/envs/tv/etc/conda/activate.d/threads.sh << 'EOF'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
EOF
conda deactivate && conda activate tv && echo $OMP_NUM_THREADS   # 1
```

> start 스크립트는 `activate_tv.sh` 로 tv 를 활성화하므로 위 훅이 적용됩니다.

---

## 12. 실행 권한

```bash
chmod +x robot_env.sh activate_tv.sh start_fsm.sh start_robot.sh start_editor.sh start_grab_sim.sh launcher.sh
```

---

## 13. 설치 검증

```bash
source activate_tv.sh

# (1) 설정 읽기 (ROBOT 은 안 줘도 h2)
python -c "import sys; sys.path.insert(0,'common'); import robot_env as r; print(r.ROBOT, r.URDF_PATH, r.SDK_DIR)"

# (2) 다른 ROBOT → 거부되어야 정상
ROBOT=g1 python -c "import sys; sys.path.insert(0,'common'); import robot_env"     # ❌ H2 전용 메시지 + 종료코드 2

# (3) 핵심 패키지
python -c "import numpy, cv2, torch, pyrealsense2, pinocchio, casadi, unitree_sdk2py; print('OK')"

# (4) NumPy 1.x + OpenCV 인코딩
python -c "
import numpy as np, cv2
assert np.__version__ < '2', np.__version__
print('imencode', cv2.imencode('.jpg', np.zeros((480,640,3), np.uint8))[0])"

# (5) RealSense
python -c "import pyrealsense2 as rs; print(rs.context().devices[0].get_info(rs.camera_info.name))"

# (6) (선택) IK 벤치마크 — 스레드 고정 시 수 ms
cd common && python -c "
import time, numpy as np, pinocchio as pin
import robot_env
from ctrl.robot_arm_ik import ArmIK
ik = ArmIK(); q=np.zeros(14); dq=np.zeros(14)
L=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3, 0.2,0.1])).homogeneous
R=pin.SE3(pin.Quaternion(1,0,0,0),np.array([0.3,-0.2,0.1])).homogeneous
ik.solve_ik(L,R,q,dq); t=time.time()
for _ in range(50): ik.solve_ik(L,R,q,dq)
print(f'IK avg: {(time.time()-t)/50*1000:.1f} ms')"; cd ..

# (7) 로봇 식별값 (로봇 연결 후, 읽기 전용) — 3단계 안전장치 기준값 측정용
python utils/check_robot_id.py
```

---

## 문제 해결

| 증상 | 원인 / 해결 |
| --- | --- |
| `ROBOT='…' — 이 저장소는 H2 전용` | `unset ROBOT` (안 주면 h2) |
| `conda 를 찾지 못했습니다` | `export CONDA_BASE=<conda 설치 경로>` |
| sudo 가 계속 비밀번호 요구 (FSM) | sudoers 경로 ≠ `$TV_PY`. 8단계 확인 |
| `CondaToSNonInteractiveError` | 기본 채널을 쓴 경우 — 5단계처럼 `--override-channels -c conda-forge` 로 다시 생성 |
| `cyclonedds` 빌드 실패 | Python 3.10 휠이 안 받아진 경우 — `python -V` 가 3.10 인지(tv 활성화) 확인 |
| `scikit-sparse`: `cholmod.h` 없음 | pip 로 빌드하려 한 것 — 5단계 conda-forge `scikit-sparse` 로 설치 |
| OpenCV `img is not a numpy array` | NumPy 2.x. `pip install "numpy<2"` |
| IK 가 수십 ms | BLAS 스레드 고정 (11단계) |
| RealSense 인식 안 됨 / 권한 오류 | USB 3.0 포트, 3단계 udev 규칙 파일 설치 후 재연결 |
| RealSense 가 `2.1` (USB2) 로 잡힘 | **케이블 교체** (USB2 전용 케이블이면 2.1 — 운영 PC 에서 케이블 교체로 3.2 확인), 카메라 C 커넥터 끝까지, 허브 없이 파란(SS) 포트 |
| USB 시리얼(`/dev/ttyUSB*` · `ttyACM*`) 열기 실패 | `dialout` 그룹, ModemManager (10단계) |
| Dex1 서비스 `eth0: does not match an available interface` | `-n <유선 인터페이스>` 붙이기 (10-1) |
| Dex1 서비스 `free(): invalid pointer` (모터 찾은 직후) | CycloneDDS 두 벌 섞임 — 10-1 링커 옵션으로 다시 빌드, `ldd` 에 `0debian` 없어야 함 |
| Dex1 보정 `only one successful change is allowed each power-up` | 그리퍼 24 V 전원 껐다 켜고 다시 (10-2) |
| Dex1 펴진 상태 q 가 음수 · 벌리라는 명령에 안 움직임 | 덜 닫힌 채 보정됨 — 10-2 다시 |
| `pickle.load` 시 `class version ...` | 다른 pinocchio 버전의 캐시 → `robots/h2/` 의 `robot.yaml ik_cache` 파일 삭제 후 재생성 |
| 서버가 남아 있음 | `./start_robot.sh` 가 시작 시 TERM → 대기 → KILL 로 정리. 수동: `pgrep -af "python.*(rs_stream|arm_server|robot_server)"` 확인 후 `kill` |
