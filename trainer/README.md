# trainer — 학습 PC (RTX 5090) ACT 학습 웹

로봇 PC 웹(`physical_ai`, :50030)에서 받은 **데이터셋 zip** 을 올리면 ACT 를 학습하고, OpenVINO IR 로 바꿔
**model zip** 을 내려 줍니다. 그 zip 을 로봇 웹 Models 에 올리면 끝.

```
로봇 PC :50030  ──데이터셋 zip──▶  학습 PC :50100  ──model zip──▶  로봇 PC :50030
 (수집)                              (학습 → OpenVINO 변환)              (추론 + 안전 필터)
```

이 폴더(`trainer/`)만 있으면 됩니다 — 로봇 SDK·URDF 는 필요 없음.

## 설치 (Ubuntu, RTX 5090)

1. NVIDIA 드라이버 확인 — `nvidia-smi` 오른쪽 위 **CUDA Version 이 12.8 이상**이어야 합니다.
   RTX 50 시리즈(Blackwell, `sm_120`)는 CUDA 12.8 이상으로 빌드된 PyTorch(2.7 이상)만 돕니다.
2. conda 환경 (이름 `act` — 다른 이름이면 실행 때 `TRAINER_ENV=<이름>`)
   ```bash
   conda create -n act python=3.11 -y
   conda activate act
   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
   pip install -r trainer/requirements.txt
   pip install openvino==<로봇 PC 와 같은 버전>
   ```
   - 로봇 PC 버전 확인: `source activate_tv.sh && python -c "import openvino; print(openvino.__version__)"`
     (학습 PC 가 더 새 버전이면 로봇이 IR 을 못 읽을 수 있음 — 로봇 웹 모델 목록에 주황색 경고로 표시)
   - cu128 주소는 PyTorch 설치 페이지(https://pytorch.org/get-started/locally/)에서 지금 것으로 확인.
3. 확인
   ```bash
   python -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0), torch.cuda.get_arch_list())"
   ```
   `True`, `NVIDIA GeForce RTX 5090`, 목록에 `sm_120` 이 있어야 함.

ResNet18 ImageNet 가중치(45 MB)는 첫 학습 때 인터넷에서 한 번 받습니다 (`~/.cache/torch`). 못 받으면 무작위 초기화로 학습하고 로그에 경고.

## 실행

```bash
./trainer/start_trainer.sh
# → http://<학습 PC IP>:50100/   처음 뜰 때 터미널에 '접근 키' 출력 → 웹에서 한 번 입력 (브라우저에 저장)
```

- 키 바꾸기: `trainer/data/key.txt` 삭제 후 재시작, 또는 `TRAINER_KEY=원하는키 ./trainer/start_trainer.sh`
- 데이터 위치: `TRAINER_DATA=/mnt/ssd/trainer ./trainer/start_trainer.sh` (기본 `trainer/data/`, git 에 안 올라감)
- LAN 전용. 인터넷에 열지 마세요 (키는 간단한 잠금일 뿐).

## 웹 사용

1. **Datasets** — 로봇 웹에서 받은 zip 업로드 (진행률 표시). 성공 에피소드 수·분·이미지 크기 확인.
2. **New Training** — 데이터셋 고르고 설정 → 큐에 넣기 (GPU 하나라 한 번에 하나씩, 나머지는 대기).
3. **Job** — step·loss·검증 오차(°)·남은 시간, loss 곡선, 로그.
   - **지금 마치기**: 지금까지 학습한 것으로 바로 변환·zip (loss 가 충분히 내려갔을 때)
   - **취소**: 중단 (zip 안 만듦)
4. 끝나면 **⬇ model zip** → 로봇 웹 Models 에 업로드 → 로드 → 정책 시작.

## 학습 설정 (기본값 = LeRobot ACT 기본)

| 항목 | 기본 | 설명 |
|---|---|---|
| step | 40000 | 에피소드 50개(약 3만 프레임) 기준. 검증 오차가 더 안 내려가면 '지금 마치기' |
| batch | 8 | 5090 이면 16~32 도 됨 (lr 도 같이 올릴 것) |
| 이미지 | 데이터셋 그대로 | 로봇 웹 저장 해상도 (기본 320×240). 640×480 은 느리고 메모리 많이 씀 |
| chunk | 100 | 한 번에 예측하는 행동 수 = 30 Hz 에서 3.3 초. 데모의 정지 구간보다 길어야 정책이 안 멈춤 |
| KL 가중 | 10 | CVAE |
| lr | 1e-5 | 백본도 1e-5 |
| 증강 | 켬 | 이동 ±2 %, 밝기·대비·채도 ±20 % (조명·거치대 조임 차이 대비) |
| 검증 비율 | 0.1 | 에피소드 단위로 떼어 냄 (에피소드 5 개 미만이면 검증 없음) |
| bf16 AMP | 켬 | 5090 에서 빠름. loss 가 nan 이면 끄기 |

## 결과 파일 (`trainer/data/jobs/<작업>/`)

| 파일 | 내용 |
|---|---|
| `<이름>.zip` | 로봇에 올릴 것: `policy.xml/.bin` (OpenVINO IR, FP16 가중치) + `meta.json` |
| `ckpt_last.pt` | 가중치·옵티마이저·통계 — 이어 학습: `python trainer/train_act.py --dataset <데이터셋> --out <새 폴더> --resume <이 파일>` |
| `train.log` | 학습 로그 |

모델 입출력: `image [1,3,H,W]` 0~1 RGB, `state [1,14]` rad → `action [1,chunk,14]` rad.
정규화·역정규화가 모델 안에 들어 있어 로봇은 그대로 넣고 그대로 씁니다.
변환 후 torch 와 OpenVINO(CPU f32) 결과 최대 차이를 `meta.json export_max_abs_diff_deg` 와 웹에 표시.

## 왜 LeRobot 패키지를 직접 안 쓰나

ACT 구조·기본값은 LeRobot `modeling_act.py` 와 같게 `act_model.py` 에 따로 구현했습니다.
- LeRobot 은 torch 버전 범위를 고정해 RTX 5090 용 torch(cu128)와 자주 충돌하고, 버전마다 API(정규화 위치·데이터셋 형식)가 바뀜
- 데이터셋 영상 디코딩(torchcodec/ffmpeg) 의존을 없앰 — 에피소드 JPEG 를 그대로 읽음
- 정규화를 모델 안에 넣어 OpenVINO 변환·로봇 추론을 단순하게

필요하면 나중에 데이터셋을 LeRobot 형식으로 바꾸는 변환기를 따로 만들 수 있습니다.
