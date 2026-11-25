# trainer — LeRobot 학습 웹 (:50100)

LeRobot v3 데이터셋으로 `lerobot-train` 을 돌리고 **lerobot 모델** (pretrained_model 폴더 + `h2_model.json`) 을 만듭니다.
미니 PC (Intel Arc, 홈 화면 → 학습) 와 학습 PC (RTX 5090) 가 같은 코드.

```
수집 PC                           학습 웹 :50100                              추론 (수집 PC)
data/datasets/<이름>/lerobot  ──▶  lerobot-train (정책 종류 고름)   ──▶  data/policies/<이름>   (같은 PC 면 자동)
LeRobot v3 zip (다른 PC)      ──▶  마지막 체크포인트 → model zip    ──▶  수집 화면에 zip 올리기 (다른 PC)
```

데이터셋 · 학습 · 모델은 서로 독립 — 이어 주는 약속은 state 16 (관절 순서) · action 16 · 카메라 이름 · 크기 · 30 fps 뿐.

## 설치 — 처음 한 번 (lerobot 가상환경 둘)

| 환경 | 쓰는 곳 | torch | 설치 |
|---|---|---|---|
| `lerobot` | 로봇: 수집 v3 변환 · 추론 | CPU + OpenVINO (ACT 추론은 OpenVINO 로 Intel GPU) | `./apps/trainer/install_lerobot.sh robot` |
| `lerobot_train` | 학습 (이 웹) | 미니 PC: XPU · 5090: CUDA | `./apps/trainer/install_lerobot.sh train-xpu` · `./apps/trainer/install_lerobot.sh train-cuda` |

- 둘 다 python 3.10 · ffmpeg · torch 2.10.0 · torchvision 0.25.0 · lerobot 0.4.4 · fastapi · uvicorn · openvino
- 수집 · 추론은 GPU torch 가 필요 없음 → XPU 문제가 학습에만 걸림. 학습 PC (5090) 는 `lerobot_train` 만 있으면 됨
- 환경 이름은 고정 (`lerobot` · `lerobot_train`). 없으면 학습 웹 · 추론이 설치 명령을 띄우고 멈춤
- lerobot 0.4.4 는 `torch<2.11` 을 요구 → 예전 학습 환경 (act · act_xpu, torch 2.11) 과 따로 만듦
- Arc B390 · torch 2.10.0+xpu: 잡힘 · 2048² 행렬곱 4.9 ms (2026-11-25 미니 PC). 긴 학습 (클럭 1200 MHz 제한 상태) 은 **확인 필요**
- 다른 위치에 만들었으면 `config/local.yaml` 에 `lerobot_python:` · `lerobot_train_python:`

## 실행

```bash
./apps/trainer/start_trainer.sh            # 학습 PC → http://<PC>:50100/
./apps/trainer/start_trainer_xpu.sh        # 미니 PC 단독 (보통은 홈 화면 → 학습이 띄움)
```

- 미니 PC Intel GPU 학습은 GPU 최대 클럭을 1200 MHz 로 제한한 상태에서만 (제한 없이 돌리면 PC 멈춤, 2026-11-10 실기).
  홈 화면이 띄우면 런처가 제한 · 복원, `start_trainer_xpu.sh` 는 스스로 제한 · 복원. 끌 때 학습 프로세스부터 끄고 클럭을 되돌림
- 데이터 위치: `TRAINER_DATA=/mnt/ssd/trainer` (기본 `data/trainer`). LAN 전용 — 접근 키 없음

## 웹 사용

1. **데이터셋** — 이 PC 에서 수집한 것은 바로 보임 (수집). 다른 PC 에서 받은 LeRobot v3 zip 은 [올리기]. 예전 형식 zip 은 v3 로 바꿔 넣음
2. **새 학습** — 정책 종류 (lerobot 에 있는 것: act · diffusion · smolvla · pi0 · pi05 · vqbet · tdmpc …) · step · batch · 장치 · 저장 주기 ·
   이어서 학습 / 미세 조정 (이 PC 모델 이름 또는 Hub id) · 추가 옵션 (lerobot-train 인자 그대로) → 큐 (한 번에 하나)
3. **진행** — step · loss 곡선 · 마지막 체크포인트 · 남은 시간 · 로그
   - **지금 마치기**: 학습을 멈추고 마지막 체크포인트로 모델을 만듦 · **취소**: 모델 안 만듦
   - 끊긴 작업 (서버가 꺼짐 → **중단됨**): **이어서 학습** = 학습 상태까지 저장된 마지막 체크포인트에서 같은 설정으로 이어 감 (lerobot `--resume`) ·
     **지금 마치기** = 남은 체크포인트로 바로 모델
   - SSH 터미널에서 띄운 학습 웹은 SSH 가 끊기면 같이 꺼짐 (2026-11-25, 82 % 에서) → 긴 학습은 홈 → 학습 또는 tmux 안에서
4. 끝나면 **⬇ model zip** (다른 PC 로) · 이 PC 가 수집 PC 면 `data/policies/<이름>` 에 자동 저장 → 추론에서 고름

- 사전학습 바탕 모델이 필요한 정책 (smolvla · pi0 · pi05 · groot …) 은 Hub 에서 받으므로 인터넷이 필요. 나머지는 오프라인 (`HF_HUB_OFFLINE=1`)
- 정책마다 카메라 조건이 다름 — 예: diffusion 은 모든 카메라 크기가 같아야 함 (수집 때 카메라 크기를 같게)

## 결과 (`data/trainer/jobs/<작업>/`)

| 경로 | 내용 |
|---|---|
| `out/checkpoints/<step>/pretrained_model/` | lerobot 체크포인트 (저장 주기마다) |
| `model/<이름>/` · `<이름>.zip` | 마지막 체크포인트 + `h2_model.json` (데이터셋 · 작업 · 카메라 맞춤 방식 · 관절 이름 · step · loss) |
| `train.log` | lerobot-train 로그 |

## 확인 (2026-11-24, 컨테이너 CPU)

- ACT · diffusion 학습 → 모델 저장 · zip, 지금 마치기 (마지막 체크포인트), loss 곡선, 예전 형식 zip → v3 변환 후 학습
- 5090 · 미니 PC Intel GPU 위 학습은 아직

예전 자체 ACT 학습기 (OpenVINO zip) 는 `tools/legacy_act/` 로 옮김.
