# legacy_act — 예전 자체 ACT 학습기 (LeRobot 전환 전)

지금 흐름 (수집 → LeRobot v3 · lerobot-train · lerobot 모델 추론) 에서는 쓰지 않음. 예전 형식 데이터셋 (h2-gripper-episodes-v1) 으로
예전 OpenVINO model zip (h2-gripper-act-v1) 을 다시 만들어야 할 때만.

- `train_act.py` — 학습 → OpenVINO IR → model zip (`python tools/legacy_act/train_act.py --dataset <폴더> --out <폴더>`)
- `act_model.py` · `dataset.py` — 위 학습기가 씀
- `xpu_check.py` · `install_trainer*.sh` — 예전 학습 환경 (act · act_xpu, torch 2.11)

예전 데이터셋을 지금 흐름으로 옮기기: `python tools/export_lerobot.py <데이터셋 폴더 또는 zip> --out <폴더>` (lerobot 환경)
