#!/usr/bin/env bash
# 로봇 PC (Intel GPU) 학습 웹 → http://localhost:50100/
#   conda env act_xpu (./trainer/install_trainer_xpu.sh 로 설치) 만 씀 — 로봇 tv 환경과 별개
#   학습 장치는 Intel XPU (Arc 내장 그래픽). 학습하는 동안 로봇 서버 · 수집은 끄고 쓸 것
HERE="$(cd "$(dirname "$0")" && pwd)"
export TRAINER_ENV=act_xpu
# 로봇 PC 는 코어 · 메모리가 적음 — 데이터 로더 2 개 · CPU 연산 스레드 4 개 (학습이 PC 를 다 차지해 SSH 가 끊기지 않게). 바꾸려면 앞에 지정
# 2026-11-10 실기: Intel GPU (XPU) 로 학습을 시작하면 약 25 초 뒤 와이파이 펌웨어 오류 → 시스템이 멈춤 (커널 7.0.0-42, xe 드라이버)
#   → 학습 장치 기본 CPU (64 GB · 다코어). Intel GPU 를 다시 시험하려면 웹 '학습 장치' 에서 Intel GPU 를 고름 (유선 · 화면 앞에서)
export TRAINER_DEVICE="${TRAINER_DEVICE:-cpu}"
export TRAINER_NUM_WORKERS="${TRAINER_NUM_WORKERS:-4}"
export TRAINER_THREADS="${TRAINER_THREADS:-$(( $(nproc) > 6 ? $(nproc) - 4 : 2 ))}"     # 코어 몇 개는 SSH · 웹 몫으로 남김
exec "$HERE/start_trainer.sh" "$@"
