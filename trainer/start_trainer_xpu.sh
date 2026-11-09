#!/usr/bin/env bash
# 로봇 PC (Intel GPU) 학습 웹 → http://localhost:50100/
#   conda env act_xpu (./trainer/install_trainer_xpu.sh 로 설치) 만 씀 — 로봇 tv 환경과 별개
#   학습 장치는 Intel XPU (Arc 내장 그래픽). 학습하는 동안 로봇 서버 · 수집은 끄고 쓸 것
HERE="$(cd "$(dirname "$0")" && pwd)"
export TRAINER_ENV=act_xpu
exec "$HERE/start_trainer.sh" "$@"
