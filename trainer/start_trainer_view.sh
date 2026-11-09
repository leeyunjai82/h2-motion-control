#!/usr/bin/env bash
# 학습 결과 보여주기용 → http://<이 PC IP>:50100/
#   보기 전용: 학습 그래프 · 기록 · model zip 받기만. 학습 · 업로드 · 지우기 안 됨 (대기 중인 학습도 시작 안 함)
#   학습할 때는 이걸 끄고 ./trainer/start_trainer_xpu.sh
HERE="$(cd "$(dirname "$0")" && pwd)"
export TRAINER_VIEW=1
export TRAINER_ENV="${TRAINER_ENV:-act_xpu act}"
exec "$HERE/start_trainer.sh" "$@"
