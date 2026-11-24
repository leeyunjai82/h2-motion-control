#!/usr/bin/env bash
# LeRobot 학습 웹 → http://<PC>:50100/
# 사용: ./apps/trainer/start_trainer.sh        (처음이면 먼저 ./apps/trainer/install_lerobot.sh)
#   python: LEROBOT_PY=<경로> → config/local.yaml lerobot_python → ~/miniconda3/envs/lerobot/bin/python
#   데이터 위치 바꾸기: TRAINER_DATA=/mnt/ssd/trainer ./apps/trainer/start_trainer.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PY="${LEROBOT_PY:-$(cd "$REPO" && python3 -c "from pitron.learning import lerobot_env; print(lerobot_env.python())" 2>/dev/null)}"
[ -n "$PY" ] || PY="$HOME/miniconda3/envs/lerobot/bin/python"
echo "[trainer] python: $PY"
if [ ! -x "$PY" ] || ! "$PY" -c "import lerobot, fastapi, uvicorn" 2>/dev/null; then
  echo "❌ lerobot 환경 없음 — 먼저 설치: ./apps/trainer/install_lerobot.sh (다른 위치면 config/local.yaml lerobot_python)" >&2
  exit 1
fi
export LEROBOT_PY="$PY"
DATA="${TRAINER_DATA:-$REPO/data/trainer}"
[ -z "${TRAINER_DATA:-}" ] && [ ! -e "$REPO/data/trainer" ] && [ -d "$REPO/trainer/data" ] && DATA="$REPO/trainer/data"   # 옛 위치 (옮기기 전)
LOGD="$DATA/logs"
mkdir -p "$LOGD"
"$PY" -u "$HERE/trainer_server.py" 2>&1 | tee -a "$LOGD/trainer_$(date +%Y%m%d).log"
