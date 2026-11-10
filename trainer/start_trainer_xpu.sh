#!/usr/bin/env bash
# 로봇 PC (Intel GPU) 학습 웹 → http://localhost:50100/
#   conda env act_xpu (./trainer/install_trainer_xpu.sh 로 설치) 만 씀 — 로봇 tv 환경과 별개
#   학습하는 동안 로봇 서버 · 수집은 끄고 쓸 것. 종료: Ctrl+C (GPU 클럭 제한도 원래대로)
#
# 2026-11-10 실기 (NUCS BOX-358H · Arc B390 · 커널 7.0.0-42 · torch 2.11.0+xpu):
#   GPU 최대 클럭 2500 MHz 로 학습하면 시작 순간 95 ℃ · 시스템 전체가 몇 초씩 멈춤 · 와이파이 카드가 떨어지고 약 1 분 뒤 PC 멈춤.
#   최대 클럭을 1200 MHz 로 제한하니 와이파이 · PC 가 버팀 (trainer/xpu_check.py --augment --iters 1000, utils/sys_watch.sh 로 확인)
#   → 이 스크립트가 띄울 때 GPU 최대 클럭을 XPU_MAX_MHZ (기본 1200) 로 제한하고, 끌 때 원래 값으로 되돌림 (sudo 필요)
HERE="$(cd "$(dirname "$0")" && pwd)"
export TRAINER_ENV=act_xpu
export TRAINER_DEVICE="${TRAINER_DEVICE:-xpu}"           # 웹 '학습 장치' 기본값 (CPU 로 바꿀 수 있음)
export TRAINER_NUM_WORKERS="${TRAINER_NUM_WORKERS:-2}"   # 데이터 로더 (로봇 PC: 학습이 PC 를 다 차지하지 않게)
export TRAINER_THREADS="${TRAINER_THREADS:-$(( $(nproc) > 6 ? $(nproc) - 4 : 2 ))}"     # 코어 몇 개는 SSH · 웹 몫으로 남김
XPU_MAX_MHZ="${XPU_MAX_MHZ:-1200}"

FREQ=$(ls /sys/class/drm/card*/device/tile0/gt0/freq0/max_freq 2>/dev/null | head -1)
OLD=""
if [ -n "$FREQ" ]; then
  OLD=$(cat "$FREQ")
  MINF=$(cat "$(dirname "$FREQ")/min_freq")
  if [ "$XPU_MAX_MHZ" -lt "$MINF" ]; then
    echo "❌ XPU_MAX_MHZ $XPU_MAX_MHZ 가 GPU 최소 클럭 $MINF 보다 작음" >&2; exit 1
  fi
  echo "[trainer] GPU 최대 클럭 $OLD → $XPU_MAX_MHZ MHz 로 제한 (끌 때 되돌림, sudo 비밀번호를 물을 수 있음)"
  echo "$XPU_MAX_MHZ" | sudo tee "$FREQ" >/dev/null || { echo "❌ GPU 클럭 제한 실패 — Intel GPU 학습은 위험 (PC 멈춤). 웹에서 학습 장치 CPU 로" >&2; }
  echo "[trainer] 지금 GPU 최대 클럭: $(cat "$FREQ") MHz"
else
  echo "⚠ GPU 클럭 설정 파일 (xe tile0/gt0/freq0/max_freq) 없음 — 제한 없이 띄움" >&2
fi
restore() {
  # 학습 프로세스는 서버와 따로 (start_new_session) 라 서버를 꺼도 남음 → 클럭을 되돌리기 전에 먼저 끔 (제한 없이 계속 돌면 PC 멈춤)
  if pgrep -f "trainer/train_act.py" >/dev/null; then
    echo "[trainer] 돌던 학습 끔 (클럭 제한을 풀기 전에)"
    pkill -TERM -f "trainer/train_act.py"; sleep 3; pkill -KILL -f "trainer/train_act.py" 2>/dev/null
  fi
  if [ -n "$FREQ" ] && [ -n "$OLD" ]; then
    echo "$OLD" | sudo tee "$FREQ" >/dev/null && echo "[trainer] GPU 최대 클럭 되돌림 → $(cat "$FREQ") MHz"
  fi
}
trap restore EXIT
trap 'exit 130' INT TERM

"$HERE/start_trainer.sh" "$@"
