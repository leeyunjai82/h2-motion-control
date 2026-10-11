#!/usr/bin/env bash
# 피지컬 AI 로봇 PC 웹 (수집 · 데이터셋 다운로드 · 모델 업로드 · 정책 추론) → http://<로봇 PC>:50030/
# 사용: ./physical_ai/start_policy.sh             실기 — start_robot.sh 를 먼저 띄운 뒤 다른 터미널에서
#       ./physical_ai/start_policy.sh sim         시뮬 — ./start_grab_sim.sh 를 먼저 띄운 뒤
#       ./physical_ai/start_policy.sh real-cam    시뮬 + 실물 D435i — ./start_grab_sim.sh real-cam 을 먼저
# 종료: Ctrl+C (정책이 돌고 있으면 arm_server 스트림 세션은 2 초 뒤 스스로 끝나고 팔은 마지막 자세 유지)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
case "${1:-}" in
  "")       unset ROBOT_SIM SIM_CAMERA ;;
  sim)      export ROBOT_SIM=1; unset SIM_CAMERA ;;
  real-cam) export ROBOT_SIM=1 SIM_CAMERA=real ;;
  *) echo "Usage: $0 [sim|real-cam]" >&2; exit 1 ;;
esac
source "$ROOT/robot_env.sh" || exit 1
if [ -z "${ROBOT_SIM:-}" ]; then require_robot; fi
if curl -s -m 1 -o /dev/null "http://localhost:50030/" 2>/dev/null; then
  echo "❌ 50030 사용 중 — policy_server 가 이미 떠 있음" >&2; exit 1
fi
if ! curl -s -m 1 -o /dev/null "http://localhost:50022/status" 2>/dev/null; then
  echo "⚠ arm_server(50022) 응답 없음 — start_robot.sh / start_grab_sim.sh 를 먼저 띄우세요 (화면은 뜨지만 수집·추론 불가)" >&2
fi
source "$ROOT/activate_tv.sh" || exit 1
mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/policy_$(date '+%Y%m%d').log"
echo "[policy] http://localhost:50030/   로그: $LOG"
cd "$ROOT/physical_ai"
python -u policy_server.py 2>&1 | stamp | tee -a "$LOG"
