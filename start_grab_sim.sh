#!/usr/bin/env bash
# 잡기 시뮬레이터 — 로봇 없이 잡기/내려놓기/건네기 시퀀스 시험 (모션 에디터는 ./start_editor.sh, 이것과 다름)
# 사용: ./start_grab_sim.sh            가상 카메라 (가상 박스)     → http://localhost:50010/
#       ./start_grab_sim.sh real-cam   실물 D435i + 실제 박스 인식 → http://localhost:50012/
# 종료: Ctrl+C
#
# 구성 (ROBOT_SIM=1 → DDS 도메인 1, 실기 도메인 0 과 분리):
#   fake_robot (DDS 가짜 로봇) → arm_server(50022) → robot_server(50000) → dashboard(50003)
#   → sim_server(50010, 가짜 detect_box + 조작 화면)
#   rs_stream / detect_box 는 띄우지 않는다 (카메라 없음).
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
source "$ROOT/config/ports.env"   # 포트 (한 곳)
source "$ROOT/robot_env.sh" || exit 1
# 시뮬은 enabled: false 로봇도 허용 → robots/<ROBOT>/robot.yaml 존재만 확인
if [ -z "${ROBOT:-}" ] || [ ! -f "$ROOT/robots/${ROBOT}/robot.yaml" ]; then
  echo "❌ robots/${ROBOT:-h2}/robot.yaml 없음" >&2
  exit 2
fi
export ROBOT_SIM=1
unset ROBOT_CHECK
CAM="${1:-virtual}"
case "$CAM" in
  virtual)  unset SIM_CAMERA ;;
  real-cam) export SIM_CAMERA=real ;;   # 카메라·인식(rs_stream + detect_box) 실물, 로봇만 가상
  *) echo "Usage: $0 [real-cam]" >&2; exit 1 ;;
esac

LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"
DAY="$(date '+%Y%m%d')"
# stamp() 는 robot_env.sh (mawk 줄 단위 처리 포함)

# 실기 스택과 같은 포트를 쓰므로, 이미 떠 있으면 아무것도 죽이지 않고 중단
busy=""
for p in ${PORT_ROBOT_SERVER} ${PORT_RS_STREAM} ${PORT_DASHBOARD} ${PORT_DETECT_BOX} ${PORT_SIM_BOX} ${PORT_ARM_SERVER}; do
  if curl -s -m 1 -o /dev/null "http://localhost:$p/" 2>/dev/null; then busy="$busy $p"; fi
done
if [ -n "$busy" ]; then
  echo "❌ 포트 사용 중:$busy — 실행 중인 start_robot.sh / start_grab_sim.sh 를 먼저 종료하세요" >&2
  exit 1
fi

source "$ROOT/activate_tv.sh" || exit 1
cd "$ROOT"

PIDS=(); NAMES=()
cleanup() {
  trap '' INT TERM EXIT
  echo ""; echo "[stop] 종료 중..."
  for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for i in $(seq 1 8); do
    alive=0; for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [ $alive -eq 0 ] && break; sleep 1
  done
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" 2>/dev/null; then echo "[stop] ${NAMES[$i]} 강제 종료"; kill -9 "${PIDS[$i]}" 2>/dev/null || true; fi
  done
  echo "[stop] 완료"; exit 0
}
trap cleanup INT TERM EXIT

run() {  # run <이름> <스크립트> <대기초>
  echo "[sim] $1 ..."
  python -u "$2" > >(stamp >> "$LOG_DIR/sim_$1_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("$1")
  sleep "$3"
}

wait_http() {  # wait_http <url> <초>
  for i in $(seq 1 "$2"); do curl -s -m 1 -o /dev/null "$1" && return 0; sleep 1; done
  echo "[sim] ⚠ 응답 없음: $1 — 로그 확인: $LOG_DIR/sim_*_$DAY.log"; return 1
}

run "fake_robot"   "$ROOT/sim/fake_robot.py" 2
run "arm_server"   "services/arm/arm_server.py"           1
wait_http "http://localhost:${PORT_ARM_SERVER}/status" 60
run "robot_server" "apps/box_handover/robot_server.py"    1
wait_http "http://localhost:${PORT_ROBOT_SERVER}/grab_status" 60
run "dashboard"    "services/dashboard/dashboard.py"      1
if [ "$CAM" = "real-cam" ]; then
  run "rs_stream"  "services/camera/rs_stream.py"         4
  run "detect_box" "apps/box_handover/detect_box.py"      2
  wait_http "http://localhost:${PORT_DETECT_BOX}/status" 60
  run "sim_server" "$ROOT/apps/box_handover/sim_server.py" 1
  wait_http "http://localhost:${PORT_SIM_BOX}/sim/state" 30
  SIM_URL="http://localhost:${PORT_SIM_BOX}/"
else
  run "sim_server" "$ROOT/apps/box_handover/sim_server.py" 1
  wait_http "http://localhost:${PORT_DETECT_BOX}/status" 30
  SIM_URL="http://localhost:${PORT_DETECT_BOX}/"
fi

cat <<EOF

  ✓ 시뮬레이터 실행 중  (ROBOT=$ROBOT, 카메라=$CAM, ROBOT_SIM=1, DDS 도메인 1)
    - 시뮬 화면    : $SIM_URL        (잡기 · 상태 · 카메라 · 3D)
    - 제어 UI      : http://localhost:${PORT_ROBOT_SERVER}/        (실기와 같은 화면)
    - 3D 뷰어      : http://localhost:${PORT_DASHBOARD}/dashboard
    - arm_server   : http://localhost:${PORT_ARM_SERVER}/status
  로그: $LOG_DIR/sim_*_$DAY.log
  종료: Ctrl+C
EOF
wait
