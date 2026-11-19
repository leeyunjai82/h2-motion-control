#!/usr/bin/env bash
# 모션 에디터 (apps/motion_editor/simulator.py :8000) — 관절·IK 로 팔 모션을 만들고 저장 (robots/h2/motions/)
#   모드는 반드시 지정 (기본값 없음)
#
#   ./start_editor.sh virtual
#       가상: URDF/메시 3D + fake_robot (ROBOT_SIM=1, DDS 도메인 1). 로봇 없이 모션 편집·모터 번호 화면 연습.
#       띄우는 것: fake_robot → arm_server → dashboard(3D) → 에디터
#
#   ./start_editor.sh real
#       실기: 실제 로봇 팔이 움직인다 (DDS 도메인 0). start_robot.sh 와 같이 써도 됨 —
#       arm_server / dashboard 가 이미 떠 있으면 재사용.
#   ROBOT_CHECK=1 ./start_editor.sh real
#       모터 번호 확인 전용 (arm_server / dashboard / 에디터만, 잡기 서버는 거부). 시작 시 yes 확인.
#       로봇을 지지한 상태에서 사용.
#
#   화면: http://localhost:8000/            모션 에디터
#         http://localhost:8000/check       모터 번호 확인 (슬롯별 각도·Δ·±5° 이동·판정 저장)
#         http://localhost:50003/dashboard  3D (robot.yaml 관절 맵으로 그림)
#   종료: Ctrl+C — 이 스크립트가 띄운 것만 정리 (재사용한 arm_server/dashboard 는 그대로)
#
#   잡기 시퀀스를 로봇 없이 시험하는 것은 ./start_grab_sim.sh (이것과 다름)
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-}"
if [ "$MODE" != "virtual" ] && [ "$MODE" != "real" ]; then
  echo "Usage: $0 virtual|real" >&2
  echo "  virtual : 가상 (URDF 3D + fake_robot, 로봇 없이)" >&2
  echo "  real    : 실기 — 실제 팔이 움직임 (ROBOT_CHECK=1 이면 모터 번호 확인 전용)" >&2
  exit 1
fi

source "$ROOT/robot_env.sh" || exit 1
if [ "$MODE" = "virtual" ]; then
  # 가상은 robot.yaml 존재만 확인
  if [ -z "${ROBOT:-}" ] || [ ! -f "$ROOT/robots/${ROBOT}/robot.yaml" ]; then
    echo "❌ robots/${ROBOT:-h2}/robot.yaml 없음" >&2
    exit 2
  fi
  export ROBOT_SIM=1
  unset ROBOT_CHECK
else
  unset ROBOT_SIM
  if [ "${ROBOT_CHECK:-}" = "1" ]; then
    if [ -z "${ROBOT:-}" ] || [ ! -f "$ROOT/robots/${ROBOT}/robot.yaml" ]; then
      echo "❌ robots/${ROBOT:-h2}/robot.yaml 없음" >&2; exit 2
    fi
    echo "⚠️  실기 모터 번호 확인 모드 (ROBOT=$ROBOT) — 로봇을 지지하고, 주변을 비우고, E-STOP 을 손에 두세요."
    read -r -p "   계속하려면 yes 입력: " ans
    [ "$ans" = "yes" ] || { echo "취소"; exit 1; }
    export ROBOT_CHECK=1
  else
    require_robot     # robot.yaml enabled: true 만
  fi
fi

LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"
DAY="$(date '+%Y%m%d')"
TAG="$([ "$MODE" = virtual ] && echo vsim_ || echo "")"
# stamp() 는 robot_env.sh (mawk 줄 단위 처리 포함)

up() { curl -s -m 1 -o /dev/null "$1" 2>/dev/null; }

# 에디터가 이미 떠 있으면 중단 (아무것도 죽이지 않음)
if up "http://localhost:8000/"; then
  echo "❌ 포트 8000 사용 중 — 실행 중인 모션 에디터를 먼저 종료하세요" >&2; exit 1
fi
if [ "$MODE" = "virtual" ]; then
  # 가상 모드는 실기 스택과 같은 포트를 쓰므로 떠 있으면 중단
  for p in 50003 50022; do
    if up "http://localhost:$p/"; then
      echo "❌ 포트 $p 사용 중 (실기/다른 시뮬 스택) — 먼저 종료하세요" >&2; exit 1
    fi
  done
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

run() {  # run <이름> <스크립트>
  echo "[start] $1 ..."
  python -u "$2" > >(stamp >> "$LOG_DIR/${TAG}$1_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("$1")
}
wait_http() {  # wait_http <url> <초>
  for i in $(seq 1 "$2"); do up "$1" && return 0; sleep 1; done
  echo "[start] ⚠ 응답 없음: $1 — 로그: $LOG_DIR/${TAG}*_$DAY.log"; return 1
}

if [ "$MODE" = "virtual" ]; then
  run "fake_robot" "$ROOT/sim/fake_robot.py"; sleep 2
fi

# arm_server — 실기에서 이미 떠 있으면(start_robot.sh 등) 재사용
if [ "$MODE" = "real" ] && up "http://localhost:50022/status"; then
  echo "[start] arm_server    (50022) 이미 실행 중 — 재사용"
else
  run "arm_server" "services/arm/arm_server.py"
  wait_http "http://localhost:50022/status" 60
fi

# dashboard (3D) — 실기에서 이미 떠 있으면 재사용
if [ "$MODE" = "real" ] && up "http://localhost:50003/"; then
  echo "[start] dashboard     (50003) 이미 실행 중 — 재사용"
else
  run "dashboard" "services/dashboard/dashboard.py"
fi

run "simulator" "apps/motion_editor/simulator.py"
wait_http "http://localhost:8000/" 30

echo ""
echo "  ✓ 모션 에디터 실행 중  (ROBOT=$ROBOT, 모드=$MODE$([ "${ROBOT_CHECK:-}" = 1 ] && echo ' · 모터 번호 확인'))"
echo "    - 모션 에디터     : http://localhost:8000/"
echo "    - 모터 번호 확인  : http://localhost:8000/check"
echo "    - 3D             : http://localhost:50003/dashboard"
echo "    - arm_server     : http://localhost:50022/status"
echo "    - 로그            : $LOG_DIR/${TAG}*_$DAY.log"
echo "    - 종료            : Ctrl+C"
echo ""
wait
