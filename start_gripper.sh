#!/usr/bin/env bash
# 그리퍼 잡기 — 한 창에서: Dex1-1 서비스 (sudo) → arm_server → 잡기 웹 (gripper/pick_web.py :50030)
#   박스 양손 잡기(start_robot.sh)와 따로. 박스 잡기 서버 · rs_stream · 대시보드는 띄우지 않음 (잡기 웹이 카메라를 직접 엶)
#
#   먼저 (한 번):  ROBOT_CHECK=1 ./start_fsm.sh stand        # 거치대 + FSM 703
#   ./start_gripper.sh [인터페이스]       # 기본 enp45s0 — 실기: 팔·그리퍼가 움직임
#   ./start_gripper.sh --sim             # 로봇 없이: fake_robot(DDS 도메인 1) + arm_server + 잡기 웹 (가짜 카메라 · 그리퍼)
#
#   그리퍼 서비스 실행 파일: DEX1_BIN (기본 ~/project/dex1_1_service/bin/dex1_1_gripper_server)
#   종료: Ctrl+C — 잡기 웹(그리퍼 송신 멈춤 → 힘 풀림) → arm_server(팔 제어권 반납) → 그리퍼 서비스 순서.
#         박스를 잡고 있으면 먼저 웹에서 내려놓을 것. 이미 떠 있던 arm_server 는 그대로 둠
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
SIM=0; IFACE="enp45s0"
for a in "$@"; do
  case "$a" in
    --sim) SIM=1 ;;
    -h|--help) sed -n 2,11p "$0"; exit 0 ;;
    *) IFACE="$a" ;;
  esac
done
DEX1_BIN="${DEX1_BIN:-$HOME/project/dex1_1_service/bin/dex1_1_gripper_server}"

source "$ROOT/robot_env.sh" || exit 1
if [ ! -f "$ROOT/robots/${ROBOT}/robot.yaml" ]; then echo "❌ robots/${ROBOT}/robot.yaml 없음" >&2; exit 2; fi

up() { curl -s -m 1 -o /dev/null "$1" 2>/dev/null; }
if up "http://localhost:50030/"; then echo "❌ 포트 50030 사용 중 — 실행 중인 잡기 웹을 먼저 끄세요" >&2; exit 1; fi

if [ $SIM -eq 0 ]; then
  echo "⚠️  실기 그리퍼 잡기 (ROBOT=$ROBOT, $IFACE) — 로봇을 거치대에 걸고 FSM 703 (start_fsm.sh stand),"
  echo "    주변을 비우고, E-STOP 을 손에 두세요."
  read -r -p "   계속하려면 yes 입력: " ans
  [ "$ans" = "yes" ] || { echo "취소"; exit 1; }
  [ -x "$DEX1_BIN" ] || { echo "❌ 그리퍼 서비스 없음: $DEX1_BIN (INSTALL.md 10단계, 다른 위치면 DEX1_BIN=...)" >&2; exit 1; }
  ip -br addr show "$IFACE" 2>/dev/null | grep -q "192.168.123." || {
    echo "❌ $IFACE 에 192.168.123.x 주소 없음 — 로봇 123 대역 포트에 연결됐는지 확인 (ip -br addr)" >&2; exit 1; }
  sudo -v || exit 1        # 그리퍼 서비스용 — 비밀번호를 여기서 한 번 (끌 때 sudo kill 도 이 인증으로)
else
  for p in 50022; do
    if up "http://localhost:$p/status"; then echo "❌ 포트 $p 사용 중 (실기 arm_server?) — --sim 은 따로 띄움, 먼저 끄세요" >&2; exit 1; fi
  done
fi

source "$ROOT/activate_tv.sh" || exit 1
LOG_DIR="$ROOT/logs"; mkdir -p "$LOG_DIR"; DAY="$(date '+%Y%m%d')"
TAG="$([ $SIM -eq 1 ] && echo vsim_ || echo "")"

alive() { ps -p "$1" >/dev/null 2>&1; }   # kill -0 은 sudo 로 띄운 (root) 프로세스에 권한이 없어 '없음'으로 잘못 나옴
PIDS=(); NAMES=()          # 띄운 순서 — 끌 때는 거꾸로
KEEP=""                    # sudo 인증 유지 (실행 동안 만료되면 끌 때 비밀번호를 다시 물음)
cleanup() {
  trap '' INT TERM EXIT
  echo ""; echo "[stop] 종료 중..."
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
    pid="${PIDS[$i]}"; alive "$pid" || continue
    echo "[stop] ${NAMES[$i]}"
    if [ "${NAMES[$i]}" = "dex1_service" ]; then sudo -n kill -TERM "$pid" 2>/dev/null || sudo kill -TERM "$pid"; else kill -TERM "$pid" 2>/dev/null || true; fi
    for k in $(seq 1 8); do alive "$pid" || break; sleep 1; done
    if alive "$pid"; then
      echo "[stop] ${NAMES[$i]} 강제 종료"
      if [ "${NAMES[$i]}" = "dex1_service" ]; then sudo kill -9 "$pid" 2>/dev/null || true; else kill -9 "$pid" 2>/dev/null || true; fi
    fi
  done
  [ -n "$KEEP" ] && kill "$KEEP" 2>/dev/null
  if [ $SIM -eq 0 ] && pgrep -f dex1_1_gripper_server >/dev/null; then
    echo "[stop] ⚠ 그리퍼 서비스가 남아 있음 — sudo pkill -f dex1_1_gripper_server"
  fi
  echo "[stop] 완료"; exit 0
}
trap cleanup INT TERM EXIT

wait_http() {  # wait_http <url> <초>
  for i in $(seq 1 "$2"); do up "$1" && return 0; sleep 1; done
  echo "[start] ⚠ 응답 없음: $1 — 로그: $LOG_DIR/${TAG}*_$DAY.log"; return 1
}

# 1) 그리퍼 서비스 (실기만)
if [ $SIM -eq 0 ]; then
  ( while true; do sudo -n true 2>/dev/null; sleep 60; done ) & KEEP=$!
  if pgrep -f dex1_1_gripper_server >/dev/null; then
    echo "❌ 그리퍼 서비스가 이미 떠 있음 (다른 창?) — 끄고 다시: sudo pkill -f dex1_1_gripper_server" >&2; exit 1
  fi
  DEX1_SEEN=$(grep -c "Gripper Server started" "$LOG_DIR/dex1_service_$DAY.log" 2>/dev/null || true); DEX1_SEEN=${DEX1_SEEN:-0}
  echo "[start] dex1_service  ($DEX1_BIN -n $IFACE) ..."
  ( cd "$(dirname "$DEX1_BIN")" && exec sudo "$DEX1_BIN" -n "$IFACE" ) > >(stamp >> "$LOG_DIR/dex1_service_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("dex1_service")
  ok=0
  for i in $(seq 1 15); do
    n=$(grep -c "Gripper Server started" "$LOG_DIR/dex1_service_$DAY.log" 2>/dev/null || true)
    [ "${n:-0}" -gt "$DEX1_SEEN" ] && { ok=1; break; }
    alive "${PIDS[-1]}" || break
    sleep 1
  done
  [ $ok -eq 1 ] || { echo "❌ 그리퍼 서비스가 안 뜸 — 로그: $LOG_DIR/dex1_service_$DAY.log" >&2; exit 1; }
fi

# 2) arm_server (실기에서 이미 떠 있으면 재사용) — 모터 번호 확인 모드(ROBOT_CHECK=1): 잡기·보행 서버는 거부되는 모드
cd "$ROOT/common"
if [ $SIM -eq 1 ]; then
  echo "[start] fake_robot ..."
  ROBOT_SIM=1 python -u "$ROOT/sim/fake_robot.py" > >(stamp >> "$LOG_DIR/${TAG}fake_robot_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("fake_robot"); sleep 2
fi
if [ $SIM -eq 0 ] && up "http://localhost:50022/status"; then
  echo "[start] arm_server    (50022) 이미 실행 중 — 재사용"
else
  echo "[start] arm_server    (50022) ..."
  if [ $SIM -eq 1 ]; then
    ROBOT_SIM=1 python -u arm_server.py > >(stamp >> "$LOG_DIR/${TAG}arm_server_$DAY.log") 2>&1 &
  else
    ROBOT_CHECK=1 python -u arm_server.py > >(stamp >> "$LOG_DIR/arm_server_$DAY.log") 2>&1 &
  fi
  PIDS+=($!); NAMES+=("arm_server")
  wait_http "http://localhost:50022/status" 60 || exit 1
fi

# 3) 잡기 웹
cd "$ROOT"
echo "[start] pick_web      (50030) ..."
if [ $SIM -eq 1 ]; then
  ROBOT_SIM=1 python -u gripper/pick_web.py --sim --sim-top-z 0.14 > >(stamp >> "$LOG_DIR/${TAG}pick_web_$DAY.log") 2>&1 &
else
  python -u gripper/pick_web.py "$IFACE" > >(stamp >> "$LOG_DIR/pick_web_$DAY.log") 2>&1 &
fi
PIDS+=($!); NAMES+=("pick_web")
wait_http "http://localhost:50030/" 60 || exit 1

echo ""
echo "  ✓ 그리퍼 잡기 실행 중 ($([ $SIM -eq 1 ] && echo 'SIM — 가짜 로봇 · 카메라 · 그리퍼' || echo "실기, $IFACE"))"
echo "    - 잡기 웹     : http://$(hostname -I 2>/dev/null | awk '{print $1}'):50030/  (이 PC: http://localhost:50030/)"
echo "    - arm_server  : http://localhost:50022/status"
echo "    - 로그         : $LOG_DIR/${TAG}{dex1_service,arm_server,pick_web}_$DAY.log"
echo "    - 종료         : Ctrl+C"
echo ""
# 띄운 것이 죽으면 바로 알림 (다시 띄우지는 않음 — 그리퍼 서비스가 죽으면 그리퍼 힘이 풀림)
DEAD=()
while true; do
  for i in "${!PIDS[@]}"; do
    [[ " ${DEAD[*]-} " == *" $i "* ]] && continue
    if ! alive "${PIDS[$i]}"; then
      DEAD+=("$i")
      echo ""
      echo "  ⚠⚠ $(date '+%H:%M:%S') ${NAMES[$i]} 가 종료됨 — 로그: $LOG_DIR/${TAG}${NAMES[$i]}_$DAY.log"
      [ "${NAMES[$i]}" = "dex1_service" ] && echo "     그리퍼 힘이 풀렸을 수 있음. 커널 기록: sudo dmesg | grep -i -E 'dex1|segfault' | tail -5"
      echo "     팔을 [기본 자세로] 보낸 뒤 Ctrl+C 로 끄고 다시 실행하세요."
    fi
  done
  sleep 1
done
