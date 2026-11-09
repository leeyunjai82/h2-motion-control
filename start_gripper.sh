#!/usr/bin/env bash
# 그리퍼 잡기 — 한 창에서: Dex1-1 서비스 (sudo) → arm_server → 잡기 웹 (gripper/pick_web.py :50030)
#   박스 양손 잡기(start_robot.sh)와 따로. 박스 잡기 서버 · rs_stream · 대시보드는 띄우지 않음 (잡기 웹이 카메라를 직접 엶)
#
#   먼저 (한 번):  ROBOT_CHECK=1 ./start_fsm.sh stand        # 거치대 + FSM 703
#   ./start_gripper.sh [인터페이스]       # 기본 enp45s0 — 실기: 팔·그리퍼가 움직임
#   ./start_gripper.sh --sim             # 로봇 없이: fake_robot(DDS 도메인 1) + arm_server + 잡기 웹 (가짜 카메라 · 그리퍼)
#   ./start_gripper.sh --vr              # VR 수집까지: + xr_virtual --real (Quest 입력) + Quest 페이지 열기 → 수집 화면 /collect
#                                          (보통 ./start_collect.sh 로 실행. xr_teleoperate 경로: XR_DIR, 기본 ~/xr_teleoperate)
#   ./start_gripper.sh --no-cam-check    # 시작 때 카메라 숙임 확인(check_rsimu 3 초, 저장 안 함 — 기준값과 1° 넘게 다르면 멈춤)을 건너뜀
#
#   그리퍼 서비스 실행 파일: DEX1_BIN (기본 ~/project/dex1_1_service/bin/dex1_1_gripper_server)
#   종료: Ctrl+C — 잡기 웹(그리퍼 송신 멈춤 → 힘 풀림) → arm_server(팔 제어권 반납) → 그리퍼 서비스 순서.
#         박스를 잡고 있으면 먼저 웹에서 내려놓을 것. 이미 떠 있던 arm_server 는 그대로 둠
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
SIM=0; IFACE="enp45s0"; CAMCHECK=1; VR=0
for a in "$@"; do
  case "$a" in
    --sim) SIM=1 ;;
    --no-cam-check) CAMCHECK=0 ;;
    --vr) VR=1 ;;
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
  DLOG="$LOG_DIR/dex1_service_$DAY.log"; touch "$DLOG"
  # 서비스는 시작할 때 포트를 한 번 빠르게 훑어 모터를 찾음 — 한 손이 그때 응답을 못 하면 빠진 채 뜸
  # (2026-11-05 · 11-06 ID 0 이 빠졌다가 서비스만 다시 띄우자 찾음) → 둘 다 찾을 때까지 최대 3 번
  found=""
  for attempt in 1 2 3; do
    DEX1_SEEN=$(grep -c "Gripper Server started" "$DLOG" || true); DEX1_SEEN=${DEX1_SEEN:-0}
    DEX1_LINE0=$(wc -l < "$DLOG")
    echo "[start] dex1_service  ($DEX1_BIN -n $IFACE) ... (시도 $attempt/3)"
    ( cd "$(dirname "$DEX1_BIN")" && exec sudo "$DEX1_BIN" -n "$IFACE" ) > >(stamp >> "$DLOG") 2>&1 &
    DPID=$!
    ok=0
    for i in $(seq 1 15); do
      n=$(grep -c "Gripper Server started" "$DLOG" || true)
      [ "${n:-0}" -gt "$DEX1_SEEN" ] && { ok=1; break; }
      alive "$DPID" || break
      sleep 1
    done
    if [ $ok -eq 1 ]; then
      sleep 0.5
      found=$(tail -n +"$((DEX1_LINE0 + 1))" "$DLOG" | grep -o "Motor ID: [01]" | sort -u | tr '\n' ' ')
      echo "[start] dex1_service  찾은 모터: ${found:-없음}"
      if [[ "$found" == *"Motor ID: 0"* && "$found" == *"Motor ID: 1"* ]]; then
        PIDS+=("$DPID"); NAMES+=("dex1_service"); break
      fi
    else
      echo "[start] dex1_service  안 뜸 — 로그: $DLOG"
    fi
    sudo -n kill -TERM "$DPID" 2>/dev/null || sudo kill -TERM "$DPID"
    for k in $(seq 1 5); do alive "$DPID" || break; sleep 1; done
    alive "$DPID" && { sudo kill -9 "$DPID" 2>/dev/null || true; }
    found=""
    [ $attempt -lt 3 ] && { echo "[start] 둘 다 못 찾음 — 2 초 뒤 다시"; sleep 2; }
  done
  if [[ "$found" != *"Motor ID: 0"* || "$found" != *"Motor ID: 1"* ]]; then
    echo "❌ 3 번 다 그리퍼 둘 다 못 찾음 (ID 0 = 오른손, ID 1 = 왼손) — 빠진 손 그리퍼의 전원 · 485 선 확인 후 다시" >&2; exit 1
  fi
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

# 3) 카메라 숙임 확인 (실기, 저장 안 함) — 기준값(calib_local.yaml / robot.yaml camera.pitch_deg)은 고정, 로봇 거치대를 거기에 맞춤.
#    ACT 는 수집 · 실행 때 카메라 구도가 같아야 함 → 값을 새로 저장해 덮지 않음 (2026-11-06 사용자). 거치대가 하루에 65.4 → 61.5° 움직인 적 있음
#    |차이| ≤ CAM_OK 진행 · ≤ CAM_MAX 경고 후 진행 · 넘으면 멈춤 (거치대를 맞춘 뒤 다시, 또는 --no-cam-check)
CAM_OK=0.5; CAM_MAX=1.0
if [ $SIM -eq 0 ] && [ $CAMCHECK -eq 1 ]; then
  echo "[start] 카메라 숙임 확인 (check_rsimu 3 초, 저장 안 함 — 로봇을 건드리지 마세요) ..."
  CR=$(cd "$ROOT" && sudo "$TV_PY" "$ROOT/utils/check_rsimu.py" --sec 3 "$IFACE" 2>&1)
  echo "$CR" | stamp >> "$LOG_DIR/check_rsimu_$DAY.log"
  echo "$CR" | grep -E "▶|지금 camera.pitch_deg|❌" | sed 's/^/          /'
  DIFF=$(echo "$CR" | grep "지금 camera.pitch_deg" | sed -n 's/.*(차이 \([-+0-9.]*\).*/\1/p')
  if [ -z "$DIFF" ]; then
    echo "  ⚠ 카메라 숙임을 못 잼 — 확인 없이 계속 (로그: $LOG_DIR/check_rsimu_$DAY.log)"
  else
    ABS=$(echo "$DIFF" | tr -d '+-')
    DIR=$(awk -v d="$DIFF" 'BEGIN{print (d>0)?"덜 숙이게 (위로)":"더 숙이게 (아래로)"}')
    if awk -v a="$ABS" -v m="$CAM_OK" 'BEGIN{exit !(a<=m)}'; then
      echo "  ✓ 카메라 숙임 기준값과 ${DIFF}° — OK"
    elif awk -v a="$ABS" -v m="$CAM_MAX" 'BEGIN{exit !(a<=m)}'; then
      echo "  ⚠ 카메라 숙임 기준값과 ${DIFF}° — 진행하지만, 데이터 수집 전에 거치대를 ${ABS}° ${DIR} 맞추는 것을 권장"
    else
      echo "❌ 카메라 숙임이 기준값과 ${DIFF}° 다름 (허용 ±${CAM_MAX}°) — 거치대를 ${ABS}° ${DIR} 맞추세요." >&2
      echo "   보면서 맞추기: sudo $TV_PY utils/check_rsimu.py   (diff 열이 0 근처가 되게, Ctrl+C 종료)" >&2
      echo "   맞춘 뒤 ./start_gripper.sh 다시 (확인 없이: --no-cam-check)" >&2
      exit 1
    fi
  fi
fi

# 4) 잡기 웹
cd "$ROOT"
echo "[start] pick_web      (50030) ..."
if [ $SIM -eq 1 ]; then
  ROBOT_SIM=1 python -u gripper/pick_web.py --sim --sim-top-z 0.14 > >(stamp >> "$LOG_DIR/${TAG}pick_web_$DAY.log") 2>&1 &
else
  python -u gripper/pick_web.py "$IFACE" > >(stamp >> "$LOG_DIR/pick_web_$DAY.log") 2>&1 &
fi
PIDS+=($!); NAMES+=("pick_web")
wait_http "http://localhost:50030/" 60 || exit 1

# 5) VR 수집 (--vr): Quest 입력 (xr_teleoperate + H2 IK) → 잡기 웹, Quest 페이지 열기
if [ $VR -eq 1 ]; then
  XR_DIR="${XR_DIR:-$HOME/xr_teleoperate}"
  [ -d "$XR_DIR/teleop" ] || { echo "❌ xr_teleoperate 없음: $XR_DIR (XR_DIR 로 지정)" >&2; exit 1; }
  # 앞에 띄운 xr_virtual 이 남아 8012 를 잡고 있으면 새 것이 포트를 못 열고 Quest 는 옛 것에 붙음 (입력이 안 옴) → 먼저 정리
  if pgrep -f "gripper/xr_virtual.py" >/dev/null; then
    echo "[start] 남아 있는 xr_virtual 끔"; pkill -f "gripper/xr_virtual.py"; sleep 2
    pgrep -f "gripper/xr_virtual.py" >/dev/null && { pkill -9 -f "gripper/xr_virtual.py"; sleep 1; }
  fi
  if curl -sk -m 1 -o /dev/null https://localhost:8012/; then
    echo "❌ 8012 포트를 다른 프로그램이 쓰는 중 — 그 프로그램을 끄고 다시:" >&2
    ss -ltnp 2>/dev/null | grep ':8012 ' >&2
    exit 1
  fi
  echo "[start] xr_virtual    (Vuer 8012, Quest 입력 → 잡기 웹) ..."
  python -u gripper/xr_virtual.py --xr "$XR_DIR" --real --display pass-through > >(stamp >> "$LOG_DIR/${TAG}xr_virtual_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("xr_virtual")
  for _ in $(seq 1 60); do curl -sk -m 1 -o /dev/null https://localhost:8012/ && break; sleep 1; done
  curl -sk -m 1 -o /dev/null https://localhost:8012/ || { echo "❌ Vuer (8012) 가 안 뜸 — 로그: $LOG_DIR/${TAG}xr_virtual_$DAY.log" >&2; exit 1; }
  if command -v adb >/dev/null && [ "$(adb get-state 2>/dev/null)" = "device" ]; then
    adb shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null 2>&1
    adb reverse tcp:8012 tcp:8012 >/dev/null
    adb shell am start -a android.intent.action.VIEW -d "https://localhost:8012/?ws=wss://localhost:8012" com.oculus.browser >/dev/null
    echo "  ✓ Quest: 페이지 열음 — 컨트롤러로 왼쪽 아래 'pass-through'"
  else
    echo "  ⚠ Quest 가 adb 에 안 잡힘 (USB · 디버깅 허용 확인) — 연결 뒤:"
    echo "     adb reverse tcp:8012 tcp:8012 && adb shell am start -a android.intent.action.VIEW -d \"https://localhost:8012/?ws=wss://localhost:8012\" com.oculus.browser"
  fi
fi

echo ""
echo "  ✓ 그리퍼 잡기 실행 중 ($([ $SIM -eq 1 ] && echo 'SIM — 가짜 로봇 · 카메라 · 그리퍼' || echo "실기, $IFACE"))"
echo "    - 잡기 웹     : http://$(hostname -I 2>/dev/null | awk '{print $1}'):50030/  (이 PC: http://localhost:50030/)"
[ $VR -eq 1 ] && echo "    - 수집 화면   : http://$(hostname -I 2>/dev/null | awk '{print $1}'):50030/collect  (이 PC: http://localhost:50030/collect)"
echo "    - arm_server  : http://localhost:50022/status"
echo "    - 로그         : $LOG_DIR/${TAG}{dex1_service,arm_server,pick_web$([ $VR -eq 1 ] && echo ',xr_virtual')}_$DAY.log"
echo "    - 종료         : Ctrl+C"
echo ""
# 띄운 것이 죽으면 바로 알림 (다시 띄우지는 않음 — 그리퍼 서비스가 죽으면 그리퍼 힘이 풀림)
# 485–USB 보드 포트가 사라지면 알림 (2026-11-06 보드 USB 가 빠져 오른손 → 전체 응답 없음, 다시 꽂아 복구)
DEAD=(); TTY0=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' '); TTY_WARNED=0
while true; do
  if [ $SIM -eq 0 ]; then
    TTY=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')
    if [ "$TTY" != "$TTY0" ] && [ $TTY_WARNED -eq 0 ]; then
      TTY_WARNED=1
      echo ""
      echo "  ⚠⚠ $(date '+%H:%M:%S') 485–USB 보드 포트가 바뀜: 시작 [${TTY0}] → 지금 [${TTY:-없음}]"
      echo "     보드 USB 가 빠졌거나 다시 잡힘 — 그리퍼가 명령을 안 받을 수 있음. 팔을 [기본 자세로] → Ctrl+C →"
      echo "     USB 를 단단히 꽂고 ./start_gripper.sh 다시"
    fi
  fi
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
