import os
import sys
import time

# 로봇 선택 — start_fsm.sh 가 sudo 뒤에서 두 번째 인자로 넘긴다 (sudo 가 환경변수를 지우므로).
#   python init_fsm.py <stand|sit|bal|damp|no-bal> <robot> [check]
#   <robot> : h2 (이 저장소는 H2 전용 — 다른 값이면 robot_env 가 거부)
#   check : 모터 번호 확인 모드 (ROBOT_CHECK=1) — enabled: false 일 때 기립/앉기용. start_fsm.sh 가 확인 후 넘긴다.
# FSM ID 는 robots/h2/robot.yaml fsm (H2: damp 1 Damp / lock 4 FixStand / run 703 PhaseWalk(제자리 밸런스) / sit 3 Sit).
#   stand  = damp → 5초 → lock → 10초 → run (1 → 4 → 703)
#   bal    = run (703),  no-bal = lock (4),  damp = 1,  sit = 3
#   <FSM ID> = robot.yaml fsm.names 에 있는 ID 만 (0 Zero Torque, 1 Damp, 2 Squat, 3 Sit, 4 FixStand, 601 HybridWalk, 701 WalkNew, 703 PhaseWalk)
if len(sys.argv) < 3 or not (sys.argv[1] in ("stand", "sit", "bal", "damp", "no-bal") or sys.argv[1].isdigit()):
    print("Usage: python init_fsm.py [stand|sit|bal|damp|no-bal|<FSM ID>] <robot=h2> [check]")
    sys.exit(1)

os.environ["ROBOT"] = sys.argv[2]
if len(sys.argv) > 3:
    if sys.argv[3] != "check":
        print("Usage: python init_fsm.py [stand|sit|bal|damp|no-bal|<FSM ID>] <robot=h2> [check]")
        sys.exit(1)
    os.environ["ROBOT_CHECK"] = "1"
import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env   # ROBOT 미지정 / robot.yaml 없음 / enabled: false 면 여기서 종료

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

LocoClient = robot_env.loco_client_class()   # robot.yaml sdk.loco_module
F = robot_env.FSM
DAMP, LOCK, RUN, SIT = int(F["damp"]), int(F["lock"]), int(F["run"]), int(F["sit"])

def setfsmid(n):
    print(f"[setfsmid-START]: id({n})")
    res = loco_client.SetFsmId(n)
    print(f"[setfsmid-END]: Result({res})")

mode = sys.argv[1]

robot_env.dds_init()

# 연결된 로봇 확인 (robot.yaml identity.mode_machine) — 다르면 FSM 명령을 보내지 않는다
_sub = ChannelSubscriber("rt/lowstate", hg_LowState)
_sub.Init()
_msg = None
_t_end = time.time() + 3.0
while _msg is None and time.time() < _t_end:
    _msg = _sub.Read(0.5)
if _msg is None:
    print("[init_fsm] ⚠️ rt/lowstate 수신 없음 — 로봇 확인 불가")
    if robot_env.CHECK:
        print("[init_fsm] ❌ 모터 번호 확인 모드: 로봇 상태 수신 없이 FSM 명령하지 않음 (네트워크 192.168.123.x / 전원 확인)")
        sys.exit(3)
    if (robot_env.CFG.get("identity") or {}).get("mode_machine") is not None:
        print("[init_fsm] ❌ identity.mode_machine 이 설정된 로봇은 확인 없이 명령하지 않음")
        sys.exit(3)
else:
    _ok, _why = robot_env.check_identity(_msg)
    print(_why)
    if not _ok:
        sys.exit(3)

loco_client = LocoClient()
loco_client.Init()
loco_client.SetTimeout(10.0)

if mode == "stand":
    setfsmid(DAMP)
    time.sleep(5)
    setfsmid(LOCK)
    time.sleep(10)
    setfsmid(RUN)
elif mode == "sit":
    time.sleep(3)
    setfsmid(SIT)
elif mode == "bal":
    time.sleep(3)
    setfsmid(RUN)
elif mode == "no-bal":
    time.sleep(3)
    setfsmid(LOCK)
elif mode == "damp":
    time.sleep(3)
    setfsmid(DAMP)
elif mode.isdigit():
    # FSM ID 직접 — robot.yaml fsm.names 에 있는 ID 만 (오타·임의 ID 방지)
    fid = int(mode)
    if fid not in {int(k) for k in F["names"]}:
        print(f"[init_fsm] ❌ FSM {fid} 는 robot.yaml fsm.names 에 없음 — 거부 ({sorted(int(k) for k in F['names'])})")
        sys.exit(1)
    time.sleep(3)
    setfsmid(fid)
