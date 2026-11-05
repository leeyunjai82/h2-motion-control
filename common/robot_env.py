"""
robot_env.py — 로봇 설정(robots/h2/robot.yaml) 읽기. 이 저장소는 H2 전용.

  python robot_server.py            # ROBOT 을 안 주면 h2

  · ROBOT 환경변수는 남겨 두었지만 h2 만 받는다 (다른 값이면 import 시점에 종료).
  · robot.yaml 의 enabled=false 면 실행 거부 (시뮬 ROBOT_SIM=1 · 모터 번호 확인 ROBOT_CHECK=1 제외).
  · 로봇 파일(URDF, 메시, 모션 JSON, IK 모델 캐시, robot.yaml)은 robots/h2/ 에 있다.
"""
import os
import sys

import numpy as np
import yaml

REPO_ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMON_DIR = os.path.join(REPO_ROOT, "common")
ROBOTS_DIR = os.path.join(REPO_ROOT, "robots")


def _available():
    try:
        return sorted(d for d in os.listdir(ROBOTS_DIR)
                      if os.path.isfile(os.path.join(ROBOTS_DIR, d, "robot.yaml")))
    except OSError:
        return []


def _fail(msg):
    print(f"[robot_env] ❌ {msg}", file=sys.stderr)
    print(f"[robot_env]    이 저장소는 H2 전용 — ROBOT 은 비우거나 h2   (robots/: {', '.join(_available())})",
          file=sys.stderr)
    sys.exit(2)


def load_config(robot):
    """robots/<robot>/robot.yaml 을 읽어 dict 로 (검증 포함). 실패 시 ValueError."""
    path = os.path.join(ROBOTS_DIR, robot, "robot.yaml")
    if not os.path.isfile(path):
        raise ValueError(f"설정 파일 없음: {path}")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict) or cfg.get("name") != robot:
        raise ValueError(f"{path}: name 이 '{robot}' 가 아님")

    j = cfg["joints"]
    n = int(j["motor_slots"])
    slots = list(j["arm"]) + list(j["waist"]) + list(j.get("head") or []) + [j["weight_slot"]]
    if len(j["arm"]) != 14 or len(j["waist"]) != 3:
        raise ValueError(f"{path}: arm 14개 / waist 3개 이어야 함")
    if any(not (0 <= int(s) < n) for s in slots + list(j["init_slots"]) + list(j["map"].values())):
        raise ValueError(f"{path}: 슬롯 번호가 0~{n - 1} 범위 밖")
    if len(set(slots)) != len(slots):
        raise ValueError(f"{path}: arm/waist/head/weight 슬롯 중복")
    for k in ("x", "y", "z", "pitch_deg"):
        if cfg["camera"].get(k) is None:
            raise ValueError(f"{path}: camera.{k} 없음")
    _k = cfg["camera"].get("intrinsics")
    if not (isinstance(_k, list) and len(_k) == 4 and all(isinstance(v, (int, float)) for v in _k)):
        raise ValueError(f"{path}: camera.intrinsics [fx, fy, ppx, ppy] (camera.resolution 컬러) 4개 필요")
    _r = cfg["camera"].get("resolution", [640, 480])
    if not (isinstance(_r, list) and len(_r) == 2 and all(isinstance(v, int) and v > 0 for v in _r)):
        raise ValueError(f"{path}: camera.resolution [W, H] 정수 2개 필요")

    # 실행에 꼭 필요한 값 — enabled: true 일 때만 검사 (enabled: false 로봇은 "확인 필요" 를 null 로 둔다)
    if cfg.get("enabled", False):
        if not isinstance(cfg.get("default_arm_deg"), list) or len(cfg["default_arm_deg"]) != 14:
            raise ValueError(f"{path}: default_arm_deg 14개 필요")
        if cfg.get("frames", {}).get("pelvis_to_torso") is None or cfg.get("grab", {}).get("z_offset") is None:
            raise ValueError(f"{path}: frames.pelvis_to_torso / grab.z_offset 필요")
        ident = cfg.get("identity") or {}
        if ident.get("required") and ident.get("mode_machine") is None:
            raise ValueError(f"{path}: identity.required 인데 mode_machine 이 null")
        if j.get("head") and (cfg["gains"].get("kp_head") is None or cfg["gains"].get("kd_head") is None):
            raise ValueError(f"{path}: head 슬롯이 있으면 gains.kp_head / kd_head 필요")
    return cfg


# 시뮬레이션 모드 (ROBOT_SIM=1, start_grab_sim.sh) — sim/fake_robot.py 가 로봇 역할.
#   DDS 도메인 1 을 써서 실기(도메인 0)와 절대 섞이지 않게 한다.
#   enabled: false 로봇(H2 등)도 시뮬에서는 실행 허용 (실기 명령 경로는 계속 닫힘).
SIM = os.environ.get("ROBOT_SIM", "").strip() == "1"
DDS_DOMAIN = 1 if SIM else 0

# 실기 모터 번호 확인 모드 (ROBOT_CHECK=1, start_editor.sh real) — enabled: false 로봇을 실기에서
# '확인 목적으로만' 띄운다. 허용 프로세스: arm_server / simulator / dashboard 뿐 (잡기 서버는 거부).
CHECK = os.environ.get("ROBOT_CHECK", "").strip() == "1"
CHECK_ALLOWED = {"arm_server.py", "simulator.py", "dashboard.py", "init_fsm.py", "robot_state.py", "arm_sdk_test.py", "cam_marker_check.py", "hand_to_marker.py"}   # init_fsm: start_fsm.sh, robot_state: 읽기 전용 진단

ROBOT = os.environ.get("ROBOT", "").strip().lower() or "h2"
if ROBOT != "h2":
    _fail(f"ROBOT='{ROBOT}' — 이 저장소는 H2 전용 (G1 코드는 git 이력에)")

ROBOT_DIR = os.path.join(ROBOTS_DIR, ROBOT)
try:
    CFG = load_config(ROBOT)
except Exception as e:  # noqa: BLE001 — 어떤 오류든 실행 거부
    _fail(f"ROBOT='{ROBOT}' 설정 오류 — 실행 거부: {e}")

# 장착 보정 (이 PC 에서 잰 실측 — git 에 안 올림, .gitignore calib_*.yaml): robots/<ROBOT>/calib_local.yaml
#   거치대를 다시 조이면 숙임각이 바뀜 (2026-10-11 46.0 → 42.5) → utils/check_rsimu.py --save 로 다시 재서 저장.
#   있으면 robot.yaml camera 의 x · y · z · pitch_deg 를 덮어씀 (나머지 키는 무시). 읽기 오류면 실행 거부 (틀린 각도로 잡지 않게).
CAMERA_YAML = dict(CFG["camera"])          # robot.yaml 원래 값 (비교·표시용)
LOCAL_CALIB_PATH = os.path.join(ROBOT_DIR, "calib_local.yaml")
LOCAL_CALIB = {}
CAMERA_SOURCE = "robot.yaml"
if os.path.isfile(LOCAL_CALIB_PATH):
    try:
        with open(LOCAL_CALIB_PATH, encoding="utf-8") as _f:
            LOCAL_CALIB = yaml.safe_load(_f) or {}
        _cam = LOCAL_CALIB.get("camera") or {}
        _applied = []
        for _k in ("x", "y", "z", "pitch_deg"):
            if _cam.get(_k) is not None:
                CFG["camera"][_k] = float(_cam[_k])
                _applied.append(f"{_k} {CAMERA_YAML.get(_k)} → {CFG['camera'][_k]}")
        if _applied:
            _when = (LOCAL_CALIB.get("measured") or {}).get("time", "?")
            CAMERA_SOURCE = f"calib_local.yaml ({_when})"
            print(f"[robot_env] 장착 보정 적용 robots/{ROBOT}/calib_local.yaml ({_when}): camera " + ", ".join(_applied),
                  file=sys.stderr)
    except Exception as e:  # noqa: BLE001
        _fail(f"{LOCAL_CALIB_PATH} 읽기 오류 — 실행 거부: {e} (지우거나 utils/check_rsimu.py --save 로 다시 만들 것)")
# 로봇별 unitree_sdk2py (robot.yaml sdk.commit — H2). activate_tv.sh(ensure_robot_sdk) 가 third_party/ 에 받아 둔다.
#   sys.path 앞에 넣어 tv 환경 SDK(G1 고정 커밋)보다 먼저 쓰게 한다. sudo 실행(init_fsm)에서도 같은 경로.
SDK_DIR = None
_sdk_commit = (CFG.get("sdk") or {}).get("commit")
if _sdk_commit:
    SDK_DIR = os.path.join(REPO_ROOT, "third_party", f"unitree_sdk2_python-{str(_sdk_commit)[:7]}")
    if not os.path.isfile(os.path.join(SDK_DIR, "unitree_sdk2py", "__init__.py")):
        _fail(f"ROBOT='{ROBOT}' 용 unitree_sdk2py {str(_sdk_commit)[:7]} 없음: {SDK_DIR} — "
              f"'ROBOT={ROBOT} source activate_tv.sh' 를 한 번 실행하면 자동으로 받습니다")
    _loaded = sys.modules.get("unitree_sdk2py")
    if _loaded is not None and not os.path.abspath(_loaded.__file__).startswith(SDK_DIR + os.sep):
        _fail(f"unitree_sdk2py 가 robot_env 보다 먼저 import 됨 ({_loaded.__file__}) — "
              f"ROBOT='{ROBOT}' 는 {SDK_DIR} 를 써야 함 (import 순서 확인)")
    if SDK_DIR not in sys.path:
        sys.path.insert(0, SDK_DIR)
if CHECK and os.path.basename(sys.argv[0]) not in CHECK_ALLOWED:
    _fail(f"ROBOT_CHECK=1 (모터 번호 확인 모드) 에서는 {sorted(CHECK_ALLOWED)} 만 실행 — "
          f"{os.path.basename(sys.argv[0])} 거부")
if not CFG.get("enabled", False):
    if not (SIM or CHECK):
        _fail(f"ROBOT='{ROBOT}' 는 robot.yaml 에서 enabled: false — 실행 거부 "
              f"({CFG.get('disabled_reason', '사유 미기재')})")
    if SIM:
        print(f"[robot_env] ⚠️ 시뮬레이션: ROBOT='{ROBOT}' enabled: false 지만 ROBOT_SIM=1 (DDS 도메인 {DDS_DOMAIN}) 로 실행",
              file=sys.stderr)
    else:
        print(f"[robot_env] ⚠️⚠️ 실기 모터 번호 확인 모드: ROBOT='{ROBOT}' enabled: false — "
              f"{os.path.basename(sys.argv[0])} 만 실행 (잡기·보행 서버 거부). 로봇을 지지한 상태에서만 사용",
              file=sys.stderr)
if SIM:
    # 실기에서 아직 재지 않은 값 — 시뮬 전용 값(robot.yaml sim:)으로 채운다 (실기 경로에는 영향 없음)
    _sim = CFG.get("sim") or {}
    if CFG.get("default_arm_deg") is None:
        CFG["default_arm_deg"] = list(_sim.get("default_arm_deg") or [0.0] * 14)
        print(f"[robot_env] ⚠️ 시뮬레이션: default_arm_deg 미측정 → sim 값 {CFG['default_arm_deg']}", file=sys.stderr)

URDF_PATH     = os.path.join(ROBOT_DIR, CFG["urdf"])
MESH_DIR      = os.path.join(ROBOT_DIR, "meshes")
MOTIONS_DIR   = os.path.join(ROBOT_DIR, "motions")
IK_CACHE_PATH = os.path.join(ROBOT_DIR, CFG["ik_cache"])
VENDOR_DIR    = os.path.join(COMMON_DIR, "assets", "vendor")   # three.min.js (로봇 무관)

# ---- 자주 쓰는 값 ----
CAMERA       = CFG["camera"]
CAMERA_X     = float(CAMERA["x"])
CAMERA_Y     = float(CAMERA["y"])
CAMERA_Z     = float(CAMERA["z"])
CAMERA_PITCH_DEG = float(CAMERA["pitch_deg"])
CAMERA_PITCH = float(np.radians(CAMERA_PITCH_DEG))
CAMERA_K     = tuple(float(v) for v in CAMERA["intrinsics"])   # (fx, fy, ppx, ppy) 컬러 CAMERA_RES — detect_box / sim
CAMERA_RES   = tuple(int(v) for v in CAMERA.get("resolution", [640, 480]))   # (W, H) 컬러·깊이 — rs_stream / detect_box / sim

JOINTS = CFG["joints"]
FSM    = CFG["fsm"]


_dds_inited = None


def dds_init(interface=None):
    """ChannelFactoryInitialize(DDS_DOMAIN[, interface]).

    실기: 기존과 똑같이 매번 호출한다 (동작 변경 없음).
    시뮬: 같은 프로세스에서 두 번째 호출은 건너뛴다 — 일부 환경에서 cyclonedds 가 같은 도메인을
          다시 만들 때 'create domain error' 로 실패하기 때문 (arm_server → robot_arm 이 두 번 부름).
    """
    global _dds_inited
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    if SIM and _dds_inited is not None:
        return
    if interface:
        ChannelFactoryInitialize(DDS_DOMAIN, interface)
    else:
        ChannelFactoryInitialize(DDS_DOMAIN)
    _dds_inited = DDS_DOMAIN


def loco_client_class():
    """robot.yaml sdk.loco_module 의 LocoClient 클래스 (import 실패 시 ImportError)."""
    import importlib
    return importlib.import_module(CFG["sdk"]["loco_module"]).LocoClient


def check_identity(lowstate_msg):
    """rt/lowstate 메시지로 연결된 로봇이 설정과 같은지 확인 → (ok, 메시지).

    unitree_hg LowState_.motor_state 는 35칸 고정이라 관절 수로는 구분할 수 없어
    mode_machine 을 기준으로 쓴다. 기준값이 null 이면 검사 생략(경고).
    """
    want = (CFG.get("identity") or {}).get("mode_machine")
    got = int(getattr(lowstate_msg, "mode_machine", -1))
    if want is None:
        return True, (f"[robot_env] ⚠️ ROBOT={ROBOT} identity.mode_machine 미설정 — 로봇 확인 생략 "
                      f"(수신 mode_machine={got}, utils/check_robot_id.py 로 측정 후 robot.yaml 에 기입)")
    if got != int(want):
        return False, (f"[robot_env] ❌ 연결된 로봇 mode_machine={got} ≠ robots/{ROBOT}/robot.yaml "
                       f"identity.mode_machine={want} — 다른 로봇이 연결된 것으로 보고 거부")
    return True, f"[robot_env] ✓ 로봇 확인: ROBOT={ROBOT} mode_machine={got}"
