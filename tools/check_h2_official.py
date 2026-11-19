#!/usr/bin/env python3
"""H2 공식 일치 검사 — 이 repo 의 ROBOT=h2 ArmController / ArmIK  vs  xr_teleoperate H2_ArmController / H2_ArmIK.

로봇 없이 실행 (unitree_sdk2py 는 가짜 모듈, tv 환경: numpy·pinocchio·casadi·pyyaml·logging_mp 필요).
  git clone https://github.com/unitreerobotics/xr_teleoperate /tmp/xr && git -C /tmp/xr checkout 817fb00
  python tools/check_h2_official.py /tmp/xr .

검사: 같은 가짜 DDS(lowstate) 로 두 컨트롤러를 띄워 같은 팔 목표를 줬을 때 LowCmd 35 슬롯 (mode/q/dq/tau/kp/kd)·헤더·
      속도 제한 (simulation_mode True/False — False 는 clip 경로), 우리 쪽 허리 명령은 무시되는지,
      IK 축소 모델 관절 이름·한계, 같은 목표 40개 solve_ik 결과.
  공식과 다른 점은 EnableArmSDK 호출 하나 (실기에서 필요, simulation_mode / ROBOT_SIM 에서는 호출 안 함 → 비교 대상 아님).
"""
import os, sys, types, time, copy, importlib.util
import numpy as np
XR, REPO = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
os.environ["ROBOT"] = "h2"; os.environ["ROBOT_SIM"] = "1"
import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env as RE

fails = []
def check(name, a, b):
    ok = (a == b) if not isinstance(a, np.ndarray) else np.array_equal(a, b)
    print(("  ✓ " if ok else "  ✗ ") + name + ("" if ok else f"\n      공식={a!r}\n      우리={b!r}"))
    if not ok: fails.append(name)

class MC:
    def __init__(self): self.mode=0; self.q=0.0; self.dq=0.0; self.tau=0.0; self.kp=0.0; self.kd=0.0
class LowCmd:
    def __init__(self): self.mode_pr=0; self.mode_machine=0; self.motor_cmd=[MC() for _ in range(35)]; self.crc=0
class MS:
    def __init__(self, i): self.q=0.01*i+0.003; self.dq=0.001*i
class IMU: rpy=[0,0,0]; accelerometer=[0,0,0]; gyroscope=[0,0,0]
class LS:
    def __init__(self): self.motor_state=[MS(i) for i in range(35)]; self.imu_state=IMU(); self.mode_machine=1
WRITES = []; PUBS = []
class Pub:
    def __init__(self, topic, t): self.topic=topic; PUBS.append(self)
    def Init(self): pass
    def Write(self, m): WRITES.append((self, self.topic, copy.deepcopy(m)))
class Sub:
    def __init__(self, topic, t): pass
    def Init(self, *a, **k): pass
    def Read(self, *a): time.sleep(0.001); return LS()
for name in ["unitree_sdk2py","unitree_sdk2py.core","unitree_sdk2py.core.channel","unitree_sdk2py.idl",
             "unitree_sdk2py.idl.unitree_hg","unitree_sdk2py.idl.unitree_hg.msg","unitree_sdk2py.idl.unitree_hg.msg.dds_",
             "unitree_sdk2py.idl.unitree_go","unitree_sdk2py.idl.unitree_go.msg","unitree_sdk2py.idl.unitree_go.msg.dds_",
             "unitree_sdk2py.idl.default","unitree_sdk2py.utils","unitree_sdk2py.utils.crc",
             "teleop","teleop.robot_control","teleop.robot_control.dds_utils"]:
    sys.modules[name] = types.ModuleType(name)
ch = sys.modules["unitree_sdk2py.core.channel"]
ch.ChannelPublisher, ch.ChannelSubscriber, ch.ChannelFactoryInitialize = Pub, Sub, (lambda *a, **k: None)
for d in ("unitree_sdk2py.idl.unitree_hg.msg.dds_", "unitree_sdk2py.idl.unitree_go.msg.dds_"):
    sys.modules[d].LowCmd_ = LowCmd; sys.modules[d].LowState_ = LS
sys.modules["unitree_sdk2py.idl.default"].unitree_hg_msg_dds__LowCmd_ = LowCmd
sys.modules["unitree_sdk2py.idl.default"].unitree_go_msg_dds__LowCmd_ = LowCmd
sys.modules["unitree_sdk2py.utils.crc"].CRC = type("CRC", (), {"Crc": lambda self, m: 0})
def _wait(cond, name, timeout=5):
    t = time.time()
    while not cond() and time.time() - t < timeout: time.sleep(0.01)
sys.modules["teleop.robot_control.dds_utils"].wait_for_dds = _wait

def load(path, modname):
    spec = importlib.util.spec_from_file_location(modname, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

def run(cls, sim, ours):
    WRITES.clear(); n0 = len(PUBS)
    a = cls(motion_mode=True, simulation_mode=sim)
    mine = PUBS[n0]                                  # 이 인스턴스의 publisher (앞 인스턴스 스레드가 계속 쓰므로 구분)
    time.sleep(0.2)
    a.ctrl_dual_arm(np.linspace(-0.5, 0.5, 14), np.linspace(-0.1, 0.1, 14))
    if ours:
        a.ctrl_waist([0.1, -0.05, 0.02])          # 공식은 허리 명령 없음 → 우리도 무시돼야 함
    time.sleep(0.2)
    _, topic, msg = [w for w in WRITES if w[0] is mine][-1]
    snap = [(c.mode, round(c.q, 9), c.dq, round(c.tau, 9), c.kp, c.kd) for c in msg.motor_cmd]
    return topic, (msg.mode_pr, msg.mode_machine), snap, a.get_current_dual_arm_q().tolist(), a.arm_velocity_limit

off = load(os.path.join(XR, "teleop/robot_control/robot_arm.py"), "xr_robot_arm")
our = load(os.path.join(REPO, "pitron/drivers/arm/robot_arm.py"), "our_robot_arm")
for sim in (True, False):
    print(f"[ArmController] motion_mode=True simulation_mode={sim} (False = 속도 제한 clip 경로)")
    o = run(off.H2_ArmController, sim, False)
    n = run(our.ArmController, sim, True)
    check("토픽", o[0], n[0]); check("헤더 (mode_pr, mode_machine)", o[1], n[1])
    for i in range(35):
        if o[2][i] != n[2][i]: check(f"motor_cmd[{i}] (mode,q,dq,tau,kp,kd)", o[2][i], n[2][i])
    check("motor_cmd 35 슬롯 전체", o[2], n[2])
    check("팔 상태 q (슬롯 15–28)", o[3], n[3])
    check("팔 속도 제한 rad/s", o[4], n[4])

print("[ArmIK] 같은 목표 열 → solve_ik 결과")
sys.path.insert(0, XR)
for name in ["teleop.utils", "teleop.utils.weighted_moving_filter"]:
    sys.modules.pop(name, None)
sys.modules.pop("teleop", None)
import importlib
cwd = os.getcwd(); os.chdir(os.path.join(XR, "teleop"))   # 공식은 ../assets/h2/H2.urdf, 캐시 h2_model_cache.pkl (cwd 기준)
xr_ik_mod = load(os.path.join(XR, "teleop/robot_control/robot_arm_ik.py"), "xr_robot_arm_ik")
ik_o = xr_ik_mod.H2_ArmIK()
os.chdir(cwd)
our_ik_mod = load(os.path.join(REPO, "pitron/motion/robot_arm_ik.py"), "our_robot_arm_ik")
ik_n = our_ik_mod.ArmIK()
import pinocchio as pin
mo, mn = ik_o.reduced_robot.model, ik_n.reduced_robot.model
check("축소 모델 관절 이름", list(mo.names), list(mn.names))
check("관절 하한", np.round(mo.lowerPositionLimit, 9).tolist(), np.round(mn.lowerPositionLimit, 9).tolist())
check("관절 상한", np.round(mo.upperPositionLimit, 9).tolist(), np.round(mn.upperPositionLimit, 9).tolist())
rng = np.random.default_rng(0)
q0 = np.radians(np.array(RE.CFG["default_arm_deg"], dtype=float))
qo, qn, maxd = q0.copy(), q0.copy(), 0.0
for k in range(40):
    L = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25 + 0.2 * rng.random(), 0.15 + 0.1 * rng.random(), 0.0 + 0.3 * rng.random()])).homogeneous
    R = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25 + 0.2 * rng.random(), -0.15 - 0.1 * rng.random(), 0.0 + 0.3 * rng.random()])).homogeneous
    qo, _ = ik_o.solve_ik(L, R, qo, None)
    qn, _ = ik_n.solve_ik(L, R, qn, None)
    maxd = max(maxd, float(np.abs(qo - qn).max()))
check(f"solve_ik 40회 관절각 최대 차이 {np.degrees(maxd):.2e}° (< 1e-6°)", True, bool(np.degrees(maxd) < 1e-6))
print("\n결과:", "✓ 공식과 동일" if not fails else f"✗ {len(fails)}건 다름")
