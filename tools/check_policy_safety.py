#!/usr/bin/env python3
"""
check_policy_safety.py — 실기 전 정책 실행기 안전 점검 (로봇 · arm_server 없이, 이 PC 의 실제 모델 · 추론 장치로)

  source activate_tv.sh
  python tools/check_policy_safety.py                       → 쓸 수 있는 모델 목록 · lerobot 환경 확인
  python tools/check_policy_safety.py act_xxx --device GPU  → 그 모델 점검 (이름은 목록에서)

  apps/gripper_pick/policy.py 의 PolicyRunner 를 그대로 쓰고, 팔 서버 · 카메라 · 그리퍼만 가짜 (팔 명령은 기록만).
  추론은 진짜 (lerobot 가상환경 프로세스 · OpenVINO/torch) — 이 PC 의 타이밍으로 확인.
  항목마다 PASS / FAIL. 하나라도 FAIL 이면 실기 금지. 각 항목은 그 항목이 시험하는 필터만 켜고 돌림 (다른 필터가 가리지 않게).

    1  불러오기             모델 · 장치 · 첫 추론 시간
    2  정상 실행            지령 주기 (30 Hz) · 가장 긴 간격 < 0.2 s (arm_server 가 0.2 s 뒤 멈추는 기준) · 관절 한계 · 그리퍼 범위
    3  NaN 출력             → 멈춤
    4  학습 범위 밖         → 멈춤
    5  한 틱 큰 점프        → 멈춤 (범위 안이어도 지금 목표에서 크게 튀면)
    6  지령 ↔ 실측 차이     → 멈춤
    7  테이블 아래          → 멈춤
    8  몸통 · 양팔 · 뒤 한계 → 멈춤
    9  그리퍼 조임 제한     지금 손가락보다 squeeze 만큼만 더 조임
   10  [정지]               → 멈춤, /stream/end 뒤 팔 명령 없음, 추론 중이어도 프로세스는 살아 있고 다음 시작 됨
   11  시작 조건            준비 자세가 아니면 · 테이블을 안 쟀으면 시작 거부
   12  추론 무응답          → 멈춤 + 추론 프로세스 끔 + 다시 불러오기 요구
"""
import argparse
import json
import os
import sys
import time
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "apps", "gripper_pick"))
sys.path.append(ROOT)
try:
    import numpy as np
    import pick_core as pc  # noqa: E402
except ImportError as e:
    sys.exit(f"❌ {e} — 로봇 tv 환경에서 실행: source activate_tv.sh && python tools/check_policy_safety.py")
import policy as pol  # noqa: E402

RES = []


def result(name, ok, detail=""):
    RES.append((name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


class FakeArm:
    """pick_core 대신 — arm_server /stream 만 흉내 (받은 명령 기록, meas_off 로 실측을 일부러 어긋나게). 나머지는 진짜 pick_core."""

    def __init__(self, q0):
        self.q0 = q0
        self.LAST_CMD = {"q": None, "t": 0}
        self.reset()

    def __getattr__(self, k):                   # SIDES · Q_LO · Q_HI · P · MM_PER_RAD · at_ready · Model … = 진짜 pick_core
        return getattr(pc, k)

    def reset(self):
        self.cmds, self.ended_at, self.after_end, self.meas_off, self.open = [], None, 0, 0.0, False

    def call(self, path, body=None, timeout=1.0):
        if path == "/stream/begin":
            self.open, self.ended_at = True, None
            return {"token": "t", "target_deg": np.degrees(self.q0).tolist()}
        if path == "/stream/cmd":
            if not self.open:
                self.after_end += 1
            self.cmds.append((time.time(), np.array(body["deg"], float)))
            m = list(body["deg"])
            m[0] += self.meas_off
            return {"target_deg": body["deg"], "meas_deg": m}
        if path == "/stream/end":
            self.open, self.ended_at = False, time.time()
        return {}


def main():
    ap = argparse.ArgumentParser(description="정책 실행기 안전 점검 (로봇 없이)")
    ap.add_argument("model", nargs="?", help="data/policies 안 모델 이름 (없으면 목록만)")
    ap.add_argument("--device", default="CPU", choices=("CPU", "GPU", "NPU"))
    ap.add_argument("--sec", type=float, default=6.0, help="정상 실행 시간 [s]")
    a = ap.parse_args()
    if not a.model or not os.path.isfile(os.path.join(pol.MODEL_DIR, a.model, "config.json")):
        return list_models(a.model)

    mdl = pc.Model()
    qm = mdl.to_model(pc.default_q_ctrl())
    for side in pc.SIDES:                                       # 준비 자세 (수집 · 정책 시작 자세)
        qm, _, _ = mdl.ik(qm, side, pc.READY_TCP[side], pc.READY_R[side], iters=600)
    q_ready = mdl.to_ctrl(qm)
    z_low = min(mdl.lowest(mdl.to_model(q_ready), s) for s in pc.SIDES)
    TABLE_OK = (z_low - 0.30, 0.0, 0.0)                         # 준비 자세 집게보다 30 cm 아래 테이블 (정상)
    arm = FakeArm(q_ready)
    grip_log, grip_q = [], {"left": 0.5, "right": 0.5}
    h2 = pol._read(os.path.join(pol.MODEL_DIR, a.model, "h2_model.json")) or {}
    names = h2.get("state_names") or [f"j{i}" for i in range(16)]
    r = types.SimpleNamespace(
        mdl=mdl, cam=types.SimpleNamespace(frame=lambda: (np.full((480, 848, 3), 110, np.uint8), time.time())),
        wrist=types.SimpleNamespace(names=lambda: ["wrist_left", "wrist_right"], frame=lambda n: (np.full((480, 640, 3), 90, np.uint8), time.time())),
        grip=types.SimpleNamespace(state=lambda s: {"q": grip_q[s]}, set=lambda s, q: grip_log.append((s, q)) or True),
        cam_pitch=None, arm_q=q_ready, table_plane=TABLE_OK, rec=types.SimpleNamespace(active=False, pending=None),
        busy=lambda: False, say=lambda m: None, rec_state_names=lambda: names)
    R = pol.PolicyRunner(r, arm)
    vr_real = pol.vr_settings.get
    body_real = R.body_check

    def run(sec, stop=True):
        arm.reset()
        R.start()
        t = time.time()
        while R.running() and time.time() - t < sec:
            time.sleep(0.05)
        if R.running() and stop:
            R.stop()
        R.th.join(30)
        return R.state.get("why_end", "")

    def only(*keep):
        """이번 항목이 시험하는 필터만 켜고 나머지 (범위 · 점프 · 몸통) 는 끔."""
        R.cfg = dict(pol.DEFAULTS)
        if "range" in keep:
            R.policy.a_min, R.policy.a_max = A0
        else:
            R.cfg["range_tol"] = 1e9
            R.policy.a_min, R.policy.a_max = None, None
        if "jump" not in keep:
            R.cfg["jump_rad"] = 1e9
        R.body_check = body_real if "body" in keep else (lambda qm_, cfg_: "")
        pol.vr_settings.get = vr_real
        r.table_plane = TABLE_OK
        arm.meas_off = 0.0

    def ensure_alive():
        if not R.policy.alive():
            R.load(a.model, a.device)

    print(f"[점검] 모델 {a.model} · {a.device}", flush=True)
    # 1 불러오기
    try:
        t0 = time.time()
        info = R.load(a.model, a.device)
        result("1 불러오기", True, f"{info['policy']} · {info['backend']} · 첫 추론 {info['infer_ms']} ms · 준비 {time.time() - t0:.1f} s"
               + (f" · OpenVINO 차이 {info['ov_diff']:.2e}" if info.get("ov_diff") is not None else ""))
    except Exception as e:  # noqa: BLE001
        result("1 불러오기", False, str(e))
        return report()
    bad = R.check()
    result("시작 조건 (가짜 준비 자세 · 테이블)", not bad, "; ".join(bad))
    if bad:
        return report()
    real_act = R.policy.act
    A0 = (R.policy.a_min, R.policy.a_max)
    st16 = R.state16(q_ready, grip_q)

    # 2 정상 실행 (타이밍 · 한계만 — 범위 · 점프 · 몸통은 따로)
    only()
    grip_log.clear()
    why = run(a.sec)
    ts = np.array([c[0] for c in arm.cmds])
    dt = np.diff(ts) if len(ts) > 1 else np.array([0.0])
    qs = np.radians(np.array([c[1] for c in arm.cmds])) if arm.cmds else np.zeros((0, 14))
    qsm = np.array([mdl.to_model(q) for q in qs]) if len(qs) else np.zeros((0, 14))
    result("2 정상 실행 · 지령 주기", len(dt) > 10 and float(dt.max()) < 0.2,
           f"명령 {len(arm.cmds)} · 평균 {dt.mean() * 1000:.1f} ms · 최대 간격 {dt.max() * 1000:.0f} ms · 끝: {why}")
    result("2 정상 실행 · 관절 한계 (URDF)", bool(np.all(qsm >= mdl.lo - 1e-6) and np.all(qsm <= mdl.hi + 1e-6)))
    result("2 정상 실행 · 그리퍼 범위", all(pc.Q_LO - 1e-9 <= q <= pc.Q_HI + 1e-9 for _, q in grip_log), f"그리퍼 명령 {len(grip_log)}")
    result("2 정상 실행 · 끝난 뒤 명령 없음", arm.after_end == 0, f"/stream/end 뒤 명령 {arm.after_end}")
    only("range", "jump", "body")
    why = run(a.sec)
    print(f"  참고  기본 기준 그대로 {a.sec:.0f} 초: 끝 = {why}  (가짜 회색 영상이라 참고만 — 안전 필터로 멈췄으면 실기 영상에서도 확인)", flush=True)

    # 3 NaN
    only()
    R.policy.act = lambda *x, **k: (np.full(16, np.nan), 1.0)
    why = run(3)
    result("3 NaN 출력 → 멈춤", "NaN" in why, why)
    # 4 학습 범위 밖
    only("range")
    if R.policy.a_max is not None:
        far = R.policy.a_max + 1.0
        R.policy.act = lambda *x, **k: (far.copy(), 1.0)
        why = run(3)
        result("4 학습 범위 밖 → 멈춤", "학습 범위" in why, why)
    else:
        result("4 학습 범위 밖 → 멈춤", False, "모델에 action 범위 통계 없음 — 범위 검사가 안 됨")
    # 5 한 틱 큰 점프 (0.6 rad)
    only("jump")
    jump = st16.copy()
    jump[0] += 0.6
    R.policy.act = lambda *x, **k: (jump.copy(), 1.0)
    why = run(3)
    result("5 한 틱 큰 점프 → 멈춤", "점프" in why and "관절 0" in why, why)
    # 6 지령 ↔ 실측
    only()
    R.policy.act = lambda *x, **k: (st16.copy(), 1.0)
    arm.reset()
    R.start()
    arm.meas_off = 30.0
    t = time.time()
    while R.running() and time.time() - t < 3:
        time.sleep(0.05)
    if R.running():
        R.stop()
    R.th.join(30)
    why = R.state.get("why_end", "")
    result("6 지령 ↔ 실측 차이 → 멈춤", "지령 ↔ 실측" in why, why)
    # 7 테이블 아래 — 준비 자세 집게보다 5 cm 높은 테이블
    only()
    r.table_plane = (z_low + 0.05, 0.0, 0.0)
    R.policy.act = lambda *x, **k: (st16.copy(), 1.0)
    why = run(3)
    result("7 테이블 아래 → 멈춤", "테이블" in why, why)
    # 8 몸통 · 양팔 · 뒤 한계 — 기준을 일부러 크게 (준비 자세에서도 걸리게)
    for label, over, key in (("양팔 사이", {"arm_gap_m": 5.0}, "양팔 사이"), ("뒤 한계", {"back_limit_m": 5.0}, "뒤로")):
        only("body")
        pol.vr_settings.get = lambda over=over: {**vr_real(), **over}
        R.policy.act = lambda *x, **k: (st16.copy(), 1.0)
        why = run(3)
        result(f"8 {label} → 멈춤", key in why, why)
    only("body")                                    # 몸통 검사는 몸 앞 영역 (|y| < body_half_y) 의 점만 봄 — 준비 자세는 몸 옆이라
    old = (pc.P["body_clear_m"], pc.P["body_half_y"])  #   영역을 넓히고 기준을 크게 해서 검사가 이어져 있는지만 봄
    pc.P["body_clear_m"], pc.P["body_half_y"] = 5.0, 5.0
    R.policy.act = lambda *x, **k: (st16.copy(), 1.0)
    why = run(3)
    pc.P["body_clear_m"], pc.P["body_half_y"] = old
    result("8 몸통 → 멈춤", "몸통" in why, why)
    # 9 그리퍼 조임 제한 — 손가락이 벌어져 있을 때 (q 3.0) 정책이 완전 닫힘을 내도 지금 q 보다 squeeze 만큼만
    only()
    grip_q.update(left=3.0, right=3.0)
    cl = st16.copy()
    cl[7], cl[15] = pc.Q_LO, pc.Q_LO
    R.policy.act = lambda *x, **k: (cl.copy(), 1.0)
    grip_log.clear()
    run(1.5)
    lo_sent = min(q for _, q in grip_log) if grip_log else None
    want = 3.0 - R.squeeze_q
    grip_q.update(left=0.5, right=0.5)
    result("9 그리퍼 조임 제한", lo_sent is not None and abs(lo_sent - want) < 1e-6 and want > pc.Q_LO,
           f"보낸 최소 q {lo_sent if lo_sent is None else round(lo_sent, 3)} · 기대 {want:.3f} (지금 3.0 − squeeze {R.squeeze_q:.3f}, 정책은 {pc.Q_LO})")
    # 10 [정지] — 진짜 추론으로 1.5 초 돌다가 (느린 정책이면 추론 중에) 정지
    only()
    R.policy.act = real_act
    arm.reset()
    R.start()
    time.sleep(1.5)
    R.stop()
    R.th.join(30)
    n_at_end = len(arm.cmds)
    time.sleep(0.5)
    ok = R.state.get("why_end", "").startswith("정지") and arm.after_end == 0 and len(arm.cmds) == n_at_end and arm.ended_at is not None
    result("10 [정지] → 멈춤 · 끝난 뒤 명령 없음", ok, f"끝: {R.state.get('why_end')} · 끝난 뒤 명령 {arm.after_end}")
    alive = R.policy.alive()
    t = time.time()
    why = ""
    while time.time() - t < 90:                     # 느린 정책: 앞 추론이 끝나야 다음 시작이 됨
        try:
            why = run(1.0)
            if not why.startswith("오류"):
                break
        except RuntimeError as e:
            why = str(e)
        time.sleep(2)
    result("10 [정지] 뒤 프로세스 살아 있고 다음 시작 됨", alive and why.startswith("정지"), f"살아 있음 {alive} · 다음 회 끝: {why}")
    ensure_alive()
    # 11 시작 조건
    only("range", "jump", "body")
    r.arm_q = pc.default_q_ctrl()
    bad = R.check()
    result("11 준비 자세 아님 → 시작 거부", any("준비 자세" in b for b in bad), "; ".join(bad)[:120])
    r.arm_q = q_ready
    r.table_plane = None
    bad = R.check()
    try:
        R.start()
        started = True
        R.stop()
        R.th.join(30)
    except RuntimeError:
        started = False
    result("11 테이블 안 잼 → 시작 거부", (not started) and any("테이블" in b for b in bad), "; ".join(bad)[:120])
    r.table_plane = TABLE_OK
    # 12 추론 무응답 — 답을 안 주는 추론 흉내
    only()
    R.policy.act = lambda *x, **k: (time.sleep(30), None)[1]
    R.cfg["stall_s"] = 2.0
    why = run(10, stop=False)
    result("12 추론 무응답 → 멈춤 · 프로세스 끔", "못 받음" in why and not R.policy.alive() and bool(R.check()),
           f"{why} · 다시 시작 조건: {R.check()[:1]}")
    R.policy.close()
    return report()


def list_models(asked):
    """모델 이름이 없거나 틀렸을 때: lerobot 환경 · 모델 목록."""
    py = pol.lerobot_env.python()
    print(f"lerobot 환경 (추론): {py} — {'있음' if os.path.isfile(py) else '없음 → 먼저 ./apps/trainer/install_lerobot.sh robot'}")
    if asked:
        print(f"❌ 모델 없음: {asked} (data/policies/{asked}/config.json)")
    ms = pol.models()
    old = [n for n in (os.listdir(pol.MODEL_DIR) if os.path.isdir(pol.MODEL_DIR) else [])
           if os.path.isfile(os.path.join(pol.MODEL_DIR, n, "meta.json")) and not os.path.isfile(os.path.join(pol.MODEL_DIR, n, "config.json"))]
    print(f"\n점검할 수 있는 lerobot 모델 ({pol.MODEL_DIR}): {len(ms)} 개")
    for m in ms:
        print(f"  {m['name']:<40} {m['policy']:<10} {m.get('dataset') or '-'}  step {m.get('step') or '-'}")
    if old:
        print(f"\n예전 형식 모델 (OpenVINO zip, 이제 안 씀 — 점검 대상 아님): {', '.join(sorted(old))}")
    if ms:
        print(f"\n실행 예:  python tools/check_policy_safety.py {ms[0]['name']} --device GPU")
    else:
        print("\nlerobot 모델이 없음 — 학습 (홈 → 학습) 으로 먼저 만들거나 학습 웹에서 받은 model zip 을 수집 화면 [zip 올리기] 로")
    return 1


def report():
    n_bad = sum(not ok for _, ok, _ in RES)
    print("\n" + ("✓ 전부 PASS — 다음은 로봇을 매단 상태에서 저속 실기 점검" if n_bad == 0 else f"❌ FAIL {n_bad} 개 — 실기 금지"), flush=True)
    out = os.path.join(ROOT, "data", "logs", f"policy_safety_{time.strftime('%Y%m%d_%H%M%S')}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump([{"name": n, "ok": ok, "detail": d} for n, ok, d in RES], f, ensure_ascii=False, indent=1)
    print(f"기록: {out}")
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
