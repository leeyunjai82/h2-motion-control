#!/usr/bin/env python3
"""
robot_services.py — 로봇 서비스 목록 보기 / 켜기·끄기 (앱의 서비스 on/off 와 같은 기능을 PC 에서)

  RPC 서비스 'robot_state' (API 1003 ServiceList · 1001 ServiceSwitch) — SDK 814556d 의 go2 RobotStateClient 를 그대로 씀.
  H2 실기 응답 확인 (2026-10-09): API 1.0.0.2, 서비스 27 개 (video_hub, stereo_patch_pc1 포함).
  status: 0 = 켜짐, 1 = 꺼짐 (실기 2026-10-09 확정 — head_cam_on 이 video_hub 끄고 stereo_patch_pc1 켠 뒤 1 / 0).
  켜기는 서비스가 뜰 때까지 응답이 늦어 3 초를 넘김 (실측 code 3104 = 응답 시간 초과) → 기본 대기 10 초,
  바꾼 뒤 목록을 다시 읽어 원하는 상태가 됐는지로 성공 판정.
  머리 카메라 준비는 tools/head_cam_on.py (start_robot.sh 가 실행).

  python tools/robot_services.py                         # 목록 (읽기 전용, 명령 없음)
  python tools/robot_services.py --off video_hub         # 끄기 (yes 확인)
  python tools/robot_services.py --on <서비스 이름>       # 켜기 (이름은 목록에 나온 그대로)
  python tools/robot_services.py --off video_hub --on <이름> --yes    # 확인 없이 (스크립트용)
  python tools/robot_services.py --iface <인터페이스>     # DDS 네트워크 인터페이스 지정 (기본 자동)

  protect True 인 서비스는 바꾸지 않음 (로봇도 거부, 코드 5202).
  보행·균형 관련 서비스(ai_sport, state_estimator, motion_switcher 등)를 끄면 로봇이 넘어질 수 있음 → 이름을 정확히 확인하고 쓸 것.
"""
import argparse
import os
import sys

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.core import robot_env   # noqa: E402  H2 SDK 경로(third_party) 를 먼저 쓰게 함

from unitree_sdk2py.go2.robot_state.robot_state_api import (   # noqa: E402
    ROBOT_STATE_ERR_SERVICE_PROTECTED, ROBOT_STATE_ERR_SERVICE_SWITCH)
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient   # noqa: E402
from unitree_sdk2py.rpc.internal import RPC_ERR_CLIENT_API_TIMEOUT   # noqa: E402

STATUS_ON, STATUS_OFF = 0, 1                 # 실기 2026-10-09 확정


def status_text(st):
    return {STATUS_ON: "켜짐", STATUS_OFF: "꺼짐"}.get(st, f"? ({st})")


def service_list(rsc):
    code, lst = rsc.ServiceList()
    if code != 0 or lst is None:
        print(f"[services] ❌ ServiceList 실패 (code {code})")
        return None
    return {s.name: s for s in lst}


def show(svcs):
    w = max([len(n) for n in svcs] + [4])
    print(f"  {'name':{w}s}  status  상태  protect")
    for n in sorted(svcs):
        s = svcs[n]
        print(f"  {n:{w}s}  {s.status!s:6s}  {status_text(s.status):4s}  {s.protect}")


def main():
    ap = argparse.ArgumentParser(description="로봇 서비스 목록 / 켜기·끄기 (robot_state RPC)")
    ap.add_argument("--on", action="append", default=[], metavar="NAME", help="켤 서비스 (여러 번 가능)")
    ap.add_argument("--off", action="append", default=[], metavar="NAME", help="끌 서비스 (여러 번 가능)")
    ap.add_argument("--yes", action="store_true", help="확인 묻지 않음")
    ap.add_argument("--iface", default=None, help="DDS 네트워크 인터페이스 (예: enp3s0)")
    ap.add_argument("--timeout", type=float, default=10.0, help="RPC 응답 대기 [s] (켜기는 3 초를 넘김)")
    a = ap.parse_args()

    robot_env.dds_init(a.iface)
    rsc = RobotStateClient()
    rsc.SetTimeout(a.timeout)
    rsc.Init()

    code, ver = rsc.GetServerApiVersion()
    if code != 0:
        print(f"[services] ❌ robot_state 서비스 응답 없음 (code {code}) — 네트워크/인터페이스 또는 로봇 전원 확인")
        print("           확인: ping 192.168.123.161 · python tools/check_robot_id.py (rt/lowstate 수신)")
        sys.exit(1)
    print(f"[services] robot_state API {ver}")

    svcs = service_list(rsc)
    if svcs is None:
        sys.exit(1)
    print(f"[services] {len(svcs)} 개")
    show(svcs)

    todo = [(n, False) for n in a.off] + [(n, True) for n in a.on]
    if not todo:
        return

    for n, _ in todo:
        if n not in svcs:
            print(f"[services] ❌ '{n}' 없음 — 위 목록의 이름을 그대로 쓸 것")
            sys.exit(2)
        if svcs[n].protect:
            print(f"[services] ❌ '{n}' 은 protect — 바꾸지 않음")
            sys.exit(2)
    plan = ", ".join(f"{n} {'켜기' if on else '끄기'}" for n, on in todo)
    if not a.yes:
        if input(f"[services] {plan} — 진행하려면 yes: ").strip().lower() != "yes":
            print("[services] 취소")
            return

    for n, on in todo:
        r = rsc.ServiceSwitch(n, on)
        msg = {0: "OK", ROBOT_STATE_ERR_SERVICE_PROTECTED: "protect 로 거부",
               ROBOT_STATE_ERR_SERVICE_SWITCH: "전환 실패",
               RPC_ERR_CLIENT_API_TIMEOUT: "응답 시간 초과 — 아래 목록으로 확인"}.get(r, f"code {r}")
        print(f"[services] {n} {'켜기' if on else '끄기'} → {msg}")

    # 성공 판정 = 바꾼 뒤 상태 (켜기는 응답이 늦어 시간 초과가 나도 실제로는 켜져 있을 수 있음)
    svcs = service_list(rsc)
    if svcs is None:
        sys.exit(1)
    print("[services] 바꾼 뒤")
    show({n: svcs[n] for n, _ in todo if n in svcs})
    bad = [n for n, on in todo if svcs[n].status != (STATUS_ON if on else STATUS_OFF)]
    if bad:
        print(f"[services] ❌ 원하는 상태가 아님: {', '.join(bad)}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
