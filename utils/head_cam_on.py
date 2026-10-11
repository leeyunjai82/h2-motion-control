#!/usr/bin/env python3
"""
head_cam_on.py — 머리 카메라 RGB 를 이 PC 가 받도록 로봇 쪽 서비스를 준비 (앱에서 하던 것을 PC 에서)

  로봇을 다시 켜면 앱 설정이 기본(video_hub 켜짐 · Stereo patch PC1 꺼짐)으로 돌아감 (실기 2026-10-09) → 켤 때마다 필요.
  start_robot.sh 가 head_track 과 같이 실행 (로그 logs/head_track_<날짜>.log). 혼자 실행해도 됨.

  1) video_hub 끄기 · stereo_patch_pc1 켜기       robot_state RPC (앱의 서비스 on/off 와 같음, utils/robot_services.py 와 같은 API)
                                                   이미 그 상태면 건너뜀 (status 0 = 켜짐, 1 = 꺼짐 — 실기 확정)
  2) PC1 9080 이 열리면 RGB 수신 IP = 이 PC        http://192.168.123.161:9080/set?ip=<이 PC>  (응답 'IP 已更新为 …')
  3) stereo_patch_pc1 껐다 켜기                    수신 IP 적용 (공식 문서 절차)
  4) 끌 서비스(video_hub)가 다시 켜졌는지 확인      켜져 있으면 한 번 더 끔 (공식 문서: video_hub 와 충돌 가능, 둘 다 PC1 에서 돎)
  → head_track 이 RGB 를 받기 시작 (6 초마다 다시 시도하므로 head_track 재시작 불필요)

  실기 2026-10-11: 영상은 나오는데 앱 서비스 목록에서 video_hub 가 켜져 있었음 (사용자) → 4) 추가.
  나중에 다시 켜지는 원인(앱 카메라 화면 등)은 확인 필요 — 지금 상태: python utils/robot_services.py

  실기 2026-10-09: 1)–3) 후 Head Vision 영상 나옴. 9080 은 stereo_patch_pc1 이 켜져 있을 때 열림 (끄면 닫힘).
  켜기 RPC 는 서비스가 뜰 때까지 응답이 늦어 3 초를 넘김 (code 3104 = 응답 시간 초과) → 대기 10 초,
  그래도 시간 초과면 실패로 보지 않고 9080 이 열리는지로 판정.

  서비스 이름·켜기 여부: robot.yaml head_track.robot_setup. 거기 적힌 서비스 말고는 건드리지 않음.

  python utils/head_cam_on.py
  python utils/head_cam_on.py --no-ip              # 1) 만 (수신 IP 가 이미 이 PC 면 재시작 생략)
  python utils/head_cam_on.py --iface <인터페이스>  # DDS 네트워크 인터페이스 (기본 자동)
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # noqa: E402  H2 SDK 경로(third_party) 를 먼저 쓰게 함

from ctrl.head_cam import PC1, RGB_PORTS, SET_IP_PORT, my_ip, port_open, set_receive_ip   # noqa: E402

STATUS_ON, STATUS_OFF = 0, 1                 # robot_state ServiceList status (실기 2026-10-09 확정)
RPC_TIMEOUT_S = 10.0                         # 켜기 응답이 3 초를 넘김 (실측)


def log(msg):
    print(f"[head_cam_on] {msg}", flush=True)


def status_text(st):
    return {STATUS_ON: "켜짐", STATUS_OFF: "꺼짐"}.get(st, f"? ({st})")


def wait_port(want_open, sec):
    """PC1:9080 이 want_open 상태가 될 때까지 최대 sec 초."""
    t_end = time.time() + sec
    while time.time() < t_end:
        if port_open(PC1, SET_IP_PORT, 1.0) == want_open:
            return True
        time.sleep(0.5)
    return False


def switch(rsc, name, on):
    """서비스 켜기/끄기 → True(OK) / None(응답 시간 초과 — 결과는 포트·목록으로 확인) / False(실패)."""
    from unitree_sdk2py.rpc.internal import RPC_ERR_CLIENT_API_TIMEOUT
    r = rsc.ServiceSwitch(name, on)
    what = "켜기" if on else "끄기"
    if r == 0:
        log(f"{name} {what} -> OK")
        return True
    if r == RPC_ERR_CLIENT_API_TIMEOUT:
        log(f"{name} {what} -> 응답 시간 초과 ({RPC_TIMEOUT_S:.0f} s) — 실제 상태로 확인")
        return None
    log(f"{name} {what} -> 실패 code {r}")
    return False


def recheck_off(rsc, off):
    """마지막에 끌 서비스가 꺼져 있는지 다시 확인 — 켜져 있으면 한 번 더 끄고 결과를 남김."""
    time.sleep(2.0)
    code, lst = rsc.ServiceList()
    if code != 0 or lst is None:
        log(f"⚠️ 마지막 확인: 서비스 목록 못 받음 (code {code})")
        return
    st = {s.name: s.status for s in lst}
    for n in off:
        if st.get(n) == STATUS_OFF:
            log(f"확인: {n} 꺼짐")
            continue
        log(f"⚠️ {n} 가 다시 켜져 있음 ({status_text(st.get(n))}) — 한 번 더 끔")
        switch(rsc, n, False)
        time.sleep(2.0)
        code, lst = rsc.ServiceList()
        now = {s.name: s.status for s in (lst or [])}.get(n)
        if now == STATUS_OFF:
            log(f"확인: {n} 꺼짐")
        else:
            log(f"⚠️ {n} 여전히 {status_text(now)} — 로봇 쪽에서 다시 켜는 것으로 보임. 영상이 끊기면 앱에서 끄고 head_cam_on 다시 실행")


def main():
    ap = argparse.ArgumentParser(description="머리 카메라용 로봇 서비스 준비 (video_hub 끔, stereo_patch_pc1 켬, 수신 IP)")
    ap.add_argument("--no-ip", action="store_true", help="수신 IP 설정·서비스 재시작 생략")
    ap.add_argument("--iface", default=None, help="DDS 네트워크 인터페이스")
    ap.add_argument("--wait", type=float, default=30.0, help="서비스가 뜨길 기다리는 최대 시간 [s]")
    a = ap.parse_args()

    cfg = (robot_env.CFG.get("head_track") or {}).get("robot_setup") or {}
    if not cfg.get("enabled", False):
        log("robot.yaml head_track.robot_setup.enabled 가 아님 -> 건너뜀 (앱에서 직접: video_hub 끔 · Stereo patch PC1 켬)")
        return 0
    if robot_env.SIM:
        log("시뮬 (ROBOT_SIM) -> 건너뜀")
        return 0
    off = [str(n) for n in (cfg.get("stop") or [])]
    on = str(cfg.get("start") or "")
    if not on:
        log("❌ robot.yaml head_track.robot_setup.start (켤 서비스) 없음")
        return 1

    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    robot_env.dds_init(a.iface)
    rsc = RobotStateClient()
    rsc.SetTimeout(RPC_TIMEOUT_S)
    rsc.Init()

    code, lst = rsc.ServiceList()
    if code != 0 or lst is None:
        log(f"❌ 로봇 서비스 목록 못 받음 (code {code}) — 네트워크 확인, 또는 앱에서 직접: video_hub 끔 · Stereo patch PC1 켬")
        return 1
    svcs = {s.name: s for s in lst}
    for n in off + [on]:
        if n not in svcs:
            log(f"❌ 로봇에 '{n}' 서비스 없음 — robot.yaml head_track.robot_setup 확인 (목록: python utils/robot_services.py)")
            return 1
        if svcs[n].protect:
            log(f"❌ '{n}' 은 protect — 바꾸지 않음")
            return 1
    log("지금: " + ", ".join(f"{n} {status_text(svcs[n].status)}" for n in off + [on]))

    # 1) 끌 것 끄고 켤 것 켜기 — 이미 그 상태면 건너뜀
    for n in off:
        if svcs[n].status == STATUS_OFF:
            log(f"{n} 이미 꺼짐")
        elif switch(rsc, n, False) is False:
            return 1
    if svcs[on].status == STATUS_ON:
        log(f"{on} 이미 켜짐")
    elif switch(rsc, on, True) is False:
        return 1

    if not wait_port(True, a.wait):
        log(f"❌ {PC1}:{SET_IP_PORT} 가 {a.wait:.0f} 초 안에 안 열림 — {on} 이 켜졌는지 앱에서 확인")
        return 1
    log(f"{PC1}:{SET_IP_PORT} 열림")

    me = my_ip(PC1)
    if a.no_ip or not cfg.get("set_ip", True):
        log(f"수신 IP 설정 생략 — 로봇에 설정된 수신 IP 가 이 PC({me})여야 RGB 가 옴")
        recheck_off(rsc, off)
        return 0

    # 2) 수신 IP = 이 PC
    good, msg = set_receive_ip(me, PC1)
    log(msg)
    if not good:
        return 1

    # 3) 수신 IP 적용 — 서비스 껐다 켜기
    if switch(rsc, on, False) is False:
        return 1
    if not wait_port(False, 10.0):
        log(f"⚠️ {PC1}:{SET_IP_PORT} 가 10 초 안에 안 닫힘 — 그대로 다시 켬")
    time.sleep(1.0)
    if switch(rsc, on, True) is False:
        return 1
    if not wait_port(True, a.wait):
        log(f"❌ 재시작 뒤 {PC1}:{SET_IP_PORT} 가 {a.wait:.0f} 초 안에 안 열림 — 앱에서 Stereo patch PC1 확인")
        return 1

    recheck_off(rsc, off)
    ports = "/".join(str(RGB_PORTS[k]) for k in ("left", "right"))
    log(f"✅ 준비 끝 — 수신 IP {me}, RGB UDP {ports} -> head_track (Head Vision 카드)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
