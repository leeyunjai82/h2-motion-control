#!/usr/bin/env python3
# Version: 1.0
# Changes:
#   1.0 - leeyunjai82/h2-motion-control.red 브랜치 claude/camera-depth-check-w8pk3s (c5e8f2a) 에서 옮김 (사용자 2026-10-16).
#         이 저장소 기준 안내 추가 (머리 카메라 기본 경로 = 공식 인터페이스), 끝 메시지만 바꿈 — 동작은 같음
"""
probe_h2_camera.py — H2 카메라 장치 식별 (카메라가 USB 로 직접 보이는 보드에서 실행)

  이 저장소의 머리 쌍안 카메라 기본 경로는 공식 'Bilateral Data Stream Acquisition Interface'
  (PC1 → RTP/H264 RGB + TCP 5000 Y16 깊이): common/ctrl/head_cam.py · utils/check_head_cam.py.
  이 스크립트는 그 경로를 쓰지 않고, 카메라가 USB 로 붙은 보드(PC2 등)에서 장치를 직접 확인할 때 씀.

H2 헤드 카메라는 RealSense 가 아니라 광각 쌍안(스테레오) 카메라이므로
pyrealsense2 로는 보이지 않을 수 있습니다. 이 스크립트는
  1) /dev/video* 장치 이름 / USB VID:PID / 지원 포맷·해상도 나열
  2) RealSense 장치가 있는지 별도 확인 (pyrealsense2 설치 시)
  3) 각 video 장치에서 1프레임씩 캡처해 PNG 저장
     → 좌/우 이미지가 한 프레임에 나란히(side-by-side) 오는지 눈으로 확인
를 수행합니다.

사용 예)
  python utils/probe_h2_camera.py                 # 나열 + 스냅샷(./cam_probe)
  python utils/probe_h2_camera.py --out /tmp/cam  # 저장 경로 지정
  python utils/probe_h2_camera.py --no-capture    # 나열만
"""
import argparse
import glob
import os
import shutil
import subprocess

import cv2


def read_sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_info(video_node):
    """/sys/class/video4linux/videoN/device → 상위 USB 장치의 idVendor/idProduct/product 탐색"""
    dev = os.path.realpath(f"/sys/class/video4linux/{video_node}/device")
    cur = dev
    for _ in range(4):
        vid = read_sysfs(os.path.join(cur, "idVendor"))
        if vid:
            return {
                "vid": vid,
                "pid": read_sysfs(os.path.join(cur, "idProduct")),
                "product": read_sysfs(os.path.join(cur, "product")),
                "manufacturer": read_sysfs(os.path.join(cur, "manufacturer")),
                "speed_mbps": read_sysfs(os.path.join(cur, "speed")),
                "sysfs": cur,
            }
        cur = os.path.dirname(cur)
    return {"sysfs": dev}


def v4l2_formats(dev):
    if not shutil.which("v4l2-ctl"):
        return None
    try:
        out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                             capture_output=True, text=True, timeout=5).stdout
        return out.strip()
    except Exception as e:
        return f"(v4l2-ctl 실패: {e})"


def list_video_devices():
    nodes = sorted(glob.glob("/sys/class/video4linux/video*"),
                   key=lambda p: int(p.rsplit("video", 1)[1]))
    devices = []
    print("=== /dev/video* ===")
    if not nodes:
        print("[ERR] video 장치 없음 (이 보드에 카메라가 연결되어 있지 않을 수 있음 → PC2 에서 실행 확인)")
        return devices
    for n in nodes:
        node = os.path.basename(n)
        dev = f"/dev/{node}"
        name = read_sysfs(os.path.join(n, "name"))
        u = usb_info(node)
        print(f"\n[{dev}] name={name}")
        if "vid" in u:
            print(f"  usb {u['vid']}:{u['pid']}  product={u['product']}  "
                  f"manufacturer={u['manufacturer']}  speed={u['speed_mbps']} Mbps")
            if u["speed_mbps"] == "480":
                print("  [WARN] USB 2.0 속도로 연결됨")
        fm = v4l2_formats(dev)
        if fm is None:
            print("  (v4l2-ctl 없음: sudo apt install v4l-utils 하면 포맷 목록 확인 가능)")
        elif fm:
            for line in fm.splitlines():
                print("  " + line)
        devices.append(dev)
    return devices


def check_realsense():
    print("\n=== RealSense ===")
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("pyrealsense2 미설치 → 건너뜀")
        return
    devs = rs.context().query_devices()
    if len(devs) == 0:
        print("RealSense 장치 없음")
    for d in devs:
        print(f"{d.get_info(rs.camera_info.name)}  serial={d.get_info(rs.camera_info.serial_number)}")


def capture(devices, out_dir):
    print(f"\n=== 스냅샷 → {out_dir} ===")
    os.makedirs(out_dir, exist_ok=True)
    for dev in devices:
        idx = int(dev.rsplit("video", 1)[1])
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"[{dev}] open 실패 (metadata 노드이거나 다른 프로세스가 점유 중)")
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        ok, frame = False, None
        for _ in range(10):  # 노출 안정화
            ok, frame = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if not ok or frame is None:
            print(f"[{dev}] 프레임 수신 실패 ({w}x{h})")
            continue
        path = os.path.join(out_dir, f"video{idx}_{frame.shape[1]}x{frame.shape[0]}.png")
        cv2.imwrite(path, frame)
        hint = "  ← 가로:세로 ≥ 2:1, side-by-side 스테레오 가능성" if frame.shape[1] >= 2 * frame.shape[0] else ""
        print(f"[{dev}] {frame.shape[1]}x{frame.shape[0]} → {path}{hint}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="cam_probe")
    ap.add_argument("--no-capture", action="store_true")
    args = ap.parse_args()

    devices = list_video_devices()
    check_realsense()
    if devices and not args.no_capture:
        capture(devices, args.out)
    print("\n[DONE] 장치 목록" + ("" if args.no_capture else f" + 스냅샷({args.out})") + " 확인 끝")


if __name__ == "__main__":
    main()
