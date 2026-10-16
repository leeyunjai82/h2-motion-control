#!/usr/bin/env python3
# Version: 1.0
# Changes:
#   1.0 - leeyunjai82/h2-motion-control.red 브랜치 claude/camera-depth-check-w8pk3s (417d8a2 · c5e8f2a) 에서 옮김 (사용자 2026-10-16).
#         이 저장소에 맞춤: 기본 해상도 = robots/h2/robot.yaml camera.resolution (없으면 848×480),
#         웹 포트 50001 → 50015 (50001 은 rs_stream), 카메라를 다른 프로그램이 쓰고 있으면 안내하고 끝냄
"""
check_depth.py — 등판 거치대 D435i(RealSense) 깊이 확인 (단독 실행)

  start_robot.sh / rs_stream.py 등 카메라를 쓰는 프로그램은 먼저 끌 것 (D435i 동시 사용 불가).
  머리 쌍안 카메라는 RealSense 가 아님 → utils/check_head_cam.py (공식 인터페이스, 깊이 포함)

1) 연결된 RealSense 장치 / USB 타입 / depth scale 출력
2) color + depth 스트림 시작 (depth 는 color 에 align)
3) 화면 중앙 ROI 의 거리(m)와 유효 픽셀 비율을 1초마다 출력
4) --web 사용 시 MJPEG 서버로 브라우저에서 확인
     http://<로봇PC IP>:50015/            (color | depth 나란히)
     http://<로봇PC IP>:50015/video_feed  (color)
     http://<로봇PC IP>:50015/depth_feed  (depth colormap)
   --save 사용 시 color/depth PNG + raw depth(.npy) 저장 후 종료

사용 예)
  python utils/check_depth.py              # 터미널 출력만
  python utils/check_depth.py --web        # 브라우저 확인
  python utils/check_depth.py --save out   # 스냅샷 저장
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("pyrealsense2 가 설치되어 있지 않습니다: pip install pyrealsense2")
    sys.exit(1)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_PORT = 50015            # 50001 = rs_stream, 50014 = check_head_cam --web


def default_resolution():
    """robots/h2/robot.yaml camera.resolution [W, H] (rs_stream·detect_box 와 같은 값). 못 읽으면 848×480."""
    try:
        import yaml
        with open(os.path.join(ROOT, "robots", "h2", "robot.yaml"), encoding="utf-8") as f:
            w, h = yaml.safe_load(f)["camera"]["resolution"]
        return int(w), int(h)
    except Exception:   # noqa: BLE001
        return 848, 480


def list_devices():
    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        print("[ERR] RealSense 장치를 찾지 못했습니다. (USB 3.0 포트 / 케이블 / 다른 프로세스 점유 확인)")
        sys.exit(1)
    for d in devs:
        info = lambda k: d.get_info(k) if d.supports(k) else "-"
        print(f"[DEV] {info(rs.camera_info.name)}  "
              f"serial={info(rs.camera_info.serial_number)}  "
              f"fw={info(rs.camera_info.firmware_version)}  "
              f"usb={info(rs.camera_info.usb_type_descriptor)}")
        usb = info(rs.camera_info.usb_type_descriptor)
        if usb.startswith("2"):
            print("[WARN] USB 2.x 로 연결됨 → 해상도/fps 제한. USB 3 포트로 옮기거나 --width 424 --height 240 --fps 15 로 시도")
    return devs


class DepthCam:
    def __init__(self, width, height, fps, serial=None):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if serial:
            cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            self.profile = self.pipeline.start(cfg)
        except RuntimeError as e:
            print(f"[ERR] 스트림 시작 실패: {e}")
            print("      rs_stream / start_robot.sh 가 카메라를 쓰고 있으면 먼저 끄세요 (D435i 동시 사용 불가).")
            print("      해상도·fps 조합이 안 맞으면 --width/--height/--fps 로 바꿔 보세요.")
            sys.exit(1)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()  # z16 1 unit → m
        self.align = rs.align(rs.stream.color)

        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        print(f"[CFG] {width}x{height}@{fps}  depth_scale={self.depth_scale:.6f} m/unit")
        print(f"[CFG] color intrinsics fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.ppx:.1f} cy={intr.ppy:.1f}")

        self.lock = threading.Lock()
        self.color = None
        self.depth = None  # uint16 raw
        self.running = True

        # 자동 노출 안정화용 프레임 버리기
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def loop(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(5000)
            except RuntimeError as e:
                print(f"[ERR] 프레임 수신 실패: {e}")
                continue
            frames = self.align.process(frames)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue
            with self.lock:
                self.depth = np.asanyarray(d.get_data()).copy()
                self.color = np.asanyarray(c.get_data()).copy()

    def snapshot(self):
        with self.lock:
            if self.color is None:
                return None, None
            return self.color.copy(), self.depth.copy()

    def stop(self):
        self.running = False
        self.pipeline.stop()


def center_stats(depth_raw, scale, roi=20):
    h, w = depth_raw.shape
    patch = depth_raw[h // 2 - roi:h // 2 + roi, w // 2 - roi:w // 2 + roi]
    valid = patch[patch > 0]
    ratio = valid.size / patch.size
    if valid.size == 0:
        return None, ratio
    return float(np.median(valid)) * scale, ratio


def colorize(depth_raw, scale, max_m=4.0):
    d_m = depth_raw.astype(np.float32) * scale
    d8 = np.clip(d_m / max_m * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(255 - d8, cv2.COLORMAP_JET)
    vis[depth_raw == 0] = 0  # 무효 픽셀은 검정
    return vis


def annotate(img, depth_raw, scale, roi=20):
    h, w = img.shape[:2]
    dist, ratio = center_stats(depth_raw, scale, roi)
    cv2.rectangle(img, (w // 2 - roi, h // 2 - roi), (w // 2 + roi, h // 2 + roi), (255, 255, 255), 2)
    txt = f"{dist:.3f} m  valid {ratio * 100:.0f}%" if dist else f"no depth  valid {ratio * 100:.0f}%"
    cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


def run_web(cam, port, max_m):
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn

    app = FastAPI()

    def mjpeg(kind):
        while True:
            color, depth = cam.snapshot()
            if color is None:
                time.sleep(0.05)
                continue
            if kind == "color":
                img = annotate(color, depth, cam.depth_scale)
            else:
                img = annotate(colorize(depth, cam.depth_scale, max_m), depth, cam.depth_scale)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
            time.sleep(1 / 15)

    @app.get("/")
    def index():
        return HTMLResponse(
            "<html><body style='background:#111;color:#eee;font-family:sans-serif'>"
            "<h3>H2 RealSense depth check</h3>"
            "<img src='/video_feed' style='max-width:49%'> "
            "<img src='/depth_feed' style='max-width:49%'></body></html>")

    @app.get("/video_feed")
    def video_feed():
        return StreamingResponse(mjpeg("color"), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/depth_feed")
    def depth_feed():
        return StreamingResponse(mjpeg("depth"), media_type="multipart/x-mixed-replace; boundary=frame")

    print(f"[WEB] http://0.0.0.0:{port}/")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def main():
    dw, dh = default_resolution()
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=dw, help=f"기본 {dw} (robot.yaml camera.resolution)")
    ap.add_argument("--height", type=int, default=dh, help=f"기본 {dh} (robot.yaml camera.resolution)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--serial", default=None, help="장치가 여러 대일 때 시리얼 지정")
    ap.add_argument("--max-m", type=float, default=4.0, help="colormap 최대 거리(m)")
    ap.add_argument("--web", action="store_true", help="MJPEG 웹 서버 실행")
    ap.add_argument("--port", type=int, default=WEB_PORT)
    ap.add_argument("--save", default=None, help="스냅샷 저장 디렉터리 (저장 후 종료)")
    args = ap.parse_args()

    list_devices()
    cam = DepthCam(args.width, args.height, args.fps, args.serial)
    threading.Thread(target=cam.loop, daemon=True).start()

    try:
        while cam.snapshot()[0] is None:
            time.sleep(0.05)

        if args.save:
            os.makedirs(args.save, exist_ok=True)
            color, depth = cam.snapshot()
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(os.path.join(args.save, f"color_{ts}.png"), color)
            cv2.imwrite(os.path.join(args.save, f"depth_{ts}.png"), colorize(depth, cam.depth_scale, args.max_m))
            np.save(os.path.join(args.save, f"depth_raw_{ts}.npy"), depth)
            dist, ratio = center_stats(depth, cam.depth_scale)
            print(f"[SAVE] {args.save}/  center={dist} m  valid={ratio * 100:.0f}%")
            return

        if args.web:
            threading.Thread(target=run_web, args=(cam, args.port, args.max_m), daemon=True).start()

        print("=== 중앙 거리 출력 (Ctrl+C 종료) ===")
        while True:
            _, depth = cam.snapshot()
            dist, ratio = center_stats(depth, cam.depth_scale)
            d_all = depth[depth > 0].astype(np.float32) * cam.depth_scale
            rng = f"{d_all.min():.2f}~{d_all.max():.2f} m" if d_all.size else "-"
            print(f"center={'%.3f m' % dist if dist else 'N/A':>9}  "
                  f"center_valid={ratio * 100:3.0f}%  "
                  f"frame_valid={d_all.size / depth.size * 100:3.0f}%  range={rng}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
