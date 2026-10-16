#!/usr/bin/env python3
# Version: 1.0
# Changes:
#   1.0 - leeyunjai82/h2-motion-control.red 브랜치 claude/camera-depth-check-w8pk3s (c3ef9e3) 에서 옮김 (사용자 2026-10-16).
#         이 저장소에 맞춤: 웹 포트 50001 → 50016 (50001 은 rs_stream), 기본 경로 안내 추가 — 동작은 같음
"""
check_depth_teleimager.py — H2 헤드 카메라 depth 확인 (teleimager 서버 경유)

  이 저장소의 머리 쌍안 카메라 기본 경로는 공식 'Bilateral Data Stream Acquisition Interface'
  (PC1 → RTP/H264 RGB + TCP 5000 Y16 깊이, 로봇이 계산한 깊이): common/ctrl/head_cam.py · utils/check_head_cam.py.
  이 스크립트는 teleimager(xr_teleoperate) 경로로 받아 깊이를 PC 에서 직접 계산해 보는 비교용.

H2 카메라 영상은 PC2 의 teleimager-server 가 ZMQ 로 publish 합니다.
  - REQ  tcp://<host>:60000  "GET_DATA" → 카메라 설정(JSON)
  - SUB  tcp://<host>:<zmq_port>        → JPEG bytes (head_camera 기본 55555)

teleimager 는 depth 를 네트워크로 보내지 않습니다
(RealSense 의 enable_depth 도 서버 내부에서만 읽고 publish 하지 않음).
따라서 binocular(좌/우 side-by-side) 프레임을 받아 OpenCV StereoSGBM 으로
disparity 를 직접 계산합니다.
  - --fx, --baseline 을 주면 depth(m) = fx * baseline / disparity 로 거리 출력
  - 안 주면 disparity(px) 만 출력 (H2 카메라 fx/baseline 은 확인 필요)
  - 140° 광각이라 rectify(캘리브레이션) 없이 계산한 값은 참고용입니다.

필요 패키지: pip install pyzmq opencv-python numpy

사용 예)
  python utils/check_depth_teleimager.py --host 192.168.123.164
  python utils/check_depth_teleimager.py --host 192.168.123.164 --web
  python utils/check_depth_teleimager.py --host 192.168.123.164 --save out
  python utils/check_depth_teleimager.py --host 192.168.123.164 --fx 400 --baseline 0.06
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

try:
    import zmq
except ImportError:
    print("pyzmq 가 필요합니다: pip install pyzmq")
    sys.exit(1)


def request_config(host, port, timeout_ms=2000):
    ctx = zmq.Context.instance()
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, 0)
    s.connect(f"tcp://{host}:{port}")
    try:
        s.send(b"GET_DATA")
        if s.poll(timeout_ms) & zmq.POLLIN:
            return s.recv_json()
        return None
    finally:
        s.close()


def print_config(cfg):
    print("=== teleimager camera config ===")
    for topic, c in cfg.get("camera", {}).items():
        print(f"[{topic}] type={c.get('type')} shape(HxW)={c.get('image_shape')} "
              f"binocular={c.get('binocular')} fps={c.get('fps')} "
              f"zmq={'on' if c.get('enable_zmq') else 'off'}:{c.get('zmq_port')} "
              f"webrtc={'on' if c.get('enable_webrtc') else 'off'}:{c.get('webrtc_port')} "
              f"enable_depth={c.get('enable_depth', False)}")
        ids = {k: c.get(k) for k in ("serial_number", "physical_path", "bcd_device", "vid_pid", "video_id") if c.get(k)}
        if ids:
            print(f"    id: {ids}")


class FrameSub:
    def __init__(self, host, port):
        self.sock = zmq.Context.instance().socket(zmq.SUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(f"tcp://{host}:{port}")
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")

    def read(self, timeout_ms=500):
        if not (self.sock.poll(timeout_ms) & zmq.POLLIN):
            return None
        buf = self.sock.recv()
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)

    def close(self):
        self.sock.close()


class StereoDepth:
    def __init__(self, num_disp=128, block=7, fx=None, baseline=None, scale=0.5):
        self.scale = scale  # 연산량 절감을 위한 축소 비율
        self.fx = fx * scale if fx else None
        self.baseline = baseline
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disp, blockSize=block,
            P1=8 * 3 * block ** 2, P2=32 * 3 * block ** 2,
            disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def compute(self, frame):
        w = frame.shape[1] // 2
        left, right = frame[:, :w], frame[:, w:]
        if self.scale != 1.0:
            left = cv2.resize(left, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disp = self.sgbm.compute(gl, gr).astype(np.float32) / 16.0
        disp[disp <= 0] = np.nan
        return left, disp

    def depth_m(self, disp):
        if not (self.fx and self.baseline):
            return None
        return self.fx * self.baseline / disp


def center_stats(arr, roi=10):
    h, w = arr.shape
    patch = arr[h // 2 - roi:h // 2 + roi, w // 2 - roi:w // 2 + roi]
    valid = patch[np.isfinite(patch)]
    ratio = valid.size / patch.size
    return (float(np.median(valid)) if valid.size else None), ratio


def colorize_disp(disp, num_disp):
    d = np.nan_to_num(disp, nan=0.0)
    d8 = np.clip(d / num_disp * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
    vis[~np.isfinite(disp)] = 0
    return vis


def overlay(img, text, roi=10):
    h, w = img.shape[:2]
    cv2.rectangle(img, (w // 2 - roi, h // 2 - roi), (w // 2 + roi, h // 2 + roi), (255, 255, 255), 2)
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img


class Worker:
    def __init__(self, sub, stereo, binocular, num_disp):
        self.sub, self.stereo, self.binocular, self.num_disp = sub, stereo, binocular, num_disp
        self.lock = threading.Lock()
        self.raw = self.left = self.disp = None
        self.text = "waiting"
        self.running = True
        self.fps = 0.0

    def loop(self):
        t_prev = time.time()
        stalled = False
        while self.running:
            frame = self.sub.read()
            if frame is None:
                if not stalled:
                    print("[WARN] 프레임 수신 중단 (서버 실행 / zmq_port / 방화벽 확인)")
                    stalled = True
                continue
            if stalled:
                print("[INFO] 프레임 수신 재개")
                stalled = False
            now = time.time()
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            if not self.binocular:
                with self.lock:
                    self.raw, self.left, self.disp = frame, frame, None
                    self.text = "monocular: depth 계산 불가"
                continue
            left, disp = self.stereo.compute(frame)
            d_c, ratio = center_stats(disp)
            depth = self.stereo.depth_m(disp)
            if depth is not None:
                z_c, _ = center_stats(depth)
                txt = f"center {z_c:.3f} m (disp {d_c:.1f}px) valid {ratio*100:.0f}%" if z_c else f"no depth valid {ratio*100:.0f}%"
            else:
                txt = f"center disp {d_c:.1f}px valid {ratio*100:.0f}%" if d_c else f"no disparity valid {ratio*100:.0f}%"
            with self.lock:
                self.raw, self.left, self.disp, self.text = frame, left, disp, txt

    def snapshot(self):
        with self.lock:
            return self.raw, self.left, self.disp, self.text


def run_web(worker, port):
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, StreamingResponse
    import uvicorn

    app = FastAPI()

    def mjpeg(kind):
        while True:
            raw, left, disp, text = worker.snapshot()
            if raw is None:
                time.sleep(0.05)
                continue
            if kind == "raw":
                img = raw.copy()
            elif kind == "depth" and disp is not None:
                img = overlay(colorize_disp(disp, worker.num_disp), text)
            else:
                img = overlay(left.copy(), text)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
            time.sleep(1 / 15)

    @app.get("/")
    def index():
        return HTMLResponse(
            "<html><body style='background:#111;color:#eee;font-family:sans-serif'>"
            "<h3>H2 head camera (teleimager) depth check</h3>"
            "<img src='/video_feed' style='max-width:49%'> "
            "<img src='/depth_feed' style='max-width:49%'><br>"
            "<img src='/raw_feed' style='max-width:98%'></body></html>")

    @app.get("/video_feed")
    def video_feed():
        return StreamingResponse(mjpeg("left"), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/depth_feed")
    def depth_feed():
        return StreamingResponse(mjpeg("depth"), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/raw_feed")
    def raw_feed():
        return StreamingResponse(mjpeg("raw"), media_type="multipart/x-mixed-replace; boundary=frame")

    print(f"[WEB] http://0.0.0.0:{port}/")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.123.164", help="teleimager-server IP (H2 PC2 IP 확인 필요)")
    ap.add_argument("--req-port", type=int, default=60000)
    ap.add_argument("--topic", default="head_camera")
    ap.add_argument("--zmq-port", type=int, default=None, help="설정 조회 실패 시 직접 지정 (기본 55555)")
    ap.add_argument("--binocular", choices=["auto", "yes", "no"], default="auto")
    ap.add_argument("--num-disp", type=int, default=128, help="16의 배수")
    ap.add_argument("--block", type=int, default=7)
    ap.add_argument("--scale", type=float, default=0.5, help="SGBM 입력 축소 비율")
    ap.add_argument("--fx", type=float, default=None, help="원본 좌측 이미지 기준 focal length(px)")
    ap.add_argument("--baseline", type=float, default=None, help="좌우 카메라 간격(m)")
    ap.add_argument("--web", action="store_true")
    ap.add_argument("--port", type=int, default=50016, help="웹 포트 (50001 = rs_stream, 50014 = check_head_cam, 50015 = check_depth)")
    ap.add_argument("--save", default=None, help="스냅샷 저장 디렉터리 (저장 후 종료)")
    args = ap.parse_args()

    cfg = request_config(args.host, args.req_port)
    cam = {}
    if cfg is None:
        print(f"[WARN] {args.host}:{args.req_port} 설정 조회 실패 (teleimager-server 실행 여부 확인)")
    else:
        print_config(cfg)
        cam = cfg.get("camera", {}).get(args.topic, {})
        if not cam:
            print(f"[WARN] topic '{args.topic}' 없음")
        elif not cam.get("enable_zmq"):
            print(f"[ERR] '{args.topic}' 의 enable_zmq 가 꺼져 있음 → 서버 yaml 에서 켜야 ZMQ 로 받을 수 있음")
            sys.exit(1)

    zmq_port = args.zmq_port or cam.get("zmq_port") or 55555
    if args.binocular == "auto":
        binocular = bool(cam.get("binocular", True))
    else:
        binocular = args.binocular == "yes"
    print(f"[SUB] tcp://{args.host}:{zmq_port}  binocular={binocular}")

    sub = FrameSub(args.host, zmq_port)
    first = sub.read(5000)
    if first is None:
        print("[ERR] 5초간 프레임 없음")
        sys.exit(1)
    h, w = first.shape[:2]
    print(f"[SUB] 첫 프레임 {w}x{h}" + ("  (좌/우 각 %dx%d)" % (w // 2, h) if binocular else ""))
    if binocular and w < 2 * h * 0.9:
        print("[WARN] 가로:세로 비율이 side-by-side 스테레오로 보이지 않음 → --binocular no 확인")

    stereo = StereoDepth(args.num_disp, args.block, args.fx, args.baseline, args.scale)
    worker = Worker(sub, stereo, binocular, args.num_disp)
    t_worker = threading.Thread(target=worker.loop, daemon=True)
    t_worker.start()

    try:
        while worker.snapshot()[0] is None:
            time.sleep(0.05)

        if args.save:
            time.sleep(1.0)
            raw, left, disp, text = worker.snapshot()
            os.makedirs(args.save, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(os.path.join(args.save, f"raw_{ts}.png"), raw)
            if disp is not None:
                cv2.imwrite(os.path.join(args.save, f"disp_{ts}.png"), colorize_disp(disp, args.num_disp))
                np.save(os.path.join(args.save, f"disp_{ts}.npy"), disp)
            if cfg is not None:
                with open(os.path.join(args.save, f"config_{ts}.json"), "w") as f:
                    json.dump(cfg, f, indent=2, ensure_ascii=False)
            print(f"[SAVE] {args.save}/  {text}")
            return

        if args.web:
            threading.Thread(target=run_web, args=(worker, args.port), daemon=True).start()

        print("=== 중앙 disparity/depth 출력 (Ctrl+C 종료) ===")
        while True:
            _, _, _, text = worker.snapshot()
            print(f"fps={worker.fps:5.1f}  {text}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        worker.running = False
        t_worker.join(timeout=3.0)  # poll(2s) 종료 대기 후 소켓 close (zmq abort 방지)
        sub.close()


if __name__ == "__main__":
    main()
