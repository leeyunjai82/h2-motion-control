#!/usr/bin/env python3
# Version: 1.2
"""
RealSense → HTTP MJPEG 서버 (canvas viewer)
  1.2 - 해상도를 robot.yaml camera.resolution 에서 (H2: 848×480 — 640×480 보다 가로 화각 56° → 70°), 깊이 보기 크기도 같은 비율

라우트:
  /            : canvas 기반 뷰어
  /video_feed  : color MJPEG (ik_box.py 등에서 사용)
  /depth_feed  : depth MJPEG (해상도 절반 q60, 디버깅용 시각화)
  /depth_raw   : depth 16bit PNG 스트림 (mm 원본, detect_box용) ← v1.1 추가
"""

import threading
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import robot_env   # ROBOT 미지정/미지원이면 여기서 종료 — camera.resolution
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware


COLOR_W, COLOR_H = robot_env.CAMERA_RES   # 컬러·깊이 (깊이는 컬러에 정렬) — robot.yaml camera.resolution
FPS = 30
DEPTH_W, DEPTH_H = COLOR_W // 2, COLOR_H // 2   # /depth_feed 보기용 (같은 비율)
COLOR_Q, DEPTH_Q      = 80, 60
DEPTH_MAX_MM          = 3000
FAIL_RESTART          = 3      # wait_for_frames(2초) 연속 실패 횟수 → 카메라 다시 열기 (약 6초)
STAT_EVERY_SEC        = 30     # 수신 fps 로그 주기


class FrameBuffer:
    def __init__(self):
        self.data = None
        self.frame_id = 0
        self.cond = threading.Condition()
    def update(self, data: bytes):
        with self.cond:
            self.data = data
            self.frame_id += 1
            self.cond.notify_all()
    def wait_new(self, last_id: int, timeout: float = 1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout=timeout)
            return self.data, self.frame_id


color_buf     = FrameBuffer()
depth_buf     = FrameBuffer()   # 시각화 JPEG
depth_raw_buf = FrameBuffer()   # 16bit PNG (mm 원본)

pipeline  = None
align     = None
stop_flag = threading.Event()


def init_camera():
    global pipeline, align
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16,  FPS)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    for _ in range(15):
        pipeline.wait_for_frames()
    print(f"[RS] 카메라 시작 ({COLOR_W}x{COLOR_H}@{FPS}fps)")


def restart_camera():
    """카메라 다시 열기 — USB 순간 끊김(전원·케이블) 후 장치가 다시 잡히면 복구. 성공할 때까지 2초 간격 재시도."""
    global pipeline
    try:
        if pipeline:
            pipeline.stop()
    except Exception:
        pass
    n = 0
    while not stop_flag.is_set():
        n += 1
        try:
            init_camera()
            print(f"[RS] ✓ 카메라 재연결 성공 (시도 {n})", flush=True)
            return True
        except Exception as e:
            if n == 1 or n % 10 == 0:
                print(f"[RS] 재연결 실패 (시도 {n}): {e} — 2초 후 재시도 (USB 연결 확인)", flush=True)
            time.sleep(2.0)
    return False


def capture_loop():
    depth_lut = np.clip(
        np.arange(65536, dtype=np.float32) * (255.0 / DEPTH_MAX_MM), 0, 255
    ).astype(np.uint8)
    color_enc = [cv2.IMWRITE_JPEG_QUALITY, COLOR_Q]
    depth_enc = [cv2.IMWRITE_JPEG_QUALITY, DEPTH_Q]
    # PNG 압축 레벨 낮게 (속도 우선)
    png_enc = [cv2.IMWRITE_PNG_COMPRESSION, 1]

    fails = 0
    n_ok, t_stat = 0, time.time()
    while not stop_flag.is_set():
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception as e:
            fails += 1
            print(f"[RS] ⚠️ 프레임 수신 실패 {fails}/{FAIL_RESTART}: {e}", flush=True)
            if fails >= FAIL_RESTART:
                print("[RS] 카메라 응답 없음 — 다시 연다", flush=True)
                restart_camera()
                fails = 0
            continue
        if fails:
            print(f"[RS] 프레임 수신 복구 (연속 실패 {fails}회 후)", flush=True)
            fails = 0
        n_ok += 1
        if time.time() - t_stat >= STAT_EVERY_SEC:
            print(f"[RS] 수신 {n_ok / (time.time() - t_stat):.1f} fps", flush=True)
            n_ok, t_stat = 0, time.time()
        aligned = align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf or not df:
            continue

        # --- color ---
        color_img = np.asanyarray(cf.get_data()).copy()
        ok, buf = cv2.imencode('.jpg', color_img, color_enc)
        if ok:
            color_buf.update(buf.tobytes())

        depth_img = np.asanyarray(df.get_data())   # uint16 mm

        # --- depth 시각화 (JET) ---
        d_color = cv2.applyColorMap(depth_lut[depth_img], cv2.COLORMAP_JET)
        d_color[depth_img == 0] = 0
        d_small = cv2.resize(d_color, (DEPTH_W, DEPTH_H), interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode('.jpg', d_small, depth_enc)
        if ok:
            depth_buf.update(buf.tobytes())

        # --- depth raw (16bit PNG, mm 원본) ---
        ok, buf = cv2.imencode('.png', depth_img, png_enc)
        if ok:
            depth_raw_buf.update(buf.tobytes())


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_camera()
    threading.Thread(target=capture_loop, daemon=True).start()
    yield
    stop_flag.set()
    if pipeline:
        try: pipeline.stop()
        except Exception: pass


app = FastAPI(title="RealSense MJPEG Stream", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


def mjpeg_generator(buffer: FrameBuffer, content_type=b'image/jpeg'):
    last_id = -1
    boundary = b'--frame\r\nContent-Type: ' + content_type + b'\r\n\r\n'
    while True:
        data, fid = buffer.wait_new(last_id, timeout=1.0)
        if data is None or fid == last_id:
            continue
        last_id = fid
        yield boundary + data + b'\r\n'


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(mjpeg_generator(color_buf),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_feed")
async def depth_feed():
    return StreamingResponse(mjpeg_generator(depth_buf),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_raw")
async def depth_raw():
    """16bit PNG (mm 원본) 스트림 — detect_box.py가 디코딩해서 사용."""
    return StreamingResponse(mjpeg_generator(depth_raw_buf, content_type=b'image/png'),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/")
async def index():
    return HTMLResponse("<h2>RealSense Stream</h2>"
                        "<p>/video_feed (color), /depth_feed (vis), /depth_raw (16bit mm)</p>"
                        '<img src="/video_feed" style="width:100%;max-width:848px">')


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50001, timeout_graceful_shutdown=2)
