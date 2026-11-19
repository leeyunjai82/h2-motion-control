#!/usr/bin/env python3
"""
check_head_cam.py — H2 머리 쌍안 카메라 수신 확인: 왼눈(또는 오른눈·원본) RGB + 깊이 (로봇 명령 없음)

  공식 문서 'Bilateral Data Stream Acquisition Interface' (support.unitree.com H2_developer, 2026-08-11 판) 기준
    RGB  : PC1 이 RTP/H264 UDP 유니캐스트로 '수신 IP' 에 보냄 (기본 192.168.123.170)
           5002 원본 쌍안 1920×1080 15 fps (왜곡 보정 전) / 5004 왼눈 544×448 10 fps / 5006 오른눈 544×448 10 fps
    깊이 : PC1(192.168.123.161) TCP 5000 에 접속해서 받음 — Y16 544×448 10 fps (0·65535 = 무효)
           문서·공식 스크립트는 mm (÷1000 = m) 라고 하지만 실측 2026-10-09 실내에서 가운데 28 m 로 나옴 → 단위 확인 필요
           (--depth-scale 로 바꿔 볼 수 있음, 웹/창에서 클릭하면 원래 값 raw 도 같이 표시)
           프레임 = 헤더 36 B (<IQQIIII: magic 'Y16 ' 0x59313620, seq, 보낸 쪽 시각 us, w, h, format, data_size) + w·h·2 B
           (공식 unitree-dep-img 1.0.0 deb 의 dep_img_client.c 와 같은 형식 → deb·v4l2loopback 없이 직접 읽음)

  준비
    로봇: python tools/head_cam_on.py 한 번 (video_hub 끔 · stereo_patch_pc1 켬 · 수신 IP = 이 PC · 재시작 — start_robot.sh 도 실행)
          또는 앱에서 video_hub 끄기 (충돌) → 'Stereo patch PC1' 서비스 켜기 (기본 자동 시작 아님, 로봇을 다시 켜면 기본으로 돌아감)
          RGB 수신 IP 를 이 PC 로: --set-ip (= curl "http://192.168.123.161:9080/set?ip=<이 PC IP>") → 앱에서 서비스 껐다 켜기
    이 PC: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                            gstreamer1.0-plugins-ugly gstreamer1.0-libav
           방화벽: sudo ufw allow from 192.168.123.0/24
    공식 unitree-dep-img 와 동시에 켜지 말 것 (깊이 서버 동시 접속 가능 여부 확인 필요)

  python tools/check_head_cam.py                       # 화면 있으면 창, 없으면(SSH) 웹 http://<이 PC 의 아무 IP>:50014/
                                                       #   RGB | 깊이 | 겹쳐 보기, 클릭 = 그 점 깊이 (raw·m), 저장 버튼
  python tools/check_head_cam.py --web 50014           # 화면이 있어도 웹으로
  python tools/check_head_cam.py --sec 10              # 10 초 통계만 (해상도, fps, 깊이 유효율·가운데 값)
  python tools/check_head_cam.py --save logs/headcam   # 1 초마다 RGB png + 깊이 npy(uint16 raw) 저장 (내부 파라미터·정렬 확인용)
  python tools/check_head_cam.py --depth-scale 0.0001  # 깊이 1 = 0.1 mm 로 보고 표시
  python tools/check_head_cam.py --rgb raw             # 5002 원본 1920×1080 쌍안
  python tools/check_head_cam.py --set-ip              # RGB 수신 IP 를 이 PC 로 바꾸고 끝 (그다음 앱에서 서비스 재시작)

확인 필요 (문서에 없음): 깊이 단위, 내부 파라미터, 깊이가 왼눈 544×448 과 픽셀 정렬인지, 머리 링크 기준 카메라 위치
                        (URDF 에 카메라 프레임 없음 — 머리 pitch 29 / yaw 30 관절 따라 움직임).
"""
import argparse
import json
import os
import shutil
import sys
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron.drivers.camera.head_cam import PC1, RGB_PORTS, DepthRx, RgbRx, my_ip, web_urls   # noqa: E402

WIN = "H2 head camera (q quit, s save, click = depth)"


def valid_mask(d):
    return (d > 0) & (d < 65535)


def colorize(d, rng):
    """uint16 raw → 컬러 (가까울수록 빨강, 무효 = 진회색). rng = (lo, hi) raw 값."""
    lo, hi = rng
    bad = ~valid_mask(d)
    u8 = (np.clip((d.astype(np.float32) - lo) / max(hi - lo, 1.0), 0, 1) * 255).astype(np.uint8)
    c = cv2.applyColorMap(255 - u8, cv2.COLORMAP_JET)
    c[bad] = (40, 40, 40)
    return c


def color_range(d, rng_m, scale):
    """색 범위 [raw]: rng_m 이 None 이면 유효 값 1–99 % (단위를 몰라도 보이게)."""
    if rng_m is not None:
        return rng_m[0] / scale, rng_m[1] / scale
    v = d[valid_mask(d)]
    return (float(np.percentile(v, 1)), float(np.percentile(v, 99))) if len(v) else (0.0, 1.0)


def depth_at(d, u, v, r=1):
    """(u, v) 둘레 (2r+1)² 의 유효 raw 중앙값, 없으면 0."""
    p = d[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].ravel()
    p = p[(p > 0) & (p < 65535)]
    return int(np.median(p)) if len(p) else 0


def depth_stats(d):
    valid = valid_mask(d)
    h, w = d.shape
    c = depth_at(d, w // 2, h // 2, 10)
    if valid.any():
        v = d[valid]
        lo, hi = np.percentile(v, 1), np.percentile(v, 99)
    else:
        lo = hi = 0
    return valid.mean() * 100, c, lo, hi


def compose(rgb, dep, rng_m, scale):
    """RGB | 깊이 | 겹쳐 보기 한 장 + 패널 배치 {이름: (x0, 폭, sx, sy)}."""
    H = dep[0].shape[0] if dep is not None else (min(rgb[0].shape[0], 540) if rgb is not None else 448)
    panels, layout, x0 = [], {}, 0
    if rgb is not None:
        img = rgb[0]
        pw = int(round(img.shape[1] * H / img.shape[0]))
        panels.append(cv2.resize(img, (pw, H)))
        layout["rgb"] = (x0, pw, img.shape[1] / pw, img.shape[0] / H); x0 += pw
    if dep is not None:
        d = dep[0]
        dh, dw = d.shape
        rr = color_range(d, rng_m, scale)
        c = colorize(d, rr)
        cd = depth_at(d, dw // 2, dh // 2, 10)
        cv2.drawMarker(c, (dw // 2, dh // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(c, f"center raw {cd}  = {cd * scale:.3f} m", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(c, f"color {rr[0] * scale:.2f}-{rr[1] * scale:.2f} m", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        pw = int(round(dw * H / dh))
        panels.append(cv2.resize(c, (pw, H), interpolation=cv2.INTER_NEAREST))
        layout["depth"] = (x0, pw, dw / pw, dh / H); x0 += pw
        if rgb is not None and rgb[0].shape[:2] == d.shape:                # 같은 크기면 겹쳐서 정렬 확인
            ov = cv2.addWeighted(rgb[0], 0.5, colorize(d, rr), 0.5, 0)
            panels.append(cv2.resize(ov, (pw, H)))
            layout["overlay"] = (x0, pw, dw / pw, dh / H); x0 += pw
    return (np.hstack(panels) if panels else None), layout


def click_depth(x, y, layout, rgb, dep, scale):
    """합친 영상의 (x, y) → {panel, u, v, raw, m} 또는 오류 문자열."""
    for name, (x0, pw, sx, sy) in layout.items():
        if x0 <= x < x0 + pw:
            u, v = int((x - x0) * sx), int(y * sy)
            if dep is None or (name == "rgb" and rgb[0].shape[:2] != dep[0].shape):
                return {"panel": name, "u": u, "v": v, "error": "RGB and depth sizes differ, no depth pixel mapping"}
            raw = depth_at(dep[0], u, v)
            return {"panel": name, "u": u, "v": v, "raw": raw, "m": round(raw * scale, 4) if raw else None}
    return {"error": "outside image"}


def save_pair(out, idx, rgb, dep, scale):
    os.makedirs(out, exist_ok=True)
    meta = {"i": idx, "t": time.time(), "depth_scale_m": scale}
    if rgb is not None:
        cv2.imwrite(f"{out}/rgb_{idx:04d}.png", rgb[0])
        meta.update(rgb_t=rgb[1], rgb_size=list(rgb[0].shape[1::-1]))
    if dep is not None:
        np.save(f"{out}/depth_{idx:04d}.npy", dep[0])
        cv2.imwrite(f"{out}/depth_{idx:04d}.png", colorize(dep[0], color_range(dep[0], None, scale)))
        meta.update(depth_t=dep[3], depth_seq=dep[1], depth_send_us=dep[2], depth_size=list(dep[0].shape[::-1]))
    if rgb is not None and dep is not None:
        meta["rgb_minus_depth_ms"] = round((rgb[1] - dep[3]) * 1000, 1)
    with open(f"{out}/meta.jsonl", "a") as f:
        f.write(json.dumps(meta) + "\n")
    msg = f"[save] {out} #{idx:04d}" + (f"  RGB−depth receive time diff {meta['rgb_minus_depth_ms']} ms" if "rgb_minus_depth_ms" in meta else "")
    print(msg)
    return msg


def report(rgb_rx, dep_rx, sec, scale):
    print(f"\n=== results after {sec:.0f} s ===")
    if rgb_rx is not None:
        if rgb_rx.count:
            w, h = rgb_rx.size
            print(f"  RGB  (UDP {rgb_rx.port}) {w}×{h}, caps {rgb_rx.caps_fps} fps, received {rgb_rx.count / sec:.1f} fps")
        else:
            print(f"  RGB  (UDP {rgb_rx.port}) ❌ nothing received — {rgb_rx.err}")
    if dep_rx is not None:
        f = dep_rx.latest()
        if f is not None:
            d, seq, ts, tr = f
            ok, c, lo, hi = depth_stats(d)
            print(f"  depth (TCP {dep_rx.host}:{dep_rx.port}) {d.shape[1]}×{d.shape[0]}, received {dep_rx.count / sec:.1f} fps, "
                  f"seq gaps {dep_rx.seq_gaps}, last seq {seq}")
            print(f"        valid {ok:.0f} %, center 21×21 raw {c} = {c * scale:.3f} m, valid 1–99 % raw {lo:.0f}–{hi:.0f} "
                  f"= {lo * scale:.3f}–{hi * scale:.3f} m  (scale {scale} m/unit)")
            print(f"        (note) receive time − send time {(tr - ts / 1e6) * 1000:+.0f} ms — only meaningful if robot and PC clocks are synced")
        else:
            print(f"  depth (TCP {dep_rx.host}:{dep_rx.port}) ❌ nothing received — {dep_rx.err}")


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H2 head camera</title><style>body{font-family:sans-serif;margin:10px;background:#111;color:#eee}
img{max-width:100%;cursor:crosshair;border:1px solid #444}button{font-size:15px;margin:4px;padding:5px 12px}
pre{background:#222;padding:8px;white-space:pre-wrap;max-height:40vh;overflow:auto}</style></head><body>
<h3>H2 head camera — RGB | depth | overlay (click = depth at that point)</h3>
<img id="v" src="/video"><div><button onclick="sv()">Save</button> <span id="st"></span></div><pre id="log"></pre>
<script>
const v=document.getElementById('v'),log=document.getElementById('log');
function add(t){log.textContent=t+'\\n'+log.textContent}
v.addEventListener('click',e=>{const r=v.getBoundingClientRect();const x=Math.round((e.clientX-r.left)*v.naturalWidth/r.width),
y=Math.round((e.clientY-r.top)*v.naturalHeight/r.height);fetch(`/depth?x=${x}&y=${y}`).then(r=>r.json()).then(j=>{
add(j.error?JSON.stringify(j):`[${j.panel}] (${j.u}, ${j.v})  raw ${j.raw}  → ${j.m} m`)})});
function sv(){fetch('/save').then(r=>r.json()).then(j=>add(j.msg))}
setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('st').textContent=j.text}),1000)
</script></body></html>"""


def run_web(port, rgb_rx, dep_rx, a, scale, rng_m, t0):
    st = {"jpg": None, "layout": {}, "rgb": None, "dep": None, "idx": 0, "t_save": 0.0}
    lock = threading.Lock()

    def producer():
        while True:
            rgb = rgb_rx.latest() if rgb_rx else None
            dep = dep_rx.latest() if dep_rx else None
            img, layout = compose(rgb, dep, rng_m, scale)
            if img is not None:
                ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                with lock:
                    st.update(jpg=j.tobytes() if ok else st["jpg"], layout=layout, rgb=rgb, dep=dep)
            if a.save and a.save_every > 0 and time.time() - st["t_save"] >= a.save_every and (rgb is not None or dep is not None):
                save_pair(a.save, st["idx"], rgb, dep, scale); st["idx"] += 1; st["t_save"] = time.time()
            time.sleep(0.08)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, obj):
            b = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            if u.path == "/":
                b = PAGE.encode()
                self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
            elif u.path == "/video":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
                last = None
                try:
                    while True:
                        with lock:
                            j = st["jpg"]
                        if j is not None and j is not last:
                            last = j
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(j)).encode()
                                             + b"\r\n\r\n" + j + b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif u.path == "/depth":
                with lock:
                    layout, rgb, dep = st["layout"], st["rgb"], st["dep"]
                r = click_depth(int(q.get("x", [0])[0]), int(q.get("y", [0])[0]), layout, rgb, dep, scale)
                if "raw" in r:
                    print(f"[click] {r['panel']} ({r['u']}, {r['v']}) raw {r['raw']} → {r['m']} m (scale {scale})")
                self._json(r)
            elif u.path == "/save":
                with lock:
                    rgb, dep = st["rgb"], st["dep"]
                msg = save_pair(a.save or "logs/headcam", st["idx"], rgb, dep, scale); st["idx"] += 1
                self._json({"msg": msg})
            elif u.path == "/status":
                now = time.time()
                s = []
                if rgb_rx:
                    s.append(f"RGB {rgb_rx.count / (now - t0):.1f} fps" + (f" ({rgb_rx.err})" if rgb_rx.err else ""))
                if dep_rx:
                    s.append(f"depth {dep_rx.count / (now - t0):.1f} fps" + (f" ({dep_rx.err})" if dep_rx.err else ""))
                self._json({"text": " | ".join(s)})
            else:
                self.send_error(404)

    threading.Thread(target=producer, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    print(f"[head_cam] web view (Ctrl+C to quit):\n    {web_urls(port, a.host)}")
    srv.serve_forever()


def run_gui(rgb_rx, dep_rx, a, scale, rng_m, t0):
    click, state = {}, {"layout": {}}

    def on_mouse(ev, x, y, _flags, _p):
        if ev == cv2.EVENT_LBUTTONDOWN:
            click["xy"] = (x, y)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, on_mouse)
    t_save, idx, last_status = 0.0, 0, 0.0
    try:
        while True:
            rgb = rgb_rx.latest() if rgb_rx else None
            dep = dep_rx.latest() if dep_rx else None
            img, state["layout"] = compose(rgb, dep, rng_m, scale)
            if img is not None:
                cv2.imshow(WIN, img)
            if "xy" in click:
                r = click_depth(*click.pop("xy"), state["layout"], rgb, dep, scale)
                print(f"[click] {r}" if "raw" not in r else f"[click] {r['panel']} ({r['u']}, {r['v']}) raw {r['raw']} → {r['m']} m")
            now = time.time()
            if a.save and a.save_every > 0 and now - t_save >= a.save_every and (rgb is not None or dep is not None):
                save_pair(a.save, idx, rgb, dep, scale); idx += 1; t_save = now
            if now - last_status > 5.0:
                s = []
                if rgb_rx:
                    s.append(f"RGB {rgb_rx.count / (now - t0):.1f} fps" + (f" ({rgb_rx.err})" if rgb_rx.err else ""))
                if dep_rx:
                    s.append(f"depth {dep_rx.count / (now - t0):.1f} fps" + (f" ({dep_rx.err})" if dep_rx.err else ""))
                print("[head_cam] " + " | ".join(s))
                last_status = now
            k = cv2.waitKey(30) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s") and (rgb is not None or dep is not None):
                save_pair(a.save or "logs/headcam", idx, rgb, dep, scale); idx += 1
    finally:
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="Check H2 head stereo camera RGB + depth reception")
    ap.add_argument("--host", default=PC1, help="PC1 IP (depth server / receive-IP setting server)")
    ap.add_argument("--depth-port", type=int, default=5000)
    ap.add_argument("--rgb", choices=("left", "right", "raw", "none"), default="left")
    ap.add_argument("--rgb-port", type=int, default=None, help="RTP port override (default left 5004 / right 5006 / raw 5002)")
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--sec", type=float, default=0.0, help="receive for this long, then print stats only (0 = window/web view)")
    ap.add_argument("--web", type=int, default=None, help="web view port (50014 automatically if no display)")
    ap.add_argument("--save", default="", help="save folder (RGB png, depth npy uint16 raw + png, meta.jsonl)")
    ap.add_argument("--save-every", type=float, default=1.0, help="auto-save interval [s] (0 = only via Save button / s key)")
    ap.add_argument("--depth-scale", type=float, default=0.001, help="meters per depth unit (docs: mm → 0.001, unverified)")
    ap.add_argument("--range", default="auto", help="depth color range 'lo,hi' [m] or auto (valid values 1–99 %%)")
    ap.add_argument("--probe-sec", type=float, default=6.0, help="time to wait for the RGB stream [s]")
    ap.add_argument("--set-ip", action="store_true", help="set RGB receive IP to this PC and exit (then restart the service in the app)")
    a = ap.parse_args()
    scale = a.depth_scale
    rng_m = None if a.range == "auto" else tuple(float(v) for v in a.range.split(","))
    me = my_ip(a.host)
    print(f"[head_cam] this PC IP {me} (route to PC1 {a.host}) — RGB is sent only to the receive IP set on the robot (default 192.168.123.170)")

    if a.set_ip:
        url = f"http://{a.host}:9080/set?ip={me}"
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                body = r.read(300).decode(errors="replace")
            print(f"[head_cam] {url} → HTTP {r.status} {body.strip()[:200]}")
            print("[head_cam] Turn the 'Stereo patch PC1' service off and on in the app to apply (per official docs).")
        except OSError as e:
            print(f"[head_cam] ❌ {url} failed: {e} — check that the service is on in the app")
        return

    # 화면 없는데 OpenCV(Qt) 창을 열면 프로세스가 강제 종료됨 → 미리 판단해서 웹으로
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) or sys.platform != "linux"
    web_port = a.web if a.web is not None else (None if has_display else 50014)

    dep_rx = None
    if not a.no_depth:
        dep_rx = DepthRx(a.host, a.depth_port)
        dep_rx.start()

    rgb_rx = None
    if a.rgb != "none":
        if shutil.which("gst-launch-1.0") is None:
            print("[head_cam] ❌ gst-launch-1.0 not found → sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good "
                  "gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav  (continuing with depth only, no RGB)")
        else:
            rgb_rx = RgbRx(a.rgb_port or RGB_PORTS[a.rgb])
            print(f"[head_cam] waiting for RGB UDP {rgb_rx.port} ({a.probe_sec:.0f} s)...")
            if rgb_rx.probe(a.probe_sec):
                print(f"[head_cam] RGB {rgb_rx.size[0]}×{rgb_rx.size[1]}, caps {rgb_rx.caps_fps} fps")
                rgb_rx.start()
            else:
                print(f"[head_cam] ❌ RGB UDP {rgb_rx.port} nothing received — check: video_hub off in app · Stereo patch PC1 on · "
                      f"receive IP = {me} (--set-ip, then restart the service) · ufw. Continuing with depth only")
                rgb_rx = None
    if rgb_rx is None and dep_rx is None:
        return

    for rx in (rgb_rx, dep_rx):              # fps 는 여기서부터 (RGB 기다리는 동안 받은 깊이 빼기)
        if rx is not None:
            with rx.lock:
                rx.count = 0
                if rx is dep_rx:
                    rx.seq_gaps = 0
    t0 = time.time()
    try:
        if a.sec > 0:
            t_save, idx = 0.0, 0
            while time.time() - t0 < a.sec:
                if a.save and a.save_every > 0 and time.time() - t_save >= a.save_every:
                    rgb, dep = (rgb_rx.latest() if rgb_rx else None), (dep_rx.latest() if dep_rx else None)
                    if rgb is not None or dep is not None:
                        save_pair(a.save, idx, rgb, dep, scale); idx += 1; t_save = time.time()
                time.sleep(0.05)
        elif web_port is not None:
            run_web(web_port, rgb_rx, dep_rx, a, scale, rng_m, t0)
        else:
            run_gui(rgb_rx, dep_rx, a, scale, rng_m, t0)
        report(rgb_rx, dep_rx, time.time() - t0, scale)
    except KeyboardInterrupt:
        report(rgb_rx, dep_rx, time.time() - t0, scale)
    finally:
        if rgb_rx:
            rgb_rx.stop = True
            rgb_rx.kill()
        if dep_rx:
            dep_rx.stop = True


if __name__ == "__main__":
    main()
