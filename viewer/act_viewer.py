#!/usr/bin/env python3
"""
act_viewer.py — ACT 데이터셋 · 모델 보기 (전시용, 로봇 · GPU 필요 없음)

  Windows: viewer/start_viewer.bat 더블클릭 — 처음 한 번 viewer/.venv 가상환경을 만들고 requirements.txt 설치 (Python 3.8+ 필요)
  Linux  : python3 -m venv viewer/.venv && viewer/.venv/bin/pip install -r viewer/requirements.txt
           viewer/.venv/bin/python viewer/act_viewer.py [--data <폴더>] [--port 8800] [--no-browser]

  viewer/data/ 에 zip 을 넣어 두면 켤 때 자동으로 읽음 (화면에 끌어다 놓아도 됨)
    데이터셋 zip : 수집 페이지 (/collect) [Dataset zip]   — info.json format h2-gripper-episodes-v1
    모델 zip     : 학습 웹 (:50100) model zip            — meta.json format h2-gripper-act-v1
  zip 은 브라우저가 필요한 부분만 읽음 (HTTP Range) — 서버는 파일만 내줌, 모델 실행 안 함
  3D 로봇 = robots/h2/H2.urdf + 메시, 손 = gripper/dex1_1 (Unitree Dex1-1) + 어댑터 (gripper/gripper.yaml xr_view.dex1_view)
"""
import argparse
import json
import mimetypes
import os
import re
import sys
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PAGE = os.path.join(HERE, "act_viewer.html")
THREE = os.path.join(REPO, "common", "assets", "vendor", "three.min.js")
URDF = os.path.join(REPO, "robots", "h2", "H2.urdf")
GRIP_YAML = os.path.join(REPO, "gripper", "gripper.yaml")
MESH_DIRS = {"h2": os.path.join(REPO, "robots", "h2", "meshes"),
             "dex1": os.path.join(REPO, "gripper", "dex1_1", "meshes"),
             "adapter": os.path.join(REPO, "gripper", "adapter")}
CHUNK = 1 << 20


def robot_info():
    """3D 로봇에 필요한 것: H2 URDF 글자 + Dex1 URDF 글자 + 장착 · 그리퍼 값 (gripper.yaml). 못 읽으면 그 항목 None."""
    out = {"urdf": open(URDF, encoding="utf-8").read(), "dex1": None, "grip": None, "warn": []}
    try:
        import yaml
    except ImportError:
        out["warn"].append("PyYAML 없음 (viewer/requirements.txt 설치) — 손은 그리퍼 없이 그림")
        return out
    with open(GRIP_YAML, encoding="utf-8") as f:
        g = yaml.safe_load(f)
    dv = ((g.get("xr_view") or {}).get("dex1_view")) or {}
    out["grip"] = {"q_hi": float(g["q_range"][1]), "mm_per_rad": float(g["pick"]["mm_per_rad"])}
    durdf = os.path.join(REPO, "gripper", str(dv.get("urdf", "")))
    if dv and os.path.isfile(durdf):
        a = dv.get("adapter_stl") or {}
        apath = os.path.join(REPO, "gripper", str(a.get("path", "")))
        out["dex1"] = {"urdf": open(durdf, encoding="utf-8").read(),
                       "origin_x": float(g["tcp_offset"][0]) - float(dv["origin_to_tcp_m"]),
                       "mount_rpy_deg": [float(v) for v in dv["mount_rpy_deg"]],
                       "wrist_face": float(dv["wrist_face_m"]), "adapter": [float(v) for v in dv["adapter_m"]],
                       "adapter_stl": ({"file": os.path.basename(apath), "scale": float(a["scale"]),
                                        "origin_mm": [float(v) for v in a["origin_mm"]], "at_m": [float(v) for v in a["at_m"]],
                                        "rot": [[float(v) for v in r] for r in a["rot"]]}
                                       if a and os.path.isfile(apath) else None)}
    else:
        out["warn"].append(f"Dex1 URDF 없음 ({durdf}) — 손은 그리퍼 없이 그림")
    return out


class Handler(BaseHTTPRequestHandler):
    data_dir = os.path.join(HERE, "data")

    def log_message(self, fmt, *args):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def _file(self, path, ctype=None):
        """파일 (Range 지원 — 브라우저가 zip 의 필요한 부분만 읽음)"""
        if not os.path.isfile(path):
            return self.send_error(HTTPStatus.NOT_FOUND)
        size = os.path.getsize(path)
        ctype = ctype or mimetypes.guess_type(path)[0] or "application/octet-stream"
        m = re.match(r"bytes=(\d*)-(\d*)$", self.headers.get("Range", "") or "")
        if m and (m.group(1) or m.group(2)):
            if m.group(1):
                start = int(m.group(1))
                end = min(int(m.group(2)) if m.group(2) else size - 1, size - 1)
            else:
                start, end = max(0, size - int(m.group(2))), size - 1
            if start > end or start >= size:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
            self.send_response(HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        else:
            start, end = 0, size - 1
            self.send_response(HTTPStatus.OK)
        n = end - start + 1
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(n))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        with open(path, "rb") as f:
            f.seek(start)
            while n > 0:
                b = f.read(min(CHUNK, n))
                if not b:
                    break
                self.wfile.write(b)
                n -= len(b)

    def do_GET(self):
        u = urlparse(self.path)
        p = unquote(u.path)
        try:
            if p in ("/", "/index.html"):
                return self._file(PAGE, "text/html; charset=utf-8")
            if p == "/vendor/three.min.js":
                return self._file(THREE, "application/javascript")
            if p == "/api/robot":
                return self._json(robot_info())
            if p == "/api/files":
                d = self.data_dir
                fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".zip")) if os.path.isdir(d) else []
                return self._json({"dir": d, "files": [{"name": f, "size": os.path.getsize(os.path.join(d, f))} for f in fs]})
            if p.startswith("/data/"):
                name = os.path.basename(p[len("/data/"):])
                return self._file(os.path.join(self.data_dir, name), "application/zip")
            if p == "/api/mesh":
                q = parse_qs(u.query)
                d = MESH_DIRS.get((q.get("d") or [""])[0])
                name = os.path.basename((q.get("f") or [""])[0])
                if not d or not name:
                    return self.send_error(HTTPStatus.NOT_FOUND)
                path = os.path.join(d, name)
                if not os.path.isfile(path):          # URDF 와 대소문자가 다른 파일 (Windows 외)
                    hit = [f for f in os.listdir(d) if f.lower() == name.lower()]
                    path = os.path.join(d, hit[0]) if hit else path
                return self._file(path, "application/octet-stream")
            self.send_error(HTTPStatus.NOT_FOUND)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass


def main():
    ap = argparse.ArgumentParser(description="ACT 데이터셋 · 모델 보기")
    ap.add_argument("--data", default=os.path.join(HERE, "data"), help="zip 을 둔 폴더 (기본 viewer/data)")
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--host", default="127.0.0.1", help="다른 PC 에서 보려면 0.0.0.0")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.data, exist_ok=True)
    Handler.data_dir = os.path.abspath(a.data)
    for need in (PAGE, THREE, URDF):
        if not os.path.isfile(need):
            sys.exit(f"❌ 파일 없음: {need} (저장소를 통째로 clone 했는지 확인)")
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    url = f"http://127.0.0.1:{a.port}/"
    zips = [f for f in os.listdir(a.data) if f.lower().endswith(".zip")]
    print(f"[act_viewer] {url}   (끄기: Ctrl+C)")
    print(f"[act_viewer] zip 폴더: {Handler.data_dir} — {len(zips)} 개 {zips[:6]}")
    if not a.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
