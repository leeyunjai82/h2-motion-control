"""
ports.py — 포트 (config/ports.env 한 곳에서 읽음, 표준 라이브러리만)

  from pitron.ports import PORTS, url
  PORTS["arm_server"]              # 50022
  url("detect_box", "/pose")       # http://localhost:50010/pose
  fill(html)                       # 화면 글자 안의 __PORT_ARM_SERVER__ 같은 자리를 숫자로
"""
import os
import re

from pitron import paths

ENV_PATH = os.path.join(paths.REPO_ROOT, "config", "ports.env")


def _load(path=ENV_PATH):
    out = {}
    with open(path, encoding="utf-8") as f:
        for ln in f:
            m = re.match(r"^\s*PORT_([A-Z0-9_]+)\s*=\s*(\d+)", ln)
            if m:
                out[m.group(1).lower()] = int(m.group(2))
    return out


PORTS = _load()


def url(name, path="", host="localhost"):
    return f"http://{host}:{PORTS[name]}{path}"


def fill(text):
    return re.sub(r"__PORT_([A-Z0-9_]+)__", lambda m: str(PORTS[m.group(1).lower()]), text)
