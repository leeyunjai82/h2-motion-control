"""
ports.py — 포트 (config/ports.env 한 곳에서 읽음, 표준 라이브러리만)

  from pitron.ports import PORTS, url
  PORTS["arm_server"]              # 50022
  url("detect_box", "/pose")       # http://localhost:50010/pose
  fill(html)                       # 화면 글자 안의 __PORT_ARM_SERVER__ 같은 자리를 숫자로
  with_home(html)                  # fill + 왼쪽 아래 "⌂ 홈" (런처) 링크
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


HOME_LINK = ('<a id="pitron-home" href="#" title="홈 (런처) — FSM · 기능 전환 · 종료" '
             'onclick="location.href=location.protocol+\'//\'+location.hostname+\':__PORT_LAUNCHER__/\';return false" '
             'style="position:fixed;left:10px;bottom:10px;z-index:9999;font:600 12px ui-monospace,Menlo,monospace;'
             'padding:6px 12px;border-radius:999px;background:#0e1116e6;border:1px solid #0b5a70;color:#00b7eb;text-decoration:none">⌂ 홈</a>'
             '<script>(function(){if(window.top===window)return;'      # 홈 앱 안 (iframe): 홈 링크 · 자기 머리글을 숨겨 앱 머리글 하나만 보이게
             'document.documentElement.classList.add("in-app");'      # 화면마다 .in-app 으로 앱 안에서 숨길 것을 정함
             'var h=document.getElementById("pitron-home");if(h)h.remove();'
             'var st=document.createElement("style");st.textContent="#g1-lang-btn,.port-label{display:none!important}";document.head.appendChild(st);'
             'var tb=document.querySelector(".topbar,.top,.app-header");if(!tb)return;'
             'if(!tb.querySelector("input,select,button:not(.lang-btn),label.sw")){tb.style.display="none";return;}'   # 정보만 있는 머리글 → 통째로
             'tb.querySelectorAll(".logo,.brand,#livedot,#lang-slot,.app-title .brand").forEach(function(e){e.style.display="none";});'
             '})();</script>')


_I18N = {"mt": None, "js": ""}


def i18n_inline():
    """web/i18n.js 를 <script> 로 (페이지가 따로 불러오지 않을 때 — 한/영 전환을 모든 화면에 같게)."""
    p = os.path.join(paths.REPO_ROOT, "web", "i18n.js")
    try:
        mt = os.path.getmtime(p)
        if mt != _I18N["mt"]:
            with open(p, encoding="utf-8") as f:
                _I18N.update(mt=mt, js=f.read())
    except OSError:
        return ""
    return "<script>" + _I18N["js"] + "</script>"


def with_home(html):
    add = HOME_LINK if "i18n.js" in html else i18n_inline() + HOME_LINK
    i = html.rfind("</body>")
    return fill(html[:i] + add + html[i:] if i >= 0 else html + add)
