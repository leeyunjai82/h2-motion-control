"""
paths.py — 저장소 폴더 위치 (표준 라이브러리만 · import 해도 아무 일 없음 — 학습 PC · Windows 에서도 씀)

  robots/   로봇 · 손 정의 (URDF · 메시 · yaml)
  models/   AI 모델 파일 (git)
  web/      공용 웹 (three.js · i18n.js)
  data/     수집 데이터 · 받은 모델 · 학습 작업 (git 제외)
  logs/     실행 기록 (git 제외)
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOTS_DIR = os.path.join(REPO_ROOT, "robots")
MODELS_DIR = os.path.join(REPO_ROOT, "models")
WEB_DIR = os.path.join(REPO_ROOT, "web")
VENDOR_DIR = os.path.join(WEB_DIR, "vendor")
DATA_DIR = os.path.join(REPO_ROOT, "data")
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
I18N_JS = os.path.join(WEB_DIR, "i18n.js")


def data_dir(name, legacy=None):
    """data/<name>. 새 위치가 아직 없고 옛 위치(legacy, 저장소 기준)에 내용이 있으면 옛 위치를 그대로 씀 (데이터를 옮기기 전에도 동작)."""
    new = os.path.join(DATA_DIR, name)
    if legacy and not os.path.exists(new):
        old = os.path.join(REPO_ROOT, legacy)
        if os.path.isdir(old) and os.listdir(old):
            print(f"[paths] ⚠️ 옛 위치 사용 중: {legacy} — 옮기려면: mkdir -p data && mv {legacy} data/{name}", file=sys.stderr)
            return old
    return new
