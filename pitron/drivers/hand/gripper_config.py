"""
gripper_config.py — Dex1-1 그리퍼 설정 읽기

  from pitron.drivers.hand import gripper_config
  cfg = gripper_config.load()              # 장치 값만 (robots/h2/hands/dex1/dex1.yaml: 토픽 · q 범위 · TCP · 게인)
  cfg = gripper_config.load(app_yaml)      # + 앱 값 (예: apps/gripper_pick/gripper.yaml 의 pick · xr_view) — 같은 키는 장치 값 우선
  # 파일이 없거나 값이 틀리면 SystemExit — 기본값으로 대신하지 않음 (좌우 토픽이 뒤바뀌면 위험)
"""
import os
import sys

import yaml

from pitron import paths

PATH = os.path.join(paths.ROBOTS_DIR, "h2", "hands", "dex1", "dex1.yaml")


def _num_list(v, n):
    return isinstance(v, list) and len(v) == n and all(isinstance(x, (int, float)) for x in v)


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except OSError as e:
        sys.exit(f"[gripper] 설정 파일을 못 읽음: {path} ({e})")


def load(app_yaml=None, path=PATH):
    cfg = {**(_read(app_yaml) if app_yaml else {}), **_read(path)}
    t = cfg.get("topics") or {}
    if not (isinstance(t.get("left"), str) and isinstance(t.get("right"), str) and t["left"] != t["right"]):
        sys.exit(f"[gripper] {path}: topics.left / topics.right 가 없거나 같음")
    if not _num_list(cfg.get("q_range"), 2) or not (cfg["q_range"][0] < cfg["q_range"][1]):
        sys.exit(f"[gripper] {path}: q_range [닫힘, 열림] 필요")
    if not _num_list(cfg.get("tcp_offset"), 3):
        sys.exit(f"[gripper] {path}: tcp_offset [x, y, z] 필요")
    for k in ("kp", "kd", "max_vel", "rate"):
        if not isinstance(cfg.get(k), (int, float)) or cfg[k] < 0:
            sys.exit(f"[gripper] {path}: {k} 숫자 필요")
    return cfg
