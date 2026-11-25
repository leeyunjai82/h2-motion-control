"""
vr_settings.py — VR 조작 설정 (웹에서 바꿈). 기본값 = gripper.yaml xr_view, 웹에서 바꾼 값 = data/settings/vr.json (git 제외)

  웹에서 바꾼 값만 vr.json 에 저장 → git pull 로 gripper.yaml 기본값이 바뀌어도 충돌 없음. [기본값으로] = vr.json 지움.
  값은 VR 세션을 시작할 때 읽음 (조작 중에는 바꿀 수 없음). 범위 밖 값은 저장하지 않음.
"""
import json
import os
import threading

import os as _os, sys as _sys  # noqa: E401
_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", ".."))  # 저장소 루트 (pitron)
if _ROOT not in _sys.path:
    _sys.path.append(_ROOT)
from pitron import paths  # noqa: E402

import pick_core as pc  # noqa: E402

FILE = os.path.join(paths.data_dir("settings"), "vr.json")
_lk = threading.Lock()

# 키 → (설명, 단위, 최소, 최대) — 범위는 소프트웨어 한계 (안전 거리를 너무 줄이지 않게). 하드웨어 정격 아님
OBJ_MM = float(pc.P["object_mm"])
FIELDS = {
    "hand_offset_m": ("사람 손 → 로봇 손 위치 오프셋 [앞 · 바깥 · 위]", "m", -0.30, 0.30),
    "grip_open_mm": ("트리거를 놓았을 때 그리퍼 벌림", "mm", OBJ_MM + 5.0, round(pc.gap_mm(pc.Q_HI), 1)),
    "real_max_vel": ("관절 속도 기본값", "rad/s", 0.05, 1.0),
    "arm_gap_m": ("양팔 사이 최소 거리", "m", 0.10, 0.30),
    "back_limit_m": ("팔 · 그리퍼 뒤 한계 (pelvis x, 이보다 뒤면 그 손 멈춤)", "m", -0.20, 0.0),
    "table_x_min_m": ("테이블 앞 끝 (이보다 앞에 있는 그리퍼만 테이블 높이 검사)", "m", 0.05, 0.25),
}


def defaults():
    x = pc.CFG.get("xr_view") or {}
    miss = [k for k in FIELDS if k not in x]
    if miss:
        raise SystemExit(f"[vr] gripper.yaml xr_view: 에 {miss} 없음 — 기본값으로 대신하지 않음")
    return {k: ([float(v) for v in x[k]] if k == "hand_offset_m" else float(x[k])) for k in FIELDS}


def _saved():
    try:
        with open(FILE, encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def check(k, v):
    """→ 정리한 값 (범위 밖이면 ValueError)."""
    if k not in FIELDS:
        raise ValueError(f"모르는 설정: {k}")
    _, unit, lo, hi = FIELDS[k]
    if k == "hand_offset_m":
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            raise ValueError("손 오프셋은 숫자 3 개 [앞, 바깥, 위]")
        try:
            out = [round(float(x), 3) for x in v]
        except (OverflowError, ValueError, TypeError):
            raise ValueError("손 오프셋은 숫자 3 개 [앞, 바깥, 위]")
        if any(isinstance(x, bool) for x in v) or any(not (lo <= x <= hi) for x in out):
            raise ValueError(f"손 오프셋은 {lo} ~ {hi} {unit}")
        return out
    try:
        x = round(float(v), 3)
    except (OverflowError, ValueError, TypeError):
        raise ValueError(f"{FIELDS[k][0]}: 숫자가 아님")
    if isinstance(v, bool) or not (lo <= x <= hi):
        raise ValueError(f"{FIELDS[k][0]}: {lo} ~ {hi} {unit}")
    return x


def get():
    """기본값 + 웹에서 바꾼 값 (범위 밖으로 저장된 값은 무시하고 기본값)."""
    d = defaults()
    for k, v in _saved().items():
        try:
            d[k] = check(k, v)
        except (ValueError, TypeError):
            pass
    return d


def save(changes):
    """changes {키: 값} → 검사 후 vr.json 에 더함. → 지금 설정"""
    clean = {k: check(k, v) for k, v in (changes or {}).items()}
    with _lk:
        cur = _saved()
        cur.update(clean)
        base = defaults()
        cur = {k: v for k, v in cur.items() if k in FIELDS and v != base[k]}     # 기본값과 같으면 지움
        os.makedirs(os.path.dirname(FILE), exist_ok=True)
        with open(FILE + ".tmp", "w", encoding="utf-8") as f:
            json.dump(cur, f, ensure_ascii=False, indent=1)
        os.replace(FILE + ".tmp", FILE)
    return get()


def reset():
    with _lk:
        try:
            os.remove(FILE)
        except OSError:
            pass
    return get()


def describe():
    """화면용: 값 · 기본값 · 범위 · 바꿨는지."""
    cur, base = get(), defaults()
    return {k: {"value": cur[k], "default": base[k], "label": FIELDS[k][0], "unit": FIELDS[k][1], "min": FIELDS[k][2],
                "max": FIELDS[k][3], "changed": cur[k] != base[k]} for k in FIELDS}
