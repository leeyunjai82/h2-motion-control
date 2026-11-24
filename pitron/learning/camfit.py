"""
camfit.py — 카메라 영상을 정해진 크기로 맞추기 (수집 · 추론 공용, 같은 함수라 학습 영상과 추론 영상이 같게 됨)

  crop    비율 유지, 짧은 쪽을 맞춘 뒤 가운데를 잘라냄 (시야 일부 잃음)
  pad     비율 유지, 긴 쪽을 맞춘 뒤 남는 곳을 검은 띠로 (빈 픽셀)
  stretch 비율 무시, 그대로 늘이거나 줄임 (예전 녹화 방식)
"""
import cv2
import numpy as np

MODES = ("crop", "pad", "stretch")
LABEL = {"crop": "자르기", "pad": "검은 띠", "stretch": "눌러 줄이기"}


def fit(img, w, h, mode="stretch"):
    """img (H,W,3) → (h,w,3). 줄일 때 INTER_AREA, 늘일 때 INTER_LINEAR."""
    w, h = int(w), int(h)
    ih, iw = img.shape[:2]
    if (iw, ih) == (w, h):
        return img
    if mode not in MODES:
        raise ValueError(f"맞춤 방식 {mode} — {MODES} 중 하나")
    if mode == "stretch":
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA if iw * ih > w * h else cv2.INTER_LINEAR)
    s = max(w / iw, h / ih) if mode == "crop" else min(w / iw, h / ih)
    nw, nh = max(1, round(iw * s)), max(1, round(ih * s))
    r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    if mode == "crop":
        x, y = (nw - w) // 2, (nh - h) // 2
        return np.ascontiguousarray(r[y:y + h, x:x + w])
    out = np.zeros((h, w, img.shape[2]), img.dtype)
    x, y = (w - nw) // 2, (h - nh) // 2
    out[y:y + nh, x:x + nw] = r
    return out
