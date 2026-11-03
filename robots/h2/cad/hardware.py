#!/usr/bin/env python3
"""
H2 카메라 거치대 + 등판 손잡이 구매 볼트·너트를 STEP 하나로 (보관용, 나사산 생략).

  pip install cadquery          # 2.8 에서 확인
  python hardware.py            # → h2_hardware.step (이 폴더)

구매 목록 (등판 손잡이까지 달 때 — camera_bracket/README.md · back_handle/README.md 볼트 절)
  M6×50 접시머리 렌치볼트 4  등판 위 2 + 아래 2 (요크 + 손잡이 같이 조임)
  M6×40 접시머리 렌치볼트 4  요크 발 ↔ 가로대 (M6×50 은 가로대 막힌 구멍 끝 |y| 87 을 8.25 mm 넘어 못 씀)
  M6 육각 너트            4  가로대 끝 너트 홈 10.4 × 5.4 → 맞변 10 · 두께 5 이하
  (M3×5 2 개는 카메라 뒷면 — 가지고 있는 것 사용, 이 파일에 없음)

모양: 볼트는 bracket.py 와 같은 치수 (사용자 M6.stl: 머리 Ø11.1 · 90°, 길이 = 머리 포함 전체, 렌치 4 mm).
      너트는 구매 목록대로 육각 (bracket.py 조립 모델은 사용자 M6.stl 사각 10 × 10 × 5 — 홈은 둘 다 들어감).
배치: 볼트는 머리를 위로 세워 줄마다 20 mm 간격 (1 줄 M6×50, 2 줄 M6×40), 3 줄 너트. 단위 mm.
"""
import math
import os

import cadquery as cq

D = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(D, "h2_hardware.step")
PITCH, ROW = 20.0, 25.0
STEEL = cq.Color(0.75, 0.75, 0.78)


def csk_screw(length, head_d=11.1, dia=6.0, af=4.0):
    """접시머리 렌치볼트, 머리 윗면 중심이 원점 · 나사는 −z (bracket.py csk_screw 와 같은 모양)."""
    hh = (head_d - dia) / 2                              # 90° 접시
    head = cq.Workplane().add(cq.Solid.makeCone(head_d / 2, dia / 2, hh, cq.Vector(0, 0, 0), cq.Vector(0, 0, -1)))
    shank = cq.Workplane("XY").circle(dia / 2).extrude(-length)
    sock = cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(-hh * 0.9)
    return shank.union(head).cut(sock)


def hex_nut(af=10.0, t=5.0, hole=6.0):
    """M6 육각 너트 (맞변 af, 두께 t), 아랫면이 z 0."""
    return cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).circle(hole / 2).extrude(t)


ITEMS = [  # (이름, 모델, 줄, 개수)
    ("M6x50_csk_back_plate", csk_screw(50.0), 0, 4),
    ("M6x40_csk_yoke_bar", csk_screw(40.0), 1, 4),
    ("M6_hex_nut", hex_nut(), 2, 4),
]

asm = cq.Assembly(name="h2_hardware")
for name, part, row, n in ITEMS:
    for i in range(n):
        asm.add(part.translate((i * PITCH, -row * ROW, 0)), name=f"{name}_{i + 1}", color=STEEL)
    bb = part.val().BoundingBox()
    print(f"{name:22s} × {n}  {bb.xlen:4.1f} × {bb.ylen:4.1f} × 길이 {bb.zlen:4.1f} mm")
asm.export(OUT)
print("ok →", OUT)
