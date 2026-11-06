#!/usr/bin/env python3
"""
pick_report.py — 잡기 웹 기록(logs/pick_YYYYMMDD.jsonl) 요약: 실행마다 결과 · 박스 위치 · 닫은 간격 · 도착 오차

  python gripper/pick_report.py                 # 오늘
  python gripper/pick_report.py 20261106        # 날짜 지정
  python gripper/pick_report.py logs/pick_20261106.jsonl

판단 기준 (박스 폭 W, 닫기 목표 W − squeeze)
  · 닫은 간격 ≈ W (W − 3 ~ W + 2 mm) — 박스 양옆을 똑바로 쥠
  · W + 2 mm 보다 넓음 — 비스듬히 · 모서리를 쥠 (위치 · 각도 어긋남)
  · 목표 근처 — 빗나감 (웹이 '놓침' 으로 멈춤)
  · 도착 오차 — 팔이 보낸 자리에 간 정도 (관절각 FK). 몇 mm 면 팔은 정확, 남는 어긋남은 카메라 · 보정 쪽
"""
import datetime
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RE_CLOSE = re.compile(r"그리퍼 닫기: q (\S+) → ([\d.]+) \(≈ (\d+) mm, 목표 (\d+) mm")
RE_TAU = re.compile(r"토크 ([-\d.]+|None)")


def load(arg):
    if arg and arg.endswith(".jsonl"):
        path = arg
    else:
        day = arg or datetime.date.today().strftime("%Y%m%d")
        path = os.path.join(ROOT, "logs", f"pick_{day}.jsonl")
    if not os.path.isfile(path):
        sys.exit(f"기록 없음: {path}")
    recs = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    recs.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return path, recs


def main():
    path, recs = load(sys.argv[1] if len(sys.argv) > 1 else None)
    plans, runs = {}, []
    cur = None
    for r in recs:
        pid = r.get("plan_id")
        if "plan" in r and r["plan"]:
            plans[pid] = r["plan"]
        ev = r.get("event", "")
        if ev.startswith("실행 #"):
            cur = {"plan_id": pid, "t": r.get("t"), "close": None, "arrivals": [], "result": None}
            runs.append(cur)
        if cur is None:
            continue
        m = RE_CLOSE.search(ev)
        if m:
            cur["close"] = (float(m.group(3)), float(m.group(4)))
            mt = RE_TAU.search(ev)
            cur["tau"] = mt.group(1) if mt else None
        if "arrival" in r:
            cur["arrivals"].append((r["arrival"], r.get("err_mm")))
        if "result" in r:
            cur["result"] = r["result"]
            cur = None

    print(f"{path} — 실행 {len(runs)} 번\n")
    n_ok = 0
    for i, run in enumerate(runs, 1):
        p = plans.get(run["plan_id"]) or {}
        w = p.get("width_mm")
        top = p.get("top")
        when = datetime.datetime.fromtimestamp(run["t"]).strftime("%H:%M:%S") if run["t"] else "?"
        res = {"done": "끝까지", "missed": "놓침", "stopped": "멈춤", "error": "오류", None: "기록 끊김"}.get(run["result"], run["result"])
        verdict = ""
        if run["close"] and w:
            gap, tgt = run["close"]
            if gap <= tgt + 3:
                verdict = "빗나감"
            elif gap <= w + 2:
                verdict = "똑바로 쥠"
            else:
                verdict = f"넓게 쥠 (+{gap - w:.0f} mm, 비스듬히?)"
        good = run["result"] == "done" and verdict == "똑바로 쥠"
        n_ok += good
        arr = [e for name, e in run["arrivals"] if e and name in ("접근 위치로", "내려가기", "통 위로", "통에 넣는 높이로")]
        worst = max((max(abs(v) for v in e) for e in arr), default=None)
        print(f"#{i:2d} {when} {p.get('side', '?'):5s} 박스 {top and [round(v, 3) for v in top]}  폭 {w}  "
              f"→ {res:5s} | 닫은 간격 {run['close'][0] if run['close'] else '-'} mm 토크 {run.get('tau', '-')} {verdict:12s} | "
              f"도착 오차 최대 {worst if worst is not None else '-'} mm {'✓' if good else ''}")
    if runs:
        print(f"\n제대로 끝난 실행 (끝까지 + 똑바로 쥠): {n_ok} / {len(runs)}")


if __name__ == "__main__":
    main()
