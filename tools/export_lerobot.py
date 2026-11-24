#!/usr/bin/env python3
"""
export_lerobot.py — 예전 수집 데이터셋 (폴더 · zip) → LeRobot v3.0 새 폴더. lerobot 가상환경의 python 으로.

    python tools/export_lerobot.py old_dataset.zip --out ~/lerobot_data/old --task "pick up the cube"
    옵션: --all (실패 에피소드 포함) · --vcodec h264|libsvtav1 · --repo-id local/old

  수집 PC 의 데이터셋은 자동으로 data/datasets/<이름>/lerobot 에 만들어지므로 이 도구는 다른 곳에 내보낼 때만.
  변환 내용은 pitron/learning/lerobot_sync.py 와 같음.
"""
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pitron.learning import lerobot_sync  # noqa: E402

if __name__ == "__main__":
    if "--out" not in sys.argv:
        sys.exit("--out 폴더를 주세요 (수집 PC 데이터셋은 자동으로 <데이터셋>/lerobot 에 만들어짐)")
    lerobot_sync.main()
