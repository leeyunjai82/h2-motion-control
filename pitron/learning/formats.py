"""
formats.py — ACT 데이터셋 · 모델 zip 형식 (한 곳). 저장 (recorder) · 학습 (dataset · train_act) · 실행 (policy) · 보기 (act_viewer) 가 같이 씀.
  표준 라이브러리만 — 어디서 import 해도 아무 일 없음. 형식을 바꾸면 이름의 -v1 을 올리고 읽는 쪽을 같이 고칠 것.

데이터셋 (DATASET_FORMAT)
  <dataset>/info.json                       format · fps · cameras{이름: {w, h}} · state_names · created
  <dataset>/episodes/ep_000001/episode.npz  t(N,) · state(N,16) · action(N,16) · tcp(N,6) · stage(N,) ·
                                            jpeg_<카메라>(M,) uint8 (JPEG 이어 붙임) · jpeg_off_<카메라>(N+1,)
  <dataset>/episodes/ep_000001/meta.json    format · ok · result · side · fps · n · duration_s · cameras · state_names · stages …
  state · action 16 = STATE_LAYOUT (왼팔 7 · 왼 그리퍼 · 오른팔 7 · 오른 그리퍼). npz 는 pickle 없이 저장 · 읽기.

모델 zip (MODEL_FORMAT)
  <이름>/policy.xml · policy.bin   OpenVINO IR — 입력 state [1,D] · image_<카메라> [1,3,H,W] 0~1 RGB → action [1,chunk,D]
  <이름>/meta.json                 format · side · cameras · chunk_size · stats · start_pose · end_pose · 학습 정보
"""
import re

DATASET_FORMAT = "h2-gripper-episodes-v1"
DATASET_FORMAT_BOX = "h2-episodes-v1"       # 예전 박스용 (2026-11-06 삭제된 physical_ai) — 학습기 읽기만 남김
MODEL_FORMAT = "h2-gripper-act-v1"
MODEL_FORMAT_BOX = "h2-act-v1"

FPS = 30.0
STATE_LAYOUT = ("왼팔 7", "왼 그리퍼 q", "오른팔 7", "오른 그리퍼 q")
SIDE_IDX = {"left": list(range(0, 8)), "right": list(range(8, 16)), "both": list(range(16))}   # 손별 state · action 자리
MODEL_FILES = ("policy.xml", "policy.bin", "meta.json")
DATASET_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,40}$")
EPISODE_RE = re.compile(r"^ep_\d{6}$")
