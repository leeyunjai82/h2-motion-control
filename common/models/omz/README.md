# OpenVINO Open Model Zoo 모델 (머리 카메라 head_track)

| 파일 | 용도 | 입력 | 출처 |
| --- | --- | --- | --- |
| `face-detection-retail-0004.xml/.bin` | 얼굴 | 1×3×300×300 BGR | Intel Open Model Zoo 2023.0, FP16 |
| `person-detection-0201.xml/.bin` | 사람 | 1×3×384×384 BGR | Intel Open Model Zoo 2023.0, FP16 |
| `age-gender-recognition-retail-0013.xml/.bin` | 왼눈 얼굴 → 나이·성별 | 1×3×62×62 BGR | Intel Open Model Zoo 2023.0, FP16 |
| `person-detection-action-recognition-0006.xml/.bin` | 오른눈 사람 + 행동 6종 | 1×400×680×3 BGR (NHWC) | Intel Open Model Zoo 2023.0, FP16 |

- 얼굴·사람 출력: 1×1×200×7 = `[image_id, label, conf, x_min, y_min, x_max, y_max]` (0–1)
- 나이·성별: `fc3_a` = 나이/100, `prob` = [여, 남]. 얼굴 박스를 1.2 배 정사각형으로 넓혀 자름 (Intel interactive_face_detection 데모와 같음).
  18~75 세로 학습 (아이는 안 맞음). 문서: 성별 95.8 %, 나이 평균 오차 6.99 년
- 행동 0006: sitting, writing, raising_hand, standing, turned_around, lie_on_desk. 풀기 = Intel smart_classroom 데모
  (action_detector.cpp) 의 사전박스·분산·Soft-NMS. **행동 출력 [1,H,W,6] 은 (칸, 행동) 순서로 읽어야 함** — 데모 C++ 처럼
  (행동, 칸) 순서로 읽으면 이 모델 문서 예시 그림(앉음 4 · 손 듦 1)과 안 맞음 (2026-10-12 확인). 문서: 사람 AP 90.7 %, 행동 80.74 % (교실 시험셋)
- 받은 곳: `https://storage.openvinotoolkit.org/repositories/open_model_zoo/2023.0/models_bin/1/<이름>/FP16/<이름>.{xml,bin}` (2026-10-08)
- 라이선스: Apache-2.0 (Open Model Zoo)
- 사람 모델 고른 이유: `person-detection-retail-0013` 은 매장 천장 카메라용이라 눈높이 시험 사진에서 가운데 사람을 놓침 (0.05 미만),
  `person-detection-0201` 은 같은 사진 0.81, 절반 크기에서도 0.81. `person-detection-0200`(256) 은 절반 크기에서 놓침
- 받은 날: 나이·성별 · 0006 은 2026-10-12 (같은 주소)
- 쓰는 곳: `common/ctrl/ov_detect.py` (OvSSD · OvAgeGender · OvActions) → `common/head_track.py`
