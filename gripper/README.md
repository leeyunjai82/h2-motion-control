# gripper/ — Dex1-1 그리퍼 (한 팔로 작은 박스 잡기)

박스 양손 잡기(`common/robot_server.py`, `start_robot.sh`)와 따로 씁니다. 같이 쓰는 것은 arm_server(팔 명령을 혼자 쥐는 서버)와 카메라 보정값(`robots/h2/robot.yaml`, `calib_local.yaml`)뿐입니다.

| 파일 | 하는 일 | 확인 |
| --- | --- | --- |
| `gripper.yaml` | 손 ↔ 토픽, q 범위, TCP, 송신 게인, 잡기 값(`pick:`) | 토픽 · q 범위 실측, TCP · mm/rad 는 계산값 |
| `check_dex1.py` | 그리퍼 상태 읽기 (명령 없음) | 실기 |
| `move_test.py` | 그리퍼만 천천히 여닫기 | 실기 (로봇에 단 상태 포함) |
| `dex1_gripper.py` | 그리퍼 연속 송신 클래스 (pick_web 이 씀) | 실기 안 해 봄 |
| `pick_core.py` | IK · 경로 계획 · arm_server `/stream` 실행 | 오프라인 + 가짜 arm_server 만 |
| `pick_web.py` | 잡기 웹 (포트 50030) | 가짜 카메라 · 가짜 그리퍼 · 가짜 arm_server 만 |

## 잡기 웹

```bash
ROBOT_CHECK=1 ./start_fsm.sh stand            # 거치대 + FSM 703
ROBOT_CHECK=1 ./start_editor.sh real          # arm_server (hold · weight 1)
cd ~/project/dex1_1_service/bin && sudo ./dex1_1_gripper_server -n enp45s0    # 다른 터미널
source activate_tv.sh && python gripper/pick_web.py enp45s0                   # 다른 터미널, ROBOT_CHECK 없이
# 브라우저: http://<미니 PC>:50030/
```

1. 손 선택 → **[준비 자세로]**: 그 팔만 관절 보간 (약 4.4 초). 기본 자세에서는 그리퍼가 아래를 봐서 이동 중 몸 앞
   x 0.25–0.46 m 에서 pelvis 아래 약 15 cm 까지 내려감 (계산) → **테이블이 그 경로에 없을 때만**. 확인 창에 값이 나옴
2. 박스 크기 (기본 50 mm), RPY (기본 0, 20, 0) → 박스 윗면 가운데 클릭 → 계획 (팔 안 움직임)
3. [실행] → 단계별이면 [다음 단계] 마다 한 단계: 그리퍼 벌림 → 접근 위치(10 cm 위) → 내려가기 → 닫기 →
   들기 8 cm → 2 초 → 내려놓기 → 벌림 → 빼기 → 준비 자세
4. [정지] = 그 자리에서 멈춤 (그리퍼는 그 상태 유지) → [복귀] = 위로 → 준비 자세
5. 끝나면 [기본 자세로] (같은 경로 반대 — 테이블 확인)

로봇 없이 화면 · 계획만: `python gripper/pick_web.py --sim --sim-top-z 0.14` (arm_server 는 `./start_editor.sh virtual` 로).

### 계산으로 정한 것 (실물 확인 전)

- H2 손목 pitch ±33° (URDF) + 집게 가운데가 손목에서 19.1 cm → 위에서 수직으로(pitch 90°)는 앞쪽 어디서도 IK 가 안 풀림.
  테이블 110 cm (pelvis +0.09) · 박스 5–6 cm 에서 pitch 10–20° 면 x 0.35–0.50 · y ±0.10–0.30 모두 계획 OK → 기본 (0, 20, 0)
- 접근은 항상 잡는 자리 바로 위에서 수직으로. 잡는 높이 = 박스 가운데, 그리퍼 몸통(높이 67 mm)이 바닥에 10 mm 안쪽으로
  오면 그만큼 올림. 패드가 박스와 15 mm 미만 겹치면 계획 거부
- 박스 6 cm 면 벌림 최대 약 79 mm 에서 한쪽 여유 약 10 mm → 5 cm 를 권장 (한쪽 12.5 mm)
- 닫기: 박스 − 8 mm 를 목표로 kp 5 로 밈. 닫은 뒤 간격이 목표 + 3 mm 보다 크면 잡음, 아니면 놓침 → 벌리고 멈춤.
  쥐는 힘은 모름
- 팔 관절 속도: 계획 최고 0.4 rad/s, arm_server 스트림 상한 1.0 rad/s. 명령 ↔ 실측 15° 넘게 벌어지면 멈춤
- 기록: `logs/pick_YYYYMMDD.jsonl` (계획 · 단계 · 결과)
