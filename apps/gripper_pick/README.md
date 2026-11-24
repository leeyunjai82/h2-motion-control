# apps/gripper_pick — Dex1-1 그리퍼 (한 팔로 작은 박스 잡기)

박스 양손 잡기(`apps/box_handover/robot_server.py`, `start_robot.sh`)와 따로 씁니다. 같이 쓰는 것은 arm_server(팔 명령을 혼자 쥐는 서버)와 카메라 보정값(`robots/h2/robot.yaml`, `calib_local.yaml`)뿐입니다.

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
ROBOT_CHECK=1 ./start_fsm.sh stand     # 거치대 + FSM 703 (한 번)
./start_gripper.sh                     # 한 창: 그리퍼 서비스(sudo 비밀번호 한 번) → arm_server → 잡기 웹. Ctrl+C 로 셋 다 끔
# 브라우저: http://<미니 PC>:50030/
```

1. 손 선택 → **[준비 자세로]**: 그 팔만 관절 보간 (경유 자세를 거치면 약 10 초). 준비 자세 = 집게 가운데 (0.30, ±0.35, 0.20) ·
   숙임 0 (`gripper.yaml pick.ready_tcp_m`) — 카메라 화면 밖. 기본 자세에서 바로 가면 그리퍼가 몸 앞 낮은 곳을 지나가서
   경유 자세 (0.20, ±0.30, 0.25)·20° 를 거침 → 몸 앞 0.25 m 넘는 곳에서 그리퍼 가장 낮은 점 z 0.167 (계산)
2. **[테이블 점 추가]** → 테이블 빈 곳 클릭, 앞 · 뒤 · 옆 2–3 곳 (점마다 31 × 31 픽셀 × 10 프레임 깊이 중앙값) → 기울어진 평면으로 맞춤.
   몸통이 뒤로 약 4° 기울어 서 있어 (check_rsimu 2026-11-06) 로봇 좌표에서 테이블이 앞으로 10 cm 에 약 7 mm 낮아 보임 — 1 곳이면 수평으로 봄.
   테이블을 옮기면 [테이블 지우기] 후 다시. 카메라 숙임은 start_gripper.sh 가 시작 때 재서 기준값과 비교만 (저장 안 함 — ACT 는 카메라 구도가 같아야 하므로 거치대를 기준에 맞춤, 1° 넘게 다르면 멈춤)
3. 잡는 폭 · 높이 [mm], RPY (기본 0, 20, 0), 놓는 곳 (통 · 제자리), 통 높이 [mm, 테이블 → 테두리]
   → ① 박스 윗면 가운데 클릭 → ② (통이면) 통 **입구** 가운데 클릭 → 계획 (팔 안 움직임)
   - 위치는 깊이를 안 씀: 클릭 픽셀 시선 ∩ 평면 (박스 = 테이블 + 박스 높이, 통 = 테이블 + 통 높이). 깊이로 잰 점과의 차이는 참고로만 표시
4. [실행] → 단계별이면 [다음 단계] 마다 한 단계
   - 통: 그리퍼 벌림 → 접근(10 cm 위) → 내려가기 → 닫기 → 들기(박스 아랫면이 테두리 3 cm 위) → 통 위로 →
     박스 아랫면이 테두리 1 cm 위까지 → 벌림(떨어뜨림) → 위로 → 준비 자세
   - 제자리: … 닫기 → 들기 8 cm → 2 초 → 내려놓기 → 벌림 → 빼기 → 준비 자세
   - 통 테두리 높이 80 · 120 mm 는 계획 OK, 150 mm 는 통 위에서 손목 한계로 안 닿음 (오프라인, 박스 x 0.40–0.48)
5. [정지] = 그 자리에서 멈춤 (그리퍼는 그 상태 유지) → [복귀] = 위로 → 준비 자세
6. 끝나면 [기본 자세로] (같은 경로 반대)

## 녹화 → LeRobot v3 — `pitron/learning/recorder.py` · `lerobot_sync.py`

수집 화면: 데이터셋 이름 → [수집 켬] → 시연 → 에피소드마다 **[저장] / [버리기]**. 새 데이터셋은 처음 저장할 때 아래 설정으로 만들어지고 그 뒤로 고정:
카메라 크기 (320×240 · 424×240 · 640×480 · 카메라별 원래 크기) · 맞춤 방식 (자르기 · 검은 띠 · 눌러 줄이기, `pitron/learning/camfit.py`) · 작업 설명 (바꾸면 v3 를 다시 만듦).
모든 카메라가 같은 크기여야 하는 정책 (예: diffusion) 이 있어 기본은 320×240 · 자르기.

```
data/datasets/<데이터셋>/info.json                       원본 녹화 (format h2-gripper-episodes-v1, cameras · fit · task · state_names)
data/datasets/<데이터셋>/episodes/ep_000001/…            원본 에피소드 (npz · meta) — 수집 화면의 에피소드 보기 · 지우기
data/datasets/<데이터셋>/lerobot/                        LeRobot v3.0 (성공 에피소드만) — 다운로드 · 학습은 이것
```

- 저장 · 지우기 · 작업 설명 바꿈 → 백그라운드에서 lerobot 가상환경 프로세스가 `lerobot/` 를 맞춤 (새 에피소드는 이어 붙이고, 지웠으면 다시 만듦, nice 10)
- 켤 때 예전 데이터셋도 v3 로 맞춤. 영상은 h264 (브라우저 · 윈도우 뷰어에서 바로 재생)
- v3 내용: `observation.state` (16) · `action` (16) · `observation.images.<카메라>` · `tcp` (6) · `stage` (분석용, 학습 입력 아님) · `meta/h2.json` (에피소드별 손 · 결과 · 단계)
- state = [왼팔 7 실측, 왼 그리퍼 q, 오른팔 7 실측, 오른 그리퍼 q] · action = [왼팔 7 지령, 왼 그리퍼 목표, 오른팔 7 지령, 오른 그리퍼 목표]
- 확인: 컨테이너에서 녹화 → v3 (처음 · 이어 붙이기 · 지운 뒤 다시) · zip · lerobot 학습 (2026-11-24). 실기 녹화는 아직

## 정책 실행 (lerobot 모델) — `policy.py` · `pitron/learning/lerobot_infer.py`

수집 화면 추론 칸: 모델 고르기 (이 PC 에서 학습한 모델은 자동으로 목록에, 다른 PC 모델은 zip 올리기) · 장치 → [불러오기] → 시작 조건 OK → [▶ 정책 시작].
모델 = lerobot `pretrained_model` 폴더 (+ `h2_model.json`). 정책 종류 상관없음 — lerobot 표준대로 (`config.json` type → 정책 클래스 · 정규화 · `select_action`).
ACT 는 OpenVINO 로 (CPU · GPU · NPU, 변환 결과는 `<모델>/openvino/` 캐시), 나머지는 torch (CPU · Intel GPU). 데이터셋은 안 씀.

- 시작 조건: 양팔 준비 자세 (수집과 같은 시작) · 카메라 (모델이 쓰는 것 전부) · 팔 실측 · 그리퍼 상태 · 카메라 숙임 ±1° · 관절 순서 (h2_model.json)
- 제어 30 Hz: 한 틱에 추론 요청 하나. 정책이 chunk 를 새로 계산하는 동안은 마지막 목표를 다시 보내 제자리에서 기다림 (arm_server 세션 유지)
- 멈춤 (안전): 정책 출력이 모델의 학습 범위 (action min/max) 를 0.15 rad 넘게 벗어남 · 집게 가운데가 작업 범위 밖 (`config/local.yaml policy_tcp_box`, 없으면 안 봄) ·
  그리퍼가 테이블 평면 + 5 mm 아래 · 지령 ↔ 실측 15° · 추론 답 15 초 없음 · 최대 90 초
- 한 회 끝: 8 초 뒤 지령이 양팔 준비 자세에 1 초 머묾 (반복이면 2 초 뒤 다음 회)
- 영상은 수집과 같게 (모델 입력 크기로 camfit → JPEG q90 → RGB)
- 확인 (컨테이너 CPU, 가짜 팔 서버): ACT OpenVINO 첫 추론 85 ms · 지령 주기 33 ms · torch 와 차이 1.5e-6 /
  diffusion torch chunk 6.6 초 동안 제자리 대기 후 이어감 · 추론 대기 한도 넘으면 멈춤 (2026-11-24). 미니 PC · 실기는 아직

카메라만 확인 (로봇 꺼짐 · 충전 중): `source activate_tv.sh && python apps/gripper_pick/pick_web.py --cam-only` → D435i · 손목 카메라 화면만.

로봇 없이 화면 · 계획만: `./start_gripper.sh --sim` (fake_robot · arm_server · 잡기 웹, 가짜 카메라 · 그리퍼).

## VR 연결 · 설정 (수집 화면 "VR 연결" 카드)

- `./start_collect.sh` (홈 → VR 수집) 로 켜면 잡기 웹이 Quest 연결 프로그램 (`xr_virtual.py --real`, Vuer :8012) 을 켜고 adb 로 Quest 페이지를 엶
- 끊기면: [재연결] (연결 프로그램 다시 켜기) · [Quest 페이지 열기] (adb reverse + 헤드셋 브라우저) · [끊기] — VR 조작 중에는 막힘
- 상태: 연결 프로그램 (Vuer 포트) · Quest (USB, adb) · Quest 입력 (헤드셋에서 버튼을 눌러 입력이 들어오는지, 1 초마다 신호)
- 화면 방식: pass-through (주변 보임, 기본) · immersive (카메라 영상) — 바꾸면 다시 연결
- [설정 ▸]: 손 오프셋 · 그리퍼 벌림 · 관절 속도 기본값 · 양팔 최소 거리 · 뒤 한계 · 테이블 앞 끝. 기본값 = `gripper.yaml xr_view`,
  바꾼 값 = `data/settings/vr.json` (git 제외 — pull 충돌 없음). 다음 [● 시작] 부터 적용, 조작 중에는 못 바꿈. 범위 밖 값은 저장 안 됨 (`vr_settings.py FIELDS`)
- 확인: 컨테이너 가짜 로봇에서 설정 저장 · 범위 검사 · 기본값 · 세션 시작 때 다시 읽기 · 연결 실패 이유 표시 (2026-11-24). Quest · adb 실기는 아직

## xr_teleoperate 가상 실행 — `xr_virtual.py` + `xr_viewer.py` (로봇 명령 없음)

Unitree xr_teleoperate 의 Quest 입력 처리 (televuer) 와 H2 IK (`H2_ArmIK`) 를 그대로 쓰고, 로봇 대신 가상 H2 3D 뷰어로 보냄.
xr_teleoperate H2 URDF 와 이 저장소 URDF 는 팔 관절 이름 · 원점 · 한계가 같음 (2026-11-09 비교).

준비 (한 번): `git clone --recurse-submodules https://github.com/unitreerobotics/xr_teleoperate ~/xr_teleoperate`,
`cd ~/xr_teleoperate/teleop/televuer && pip install -e .`, `pip install 'vuer[all]==0.0.60' "params-proto>=2.13,<3"`
(params-proto 3.x 는 vuer 0.0.60 과 안 맞음), 인증서 `~/.config/xr_teleoperate/{cert,key}.pem`.

```bash
python apps/gripper_pick/xr_viewer.py --table-z -1.0                  # 터미널 1: 뷰어 http://localhost:50040/
python apps/gripper_pick/xr_virtual.py --xr ~/xr_teleoperate          # 터미널 2: Vuer :8012 + H2 IK → 뷰어
adb reverse tcp:8012 tcp:8012                               # Quest USB
adb shell am start -a android.intent.action.VIEW -d "https://localhost:8012/?ws=wss://localhost:8012" com.oculus.browser
```
- 조작은 xr_teleoperate 와 같음 (헤드셋을 머리에 쓰고 컨트롤러 쥔 손 = 로봇 손, 머리 기준 절대 위치 · 배율 없음, 트리거 = 그리퍼)
- 뷰어: H2 URDF + Unitree Dex1-1 메시 (`dex1_1/`, BSD-3) + 사용자 어댑터 STL (`adapter/adapter.stl`) — 손목 앞면 57.7 mm 부터 기본 손 잘라 냄,
  어댑터 4 mm 만 손목 밖, Dex1 원점 = tcp_offset 0.1844 − 0.123
- 우리 안전 검사 (몸통 · 테이블 · 다른 팔 · 뒤 한계, gripper.yaml `xr_view`) 를 "실제였으면 멈춤" 으로 표시만 함 — xr_teleoperate 에는 이 검사가 없음
- xr_teleoperate 를 실제 로봇에 쓸 때 주의 (소스 확인): H2 관절 속도 제한 기본 30 rad/s · 종료 시 팔 관절 0 으로 이동 (go_home) · rt/lowcmd 직접 송신
  (arm_server 와 동시 사용 금지) · 녹화 형식이 우리 trainer 와 다름

## VR 조작 — 실제 팔 (`xr_real.py`, 잡기 웹 [VR 시작])

xr_teleoperate 입력 + H2 IK (`xr_virtual.py --real`) → 잡기 웹 `/api/xr/cmd` → 안전 검사 → arm_server `/stream` · Dex1.
xr_teleoperate 의 로봇 제어부 (속도 30 rad/s · 끌 때 0 자세) 는 쓰지 않음. 녹화는 잡기 웹 녹화기 그대로 ([VR 시작]~[VR 끝] = 한 에피소드,
meta side = 실제로 움직인 손).

```bash
./start_gripper.sh                                          # 터미널 1: 잡기 웹 (로봇 · 카메라)
python apps/gripper_pick/xr_viewer.py --table-z <테이블 z>            # 터미널 2 (선택): 가상 뷰어 — 보낸 목표를 같이 그림
python apps/gripper_pick/xr_virtual.py --xr ~/xr_teleoperate --real   # 터미널 3: Quest 입력 → 잡기 웹 (세션 열릴 때만 보냄)
adb reverse tcp:8012 tcp:8012                               # Quest: https://localhost:8012/?ws=wss://localhost:8012 → 'Virtual Reality'
```
잡기 웹: 테이블 재기 → (수집 켬) → VR 조작 칸 속도 (0.2 / 0.3 / 0.5 rad/s) → [VR 시작] → 조작 → [VR 끝] → 저장 / 버리기.

- Quest 화면: 머리 카메라 + 아래 양쪽 손목 카메라 (잡기 웹 영상), 위에 상태 (ROBOT ON / STOP: R:TABLE …)
- 그립(중지) 누른 손만 팔이 움직임 · 뗀 손은 그 자리. 그리퍼는 트리거만 따름: 놓음 75 mm ~ 끝까지 42 mm (물건 50 mm 고정, pick.object_mm), 한 번에 지금 위치보다 8 mm 까지만 조임
- 시작 조건: 실기 · 다른 동작 없음 · arm_server hold · 테이블 잼 · 카메라 숙임 · 485 보드 · 앞 에피소드 결정
- 손별 멈춤: 몸통 (body_clear_m) · 테이블 (잰 평면 + tip_clear_mm, x > xr_view.table_x_min_m) · 다른 팔 (xr_view.arm_gap_m) ·
  뒤 (xr_view.back_limit_m) · IK 튐 (0.5 rad) → 그 손은 그 명령을 버리고 그 자리
- 세션 끝: [VR 끝] · [정지] · 입력 1.5 초 없음 · 명령 · 실측 차이 > track_abort_deg · arm_server 거부 → 팔은 마지막 목표에서 멈춤
- 확인: 가짜 arm_server · 가짜 xr_teleoperate (IK · Quest 입력) · 잡기 웹 --sim 으로 끝까지 (세션 전 송신 0, 데드맨, 그리퍼 유지,
  테이블 · 몸통 · 뒤 멈춤, 정지 버튼, 입력 끊김, 녹화 meta side). **실기 · 실제 Quest · 실제 H2_ArmIK 로는 아직**
