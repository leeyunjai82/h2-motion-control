# H2 Motion Control

**Unitree H2** 휴머노이드로 박스를 잡아 내려놓거나 건네는 시연 패키지입니다.
외장 Intel mini PC(Ubuntu 24.04) + 가슴의 Intel RealSense D435i(박스 인식) + H2 머리 쌍안 카메라(얼굴·나이성별·행동 보기)로 구성하고,
기능별 작은 HTTP 서버로 나눠 돌립니다.

- H2 전용입니다. 로봇 파일·설정은 `robots/h2/`, 공통 코드는 `common/`.
  `ROBOT` 환경변수는 안 줘도 됩니다 (기본 `h2`, 다른 값은 거부). G1 코드는 git 이력에 있습니다.
- 설치 : [**INSTALL.md**](./INSTALL.md) · 가상환경 다시 만들기 : [**REBUILD_ENV.md**](./REBUILD_ENV.md) (스냅샷 `utils/env_snapshot.sh`)
- H2 실측·근거·확인 필요 목록 : [**robots/h2/FACTS.md**](./robots/h2/FACTS.md)
- 카메라 거치대·등판 손잡이 (3D 출력) : [`robots/h2/cad/`](./robots/h2/cad/) — `h2_print_all.step` 에 출력 부품 전부

---

## ⚠️ 안전 수칙 (먼저 읽기)

1. **거치대 고정** — 로봇을 거치대(스탠드)에 걸고 시작합니다.
2. **`./start_fsm.sh stand`** 는 즉시 모터에 힘이 들어갑니다 (1 Damp → 5초 → 4 FixStand → 10초 → 703 PhaseWalk). **사람이 로봇을 붙잡고** 실행하세요.
3. 끝낼 때는 **`./start_fsm.sh damp`** (거치대에 건 채로 — sit 은 쓰지 않음).
4. 작업 공간에 사람·장애물이 없게 하고, **비상정지(E-STOP)** 위치를 미리 확인하세요.
5. 연결된 로봇이 H2 인지 서버가 기동 시 확인합니다 (`robot.yaml identity.mode_machine`, 다르면 팔·FSM 명령 거부).

---

## 구성

자원(하드웨어)마다 소유 프로세스는 하나. 나머지는 HTTP 로 통신합니다.

```
rs_stream      :50001   RealSense D435i (단독 점유) → MJPEG / depth API
detect_box     :50010   박스 인식(YOLO seg, OpenVINO GPU), 파지점 (rs_stream 사용)
arm_server     :50022   rt/arm_sdk 단독 점유 — 팔, IK, hold/release
head_track     :50013   머리 카메라 인식 — 왼눈 얼굴+나이·성별(NPU), 오른눈 행동 앉음·손 듦·서 있음 OMZ 0006(NPU), 보기 전용
robot_server   :50000   잡기 시퀀스 + 제어 웹 UI (CPU·GPU·NPU 사용률, System 카드 = 로봇 FSM·서버 상태)
dashboard      :50003   3D URDF 뷰어 / 관절 상태 (rt/lowstate 읽기 전용)
simulator      :8000    모션 에디터 (Joint + IK), arm_server 경유
launcher       :80      FSM 버튼 + start_robot.sh 실행 웹
policy_server  :50030   피지컬 AI — 에피소드 수집 · 데이터셋 zip · 모델 업로드 · ACT 정책 추론 (따로 실행, physical_ai/)
trainer        :50100   학습 PC(RTX 5090) — 데이터셋 업로드 → ACT 학습 → OpenVINO model zip (trainer/)
```

- **팔**: 모든 프로세스는 `arm_server` 를 거칩니다 (`rt/arm_sdk` 는 publisher 하나만 허용).
- **허리·머리**: 703 에서 `rt/arm_sdk` 로 안 움직입니다 (FACTS.md). 그래서 허리 yaw 정렬·좌우 건네기·머리 추종은 없습니다.
- **보행**: 없음 (이 패키지는 서서 팔만 씁니다).

## 빠른 시작

```bash
cd ~/project/h2-motion-control

# 1. 자세 (로봇을 붙잡고!)
./start_fsm.sh stand        # 끝낼 때: ./start_fsm.sh damp

# 2. 전체 스택
./start_robot.sh
#   → 제어 UI : http://<pc-ip>:50000/     (Grab Mode → Box, Handover → Place / Center, Grab Now)
#   → 3D 뷰어 : http://<pc-ip>:50003/dashboard
#   → 박스 인식 : http://<pc-ip>:50010/   (자동 잡기 ON)
#   → 머리 카메라 : http://<pc-ip>:50013/ (로봇 쪽 서비스는 start_robot.sh 가 자동 준비 — utils/head_cam_on.py)

# 3. 모션 에디터 — 모드 지정 필수 (virtual | real)
./start_editor.sh real     # 실기 (start_robot.sh 와 함께 써도 됨 — arm_server 재사용)

# (선택) 웹 런처 — FSM 버튼 + start_robot.sh 실행 (포트 80, sudo)
./launcher.sh
```

## 잡기 시퀀스 (robot_server)

Box 버튼 → 대기 자세 → (자동 또는 Grab Now) → 재검출 → 위쪽 접근 → 측면 하강 → 잡기 → 몸쪽으로 당기기 → 들기 →
**Place**: 원래 자리에 내려놓기 / **Center**: 몸 정면 가운데로 건네기 — 옆에서 잡았어도 가운데로 옮김 (2초 뒤 놓기) /
**Left·Right**: 정면 건네기 자리를 몸통 기준 10° 돌려 건네기 (로봇 앞에 선 사람 기준 왼쪽·오른쪽, 허리 고정이라 팔로만) → 대기 자세 복귀.

값은 전부 `robots/h2/robot.yaml grab` 에 있습니다.

| 키 | 지금 값 | 뜻 |
| --- | --- | --- |
| `ready_xyz` | [0.10, 0.25, 0.30] | 대기·복귀 왼손 위치 (IK pelvis 기준 m, 오른손 y 반대) |
| `z_offset` | 0.02 | 잡는 높이 = 박스 윗면 − H/2 + z_offset |
| `lift_above` | 0.10 | 들기 높이 = 박스 윗면 + 이 값 (0.15 면 어깨가 거치대 요크에 닿음) |
| `pull_x` | −0.15 | 잡은 뒤 몸쪽으로 당김 |
| `squeeze` | 0.03 | 잡을 때 양손을 박스 안쪽으로 더 미는 거리 (각 손, 쥐는 힘) — 건네기까지 유지 |
| `wrist_rpy_deg` | 왼 [−10, 0, −15] · 오른 [10, 0, 15] (정면 V 자, 손끝 안쪽 15°) | 손목 기본 자세 [roll, pitch, yaw] (웹 Wrist RPY 기본값) |
| `box_yaw_follow` | true | 비스듬한 박스: 박스 회전만큼 손목 yaw 를 더해 잡고(손바닥이 옆면과 평행), 대칭 정렬에서 반듯하게 폄 |
| `max_box_yaw_deg` | 25 | 이보다 더 돌아 있으면 잡지 않음 (로그에 ❌). 오프라인: ±24° 까지 박스 중심 x 0.35 는 모두 도달, 0.40 은 일부 2–4 cm 부족 |
| `handover_open` | 0.15 | 건네기(Left·Center·Right) 놓을 때 각 손을 벌리는 거리 |
| `handover_raise` | 0.05 | 건네기 높이 = 들기 높이 + 이 값 (앞으로 내밀면서 같이 올림). 오프라인: 팔 ↔ 거치대 요크 Left·Right 최소 윗면 +0.20 까지 15 mm · +0.25 면 9 mm |
| `handover_side_deg` | 10 | 건네기 Left·Right 각 (사람 기준). 오프라인 IK 10°: 손 오차 ≤ 1.1 cm · 팔 ↔ 거치대 ≥ 16 mm (15° 는 9.6 mm) |
| `place_open` | 0.15 | Place(제자리 내려놓기) 뒤 각 손을 벌리는 거리 — 벌린 채 들어 올림 (이전 0.10) |
| `auto_zone` | x 0.32–0.45 | 자동 잡기 영역 (박스 중심, torso 기준) — 테이블 110 cm · 카메라 46.0° 기준 |
| `default_handover` | place | 기본 = 제자리 내려놓기 |

오프라인 확인 도구: `python utils/grab_reach.py` (박스 위치별로 서버와 같은 IK 로 손이 끝까지 가는지).

## 잡기 시뮬레이터 (`start_grab_sim.sh`, 로봇 없이 시험)

```bash
./start_grab_sim.sh            # 가상 카메라 + 가상 박스  → http://<pc-ip>:50010/
./start_grab_sim.sh real-cam   # 실물 D435i + 실제 박스 인식, 로봇만 가상 → http://<pc-ip>:50012/
```

- `sim/fake_robot.py` 가 로봇 역할 (`rt/arm_sdk` 를 받아 관절을 움직이고 `rt/lowstate` 를 냄, 기구학만).
- `sim/sim_server.py` 가 detect_box(50010) 자리를 대신 → **robot_server / arm_server 는 실기와 같은 코드** 로 잡기 시퀀스를 돕니다.
- **안전**: `ROBOT_SIM=1` → DDS 도메인 1. 실기(도메인 0)와 섞이지 않고, 실기 스택이 떠 있으면(포트 사용 중) 시작을 거부합니다.

## 피지컬 AI — ACT 모방학습 (`physical_ai/`, `trainer/`)

지금 잡기 시퀀스를 시연자로 써서 에피소드를 모으고(로봇 PC :50030), 학습 PC(RTX 5090, :50100)에서 ACT 를 학습해
OpenVINO 로 로봇에서 돌립니다. 기존 잡기·제어 기능은 그대로입니다.

```bash
./physical_ai/start_policy.sh         # 로봇 PC (start_robot.sh 뒤, 다른 터미널) → http://<로봇 PC>:50030/
./physical_ai/start_policy.sh sim     # 시뮬 (start_grab_sim.sh 뒤)
./trainer/start_trainer.sh            # 학습 PC → http://<학습 PC>:50100/ (접근 키는 터미널에)
```

- 수집: 자동 반복(매 회 Box 대기 자세 → 잡기 → 쉬는 동안 박스 옮김) / 잡기 연동 / 수동. 성공 잡기만 저장.
- 추론: 팔 지령은 arm_server `/stream/begin|cmd|end` (속도 제한·0.2 s 무지령 정지·2 s 세션 종료) + 안전 필터
  (데이터셋 관절 범위·손 작업 공간·추종 오차·온도·토크). 잡기 모드 OFF 에서만 시작.
- 자세한 것: `physical_ai/README.md` (수집·안전·데이터 형식), `trainer/README.md` (5090 설치·학습 설정).

## 모션 에디터 (`start_editor.sh`)

| 명령 | 모드 | 띄우는 것 |
| --- | --- | --- |
| `./start_editor.sh virtual` | **가상** — URDF/메시 3D + fake_robot (DDS 도메인 1) | fake_robot, arm_server, dashboard, simulator |
| `./start_editor.sh real` | **실기** — 실제 로봇이 움직임 | arm_server·dashboard(떠 있으면 재사용), simulator |
| `ROBOT_CHECK=1 ./start_editor.sh real` | **실기 · 모터 번호 확인 전용** | 위와 같음. 잡기 서버는 실행 거부, 시작 시 `yes` 확인 |

모터 번호 확인 화면: `http://<pc-ip>:8000/check` — 절차는 `robots/h2/FACTS.md` 와 `common/joint_check.py` 머리말.

## 카메라

- **D435i (가슴, 박스 인식)**: 등판 거치대 `robots/h2/cad/camera_bracket/` (지금 v3 요크 + v1 가로대: 숙임 실측 46.0°, z 0.365). 장착값은 `robot.yaml camera`
  (x, y, z [m, torso_link 기준], pitch_deg, resolution, intrinsics). 해상도 848×480 (2026-10-12, 640×480 보다 가로 화각 56° → 70° —
  박스 좌우 여유 x 0.38 에서 1.3 → 10.8 cm, 숙임 50° 거치대보다 효과 큼). rs_stream · detect_box · 시뮬 · 수집 웹이 모두 `camera.resolution` 을 씀.
  **RealSense 는 한 프로그램만 열 수 있음** — 다른 프로젝트(예: pibog-v1)가 먼저 열면 rs_stream 이 `errno=16 busy` / `errno=5` 로 못 뜸
  (`fuser /dev/video*` 로 확인). 숙임각 실측·저장: `sudo ROBOT=h2 python utils/check_rsimu.py --save` (start_robot.sh 끈 상태, 703 서기)
  → `robots/h2/calib_local.yaml` (이 PC 전용, git 에 안 올라감) 이 robot.yaml 값을 덮어씀. 거치대를 다시 조일 때마다 다시 잴 것.
  지금 쓰는 값은 제어 화면 위 막대 System 칩 → Camera 줄. 위치 확인: `utils/cam_marker_check.py`.
- **머리 쌍안 카메라**: `utils/check_head_cam.py` (수신 확인·깊이 클릭), 인식은 `common/head_track.py` (`robot.yaml head_track`).
  로봇을 다시 켜면 앱 설정이 기본(video_hub 켜짐 · Stereo patch 꺼짐)으로 돌아감 → `start_robot.sh` 가 `utils/head_cam_on.py` 로 매번 자동 준비
  (video_hub 끔 · stereo_patch_pc1 켬 · 수신 IP = 이 PC · 재시작, `robot.yaml head_track.robot_setup`). 서비스 목록: `utils/robot_services.py`.

## 디렉터리

```
h2-motion-control/
├── robot_env.sh            # ROBOT(h2) 확인 + conda/tv python 경로 (스크립트 공용)
├── activate_tv.sh          # tv conda 환경 활성화 + H2 SDK(third_party/) 준비
├── start_fsm.sh            # 자세 전환 (stand / damp / ...)
├── start_robot.sh          # 전체 스택 (rs_stream, arm, robot, dashboard, detect_box, head_track)
├── start_editor.sh         # 모션 에디터 (:8000, virtual | real)
├── start_grab_sim.sh       # 잡기 시뮬레이터 (fake_robot + 서버 + 가상 박스, DDS 도메인 1)
├── launcher.sh / run_launcher.py   # 웹 런처 (:80)
├── sim/                    # fake_robot.py, sim_server.py
├── physical_ai/            # 로봇 PC 피지컬 AI 웹 (policy_server.py :50030, 수집·추론, data/ 는 git 제외)
├── trainer/                # 학습 PC ACT 학습 웹 (trainer_server.py :50100, act_model.py, train_act.py)
├── common/
│   ├── robot_env.py        # robots/h2/robot.yaml 읽기, SDK 경로
│   ├── robot_server.py     # 잡기 시퀀스 + 웹 UI (robot_web.html)
│   ├── arm_server.py       # 팔 HTTP 서버 (arm_sdk 소유)
│   ├── head_track.py       # 머리 카메라 인식
│   ├── simulator.py        # 모션 에디터 (simulator.html, joint_check.py)
│   ├── dashboard.py        # 3D URDF 뷰어
│   ├── rs_stream.py        # D435i 서버
│   ├── ctrl/               # 팔 래퍼, IK, detect_box, 머리 카메라 수신, OpenVINO 검출, 사용률(hw_usage), 손, TTS
│   ├── models/             # 박스 seg 모델, OMZ 얼굴·사람·나이성별·행동(0006), polar11s(자세)·yolo11s(COCO) 예비
│   └── assets/vendor/      # three.min.js
├── robots/h2/              # URDF, meshes/, motions/, IK 모델 캐시, robot.yaml, FACTS.md, cad/
├── third_party/            # unitree_sdk2_python-814556d (activate_tv.sh 가 받음)
└── utils/                  # init_fsm, 카메라 보정, IMU·식별 확인, grab_reach, arm_sdk_test, env_snapshot(가상환경 기록), robot_services 등
```

## 설정 — `robots/h2/robot.yaml`

| 항목 | 키 |
| --- | --- |
| 관절 맵(URDF 이름→슬롯), 팔/허리/머리 슬롯, weight 슬롯, 이득 | `joints`, `gains` |
| FSM ID / 전이 규칙 / launcher 문구 | `fsm` |
| SDK (H2 전용 unitree_sdk2py 커밋) | `sdk` |
| IK 잠금 관절, 손끝(L_ee/R_ee), 모델 캐시 | `ik`, `ik_cache` |
| 기본 팔 자세 | `default_arm_deg` |
| pelvis→torso, 잡기 값 | `frames`, `grab` |
| D435i 장착 위치 | `camera` |
| 인식 장치 (박스 = GPU) | `vision` |
| 머리 카메라 인식 | `head_track` |
| 연결 로봇 확인 | `identity.mode_machine` (`utils/check_robot_id.py` 로 측정) |

로그는 `logs/<name>_<date>.log` 에 타임스탬프와 함께 기록됩니다. 종료는 `Ctrl+C`.
