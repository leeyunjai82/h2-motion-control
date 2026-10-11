# H2 확인 사실표

H2 지원(4단계)을 위해 **출처로 확인한 값**과 **확인이 필요한 값**을 나눠 기록합니다.
추측한 값은 넣지 않습니다. 실기에서 확인하면 이 표를 갱신한 뒤 config(`robot.yaml`)로 옮깁니다.

> 현재 `robots/h2/robot.yaml` 은 `enabled: true` (2026-10-07 사용자 승인, 실기 확인 항목은 아래 표).
>
> **공식 xr_teleoperate `817fb00` 와 일치 (2026-10-07, 사용자 지시 "공식이랑 똑같이"):**
> 공통 `common/ctrl/robot_arm.py` `ArmController` / `robot_arm_ik.py` `ArmIK` (예전 이름 `G1_29_*` 는 별칭) 를 `ROBOT=h2` 로 띄운 결과가
> 공식 `H2_ArmController` / `H2_ArmIK` 와 같음 — 같은 가짜 DDS 로 LowCmd 35 슬롯 (mode/q/dq/tau/kp/kd) · 헤더 · 속도 제한(clip 경로 포함),
> 같은 목표 40개 `solve_ik` 결과 차이 0, 축소 모델 관절 한계 동일.
> 공식과 다른 점은 하나: **`EnableArmSDK` 호출** (실기에서 없으면 팔이 안 움직임 — 아래 표).
> 맞춘 항목: URDF·메시 (`assets/h2`, 0c1b4a2 팔 관절 한계 축소), IK 비용 0.8 / 0.01, 초기화 슬롯 0–34, weak 5/11 + 어깨·팔꿈치,
> 허리·머리 kp 300 / kd 5, 허리 12 roll / 13 pitch / 14 yaw 및 시작 각도 유지(명령 안 함), 팔 속도 제한 30 rad/s.

## 작업 범위 (2026-10 결정)

- **FSM 703 에서 제자리에 서서** (거치대) 테이블 위 박스를 잡아 **제자리에 다시 내려놓기(Place, 기본)** 또는 **정면 건네기(Center)**.
- 보행 중 잡기, 들고 걷기, 마커 추종은 하지 않습니다 (`features.locomotion: false`).
- 시퀀스: 인식 → 접근 → 측면 하강 → 잡기 → 끌어당기기 → 대칭 정렬 → 들기 → Place(원래 자리 위 → 내려놓기 → 손 벌림 → 손 위로 → Home) / Center(건네기, 2초 뒤 손 벌림).
- **허리 고정** — arm_sdk 로 허리가 안 움직임 (실기 2026-10-07). `grab.waist_locked: true`, `joints.waist_hold_initial: true` (시작 각도 유지, 명령 안 보냄 — 공식과 같음).
- **손바닥 마주보기**로 양손 파지 (G1 과 같은 단위 회전 — 아래 "손 자세" 참고).
- **테이블을 높여서** 맞춤 (아래 "시퀀스 IK 가능 범위" 참고).

## 출처

| 기호 | 출처 |
| --- | --- |
| SDK | `unitreerobotics/unitree_sdk2_python` master `814556d` (2026-09-21) |
| SDK-arm | 위 SDK `example/h2/high_level/h2_arm_sdk_dds_example.py` (최종 수정 `e6cd8af`, 2026-08-20) |
| SDK-low | 위 SDK `example/h2/low_level/h2_ankle_swing_example.py` (관절 순서 수정 `65691c8`, 2026-07-21) |
| XR | `unitreerobotics/xr_teleoperate` master `817fb00` (2026-09-07) `teleop/robot_control/robot_arm.py` |
| ROS | `unitreerobotics/unitree_ros` `5994d4f` (2026-09-30) `robots/h2_description/H2.urdf` |
| H2R | `leeyunjai82/h2-motion-control.red` `8e7996d` — **실기에서 simulator 동작 확인** (사용자 보고, 2026-10) |
| STEP | Unitree 공식 H2 단순화 모델 `H2_简化模型_260601.STEP` (oss-global-cdn.unitree.com/static/7585d210dc9b47b6907b70f46f8c24ac.zip, 2026-06-01, 190 MB — repo 미포함). URDF `torso_link` 메시에 정합: STEP→torso x = 0.6932 − y_s, y = x_s − 0.0011, z = z_s − 0.4729 [m] (중앙 오차 1.2 mm) |
| DOC-joint | 공식 문서 H2 SDK 开发指南 「关节电机顺序」 (`H2_developer/joint_motor_sequence`, 2026-07-30 갱신) — 슬롯 0–30 이름·한계, 허리 12/13 = 모터 A/B (병렬) |
| DOC-low | 공식 문서 H2 SDK 开发指南 「底层服务接口」 (`H2_developer/basic_services_interface`, 2026-03-18 갱신) — `rt/lowstate` / `rt/lowcmd` 구조체 |
| 등판도면 | 사용자 제공 등판 도면 — 위 2×M6×18 간격 150, 아래 2×M6×13 간격 176, 위아래 100 |

## 확인된 값

| 항목 | 값 | 근거 |
| --- | --- | --- |
| DDS 메시지 | `unitree_hg` `LowCmd_` / `LowState_` (motor 35 슬롯 고정 배열) | SDK, XR, H2R(실기) |
| DDS 구조체 필드 | SDK `814556d` IDL (Python `unitree_hg/msg/dds_`, C++ `include/unitree/idl/hg`) 그대로 사용 — `MotorState_` = mode, q, dq, ddq, tau_est, temperature[2], vol, sensor[2], motorstate, reserve[4] / `MotorCmd_` = mode, q, dq, tau, kp, kd, reserve(1개). **DOC-low 는 다름**: `MotorState_` 에 q_raw·dq_raw·ddq_raw 가 더 있고 sensor 가 vol 앞, `MotorCmd_.reserve[3]`. 실기는 SDK IDL 로 35 슬롯 수신(허리 pitch 13·팔 15–28 값 정상)과 arm_sdk 송신(CRC 통과, 팔 동작)이 모두 됨 → **문서 쪽 오기로 판단, IDL 바꾸지 않음** (문서대로 바꾸면 슬롯 1 부터 어긋남) | SDK, 실기 2026-10-07~09, DOC-low (2026-10-10 대조) |
| `MotorState_.temperature[2]` | [0] 외부(外表), [1] 권선(绕组) °C. dashboard·joint_check 는 둘 중 큰 값 표시 | DOC-low |
| `mode_pr` | 병렬 기구(발목·허리) 제어 방식 — **0 = PR** (roll/pitch 관절각, 기본), 1 = AB (모터 A/B). 이 repo 는 LowCmd 에 0 | DOC-low, XR |
| `LowState_.tick` | 1 ms 마다 1 증가 | DOC-low |
| 관절 한계 (문서 vs `H2.urdf`) | 허리 12–14 · 손목 pitch 20/27 · 머리 29/30 은 같음. **팔은 URDF 가 더 좁음** (어깨 pitch 위 1.20 ↔ 문서 1.83, roll 1.80 ↔ 2.49, yaw ±1.57 ↔ ±2.62, 팔꿈치 위 1.60 ↔ 3.07, 손목 roll ±1.65 ↔ ±2.62, 손목 yaw ±1.22 ↔ ±1.2217 rad) — xr_teleoperate `0c1b4a2` 축소 그대로, 하드웨어 한계 안쪽이라 IK 가 넘을 일 없음. 머리 문서 −0.5236..0.83775 (−30..+48°) · ±1.7453 (±100°) | DOC-joint, XR |
| 팔 제어 토픽 | `rt/arm_sdk` | SDK-arm, XR, H2R(실기) |
| arm_sdk weight 슬롯 | `motor_cmd[31].q` | SDK-arm, XR, H2R(실기) |
| 어깨·팔꿈치 | 왼 15 ShoulderPitch, 16 Roll, 17 Yaw, 18 Elbow / 오른 22–25 같은 순서 | SDK-arm, SDK-low, XR, H2R 모두 일치 |
| 헤드 | 29 HeadPitch, 30 HeadYaw | SDK-arm(`e6cd8af` 에서 수정), SDK-low, XR, H2R |
| LocoClient | `unitree_sdk2py.h2.loco.h2_loco_client.LocoClient` | SDK. **현재 requirements 고정 커밋 `f559291` 에는 없음 → SDK 업그레이드 필요** |
| FSM ID | 0 ZeroTorque, 1 Damp, 2 Squat, 3 Sit, 4 StandUp, **601 Start** | SDK `h2_loco_client.py` (`Start()` 가 500 → 601 로 바뀐 커밋 `6dec8b2`, 2026-07-02) |
| 기립 순서 | 1 → (5초) → 4 → (10초) → **703** (이 repo, 2026-10-07 변경). H2R 는 → 601 | 601 에서는 arm_sdk 가 안 먹음 (실기) |
| ~~FSM 601 에서 arm_sdk 동작 (EnableArmSDK 없이)~~ | **실기 2026-10-07 재확인: 동작 안 함** (h2-motion-control.red simulator.py 그대로 실행해도 안 움직임) | 사용자 실기 |
| FSM 조회 API | `GetFsmId` 7001, `GetAvailableFsmIds` 7008 (ID+이름), `GetArmSdkStatus` 7007 | SDK `h2_loco_api.py`. 실기 응답 여부는 확인 필요 |
| URDF | `robots/h2/H2.urdf` = ROS `H2.urdf` (H2R 의 URDF 와 바이트 동일) | ROS, H2R |
| 메시 | ROS `5994d4f` (H2R 대비 `head_pitch_link.stl`, `head_yaw_link.stl` 만 갱신 — 시각용) | ROS |
| **`LowState_.mode_machine`** | **1** (`mode_pr` 0, `version` [0, 0]) | 실측 2026-10-07 `utils/check_robot_id.py` 부팅 직후, 20회 관측 모두 1 → `robot.yaml identity.mode_machine` |
| 살아 있는 모터 슬롯 | 0–30 (31개). 31–34 온도·전압 0 (weight 슬롯 31 에 모터 없음) | 실측 2026-10-07 |
| 부팅 직후 다리 0–3, 6–9 | `mode 10`, `vol 0.00` (온도는 정상 출력). 발목 4/5/10/11·상체는 `mode 1`, 73.5–74.0 V | 실측 2026-10-07. mode 10 의미는 확인 필요 (이번 범위에서 다리 미사용) |
| 기본 팔 자세 (FSM 601, arm_sdk 전) | 슬롯 15–28 [deg] = [6.2, 23.7, -25.8, 63.8, 0.6, 0.3, 0.1, 3.2, -19.9, 18.4, 62.6, -0.3, 0.7, 0.1] | 실측 2026-10-07 거치대, 1회 → `robot.yaml default_arm_deg`. 좌우 어깨 roll(16 +, 23 −)·yaw(17 −, 24 +) 부호가 대칭 → 15–18/22–25 배치와 일치. 손목 6개는 모두 ≈0 이라 순서 판정 불가 (jog 로 확인) |
| 601 기립 후 다리 0–3, 6–9 | 여전히 `mode 10`, `vol 0.00` → 부팅 직후와 같음 (기립·밸런스 중에도 이 값) | 실측 2026-10-07 |
| 실기 FSM 목록 (`GetAvailableFsmIds` 7008) | 0 Invalid, 1 Passive, 2 Protection, 3 Sit, **4 FixStand**, 5 HybridPassive, 502 HumanMimic, 503 HumanMimic2, 100 BeyondMimic, **601 HybridWalk**, 701 WalkNew, **703 PhaseWalk** (+ 100xxx/502xxx/503xxx 모션 ID 다수) | 실측 2026-10-07 `utils/robot_state.py` (code 0 응답) |
| arm_sdk 지원 FSM | SDK master `814556d` 예제 `ARM_SDK_SUPPORTED_FSM_IDS = {4, 703}` + `EnableArmSDK()` (601 은 목록에 없음) | SDK 예제. 601 에서 jog 했을 때 팔이 안 움직였다는 사용자 보고 있음 (2026-10-07) |
| FSM 4 상태 | `GetFsmId` (0, 4), `GetFsmMode` (0, 0), **`GetArmSdkStatus` (0, True)** — EnableArmSDK 호출 없이 True. rt/arm_sdk 240 Hz 수신, weight 1.0, 팔 명령 vs 실측 차이 ≤ 1.3° | 실측 2026-10-07 `start_fsm.sh no-bal` 후. jog 추종 여부는 확인 중 |
| LowCmd 헤더 `mode_machine` | xr_teleoperate `817fb00` `H2_ArmController`: `msg.mode_pr = 0`, `msg.mode_machine = lowstate.mode_machine`. 이 repo·H2R 는 0 을 보냄 → `robot.yaml lowcmd.mode_machine: lowstate` 로 변경 (2026-10-07). 실기 효과는 `utils/arm_sdk_test.py` 로 확인 중 | XR, H2R |
| 허리 슬롯 | xr_teleoperate `817fb00` "[fix] H2 waist's joint index": **12 WaistRoll, 13 WaistPitch, 14 WaistYaw** (SDK low-level 예제도 같음) → **2026-10-07 공식 순서로 변경** (이전 yaml 12 yaw / 13 roll / 14 pitch 는 H2R 기준). arm_sdk 로 허리가 안 움직여 실기 판정은 못 함 — 허리에 명령을 안 보내므로 동작 영향 없음 | XR |
| **EnableArmSDK 필요** | 호출 없이: FSM 4·601, LowCmd mode_machine 0·1, 메시지 구성(이 repo / 공식 예제) 모두 팔 0.0° (`GetArmSdkStatus` 는 True 로 나옴). 공식 예제(SDK `814556d`, `EnableArmSDK` 결과 0) 에서는 팔이 움직임 — Stage 1(0자세로) 중 팔이 부딪혀 충돌음, 1.1 s 에 Ctrl+C | 실기 2026-10-07 → `robot.yaml sdk.enable_arm_sdk: true` (robot_arm.py 송신 전 호출, arm_server 종료 시 weight 0 후 Disable) |
| **허리 12–14 는 arm_sdk 로 안 움직임** | FSM 4 + EnableArmSDK 상태에서 팔은 움직이는데 허리 3축 모두 반응 없음 (simulator 에서 명령). SDK `814556d` 공식 예제의 `upper_body_joints` 도 팔 14 + 머리 2 뿐 (허리 없음) | 실기 2026-10-07 사용자 → `grab.waist_locked: true` (정면 건네기만) |
| 팔 추종 | `arm_sdk_test --enable` 슬롯 15 +5° 명령 → +2.7° (kp 80 / kd 3, robot.yaml gains 의 kp_low). xr_teleoperate H2 는 팔 kp 140 / kd 3, 손목 50 / 2 | 실기 2026-10-07 |
| **팔 슬롯 15–28 매핑 ✓** | 15 L ShoulderPitch, 16 Roll, 17 Yaw, 18 Elbow, 19 WristRoll, 20 WristPitch, 21 WristYaw / 22–28 오른쪽 같은 순서 — `robot.yaml joints.map` 그대로 맞음. 23(오른 roll) +8° 는 몸통 쪽이라 3.1° 에서 막힘, −8° 는 −5.4° | 실기 2026-10-07 `arm_sdk_test.py --enable --slot N --deg 8`, 사용자 육안 확인 |
| FSM 703 (PhaseWalk) | `start_fsm.sh 703` (4 → 703) Result 0, `GetFsmId` (0, 703). 다리는 601 과 같은 밸런싱 서기 (걷지 않음, 사용자 확인). EnableArmSDK 후 팔 15 +8° → +6.8° ✓ (게인 140/3). **허리 12/13/14 +5° → 0.0° (안 움직임)** | 실기 2026-10-07 |
| 허리 결론 | FSM 4·703 모두 arm_sdk 로 허리 안 움직임 → H2 는 이 방식으로 허리 제어 불가, `waist_locked: true` 유지. SDK H2 예제도 허리 없음 (794fb2b 팔 14 → e6cd8af 팔 14 + 머리 2), G1 예제는 허리 포함. 예제 구성(`arm_sdk_test.py --enable --style sdk --slot 14/12 --deg 5`) 재확인은 보류. **디버그 모드(rt/lowcmd)는 사용 안 함** (사용자 결정 2026-10-07) | 실기 2026-10-07 **자료 조사 2026-10-10**: 공식 H2 문서 중·영문 전체 · 사용설명서 V1.1 · 고객센터 도움말에 서 있는 상태의 허리 제어 방법 없음 (디버그 모드만). NVIDIA 팀 xr_teleoperate 포크 (`binliunls/xr_teleoperate` `haochens/180hz-tactile-capture` `90dee7b`, `teleop/robot_control/robot_arm.py` L1487) 도 "waist joints are NOT controllable from arm_sdk yet" — 같은 결론. G1 은 앱 '腰部电机锁住' 스위치 해제 + FSM 501(3Dof-waist) 에서 arm_sdk 허리 가능 — H2 앱에 같은 스위치가 있는지는 확인 필요 (설명서에는 [设置]→[数据]→[机器人]→[标定] 만 나옴) |
| 기본 팔 자세 (FSM 703) | [8.2, 18.2, -12.1, 73.7, 0.8, 0.9, 0.1, 8.1, -18.2, 12.4, 73.4, -0.3, 1.2, -0.5] | 실측 2026-10-07 → `robot.yaml default_arm_deg` (601 값 대체) |
| 온도·전압 추이 | 어깨 pitch 15/22: 52 (부팅) → 53 (601) → 59 → **61/60 °C** (703 진입 직후). 허리 55–56 °C. 전압 74.0 → 71.0–71.5 V | 실측 2026-10-07. 한계값 자료 없음 (확인 필요) |
| D435i 숙임각 | 39.1° (IMU, 중력 기준), 좌우 +2.2°. 로봇 IMU pitch −0.3° / roll +0.2° (703) → `camera.pitch_deg 39.1` | 실측 2026-10-07 `utils/cam_tilt.py` |
| D435i 마커 보정 (이전 가슴 거치대) | depth 기준 2점(0.45/0.55, 테이블 렌즈 아래 0.51 m): 축척 0.101/0.100, 숙임 35.2°, camera.x 0.075 (두 점 오차 0). IMU 는 같은 때 36.3° (앞서 39.1° — 로봇 자세 차이 추정). PnP(마커 크기) 거리는 이 조건에서 10 cm 이동을 5 cm 로 냄 → 사용 안 함. ⚠️ 마커·IMU 숙임각은 둘 다 중력 기준 — 로봇이 뒤로 기운 만큼 몸통 기준(`camera.pitch_deg` 의 뜻)과 다름 (사용자: 로봇이 약간 뒤로 서 있음). 몸통 기준은 `utils/check_rsimu.py` (D435i IMU − 로봇 IMU pitch − 허리 pitch) 로 재측정 예정. 마커 35.2° vs IMU 36.3° 의 1.1° 차이 원인은 확인 필요 (테이블 기울기 / D435i IMU 보정·축 정렬) | 실측 2026-10-07 `utils/cam_marker_check.py` |
| 서 있을 때 기울기 (FSM 703) | 로봇 IMU pitch **−3.66°** (뒤로 기움, roll +0.33°), 허리 pitch −0.05° → 몸통 −3.7°. 완성 거치대 D435i: 중력 기준 43.60° → 몸통 기준 **47.4°** (설계 45°, 앞서 완성 전 거치대 41.4° 는 폐기). 부호는 이전 거치대(설계 40° ↔ 중력 36.3° + 3.7° = 40.0°)로 간접 확인. **거치대 v3 (2026-10-09)**: 로봇 IMU pitch −1.11°, 허리 −1.00° → 몸통 −2.11° (뒤로), D435i 중력 기준 62.75° → 몸통 기준 **64.87° ± 0.26** (설계 60°, +4.9°). 팔을 든 자세와 내린 자세 차이 작음 (사용자 확인). **v3 요크 + v1 가로대 (2026-10-09, 60° 는 앞이 안 보여 사용자가 가로대만 교체)**: 로봇 IMU −0.93°, 허리 −0.96° → 몸통 −1.89°, D435i 중력 기준 44.12° → 몸통 기준 **46.01° ± 0.26** (같은 가로대를 v1 요크에 달았을 때 47.4°) | 실측 2026-10-09 `utils/check_rsimu.py --sec 5` |
| 서 있을 때 높이 (FSM 703) | 바닥→허리 관절(torso_link 원점) **1.13 m** → pelvis ≈ **1.01 m** (URDF 다리 편 자세 1.04 보다 3 cm 낮음), 바닥→D435i 렌즈 1.51 m | 실측 2026-10-07 |
| IK 축소 모델 | 다리 12 + 허리 3 + head 2 잠금, 팔 14, `L_ee`/`R_ee` = `*_wrist_yaw_joint` + x 0.05 m | XR `robot_arm_ik.py`, H2R. G1 과 같은 구조 |

## 등판 M6 장착 구멍 (STEP + 등판도면, 2026-10-08)

| 항목 | 값 (torso_link, mm) | 근거 |
| --- | --- | --- |
| 위 2×M6 | y ±75, z 389.1, 나사 시작면 x −55.3, 축 = x (수평 앞뒤) | 도면 150 / STEP 원통 r2.5 21 mm (도면 나사 깊이 18) |
| 아래 2×M6 | y ±88, z 289.1, 나사 시작면 x −63.3 | 도면 176 · 100 / STEP 원통 r2.5 13 mm (도면 13) |
| 우물 | 구멍마다 등 커버에 지름 10 mm 우물 — 테두리→바닥 위 약 13.9, 아래 약 12.4 | STEP (단순화 모델 — 실물 버니어 확인 필요) |
| 나사 쪽 재질 | STEP 에서는 커버와 한 덩어리 (인서트 여부 모름) | 확인 필요 — 체결 토크 낮게 |
| 머리 이동 범위 | 숙임 −30..+48°, 좌우 ±100° 에서 머리·목이 차지하는 공간: \|y\| ≥ 105 면 없음, 가슴 앞 가운데는 z 0.41 이상 | URDF 관절 + STEP 머리 형상 |

→ 카메라 거치대: `robots/h2/cad/camera_bracket/` (3부품, 설계값 camera x 0.130 · z 0.350 · pitch 45°, 장착 후 보정)

## 공식 자료끼리 충돌 → h2-motion-control.red 실기 기준으로 결정 (2026-10)

**2026-10-10 공식 문서 「关节电机顺序」(DOC-joint, 2026-07-30 판) 로 확정 — 지금 채택값과 손목·허리·발목 모두 일치.**
틀린 쪽: SDK-arm 예제 허리, SDK-low 예제 손목, SDK C++ `example/h2/low_level/h2_ankle_swing_example.cpp` 허리 (Yaw/Roll/Pitch, G1 순서).

| 항목 | SDK-arm | SDK-low | XR | H2R | **DOC-joint** |
| --- | --- | --- | --- | --- | --- |
| 손목 19/20/21 (오른 26/27/28) | Roll/Pitch/Yaw | **Yaw/Pitch/Roll** | Roll/Pitch/Yaw | Roll/Pitch/Yaw ← 채택 (사용자 실기 확인) | **Roll/Pitch/Yaw** (小臂横滚/俯仰/偏航) ✓ |
| 허리 12/13/14 | Yaw/Roll/Pitch | **Roll/Pitch/Yaw** | **Roll/Pitch/Yaw** ← 채택 (2026-10-07, XR 최신 fix) | Yaw/Roll/Pitch (이전 채택) | **Roll(모터 A)/Pitch(모터 B)/Yaw** ✓ |
| 발목 4/5 (우리 기능 무관) | Pitch/Roll | Roll/Pitch | Roll/Pitch | Roll/Pitch | **Roll/Pitch** ✓ |
| arm_sdk 허용 FSM | {4, 703} + `EnableArmSDK()` 필요 | — | (검사 없음) | 601 에서 동작 (실기) | (문서에 없음) |

**실기 확인 방법** (H2R simulator, 관절 모드): 관절 하나만 조금 움직이고 실제로 어느 축이 도는지 확인.
- 허리 12 → 몸통이 좌우로 도는가(yaw) / 옆으로 기우는가(roll) / 앞뒤로 숙는가(pitch)
- 손목 19 → 손등 축 비틀림(roll) / 손바닥 면 꺾임(pitch) / 좌우 꺾임(yaw)
- 주의: 관절 모드는 순서가 틀려도 "어느 모터인가는" 움직여 정상처럼 보입니다. **IK 는 URDF 체인 순서(roll→pitch→yaw)로 19/20/21 에 넣으므로 순서가 다르면 손 자세가 틀어집니다.**

## 확인 필요 (자료 없음)

| 항목 | 비고 |
| --- | --- |
| 모터 온도 한계 | 부팅 직후(서 있지 않음) 허리 12–14·어깨 pitch 15/22 가 50–52 °C, 손목 20/21/27/28 이 41–48 °C (실측 2026-10-07). 경고/보호 온도 자료 없음 |
| 펌웨어 버전 | 7109(EnableArmSDK) 필요 여부가 펌웨어에 따라 달라질 수 있음 참고: 고객센터 'H2遥控贴纸及运控介绍 SP1' 은 整包 1.0.2.6 · AI Sport 2.5.4.8 기준 — 운동제어 2 종(R1+Y 常规运控1 / R2+Y 常规运控2), 팔 상호작용 동작은 운동제어 2 에서만. 703 이 어느 쪽인지·우리 로봇 버전은 확인 필요 (앱에서 확인) |
| 손 종류 / SDK | H2R 는 G1 과 같은 `mandro3.py` 포함 — 실제 장착 손 확인 |
| D435i 장착 위치·외부 파라미터 | **2026-10-09 등판 거치대로 교체** (`robots/h2/cad/camera_bracket`, 3부품, 등판 M6 4개). **지금 (2026-10-09): v3 요크 + v1 가로대** — `robot.yaml camera` = x 0.130 · y 0 · z 0.365 (설계값) · 숙임 **46.0°** (check_rsimu 실측) — x 는 마커 확인 전 (`placeholder: true`). v3 가로대(60°)는 실측 64.9° 였으나 앞이 너무 안 보여 교체. v1 전체는 z 0.350 · 47.4°. 보정: `utils/check_rsimu.py --sec 5` (몸통 기준 숙임) → `utils/cam_marker_check.py --points 0.35,0.45` (x). y 는 컬러 센서 옆 위치 확인 필요. `grab.z_offset` 은 이전 거치대 기준으로 맞춘 값이라 첫 잡기에서 다시 확인 |
| H2 기본(헤드) 카메라 | 쌍안. 공식 문서 'Bilateral Data Stream Acquisition Interface' (2026-08-11 판, 사용자 제공): 앱에서 video_hub 끄고 'Stereo patch PC1' 서비스 켬 (자동 시작 아님). **실기 2026-10-09: 로봇을 다시 켜면 앱 설정이 기본(video_hub 켜짐 · Stereo patch 꺼짐)으로 돌아감 → 켤 때마다 다시.** 수신 IP(9080 set) 가 재부팅 뒤에도 남는지는 확인 필요. **robot_state RPC (SDK go2 RobotStateClient) 실기 응답 확인 2026-10-09** — 서비스 27 개, `video_hub` · `stereo_patch_pc1` 포함 (`utils/robot_services.py`). status **0 = 켜짐, 1 = 꺼짐 (실기 확정 2026-10-09)**. 켜기 RPC 는 서비스가 뜰 때까지 응답이 늦어 3 초를 넘김 (code 3104 = 응답 시간 초과, 실제로는 켜짐). `start_robot.sh` 가 `utils/head_cam_on.py` 로 매번 video_hub 끔 → stereo_patch_pc1 켬 → 9080 수신 IP = 이 PC → 재시작 (`robot.yaml head_track.robot_setup`). **실기 동작 확인 2026-10-09 — Head Vision 영상 나옴.** 9080 은 stereo_patch_pc1 이 켜져 있을 때 열리고 `/set?ip=` 응답은 `OK: IP 已更新为 <ip>`. RGB = RTP/H264 UDP 유니캐스트 → 수신 IP (기본 192.168.123.170, `http://192.168.123.161:9080/set?ip=…` 후 서비스 재시작): 5002 원본 쌍안 1920×1080 15 fps(왜곡 보정 전) / 5004 왼눈 · 5006 오른눈 544×448 10 fps. **깊이** = PC1 TCP 5000, Y16 544×448 10 fps, mm (0·65535 무효), 헤더 36 B `<IQQIIII` (magic 0x59313620) — 공식 deb `unitree-dep-img` 1.0.0 의 dep_img_client.c 확인. 확인용 `utils/check_head_cam.py` (deb 없이 직접 수신). **내부 파라미터·깊이↔왼눈 정렬·머리 링크 기준 위치는 문서에 없음** (URDF 에 카메라 프레임 없음, 머리 29/30 관절 따라 움직임) → 실측 필요. 문서의 설치 패키지 2·3 이름이 바뀌어 있음 (2 = v4l2loopback-dkms, 3 = v4l-utils). 실기 수신 확인 필요 |
| 머리 29/30 arm_sdk 명령 | **실기 2026-10-09: 703 에서 안 움직임** — EnableArmSDK 0, GetArmSdkStatus True, 팔은 움직이는데 머리는 30(yaw) +10° (style sdk, kp 30/kd 1) · +5° (style ours, mode 1, kp 300/kd 5) 모두 실측 변화 0.0°, 토크 |tau| 최대 0.08 Nm (kp 300·5° 면 약 26 Nm 이어야 함) → 명령이 모터에 안 닿음 (허리와 같음). 29(pitch) +10° 도 0.0°. 공식 SDK `h2_arm_sdk_dds_example.py`(814556d)는 FSM {4, 703} 에서 팔 + 머리를 명령 → 펌웨어 버전 차이 추정 (확인 필요, 유니트리 문의). FSM 4 는 시연에 안 써서 미시험. `common/head_track.py` 는 보기 전용 (추종 코드는 8dddfe4 에 있음) **자료 조사 2026-10-10**: 머리를 arm_sdk 로 보내는 공개 코드는 SDK 예제(kp 30/kd 1) · NVIDIA 포크(`--motion`, kp 150/kd 10, EnableArmSDK 없음) · MYBOTSHOP 웹 UI(비공개, 'PhaseWalk + Upper Body') 뿐이고 실제로 움직였다는 기록·필요 펌웨어 버전은 어디에도 없음. 공식 xr_teleoperate `817fb00` H2 는 머리·허리를 시작 각도로 잡기만 함. 유일한 비디버그 머리 동작 후보: 팔 동작 서비스 `arm` (API 7106 실행 / 7107 목록, SDK PR #40 설명에 '끄덕임·고개 젓기' 포함이라고 함 — SDK 클라이언트의 로컬 목록 15 개에는 없음, 로봇 7107 응답으로 확인 필요, 정해진 동작만) **실기 2026-10-11 공식 예제 구성 그대로 재확인 → 안 움직임 확정**: `utils/h2_head_official.py` (SDK 814556d 예제에서 팔 목표만 현재각 유지) 와 `arm_sdk_test.py --style official` (mode 0 · mode_machine 0 · 허리 명령 없음 · weight 즉시 1.0), FSM 703, EnableArmSDK 0 → GetArmSdkStatus True. 머리 명령 pitch 3.3→14.5° · yaw 1.3→19.1° 에도 실측 3.3° / 1.3° 그대로, 토크도 시작 값 그대로 (pitch −0.59 · yaw −0.14 Nm — 내장 제어가 계속 잡고 있음, 명령이 닿으면 약 5–8 Nm). 머리 모터 mode 1, motorstate 0 (정상). 내장 팔 서비스 `g1_arm_example` 는 EnableArmSDK 뒤에도 켜짐. **FSM 4 (매단 상태) 도 안 움직임** (사용자 2026-10-11) → 머리 arm_sdk 제어 포기 (사용자 결정) |
| 테이블 높이 | 아래 시퀀스 IK 가능 범위 + 실측 pelvis 높이로 결정 |
| IMU | URDF 에 `imu_in_torso`, `imu_in_pelvis` 링크. `rt/lowstate.imu_state` 가 어느 쪽인지 확인 필요 |

## IK 도달 범위 (오프라인 근사, 2026-10)

조건: 양손 y = ±0.17 m, 손 자세 = 단위 회전(G1 `wrist_params` 기본값), 허리·헤드 고정, pelvis 기준.
pinocchio DLS 로 계산 (서버의 casadi 풀이와 동일하지 않음 — 경향 확인용).

| | G1 | H2 |
| --- | --- | --- |
| 1 cm / 5° 이내로 풀리는 z (x 0.2–0.4 m) | z ≥ 0.0 ~ +0.10 | **z ≥ +0.20** |
| 위치만 (자세 무시) | z ≥ −0.10 | z ≥ 0.0 |

→ G1 과 같은 손 자세면 H2 는 pelvis 위 0.2 m 이상에서만 잡을 수 있습니다. 테이블 높이 / 허리 pitch / 손목 자세 중 무엇으로 맞출지 결정 필요.

## 손 자세 (단위 회전 = 손바닥 마주보기)

zero 자세에서 손목 프레임 회전 = 단위 행렬 (G1·H2 모두 URDF FK 로 확인).
손 메시의 가장 얇은 주축(= 손바닥 법선) 이 손목 프레임 기준 거의 y 축:
- H2 `left_hand_link.stl`: 크기 x 0.164 / y 0.086 / z 0.114 m, 얇은 축 (0.22, 0.98, −0.04)
- G1 `left_rubber_hand.STL`: 크기 x 0.132 / y 0.067 / z 0.106 m, 얇은 축 (0.31, 0.95, 0.03)

→ 단위 회전이면 양손 손바닥 면이 ±y (서로 마주봄). G1 이 실기에서 쓰는 자세와 같은 조건. (손바닥/손등 방향 구분은 3D 뷰어·실기로 최종 확인)

## 시퀀스 IK 가능 범위 (오프라인 근사, 허리 0, 손 마주보기)

박스 W 0.28 × H 0.12 m (예시 — 실제 박스 치수로 다시 계산), G1 `robot_server` 오프셋 그대로
(GRIP_EXTRA −0.05, APPROACH_EXTRA 0.10, GRAB_Z_OFFSET 0.08, 들기 = 윗면 + 0.15).
접근 → 하강 → 잡기 → 들기 → 옆 이동 → 내리기 → 손 벌림 7점을 이어 풀어 전부 1 cm / 5° 이내면 가능.

| 옆 이동 dy | 가능한 박스 윗면 z (pelvis 기준) × 박스 중심 x |
| --- | --- |
| 0 | top +0.15: x 0.25–0.40 / top +0.20: x 0.25–0.45 |
| 0.10 m | top +0.15: x 0.25–0.35 / top +0.20: x 0.25–0.40 |
| 0.15 m | top +0.15: x 0.25–0.30 / top +0.20: x 0.25–0.40 |
| 0.20 m | top +0.20: x 0.25–0.30 |
| 0.25 m | 거의 불가 |

권장 시작점: **박스 윗면 = pelvis + 0.20 m, 박스 중심 x 0.30–0.35 m, 옆 이동 ≤ 0.15 m**.
테이블 높이 = (서 있을 때 pelvis 높이, 실측) + 0.20 − 박스 높이.
(근사 계산이라 경계 칸은 들쭉날쭉함 — 실제 서버 IK(casadi)로 같은 점을 다시 확인할 것)

## 건네기 IK 가능 범위 (오프라인 근사, 허리 0, 정면 건네기 HANDOVER_X 0.30)

접근 → 하강 → 잡기 → 들기 → 건네기(x 0.30, 들기 높이) → 손 벌림 6점, 박스 W 0.28 × H 0.12 (예시).

| 박스 중심 y | top +0.10 | top +0.15 | top +0.20 | top +0.25 |
| --- | --- | --- | --- | --- |
| 0.00 | x 0.25 | x 0.25–0.40 | x 0.25–0.40 | x 0.30 |
| ±0.05 | x 0.25 | x 0.25–0.35 | x 0.25–0.40 | 불가 |
| ±0.10 | 불가 | x 0.25–0.35 | x 0.25–0.40 | 불가 |

→ 허리 yaw 정렬 없이도 박스가 정면 ±10 cm 안이면 가능. **yaw 정렬을 쓰므로** 실제로는 박스가 옆에 있어도 몸통(torso) 기준 정면으로 돌려서 잡는다 (카메라·팔이 모두 torso 에 붙어 있어 IK 는 torso 기준 그대로). 권장: 박스 윗면 pelvis + 0.20 m, x 0.30–0.35 m.
건네는 높이 = 박스 윗면 + 0.15 = **pelvis + 약 0.35 m** (바닥 기준은 서 있을 때 pelvis 실측 후 확정).

⚠️ box 모드 건네기는 G1 과 같이 **받았는지 확인하지 않고 2초 뒤 손을 벌립니다** (`HANDOVER_HOLD_SEC`).
아무도 안 받으면 그 높이에서 박스가 떨어집니다. H2 는 G1 보다 높은 위치라 받는 사람이 있을 때만 실행할 것.

## 시뮬레이터 검증 (start_grab_sim.sh, 2026-10) — 실제 서버 코드 그대로

`sim/fake_robot.py` (기구학 가짜 로봇) + `sim/sim_server.py` (가짜 detect_box) 로 robot_server / arm_server 를 수정 없이 실행.
위 "IK 가능 범위" 표들은 손 목표를 박스 중심 x 에 둔 근사이고, 실제 box 모드는 손 목표 x = 박스 x − 0.15 (`GRAB_X_OFFSET`) —
**아래 시뮬 결과를 우선**한다.

| 시나리오 (박스 W 0.28 · D 0.20 · H 0.12, 윗면 pelvis+0.20) | 결과 |
| --- | --- |
| 정면 x 0.45, 정면 건네기 | 10단계 완료. 단계 끝 손 목표 오차 ≤ 1.3 cm |
| 왼쪽 y 0.10, 좌측 건네기 | 허리 yaw 12.5° 정렬 → 재검출 시 torso y 0.000 → 잡기 오차 0.1–0.3 cm → yaw 30° 건네기 → 복귀 |

### 발견 1 — 카메라→IK 좌표 차이 (수정함)

잡기 코드는 카메라에서 바꾼 torso_link 좌표를 그대로 IK(pelvis 기준) 목표로 쓴다. 두 좌표 차이(z)는 G1 4.4 cm, **H2 12.3 cm**.
G1 은 `grab.z_offset 0.08` 이 실험으로 흡수 (robot_server 주석). H2 에 같은 방식을 쓰면:

| (윗면 pelvis+0.20, H 0.12) | G1 방식 그대로 | `exact_ik_frame: true` + `z_offset 0.036` |
| --- | --- | --- |
| 위쪽 접근 손 높이 − 박스 윗면 | **−2.3 cm (박스와 충돌)** | +9.7 cm |
| 잡기 손 높이 − 박스 옆면 중간 | −4.3 cm | +3.7 cm (G1 실효 +3.6 cm 와 같음) |
| 들기 손 높이 − 박스 윗면 | +2.7 cm | +14 cm |

→ H2 는 `frames.exact_ik_frame: true` (torso + pelvis_to_torso 로 정확 변환), `grab.z_offset 0.036` (= G1 실효값) 로 시작. 실기 실험으로 재조정.
G1 은 `exact_ik_frame: false` 로 기존 동작 그대로.

### 발견 2 — 카메라 장착 높이와 시야

detect_box 의 K(640×480)로 계산. 박스 중심 x 0.45 m 일 때 화면에 박스 윗면이 다 들어오는 좌우 범위:

| 장착 (torso 기준 x, z, 아래각) | 좌우 범위 |
| --- | --- |
| 0.10, 0.35, 50° (첫 임시값) | ±0.05 m |
| 0.06, 0.50, 52° (**현재 임시값**) | ±0.12 m |
| 0.05, 0.60, 58° | ±0.18 m |

카메라가 박스에 너무 가까우면 박스가 화면 대부분을 차지해 조금만 옆에 있어도 잘린다 — 장착 시 참고.

## 실기 모터 번호 확인 절차 (최종 조정 전)

`ROBOT_CHECK=1 ROBOT=h2 ./start_editor.sh real` → `http://<pc>:8000/check` (README "모터 번호 확인" 참고).

| 확인 대상 | 방법 | 이번에 특히 볼 것 |
| --- | --- | --- |
| 허리 12/13/14 | (arm_sdk 로 안 움직여 확인 불가) | 공식 XR 순서 12 roll / 13 pitch / 14 yaw 사용, 명령 안 보냄 |
| 손목 19–21, 26–28 | 명령 ±5° | roll/pitch/yaw 순서 — 공식 자료끼리 다름 |
| 어깨·팔꿈치 15–18, 22–25 | 명령 ±5° | (자료 일치, 재확인) |
| 헤드 29/30, 다리 0–11 | 제어권 반납 후 손으로 → Δ | 발목 4/5 순서 |
| `mode_machine` | 화면 상단 | `robot.yaml identity.mode_machine` 에 기입 |
| 기본 팔 자세 | FSM 601 기립, arm_server release 상태에서 각도 | `default_arm_deg` 14개 |

결과 파일(`robots/h2/joint_check_*.json`)로 `robot.yaml` 을 고친 뒤 `enabled: true` 로 바꾼다.
