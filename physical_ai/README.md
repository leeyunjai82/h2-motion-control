# physical_ai — 로봇 PC 수집 · 추론 웹 (ACT 모방학습)

지금 잡기 코드(robot_server 스크립트)를 **시연자**로 써서 에피소드를 모으고, 학습 PC(`trainer/`)에서 ACT 를 학습한 뒤,
그 정책을 로봇에서 OpenVINO 로 돌립니다. 기존 잡기·제어 기능은 그대로이고, 이 폴더의 서버는 기존 API 만 씁니다.

```
로봇 PC :50030 (이 폴더)                         학습 PC :50100 (trainer/)
 ① 수집: 잡기 1 회 = 에피소드 1 개  ── zip ──▶  ② ACT 학습 → OpenVINO IR
 ③ 추론: 카메라+관절 → 정책 → 안전 필터  ◀── model zip ──┘
         → arm_server /stream/cmd (30 Hz)
```

## 실행

```bash
./start_robot.sh                      # 먼저 (실기)
./physical_ai/start_policy.sh         # 다른 터미널 → http://<로봇 PC>:50030/

./start_grab_sim.sh                   # 시뮬
./physical_ai/start_policy.sh sim
```

기존 파일 중 바뀐 것은 `common/arm_server.py` 의 스트리밍 엔드포인트(`/stream/begin|cmd|end`) 추가 하나뿐입니다.
`/joints` 는 매번 보간이 끝날 때까지 잡고 있어(smoothstep, 409) 30 Hz 지령에 못 씀.

## ① 수집

| 방법 | 쓰는 때 |
|---|---|
| **자동 반복** | 횟수·쉬는 시간 지정 → 매 회 Box 대기 자세 → 박스 보이면 Grab Now → 끝나면 쉬는 동안 **사람이 박스를 다른 자리로** → 반복 |
| **잡기 연동 기록** | 켜 두면 제어 UI 의 Grab Now·자동 잡기가 돌 때마다 1 에피소드. 매번 **Box 버튼으로 대기 자세를 만든 뒤** 잡기 (잡기 끝 ⑪ Home 은 손목 회전이 없는 자세라 그대로 이어서 잡으면 시작 자세가 달라짐) |
| **수동 기록** | 시작/정지 사이를 그대로 기록 (잡기와 무관) |

- 30 Hz · D435i RGB (rs_stream `/video_feed`) + 팔 14 축 실측·토크(`rt/lowstate`) + 지령(`rt/arm_sdk`) + 손 위치(FK) + 잡기 단계.
- 성공(robot_server `/grab_history` 의 ok = 복귀까지 감)만 저장. '실패한 잡기도 저장' 으로 바꿀 수 있음.
- **잡은 뒤는 place(정면에 내려놓기) 권장** — 사람 없이 반복되고, 건네기(center)는 '받기 대기' 2 초 고정 정지가 있어
  정책이 그 자리에서 멈춰 버리기 쉬움.
- 박스 위치·각도를 고르게 바꿔 주세요 (같은 자리만 모으면 그 자리만 잡음). 대략 50 회부터 시작, 부족하면 더.
- 저장 해상도 = 카메라 절반 (기본, H2 424×240 — 에피소드당 약 5 MB) / 원본 (H2 848×480, robot.yaml camera.resolution).

## ② 학습 PC 로

Episodes 카드 **⬇ 데이터셋 zip** → 학습 PC 웹(:50100)에 업로드 → 학습 → model zip. 자세한 것은 `trainer/README.md`.

## ③ 추론

1. Models — model zip 업로드 → 실행 장치(CPU/GPU/NPU) 고르고 **로드** (컴파일 + 시험 추론 시간 표시)
2. 시작 조건이 모두 초록이면 **▶ 정책 시작**
   - 팔 hold · 잡기 모드 **OFF** · 잡기/모션 안 도는 중 · 카메라 · `rt/lowstate` 살아 있음 · 수집 안 하는 중
3. 데이터셋 첫 자세(평균)로 천천히 이동(관절 공간, park 경로의 반대) → 30 Hz 정책
4. **STOP / Esc** — 정책만 멈추고 팔은 그 자리 유지 (박스를 든 채일 수 있음). 그 뒤 제어 UI Stop 이나 OFF 로 내리기.

정책 루프: 카메라 최신 프레임 + 실측 → OpenVINO → (앙상블: 매 스텝 추론, 겹치는 예측을 지수 가중 평균 /
청크: N 스텝마다 추론) → 안전 필터 → `arm_server /stream/cmd`.

### 안전 장치

| 단계 | 내용 (기본값, 웹에서 바꿈) |
|---|---|
| 관절 범위 | 데이터셋 지령 최소~최대 ± 5° 로 자름 (URDF 한계 안) |
| 속도 | arm_server 스트림 세션이 지령 사이 이동량을 1.5 rad/s × 경과시간(≤0.1 s)으로 제한 (arm_server 상한 3 rad/s) |
| 손 작업 공간 | FK 손 위치가 데이터셋 손 범위 ± 5 cm 밖이면 그 지령 거부(전 지령 유지), 연속 10 회면 정지 |
| 두 손 거리 | 데이터셋 최소 − 3 cm 보다 가까우면 거부 |
| 추종 오차 | 지령−실측 > 20° 가 0.5 s 이어지면 정지 (막힘·충돌 의심) |
| 온도 | 팔 모터 70 °C 넘으면 정지 — 소프트웨어 정지값. **H2 팔 모터 자체 보호 온도는 확인 필요** |
| 토크 | \|tau_est\| ÷ URDF effort > 0.9 가 0.3 s 이어지면 정지 |
| 끊김 | 카메라 0.3 s · `rt/lowstate` 0.1 s · 루프 지연 0.3 s 넘으면 정지 |
| arm_server | 0.2 s 지령 없으면 마지막 목표에서 멈춤, 2 s 없으면 세션 끝 (정책 서버가 죽어도 팔은 그 자리) |
| 다른 동작 | 세션 동안 arm_server 팔 잠금 → robot_server 잡기·모션은 409. 제어 UI Stop(`/stop_motion`·`/freeze`)은 세션을 끊음 |
| 최대 시간 | 60 s |

안전 필터는 정책이 **데이터에 있던 범위 밖으로** 나가는 것을 막을 뿐, 그 범위 안에서 틀린 동작(박스를 놓치는 등)은 못 막습니다.
처음에는 박스 없이·손을 리모컨 E-stop 에 둔 채로 시작해 보세요.

## 데이터 형식

`physical_ai/data/datasets/<이름>/` (git 에 안 올라감)

```
info.json                        name, robot, fps, joint_names, format = h2-episodes-v1
episodes/ep_000001/episode.npz   (np.savez, pickle 없음)
    t        (N,)    float64  에피소드 시작부터 s
    state    (N,14)  float32  팔 실측 q [rad]  — robot.yaml joints.arm 순서 (arm_server 14 축과 같음)
    action   (N,14)  float32  팔 지령 q [rad]  — rt/arm_sdk motor_cmd.q
    tau      (N,14)  float32  팔 토크 [Nm]  — rt/lowstate motor_state.tau_est (2026-10-13 부터, 그 전 에피소드에는 없음)
    ee       (N,6)   float32  손끝 위치 L xyz, R xyz [m] — IK 좌표 (pelvis 기준), 실측 q 로 FK
    stage    (N,)    int8     robot_server GRAB_STAGES 순번 (-1 = 없음)
    weight   (N,)    float32  arm_sdk weight
    img_t    (N,)    float64  그 프레임 카메라 수신 시각 (에피소드 기준 s)
    jpeg     (M,)    uint8    JPEG 를 이어 붙인 것
    jpeg_off (N+1,)  int64    프레임 i = jpeg[jpeg_off[i]:jpeg_off[i+1]]
episodes/ep_000001/meta.json     n, duration, ok, source(auto/grab/manual), image{w,h}, joint_names, stage_names,
                                 tau_abs_max (축별 |tau| 최대 Nm), sim_box …
episodes/ep_000001/thumb.jpg     첫 프레임
```

모델 `physical_ai/data/models/<이름>/policy.xml|.bin|meta.json` — `meta.json` 의 `format = h2-act-v1`,
`joint_names` 가 이 로봇 팔 14 축 순서와 같아야 업로드됨. 안전 필터 범위(`stats.action_min/max`, `ee_min/max`,
`hand_gap_min`)와 시작 자세(`start_pose`)도 여기서 읽음.

## 시뮬 검증 (2026-10-11, 컨테이너)

`start_grab_sim.sh` + `start_policy.sh sim` → 자동 반복(시뮬 박스 무작위) → zip → trainer(CPU) 학습 → model zip →
업로드·로드 → 정책 시작. 시뮬 카메라는 박스 다각형만 그리고 박스가 손을 따라 움직이지 않으므로
**흐름 확인용**이고 성능 확인은 아님.
