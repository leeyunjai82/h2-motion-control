/* i18n.js — 웹 UI 한/영 전환 (launcher :80 / robot_web :50000 / dashboard :50003 / simulator :8000 / physical AI policy_web :50030 공용)
 *
 * 동작
 *  - 화면의 한글 텍스트를 아래 사전으로 영어로 바꾼다 (텍스트 노드 + title/placeholder + alert/confirm/prompt).
 *  - JS 가 나중에 그리는 텍스트도 MutationObserver 로 자동 번역.
 *  - 원문은 노드에 보관 → 한국어로 되돌리면 복원.
 *  - 사전에 없는 한글은 그대로 둔다.
 *  - 언어: ?lang=en|ko → localStorage('g1_lang') → 기본 ko
 *    (포트가 다르면 localStorage 가 따로라, iframe 은 ?lang= / postMessage 로 맞춘다)
 *  - 버튼: id="lang-slot" 요소가 있으면 그 안, 없으면 우측 하단 고정. ?embed=1 이면 버튼 숨김.
 */
(function () {
  'use strict';

  // ---------- 사전 (ko → en). 긴 문구가 먼저 적용된다 ----------
  const DICT = {
    // ===== launcher =====
    '웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요': 'Web buttons are NOT an emergency stop — keep the E-STOP / remote in hand',
    '순서로, 로봇이 자리 잡은 걸 보고 다음 단계를 누르세요': 'in order — press the next step once the robot has settled',
    '일어서기:': 'Stand up:',
    '자세 (FSM)': 'Posture (FSM)',
    '현재 FSM': 'Current FSM',
    '힘 빼기 · 항상 가능': 'power off · always allowed',
    '밸런스 없음 · 1 / 703 에서': 'no balance · from 1 / 703',
    '밸런스 · arm_sdk · 4 에서': 'balance · arm_sdk · from 4',
    '밸런스 없음 · 서 있을 때': 'no balance · while standing',
    'FSM 703 에서만 · 6개 서버': 'FSM 703 only · 6 servers',
    'Robot 시작': 'Start Robot',
    'Robot 정지': 'Stop Robot',
    'Ctrl+C 와 동일': 'same as Ctrl+C',
    '로봇 서버': 'Robot servers',
    '지금 서 있습니다 — 힘이 빠져 넘어집니다!': 'It is standing — it will lose power and fall!',
    '현재 상태 확인 불가 — 서 있다면 힘이 빠져 넘어집니다!': 'State unknown — if standing, it will lose power and fall!',
    '모터 힘이 빠집니다': 'Motors will go limp',
    '· 로봇을 사람이 받치고 있거나 스탠드에 묶여 있습니까?': '· Is someone supporting the robot, or is it tied to the stand?',
    '· 주변에 사람/장애물이 없습니까?': '· Is the area clear of people / obstacles?',
    '로봇이 천천히 앉습니다 (밸런스 제어 없음)': 'The robot sits down slowly (no balance control)',
    '· 팔을 몸 옆으로 내렸습니까?': '· Are the arms lowered to the sides?',
    '· 앉는 동안 로봇을 받치고 있습니까?': '· Are you supporting the robot while it sits?',
    '· 엉덩이 아래 공간이 비어 있습니까?': '· Is the space under the hips clear?',
    'start_robot.sh 를 실행합니다.': 'Run start_robot.sh.',
    '· arm_server 가 기동하면서 팔/허리 제어권(weight=1)을 잡습니다.': '· arm_server takes arm/waist control (weight=1) on startup.',
    '로봇 서버를 정지합니다 (팔 제어권 반납 후 종료).': 'Stop the robot servers (release arm control, then exit).',
    '순서 제한 없음': 'no order restriction',
    '마지막 전송': 'last sent',
    '밸런스 제어': 'balance control',
    '밸런스 없음': 'no balance',
    'launcher 연결 끊김': 'launcher disconnected',
    'Robot 서버 실행 중 — 먼저 [Robot 정지]': 'Robot servers running — press [Stop Robot] first',
    '— 1 → 4 → 703 로 PhaseWalk(703) 진입 후 시작': '— enter PhaseWalk (703) via 1 → 4 → 703 first',
    '4 는 FSM 1(Damp) 또는 703 에서만': '4 only from FSM 1 (Damp) or 703',
    '703 은 FSM 4(FixStand) 에서만': '703 only from FSM 4 (FixStand)',
    'Sit 은 서 있을 때만': 'Sit only while standing',
    '대기 취소 — Damp 로 교체': 'pending cancelled — replaced by Damp',
    '전송 직전 거부': 'rejected right before sending',
    '전송 완료': 'sent',
    '대기/전송 중': 'pending / sending',
    'FSM 조회 실패': 'FSM query failed',
    'LocoClient 초기화 실패': 'LocoClient init failed',
    'FSM 확인 불가': 'FSM unknown',
    '703 상태인지 직접 확인할 것': 'check manually that it is in 703',
    '종료 지연 — 프로세스 그룹 SIGKILL': 'stop timed out — SIGKILL process group',
    '알 수 없는 단계': 'unknown step',
    '위치제어': 'position control',

    // ===== robot_web =====
    'Box 인식': 'Box detection',
    '정지·기본자세': 'stop · home pose',
    '제어권 반납': 'release control',
    // System 카드 (robot_server /system — ctrl/sys_watch.py)
    'arm_server 미연결 — 팔이 안 움직임 (arm_server 로그 확인, 3초마다 다시 연결 시도)':
      'arm_server not connected — arms will not move (check the arm_server log; retrying every 3 s)',
    '제어권 반납(release) 상태 — 팔이 안 움직임 (arm_server 재시작 또는 POST /arm_hold)':
      'control released — arms will not move (restart arm_server or POST /arm_hold)',
    '제어권 반납(release) 상태 — 팔 명령이 먹지 않음': 'control released — arm commands have no effect',
    'arm_server 가 떠 있지만 팔 미초기화 — arm_server 로그 확인': 'arm_server is up but the arm is not initialized — check its log',
    'arm_sdk 제어권 점유 (hold)': 'arm_sdk control held',
    '이 FSM 에서는 팔이 안 움직임': 'arms do not move in this FSM',
    '팔이 움직이는 FSM:': 'arms move in FSM:',
    '팔(arm_sdk) 동작 FSM': 'arm_sdk FSM',
    'dashboard 는 떠 있지만 rt/lowstate 미수신': 'dashboard is up but rt/lowstate is not received',
    '3D 뷰어 · rt/lowstate 수신': '3D viewer · receiving rt/lowstate',
    '머리 카메라 RGB 수신 중': 'receiving head camera RGB',
    '로봇 전원·네트워크 확인': 'check robot power / network',
    '로봇 FSM': 'Robot FSM',
    '박스 인식': 'box detection',
    '준비 안 됨': 'not ready',
    '조회 실패': 'query failed',
    'RGB 없음': 'no RGB',
    '로봇 없음': 'no robot',
    '자동 ON': 'auto ON',
    '이 PC': 'this PC', '장치': 'Device', '사용률': 'Usage', '모델': 'Model',
    '박스 seg': 'box seg', '얼굴·사람': 'face·person', '사물': 'objects',
    '로봇 앞에 선 사람 기준': "as seen by the person facing the robot",
    '사람 기준 왼쪽': "person's left", '사람 기준 오른쪽': "person's right",
    'start_grab_sim.sh — 가상 로봇에는 FSM 없음': 'start_grab_sim.sh — the virtual robot has no FSM',
    '시뮬': 'sim',
    '⚠ 걷기 프레임 — 재생 시 건너뜀 (보행 없음)': '⚠ walk frame — skipped on playback (no walking)',
    '내려놓기': 'put down',
    '상태 확인 불가': 'state unknown',
    '응답 없음': 'no response',
    '잡는 중': 'grabbing',
    '모션 실행': 'motion running',
    'AI 인식': 'AI vision',
    '처리 속도': 'Throughput',
    '박스 W×D×H': 'Box W×D×H',
    '마지막 잡기': 'Last grab',
    '기본 자세': 'home pose',
    '기본자세': 'home pose',
    // 잡기 단계 (robot_server GRAB_STAGES, 칸은 띄어쓰기 없이 표시)
    '허리 정렬': 'Waist align', '허리정렬': 'Waist',
    '위쪽 접근': 'Approach', '위쪽접근': 'Approach',
    '측면 하강': 'Lower', '측면하강': 'Lower',
    '받기 대기': 'Wait take', '받기대기': 'Wait take',
    '재검출': 'Re-detect', '잡기': 'Grip', '들기': 'Lift', '건네기': 'Hand over',
    '놓기': 'Release', '복귀': 'Return',
    // robot_server 오류
    '동작 중 - 정지 후 전환': 'busy - stop before switching',
    '잡기/모션 동작 중': 'grab / motion in progress',
    '파일 파싱 오류': 'file parse error',
    '파일 없음': 'file not found',
    '빈 모션': 'empty motion',
    '검출 없음': 'nothing detected',
    'arm 전환 중': 'arm switching',
    '미초기화': 'not initialized',
    '동작 중': 'busy',

    // ===== dashboard =====
    '인식 박스 · 폭': 'Detected box · W',
    '손 목표 (IK)': 'Hand target (IK)',
    '인식 박스': 'Detected box',
    '폭': 'W',
    '비교': 'Compare',
    '차이': 'diff',
    '기울기': 'tilt',
    '평면': 'plane',
    // 추정 방식 비교 (robot_web AI 오버레이)
    '추정 비교 · 사용': 'Estimator · using',
    '윗면 기울기': 'Top tilt',
    '파지점 차이 L/R': 'Grip pt diff L/R',
    '중심 차이': 'Center diff',
    '높이 기본/평면': 'Height legacy/plane',
    '로봇 연결 시 표시': 'shown when the robot is connected',
    '부하 (|토크| / 최대 토크)': 'Load (|torque| / max torque)',
    '모터 온도': 'Motor temp',
    '데이터 없음': 'no data',

    // ===== simulator =====
    'Joint · 관절': 'Joint',
    'IK · 좌표': 'IK · Cartesian',
    '손: 확인 중...': 'Hand: checking...',
    '손: 연결됨': 'Hand: connected',
    '손: 미연결': 'Hand: not connected',
    '홈 자세': 'Home pose',
    '오른쪽 팔': 'Right arm',
    '왼쪽 팔': 'Left arm',
    '허리 (Waist)': 'Waist',
    'X (전후)': 'X (fwd/back)',
    'Y (좌우)': 'Y (left/right)',
    'Z (상하)': 'Z (up/down)',
    'Yaw (좌우)': 'Yaw',
    'Roll (기울기)': 'Roll',
    'Pitch (숙임)': 'Pitch',
    '키보드: 누르는 동안 이동 · 떼면 정지': 'Keyboard: hold to move · release to stop',
    '좌측 이동': 'Strafe L',
    '우측 이동': 'Strafe R',
    '좌회전': 'Turn L',
    '우회전': 'Turn R',
    '손 제어': 'Hand control',
    '서버 연결 대기 중...': 'Waiting for server...',
    '손 펴기 (unfold)': 'Open hand (unfold)',
    '타임라인 에디터': 'Timeline editor',
    '프레임 시간 (초)': 'Frame time (s)',
    '이동 명령': 'Move command',
    '손 모션': 'Hand motion',
    '손 선택': 'Hand',
    '현재 자세 저장': 'Save current pose',
    '포함 (현재 모드 기준)': 'Include (current mode)',
    '포함 (팔)': 'Include (arm)',
    '포함 (XYZ + Rotation)': 'Include (XYZ + Rotation)',
    '안전 모드 (Joint)': 'Safe mode (Joint)',
    'x0.5 제한': 'x0.5 limit',
    '프레임 추가': 'Add frame',
    '불러오기': 'Load',
    '파일 이름:': 'File name:',
    '파일 오류': 'File error',

    // ===== physical AI (policy_web :50030) — 화면 =====
    'Collect · 수집': 'Collect',
    'Auto Repeat · 자동 반복': 'Auto Repeat',
    'Camera · 정책 입력': 'Camera · policy input',
    'Policy Run · 추론': 'Policy Run',
    'Policy · Safety 설정': 'Policy · Safety settings',
    '잡기 연동 기록': 'Record with grabs',
    '— 잡기(Grab Now·자동 잡기)가 돌 때마다 1 에피소드': '— 1 episode per grab (Grab Now · Auto Grab)',
    '저장 해상도': 'Save resolution',
    '기록 범위': 'Record span',
    '복귀까지 (전체)': 'Until return (full)',
    '들기까지': 'Until lift',
    '실패한 잡기도 저장': 'Also save failed grabs',
    '수동 기록': 'Manual record',
    '기록 정지': 'Stop record',
    '30 Hz · D435i RGB + 팔 14축 실측(rt/lowstate) + 지령(rt/arm_sdk) + 손 위치 + 잡기 단계. 잡기 성공(복귀까지 감)만 저장 — robot_server':
      '30 Hz · D435i RGB + 14 arm joints measured (rt/lowstate) + commanded (rt/arm_sdk) + hand positions + grab stage. Only successful grabs (through return) are saved — per robot_server',
    ' 기준.': '.',
    '쉬는 시간 s': 'Rest s',
    '쉬는 시간': 'rest time',
    '박스 대기 s': 'Box wait s',
    '잡은 뒤': 'After grab',
    'place — 제자리 내려놓기': 'place — put it down',
    'center — 건네기': 'center — hand over',
    'left — 건네기 (사람 기준 왼쪽)': "left — hand over (person's left)",
    'right — 건네기 (사람 기준 오른쪽)': "right — hand over (person's right)",
    '끝나면 모드 OFF': 'Mode OFF when done',
    '시뮬: 매 회 박스 위치 무작위': 'Sim: random box position each run',
    '자동 수집 시작': 'Start auto collect',
    'Box 모드 → 박스 보이면 잡기 → 끝나면': 'Box mode → grabs when the box is seen → when done, during the',
    '동안 박스를 다른 자리로 옮기세요 → 반복.': 'move the box to another spot → repeat.',
    "권장 (건네기는 '받기 대기' 2 초 정지가 있어 정책이 멈춰 버리기 쉬움). 잡은 뒤 방식은 제어 UI 설정도 같이 바꿉니다.":
      "recommended (hand-over has a 2 s 'wait take' pause, so the policy easily stalls). The after-grab mode also changes the control UI setting.",
    'SIM 가상 카메라': 'SIM virtual camera',
    'SIM · 실물 D435i': 'SIM · real D435i',
    '용량 1/4': 'size 1/4',
    '원본': 'original',
    '정책 시작': 'Start policy',
    '반복 시작': 'Start repeat',
    '회 · 사이 쉬기': 'runs · rest',
    "한 회 = 시작 → 데모 끝 자세로 돌아오면 '완료'. 반복은 매 회 박스가 보일 때 시작, 완료가 아닌 정지면 멈춤":
      "One run = start → 'done' when back at the demo end pose. Repeat starts each run once the box is seen, and stops on any stop other than done",
    '데모 끝 자세 모름 — 8 초 뒤 멈춰 있으면 완료': 'demo end pose unknown — done if still after 8 s',
    '루프 Hz': 'Loop Hz',
    '추론 ms': 'Infer ms',
    '속도 제한': 'Vel limit',
    '범위 자름': 'Clipped',
    '정지 사유': 'Stop reason',
    '최근 거부': 'last reject',
    '시작하면 데이터셋 첫 자세로 천천히 이동 → 정책. 막대: 회색 칸 = 허용 범위(데이터셋 ± 여유),':
      'On start, moves slowly to the dataset first pose → policy. Bars: gray band = allowed range (dataset ± margin),',
    '= 지령. STOP 은 정책만 멈추고 팔은 그 자리 유지 (박스를 들고 있을 수 있음).':
      '= commanded. STOP halts only the policy; the arms hold where they are (may be holding the box).',
    '파랑': 'blue',
    '주황': 'orange',
    '모터 최고': 'Motor max',
    '행을 누르면 그 에피소드 영상·그래프 (마우스를 올리면 첫 프레임)': 'Click a row for its video · graphs (hover for the first frame)',
    '데이터셋 zip (학습 PC 로)': 'Dataset zip (to training PC)',
    '데이터셋 삭제': 'Delete dataset',
    '데이터셋': 'Dataset',
    '새로': 'New',
    '업로드': 'Upload',
    '실행 장치': 'Device',
    '로드됨': 'Loaded',
    '로드 안 됨': 'not loaded',
    '로드 실패': 'Load failed',
    'trainer(학습 PC)가 만든 model zip 을 올리고 로드. NPU 는 첫 컴파일이 오래 걸릴 수 있음.':
      'Upload and load a model zip made by the trainer (training PC). The first NPU compile can take a while.',
    '실행 방식': 'Exec mode',
    '앙상블 (매 스텝)': 'Ensemble',
    '앙상블 계수': 'Ensemble coeff',
    '청크 스텝': 'Chunk steps',
    '청크': 'Chunk',
    '최대 속도 rad/s': 'Max vel rad/s',
    '관절 여유 °': 'Joint margin °',
    '손 공간 여유 m': 'Hand space margin m',
    '최대 시간 s': 'Max time s',
    '추종 오차 °': 'Tracking err °',
    '토크 비율': 'Torque ratio',
    '연속 거부': 'Reject streak',
    '모터 온도는 표시만 하고 멈추지 않음 — H2 팔 모터 보호 온도는': 'Motor temperature is display-only, it never stops — H2 arm motor protection temperature:',
    '확인 필요': 'to be confirmed',
    '토크 비율 = |tau_est| ÷ URDF effort. 다음 실행부터 적용.': 'Torque ratio = |tau_est| ÷ URDF effort. Applies from the next run.',
    // 에피소드 보기
    '앞 번호 에피소드': 'Previous episode',
    '뒷 번호 에피소드': 'Next episode',
    '첫 에피소드': 'first episode',
    '마지막 에피소드': 'last episode',
    '에피소드': 'Episode',
    '불러오는 중…': 'loading…',
    '영상 불러오기 실패': 'video load failed',
    '잡기 단계': 'Grab stages',
    '단계 없음': 'no stage',
    'Space 재생/멈춤 · ←/→ 한 프레임 (Shift 10) · 그래프를 누르면 그 시각으로': 'Space play/pause · ←/→ one frame (Shift 10) · click a graph to jump there',
    '각도 °': 'Angle °',
    '토크 Nm (실측 tau_est)': 'Torque Nm (measured tau_est)',
    '두 손 거리 cm': 'Hand gap cm',
    '|토크| 최대': '|torque| max',
    '토크 없음 (예전 에피소드)': 'no torque (older episode)',
    '지령 손 위치 없음 (예전 에피소드)': 'no commanded hand pos (older episode)',
    '토크 기록 없음 (예전 에피소드)': 'no torque recorded (older episode)',
    '두 손': 'hand gap',
    '실측': 'measured',
    '지령': 'commanded',
    '프레임': 'frames',
    '시각': 'Time',
    '출처': 'Source',
    '결과': 'Result',
    '각도': 'Angle',
    '관절': 'Joint',
    '닫기': 'Close',
    '삭제?': 'delete?',
    '삭제됨': 'deleted',
    '삭제': 'Delete',
    // 상태 · 토스트 · 확인창
    '들기 지남 — 잡기 끝나길 기다림': 'past lift — waiting for the grab to finish',
    '저장 —': 'saved —',
    '버림 —': 'discarded —',
    '저장됨 — 다음 실행부터': 'Saved — applies from the next run',
    '서버 연결 실패': 'server connection failed',
    '새 데이터셋 이름 (영문·숫자·_.-)': 'New dataset name (letters · digits · _.-)',
    'model zip 을 고르세요': 'choose a model zip',
    '업로드 중...': 'uploading...',
    '업로드됨': 'uploaded',
    '컴파일 중': 'compiling',
    '정책 시작 — 시작 자세로 이동': 'Policy started — moving to the start pose',
    '성공': 'ok',
    '기록': 'Rec',
    '횟수': 'Count',
    '자동': 'Auto',
    '반복': 'Repeat',
    '단계': 'Stage',
    '스텝': 'Steps',
    '로드': 'Load',
    '시작': 'Start',
    '중지': 'Stop',
    '분': 'min',
    // ===== physical AI — 서버가 보내는 단계 · 사유 · 오류 =====
    '시작 조건 안 맞음': 'start conditions not met',
    '시작 조건 기다림': 'waiting for start conditions',
    '쉬는 중': 'resting',
    '중지됨': 'stopped',
    '끝': 'finished',
    '시작 자세로': 'to start pose',
    '시작 자세 이동 중': 'while moving to the start pose',
    '정책 실행 / 자동 수집 중': 'policy / auto collect running',
    '정책 실행 중 — 먼저 정지': 'policy running — stop it first',
    '정책 실행 중': 'policy running',
    '정책 실행': 'policy running',
    '완료 — 데모 끝 자세로 돌아옴': 'done — back at the demo end pose',
    '최대 시간': 'max time',
    '카메라 끊김': 'camera lost',
    '동작 시작됨': 'started moving',
    '루프 지연': 'loop delay',
    '장치를 GPU 로 바꾸거나 청크 방식으로': 'switch the device to GPU or use chunk mode',
    'Stop 버튼': 'Stop button',
    '손 작업 공간 밖': 'hand outside workspace',
    '두 손 너무 가까움': 'hands too close',
    '추종 오차': 'tracking error',
    '막힘/충돌 의심': 'blocked / collision suspected',
    '토크': 'torque',
    '팔 동작 없음': 'Arms idle',
    '잡기 모드 Box (대기 자세)': 'Grab mode Box (ready pose)',
    '제어 화면 Box 버튼': 'Box button on the control page',
    '잡기/모션 안 도는 중': 'Grab/motion idle',
    '응답 없음 — 충돌할 잡기 없음': 'no response — no grab to conflict with',
    '프레임 없음': 'no frames',
    '수집 안 하는 중': 'Not collecting',
    '제어 화면 Auto Grab 끄기': 'turn off Auto Grab on the control page',
    '박스 안 보임': 'no box seen',
    '너비': 'width',
    '박스 위치 바꾸세요': 'move the box',
    '시뮬 박스': 'sim box',
    '보이는 위치를 못 찾음': 'no visible spot found',
    '이전 잡기 끝나기 기다림': 'waiting for the previous grab to finish',
    '대기 자세 이동': 'moving to the ready pose',
    '박스 인식 기다림': 'waiting for box detection',
    '건너뜀': 'skipped',
    '잡기 시작 안 됨': 'grab did not start',
    '잡기 중': 'grabbing',
    '기록 오류': 'record error',
    '마지막 단계': 'last stage',
    'grab_history 에 기록 없음': 'no record in grab_history',
    '수동 실패 표시': 'marked failed manually',
    '이미 실행 중': 'already running',
    '모델 없음 — 먼저 로드': 'no model — load one first',
    '다른 모델 로드 중': 'another model is loading',
    '이미 자동 수집 중': 'auto collect already running',
    '기록 중 — 먼저 정지': 'recording — stop it first',
    '이미 기록 중': 'already recording',
    '에피소드 없음': 'no episodes',
    '기록 중인 데이터셋': 'dataset is being recorded',
    '실행 중인 모델': 'model in use',
    '에피소드 id': 'episode id',
    'zip 아님': 'not a zip',
    'zip 안 경로 이상': 'bad path in zip',
    'zip 안에 meta.json 이 하나 있어야 함 (trainer 가 만든 model zip)': 'zip must contain exactly one meta.json (a model zip from the trainer)',
    '이 로봇과 안 맞는 모델': "model doesn't match this robot",
    'joint_names 가 이 로봇 팔 14축 순서와 다름 (robot.yaml joints.arm)': "joint_names differ from this robot's 14 arm joints order (robot.yaml joints.arm)",
    'state_dim/action_dim 은 14': 'state_dim/action_dim must be 14',
    '이름은 영문·숫자·_.- 1~64자': 'name must be 1–64 chars of letters / digits / _.-',
    '모델 이름은 영문·숫자·_.- 1~64자': 'model name must be 1–64 chars of letters / digits / _.-',
    '로드 실패하면 학습 PC 를 같은 버전으로': 'if loading fails, use the same version on the training PC',
    '학습 PC': 'training PC',
    'DDS 구독 실패': 'DDS subscribe failed',
    '끝 자세까지': 'to end pose',
    '시작 자세에서 최대': 'max from start pose',
    '카메라': 'Camera',
    '모델 없음': 'no model',
    '모델 입력 이름': 'model input names',
    '필요': 'required',
    // ===== 공통 단어 (앞뒤가 한글이 아닐 때만 바뀜) =====
    '연결 끊김': 'disconnected',
    '확인 불가': 'unknown',
    '확인 중': 'checking',
    '실행 중': 'running',
    '전송 중': 'sending',
    '전환 중': 'switching',
    '시간 초과': 'Timeout',
    '정지됨': 'Stopped',
    '취소됨': 'cancelled',
    '미연결': 'not connected',
    '연결됨': 'connected',
    '오른손': 'Right hand',
    '왼손': 'Left hand',
    '양손': 'Both',
    '경고': 'Warning',
    '대기': 'Idle',
    '취소': 'Cancel',
    '실행': 'Run',
    '정지': 'Stop',
    '거부': 'rejected',
    '실패': 'failed',
    '오류': 'error',
    '종료': 'exit',
    '완료': 'done',
    '중단': 'aborted',
    '최근': 'recent',
    '연결': 'Link',
    '작업': 'Task',
    '걷기': 'walk',
    '추론': 'Inference',
    '신뢰도': 'Confidence',
    '거리': 'Distance',
    '높이': 'H',
    '기본': 'Default',
    '부하': 'Load',
    '자동 회전': 'Auto orbit',
    '천천히 좌우로 회전 (마우스 조작 시 멈추고 5s 후 재개)': 'Slow side-to-side orbit (pauses on mouse input, resumes after 5 s)',
    '끊김': 'lost',
    '현재': 'now',
    '에서만': 'only',
    '이동': 'Move',
    '전진': 'Forward',
    '후진': 'Back',
    '좌측': 'Left',
    '우측': 'Right',
    '없음': 'None',
    '포함': 'Include',
    '유형': 'Type',
    '내용': 'Content',
    '시간': 'Time',
    '재생': 'Play',
    '저장': 'Save',
    '자세': 'pose',
    '허리': 'waist',
    '팔': 'arm',
  };

  // 숫자가 들어간 문구 (사전보다 먼저)
  const RULES = [
    [/시작 자세에서 최대 (\d+)° \(완료는 (\d+)° 넘게 움직인 뒤\)/g, 'max $1° from start pose (done needs > $2° of motion first)'],
    [/끝 자세까지 ([\d.]+)° \(([^,)]+), 완료는 (\d+)° 안\)/g, '$1° to end pose ($2, done within $3°)'],
    [/데이터셋 '([^']*)' 의 에피소드를 전부 지웁니다\. 되돌릴 수 없음\. 계속\?/g, "Delete all episodes of dataset '$1'? This cannot be undone. Continue?"],
    [/^모델 (\S+) 삭제\?$/, 'Delete model $1?'],
    [/^(\S+) 삭제\?$/, 'Delete $1?'],
    [/끝 — (\d+)s 동안 시작 조건 안 맞음:/g, 'finished — start conditions not met for $1s:'],
    [/끝 — (\d+) 회째 정지:/g, 'finished — stopped on run $1:'],
    [/범위 (\d+)–(\d+) cm 밖/g, 'outside $1–$2 cm'],
    [/연속 거부 (\d+)회/g, '$1 rejects in a row'],
    [/= 최대의 (\d+)%/g, '= $1% of max'],
    [/끊김 누적 (\d+)초 넘음/g, 'lost for over $1 s in total'],
    [/(\d+)초 넘음/g, 'over $1 s'],
    [/zip 안에 (\S+) 없음/g, 'zip has no $1'],
    [/(\d+(?:\.\d+)?)s 전/g, '$1s ago'],
    [/FSM 은 (\([^)]*\)) 중 하나/g, 'FSM must be one of $1'],
    [/(\d+)초 후 전송/g, 'sending in $1s'],
    [/(\d+)분 (\d+)초/g, '$1m $2s'],
    [/(\d+)개 서버/g, '$1 servers'],
    [/(\d+(?:\.\d+)?)초/g, '$1s'],
  ];

  const HANGUL = /[가-힣]/;
  const esc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  const PHRASES = Object.keys(DICT).sort((a, b) => b.length - a.length)
    // 키의 공백은 줄바꿈·들여쓰기도 맞춤 (HTML 안내문이 여러 줄로 나뉘어 있어도 같은 문장)
    .map(k => [new RegExp('(?<![가-힣])' + esc(k).replace(/ /g, '\\s+') + '(?![가-힣])', 'g'), DICT[k]]);

  function tr(s) {
    if (!s || !HANGUL.test(s)) return s;
    for (const [re, to] of RULES) s = s.replace(re, to);
    for (const [re, to] of PHRASES) { if (!HANGUL.test(s)) break; s = s.replace(re, to); }
    return s;
  }

  // ---------- 상태 ----------
  const q = new URLSearchParams(location.search);
  let lang = q.get('lang');
  if (lang !== 'en' && lang !== 'ko') {
    try { lang = localStorage.getItem('g1_lang'); } catch (e) { lang = null; }
  }
  if (lang !== 'en') lang = 'ko';
  const embed = q.get('embed') === '1';

  const ORIG = '__g1ko', DONE = '__g1en';
  const ATTRS = ['title', 'placeholder'];

  function trNode(n) {
    if (n.nodeType === 3) {                         // 텍스트
      const p = n.parentNode;
      if (p && (p.nodeName === 'SCRIPT' || p.nodeName === 'STYLE')) return;
      const v = n.nodeValue;
      if (lang === 'en') {
        if (HANGUL.test(v)) { const t = tr(v); if (t !== v) { n[ORIG] = v; n[DONE] = t; n.nodeValue = t; } }
      } else if (n[ORIG] !== undefined && v === n[DONE]) {
        n.nodeValue = n[ORIG]; delete n[ORIG]; delete n[DONE];
      }
    } else if (n.nodeType === 1) {                  // 요소 속성
      for (const a of ATTRS) {
        const v = n.getAttribute && n.getAttribute(a);
        if (v == null) continue;
        const ko = '__g1ko_' + a, en = '__g1en_' + a;
        if (lang === 'en') {
          if (HANGUL.test(v)) { const t = tr(v); if (t !== v) { n[ko] = v; n[en] = t; n.setAttribute(a, t); } }
        } else if (n[ko] !== undefined && v === n[en]) { n.setAttribute(a, n[ko]); delete n[ko]; delete n[en]; }
      }
    }
  }
  function walk(root) {
    if (!root) return;
    trNode(root);
    const it = document.createTreeWalker(root, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
    let n; while ((n = it.nextNode())) trNode(n);
  }

  let busy = false;
  const mo = new MutationObserver(muts => {
    if (busy || lang !== 'en') return;
    busy = true;
    try {
      for (const m of muts) {
        if (m.type === 'characterData') trNode(m.target);
        else if (m.type === 'attributes') trNode(m.target);
        else m.addedNodes.forEach(walk);
      }
    } finally { busy = false; }
  });

  // alert / confirm / prompt 문구
  const _alert = window.alert.bind(window), _confirm = window.confirm.bind(window), _prompt = window.prompt.bind(window);
  window.alert = m => _alert(lang === 'en' ? tr(String(m)) : m);
  window.confirm = m => _confirm(lang === 'en' ? tr(String(m)) : m);
  window.prompt = (m, d) => _prompt(lang === 'en' ? tr(String(m)) : m, d);

  // ---------- 버튼 ----------
  function renderBtn() {
    if (embed) return;
    let b = document.getElementById('g1-lang-btn');
    if (!b) {
      b = document.createElement('button');
      b.id = 'g1-lang-btn'; b.type = 'button';
      b.title = 'Language / 언어';
      b.style.cssText = 'font:600 11px/1 ui-monospace,Menlo,monospace;border-radius:999px;cursor:pointer;' +
        'padding:5px 10px;background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;letter-spacing:.5px;white-space:nowrap';
      b.addEventListener('click', () => setLang(lang === 'en' ? 'ko' : 'en'));
      const slot = document.getElementById('lang-slot');
      if (slot) slot.appendChild(b);
      else { b.style.position = 'fixed'; b.style.right = '12px'; b.style.bottom = '12px'; b.style.zIndex = '9999'; document.body.appendChild(b); }
    }
    b.innerHTML = lang === 'en'
      ? '<span style="opacity:.45">한</span> | <b style="color:#4aa8ff">EN</b>'
      : '<b style="color:#4aa8ff">한</b> | <span style="opacity:.45">EN</span>';
  }

  let domReady = document.readyState !== 'loading';
  function setLang(l, fromParent) {
    lang = l === 'en' ? 'en' : 'ko';
    try { localStorage.setItem('g1_lang', lang); } catch (e) {}
    document.documentElement.lang = lang;
    busy = true; try { walk(document.body); } finally { busy = false; }
    if (domReady) renderBtn();          // 버튼 자리(#lang-slot)가 파싱된 뒤에 그린다
    // iframe 에도 전달 (포트가 다르면 localStorage 공유가 안 됨)
    if (!fromParent) document.querySelectorAll('iframe').forEach(f => {
      try { f.contentWindow.postMessage({ g1lang: lang }, '*'); } catch (e) {}
    });
    window.dispatchEvent(new CustomEvent('g1lang', { detail: lang }));
  }
  window.addEventListener('message', e => {
    if (e.data && (e.data.g1lang === 'en' || e.data.g1lang === 'ko') && e.data.g1lang !== lang) setLang(e.data.g1lang, true);
  });

  window.G1I18N = { get lang() { return lang; }, setLang, tr };

  function start() {
    mo.observe(document.body, { childList: true, subtree: true, characterData: true, attributes: true, attributeFilter: ATTRS });
    setLang(lang, true);
    if (!domReady) document.addEventListener('DOMContentLoaded', () => { domReady = true; renderBtn(); });
  }
  if (document.body) start(); else document.addEventListener('DOMContentLoaded', start);
})();
