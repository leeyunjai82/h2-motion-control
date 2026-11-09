#!/usr/bin/env bash
# VR 데이터 수집 — 한 창에서: 그리퍼 서비스 → arm_server → 잡기 웹 → Quest 입력 (xr_virtual --real) → Quest 페이지 열기
#   ./start_collect.sh                 → 수집 화면 http://localhost:50030/collect
#   (= ./start_gripper.sh --vr — 다른 옵션도 그대로: --no-cam-check 등). 종료: Ctrl+C
#   학습은 따로: ./trainer/start_trainer_xpu.sh (로봇 PC Intel GPU) — 수집과 동시에 돌리지 말 것
exec "$(cd "$(dirname "$0")" && pwd)/start_gripper.sh" --vr "$@"
