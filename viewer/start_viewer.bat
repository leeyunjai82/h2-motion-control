@echo off
rem ACT 데이터셋 · 모델 보기 — 더블클릭 (처음 한 번 viewer\.venv 가상환경 설치). zip 은 viewer\data 에 넣어 둠
chcp 65001 >nul
cd /d "%~dp0"
set VENV=%~dp0.venv
if exist "%VENV%\Scripts\python.exe" goto run
where python >nul 2>nul
if errorlevel 1 (
  echo [act_viewer] Python 없음 - https://www.python.org 에서 3.8 이상 설치 ^(Add python.exe to PATH 체크^) 후 다시 실행
  pause
  exit /b 1
)
echo [act_viewer] 가상환경 만드는 중: %VENV%
python -m venv "%VENV%"
if errorlevel 1 (
  echo [act_viewer] 가상환경 만들기 실패
  pause
  exit /b 1
)
"%VENV%\Scripts\python.exe" -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 (
  echo [act_viewer] 패키지 설치 실패 - 인터넷 연결 확인. 다시 설치하려면 viewer\.venv 폴더를 지우고 실행
  rmdir /s /q "%VENV%"
  pause
  exit /b 1
)
:run
"%VENV%\Scripts\python.exe" "%~dp0act_viewer.py" %*
pause
