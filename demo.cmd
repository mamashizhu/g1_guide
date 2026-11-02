@echo off
rem G1 guide full demo (README 5.10): WSL simulation + web console + vision + voice.
rem   demo.cmd                      start everything, then open http://localhost:8080
rem   demo.cmd headless:=true       extra arguments go to "ros2 launch g1_guide demo.launch.py"
rem   demo.cmd nosim                the simulation is already running in WSL
rem   set G1_VOICE_ARGS=--barge-in  extra arguments for voice\app.py (e.g. --wake, --offline)
rem Keep this file ASCII only: cmd reads batch files as GBK.
setlocal
set "ROOT=%~dp0"
if /i "%~1"=="nosim" goto wait_ros
start "G1 simulation (WSL) - close this window to stop" wsl -e bash -ic "ros2 launch g1_guide demo.launch.py %*"

:wait_ros
echo Waiting for rosbridge on localhost:9090 (the simulation takes 20-40 s to start) ...
set /a TRIES=0
:wait_loop
curl -s -o NUL http://localhost:9090 && goto ready
set /a TRIES+=1
if %TRIES% GEQ 90 (
  echo rosbridge did not come up. Check the WSL window, or start it by hand:
  echo   ros2 launch g1_guide demo.launch.py
  pause
  exit /b 1
)
timeout /t 2 /nobreak >NUL
goto wait_loop

:ready
echo rosbridge is up. Starting vision and voice, opening the web console.
start "G1 vision" "%ROOT%vision\.venv\Scripts\python.exe" "%ROOT%vision\watch.py"
start "G1 voice - press Enter to interrupt" "%ROOT%voice\.venv\Scripts\python.exe" "%ROOT%voice\app.py" %G1_VOICE_ARGS%
timeout /t 3 /nobreak >NUL
start "" http://localhost:8080
endlocal
