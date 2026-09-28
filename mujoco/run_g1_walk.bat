@echo off
rem Double-click: G1 walks in MuJoCo with Unitree's pretrained policy.
rem Extra args go to g1_walk.py, e.g.  run_g1_walk.bat --vx 0.3
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
set PY=D:\python\python.exe
if not exist "%PY%" set PY=python
"%PY%" g1_walk.py %*
if errorlevel 1 pause
