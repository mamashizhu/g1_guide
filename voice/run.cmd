@echo off
rem Voice guide launcher: works from any directory, passes through arguments (e.g. --no-robot, --text).
"%~dp0.venv\Scripts\python.exe" "%~dp0app.py" %*
