@echo off
rem Voice guide console window: works from any directory, passes through arguments (e.g. --no-robot).
start "" "%~dp0.venv\Scripts\pythonw.exe" "%~dp0gui.py" %*
