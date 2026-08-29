@echo off
setlocal
cd /d "%~dp0\.."
set "GOMOKU_PYTHON=D:\Anaconda\python.exe"
if not exist "%GOMOKU_PYTHON%" set "GOMOKU_PYTHON=python"
"%GOMOKU_PYTHON%" -m app.main --dry-run --test-pattern
if errorlevel 1 pause
