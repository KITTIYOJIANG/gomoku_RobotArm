@echo off
setlocal
cd /d "%~dp0\.."
set "J1_PYTHON=python"
if exist "D:\Anaconda\python.exe" (
  "D:\Anaconda\python.exe" -c "import PySide6" >nul 2>&1
  if not errorlevel 1 set "J1_PYTHON=D:\Anaconda\python.exe"
)
"%J1_PYTHON%" -m app.main %*
if errorlevel 1 (
  echo.
  echo J1 GUI exited with an error.
  pause
  exit /b 1
)
