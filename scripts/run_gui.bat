@echo off
setlocal
cd /d "%~dp0\.."
python -m app.main %*
if errorlevel 1 (
  echo.
  echo J1 GUI exited with an error.
  pause
  exit /b 1
)
