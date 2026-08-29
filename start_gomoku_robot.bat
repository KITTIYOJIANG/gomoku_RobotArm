@echo off
setlocal
cd /d "%~dp0"

set "GOMOKU_PYTHON=D:\Anaconda\python.exe"
if not exist "%GOMOKU_PYTHON%" set "GOMOKU_PYTHON=python"

"%GOMOKU_PYTHON%" -c "import PySide6, cv2, serial"
if errorlevel 1 (
    echo.
    echo Gomoku Robot environment check failed.
    echo Run scripts\install_dependencies.bat or use the supported Python environment.
    pause
    exit /b 1
)

echo Starting Gomoku Robot Integrated V1.
echo Camera and serial will NOT auto-connect.
"%GOMOKU_PYTHON%" -m app.main
if errorlevel 1 pause
