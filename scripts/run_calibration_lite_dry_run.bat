@echo off
setlocal
cd /d "%~dp0.."
"D:\Anaconda\python.exe" -m app.calibration_lite.main --dry-run
endlocal
