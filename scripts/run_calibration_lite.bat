@echo off
setlocal
cd /d "%~dp0.."
D:\Anaconda\python.exe -m app.calibration_lite.main %*
if errorlevel 1 pause
endlocal
