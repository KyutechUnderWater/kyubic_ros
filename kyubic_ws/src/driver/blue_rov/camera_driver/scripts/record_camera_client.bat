@echo off
REM Windows 地上PC で BlueROV カメラ映像を録画するラッパー。
REM 事前に FFmpeg をインストールし、ロボット側で camera_driver を起動してください。

setlocal
set "SCRIPT_DIR=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%record_camera_client.ps1" %*
