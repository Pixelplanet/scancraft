@echo off
rem Starts ScanCraft and writes every scan's raw stream to the "recordings" folder next to this file.
rem Streams are several MB per second: delete recordings you no longer need.
set "SCANCRAFT_RECORD_DIR=%~dp0recordings"
if not exist "%SCANCRAFT_RECORD_DIR%" mkdir "%SCANCRAFT_RECORD_DIR%"
start "" "%~dp0scancraft.exe" %*
