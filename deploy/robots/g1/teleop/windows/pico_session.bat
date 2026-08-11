@echo off
REM PICO teleop session setup - double-click wrapper.
REM Running the .ps1 directly is blocked by ExecutionPolicy, so launch it with Bypass.
REM Args pass through:  pico_session.bat -CheckOnly
REM                     pico_session.bat -Com1 192.168.50.211 -Publish
REM
REM NOTE: pico_session.ps1 MUST stay UTF-8 *with BOM*. Windows PowerShell 5.1 reads a
REM       BOM-less file as ANSI (cp949 on Korean systems); 3-byte Korean UTF-8 chars then
REM       decode as 2-byte DBCS and the leftover byte swallows the following newline,
REM       which breaks here-string terminators. Do not re-save without the BOM.

setlocal
set "HERE=%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%HERE%pico_session.ps1" %*
echo.
pause
