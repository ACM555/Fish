@echo off
rem =====================================================================
rem  fishmon launcher  --  double-click this file to start monitoring.
rem  This file is intentionally pure ASCII: cmd.exe reads .bat files using
rem  the OEM code page (936/GBK on this machine), so UTF-8 Chinese comments
rem  or messages would corrupt parsing. All Chinese output comes from
rem  fishmon.py, which switches the console to UTF-8 at startup.
rem
rem  It only OBSERVES. It never modifies test.py.
rem =====================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"
title fishmon - fish robot obstacle race monitor

set "PYEXE="
for %%P in (
  "D:\Python311\python.exe"
  "D:\Python310\python.exe"
  "D:\Python312\python.exe"
  "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
  "%LOCALAPPDATA%\Programs\Python\Python310\python.exe"
) do (
  if not defined PYEXE if exist %%P (
    %%P -c "import psutil" >nul 2>&1
    if !errorlevel! equ 0 set "PYEXE=%%~P"
  )
)

if not defined PYEXE (
  for %%C in (python.exe py.exe) do (
    if not defined PYEXE (
      where %%C >nul 2>&1
      if !errorlevel! equ 0 (
        %%C -c "import psutil" >nul 2>&1
        if !errorlevel! equ 0 set "PYEXE=%%C"
      )
    )
  )
)

if not defined PYEXE goto :nopsutil

echo [launcher] python: %PYEXE%
"%PYEXE%" -B fishmon.py --start-if-present --stop-when-idle
goto :done

:nopsutil
echo [ERROR] No Python interpreter with psutil was found.
echo         This tool needs psutil to read process/thread/CPU/memory metrics.
echo.
echo   Fix (either one):
echo     1) install it:  D:\Python311\python.exe -m pip install psutil
echo     2) edit the candidate list at the top of this .bat file.
echo.
echo   Note: the Python bundled with the simulator is 3.7 and has NO psutil.

:done
echo.
echo [launcher] finished. Reports are in the monitor_reports\ folder.
pause
endlocal
