@echo off
setlocal

rem =====================================================
rem Usage:
rem   post_build_bin.bat "<TARGET_PATH>" "<TOOLKIT_DIR>"
rem Example from IAR Post Build Command Line:
rem   "$PROJ_DIR$\post_build_bin.bat" "$TARGET_PATH$" "$TOOLKIT_DIR$"
rem
rem %1 = .out full path
rem %2 = IAR toolkit directory, for example:
rem      C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.4\arm
rem =====================================================

rem ===== input / output =====
set "OUT_FILE=%~1"
set "OUT_DIR=%~dp1"

rem ===== IAR ielftool path from parameter 2: TOOLKIT_DIR =====
set "TOOLKIT_DIR=%~2"
set "IELFTOOL=%TOOLKIT_DIR%\bin\ielftool.exe"

rem ===== check arguments =====
if "%OUT_FILE%"=="" (
    echo ERROR: No input .out file. Parameter 1 is empty.
    exit /b 1
)

if "%TOOLKIT_DIR%"=="" (
    echo ERROR: No IAR toolkit directory. Parameter 2 is empty.
    exit /b 1
)

if not exist "%OUT_FILE%" (
    echo ERROR: OUT file not found: "%OUT_FILE%"
    exit /b 1
)

if not exist "%IELFTOOL%" (
    echo ERROR: ielftool.exe not found: "%IELFTOOL%"
    exit /b 1
)

echo Input OUT : "%OUT_FILE%"
echo Output DIR: "%OUT_DIR%"
echo IELFTOOL  : "%IELFTOOL%"

rem ===== Generate APROM.bin =====
"%IELFTOOL%" "%OUT_FILE%" --bin=0x00000000-0x00007FFF "%OUT_DIR%APROM.bin"
if errorlevel 1 (
    echo ERROR: Generate APROM.bin failed
    exit /b 1
)

rem ===== Generate LDROM.bin =====
"%IELFTOOL%" "%OUT_FILE%" --bin=0x00100000-0x00100FFF "%OUT_DIR%LDROM.bin"
if errorlevel 1 (
    echo ERROR: Generate LDROM.bin failed
    exit /b 1
)

echo Generate APROM.bin and LDROM.bin successfully
exit /b 0
