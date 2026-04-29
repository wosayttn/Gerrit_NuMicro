@echo off
setlocal EnableExtensions

echo +============================================================
echo ==========================================
echo Post Build - Generate APROM / LDROM
echo ==========================================

REM ============================================================
REM Project setting
REM ============================================================
set "PROJECT=UTCPD_Source_180W"
set "TOOLCHAIN=ARMCLANG"
set "CONFIG=Release"

REM Batch file location should be under:
REM ...\VSCode\post_build.bat
set "ROOT=%~dp0"
set "OUTDIR=%ROOT%out\%PROJECT%\%TOOLCHAIN%\%CONFIG%"
set "ELF=%OUTDIR%\%PROJECT%.axf"

echo [INFO] ELF = %ELF%

if not exist "%ELF%" (
    echo [ERROR] ELF file not found.
    exit /b 1
)

REM ============================================================
REM Find fromelf.exe
REM Priority:
REM   1. FROMELF env
REM   2. where fromelf
REM   3. common Keil path
REM ============================================================
set "FROMELF="

if defined FROMELF (
    if exist "%FROMELF%" (
        set "FROMELF_EXE=%FROMELF%"
        goto :FOUND_FROMELF
    )
)

for /f "delims=" %%i in ('where fromelf 2^>nul') do (
    set "FROMELF_EXE=%%i"
    goto :FOUND_FROMELF
)

if exist "C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe" (
    set "FROMELF_EXE=C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe"
    goto :FOUND_FROMELF
)

if exist "%LocalAppData%\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe" (
    set "FROMELF_EXE=%LocalAppData%\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe"
    goto :FOUND_FROMELF
)

echo [ERROR] fromelf.exe not found.
echo [ERROR] Please install ARM Compiler 6 / Keil MDK or add fromelf to PATH.
exit /b 1

:FOUND_FROMELF
echo [INFO] FROMELF = %FROMELF_EXE%

REM ============================================================
REM Clean old output files
REM ============================================================
if exist "%OUTDIR%\*.bin" del /f /q "%OUTDIR%\*.bin" >nul 2>nul

REM ============================================================
REM Generate BIN files
REM Note:
REM fromelf --bin will generate one or more .bin files based on
REM load regions in the AXF/scatter file.
REM ============================================================
echo [INFO] Generating binary files ...
"%FROMELF_EXE%" --bin --output="%OUTDIR%\%PROJECT%.bin" "%ELF%"
if errorlevel 1 (
    echo [ERROR] fromelf failed.
    exit /b 1
)

echo.
echo [INFO] Output files:
dir "%OUTDIR%\*.bin"

echo.
echo [INFO] Post build completed successfully.
exit /b 0