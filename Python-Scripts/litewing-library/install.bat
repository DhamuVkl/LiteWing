@echo off
setlocal enabledelayedexpansion

echo.
echo   ============================================
echo        LiteWing Library Installer
echo   ============================================
echo.

REM --- Configuration ---
set "PYTHON_VERSION=3.11.9"
set "PYTHON_INSTALLER=python-%PYTHON_VERSION%-amd64.exe"
set "PYTHON_URL=https://www.python.org/ftp/python/%PYTHON_VERSION%/%PYTHON_INSTALLER%"
set "PYTHON_INSTALL_DIR=%LOCALAPPDATA%\Programs\Python\Python311"

set "GIT_VERSION=2.47.1.2"
set "GIT_INSTALLER=Git-%GIT_VERSION%-64-bit.exe"
set "GIT_URL=https://github.com/git-for-windows/git/releases/download/v%GIT_VERSION%.windows.1/%GIT_INSTALLER%"

REM =========================================
REM  STEP 1: Check System Date and Time
REM =========================================
echo   [Step 1/4] Checking system date and time...

for /f %%y in ('powershell -NoProfile -Command "(Get-Date).Year"') do set "CURRENT_YEAR=%%y"

if "!CURRENT_YEAR!" == "" (
    echo   [WARN] Could not read system date.
    echo          SSL downloads may fail if the clock is wrong.
    echo.
    goto :date_done
)

if !CURRENT_YEAR! LSS 2025 goto :date_wrong
if !CURRENT_YEAR! GTR 2030 goto :date_wrong

echo   [OK] System date: %DATE% %TIME%
echo.
goto :date_done

:date_wrong
echo.
echo   [ERROR] System date appears to be wrong!
echo           Current date: %DATE% %TIME%
echo           Expected year: 2025 or later
echo.
echo   Downloads will FAIL with incorrect date (SSL errors).
echo.
echo   Fix: Right-click the clock in taskbar - Adjust date/time
echo        Turn ON "Set time automatically"
echo.
echo   After fixing the date, run this installer again.
echo.
pause
exit /b 1

:date_done

REM =========================================
REM  STEP 2: Check / Install Python 3.11
REM =========================================
echo   [Step 2/4] Checking for Python 3.11...

set "PY311_CMD="

REM Method 1: py launcher
py -3.11 --version >nul 2>&1
if not errorlevel 1 (
    set "PY311_CMD=py -3.11"
    goto :python_found
)

REM Method 2: default python command
for /f "tokens=2 delims= " %%v in ('python --version 2^>^&1') do (
    echo %%v | findstr /b "3.11" >nul
    if not errorlevel 1 (
        set "PY311_CMD=python"
        goto :python_found
    )
)

REM Method 3: common install path
if exist "%PYTHON_INSTALL_DIR%\python.exe" (
    set "PY311_CMD=%PYTHON_INSTALL_DIR%\python.exe"
    goto :python_found
)

REM --- Not found — download and install ---
echo   [!] Python 3.11 not found. Installing automatically...
echo.

call :download_file "%PYTHON_URL%" "%TEMP%\%PYTHON_INSTALLER%" "Python %PYTHON_VERSION%"
if errorlevel 1 (
    echo.
    echo   [ERROR] Python download failed!
    echo   Manual download: https://www.python.org/downloads/release/python-3119/
    echo.
    pause
    exit /b 1
)

echo.
echo   Installing Python %PYTHON_VERSION%...
echo   (This takes 1-2 minutes, please wait...)
echo.

"%TEMP%\%PYTHON_INSTALLER%" /quiet InstallAllUsers=0 PrependPath=1 Include_test=0 Include_launcher=1

if errorlevel 1 (
    echo   [ERROR] Python installation failed!
    echo   Try running the installer manually: %TEMP%\%PYTHON_INSTALLER%
    echo.
    pause
    exit /b 1
)

echo   [OK] Python %PYTHON_VERSION% installed!
echo.
del "%TEMP%\%PYTHON_INSTALLER%" >nul 2>&1

REM Refresh PATH for this session
set "PATH=%PYTHON_INSTALL_DIR%;%PYTHON_INSTALL_DIR%\Scripts;%PATH%"
set "PY311_CMD=%PYTHON_INSTALL_DIR%\python.exe"

:python_found
echo   [OK] Python: !PY311_CMD!
for /f "tokens=*" %%v in ('!PY311_CMD! --version 2^>^&1') do echo        %%v
echo.

REM =========================================
REM  STEP 3: Check / Install Git
REM =========================================
echo   [Step 3/4] Checking for Git...

git --version >nul 2>&1
if not errorlevel 1 (
    for /f "tokens=*" %%v in ('git --version 2^>^&1') do echo   [OK] %%v
    echo.
    goto :git_found
)

REM Check common install location
if exist "%ProgramFiles%\Git\cmd\git.exe" (
    set "PATH=%ProgramFiles%\Git\cmd;%PATH%"
    for /f "tokens=*" %%v in ('git --version 2^>^&1') do echo   [OK] %%v
    echo.
    goto :git_found
)

REM --- Git not found — download and install ---
echo   [!] Git not found. Installing automatically...
echo.

call :download_file "%GIT_URL%" "%TEMP%\%GIT_INSTALLER%" "Git %GIT_VERSION%"
if errorlevel 1 (
    echo.
    echo   [ERROR] Git download failed!
    echo   Manual download: https://git-scm.com/download/win
    echo.
    pause
    exit /b 1
)

echo.
echo   Installing Git %GIT_VERSION%...
echo   (This takes 1-2 minutes, please wait...)
echo.

"%TEMP%\%GIT_INSTALLER%" /VERYSILENT /NORESTART /NOCANCEL /SP- /CLOSEAPPLICATIONS /RESTARTAPPLICATIONS

if errorlevel 1 (
    echo   [ERROR] Git installation failed!
    echo   Try running the installer manually: %TEMP%\%GIT_INSTALLER%
    echo.
    pause
    exit /b 1
)

echo   [OK] Git installed!
echo.
del "%TEMP%\%GIT_INSTALLER%" >nul 2>&1

REM Refresh PATH for this session
set "PATH=%ProgramFiles%\Git\cmd;%PATH%"

git --version >nul 2>&1
if errorlevel 1 (
    echo   [WARN] Git installed but not detected in PATH.
    echo          Please close this window and run install.bat again.
    echo.
    pause
    exit /b 1
)
for /f "tokens=*" %%v in ('git --version 2^>^&1') do echo   [OK] %%v
echo.

:git_found

REM =========================================
REM  STEP 4: Install LiteWing Library
REM =========================================
echo   [Step 4/4] Installing LiteWing library...
echo   (Downloads cflib + matplotlib = may take a few minutes)
echo.

!PY311_CMD! -m pip install --upgrade pip >nul 2>&1
!PY311_CMD! -m pip install -e .

if errorlevel 1 (
    echo.
    echo   [ERROR] LiteWing installation failed!
    echo   Try: !PY311_CMD! -m pip install -e . --verbose
    echo.
    pause
    exit /b 1
)

REM =========================================
REM  Verify
REM =========================================
echo.
!PY311_CMD! -c "from litewing import LiteWing; print('  [OK] LiteWing library loaded successfully!')" 2>nul

echo.
echo   ============================================
echo        Installation Complete!
echo   ============================================
echo.
echo   All prerequisites installed:
echo     [OK] Python 3.11
echo     [OK] Git
echo     [OK] LiteWing (cflib + matplotlib)
echo.
echo   To fly your drone:
echo     1. Turn on the drone
echo     2. Connect to the drone's WiFi network
echo     3. Run: !PY311_CMD! examples\level_1\01_battery_voltage.py
echo.
pause
exit /b 0

REM =========================================
REM  Helper: Download a file using PowerShell
REM  Usage: call :download_file "URL" "OUTPUT_PATH" "LABEL"
REM =========================================
:download_file
set "DL_URL=%~1"
set "DL_OUT=%~2"
set "DL_LABEL=%~3"

echo   Downloading %DL_LABEL%...

REM Write PowerShell script to temp file to avoid escaping issues
set "PS_SCRIPT=%TEMP%\litewing_download.ps1"

echo [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12 > "%PS_SCRIPT%"
echo $url = '%DL_URL%' >> "%PS_SCRIPT%"
echo $out = '%DL_OUT%' >> "%PS_SCRIPT%"
echo Write-Host '  Downloading...' -ForegroundColor Cyan >> "%PS_SCRIPT%"
echo try { >> "%PS_SCRIPT%"
echo     Invoke-WebRequest -Uri $url -OutFile $out -UseBasicParsing >> "%PS_SCRIPT%"
echo     Write-Host '  Download complete!' -ForegroundColor Green >> "%PS_SCRIPT%"
echo } catch { >> "%PS_SCRIPT%"
echo     Write-Host ('  Error: ' + $_.Exception.Message) -ForegroundColor Red >> "%PS_SCRIPT%"
echo     exit 1 >> "%PS_SCRIPT%"
echo } >> "%PS_SCRIPT%"

powershell -NoProfile -ExecutionPolicy Bypass -File "%PS_SCRIPT%"
set "DL_RESULT=%ERRORLEVEL%"

del "%PS_SCRIPT%" >nul 2>&1

if not exist "%DL_OUT%" exit /b 1
if %DL_RESULT% NEQ 0 exit /b 1
exit /b 0
