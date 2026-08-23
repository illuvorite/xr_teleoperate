@echo off
chcp 65001 >nul
echo ==========================================
echo   xr_teleoperate ?????????
echo ==========================================
echo.

REM ?? Git
where git >nul 2>nul
if %errorlevel% neq 0 (
    echo [??] git ???
    pause
    exit /b 1
)

REM ????
set "SCRIPT_DIR=%~dp0"
set "PROJECT_ROOT=%SCRIPT_DIR%\..\.."
set "OUTPUT_DIR=%PROJECT_ROOT%\dist"

REM ??????
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

cd /d "%PROJECT_ROOT%"

REM ??????
echo [1/4] ??? Git ???...
git submodule update --init --depth 1

REM ???????
echo [2/4] ???????...
powershell -Command "Compress-Archive -Path '.' -DestinationPath '%OUTPUT_DIR%\xr_teleoperate-source.zip' -Exclude '.git','.gitmodules','.kilo','__pycache__','*.pyc','*.log','data','guidelogs','.env','dist','node_modules'"

REM ??????
echo [3/4] ??????...
pip freeze > "%OUTPUT_DIR%\requirements-frozen.txt"

REM ??????
echo [4/4] ??????...
copy "%SCRIPT_DIR%\offline_deploy.sh" "%OUTPUT_DIR%\install.sh" >nul

echo.
echo ==========================================
echo   ?????
echo ==========================================
echo.
echo ????: %OUTPUT_DIR%
echo.
dir /b "%OUTPUT_DIR%"
echo.
echo ????????:
echo   1. ?? dist\ ??? Ubuntu 20.04 ??
echo   2. cd xr_teleoperate/dist
echo   3. bash install.sh
echo   4. ?? .env ???
echo   5. ??????
echo.
pause
