@echo off
chcp 65001 >nul
echo ==========================================
echo   xr_teleoperate ???? (Windows)
echo ==========================================
echo.

REM ?? Git
where git >nul 2>nul
if %errorlevel% neq 0 (
    echo [??] git ???????? Git
    pause
    exit /b 1
)

REM ?? Docker
docker info >nul 2>&1
if %errorlevel% neq 0 (
    echo [??] Docker ???????? Docker Desktop
    pause
    exit /b 1
)

REM ????
set "SCRIPT_DIR=%~dp0"
set "DEPLOY_DIR=%SCRIPT_DIR%.."

REM ?? .env ??
if not exist "%DEPLOY_DIR%\.env" (
    echo [??] ?? .env ??...
    copy "%DEPLOY_DIR%\.env.example" "%DEPLOY_DIR%\.env" >nul
    echo [??] ??? .env ????? IMG_SERVER_IP ???
    pause
)

REM ????
echo.
echo [1/3] ?? Docker ??...
cd /d "%DEPLOY_DIR%"
docker compose build
if %errorlevel% neq 0 (
    echo [??] ????
    pause
    exit /b 1
)

REM ????
echo.
echo [2/3] ????...
set "XR_IMAGE=dopamineillusory/xr-teleoperate:latest"
set "TELE_IMAGE=dopamineillusory/teleimager:latest"

docker tag xr-teleoperate:latest %XR_IMAGE%
docker tag teleimager:latest %TELE_IMAGE%

REM ????
echo.
echo [3/3] ??? Docker Hub...
docker login
docker push %XR_IMAGE%
docker push %TELE_IMAGE%

echo.
echo ==========================================
echo   ?????
echo ==========================================
echo.
echo ????????:
echo   git clone --depth 1 https://github.com/illuvorite/xr_teleoperate.git
echo   cd xr_teleoperate/deploy
echo   docker compose -f docker-compose.remote.yml pull
echo   docker compose -f docker-compose.remote.yml up -d
echo.
pause
