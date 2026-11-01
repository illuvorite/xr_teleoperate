@echo off
setlocal
where node >nul 2>nul
if errorlevel 1 (
  echo Node.js is required to build the deployment packages.
  exit /b 1
)
node "%~dp0package_release.cjs" %*
exit /b %errorlevel%
