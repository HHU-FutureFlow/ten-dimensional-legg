@echo off
setlocal
cd /d "%~dp0.."
where node >nul 2>nul
if errorlevel 1 (
  echo Please install Node.js 24 or newer, then run this script again.
  pause
  exit /b 1
)
call npm run setup
if errorlevel 1 goto failed
call npm run build
if errorlevel 1 goto failed
node --env-file-if-exists=.env scripts/launch.mjs
if errorlevel 1 goto failed
exit /b 0
:failed
echo Startup failed. See the error above and README.md.
pause
exit /b 1
