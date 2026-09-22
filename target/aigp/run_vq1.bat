@echo off
setlocal
set "SIM_DIR=%~dp0.runtime\vq1"
set "SHIPPING=%SIM_DIR%\FlightSim\Binaries\Win64\DCGame-Win64-Shipping.exe"

python "%~dp0extract_vq1.py"
if errorlevel 1 exit /b 1

pushd "%SIM_DIR%"
"%SHIPPING%" "/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase" -windowed -ResX=1280 -ResY=720 -nosound -NoSplash %*
set "SIM_STATUS=%ERRORLEVEL%"
popd
exit /b %SIM_STATUS%
