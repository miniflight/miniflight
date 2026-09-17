@echo off
setlocal
set "SIM_DIR=%~dp0AI-GP Simulator v1.0.3391-VQ1\AIGP_VQ1_3391"
set "SHIPPING=%SIM_DIR%\FlightSim\Binaries\Win64\DCGame-Win64-Shipping.exe"

if not exist "%SHIPPING%" (
    echo Run python target/aigp/extract_vq1.py first.
    exit /b 1
)

pushd "%SIM_DIR%"
"%SHIPPING%" "/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase" -windowed -ResX=1280 -ResY=720 -nosound -NoSplash %*
set "SIM_STATUS=%ERRORLEVEL%"
popd
exit /b %SIM_STATUS%
