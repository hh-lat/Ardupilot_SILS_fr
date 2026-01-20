@echo off

REM ===== USER-SPECIFIC PATHS =====
set FGDIR=C:\Program Files\FlightGear 2024.1
set FGDATA=C:\Users\rajat\FlightGear\Downloads\fgdata_2024_1
set AUTOTESTDIR=%~dp0\aircraft

echo Using FlightGear binary: %FGDIR%
echo Using FlightGear data:   %FGDATA%

cd "%FGDIR%\bin"

fgfs ^
 --fg-root="%FGDATA%" ^
 --native-fdm=socket,in,10,127.0.0.1,5503,udp ^
 --fdm=external ^
 --aircraft=Rascal110-JSBSim ^
 --fg-aircraft="%AUTOTESTDIR%" ^
 --airport=KSFO ^
 --geometry=650x550 ^
 --timeofday=noon ^
 --disable-hud-3d ^
 --disable-horizon-effect ^
 --disable-sound ^
 --disable-fullscreen ^
 --disable-random-objects ^
 --disable-ai-models ^
 --fog-disable ^
 --disable-specular-highlight ^
 --disable-anti-alias-hud ^
 --wind=0@0

pause
