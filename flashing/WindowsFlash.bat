:: Windows Flash Script for ESP8266
:: Usage: WindowsFlash.bat <baudrate> <COM port> <esptool path> <flash file>
:: Example: WindowsFlash.bat 921600 COM15 C:\path\to\esptool.py C:\path\to\flashfile.bin
REM This script continuously flashes the ESP8266 with the specified parameters.
REM Make sure to run this script in a command prompt with administrative privileges.

@echo off   

set baudrate=%1
set COM=%2
set esptool=%3
set flashfile=%4
:loop
::python C:\Users\mbgpcsk4\.platformio\packages\tool-esptoolpy\esptool.py --after soft_reset --chip esp8266 --port "COM15" --baud 921600 write_flash 0x0 C:\Work\Kahuna\.pio\build\esp07s\mavesp-esp07s-2.4.0.bin
python %esptool% --after soft_reset --chip esp8266 --port %COM% --baud %baudrate% write_flash 0x0 %flashfile%

TIMEOUT /T 3
goto loop

:: WindowsFlash.bat 921600 COM15 C:\Users\mbgpcsk4\.platformio\packages\tool-esptoolpy\esptool.py C:\Work\Kahuna\.pio\build\esp07s\mavesp-esp07s-2.4.0.bin