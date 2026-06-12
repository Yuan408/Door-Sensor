@echo off
set MSYSTEM=
set OSTYPE=
set TERM=
cd /d D:\MCU_project\esp32\test
call D:\Espressif\v5.4.4\esp-idf\export.bat > D:\MCU_project\esp32\test\flash_output.txt 2>&1
echo === FLASH START === >> D:\MCU_project\esp32\test\flash_output.txt
idf.py -p COM6 flash >> D:\MCU_project\esp32\test\flash_output.txt 2>&1
echo === FLASH END, exit code: %ERRORLEVEL% === >> D:\MCU_project\esp32\test\flash_output.txt
