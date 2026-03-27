@echo off
REM ============================================================================
REM NCLite ESP32-S3 Production Firmware Flasher
REM ============================================================================
REM Purpose: Flash production firmware to ESP32-S3 boards in factory/production
REM Usage:   flash_production.bat COM_PORT
REM Example: flash_production.bat COM3
REM ============================================================================

echo.
echo ========================================
echo  NCLite ESP32-S3 Production Flasher
echo ========================================
echo.

REM Check if COM port argument is provided
if "%1"=="" (
    echo ERROR: COM port not specified!
    echo.
    echo Usage: flash_production.bat COM_PORT
    echo Example: flash_production.bat COM3
    echo.
    echo Available COM ports:
    mode | findstr "COM"
    echo.
    pause
    exit /b 1
)

set PORT=%1
set BAUD=921600
set CHIP=esp32s3
set FLASH_MODE=dio
set FLASH_FREQ=80m
set FLASH_SIZE=8MB

echo Configuration:
echo   - Target Chip: %CHIP%
echo   - COM Port: %PORT%
echo   - Baud Rate: %BAUD%
echo   - Flash Mode: %FLASH_MODE%
echo   - Flash Freq: %FLASH_FREQ%
echo   - Flash Size: %FLASH_SIZE%
echo.

REM Check if esptool is installed
where esptool.py >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: esptool.py not found!
    echo.
    echo Please install esptool:
    echo   pip install esptool
    echo.
    pause
    exit /b 1
)

REM Check if firmware files exist
if not exist "build\bootloader\bootloader.bin" (
    echo ERROR: bootloader.bin not found!
    echo Please build the firmware first: idf.py build
    echo.
    pause
    exit /b 1
)

if not exist "build\partition_table\partition-table.bin" (
    echo ERROR: partition-table.bin not found!
    echo Please build the firmware first: idf.py build
    echo.
    pause
    exit /b 1
)

if not exist "build\nitara.connector.esp32s3.bin" (
    echo ERROR: nitara.connector.esp32s3.bin not found!
    echo Please build the firmware first: idf.py build
    echo.
    pause
    exit /b 1
)

echo ========================================
echo  Starting Flash Process...
echo ========================================
echo.
echo NOTE: If flashing fails, press and hold BOOT button on the board
echo.

REM Flash the firmware
esptool.py --chip %CHIP% --port %PORT% --baud %BAUD% ^
  --before default_reset --after hard_reset write_flash ^
  -z --flash_mode %FLASH_MODE% --flash_freq %FLASH_FREQ% --flash_size %FLASH_SIZE% ^
  0x0 build\bootloader\bootloader.bin ^
  0x8000 build\partition_table\partition-table.bin ^
  0x10000 build\nitara.connector.esp32s3.bin

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo  FLASHING SUCCESSFUL!
    echo ========================================
    echo.
    echo Board is ready for testing.
    echo Device will appear as: NCLite-ESP32
    echo.
) else (
    echo.
    echo ========================================
    echo  FLASHING FAILED!
    echo ========================================
    echo.
    echo Troubleshooting:
    echo   1. Press and hold BOOT button while flashing
    echo   2. Check USB cable connection
    echo   3. Verify COM port in Device Manager
    echo   4. Try lower baud rate: edit this file and change BAUD to 115200
    echo.
    pause
    exit /b 1
)

pause
