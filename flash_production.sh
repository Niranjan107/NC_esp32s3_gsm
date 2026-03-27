#!/bin/bash
# ============================================================================
# NCLite ESP32-S3 Production Firmware Flasher (Linux/Mac)
# ============================================================================
# Purpose: Flash production firmware to ESP32-S3 boards in factory/production
# Usage:   ./flash_production.sh /dev/ttyUSB0
# Example: ./flash_production.sh /dev/ttyUSB0
# ============================================================================

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo "========================================"
echo " NCLite ESP32-S3 Production Flasher"
echo "========================================"
echo ""

# Check if port argument is provided
if [ -z "$1" ]; then
    echo -e "${RED}ERROR: Serial port not specified!${NC}"
    echo ""
    echo "Usage: ./flash_production.sh SERIAL_PORT"
    echo "Example: ./flash_production.sh /dev/ttyUSB0"
    echo ""
    echo "Available serial ports:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"
    echo ""
    exit 1
fi

PORT="$1"
BAUD=921600
CHIP=esp32s3
FLASH_MODE=dio
FLASH_FREQ=80m
FLASH_SIZE=8MB

echo "Configuration:"
echo "  - Target Chip: $CHIP"
echo "  - Serial Port: $PORT"
echo "  - Baud Rate: $BAUD"
echo "  - Flash Mode: $FLASH_MODE"
echo "  - Flash Freq: $FLASH_FREQ"
echo "  - Flash Size: $FLASH_SIZE"
echo ""

# Check if esptool is installed
if ! command -v esptool.py &> /dev/null; then
    echo -e "${RED}ERROR: esptool.py not found!${NC}"
    echo ""
    echo "Please install esptool:"
    echo "  pip install esptool"
    echo ""
    exit 1
fi

# Check if firmware files exist
if [ ! -f "build/bootloader/bootloader.bin" ]; then
    echo -e "${RED}ERROR: bootloader.bin not found!${NC}"
    echo "Please build the firmware first: idf.py build"
    echo ""
    exit 1
fi

if [ ! -f "build/partition_table/partition-table.bin" ]; then
    echo -e "${RED}ERROR: partition-table.bin not found!${NC}"
    echo "Please build the firmware first: idf.py build"
    echo ""
    exit 1
fi

if [ ! -f "build/nitara.connector.esp32s3.bin" ]; then
    echo -e "${RED}ERROR: nitara.connector.esp32s3.bin not found!${NC}"
    echo "Please build the firmware first: idf.py build"
    echo ""
    exit 1
fi

echo "========================================"
echo " Starting Flash Process..."
echo "========================================"
echo ""
echo -e "${YELLOW}NOTE: If flashing fails, press and hold BOOT button on the board${NC}"
echo ""

# Flash the firmware
esptool.py --chip $CHIP --port $PORT --baud $BAUD \
  --before default_reset --after hard_reset write_flash \
  -z --flash_mode $FLASH_MODE --flash_freq $FLASH_FREQ --flash_size $FLASH_SIZE \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/nitara.connector.esp32s3.bin

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN} FLASHING SUCCESSFUL!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Board is ready for testing."
    echo "Device will appear as: NCLite-ESP32"
    echo ""
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED} FLASHING FAILED!${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Press and hold BOOT button while flashing"
    echo "  2. Check USB cable connection"
    echo "  3. Verify serial port permissions: sudo usermod -a -G dialout $USER"
    echo "  4. Try lower baud rate: edit this file and change BAUD to 115200"
    echo ""
    exit 1
fi
