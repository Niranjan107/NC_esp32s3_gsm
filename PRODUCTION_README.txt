# Production Firmware Package - README

## Quick Start Guide for Production Flashing

This package contains everything needed to flash NCLite ESP32-S3 boards in production.

---

## What's Included

- `build/` - Compiled firmware binaries
- `flash_production.bat` - Windows flashing script
- `flash_production.sh` - Linux/Mac flashing script
- `README.txt` - This file

---

## Prerequisites

1. **Install Python 3.x** (if not already installed)
   - Download from: https://www.python.org/downloads/

2. **Install esptool**
   ```
   pip install esptool
   ```

3. **Install USB Drivers** (Windows only, if needed)
   - CP210x: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   - CH340: http://www.wch-ic.com/downloads/CH341SER_EXE.html

---

## Flashing Instructions

### Windows

1. Connect ESP32-S3 board via USB
2. Check COM port in Device Manager (e.g., COM3)
3. Open Command Prompt in this directory
4. Run:
   ```
   flash_production.bat COM3
   ```
   (Replace COM3 with your actual port)

### Linux/Mac

1. Connect ESP32-S3 board via USB
2. Check serial port:
   ```
   ls /dev/ttyUSB*
   ```
3. Make script executable:
   ```
   chmod +x flash_production.sh
   ```
4. Run:
   ```
   ./flash_production.sh /dev/ttyUSB0
   ```
   (Replace /dev/ttyUSB0 with your actual port)

---

## Troubleshooting

### "Failed to connect"
- Press and hold BOOT button while running flash command
- Check USB cable (must be data cable, not charge-only)
- Verify COM port in Device Manager

### "Permission denied" (Linux/Mac)
```
sudo usermod -a -G dialout $USER
```
Then log out and log back in

### "esptool not found"
```
pip install esptool
```

---

## Verification

After flashing:
1. Open serial monitor at 115200 baud
2. You should see boot messages
3. Device will advertise as "NCLite-ESP32" via Bluetooth

---

## Support

For issues, contact: [Your support email/contact]

Firmware Version: [Version number]
Build Date: [Build date]
