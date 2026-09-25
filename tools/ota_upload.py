#!/usr/bin/env python3
"""
BLE OTA Firmware Upload Tool for NCLite ESP32-S3

This script uploads firmware to the NCLite device via BLE OTA.

Requirements:
    pip install bleak

Usage:
    python ota_upload.py [device_name] [firmware_file]

Examples:
    python ota_upload.py NitaraCLE5G firmware.bin
    python ota_upload.py                           # Uses defaults

Protocol:
    1. Connect to device via BLE
    2. Send ota_begin with firmware size
    3. Send firmware in Base64 encoded chunks via ota_write
    4. Send ota_end to verify and reboot
"""

import asyncio
import sys
import os
import base64
import json
import time
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Error: bleak library not installed")
    print("Install with: pip install bleak")
    sys.exit(1)

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to device
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify from device

# OTA Configuration
CHUNK_SIZE = 300  # Binary bytes per chunk (will be ~400 bytes Base64)
RESPONSE_TIMEOUT = 10.0  # Seconds to wait for response

class BleOtaUploader:
    def __init__(self, device_name: str = "NitaraCLE5G"):
        self.device_name = device_name
        self.client: Optional[BleakClient] = None
        self.response_event = asyncio.Event()
        self.last_response = None

    async def notification_handler(self, sender, data: bytearray):
        """Handle notifications from device"""
        try:
            response = data.decode('utf-8')
            print(f"  <- {response[:100]}{'...' if len(response) > 100 else ''}")
            self.last_response = response
            self.response_event.set()
        except Exception as e:
            print(f"  <- [Error decoding: {e}]")

    async def find_device(self) -> Optional[str]:
        """Scan for device and return address"""
        print(f"Scanning for {self.device_name}...")
        devices = await BleakScanner.discover(timeout=10.0)

        for device in devices:
            if device.name and self.device_name in device.name:
                print(f"Found: {device.name} ({device.address})")
                return device.address

        print(f"Device '{self.device_name}' not found")
        return None

    async def connect(self) -> bool:
        """Connect to device"""
        address = await self.find_device()
        if not address:
            return False

        print(f"Connecting to {address}...")
        self.client = BleakClient(address)

        try:
            await self.client.connect()
            print("Connected!")

            # Enable notifications
            await self.client.start_notify(NUS_TX_UUID, self.notification_handler)
            print("Notifications enabled")
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("Disconnected")

    async def send_command(self, cmd: dict) -> Optional[dict]:
        """Send JSON command and wait for response"""
        self.response_event.clear()
        self.last_response = None

        # Format command
        cmd_str = json.dumps(cmd) + "#"
        cmd_bytes = cmd_str.encode('utf-8')

        # Send command (may need to split for large commands)
        chunk_size = 500  # BLE MTU
        for i in range(0, len(cmd_bytes), chunk_size):
            chunk = cmd_bytes[i:i+chunk_size]
            await self.client.write_gatt_char(NUS_RX_UUID, chunk)

        # Wait for response
        try:
            await asyncio.wait_for(self.response_event.wait(), RESPONSE_TIMEOUT)

            if self.last_response:
                # Parse response JSON
                try:
                    return json.loads(self.last_response)
                except:
                    return {"raw": self.last_response}
        except asyncio.TimeoutError:
            print("  [Timeout waiting for response]")
            return None

    async def upload_firmware(self, firmware_path: str) -> bool:
        """Upload firmware file via OTA"""

        # Read firmware file
        if not os.path.exists(firmware_path):
            print(f"Error: File not found: {firmware_path}")
            return False

        with open(firmware_path, 'rb') as f:
            firmware = f.read()

        fw_size = len(firmware)
        print(f"Firmware: {firmware_path}")
        print(f"Size: {fw_size} bytes ({fw_size/1024:.1f} KB)")

        # Step 1: Begin OTA
        print("\n[1/3] Starting OTA...")
        resp = await self.send_command({"command": "ota_begin", "size": fw_size})

        if not resp or resp.get("status_code") != 0:
            print(f"Error: ota_begin failed: {resp}")
            return False

        print("  OTA ready")

        # Step 2: Send firmware chunks
        print("\n[2/3] Uploading firmware...")
        total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE

        start_time = time.time()

        for seq in range(total_chunks):
            offset = seq * CHUNK_SIZE
            chunk = firmware[offset:offset + CHUNK_SIZE]

            # Base64 encode chunk
            b64_chunk = base64.b64encode(chunk).decode('utf-8')

            # Send chunk
            print(f"  -> ota_write seq={seq}/{total_chunks-1} ({len(chunk)} bytes)", end='')

            resp = await self.send_command({
                "command": "ota_write",
                "seq": seq,
                "data": b64_chunk
            })

            if not resp or resp.get("status_code") != 0:
                print(f"\nError: ota_write failed at seq {seq}: {resp}")
                await self.send_command({"command": "ota_abort"})
                return False

            # Show progress
            progress = resp.get("data", {}).get("pct", 0) if isinstance(resp.get("data"), dict) else 0
            print(f" [{progress}%]")

        elapsed = time.time() - start_time
        print(f"\n  Upload complete in {elapsed:.1f}s ({fw_size/elapsed/1024:.1f} KB/s)")

        # Step 3: End OTA
        print("\n[3/3] Verifying and rebooting...")
        resp = await self.send_command({"command": "ota_end"})

        if not resp or resp.get("status_code") != 0:
            print(f"Error: ota_end failed: {resp}")
            return False

        print("  Firmware verified!")
        print("  Device is rebooting to new firmware...")

        return True


async def main():
    # Parse arguments
    device_name = "NitaraCLE5G"
    firmware_path = None

    if len(sys.argv) >= 2:
        device_name = sys.argv[1]
    if len(sys.argv) >= 3:
        firmware_path = sys.argv[2]
    else:
        # Look for firmware in common locations
        search_paths = [
            "build/nitara_connector.bin",
            "../build/nitara_connector.bin",
            "firmware.bin"
        ]
        for path in search_paths:
            if os.path.exists(path):
                firmware_path = path
                break

    if not firmware_path:
        print("Usage: python ota_upload.py [device_name] <firmware.bin>")
        print("\nExamples:")
        print("  python ota_upload.py NitaraCLE5G build/nitara_connector.bin")
        print("  python ota_upload.py NitaraCLE5G firmware.bin")
        sys.exit(1)

    print("=" * 50)
    print("NCLite BLE OTA Upload Tool")
    print("=" * 50)

    uploader = BleOtaUploader(device_name)

    try:
        # Connect
        if not await uploader.connect():
            sys.exit(1)

        # Upload
        success = await uploader.upload_firmware(firmware_path)

        if success:
            print("\n" + "=" * 50)
            print("OTA UPDATE SUCCESSFUL!")
            print("=" * 50)
        else:
            print("\n" + "=" * 50)
            print("OTA UPDATE FAILED")
            print("=" * 50)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nAborted by user")
        await uploader.send_command({"command": "ota_abort"})

    finally:
        await uploader.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
