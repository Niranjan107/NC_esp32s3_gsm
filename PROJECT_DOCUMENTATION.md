# NCLite ESP32-S3 Connector - Project Documentation

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Architecture](#2-architecture)
3. [Hardware Configuration](#3-hardware-configuration)
4. [Components Overview](#4-components-overview)
5. [Detailed Component Documentation](#5-detailed-component-documentation)
6. [JSON Command Protocol](#6-json-command-protocol)
7. [Data Flow](#7-data-flow)
8. [Building and Flashing](#8-building-and-flashing)

---

## 1. Project Overview

### Purpose
The NCLite ESP32-S3 Connector is a dairy industry device that connects:
- **Weighing Machines (WM)** - For measuring milk weight
- **Milk Analyzers (MA)** - For analyzing milk quality (FAT, SNF, etc.)
- **Thermal Printers** - For printing receipts

All devices communicate via UART and data is transmitted to a mobile app via **BLE (Bluetooth Low Energy)**.

### Key Features
- BLE Serial Port Profile (SPP) using Nordic UART Service (NUS)
- JSON-based command protocol (compatible with Pico2W connector)
- Persistent configuration storage (NVS)
- Support for multiple device models
- Real-time data streaming and single-read modes
- ESC/POS thermal printer support
- Battery voltage monitoring via ADC (GPIO6)
- Over-The-Air (OTA) firmware updates via BLE

### Technology Stack
- **MCU**: ESP32-S3
- **Framework**: ESP-IDF v5.5
- **BLE Stack**: Bluedroid
- **Language**: C

---

## 2. Architecture

```
+------------------+     +------------------+     +------------------+
|  Mobile App      |     |  ESP32-S3        |     |  Peripherals     |
|  (Android/iOS)   |     |  Connector       |     |                  |
+------------------+     +------------------+     +------------------+
        |                        |                        |
        | BLE (NUS)              |                        |
        |<---------------------->|                        |
        |                        |    UART0               |
        |                        |<---------------------->| Weighing Machine
        |                        |                        |
        |                        |    UART2               |
        |                        |<---------------------->| Milk Analyzer
        |                        |                        |
        |                        |    UART1               |
        |                        |----------------------->| Thermal Printer
        |                        |                        |
+------------------+     +------------------+     +------------------+
```

### Component Interaction
```
                    +-------------+
                    |   main.c    |
                    | (Entry Point)|
                    +------+------+
                           |
        +------------------+------------------+
        |                  |                  |
+-------v-------+  +-------v-------+  +-------v-------+
|   ble_spp     |  |  cmd_parser   |  |    config     |
| (BLE Service) |  | (JSON Parser) |  | (NVS Storage) |
+-------+-------+  +-------+-------+  +---------------+
        |                  |
        |    +-------------+-------------+
        |    |             |             |
+-------v----v--+  +-------v-------+  +--v------------+
|    wm_uart    |  |   ma_uart     |  | printer_uart  |
| (Weight Data) |  | (Milk Data)   |  | (Print Recv)  |
+---------------+  +---------------+  +---------------+
```

---

## 3. Hardware Configuration

### Default GPIO Pin Assignments

| Function | UART | TX Pin | RX Pin | Default Baud |
|----------|------|--------|--------|--------------|
| WM (Weighing Machine) | UART0 | 43 | 44 | 9600 |
| MA (Milk Analyzer) | UART2 | 40 | 39 | 9600 |
| Printer | UART1 | 17 | 18 | 9600 |
| USB-CDC Console | - | - | - | - |
| Status LED | - | GPIO 36 | - | - |
| Battery ADC | - | GPIO 6 | - | - |

### BLE Configuration
- **Device Name**: `NitaraCLE4`
- **MTU Size**: 500 bytes
- **Service UUID**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART Service)

---

## 4. Components Overview

| Component | Location | Purpose |
|-----------|----------|---------|
| `main` | `main/main.c` | Application entry point, initialization |
| `ble_spp` | `components/ble_spp/` | BLE Serial Port Profile (Nordic UART Service) |
| `cmd_parser` | `components/cmd_parser/` | JSON command parsing and dispatch |
| `config` | `components/config/` | Device configuration and NVS storage |
| `wm_uart` | `components/wm_uart/` | Weighing Machine UART communication |
| `ma_uart` | `components/ma_uart/` | Milk Analyzer UART communication |
| `printer_uart` | `components/printer_uart/` | Thermal Printer UART (ESC/POS) |
| `battery` | `components/battery/` | Battery voltage monitoring via ADC |
| `ota` | `components/ota/` | Over-The-Air firmware updates via BLE |
| `common` | `components/common/` | Shared utilities and definitions |

---

## 5. Detailed Component Documentation

---

### 5.1 Main Application (`main/main.c`)

#### Purpose
Entry point for the application. Initializes all components in the correct order and creates FreeRTOS tasks.

#### Initialization Sequence
1. Wait 2 seconds for USB-CDC enumeration
2. Initialize NVS (Non-Volatile Storage)
3. Initialize USB console
4. Initialize command parser
5. Start LED task
6. Initialize WM UART (if enabled)
7. Initialize MA UART (if enabled)
8. Initialize Printer UART (if enabled)
9. Initialize Battery Monitor (if enabled)
10. Initialize OTA module (if enabled) and mark firmware valid
11. Initialize BLE SPP (if enabled)
12. Start console task

#### Functions

| Function | Description |
|----------|-------------|
| `app_main()` | Main entry point - initializes all components |
| `led_task()` | FreeRTOS task for status LED (heartbeat/activity indication) |
| `console_task()` | FreeRTOS task for USB console JSON command processing |
| `init_nvs()` | Initialize NVS flash storage |
| `init_usb_console()` | Initialize USB-CDC console |
| `on_activity()` | Callback for LED activity indication |
| `wm_ble_data_callback()` | Wrapper to send WM data to BLE |
| `ma_ble_data_callback()` | Wrapper to send MA data to BLE |

#### LED Behavior
- **Idle**: Slow heartbeat (100ms ON, 900ms OFF)
- **Activity**: Rapid blink 5 times (100ms ON/OFF)

---

### 5.2 BLE SPP Component (`components/ble_spp/`)

#### Purpose
Implements BLE Serial Port Profile using Nordic UART Service (NUS) for communication with mobile apps.

#### Files

| File | Description |
|------|-------------|
| `ble_spp.c` | Implementation of BLE GATT server |
| `include/ble_spp.h` | Public API and type definitions |

#### UUIDs (Nordic UART Service)
```
Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
RX Char:  6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (Write - phone to device)
TX Char:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (Notify - device to phone)
```

#### States
```c
typedef enum {
    BLE_STATE_IDLE = 0,         // Not initialized
    BLE_STATE_ADVERTISING,      // Waiting for connection
    BLE_STATE_CONNECTED,        // Connected to client
    BLE_STATE_DISCONNECTED      // Disconnected, will restart advertising
} ble_spp_state_t;
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `ble_spp_init()` | Initialize BLE stack and start advertising |
| `ble_spp_deinit()` | Deinitialize BLE stack |
| `ble_spp_send()` | Send data to connected client via notify |
| `ble_spp_register_rx_callback()` | Register callback for received data |
| `ble_spp_is_connected()` | Check if client is connected |
| `ble_spp_notify_enabled()` | Check if notifications are enabled |
| `ble_spp_get_state()` | Get current BLE state |
| `ble_spp_get_device_name()` | Get BLE device name |
| `ble_spp_output_callback()` | Callback wrapper for cmd_parser output |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `process_ble_rx_data()` | Process received JSON commands |
| `uuid128_cmp()` | Compare 128-bit UUIDs |
| `gap_event_handler()` | Handle GAP events (advertising, connection params) |
| `gatts_profile_event_handler()` | Handle GATT server events |
| `gatts_event_handler()` | Main GATT event dispatcher |

---

### 5.3 Command Parser Component (`components/cmd_parser/`)

#### Purpose
Parses JSON commands from USB console and BLE, dispatches to appropriate handlers.

#### Files

| File | Description |
|------|-------------|
| `cmd_parser.c` | JSON parsing and command handlers |
| `include/cmd_parser.h` | Command definitions and API |

#### Supported Commands

| Command | Description |
|---------|-------------|
| `reset_config` | Reset all configurations to defaults |
| `get_current_config` | Get current MA/WM/Printer configurations |
| `wm_port_config` | Configure Weighing Machine UART |
| `ma_port_config` | Configure Milk Analyzer UART |
| `printer_port_config` | Configure Printer UART |
| `print_receipt` | Print receipt with data |
| `check_printer_status` | Check if printer is ready |
| `reprint_last_receipt` | Reprint the last receipt |
| `ncle_get_unique_id` | Get device unique ID (MAC-based) |
| `get_firmware_version` | Get firmware version string |
| `self_diagnosis` | Run self-diagnosis and report status |
| `get_battery_status` | Get battery percentage (0-100) |
| `ota_begin` | Start OTA update with firmware size |
| `ota_write` | Write Base64 firmware chunk |
| `ota_end` | Finalize and verify OTA update |
| `ota_abort` | Abort OTA update in progress |
| `ota_status` | Get current OTA status |

#### Public Functions

| Function | Description |
|----------|-------------|
| `cmd_parser_init()` | Initialize command parser |
| `cmd_parser_register_output_callback()` | Register BLE output callback |
| `parse_and_process_commands()` | Parse JSON and execute command |
| `send_response()` | Send JSON response to USB and BLE |
| `send_status()` | Alias for send_response |

#### Internal Command Handlers

| Function | Description |
|----------|-------------|
| `process_wm_config()` | Handle wm_port_config command |
| `process_ma_config()` | Handle ma_port_config command |
| `process_printer_config()` | Handle printer_port_config command |
| `process_reset_config()` | Handle reset_config command |
| `process_get_current_config()` | Handle get_current_config command |
| `process_get_unique_id()` | Handle ncle_get_unique_id command |
| `process_get_firmware_version()` | Handle get_firmware_version command |
| `process_self_diagnosis()` | Handle self_diagnosis command |

---

### 5.4 Config Component (`components/config/`)

#### Purpose
Manages device configuration and persistent storage using ESP-IDF NVS (Non-Volatile Storage).

#### Files

| File | Description |
|------|-------------|
| `device_config.c` | NVS read/write implementation |
| `device_config.h` | Configuration structures and model definitions |

#### Configuration Structure
```c
typedef struct {
    uart_port_config_t ma_config;       // Milk Analyzer config
    uart_port_config_t wm_config;       // Weighing Machine config
    uart_port_config_t printer_config;  // Printer config
    char device_name[32];               // BLE device name
} device_config_t;
```

#### Device Model Definitions

**Milk Analyzer Models (1001-4999)**:
- 1001: Ekomilk Bond
- 1002: Ekomilk Ultra DPS
- 3001: Akashganga
- 4001: Prompt Fatomatic
- ...and more

**Weighing Machine Models (9000-9006)**:
- 9001: Type 1 (Kg)
- 9002: Type 2 (Lt)
- 9006: Dollar prefix format

**Printer Models**:
- 8000: Generic ESC/POS

#### Public Functions

| Function | Description |
|----------|-------------|
| `config_load_defaults()` | Load default configuration values |
| `config_load_from_nvs()` | Load configuration from NVS |
| `config_save_to_nvs()` | Save configuration to NVS |
| `config_reset_to_defaults()` | Reset and save defaults |
| `config_set_ma_port()` | Set and save MA configuration |
| `config_set_wm_port()` | Set and save WM configuration |
| `config_set_printer_port()` | Set and save Printer configuration |
| `config_get_json()` | Get configuration as JSON string |

#### Global Variable
```c
extern device_config_t g_device_config;  // Global configuration instance
```

---

### 5.5 WM UART Component (`components/wm_uart/`)

#### Purpose
Handles UART communication with weighing machines. Receives weight data, processes it, and outputs JSON.

#### Files

| File | Description |
|------|-------------|
| `wm_uart.c` | UART driver and data processing |
| `wm_uart.h` | Public API and configuration |

#### Features
- Newline and timeout-based packet detection
- Prefix stripping (ST, GS, US, NT, OL, W:, WT:, etc.)
- Non-printable character filtering
- Duplicate reading filtering (single-read mode)
- Configurable end character detection
- JSON output format

#### Operating Modes
```c
#define WM_MODE_SINGLE  0   // Wait for duplicate reading, output once, stop
#define WM_MODE_STREAM  1   // Continuous output every Nth sample
```

#### End Character Options
```c
typedef enum {
    WM_END_CHAR_LF_CR   = 0,    // '\n' OR '\r' (default)
    WM_END_CHAR_LF      = 1,    // '\n' only
    WM_END_CHAR_CR      = 2,    // '\r' only
    WM_END_CHAR_FF      = 3,    // Form feed
    WM_END_CHAR_NONE    = 4,    // Timeout only
    // ... more options
} wm_end_char_t;
```

#### JSON Output Format
```json
{"device":"wm","data":"0017.31Kg","model":9001 }
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `wm_uart_init()` | Initialize WM UART driver |
| `wm_uart_deinit()` | Deinitialize WM UART driver |
| `wm_uart_start()` | Start receive task |
| `wm_uart_stop()` | Stop receive task |
| `wm_uart_set_baud()` | Set baud rate at runtime |
| `wm_uart_get_config()` | Get current configuration |
| `wm_uart_save_config()` | Save configuration to NVS |
| `wm_uart_set_data_callback()` | Register data callback (for BLE) |
| `wm_uart_set_activity_callback()` | Register activity callback (for LED) |
| `wm_uart_set_stream_mode()` | Set single-read or stream mode |
| `wm_uart_set_sample_rate()` | Set sampling rate for stream mode |
| `wm_uart_set_model_id()` | Set model ID for JSON output |
| `wm_uart_set_end_char()` | Set packet end character |
| `wm_uart_is_connected()` | Check if WM is sending data |
| `wm_uart_print_status()` | Print status to console |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `strip_wm_data()` | Remove prefixes and trim whitespace |
| `is_printable_byte()` | Check if byte is printable ASCII |
| `get_end_char_byte()` | Get byte value for end character |
| `is_end_char()` | Check if byte is configured end character |
| `get_uart_parity()` | Convert parity enum to UART setting |
| `output_wm_data()` | Output JSON and call callbacks |
| `process_wm_packet()` | Process complete packet |
| `uart_rx_task()` | FreeRTOS task for UART receive |

---

### 5.6 MA UART Component (`components/ma_uart/`)

#### Purpose
Handles UART communication with milk analyzers. Receives milk analysis data and outputs JSON.

#### Files

| File | Description |
|------|-------------|
| `ma_uart.c` | UART driver and data processing |
| `ma_uart.h` | Public API and configuration |

#### Features
- Model-based detection modes:
  - **Timeout mode** (default): Wait for silence after data
  - **Parentheses mode** (model 3000-3999): Capture between '(' and ')'
  - **Newline mode** (model 4000-4999): End on '\n'
- Immediate output (no duplicate detection - receipts are always unique)
- Automatic WM trigger after MA data (dairy workflow)
- JSON output with preserved newlines

#### Detection Modes
```c
typedef enum {
    MA_DETECT_TIMEOUT = 0,      // Default (timeout-based)
    MA_DETECT_PARENTHESES = 1,  // Model 3000-3999
    MA_DETECT_NEWLINE = 2       // Model 4000-4999
} ma_detect_mode_t;
```

#### JSON Output Format
```json
{"device":"ma","data":"Provisional Acknowldgement Slip\n\nDIARY CRAFT PVT LTD\n...","model":1002 }
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `ma_uart_init()` | Initialize MA UART driver |
| `ma_uart_deinit()` | Deinitialize MA UART driver |
| `ma_uart_start()` | Start receive task |
| `ma_uart_stop()` | Stop receive task |
| `ma_uart_set_baud()` | Set baud rate at runtime |
| `ma_uart_get_config()` | Get current configuration |
| `ma_uart_save_config()` | Save configuration to NVS |
| `ma_uart_set_data_callback()` | Register data callback (for BLE) |
| `ma_uart_set_activity_callback()` | Register activity callback (for LED) |
| `ma_uart_set_stream_mode()` | Set single-read or stream mode |
| `ma_uart_set_model_id()` | Set model ID (determines detection mode) |
| `ma_uart_get_detect_mode()` | Get detection mode for model ID |
| `ma_uart_is_connected()` | Check if MA is sending data |
| `ma_uart_print_status()` | Print status to console |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `get_uart_parity()` | Convert parity enum to UART setting |
| `update_detect_mode()` | Update detection mode based on model |
| `remove_extra_spaces()` | Clean up extra spaces in data |
| `output_ma_data()` | Output JSON and call callbacks |
| `process_ma_packet()` | Process complete packet, trigger WM |
| `uart_rx_task()` | FreeRTOS task for UART receive |

---

### 5.7 Printer UART Component (`components/printer_uart/`)

#### Purpose
Handles UART communication with ESC/POS thermal printers. Supports text formatting, special characters, and receipt storage.

#### Files

| File | Description |
|------|-------------|
| `printer_uart.c` | UART driver and ESC/POS commands |
| `printer_uart.h` | Public API and command definitions |

#### Features
- Full ESC/POS command support
- Special character parsing (pipe escape sequences)
- Receipt storage for reprint functionality
- Configurable baud rate and parity
- Thread-safe with mutex

#### Special Character Sequences (Pipe Escapes)
| Sequence | Description |
|----------|-------------|
| `\|s` | Start/reset printer |
| `\|n` | New line |
| `\|c` | Cut paper |
| `\|f` | Feed paper (4 lines) |
| `\|b` | Bold ON |
| `\|B` | Bold OFF |
| `\|C` | Center align |
| `\|l` | Left align |
| `\|r` | Right align |
| `\|d` | Double height ON |
| `\|D` | Double height OFF |
| `\|u` | Underline ON |
| `\|U` | Underline OFF |
| `\|i` | Inverse ON |
| `\|I` | Inverse OFF |
| `\|t` | Tab |
| `\|\|` | Literal pipe character |

#### Public Functions - Initialization

| Function | Description |
|----------|-------------|
| `printer_uart_init()` | Initialize printer UART |
| `printer_uart_deinit()` | Deinitialize printer UART |
| `printer_uart_is_initialized()` | Check if initialized |

#### Public Functions - Configuration

| Function | Description |
|----------|-------------|
| `printer_uart_set_baud()` | Set baud rate |
| `printer_uart_set_parity()` | Set parity |
| `printer_uart_get_config()` | Get configuration |
| `printer_uart_save_config()` | Save to NVS |

#### Public Functions - ESC/POS Commands

| Function | ESC/POS Command | Description |
|----------|-----------------|-------------|
| `printer_start()` | ESC @ | Initialize printer |
| `printer_new_line()` | LF | Print new line |
| `printer_cut_paper()` | GS V | Cut paper |
| `printer_feed_paper()` | ESC d n | Feed n lines |
| `printer_set_bold()` | ESC E 1 | Bold ON |
| `printer_end_bold()` | ESC E 0 | Bold OFF |
| `printer_set_center_align()` | ESC a 1 | Center align |
| `printer_set_left_align()` | ESC a 0 | Left align |
| `printer_set_right_align()` | ESC a 2 | Right align |
| `printer_set_double_height()` | GS ! 16 | Double height |
| `printer_set_normal_height()` | GS ! 0 | Normal height |
| `printer_set_underline()` | ESC - 1 | Underline ON |
| `printer_end_underline()` | ESC - 0 | Underline OFF |
| `printer_set_inverse()` | GS B 1 | Inverse ON |
| `printer_end_inverse()` | GS B 0 | Inverse OFF |

#### Public Functions - High-Level Printing

| Function | Description |
|----------|-------------|
| `printer_print_with_special_chars()` | Print with pipe escape parsing |
| `printer_print_raw()` | Print raw text |
| `printer_print_char()` | Print single character |
| `printer_reprint_last_receipt()` | Reprint stored receipt |
| `printer_store_receipt()` | Store receipt data |
| `printer_clear_receipt()` | Clear stored receipt |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `get_uart_parity()` | Convert parity enum |
| `send_bytes()` | Send raw bytes to printer |
| `send_byte()` | Send single byte |
| `send_command()` | Send ESC/POS command (1-3 bytes) |
| `send_string()` | Send string to printer |

---

### 5.8 Common Component (`components/common/`)

#### Purpose
Shared definitions, types, and utility functions used across all components.

#### Files

| File | Description |
|------|-------------|
| `common.c` | Utility function implementations |
| `common.h` | Shared types and definitions |

#### Constants
```c
#define NCLE_FIRMWARE_VERSION "2.0.0.1000"
#define WM_UART_BUF_SIZE      256
#define MA_UART_BUF_SIZE      1024
#define PRINTER_UART_BUF_SIZE 2048
#define CMD_BUF_SIZE          512
#define RESPONSE_BUF_SIZE     1024
```

#### Types
```c
typedef enum {
    DEVICE_TYPE_MA = 0,     // Milk Analyzer
    DEVICE_TYPE_WM,         // Weighing Machine
    DEVICE_TYPE_PRINTER,    // Printer
    DEVICE_TYPE_MAX
} device_type_t;

typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN = 1,
    PARITY_ODD = 2
} parity_t;

typedef struct {
    uint32_t baud_rate;
    data_bits_t data_bits;
    stop_bits_t stop_bits;
    parity_t parity;
    bool stream_mode;
    int model;
} uart_port_config_t;
```

#### Status Codes
```c
#define STATUS_OK               0
#define STATUS_ERR              -1
#define STATUS_COMM_TIMEOUT     -2
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `common_init()` | Initialize common module |
| `get_device_type_string()` | Get string for device type enum |
| `get_unique_id()` | Get unique device ID (MAC-based) |

---

### 5.9 Battery Component (`components/battery/`)

#### Purpose
Monitors battery voltage via ESP32-S3 ADC on GPIO6. Provides voltage readings, percentage estimates, and JSON output for the mobile app to display battery status.

#### Files

| File | Description |
|------|-------------|
| `battery.c` | ADC driver and voltage calculation |
| `battery.h` | Public API and configuration constants |

#### Hardware Setup
```
Battery +  ────┬──── R1 (100K) ────┬──── GPIO6 (ADC1_CH5)
               │                    │
               │              R2 (100K)
               │                    │
Battery -  ────┴────────────────────┴──── GND

Vout = Vbat × R2/(R1+R2) = Vbat × 0.5
Example: 4.2V battery → 2.1V at GPIO6
```

#### ADC Characteristics
- **ADC Unit**: ADC1_CHANNEL_5 (GPIO6)
- **Resolution**: 12-bit (0-4095)
- **Attenuation**: ADC_ATTEN_DB_12 (0-3.3V range)
- **Samples**: 64 samples averaged for noise reduction
- **Calibration**: Uses ESP-IDF curve fitting/line fitting from eFuse

#### Configuration (via menuconfig)
```
NCLite CLEV4 → Battery Monitor
├── Enable Battery Monitor (NCLE_BATTERY_ENABLE)
├── Battery ADC GPIO Pin (NCLE_BATTERY_ADC_GPIO) - default: 6
├── Voltage Divider Ratio x100 (NCLE_BATTERY_DIVIDER_RATIO) - default: 200
├── Battery Full Voltage mV (NCLE_BATTERY_FULL_MV) - default: 4200
└── Battery Empty Voltage mV (NCLE_BATTERY_EMPTY_MV) - default: 3000
```

#### Battery Status Levels
| Status | Percentage Range |
|--------|------------------|
| `full` | 81-100% |
| `good` | 51-80% |
| `low` | 21-50% |
| `critical` | 0-20% |

#### Percentage Calculation
```c
// Linear interpolation between empty and full voltage
percentage = (voltage - EMPTY_MV) × 100 / (FULL_MV - EMPTY_MV)

// Example with defaults (3000-4200mV range):
// 4200mV → 100%
// 3850mV → 70%
// 3600mV → 50%
// 3000mV → 0%
```

#### JSON Command/Response

**Request:**
```json
{"command":"get_battery_status"}#
```

**Response:**
```json
{"response_message":"battery","status_code":0,"data":{"pct":70}}
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `battery_init()` | Initialize ADC for GPIO6, load calibration |
| `battery_deinit()` | Release ADC resources |
| `battery_get_voltage_mv()` | Read battery voltage in millivolts |
| `battery_get_percentage()` | Get estimated percentage (0-100) |
| `battery_get_status()` | Get status enum (full/good/low/critical) |
| `battery_get_info()` | Get all battery metrics in one call |
| `battery_get_json()` | Format battery info as JSON string |
| `battery_is_low()` | Quick check for low battery condition |
| `battery_set_divider_ratio()` | Override voltage divider ratio at runtime |
| `battery_get_raw_adc()` | Get raw ADC value for debugging |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `init_adc_calibration()` | Initialize ADC calibration scheme |
| `deinit_adc_calibration()` | Free calibration resources |
| `read_adc_averaged()` | Read N samples and average |
| `raw_to_battery_voltage_mv()` | Convert raw ADC to battery voltage |
| `voltage_to_percentage()` | Map voltage to 0-100% |
| `percentage_to_status()` | Map percentage to status enum |
| `status_to_string()` | Convert status enum to string |

---

### 5.10 OTA Component (`components/ota/`)

#### Purpose
Implements BLE-based Over-The-Air (OTA) firmware updates using ESP-IDF's OTA APIs. Receives firmware in chunks from mobile app, writes to OTA partition, verifies, and reboots to new firmware.

#### Files

| File | Description |
|------|-------------|
| `ota.c` | OTA implementation using ESP-IDF OTA APIs |
| `ota.h` | Public API, state enums, error codes |

#### ESP-IDF OTA Flow
```
1. esp_ota_begin()           - Erase and prepare OTA partition
2. esp_ota_write()           - Write firmware data (called multiple times)
3. esp_ota_end()             - Validate written data
4. esp_ota_set_boot_partition() - Set next boot partition
5. esp_restart()             - Reboot to new firmware
```

#### Partition Layout (partitions.csv)
```
nvs,      data, nvs,      0x9000,   0x4000
otadata,  data, ota,      0xd000,   0x2000
phy_init, data, phy,      0xf000,   0x1000
ota_0,    app,  ota_0,    0x10000,  0x180000  (1.5 MB)
ota_1,    app,  ota_1,    0x190000, 0x180000  (1.5 MB)
nvs_key,  data, nvs_keys, 0x310000, 0x1000
```

#### OTA States
```c
typedef enum {
    OTA_STATE_IDLE = 0,      // Not started
    OTA_STATE_READY,         // ota_begin called, ready for data
    OTA_STATE_RECEIVING,     // Receiving firmware chunks
    OTA_STATE_VERIFYING,     // Verifying firmware
    OTA_STATE_COMPLETE,      // OTA successful, ready to reboot
    OTA_STATE_ERROR          // Error occurred
} ota_state_t;
```

#### Error Codes
```c
typedef enum {
    OTA_ERR_NONE = 0,
    OTA_ERR_NOT_INITIALIZED,
    OTA_ERR_ALREADY_STARTED,
    OTA_ERR_PARTITION_NOT_FOUND,
    OTA_ERR_PARTITION_TOO_SMALL,
    OTA_ERR_WRITE_FAILED,
    OTA_ERR_INVALID_CHUNK,
    OTA_ERR_SEQUENCE_ERROR,
    OTA_ERR_SIZE_MISMATCH,
    OTA_ERR_VERIFY_FAILED,
    OTA_ERR_SET_BOOT_FAILED,
    OTA_ERR_TIMEOUT,
    OTA_ERR_ABORTED
} ota_error_t;
```

#### Configuration (via menuconfig)
```
NCLite CLEV4 → OTA Update
├── Enable OTA Updates (NCLE_OTA_ENABLE) - default: y
└── Enable Automatic Rollback (NCLE_OTA_ROLLBACK_ENABLE) - default: y
```

#### Rollback Protection
ESP-IDF supports automatic rollback if new firmware fails to boot. Call `ota_mark_valid()` after successful startup to confirm the firmware is working and disable rollback.

#### JSON Commands

**ota_begin** - Start OTA update
```json
{"command":"ota_begin","size":524288}#
```
Response:
```json
{"response_message":"ota_begin","status_code":0,"data":{"state":"ready"}}
```

**ota_write** - Write firmware chunk (Base64 encoded)
```json
{"command":"ota_write","seq":0,"data":"<base64_data>"}#
```
Response:
```json
{"response_message":"ota_write","status_code":0,"data":{"seq":0,"pct":10}}
```

**ota_end** - Finalize and verify OTA
```json
{"command":"ota_end"}#
```
Response:
```json
{"response_message":"ota_end","status_code":0,"data":{"state":"complete"}}
```

**ota_abort** - Abort OTA update
```json
{"command":"ota_abort"}#
```
Response:
```json
{"response_message":"ota_abort","status_code":0}
```

**ota_status** - Get OTA status
```json
{"command":"ota_status"}#
```
Response:
```json
{"response_message":"ota_status","status_code":0,"data":{"state":"idle","pct":0}}
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `ota_init()` | Initialize OTA module |
| `ota_begin()` | Start OTA (erases target partition) |
| `ota_write_chunk()` | Write firmware chunk with sequence number |
| `ota_end()` | Finalize, verify, and set boot partition |
| `ota_abort()` | Abort OTA in progress |
| `ota_reboot()` | Reboot to new firmware |
| `ota_get_progress()` | Get current OTA progress |
| `ota_get_state()` | Get current OTA state |
| `ota_is_in_progress()` | Check if OTA is in progress |
| `ota_error_to_string()` | Convert error code to string |
| `ota_mark_valid()` | Mark current firmware as valid (disable rollback) |
| `ota_get_running_partition_info()` | Get running partition details |

#### Internal Functions

| Function | Description |
|----------|-------------|
| `reset_ota_state()` | Reset all OTA state variables |
| `set_error()` | Set error state and log |

#### OTA Data Flow
```
Mobile App                    ESP32-S3
----------                    --------
    |                             |
    | {"command":"ota_begin","size":524288}#
    |----------------------------->|
    |                             | esp_ota_begin()
    |<-----------------------------|
    | {"response":"ota_begin","status":0}
    |                             |
    | {"command":"ota_write","seq":0,"data":"<base64>"}#
    |----------------------------->|
    |                             | decode + esp_ota_write()
    |<-----------------------------|
    | {"response":"ota_write","status":0,"pct":10}
    |                             |
    |     ... (repeat for all chunks) ...
    |                             |
    | {"command":"ota_end"}#
    |----------------------------->|
    |                             | esp_ota_end()
    |                             | esp_ota_set_boot_partition()
    |<-----------------------------|
    | {"response":"ota_end","status":0}
    |                             |
    |                             | esp_restart()
```

#### Python Test Tool
A Python script is provided at `tools/ota_upload.py` for testing OTA updates via BLE:
```bash
# Install dependency
pip install bleak

# Upload firmware
python tools/ota_upload.py NitaraCLE4 build/nitara_connector.bin
```

---

## 6. JSON Command Protocol

### Command Format
```json
{"command":"<command_name>","param1":"value1",...}#
```
Commands are terminated with `#` or newline.

### Response Format
```json
{"response_message":"<message>","status_code":<code>,"data":<optional>}
```

### Example Commands

#### Configure Weighing Machine
```json
{"command":"wm_port_config","model":9001,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":1}#
```

#### Configure Milk Analyzer
```json
{"command":"ma_port_config","model":1002,"baud_rate":1200,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}#
```

#### Configure Printer
```json
{"command":"printer_port_config","model":8000,"baud_rate":9600,"data_bits":8,"stop_bits":0,"parity":0,"stream":0}#
```

#### Print Receipt
```json
{"command":"print_receipt","data":"......... NITARA Dairy ..........|n Milk Collection Receipt |n"}#
```

#### Get Current Config
```json
{"command":"get_current_config"}#
```

#### Get Firmware Version
```json
{"command":"get_firmware_version"}#
```

#### Get Unique ID
```json
{"command":"ncle_get_unique_id"}#
```

#### Get Battery Status
```json
{"command":"get_battery_status"}#
```
**Response:**
```json
{"response_message":"battery","status_code":0,"data":{"pct":70}}
```

---

## 7. Data Flow

### WM Data Flow (Single-Read Mode)
```
WM Device -> UART0 RX -> wm_uart_rx_task -> process_wm_packet
                                                    |
                                        Wait for duplicate
                                                    |
                                        output_wm_data()
                                                    |
                            +----------------------+----------------------+
                            |                                             |
                    puts() (Console)                           s_data_callback()
                                                                          |
                                                              wm_ble_data_callback()
                                                                          |
                                                              ble_spp_output_callback()
                                                                          |
                                                              BLE Notify -> Mobile App
```

### MA -> WM Dairy Workflow
```
MA Device -> UART2 RX -> ma_uart_rx_task -> process_ma_packet
                                                    |
                                        output_ma_data() -> BLE
                                                    |
                                        wm_uart_start() (Trigger WM)
                                                    |
                                        WM starts receiving weight data
```

### Print Receipt Flow
```
Mobile App -> BLE Write -> ble_spp RX -> process_ble_rx_data
                                                    |
                                        parse_and_process_commands()
                                                    |
                                        CMD_PRINT_RECEIPT handler
                                                    |
                                        printer_print_with_special_chars()
                                                    |
                                        UART1 TX -> Thermal Printer
```

---

## 8. Building and Flashing

### Prerequisites
- ESP-IDF v5.5 or later
- Python 3.8+

### Build
```bash
idf.py build
```

### Flash
```bash
idf.py -p COMx flash
```

### Monitor
```bash
idf.py -p COMx monitor
```

### Configuration (menuconfig)
```bash
idf.py menuconfig
```
Navigate to `NCLite Configuration` to modify:
- UART pin assignments
- Buffer sizes
- Feature enables/disables
- BLE settings

---

## Appendix: File Structure

```
nitara.connector.esp32s3/
├── main/
│   ├── main.c                    # Application entry point
│   ├── CMakeLists.txt
│   └── Kconfig.projbuild         # menuconfig options
├── components/
│   ├── ble_spp/
│   │   ├── ble_spp.c             # BLE implementation
│   │   ├── include/
│   │   │   └── ble_spp.h         # BLE API
│   │   └── CMakeLists.txt
│   ├── cmd_parser/
│   │   ├── cmd_parser.c          # JSON parser
│   │   ├── include/
│   │   │   └── cmd_parser.h      # Parser API
│   │   └── CMakeLists.txt
│   ├── config/
│   │   ├── device_config.c       # NVS storage
│   │   ├── device_config.h       # Config structures
│   │   └── CMakeLists.txt
│   ├── wm_uart/
│   │   ├── wm_uart.c             # WM UART driver
│   │   ├── wm_uart.h             # WM API
│   │   └── CMakeLists.txt
│   ├── ma_uart/
│   │   ├── ma_uart.c             # MA UART driver
│   │   ├── ma_uart.h             # MA API
│   │   └── CMakeLists.txt
│   ├── printer_uart/
│   │   ├── printer_uart.c        # Printer driver
│   │   ├── printer_uart.h        # Printer API
│   │   └── CMakeLists.txt
│   ├── battery/
│   │   ├── battery.c             # ADC battery monitor
│   │   ├── battery.h             # Battery API
│   │   └── CMakeLists.txt
│   ├── ota/
│   │   ├── ota.c                 # OTA implementation
│   │   ├── ota.h                 # OTA API
│   │   └── CMakeLists.txt
│   └── common/
│       ├── common.c              # Utilities
│       ├── common.h              # Shared definitions
│       └── CMakeLists.txt
├── tools/
│   └── ota_upload.py             # Python BLE OTA upload script
├── partitions.csv                # OTA partition table
├── sdkconfig.defaults            # Default SDK configuration
├── CMakeLists.txt                # Main build file
└── PROJECT_DOCUMENTATION.md      # This file
```

---

**Document Version**: 1.0
**Last Updated**: January 2026
**Firmware Version**: 2.0.0.1000
