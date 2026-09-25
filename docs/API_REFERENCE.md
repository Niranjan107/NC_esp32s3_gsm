# NCLite ESP32-S3 Connector - API Reference

> **OUT OF DATE.** This file documents 8 of the 32 commands the firmware
> accepts, and predates GSM, MQTT, FOTA, the reading buffer and link modes -
> it still describes MA, the printer and BLE as "planned". For the current
> command set see **[commands.md](commands.md)**, and for connectivity see
> **[link-modes.md](link-modes.md)**.
>
> Kept for the component-architecture notes below, which are still broadly
> right apart from the flat layout (components now live under base/,
> connectivity/ and application/).

## Project Overview

NCLite ESP32-S3 is a connector device that interfaces with:
- **Weighing Machine (WM)** - via UART
- **Milk Analyzer (MA)** - via UART (planned)
- **Printer** - via PIO (planned)
- **Mobile App** - via BLE (planned)

Commands are received as JSON from USB or BLE and responses are sent back in JSON format.

---

## Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        main.c                                │
│  - App initialization                                        │
│  - Console task (USB menu + JSON mode)                       │
│  - LED task                                                  │
└─────────────────────────────────────────────────────────────┘
         │              │              │              │
         ▼              ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│  wm_uart    │ │ cmd_parser  │ │   config    │ │   common    │
│  component  │ │  component  │ │  component  │ │  component  │
└─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘
```

---

## Component: wm_uart

**Location:** `components/wm_uart/`

**Purpose:** Handle UART communication with Weighing Machine

### Configuration (wm_uart.h)

| Define | Default | Description |
|--------|---------|-------------|
| `WM_UART_NUM` | 0 | UART port number |
| `WM_UART_TX_PIN` | 43 | TX GPIO pin |
| `WM_UART_RX_PIN` | 44 | RX GPIO pin |
| `WM_UART_BAUD_RATE` | 9600 | Default baud rate |
| `WM_RX_TIMEOUT_MS` | 500 | Packet timeout |
| `WM_MIN_PAYLOAD_LEN` | 3 | Minimum valid packet length |
| `WM_MODEL_ID` | 9000 | Default model ID |

### Data Types

```c
// Stream mode options
typedef enum {
    WM_MODE_SINGLE = 0,  // Wait for duplicate, output once, stop
    WM_MODE_STREAM = 1   // Continuous output every Nth sample
} wm_stream_mode_t;

// End character options
typedef enum {
    WM_END_CHAR_LF_CR   = 0,   // '\n' OR '\r' (default)
    WM_END_CHAR_LF      = 1,   // '\n' only
    WM_END_CHAR_CR      = 2,   // '\r' only
    WM_END_CHAR_FF      = 3,   // Form feed
    WM_END_CHAR_NONE    = 4,   // Timeout only
    WM_END_CHAR_VT      = 5,   // Vertical tab
    WM_END_CHAR_ETB     = 6,   // End of transmission block
    WM_END_CHAR_ESC     = 7,   // Escape
    WM_END_CHAR_CSI     = 8,   // Control sequence introducer
    WM_END_CHAR_SPACE   = 9,   // Space
    WM_END_CHAR_NULL    = 10   // Null character
} wm_end_char_t;

// UART configuration
typedef struct {
    int uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int parity;  // 0=None, 1=Odd, 2=Even
} wm_config_t;

// Statistics
typedef struct {
    uint32_t packet_count;
    uint32_t filtered_bytes;
} wm_stats_t;

// Callbacks
typedef void (*wm_data_callback_t)(const char *json_data, int len);
typedef void (*wm_activity_callback_t)(void);
```

### Functions

#### Initialization

```c
esp_err_t wm_uart_init(void);
```
- **Description:** Initialize WM UART driver
- **Input:** None
- **Output:** `ESP_OK` on success, error code on failure
- **Notes:**
  - Loads config from NVS if available
  - Saves defaults to NVS on first boot
  - Configures UART hardware

```c
esp_err_t wm_uart_deinit(void);
```
- **Description:** Deinitialize WM UART driver
- **Input:** None
- **Output:** `ESP_OK` on success

#### Control

```c
esp_err_t wm_uart_start(void);
```
- **Description:** Start WM receive task
- **Input:** None
- **Output:** `ESP_OK` on success
- **Notes:** Creates FreeRTOS task for UART reception

```c
void wm_uart_stop(void);
```
- **Description:** Stop WM receive task
- **Input:** None
- **Output:** None

#### Configuration

```c
esp_err_t wm_uart_set_baud(int new_baud);
```
- **Description:** Change UART baud rate
- **Input:** `new_baud` - New baud rate (e.g., 9600, 19200, 115200)
- **Output:** `ESP_OK` on success
- **Notes:** Takes effect immediately, call `wm_uart_save_config()` to persist

```c
void wm_uart_get_config(wm_config_t *config);
```
- **Description:** Get current UART configuration
- **Input:** `config` - Pointer to config struct to fill
- **Output:** None (fills config struct)

```c
esp_err_t wm_uart_save_config(void);
```
- **Description:** Save current config to NVS
- **Input:** None
- **Output:** `ESP_OK` on success
- **Notes:** Persists: baud, parity, stream_mode, sample_rate, model_id, end_char

#### Stream Mode

```c
void wm_uart_set_stream_mode(int mode);
```
- **Description:** Set stream mode
- **Input:** `mode` - `WM_MODE_SINGLE` (0) or `WM_MODE_STREAM` (1)
- **Output:** None

```c
int wm_uart_get_stream_mode(void);
```
- **Description:** Get current stream mode
- **Input:** None
- **Output:** Current mode (0 or 1)

```c
void wm_uart_set_sample_rate(int rate);
```
- **Description:** Set sample rate (stream mode only)
- **Input:** `rate` - Output every Nth sample (1-100)
- **Output:** None

```c
int wm_uart_get_sample_rate(void);
```
- **Description:** Get current sample rate
- **Input:** None
- **Output:** Current sample rate

#### Model ID

```c
void wm_uart_set_model_id(int model_id);
```
- **Description:** Set WM model ID (included in JSON output)
- **Input:** `model_id` - Model identifier
- **Output:** None

```c
int wm_uart_get_model_id(void);
```
- **Description:** Get current model ID
- **Input:** None
- **Output:** Current model ID

#### End Character

```c
void wm_uart_set_end_char(wm_end_char_t end_char);
```
- **Description:** Set packet end character
- **Input:** `end_char` - End character enum value (0-10)
- **Output:** None

```c
wm_end_char_t wm_uart_get_end_char(void);
```
- **Description:** Get current end character setting
- **Input:** None
- **Output:** Current end character enum value

```c
const char* wm_uart_end_char_to_string(wm_end_char_t end_char);
```
- **Description:** Convert end char enum to human-readable string
- **Input:** `end_char` - End character enum value
- **Output:** String description (e.g., "LF+CR (\\n or \\r)")

```c
int wm_uart_end_char_to_byte(wm_end_char_t end_char);
```
- **Description:** Convert end char enum to byte value
- **Input:** `end_char` - End character enum value
- **Output:** Byte value, -1 for NONE, -2 for LF_CR

#### Self-Diagnosis

```c
bool wm_uart_is_connected(void);
```
- **Description:** Check if WM is connected (data received recently)
- **Input:** None
- **Output:** `true` if connected, `false` if not
- **Notes:** Returns false if no data received in last 5 seconds

```c
int wm_uart_get_last_packet_age_ms(void);
```
- **Description:** Get time since last packet received
- **Input:** None
- **Output:** Milliseconds since last packet, -1 if never received

#### Callbacks

```c
void wm_uart_set_data_callback(wm_data_callback_t callback);
```
- **Description:** Register callback for WM data (for BLE transmission)
- **Input:** `callback` - Function pointer `void (*)(const char *json, int len)`
- **Output:** None

```c
void wm_uart_set_activity_callback(wm_activity_callback_t callback);
```
- **Description:** Register callback for activity (for LED blink)
- **Input:** `callback` - Function pointer `void (*)(void)`
- **Output:** None

#### Status

```c
void wm_uart_print_status(void);
```
- **Description:** Print WM status to console
- **Input:** None
- **Output:** None (prints to stdout)

```c
void wm_uart_get_stats(wm_stats_t *stats);
```
- **Description:** Get WM statistics
- **Input:** `stats` - Pointer to stats struct to fill
- **Output:** None (fills stats struct)

```c
void wm_uart_reset_stats(void);
```
- **Description:** Reset statistics counters
- **Input:** None
- **Output:** None

### Output Format

WM data is output as JSON:
```json
{"device":"wm","data":"123.45","model":9000 }
```

---

## Component: cmd_parser

**Location:** `components/cmd_parser/`

**Purpose:** Parse and process JSON commands from USB/BLE

### Command Format

**Input:**
```json
{"command":"command_name","param1":"value1","param2":123}#
```

**Output:**
```json
{"response_message":"message","status_code":0,"data":{...}}
```

### Status Codes

| Code | Constant | Description |
|------|----------|-------------|
| 0 | `STATUS_OK` | Success |
| -1 | `STATUS_ERR` | Error |

### Functions

```c
esp_err_t cmd_parser_init(void);
```
- **Description:** Initialize command parser
- **Input:** None
- **Output:** `ESP_OK`

```c
void cmd_parser_register_output_callback(cmd_output_callback_t callback);
```
- **Description:** Register callback for BLE output
- **Input:** `callback` - Function pointer `void (*)(const char *data, int len)`
- **Output:** None

```c
void parse_and_process_commands(char *json_str, int json_len);
```
- **Description:** Parse and execute a JSON command
- **Input:**
  - `json_str` - JSON string (null-terminated)
  - `json_len` - Length of JSON string
- **Output:** None (sends response via USB and callback)

```c
void send_response(const char *msg, int status_code, const char *data);
```
- **Description:** Send a JSON response
- **Input:**
  - `msg` - Response message string
  - `status_code` - Status code (0 = OK, -1 = Error)
  - `data` - Optional JSON data string (can be NULL)
- **Output:** None (prints to USB, calls BLE callback)

```c
void send_status(const char *msg, int status_code, const char *data);
```
- **Description:** Alias for `send_response()`

### Supported Commands

#### get_firmware_version
```json
// Input
{"command":"get_firmware_version"}

// Output
{"response_message":"get_firmware_version","status_code":0,"data":"2.0.0.1000"}
```

#### ncle_get_unique_id
```json
// Input
{"command":"ncle_get_unique_id"}

// Output
{"response_message":"get_unique_id","status_code":0,"data":"98A316F082F2"}
```

#### get_current_config
```json
// Input
{"command":"get_current_config"}

// Output (3 responses + success)
{"response_message":"get_ma_config","status_code":0,"data":{"device":"ma",...}}
{"response_message":"get_wm_config","status_code":0,"data":{"device":"wm",...}}
{"response_message":"get_printer_config","status_code":0,"data":{"device":"printer",...}}
{"response_message":"get_current_config_success","status_code":0}
```

#### reset_config
```json
// Input
{"command":"reset_config"}

// Output
{"response_message":"reset_config_success","status_code":0}
```

#### wm_port_config
```json
// Input
{"command":"wm_port_config","model":9000,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":1}

// Output
{"response_message":"wm_port_config_success","status_code":0}
```

#### ma_port_config
```json
// Input
{"command":"ma_port_config","model":1,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}

// Output
{"response_message":"ma_port_config_success","status_code":0}
```

#### printer_port_config
```json
// Input
{"command":"printer_port_config","model":1,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}

// Output
{"response_message":"printer_port_config_success","status_code":0}
```

#### self_diagnosis
```json
// Input
{"command":"self_diagnosis"}

// Output
{"response_message":"self_diagnosis","status_code":0}
{"device":"self_diagnosis","firmware_version":"2.0.0.1000","hardware_id":"98A316F082F2","milk_analyzer_status":"not_implemented","weighing_machine_status":"connected","Printer_status":"not_implemented"}
```

---

## Component: config

**Location:** `components/config/`

**Purpose:** Manage device configuration with NVS persistence

### Data Types

```c
typedef struct {
    int model;
    uint32_t baud_rate;
    int data_bits;
    int stop_bits;
    int parity;
    int stream;
} port_config_t;

typedef struct {
    port_config_t ma_config;
    port_config_t wm_config;
    port_config_t printer_config;
} device_config_t;
```

### Global Variables

```c
extern device_config_t g_device_config;
```

### Functions

```c
esp_err_t device_config_init(void);
```
- **Description:** Initialize device config (load from NVS or use defaults)
- **Input:** None
- **Output:** `ESP_OK` on success

```c
esp_err_t device_config_save(void);
```
- **Description:** Save current config to NVS
- **Input:** None
- **Output:** `ESP_OK` on success

```c
void device_config_reset(void);
```
- **Description:** Reset config to defaults
- **Input:** None
- **Output:** None

---

## Component: common

**Location:** `components/common/`

**Purpose:** Common utilities and definitions

### Functions

```c
uint64_t get_device_id(void);
```
- **Description:** Get unique hardware ID (ESP32 MAC address)
- **Input:** None
- **Output:** 64-bit unique ID

### Constants

```c
#define NCLE_FIRMWARE_VERSION "2.0.0.1000"
```

---

## Main Application (main.c)

**Location:** `main/main.c`

### Console Menu Keys

| Key | Function |
|-----|----------|
| `1` | Show status |
| `4` | Set WM baud 19200 |
| `5` | Set WM baud 9600 |
| `6` | Set WM baud 115200 |
| `0` | Stream mode |
| `9` | Single-read mode |
| `+` | Increase sample rate |
| `-` | Decrease sample rate |
| `e` | Cycle end character |
| `w` | Start WM UART |
| `x` | Stop WM UART |
| `s` | Save WM config |
| `j` | Enter JSON command mode |
| `r` | Reboot |
| `m` | Show menu |

### JSON Command Mode

1. Press `j` to enter JSON mode
2. Type JSON command: `{"command":"self_diagnosis"}`
3. Press `Enter` or `#` to execute
4. Press `ESC` to exit JSON mode

---

## NVS Storage

### WM Config Namespace: `wm_config`

| Key | Type | Description |
|-----|------|-------------|
| `baud` | int32 | Baud rate |
| `parity` | int32 | Parity setting |
| `rx_pin` | int32 | RX GPIO pin |
| `tx_pin` | int32 | TX GPIO pin |
| `stream` | int32 | Stream mode (0/1) |
| `sample_rate` | int32 | Sample rate |
| `model_id` | int32 | Model ID |
| `end_char` | int32 | End character (0-10) |

---

## Build Configuration (menuconfig)

### NCLite Configuration

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_NCLE_STATUS_LED_GPIO` | 36 | Status LED GPIO |
| `CONFIG_NCLE_STATUS_LED_ACTIVE_HIGH` | yes | LED active level |
| `CONFIG_NCLE_WM_ENABLE` | yes | Enable WM module |
| `CONFIG_NCLE_WM_TX_PIN` | 43 | WM TX GPIO |
| `CONFIG_NCLE_WM_RX_PIN` | 44 | WM RX GPIO |
| `CONFIG_NCLE_WM_BAUD_RATE` | 9600 | WM default baud |

---

## Error Handling

All ESP-IDF functions return `esp_err_t`:
- `ESP_OK` (0) - Success
- `ESP_FAIL` (-1) - General failure
- `ESP_ERR_NO_MEM` - Memory allocation failed
- `ESP_ERR_INVALID_STATE` - Invalid state for operation

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.0.0.1000 | Dec 2024 | Initial ESP32-S3 port from Pico2W |

---

## Author

Nitara Technologies
