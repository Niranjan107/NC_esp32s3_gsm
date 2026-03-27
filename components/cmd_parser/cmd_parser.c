/**
 * @file cmd_parser.c
 * @brief Command parser implementation for NCLite ESP32-S3
 *
 * PURPOSE:
 * This module is the CENTRAL COMMAND PROCESSOR for all JSON commands received
 * from the mobile app (via BLE) or USB console. It parses JSON, validates commands,
 * dispatches to appropriate handlers, and sends responses back.
 *
 * WHY JSON?
 * - Human-readable format for debugging
 * - Self-describing structure
 * - Easy to parse with cJSON library
 * - Compatible with mobile apps (Android/iOS)
 *
 * ARCHITECTURE:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         Mobile App / USB Console                        │
 * │                                                                          │
 * │  Sends JSON commands like:                                              │
 * │  {"command":"wm_port_config","model":9000,"baud_rate":9600,...}#        │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              JSON command
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         cmd_parser.c (THIS FILE)                        │
 * │                                                                          │
 * │  1. parse_and_process_commands() - Entry point                          │
 * │     - Parse JSON with cJSON library                                     │
 * │     - Extract "command" field                                           │
 * │     - Dispatch to appropriate handler                                   │
 * │                                                                          │
 * │  2. Command Handlers (process_xxx_config):                              │
 * │     - Validate parameters                                               │
 * │     - Update g_device_config                                            │
 * │     - Save to NVS                                                       │
 * │     - Apply to UART modules                                             │
 * │                                                                          │
 * │  3. send_response() - Send JSON response back                           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                           JSON response
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                   USB Console + BLE (via s_output_callback)            │
 * │                                                                          │
 * │  {"response_message":"wm_config_success","status_code":0}               │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * SUPPORTED COMMANDS:
 * - wm_port_config     : Configure Weighing Machine UART
 * - ma_port_config     : Configure Milk Analyzer UART
 * - printer_port_config: Configure Thermal Printer UART
 * - reset_config       : Reset all configs to defaults
 * - get_current_config : Get all current configurations
 * - ncle_get_unique_id : Get device unique ID
 * - get_firmware_version: Get firmware version
 * - self_diagnosis     : Run self-diagnostics
 * - print_receipt      : Print receipt data
 * - check_printer_status: Check if printer is ready
 * - reprint_last       : Reprint last receipt
 *
 * JSON RESPONSE FORMAT:
 * {"response_message":"<msg>","status_code":<0|1>,"data":<optional_json>}
 * - status_code 0 = success (STATUS_OK)
 * - status_code 1 = error (STATUS_ERR)
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "cJSON.h"

#include "cmd_parser.h"
#include "common.h"
#include "device_config.h"
#include "wm_uart.h"
#include "ma_uart.h"

#ifdef CONFIG_NCLE_PRINTER_ENABLE
#include "printer_uart.h"
#endif

#ifdef CONFIG_NCLE_BATTERY_ENABLE
#include "battery.h"
#endif

#ifdef CONFIG_NCLE_OTA_ENABLE
#include "ota.h"
#include "mbedtls/base64.h"
#endif

static const char *TAG = "CMD_PARSER";

/*******************************************************************************
 * Private Variables
 *
 * PURPOSE: Module-level state for command parsing and response handling
 ******************************************************************************/

/**
 * BLE Output Callback
 * PURPOSE: Function pointer to send responses via BLE
 *
 * WHY CALLBACK?
 * cmd_parser doesn't directly depend on ble_spp module. Instead, main.c
 * registers ble_spp_send as the callback during initialization.
 * This keeps modules loosely coupled.
 *
 * Set by: cmd_parser_register_output_callback() called from main.c
 * Used by: send_response() to send JSON responses via BLE
 */
static cmd_output_callback_t s_output_callback = NULL;

/**
 * Response Buffer
 * PURPOSE: Static buffer for building JSON response strings
 *
 * WHY STATIC?
 * - Avoids stack allocation for potentially large responses
 * - Single buffer is sufficient (commands processed sequentially)
 * - Size defined by RESPONSE_BUF_SIZE constant
 */
static char s_response_buffer[RESPONSE_BUF_SIZE];

/*******************************************************************************
 * Private Function Prototypes
 *
 * PURPOSE: Forward declarations for command handler functions
 * Each handler processes a specific JSON command type
 ******************************************************************************/

static void process_wm_config(cJSON *root);       /* Handle wm_port_config command */
static void process_ma_config(cJSON *root);       /* Handle ma_port_config command */
static void process_printer_config(cJSON *root);  /* Handle printer_port_config command */
static void process_reset_config(void);           /* Handle reset_config command */
static void process_get_current_config(void);     /* Handle get_current_config command */
static void process_get_unique_id(void);          /* Handle ncle_get_unique_id command */
static void process_get_firmware_version(void);   /* Handle get_firmware_version command */
static void process_self_diagnosis(void);         /* Handle self_diagnosis command */

/*******************************************************************************
 * Public Functions
 *
 * PURPOSE: External API for command parsing module
 ******************************************************************************/

/**
 * @brief Initialize the command parser module
 *
 * PURPOSE:
 * Prepare the command parser for operation. Currently just resets
 * the output callback to NULL.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK always
 *
 * CALLED FROM: app_main() in main.c during startup
 */
esp_err_t cmd_parser_init(void)
{
    ESP_LOGI(TAG, "Command parser initialized");
    s_output_callback = NULL;
    return ESP_OK;
}

/**
 * @brief Register callback function for sending responses via BLE
 *
 * PURPOSE:
 * Set the function that will be called to send responses over BLE.
 * This decouples cmd_parser from ble_spp module.
 *
 * INPUT:
 * @param callback - Function pointer: void callback(const char *data, unsigned int len)
 *                   Typically ble_spp_output_callback
 *
 * OUTPUT: None
 *
 * CALLED FROM: app_main() in main.c after BLE initialization
 *
 * EXAMPLE:
 *   cmd_parser_register_output_callback(ble_spp_output_callback);
 */
void cmd_parser_register_output_callback(cmd_output_callback_t callback)
{
    s_output_callback = callback;
    ESP_LOGI(TAG, "Output callback registered");
}

/**
 * @brief Send a JSON response back to the mobile app
 *
 * PURPOSE:
 * Build and send a JSON-formatted response to both USB console and BLE.
 * This is the PRIMARY FUNCTION for sending responses to the mobile app.
 *
 * INPUT:
 * @param msg         - Response message string (e.g., "wm_config_success")
 * @param status_code - Status code: STATUS_OK (0) or STATUS_ERR (1)
 * @param data        - Optional JSON data string, or NULL if no data
 *                      Must be valid JSON if provided (e.g., "{...}" or "\"string\"")
 *
 * OUTPUT:
 * No return value. Side effects:
 * - Prints response to USB console via printf()
 * - Sends response to BLE via s_output_callback (if registered)
 *
 * RESPONSE FORMAT:
 * With data:    {"response_message":"msg","status_code":0,"data":{...}}
 * Without data: {"response_message":"msg","status_code":0}
 *
 * EXAMPLE:
 *   send_response("wm_config_success", STATUS_OK, NULL);
 *   → {"response_message":"wm_config_success","status_code":0}
 *
 *   send_response("get_unique_id", STATUS_OK, "\"ABC123DEF456\"");
 *   → {"response_message":"get_unique_id","status_code":0,"data":"ABC123DEF456"}
 */
void send_response(const char *msg, int status_code, const char *data)
{
    int len;

    if (data != NULL) {
        len = snprintf(s_response_buffer, sizeof(s_response_buffer),
                       "{\"response_message\":\"%s\",\"status_code\":%d,\"data\":%s}\n",
                       msg, status_code, data);
    } else {
        len = snprintf(s_response_buffer, sizeof(s_response_buffer),
                       "{\"response_message\":\"%s\",\"status_code\":%d}\n",
                       msg, status_code);
    }

    // Output to USB console
    printf("%s", s_response_buffer);

    // Output to BLE (if callback registered)
    if (s_output_callback != NULL) {
        s_output_callback(s_response_buffer, len);
    }
}

/**
 * @brief Send status (alias for send_response)
 *
 * PURPOSE:
 * Alias function for send_response() for code readability.
 * Used when sending status updates rather than command responses.
 *
 * INPUT:
 * @param msg         - Status message string
 * @param status_code - STATUS_OK (0) or STATUS_ERR (1)
 * @param data        - Optional JSON data string
 *
 * OUTPUT:
 * No return value. Calls send_response() internally.
 */
void send_status(const char *msg, int status_code, const char *data)
{
    send_response(msg, status_code, data);
}

/**
 * @brief Parse and process a JSON command string
 *
 * PURPOSE:
 * This is the MAIN ENTRY POINT for command processing. It receives raw JSON
 * strings from BLE (process_ble_rx_data) or USB console, parses them, and
 * dispatches to the appropriate command handler.
 *
 * INPUT:
 * @param json_str - Raw JSON command string (e.g., '{"command":"wm_port_config",...}')
 * @param json_len - Length of JSON string in bytes
 *
 * OUTPUT:
 * No return value. Side effects:
 * - Parses JSON and dispatches to appropriate handler
 * - Sends response via send_response()
 *
 * COMMAND FORMAT:
 * {"command":"<cmd_name>", ...additional parameters...}
 *
 * SUPPORTED COMMANDS:
 * - "wm_port_config"      → process_wm_config()
 * - "ma_port_config"      → process_ma_config()
 * - "printer_port_config" → process_printer_config()
 * - "reset_config"        → process_reset_config()
 * - "get_current_config"  → process_get_current_config()
 * - "ncle_get_unique_id"  → process_get_unique_id()
 * - "get_firmware_version"→ process_get_firmware_version()
 * - "self_diagnosis"      → process_self_diagnosis()
 * - "print_receipt"       → Direct printing
 * - "check_printer_status"→ Check printer
 * - "reprint_last"        → Reprint last receipt
 *
 * ERROR HANDLING:
 * - Invalid JSON → sends RESP_INVALID_JSON
 * - Missing "command" key → sends RESP_COMMAND_NOT_FOUND
 * - Unknown command → sends RESP_UNDEFINED_COMMAND
 *
 * CALLED FROM: process_ble_rx_data() in ble_spp.c
 */
void parse_and_process_commands(char *json_str, int json_len)
{
    // Parse JSON
    cJSON *root = cJSON_ParseWithLength(json_str, json_len);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON parse error: %s", error_ptr ? error_ptr : "unknown");
        send_response(RESP_INVALID_JSON, STATUS_ERR, NULL);
        return;
    }

    // Extract "command" key
    cJSON *cmd_item = cJSON_GetObjectItem(root, KEY_COMMAND);
    if (cmd_item == NULL || !cJSON_IsString(cmd_item)) {
        ESP_LOGE(TAG, "Missing or invalid 'command' key");
        send_response(RESP_COMMAND_NOT_FOUND, STATUS_ERR, NULL);
        cJSON_Delete(root);
        return;
    }

    const char *cmd = cmd_item->valuestring;
    ESP_LOGI(TAG, "Processing command: %s", cmd);

    // Command dispatch
    if (strcmp(cmd, CMD_RESET_CONFIG) == 0) {
        process_reset_config();
    }
    else if (strcmp(cmd, CMD_GET_CURRENT_CONFIG) == 0) {
        process_get_current_config();
    }
    else if (strcmp(cmd, CMD_WM_PORT_CONFIG) == 0) {
        process_wm_config(root);
    }
    else if (strcmp(cmd, CMD_MA_PORT_CONFIG) == 0) {
        process_ma_config(root);
    }
    else if (strcmp(cmd, CMD_PRINTER_PORT_CONFIG) == 0) {
        process_printer_config(root);
    }
    else if (strcmp(cmd, CMD_GET_UNIQUE_ID) == 0) {
        process_get_unique_id();
    }
    else if (strcmp(cmd, CMD_GET_FIRMWARE_VERSION) == 0) {
        process_get_firmware_version();
    }
    else if (strcmp(cmd, CMD_SELF_DIAGNOSIS) == 0) {
        process_self_diagnosis();
    }
    /*-------------------------------------------------------------------------
     * Battery Status Command
     * Returns battery percentage (0-100) only for compact BLE response
     * JSON: {"command": "get_battery_status"}#
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_GET_BATTERY_STATUS) == 0) {
#ifdef CONFIG_NCLE_BATTERY_ENABLE
        uint8_t pct = battery_get_percentage();
        static char batt_data[16];
        snprintf(batt_data, sizeof(batt_data), "{\"pct\":%d}", pct);
        send_response("battery", STATUS_OK, batt_data);
#else
        send_response("battery_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, CMD_PRINT_RECEIPT) == 0) {
#ifdef CONFIG_NCLE_PRINTER_ENABLE
        // Extract receipt data from JSON
        cJSON *data_item = cJSON_GetObjectItem(root, KEY_DATA);
        if (data_item == NULL || !cJSON_IsString(data_item)) {
            ESP_LOGE(TAG, "Missing 'data' field for print_receipt");
            send_response("print_receipt_fail", STATUS_ERR, NULL);
        } else {
            const char *receipt_data = data_item->valuestring;
            ESP_LOGI(TAG, "Printing receipt (%d bytes)", (int)strlen(receipt_data));
            if (printer_print_with_special_chars(receipt_data)) {
                send_response("print_receipt_success", STATUS_OK, NULL);
            } else {
                send_response("print_receipt_fail", STATUS_ERR, NULL);
            }
        }
#else
        send_response("printer_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, CMD_CHECK_PRINTER_STATUS) == 0) {
#ifdef CONFIG_NCLE_PRINTER_ENABLE
        bool is_ready = printer_check_status();
        send_response("check_printer_status", STATUS_OK, is_ready ? "\"ready\"" : "\"not_ready\"");
#else
        send_response("printer_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, CMD_REPRINT_LAST_RECEIPT) == 0) {
#ifdef CONFIG_NCLE_PRINTER_ENABLE
        if (printer_reprint_last_receipt()) {
            send_response("reprint_success", STATUS_OK, NULL);
        } else {
            send_response("reprint_fail_no_receipt", STATUS_ERR, NULL);
        }
#else
        send_response("printer_not_enabled", STATUS_ERR, NULL);
#endif
    }
    /*-------------------------------------------------------------------------
     * OTA Update Commands
     * Handles firmware updates over BLE
     * Commands: ota_begin, ota_write, ota_end, ota_abort
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, "ota_begin") == 0) {
#ifdef CONFIG_NCLE_OTA_ENABLE
        cJSON *size_item = cJSON_GetObjectItem(root, "size");
        if (size_item == NULL || !cJSON_IsNumber(size_item)) {
            send_response("ota_begin", STATUS_ERR, "\"missing size\"");
        } else {
            uint32_t fw_size = (uint32_t)size_item->valuedouble;
            esp_err_t err = ota_begin(fw_size);
            if (err == ESP_OK) {
                static char ota_resp[64];
                snprintf(ota_resp, sizeof(ota_resp), "{\"ready\":true,\"size\":%lu}", fw_size);
                send_response("ota_begin", STATUS_OK, ota_resp);
            } else {
                send_response("ota_begin", STATUS_ERR, "\"begin failed\"");
            }
        }
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, "ota_write") == 0) {
#ifdef CONFIG_NCLE_OTA_ENABLE
        cJSON *seq_item = cJSON_GetObjectItem(root, "seq");
        cJSON *data_item = cJSON_GetObjectItem(root, "data");
        if (seq_item == NULL || data_item == NULL || !cJSON_IsString(data_item)) {
            send_response("ota_write", STATUS_ERR, "\"missing params\"");
        } else {
            uint32_t seq = (uint32_t)seq_item->valuedouble;
            const char *b64_data = data_item->valuestring;
            size_t b64_len = strlen(b64_data);

            // Decode Base64
            static uint8_t decode_buf[512];
            size_t decoded_len = 0;
            int ret = mbedtls_base64_decode(decode_buf, sizeof(decode_buf),
                                            &decoded_len,
                                            (const unsigned char *)b64_data, b64_len);
            if (ret != 0) {
                send_response("ota_write", STATUS_ERR, "\"decode failed\"");
            } else {
                esp_err_t err = ota_write_chunk(decode_buf, decoded_len, seq);
                if (err == ESP_OK) {
                    ota_progress_t prog;
                    ota_get_progress(&prog);
                    static char ota_resp[48];
                    snprintf(ota_resp, sizeof(ota_resp), "{\"pct\":%d,\"seq\":%lu}",
                             prog.progress_percent, seq);
                    send_response("ota_write", STATUS_OK, ota_resp);
                } else {
                    send_response("ota_write", STATUS_ERR, "\"write failed\"");
                }
            }
        }
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, "ota_end") == 0) {
#ifdef CONFIG_NCLE_OTA_ENABLE
        esp_err_t err = ota_end();
        if (err == ESP_OK) {
            send_response("ota_end", STATUS_OK, "{\"verified\":true}");
            // Delay to allow response to be sent, then reboot
            vTaskDelay(pdMS_TO_TICKS(1000));
            ota_reboot();
        } else {
            send_response("ota_end", STATUS_ERR, "\"verify failed\"");
        }
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, "ota_abort") == 0) {
#ifdef CONFIG_NCLE_OTA_ENABLE
        ota_abort();
        send_response("ota_abort", STATUS_OK, NULL);
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, "ota_status") == 0) {
#ifdef CONFIG_NCLE_OTA_ENABLE
        ota_progress_t prog;
        ota_get_progress(&prog);
        static char ota_resp[128];
        snprintf(ota_resp, sizeof(ota_resp),
                 "{\"state\":%d,\"pct\":%d,\"recv\":%lu,\"total\":%lu}",
                 prog.state, prog.progress_percent,
                 prog.received_size, prog.total_size);
        send_response("ota_status", STATUS_OK, ota_resp);
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
        send_response(RESP_UNDEFINED_COMMAND, STATUS_ERR, NULL);
    }

    // Cleanup
    cJSON_Delete(root);
}

/*******************************************************************************
 * Private Functions - Command Handlers
 *
 * PURPOSE: Each function handles a specific JSON command type
 * All handlers follow the same pattern:
 * 1. Extract parameters from JSON
 * 2. Validate parameters
 * 3. Update g_device_config
 * 4. Save to NVS
 * 5. Apply to hardware (UART modules)
 * 6. Send response
 ******************************************************************************/

/**
 * @brief Process wm_port_config command - Configure Weighing Machine UART
 *
 * PURPOSE:
 * Configure the Weighing Machine (WM) UART port settings when the mobile app
 * sends the wm_port_config command. Updates config, saves to NVS, and applies
 * to the WM UART hardware.
 *
 * INPUT:
 * @param root - Parsed cJSON object containing the command
 *
 * EXPECTED JSON FORMAT:
 * {"command":"wm_port_config","model":9000,"baud_rate":9600,
 *  "data_bits":8,"stop_bits":1,"parity":0,"stream":0}
 *
 * PARAMETERS:
 * - model     : WM model ID (e.g., 9000 for specific WM type)
 * - baud_rate : Serial baud rate (e.g., 9600, 19200)
 * - data_bits : Data bits (typically 8)
 * - stop_bits : Stop bits (1 or 2)
 * - parity    : 0=none, 1=even, 2=odd
 * - stream    : 0=on-demand, 1=continuous streaming
 *
 * OUTPUT:
 * Sends response via send_response():
 * - Success: "wm_config_success"
 * - Failure: "wm_config_fail"
 *
 * SIDE EFFECTS:
 * - Updates g_device_config.wm_config
 * - Saves config to NVS
 * - Applies baud rate to WM UART
 * - Auto-starts WM receive task
 */
static void process_wm_config(cJSON *root)
{
    // Extract config parameters
    cJSON *model = cJSON_GetObjectItem(root, KEY_MODEL);
    cJSON *baud = cJSON_GetObjectItem(root, KEY_BAUD_RATE);
    cJSON *data_bits = cJSON_GetObjectItem(root, KEY_DATA_BITS);
    cJSON *stop_bits = cJSON_GetObjectItem(root, KEY_STOP_BITS);
    cJSON *parity = cJSON_GetObjectItem(root, KEY_PARITY);
    cJSON *stream = cJSON_GetObjectItem(root, KEY_STREAM);

    // Validate required fields
    if (!model || !baud || !data_bits || !stop_bits || !parity || !stream) {
        ESP_LOGE(TAG, "Missing WM config parameters");
        send_response(RESP_WM_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

    // Update global config
    g_device_config.wm_config.model = model->valueint;
    g_device_config.wm_config.baud_rate = baud->valueint;
    g_device_config.wm_config.data_bits = data_bits->valueint;
    g_device_config.wm_config.stop_bits = stop_bits->valueint;
    g_device_config.wm_config.parity = parity->valueint;
    g_device_config.wm_config.stream_mode = stream->valueint;

    // Save to NVS
    esp_err_t err = config_set_wm_port(&g_device_config.wm_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WM config to NVS");
        send_response(RESP_WM_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

    // Apply to WM UART at runtime
    wm_uart_set_baud(baud->valueint);
    wm_uart_set_model_id(model->valueint);
    wm_uart_set_stream_mode(stream->valueint);

    // Auto-start WM receiving when JSON config is sent (for both stream=0 and stream=1)
    // This allows user to send config and immediately start receiving
    ESP_LOGI(TAG, "Auto-starting WM receive task");
    wm_uart_start();

    ESP_LOGI(TAG, "WM config applied: model=%d, baud=%d, stream=%d",
             model->valueint, baud->valueint, stream->valueint);

    send_response(RESP_WM_CONFIG_SUCCESS, STATUS_OK, NULL);
}

/**
 * @brief Process ma_port_config command - Configure Milk Analyzer UART
 *
 * PURPOSE:
 * Configure the Milk Analyzer (MA) UART port settings. Similar to WM config
 * but for milk analysis equipment.
 *
 * INPUT:
 * @param root - Parsed cJSON object containing the command
 *
 * EXPECTED JSON FORMAT:
 * {"command":"ma_port_config","model":1001,"baud_rate":9600,
 *  "data_bits":8,"stop_bits":1,"parity":0,"stream":0}
 *
 * PARAMETERS:
 * - model     : MA model ID (e.g., 1001 for specific MA type)
 * - baud_rate : Serial baud rate
 * - data_bits : Data bits (typically 8)
 * - stop_bits : Stop bits (1 or 2)
 * - parity    : 0=none, 1=even, 2=odd
 * - stream    : 0=on-demand, 1=continuous streaming
 *
 * OUTPUT:
 * Sends response:
 * - Success: "ma_config_success"
 * - Failure: "ma_config_fail"
 *
 * NOTE: MA is always running (always receives data). The stream setting
 * only controls whether data is forwarded to BLE, not whether it's received.
 */
static void process_ma_config(cJSON *root)
{
    // Extract config parameters
    cJSON *model = cJSON_GetObjectItem(root, KEY_MODEL);
    cJSON *baud = cJSON_GetObjectItem(root, KEY_BAUD_RATE);
    cJSON *data_bits = cJSON_GetObjectItem(root, KEY_DATA_BITS);
    cJSON *stop_bits = cJSON_GetObjectItem(root, KEY_STOP_BITS);
    cJSON *parity = cJSON_GetObjectItem(root, KEY_PARITY);
    cJSON *stream = cJSON_GetObjectItem(root, KEY_STREAM);

    // Validate required fields
    if (!model || !baud || !data_bits || !stop_bits || !parity || !stream) {
        ESP_LOGE(TAG, "Missing MA config parameters");
        send_response(RESP_MA_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

    // Update global config
    g_device_config.ma_config.model = model->valueint;
    g_device_config.ma_config.baud_rate = baud->valueint;
    g_device_config.ma_config.data_bits = data_bits->valueint;
    g_device_config.ma_config.stop_bits = stop_bits->valueint;
    g_device_config.ma_config.parity = parity->valueint;
    g_device_config.ma_config.stream_mode = stream->valueint;

    // Save to NVS
    esp_err_t err = config_set_ma_port(&g_device_config.ma_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save MA config to NVS");
        send_response(RESP_MA_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

    // Apply to MA UART at runtime
    ma_uart_set_baud(baud->valueint);
    ma_uart_set_model_id(model->valueint);
    ma_uart_set_stream_mode(stream->valueint);
    // Note: MA is always running (like Pico2W PIO behavior)
    // stream setting only controls output filtering, not receiving

    ESP_LOGI(TAG, "MA config applied: model=%d, baud=%d, stream=%d",
             model->valueint, baud->valueint, stream->valueint);

    send_response(RESP_MA_CONFIG_SUCCESS, STATUS_OK, NULL);
}

/**
 * @brief Process printer_port_config command - Configure Thermal Printer UART
 *
 * PURPOSE:
 * Configure the Thermal Printer UART port settings for ESC/POS printing.
 *
 * INPUT:
 * @param root - Parsed cJSON object containing the command
 *
 * EXPECTED JSON FORMAT:
 * {"command":"printer_port_config","model":8000,"baud_rate":9600,
 *  "data_bits":8,"stop_bits":1,"parity":0,"stream":0}
 *
 * PARAMETERS:
 * - model     : Printer model ID
 * - baud_rate : Serial baud rate (commonly 9600 or 19200)
 * - data_bits : Data bits (typically 8)
 * - stop_bits : Stop bits (1 or 2)
 * - parity    : 0=none, 1=even, 2=odd (some printers need odd parity)
 * - stream    : Not used for printer (always 0)
 *
 * OUTPUT:
 * Sends response:
 * - Success: "printer_config_success"
 * - Failure: "printer_config_fail"
 *
 * NOTE: Only baud rate and parity are applied to printer hardware.
 * Other settings are stored but may not affect printer operation.
 */
static void process_printer_config(cJSON *root)
{
    // Extract config parameters
    cJSON *model = cJSON_GetObjectItem(root, KEY_MODEL);
    cJSON *baud = cJSON_GetObjectItem(root, KEY_BAUD_RATE);
    cJSON *data_bits = cJSON_GetObjectItem(root, KEY_DATA_BITS);
    cJSON *stop_bits = cJSON_GetObjectItem(root, KEY_STOP_BITS);
    cJSON *parity = cJSON_GetObjectItem(root, KEY_PARITY);
    cJSON *stream = cJSON_GetObjectItem(root, KEY_STREAM);

    // Validate required fields
    if (!model || !baud || !data_bits || !stop_bits || !parity || !stream) {
        ESP_LOGE(TAG, "Missing Printer config parameters");
        send_response(RESP_PRINTER_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

    // Update global config
    g_device_config.printer_config.model = model->valueint;
    g_device_config.printer_config.baud_rate = baud->valueint;
    g_device_config.printer_config.data_bits = data_bits->valueint;
    g_device_config.printer_config.stop_bits = stop_bits->valueint;
    g_device_config.printer_config.parity = parity->valueint;
    g_device_config.printer_config.stream_mode = stream->valueint;

    // Save to NVS
    esp_err_t err = config_set_printer_port(&g_device_config.printer_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Printer config to NVS");
        send_response(RESP_PRINTER_CONFIG_FAIL, STATUS_ERR, NULL);
        return;
    }

#ifdef CONFIG_NCLE_PRINTER_ENABLE
    // Apply to Printer UART at runtime
    printer_uart_set_baud(baud->valueint);
    printer_uart_set_parity(parity->valueint);
#endif

    ESP_LOGI(TAG, "Printer config applied: model=%d, baud=%d, parity=%d",
             model->valueint, baud->valueint, parity->valueint);

    send_response(RESP_PRINTER_CONFIG_SUCCESS, STATUS_OK, NULL);
}

/**
 * @brief Process reset_config command - Reset all configurations to defaults
 *
 * PURPOSE:
 * Reset all device configurations (WM, MA, Printer) to their default values.
 * Saves defaults to NVS and applies to running UART modules.
 *
 * INPUT: None (command has no parameters)
 *
 * OUTPUT:
 * Sends response:
 * - Success: "reset_config_success"
 * - Failure: "reset_config_fail"
 *
 * SIDE EFFECTS:
 * - Resets g_device_config to defaults (defined in device_config.c)
 * - Saves defaults to NVS
 * - Applies default baud rates and settings to WM, MA, Printer UARTs
 *
 * DEFAULTS (from device_config.c):
 * - WM: model=9000, baud=9600, stream=0
 * - MA: model=1001, baud=9600, stream=0
 * - Printer: model=8000, baud=9600, parity=0
 */
static void process_reset_config(void)
{
    ESP_LOGI(TAG, "Resetting all configs to defaults");

    // Reset all device configs to defaults and save to NVS
    esp_err_t err = config_reset_to_defaults();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset config");
        send_response("reset_config_fail", STATUS_ERR, NULL);
        return;
    }

    // Apply WM defaults to UART
    wm_uart_set_baud(g_device_config.wm_config.baud_rate);
    wm_uart_set_model_id(g_device_config.wm_config.model);
    wm_uart_set_stream_mode(g_device_config.wm_config.stream_mode);

    // Apply MA defaults to UART
    ma_uart_set_baud(g_device_config.ma_config.baud_rate);
    ma_uart_set_model_id(g_device_config.ma_config.model);
    ma_uart_set_stream_mode(g_device_config.ma_config.stream_mode);

    // TODO: Apply Printer defaults when ready

    send_response(RESP_RESET_CONFIG_SUCCESS, STATUS_OK, NULL);
}

/**
 * @brief Process get_current_config command - Return all current configurations
 *
 * PURPOSE:
 * Send all current device configurations to the mobile app.
 * Sends separate responses for MA, WM, and Printer configs, followed
 * by a success message.
 *
 * INPUT: None (command has no parameters)
 *
 * OUTPUT:
 * Sends 4 responses in sequence:
 * 1. {"response_message":"get_ma_config","data":{ma config}}
 * 2. {"response_message":"get_wm_config","data":{wm config}}
 * 3. {"response_message":"get_printer_config","data":{printer config}}
 * 4. {"response_message":"get_current_config_success"}
 *
 * DATA FORMAT for each device:
 * {"device":"wm","model":9000,"baud_rate":9600,"data_bits":8,
 *  "stop_bits":1,"parity":0,"stream":0}
 *
 * WHY MULTIPLE RESPONSES?
 * This matches the Pico2W connector behavior that the mobile app expects.
 * The app processes each response separately.
 */
static void process_get_current_config(void)
{
    static char data_buf[256];  // static to avoid stack overflow

    // Send MA config
    snprintf(data_buf, sizeof(data_buf),
             "{\"device\":\"ma\",\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,"
             "\"stop_bits\":%d,\"parity\":%d,\"stream\":%d}",
             g_device_config.ma_config.model,
             (unsigned long)g_device_config.ma_config.baud_rate,
             g_device_config.ma_config.data_bits,
             g_device_config.ma_config.stop_bits,
             g_device_config.ma_config.parity,
             g_device_config.ma_config.stream_mode ? 1 : 0);
    send_response("get_ma_config", STATUS_OK, data_buf);

    // Send WM config
    snprintf(data_buf, sizeof(data_buf),
             "{\"device\":\"wm\",\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,"
             "\"stop_bits\":%d,\"parity\":%d,\"stream\":%d}",
             g_device_config.wm_config.model,
             (unsigned long)g_device_config.wm_config.baud_rate,
             g_device_config.wm_config.data_bits,
             g_device_config.wm_config.stop_bits,
             g_device_config.wm_config.parity,
             g_device_config.wm_config.stream_mode ? 1 : 0);
    send_response("get_wm_config", STATUS_OK, data_buf);

    // Send Printer config
    snprintf(data_buf, sizeof(data_buf),
             "{\"device\":\"printer\",\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,"
             "\"stop_bits\":%d,\"parity\":%d,\"stream\":%d}",
             g_device_config.printer_config.model,
             (unsigned long)g_device_config.printer_config.baud_rate,
             g_device_config.printer_config.data_bits,
             g_device_config.printer_config.stop_bits,
             g_device_config.printer_config.parity,
             g_device_config.printer_config.stream_mode ? 1 : 0);
    send_response("get_printer_config", STATUS_OK, data_buf);

    // Final success message
    send_response(RESP_GET_CONFIG_SUCCESS, STATUS_OK, NULL);
}

/**
 * @brief Process ncle_get_unique_id command - Return device unique ID
 *
 * PURPOSE:
 * Return the unique hardware ID of the ESP32-S3 device.
 * Used by mobile app to identify and track individual devices.
 *
 * INPUT: None
 *
 * OUTPUT:
 * Sends response with 12-character hex ID:
 * {"response_message":"ncle_get_unique_id","status_code":0,"data":"AABBCCDDEEFF"}
 *
 * ID SOURCE:
 * Uses the ESP32-S3's built-in unique MAC address (eFuse).
 * See get_unique_id() in common.c for implementation.
 */
static void process_get_unique_id(void)
{
    char id_buf[32];
    uint64_t id = get_unique_id();

    // Format as quoted hex string (matches Pico2W format)
    snprintf(id_buf, sizeof(id_buf), "\"%012llX\"", (unsigned long long)id);

    send_response(CMD_GET_UNIQUE_ID, STATUS_OK, id_buf);
}

/**
 * @brief Process get_firmware_version command - Return firmware version
 *
 * PURPOSE:
 * Return the current firmware version string.
 * Used by mobile app to check for updates or compatibility.
 *
 * INPUT: None
 *
 * OUTPUT:
 * Sends response with version string:
 * {"response_message":"get_firmware_version","status_code":0,"data":"1.0.0"}
 *
 * VERSION SOURCE:
 * NCLE_FIRMWARE_VERSION constant defined in common.h
 */
static void process_get_firmware_version(void)
{
    char ver_buf[64];
    snprintf(ver_buf, sizeof(ver_buf), "\"%s\"", NCLE_FIRMWARE_VERSION);

    send_response(CMD_GET_FIRMWARE_VERSION, STATUS_OK, ver_buf);
}

/**
 * @brief Process self_diagnosis command - Run device self-diagnostics
 *
 * PURPOSE:
 * Perform self-diagnostic checks on all device components and report
 * their status. Used by mobile app or technicians to verify device health.
 *
 * INPUT: None
 *
 * OUTPUT:
 * Sends two responses:
 * 1. Acknowledgment: {"response_message":"self_diagnosis","status_code":0}
 * 2. Diagnostic report (raw JSON):
 *    {"device":"self_diagnosis",
 *     "firmware_version":"1.0.0",
 *     "hardware_id":"AABBCCDDEEFF",
 *     "milk_analyzer_status":"connected|not_connected",
 *     "weighing_machine_status":"connected|not_connected",
 *     "Printer_status":"ready|not_initialized|disabled"}
 *
 * CHECKS PERFORMED:
 * - WM: Checks if data has been received recently (wm_uart_is_connected)
 * - MA: Checks if data has been received recently (ma_uart_is_connected)
 * - Printer: Checks if printer module is initialized
 */
static void process_self_diagnosis(void)
{
    // Acknowledge command received
    send_response(CMD_SELF_DIAGNOSIS, STATUS_OK, NULL);

    ESP_LOGI(TAG, "Self diagnosis started...");

    // Check WM connection status
    const char *wm_status = wm_uart_is_connected() ? "connected" : "not_connected";

    // Check MA connection status
    const char *ma_status = ma_uart_is_connected() ? "connected" : "not_connected";

    // Check Printer status
#ifdef CONFIG_NCLE_PRINTER_ENABLE
    const char *printer_status = printer_uart_is_initialized() ? "ready" : "not_initialized";
#else
    const char *printer_status = "disabled";
#endif

    // Check Battery status
#ifdef CONFIG_NCLE_BATTERY_ENABLE
    battery_info_t batt_info;
    battery_get_info(&batt_info);
    static char batt_str[64];
    snprintf(batt_str, sizeof(batt_str), "%lumV (%d%%)", batt_info.voltage_mv, batt_info.percentage);
    const char *battery_status = batt_str;
#else
    const char *battery_status = "disabled";
#endif

    // Build diagnosis response (static to avoid stack overflow)
    static char diag_buf[640];
    snprintf(diag_buf, sizeof(diag_buf),
             "{\"device\":\"self_diagnosis\","
             "\"firmware_version\":\"%s\","
             "\"hardware_id\":\"%012llX\","
             "\"milk_analyzer_status\":\"%s\","
             "\"weighing_machine_status\":\"%s\","
             "\"Printer_status\":\"%s\","
             "\"battery_status\":\"%s\"}",
             NCLE_FIRMWARE_VERSION,
             (unsigned long long)get_unique_id(),
             ma_status,
             wm_status,
             printer_status,
             battery_status);

    // Output to USB
    printf("%s\n", diag_buf);

    // Output to BLE
    if (s_output_callback) {
        s_output_callback(diag_buf, strlen(diag_buf));
    }
}
