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

/* Application layer, present from Step 4 of the GSM port onwards. Every use of
 * it below is already inside CONFIG_NCLE_MQTT_ENABLE, so the header is only
 * needed when that layer exists. */
#ifdef CONFIG_NCLE_MQTT_ENABLE
#include "fota.h"
#endif

/* No connectivity header here, deliberately. This file is shared by every CLV4
 * product, so it must not know whether the device has WiFi, GSM or neither: it
 * asks net_link (base/common) which link is up and lets that link describe
 * itself. The wifi_* commands live in the wifi_sta component and register
 * themselves; the only WiFi text left below is two command NAMES in the MQTT
 * restriction, which are compared as strings, not called. */
#include "net_link.h"

#ifdef CONFIG_NCLE_MQTT_ENABLE
#include "mqtt_client_svc.h"
#include "store_forward.h"   /* storage info + clear for the buffer commands */
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
 * Registered Commands
 * PURPOSE: commands owned by components OUTSIDE base, so this shared file never
 * learns what they are.
 *
 * WHY?
 * wifi_config/wifi_status/wifi_erase used to live in the dispatch chain below,
 * which meant this base component - supposedly identical in every CLV4 product -
 * knew about WiFi. The GSM product would then have added apn_config/sim_status
 * to the same chain and base would become a drawer holding every product's
 * commands. Now connectivity registers its own; base stays variant-free.
 *
 * Fixed table, no allocation. Registration happens once during startup, before
 * any command can arrive, so no locking is needed.
 *
 * Set by: cmd_parser_register(), called from each connectivity component's init
 * Used by: parse_and_process_commands(), AFTER every built-in command is tried
 */
#define CMD_REG_MAX 8

typedef struct {
    const char   *name;
    cmd_handler_t handler;
} cmd_reg_t;

static cmd_reg_t s_registry[CMD_REG_MAX];
static int       s_registry_count = 0;

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
static void process_diag(void);                   /* Handle diag command (connectivity + config) */

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

    // Populate g_device_config from NVS so get_current_config reports the
    // saved configuration after a reboot (defaults first, then any saved
    // values on top). The UART modules load their own NVS separately.
    config_load_defaults(&g_device_config);
    config_load_from_nvs(&g_device_config);

    // Print the loaded config to the console at boot so the current
    // configuration is visible without sending get_current_config.
    char cfg_json[RESPONSE_BUF_SIZE];
    config_get_json(cfg_json, sizeof(cfg_json));
    ESP_LOGI(TAG, "Loaded config: %s", cfg_json);

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
 * @brief Register a command owned by a component outside base. See cmd_parser.h.
 */
bool cmd_parser_register(const char *command, cmd_handler_t handler)
{
    if (command == NULL || handler == NULL) {
        ESP_LOGE(TAG, "register: NULL command or handler");
        return false;
    }
    if (s_registry_count >= CMD_REG_MAX) {
        ESP_LOGE(TAG, "register: table full, '%s' not registered", command);
        return false;
    }
    for (int i = 0; i < s_registry_count; i++) {
        if (strcmp(s_registry[i].name, command) == 0) {
            ESP_LOGW(TAG, "register: '%s' already registered", command);
            return false;
        }
    }

    s_registry[s_registry_count].name    = command;
    s_registry[s_registry_count].handler = handler;
    s_registry_count++;
    ESP_LOGI(TAG, "Registered command '%s'", command);
    return true;
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
void parse_and_process_commands(char *json_str, int json_len, cmd_source_t source)
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

    /* Restrict risky commands to local channels (BLE / USB console). These can
     * disconnect the device from its network or wipe its setup, so they must
     * NOT be triggerable remotely from the cloud (MQTT). Gated by a menuconfig
     * flag (NCLE_MQTT_RESTRICT_RISKY_CMDS, default on) so it can be toggled
     * without code changes. */
#ifdef CONFIG_NCLE_MQTT_RESTRICT_RISKY_CMDS
    if (source == CMD_SRC_MQTT &&
        (strcmp(cmd, CMD_WIFI_CONFIG) == 0 ||
         strcmp(cmd, CMD_WIFI_ERASE)  == 0 ||
         strcmp(cmd, CMD_RESET_CONFIG) == 0 ||
         /* set_ble_data decides whether the app sees readings at all. Allowing
          * it from the cloud would let one bad command blind every device in
          * the field, with no local way to notice. BLE/console only. */
         strcmp(cmd, CMD_SET_BLE_DATA) == 0 ||
         /* set_store_forward turns OFF the never-lose-a-reading guarantee. It
          * is a per-site installation decision, not a remote one. */
         strcmp(cmd, CMD_SET_STORE_FORWARD) == 0)) {
        ESP_LOGW(TAG, "Command '%s' blocked over MQTT (BLE/console only)", cmd);
        send_response("command_not_allowed_remotely", STATUS_ERR, NULL);
        cJSON_Delete(root);
        return;
    }
#endif

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
    /*-------------------------------------------------------------------------
     * NOTE: wifi_config / wifi_status / wifi_erase used to be handled here.
     * They now live in the wifi_sta component (connectivity layer), which
     * registers them via cmd_parser_register() at startup - see the registry
     * lookup in the final `else` below. This file is shared by every CLV4
     * product, so it must not know which transport a product happens to have.
     * The MQTT restriction on the risky two stays here: it matches on the
     * command NAME before dispatch, so it is unaffected by where they live.
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_DIAG) == 0) {
        process_diag();
    }
    else if (strcmp(cmd, CMD_PRINT_RECEIPT) == 0) {
#ifdef CONFIG_NCLE_PRINTER_ENABLE
        // Extract receipt data from JSON
        cJSON *data_item = cJSON_GetObjectItem(root, KEY_DATA);
        if (data_item == NULL || !cJSON_IsString(data_item)) {
            ESP_LOGE(TAG, "Missing 'data' field for print_receipt");
            send_response("print_receipt_fail", STATUS_ERR, NULL);
        } else {
            // T3 - the app's print data is back. mark_once so the FIRST chunk
            // of a multi-chunk receipt sets it, not the last.
            timing_mark_once(TIMING_T3_CMD_RX);
            const char *receipt_data = data_item->valuestring;
            ESP_LOGI(TAG, "Printing receipt (%d bytes)", (int)strlen(receipt_data));
            // Echo the exact receipt content to the console for debugging: if
            // this shows but the paper is blank, the data arrived fine and the
            // fault is the printer/hardware (not the WM/MA/cloud path).
            ESP_LOGI(TAG, "Receipt content >>>\n%s\n<<< end receipt", receipt_data);
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
    else if (strcmp(cmd, CMD_FOTA_START) == 0) {
/* FOTA lives in application/, which the GSM product gains in Step 4 - it is not
 * part of the OTA component, so CONFIG_NCLE_OTA_ENABLE alone is not enough to
 * know it exists. */
#if defined(CONFIG_NCLE_OTA_ENABLE) && defined(CONFIG_NCLE_MQTT_ENABLE)
        cJSON *url_item = cJSON_GetObjectItem(root, KEY_URL);
        if (url_item == NULL || !cJSON_IsString(url_item)) {
            send_response("fota_start", STATUS_ERR, "\"missing url\"");
        } else {
            const char *url = url_item->valuestring;
            esp_err_t err = fota_start(url);
            if (err == ESP_OK) {
                send_response("fota_start", STATUS_OK, "{\"state\":\"started\"}");
            } else if (err == ESP_ERR_INVALID_STATE) {
                send_response("fota_start", STATUS_ERR, "\"fota_start_fail_busy\"");
            } else {
                send_response("fota_start", STATUS_ERR, "\"start failed\"");
            }
        }
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    /*-------------------------------------------------------------------------
     * Offline buffer storage
     *
     * JSON: {"command":"get_storage_info"}#
     *       {"command":"clear_buffer","confirm":true}#
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_GET_STORAGE_INFO) == 0) {
#ifdef CONFIG_NCLE_MQTT_ENABLE
        sf_info_t si;
        sf_get_info(&si);
        static char storage_data[256];
        snprintf(storage_data, sizeof(storage_data),
                 "{\"total_bytes\":%lu,\"used_bytes\":%lu,\"free_bytes\":%lu,"
                 "\"used_pct\":%u,\"records\":%lu,\"capacity_left\":%lu,\"evicted\":%lu}",
                 (unsigned long)si.total_bytes, (unsigned long)si.used_bytes,
                 (unsigned long)si.free_bytes, (unsigned)si.used_pct,
                 (unsigned long)si.records, (unsigned long)si.capacity_left,
                 (unsigned long)si.evicted);
        send_response("get_storage_info", STATUS_OK, storage_data);
#else
        send_response("get_storage_info", STATUS_ERR, "\"mqtt_not_enabled\"");
#endif
    }
    else if (strcmp(cmd, CMD_CLEAR_BUFFER) == 0) {
#ifdef CONFIG_NCLE_MQTT_ENABLE
        /* Destructive: these are milk readings the server has never received.
         * The explicit confirm flag means a stray or mis-tapped message cannot
         * wipe a collection centre's unsent data. */
        cJSON *confirm = cJSON_GetObjectItem(root, "confirm");
        if (confirm == NULL || !cJSON_IsTrue(confirm)) {
            send_response("clear_buffer", STATUS_ERR, "\"confirm_required\"");
        } else {
            uint32_t deleted = sf_clear_all();
            static char cleared[64];
            snprintf(cleared, sizeof(cleared), "{\"deleted\":%lu}",
                     (unsigned long)deleted);
            send_response("clear_buffer", STATUS_OK, cleared);
        }
#else
        send_response("clear_buffer", STATUS_ERR, "\"mqtt_not_enabled\"");
#endif
    }
    /*-------------------------------------------------------------------------
     * Cycle Timing Instrumentation
     * Reports how long one collection cycle took and where the time went.
     *
     * JSON: {"command":"get_cycle_timing"}#
     *       {"command":"set_timing_debug","enable":1}#
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_GET_CYCLE_TIMING) == 0) {
        int32_t t1 = timing_get_ms(TIMING_T1_MA_DONE);
        int32_t t2 = timing_get_ms(TIMING_T2_APP_SENT);
        int32_t t3 = timing_get_ms(TIMING_T3_CMD_RX);
        int32_t t4 = timing_get_ms(TIMING_T4_PRINT_FIRST);
        int32_t t5 = timing_get_ms(TIMING_T5_PRINT_DONE);
        static char timing_data[288];
        snprintf(timing_data, sizeof(timing_data),
                 "{\"t0\":%ld,\"t1\":%ld,\"t2\":%ld,\"t3\":%ld,\"t4\":%ld,\"t5\":%ld,"
                 "\"wm\":%ld,"
                 "\"ma_rx\":%ld,\"app_gap\":%ld,\"print\":%ld,\"total\":%ld}",
                 (long)timing_get_ms(TIMING_T0_MA_FIRST),
                 (long)t1, (long)t2, (long)t3, (long)t4, (long)t5,
                 (long)timing_get_ms(TIMING_WM_RX),
                 (long)t1,                                              // T1-T0
                 (long)((t3 >= 0 && t2 >= 0) ? (t3 - t2) : -1),         // T3-T2
                 (long)((t5 >= 0 && t4 >= 0) ? (t5 - t4) : -1),         // T5-T4
                 (long)t5);                                             // T5-T0
        send_response("get_cycle_timing", STATUS_OK, timing_data);
    }
    else if (strcmp(cmd, CMD_SET_TIMING_DEBUG) == 0) {
        cJSON *enable = cJSON_GetObjectItem(root, "enable");
        if (enable == NULL) {
            send_response("set_timing_debug", STATUS_ERR, "\"missing enable\"");
        } else {
            bool on = (enable->valueint != 0);
            timing_set_receipt_print(on);
            send_response("set_timing_debug", STATUS_OK,
                          on ? "{\"receipt_timing\":true}" : "{\"receipt_timing\":false}");
        }
    }
    /*-------------------------------------------------------------------------
     * BLE reading delivery - ONE command controls where readings go.
     * Readings always go to MQTT; this decides the BLE copy and, for "app",
     * whether they are buffered to flash as well.
     *
     *   auto   - BLE readings only while the broker is DOWN (default). Once
     *            the cloud is carrying them the app stops receiving, so the
     *            server is not sent the same reading twice. Buffer ON.
     *   always - both paths at once (behaviour before 2.0.0.1005). Buffer ON.
     *   app    - BLE always, and the flash buffer OFF. For a site where the
     *            mobile app is the delivery path: buffering there would fill
     *            the ~2800-record buffer over a few weeks and then evict in a
     *            loop, because nothing on this device will ever deliver it.
     *   off    - never over BLE. Buffer ON.
     *
     * "app" is the only mode that touches the buffer, and it is why the buffer
     * is set here rather than left to a second command: the two are wanted as a
     * pair, and setting one without the other fails silently.
     *
     * Command replies are unaffected and always go over BLE.
     *
     * JSON: {"command":"set_ble_data","mode":"app"}#
     *       {"command":"get_ble_data"}#
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_SET_BLE_DATA) == 0) {
        cJSON *mode = cJSON_GetObjectItem(root, "mode");
        if (!cJSON_IsString(mode)) {
            send_response("set_ble_data", STATUS_ERR, "\"missing mode\"");
        } else {
            uint8_t m;
            bool buffer_on = true;      /* only "app" turns the buffer off */
            if      (strcmp(mode->valuestring, "auto")   == 0) m = BLE_DATA_AUTO;
            else if (strcmp(mode->valuestring, "always") == 0) m = BLE_DATA_ALWAYS;
            else if (strcmp(mode->valuestring, "off")    == 0) m = BLE_DATA_OFF;
            else if (strcmp(mode->valuestring, "app")    == 0) {
                m = BLE_DATA_ALWAYS;
                buffer_on = false;
            }
            else {
                send_response("set_ble_data", STATUS_ERR, "\"invalid mode\"");
                goto ble_data_done;
            }
            config_set_store_forward(buffer_on);

            /* Turning the buffer off strands whatever is already in it: with
             * no buffering there is no flush loop pass that will ever clear
             * those files, so they would sit in flash forever. Delete the
             * DATA only - meter/printer/WiFi configuration is untouched. */
            unsigned long cleared = 0;
#ifdef CONFIG_NCLE_MQTT_ENABLE
            if (!buffer_on) {
                cleared = (unsigned long)sf_clear_all();
                ESP_LOGW(TAG, "Buffer disabled -> cleared %lu stranded reading(s)",
                         cleared);
            }
#endif

            if (config_set_ble_data_mode(m) == ESP_OK) {
                static char bd[104];
                snprintf(bd, sizeof(bd),
                         "{\"mode\":\"%s\",\"store_forward\":%s,\"cleared\":%lu}",
                         mode->valuestring, buffer_on ? "true" : "false", cleared);
                ESP_LOGW(TAG, "BLE reading delivery '%s' (buffer %s)",
                         mode->valuestring, buffer_on ? "on" : "OFF");
                send_response("set_ble_data_success", STATUS_OK, bd);
            } else {
                send_response("set_ble_data", STATUS_ERR, "\"save failed\"");
            }
        }
    ble_data_done: ;
    }
    /*-------------------------------------------------------------------------
     * Store-and-forward on/off
     * OFF only for a site that will never have WiFi, where the mobile app is
     * the delivery path. Buffering there fills the ~2800-record buffer over a
     * few weeks and then evicts in a loop for no benefit.
     *
     * Leave it ON anywhere WiFi exists, even intermittently - it is what makes
     * a reading survive an outage.
     *
     * JSON: {"command":"set_store_forward","enable":0}#
     *-----------------------------------------------------------------------*/
    else if (strcmp(cmd, CMD_SET_STORE_FORWARD) == 0) {
        cJSON *enable = cJSON_GetObjectItem(root, "enable");
        if (enable == NULL) {
            send_response("set_store_forward", STATUS_ERR, "\"missing enable\"");
        } else {
            bool on = cJSON_IsBool(enable) ? cJSON_IsTrue(enable)
                                           : (enable->valueint != 0);
            if (config_set_store_forward(on) == ESP_OK) {
                ESP_LOGW(TAG, "Store-and-forward %s", on ? "ENABLED" : "DISABLED");
                send_response("set_store_forward_success", STATUS_OK,
                              on ? "{\"store_forward\":true}"
                                 : "{\"store_forward\":false}");
            } else {
                send_response("set_store_forward", STATUS_ERR, "\"save failed\"");
            }
        }
    }
    else if (strcmp(cmd, CMD_GET_BLE_DATA) == 0) {
        /* Report "app" rather than "always" when the buffer is also off, so
         * what comes back matches what was set. */
        uint8_t bd_m = config_get_ble_data_mode();
        bool    sf_o = config_get_store_forward();
        const char *name = (bd_m == BLE_DATA_ALWAYS && !sf_o)
                               ? "app" : config_ble_data_mode_name(bd_m);
        static char bd[80];
        snprintf(bd, sizeof(bd), "{\"mode\":\"%s\",\"store_forward\":%s}",
                 name, sf_o ? "true" : "false");
        send_response("get_ble_data", STATUS_OK, bd);
    }
    else if (strcmp(cmd, CMD_FOTA_ROLLBACK) == 0) {
/* Same as fota_start above: FOTA arrives with application/ in Step 4. */
#if defined(CONFIG_NCLE_OTA_ENABLE) && defined(CONFIG_NCLE_MQTT_ENABLE)
        esp_err_t err = fota_rollback();
        if (err == ESP_OK) {
            send_response("fota_rollback", STATUS_OK, "{\"state\":\"rolling_back\"}");
        } else if (err == ESP_ERR_NOT_FOUND) {
            send_response("fota_rollback", STATUS_ERR, "\"no_previous_firmware\"");
        } else {
            send_response("fota_rollback", STATUS_ERR, "\"rollback_failed\"");
        }
#else
        send_response("ota_not_enabled", STATUS_ERR, NULL);
#endif
    }
    else {
        /* Not a built-in command. Before giving up, check the commands other
         * components registered (wifi_sta's wifi_* today, the GSM component's
         * own later). Deliberately LAST: every built-in branch above has
         * already been tried, so a registered command can never shadow a base
         * command whatever name it picks. */
        bool handled = false;
        for (int i = 0; i < s_registry_count; i++) {
            if (strcmp(cmd, s_registry[i].name) == 0) {
                s_registry[i].handler(root);
                handled = true;
                break;
            }
        }
        if (!handled) {
            ESP_LOGW(TAG, "Unknown command: %s", cmd);
            send_response(RESP_UNDEFINED_COMMAND, STATUS_ERR, NULL);
        }
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

    // Persist to the WM module's own NVS so the config survives reboot
    // (wm_uart loads from this NVS on boot).
    wm_uart_save_config();

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

    // Persist to the MA module's own NVS so the config survives reboot.
    ma_uart_save_config();

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
    // Persist to the printer module's own NVS so the config survives reboot.
    printer_uart_save_config();
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

    // Report the WM/MA/printer defaults (model/baud/parity/etc.) so the user
    // sees exactly what was reset and knows what to reconfigure. device_name is
    // intentionally omitted (unused field); WiFi credentials are NOT affected.
    static char cfg_json[RESPONSE_BUF_SIZE];
    snprintf(cfg_json, sizeof(cfg_json),
        "{\"ma\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d,\"stream\":%d},"
        "\"wm\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d,\"stream\":%d},"
        "\"printer\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d}}",
        g_device_config.ma_config.model, (unsigned long)g_device_config.ma_config.baud_rate,
        g_device_config.ma_config.data_bits, g_device_config.ma_config.stop_bits,
        g_device_config.ma_config.parity, g_device_config.ma_config.stream_mode ? 1 : 0,
        g_device_config.wm_config.model, (unsigned long)g_device_config.wm_config.baud_rate,
        g_device_config.wm_config.data_bits, g_device_config.wm_config.stop_bits,
        g_device_config.wm_config.parity, g_device_config.wm_config.stream_mode ? 1 : 0,
        g_device_config.printer_config.model, (unsigned long)g_device_config.printer_config.baud_rate,
        g_device_config.printer_config.data_bits, g_device_config.printer_config.stop_bits,
        g_device_config.printer_config.parity);
    send_response(RESP_RESET_CONFIG_SUCCESS, STATUS_OK, cfg_json);
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

/**
 * @brief Process "diag" command - on-demand connectivity + config snapshot.
 *
 * Read-only. Gathers the active network link's own status, MQTT broker state +
 * publish/drop counters, and the current WM/MA/printer serial config (incl.
 * parity), then sends ONE JSON snapshot to BLE + USB console. Touches no data
 * path, so it works even if MQTT is down (it just reports mqtt.connected=false).
 *
 * Reply (WiFi product):
 * {"device":"diag","device_id":"d0cf1319a352","link":"wifi",
 *  "wifi":{"connected":true,"ssid":"GormalOne","ip":"192.168.1.244","rssi":-61},
 *  "mqtt":{"connected":true,"broker":"wss://...","published":5,"dropped":0,...},
 *  "config":{"ma":{...},"wm":{...},"printer":{...},"device_name":""}}
 *
 * "link" names the connectivity object, so the GSM product returns the same
 * shape with "link":"gsm" and a "gsm":{...} object instead - no change here.
 * With no link up it reads "link":"none" and the object is {}.
 */
static void process_diag(void)
{
    // Current WM/MA/printer config (incl. parity) as a JSON object.
    static char cfg_json[RESPONSE_BUF_SIZE];
    config_get_json(cfg_json, sizeof(cfg_json));

    // Connectivity status, asked FROM the active link rather than read out of a
    // transport we assume is there. On this product that is wifi_sta reporting
    // ssid/ip/rssi; on the GSM product it will be the gsm component reporting
    // SIM and signal - and this function does not change. "none" when no link is
    // up, "unconfigured" when the build has no connectivity layer at all.
    const char *link_name = net_link_active();
    static char link_json[160];
    net_link_status_json(link_json, sizeof(link_json));

    // MQTT status + publish/drop counters
    static char mqtt_json[320];
#ifdef CONFIG_NCLE_MQTT_ENABLE
    mqtt_svc_stats_t mst;
    mqtt_svc_get_stats(&mst);
    sf_info_t sfi;
    sf_get_info(&sfi);
    snprintf(mqtt_json, sizeof(mqtt_json),
             "{\"connected\":%s,\"broker\":\"%s\",\"published\":%lu,\"dropped\":%lu,"
             "\"deduped\":%lu,\"buffered\":%lu,\"buffer_used_pct\":%u,\"evicted\":%lu}",
             mst.connected ? "true" : "false", mqtt_svc_broker_uri(),
             (unsigned long)mst.published, (unsigned long)mst.dropped,
             (unsigned long)mst.deduped, (unsigned long)mst.buffered,
             (unsigned)sfi.used_pct, (unsigned long)sfi.evicted);
#else
    snprintf(mqtt_json, sizeof(mqtt_json), "{\"enabled\":false}");
#endif

    // Assemble the full snapshot (static to avoid stack overflow).
    // The connectivity object is keyed by the link's own NAME - "wifi" here,
    // "gsm" on the GSM product - and "link" says which name to look for. On this
    // WiFi product the reply is therefore unchanged apart from the added "link"
    // field, so existing readers of diag.wifi keep working.
    // "ble_data" is the configured mode, not the momentary state: with "auto"
    // the app stops receiving readings as soon as the broker connects. Without
    // it, "the app shows nothing" is indistinguishable from a fault.
    // "always" with the buffer off is the "app" mode - report it as it was set.
    uint8_t bd_mode = config_get_ble_data_mode();
    bool    sf_on   = config_get_store_forward();
    const char *bd_name = (bd_mode == BLE_DATA_ALWAYS && !sf_on)
                              ? "app" : config_ble_data_mode_name(bd_mode);
    static char diag_buf[RESPONSE_BUF_SIZE + 512];
    int len = snprintf(diag_buf, sizeof(diag_buf),
             "{\"device\":\"diag\",\"device_id\":\"%012llx\",\"link\":\"%s\","
             "\"%s\":%s,\"mqtt\":%s,\"ble_data\":\"%s\","
             "\"store_forward\":%s,\"config\":%s}\n",
             (unsigned long long)get_unique_id(), link_name,
             link_name, link_json, mqtt_json, bd_name,
             sf_on ? "true" : "false", cfg_json);

    // Output to USB console
    printf("%s", diag_buf);

    // Output to BLE (if callback registered)
    if (s_output_callback) {
        s_output_callback(diag_buf, len);
    }
}
