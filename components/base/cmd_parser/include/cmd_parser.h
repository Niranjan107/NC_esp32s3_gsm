/**
 * @file cmd_parser.h
 * @brief Command parser for processing JSON commands from app (USB/BLE)
 *
 * Command format: {"command": "<cmd>", "param1": "value1", ...}#
 * Response format: {"response_message": "<msg>", "status_code": <code>, "data": <optional>}
 */

#ifndef _CMD_PARSER_H_
#define _CMD_PARSER_H_

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Command String Constants (must match Pico2W exactly)
 ******************************************************************************/

// JSON key for command field
#define KEY_COMMAND             "command"

// Device configuration commands
#define CMD_RESET_CONFIG        "reset_config"
#define CMD_GET_CURRENT_CONFIG  "get_current_config"
#define CMD_MA_PORT_CONFIG      "ma_port_config"
#define CMD_WM_PORT_CONFIG      "wm_port_config"
#define CMD_PRINTER_PORT_CONFIG "printer_port_config"

// Printer commands
#define CMD_PRINT_RECEIPT       "print_receipt"
#define CMD_CHECK_PRINTER_STATUS "check_printer_status"
#define CMD_REPRINT_LAST_RECEIPT "reprint_last_receipt"

// Device info commands
#define CMD_GET_UNIQUE_ID       "ncle_get_unique_id"
#define CMD_GET_FIRMWARE_VERSION "get_firmware_version"
#define CMD_SELF_DIAGNOSIS      "self_diagnosis"
#define CMD_GET_BATTERY_STATUS  "get_battery_status"

// WiFi commands (Stage 1: BLE provisioning)
#define CMD_WIFI_CONFIG         "wifi_config"
#define CMD_WIFI_STATUS         "wifi_status"
#define CMD_WIFI_ERASE          "wifi_erase"

/* Which connectivity stack runs: gsm (default), wifi, or off. Registered by
 * the link_mode component, not handled here. */
#define CMD_SET_LINK_MODE       "set_link_mode"
#define CMD_GET_LINK_MODE       "get_link_mode"

// Diagnostics (on-demand): connectivity + counters + WM/MA/printer config
#define CMD_DIAG                "diag"

// Cycle timing instrumentation
#define CMD_GET_CYCLE_TIMING    "get_cycle_timing"
#define CMD_SET_TIMING_DEBUG    "set_timing_debug"

// BLE reading delivery (auto / always / app / off) - see device_config.h
// "app" also turns the flash buffer off and clears it.
#define CMD_SET_BLE_DATA        "set_ble_data"
#define CMD_GET_BLE_DATA        "get_ble_data"

// Store-and-forward on/off (off only for permanently app-only sites)
#define CMD_SET_STORE_FORWARD   "set_store_forward"

// Offline buffer (store-and-forward) storage
#define CMD_GET_STORAGE_INFO    "get_storage_info"
#define CMD_CLEAR_BUFFER        "clear_buffer"

// FOTA commands
#define CMD_FOTA_START          "fota_start"
#define CMD_FOTA_ROLLBACK       "fota_rollback"

// FOTA parameter keys
#define KEY_URL                 "url"

/*******************************************************************************
 * Config Parameter Keys
 ******************************************************************************/

#define KEY_MODEL               "model"
#define KEY_BAUD_RATE           "baud_rate"
#define KEY_DATA_BITS           "data_bits"
#define KEY_STOP_BITS           "stop_bits"
#define KEY_PARITY              "parity"
#define KEY_STREAM              "stream"
#define KEY_DATA                "data"
#define KEY_SSID                "ssid"
#define KEY_PASSWORD            "password"

/*******************************************************************************
 * Response Messages
 ******************************************************************************/

// Success messages
#define RESP_RESET_CONFIG_SUCCESS       "reset_config_success"
#define RESP_GET_CONFIG_SUCCESS         "get_current_config_success"
#define RESP_MA_CONFIG_SUCCESS          "ma_config_success"
#define RESP_WM_CONFIG_SUCCESS          "wm_config_success"
#define RESP_PRINTER_CONFIG_SUCCESS     "printer_config_success"
#define RESP_NCLITE_CONNECTED           "nclite_connected"

// Failure messages
#define RESP_MA_CONFIG_FAIL             "ma_config_fail"
#define RESP_WM_CONFIG_FAIL             "wm_config_fail"
#define RESP_PRINTER_CONFIG_FAIL        "printer_config_fail"
#define RESP_INVALID_JSON               "invalid_json_string"
#define RESP_COMMAND_NOT_FOUND          "command_not_found"
#define RESP_UNDEFINED_COMMAND          "undefined_command"

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Callback function type for sending responses (to BLE)
 * @param data Response data to send
 * @param len Length of data
 */
typedef void (*cmd_output_callback_t)(const char *data, size_t len);

/**
 * @brief Handler for a command registered by another component.
 *
 * @param root the parsed command object; the handler must NOT delete it (the
 *             parser owns it) and should reply with send_response().
 */
typedef void (*cmd_handler_t)(cJSON *root);

/**
 * @brief Where a command came from. Used to restrict risky commands to
 * local/on-site channels (BLE/console) and block them from the cloud (MQTT).
 */
typedef enum {
    CMD_SRC_BLE = 0,   /* local mobile app over BLE (trusted, on-site) */
    CMD_SRC_CONSOLE,   /* local USB console (trusted, on-site) */
    CMD_SRC_MQTT,      /* remote cloud/server over MQTT (restricted) */
} cmd_source_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize the command parser
 * @return ESP_OK on success
 */
esp_err_t cmd_parser_init(void);

/**
 * @brief Register output callback for BLE responses
 * @param callback Function to call when sending responses
 */
void cmd_parser_register_output_callback(cmd_output_callback_t callback);

/**
 * @brief Register a command owned by another component.
 *
 * Lets a component outside `base` add its own commands without this shared file
 * learning what they are - `wifi_sta` registers wifi_config/wifi_status/
 * wifi_erase this way, and the GSM product will register apn_config/sim_status
 * the same way, so the parser stays identical across products.
 *
 * Dispatch order: every BUILT-IN command is tried first, and only then the
 * registered ones. A registered command therefore can never shadow a base
 * command, whatever name it picks.
 *
 * INPUT:
 * @param command  command name; must be a string literal or otherwise outlive
 *                 the program (it is stored by pointer, not copied)
 * @param handler  called with the parsed JSON object when the command arrives
 *
 * OUTPUT:
 * @return true if registered. false - and logged - if the arguments are NULL,
 *         the name is already registered, or all CMD_REG_MAX slots are used.
 */
bool cmd_parser_register(const char *command, cmd_handler_t handler);

/**
 * @brief Parse and process a JSON command string
 * @param json_str The JSON command string (null-terminated)
 * @param json_len Length of the JSON string
 *
 * Command format: {"command": "cmd_name", ...}
 * Commands are terminated with '#' character
 *
 * @param source Where the command came from (CMD_SRC_BLE/CONSOLE/MQTT).
 *               Risky commands (wifi_config/wifi_erase/reset_config) are
 *               rejected when source == CMD_SRC_MQTT.
 */
void parse_and_process_commands(char *json_str, int json_len, cmd_source_t source);

/**
 * @brief Send a response back to the app
 * @param msg Response message
 * @param status_code Status code (0 = success, negative = error)
 * @param data Optional data (JSON formatted, can be NULL)
 */
void send_response(const char *msg, int status_code, const char *data);

/**
 * @brief Send a status message (alias for send_response)
 * @param msg Status message
 * @param status_code Status code
 * @param data Optional data
 */
void send_status(const char *msg, int status_code, const char *data);

#ifdef __cplusplus
}
#endif

#endif // _CMD_PARSER_H_
