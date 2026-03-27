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
#include "esp_err.h"
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
 * @brief Parse and process a JSON command string
 * @param json_str The JSON command string (null-terminated)
 * @param json_len Length of the JSON string
 *
 * Command format: {"command": "cmd_name", ...}
 * Commands are terminated with '#' character
 */
void parse_and_process_commands(char *json_str, int json_len);

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
