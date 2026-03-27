/**
 * @file device_config.h
 * @brief Device configuration structures and NVS storage
 */

#ifndef _DEVICE_CONFIG_H_
#define _DEVICE_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Device model definitions (from original project)

// Milk Analyzer Models
#define MA_MODEL_UNKNOWN            -1
#define MA_MODEL_EKOMILK_BOND       1001
#define MA_MODEL_EKOMILK_ULTRA_DPS  1002
#define MA_MODEL_ORIGHT_EKOMILK     1003
#define MA_MODEL_EKOMILK_ULTRA_V1   1004
#define MA_MODEL_ULTRASCAN          2001
#define MA_MODEL_REIL_EMT           2002
#define MA_MODEL_LACTOSURE_ECO      2003
#define MA_MODEL_ESSAE              2004
#define MA_MODEL_AKASHGANGA         3001
#define MA_MODEL_EKOMILK_ULTRA_PRO  3002
#define MA_MODEL_INDIZ_SMART        3003
#define MA_MODEL_PROMPT_ISMART      3004
#define MA_MODEL_PROMPT_FATOMATIC   4001
#define MA_MODEL_INDIZ_MILKOFAT     4002
#define MA_MODEL_FATOCARE_SOLAR     4003
#define MA_MODEL_STIPL              5001

// Weighing Machine Models
#define WM_MODEL_UNKNOWN            -1
#define WM_MODEL_TYPE_1_KG          9001
#define WM_MODEL_TYPE_2_LT          9002
#define WM_MODEL_TYPE_3_LT          9003
#define WM_MODEL_TYPE_4_KG          9004
#define WM_MODEL_TYPE_4_LT          9005
#define WM_MODEL_TYPE_6             9006    // Dollar prefix format ($L000.88)
#define WM_MODEL_CONT               9000

// Printer Models
#define PRINTER_MODEL_UNKNOWN       -1
#define PRINTER_MODEL_GENERIC       8000

// Default configuration values
#define DEFAULT_BAUD_RATE           9600
#define DEFAULT_DATA_BITS           DATA_BITS_8
#define DEFAULT_STOP_BITS           STOP_BITS_1
#define DEFAULT_PARITY              PARITY_NONE

// Device configuration structure
typedef struct {
    uart_port_config_t ma_config;       // Milk Analyzer config
    uart_port_config_t wm_config;       // Weighing Machine config
    uart_port_config_t printer_config;  // Printer config
    char device_name[32];               // BLE device name
} device_config_t;

// Global device configuration (extern)
extern device_config_t g_device_config;

// Configuration functions
void config_load_defaults(device_config_t *config);
esp_err_t config_load_from_nvs(device_config_t *config);
esp_err_t config_save_to_nvs(const device_config_t *config);
esp_err_t config_reset_to_defaults(void);

// Individual port configuration
esp_err_t config_set_ma_port(const uart_port_config_t *config);
esp_err_t config_set_wm_port(const uart_port_config_t *config);
esp_err_t config_set_printer_port(const uart_port_config_t *config);

// Get configuration JSON
void config_get_json(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // _DEVICE_CONFIG_H_
