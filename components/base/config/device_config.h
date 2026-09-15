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

// Where MA/WM readings are delivered over BLE. The cloud path is never
// affected by this setting - readings always go to MQTT (and to the flash
// buffer when the broker is down).
//
//   AUTO   - BLE readings only while MQTT is DOWN. Once the broker is
//            connected the app stops receiving readings, because the cloud
//            is already carrying them. This is the default.
//   ALWAYS - BLE readings always sent, i.e. both paths at once (pre-2.0.0.1005
//            behaviour). Kept for sites that need the app to display live.
//   OFF    - BLE readings never sent.
//
// Gated on the BROKER being connected, not on WiFi: a router with no internet
// gives connected=true/cloud=false, and there the app must keep receiving or
// the reading would be visible nowhere.
//
// Command replies are NOT affected - they always go over BLE, so the app can
// still provision WiFi and run diag on a device that is online.
#define BLE_DATA_AUTO               0
#define BLE_DATA_ALWAYS             1
#define BLE_DATA_OFF                2
#define DEFAULT_BLE_DATA_MODE       BLE_DATA_AUTO

// Store-and-forward: buffer every MA reading to flash and delete it only once
// the broker has acknowledged it.
//
// ON (default) - the reliability guarantee. A reading survives a WiFi outage,
//                a dead-looking link, and a power cut.
// OFF          - for a site that will NEVER have WiFi, where the mobile app is
//                the only delivery path. Buffering there just fills the ~2800
//                record buffer over a few weeks and then evicts in a loop.
//
// Turn this off ONLY for a permanently app-only site. At a site with
// intermittent WiFi it would discard readings during exactly the outages the
// buffer exists to cover.
#define DEFAULT_STORE_FORWARD       true

// Device configuration structure
typedef struct {
    uart_port_config_t ma_config;       // Milk Analyzer config
    uart_port_config_t wm_config;       // Weighing Machine config
    uart_port_config_t printer_config;  // Printer config
    char device_name[32];               // BLE device name
    uint8_t ble_data_mode;              // BLE_DATA_AUTO / _ALWAYS / _OFF
    bool store_forward;                 // buffer readings to flash (default on)
} device_config_t;

// Global device configuration (extern)
extern device_config_t g_device_config;

// Configuration functions
void config_load_defaults(device_config_t *config);
esp_err_t config_load_from_nvs(device_config_t *config);
esp_err_t config_save_to_nvs(const device_config_t *config);
esp_err_t config_reset_to_defaults(void);

// BLE reading delivery mode (BLE_DATA_AUTO / _ALWAYS / _OFF)
esp_err_t config_set_ble_data_mode(uint8_t mode);
uint8_t   config_get_ble_data_mode(void);
const char *config_ble_data_mode_name(uint8_t mode);

// Store-and-forward on/off (off only for permanently app-only sites)
esp_err_t config_set_store_forward(bool enable);
bool      config_get_store_forward(void);

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
