/**
 * @file wifi_sta.h
 * @brief WiFi Station Mode - Public API
 *
 * Provides WiFi STA connectivity for the Master ESP32-S3 board.
 * Handles connection to a WiFi Access Point (router).
 * This is base infrastructure — HTTP/MQTT/OTA built on top by others.
 */

#ifndef WIFI_STA_H
#define WIFI_STA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum lengths */
#define WIFI_STA_SSID_MAX       32
#define WIFI_STA_PASSWORD_MAX   64
#define WIFI_STA_IP_MAX         16   /* "xxx.xxx.xxx.xxx\0" */

/* Maximum scan results */
#define WIFI_STA_SCAN_MAX       20

/* WiFi connection state */
typedef enum {
    WIFI_STA_STATE_IDLE = 0,        /* Not initialized or disconnected */
    WIFI_STA_STATE_CONNECTING,      /* Attempting to connect */
    WIFI_STA_STATE_CONNECTED,       /* Connected to AP, got IP */
    WIFI_STA_STATE_DISCONNECTED,    /* Was connected, now disconnected */
} wifi_sta_state_t;

/* WiFi status info */
typedef struct {
    wifi_sta_state_t state;
    char ssid[WIFI_STA_SSID_MAX + 1];
    char ip[WIFI_STA_IP_MAX];
    int8_t rssi;                    /* Signal strength in dBm */
    uint8_t bars;                   /* Signal bars 0-5 */
    bool connected;
} wifi_sta_status_t;

/* WiFi scan result entry */
typedef struct {
    char ssid[WIFI_STA_SSID_MAX + 1];
    int8_t rssi;
    uint8_t channel;
    bool secure;                    /* true if password protected */
} wifi_sta_scan_result_t;

/**
 * @brief Initialize WiFi in Station mode
 *
 * Sets up ESP-IDF WiFi driver, registers event handlers.
 * Must be called once before connect/scan.
 *
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_init(void);

/**
 * @brief Register wifi_config / wifi_status / wifi_erase. Call at boot.
 *
 * Deliberately independent of whether WiFi is running: the operator needs to
 * enter credentials while the device is still in gsm mode, or switching to
 * wifi could never succeed the first time. The handlers cope with the stack
 * being down - wifi_config stores without testing, wifi_status reports
 * whether credentials exist.
 */
void wifi_commands_register(void);

/**
 * @brief Register WiFi with net_link. Idempotent; call before starting MQTT.
 *
 * Separate from init() because init/deinit run repeatedly as the operator
 * switches link modes, while net_link registration must happen at most once -
 * the table has 2 slots and no unregister.
 */
void wifi_net_link_register(void);

/**
 * @brief Deinitialize WiFi
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_deinit(void);

/**
 * @brief Connect to a WiFi Access Point
 *
 * Starts connection attempt. Non-blocking — use wifi_sta_get_status()
 * or wifi_sta_is_connected() to check result.
 * Auto-retries up to 5 times on disconnect.
 *
 * @param ssid     SSID of the AP (max 32 chars)
 * @param password Password of the AP (max 64 chars, NULL for open network)
 * @return ESP_OK if connection attempt started
 */
esp_err_t ncle_wifi_sta_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from current AP
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_disconnect(void);

/**
 * @brief Scan for available WiFi networks (blocking)
 *
 * @param results   Array to fill with scan results
 * @param max       Maximum number of results (array size)
 * @param found     Output: number of networks found
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_scan(wifi_sta_scan_result_t *results, uint16_t max, uint16_t *found);

/**
 * @brief Get current WiFi status
 * @param status Output status structure
 */
void ncle_wifi_sta_get_status(wifi_sta_status_t *status);

/**
 * @brief Quick check if WiFi is connected
 * @return true if connected to AP with IP
 */
bool ncle_wifi_sta_is_connected(void);

/**
 * @brief Tell the link that saved credentials changed (saved or erased).
 *
 * net_link_t::is_provisioned caches "is an SSID saved" - it is asked once per
 * reading, and store-and-forward turns on it. Call this after wifi_config
 * succeeds or wifi_erase runs, so the next reading re-reads NVS instead of
 * buffering (or not) by a stale answer.
 */
void wifi_link_provisioned_changed(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_STA_H */
