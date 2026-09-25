/**
 * @file wifi_sta_config.h
 * @brief WiFi STA NVS Configuration - Credential persistence
 *
 * Saves/loads WiFi credentials (SSID + password) to NVS flash.
 * Uses version checking to handle firmware upgrades (same pattern
 * as printer_uart, ma_uart, wm_uart).
 */

#ifndef WIFI_STA_CONFIG_H
#define WIFI_STA_CONFIG_H

#include "esp_err.h"
#include "wifi_sta.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Save WiFi credentials to NVS
 *
 * @param ssid     SSID to save
 * @param password Password to save (empty string for open network)
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_config_save(const char *ssid, const char *password);

/**
 * @brief Load WiFi credentials from NVS
 *
 * @param ssid     Buffer to receive SSID (min WIFI_STA_SSID_MAX+1)
 * @param password Buffer to receive password (min WIFI_STA_PASSWORD_MAX+1)
 * @return ESP_OK if credentials found, ESP_ERR_NOT_FOUND if empty
 */
esp_err_t ncle_wifi_sta_config_load(char *ssid, char *password);

/**
 * @brief Erase saved WiFi credentials from NVS
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_config_erase(void);

/**
 * @brief Persist a flag indicating WiFi has connected successfully at least once.
 * @param connected flag value to store
 * @return ESP_OK on success
 */
esp_err_t ncle_wifi_sta_config_set_connected(bool connected);

/**
 * @brief Read the persisted connected flag.
 * @return true if set, false otherwise
 */
bool ncle_wifi_sta_config_get_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_STA_CONFIG_H */
