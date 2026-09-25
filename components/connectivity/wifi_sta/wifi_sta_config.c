/**
 * @file wifi_sta_config.c
 * @brief WiFi STA NVS Configuration - Save/Load/Erase credentials
 *
 * Persists WiFi credentials (SSID + password) to NVS flash.
 * Uses version checking to handle firmware upgrades gracefully
 * (same pattern as printer_uart, ma_uart, wm_uart).
 *
 * NVS Namespace: "wifi_cfg"
 * Keys: "version", "ssid", "password"
 */

#include "wifi_sta_config.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "WIFI_CFG";

#define NVS_NAMESPACE       "wifi_cfg"
#define NVS_KEY_VERSION     "version"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASSWORD    "password"
#define NVS_CONFIG_VERSION  1           /* Increment when config format changes */

/**
 * @brief Save WiFi credentials to NVS.
 *
 * INPUT:
 * @param ssid     - SSID string to save
 * @param password - password string to save (NULL for open network)
 *
 * OUTPUT:
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ssid is NULL, or NVS error
 */
esp_err_t ncle_wifi_sta_config_save(const char *ssid, const char *password)
{
    if (!ssid) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_i32(nvs, NVS_KEY_VERSION, NVS_CONFIG_VERSION);
    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASSWORD, password ? password : "");

    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Credentials saved: SSID=%s", ssid);
    } else {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * @brief Load WiFi credentials from NVS.
 *
 * INPUT:
 * @param ssid     - buffer to receive saved SSID (min WIFI_STA_SSID_MAX+1)
 * @param password - buffer to receive saved password (min WIFI_STA_PASSWORD_MAX+1)
 *
 * OUTPUT:
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no saved credentials
 */
esp_err_t ncle_wifi_sta_config_load(char *ssid, char *password)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not available: %s", esp_err_to_name(err));
        return err;
    }

    /* Check config version — erase if mismatch */
    int32_t version = 0;
    nvs_get_i32(nvs, NVS_KEY_VERSION, &version);
    if (version != NVS_CONFIG_VERSION) {
        ESP_LOGW(TAG, "NVS config version mismatch (%d != %d) — resetting",
                 (int)version, NVS_CONFIG_VERSION);
        nvs_erase_all(nvs);
        nvs_set_i32(nvs, NVS_KEY_VERSION, NVS_CONFIG_VERSION);
        nvs_commit(nvs);
        nvs_close(nvs);
        return ESP_ERR_NOT_FOUND;
    }

    /* Load SSID */
    size_t ssid_len = WIFI_STA_SSID_MAX + 1;
    err = nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_len);
    if (err != ESP_OK) {
        nvs_close(nvs);
        ESP_LOGW(TAG, "No saved SSID");
        return ESP_ERR_NOT_FOUND;
    }

    /* Load password */
    size_t pass_len = WIFI_STA_PASSWORD_MAX + 1;
    err = nvs_get_str(nvs, NVS_KEY_PASSWORD, password, &pass_len);
    if (err != ESP_OK) {
        password[0] = '\0';  /* Open network */
    }

    nvs_close(nvs);

    ESP_LOGI(TAG, "Credentials loaded: SSID=%s", ssid);
    return ESP_OK;
}

/**
 * @brief Erase all saved WiFi credentials from NVS.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success, or NVS error
 */
esp_err_t ncle_wifi_sta_config_erase(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    nvs_erase_all(nvs);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Credentials erased");
    return ESP_OK;
}

#define NVS_KEY_CONNECTED   "connected"

/**
 * @brief Persist the "successfully connected at least once" flag.
 *
 * INPUT:
 * @param connected - flag value to store
 *
 * OUTPUT:
 * @return ESP_OK on success, or NVS error
 */
esp_err_t ncle_wifi_sta_config_set_connected(bool connected)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    nvs_set_u8(nvs, NVS_KEY_CONNECTED, connected ? 1 : 0);
    err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

/**
 * @brief Read the persisted connected flag (false if never set).
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if the flag is set, false otherwise
 */
bool ncle_wifi_sta_config_get_connected(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    uint8_t v = 0;
    nvs_get_u8(nvs, NVS_KEY_CONNECTED, &v);
    nvs_close(nvs);
    return v != 0;
}
