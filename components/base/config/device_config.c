/**
 * @file device_config.c
 * @brief Device configuration and NVS storage implementation
 *
 * PURPOSE:
 * This module manages persistent configuration for all device peripherals
 * (MA, WM, Printer). Configuration is stored in ESP32's NVS (Non-Volatile Storage)
 * and survives power cycles.
 *
 * WHY NVS?
 * - Flash-based key-value storage built into ESP-IDF
 * - Wear-leveling for flash longevity
 * - Survives firmware updates
 * - Simple API for reading/writing settings
 *
 * STORED CONFIGURATIONS:
 * - MA config: baud_rate, data_bits, stop_bits, parity, model, stream_mode
 * - WM config: baud_rate, data_bits, stop_bits, parity, model, stream_mode
 * - Printer config: baud_rate, data_bits, stop_bits, parity, model
 * - Device name: BLE advertised name
 *
 * GLOBAL CONFIG VARIABLE:
 * g_device_config - Global structure holding all current configuration
 * - Loaded from NVS on startup
 * - Modified by JSON commands from mobile app
 * - Saved back to NVS when changed
 *
 * USAGE:
 * 1. On startup: config_load_defaults() → config_load_from_nvs()
 * 2. App sends wm_port_config → cmd_parser updates g_device_config
 * 3. config_set_wm_port() saves to NVS
 * 4. On next boot, config is restored from NVS
 */

#include "device_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CONFIG";
static const char *NVS_NAMESPACE = "ncle_config";

// NVS keys
static const char *KEY_MA_CONFIG = "ma_cfg";
static const char *KEY_WM_CONFIG = "wm_cfg";
static const char *KEY_PRINTER_CONFIG = "printer_cfg";
static const char *KEY_DEVICE_NAME = "dev_name";
static const char *KEY_BLE_DATA_MODE = "ble_data";
static const char *KEY_STORE_FORWARD = "sf_on";

/**
 * Global device configuration structure
 * PURPOSE: Single source of truth for all device settings
 * Accessible from any module via extern declaration in device_config.h
 */
device_config_t g_device_config;

/**
 * @brief Load default configuration values
 *
 * PURPOSE:
 * Initialize configuration structure with factory defaults.
 * Called on first boot or when resetting to defaults.
 *
 * INPUT:
 * @param config - Pointer to configuration structure to populate
 *
 * OUTPUT:
 * Fills config structure with default values
 *
 * DEFAULT VALUES:
 * - All ports: 9600 baud, 8N1, stream=false
 * - MA model: MA_MODEL_UNKNOWN
 * - WM model: WM_MODEL_UNKNOWN
 * - Printer model: PRINTER_MODEL_UNKNOWN (-1)
 * - Device name: "Nitara BLE"
 */
void config_load_defaults(device_config_t *config)
{
    // Milk Analyzer defaults
    config->ma_config.baud_rate = DEFAULT_BAUD_RATE;
    config->ma_config.data_bits = DEFAULT_DATA_BITS;
    config->ma_config.stop_bits = DEFAULT_STOP_BITS;
    config->ma_config.parity = DEFAULT_PARITY;
    config->ma_config.stream_mode = false;
    config->ma_config.model = MA_MODEL_UNKNOWN;

    // Weighing Machine defaults
    config->wm_config.baud_rate = DEFAULT_BAUD_RATE;
    config->wm_config.data_bits = DEFAULT_DATA_BITS;
    config->wm_config.stop_bits = DEFAULT_STOP_BITS;
    config->wm_config.parity = DEFAULT_PARITY;
    config->wm_config.stream_mode = false;
    config->wm_config.model = WM_MODEL_UNKNOWN;

    // Printer defaults
    config->printer_config.baud_rate = DEFAULT_BAUD_RATE;
    config->printer_config.data_bits = DEFAULT_DATA_BITS;
    config->printer_config.stop_bits = DEFAULT_STOP_BITS;
    config->printer_config.parity = DEFAULT_PARITY;
    config->printer_config.stream_mode = false;
    config->printer_config.model = PRINTER_MODEL_UNKNOWN;  // -1 like Pico2W

    // Device name
    strncpy(config->device_name, "Nitara BLE", sizeof(config->device_name) - 1);

    // BLE reading delivery: AUTO = app stops receiving once the broker is up
    config->ble_data_mode = DEFAULT_BLE_DATA_MODE;

    // Store-and-forward on: never lose a reading to a network outage
    config->store_forward = DEFAULT_STORE_FORWARD;

    ESP_LOGI(TAG, "Default configuration loaded");
}

/**
 * @brief Load configuration from NVS
 */
esp_err_t config_load_from_nvs(device_config_t *config)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found, using defaults");
        return err;
    }

    size_t len;

    // Load MA config
    len = sizeof(uart_port_config_t);
    err = nvs_get_blob(handle, KEY_MA_CONFIG, &config->ma_config, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MA config not found in NVS");
    }

    // Load WM config
    len = sizeof(uart_port_config_t);
    err = nvs_get_blob(handle, KEY_WM_CONFIG, &config->wm_config, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WM config not found in NVS");
    }

    // Load Printer config
    len = sizeof(uart_port_config_t);
    err = nvs_get_blob(handle, KEY_PRINTER_CONFIG, &config->printer_config, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Printer config not found in NVS");
    }

    // Load device name
    len = sizeof(config->device_name);
    err = nvs_get_str(handle, KEY_DEVICE_NAME, config->device_name, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device name not found in NVS");
    }

    // Load BLE reading delivery mode. Absent on a device flashed before
    // 2.0.0.1005, so keep the default the caller already loaded rather than
    // leaving the field uninitialised.
    uint8_t mode;
    err = nvs_get_u8(handle, KEY_BLE_DATA_MODE, &mode);
    if (err == ESP_OK && mode <= BLE_DATA_OFF) {
        config->ble_data_mode = mode;
    } else {
        config->ble_data_mode = DEFAULT_BLE_DATA_MODE;
    }

    // Store-and-forward. Absent on a device flashed before 2.0.0.1005 -> ON,
    // so an upgrade never silently drops the reliability guarantee.
    uint8_t sf_on;
    err = nvs_get_u8(handle, KEY_STORE_FORWARD, &sf_on);
    config->store_forward = (err == ESP_OK) ? (sf_on != 0) : DEFAULT_STORE_FORWARD;

    nvs_close(handle);
    ESP_LOGI(TAG, "Configuration loaded from NVS");
    return ESP_OK;
}

/**
 * @brief Save configuration to NVS
 */
esp_err_t config_save_to_nvs(const device_config_t *config)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        return err;
    }

    // Save MA config
    err = nvs_set_blob(handle, KEY_MA_CONFIG, &config->ma_config, sizeof(uart_port_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save MA config");
    }

    // Save WM config
    err = nvs_set_blob(handle, KEY_WM_CONFIG, &config->wm_config, sizeof(uart_port_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WM config");
    }

    // Save Printer config
    err = nvs_set_blob(handle, KEY_PRINTER_CONFIG, &config->printer_config, sizeof(uart_port_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Printer config");
    }

    // Save device name
    err = nvs_set_str(handle, KEY_DEVICE_NAME, config->device_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save device name");
    }

    // Save BLE reading delivery mode
    err = nvs_set_u8(handle, KEY_BLE_DATA_MODE, config->ble_data_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save BLE data mode");
    }

    // Save store-and-forward flag
    err = nvs_set_u8(handle, KEY_STORE_FORWARD, config->store_forward ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save store-forward flag");
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Configuration saved to NVS");
    return err;
}

/**
 * @brief Reset configuration to defaults
 */
esp_err_t config_reset_to_defaults(void)
{
    config_load_defaults(&g_device_config);
    return config_save_to_nvs(&g_device_config);
}

/**
 * @brief Set MA port configuration
 */
esp_err_t config_set_ma_port(const uart_port_config_t *config)
{
    memcpy(&g_device_config.ma_config, config, sizeof(uart_port_config_t));
    return config_save_to_nvs(&g_device_config);
}

/**
 * @brief Set where MA/WM readings are delivered over BLE.
 *
 * Only affects the BLE copy. Readings always go to MQTT, and to the flash
 * buffer while the broker is down, whatever this is set to.
 */
esp_err_t config_set_ble_data_mode(uint8_t mode)
{
    if (mode > BLE_DATA_OFF) {
        return ESP_ERR_INVALID_ARG;
    }
    g_device_config.ble_data_mode = mode;
    return config_save_to_nvs(&g_device_config);
}

uint8_t config_get_ble_data_mode(void)
{
    return g_device_config.ble_data_mode;
}

/**
 * @brief Turn the flash buffer on or off.
 *
 * OFF is for a site that will never have WiFi, where the app is the only
 * delivery path. Everywhere else this must stay ON - it is what makes a
 * reading survive an outage.
 */
esp_err_t config_set_store_forward(bool enable)
{
    g_device_config.store_forward = enable;
    return config_save_to_nvs(&g_device_config);
}

bool config_get_store_forward(void)
{
    return g_device_config.store_forward;
}

const char *config_ble_data_mode_name(uint8_t mode)
{
    switch (mode) {
        case BLE_DATA_AUTO:   return "auto";
        case BLE_DATA_ALWAYS: return "always";
        case BLE_DATA_OFF:    return "off";
        default:              return "unknown";
    }
}

/**
 * @brief Set WM port configuration
 */
esp_err_t config_set_wm_port(const uart_port_config_t *config)
{
    memcpy(&g_device_config.wm_config, config, sizeof(uart_port_config_t));
    return config_save_to_nvs(&g_device_config);
}

/**
 * @brief Set Printer port configuration
 */
esp_err_t config_set_printer_port(const uart_port_config_t *config)
{
    memcpy(&g_device_config.printer_config, config, sizeof(uart_port_config_t));
    return config_save_to_nvs(&g_device_config);
}

/**
 * @brief Get configuration as JSON string
 */
void config_get_json(char *buffer, size_t buffer_size)
{
    snprintf(buffer, buffer_size,
        "{"
        "\"ma\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d,\"stream\":%s},"
        "\"wm\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d,\"stream\":%s},"
        "\"printer\":{\"model\":%d,\"baud_rate\":%lu,\"data_bits\":%d,\"stop_bits\":%d,\"parity\":%d},"
        "\"device_name\":\"%s\""
        "}",
        g_device_config.ma_config.model,
        g_device_config.ma_config.baud_rate,
        g_device_config.ma_config.data_bits,
        g_device_config.ma_config.stop_bits,
        g_device_config.ma_config.parity,
        g_device_config.ma_config.stream_mode ? "true" : "false",
        g_device_config.wm_config.model,
        g_device_config.wm_config.baud_rate,
        g_device_config.wm_config.data_bits,
        g_device_config.wm_config.stop_bits,
        g_device_config.wm_config.parity,
        g_device_config.wm_config.stream_mode ? "true" : "false",
        g_device_config.printer_config.model,
        g_device_config.printer_config.baud_rate,
        g_device_config.printer_config.data_bits,
        g_device_config.printer_config.stop_bits,
        g_device_config.printer_config.parity,
        g_device_config.device_name
    );
}
