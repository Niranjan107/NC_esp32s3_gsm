/**
 * @file common.c
 * @brief Common utilities implementation for NCLite ESP32-S3
 *
 * PURPOSE:
 * Provides shared utility functions and global resources used across
 * all modules in the NCLite connector firmware.
 *
 * KEY FUNCTIONS:
 * - get_unique_id(): Returns device's unique hardware ID (from BT MAC)
 * - get_device_type_string(): Convert device type enum to string
 *
 * GLOBAL RESOURCES:
 * - cmd_queue: FreeRTOS queue for command messages (if used)
 * - response_queue: FreeRTOS queue for responses (if used)
 *
 * UNIQUE ID:
 * The device's unique ID is derived from the ESP32's Bluetooth MAC address
 * stored in eFuse (factory-programmed, cannot be changed).
 * Format: 48-bit MAC converted to 64-bit integer
 * Used by mobile app to identify and track individual devices.
 */

#include "common.h"
#include "esp_mac.h"
#include "esp_log.h"

static const char *TAG = "COMMON";

// Global queues (defined in main.c)
QueueHandle_t cmd_queue = NULL;
QueueHandle_t response_queue = NULL;

/**
 * @brief Initialize common module
 */
void common_init(void)
{
    ESP_LOGI(TAG, "Common module initialized");
}

/**
 * @brief Get string representation of device type
 */
const char* get_device_type_string(device_type_t type)
{
    switch (type) {
        case DEVICE_TYPE_MA:
            return "ma";
        case DEVICE_TYPE_WM:
            return "wm";
        case DEVICE_TYPE_PRINTER:
            return "printer";
        default:
            return "unknown";
    }
}

/**
 * @brief Get unique device ID (based on Bluetooth MAC address)
 *
 * PURPOSE:
 * Return a unique hardware identifier for this device.
 * Used by mobile app to identify and track individual connectors.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return 64-bit unique ID (48-bit MAC in lower 6 bytes)
 *
 * ID SOURCE:
 * Uses ESP32's Bluetooth MAC address from eFuse.
 * This is factory-programmed and unique per chip.
 *
 * EXAMPLE:
 * MAC: AA:BB:CC:DD:EE:FF → ID: 0x0000AABBCCDDEEFF
 * Displayed as: "AABBCCDDEEFF" (12 hex digits)
 */
uint64_t get_unique_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);

    uint64_t id = 0;
    for (int i = 0; i < 6; i++) {
        id = (id << 8) | mac[i];
    }
    return id;
}
