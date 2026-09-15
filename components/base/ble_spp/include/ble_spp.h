/**
 * @file ble_spp.h
 * @brief BLE SPP (Serial Port Profile) using Nordic UART Service
 *
 * Provides BLE serial communication compatible with Pico2W connector.
 * Uses Nordic UART Service (NUS) UUIDs for compatibility with
 * "Serial Bluetooth Terminal" app and similar.
 *
 * Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX Char UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write - receive from phone)
 * TX Char UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify - send to phone)
 */

#ifndef BLE_SPP_H
#define BLE_SPP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/** BLE Device name - matches Pico2W naming convention */
#define BLE_DEVICE_NAME         "NitaraCLE4"

/** Maximum MTU size */
#define BLE_MTU_SIZE            500

/** Maximum receive buffer size */
#define BLE_RX_BUFFER_SIZE      1024

/*******************************************************************************
 * Types
 ******************************************************************************/

/**
 * @brief Callback function type for received BLE data
 * @param data Pointer to received data
 * @param len Length of received data
 */
typedef void (*ble_spp_rx_callback_t)(const char *data, int len);

/**
 * @brief BLE connection state
 */
typedef enum {
    BLE_STATE_IDLE = 0,         /**< Not initialized */
    BLE_STATE_ADVERTISING,      /**< Advertising, waiting for connection */
    BLE_STATE_CONNECTED,        /**< Connected to client */
    BLE_STATE_DISCONNECTED      /**< Disconnected, will restart advertising */
} ble_spp_state_t;

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * @brief Initialize BLE SPP module
 *
 * Initializes Bluetooth controller, Bluedroid stack, and GATT server.
 * Starts advertising after initialization.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_spp_init(void);

/**
 * @brief Deinitialize BLE SPP module
 *
 * Stops advertising, disconnects any clients, and releases resources.
 *
 * @return ESP_OK on success
 */
esp_err_t ble_spp_deinit(void);

/**
 * @brief Send data to connected BLE client via notify
 *
 * Sends data to the connected client using the TX characteristic.
 * If no client is connected or notifications are not enabled, returns error.
 *
 * @param data Pointer to data to send
 * @param len Length of data (max BLE_MTU_SIZE)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t ble_spp_send(const char *data, int len);

/**
 * @brief Register callback for received BLE data
 *
 * The callback will be called when data is received from the connected client.
 *
 * @param callback Function to call when data is received
 */
void ble_spp_register_rx_callback(ble_spp_rx_callback_t callback);

/**
 * @brief Check if BLE client is connected
 *
 * @return true if connected, false otherwise
 */
bool ble_spp_is_connected(void);

/**
 * @brief Check if notifications are enabled
 *
 * @return true if notifications are enabled, false otherwise
 */
bool ble_spp_notify_enabled(void);

/**
 * @brief Get current BLE state
 *
 * @return Current BLE state
 */
ble_spp_state_t ble_spp_get_state(void);

/**
 * @brief Get BLE device name
 *
 * @return Pointer to device name string
 */
const char *ble_spp_get_device_name(void);

/**
 * @brief Callback wrapper for cmd_parser output
 *
 * This function is compatible with cmd_output_callback_t and can be
 * registered with cmd_parser_register_output_callback().
 *
 * @param data Data to send
 * @param len Length of data
 */
void ble_spp_output_callback(const char *data, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_SPP_H */
