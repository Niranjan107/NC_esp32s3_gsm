/**
 * @file wifi_sta_task.h
 * @brief WiFi STA task (CLV4 headless): boot auto-connect + status callback.
 *
 * On start: init WiFi, load saved credentials from NVS (or dev defaults),
 * auto-connect, then emit WiFi state changes through a registered callback.
 * No display/SPI dependency.
 */
#ifndef WIFI_STA_TASK_H
#define WIFI_STA_TASK_H

#include "esp_err.h"
#include "wifi_sta.h"   /* for wifi_sta_status_t */

#ifdef __cplusplus
extern "C" {
#endif

/** Callback invoked on every WiFi state change. */
typedef void (*wifi_status_cb_t)(const wifi_sta_status_t *status);

/** Register the status-change callback (e.g. main.c forwards to BLE). */
void ncle_wifi_sta_task_set_status_callback(wifi_status_cb_t cb);

/**
 * @brief Provision new WiFi credentials using "test before save".
 *
 * Connects to test the given credentials WITHOUT immediately overwriting
 * the saved ones. If the connection succeeds, the credentials are saved to
 * NVS. If it fails (e.g. wrong password), the device reverts to the
 * previously saved credentials and reports failure. Safe to call from
 * another task (e.g. cmd_parser).
 *
 * @param ssid     SSID to test
 * @param password Password to test (NULL/empty for open network)
 */
void ncle_wifi_sta_provision(const char *ssid, const char *password);

/** Start the WiFi STA task (inits WiFi, auto-connects from NVS/dev creds). */
esp_err_t ncle_wifi_sta_task_start(void);

/** Stop the WiFi STA task and deinit WiFi. */
esp_err_t ncle_wifi_sta_task_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_STA_TASK_H */
