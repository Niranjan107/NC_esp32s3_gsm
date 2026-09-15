#ifndef FOTA_H
#define FOTA_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HTTPS-pull OTA: device downloads a firmware .bin from a URL and reboots.
 * Runs on its own task; status is delivered via the output callback as JSON. */

/* Status callback: receives {"fota":"downloading","percent":N} /
 * {"fota":"success"} / {"fota":"failed","reason":"..."} for BLE + /resp. */
typedef void (*fota_output_cb_t)(const char *json, int len);

/* Register where FOTA status JSON goes (call once at startup). */
void fota_set_output_callback(fota_output_cb_t cb);

/* Start a firmware download from `url` (http:// or https://) on a new task.
 * Returns ESP_OK if started, ESP_ERR_INVALID_STATE if one is in progress,
 * ESP_ERR_INVALID_ARG on a bad/too-long URL. On success the device reboots. */
esp_err_t fota_start(const char *url);

/* Manually revert to the PREVIOUS firmware (the other OTA slot) and reboot.
 * For rolling back a bad update that booted fine but misbehaves later - the
 * previous image stays in its slot until the next update overwrites it, so it
 * is always available. Triggered by the `fota_rollback` command over
 * MQTT/BLE/USB (BLE/USB are the local escape hatch if the bad update broke
 * WiFi). Returns ESP_OK (device will reboot into the previous firmware), or
 * ESP_ERR_NOT_FOUND if there is no valid previous image to fall back to. */
esp_err_t fota_rollback(void);

#ifdef __cplusplus
}
#endif

#endif /* FOTA_H */
