/**
 * @file mqtt_client_svc.h
 * @brief MQTT cloud connectivity service (wraps ESP-IDF esp-mqtt).
 *
 * Isolated module: own FreeRTOS task + outgoing queue. The rest of the
 * firmware hands it a COPY of already-formed JSON; it never blocks callers.
 */
#ifndef MQTT_CLIENT_SVC_H
#define MQTT_CLIENT_SVC_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Runtime counters for the on-demand diagnostics (BLE `diag` command). */
typedef struct {
    bool     connected;   /* currently connected to the broker */
    uint32_t published;   /* readings successfully sent to the broker */
    uint32_t dropped;     /* readings not sent (buffer full / store-fwd off) */
    uint32_t deduped;     /* continuous-meter repeats skipped (not re-sent) */
    uint32_t buffered;    /* readings waiting on flash (store-and-forward) */
} mqtt_svc_stats_t;

/** Callback invoked with an incoming command payload (from clv4/<id>/cmd). */
typedef void (*mqtt_cmd_cb_t)(const char *data, int len);

/** Initialise + start the MQTT service (builds device id, starts task/client). */
esp_err_t mqtt_svc_start(void);

/** True once connected to the broker. */
bool mqtt_svc_is_connected(void);

/**
 * @brief The broker URI this build actually uses, for diag.
 *
 * Must be asked for rather than read from Kconfig: the real address comes from
 * the gitignored mqtt_secrets.h and only falls back to CONFIG_NCLE_MQTT_BROKER_URI
 * when that file is absent. `diag` used to print the Kconfig value directly and
 * so reported the public test broker while connected to the UAT one.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return static string, never NULL. Contains no credentials (user/password are
 *         separate fields, not embedded in the URI).
 */
const char *mqtt_svc_broker_uri(void);

/**
 * @brief Release the broker connection so something else can use the memory.
 *
 * Exists for FOTA. An HTTPS download needs a contiguous TLS receive buffer sized
 * to whatever record the server sends - up to ~16.7 KB from a CDN - and on this
 * board that block is not available while an MQTT TLS session is also open:
 * measured 7,680 bytes largest-free during a download, against 16,749 needed.
 * Stopping the client frees its session and closes the gap.
 *
 * Readings are NOT lost while suspended: they continue to be written to flash by
 * store-and-forward and are delivered once the link is back. What IS lost is
 * live reporting - progress messages cannot reach the broker until resume, so
 * they appear on BLE and the console only.
 *
 * Safe to call when MQTT is already stopped or was never started.
 */
void mqtt_svc_suspend(void);

/**
 * @brief Reconnect after mqtt_svc_suspend().
 *
 * Only needed when the operation that suspended it did NOT end in a reboot -
 * a successful FOTA reboots into the new firmware, so only the failure path
 * calls this.
 */
void mqtt_svc_resume(void);

/** Copy the current diagnostics counters (connected/published/dropped). */
void mqtt_svc_get_stats(mqtt_svc_stats_t *out);

/** Enqueue a reading (JSON copy) for publishing to clv4/<id>/data. Non-blocking. */
void mqtt_svc_publish_data(const char *json, int len);

/** Publish a command reply/ack to clv4/<id>/resp (best-effort). */
void mqtt_svc_publish_resp(const char *json, int len);

/** Register the callback that receives incoming commands from clv4/<id>/cmd. */
void mqtt_svc_set_cmd_callback(mqtt_cmd_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_CLIENT_SVC_H */
