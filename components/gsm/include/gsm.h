#ifndef GSM_H
#define GSM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Network registration status (matches AT+CREG result codes) */
typedef enum {
    GSM_NET_NOT_REGISTERED      = 0,
    GSM_NET_REGISTERED_HOME     = 1,
    GSM_NET_SEARCHING           = 2,
    GSM_NET_DENIED              = 3,
    GSM_NET_UNKNOWN             = 4,
    GSM_NET_REGISTERED_ROAMING  = 5,
} gsm_network_status_t;

/* ===== Lifecycle ===== */
esp_err_t gsm_init(void);
esp_err_t gsm_deinit(void);

/* ===== Power control ===== */
esp_err_t gsm_power_on(void);
esp_err_t gsm_power_off(void);
esp_err_t gsm_reset(void);

/* ===== AT command engine ===== */
bool      gsm_is_alive(void);
esp_err_t gsm_send_at_command(const char *cmd, char *response,
                              size_t response_size, uint32_t timeout_ms);

/* ===== Helpers (parse common AT replies) ===== */
esp_err_t gsm_get_module_info(char *info, size_t info_size);
esp_err_t gsm_get_signal_strength(uint8_t *rssi, uint8_t *ber);
esp_err_t gsm_get_network_status(gsm_network_status_t *status);

/* Read SIM phone number via AT+CNUM. Note: many SIMs do not have the
 * MSISDN provisioned by the carrier; in that case the function returns
 * ESP_FAIL and 'number' is set to "". Always works for SIMs where the
 * carrier has written the number to the EF_MSISDN file. */
esp_err_t gsm_get_phone_number(char *number, size_t number_size);

/* Read SIM card serial number (ICCID) via AT+QCCID. Always available. */
esp_err_t gsm_get_iccid(char *iccid, size_t iccid_size);

/* ===== Data session (PDP context) ===== */
/* Activate PDP context using CONFIG_NCLE_GSM_APN. Required before any HTTP/TCP. */
esp_err_t gsm_pdp_activate(void);
esp_err_t gsm_pdp_deactivate(void);

/* ===== HTTP (testing/verification only — senior should harden for production) =====
 *
 * Quectel HTTP commands wrapped in convenience functions. Supports HTTP and HTTPS.
 * For HTTPS, certificate verification is controlled by CONFIG_NCLE_GSM_HTTP_INSECURE.
 *
 * Both functions activate the PDP context if not already active.
 *
 * @param url            Full URL including scheme (http:// or https://)
 * @param body           POST body (NULL for GET)
 * @param http_code_out  Server response code (e.g., 200) — output
 * @param resp_body      Optional response body buffer (NULL to discard)
 * @param resp_body_size Size of resp_body
 *
 * @return ESP_OK on success (request sent and response received), error otherwise.
 *         http_code_out reflects the server's HTTP status; check it independently.
 */
esp_err_t gsm_http_post(const char *url, const char *body,
                        int *http_code_out,
                        char *resp_body, size_t resp_body_size);
esp_err_t gsm_http_get(const char *url,
                       int *http_code_out,
                       char *resp_body, size_t resp_body_size);

/* ===== Ping (verify IP reachability — proves data plan is actually working) ===== */
typedef struct {
    bool     reachable;        /* true if any pings came back */
    uint8_t  sent;
    uint8_t  received;
    uint8_t  loss_pct;         /* 0=perfect, 100=nothing came back */
    uint16_t rtt_min_ms;
    uint16_t rtt_avg_ms;
    uint16_t rtt_max_ms;
} gsm_ping_result_t;

/**
 * @brief Ping a host using AT+QPING. Activates PDP context if needed.
 * @param host       Hostname or IP. Pass NULL to default to "8.8.8.8" (Google DNS).
 * @param count      Number of pings (default 4 if 0).
 * @param timeout_s  Per-ping timeout in seconds (default 5 if 0).
 * @param out        Result struct.
 */
esp_err_t gsm_ping(const char *host, uint8_t count, uint16_t timeout_s,
                   gsm_ping_result_t *out);

#ifdef __cplusplus
}
#endif
#endif /* GSM_H */
