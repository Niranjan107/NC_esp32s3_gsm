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

/* ===== SIM presence ===== */
typedef enum {
    GSM_SIM_READY = 0,      /* card present and unlocked  */
    GSM_SIM_ABSENT,         /* no card in the holder      */
    GSM_SIM_PIN_REQUIRED,   /* present but PIN/PUK locked */
    GSM_SIM_ERROR,          /* modem did not answer       */
} gsm_sim_status_t;

/* Check SIM presence via AT+CPIN?. Call before attempting a data session:
 * an empty holder otherwise looks like "every APN failed". */
gsm_sim_status_t gsm_get_sim_status(void);
const char      *gsm_sim_status_str(gsm_sim_status_t s);

/* Drop the short-lived SIM status cache, forcing the next query to ask the
 * modem. Call after a reset or power cycle, where the cached answer describes
 * the state before it. */
void             gsm_sim_cache_invalidate(void);

/* ===== Data link (PPP via esp_modem) =====
 *
 * These replace the AT-command data path. Once gsm_ppp_start() has run and
 * gsm_pdp_is_active() reports true, the modem is a normal ESP-IDF network
 * interface: esp-mqtt and esp_http_client work over it directly.
 */

/**
 * @brief Bring the data link up, discovering the APN if necessary.
 *
 * Checks modem and SIM presence first, then: stored APN -> each built-in
 * candidate in turn. The working APN is stored, so later boots connect
 * immediately; a stored APN that stops working is discarded and the trial
 * re-runs, so swapping the SIM needs no manual reconfiguration.
 *
 * BLOCKING - worst case around two minutes when every APN fails. Call from
 * gsm_task, never from app_main or a callback.
 *
 * @return ESP_OK           PPP up, IP assigned
 *         ESP_ERR_NOT_FOUND  modem absent, or no APN worked
 *         ESP_ERR_INVALID_STATE  SIM absent or PIN-locked
 */
esp_err_t gsm_ppp_start(void);

/**
 * @brief Introduce this component to net_link (base/common).
 *
 * After this, the application layer can ask net_link_is_up() without knowing
 * the transport is GSM - which is what lets components/application/ be shared
 * with the WiFi product unchanged. Call once, before starting MQTT.
 */
void gsm_net_link_register(void);

/* ===== Fault diagnosis =====
 *
 * Connectivity fails in six distinct ways, each needing a DIFFERENT fix. The
 * numeric status fields (rssi=4, net=3, data=0) are accurate but useless to
 * whoever is standing in front of the machine - and two of these are actively
 * misleading if not separated:
 *
 *   WEAK SIGNAL   the device half-works and drops randomly, which is harder to
 *                 diagnose than a clean failure because nothing looks broken
 *                 until data goes missing.
 *   SIM BARRED    the network REFUSES the SIM (unpaid bill, blocked IMEI).
 *                 Reported as "not registered" it looks like an antenna fault,
 *                 so someone checks the antenna when the real fix is a phone
 *                 call to the carrier.
 *
 * Checks run in order and stop at the first failure: reporting "no internet"
 * when the SIM is not inserted would send someone to fix the wrong thing.
 */
typedef enum {
    GSM_FAULT_NONE = 0,         /* everything works                      */
    GSM_FAULT_MODEM_DEAD,       /* no reply to AT - power/wiring         */
    GSM_FAULT_SIM_ABSENT,       /* no card in the holder                 */
    GSM_FAULT_SIM_LOCKED,       /* PIN/PUK required                      */
    GSM_FAULT_SIM_FAILURE,      /* card present but faulty               */
    GSM_FAULT_NO_SIGNAL,        /* CSQ 99 - antenna disconnected         */
    GSM_FAULT_WEAK_SIGNAL,      /* CSQ < 10 - antenna loose/bad location */
    GSM_FAULT_SIM_BARRED,       /* CREG 3 - network refused this SIM     */
    GSM_FAULT_NO_COVERAGE,      /* CREG 2 too long - no network here     */
    GSM_FAULT_NO_DATA_LINK,     /* registered but no IP - APN wrong      */
    GSM_FAULT_NO_INTERNET,      /* IP assigned but nothing routes        */
} gsm_fault_t;

/* Short machine-readable name, e.g. "sim_barred". Never NULL. */
const char *gsm_fault_name(gsm_fault_t f);

/* One sentence describing what is wrong, in plain language. Never NULL. */
const char *gsm_fault_problem(gsm_fault_t f);

/* One sentence telling the user what to DO about it. Never NULL. */
const char *gsm_fault_action(gsm_fault_t f);

/**
 * @brief Run the six checks in order and report the first failure.
 *
 * Prints a numbered stage-by-stage summary to the log, so whoever reads it -
 * engineer or field technician - sees exactly which stage failed and what to
 * do, without having to interpret AT traffic.
 *
 * @param check_internet  also verify traffic actually flows (costs a ping).
 *                        Pass false for a quick check that stops at "got IP".
 * @return GSM_FAULT_NONE when every stage passed.
 */
gsm_fault_t gsm_diagnose(bool check_internet);

/* Signal strength below this (AT+CSQ scale, 0-31) is reported as WEAK: the
 * link comes and goes rather than failing cleanly. */
#define GSM_RSSI_WEAK_THRESHOLD   10

/* ===== Connectivity self-test =====
 *
 * Both use STANDARD APIs - lwIP's esp_ping and an unmodified esp_http_client -
 * with no modem-specific transport. That is the point: if the PPP interface is
 * genuine, ordinary socket code works unchanged. Needing a modem-aware shim
 * would mean the interface is an AT wrapper, not a real netif.
 */

/* ICMP ping via lwIP (NOT AT+QPING). host NULL => "8.8.8.8", count 0 => 4.
 * Returns ESP_OK if at least one reply came back. Blocking. */
esp_err_t gsm_test_ping(const char *host, uint32_t count);

/* HTTP GET with stock esp_http_client. url NULL => "http://example.com".
 * Proves TCP + DNS work, which ping alone does not. Blocking. */
esp_err_t gsm_test_http_get(const char *url);

/* ===== APN configuration (BLE / USB console) ===== */

/* Set the APN explicitly and try to connect with it. The value is stored first,
 * so a reboot uses it directly. Unlike a trialled APN it is NOT discarded on
 * failure - the user set it deliberately. */
esp_err_t gsm_apn_set_and_connect(const char *apn);

/* Store (or clear, with NULL/"") the APN without connecting. */
esp_err_t gsm_apn_store(const char *apn);

/* Read the stored APN. Returns ESP_ERR_NOT_FOUND when none is set. */
esp_err_t gsm_apn_get(char *out, size_t out_size);

/* ===== Enable/disable preference (persisted) =====
 *
 * gsm_disable must survive a reboot: with auto-start enabled, a choice held
 * only in RAM would be silently undone at the next power cycle - the device
 * back online after the user deliberately turned it off.
 * Defaults to enabled on a device that has never been configured.
 */
bool      gsm_is_enabled_pref(void);
esp_err_t gsm_set_enabled_pref(bool enabled);

/* Return to pure command mode, dropping the data link. */
esp_err_t gsm_ppp_stop(void);

/* Signal strength sampled immediately before entering data mode. AT+CSQ cannot
 * run while PPP owns the UART, so this is the last true reading available for
 * the session. Returns false if none was obtained, so the caller can avoid
 * presenting a stale value as if it were current. */
bool gsm_get_data_mode_signal(uint8_t *rssi, uint8_t *ber);

/**
 * @brief Is the data link usable right now?
 *
 * Reports what lwIP believes (PPP got an IP), NOT what an AT command claimed -
 * a modem can report a PDP context active while no traffic passes. This is
 * deliberately separate from gsm_get_network_status(): a SIM can be registered
 * on the network and still have no working data connection (expired data pack),
 * and the field team needs to see which of the two failed.
 */
bool gsm_pdp_is_active(void);

/* ===== Data session (PDP context) — legacy AT path ===== */
/* Activate PDP context using CONFIG_NCLE_GSM_APN. Required before the
 * AT-command HTTP helpers below. Not needed for the PPP path above. */
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
