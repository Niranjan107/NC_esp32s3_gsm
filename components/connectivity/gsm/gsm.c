#include "gsm.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

/* esp_modem owns the UART once gsm_init() has run: it installs its own DTE on
 * GSM_UART_NUM. Nothing in this file may call uart_driver_install() or the raw
 * uart_read_bytes()/uart_write_bytes() path on that port any more - two drivers
 * on one UART either fail at install or silently corrupt RX, and once PPP is up
 * raw reads would consume PPP frames. All AT traffic goes through
 * esp_modem_at(), which is CMUX-safe. See docs/GSM_PORT_PLAN.md Step 1. */
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs.h"
#include <inttypes.h>
#include "ping/ping_sock.h"
#include "lwip/inet.h"
#include "esp_http_client.h"
#include "net_link.h"        /* the contract the application layer talks to */
#include "gsm_task.h"        /* cached status for status_json */

static const char *TAG = "GSM";

#define GSM_UART_NUM        CONFIG_NCLE_GSM_UART_NUM
#define GSM_UART_TX_PIN     CONFIG_NCLE_GSM_UART_TX_PIN
#define GSM_UART_RX_PIN     CONFIG_NCLE_GSM_UART_RX_PIN
#define GSM_PWRKEY_PIN      CONFIG_NCLE_GSM_PWRKEY_PIN
#define GSM_RST_PIN         CONFIG_NCLE_GSM_RST_PIN
#define GSM_UART_BAUD_RATE  CONFIG_NCLE_GSM_UART_BAUD_RATE

#define GSM_RX_BUFFER_SIZE  2048
#define GSM_TX_BUFFER_SIZE  1024

/* Reset polarity: by default Conn_plus board has an inverter on RESET,
 * so HIGH = assert, LOW = release. Direct (datasheet-spec) boards disable
 * the flag in menuconfig. */
#ifdef CONFIG_NCLE_GSM_RST_INVERTED
#define GSM_RST_ASSERT      1
#define GSM_RST_RELEASE     0
#else
#define GSM_RST_ASSERT      0
#define GSM_RST_RELEASE     1
#endif

static bool s_initialized = false;
static SemaphoreHandle_t s_uart_mutex = NULL;

/* esp_modem handles. s_dce is the modem itself; s_ppp_netif is the lwIP
 * interface it feeds once we switch to data mode. */
static esp_modem_dce_t  *s_dce       = NULL;
static esp_netif_t      *s_ppp_netif = NULL;

/* Set true only by the IP_EVENT_PPP_GOT_IP handler and cleared on LOST_IP, so
 * it reflects what lwIP actually believes - not what an AT command claimed.
 * gsm_pdp_is_active() reports this. */
static volatile bool s_ppp_got_ip = false;

/* Cached module identity ("Quectel EC200U"), filled by gsm_get_module_info()
 * so the diagnostic can name the hardware without another AT round trip. */
static char s_module_info[48] = {0};

/* Signal strength sampled immediately before entering data mode. AT+CSQ cannot
 * run once PPP owns the UART, so this is the last true reading available for
 * the duration of the session. */
static uint8_t s_rssi_at_data_entry = 99;
static uint8_t s_ber_at_data_entry  = 99;

/* Report the signal captured on the way into data mode. Returns false when no
 * reading was obtained, so the caller can fall back rather than show a stale
 * number as if it were current. */
bool gsm_get_data_mode_signal(uint8_t *rssi, uint8_t *ber)
{
    if (s_rssi_at_data_entry == 99) return false;
    if (rssi) *rssi = s_rssi_at_data_entry;
    if (ber)  *ber  = s_ber_at_data_entry;
    return true;
}

/* ============================================================================
 * APN selection
 * ============================================================================
 * Deliberately NOT an IMSI/MCC-MNC lookup table. Identifying the carrier from
 * the SIM tells you the CARRIER, not whether an APN actually carries traffic -
 * an Airtel M2M SIM matches "Airtel" perfectly and then fails, because it needs
 * airteliot.com rather than airtelgprs.com. Trying APNs in order tests the only
 * thing that matters: does a data session come up.
 *
 * Order of preference:
 *   1. APN stored in NVS (set over BLE/USB, or learned from a previous success)
 *   2. Each candidate below, in turn, until PDP activation succeeds
 *   3. On success the winner is written to NVS, so later boots skip the trial
 *
 * A stored APN that stops working (SIM swapped, carrier changed) is retried
 * once and then discarded, falling back to the trial - so a SIM swap needs no
 * manual reconfiguration.
 * ===========================================================================*/
#define GSM_NVS_NAMESPACE   "gsm"
#define GSM_NVS_KEY_APN     "apn"
#define GSM_APN_MAX_LEN     64

/* Ordered by how likely they are to be the right answer on this product.
 * airteliot.com is included because Airtel IoT/M2M SIMs - the kind normally
 * bought for machines like this - do not use the consumer APN. */
static const char *const GSM_APN_CANDIDATES[] = {
    "airtelgprs.com",   /* Airtel consumer          */
    "jionet",           /* Jio                      */
    "bsnlnet",          /* BSNL / MTNL              */
    "www",              /* Vi (Vodafone Idea)       */
    "airteliot.com",    /* Airtel IoT / M2M         */
};
#define GSM_APN_CANDIDATE_COUNT \
    (sizeof(GSM_APN_CANDIDATES) / sizeof(GSM_APN_CANDIDATES[0]))

/**
 * @brief Read the stored APN, if any.
 *
 * OUTPUT:
 * @return true when a non-empty APN was loaded into out.
 */
static bool gsm_apn_load(char *out, size_t out_size)
{
    nvs_handle_t nvs;
    size_t len = out_size;

    if (nvs_open(GSM_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_get_str(nvs, GSM_NVS_KEY_APN, out, &len);
    nvs_close(nvs);

    return (err == ESP_OK && out[0] != '\0');
}

/**
 * @brief Persist an APN so later boots skip the trial.
 *
 * INPUT:
 * @param apn  APN string, or NULL/"" to clear a stored value that stopped working
 */
esp_err_t gsm_apn_store(const char *apn)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(GSM_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    if (apn == NULL || apn[0] == '\0') {
        err = nvs_erase_key(nvs, GSM_NVS_KEY_APN);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;   /* nothing to clear */
        ESP_LOGI(TAG, "APN cleared from NVS");
    } else {
        err = nvs_set_str(nvs, GSM_NVS_KEY_APN, apn);
        ESP_LOGI(TAG, "APN stored in NVS: %s", apn);
    }

    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

esp_err_t gsm_apn_get(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';
    return gsm_apn_load(out, out_size) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ===== Enable/disable preference =====
 *
 * Persisted so that a user who sends gsm_disable stays disabled across a power
 * cycle. Without this, auto-start would silently undo their choice at the next
 * reboot - the device would be back online after they deliberately turned it
 * off, with no indication why.
 */
#define GSM_NVS_KEY_ENABLED  "enabled"

bool gsm_is_enabled_pref(void)
{
    nvs_handle_t nvs;
    uint8_t enabled = 1;      /* default ON: a fresh device should connect */

    if (nvs_open(GSM_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u8(nvs, GSM_NVS_KEY_ENABLED, &enabled) != ESP_OK) {
            enabled = 1;      /* key absent - never configured, so default ON */
        }
        nvs_close(nvs);
    }
    return (enabled != 0);
}

esp_err_t gsm_set_enabled_pref(bool enabled)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(GSM_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(nvs, GSM_NVS_KEY_ENABLED, enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "GSM %s (saved - survives reboot)",
             enabled ? "ENABLED" : "DISABLED");
    return err;
}

/* PPP/IP event handler: the single source of truth for "is the data link up".
 * Registered in gsm_init(), so the flag is correct even if PPP drops on its own. */
static void gsm_ip_event_handler(void *arg, esp_event_base_t base,
                                 int32_t event_id, void *event_data)
{
    /* Two event bases arrive here and their id values overlap numerically, so
     * they must be separated by base before switching. */
    if (base == NETIF_PPP_STATUS) {
        switch (event_id) {
            /* The LCP keepalive failing is how a modem that vanished
             * mid-session is detected. Nothing else can see it: the UART is
             * carrying PPP frames so AT commands are unavailable, and without
             * this the link sat reporting data=1 indefinitely with the modem
             * powered off. */
            case NETIF_PPP_ERRORPEERDEAD:
                ESP_LOGE(TAG, "PPP peer dead (no keepalive reply) - modem gone");
                s_ppp_got_ip = false;
                break;
            case NETIF_PPP_ERRORCONNECT:
                ESP_LOGW(TAG, "PPP connection lost");
                s_ppp_got_ip = false;
                break;
            case NETIF_PPP_PHASE_DEAD:
                if (s_ppp_got_ip) {
                    ESP_LOGW(TAG, "PPP phase DEAD - data link down");
                    s_ppp_got_ip = false;
                }
                break;
            default:
                break;
        }
        return;
    }

    switch (event_id) {
        case IP_EVENT_PPP_GOT_IP: {
            ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
            esp_netif_dns_info_t dns1 = {0}, dns2 = {0};
            esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN, &dns1);
            esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_BACKUP, &dns2);

            ESP_LOGI(TAG, "=== PPP GOT IP ===");
            ESP_LOGI(TAG, "  IP      : " IPSTR, IP2STR(&e->ip_info.ip));
            ESP_LOGI(TAG, "  Gateway : " IPSTR, IP2STR(&e->ip_info.gw));
            ESP_LOGI(TAG, "  Netmask : " IPSTR, IP2STR(&e->ip_info.netmask));
            ESP_LOGI(TAG, "  DNS1    : " IPSTR, IP2STR(&dns1.ip.u_addr.ip4));
            ESP_LOGI(TAG, "  DNS2    : " IPSTR, IP2STR(&dns2.ip.u_addr.ip4));
            s_ppp_got_ip = true;
            break;
        }
        case IP_EVENT_PPP_LOST_IP:
            ESP_LOGW(TAG, "PPP lost IP - data link is down");
            s_ppp_got_ip = false;
            break;
        default:
            break;
    }
}

bool gsm_pdp_is_active(void)
{
    /* Deliberately reports the lwIP view, not an AT-command claim: a modem can
     * report a PDP context active while no traffic passes. Registration state
     * is a separate question - see gsm_get_network_status(). */
    return s_ppp_got_ip;
}

static void gsm_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GSM_PWRKEY_PIN) | (1ULL << GSM_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en   = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(GSM_RST_PIN, GSM_RST_RELEASE);
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    ESP_LOGI(TAG, "GPIO: PWRKEY=%d, RST=%d (inverted=%d)",
             GSM_PWRKEY_PIN, GSM_RST_PIN,
#ifdef CONFIG_NCLE_GSM_RST_INVERTED
             1
#else
             0
#endif
            );
}

/**
 * @brief Create the esp_modem DTE/DCE and the PPP netif. Replaces the old
 *        gsm_uart_init(): esp_modem installs the UART driver itself.
 *
 * The DCE is created in COMMAND mode, so every existing AT helper keeps working
 * exactly as before. Data mode is entered separately by gsm_ppp_start().
 *
 * OUTPUT:
 * @return ESP_OK when the DCE and netif exist, error otherwise.
 */
/* esp_modem logs "Rx Break" (esp_modem_uart.cpp) for EVERY break event. An
 * unpowered modem holds its TX line low, which the UART reports as a continuous
 * break - thousands of identical warnings that bury every useful line in the
 * log, which is exactly when you are trying to read it.
 *
 * Rather than silence the condition (a real wiring fault would then be
 * invisible), demote esp_modem's per-event warning and report the SAME
 * information once every few seconds with a count. One line instead of a
 * thousand, and the count says more than any single event could: a steady rate
 * means an unpowered modem, a sporadic one means a flaky connection. */
#define GSM_BREAK_REPORT_INTERVAL_MS  5000

static void gsm_quiet_uart_break_spam(void)
{
    /* esp_modem's UART terminal tag. Errors still print. */
    esp_log_level_set("uart_terminal", ESP_LOG_ERROR);
}

/* Called from the poll loop and the init retry path to emit the summary. */
static void gsm_report_line_state(void)
{
    static int64_t last_report_us = 0;
    int64_t now = esp_timer_get_time();

    if (last_report_us == 0) {
        last_report_us = now;
        return;
    }
    if ((now - last_report_us) < (GSM_BREAK_REPORT_INTERVAL_MS * 1000)) {
        return;
    }
    last_report_us = now;

    /* A break condition means RX has been held low. If the modem is answering
     * AT commands the line is obviously fine, so only report when it is not. */
    if (s_dce && !s_ppp_got_ip && esp_modem_sync(s_dce) != ESP_OK) {
        ESP_LOGW(TAG, "UART RX idle-low - modem unpowered or TX disconnected "
                      "(check GPIO%d <- module TX, and module power 3.8-4.2V)",
                 GSM_UART_RX_PIN);
    }
}

static esp_err_t gsm_modem_init(void)
{
    if (s_dce != NULL) return ESP_OK;      /* already built */

    gsm_quiet_uart_break_spam();

    /* The APN passed here is only a placeholder: gsm_ppp_start() calls
     * esp_modem_set_apn() with the stored or trialled value before entering
     * data mode. Using the Kconfig value keeps the DCE config valid until then. */
    const char *initial_apn = CONFIG_NCLE_GSM_APN;

    /* PPP netif - this is what makes the modem a first-class ESP-IDF interface,
     * so esp-mqtt / esp_http_client can open ordinary sockets over it. */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_PPP();
    s_ppp_netif = esp_netif_new(&netif_cfg);
    if (s_ppp_netif == NULL) {
        ESP_LOGE(TAG, "esp_netif_new(PPP) failed");
        return ESP_FAIL;
    }

    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.port_num   = GSM_UART_NUM;
    dte_cfg.uart_config.tx_io_num  = GSM_UART_TX_PIN;
    dte_cfg.uart_config.rx_io_num  = GSM_UART_RX_PIN;
    dte_cfg.uart_config.rts_io_num = UART_PIN_NO_CHANGE;
    dte_cfg.uart_config.cts_io_num = UART_PIN_NO_CHANGE;
    dte_cfg.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_cfg.uart_config.baud_rate  = GSM_UART_BAUD_RATE;
    dte_cfg.uart_config.rx_buffer_size = GSM_RX_BUFFER_SIZE;
    dte_cfg.uart_config.tx_buffer_size = GSM_TX_BUFFER_SIZE;

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(initial_apn);

    /* EC200U speaks the standard Quectel/BG96 command set for the parts
     * esp_modem drives (CGDATA, CMUX, CSQ, CREG). */
    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_BG96, &dte_cfg, &dce_cfg, s_ppp_netif);
    if (s_dce == NULL) {
        ESP_LOGE(TAG, "esp_modem_new_dev failed");
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "esp_modem DTE created: UART%d TX=%d RX=%d @%d",
             GSM_UART_NUM, GSM_UART_TX_PIN, GSM_UART_RX_PIN, GSM_UART_BAUD_RATE);
    return ESP_OK;
}

/**
 * @brief Is a SIM present and ready? (AT+CPIN?)
 *
 * OUTPUT:
 * @return GSM_SIM_READY        card present and unlocked
 *         GSM_SIM_ABSENT       no card in the holder
 *         GSM_SIM_PIN_REQUIRED card present but locked
 *         GSM_SIM_ERROR        modem did not answer
 */
/* Short-lived cache of the SIM answer.
 *
 * Several callers ask independently - the startup sequence, each poll, the
 * gsm_ppp_start() precondition, and gsm_diagnose(). With a 5-attempt retry loop
 * inside, an absent SIM turned one question into five separate 5-second probes
 * per cycle: about 25 seconds of AT traffic all establishing the same fact.
 *
 * A few seconds of caching collapses that to one real check. The window is
 * short enough that inserting a SIM is still noticed within one poll. */
#define GSM_SIM_CACHE_VALID_MS   5000

static gsm_sim_status_t s_sim_cached      = GSM_SIM_ERROR;
static int64_t          s_sim_cached_at   = 0;

/* Force the next gsm_get_sim_status() to talk to the modem. Called after a
 * reset or power cycle, where the cached answer describes the old state. */
void gsm_sim_cache_invalidate(void)
{
    s_sim_cached_at = 0;
}

gsm_sim_status_t gsm_get_sim_status(void)
{
    char resp[128] = {0};
    bool saw_absent = false;

    int64_t now = esp_timer_get_time();
    if (s_sim_cached_at != 0 &&
        (now - s_sim_cached_at) < (GSM_SIM_CACHE_VALID_MS * 1000)) {
        return s_sim_cached;
    }

    /* The SIM interface is not ready the instant the modem boots or resets: the
     * module answers "+CME ERROR: 14" (SIM busy) for a second or two while it
     * initialises the card. Treating that as "no SIM" reports a hardware fault
     * that does not exist, so retry a few times before concluding anything. */
    for (int attempt = 0; attempt < 5; attempt++) {
        resp[0] = '\0';
        esp_err_t err = gsm_send_at_command("+CPIN?", resp, sizeof(resp), 5000);

        if (err == ESP_OK) {
            gsm_sim_status_t r = GSM_SIM_ERROR;
            if (strstr(resp, "READY")) {
                r = GSM_SIM_READY;
            } else if (strstr(resp, "SIM PIN") || strstr(resp, "SIM PUK")) {
                r = GSM_SIM_PIN_REQUIRED;
            } else if (strstr(resp, "POWERED DOWN") || strstr(resp, "RDY")) {
                /* The module is mid-boot - it answers "POWERED DOWN" while
                 * coming back from a power cycle. That is not a SIM fault, but
                 * reporting it as one told the user to clean the contacts of a
                 * SIM that was fine and READY ten seconds later. Wait instead. */
                ESP_LOGI(TAG, "modem still booting, waiting (%d/5)...", attempt + 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            } else {
                ESP_LOGW(TAG, "Unrecognised +CPIN? reply: %s", resp);
            }
            s_sim_cached    = r;
            s_sim_cached_at = esp_timer_get_time();
            return r;
        }

        /* Non-OK: the reply text distinguishes the real cases.
         *   CME 12 = SIM PUK required   -> genuinely locked, definitive
         *   CME 13 = SIM failure        -> genuinely faulty, definitive
         *   CME 10 = SIM not inserted   -> NOT definitive, see below
         *   CME 14 = SIM busy           -> transient by definition
         */
        if (strstr(resp, "+CME ERROR: 12")) {
            s_sim_cached = GSM_SIM_PIN_REQUIRED;
            s_sim_cached_at = esp_timer_get_time();
            return GSM_SIM_PIN_REQUIRED;
        }
        if (strstr(resp, "+CME ERROR: 13")) {
            s_sim_cached = GSM_SIM_ERROR;
            s_sim_cached_at = esp_timer_get_time();
            return GSM_SIM_ERROR;
        }

        /* CME 10 is reported as "SIM not inserted", but a freshly booted module
         * answers 10 for a moment before its SIM interface comes up - observed
         * on this EC200U returning 10 and then READY 70 ms later. Treating the
         * first answer as final reported "no SIM" on a device that went on to
         * open a working data session: a contradiction that would send someone
         * to site to check a SIM that was fine. So retry it like 14, and only
         * conclude ABSENT if it persists across every attempt. */
        if (strstr(resp, "+CME ERROR: 10")) {
            ESP_LOGI(TAG, "SIM reported absent, re-checking (%d/5)...", attempt + 1);
            saw_absent = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (strstr(resp, "+CME ERROR: 14")) {
            ESP_LOGI(TAG, "SIM busy, retrying (%d/5)...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Anything else (no reply at all) - retry, the modem may still be
         * settling after a reset. */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Persistently "not inserted" across every attempt: now believe it. */
    if (saw_absent) {
        ESP_LOGW(TAG, "SIM absent (consistent across retries)");
        s_sim_cached    = GSM_SIM_ABSENT;
        s_sim_cached_at = esp_timer_get_time();
        return GSM_SIM_ABSENT;
    }

    ESP_LOGW(TAG, "SIM status unresolved after retries: %s", resp);
    s_sim_cached    = GSM_SIM_ERROR;
    s_sim_cached_at = esp_timer_get_time();
    return GSM_SIM_ERROR;
}

const char *gsm_sim_status_str(gsm_sim_status_t s)
{
    switch (s) {
        case GSM_SIM_READY:        return "ready";
        case GSM_SIM_ABSENT:       return "absent";
        case GSM_SIM_PIN_REQUIRED: return "pin_required";
        default:                   return "error";
    }
}

/* Tear down the DCE and PPP netif. Safe to call when they were never created. */
static void gsm_modem_deinit(void)
{
    /* Unregister before destroying the netif, so a late PPP event cannot fire
     * into a freed interface. Also stops "handler already registered,
     * overwriting" warnings leaking a handler slot on every reinit. */
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_PPP_GOT_IP,
                                 &gsm_ip_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_PPP_LOST_IP,
                                 &gsm_ip_event_handler);
    esp_event_handler_unregister(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                                 &gsm_ip_event_handler);

    if (s_dce) {
        esp_modem_destroy(s_dce);
        s_dce = NULL;
    }
    if (s_ppp_netif) {
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
    }
    s_ppp_got_ip = false;
}

/* How long to wait for PPP to hand us an IP after switching to CMUX, before
 * declaring this APN a failure and moving to the next candidate. PDP activation
 * on a good link is a few seconds; 20s is generous without stalling the trial. */
/* A good link hands over an IP in ~6s. 30s leaves room for a slow PDP
 * activation or a congested cell without stalling the whole trial. */
#define GSM_PPP_IP_TIMEOUT_MS   30000

/**
 * @brief Try ONE APN: set it, switch to CMUX, wait for an IP.
 *
 * OUTPUT:
 * @return ESP_OK if an IP arrived within GSM_PPP_IP_TIMEOUT_MS.
 */
/**
 * @brief Return the modem to a usable command-mode state after a failed CMUX
 *        or PPP attempt.
 *
 * WHY THIS IS NOT JUST set_mode(COMMAND): when a CMUX switch fails or PPP never
 * negotiates, the modem can be left mid-protocol - still framing CMUX, or with
 * lwIP holding the PPP session half-open. The ESP32 then sees a continuous
 * stream of line breaks ("uart_terminal: Rx Break") and EVERY later AT command
 * fails, so one bad APN attempt poisons the UART for everything after it.
 *
 * Stopping the netif first, then dropping to command mode, then draining
 * whatever noise is still in the RX FIFO, leaves the port genuinely clean for
 * the next attempt.
 */
static void gsm_cmux_teardown(void)
{
    if (s_dce == NULL) return;

    /* 1. Take PPP down from the lwIP side so it stops driving the link. */
    if (s_ppp_netif) {
        esp_netif_action_stop(s_ppp_netif, NULL, 0, NULL);
    }
    s_ppp_got_ip = false;

    /* 2. Back to plain command mode. */
    esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
    vTaskDelay(pdMS_TO_TICKS(500));      /* let the modem settle */

    /* 3. Drain any residual framing bytes so the next AT reply is not prefixed
     * with junk from the aborted session. */
    uart_flush_input(GSM_UART_NUM);

    /* 4. Confirm the modem is actually answering again. If not, the caller's
     * next command will fail and the task's dead-modem recovery takes over. */
    for (int i = 0; i < 3; i++) {
        if (esp_modem_sync(s_dce) == ESP_OK) {
            ESP_LOGI(TAG, "command mode restored");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "modem not answering after CMUX teardown");
}

static esp_err_t gsm_ppp_try_apn(const char *apn, bool first_attempt)
{
    ESP_LOGI(TAG, "--- trying APN '%s' ---", apn);

    /* Only clean up when a PREVIOUS attempt could have left the modem in a bad
     * state. On the first attempt the modem is already in command mode from
     * gsm_init(), and running the teardown here costs ~8.5s of the connection
     * budget for nothing - which was enough to turn a working 6s connect into
     * a 20s timeout. */
    if (!first_attempt) {
        gsm_cmux_teardown();
    }
    s_ppp_got_ip = false;

    esp_err_t err = esp_modem_set_apn(s_dce, apn);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_apn('%s') failed: 0x%x", apn, err);
        return err;
    }

    /* Capture signal strength NOW, while AT is still available. Once PPP owns
     * the UART, AT+CSQ cannot run, so whatever is cached at this moment is what
     * the status will report for the whole session. Without this the value left
     * over from before a recovery was stale - typically 99 ("unknown"), which
     * displayed as 0 bars on a connection that was working perfectly. */
    uint8_t rssi = 99, ber = 99;
    if (gsm_get_signal_strength(&rssi, &ber) == ESP_OK && rssi != 99) {
        s_rssi_at_data_entry = rssi;
        s_ber_at_data_entry  = ber;
        ESP_LOGI(TAG, "signal at data-mode entry: csq %d", rssi);
    }

    /* Plain DATA mode, NOT CMUX.
     *
     * CMUX was the original choice, so RSSI polling could share the UART with
     * PPP. On this EC200U it proved unstable: seconds after PPP connected the
     * link collapsed into a continuous "Restarting CMUX state machine
     * (reason: 6)" storm, the AT channel went blind (AT+CSQ returning a bare
     * OK), and tearing the modem down mid-storm crashed the device.
     *
     * A stable data link with no RSSI polling beats an unstable one with it.
     * While PPP is up, gsm_task stops issuing AT commands entirely and reports
     * the link from the netif instead - which is what the four-state model
     * actually needs. Signal strength resumes when the link is down. */
    err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DATA switch failed for '%s': 0x%x", apn, err);
        gsm_cmux_teardown();     /* half-switched mode would poison the UART */
        return err;
    }

    /* Wait for the IP_EVENT_PPP_GOT_IP handler to fire. */
    const TickType_t step = pdMS_TO_TICKS(250);
    int waited_ms = 0;
    while (waited_ms < GSM_PPP_IP_TIMEOUT_MS) {
        if (s_ppp_got_ip) {
            ESP_LOGI(TAG, "APN '%s' WORKED (IP in %d ms)", apn, waited_ms);
            return ESP_OK;
        }
        vTaskDelay(step);
        waited_ms += 250;
    }

    ESP_LOGW(TAG, "APN '%s' gave no IP after %d ms", apn, GSM_PPP_IP_TIMEOUT_MS);
    gsm_cmux_teardown();
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Bring the data link up, discovering the APN if necessary.
 *
 * Order: stored APN (fast path) -> each candidate in turn. On success the
 * working APN is stored, so subsequent boots connect immediately. A stored APN
 * that has stopped working is discarded and the trial re-runs, so swapping the
 * SIM needs no manual reconfiguration.
 *
 * NOTE: blocking. Worst case is roughly
 *       (1 + GSM_APN_CANDIDATE_COUNT) * GSM_PPP_IP_TIMEOUT_MS, so call it from
 *       gsm_task, never from app_main or a callback.
 *
 * OUTPUT:
 * @return ESP_OK when PPP is up and an IP has been assigned.
 *         ESP_ERR_NOT_FOUND when every APN failed - the caller should report
 *         "APN required" so the field team can set one over BLE/USB.
 */
esp_err_t gsm_ppp_start(void)
{
    if (s_dce == NULL) return ESP_ERR_INVALID_STATE;

    /* Preconditions, checked in dependency order. Without these a missing modem
     * or an empty SIM holder would spend ~2 minutes failing every APN in turn
     * and then report "no APN worked" - which sends the field team looking for
     * a carrier problem that does not exist. Each check below fails in seconds
     * and names the actual fault. */

    /* 1. Is the modem there and answering? */
    if (!gsm_is_alive()) {
        ESP_LOGE(TAG, "Modem not responding - check power, wiring and PWRKEY");
        return ESP_ERR_NOT_FOUND;
    }

    /* 2. Is a SIM present and unlocked? */
    gsm_sim_status_t sim = gsm_get_sim_status();
    if (sim != GSM_SIM_READY) {
        ESP_LOGE(TAG, "SIM not usable: %s", gsm_sim_status_str(sim));
        if (sim == GSM_SIM_ABSENT) {
            ESP_LOGE(TAG, "  -> insert a SIM card");
        } else if (sim == GSM_SIM_PIN_REQUIRED) {
            ESP_LOGE(TAG, "  -> SIM is PIN-locked; disable the PIN on a phone");
        }
        return ESP_ERR_INVALID_STATE;
    }

    /* 3. Registered on a network? Not fatal - registration can complete while
     * we are trying APNs - but log it, because "no APN worked" on an
     * unregistered SIM means no coverage, not a wrong APN. */
    gsm_network_status_t net = GSM_NET_UNKNOWN;
    if (gsm_get_network_status(&net) == ESP_OK) {
        bool registered = (net == GSM_NET_REGISTERED_HOME ||
                           net == GSM_NET_REGISTERED_ROAMING);
        if (!registered) {
            ESP_LOGW(TAG, "Not registered yet (CREG=%d) - APN trial may fail "
                          "for lack of coverage rather than a wrong APN", net);
        }
    }

    char stored[GSM_APN_MAX_LEN] = {0};

    /* 1. Fast path: an APN we were given, or learned last time. */
    bool first = true;

    if (gsm_apn_load(stored, sizeof(stored))) {
        ESP_LOGI(TAG, "Using stored APN: %s", stored);
        if (gsm_ppp_try_apn(stored, first) == ESP_OK) {
            return ESP_OK;
        }
        first = false;
        ESP_LOGW(TAG, "Stored APN '%s' no longer works - re-running trial", stored);
        gsm_apn_store(NULL);      /* discard, so a SIM swap self-heals */
    }

    /* 2. Trial: first candidate that yields an IP wins. */
    ESP_LOGI(TAG, "Trying %d candidate APNs...", (int)GSM_APN_CANDIDATE_COUNT);
    for (size_t i = 0; i < GSM_APN_CANDIDATE_COUNT; i++) {
        if (gsm_ppp_try_apn(GSM_APN_CANDIDATES[i], first) == ESP_OK) {
            gsm_apn_store(GSM_APN_CANDIDATES[i]);   /* 3. remember the winner */
            return ESP_OK;
        }
        first = false;
    }

    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "No APN worked. Possible causes:");
    ESP_LOGE(TAG, "  - SIM has no active data plan");
    ESP_LOGE(TAG, "  - M2M/enterprise SIM with a private APN");
    ESP_LOGE(TAG, "    -> set it over BLE/USB: {\"command\":\"apn_config\",...}");
    ESP_LOGE(TAG, "  - not registered on the network (check gsm_get_network_status)");
    ESP_LOGE(TAG, "========================================");
    return ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * net_link registration - the contract with the application layer
 * ============================================================================
 * The application (MQTT publish, store-and-forward, FOTA) asks exactly one
 * question about the network: "can I send right now?". It must never learn
 * WHETHER that link is GSM - otherwise components/application/ could not have
 * been copied from the WiFi product unchanged, which it was.
 *
 * So the GSM component introduces itself here, and everything above it keeps
 * calling net_link_is_up().
 * ===========================================================================*/

static bool gsm_link_is_up(void)
{
    /* Deliberately the PPP/lwIP view, not an AT-command claim: a modem can
     * report a PDP context active while nothing routes. Must not block - this
     * is called from the MQTT task's polling loop. */
    return gsm_pdp_is_active();
}

/**
 * @brief Describe this link for `diag` (net_link_t::status_json).
 *
 * The WiFi product's version of this reports ssid/ip/rssi; this one reports
 * SIM and signal. diag itself is unchanged either way - it prints whatever the
 * active link says about itself, which is what makes it transport-neutral.
 *
 * "operator" is deliberately absent: naming the carrier needs AT+COPS?, which
 * is not wrapped, and the ICCID already identifies the SIM for support.
 */
static int gsm_link_status_json(char *buf, size_t size)
{
    gsm_status_t s;
    gsm_task_get_last_status(&s);

    return snprintf(buf, size,
                    "{\"connected\":%s,\"sim\":\"%s\",\"registered\":%s,"
                    "\"rssi\":%d,\"bars\":%d,\"iccid\":\"%s\",\"fault\":\"%s\"}",
                    s.data_up    ? "true" : "false",
                    gsm_sim_status_str(s.sim_status),
                    s.registered ? "true" : "false",
                    s.rssi, s.bars, s.iccid,
                    gsm_fault_name(s.fault));
}

void gsm_net_link_register(void)
{
    /* One-shot. net_link is a 2-slot append-only table with no unregister, and
     * link_mode calls this every time the operator switches back to gsm - a
     * second registration would take the slot WiFi needs, and the third would
     * fail outright. The failure is quiet: the application only asks "is
     * anything up", so a link missing from the table reads as permanently
     * offline on a modem that is working fine. */
    static bool s_registered = false;
    if (s_registered) {
        return;
    }

    /* Static storage: net_link keeps the pointer, so a stack copy would
     * dangle. is_provisioned is left NULL - a SIM needs no credentials
     * entered, unlike WiFi, and net_link treats absent as "yes". */
    static const net_link_t gsm_link = {
        .name        = "gsm",
        .is_up       = gsm_link_is_up,
        .status_json = gsm_link_status_json,
    };

    if (net_link_register(&gsm_link)) {
        s_registered = true;
        ESP_LOGI(TAG, "registered with net_link as '%s'", gsm_link.name);
    } else {
        ESP_LOGE(TAG, "net_link registration FAILED - the application layer "
                      "will believe the device is permanently offline");
    }
}

/* ============================================================================
 * Fault diagnosis
 * ============================================================================
 * Six stages, checked in order, stopping at the first failure. Each fault
 * carries both what is wrong and what to DO, so the log is readable by whoever
 * is standing in front of the machine - not just by someone who knows that
 * "net=3" means the carrier barred the SIM.
 * ===========================================================================*/

const char *gsm_fault_name(gsm_fault_t f)
{
    switch (f) {
        case GSM_FAULT_NONE:         return "ok";
        case GSM_FAULT_MODEM_DEAD:   return "modem_dead";
        case GSM_FAULT_SIM_ABSENT:   return "sim_absent";
        case GSM_FAULT_SIM_LOCKED:   return "sim_locked";
        case GSM_FAULT_SIM_FAILURE:  return "sim_failure";
        case GSM_FAULT_NO_SIGNAL:    return "no_signal";
        case GSM_FAULT_WEAK_SIGNAL:  return "weak_signal";
        case GSM_FAULT_SIM_BARRED:   return "sim_barred";
        case GSM_FAULT_NO_COVERAGE:  return "no_coverage";
        case GSM_FAULT_NO_DATA_LINK: return "no_data_link";
        case GSM_FAULT_NO_INTERNET:  return "no_internet";
        default:                     return "unknown";
    }
}

const char *gsm_fault_problem(gsm_fault_t f)
{
    switch (f) {
        case GSM_FAULT_NONE:
            return "Everything working";
        case GSM_FAULT_MODEM_DEAD:
            return "GSM module is not responding";
        case GSM_FAULT_SIM_ABSENT:
            return "No SIM card detected";
        case GSM_FAULT_SIM_LOCKED:
            return "SIM card is PIN locked";
        case GSM_FAULT_SIM_FAILURE:
            return "SIM card is present but faulty";
        case GSM_FAULT_NO_SIGNAL:
            return "No mobile signal at all";
        case GSM_FAULT_WEAK_SIGNAL:
            return "Mobile signal is too weak - connection will drop randomly";
        case GSM_FAULT_SIM_BARRED:
            return "Network REFUSED this SIM card";
        case GSM_FAULT_NO_COVERAGE:
            return "No mobile network found in this area";
        case GSM_FAULT_NO_DATA_LINK:
            return "Registered on network but could not get an IP address";
        case GSM_FAULT_NO_INTERNET:
            return "Got an IP address but no data is flowing";
        default:
            return "Unknown fault";
    }
}

const char *gsm_fault_action(gsm_fault_t f)
{
    switch (f) {
        case GSM_FAULT_NONE:
            return "No action needed";
        case GSM_FAULT_MODEM_DEAD:
            return "Check module power (3.8-4.2V) and the TX/RX wiring";
        case GSM_FAULT_SIM_ABSENT:
            return "Insert a SIM card into the holder";
        case GSM_FAULT_SIM_LOCKED:
            return "Put the SIM in a phone and switch the PIN lock OFF";
        case GSM_FAULT_SIM_FAILURE:
            return "Clean the SIM contacts, reseat it, or try another SIM";
        case GSM_FAULT_NO_SIGNAL:
            return "Antenna is disconnected - reconnect it";
        case GSM_FAULT_WEAK_SIGNAL:
            return "Check the antenna is screwed on firmly, or move the device "
                   "near a window / outside";
        case GSM_FAULT_SIM_BARRED:
            return "Call the carrier - the SIM may be barred or the bill unpaid";
        case GSM_FAULT_NO_COVERAGE:
            return "Move to an area with network coverage, or check the antenna";
        case GSM_FAULT_NO_DATA_LINK:
            return "Set the correct APN from the mobile app (apn_config)";
        case GSM_FAULT_NO_INTERNET:
            return "Recharge the data pack - the SIM has no active data";
        default:
            return "Contact support";
    }
}

/* Print the outcome so it reads the same in the console and in a saved log. */
static void gsm_report_fault(gsm_fault_t f)
{
    if (f == GSM_FAULT_NONE) {
        ESP_LOGI(TAG, ">>> ALL OK - GSM connected and internet working <<<");
        return;
    }
    ESP_LOGE(TAG, ">>> PROBLEM: %s", gsm_fault_problem(f));
    ESP_LOGE(TAG, ">>> ACTION : %s", gsm_fault_action(f));
}

gsm_fault_t gsm_diagnose(bool check_internet)
{
    ESP_LOGI(TAG, "======== GSM DIAGNOSTIC ========");

    /* While PPP is up the modem is in DATA mode: the UART carries PPP frames,
     * so an AT command gets no reply. Probing anyway would report "modem not
     * responding" on a device whose internet is demonstrably working - and
     * worse, the AT bytes would be injected into the data stream.
     *
     * A live data link is itself proof of stages 1-5: the modem answered, the
     * SIM is valid, the network registered, and an IP was assigned. So report
     * those from what we already know and go straight to the traffic test. */
    if (gsm_pdp_is_active()) {
        ESP_LOGI(TAG, "[1/6] MODEM ......... OK (%s, in data mode)",
                 s_module_info[0] ? s_module_info : "responding");
        ESP_LOGI(TAG, "[2/6] SIM ........... OK (data link proves it)");
        ESP_LOGI(TAG, "[3/6] SIGNAL ........ OK (data link proves it)");
        ESP_LOGI(TAG, "[4/6] NETWORK ....... OK (registered)");
        ESP_LOGI(TAG, "[5/6] DATA .......... OK (IP assigned)");

        if (!check_internet) {
            ESP_LOGI(TAG, "[6/6] INTERNET ...... skipped");
            ESP_LOGI(TAG, "================================");
            return GSM_FAULT_NONE;
        }
        if (gsm_test_ping("8.8.8.8", 3) != ESP_OK) {
            ESP_LOGE(TAG, "[6/6] INTERNET ...... NO TRAFFIC");
            gsm_report_fault(GSM_FAULT_NO_INTERNET);
            ESP_LOGI(TAG, "================================");
            return GSM_FAULT_NO_INTERNET;
        }
        ESP_LOGI(TAG, "[6/6] INTERNET ...... OK");
        gsm_report_fault(GSM_FAULT_NONE);
        ESP_LOGI(TAG, "================================");
        return GSM_FAULT_NONE;
    }

    /* --- [1/6] Modem ------------------------------------------------------ */
    if (s_dce == NULL || !gsm_is_alive()) {
        ESP_LOGE(TAG, "[1/6] MODEM ......... FAILED");
        gsm_report_fault(GSM_FAULT_MODEM_DEAD);
        return GSM_FAULT_MODEM_DEAD;
    }
    ESP_LOGI(TAG, "[1/6] MODEM ......... OK (%s)",
             s_module_info[0] ? s_module_info : "responding");

    /* --- [2/6] SIM -------------------------------------------------------- */
    gsm_sim_status_t sim = gsm_get_sim_status();
    if (sim != GSM_SIM_READY) {
        gsm_fault_t f = (sim == GSM_SIM_ABSENT)       ? GSM_FAULT_SIM_ABSENT
                      : (sim == GSM_SIM_PIN_REQUIRED) ? GSM_FAULT_SIM_LOCKED
                                                      : GSM_FAULT_SIM_FAILURE;
        ESP_LOGE(TAG, "[2/6] SIM ........... FAILED (%s)",
                 gsm_sim_status_str(sim));
        gsm_report_fault(f);
        return f;
    }
    {
        char iccid[24] = {0};
        if (gsm_get_iccid(iccid, sizeof(iccid)) == ESP_OK) {
            ESP_LOGI(TAG, "[2/6] SIM ........... OK (%s)", iccid);
        } else {
            ESP_LOGI(TAG, "[2/6] SIM ........... OK");
        }
    }

    /* --- [3/6] Signal ----------------------------------------------------- */
    uint8_t rssi = 99, ber = 99;
    if (gsm_get_signal_strength(&rssi, &ber) != ESP_OK || rssi == 99) {
        ESP_LOGE(TAG, "[3/6] SIGNAL ........ NONE (csq 99)");
        gsm_report_fault(GSM_FAULT_NO_SIGNAL);
        return GSM_FAULT_NO_SIGNAL;
    }
    if (rssi < GSM_RSSI_WEAK_THRESHOLD) {
        /* Not a hard failure - it may still connect - but it is the fault that
         * causes intermittent drops nobody can explain, so name it plainly. */
        ESP_LOGW(TAG, "[3/6] SIGNAL ........ WEAK (csq %d of 31)", rssi);
        gsm_report_fault(GSM_FAULT_WEAK_SIGNAL);
        return GSM_FAULT_WEAK_SIGNAL;
    }
    ESP_LOGI(TAG, "[3/6] SIGNAL ........ OK (csq %d of 31, %d bars)",
             rssi, (rssi >= 25) ? 5 : (rssi >= 19) ? 4 : (rssi >= 13) ? 3 : 2);

    /* --- [4/6] Network registration --------------------------------------- */
    gsm_network_status_t net = GSM_NET_UNKNOWN;
    gsm_get_network_status(&net);

    if (net == GSM_NET_DENIED) {
        /* The distinction that matters: the network SAW this SIM and refused
         * it. Reported as "not registered" this looks like an antenna fault. */
        ESP_LOGE(TAG, "[4/6] NETWORK ....... DENIED (creg 3)");
        gsm_report_fault(GSM_FAULT_SIM_BARRED);
        return GSM_FAULT_SIM_BARRED;
    }
    if (net != GSM_NET_REGISTERED_HOME && net != GSM_NET_REGISTERED_ROAMING) {
        ESP_LOGE(TAG, "[4/6] NETWORK ....... NOT REGISTERED (creg %d)", (int)net);
        gsm_report_fault(GSM_FAULT_NO_COVERAGE);
        return GSM_FAULT_NO_COVERAGE;
    }
    ESP_LOGI(TAG, "[4/6] NETWORK ....... OK (%s)",
             (net == GSM_NET_REGISTERED_ROAMING) ? "registered, roaming"
                                                 : "registered, home");

    /* --- [5/6] Data link -------------------------------------------------- */
    if (!gsm_pdp_is_active()) {
        ESP_LOGE(TAG, "[5/6] DATA .......... NO IP ADDRESS");
        gsm_report_fault(GSM_FAULT_NO_DATA_LINK);
        return GSM_FAULT_NO_DATA_LINK;
    }
    ESP_LOGI(TAG, "[5/6] DATA .......... OK (IP assigned)");

    /* --- [6/6] Internet --------------------------------------------------- */
    if (!check_internet) {
        ESP_LOGI(TAG, "[6/6] INTERNET ...... skipped");
        ESP_LOGI(TAG, "================================");
        return GSM_FAULT_NONE;
    }
    if (gsm_test_ping("8.8.8.8", 3) != ESP_OK) {
        ESP_LOGE(TAG, "[6/6] INTERNET ...... NO TRAFFIC");
        gsm_report_fault(GSM_FAULT_NO_INTERNET);
        return GSM_FAULT_NO_INTERNET;
    }
    ESP_LOGI(TAG, "[6/6] INTERNET ...... OK");

    gsm_report_fault(GSM_FAULT_NONE);
    ESP_LOGI(TAG, "================================");
    return GSM_FAULT_NONE;
}

/* ============================================================================
 * Connectivity self-test (Step 1 definition-of-done)
 * ============================================================================
 * These use the STANDARD lwIP and ESP-IDF APIs on purpose - esp_ping and an
 * unmodified esp_http_client, with no modem-specific transport and no AT+Q...
 * anywhere in the path. That is the whole point: if the PPP port is real,
 * ordinary socket code works unchanged. If any of this needed a modem-aware
 * shim, the interface would be an AT wrapper wearing a socket costume.
 * ===========================================================================*/

typedef struct {
    uint32_t          transmitted;
    uint32_t          received;
    uint32_t          total_time_ms;
    SemaphoreHandle_t done;
} gsm_ping_ctx_t;

static void gsm_ping_on_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t  ttl;
    uint16_t seqno;
    uint32_t elapsed_ms;
    ip_addr_t target;

    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO,   &seqno,      sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL,     &ttl,        sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR,  &target,     sizeof(target));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_ms, sizeof(elapsed_ms));

    ESP_LOGI(TAG, "  reply from %s: seq=%d ttl=%d time=%" PRIu32 " ms",
             ipaddr_ntoa(&target), seqno, ttl, elapsed_ms);
}

static void gsm_ping_on_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    ESP_LOGW(TAG, "  seq=%d TIMEOUT", seqno);
}

static void gsm_ping_on_end(esp_ping_handle_t hdl, void *args)
{
    gsm_ping_ctx_t *ctx = (gsm_ping_ctx_t *)args;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &ctx->transmitted,
                         sizeof(ctx->transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &ctx->received,
                         sizeof(ctx->received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &ctx->total_time_ms,
                         sizeof(ctx->total_time_ms));
    xSemaphoreGive(ctx->done);
}

esp_err_t gsm_test_ping(const char *host, uint32_t count)
{
    if (!gsm_pdp_is_active()) {
        ESP_LOGE(TAG, "ping: no data link");
        return ESP_ERR_INVALID_STATE;
    }
    if (host == NULL) host = "8.8.8.8";
    if (count == 0)   count = 4;

    ip_addr_t target;
    if (!ipaddr_aton(host, &target)) {
        ESP_LOGE(TAG, "ping: bad address '%s'", host);
        return ESP_ERR_INVALID_ARG;
    }

    gsm_ping_ctx_t ctx = {0};
    ctx.done = xSemaphoreCreateBinary();
    if (ctx.done == NULL) return ESP_ERR_NO_MEM;

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr = target;
    cfg.count       = count;
    cfg.timeout_ms  = 5000;
    cfg.interval_ms = 1000;

    esp_ping_callbacks_t cbs = {
        .on_ping_success = gsm_ping_on_success,
        .on_ping_timeout = gsm_ping_on_timeout,
        .on_ping_end     = gsm_ping_on_end,
        .cb_args         = &ctx,
    };

    esp_ping_handle_t ping;
    esp_err_t err = esp_ping_new_session(&cfg, &cbs, &ping);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ping: session create failed 0x%x", err);
        vSemaphoreDelete(ctx.done);
        return err;
    }

    ESP_LOGI(TAG, "=== PING %s (%" PRIu32 " packets, lwIP not AT+QPING) ===",
             host, count);
    esp_ping_start(ping);

    /* count * (interval + timeout) plus slack */
    TickType_t wait = pdMS_TO_TICKS(count * 6000 + 5000);
    if (xSemaphoreTake(ctx.done, wait) != pdTRUE) {
        ESP_LOGW(TAG, "ping: did not finish in time");
    }
    esp_ping_stop(ping);
    esp_ping_delete_session(ping);
    vSemaphoreDelete(ctx.done);

    uint32_t loss = (ctx.transmitted > 0)
                    ? (100 * (ctx.transmitted - ctx.received)) / ctx.transmitted
                    : 100;

    ESP_LOGI(TAG, "=== PING RESULT: %" PRIu32 " sent, %" PRIu32 " received, "
                  "%" PRIu32 "%% loss, %" PRIu32 " ms total ===",
             ctx.transmitted, ctx.received, loss, ctx.total_time_ms);

    return (ctx.received > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t gsm_test_http_get(const char *url)
{
    if (!gsm_pdp_is_active()) {
        ESP_LOGE(TAG, "http: no data link");
        return ESP_ERR_INVALID_STATE;
    }
    if (url == NULL) url = "http://example.com";

    ESP_LOGI(TAG, "=== HTTP GET %s (stock esp_http_client) ===", url);

    /* Deliberately the plain ESP-IDF client with default transport. No modem
     * special-casing: this is the test that separates a real socket interface
     * from an AT wrapper. */
    esp_http_client_config_t cfg = {
        .url         = url,
        .method      = HTTP_METHOD_GET,
        .timeout_ms  = 15000,
        .buffer_size = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "http: client init failed");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int len    = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "=== HTTP RESULT: status=%d, content-length=%d ===",
                 status, len);
        if (status < 200 || status >= 400) err = ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "=== HTTP FAILED: %s ===", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

/**
 * @brief Set the APN explicitly (from BLE/USB console) and bring the link up.
 *
 * Stores the APN first, so a reboot uses it directly. On failure the stored
 * value is kept - the user set it deliberately, so it is not silently discarded;
 * the caller reports the failure instead.
 */
esp_err_t gsm_apn_set_and_connect(const char *apn)
{
    if (apn == NULL || apn[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (s_dce == NULL) return ESP_ERR_INVALID_STATE;

    esp_err_t err = gsm_apn_store(apn);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not persist APN '%s': 0x%x", apn, err);
    }

    /* Not a first attempt: the caller may be switching APN while a previous
     * session is up, so clean up first. */
    return gsm_ppp_try_apn(apn, false);
}

/** Return the modem to pure command mode, dropping the data link. */
esp_err_t gsm_ppp_stop(void)
{
    if (s_dce == NULL) return ESP_ERR_INVALID_STATE;

    /* Bring the netif down BEFORE the mode switch, so lwIP stops handing PPP
     * frames to a modem that is no longer in data mode. Doing it the other way
     * round leaves esp_modem's worker processing a stream that has changed
     * meaning underneath it. */
    if (s_ppp_netif) {
        esp_netif_action_stop(s_ppp_netif, NULL, 0, NULL);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    s_ppp_got_ip = false;

    esp_err_t err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_modem_set_mode(COMMAND) failed: 0x%x", err);
    }
    vTaskDelay(pdMS_TO_TICKS(300));    /* let the worker settle */
    return err;
}

esp_err_t gsm_power_on(void)
{
    ESP_LOGI(TAG, "Power ON sequence...");

    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(GSM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    ESP_LOGI(TAG, "Waiting for module boot...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    return ESP_OK;
}

esp_err_t gsm_power_off(void)
{
    ESP_LOGI(TAG, "Power OFF sequence...");

    gpio_set_level(GSM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Power OFF complete");

    return ESP_OK;
}

esp_err_t gsm_reset(void)
{
    ESP_LOGI(TAG, "Hardware RESET (assert=%d, release=%d)...",
             GSM_RST_ASSERT, GSM_RST_RELEASE);

    gpio_set_level(GSM_RST_PIN, GSM_RST_ASSERT);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(GSM_RST_PIN, GSM_RST_RELEASE);
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "RESET complete");
    return ESP_OK;
}

/**
 * @brief Send one AT command through esp_modem and capture the reply.
 *
 * Signature and semantics are unchanged from the raw-UART version, so every
 * caller (the four diagnostic helpers, gsm_task.c, cmd_parser) works as before.
 * Only the transport underneath changed: esp_modem owns the UART, and this call
 * is safe while PPP is up because esp_modem multiplexes it (CMUX) instead of
 * stealing bytes from the data stream.
 *
 * INPUT:
 * @param command       AT command WITHOUT the "AT" prefix (e.g. "+CSQ")
 * @param response      optional buffer for the reply; may be NULL
 * @param response_size size of response
 * @param timeout_ms    how long to wait for the reply
 *
 * OUTPUT:
 * @return ESP_OK on a successful command, ESP_ERR_TIMEOUT on no/failed reply,
 *         ESP_ERR_INVALID_STATE before gsm_init().
 */
esp_err_t gsm_send_at_command(const char *command, char *response,
                              size_t response_size, uint32_t timeout_ms)
{
    if (s_dce == NULL) return ESP_ERR_INVALID_STATE;
    if (command == NULL) return ESP_ERR_INVALID_ARG;

    /* The mutex still serialises callers against each other. esp_modem is
     * internally thread-safe, but keeping the mutex preserves the previous
     * ordering guarantees for multi-step AT flows. */
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire AT mutex");
        return ESP_ERR_TIMEOUT;
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT%s\r", command);
    ESP_LOGI(TAG, "TX: AT%s", command);

    /* esp_modem always needs somewhere to put the reply, even when the caller
     * passed NULL, so use a scratch buffer in that case.
     *
     * NOTE: esp_modem_at_raw() takes no length argument - it writes into the
     * buffer as a C string. Every caller in this file passes >= 64 bytes and the
     * replies parsed here (+CSQ, +CREG, +QCCID, +CNUM) are far shorter, but a
     * command with a long multi-line reply could overrun a small buffer. Use a
     * local staging buffer sized for the worst case, then copy back bounded. */
    char  staging[512];
    char  scratch[256];
    char *dst = (response && response_size > 0) ? response : scratch;

    staging[0] = '\0';
    esp_err_t ret = esp_modem_at_raw(s_dce, cmd, staging, "OK", "ERROR", timeout_ms);

    /* Bounded copy back into the caller's buffer. */
    size_t dst_size = (response && response_size > 0) ? response_size : sizeof(scratch);
    strncpy(dst, staging, dst_size - 1);
    dst[dst_size - 1] = '\0';

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RX: %s", dst);
    } else {
        ESP_LOGW(TAG, "AT%s failed (0x%x): %s", command, ret, dst);
        /* Preserve the old contract: callers check for ESP_OK, and several of
         * them then strstr() the buffer, so a failed command must not look
         * like a successful one with stale content. */
        if (ret != ESP_ERR_TIMEOUT) ret = ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

static bool gsm_check_at_response(void)
{
    if (s_dce == NULL) return false;
    /* esp_modem_sync() is the "AT" -> "OK" round trip. */
    return (esp_modem_sync(s_dce) == ESP_OK);
}

bool gsm_is_alive(void)
{
    return gsm_check_at_response();
}

esp_err_t gsm_init(void)
{
    if (s_initialized) return ESP_OK;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GSM Module Initialization");
    ESP_LOGI(TAG, "========================================");

    if (s_uart_mutex == NULL) {
        s_uart_mutex = xSemaphoreCreateMutex();
        if (s_uart_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create UART mutex");
            return ESP_FAIL;
        }
    }

    gsm_gpio_init();

    /* esp_netif + the default event loop must exist before esp_modem creates
     * the PPP interface. Both are idempotent: ESP_ERR_INVALID_STATE just means
     * another component already did it. */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: 0x%x", err);
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: 0x%x", err);
        return err;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP,
                                               &gsm_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP,
                                               &gsm_ip_event_handler, NULL));
    /* NETIF_PPP_STATUS carries the keepalive/phase events - ESP_EVENT_ANY_ID
     * because the error codes and phase codes share this base. */
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                                               &gsm_ip_event_handler, NULL));

    if (gsm_modem_init() != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Waiting 2s for stabilization...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Probe-1: module already running? */
    ESP_LOGI(TAG, "Checking if module already running...");
    for (int i = 0; i < 3; i++) {
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module already ON!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Probe-2: try power-on */
    ESP_LOGI(TAG, "Module not responding, trying power-on...");
    gsm_report_line_state();      /* one summary line, not a break flood */
    gsm_power_on();

    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "AT test %d/5 after power-on", i + 1);
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module responding after power-on!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Probe-3: try hardware reset */
    ESP_LOGW(TAG, "Still no response, trying hardware reset...");
    gsm_reset();

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "AT test %d/3 after reset", i + 1);
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module responding after reset!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "FAILED to communicate with module!");
    ESP_LOGE(TAG, "Check wiring:");
    ESP_LOGE(TAG, "  ESP TX  (GPIO%d) -> Module RX",     GSM_UART_TX_PIN);
    ESP_LOGE(TAG, "  ESP RX  (GPIO%d) -> Module TX",     GSM_UART_RX_PIN);
    ESP_LOGE(TAG, "  ESP PWR (GPIO%d) -> Module PWRKEY", GSM_PWRKEY_PIN);
    ESP_LOGE(TAG, "  ESP RST (GPIO%d) -> Module RESET",  GSM_RST_PIN);
    ESP_LOGE(TAG, "  Common GND, Module power: 3.8-4.2V");
    ESP_LOGE(TAG, "  If reset doesn't pulse, try flipping NCLE_GSM_RST_INVERTED");
    ESP_LOGE(TAG, "========================================");

    gsm_modem_deinit();
    if (s_uart_mutex) {
        vSemaphoreDelete(s_uart_mutex);
        s_uart_mutex = NULL;
    }
    return ESP_FAIL;
}

esp_err_t gsm_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    /* Leave data mode BEFORE anything else touches the modem.
     *
     * gsm_modem_deinit() below calls esp_modem_destroy(), and destroying the
     * DCE while its worker is still parsing PPP frames is the documented
     * crash on this board - an InstructionFetchError, seen during failure
     * testing when a teardown ran mid-session. gsm_ppp_stop() does the
     * ordering that avoids it: netif down, settle, then back to command mode.
     *
     * This costs nothing when PPP is not up (it returns INVALID_STATE on a
     * null DCE) and it matters most in the case that is about to become
     * routine: the operator switching to wifi while the GSM link is live. */
    if (gsm_pdp_is_active()) {
        ESP_LOGI(TAG, "deinit: leaving data mode before teardown");
        gsm_ppp_stop();
    }

    gsm_power_off();
    gsm_modem_deinit();
    s_initialized = false;

    if (s_uart_mutex) {
        vSemaphoreDelete(s_uart_mutex);
        s_uart_mutex = NULL;
    }
    return ESP_OK;
}

esp_err_t gsm_get_module_info(char *info, size_t info_size)
{
    if (info == NULL || info_size == 0) return ESP_ERR_INVALID_ARG;

    char response[256];
    if (gsm_send_at_command("I", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *start = strchr(response, '\n');
    if (start) {
        start++;
        char *end = strstr(start, "\r\n");
        if (end) {
            size_t len = end - start;
            if (len < info_size) {
                memcpy(info, start, len);
                info[len] = '\0';
                strncpy(s_module_info, info, sizeof(s_module_info) - 1);
                return ESP_OK;
            }
        }
    }
    strncpy(info, response, info_size - 1);
    info[info_size - 1] = '\0';
    strncpy(s_module_info, info, sizeof(s_module_info) - 1);
    return ESP_OK;
}

esp_err_t gsm_get_signal_strength(uint8_t *rssi, uint8_t *ber)
{
    if (rssi == NULL || ber == NULL) return ESP_ERR_INVALID_ARG;

    char response[64];
    if (gsm_send_at_command("+CSQ", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *p = strstr(response, "+CSQ:");
    if (p && sscanf(p, "+CSQ: %hhu,%hhu", rssi, ber) == 2) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t gsm_get_network_status(gsm_network_status_t *status)
{
    if (status == NULL) return ESP_ERR_INVALID_ARG;

    char response[64];
    if (gsm_send_at_command("+CREG?", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *p = strstr(response, "+CREG:");
    int n, stat;
    if (p && sscanf(p, "+CREG: %d,%d", &n, &stat) == 2) {
        *status = (gsm_network_status_t)stat;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t gsm_get_phone_number(char *number, size_t number_size)
{
    if (number == NULL || number_size == 0) return ESP_ERR_INVALID_ARG;
    number[0] = '\0';

    char response[128];
    if (gsm_send_at_command("+CNUM", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Format: +CNUM: "<alpha>","<number>",<type>
     * Examples: +CNUM: "","+918123456789",145
     *           +CNUM: "","918123456789",129
     * If the SIM has no MSISDN, response is just "OK" with no +CNUM line. */
    char *p = strstr(response, "+CNUM:");
    if (p == NULL) {
        ESP_LOGW(TAG, "CNUM: SIM has no MSISDN provisioned");
        return ESP_FAIL;
    }

    /* Skip the first quoted field (alpha tag), find the second quoted field (number) */
    char *first_comma = strchr(p, ',');
    if (first_comma == NULL) return ESP_FAIL;
    char *num_start = strchr(first_comma, '"');
    if (num_start == NULL) return ESP_FAIL;
    num_start++;
    char *num_end = strchr(num_start, '"');
    if (num_end == NULL) return ESP_FAIL;

    size_t len = num_end - num_start;
    if (len >= number_size) len = number_size - 1;
    memcpy(number, num_start, len);
    number[len] = '\0';
    return ESP_OK;
}

esp_err_t gsm_get_iccid(char *iccid, size_t iccid_size)
{
    if (iccid == NULL || iccid_size == 0) return ESP_ERR_INVALID_ARG;
    iccid[0] = '\0';

    char response[128];
    if (gsm_send_at_command("+QCCID", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Format: +QCCID: <iccid_digits>
     * Example: +QCCID: 8991100123456789012F     */
    char *p = strstr(response, "+QCCID:");
    if (p == NULL) return ESP_FAIL;

    /* Skip "+QCCID:" prefix and any whitespace */
    p += strlen("+QCCID:");
    while (*p == ' ' || *p == '\t') p++;

    /* Read digits/hex chars until \r or \n */
    size_t i = 0;
    while (*p && *p != '\r' && *p != '\n' && i < iccid_size - 1) {
        iccid[i++] = *p++;
    }
    iccid[i] = '\0';
    return (i > 0) ? ESP_OK : ESP_FAIL;
}

/* ============================================================================
 * LEGACY AT-command data path (PDP + QHTTP + QPING)
 * ============================================================================
 * SUPERSEDED by the PPP path above. Kept, per the Step 1 brief, until PPP is
 * proven on hardware - but compiled out by default.
 *
 * WHY COMPILED OUT AND NOT MERELY UNUSED:
 * these functions drive the modem with raw uart_write_bytes()/uart_read_bytes()
 * on GSM_UART_NUM, because the QHTTPURL/QHTTPPOST flow needs a CONNECT prompt
 * followed by raw payload bytes, which the line-oriented AT API cannot express.
 * esp_modem now owns that UART. Calling them would put two readers on one RX
 * FIFO: the HTTP helper would steal PPP frames and PPP would steal the modem's
 * replies. That corruption is intermittent and would NOT fail the build - which
 * is exactly the class of fault this port is meant to eliminate.
 *
 * To use them, the modem must first be returned to command mode with
 * gsm_ppp_stop(), and even then they bypass esp_modem's UART ownership.
 * Enable CONFIG_NCLE_GSM_LEGACY_AT_HTTP only for A/B comparison against the
 * PPP path, never in a build that also brings PPP up.
 * ===========================================================================*/
#ifdef CONFIG_NCLE_GSM_LEGACY_AT_HTTP

static bool s_pdp_active = false;

/* Read until a needle string appears, or timeout. Returns total bytes read. */
static int gsm_read_until(const char *needle, char *buf, size_t bufsz, uint32_t timeout_ms)
{
    int total = 0;
    TickType_t start = xTaskGetTickCount();
    size_t needle_len = strlen(needle);

    while (total < (int)(bufsz - 1)) {
        TickType_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) break;

        int len = uart_read_bytes(GSM_UART_NUM,
                                  (uint8_t *)(buf + total),
                                  bufsz - 1 - total,
                                  pdMS_TO_TICKS(200));
        if (len > 0) {
            total += len;
            buf[total] = '\0';
            if (total >= (int)needle_len && strstr(buf, needle) != NULL) {
                return total;
            }
        }
    }
    buf[total] = '\0';
    return total;
}

esp_err_t gsm_pdp_activate(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    char cmd[128];
    char resp[256];

    /* Step 1: Check if context 1 is already active (e.g. from a previous
     * session that left the modem powered with PDP up). The modem persists
     * PDP state across ESP reboots if it stays powered.
     *
     * Expected format when active:
     *   +QIACT: <ctx>,<state>,<ctx_type>,"<ip>"
     * where state=1 means activated. */
    if (gsm_send_at_command("+QIACT?", resp, sizeof(resp), 2000) == ESP_OK) {
        if (strstr(resp, "+QIACT: 1,1,") != NULL) {
            ESP_LOGI(TAG, "PDP context 1 already active, reusing");
            s_pdp_active = true;
            return ESP_OK;
        }
    }

    /* Step 2: Best-effort cleanup of any half-state on context 1.
     * If it was never active this is a no-op; if it was in a stale state
     * this clears it. We ignore the result either way. */
    gsm_send_at_command("+QIDEACT=1", resp, sizeof(resp), 5000);

    /* Step 3: Configure PDP context 1: IPv4, APN, no auth */
    snprintf(cmd, sizeof(cmd), "+QICSGP=1,1,\"%s\",\"\",\"\",1", CONFIG_NCLE_GSM_APN);
    ESP_LOGI(TAG, "Configuring PDP context with APN=%s", CONFIG_NCLE_GSM_APN);
    if (gsm_send_at_command(cmd, resp, sizeof(resp), 2000) != ESP_OK) {
        ESP_LOGE(TAG, "QICSGP: no response");
        return ESP_FAIL;
    }
    if (strstr(resp, "OK") == NULL) {
        ESP_LOGE(TAG, "QICSGP returned: %s", resp);
        return ESP_FAIL;
    }

    /* Step 4: Activate PDP context (can take up to 150s per Quectel doc; cap at 30s) */
    ESP_LOGI(TAG, "Activating PDP context (up to 30s)...");
    if (gsm_send_at_command("+QIACT=1", resp, sizeof(resp), 30000) != ESP_OK) {
        ESP_LOGE(TAG, "QIACT: no response");
        return ESP_FAIL;
    }
    if (strstr(resp, "OK") == NULL) {
        ESP_LOGE(TAG, "QIACT did not return OK: %s", resp);
        return ESP_FAIL;
    }

    /* Step 5: Confirm with IP query */
    if (gsm_send_at_command("+QIACT?", resp, sizeof(resp), 2000) == ESP_OK) {
        ESP_LOGI(TAG, "PDP active: %s", resp);
    }

    s_pdp_active = true;
    return ESP_OK;
}

esp_err_t gsm_pdp_deactivate(void)
{
    if (!s_initialized || !s_pdp_active) return ESP_OK;
    char resp[64];
    gsm_send_at_command("+QIDEACT=1", resp, sizeof(resp), 5000);
    s_pdp_active = false;
    return ESP_OK;
}

/* Configure HTTP context. Detects HTTPS from URL scheme and sets SSL appropriately. */
static esp_err_t gsm_http_configure(bool is_https)
{
    char resp[128];

    if (gsm_send_at_command("+QHTTPCFG=\"contextid\",1", resp, sizeof(resp), 2000) != ESP_OK)
        return ESP_FAIL;
    if (gsm_send_at_command("+QHTTPCFG=\"responseheader\",0", resp, sizeof(resp), 2000) != ESP_OK)
        return ESP_FAIL;

    if (is_https) {
        if (gsm_send_at_command("+QHTTPCFG=\"sslctxid\",1", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
        /* TLS 1.2 */
        if (gsm_send_at_command("+QSSLCFG=\"sslversion\",1,4", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
        /* All ciphers */
        if (gsm_send_at_command("+QSSLCFG=\"ciphersuite\",1,0xFFFF", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#ifdef CONFIG_NCLE_GSM_HTTP_INSECURE
        /* seclevel=0: no certificate verification (testing only) */
        if (gsm_send_at_command("+QSSLCFG=\"seclevel\",1,0", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#else
        /* seclevel=1: verify server cert against installed CA */
        if (gsm_send_at_command("+QSSLCFG=\"seclevel\",1,1", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#endif
    }
    return ESP_OK;
}

/* Set the URL for the next HTTP operation.
 * QHTTPURL flow: AT+QHTTPURL=<len>,80 -> "CONNECT" -> raw URL bytes -> "OK". */
static esp_err_t gsm_http_set_url(const char *url)
{
    char cmd[64];
    char buf[512];
    int url_len = (int)strlen(url);

    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", url_len);
    ESP_LOGI(TAG, "TX: AT+QHTTPURL=%d,80 (url='%s')", url_len, url);
    uart_flush_input(GSM_UART_NUM);    /* clear stale RX bytes */
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    /* Wait for CONNECT prompt (up to 5s) */
    int len = gsm_read_until("CONNECT", buf, sizeof(buf), 5000);
    if (len <= 0 || strstr(buf, "CONNECT") == NULL) {
        ESP_LOGE(TAG, "QHTTPURL: no CONNECT (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPURL got CONNECT, sending %d URL bytes", url_len);

    /* Send the URL bytes */
    uart_write_bytes(GSM_UART_NUM, url, url_len);

    /* Wait for OK (give modem time to validate URL — up to 10s) */
    len = gsm_read_until("OK", buf, sizeof(buf), 10000);
    if (len <= 0 || strstr(buf, "OK") == NULL) {
        ESP_LOGE(TAG, "QHTTPURL: no OK after URL (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPURL OK (response: %s)", buf);
    return ESP_OK;
}

/* Execute POST. body may be NULL (then a 0-length body is sent — useful for GET-like POST).
 * QHTTPPOST flow: AT+QHTTPPOST=<len>,80,80 -> "CONNECT" -> body bytes -> "+QHTTPPOST: <err>,<http>,<rxlen>" */
static esp_err_t gsm_http_do_post(const char *body, int *http_code_out)
{
    char cmd[64];
    char buf[1024];
    int body_len = body ? (int)strlen(body) : 0;

    snprintf(cmd, sizeof(cmd), "AT+QHTTPPOST=%d,80,80\r\n", body_len);
    ESP_LOGI(TAG, "TX: AT+QHTTPPOST=%d,80,80 (body_len=%d)", body_len, body_len);
    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int len = gsm_read_until("CONNECT", buf, sizeof(buf), 5000);
    if (len <= 0 || strstr(buf, "CONNECT") == NULL) {
        ESP_LOGE(TAG, "QHTTPPOST: no CONNECT (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPPOST got CONNECT, sending %d body bytes", body_len);

    if (body_len > 0) {
        uart_write_bytes(GSM_UART_NUM, body, body_len);
    }

    /* Wait for the +QHTTPPOST: line within 80s (server response timeout) */
    len = gsm_read_until("+QHTTPPOST:", buf, sizeof(buf), 80000);
    if (len <= 0) {
        ESP_LOGE(TAG, "QHTTPPOST: no response (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    char *p = strstr(buf, "+QHTTPPOST:");
    int err = -1, http_code = -1, rxlen = -1;
    if (p && sscanf(p, "+QHTTPPOST: %d,%d,%d", &err, &http_code, &rxlen) >= 2) {
        ESP_LOGI(TAG, "QHTTPPOST result: err=%d http=%d len=%d", err, http_code, rxlen);
        if (http_code_out) *http_code_out = http_code;
        return (err == 0) ? ESP_OK : ESP_FAIL;
    }
    ESP_LOGE(TAG, "QHTTPPOST: parse failed (%s)", buf);
    return ESP_FAIL;
}

/* QHTTPGET flow: AT+QHTTPGET=80 -> "+QHTTPGET: <err>,<http>,<rxlen>" */
static esp_err_t gsm_http_do_get(int *http_code_out)
{
    char buf[1024];
    ESP_LOGI(TAG, "TX: AT+QHTTPGET=80");
    uart_flush_input(GSM_UART_NUM);
    const char *cmd = "AT+QHTTPGET=80\r\n";
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int len = gsm_read_until("+QHTTPGET:", buf, sizeof(buf), 80000);
    if (len <= 0) {
        ESP_LOGE(TAG, "QHTTPGET: no response (got: %s)", buf);
        return ESP_FAIL;
    }
    char *p = strstr(buf, "+QHTTPGET:");
    int err = -1, http_code = -1, rxlen = -1;
    if (p && sscanf(p, "+QHTTPGET: %d,%d,%d", &err, &http_code, &rxlen) >= 2) {
        ESP_LOGI(TAG, "QHTTPGET result: err=%d http=%d len=%d", err, http_code, rxlen);
        if (http_code_out) *http_code_out = http_code;
        return (err == 0) ? ESP_OK : ESP_FAIL;
    }
    ESP_LOGE(TAG, "QHTTPGET: parse failed (%s)", buf);
    return ESP_FAIL;
}

/* Read the response body via QHTTPREAD.
 *
 * Modem response shape:
 *   AT+QHTTPREAD=80\r\n              (echo)
 *   CONNECT\r\n                      (ready prompt)
 *   <body bytes>\r\n
 *   +QHTTPREAD: 0\r\n                (terminator with result code)
 *   OK\r\n
 *
 * Single-pass read until "+QHTTPREAD:" appears, then parse the body
 * out of the accumulated buffer. Avoids the two-step bug where the
 * first read can consume everything, leaving the second read empty. */
static esp_err_t gsm_http_read_body(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return ESP_OK;
    out[0] = '\0';

    static char buf[2048];   /* static — keep off task stack */
    ESP_LOGI(TAG, "TX: AT+QHTTPREAD=80");
    uart_flush_input(GSM_UART_NUM);
    const char *cmd = "AT+QHTTPREAD=80\r\n";
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    /* Single read: keep reading until the +QHTTPREAD: terminator appears
     * or we hit a 10s timeout. This captures the entire CONNECT+body+
     * terminator sequence in one buffer. */
    int len = gsm_read_until("+QHTTPREAD:", buf, sizeof(buf), 10000);
    if (len <= 0) {
        ESP_LOGW(TAG, "QHTTPREAD: timeout, no terminator received");
        return ESP_OK;
    }

    /* Locate the body between "CONNECT\r\n" and "\r\n+QHTTPREAD:" */
    char *body_start = strstr(buf, "CONNECT\r\n");
    if (body_start == NULL) {
        ESP_LOGW(TAG, "QHTTPREAD: no CONNECT prompt in response");
        return ESP_OK;
    }
    body_start += strlen("CONNECT\r\n");

    char *body_end = strstr(body_start, "\r\n+QHTTPREAD:");
    if (body_end == NULL || body_end <= body_start) {
        ESP_LOGW(TAG, "QHTTPREAD: empty body");
        return ESP_OK;
    }

    size_t body_len = body_end - body_start;
    if (body_len >= out_size) body_len = out_size - 1;
    memcpy(out, body_start, body_len);
    out[body_len] = '\0';
    ESP_LOGI(TAG, "QHTTPREAD: body=%d bytes", (int)body_len);
    /* Print the actual response body content so we can verify what came
     * from the server. Multi-line — server reply often has \n separators. */
    ESP_LOGI(TAG, "============ Server response begin ============");
    ESP_LOGI(TAG, "%s", out);
    ESP_LOGI(TAG, "============= Server response end =============");
    return ESP_OK;
}

esp_err_t gsm_http_post(const char *url, const char *body,
                        int *http_code_out,
                        char *resp_body, size_t resp_body_size)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (url == NULL)    return ESP_ERR_INVALID_ARG;

    /* Activate PDP if not already (uses mutex internally) */
    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    bool is_https = (strncmp(url, "https://", 8) == 0);
    if (gsm_http_configure(is_https) != ESP_OK) return ESP_FAIL;

    /* The URL upload + POST body upload + response body read ALL need raw
     * UART access. Take the mutex once and hold it through the whole flow,
     * otherwise the periodic gsm_task poll can grab the UART between the
     * POST and the QHTTPREAD, corrupting the response body read. */
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        ESP_LOGE(TAG, "http_post: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = gsm_http_set_url(url);
    if (ret == ESP_OK) ret = gsm_http_do_post(body, http_code_out);

    /* Read response body BEFORE releasing the mutex */
    if (ret == ESP_OK && resp_body && resp_body_size > 0) {
        gsm_http_read_body(resp_body, resp_body_size);
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

esp_err_t gsm_http_get(const char *url,
                       int *http_code_out,
                       char *resp_body, size_t resp_body_size)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (url == NULL)    return ESP_ERR_INVALID_ARG;

    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    bool is_https = (strncmp(url, "https://", 8) == 0);
    if (gsm_http_configure(is_https) != ESP_OK) return ESP_FAIL;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = gsm_http_set_url(url);
    if (ret == ESP_OK) ret = gsm_http_do_get(http_code_out);

    /* Read response body BEFORE releasing the mutex (same fix as POST) */
    if (ret == ESP_OK && resp_body && resp_body_size > 0) {
        gsm_http_read_body(resp_body, resp_body_size);
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/* ============================================================================
 * Ping — uses AT+QPING to verify IP reachability (proves data plan works)
 * ===========================================================================*/

/* Returns the LAST "+QPING:" line that has 7 comma-separated fields.
 * That's the SUMMARY line:    +QPING: <result>,<sent>,<rcvd>,<lost>,<min>,<max>,<avg>
 * Per-reply lines have either 6 fields (with quoted IP) or 1 field (timeout). */
static char *gsm_ping_find_summary(char *buf, int buf_len)
{
    char *p = buf;
    char *summary = NULL;
    while ((p = strstr(p, "+QPING:")) != NULL) {
        int r, s, rc, l, mn, mx, avg;
        if (sscanf(p, "+QPING: %d,%d,%d,%d,%d,%d,%d",
                   &r, &s, &rc, &l, &mn, &mx, &avg) == 7) {
            summary = p;
        }
        p++;
    }
    (void)buf_len;
    return summary;
}

esp_err_t gsm_ping(const char *host, uint8_t count, uint16_t timeout_s,
                   gsm_ping_result_t *out)
{
    if (!s_initialized || !out) return ESP_ERR_INVALID_STATE;
    if (host == NULL || host[0] == '\0') host = "8.8.8.8";
    if (count == 0)     count = 4;
    if (timeout_s == 0) timeout_s = 5;

    memset(out, 0, sizeof(*out));
    out->loss_pct = 100;

    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    /* AT+QPING returns "OK" immediately, then emits +QPING: lines asynchronously
     * over the next (count * timeout_s) seconds. We can't use gsm_send_at_command
     * because it returns on the first read gap. Instead, take the mutex and read
     * directly until the summary line appears or the time budget elapses. */
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "ping: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    char cmd[128];
    static char resp[2048];   /* static — keep off task stack */
    snprintf(cmd, sizeof(cmd), "AT+QPING=1,\"%s\",%u,%u\r\n",
             host, (unsigned)timeout_s, (unsigned)count);

    uint32_t budget_ms = (uint32_t)count * (uint32_t)timeout_s * 1000u + 5000u;
    ESP_LOGI(TAG, "Pinging %s (%u pings, %us each, total budget %lus)",
             host, count, timeout_s, (unsigned long)(budget_ms / 1000));
    ESP_LOGI(TAG, "TX: AT+QPING=1,\"%s\",%u,%u", host, timeout_s, count);

    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int total = 0;
    TickType_t start_tick = xTaskGetTickCount();
    bool have_summary = false;

    while (total < (int)(sizeof(resp) - 1) && !have_summary) {
        uint32_t elapsed_ms =
            (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms >= budget_ms) break;

        int len = uart_read_bytes(GSM_UART_NUM,
                                  (uint8_t *)(resp + total),
                                  sizeof(resp) - 1 - total,
                                  pdMS_TO_TICKS(500));
        if (len > 0) {
            total += len;
            resp[total] = '\0';
            /* Early exit when summary line appears */
            if (gsm_ping_find_summary(resp, total) != NULL) {
                have_summary = true;
            }
        }
        /* If len <= 0 we just loop and check the time budget — the modem may
         * still be doing pings between replies. Don't bail on a gap. */
    }

    xSemaphoreGive(s_uart_mutex);

    char *summary = gsm_ping_find_summary(resp, total);
    if (summary == NULL) {
        ESP_LOGE(TAG, "QPING: no summary line after %lu ms (got %d bytes):\n%s",
                 (unsigned long)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS),
                 total, resp);
        return ESP_FAIL;
    }

    int result, sent, rcvd, lost, mn, mx, avg;
    if (sscanf(summary, "+QPING: %d,%d,%d,%d,%d,%d,%d",
               &result, &sent, &rcvd, &lost, &mn, &mx, &avg) != 7) {
        ESP_LOGE(TAG, "QPING: summary parse failed: %s", summary);
        return ESP_FAIL;
    }

    out->sent       = (uint8_t)sent;
    out->received   = (uint8_t)rcvd;
    out->loss_pct   = (sent > 0) ? (uint8_t)((lost * 100) / sent) : 100;
    out->rtt_min_ms = (uint16_t)mn;
    out->rtt_avg_ms = (uint16_t)avg;
    out->rtt_max_ms = (uint16_t)mx;
    out->reachable  = (rcvd > 0);

    ESP_LOGI(TAG, "PING result: sent=%d recv=%d loss=%d%% rtt avg=%dms max=%dms",
             sent, rcvd, out->loss_pct, avg, mx);

    return ESP_OK;
}

#else /* !CONFIG_NCLE_GSM_LEGACY_AT_HTTP */

/* Stubs so callers still link while the legacy AT path is compiled out. They
 * fail loudly rather than silently corrupting the UART: the PPP path replaces
 * them (esp_http_client over the netif, and lwIP ping instead of AT+QPING). */

esp_err_t gsm_pdp_activate(void)
{
    ESP_LOGE(TAG, "gsm_pdp_activate: legacy AT path disabled - use gsm_ppp_start()");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gsm_pdp_deactivate(void)
{
    return ESP_OK;    /* nothing to tear down on the legacy path */
}

esp_err_t gsm_http_get(const char *url, int *http_code_out,
                       char *resp_body, size_t resp_body_size)
{
    (void)url; (void)http_code_out; (void)resp_body; (void)resp_body_size;
    ESP_LOGE(TAG, "gsm_http_get: legacy AT-HTTP disabled - use esp_http_client over PPP");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gsm_http_post(const char *url, const char *body, int *http_code_out,
                        char *resp_body, size_t resp_body_size)
{
    (void)url; (void)body; (void)http_code_out;
    (void)resp_body; (void)resp_body_size;
    ESP_LOGE(TAG, "gsm_http_post: legacy AT-HTTP disabled - use esp_http_client over PPP");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gsm_ping(const char *host, uint8_t count, uint16_t timeout_s,
                   gsm_ping_result_t *out)
{
    (void)host; (void)count; (void)timeout_s;
    if (out) memset(out, 0, sizeof(*out));
    ESP_LOGE(TAG, "gsm_ping: AT+QPING disabled - use the lwIP ping over PPP");
    return ESP_ERR_NOT_SUPPORTED;
}

#endif /* CONFIG_NCLE_GSM_LEGACY_AT_HTTP */

#endif /* CONFIG_NCLE_GSM_ENABLE */
