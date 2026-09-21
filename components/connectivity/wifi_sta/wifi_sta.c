/**
 * @file wifi_sta.c
 * @brief WiFi Station Mode - ESP-IDF WiFi driver wrapper
 *
 * Handles WiFi initialization, connection, disconnection, scanning,
 * and event handling. Uses ESP-IDF WiFi driver in STA mode.
 *
 * Data Flow:
 *   wifi_sta_connect(ssid, pass)
 *     → esp_wifi_connect()
 *       → WIFI_EVENT_STA_CONNECTED
 *         → IP_EVENT_STA_GOT_IP
 *           → state = CONNECTED
 */

#include "wifi_sta.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_coexist.h"
#include <stdio.h>              /* snprintf in the wifi_status handler */
#include "net_link.h"           /* announce this component as a network link */
#include "cmd_parser.h"         /* register our own commands + send_response */
#include "wifi_sta_config.h"    /* ncle_wifi_sta_config_erase */
#include "wifi_sta_task.h"      /* ncle_wifi_sta_provision (test-before-save) */
#ifdef CONFIG_NCLE_MQTT_ENABLE
#include "mqtt_client_svc.h"    /* mqtt_svc_is_connected - the "cloud" field */
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

static const char *TAG = "WIFI_STA";

/* Event group bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* Auto-retry settings */
#define WIFI_MAX_RETRY      3

/* Internal state */
static bool s_initialized = false;
static esp_netif_t *s_netif = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static wifi_sta_status_t s_status = {0};
static SemaphoreHandle_t s_status_mutex = NULL;   /* protects s_status across tasks */
static int s_retry_count = 0;
static esp_event_handler_instance_t s_wifi_handler = NULL;
static esp_event_handler_instance_t s_ip_handler = NULL;

/* =========================================================
 * WiFi Event Handler
 * ========================================================= */

/**
 * @brief WiFi/IP event handler — handles STA connect, disconnect, got-IP events.
 *
 * INPUT:
 * @param arg        - user data (unused)
 * @param event_base - event base (WIFI_EVENT or IP_EVENT)
 * @param event_id   - specific event ID
 * @param event_data - event-specific data
 *
 * OUTPUT: None (updates internal state, sets event group bits)
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA started");
                break;

            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
                ESP_LOGI(TAG, "Connected to AP: %.*s (ch=%d)",
                         event->ssid_len, event->ssid, event->channel);
                s_retry_count = 0;
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "Disconnected from AP (reason=%d)", event->reason);

                /* Always clear connection fields (mutex-protected) */
                if (s_status_mutex) xSemaphoreTake(s_status_mutex, portMAX_DELAY);
                s_status.connected = false;
                memset(s_status.ip, 0, sizeof(s_status.ip));
                s_status.rssi = 0;
                s_status.bars = 0;

                /* Auto-retry — keep CONNECTING state until retries exhausted */
                if (s_retry_count < WIFI_MAX_RETRY) {
                    s_retry_count++;
                    ESP_LOGI(TAG, "Retry %d/%d...", s_retry_count, WIFI_MAX_RETRY);
                    s_status.state = WIFI_STA_STATE_CONNECTING; /* stay CONNECTING */
                    if (s_status_mutex) xSemaphoreGive(s_status_mutex);
                    esp_wifi_connect();
                } else {
                    ESP_LOGW(TAG, "Max retries reached, giving up");
                    s_status.state = WIFI_STA_STATE_DISCONNECTED; /* now truly disconnected */
                    if (s_status_mutex) xSemaphoreGive(s_status_mutex);
                    esp_wifi_disconnect();
                    if (s_wifi_event_group) {
                        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    }
                }
                break;
            }

            default:
                break;
        }
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        if (s_status_mutex) xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        snprintf(s_status.ip, sizeof(s_status.ip), IPSTR, IP2STR(&event->ip_info.ip));
        s_status.connected = true;
        s_status.state = WIFI_STA_STATE_CONNECTED;
        if (s_status_mutex) xSemaphoreGive(s_status_mutex);

        ESP_LOGI(TAG, "Got IP: %s", s_status.ip);

        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/* =========================================================
 * Convert RSSI to signal bars (0-5)
 * ========================================================= */

/**
 * @brief Convert RSSI dBm value to signal bars (0-5).
 *
 * INPUT:
 * @param rssi - RSSI value in dBm
 *
 * OUTPUT:
 * @return signal bars 0-5
 */
static uint8_t rssi_to_bars(int8_t rssi)
{
    if (rssi >= -50) return 5;
    if (rssi >= -60) return 4;
    if (rssi >= -70) return 3;
    if (rssi >= -80) return 2;
    if (rssi >= -90) return 1;
    return 0;
}

/* =========================================================
 * PUBLIC FUNCTIONS
 * ========================================================= */

/**
 * @brief Initialize WiFi in STA mode: netif, event handlers, start WiFi.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success, or error code on failure
 */
/* =========================================================
 * Command handlers — WiFi's own commands
 *
 * These three lived in cmd_parser.c until the layer refactor. They are
 * byte-for-byte the same logic and the SAME response strings, only relocated:
 * the shared parser must not know what WiFi is, so WiFi owns them and registers
 * them itself (see ncle_wifi_sta_init).
 *
 * The `wifi_not_enabled` #else arms are gone deliberately - this file only
 * compiles when CONFIG_NCLE_WIFI_ENABLE is set, so with WiFi disabled the
 * commands are simply never registered and the parser answers
 * `undefined_command`. That is the honest reply for a build with no WiFi in it.
 *
 * cmd_parser owns `root` - handlers must not delete it.
 * ========================================================= */

static void handle_wifi_config(cJSON *root)
{
    cJSON *ssid = cJSON_GetObjectItem(root, KEY_SSID);
    cJSON *pass = cJSON_GetObjectItem(root, KEY_PASSWORD);
    if (!ssid || !cJSON_IsString(ssid) || strlen(ssid->valuestring) == 0) {
        send_response("config_update_failed", STATUS_ERR,
                      "{\"target\":\"wifi\",\"reason\":\"missing_ssid\"}");
        return;
    }

    const char *pw = (pass && cJSON_IsString(pass)) ? pass->valuestring : "";

    if (!s_initialized) {
        /* WiFi is not running - the device is in gsm or off mode. Store the
         * credentials unverified so the operator can enter them BEFORE
         * switching. Without this the device deadlocks: no credentials until
         * WiFi runs, and WiFi cannot usefully run without credentials.
         *
         * "saved" rather than "connecting" in the reply, because nothing has
         * been tested - they are proven or rejected on the switch to wifi
         * mode, and wifi_status reports the outcome. */
        esp_err_t err = ncle_wifi_sta_config_save(ssid->valuestring, pw);
        if (err != ESP_OK) {
            send_response("config_update_failed", STATUS_ERR,
                          "{\"target\":\"wifi\",\"reason\":\"nvs_write_failed\"}");
            return;
        }
        wifi_link_provisioned_changed();
        ESP_LOGI(TAG, "credentials stored for SSID '%s' (WiFi not running - "
                      "untested until the mode is switched)", ssid->valuestring);
        send_response("wifi_config", STATUS_OK, "{\"state\":\"saved\"}");
        return;
    }

    // Test-before-save: try the credentials first; they are saved to
    // NVS only if the connection succeeds, otherwise the device
    // reverts to the previously saved ones.
    ncle_wifi_sta_provision(ssid->valuestring, pw);
    send_response("wifi_config", STATUS_OK, "{\"state\":\"connecting\"}");
}

static void handle_wifi_status(cJSON *root)
{
    (void)root;

    if (!s_initialized) {
        /* Not running - gsm or off mode. Report whether credentials are
         * stored, because that is the one thing the operator needs to know
         * before switching. Reading s_status here would be worse than useless:
         * it is zeroed, so it would claim connected:false on a network that
         * has never been tried. */
        char ssid[WIFI_STA_SSID_MAX + 1] = {0};
        char pw[WIFI_STA_PASSWORD_MAX + 1] = {0};
        bool have = (ncle_wifi_sta_config_load(ssid, pw) == ESP_OK && ssid[0]);
        static char idle_data[96];
        snprintf(idle_data, sizeof(idle_data),
                 "{\"running\":false,\"provisioned\":%s,\"ssid\":\"%s\"}",
                 have ? "true" : "false", have ? ssid : "");
        send_response("wifi_status", STATUS_OK, idle_data);
        return;
    }

    wifi_sta_status_t st;
    ncle_wifi_sta_get_status(&st);
    // "cloud" = broker reachable = the REAL "internet actually works" signal.
    // WiFi 'connected' only means joined-router+got-IP; a router with no
    // internet still reports connected:true but cloud:false.
    bool cloud = false;
#ifdef CONFIG_NCLE_MQTT_ENABLE
    cloud = mqtt_svc_is_connected();
#endif
    static char wifi_data[96];
    snprintf(wifi_data, sizeof(wifi_data),
             "{\"connected\":%s,\"bars\":%d,\"rssi\":%d,\"cloud\":%s}",
             st.connected ? "true" : "false", st.bars, st.rssi,
             cloud ? "true" : "false");
    send_response("wifi_status", STATUS_OK, wifi_data);
}

static void handle_wifi_erase(cJSON *root)
{
    (void)root;
    ncle_wifi_sta_config_erase();
    /* No SSID any more: this becomes an app-only site, so store-and-forward
     * must stop buffering. Without this the cached answer would stay "yes"
     * until the next reboot. */
    wifi_link_provisioned_changed();
    send_response("wifi_erase", STATUS_OK, NULL);
}

/**
 * @brief Describe this link for `diag` (net_link_t::status_json).
 *
 * These are the exact fields diag used to build itself by calling the WiFi API
 * directly. They live here now so diag never has to know what a WiFi link is -
 * a GSM component supplies SIM/signal from its own version of this function and
 * diag is unchanged.
 *
 * INPUT:
 * @param buf, size  destination for a JSON object
 *
 * OUTPUT:
 * @return bytes written (snprintf semantics)
 */
static int wifi_link_status_json(char *buf, size_t size)
{
    wifi_sta_status_t st;
    ncle_wifi_sta_get_status(&st);
    return snprintf(buf, size,
                    "{\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}",
                    st.connected ? "true" : "false", st.ssid, st.ip, st.rssi);
}

/**
 * @brief Has this device been given WiFi credentials? (net_link_t.is_provisioned)
 *
 * "Is an SSID saved", NOT "is WiFi up". Somebody who entered credentials wants
 * the cloud, so a reading taken while the router is down must be buffered. A
 * device that was never provisioned is an app-only site, and buffering there
 * would fill flash with readings nothing will ever collect.
 *
 * Reads NVS rather than the live status so it stays true across an outage, a
 * reboot, and a failed connection attempt.
 */
/* -1 = not looked up yet. Reset to -1 by wifi_link_provisioned_changed() when
 * credentials are saved or erased, so the next reading re-reads NVS. */
static int8_t s_provisioned = -1;

static bool wifi_link_is_provisioned(void)
{
    /* Cached: asked once per reading, and re-opening NVS each time would read
     * flash on the collection path for an answer that only changes on
     * wifi_config / wifi_erase. */
    if (s_provisioned < 0) {
        char ssid[WIFI_STA_SSID_MAX + 1] = {0};
        char pass[WIFI_STA_PASSWORD_MAX + 1] = {0};
        s_provisioned = (ncle_wifi_sta_config_load(ssid, pass) == ESP_OK &&
                         ssid[0] != '\0') ? 1 : 0;
        ESP_LOGI(TAG, "Link provisioned (SSID saved): %s",
                 s_provisioned ? "yes" : "no");
    }
    return (s_provisioned == 1);
}

void wifi_link_provisioned_changed(void)
{
    s_provisioned = -1;
}

esp_err_t ncle_wifi_sta_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "WiFi STA Initialization");
    ESP_LOGI(TAG, "========================================");

    /* Create status mutex for thread-safe access */
    if (!s_status_mutex) {
        s_status_mutex = xSemaphoreCreateMutex();
    }

    /* Create event group */
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    /* Initialize TCP/IP stack (safe to call multiple times) */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create default event loop (safe to call multiple times) */
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create default WiFi STA netif */
    s_netif = esp_netif_create_default_wifi_sta();
    if (!s_netif) {
        ESP_LOGE(TAG, "Failed to create netif");
        return ESP_FAIL;
    }

    /* Initialize WiFi with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register event handlers */
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              wifi_event_handler, NULL, &s_wifi_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler");
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              wifi_event_handler, NULL, &s_ip_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler");
        return ret;
    }

    /* Set WiFi mode to STA */
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Prefer WiFi over BLE for the shared 2.4GHz radio. BLE is only an
     * occasional config channel here; WiFi (cloud data) is the priority.
     * With the default BALANCE, BLE activity starves the WPA2 4-way
     * handshake and connection is slow/unreliable when a phone is
     * connected. Preferring WiFi makes connection fast and stable; BLE
     * config commands (tiny, infrequent) still get through.
     *
     * Kept as PREFER_WIFI on the GSM product too, though BLE matters more
     * here - it is the primary control channel AND the only way to switch a
     * device back to gsm when its router has been replaced. Two reasons not
     * to soften it: the comment above records a measured failure with
     * BALANCE, not a theoretical one; and this line only ever runs while WiFi
     * is initialised, which on this product means only in wifi mode. In gsm
     * and off modes esp_wifi_init() is never called, so the preference is
     * never set and BLE has the radio to itself.
     *
     * The risk that remains is BLE responsiveness while in wifi mode. If an
     * operator cannot reach a wifi-mode device over BLE to switch it back,
     * revisit this - and measure, rather than assuming BALANCE is better. */
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
    esp_err_t coex_ret = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    ESP_LOGI(TAG, "Coexistence preference: WiFi (%s)", esp_err_to_name(coex_ret));
#endif

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = WIFI_STA_STATE_IDLE;

    s_initialized = true;

    /* Commands are NOT registered here - see wifi_commands_register(), called
     * at boot regardless of mode. On the WiFi product registering from inside
     * init was right, because init always ran. Here it would deadlock: the
     * operator cannot enter credentials without wifi_config, wifi_config does
     * not exist until WiFi initialises, and WiFi will not stay up without
     * credentials. */

    /* net_link registration is NOT done here - see wifi_net_link_register().
     * On the WiFi product init ran once, so registering from inside it was
     * safe. Here the operator switches link modes, so init/deinit run
     * repeatedly, and deinit clears s_initialized - which would let a second
     * init register a second time into a 2-slot table that has no unregister. */

    ESP_LOGI(TAG, "WiFi STA initialized (internal radio, no GPIO needed)");

    return ESP_OK;
}

/**
 * @brief Deinitialize WiFi: disconnect, stop, unregister handlers, destroy netif.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK always
 */
esp_err_t ncle_wifi_sta_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    esp_wifi_disconnect();
    esp_wifi_stop();

    if (s_wifi_handler) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_handler);
        s_wifi_handler = NULL;
    }
    if (s_ip_handler) {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_handler);
        s_ip_handler = NULL;
    }

    esp_wifi_deinit();

    if (s_netif) {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_initialized = false;

    ESP_LOGI(TAG, "WiFi STA deinitialized");
    return ESP_OK;
}

/**
 * @brief Connect to WiFi AP with given credentials.
 *
 * INPUT:
 * @param ssid     - SSID of the access point
 * @param password - password (NULL or empty for open network)
 *
 * OUTPUT:
 * @return ESP_OK if connection started, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ncle_wifi_sta_connect(const char *ssid, const char *password)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID cannot be empty");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to: %s", ssid);

    /* Configure WiFi */
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    wifi_config.sta.threshold.authmode = (password && strlen(password) > 0)
                                          ? WIFI_AUTH_WPA2_PSK
                                          : WIFI_AUTH_OPEN;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Save SSID in status (mutex-protected) */
    if (s_status_mutex) xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    strncpy(s_status.ssid, ssid, WIFI_STA_SSID_MAX);
    s_status.ssid[WIFI_STA_SSID_MAX] = '\0';
    s_status.state = WIFI_STA_STATE_CONNECTING;
    s_retry_count = 0;
    if (s_status_mutex) xSemaphoreGive(s_status_mutex);

    /* Clear event group bits */
    if (s_wifi_event_group) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }

    /* Start connection.
     * If the STA is ALREADY associated to an AP, the ESP32 rejects a direct
     * connect ("sta is connected, disconnect before connecting to new ap").
     * So when switching networks (e.g. re-provisioning to a new SSID while
     * connected), trigger a disconnect instead: the DISCONNECTED handler
     * auto-reconnects using the NEW config we just set above. If not currently
     * connected, connect directly. */
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "Already connected - disconnecting to switch to '%s'", ssid);
        ret = esp_wifi_disconnect();   /* handler reconnects to the new config */
    } else {
        ret = esp_wifi_connect();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "connect/disconnect failed: %s", esp_err_to_name(ret));
        s_status.state = WIFI_STA_STATE_IDLE;
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Disconnect from current WiFi AP and prevent auto-retry.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t ncle_wifi_sta_disconnect(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting...");
    s_retry_count = WIFI_MAX_RETRY;  /* Prevent auto-retry */

    esp_err_t ret = esp_wifi_disconnect();

    if (s_status_mutex) xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_status.connected = false;
    s_status.state = WIFI_STA_STATE_IDLE;
    memset(s_status.ip, 0, sizeof(s_status.ip));
    s_status.rssi = 0;
    s_status.bars = 0;
    if (s_status_mutex) xSemaphoreGive(s_status_mutex);

    return ret;
}

/**
 * @brief Scan for WiFi networks (blocking).
 *
 * INPUT:
 * @param results - array to store scan results
 * @param max     - maximum number of results to return
 * @param found   - pointer to store actual number of networks found
 *
 * OUTPUT:
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, or scan error
 */
esp_err_t ncle_wifi_sta_scan(wifi_sta_scan_result_t *results, uint16_t max, uint16_t *found)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!results || !found || max == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Scanning WiFi networks...");

    /* Start scan (blocking) */
    wifi_scan_config_t scan_config = {
        .show_hidden = false,
    };
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
        *found = 0;
        return ret;
    }

    /* Get results */
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    uint16_t to_get = (ap_count < max) ? ap_count : max;
    wifi_ap_record_t *ap_list = calloc(to_get, sizeof(wifi_ap_record_t));
    if (!ap_list) {
        ESP_LOGE(TAG, "Failed to allocate scan buffer");
        esp_wifi_scan_get_ap_records(&to_get, NULL);  /* Clear internal list */
        *found = 0;
        return ESP_ERR_NO_MEM;
    }

    ret = esp_wifi_scan_get_ap_records(&to_get, ap_list);
    if (ret != ESP_OK) {
        free(ap_list);
        *found = 0;
        return ret;
    }

    /* Copy to our result format */
    for (uint16_t i = 0; i < to_get; i++) {
        strncpy(results[i].ssid, (const char *)ap_list[i].ssid, WIFI_STA_SSID_MAX);
        results[i].ssid[WIFI_STA_SSID_MAX] = '\0';
        results[i].rssi = ap_list[i].rssi;
        results[i].channel = ap_list[i].primary;
        results[i].secure = (ap_list[i].authmode != WIFI_AUTH_OPEN);
    }

    free(ap_list);
    *found = to_get;

    ESP_LOGI(TAG, "Scan complete: %d networks found", to_get);
    return ESP_OK;
}

/**
 * @brief Get current WiFi status including live RSSI update.
 *
 * INPUT:
 * @param status - pointer to status structure to fill
 *
 * OUTPUT: None (fills status structure with current WiFi state)
 */
void ncle_wifi_sta_get_status(wifi_sta_status_t *status)
{
    if (!status) return;

    if (s_status_mutex) xSemaphoreTake(s_status_mutex, portMAX_DELAY);

    /* Update RSSI if connected */
    if (s_status.connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            s_status.rssi = ap.rssi;
            s_status.bars = rssi_to_bars(ap.rssi);
        }
    }

    memcpy(status, &s_status, sizeof(wifi_sta_status_t));
    if (s_status_mutex) xSemaphoreGive(s_status_mutex);
}

/**
 * @brief Check if WiFi is currently connected.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if connected, false otherwise
 */
bool ncle_wifi_sta_is_connected(void)
{
    return s_status.connected;
}

void wifi_commands_register(void)
{
    /* Called at boot in EVERY mode, so the operator can enter credentials
     * while the device is still on gsm and then switch to a WiFi that works
     * first time. The handlers check s_initialized and behave sensibly when
     * the stack is down: wifi_config stores without testing, wifi_status
     * reports whether credentials exist rather than reading a zeroed struct.
     *
     * Idempotent because cmd_parser_register refuses duplicates, but the
     * guard keeps the log clean across mode switches. */
    static bool s_cmds_registered = false;
    if (s_cmds_registered) {
        return;
    }
    s_cmds_registered = true;

    cmd_parser_register(CMD_WIFI_CONFIG, handle_wifi_config);
    cmd_parser_register(CMD_WIFI_STATUS, handle_wifi_status);
    cmd_parser_register(CMD_WIFI_ERASE,  handle_wifi_erase);
}

void wifi_net_link_register(void)
{
    /* One-shot, and deliberately outside init().
     *
     * net_link has NET_LINK_MAX == 2 slots and no unregister - registration is
     * append-only. GSM takes one slot, WiFi the other. Since the operator can
     * switch modes any number of times, init() runs repeatedly, and a
     * registration inside it would consume the last free slot on the second
     * switch and then fail for good. The failure is quiet in the worst way:
     * the application only ever asks "is anything up", so a link missing from
     * the table simply reads as permanently offline.
     *
     * Static storage: net_link keeps the pointer, so a stack copy would
     * dangle. Mirrors gsm_net_link_register(). */
    static bool s_registered = false;
    if (s_registered) {
        return;
    }

    static const net_link_t wifi_link = {
        .name           = "wifi",
        .is_up          = ncle_wifi_sta_is_connected,
        .status_json    = wifi_link_status_json,
        .is_provisioned = wifi_link_is_provisioned,
    };

    if (net_link_register(&wifi_link)) {
        s_registered = true;
        ESP_LOGI(TAG, "registered with net_link as '%s'", wifi_link.name);
    } else {
        ESP_LOGE(TAG, "net_link registration FAILED - the application layer "
                      "will believe the device is permanently offline");
    }
}
