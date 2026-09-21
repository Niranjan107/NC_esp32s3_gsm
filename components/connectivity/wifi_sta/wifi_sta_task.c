/**
 * @file wifi_sta_task.c
 * @brief WiFi STA task (CLV4 headless) — boot auto-connect + status notify.
 *
 * Headless CLV4 variant of the Conn_plus master task:
 *  - On start: init WiFi, load creds from NVS (or dev defaults), auto-connect.
 *  - On WiFi state change: invoke a registered status callback (main wires it
 *    to BLE). No SPI/display dependency.
 */
#include "wifi_sta_task.h"
#include "wifi_sta.h"
#include "wifi_sta_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "WIFI_TASK";
static TaskHandle_t   s_task_handle = NULL;
static wifi_status_cb_t s_status_cb = NULL;

/* Pending provision request (test-before-save). Written by
 * ncle_wifi_sta_provision() from another task, consumed by the WiFi task. */
static char s_prov_ssid[WIFI_STA_SSID_MAX + 1];
static char s_prov_pass[WIFI_STA_PASSWORD_MAX + 1];
static volatile bool s_prov_request = false;

void ncle_wifi_sta_provision(const char *ssid, const char *password)
{
    if (!ssid) {
        return;
    }
    strncpy(s_prov_ssid, ssid, WIFI_STA_SSID_MAX);
    s_prov_ssid[WIFI_STA_SSID_MAX] = '\0';
    if (password) {
        strncpy(s_prov_pass, password, WIFI_STA_PASSWORD_MAX);
        s_prov_pass[WIFI_STA_PASSWORD_MAX] = '\0';
    } else {
        s_prov_pass[0] = '\0';
    }
    s_prov_request = true;   /* set last so the task sees complete creds */
}

/* Persistent reconnect: seconds to wait between reconnect bursts while
 * disconnected. The driver does 3 fast retries then gives up; this task
 * re-issues connect forever so an unattended field device self-heals
 * (WiFi outages and WiFi/BLE coexistence handshake timeouts). */
#define WIFI_RECONNECT_PERIOD_S   15

void ncle_wifi_sta_task_set_status_callback(wifi_status_cb_t cb)
{
    s_status_cb = cb;
}

static void wifi_sta_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "WiFi STA task started (internal radio, no GPIO)");

    if (ncle_wifi_sta_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed");
        vTaskDelete(NULL);
        return;
    }

    /* Headless boot: load saved credentials and auto-connect. */
    char ssid[WIFI_STA_SSID_MAX + 1] = {0};
    char pass[WIFI_STA_PASSWORD_MAX + 1] = {0};
    if (ncle_wifi_sta_config_load(ssid, pass) == ESP_OK && ssid[0] != '\0') {
        ESP_LOGI(TAG, "Auto-connecting to saved SSID: %s", ssid);
        ncle_wifi_sta_connect(ssid, pass);
    }
#ifdef CONFIG_NCLE_WIFI_DEV_CREDENTIALS
    else {
        ESP_LOGI(TAG, "No saved creds; using dev default SSID: %s",
                 CONFIG_NCLE_WIFI_DEV_SSID);
        ncle_wifi_sta_connect(CONFIG_NCLE_WIFI_DEV_SSID, CONFIG_NCLE_WIFI_DEV_PASSWORD);
    }
#else
    else {
        ESP_LOGI(TAG, "No saved WiFi credentials; waiting for wifi_config over BLE");
    }
#endif

    /* Edge-based BLE notifications + persistent auto-reconnect.
     *   - "connected": on every transition INTO connected (covers first
     *     connect AND reconnects / reconfigure-while-connected).
     *   - "failed": once per disconnection episode (silent during the 15s
     *     retry churn so a connected phone isn't flooded). */
    wifi_sta_state_t prev_state = WIFI_STA_STATE_IDLE;
    bool connected_saved = false;
    bool failed_notified = false;
    int  disconnected_secs = 0;

    /* Test-before-save state: while a trial is active, the credentials being
     * tested are NOT yet in NVS. On success they are saved; on failure we
     * revert to the previously saved credentials. */
    bool trial_active = false;
    char trial_ssid[WIFI_STA_SSID_MAX + 1] = {0};
    char trial_pass[WIFI_STA_PASSWORD_MAX + 1] = {0};

    while (1) {
        wifi_sta_status_t status;
        ncle_wifi_sta_get_status(&status);

        /* New provisioning request: start a trial connection (do NOT save yet). */
        if (s_prov_request) {
            s_prov_request = false;
            strncpy(trial_ssid, s_prov_ssid, WIFI_STA_SSID_MAX);
            trial_ssid[WIFI_STA_SSID_MAX] = '\0';
            strncpy(trial_pass, s_prov_pass, WIFI_STA_PASSWORD_MAX);
            trial_pass[WIFI_STA_PASSWORD_MAX] = '\0';
            trial_active = true;
            failed_notified = false;
            ESP_LOGI(TAG, "Provision trial: testing SSID '%s' (not saved yet)", trial_ssid);
            ncle_wifi_sta_connect(trial_ssid, trial_pass);
            prev_state = WIFI_STA_STATE_CONNECTING;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Transition INTO connected -> always notify (incl. reconnects and
         * reconfigure-while-connected). */
        if (status.state == WIFI_STA_STATE_CONNECTED &&
            prev_state != WIFI_STA_STATE_CONNECTED) {
            ESP_LOGI(TAG, "WiFi connected -> notify app");
            if (s_status_cb) {
                s_status_cb(&status);
            }
            failed_notified = false;
            /* Trial succeeded: the credentials are proven good -> save to NVS. */
            if (trial_active) {
                ncle_wifi_sta_config_save(trial_ssid, trial_pass);
                ESP_LOGI(TAG, "Provision trial succeeded -> credentials saved to NVS");
                /* An SSID now exists: this site wants the cloud, so start
                 * buffering readings through outages from the next one on. */
                wifi_link_provisioned_changed();
                trial_active = false;
            }
            if (!connected_saved) {
                ncle_wifi_sta_config_set_connected(true);
                connected_saved = true;
            }
        }

        if (status.state == WIFI_STA_STATE_DISCONNECTED) {
            /* Trial failed (e.g. wrong password): report failure and revert to
             * the previously saved credentials instead of persistently
             * retrying the bad ones. */
            if (trial_active) {
                ESP_LOGW(TAG, "Provision trial failed -> reverting to saved credentials");
                trial_active = false;
                if (s_status_cb) {
                    s_status_cb(&status);   /* wifi_failed */
                }
                failed_notified = true;
                char o_ssid[WIFI_STA_SSID_MAX + 1] = {0};
                char o_pass[WIFI_STA_PASSWORD_MAX + 1] = {0};
                if (ncle_wifi_sta_config_load(o_ssid, o_pass) == ESP_OK && o_ssid[0] != '\0') {
                    ESP_LOGI(TAG, "Reverting: reconnecting to saved SSID: %s", o_ssid);
                    ncle_wifi_sta_connect(o_ssid, o_pass);
                }
                disconnected_secs = 0;
                prev_state = status.state;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            if (!failed_notified) {
                ESP_LOGI(TAG, "WiFi disconnected -> notify app (once)");
                if (s_status_cb) {
                    s_status_cb(&status);
                }
                failed_notified = true;
            }
            /* Persistent auto-reconnect (silent on BLE). The driver gives up
             * after 3 quick retries; for an unattended field device we keep
             * trying forever. Reload creds from NVS each burst so credentials
             * provisioned over BLE (saved by cmd_parser) are picked up. */
            if (++disconnected_secs >= WIFI_RECONNECT_PERIOD_S) {
                disconnected_secs = 0;
                char r_ssid[WIFI_STA_SSID_MAX + 1] = {0};
                char r_pass[WIFI_STA_PASSWORD_MAX + 1] = {0};
                if (ncle_wifi_sta_config_load(r_ssid, r_pass) == ESP_OK && r_ssid[0] != '\0') {
                    ESP_LOGI(TAG, "Auto-reconnect: retrying saved SSID: %s", r_ssid);
                    ncle_wifi_sta_connect(r_ssid, r_pass);
                }
#ifdef CONFIG_NCLE_WIFI_DEV_CREDENTIALS
                else {
                    ESP_LOGI(TAG, "Auto-reconnect: retrying dev SSID: %s",
                             CONFIG_NCLE_WIFI_DEV_SSID);
                    ncle_wifi_sta_connect(CONFIG_NCLE_WIFI_DEV_SSID,
                                          CONFIG_NCLE_WIFI_DEV_PASSWORD);
                }
#endif
            }
        } else {
            /* CONNECTING / IDLE: no BLE notification here. cmd_parser already
             * acks "connecting" to the app when a wifi_config arrives. */
            disconnected_secs = 0;
        }

        prev_state = status.state;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t ncle_wifi_sta_task_start(void)
{
    if (s_task_handle != NULL) {
        return ESP_OK;
    }
    if (xTaskCreate(wifi_sta_task, "wifi_sta", 6144, NULL, 5, &s_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create WiFi STA task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ncle_wifi_sta_task_stop(void)
{
    if (s_task_handle) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
        ncle_wifi_sta_deinit();
    }
    return ESP_OK;
}
