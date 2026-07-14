#include "gsm_task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "GSM_TASK";

#define GSM_POLL_INTERVAL_MS   CONFIG_NCLE_GSM_TASK_POLL_INTERVAL_MS

static TaskHandle_t       s_task_handle  = NULL;
static volatile bool      s_running      = false;
static gsm_status_cb_t    s_status_cb    = NULL;
static void              *s_status_ctx   = NULL;
static gsm_status_t       s_last_status  = {0};

static uint8_t rssi_to_bars(uint8_t rssi)
{
    if (rssi == 99 || rssi == 0) return 0;
    if (rssi >= 25) return 5;
    if (rssi >= 19) return 4;
    if (rssi >= 13) return 3;
    if (rssi >= 7)  return 2;
    return 1;
}

static void poll_and_report(void)
{
    gsm_status_t s = {0};

    s.alive = gsm_is_alive();

    if (s.alive) {
        if (gsm_get_signal_strength(&s.rssi, &s.ber) != ESP_OK) {
            s.rssi = 99;
            s.ber  = 99;
        }
        if (gsm_get_network_status(&s.net_status) != ESP_OK) {
            s.net_status = GSM_NET_UNKNOWN;
        }
    } else {
        s.rssi       = 99;
        s.ber        = 99;
        s.net_status = GSM_NET_UNKNOWN;
    }

    s.registered = (s.net_status == GSM_NET_REGISTERED_HOME ||
                    s.net_status == GSM_NET_REGISTERED_ROAMING);
    s.bars       = rssi_to_bars(s.rssi);

    /* Carry forward module_info captured at startup */
    memcpy(s.module_info, s_last_status.module_info, sizeof(s.module_info));

    s_last_status = s;

    if (s_status_cb) s_status_cb(&s, s_status_ctx);

    ESP_LOGI(TAG, "alive=%d reg=%d rssi=%d bars=%d net=%d",
             s.alive, s.registered, s.rssi, s.bars, (int)s.net_status);
}

static void gsm_task_body(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "GSM task starting");

    if (gsm_init() != ESP_OK) {
        ESP_LOGE(TAG, "gsm_init failed");
        gsm_status_t bad = {
            .alive = false, .registered = false,
            .rssi = 99, .ber = 99, .bars = 0,
            .net_status = GSM_NET_UNKNOWN,
        };
        s_last_status = bad;
        if (s_status_cb) s_status_cb(&bad, s_status_ctx);
        s_running = false;
        s_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    char info[64] = {0};
    if (gsm_get_module_info(info, sizeof(info)) == ESP_OK) {
        ESP_LOGI(TAG, "Module: %s", info);
        strncpy(s_last_status.module_info, info,
                sizeof(s_last_status.module_info) - 1);
    }

    poll_and_report();

    /* Wake every 1s so gsm_task_stop() is observed within ~1s */
    uint32_t accumulated_ms = 0;
    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        accumulated_ms += 1000;
        if (accumulated_ms >= GSM_POLL_INTERVAL_MS) {
            accumulated_ms = 0;
            poll_and_report();
        }
    }

    ESP_LOGI(TAG, "GSM task stopping");
    gsm_deinit();
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t gsm_task_start(void)
{
    if (s_running || s_task_handle) {
        ESP_LOGW(TAG, "Task already running");
        return ESP_OK;
    }

    s_running = true;
    BaseType_t ok = xTaskCreate(gsm_task_body, "gsm_task",
                                4096, NULL, 5, &s_task_handle);
    if (ok != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t gsm_task_stop(void)
{
    if (!s_running) return ESP_OK;
    s_running = false;
    return ESP_OK;
}

bool gsm_task_is_running(void)
{
    return s_running;
}

void gsm_task_set_status_callback(gsm_status_cb_t cb, void *ctx)
{
    s_status_cb  = cb;
    s_status_ctx = ctx;
}

void gsm_task_get_last_status(gsm_status_t *out)
{
    if (out) *out = s_last_status;
}

#endif /* CONFIG_NCLE_GSM_ENABLE */
