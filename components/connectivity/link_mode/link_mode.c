/**
 * @file link_mode.c
 * @brief Bring up exactly one connectivity stack (see link_mode.h).
 */
#include "link_mode.h"
#include "device_config.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm.h"
#include "gsm_task.h"
#endif
#ifdef CONFIG_NCLE_WIFI_ENABLE
#include "wifi_sta.h"
#include "wifi_sta_task.h"
#endif

static const char *TAG = "LINK_MODE";

/* What is actually running, which is not always what is stored: a failed
 * switch leaves the old stack up and the old mode in NVS. */
static uint8_t s_running = LINK_MODE_OFF;

uint8_t link_mode_current(void)
{
    return s_running;
}

/* ------------------------------------------------------------------------ */
/* Bring up                                                                  */
/* ------------------------------------------------------------------------ */

static esp_err_t bring_up(uint8_t mode)
{
    switch (mode) {

#ifdef CONFIG_NCLE_GSM_ENABLE
    case LINK_MODE_GSM: {
        /* A user who sent gsm_disable stays disabled. link_mode selects WHICH
         * stack may run; gsm_enable/gsm_disable is a separate, older switch
         * for whether GSM runs at all, and silently undoing it here would
         * override a deliberate choice at the next power cycle. */
        if (!gsm_is_enabled_pref()) {
            ESP_LOGI(TAG, "mode gsm, but GSM is disabled by the user "
                          "- send 'gsm_enable' to turn it on");
            s_running = LINK_MODE_GSM;      /* the selection stands */
            return ESP_OK;
        }

        gsm_net_link_register();

        esp_err_t err = gsm_task_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GSM start failed: %s", esp_err_to_name(err));
            return err;
        }
        s_running = LINK_MODE_GSM;
        ESP_LOGI(TAG, "mode: gsm");
        return ESP_OK;
    }
#endif

#ifdef CONFIG_NCLE_WIFI_ENABLE
    case LINK_MODE_WIFI: {
        /* esp_wifi_init() runs inside the task's first iteration - this is
         * where the ~81 KB goes, and nothing before this line has spent it. */
        wifi_net_link_register();

        esp_err_t err = ncle_wifi_sta_task_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
            return err;
        }
        s_running = LINK_MODE_WIFI;
        ESP_LOGI(TAG, "mode: wifi");
        return ESP_OK;
    }
#endif

    case LINK_MODE_OFF:
        s_running = LINK_MODE_OFF;
        ESP_LOGI(TAG, "mode: off - readings go to the app over BLE");
        return ESP_OK;

    default:
        /* Includes wifi on a build without CONFIG_NCLE_WIFI_ENABLE. */
        ESP_LOGE(TAG, "mode %u is not available in this build", (unsigned)mode);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

/* ------------------------------------------------------------------------ */
/* Tear down                                                                 */
/* ------------------------------------------------------------------------ */

static esp_err_t tear_down(uint8_t mode)
{
    switch (mode) {

#ifdef CONFIG_NCLE_GSM_ENABLE
    case LINK_MODE_GSM:
        ESP_LOGI(TAG, "stopping GSM");
        /* Blocks until the task has exited and released the modem. gsm_deinit
         * leaves data mode first, so this is safe with the link up. */
        return gsm_task_stop();
#endif

#ifdef CONFIG_NCLE_WIFI_ENABLE
    case LINK_MODE_WIFI:
        ESP_LOGI(TAG, "stopping WiFi");
        return ncle_wifi_sta_task_stop();
#endif

    case LINK_MODE_OFF:
    default:
        return ESP_OK;
    }
}

/* ------------------------------------------------------------------------ */
/* Public                                                                    */
/* ------------------------------------------------------------------------ */

esp_err_t link_mode_start(void)
{
    uint8_t mode = config_get_link_mode();
    ESP_LOGI(TAG, "stored mode: %s", config_link_mode_name(mode));

#ifndef CONFIG_NCLE_WIFI_ENABLE
    /* A device flashed with a WiFi build, switched to wifi, then re-flashed
     * with a GSM-only build would read wifi from NVS and find no stack to
     * start. Rewrite the stored mode rather than falling back on every boot
     * for the rest of the device's life. */
    if (mode == LINK_MODE_WIFI) {
        ESP_LOGW(TAG, "stored mode is wifi but this build has no WiFi - using gsm");
        mode = LINK_MODE_GSM;
        config_set_link_mode(LINK_MODE_GSM);
    }
#endif

    esp_err_t err = bring_up(mode);

    if (err != ESP_OK && mode != LINK_MODE_GSM) {
        /* The stored mode would not start. Fall back to gsm rather than leave
         * the device with no link at all - BLE works either way, but a device
         * that can reach the cloud is worth more than one that cannot, and the
         * operator can switch again from the app.
         *
         * The stored mode is deliberately NOT rewritten: the failure may be
         * temporary (a router that is down), and overwriting the operator's
         * choice because of one bad boot would be worse than retrying it. */
        ESP_LOGW(TAG, "stored mode %s failed to start - falling back to gsm",
                 config_link_mode_name(mode));
        err = bring_up(LINK_MODE_GSM);
    }

    return err;
}

esp_err_t link_mode_switch(uint8_t mode)
{
    if (mode > LINK_MODE_OFF) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mode == s_running) {
        ESP_LOGI(TAG, "already in %s", config_link_mode_name(mode));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "switching %s -> %s",
             config_link_mode_name(s_running), config_link_mode_name(mode));

    /* Down before up, always. The two stacks must never be initialised at the
     * same time - see the header. */
    const uint8_t previous = s_running;

    esp_err_t err = tear_down(previous);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "teardown of %s failed (%s) - staying put",
                 config_link_mode_name(previous), esp_err_to_name(err));
        return err;
    }
    s_running = LINK_MODE_OFF;

    /* Let the outgoing stack's memory actually come back before asking for the
     * incoming one's. The teardown has already waited for the task to exit;
     * this covers the deferred frees behind it. */
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "free heap after teardown: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    err = bring_up(mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bring-up of %s failed (%s) - returning to %s",
                 config_link_mode_name(mode), esp_err_to_name(err),
                 config_link_mode_name(previous));
        /* Best effort. If this fails too the device is left on BLE only, which
         * the operator can see and fix - better than a half-initialised stack. */
        bring_up(previous);
        return err;
    }

    /* Persist only on success: a mode that would not start must not be what
     * the device boots into next time. */
    config_set_link_mode(mode);
    ESP_LOGI(TAG, "mode %s stored", config_link_mode_name(mode));
    return ESP_OK;
}
