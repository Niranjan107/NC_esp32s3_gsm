#include "fota.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#ifdef CONFIG_NCLE_MQTT_ENABLE
#include "mqtt_client_svc.h"   /* suspend the broker session to free RAM */
#endif

static const char *TAG = "FOTA";

/* Long enough for a signed URL. 256 was fine for a plain path on a local server,
 * but an S3/GCS presigned link or a CDN token URL runs to 500-900 characters, and
 * a URL over this limit is refused outright - the update would fail before a
 * single byte was fetched. 1 KB of static RAM is a cheap price for not having
 * that argument with the server team. */
#define FOTA_URL_MAX 1024

static fota_output_cb_t s_cb = NULL;
static char s_url[FOTA_URL_MAX];
static volatile bool s_busy = false;
/* Last failure JSON, kept so it can be repeated to the broker after MQTT is
 * resumed - see the `done:` label. */
static char s_fail_reason[128];

void fota_set_output_callback(fota_output_cb_t cb)
{
    s_cb = cb;
}

/**
 * @brief Log free heap AND the largest contiguous block.
 *
 * The total is not what matters for an HTTPS download. mbedTLS with dynamic
 * buffers allocates a buffer sized to whatever record the SERVER sends - CDNs
 * send full 16 KB TLS records, so it needs a single ~16.7 KB CONTIGUOUS block.
 * A device with 40 KB free but fragmented into 8 KB pieces fails exactly the
 * same way as one with no memory at all, and the error looks identical. Logging
 * both numbers is the difference between diagnosing that in seconds and guessing
 * at it for an afternoon.
 */
static void fota_log_heap(const char *when)
{
    ESP_LOGI(TAG, "heap %s: free=%u largest_block=%u",
             when,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

/* Emit one status JSON to the registered callback. */
static void fota_report(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
    ESP_LOGI(TAG, "%s", buf);
    if (s_cb) s_cb(buf, n);

    /* Remember failures so they can be repeated once the broker is back (they are
     * reported while MQTT is suspended, so they only reach BLE/console first). */
    if (strstr(buf, "\"failed\"") != NULL) {
        strncpy(s_fail_reason, buf, sizeof(s_fail_reason) - 1);
        s_fail_reason[sizeof(s_fail_reason) - 1] = '\0';
    }
}

static void fota_task(void *arg)
{
    (void)arg;
    esp_http_client_config_t http_cfg = {
        .url = s_url,
        .crt_bundle_attach = esp_crt_bundle_attach, /* HTTPS trust; ignored for http */
        .keep_alive_enable = true,
        /* 60 s, not 15. This governs each socket operation, including opening the
         * connection to whatever host the download redirects to. Field devices sit
         * on phone hotspots and rural mobile links where a TLS handshake to a CDN
         * can take tens of seconds; 15 s aborted a perfectly good update here with
         * "select() timeout" simply because the link was briefly slow. A generous
         * timeout costs nothing when the network is healthy - it only decides how
         * long we wait before giving up when it is not. */
        .timeout_ms = 60000,
        /* Real download URLs are long. A GitHub release, an S3 presigned link or
         * a CDN URL all answer the first request with a 302 to a signed URL of
         * ~900 characters (token + expiry + signature). The defaults here are
         * 512 bytes each, so the redirected request does not fit and the client
         * fails with "Out of buffer" AFTER a successful TLS handshake - which
         * reads like a server fault rather than a client limit. Sized to take a
         * ~2 KB URL with its headers.
         *   buffer_size    - response headers coming back
         *   buffer_size_tx - the request we send, i.e. where the long URL goes */
        .buffer_size    = 2048,
        .buffer_size_tx = 4096,
    };
    /* Fetch the image in small HTTP Range requests instead of one long stream.
     *
     * WHY THIS EXISTS: with MBEDTLS_DYNAMIC_BUFFER, mbedTLS allocates a receive
     * buffer sized to whatever TLS record the SERVER sends - our
     * MBEDTLS_SSL_IN_CONTENT_LEN does not cap it. CDNs send full 16 KB records,
     * so a plain download demands a single contiguous ~16.7 KB block, and on this
     * board - WiFi + BLE + an MQTT TLS session + the OTA writer inside 76 KB of
     * internal RAM - that block is not there. The failure is
     * "Dynamic Impl: alloc(16749 bytes) failed" partway through the download.
     *
     * Asking for 4 KB at a time means the response body IS 4 KB, so the server
     * cannot send a bigger record than that and the allocation drops to ~4.4 KB.
     * Cheaper than disconnecting MQTT for the duration, and it keeps progress
     * reporting alive while the update runs.
     *
     * REQUIREMENT ON THE HOST: it must honour Range requests and return 206 with
     * the byte range of the RAW file. A host that serves a compressed variant
     * ranges over the compressed bytes and would assemble a corrupt image - so
     * verify any new download host with:
     *     curl -r 0-4095 <url> | cmp - <(head -c 4096 firmware.bin) */
    esp_https_ota_config_t ota_cfg = {
        .http_config           = &http_cfg,
        .partial_http_download = true,
        /* 32 KB, not 4 KB. Each range request costs a FULL TLS handshake - the
         * connection is not reused between them - so 4 KB chunks meant ~400
         * handshakes and a 6.5-minute download for a 1.6 MB image (measured).
         * 32 KB cuts that to ~50 and brings it under a minute.
         *
         * The trade-off, measured rather than assumed: chunk size drives the TLS
         * record size, and therefore the contiguous RAM needed -
         *     4 KB chunks  -> ~4.4 KB  (fits alongside a live MQTT session)
         *     32 KB chunks -> ~16.7 KB (does NOT - only 7,680 free with MQTT up)
         * so the bigger chunk REQUIRES the MQTT suspend below. That costs live
         * progress on the broker during the download - but it also shortens the
         * silence from 6.5 minutes to about one, which is the better answer to
         * "the server cannot see what is happening". device_boot confirms the
         * outcome either way. */
        .max_http_request_size  = 32768,
    };

    /* Report BEFORE suspending MQTT - this is the last message that can reach the
     * broker until the update finishes. */
    fota_report("{\"fota\":\"downloading\",\"percent\":0}");
    fota_log_heap("before");

#ifdef CONFIG_NCLE_MQTT_ENABLE
    /* Free the broker's TLS session for the duration of the download.
     *
     * Measured on this board: a download leaves 7,680 bytes as the largest free
     * block while MQTT is connected, and a CDN's 16 KB TLS record needs 16,749.
     * Partial-range downloads shrink that requirement to ~4.4 KB, but only when
     * the host honours Range - and we cannot assume every future download server
     * will. Suspending MQTT closes the gap either way, so an update works on any
     * host.
     *
     * Nothing is lost by doing this: readings continue to be written to flash by
     * store-and-forward and are delivered when the link returns. Progress
     * messages go to BLE and the console meanwhile, and a successful update
     * announces itself with device_boot after the reboot. */
    mqtt_svc_suspend();
    vTaskDelay(pdMS_TO_TICKS(500));      /* let the socket close and memory return */
    fota_log_heap("after mqtt suspend");
#endif

    esp_https_ota_handle_t h = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &h);
    fota_log_heap("after connect");
    if (err != ESP_OK) {
        fota_report("{\"fota\":\"failed\",\"reason\":\"begin %s\"}", esp_err_to_name(err));
        goto done;
    }

    int total = esp_https_ota_get_image_size(h);
    int last_decile = 0;   /* already reported 0% */
    while (1) {
        err = esp_https_ota_perform(h);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        int read = esp_https_ota_get_image_len_read(h);
        int pct = (total > 0) ? (int)((int64_t)read * 100 / total) : 0;
        int decile = (pct / 10) * 10;             /* 0,10,20,...,100 */
        if (decile > last_decile) {
            last_decile = decile;
            fota_report("{\"fota\":\"downloading\",\"percent\":%d}", decile);
        }
    }

    if (err == ESP_OK && esp_https_ota_is_complete_data_received(h)) {
        err = esp_https_ota_finish(h);
        if (err == ESP_OK) {
            if (last_decile < 100) fota_report("{\"fota\":\"downloading\",\"percent\":100}");
            fota_report("{\"fota\":\"success\"}");
            vTaskDelay(pdMS_TO_TICKS(800));   /* let the message flush before reboot */
            esp_restart();
        } else {
            fota_report("{\"fota\":\"failed\",\"reason\":\"finish %s\"}", esp_err_to_name(err));
        }
    } else if (err == ESP_OK) {
        /* The transfer ended without error but fewer bytes arrived than the image
         * header declared. Almost always the HOST, not the device: something in
         * the path served a COMPRESSED copy (jsDelivr does this - 1,122,305 bytes
         * instead of 1,675,008), or a proxy truncated the body. Reporting the raw
         * "ESP_OK" here was actively misleading, since the download did not
         * actually succeed. The image is discarded rather than installed. */
        esp_https_ota_abort(h);
        fota_report("{\"fota\":\"failed\",\"reason\":\"incomplete_download - "
                    "host must serve the raw uncompressed file\"}");
    } else {
        esp_https_ota_abort(h);
        fota_report("{\"fota\":\"failed\",\"reason\":\"download %s\"}", esp_err_to_name(err));
    }

done:
#ifdef CONFIG_NCLE_MQTT_ENABLE
    /* Only reached when the update FAILED - success reboots above. Bring the
     * broker back, then repeat the failure line, because the report above went
     * out while MQTT was suspended and reached BLE/console only. Without this the
     * server would see "downloading 0%" and then nothing at all, which is
     * indistinguishable from a device that died mid-update. */
    mqtt_svc_resume();
    for (int i = 0; i < 100 && !mqtt_svc_is_connected(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));      /* up to 10 s for the broker */
    }
    if (mqtt_svc_is_connected() && s_fail_reason[0]) {
        fota_report("%s", s_fail_reason);
    }
#endif
    s_fail_reason[0] = '\0';
    s_busy = false;
    vTaskDelete(NULL);
}

/* Reboot shortly after so the command reply flushes to BLE/MQTT first. */
static void fota_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

esp_err_t fota_rollback(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *prev    = esp_ota_get_next_update_partition(NULL);
    if (prev == NULL || prev == running) {
        ESP_LOGW(TAG, "Rollback: no other OTA slot to fall back to");
        return ESP_ERR_NOT_FOUND;
    }
    /* Confirm the other slot actually holds a valid firmware image. */
    esp_app_desc_t desc;
    if (esp_ota_get_partition_description(prev, &desc) != ESP_OK) {
        ESP_LOGW(TAG, "Rollback: previous slot has no valid firmware");
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = esp_ota_set_boot_partition(prev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Rollback: set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "Manual rollback -> booting previous firmware (v%s)", desc.version);
    xTaskCreate(fota_reboot_task, "fota_rb", 2048, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t fota_start(const char *url)
{
    if (s_busy) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!url || url[0] == '\0' || strlen(url) >= FOTA_URL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(s_url, url, FOTA_URL_MAX - 1);
    s_url[FOTA_URL_MAX - 1] = '\0';
    s_busy = true;
    if (xTaskCreate(fota_task, "fota", 8192, NULL, 5, NULL) != pdPASS) {
        s_busy = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}
