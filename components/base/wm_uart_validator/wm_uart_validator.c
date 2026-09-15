#include "wm_uart_validator.h"
#include "wm_uart.h"
#include "soft_uart_rmt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/gpio_sig_map.h"
#include "sdkconfig.h"
#include <string.h>

#define TAG "wm_validate"

typedef struct {
    uint8_t  byte;
    int64_t  ts_us;
    uint8_t  source;      /* 0 = HW, 1 = SW */
    uint8_t  frame_err;
} tagged_byte_t;

static wm_uart_validator_config_t   s_cfg;
static wm_uart_validator_stats_t    s_stats;
static soft_uart_rmt_handle_t       s_sw_handle = NULL;
static QueueHandle_t                s_byte_queue = NULL;
static TaskHandle_t                 s_task = NULL;
static volatile bool                s_running = false;

static void hw_byte_cb(uint8_t byte, int64_t ts_us, void *ctx) {
    (void)ctx;
    tagged_byte_t t = { .byte = byte, .ts_us = ts_us, .source = 0, .frame_err = 0 };
    if (s_byte_queue) xQueueSend(s_byte_queue, &t, 0);
}
static void sw_byte_cb(const soft_uart_rmt_rx_t *rx, void *ctx) {
    (void)ctx;
    /* Skip frames that failed start/stop validation. The ESP-IDF UART driver
     * drops framing-error bytes silently too, so including them on one side
     * only would skew the byte-count comparison. The frame_err counter in
     * soft_uart_rmt_stats still tracks these for reporting. */
    if (rx->frame_err) return;
    tagged_byte_t t = { .byte = rx->byte, .ts_us = rx->ts_us,
                        .source = 1, .frame_err = 0 };
    if (s_byte_queue) xQueueSend(s_byte_queue, &t, 0);
}

#define MAX_PACKET_BYTES 512
#define CLOSE_SIZE_THRESHOLD 64   /* close packet when BOTH sides reach this */
#define FORCE_CLOSE_TIMEOUT_US 2000000  /* but force-close after 2 s if one side dies */

typedef struct {
    uint8_t  bytes[MAX_PACKET_BYTES];
    int64_t  ts[MAX_PACKET_BYTES];
    uint16_t count;
    bool     truncated;
} packet_buf_t;

static packet_buf_t s_hw_pkt;
static packet_buf_t s_sw_pkt;
static int64_t      s_last_hw_ts = 0;
static int64_t      s_last_sw_ts = 0;
static int64_t      s_last_report_us = 0;
static int64_t      s_window_start_us = 0;

#define ALIGN_SEARCH_RANGE 64   /* search for best stream offset within +/- this many bytes */

static void close_packet(void)
{
    if (s_hw_pkt.count == 0 && s_sw_pkt.count == 0) return;

    s_stats.hw_packets++;
    if (s_sw_pkt.count > 0) s_stats.sw_packets++;
    s_stats.hw_bytes += s_hw_pkt.count;
    s_stats.sw_bytes += s_sw_pkt.count;

    /* Find best alignment offset k in [-ALIGN_SEARCH_RANGE, +ALIGN_SEARCH_RANGE]
     * that maximises bytes where hw[start_hw+i] == sw[start_sw+i]. Positive k
     * means SW leads HW (SW has extra bytes at the start). */
    int best_matches = 0;
    int best_k = 0;
    int best_len = 0;
    for (int k = -ALIGN_SEARCH_RANGE; k <= ALIGN_SEARCH_RANGE; k++) {
        int start_hw = (k >= 0) ? 0 : -k;
        int start_sw = (k >= 0) ? k : 0;
        int hw_avail = (int)s_hw_pkt.count - start_hw;
        int sw_avail = (int)s_sw_pkt.count - start_sw;
        if (hw_avail <= 0 || sw_avail <= 0) continue;
        int len = hw_avail < sw_avail ? hw_avail : sw_avail;
        int m = 0;
        for (int i = 0; i < len; i++) {
            if (s_hw_pkt.bytes[start_hw + i] == s_sw_pkt.bytes[start_sw + i]) m++;
        }
        if (m > best_matches) { best_matches = m; best_k = k; best_len = len; }
    }
    s_stats.matched_bytes += best_matches;

    /* Per-packet dump for debugging (print only every N-th close to avoid spam) */
    static uint32_t dump_counter = 0;
    if ((dump_counter++ % 8) == 0) {
        char hw_hex[3 * 32 + 1] = {0};
        char sw_hex[3 * 32 + 1] = {0};
        int n_hw = s_hw_pkt.count < 32 ? s_hw_pkt.count : 32;
        int n_sw = s_sw_pkt.count < 32 ? s_sw_pkt.count : 32;
        for (int i = 0; i < n_hw; i++) snprintf(hw_hex + i*3, 4, "%02X ", s_hw_pkt.bytes[i]);
        for (int i = 0; i < n_sw; i++) snprintf(sw_hex + i*3, 4, "%02X ", s_sw_pkt.bytes[i]);
        ESP_LOGI(TAG, "pkt: HW=%u SW=%u  align_k=%d  matched=%d/%d",
                 (unsigned)s_hw_pkt.count, (unsigned)s_sw_pkt.count,
                 best_k, best_matches, best_len);
        ESP_LOGI(TAG, "  HW: %s", hw_hex);
        ESP_LOGI(TAG, "  SW: %s", sw_hex);
    }

    /* Skew measurement uses the best alignment too. */
    if (best_len > 0) {
        int start_hw = (best_k >= 0) ? 0 : -best_k;
        int start_sw = (best_k >= 0) ? best_k : 0;
        int64_t skew_sum = 0;
        int64_t skew_max = 0;
        for (int i = 0; i < best_len; i++) {
            int64_t sk = s_sw_pkt.ts[start_sw + i] - s_hw_pkt.ts[start_hw + i];
            if (sk < 0) sk = -sk;
            skew_sum += sk;
            if (sk > skew_max) skew_max = sk;
        }
        s_stats.avg_skew_us = skew_sum / best_len;
        if (skew_max > s_stats.max_skew_us) s_stats.max_skew_us = skew_max;
    }

    memset(&s_hw_pkt, 0, sizeof(s_hw_pkt));
    memset(&s_sw_pkt, 0, sizeof(s_sw_pkt));
    s_window_start_us = 0;
}

static void append_byte(packet_buf_t *p, uint8_t b, int64_t ts)
{
    if (p->count < MAX_PACKET_BYTES) {
        p->bytes[p->count] = b;
        p->ts[p->count] = ts;
        p->count++;
    } else {
        p->truncated = true;
    }
}

static void maybe_report(int64_t now)
{
    if (s_last_report_us == 0) { s_last_report_us = now; return; }
    uint32_t elapsed_ms = (uint32_t)((now - s_last_report_us) / 1000);
    if (elapsed_ms < s_cfg.report_interval_ms) return;

    soft_uart_rmt_stats_t sw_stats;
    soft_uart_rmt_get_stats(s_sw_handle, &sw_stats);
    s_stats.rmt_overflows = sw_stats.rmt_overflows;
    s_stats.queue_drops   = sw_stats.queue_drops;
    s_stats.frame_errs    = sw_stats.bytes_frame_err;

    uint64_t denom = s_stats.hw_bytes > s_stats.sw_bytes ? s_stats.hw_bytes : s_stats.sw_bytes;
    double byte_match_pct = denom ? (100.0 * (double)s_stats.matched_bytes / (double)denom) : 100.0;

    ESP_LOGI(TAG, "--- baud=%d window=%lums ---",
             s_stats.baud_rate, (unsigned long)elapsed_ms);
    ESP_LOGI(TAG, "  packets  : HW=%lu SW=%lu",
             (unsigned long)s_stats.hw_packets, (unsigned long)s_stats.sw_packets);
    ESP_LOGI(TAG, "  bytes    : HW=%llu SW=%llu match=%.3f%%",
             (unsigned long long)s_stats.hw_bytes,
             (unsigned long long)s_stats.sw_bytes,
             byte_match_pct);
    ESP_LOGI(TAG, "  frame_err: %llu  rmt_overflows: %llu  q_drops: %llu",
             (unsigned long long)s_stats.frame_errs,
             (unsigned long long)s_stats.rmt_overflows,
             (unsigned long long)s_stats.queue_drops);
    ESP_LOGI(TAG, "  skew_us  : avg=%lld  max=%lld",
             (long long)s_stats.avg_skew_us, (long long)s_stats.max_skew_us);

    s_last_report_us = now;
}

static void validator_task(void *arg)
{
    (void)arg;
    const int64_t quiet_us = (int64_t)s_cfg.packet_quiet_ms * 1000;
    tagged_byte_t t;
    ESP_LOGI(TAG, "validator task started (gpio=%d baud=%d)",
             s_cfg.shared_gpio, s_cfg.baud_rate);
    while (s_running) {
        if (xQueueReceive(s_byte_queue, &t, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (s_window_start_us == 0) s_window_start_us = esp_timer_get_time();
            if (t.source == 0) { append_byte(&s_hw_pkt, t.byte, t.ts_us); s_last_hw_ts = t.ts_us; }
            else               { append_byte(&s_sw_pkt, t.byte, t.ts_us); s_last_sw_ts = t.ts_us; }
        }
        int64_t now = esp_timer_get_time();
        int64_t last = s_last_hw_ts > s_last_sw_ts ? s_last_hw_ts : s_last_sw_ts;
        bool quiet_expired = (last > 0 && (now - last) > quiet_us &&
                              (s_hw_pkt.count > 0 || s_sw_pkt.count > 0));
        /* Close when BOTH sides have enough bytes, preserving stream alignment
         * (was: close on either-side threshold, which let fast SW race ahead
         * of chunked HW and created cumulative drift). */
        bool both_ready = (s_hw_pkt.count >= CLOSE_SIZE_THRESHOLD &&
                           s_sw_pkt.count >= CLOSE_SIZE_THRESHOLD);
        /* Safety: if one side stops producing, don't hang forever. */
        bool force_timeout = (s_window_start_us > 0 &&
                              (now - s_window_start_us) > FORCE_CLOSE_TIMEOUT_US &&
                              (s_hw_pkt.count > 0 || s_sw_pkt.count > 0));
        if (quiet_expired || both_ready || force_timeout) {
            close_packet();
        }
        maybe_report(now);
    }
    ESP_LOGI(TAG, "validator task stopped");
    vTaskDelete(NULL);
}

esp_err_t wm_uart_validator_init(const wm_uart_validator_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.baud_rate = cfg->baud_rate;

    soft_uart_rmt_config_t uc = {
        .gpio_num = cfg->shared_gpio,
        .baud_rate = cfg->baud_rate,
        .data_bits = 8, .stop_bits = 1, .parity = 0,
    };
    esp_err_t err = soft_uart_rmt_init(&uc, &s_sw_handle);
    if (err != ESP_OK) return err;

    /* Route the same GPIO input ALSO into the RMT peripheral. wm_uart already
     * routed it into UART0_RXD_IN; GPIO matrix permits fan-out. */
    esp_rom_gpio_connect_in_signal(cfg->shared_gpio,
                                   RMT_SIG_IN0_IDX + cfg->rmt_channel,
                                   false);

    /* rmt_new_rx_channel() above calls gpio_config() which switches the pin
     * from IOMUX to GPIO-matrix mode. On ESP32-S3 the WM RX pin (GPIO 44) is
     * UART0's native IOMUX pin; the switch severs UART0's IOMUX path and UART0
     * stops receiving. Restore UART0's connectivity via the GPIO matrix.
     * Both UART0 and RMT can now see the same input signal (matrix fan-out). */
#if CONFIG_NCLE_WM_UART_NUM == 0
    esp_rom_gpio_connect_in_signal(cfg->shared_gpio, U0RXD_IN_IDX, false);
#elif CONFIG_NCLE_WM_UART_NUM == 1
    esp_rom_gpio_connect_in_signal(cfg->shared_gpio, U1RXD_IN_IDX, false);
#elif CONFIG_NCLE_WM_UART_NUM == 2
    esp_rom_gpio_connect_in_signal(cfg->shared_gpio, U2RXD_IN_IDX, false);
#endif

    s_byte_queue = xQueueCreate(1024, sizeof(tagged_byte_t));
    if (!s_byte_queue) {
        soft_uart_rmt_deinit(s_sw_handle);
        s_sw_handle = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t wm_uart_validator_deinit(void)
{
    if (s_running) wm_uart_validator_stop();
    if (s_sw_handle) { soft_uart_rmt_deinit(s_sw_handle); s_sw_handle = NULL; }
    if (s_byte_queue) { vQueueDelete(s_byte_queue); s_byte_queue = NULL; }
    return ESP_OK;
}

esp_err_t wm_uart_validator_start(void)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (!s_sw_handle || !s_byte_queue) return ESP_ERR_INVALID_STATE;

    wm_uart_set_raw_byte_callback(hw_byte_cb, NULL);
    esp_err_t err = soft_uart_rmt_register_byte_cb(s_sw_handle, sw_byte_cb, NULL);
    if (err != ESP_OK) { wm_uart_set_raw_byte_callback(NULL, NULL); return err; }

    err = soft_uart_rmt_start(s_sw_handle);
    if (err != ESP_OK) {
        wm_uart_set_raw_byte_callback(NULL, NULL);
        return err;
    }

    memset(&s_hw_pkt, 0, sizeof(s_hw_pkt));
    memset(&s_sw_pkt, 0, sizeof(s_sw_pkt));
    s_last_hw_ts = s_last_sw_ts = s_last_report_us = 0;
    s_running = true;

    BaseType_t ok = xTaskCreate(validator_task, "wm_validate", 6144, NULL, 4, &s_task);
    if (ok != pdPASS) {
        s_running = false;
        soft_uart_rmt_stop(s_sw_handle);
        wm_uart_set_raw_byte_callback(NULL, NULL);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t wm_uart_validator_stop(void)
{
    if (!s_running) return ESP_OK;
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    soft_uart_rmt_stop(s_sw_handle);
    wm_uart_set_raw_byte_callback(NULL, NULL);
    s_task = NULL;
    return ESP_OK;
}
void wm_uart_validator_get_stats(wm_uart_validator_stats_t *out) { if (out) *out = s_stats; }
void wm_uart_validator_reset_stats(void) { memset(&s_stats, 0, sizeof(s_stats)); s_stats.baud_rate = s_cfg.baud_rate; }
esp_err_t wm_uart_validator_run_baud_sweep(const int *bauds, size_t n,
                                           uint32_t seconds_per_step)
{
    if (!bauds || n == 0) return ESP_ERR_INVALID_ARG;
    if (!s_running) return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "=== BAUD SWEEP START (%u steps, %lu s each) ===",
             (unsigned)n, (unsigned long)seconds_per_step);
    ESP_LOGW(TAG, "OPERATOR: set WM device baud BEFORE each step.");

    for (size_t i = 0; i < n; i++) {
        int b = bauds[i];
        ESP_LOGW(TAG, "--- Step %u/%u: baud=%d - waiting 5s for operator to set WM ---",
                 (unsigned)(i + 1), (unsigned)n, b);
        vTaskDelay(pdMS_TO_TICKS(5000));

        /* Reconfigure both sides */
        wm_uart_validator_stop();
        wm_uart_set_baud(b);
        soft_uart_rmt_set_baud(s_sw_handle, b);
        s_cfg.baud_rate = b;
        wm_uart_validator_reset_stats();
        wm_uart_validator_start();

        vTaskDelay(pdMS_TO_TICKS(seconds_per_step * 1000));

        wm_uart_validator_stats_t snap;
        wm_uart_validator_get_stats(&snap);
        uint64_t denom = snap.hw_bytes > snap.sw_bytes ? snap.hw_bytes : snap.sw_bytes;
        double match_pct = denom ? (100.0 * (double)snap.matched_bytes / (double)denom) : 0.0;

        ESP_LOGW(TAG, "SWEEP RESULT: baud=%d hw_bytes=%llu sw_bytes=%llu "
                      "match=%.3f%% frame_err=%llu rmt_ovf=%llu q_drop=%llu",
                 b,
                 (unsigned long long)snap.hw_bytes,
                 (unsigned long long)snap.sw_bytes,
                 match_pct,
                 (unsigned long long)snap.frame_errs,
                 (unsigned long long)snap.rmt_overflows,
                 (unsigned long long)snap.queue_drops);
    }
    ESP_LOGW(TAG, "=== BAUD SWEEP DONE ===");
    return ESP_OK;
}
