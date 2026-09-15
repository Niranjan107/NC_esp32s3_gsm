#include "soft_uart_rmt.h"
#include "rmt_decoder.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdlib.h>

#define TAG "soft_uart_rmt"

#ifndef CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ
#define CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ 1000000
#endif
#ifndef CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS
#define CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS 64
#endif

typedef struct {
    rmt_symbol_word_t symbols[CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS];
    size_t            num_symbols;
} rx_batch_t;

struct soft_uart_rmt_obj {
    soft_uart_rmt_config_t     cfg;
    rmt_channel_handle_t       rx_chan;
    uint32_t                   bit_time_ticks;
    uint32_t                   frame_min_ticks;
    uint32_t                   frame_max_ticks;
    bool                       running;
    soft_uart_rmt_byte_cb_t    byte_cb;
    void                      *byte_cb_ctx;
    soft_uart_rmt_stats_t      stats;

    QueueHandle_t              symbol_queue;
    TaskHandle_t               task_handle;
    rmt_symbol_word_t         *rx_buffer;
    size_t                     rx_buffer_size;
    rmt_receive_config_t       rx_cfg;   /* moved to struct so the ISR can re-arm */
    rmt_decoder_state_t        dec_state;

    /* Diagnostics */
    volatile uint32_t          isr_fire_count;
    volatile uint32_t          receive_call_count;
    volatile uint32_t          receive_err_count;
    volatile int               last_receive_err;
};

static void recalc_timings(soft_uart_rmt_handle_t h)
{
    uint32_t res = h->cfg.resolution_hz;
    uint32_t baud = (uint32_t)h->cfg.baud_rate;
    h->bit_time_ticks   = (res + baud / 2u) / baud;
    h->frame_min_ticks  = h->bit_time_ticks / 4u;
    /* 20 bit-times gives headroom for slow WMs; the real inter-telegram gap
     * limit is set separately via signal_range_max_ns below. */
    h->frame_max_ticks  = h->bit_time_ticks * 20u;
    if (h->frame_min_ticks == 0) h->frame_min_ticks = 1;
}

static void on_decoded_byte(const rmt_decoder_frame_t *frame, void *ctx)
{
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)ctx;
    if (frame->frame_err) h->stats.bytes_frame_err++;
    else                  h->stats.bytes_ok++;

    if (h->byte_cb) {
        soft_uart_rmt_rx_t rx = {
            .byte      = frame->byte,
            .ts_us     = esp_timer_get_time(),
            .frame_err = frame->frame_err,
        };
        h->byte_cb(&rx, h->byte_cb_ctx);
    }
}

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t chan,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    (void)chan;
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)user_ctx;
    h->isr_fire_count++;
    BaseType_t higher = pdFALSE;
    rx_batch_t batch;
    size_t n = edata->num_symbols;
    if (n > CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS) {
        n = CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS;
        h->stats.rmt_overflows++;
    }
    /* Copy symbols out BEFORE re-arming, so RMT can overwrite rx_buffer. */
    memcpy(batch.symbols, edata->received_symbols, n * sizeof(rmt_symbol_word_t));
    batch.num_symbols = n;

    if (xQueueSendFromISR(h->symbol_queue, &batch, &higher) != pdTRUE) {
        h->stats.queue_drops++;
    }
    return higher == pdTRUE;
}

static void decoder_task(void *arg)
{
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)arg;
    /* First rmt_receive is armed from soft_uart_rmt_start (not here), so
     * batches can start arriving via the ISR before this task even runs. */

    rx_batch_t batch;
    while (h->running) {
        if (xQueueReceive(h->symbol_queue, &batch, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Re-arm RMT reception from the task context. 
             * Calling it from ISR is unsafe in ESP-IDF v5 and causes RTOS mutex violations. */
            esp_err_t err = rmt_receive(h->rx_chan, h->rx_buffer, h->rx_buffer_size, &h->rx_cfg);
            if (err == ESP_OK) {
                h->receive_call_count++;
            } else {
                h->receive_err_count++;
                h->last_receive_err = err;
            }

            for (size_t i = 0; i < batch.num_symbols; i++) {
                rmt_symbol_word_t *w = &batch.symbols[i];
                uint8_t b0 = rmt_decoder_bits_from_duration(w->duration0, h->bit_time_ticks);
                rmt_decoder_feed(&h->dec_state, (uint8_t)w->level0, b0);
                if (w->duration1 > 0) {
                    uint8_t b1 = rmt_decoder_bits_from_duration(w->duration1, h->bit_time_ticks);
                    rmt_decoder_feed(&h->dec_state, (uint8_t)w->level1, b1);
                } else {
                    /* duration1==0 = end-of-reception, line idle at level1.
                     * Feed up to a full frame to complete any pending stop
                     * bit, then reset. */
                    rmt_decoder_feed(&h->dec_state, (uint8_t)w->level1, 10);
                    rmt_decoder_reset(&h->dec_state);
                }
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg,
                             soft_uart_rmt_handle_t *out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;
    if (cfg->data_bits != 8 || cfg->stop_bits != 1 || cfg->parity != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (cfg->baud_rate < 1200 || cfg->baud_rate > 230400) {
        return ESP_ERR_INVALID_ARG;
    }

    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->cfg = *cfg;
    if (h->cfg.resolution_hz == 0) h->cfg.resolution_hz = CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ;
    if (h->cfg.rmt_mem_block_symbols == 0) h->cfg.rmt_mem_block_symbols = CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS;
    if (h->cfg.byte_queue_depth == 0) h->cfg.byte_queue_depth = 256;

    recalc_timings(h);

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num       = h->cfg.gpio_num,
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = h->cfg.resolution_hz,
        .mem_block_symbols = h->cfg.rmt_mem_block_symbols,
        .flags.invert_in = false,
        .flags.with_dma  = false,
    };
    esp_err_t err = rmt_new_rx_channel(&rx_cfg, &h->rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %d", err);
        free(h);
        return err;
    }

    ESP_LOGI(TAG, "init OK: gpio=%d baud=%d res=%lu bit_ticks=%lu",
             h->cfg.gpio_num, h->cfg.baud_rate,
             (unsigned long)h->cfg.resolution_hz, (unsigned long)h->bit_time_ticks);
    *out = h;
    return ESP_OK;
}

esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (h->running) soft_uart_rmt_stop(h);
    if (h->rx_chan) rmt_del_channel(h->rx_chan);
    free(h);
    return ESP_OK;
}

esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h)
{
    if (!h || h->running) return ESP_ERR_INVALID_STATE;

    h->rx_buffer_size = h->cfg.rmt_mem_block_symbols * sizeof(rmt_symbol_word_t) * 2;
    h->rx_buffer = (rmt_symbol_word_t *)malloc(h->rx_buffer_size);
    if (!h->rx_buffer) return ESP_ERR_NO_MEM;

    h->symbol_queue = xQueueCreate(8, sizeof(rx_batch_t));
    if (!h->symbol_queue) { free(h->rx_buffer); h->rx_buffer = NULL; return ESP_ERR_NO_MEM; }

    rmt_decoder_init(&h->dec_state, on_decoded_byte, h);

    /* Build the receive config once and store on the handle so the ISR can
     * re-arm without touching task-local data.
     *
     * signal_range_min_ns: glitch filter. The ESP32-S3 RMT filter register
     * is only 8 bits wide (capped at ~3 us at the default 80 MHz clock src),
     * so values above ~3000 ns get rejected with ESP_ERR_INVALID_ARG. 2 us
     * is within range and still filters noise spikes shorter than ~2% of
     * one bit time at 9600 baud.
     *
     * signal_range_max_ns: THIS IS THE KEY FIX FOR GLITCHES.
     * Previously this was derived from frame_max_ticks (~1.25 ms at 9600 baud).
     * Problem: a WM telegram is ~12 chars at 9600 baud = ~12.5 ms on the wire.
     * Inter-byte idle gaps inside a single telegram can exceed the old 1.25 ms
     * limit, causing RMT to fire its "done" ISR mid-telegram. Each partial
     * batch was decoded and processed as a separate (truncated) packet, giving
     * outputs like "N00r045t" or "0010.05lt" instead of "N0010.045=lt".
     *
     * Fix: use 20 ms as the max idle time. RMT will stay armed across an
     * entire WM telegram and only fire "done" when the line has been silent
     * for 20 ms (well beyond any intra-telegram gap).
     *
     * HARDWARE LIMIT: The ESP32-S3 RMT idle-threshold register is 15 bits,
     * so the maximum value at 1 MHz resolution is 32767 ticks = ~32.767 ms.
     * 100 ms (100 000 ticks) exceeds this and triggers ESP_ERR_INVALID_ARG.
     * 20 ms = 20 000 ticks is safely within the hardware limit and still
     * covers a full 13-char WM telegram (~13.5 ms at 9600 baud). */
    h->rx_cfg.signal_range_min_ns = 2000u;
    /* 20 ms inter-packet idle -- within 15-bit HW cap (~32.767 ms at 1 MHz) */
    h->rx_cfg.signal_range_max_ns = 20000000u;  /* 20 ms */
    h->rx_cfg.flags.en_partial_rx = false;
    ESP_LOGI(TAG, "rx_cfg: min_ns=%u max_ns=%u (20ms inter-packet idle)",
             (unsigned)h->rx_cfg.signal_range_min_ns,
             (unsigned)h->rx_cfg.signal_range_max_ns);

    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_cb };
    esp_err_t err = rmt_rx_register_event_callbacks(h->rx_chan, &cbs, h);
    if (err != ESP_OK) goto fail;

    err = rmt_enable(h->rx_chan);
    if (err != ESP_OK) goto fail;

    h->running = true;
    BaseType_t ok = xTaskCreate(decoder_task, "sw_uart_dec", 4096, h, 6, &h->task_handle);
    if (ok != pdPASS) { h->running = false; err = ESP_ERR_NO_MEM; goto fail; }

    /* Prime reception: first rmt_receive call; every subsequent one is
     * re-armed by the ISR itself for zero-gap continuous capture. */
    err = rmt_receive(h->rx_chan, h->rx_buffer, h->rx_buffer_size, &h->rx_cfg);
    h->receive_call_count++;
    if (err != ESP_OK) {
        h->last_receive_err = err;
        h->receive_err_count++;
        ESP_LOGE(TAG, "initial rmt_receive failed: %d", err);
        goto fail;
    }

    ESP_LOGI(TAG, "started on GPIO %d at %d baud", h->cfg.gpio_num, h->cfg.baud_rate);
    return ESP_OK;

fail:
    /* Must stop the task before tearing down shared queue + buffer.
     * If the task was created it will exit when h->running is cleared. */
    h->running = false;
    if (h->task_handle) {
        vTaskDelay(pdMS_TO_TICKS(150));   /* let the task see h->running=false */
        h->task_handle = NULL;
    }
    if (h->rx_chan) {
        rmt_disable(h->rx_chan);
    }
    if (h->symbol_queue) { vQueueDelete(h->symbol_queue); h->symbol_queue = NULL; }
    if (h->rx_buffer)    { free(h->rx_buffer); h->rx_buffer = NULL; }
    return err;
}

esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (!h->running) return ESP_OK;
    h->running = false;
    vTaskDelay(pdMS_TO_TICKS(150));
    rmt_disable(h->rx_chan);
    if (h->symbol_queue) { vQueueDelete(h->symbol_queue); h->symbol_queue = NULL; }
    if (h->rx_buffer)    { free(h->rx_buffer); h->rx_buffer = NULL; }
    h->task_handle = NULL;
    return ESP_OK;
}

esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb, void *ctx)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    h->byte_cb = cb;
    h->byte_cb_ctx = ctx;
    return ESP_OK;
}

esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int b)
{
    if (!h || b < 1200 || b > 230400) return ESP_ERR_INVALID_ARG;
    h->cfg.baud_rate = b;
    recalc_timings(h);
    return ESP_OK;
}

void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h, soft_uart_rmt_stats_t *o)
{
    if (h && o) *o = h->stats;
}

void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h)
{
    if (h) memset(&h->stats, 0, sizeof(h->stats));
}
