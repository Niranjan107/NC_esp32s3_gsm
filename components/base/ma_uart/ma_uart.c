/**
 * @file ma_uart.c
 * @brief Milk Analyzer (MA) UART Component Implementation for ESP32-S3
 *
 * PURPOSE:
 * This module handles serial communication with milk analyzers used in dairy
 * collection centers. Milk analyzers measure milk quality parameters (FAT, SNF,
 * CLR, etc.) and output the results via serial port.
 *
 * WHY THIS MODULE EXISTS:
 * Dairy collection centers use milk analyzers to measure milk quality.
 * These analyzers output test results via RS232/TTL serial port. This module:
 * 1. Receives test results from the milk analyzer
 * 2. Processes and cleans the data
 * 3. Forwards results to mobile app via BLE
 * 4. Triggers WM (weighing) after MA completes (dairy workflow)
 *
 * DAIRY WORKFLOW:
 * 1. Farmer brings milk → MA tests quality (FAT, SNF, etc.)
 * 2. MA outputs result → This module receives and forwards to app
 * 3. This module triggers WM → Weight is measured
 * 4. App has both quality and quantity data → Calculate payment
 *
 * FEATURES:
 * - Model-based end character detection (Pico2W compatible)
 *   - Model 3000-3999: Parentheses mode '(' and ')'
 *   - Model 4000-4999: Continuous newline mode '\n'
 *   - Other models: Timeout-based detection
 * - JSON wrapping for mobile app
 * - NVS persistence for configuration
 * - Automatic WM trigger after MA data (dairy workflow)
 *
 * DETECTION MODES (based on MA model):
 * 1. PARENTHESES (model 3000-3999):
 *    - Start capture on '('
 *    - Stop capture on ')'
 *    - Example: "(FAT:3.5 SNF:8.2)"
 *
 * 2. NEWLINE (model 4000-4999):
 *    - Each line is a complete reading
 *    - End on '\n'
 *
 * 3. TIMEOUT (other models):
 *    - Buffer all data
 *    - Process when timeout expires (no new data for X ms)
 *
 * DATA FLOW:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                      Milk Analyzer (RS232/TTL)                          │
 * │  Outputs: "(FAT:3.5 SNF:8.2 CLR:27.0)" or "FAT=3.5\nSNF=8.2\n" etc.    │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              Serial Data
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         ma_uart.c (THIS FILE)                           │
 * │                                                                          │
 * │  1. uart_rx_task() - Receives bytes, mode-specific detection            │
 * │  2. process_ma_packet() - Outputs immediately (no duplicate check)      │
 * │  3. output_ma_data() - Creates JSON and calls callbacks                 │
 * │  4. wm_uart_start() - Triggers WM for dairy workflow                    │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              JSON output
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  {"device":"ma","data":"FAT:3.5 SNF:8.2","model":3001 }                 │
 * │                                                                          │
 * │  → USB Console (puts)                                                   │
 * │  → BLE (s_data_callback → ble_spp_send)                                 │
 * │  → Triggers WM start for weighing                                       │
 * └─────────────────────────────────────────────────────────────────────────┘
 */

#include "ma_uart.h"
#include "wm_uart.h"  // For MA→WM trigger (Pico2W behavior)
#include "common.h"   // For cycle timing instrumentation
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "MA_UART";

// ============================================================================
// NVS Keys
// ============================================================================
#define NVS_NAMESPACE       "ma_config"
#define NVS_KEY_BAUD        "baud"
#define NVS_KEY_PARITY      "parity"
#define NVS_KEY_STREAM_MODE "stream"
#define NVS_KEY_MODEL_ID    "model_id"

// ============================================================================
// Internal State
// ============================================================================

// Current configuration
static ma_config_t s_config;
static bool s_initialized = false;

// Receive buffers
static char s_rx_buffer[MA_UART_BUF_SIZE];      // Raw receive buffer
static size_t s_rx_index = 0;
static char s_clean_buffer[MA_UART_BUF_SIZE];   // Cleaned data buffer
static char s_json_buffer[MA_UART_BUF_SIZE + 128];  // JSON output buffer

// Duplicate detection
static char s_last_reading[MA_UART_BUF_SIZE];   // Last processed value

// Timeout tracking
#define MA_RX_TIMEOUT_US    (MA_RX_TIMEOUT_MS * 1000)
static int64_t s_last_rx_time = 0;
static int64_t s_first_rx_time = 0;   /* first byte of the current message (timing/debug) */

/**
 * Largest silence between chunks of the SAME frame, measured per frame.
 *
 * The timeout must comfortably exceed the biggest mid-frame pause, or a single
 * analyser slip would be split into several packets. Reported per frame so a
 * measured value can replace the inherited 3000ms guess.
 */
static int64_t s_max_gap_us = 0;

/**
 * Adaptive receive timeout.
 *
 * s_rx_timeout_us starts at the MA_RX_TIMEOUT_MS ceiling so the very first
 * frame after boot is always safe. After each frame we take the largest pause
 * that frame contained, add 50% margin, and use that for the NEXT frame -
 * clamped between MA_RX_TIMEOUT_MIN_MS and MA_RX_TIMEOUT_MS.
 *
 * The timeout is only ever applied to the following frame, because a frame has
 * to survive its own pauses in order to measure them.
 *
 * s_learned_frame_len is the longest complete frame seen. If a frame arrives
 * dramatically shorter than that, we almost certainly cut it early - so the
 * timeout is thrown back to the ceiling and re-learned from scratch. Frame
 * length is the only signal we have that a split happened.
 */
static int64_t s_learned_gap_us = 0;
static int64_t s_rx_timeout_us = (int64_t)MA_RX_TIMEOUT_MS * 1000;
static size_t  s_learned_frame_len = 0;
static size_t  s_learned_line_count = 0;
static int     s_consecutive_short_frames = 0;
static size_t  s_rx_line_count = 0;

/**
 * Line count shared by the current run of short frames.
 *
 * A genuine format change (different analyser attached) is deterministic - it
 * truncates at the same place every time. A loose cable is not. Requiring the
 * run to agree keeps a hardware fault from being adopted as the new normal.
 */
static size_t  s_short_run_line_count __attribute__((unused)) = 0;

// Statistics
static ma_stats_t s_stats = {0};

// Task handle
static TaskHandle_t s_rx_task_handle = NULL;
static volatile bool s_task_running = false;

// Callbacks
static ma_data_callback_t s_data_callback = NULL;
static ma_activity_callback_t s_activity_callback = NULL;
static ma_frame_start_callback_t s_frame_start_callback = NULL;

// Stream mode
static int s_stream_mode = MA_MODE_SINGLE;      // 0=single-read, 1=stream
static volatile bool s_single_read_done = false; // Prevent duplicate output

// Model ID (determines detection mode)
static int s_model_id = MA_DEFAULT_MODEL_ID;

// Detection mode (derived from model)
static ma_detect_mode_t s_detect_mode = MA_DETECT_TIMEOUT;

// Parentheses mode state
static volatile bool s_capturing = false;       // For parentheses mode

// Self-diagnosis
static int64_t s_last_packet_time = 0;
#define MA_DIAGNOSIS_TIMEOUT_MS  5000           // 5 seconds without data = disconnected

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Get UART parity setting
 */
static uart_parity_t get_uart_parity(int parity)
{
    switch (parity) {
        case 1: return UART_PARITY_ODD;
        case 2: return UART_PARITY_EVEN;
        default: return UART_PARITY_DISABLE;
    }
}

/**
 * @brief Update detection mode based on model ID
 */
static void update_detect_mode(void)
{
    if (s_model_id >= 3000 && s_model_id <= 3999) {
        s_detect_mode = MA_DETECT_PARENTHESES;
        ESP_LOGI(TAG, "Detection mode: PARENTHESES (model %d)", s_model_id);
    } else if (s_model_id >= 4000 && s_model_id <= 4999) {
        s_detect_mode = MA_DETECT_NEWLINE;
        ESP_LOGI(TAG, "Detection mode: NEWLINE (model %d)", s_model_id);
    } else {
        s_detect_mode = MA_DETECT_TIMEOUT;
        ESP_LOGI(TAG, "Detection mode: TIMEOUT (model %d)", s_model_id);
    }
    /* NOTE: this used to call wm_capture_set_continuous() here. The framing kind
     * now rides on the frame-start callback instead (see the s_rx_index == 0
     * block), which keeps this base driver free of any weight/cloud knowledge
     * and means a listener's copy of the flag cannot go stale. */
}

/**
 * @brief Re-tune the receive timeout from the frame that just completed
 *
 * Called once per frame, before the buffers are reset. Takes the largest pause
 * that frame contained, adds 50% margin, and clamps the result - the new value
 * applies to the NEXT frame.
 *
 * If the frame came in far shorter than the longest one we have seen, we treat
 * that as evidence the timeout cut it early: the learned value is discarded and
 * we go back to the full ceiling. Frame length is the only split signal we have.
 */
static void update_rx_timeout(size_t frame_len, size_t line_count)
{
#if !CONFIG_NCLE_MA_ADAPTIVE_TIMEOUT
    // Adaptive timeout disabled in menuconfig - hold the fixed ceiling, which
    // is the behaviour this firmware has always had. Enable
    // NCLE_MA_ADAPTIVE_TIMEOUT to let the driver learn a shorter wait.
    // Measurement still runs, so the "MA frame: ... max_gap=" log remains
    // available for analysis without altering timing.
    (void)frame_len;
    (void)line_count;
    s_rx_timeout_us = (int64_t)MA_RX_TIMEOUT_MS * 1000;
    return;
#else
    const int64_t ceiling_us = (int64_t)MA_RX_TIMEOUT_MS * 1000;
    const int64_t floor_us   = (int64_t)MA_RX_TIMEOUT_MIN_MS * 1000;

    // Suspected split: much fewer newlines than the longest complete frame seen.
    if (s_learned_line_count > 0 && line_count < (s_learned_line_count / 2)) {

        // Only count this towards a format change if it MATCHES the previous
        // short frame. A different analyser truncates identically every time;
        // a loose cable or brown-out truncates at random points. Requiring the
        // run to agree keeps a hardware fault from being adopted as normal.
        if (s_consecutive_short_frames > 0 && line_count == s_short_run_line_count) {
            s_consecutive_short_frames++;
        } else {
            s_consecutive_short_frames = 1;
            s_short_run_line_count = line_count;
        }

        ESP_LOGW(TAG, "Short frame (%d vs %d lines, %d vs %d bytes) - suspected early cut (consecutive short: %d), restoring %dms timeout",
                 (int)line_count, (int)s_learned_line_count, (int)frame_len, (int)s_learned_frame_len, s_consecutive_short_frames, MA_RX_TIMEOUT_MS);

        s_learned_gap_us = 0;
        s_rx_timeout_us  = ceiling_us;

        // Three in a row, all the same length - that is deterministic, so treat
        // it as a genuine format/analyser change rather than a fault.
        if (s_consecutive_short_frames >= 3) {
            ESP_LOGI(TAG, "3 consecutive short frames all with %d lines. Adapting to new format/analyzer.", (int)line_count);
            s_learned_line_count = line_count;
            s_learned_frame_len = frame_len;
            s_consecutive_short_frames = 0;
            s_short_run_line_count = 0;
            // Fall through to learn the new timeout gap
        } else {
            return;   // do not learn a length from a fragment
        }
    } else {
        // Successful/full frame - reset consecutive short frames counter
        s_consecutive_short_frames = 0;
        s_short_run_line_count = 0;
    }

    if (line_count > s_learned_line_count) {
        s_learned_line_count = line_count;
    }
    if (frame_len > s_learned_frame_len) {
        s_learned_frame_len = frame_len;
    }

    if (s_max_gap_us > s_learned_gap_us) {
        s_learned_gap_us = s_max_gap_us;
    }

    // x1.5 margin over the worst pause actually observed
    int64_t want = (s_learned_gap_us * 3) / 2;
    if (want < floor_us)   want = floor_us;
    if (want > ceiling_us) want = ceiling_us;

    if (want != s_rx_timeout_us) {
        ESP_LOGI(TAG, "RX timeout adapted: %ldms -> %ldms (worst pause %ldms, line count: %d)",
                 (long)(s_rx_timeout_us / 1000), (long)(want / 1000),
                 (long)(s_learned_gap_us / 1000), (int)s_learned_line_count);
        s_rx_timeout_us = want;
    }
#endif  /* CONFIG_NCLE_MA_ADAPTIVE_TIMEOUT */
}

/**
 * @brief Remove extra spaces (utility from Pico2W)
 */
static void remove_extra_spaces(char *str)
{
    char *src = str, *dst = str;
    int space_found = 0;

    // Skip leading spaces
    while (*src == ' ') src++;

    while (*src) {
        if (*src != ' ') {
            *dst++ = *src++;
            space_found = 0;
        } else {
            if (!space_found) {
                *dst++ = ' ';
                space_found = 1;
            }
            src++;
        }
    }

    // Remove trailing space if present
    if (dst > str && *(dst - 1) == ' ')
        dst--;

    *dst = '\0';
}

/**
 * @brief Output MA data (JSON + callbacks)
 */
static void output_ma_data(void)
{
    // Update last packet time for diagnosis
    s_last_packet_time = esp_timer_get_time();

    // Clean up the data
    remove_extra_spaces(s_rx_buffer);
    strncpy(s_clean_buffer, s_rx_buffer, sizeof(s_clean_buffer) - 1);
    s_clean_buffer[sizeof(s_clean_buffer) - 1] = '\0';

    // Create JSON (matches Pico2W format)
    int json_len = snprintf(s_json_buffer, sizeof(s_json_buffer),
                            "{\"device\":\"ma\",\"data\":\"%s\",\"model\":%d }",
                            s_clean_buffer, s_model_id);

    // Log
    ESP_LOGD(TAG, "MA data: [%s]", s_clean_buffer);

    // Output JSON to console
    puts(s_json_buffer);

    timing_mark(TIMING_T2_APP_SENT);   // T2 - reading on its way to the app

    // Call data callback (for BLE transmission)
    if (s_data_callback) {
        s_data_callback(s_json_buffer, json_len);
    }

    // Call activity callback (for LED)
    if (s_activity_callback) {
        s_activity_callback();
    }

    s_stats.packet_count++;
}

/* ==================================================================
 * AccuMilk / Tricom MacPro CC-Solar binary parser (model 5000-5999)
 * ------------------------------------------------------------------
 * The DPU sends a binary frame: 7E 3A <type> 01 <values> <csum> 06 A3 E7
 *   - start 0x7E, end 0xA3 0xE7; type 0x17 = full reading, 0x1C = memid only
 *   - each value's units digit has bit 0x80 set => decimal point after it
 *   - fields in fixed order: Member, FAT, SNF, QTY, RATE, AMOUNT (no CLR)
 * Converts to clean JSON instead of forwarding raw bytes. Validated
 * against real MacPro captures.
 * ================================================================== */
#define ACCUMILK_MAX_DIGITS 64

static float accumilk_marked_number(const char *dg, const bool *mk, int a, int b)
{
    int mark = -1;
    for (int i = a; i < b; i++) { if (mk[i]) { mark = i; break; } }
    if (mark < 0) mark = b - 1;
    long ip = 0;
    for (int i = a; i <= mark && i < b; i++) ip = ip * 10 + (dg[i] - '0');
    float frac = 0.0f, div = 1.0f;
    for (int i = mark + 1; i < b; i++) { frac = frac * 10.0f + (dg[i] - '0'); div *= 10.0f; }
    return (float)ip + (div > 1.0f ? frac / div : 0.0f);
}

static float accumilk_value_1dp(const char *dg, const bool *mk, int *p, int end)
{
    int a = *p, mark = -1;
    for (int i = a; i < end; i++) { if (mk[i]) { mark = i; break; } }
    if (mark < 0) mark = a;
    long ip = 0;
    for (int i = a; i <= mark && i < end; i++) ip = ip * 10 + (dg[i] - '0');
    int dec = (mark + 1 < end) ? (dg[mark + 1] - '0') : 0;
    *p = mark + 2;
    return (float)ip + dec / 10.0f;
}

// Parse a raw AccuMilk frame and output clean JSON. Returns true if a full
// reading was decoded and sent; false otherwise (e.g. member-id-only frame).
static bool process_accumilk(const char *raw, int raw_len)
{
    int s = -1, e = -1;
    for (int i = 0; i < raw_len; i++) { if ((uint8_t)raw[i] == 0x7E) { s = i; break; } }
    if (s < 0) return false;
    for (int i = s + 1; i + 1 < raw_len; i++)
        { if ((uint8_t)raw[i] == 0xA3 && (uint8_t)raw[i + 1] == 0xE7) { e = i; break; } }
    if (e < 0 || (e - (s + 5)) < 1) return false;

    uint8_t type = (uint8_t)raw[s + 2];
    int vstart = s + 5, vend = e - 3;

    char dg[ACCUMILK_MAX_DIGITS];
    bool mk[ACCUMILK_MAX_DIGITS];
    int tokstart[ACCUMILK_MAX_DIGITS];
    int ntok = 0, n = 0;
    bool intok = false;
    for (int i = vstart; i <= vend && n < ACCUMILK_MAX_DIGITS; i++) {
        uint8_t b = (uint8_t)raw[i];
        if (b == 0x20) { intok = false; continue; }
        uint8_t c = b & 0x7F;
        if (c < '0' || c > '9') continue;
        if (!intok) { tokstart[ntok++] = n; intok = true; }
        dg[n] = c; mk[n] = (b & 0x80) != 0; n++;
    }

    int memid = 0, i = (ntok > 0) ? tokstart[0] : 0;
    while (i < n && !mk[i]) { memid = memid * 10 + (dg[i] - '0'); i++; }

    if (type != 0x17 || ntok < 3) return false;   // forward only full readings

    int end0 = (ntok > 1) ? tokstart[1] : n;
    float fat = accumilk_value_1dp(dg, mk, &i, end0);
    float snf = accumilk_value_1dp(dg, mk, &i, end0);
    int laststart = tokstart[ntok - 1];
    int j = tokstart[1];
    float qty = accumilk_value_1dp(dg, mk, &j, laststart);
    float rate = accumilk_marked_number(dg, mk, j, laststart);
    float amount = accumilk_marked_number(dg, mk, laststart, n);

    if (amount > 0.0f && fabsf(qty * rate - amount) > 0.10f) return false;   // corrupt -> drop

    int json_len = snprintf(s_json_buffer, sizeof(s_json_buffer),
        "{\"device\":\"ma\",\"memid\":%d,\"fat\":%.1f,\"snf\":%.1f,"
        "\"qty\":%.1f,\"rate\":%.2f,\"amount\":%.2f,\"model\":%d }",
        memid, fat, snf, qty, rate, amount, s_model_id);

    s_last_packet_time = esp_timer_get_time();
    puts(s_json_buffer);
    timing_mark(TIMING_T2_APP_SENT);   // T2 - reading on its way to the app
    if (s_data_callback) s_data_callback(s_json_buffer, json_len);
    if (s_activity_callback) s_activity_callback();
    s_stats.packet_count++;
    return true;
}

/**
 * @brief Process complete MA packet
 *
 * NOTE: Pico2W MA outputs IMMEDIATELY after receiving data - no duplicate detection!
 * MA receipts are always unique (timestamp, amounts change), so duplicate checking
 * doesn't make sense. Output immediately, then trigger WM for dairy workflow.
 */
static void process_ma_packet(void)
{
    if (s_rx_index < MA_MIN_PAYLOAD_LEN) {
        if (s_rx_index > 0) {
            s_rx_buffer[s_rx_index] = '\0';
            ESP_LOGW(TAG, "Incomplete packet (%d bytes): [%s]", (int)s_rx_index, s_rx_buffer);
        }
        s_rx_index = 0;
        s_last_rx_time = 0;
        // Discarded fragment - clear the measurement too, otherwise its short
        // length/line count would pollute what the next frame learns.
        s_max_gap_us = 0;
        s_rx_line_count = 0;
        return;
    }

    s_rx_buffer[s_rx_index] = '\0';

    // T1 - the frame is complete and about to be decoded.
    timing_mark(TIMING_T1_MA_DONE);

    // Pico2W compatibility: Output MA data IMMEDIATELY (no duplicate detection)
    // MA receipts are always unique, so just output and trigger WM
    ESP_LOGI(TAG, "MA data received (%d bytes) - outputting immediately", (int)s_rx_index);

    // AccuMilk / Tricom binary DPU (model 5000-5999): parse into clean JSON.
    // Falls back to raw passthrough if the frame can't be decoded.
    if (s_model_id >= 5000 && s_model_id <= 5999 &&
        process_accumilk(s_rx_buffer, (int)s_rx_index)) {
        // parsed + sent clean JSON
    } else {
        output_ma_data();
    }

    // WM is now started on the MA's FIRST byte (see uart_rx_task) so it reads in
    // parallel during the MA window; the settled weight is captured there and
    // merged into the MA message's MQTT copy. No post-MA WM trigger needed here.

    // MA-stage timing on the console, logged whether or not a receipt follows.
    ESP_LOGI(TAG, "MA timing (ms): T0=%ld T1=%ld T2=%ld",
             (long)timing_get_ms(TIMING_T0_MA_FIRST),
             (long)timing_get_ms(TIMING_T1_MA_DONE),
             (long)timing_get_ms(TIMING_T2_APP_SENT));

    // Frame shape: size, and the longest pause the analyser left mid-frame.
    // The timeout only has to outlast max_gap - everything beyond that is
    // dead waiting, which is what update_rx_timeout() trims away.
    ESP_LOGI(TAG, "MA frame: bytes=%d max_gap=%ldms (timeout was %ldms, newlines=%d)",
             (int)s_rx_index,
             (long)(s_max_gap_us / 1000),
             (long)(s_rx_timeout_us / 1000),
             (int)s_rx_line_count);

    // Re-tune for the next frame, using this frame's pauses and length - only
    // in TIMEOUT detection mode, where the idle gap is what ends a frame.
    // PARENTHESES/NEWLINE modes end on a terminator, so they keep the ceiling.
    if (s_detect_mode == MA_DETECT_TIMEOUT) {
        update_rx_timeout(s_rx_index, s_rx_line_count);
    } else {
        s_rx_timeout_us = (int64_t)MA_RX_TIMEOUT_MS * 1000;
        s_learned_gap_us = 0;
        s_learned_frame_len = 0;
        s_learned_line_count = 0;
        s_consecutive_short_frames = 0;
    }

    // Reset buffer
    s_rx_index = 0;
    s_last_rx_time = 0;
    s_max_gap_us = 0;      // start the next frame's gap measurement clean
    s_rx_line_count = 0;
}

/**
 * @brief UART RX Task
 */
static void uart_rx_task(void *arg)
{
    uint8_t rx_buffer[128];
    const char *mode_str[] = {"TIMEOUT", "PARENTHESES", "NEWLINE"};

    ESP_LOGI(TAG, "RX Task started - Mode=%s, Timeout=%dms",
             mode_str[s_detect_mode], MA_RX_TIMEOUT_MS);

    while (s_task_running) {
        int len = uart_read_bytes(s_config.uart_num, rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(100));

        // Check for timeout (all modes)
        if (s_rx_index > 0 && s_last_rx_time > 0) {
            int64_t now = esp_timer_get_time();
            if ((now - s_last_rx_time) >= s_rx_timeout_us) {
                // rx_span   = first byte -> now (receipt transmit + idle wait)
                // idle_wait = last byte  -> now (this is the MA_RX_TIMEOUT_MS window)
                ESP_LOGI(TAG, "Timeout - processing %d bytes (rx_span=%lldms, idle_wait=%lldms)",
                         (int)s_rx_index,
                         (long long)((now - s_first_rx_time) / 1000),
                         (long long)((now - s_last_rx_time) / 1000));
                process_ma_packet();
                s_capturing = false;
            }
        }

        if (len > 0) {
            // Track the biggest silence between chunks of the same frame.
            // Only meaningful once we already hold part of a frame.
            int64_t now_us = esp_timer_get_time();
            if (s_rx_index > 0 && s_last_rx_time > 0) {
                int64_t gap = now_us - s_last_rx_time;
                if (gap > s_max_gap_us) {
                    s_max_gap_us = gap;
                }
            }

            if (s_rx_index == 0) {
                // T0 - the analyser starting to talk begins a new cycle.
                timing_mark(TIMING_T0_MA_FIRST);
                // First byte of a new message - mark the start for timing.
                s_first_rx_time = now_us;
                // Tell whoever is listening that a frame has begun, and start
                // WM reading in parallel. What the listener does with the event
                // is not this driver's business - on the WiFi/GSM products it
                // opens the weight-capture window so the weight is collected
                // during the MA frame.
                if (s_frame_start_callback) {
                    s_frame_start_callback(s_detect_mode != MA_DETECT_TIMEOUT);
                }
                wm_uart_start();   // no-op if WM already running
            }
            s_last_rx_time = now_us;

            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buffer[i];

                if (byte == '\n') {
                    s_rx_line_count++;
                }

                // Handle based on detection mode
                switch (s_detect_mode) {
                    case MA_DETECT_PARENTHESES:
                        // Model 3000-3999: Capture between '(' and ')'
                        if (byte == '(') {
                            s_capturing = true;
                            s_rx_index = 0;  // Reset buffer
                        }
                        if (s_capturing) {
                            if (s_rx_index < sizeof(s_rx_buffer) - 1) {
                                s_rx_buffer[s_rx_index++] = (char)byte;
                            }
                        }
                        if (byte == ')' && s_capturing) {
                            s_capturing = false;
                            process_ma_packet();
                        }
                        break;

                    case MA_DETECT_NEWLINE:
                        // Model 4000-4999: End on newline
                        if (byte == '\n') {
                            process_ma_packet();
                        } else if (byte >= 0x20 && byte <= 0x7E) {
                            if (s_rx_index < sizeof(s_rx_buffer) - 1) {
                                s_rx_buffer[s_rx_index++] = (char)byte;
                            }
                        }
                        break;

                    case MA_DETECT_TIMEOUT:
                    default:
                        // Other models: Buffer ALL data like Pico2W (no filtering)
                        // Pico2W stores every character including newlines, spaces, etc.
                        if (s_rx_index < sizeof(s_rx_buffer) - 1) {
                            s_rx_buffer[s_rx_index++] = (char)byte;
                        }
                        break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "RX Task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t ma_uart_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Set defaults from Kconfig
    s_config.uart_num = MA_UART_NUM;
    s_config.tx_pin = MA_UART_TX_PIN;
    s_config.rx_pin = MA_UART_RX_PIN;
    s_config.baud_rate = MA_UART_BAUD_RATE;
    s_config.parity = 0;

    // Try to load from NVS
    nvs_handle_t nvs;
    bool config_exists = false;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        int32_t val;
        if (nvs_get_i32(nvs, NVS_KEY_BAUD, &val) == ESP_OK) {
            s_config.baud_rate = val;
            config_exists = true;
        }
        if (nvs_get_i32(nvs, NVS_KEY_PARITY, &val) == ESP_OK) {
            s_config.parity = val;
        }
        if (nvs_get_i32(nvs, NVS_KEY_STREAM_MODE, &val) == ESP_OK) {
            s_stream_mode = val;
        }
        if (nvs_get_i32(nvs, NVS_KEY_MODEL_ID, &val) == ESP_OK) {
            s_model_id = val;
        }
        nvs_close(nvs);
        ESP_LOGI(TAG, "Config loaded from NVS");
    } else {
        ESP_LOGI(TAG, "No saved config - using defaults");
    }

    // Update detection mode based on model
    update_detect_mode();

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(s_config.parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(s_config.uart_num, MA_UART_BUF_SIZE, MA_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_config.uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_initialized = true;

    ESP_LOGI(TAG, "Initialized: UART%d TX=%d RX=%d Baud=%d Model=%d",
             s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
             s_config.baud_rate, s_model_id);

    // Save defaults on first boot
    if (!config_exists) {
        ESP_LOGI(TAG, "First boot - saving default config to NVS");
        ma_uart_save_config();
    }

    return ESP_OK;
}

esp_err_t ma_uart_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    ma_uart_stop();
    uart_driver_delete(s_config.uart_num);
    s_initialized = false;

    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

esp_err_t ma_uart_start(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_running) {
        ESP_LOGW(TAG, "Task already running");
        return ESP_OK;
    }

    // Reset state for new reading cycle
    s_single_read_done = false;
    s_capturing = false;
    memset(s_last_reading, 0, sizeof(s_last_reading));
    s_rx_index = 0;
    s_last_rx_time = 0;

    // Flush UART buffers
    uart_flush_input(s_config.uart_num);

    s_task_running = true;

    BaseType_t ret = xTaskCreate(uart_rx_task, "ma_uart_rx", 6144, NULL, 5, &s_rx_task_handle);
    if (ret != pdPASS) {
        s_task_running = false;
        ESP_LOGE(TAG, "Failed to create RX task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void ma_uart_stop(void)
{
    if (!s_task_running) return;

    s_task_running = false;

    if (s_rx_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200));
        s_rx_task_handle = NULL;
    }
}

esp_err_t ma_uart_set_baud(int new_baud)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Changing baud: %d -> %d", s_config.baud_rate, new_baud);

    uart_flush(s_config.uart_num);
    uart_flush_input(s_config.uart_num);
    s_rx_index = 0;

    uart_config_t uart_config = {
        .baud_rate = new_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(s_config.parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(s_config.uart_num, &uart_config);
    if (ret == ESP_OK) {
        s_config.baud_rate = new_baud;
        vTaskDelay(pdMS_TO_TICKS(50));
        uart_flush_input(s_config.uart_num);
        ESP_LOGI(TAG, "Baud changed to %d", new_baud);
    }

    return ret;
}

void ma_uart_get_config(ma_config_t *config)
{
    if (config) {
        memcpy(config, &s_config, sizeof(ma_config_t));
    }
}

esp_err_t ma_uart_save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_i32(nvs, NVS_KEY_BAUD, s_config.baud_rate);
    nvs_set_i32(nvs, NVS_KEY_PARITY, s_config.parity);
    nvs_set_i32(nvs, NVS_KEY_STREAM_MODE, s_stream_mode);
    nvs_set_i32(nvs, NVS_KEY_MODEL_ID, s_model_id);

    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS (baud=%d, model=%d, stream=%d)",
                 s_config.baud_rate, s_model_id, s_stream_mode);
    }
    return err;
}

void ma_uart_set_stream_mode(int mode)
{
    s_stream_mode = (mode == MA_MODE_STREAM) ? MA_MODE_STREAM : MA_MODE_SINGLE;
    s_single_read_done = false;
    memset(s_last_reading, 0, sizeof(s_last_reading));

    ESP_LOGI(TAG, "Stream mode set to: %s",
             s_stream_mode == MA_MODE_STREAM ? "STREAM" : "SINGLE-READ");
}

int ma_uart_get_stream_mode(void)
{
    return s_stream_mode;
}

void ma_uart_set_model_id(int model_id)
{
    s_model_id = model_id;
    update_detect_mode();
}

int ma_uart_get_model_id(void)
{
    return s_model_id;
}

ma_detect_mode_t ma_uart_get_detect_mode(int model_id)
{
    if (model_id >= 3000 && model_id <= 3999) {
        return MA_DETECT_PARENTHESES;
    } else if (model_id >= 4000 && model_id <= 4999) {
        return MA_DETECT_NEWLINE;
    } else {
        return MA_DETECT_TIMEOUT;
    }
}

void ma_uart_set_data_callback(ma_data_callback_t callback)
{
    s_data_callback = callback;
}

void ma_uart_set_activity_callback(ma_activity_callback_t callback)
{
    s_activity_callback = callback;
}

void ma_uart_set_frame_start_callback(ma_frame_start_callback_t callback)
{
    s_frame_start_callback = callback;
}

void ma_uart_print_status(void)
{
    const char *mode_str[] = {"TIMEOUT", "PARENTHESES", "NEWLINE"};

    printf("\n");
    printf("=========================================\n");
    printf("   MA UART Status\n");
    printf("=========================================\n");
    printf("UART%d: TX=GPIO%d, RX=GPIO%d\n", s_config.uart_num, s_config.tx_pin, s_config.rx_pin);
    printf("Baud: %d, Parity: %s\n", s_config.baud_rate,
           s_config.parity == 0 ? "None" : (s_config.parity == 1 ? "Odd" : "Even"));
    printf("-----------------------------------------\n");
    printf("Model ID: %d\n", s_model_id);
    printf("Detection Mode: %s\n", mode_str[s_detect_mode]);
    printf("Stream Mode: %s\n", s_stream_mode == MA_MODE_STREAM ? "STREAM" : "SINGLE-READ");
    printf("Running: %s\n", s_task_running ? "YES" : "NO");
    printf("-----------------------------------------\n");
    // Self-diagnosis
    int age = ma_uart_get_last_packet_age_ms();
    if (age < 0) {
        printf("MA Connected: NO (no data received)\n");
    } else if (age < MA_DIAGNOSIS_TIMEOUT_MS) {
        printf("MA Connected: YES (last packet %d ms ago)\n", age);
    } else {
        printf("MA Connected: NO (timeout, last packet %d ms ago)\n", age);
    }
    printf("-----------------------------------------\n");
    printf("Stats: Packets=%lu, Filtered=%lu\n",
           (unsigned long)s_stats.packet_count,
           (unsigned long)s_stats.filtered_bytes);
    printf("=========================================\n\n");
}

void ma_uart_get_stats(ma_stats_t *stats)
{
    if (stats) {
        memcpy(stats, &s_stats, sizeof(ma_stats_t));
    }
}

void ma_uart_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(ma_stats_t));
    ESP_LOGI(TAG, "Stats reset");
}

bool ma_uart_is_connected(void)
{
    if (!s_task_running) {
        return false;
    }
    if (s_last_packet_time == 0) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - s_last_packet_time) / 1000;
    return (elapsed_ms < MA_DIAGNOSIS_TIMEOUT_MS);
}

int ma_uart_get_last_packet_age_ms(void)
{
    if (s_last_packet_time == 0) {
        return -1;
    }
    int64_t now = esp_timer_get_time();
    return (int)((now - s_last_packet_time) / 1000);
}
