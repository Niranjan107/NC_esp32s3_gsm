/**
 * @file wm_uart.c
 * @brief Weighing Machine (WM) UART Component Implementation
 *
 * PURPOSE:
 * This module handles serial communication with electronic weighing scales/machines
 * used in dairy collection centers. It receives weight data, processes it, and
 * forwards it to the mobile app via BLE.
 *
 * WHY THIS MODULE EXISTS:
 * Dairy collection centers use electronic weighing machines to measure milk quantity.
 * These machines output weight data via RS232/TTL serial port. This module:
 * 1. Receives serial data from the weighing machine
 * 2. Processes and cleans the data (remove prefixes, filter garbage)
 * 3. Detects stable readings (duplicate detection)
 * 4. Forwards clean weight data to mobile app via BLE
 *
 * FEATURES:
 * - UART receive with newline + timeout detection
 * - Prefix stripping (ST, GS, US, NT, OL, W:, WT:, etc.)
 * - Non-printable character filtering
 * - Duplicate reading detection (for stable weight)
 * - JSON wrapping for mobile app
 * - NVS persistence for configuration
 * - Two modes: SINGLE-READ and STREAM
 *
 * OPERATING MODES:
 * 1. SINGLE-READ (stream=0):
 *    - Wait for stable reading (same value twice in a row)
 *    - Output once and stop receiving
 *    - Used for individual weighments
 *    - Mobile app triggers restart for next weighment
 *
 * 2. STREAM (stream=1):
 *    - Continuously receive and output weight data
 *    - Optional sample rate to reduce output frequency
 *    - Used for real-time weight display
 *
 * DATA FLOW:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                     Weighing Machine (RS232/TTL)                        │
 * │  Outputs: "ST,GS,  12.350 kg\r\n" or "W:12.35\n" etc.                   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              Serial Data
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         wm_uart.c (THIS FILE)                           │
 * │                                                                          │
 * │  1. uart_rx_task() - Receives bytes, detects packet boundaries          │
 * │  2. process_wm_packet() - Validates packet size                         │
 * │  3. strip_wm_data() - Removes prefixes like "ST,GS," "W:", etc.         │
 * │  4. Duplicate detection (SINGLE mode) or sample rate (STREAM mode)      │
 * │  5. output_wm_data() - Creates JSON and calls callbacks                 │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              JSON output
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  {"device":"wm","data":"12.350 kg","model":9000 }                       │
 * │                                                                          │
 * │  → USB Console (puts)                                                   │
 * │  → BLE (s_data_callback → ble_spp_send)                                 │
 * │  → LED Activity (s_activity_callback)                                   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * KNOWN WM OUTPUT FORMATS (handled by prefix stripping):
 * - "ST,GS,  12.350 kg" → "12.350 kg"
 * - "US,NT, +12.35"     → "12.35"
 * - "W:12.35"           → "12.35"
 * - "WT:12.35"          → "12.35"
 * - "GROSS:12.35"       → "12.35"
 */

#include "wm_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
#include "soft_uart_rmt.h"
#endif

static const char *TAG = "WM_UART";

// ============================================================================
// NVS Keys
// ============================================================================
#define NVS_NAMESPACE       "wm_config"
#define NVS_KEY_BAUD        "baud"
#define NVS_KEY_PARITY      "parity"
#define NVS_KEY_STREAM_MODE "stream"
#define NVS_KEY_SAMPLE_RATE "sample_rate"
#define NVS_KEY_MODEL_ID    "model_id"
#define NVS_KEY_END_CHAR    "end_char"

// ============================================================================
// Internal State
//
// PURPOSE: Module-level variables for WM UART operation
// ============================================================================

/**
 * UART Configuration
 * - Holds current UART port, pins, baud rate, parity settings
 * - Loaded from NVS on init, updated via API
 */
static wm_config_t s_config;
static bool s_initialized = false;

/**
 * Receive Buffers
 * - s_rx_buffer: Accumulates raw bytes until newline/timeout
 * - s_clean_buffer: Holds data after prefix stripping
 * - s_json_buffer: Holds final JSON output
 */
static char s_rx_buffer[256];           // Raw receive buffer
static size_t s_rx_index = 0;           // Current position in rx buffer
static char s_clean_buffer[256];        // Cleaned data buffer (prefixes removed)
static char s_json_buffer[512];         // JSON output buffer

/**
 * Duplicate Detection (for SINGLE-READ mode)
 * When two consecutive readings are the same, we consider
 * the weight "stable" and output it.
 */
static char s_last_reading[256];        // Last processed value for comparison

/**
 * Timeout Tracking
 * If no end character received within timeout, process buffer anyway.
 * Handles WMs that don't send proper line terminators.
 */
#define WM_RX_TIMEOUT_US    (WM_RX_TIMEOUT_MS * 1000)

static int64_t s_last_rx_time = 0;      // Timestamp of last received byte

/** Statistics for debugging and status display */
static wm_stats_t s_stats = {0};

/**
 * FreeRTOS Task Handle
 * The RX task runs continuously, reading UART and processing packets.
 */
static TaskHandle_t s_rx_task_handle = NULL;
static volatile bool s_task_running = false;

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
static soft_uart_rmt_handle_t s_soft_uart_handle = NULL;
#endif

/* Raw-byte callback (NULL = disabled, zero overhead path).
 * Used by wm_uart_validator during Phase 1 soft-UART validation. */
static wm_uart_raw_byte_cb_t s_raw_byte_cb  = NULL;
static void                 *s_raw_byte_ctx = NULL;

void wm_uart_set_raw_byte_callback(wm_uart_raw_byte_cb_t cb, void *ctx)
{
    s_raw_byte_cb  = cb;
    s_raw_byte_ctx = ctx;
}

/**
 * Callbacks for Data and Activity
 * - s_data_callback: Called with JSON data (for BLE transmission)
 * - s_activity_callback: Called on activity (for LED blinking)
 */
static wm_data_callback_t s_data_callback = NULL;
static wm_activity_callback_t s_activity_callback = NULL;

/**
 * Stream Mode Configuration
 * - stream_mode: 0=SINGLE-READ (one stable reading), 1=STREAM (continuous)
 * - sample_rate: In STREAM mode, output every Nth sample
 * - sample_count: Counter for sample rate
 * - single_read_done: Flag to prevent duplicate output in SINGLE mode
 */
static int s_stream_mode = WM_DEFAULT_STREAM_MODE;  // 0=single-read, 1=stream
static int s_sample_rate = WM_DEFAULT_SAMPLE_RATE;  // Output every Nth sample
static int s_sample_count = 0;                       // Current sample counter
static volatile bool s_single_read_done = false;     // Flag to prevent duplicate output

/**
 * Model ID
 * Identifies the WM model type in JSON output.
 * Set via BLE/USB command from mobile app.
 */
static int s_model_id = WM_MODEL_ID;                 // Default from header

/**
 * End Character Configuration
 * Defines what character terminates a weight reading packet.
 * Most WMs use \n or \r, but some use other characters.
 */
static wm_end_char_t s_end_char = WM_DEFAULT_END_CHAR;  // Default: newline

/**
 * Self-Diagnosis State
 * Tracks when last valid data was received to determine
 * if WM is connected/responding.
 */
static int64_t s_last_packet_time = 0;               // Timestamp of last valid packet
#define WM_DIAGNOSIS_TIMEOUT_MS  5000                // 5 seconds without data = disconnected

// ============================================================================
// Known WM Prefixes to Strip
//
// PURPOSE: Remove standard weighing machine prefixes from raw data
//
// COMMON WM OUTPUT FORMATS:
// - "ST,GS,  12.350 kg\r\n"  → ST=Stable, GS=Gross weight
// - "US,NT, -5.00\r\n"       → US=Unstable, NT=Net weight
// - "OL,GS,\r\n"             → OL=Overload
// - "W:12.350\n"             → Simple weight format
// - "GROSS:12.350\n"         → Gross weight format
//
// After stripping: "12.350 kg" or "-5.00" etc.
// ============================================================================
static const char *s_prefixes[] = {
    "ST,", "US,", "GS,", "NT,", "OL,",  // Status prefixes with comma
    "ST ", "US ", "GS ", "NT ", "OL ",  // Status prefixes with space
    "W:", "WT:", "NET:", "GROSS:",       // Weight prefixes
    NULL
};

// ============================================================================
// Internal Functions
//
// PURPOSE: Helper functions for WM data processing
// ============================================================================

/**
 * @brief Strip known WM prefixes and trim whitespace from raw data
 *
 * PURPOSE:
 * Weighing machines often output data with status prefixes like:
 * "ST,GS,  +12.350 kg" where ST=Stable, GS=Gross
 * This function removes those prefixes to get clean weight data: "12.350 kg"
 *
 * INPUT:
 * @param src      - Source string with prefixes (e.g., "ST,GS,  +12.350 kg")
 * @param dst      - Destination buffer for cleaned string
 * @param dst_size - Size of destination buffer
 *
 * OUTPUT:
 * Cleaned string in dst (e.g., "12.350 kg")
 *
 * PROCESSING:
 * 1. Skip leading whitespace
 * 2. Strip known prefixes (may have multiple, e.g., "ST,GS,")
 * 3. Skip remaining whitespace and commas
 * 4. Skip leading '+' (keep '-' for negative values)
 * 5. Copy remaining characters
 * 6. Trim trailing whitespace
 */
static void strip_wm_data(const char *src, char *dst, size_t dst_size)
{
    if (!src || !dst || dst_size == 0) return;

    const char *p = src;
    bool found_prefix;

    // Skip known prefixes (may have multiple)
    do {
        found_prefix = false;

        // Skip leading whitespace
        while (*p == ' ' || *p == '\t') p++;

        // Check for known prefixes
        for (int i = 0; s_prefixes[i] != NULL; i++) {
            size_t prefix_len = strlen(s_prefixes[i]);
            if (strncasecmp(p, s_prefixes[i], prefix_len) == 0) {
                p += prefix_len;
                found_prefix = true;
                break;
            }
        }
    } while (found_prefix);

    // Skip remaining leading whitespace/commas
    while (*p == ' ' || *p == '\t' || *p == ',') p++;

    // Skip leading + (keep - for negative)
    if (*p == '+') p++;

    // Copy to destination
    size_t i = 0;
    while (*p && i < dst_size - 1) {
        dst[i++] = *p++;
    }
    dst[i] = '\0';

    // Trim trailing whitespace
    while (i > 0 && (dst[i-1] == ' ' || dst[i-1] == '\t')) {
        dst[--i] = '\0';
    }
}

/**
 * @brief Check if byte is printable ASCII
 */
static inline bool is_printable_byte(uint8_t byte)
{
    return (byte >= 0x20 && byte <= 0x7E);
}

/**
 * @brief Get byte value for end character enum
 * @return Byte value, or -1 for NONE (timeout only), or -2 for LF_CR (special)
 */
static int get_end_char_byte(wm_end_char_t end_char)
{
    switch (end_char) {
        case WM_END_CHAR_LF_CR: return -2;      // Special: both \n and \r
        case WM_END_CHAR_LF:    return '\n';    // 0x0A
        case WM_END_CHAR_CR:    return '\r';    // 0x0D
        case WM_END_CHAR_FF:    return '\f';    // 0x0C
        case WM_END_CHAR_VT:    return '\v';    // 0x0B
        case WM_END_CHAR_ETB:   return 0x17;    // ETB
        case WM_END_CHAR_ESC:   return 0x1B;    // ESC
        case WM_END_CHAR_CSI:   return 0x9B;    // CSI
        case WM_END_CHAR_SPACE: return ' ';     // 0x20
        case WM_END_CHAR_NULL:  return 0x00;    // NULL
        case WM_END_CHAR_NONE:
        default:                return -1;      // No terminator
    }
}

/**
 * @brief Check if received byte is the configured end character
 */
static bool is_end_char(uint8_t byte)
{
    int end_byte = get_end_char_byte(s_end_char);

    // LF_CR mode (-2): accept both \n and \r
    if (end_byte == -2) {
        return (byte == '\n' || byte == '\r');
    }

    // NONE mode (-1): no character acts as terminator
    if (end_byte < 0) {
        return false;
    }

    return (byte == (uint8_t)end_byte);
}

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
 * @brief Output WM data (JSON + callbacks)
 */
static void output_wm_data(void)
{
    // Update last packet time for diagnosis
    s_last_packet_time = esp_timer_get_time();

    // Create JSON (use configured model ID from BLE/USB)
    // Note: Space before closing } matches Pico2W format exactly
    int json_len = snprintf(s_json_buffer, sizeof(s_json_buffer),
                            "{\"device\":\"wm\",\"data\":\"%s\",\"model\":%d }",
                            s_clean_buffer, s_model_id);

    // Log debug info (use DEBUG level to reduce console spam)
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "===== PACKET #%lu =====", (unsigned long)s_stats.packet_count);
    ESP_LOGD(TAG, "Raw:   [%s]", s_rx_buffer);
    ESP_LOGD(TAG, "Clean: [%s]", s_clean_buffer);
    ESP_LOGD(TAG, "Mode: %s, Sample: %d/%d",
             s_stream_mode ? "STREAM" : "SINGLE", s_sample_count, s_sample_rate);
    ESP_LOGD(TAG, "======================");

    // Output JSON to console (like Pico2W puts())
    puts(s_json_buffer);

    // Call data callback (for BLE transmission)
    if (s_data_callback) {
        s_data_callback(s_json_buffer, json_len);
    }

    // Call activity callback (for LED)
    if (s_activity_callback) {
        s_activity_callback();
    }
}

/**
 * @brief Process complete WM packet
 */
static void process_wm_packet(void)
{
    if (s_rx_index < WM_MIN_PAYLOAD_LEN) {
        // Output error message matching Pico2W format
        if (s_rx_index > 0) {
            s_rx_buffer[s_rx_index] = '\0';
            ESP_LOGW(TAG, "Incomplete packet (%d bytes): [%s]", (int)s_rx_index, s_rx_buffer);

            // Create error JSON (matches Pico2W format)
            char error_json[256];
            int len = snprintf(error_json, sizeof(error_json),
                "{\"status_message\":\"Incomplete wm data packet\",\"data\":\"%s\"}",
                s_rx_buffer);

            // Output to console (like Pico2W puts())
            puts(error_json);

            // Also send to callback (for BLE transmission)
            if (s_data_callback) {
                s_data_callback(error_json, len);
            }
        }
        s_rx_index = 0;
        s_last_rx_time = 0;
        return;
    }

    s_rx_buffer[s_rx_index] = '\0';

    // Strip prefixes and trim
    strip_wm_data(s_rx_buffer, s_clean_buffer, sizeof(s_clean_buffer));

    s_stats.packet_count++;

    // =========================================================================
    // =========================================================================
    // SINGLE-READ MODE (stream=0): Wait for duplicate, output once, stop
    // Matches Pico2W behavior: waits for same value twice (stable reading)
    // =========================================================================
    if (s_stream_mode == WM_MODE_SINGLE) {
        // Check if we already completed single-read (prevent duplicate outputs)
        if (s_single_read_done) {
            ESP_LOGD(TAG, "Single-read already done, ignoring packet");
            s_rx_index = 0;
            s_last_rx_time = 0;
            return;
        }

        // Check for duplicate (same value as previous)
        if (strcmp(s_clean_buffer, s_last_reading) == 0) {
            // Duplicate detected = stable reading - output and stop
            s_single_read_done = true;  // Set flag BEFORE output to prevent race
            ESP_LOGI(TAG, "Stable reading detected (duplicate) - outputting");
            output_wm_data();
            wm_uart_stop();
            ESP_LOGI(TAG, "Single-read mode: UART stopped after stable reading");
        } else {
            // New/different value - store it, wait for duplicate
            strncpy(s_last_reading, s_clean_buffer, sizeof(s_last_reading) - 1);
            s_last_reading[sizeof(s_last_reading) - 1] = '\0';
            ESP_LOGD(TAG, "Waiting for stable reading: [%s]", s_clean_buffer);
        }
    }
    // =========================================================================
    // STREAM MODE (stream=1): Continuous output with sampling rate
    // =========================================================================
    else {
        s_sample_count++;

        // Output every Nth sample
        if (s_sample_count >= s_sample_rate) {
            s_sample_count = 0;
            output_wm_data();
        } else {
            ESP_LOGD(TAG, "Sample %d/%d skipped: [%s]",
                     s_sample_count, s_sample_rate, s_clean_buffer);
        }

        // Update last reading (for stats/reference)
        strncpy(s_last_reading, s_clean_buffer, sizeof(s_last_reading) - 1);
        s_last_reading[sizeof(s_last_reading) - 1] = '\0';
    }

    // Reset buffer
    s_rx_index = 0;
    s_last_rx_time = 0;
}

/**
 * @brief Process one received byte through the WM packet parser.
 *
 * Shared by the HW UART RX task and (when CONFIG_NCLE_WM_USE_SOFT_UART is
 * enabled) the soft-UART byte callback. The source of the byte doesn't matter
 * to the parser - it runs the same end-char detection, printable filter, and
 * packet-close logic for both paths.
 */
static void wm_uart_process_byte(uint8_t byte, int64_t ts_us)
{
    s_last_rx_time = ts_us;

    if (s_raw_byte_cb) {
        s_raw_byte_cb(byte, ts_us, s_raw_byte_ctx);
    }

    if (is_end_char(byte)) {
        process_wm_packet();
    } else if (is_printable_byte(byte)) {
        if (s_rx_index < sizeof(s_rx_buffer) - 1) {
            s_rx_buffer[s_rx_index++] = (char)byte;
        }
    } else {
        s_stats.filtered_bytes++;
    }
}

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
/**
 * @brief Soft UART byte callback - converts each decoded byte into a
 * wm_uart_process_byte() call, same way the HW RX task does.
 */
static void wm_soft_uart_byte_cb(const soft_uart_rmt_rx_t *rx, void *ctx)
{
    (void)ctx;
    /* Skip frames with invalid start/stop bits (line noise). */
    if (rx->frame_err) return;
    wm_uart_process_byte(rx->byte, rx->ts_us);
}

/**
 * @brief Timeout monitor for soft-UART mode.
 *
 * In HW-UART mode the RX task loop naturally polls for timeouts on every
 * iteration. With the soft UART we receive bytes via async callbacks, so
 * we need a small periodic task to check for "no byte for WM_RX_TIMEOUT_MS"
 * and flush any accumulated buffer as a packet.
 */
static void wm_soft_timeout_task(void *arg)
{
    ESP_LOGI(TAG, "Soft UART timeout task started - EndChar=%s, Timeout=%dms",
             wm_uart_end_char_to_string(s_end_char), WM_RX_TIMEOUT_MS);
    while (s_task_running) {
        if (s_rx_index > 0 && s_last_rx_time > 0) {
            int64_t now = esp_timer_get_time();
            if ((now - s_last_rx_time) >= WM_RX_TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout - processing %d bytes", (int)s_rx_index);
                process_wm_packet();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Soft UART timeout task stopped");
    vTaskDelete(NULL);
}
#endif /* CONFIG_NCLE_WM_USE_SOFT_UART */

/**
 * @brief UART RX Task
 */
static void uart_rx_task(void *arg)
{
    uint8_t rx_buffer[128];

    ESP_LOGI(TAG, "RX Task started - EndChar=%s, Timeout=%dms",
             wm_uart_end_char_to_string(s_end_char), WM_RX_TIMEOUT_MS);

    while (s_task_running) {
        int len = uart_read_bytes(s_config.uart_num, rx_buffer, sizeof(rx_buffer) - 1, pdMS_TO_TICKS(100));

        // Check for timeout first
        if (s_rx_index > 0 && s_last_rx_time > 0) {
            int64_t now = esp_timer_get_time();
            if ((now - s_last_rx_time) >= WM_RX_TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout - processing %d bytes", (int)s_rx_index);
                process_wm_packet();
            }
        }

        if (len > 0) {
            /* All bytes in one uart_read_bytes chunk share approximately
             * the same arrival timestamp. */
            int64_t chunk_ts = esp_timer_get_time();
            for (int i = 0; i < len; i++) {
                wm_uart_process_byte(rx_buffer[i], chunk_ts);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "RX Task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Public API Implementation
//
// PURPOSE: External functions called by main.c and cmd_parser.c
// These functions provide the interface for:
// - Initialization and cleanup
// - Starting/stopping data reception
// - Configuration (baud rate, stream mode, model ID)
// - Status queries
// ============================================================================

/**
 * @brief Initialize the WM UART module
 *
 * PURPOSE:
 * Set up the UART hardware for communicating with the weighing machine.
 * Loads configuration from NVS or uses defaults.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success
 *
 * INITIALIZATION:
 * 1. Load config from NVS (baud, parity, pins, stream mode, etc.)
 * 2. Configure UART driver with loaded/default settings
 * 3. Set up GPIO pins for RX (and TX if enabled)
 * 4. Save defaults to NVS on first boot
 *
 * NOTE: Does NOT start the receive task. Call wm_uart_start() to begin receiving.
 *
 * CALLED FROM: app_main() in main.c
 */
esp_err_t wm_uart_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Set defaults from Kconfig
    s_config.uart_num = WM_UART_NUM;
    s_config.tx_pin = WM_UART_TX_PIN;
    s_config.rx_pin = WM_UART_RX_PIN;
    s_config.baud_rate = WM_UART_BAUD_RATE;
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
        // GPIO pins always use Kconfig defaults (not saved to NVS)
        // Load stream mode settings
        if (nvs_get_i32(nvs, NVS_KEY_STREAM_MODE, &val) == ESP_OK) {
            s_stream_mode = val;
        }
        if (nvs_get_i32(nvs, NVS_KEY_SAMPLE_RATE, &val) == ESP_OK) {
            s_sample_rate = val;
        }
        if (nvs_get_i32(nvs, NVS_KEY_MODEL_ID, &val) == ESP_OK) {
            s_model_id = val;
        }
        if (nvs_get_i32(nvs, NVS_KEY_END_CHAR, &val) == ESP_OK) {
            // Validate end_char is within valid range (0-10)
            if (val >= 0 && val <= WM_END_CHAR_NULL) {
                s_end_char = (wm_end_char_t)val;
            } else {
                ESP_LOGW(TAG, "Invalid end_char in NVS (%d), using default", (int)val);
                s_end_char = WM_DEFAULT_END_CHAR;
            }
        }
        nvs_close(nvs);
        ESP_LOGI(TAG, "Config loaded from NVS (end_char=%d)", (int)s_end_char);
    } else {
        ESP_LOGI(TAG, "No saved config - using defaults");
        config_exists = false;
    }

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
    /* Soft UART path: skip hardware UART driver install; init RMT-based
     * software UART on the configured RX GPIO. The hardware UART
     * peripheral (UART0 by default) remains free for other use. */
    soft_uart_rmt_config_t sw_cfg = {
        .gpio_num  = s_config.rx_pin,
        .baud_rate = s_config.baud_rate,
        .data_bits = 8,
        .stop_bits = 1,
        .parity    = 0,
    };
    esp_err_t sw_err = soft_uart_rmt_init(&sw_cfg, &s_soft_uart_handle);
    if (sw_err != ESP_OK) {
        ESP_LOGE(TAG, "soft_uart_rmt_init failed: %d", sw_err);
        return sw_err;
    }
    soft_uart_rmt_register_byte_cb(s_soft_uart_handle, wm_soft_uart_byte_cb, NULL);
    s_initialized = true;
    ESP_LOGI(TAG, "Initialized: SOFT-UART RX=%d Baud=%d (UART peripheral unused)",
             s_config.rx_pin, s_config.baud_rate);
#else
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(s_config.parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(s_config.uart_num, WM_UART_BUF_SIZE, WM_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_config.uart_num, &uart_config));

    int tx_pin = (s_config.tx_pin >= 0) ? s_config.tx_pin : UART_PIN_NO_CHANGE;
    ESP_ERROR_CHECK(uart_set_pin(s_config.uart_num, tx_pin, s_config.rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_initialized = true;

#if WM_TX_ENABLED
    ESP_LOGI(TAG, "Initialized: UART%d TX=%d RX=%d Baud=%d Parity=%d",
             s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
             s_config.baud_rate, s_config.parity);
#else
    ESP_LOGI(TAG, "Initialized: UART%d TX=Disabled RX=%d Baud=%d Parity=%d",
             s_config.uart_num, s_config.rx_pin,
             s_config.baud_rate, s_config.parity);
#endif
#endif /* CONFIG_NCLE_WM_USE_SOFT_UART */

    // Save defaults on first boot (when no config exists in NVS)
    if (!config_exists) {
        ESP_LOGI(TAG, "First boot - saving default config to NVS");
        wm_uart_save_config();
    }

    return ESP_OK;
}

esp_err_t wm_uart_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    wm_uart_stop();
#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
    if (s_soft_uart_handle) {
        soft_uart_rmt_deinit(s_soft_uart_handle);
        s_soft_uart_handle = NULL;
    }
#else
    uart_driver_delete(s_config.uart_num);
#endif
    s_initialized = false;

    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

esp_err_t wm_uart_start(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_running) {
        ESP_LOGW(TAG, "Task already running - skipping start");
        return ESP_OK;
    }

    // Wait a bit to ensure previous task is fully cleaned up (if any)
    // This handles the case where MA triggers WM restart right after WM stops itself
    if (s_rx_task_handle != NULL) {
        ESP_LOGI(TAG, "Waiting for previous task to cleanup...");
        vTaskDelay(pdMS_TO_TICKS(100));
        s_rx_task_handle = NULL;
    }

    ESP_LOGI(TAG, "=== WM UART STARTING ===");

    // Reset single-read state for new reading cycle
    s_single_read_done = false;
    s_sample_count = 0;
    memset(s_last_reading, 0, sizeof(s_last_reading));
    s_rx_index = 0;
    s_last_rx_time = 0;

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
    /* Soft UART path: start the RMT receiver + the timeout-monitor task. */
    s_task_running = true;
    esp_err_t sw_err = soft_uart_rmt_start(s_soft_uart_handle);
    if (sw_err != ESP_OK) {
        s_task_running = false;
        ESP_LOGE(TAG, "soft_uart_rmt_start failed: %d", sw_err);
        return sw_err;
    }
    BaseType_t ret = xTaskCreate(wm_soft_timeout_task, "wm_uart_rx",
                                 3072, NULL, 5, &s_rx_task_handle);
    if (ret != pdPASS) {
        s_task_running = false;
        soft_uart_rmt_stop(s_soft_uart_handle);
        ESP_LOGE(TAG, "Failed to create soft-UART timeout task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WM RX (soft UART) started - ready to receive (stream=%d)", s_stream_mode);
    return ESP_OK;
#else
    // Flush UART buffers
    uart_flush_input(s_config.uart_num);

    s_task_running = true;

    BaseType_t ret = xTaskCreate(uart_rx_task, "wm_uart_rx", 6144, NULL, 5, &s_rx_task_handle);
    if (ret != pdPASS) {
        s_task_running = false;
        ESP_LOGE(TAG, "Failed to create RX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WM RX task created - ready to receive (stream=%d)", s_stream_mode);
    return ESP_OK;
#endif
}

void wm_uart_stop(void)
{
    if (!s_task_running) {
        ESP_LOGD(TAG, "Stop called but task not running");
        return;
    }

    ESP_LOGI(TAG, "=== WM UART STOPPING ===");
    s_task_running = false;

    // Give task time to exit cleanly
    if (s_rx_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200));
        // Note: Task deletes itself via vTaskDelete(NULL)
        // We just clear the handle here
    }
#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
    if (s_soft_uart_handle) {
        soft_uart_rmt_stop(s_soft_uart_handle);
    }
#endif
    ESP_LOGI(TAG, "WM stopped - ready for next customer");
}

esp_err_t wm_uart_set_baud(int new_baud)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Changing baud: %d -> %d", s_config.baud_rate, new_baud);

    // Clear software buffer
    s_rx_index = 0;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));

#ifdef CONFIG_NCLE_WM_USE_SOFT_UART
    esp_err_t ret = soft_uart_rmt_set_baud(s_soft_uart_handle, new_baud);
    if (ret == ESP_OK) {
        s_config.baud_rate = new_baud;
        ESP_LOGI(TAG, "Soft UART baud changed to %d", new_baud);
    }
#else
    // Flush buffers
    uart_flush(s_config.uart_num);
    uart_flush_input(s_config.uart_num);

    // Reconfigure
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
        s_rx_index = 0;

        ESP_LOGI(TAG, "Baud changed to %d", new_baud);
    }
#endif

    return ret;
}

void wm_uart_get_config(wm_config_t *config)
{
    if (config) {
        memcpy(config, &s_config, sizeof(wm_config_t));
    }
}

esp_err_t wm_uart_save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Save UART config
    nvs_set_i32(nvs, NVS_KEY_BAUD, s_config.baud_rate);
    nvs_set_i32(nvs, NVS_KEY_PARITY, s_config.parity);
    // GPIO pins always use Kconfig defaults (not saved to NVS)

    // Save stream mode settings
    nvs_set_i32(nvs, NVS_KEY_STREAM_MODE, s_stream_mode);
    nvs_set_i32(nvs, NVS_KEY_SAMPLE_RATE, s_sample_rate);
    nvs_set_i32(nvs, NVS_KEY_MODEL_ID, s_model_id);
    nvs_set_i32(nvs, NVS_KEY_END_CHAR, (int32_t)s_end_char);

    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS (baud=%d, mode=%d, rate=%d, model=%d, end_char=%d)",
                 s_config.baud_rate, s_stream_mode, s_sample_rate, s_model_id, (int)s_end_char);
    }
    return err;
}

void wm_uart_get_stats(wm_stats_t *stats)
{
    if (stats) {
        memcpy(stats, &s_stats, sizeof(wm_stats_t));
    }
}

void wm_uart_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(wm_stats_t));
    ESP_LOGI(TAG, "Stats reset");
}

void wm_uart_set_data_callback(wm_data_callback_t callback)
{
    s_data_callback = callback;
}

void wm_uart_set_activity_callback(wm_activity_callback_t callback)
{
    s_activity_callback = callback;
}

void wm_uart_print_status(void)
{
    printf("\n");
    printf("=========================================\n");
    printf("   WM UART Status\n");
    printf("=========================================\n");
#if WM_TX_ENABLED
    printf("UART%d: TX=GPIO%d, RX=GPIO%d\n", s_config.uart_num, s_config.tx_pin, s_config.rx_pin);
#else
    printf("UART%d: TX=Disabled, RX=GPIO%d\n", s_config.uart_num, s_config.rx_pin);
#endif
    printf("Baud: %d, Parity: %s\n", s_config.baud_rate,
           s_config.parity == 0 ? "None" : (s_config.parity == 1 ? "Odd" : "Even"));
    printf("-----------------------------------------\n");
    if (s_stream_mode == WM_MODE_STREAM) {
        printf("Mode: STREAM (output every %d samples)\n", s_sample_rate);
    } else {
        printf("Mode: SINGLE-READ (wait for duplicate, then stop)\n");
    }
    printf("Model ID: %d\n", s_model_id);
    printf("End Char: %s (e=%d)\n", wm_uart_end_char_to_string(s_end_char), (int)s_end_char);
    printf("Running: %s\n", s_task_running ? "YES" : "NO");
    printf("-----------------------------------------\n");
    // Self-diagnosis
    int age = wm_uart_get_last_packet_age_ms();
    if (age < 0) {
        printf("WM Connected: NO (no data received)\n");
    } else if (age < WM_DIAGNOSIS_TIMEOUT_MS) {
        printf("WM Connected: YES (last packet %d ms ago)\n", age);
    } else {
        printf("WM Connected: NO (timeout, last packet %d ms ago)\n", age);
    }
    printf("-----------------------------------------\n");
    printf("Stats: Packets=%lu, Filtered=%lu\n",
           (unsigned long)s_stats.packet_count,
           (unsigned long)s_stats.filtered_bytes);
    printf("=========================================\n\n");
}

// ============================================================================
// Stream Mode API
// ============================================================================

void wm_uart_set_stream_mode(int mode)
{
    s_stream_mode = (mode == WM_MODE_STREAM) ? WM_MODE_STREAM : WM_MODE_SINGLE;
    s_sample_count = 0;  // Reset sample counter
    memset(s_last_reading, 0, sizeof(s_last_reading));  // Clear last reading

    ESP_LOGI(TAG, "Stream mode set to: %s",
             s_stream_mode == WM_MODE_STREAM ? "STREAM" : "SINGLE-READ");
}

int wm_uart_get_stream_mode(void)
{
    return s_stream_mode;
}

void wm_uart_set_sample_rate(int rate)
{
    if (rate < 1) rate = 1;
    if (rate > 100) rate = 100;
    s_sample_rate = rate;
    s_sample_count = 0;  // Reset counter

    ESP_LOGI(TAG, "Sample rate set to: %d (output every %d samples)", rate, rate);
}

int wm_uart_get_sample_rate(void)
{
    return s_sample_rate;
}

// ============================================================================
// Model ID API
// ============================================================================

void wm_uart_set_model_id(int model_id)
{
    s_model_id = model_id;
    ESP_LOGI(TAG, "Model ID set to: %d", model_id);
}

int wm_uart_get_model_id(void)
{
    return s_model_id;
}

// ============================================================================
// Self-Diagnosis API
// ============================================================================

bool wm_uart_is_connected(void)
{
    // Not running = not connected
    if (!s_task_running) {
        return false;
    }

    // No packets ever received = not connected
    if (s_last_packet_time == 0) {
        return false;
    }

    // Check if we received data recently
    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - s_last_packet_time) / 1000;

    return (elapsed_ms < WM_DIAGNOSIS_TIMEOUT_MS);
}

int wm_uart_get_last_packet_age_ms(void)
{
    if (s_last_packet_time == 0) {
        return -1;  // Never received
    }

    int64_t now = esp_timer_get_time();
    return (int)((now - s_last_packet_time) / 1000);
}

// ============================================================================
// End Character Configuration API
// ============================================================================

void wm_uart_set_end_char(wm_end_char_t end_char)
{
    if (end_char > WM_END_CHAR_NULL) {
        end_char = WM_DEFAULT_END_CHAR;  // Invalid value, use default
    }
    s_end_char = end_char;

    int byte_val = wm_uart_end_char_to_byte(end_char);
    if (byte_val == -2) {
        ESP_LOGI(TAG, "End char set to: %s (e=%d)",
                 wm_uart_end_char_to_string(end_char), (int)end_char);
    } else {
        ESP_LOGI(TAG, "End char set to: %s (e=%d, byte=0x%02X)",
                 wm_uart_end_char_to_string(end_char), (int)end_char, byte_val);
    }
}

wm_end_char_t wm_uart_get_end_char(void)
{
    return s_end_char;
}

const char* wm_uart_end_char_to_string(wm_end_char_t end_char)
{
    switch (end_char) {
        case WM_END_CHAR_LF_CR: return "LF+CR (\\n or \\r)";
        case WM_END_CHAR_LF:    return "LF (\\n)";
        case WM_END_CHAR_CR:    return "CR (\\r)";
        case WM_END_CHAR_FF:    return "FF (\\f)";
        case WM_END_CHAR_NONE:  return "NONE (timeout)";
        case WM_END_CHAR_VT:    return "VT (\\v)";
        case WM_END_CHAR_ETB:   return "ETB (0x17)";
        case WM_END_CHAR_ESC:   return "ESC (0x1B)";
        case WM_END_CHAR_CSI:   return "CSI (0x9B)";
        case WM_END_CHAR_SPACE: return "SPACE";
        case WM_END_CHAR_NULL:  return "NULL (0x00)";
        default:                return "UNKNOWN";
    }
}

int wm_uart_end_char_to_byte(wm_end_char_t end_char)
{
    return get_end_char_byte(end_char);
}
