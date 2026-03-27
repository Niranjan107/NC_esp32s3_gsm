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

// Statistics
static ma_stats_t s_stats = {0};

// Task handle
static TaskHandle_t s_rx_task_handle = NULL;
static volatile bool s_task_running = false;

// Callbacks
static ma_data_callback_t s_data_callback = NULL;
static ma_activity_callback_t s_activity_callback = NULL;

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
        return;
    }

    s_rx_buffer[s_rx_index] = '\0';

    // Pico2W compatibility: Output MA data IMMEDIATELY (no duplicate detection)
    // MA receipts are always unique, so just output and trigger WM
    ESP_LOGI(TAG, "MA data received (%d bytes) - outputting immediately", (int)s_rx_index);
    output_ma_data();

    // Trigger WM to start receiving (Pico2W dairy workflow)
    // MA reads milk → WM reads weight automatically
    ESP_LOGI(TAG, "Triggering WM to start (dairy workflow)");
    wm_uart_start();

    // Reset buffer
    s_rx_index = 0;
    s_last_rx_time = 0;
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
            if ((now - s_last_rx_time) >= MA_RX_TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout - processing %d bytes", (int)s_rx_index);
                process_ma_packet();
                s_capturing = false;
            }
        }

        if (len > 0) {
            s_last_rx_time = esp_timer_get_time();

            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buffer[i];

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
