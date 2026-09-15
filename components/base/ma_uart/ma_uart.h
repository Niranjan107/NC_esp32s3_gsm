/**
 * @file ma_uart.h
 * @brief Milk Analyzer UART Driver for ESP32-S3
 *
 * Features:
 * - Model-based end character detection (matches Pico2W behavior)
 *   - Model 3000-3999: Parentheses mode '(' and ')'
 *   - Model 4000-4999: Continuous newline mode '\n'
 *   - Other models: Timeout-based detection
 * - Duplicate filtering (same as WM)
 * - NVS persistence for configuration
 * - JSON output format
 *
 * Pin configuration via menuconfig:
 * - CONFIG_NCLE_MA_UART_NUM (default: 2)
 * - CONFIG_NCLE_MA_UART_TX_PIN (default: 40)
 * - CONFIG_NCLE_MA_UART_RX_PIN (default: 39)
 * - CONFIG_NCLE_MA_UART_BAUD_RATE (default: 9600)
 */

#ifndef _MA_UART_H_
#define _MA_UART_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Kconfig Defaults (if not defined in menuconfig)
// ============================================================================
#ifndef CONFIG_NCLE_MA_ENABLE
#define CONFIG_NCLE_MA_ENABLE 1
#endif

#ifndef CONFIG_NCLE_MA_UART_NUM
#define CONFIG_NCLE_MA_UART_NUM 2
#endif

#ifndef CONFIG_NCLE_MA_UART_TX_PIN
#define CONFIG_NCLE_MA_UART_TX_PIN 40
#endif

#ifndef CONFIG_NCLE_MA_UART_RX_PIN
#define CONFIG_NCLE_MA_UART_RX_PIN 39
#endif

#ifndef CONFIG_NCLE_MA_UART_BAUD_RATE
#define CONFIG_NCLE_MA_UART_BAUD_RATE 9600
#endif

#ifndef CONFIG_NCLE_MA_UART_BUF_SIZE
#define CONFIG_NCLE_MA_UART_BUF_SIZE 2048
#endif

// ============================================================================
// Configuration Macros
// ============================================================================
#define MA_UART_NUM         CONFIG_NCLE_MA_UART_NUM
#define MA_UART_TX_PIN      CONFIG_NCLE_MA_UART_TX_PIN
#define MA_UART_RX_PIN      CONFIG_NCLE_MA_UART_RX_PIN
#define MA_UART_BAUD_RATE   CONFIG_NCLE_MA_UART_BAUD_RATE
#define MA_UART_BUF_SIZE    CONFIG_NCLE_MA_UART_BUF_SIZE

// Data receive timeout (ms) - matches Pico2W MA_DATA_DELAY
#define MA_RX_TIMEOUT_MS    3000

// Floor for the adaptive receive timeout (see CONFIG_NCLE_MA_ADAPTIVE_TIMEOUT).
// The driver never lowers the idle-gap wait below this, so a fast analyser is
// not held up for the full 3 seconds after its last byte.
#define MA_RX_TIMEOUT_MIN_MS 400

// Minimum valid payload length
#define MA_MIN_PAYLOAD_LEN  3

// Default model ID: -1 (UNKNOWN) like Pico2W
#define MA_DEFAULT_MODEL_ID -1

// ============================================================================
// Data Types
// ============================================================================

/**
 * @brief MA stream mode
 */
typedef enum {
    MA_MODE_SINGLE = 0,     // Wait for duplicate, output once, stop (like WM)
    MA_MODE_STREAM = 1      // Continuous output
} ma_stream_mode_t;

/**
 * @brief MA detection mode (based on model)
 */
typedef enum {
    MA_DETECT_TIMEOUT = 0,      // Default: timeout-based (model < 3000 or > 4999)
    MA_DETECT_PARENTHESES = 1,  // Model 3000-3999: '(' and ')'
    MA_DETECT_NEWLINE = 2       // Model 4000-4999: '\n'
} ma_detect_mode_t;

/**
 * @brief MA UART configuration
 */
typedef struct {
    int uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int parity;         // 0=None, 1=Odd, 2=Even
} ma_config_t;

/**
 * @brief MA statistics
 */
typedef struct {
    uint32_t packet_count;
    uint32_t filtered_bytes;
} ma_stats_t;

/**
 * @brief Callback for MA data output (for BLE transmission)
 */
typedef void (*ma_data_callback_t)(const char *json_data, int len);

/**
 * @brief Callback for activity indication (for LED)
 */
typedef void (*ma_activity_callback_t)(void);

/**
 * @brief Callback for the FIRST byte of a new analyser frame.
 *
 * Fires the instant the analyser starts talking - before any of the frame has
 * been decoded - which is the only moment a consumer can start something that
 * must run *during* the reading (the weight-capture window does exactly that).
 * The existing activity callback cannot serve: it fires when a reading is
 * already COMPLETE.
 *
 * @param terminator_framed  true for analysers whose frames end on a terminator
 *        (PARENTHESES 3xxx / NEWLINE 4xxx), false for idle-gap framed ones
 *        (TIMEOUT 1xxx/2xxx/5xxx). Passed on every frame rather than announced
 *        once at config time, so a consumer's copy can never go stale after a
 *        model change.
 *
 * Runs on the MA RX task - keep the handler short and non-blocking.
 */
typedef void (*ma_frame_start_callback_t)(bool terminator_framed);

// ============================================================================
// Core Functions
// ============================================================================

/**
 * @brief Initialize MA UART driver
 * @return ESP_OK on success
 */
esp_err_t ma_uart_init(void);

/**
 * @brief Deinitialize MA UART driver
 * @return ESP_OK on success
 */
esp_err_t ma_uart_deinit(void);

/**
 * @brief Start MA receive task
 * @return ESP_OK on success
 */
esp_err_t ma_uart_start(void);

/**
 * @brief Stop MA receive task
 */
void ma_uart_stop(void);

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * @brief Set baud rate (runtime)
 * @param new_baud New baud rate
 * @return ESP_OK on success
 */
esp_err_t ma_uart_set_baud(int new_baud);

/**
 * @brief Get current configuration
 * @param config Output configuration structure
 */
void ma_uart_get_config(ma_config_t *config);

/**
 * @brief Save current configuration to NVS
 * @return ESP_OK on success
 */
esp_err_t ma_uart_save_config(void);

// ============================================================================
// Stream Mode Functions
// ============================================================================

/**
 * @brief Set stream mode
 * @param mode MA_MODE_SINGLE or MA_MODE_STREAM
 */
void ma_uart_set_stream_mode(int mode);

/**
 * @brief Get current stream mode
 * @return Current mode
 */
int ma_uart_get_stream_mode(void);

// ============================================================================
// Model ID Functions
// ============================================================================

/**
 * @brief Set model ID (determines detection mode)
 * @param model_id Model ID (3000-3999=parentheses, 4000-4999=newline, other=timeout)
 */
void ma_uart_set_model_id(int model_id);

/**
 * @brief Get current model ID
 * @return Current model ID
 */
int ma_uart_get_model_id(void);

/**
 * @brief Get detection mode for a model ID
 * @param model_id Model ID to check
 * @return Detection mode
 */
ma_detect_mode_t ma_uart_get_detect_mode(int model_id);

// ============================================================================
// Callback Functions
// ============================================================================

/**
 * @brief Register data callback (for BLE transmission)
 * @param callback Callback function
 */
void ma_uart_set_data_callback(ma_data_callback_t callback);

/**
 * @brief Register activity callback (for LED)
 * @param callback Callback function
 */
void ma_uart_set_activity_callback(ma_activity_callback_t callback);

/**
 * @brief Register the frame-start callback (see ma_frame_start_callback_t)
 * @param callback Callback function, or NULL to disable
 */
void ma_uart_set_frame_start_callback(ma_frame_start_callback_t callback);

// ============================================================================
// Status Functions
// ============================================================================

/**
 * @brief Print MA status to console
 */
void ma_uart_print_status(void);

/**
 * @brief Get statistics
 * @param stats Output statistics structure
 */
void ma_uart_get_stats(ma_stats_t *stats);

/**
 * @brief Reset statistics
 */
void ma_uart_reset_stats(void);

// ============================================================================
// Self-Diagnosis Functions
// ============================================================================

/**
 * @brief Check if MA is connected (data received recently)
 * @return true if connected
 */
bool ma_uart_is_connected(void);

/**
 * @brief Get time since last packet (ms)
 * @return Milliseconds since last packet, -1 if never received
 */
int ma_uart_get_last_packet_age_ms(void);

#ifdef __cplusplus
}
#endif

#endif // _MA_UART_H_
