/**
 * @file wm_uart.h
 * @brief Weighing Machine UART Component for ESP32-S3
 *
 * Features:
 * - UART communication with weighing machines
 * - Newline and timeout-based packet detection
 * - Prefix stripping (ST, GS, US, NT, OL, W:, WT:, etc.)
 * - Non-printable character filtering
 * - Duplicate reading filtering
 * - JSON wrapping for BLE transmission
 * - NVS configuration persistence
 * - Activity LED indication
 */

#ifndef _WM_UART_H_
#define _WM_UART_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration from Kconfig (menuconfig)
// ============================================================================
#define WM_UART_NUM         CONFIG_NCLE_WM_UART_NUM
#define WM_UART_RX_PIN      CONFIG_NCLE_WM_UART_RX_PIN
#define WM_UART_BAUD_RATE   CONFIG_NCLE_WM_UART_BAUD_RATE
#define WM_UART_BUF_SIZE    CONFIG_NCLE_WM_UART_BUF_SIZE

#ifdef CONFIG_NCLE_WM_UART_TX_ENABLE
#define WM_UART_TX_PIN      CONFIG_NCLE_WM_UART_TX_PIN
#define WM_TX_ENABLED       1
#else
#define WM_UART_TX_PIN      (-1)
#define WM_TX_ENABLED       0
#endif

// ============================================================================
// Constants
// ============================================================================
#define WM_RX_TIMEOUT_MS    1000    // Packet timeout (ms)
#define WM_MIN_PAYLOAD_LEN  3       // Minimum valid payload length
#define WM_MODEL_ID         -1      // Default model ID: -1 (UNKNOWN) like Pico2W

// ============================================================================
// Stream Mode (matches Pico2W behavior)
// ============================================================================
#define WM_MODE_SINGLE      0       // Single-read: wait for duplicate, output once, stop
#define WM_MODE_STREAM      1       // Stream: continuous output every Nth sample

#define WM_DEFAULT_STREAM_MODE  WM_MODE_SINGLE  // Default mode
#define WM_DEFAULT_SAMPLE_RATE  10              // Stream: output every 10 samples (like Pico2W)

// ============================================================================
// Types
// ============================================================================

/**
 * @brief WM UART runtime configuration
 */
typedef struct {
    int uart_num;       ///< UART port number
    int tx_pin;         ///< TX GPIO pin (-1 = disabled)
    int rx_pin;         ///< RX GPIO pin
    int baud_rate;      ///< Baud rate
    int parity;         ///< Parity: 0=none, 1=odd, 2=even
} wm_config_t;

/**
 * @brief WM UART statistics
 */
typedef struct {
    uint32_t packet_count;      ///< Valid packets received
    uint32_t filtered_bytes;    ///< Non-printable bytes filtered
    uint32_t duplicate_count;   ///< Duplicate readings skipped
} wm_stats_t;

/**
 * @brief Callback for processed WM data
 * @param json_data JSON-wrapped data string
 * @param len Length of JSON data
 */
typedef void (*wm_data_callback_t)(const char *json_data, size_t len);

/**
 * @brief Callback for activity indication (LED blink)
 */
typedef void (*wm_activity_callback_t)(void);

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize WM UART component
 *
 * Loads config from NVS (or uses Kconfig defaults), initializes UART.
 *
 * @return ESP_OK on success
 */
esp_err_t wm_uart_init(void);

/**
 * @brief Deinitialize WM UART component
 * @return ESP_OK on success
 */
esp_err_t wm_uart_deinit(void);

/**
 * @brief Start WM UART receive task
 * @return ESP_OK on success
 */
esp_err_t wm_uart_start(void);

/**
 * @brief Stop WM UART receive task
 */
void wm_uart_stop(void);

/**
 * @brief Change baud rate at runtime
 * @param new_baud New baud rate
 * @return ESP_OK on success
 */
esp_err_t wm_uart_set_baud(int new_baud);

/**
 * @brief Get current configuration
 * @param config Pointer to config structure to fill
 */
void wm_uart_get_config(wm_config_t *config);

/**
 * @brief Save current config to NVS
 * @return ESP_OK on success
 */
esp_err_t wm_uart_save_config(void);

/**
 * @brief Get statistics
 * @param stats Pointer to stats structure to fill
 */
void wm_uart_get_stats(wm_stats_t *stats);

/**
 * @brief Reset statistics counters
 */
void wm_uart_reset_stats(void);

/**
 * @brief Register callback for processed WM data
 * @param callback Callback function (NULL to disable)
 */
void wm_uart_set_data_callback(wm_data_callback_t callback);

/**
 * @brief Register callback for activity indication
 * @param callback Callback function (NULL to disable)
 */
void wm_uart_set_activity_callback(wm_activity_callback_t callback);

/**
 * @brief Print status and config (for debugging)
 */
void wm_uart_print_status(void);

// ============================================================================
// Stream Mode API
// ============================================================================

/**
 * @brief Set stream mode
 * @param mode WM_MODE_SINGLE (0) or WM_MODE_STREAM (1)
 */
void wm_uart_set_stream_mode(int mode);

/**
 * @brief Get current stream mode
 * @return WM_MODE_SINGLE (0) or WM_MODE_STREAM (1)
 */
int wm_uart_get_stream_mode(void);

/**
 * @brief Set sampling rate for stream mode
 * @param rate Output every Nth sample (1 = every sample, 10 = every 10th)
 */
void wm_uart_set_sample_rate(int rate);

/**
 * @brief Get current sampling rate
 * @return Sampling rate
 */
int wm_uart_get_sample_rate(void);

// ============================================================================
// Model ID API
// ============================================================================

/**
 * @brief Set WM model ID (for JSON output)
 * @param model_id Model ID number
 */
void wm_uart_set_model_id(int model_id);

/**
 * @brief Get current model ID
 * @return Model ID
 */
int wm_uart_get_model_id(void);

// ============================================================================
// Self-Diagnosis API
// ============================================================================

/**
 * @brief Check if WM is connected and sending data
 *
 * Returns true if valid packets have been received within
 * the last 5 seconds while the UART task is running.
 *
 * @return true if connected, false if disconnected/no data
 */
bool wm_uart_is_connected(void);

/**
 * @brief Get time since last valid packet
 * @return Milliseconds since last packet, or -1 if never received
 */
int wm_uart_get_last_packet_age_ms(void);

// ============================================================================
// End Character Configuration API
// ============================================================================

/**
 * End character options (packet terminators)
 * e=0 is DEFAULT and works with most weighing machines
 */
typedef enum {
    WM_END_CHAR_LF_CR   = 0,    // '\n' OR '\r' - BOTH (default, works with most WMs)
    WM_END_CHAR_LF      = 1,    // '\n' (0x0A) - Newline only
    WM_END_CHAR_CR      = 2,    // '\r' (0x0D) - Carriage return only
    WM_END_CHAR_FF      = 3,    // '\f' (0x0C) - Form feed
    WM_END_CHAR_NONE    = 4,    // Timeout only (no terminator)
    WM_END_CHAR_VT      = 5,    // '\v' (0x0B) - Vertical tab
    WM_END_CHAR_ETB     = 6,    // 0x17 - End of transmission block
    WM_END_CHAR_ESC     = 7,    // 0x1B - Escape
    WM_END_CHAR_CSI     = 8,    // 0x9B - Control sequence introducer
    WM_END_CHAR_SPACE   = 9,    // ' ' (0x20) - Space
    WM_END_CHAR_NULL    = 10,   // 0x00 - Null
} wm_end_char_t;

#define WM_DEFAULT_END_CHAR  WM_END_CHAR_LF_CR  // Default: both \n and \r

/**
 * @brief Set packet end character (terminator)
 * @param end_char End character enum value (WM_END_CHAR_xxx)
 */
void wm_uart_set_end_char(wm_end_char_t end_char);

/**
 * @brief Get current end character setting
 * @return Current end character enum value
 */
wm_end_char_t wm_uart_get_end_char(void);

/**
 * @brief Get name string for end character
 * @param end_char End character enum value
 * @return Human-readable name string
 */
const char* wm_uart_end_char_to_string(wm_end_char_t end_char);

/**
 * @brief Get actual byte value for end character
 * @param end_char End character enum value
 * @return Byte value (0x00-0xFF), or -1 for NONE (timeout only)
 */
int wm_uart_end_char_to_byte(wm_end_char_t end_char);

#ifdef __cplusplus
}
#endif

#endif // _WM_UART_H_
