/**
 * @file printer_uart.h
 * @brief Thermal Printer UART Component for ESP32-S3
 *
 * Implements ESC/POS thermal printer control via UART.
 * Port of Pico2W printer_pio module to ESP32-S3.
 *
 * Features:
 * - ESC/POS command support (initialize, cut, feed, formatting)
 * - Special character parsing (|n, |b, |C, etc.)
 * - Receipt storage and reprint
 * - Configurable baud rate and parity
 *
 * Hardware Configuration (Kconfig):
 * - CONFIG_NCLE_PRINTER_UART_NUM (default: 1)
 * - CONFIG_NCLE_PRINTER_UART_TX_PIN (default: 17)
 * - CONFIG_NCLE_PRINTER_UART_RX_PIN (default: 18)
 * - CONFIG_NCLE_PRINTER_UART_BAUD_RATE (default: 9600)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration Defaults (can be overridden via Kconfig)
// ============================================================================

#ifndef CONFIG_NCLE_PRINTER_UART_NUM
#define CONFIG_NCLE_PRINTER_UART_NUM 1
#endif

#ifndef CONFIG_NCLE_PRINTER_UART_TX_PIN
#define CONFIG_NCLE_PRINTER_UART_TX_PIN 17
#endif

#ifndef CONFIG_NCLE_PRINTER_UART_RX_PIN
#define CONFIG_NCLE_PRINTER_UART_RX_PIN 18
#endif

#ifndef CONFIG_NCLE_PRINTER_UART_BAUD_RATE
#define CONFIG_NCLE_PRINTER_UART_BAUD_RATE 9600
#endif

#ifndef CONFIG_NCLE_PRINTER_UART_BUF_SIZE
#define CONFIG_NCLE_PRINTER_UART_BUF_SIZE 256
#endif

#ifndef CONFIG_NCLE_PRINTER_RECEIPT_BUF_SIZE
#define CONFIG_NCLE_PRINTER_RECEIPT_BUF_SIZE 2048
#endif

// ============================================================================
// Macros
// ============================================================================

#define PRINTER_UART_NUM        CONFIG_NCLE_PRINTER_UART_NUM
#define PRINTER_UART_TX_PIN     CONFIG_NCLE_PRINTER_UART_TX_PIN
#define PRINTER_UART_RX_PIN     CONFIG_NCLE_PRINTER_UART_RX_PIN
#define PRINTER_UART_BAUD_RATE  CONFIG_NCLE_PRINTER_UART_BAUD_RATE
#define PRINTER_UART_BUF_SIZE   CONFIG_NCLE_PRINTER_UART_BUF_SIZE
#define PRINTER_RECEIPT_BUF_SIZE CONFIG_NCLE_PRINTER_RECEIPT_BUF_SIZE

// ============================================================================
// Parity Constants (matches Pico2W)
// ============================================================================

#define PRINTER_PARITY_NONE  0
#define PRINTER_PARITY_EVEN  1
#define PRINTER_PARITY_ODD   2

// ============================================================================
// Configuration Structure
// ============================================================================

typedef struct {
    int uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int parity;
} printer_config_t;

// ============================================================================
// Initialization / Deinitialization
// ============================================================================

/**
 * @brief Initialize the printer UART module
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t printer_uart_init(void);

/**
 * @brief Deinitialize the printer UART module
 * @return ESP_OK on success
 */
esp_err_t printer_uart_deinit(void);

/**
 * @brief Check if printer module is initialized
 * @return true if initialized, false otherwise
 */
bool printer_uart_is_initialized(void);

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Set printer baud rate
 * @param baud_rate New baud rate
 * @return ESP_OK on success
 */
esp_err_t printer_uart_set_baud(int baud_rate);

/**
 * @brief Set printer parity
 * @param parity PRINTER_PARITY_NONE, PRINTER_PARITY_EVEN, or PRINTER_PARITY_ODD
 * @return ESP_OK on success
 */
esp_err_t printer_uart_set_parity(int parity);

/**
 * @brief Get current printer configuration
 * @param config Pointer to config structure to fill
 */
void printer_uart_get_config(printer_config_t *config);

/**
 * @brief Save configuration to NVS
 * @return ESP_OK on success
 */
esp_err_t printer_uart_save_config(void);

// ============================================================================
// ESC/POS Commands - Basic Control
// ============================================================================

/**
 * @brief Initialize/reset printer (ESC @)
 * @return true if successful
 */
bool printer_start(void);

/**
 * @brief Print a new line (LF)
 * @return true if successful
 */
bool printer_new_line(void);

/**
 * @brief Cut paper (GS V)
 * @return true if successful
 */
bool printer_cut_paper(void);

/**
 * @brief Feed paper (ESC d n)
 * @param lines Number of lines to feed (default 4)
 * @return true if successful
 */
bool printer_feed_paper(int lines);

// ============================================================================
// ESC/POS Commands - Text Formatting
// ============================================================================

/**
 * @brief Set bold text (ESC E 1)
 * @return true if successful
 */
bool printer_set_bold(void);

/**
 * @brief End bold text (ESC E 0)
 * @return true if successful
 */
bool printer_end_bold(void);

/**
 * @brief Set center alignment (ESC a 1)
 * @return true if successful
 */
bool printer_set_center_align(void);

/**
 * @brief Set left alignment (ESC a 0)
 * @return true if successful
 */
bool printer_set_left_align(void);

/**
 * @brief Set right alignment (ESC a 2)
 * @return true if successful
 */
bool printer_set_right_align(void);

/**
 * @brief Set double height text (GS ! 16)
 * @return true if successful
 */
bool printer_set_double_height(void);

/**
 * @brief Set normal height text (GS ! 0)
 * @return true if successful
 */
bool printer_set_normal_height(void);

/**
 * @brief Set underline text (ESC - 1)
 * @return true if successful
 */
bool printer_set_underline(void);

/**
 * @brief End underline text (ESC - 0)
 * @return true if successful
 */
bool printer_end_underline(void);

/**
 * @brief Set inverse text - white on black (GS B 1)
 * @return true if successful
 */
bool printer_set_inverse(void);

/**
 * @brief End inverse text (GS B 0)
 * @return true if successful
 */
bool printer_end_inverse(void);

/**
 * @brief Set Font A (ESC ! 0)
 * @return true if successful
 */
bool printer_set_font_a(void);

/**
 * @brief Set Font B (ESC ! 1)
 * @return true if successful
 */
bool printer_set_font_b(void);

// ============================================================================
// High-Level Printing Functions
// ============================================================================

/**
 * @brief Print text with special character parsing
 *
 * Special characters (pipe escape sequences):
 *   |s - Start/reset printer
 *   |n - New line
 *   |c - Cut paper
 *   |f - Feed paper (4 lines)
 *   |b - Bold ON
 *   |B - Bold OFF
 *   |C - Center align
 *   |l - Left align
 *   |r - Right align
 *   |d - Double height ON
 *   |D - Double height OFF (normal)
 *   |u - Underline ON
 *   |U - Underline OFF
 *   |i - Inverse ON
 *   |I - Inverse OFF
 *   |t - Tab
 *   || - Literal pipe character
 *   |e - Form feed
 *   |p - Space
 *   |G - Font A
 *   |g - Font B
 *
 * @param text Text to print with special characters
 * @return true if successful
 */
bool printer_print_with_special_chars(const char *text);

/**
 * @brief Print raw text without special character parsing
 * @param text Text to print
 * @return true if successful
 */
bool printer_print_raw(const char *text);

/**
 * @brief Print a single character
 * @param c Character to print
 * @return true if successful
 */
bool printer_print_char(char c);

/**
 * @brief Reprint the last receipt
 * @return true if successful, false if no receipt stored
 */
bool printer_reprint_last_receipt(void);

/**
 * @brief Store receipt data for later reprinting
 * @param data Receipt data to store
 * @return true if successful
 */
bool printer_store_receipt(const char *data);

/**
 * @brief Clear stored receipt data
 */
void printer_clear_receipt(void);

// ============================================================================
// Status Functions
// ============================================================================

/**
 * @brief Check if printer is connected/ready
 * @return true if printer is ready
 */
bool printer_check_status(void);

/**
 * @brief Print status information
 */
void printer_print_status(void);

// ============================================================================
// Callbacks
// ============================================================================

/**
 * @brief Callback type for print completion
 */
typedef void (*printer_callback_t)(bool success);

/**
 * @brief Set callback for print completion
 * @param callback Callback function
 */
void printer_set_callback(printer_callback_t callback);

#ifdef __cplusplus
}
#endif
