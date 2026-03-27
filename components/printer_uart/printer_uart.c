/**
 * @file printer_uart.c
 * @brief Thermal Printer UART Component Implementation for ESP32-S3
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * This file handles all communication with ESC/POS thermal printers.
 * It receives text from mobile app (via BLE) and sends it to the printer.
 *
 * ============================================================================
 * WHAT THIS FILE DOES:
 * ============================================================================
 * 1. Initializes UART for printer communication
 * 2. Sends ESC/POS commands (bold, center, cut paper, etc.)
 * 3. Parses special pipe escape sequences (|n, |b, |C, etc.)
 * 4. Stores last receipt for reprint functionality
 * 5. Saves configuration (baud rate, parity) to NVS
 *
 * ============================================================================
 * ESC/POS PROTOCOL:
 * ============================================================================
 * ESC/POS is a standard protocol for thermal printers.
 * Commands start with ESC (0x1B) or GS (0x1D) followed by command bytes.
 *
 * Examples:
 * - ESC @ (0x1B 0x40) = Initialize/reset printer
 * - ESC E 1 (0x1B 0x45 0x01) = Bold ON
 * - ESC a 1 (0x1B 0x61 0x01) = Center align
 * - GS V (0x1D 0x56) = Cut paper
 *
 * ============================================================================
 * PIPE ESCAPE SEQUENCES (used by mobile app):
 * ============================================================================
 * Instead of sending raw ESC/POS bytes, mobile app sends text with pipe escapes:
 *
 *   |s = Start/reset printer (ESC @)
 *   |n = New line (LF)
 *   |c = Cut paper (GS V)
 *   |f = Feed 4 lines (ESC d 4)
 *   |b = Bold ON (ESC E 1)
 *   |B = Bold OFF (ESC E 0)
 *   |C = Center align (ESC a 1)
 *   |l = Left align (ESC a 0)
 *   |r = Right align (ESC a 2)
 *   |d = Double height ON (GS ! 16)
 *   |D = Double height OFF (GS ! 0)
 *   |u = Underline ON (ESC - 1)
 *   |U = Underline OFF (ESC - 0)
 *   |i = Inverse ON (GS B 1)
 *   |I = Inverse OFF (GS B 0)
 *   |g = Font B (small) (ESC ! 1)
 *   |G = Font A (large) (ESC ! 0)
 *   |t = Tab
 *   || = Literal pipe character
 *
 * ============================================================================
 * EXAMPLE MOBILE APP COMMAND:
 * ============================================================================
 * {"command":"print_receipt","data":"|s|g NITARA Dairy |n Amount: Rs.100 |n|c"}
 *
 * This will:
 * 1. |s = Reset printer
 * 2. |g = Set small font
 * 3. " NITARA Dairy " = Print text
 * 4. |n = New line
 * 5. " Amount: Rs.100 " = Print text
 * 6. |n = New line
 * 7. |c = Cut paper
 *
 * ============================================================================
 * DATA FLOW:
 * ============================================================================
 * Mobile App → BLE → cmd_parser → printer_print_with_special_chars() → UART → Printer
 *
 * ============================================================================
 * HARDWARE CONNECTION:
 * ============================================================================
 * ESP32-S3 GPIO17 (TX) → Printer RX
 * ESP32-S3 GPIO18 (RX) → Printer TX (optional, for status)
 * ESP32-S3 GND → Printer GND
 */

#include "printer_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "PRINTER";

// ============================================================================
// NVS Keys (for saving configuration to flash)
// ============================================================================
#define NVS_NAMESPACE       "printer_cfg"  // NVS namespace for printer config
#define NVS_KEY_BAUD        "baud"         // Key for baud rate
#define NVS_KEY_PARITY      "parity"       // Key for parity setting

// ============================================================================
// ESC/POS Command Bytes
// ============================================================================
// These are the actual byte values sent to the printer

#define ESC     0x1B    // Escape - starts most ESC/POS commands
#define GS      0x1D    // Group Separator - starts GS commands
#define LF      0x0A    // Line Feed - moves paper up one line
#define FF      0x0C    // Form Feed - ejects page
#define TAB     0x09    // Tab - horizontal tab

// ============================================================================
// Internal State Variables
// ============================================================================

static printer_config_t s_config;                          // Current printer configuration
static bool s_initialized = false;                          // Is printer initialized?
static SemaphoreHandle_t s_mutex = NULL;                   // Mutex for thread safety

static char s_receipt_buffer[PRINTER_RECEIPT_BUF_SIZE];    // Stores last receipt for reprint
static bool s_receipt_stored = false;                       // Is a receipt stored?

static printer_callback_t s_callback = NULL;                // Print completion callback

// ============================================================================
// Internal Helper Functions
// ============================================================================

/**
 * @brief Convert parity enum to ESP-IDF UART parity setting
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Mobile app sends parity as integer (0, 1, 2).
 * ESP-IDF UART driver needs uart_parity_t enum.
 * This function converts between them.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param parity - Parity setting from mobile app
 *                 0 = None (most common)
 *                 1 = Odd
 *                 2 = Even
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP-IDF uart_parity_t value:
 *         UART_PARITY_DISABLE (default)
 *         UART_PARITY_ODD
 *         UART_PARITY_EVEN
 *
 * ============================================================================
 * NOTE:
 * ============================================================================
 * Most thermal printers use 8N1 (8 data bits, No parity, 1 stop bit).
 * Only change parity if your specific printer requires it.
 */
static uart_parity_t get_uart_parity(int parity)
{
    switch (parity) {
        case PRINTER_PARITY_ODD:  return UART_PARITY_ODD;   // parity=1
        case PRINTER_PARITY_EVEN: return UART_PARITY_EVEN;  // parity=2
        default:                  return UART_PARITY_DISABLE; // parity=0 (default)
    }
}

/**
 * @brief Send raw bytes to printer via UART
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Low-level function to send bytes to the printer.
 * All other send functions eventually call this.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param data - Pointer to bytes to send
 * @param len  - Number of bytes to send
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if all bytes sent successfully
 * @return false if error or printer not initialized
 *
 * ============================================================================
 * BEHAVIOR:
 * ============================================================================
 * 1. Check if initialized
 * 2. Write bytes to UART
 * 3. Wait for TX buffer to empty (ensure bytes are actually sent)
 */
static bool send_bytes(const uint8_t *data, size_t len)
{
    // Safety checks
    if (!s_initialized || !data || len == 0) {
        return false;
    }

    // Write to UART
    int written = uart_write_bytes(s_config.uart_num, data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return false;
    }

    // Wait for TX to complete (ensures bytes are actually sent before returning)
    uart_wait_tx_done(s_config.uart_num, pdMS_TO_TICKS(100));
    return true;
}

/**
 * @brief Send a single byte to printer
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Helper to send one byte. Used for sending individual characters.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param byte - Single byte to send (0x00-0xFF)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if sent successfully
 * @return false if error
 */
static bool send_byte(uint8_t byte)
{
    return send_bytes(&byte, 1);
}

/**
 * @brief Send ESC/POS command with 1-3 bytes and optional delay
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Most ESC/POS commands are 1-3 bytes. This function handles them all.
 * Also adds a delay after command (some printers need time to process).
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param b1       - First byte (usually ESC or GS, or 0 to skip)
 * @param b2       - Second byte (command code, or 0 to skip)
 * @param b3       - Third byte (parameter, or 0 to skip)
 * @param delay_ms - Milliseconds to wait after sending
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if command sent successfully
 * @return false if error
 *
 * ============================================================================
 * EXAMPLES:
 * ============================================================================
 * send_command(ESC, '@', 0, 10)    → Sends 0x1B 0x40 (initialize printer)
 * send_command(ESC, 'E', 1, 5)     → Sends 0x1B 0x45 0x01 (bold ON)
 * send_command(LF, 0, 0, 2)        → Sends 0x0A (new line)
 *
 * ============================================================================
 * THREAD SAFETY:
 * ============================================================================
 * Uses mutex to prevent multiple threads from sending simultaneously.
 * Important when BLE and console might both try to print.
 */
static bool send_command(uint8_t b1, uint8_t b2, uint8_t b3, uint32_t delay_ms)
{
    if (!s_initialized) {
        return false;
    }

    // Take mutex for thread safety
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    bool success = true;

    // Send first byte if non-zero
    if (b1) {
        if (!send_byte(b1)) success = false;
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay between bytes
    }

    // Send second byte if non-zero and first succeeded
    if (b2 && success) {
        if (!send_byte(b2)) success = false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Send third byte if non-zero and previous succeeded
    if (b3 && success) {
        if (!send_byte(b3)) success = false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Optional delay after command (printer processing time)
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Release mutex
    xSemaphoreGive(s_mutex);
    return success;
}

/**
 * @brief Send a text string to printer
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Send plain text (no escape processing) directly to printer.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param str - Null-terminated string to send
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if sent successfully
 * @return false if error
 */
static bool send_string(const char *str)
{
    if (!s_initialized || !str) {
        return false;
    }

    size_t len = strlen(str);
    if (len == 0) {
        return true;  // Empty string = success (nothing to do)
    }

    return send_bytes((const uint8_t *)str, len);
}

// ============================================================================
// Initialization / Deinitialization
// ============================================================================

/**
 * @brief Initialize the printer UART module
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Sets up UART hardware for printer communication.
 * Must be called before any other printer functions.
 *
 * ============================================================================
 * WHAT IT DOES:
 * ============================================================================
 * 1. Creates mutex for thread safety
 * 2. Sets default configuration (GPIO pins, baud rate)
 * 3. Loads saved configuration from NVS (if exists)
 * 4. Installs UART driver
 * 5. Configures UART parameters (baud, parity, etc.)
 * 6. Sets GPIO pins
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None (uses configuration from Kconfig/NVS)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK if successful
 * @return ESP_FAIL if mutex creation failed
 * @return Other ESP error codes for UART failures
 *
 * ============================================================================
 * DEFAULT CONFIGURATION:
 * ============================================================================
 * - UART1
 * - TX: GPIO17, RX: GPIO18
 * - Baud: 9600
 * - Parity: None
 * - 8 data bits, 1 stop bit
 *
 * ============================================================================
 * CALLED BY:
 * ============================================================================
 * app_main() during startup
 */
esp_err_t printer_uart_init(void)
{
    // Prevent double initialization
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create mutex for thread safety (BLE and console might print simultaneously)
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Set defaults from Kconfig
    s_config.uart_num = PRINTER_UART_NUM;
    s_config.tx_pin = PRINTER_UART_TX_PIN;
    s_config.rx_pin = PRINTER_UART_RX_PIN;
    s_config.baud_rate = PRINTER_UART_BAUD_RATE;
    s_config.parity = PRINTER_PARITY_NONE;

    // Try to load saved configuration from NVS
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        int32_t val;
        if (nvs_get_i32(nvs, NVS_KEY_BAUD, &val) == ESP_OK) {
            s_config.baud_rate = val;  // Use saved baud rate
        }
        if (nvs_get_i32(nvs, NVS_KEY_PARITY, &val) == ESP_OK) {
            s_config.parity = val;  // Use saved parity
        }
        nvs_close(nvs);
        ESP_LOGI(TAG, "Config loaded from NVS");
    }

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(s_config.parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    // Install UART driver with TX and RX buffers
    err = uart_driver_install(s_config.uart_num, PRINTER_UART_BUF_SIZE, PRINTER_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_mutex);
        return err;
    }

    // Apply UART configuration
    err = uart_param_config(s_config.uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        uart_driver_delete(s_config.uart_num);
        vSemaphoreDelete(s_mutex);
        return err;
    }

    // Set GPIO pins
    err = uart_set_pin(s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(s_config.uart_num);
        vSemaphoreDelete(s_mutex);
        return err;
    }

    // Clear receipt buffer
    memset(s_receipt_buffer, 0, sizeof(s_receipt_buffer));
    s_receipt_stored = false;

    s_initialized = true;

    ESP_LOGI(TAG, "Initialized: UART%d TX=GPIO%d RX=GPIO%d Baud=%d Parity=%d",
             s_config.uart_num, s_config.tx_pin, s_config.rx_pin,
             s_config.baud_rate, s_config.parity);

    return ESP_OK;
}

/**
 * @brief Deinitialize the printer UART module
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Cleanup resources used by printer module.
 * Call when shutting down or reinitializing.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK always
 */
esp_err_t printer_uart_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    // Delete UART driver
    uart_driver_delete(s_config.uart_num);

    // Delete mutex
    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

/**
 * @brief Check if printer module is initialized
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Quick check if printer is ready to use.
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if initialized and ready
 * @return false if not initialized
 */
bool printer_uart_is_initialized(void)
{
    return s_initialized;
}

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * @brief Set printer baud rate at runtime
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Change baud rate without restarting device.
 * Called when mobile app sends printer_port_config command.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param baud_rate - New baud rate (e.g., 9600, 19200, 38400, 115200)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK if successful
 * @return ESP_ERR_INVALID_STATE if not initialized
 *
 * ============================================================================
 * COMMON PRINTER BAUD RATES:
 * ============================================================================
 * - 9600  : Most common default
 * - 19200 : Some faster printers
 * - 38400 : High-speed printers
 * - 1200  : Some older printers
 */
esp_err_t printer_uart_set_baud(int baud_rate)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Changing baud: %d -> %d", s_config.baud_rate, baud_rate);

    // Take mutex to prevent printing during reconfiguration
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(s_config.parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(s_config.uart_num, &uart_config);
    if (ret == ESP_OK) {
        s_config.baud_rate = baud_rate;
        ESP_LOGI(TAG, "Baud changed to %d", baud_rate);
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

/**
 * @brief Set printer parity at runtime
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Change parity setting without restarting device.
 * Called when mobile app sends printer_port_config command.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param parity - Parity setting:
 *                 0 = None (most common)
 *                 1 = Odd
 *                 2 = Even
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK if successful
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if parity value invalid
 */
esp_err_t printer_uart_set_parity(int parity)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parity value
    if (parity < 0 || parity > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Changing parity: %d -> %d", s_config.parity, parity);

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = get_uart_parity(parity),
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(s_config.uart_num, &uart_config);
    if (ret == ESP_OK) {
        s_config.parity = parity;
        ESP_LOGI(TAG, "Parity changed to %d", parity);
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

/**
 * @brief Get current printer configuration
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Read current configuration (used by get_current_config command).
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param config - Pointer to structure to fill with current config
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Fills config structure with current settings
 */
void printer_uart_get_config(printer_config_t *config)
{
    if (config) {
        memcpy(config, &s_config, sizeof(printer_config_t));
    }
}

/**
 * @brief Save current configuration to NVS
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Persist configuration to flash so it survives power cycles.
 * Called after mobile app changes configuration.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None (saves current s_config)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK if saved successfully
 * @return Error code if NVS operation failed
 */
esp_err_t printer_uart_save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_i32(nvs, NVS_KEY_BAUD, s_config.baud_rate);
    nvs_set_i32(nvs, NVS_KEY_PARITY, s_config.parity);

    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS");
    }
    return err;
}

// ============================================================================
// ESC/POS Commands - Basic Control
// ============================================================================

/**
 * @brief Initialize/reset printer (ESC @)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Resets printer to default state. Clears print buffer and resets all
 * formatting (bold, underline, alignment, etc.) to defaults.
 *
 * ============================================================================
 * WHEN TO USE:
 * ============================================================================
 * - At start of every receipt (ensures clean state)
 * - After errors to reset printer
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |s = calls this function
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC @ = 0x1B 0x40
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if command sent successfully
 */
bool printer_start(void)
{
    ESP_LOGD(TAG, "CMD: Initialize printer (ESC @)");
    return send_command(ESC, '@', 0, 10);  // ESC @ with 10ms delay
}

/**
 * @brief Print a new line (LF)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Moves paper up one line. Like pressing Enter.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |n = calls this function
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * LF = 0x0A
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if command sent successfully
 */
bool printer_new_line(void)
{
    ESP_LOGD(TAG, "CMD: New line (LF)");
    return send_command(LF, 0, 0, 2);  // Just LF byte with 2ms delay
}

/**
 * @brief Cut paper (GS V)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Cuts the paper (if printer has cutter). Used at end of receipt.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |c = calls this function
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * GS V = 0x1D 0x56
 *
 * ============================================================================
 * NOTE:
 * ============================================================================
 * Not all printers have cutters. On printers without cutter,
 * this command is ignored or might feed paper slightly.
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if command sent successfully
 */
bool printer_cut_paper(void)
{
    ESP_LOGD(TAG, "CMD: Cut paper (GS V)");
    return send_command(GS, 'V', 0, 10);  // GS V with 10ms delay
}

/**
 * @brief Feed paper by N lines (ESC d n)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Feeds paper forward by specified number of lines.
 * Useful to add space before cutting.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |f = calls this with lines=4
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param lines - Number of lines to feed (1-255)
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC d n = 0x1B 0x64 n
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if command sent successfully
 */
bool printer_feed_paper(int lines)
{
    // Clamp to valid range
    if (lines < 1) lines = 1;
    if (lines > 255) lines = 255;

    ESP_LOGD(TAG, "CMD: Feed %d lines (ESC d %d)", lines, lines);
    return send_command(ESC, 'd', (uint8_t)lines, 10);
}

// ============================================================================
// ESC/POS Commands - Text Formatting
// ============================================================================

/**
 * @brief Enable bold text (ESC E 1)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Makes subsequent text bold (thicker/darker).
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |b = calls this function (bold ON)
 * |B = calls printer_end_bold() (bold OFF)
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC E 1 = 0x1B 0x45 0x01
 *
 * ============================================================================
 * USAGE:
 * ============================================================================
 * |b Bold Text |B Normal Text
 *     ↑ bold        ↑ not bold
 */
bool printer_set_bold(void)
{
    ESP_LOGD(TAG, "CMD: Bold ON (ESC E 1)");
    return send_command(ESC, 'E', 1, 5);
}

/**
 * @brief Disable bold text (ESC E 0)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |B = calls this function
 */
bool printer_end_bold(void)
{
    ESP_LOGD(TAG, "CMD: Bold OFF (ESC E 0)");
    return send_command(ESC, 'E', 0, 5);
}

/**
 * @brief Center align text (ESC a 1)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Centers text on the paper. Good for headers.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |C = calls this function
 * |l = left align (default)
 * |r = right align
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC a 1 = 0x1B 0x61 0x01
 */
bool printer_set_center_align(void)
{
    ESP_LOGD(TAG, "CMD: Center align (ESC a 1)");
    return send_command(ESC, 'a', 1, 5);
}

/**
 * @brief Left align text (ESC a 0) - Default alignment
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |l = calls this function
 */
bool printer_set_left_align(void)
{
    ESP_LOGD(TAG, "CMD: Left align (ESC a 0)");
    return send_command(ESC, 'a', 0, 5);
}

/**
 * @brief Right align text (ESC a 2)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |r = calls this function
 */
bool printer_set_right_align(void)
{
    ESP_LOGD(TAG, "CMD: Right align (ESC a 2)");
    return send_command(ESC, 'a', 2, 5);
}

/**
 * @brief Double height text (GS ! 16)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Makes text twice as tall. Good for important info.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |d = calls this function (double ON)
 * |D = calls printer_set_normal_height() (double OFF)
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * GS ! 16 = 0x1D 0x21 0x10
 */
bool printer_set_double_height(void)
{
    ESP_LOGD(TAG, "CMD: Double height (GS ! 0x10)");
    return send_command(GS, '!', 0x10, 5);
}

/**
 * @brief Normal height text (GS ! 0)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |D = calls this function
 */
bool printer_set_normal_height(void)
{
    ESP_LOGD(TAG, "CMD: Normal height (GS ! 0)");
    return send_command(GS, '!', 0, 5);
}

/**
 * @brief Enable underline (ESC - 1)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |u = calls this function (underline ON)
 * |U = calls printer_end_underline() (underline OFF)
 */
bool printer_set_underline(void)
{
    ESP_LOGD(TAG, "CMD: Underline ON (ESC - 1)");
    return send_command(ESC, '-', 1, 5);
}

/**
 * @brief Disable underline (ESC - 0)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |U = calls this function
 */
bool printer_end_underline(void)
{
    ESP_LOGD(TAG, "CMD: Underline OFF (ESC - 0)");
    return send_command(ESC, '-', 0, 5);
}

/**
 * @brief Enable inverse (white on black) printing (GS B 1)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Prints white text on black background.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |i = calls this function (inverse ON)
 * |I = calls printer_end_inverse() (inverse OFF)
 */
bool printer_set_inverse(void)
{
    ESP_LOGD(TAG, "CMD: Inverse ON (GS B 1)");
    return send_command(GS, 'B', 1, 5);
}

/**
 * @brief Disable inverse printing (GS B 0)
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |I = calls this function
 */
bool printer_end_inverse(void)
{
    ESP_LOGD(TAG, "CMD: Inverse OFF (GS B 0)");
    return send_command(GS, 'B', 0, 5);
}

/**
 * @brief Set Font A - Large/Standard font
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Font A is the larger, standard font.
 * Approximately 12 characters per inch.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |G = calls this function (Font A - large)
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC ! 0 = 0x1B 0x21 0x00
 */
bool printer_set_font_a(void)
{
    ESP_LOGD(TAG, "CMD: Font A (ESC ! 0)");
    // Reset printer first to clear any settings, then set font
    send_command(ESC, '@', 0, 5);
    return send_command(ESC, 'E', 0, 5);
}

/**
 * @brief Set Font B - Small/Condensed font
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Font B is the smaller, condensed font.
 * Approximately 17 characters per inch.
 * Allows more text on narrow receipt paper.
 *
 * ============================================================================
 * PIPE ESCAPE:
 * ============================================================================
 * |g = calls this function (Font B - small)
 *
 * ============================================================================
 * ESC/POS COMMAND:
 * ============================================================================
 * ESC ! 1 = 0x1B 0x21 0x01
 *
 * ============================================================================
 * MOBILE APP USAGE:
 * ============================================================================
 * Mobile app always uses |g (Font B) because receipts have
 * long text that needs to fit on narrow paper.
 */
bool printer_set_font_b(void)
{
    ESP_LOGD(TAG, "CMD: Font B (ESC ! 1)");
    return send_command(ESC, '!', 1, 5);
}

// ============================================================================
// High-Level Printing Functions
// ============================================================================

/**
 * @brief Print a single character
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Print one character to the printer.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param c - Character to print (ASCII)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if sent successfully
 */
bool printer_print_char(char c)
{
    if (!s_initialized) {
        return false;
    }
    return send_command((uint8_t)c, 0, 0, 2);  // Send char with 2ms delay
}

/**
 * @brief Print raw text without escape processing
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Print text directly without processing pipe escapes.
 * Used for sending pre-formatted text.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param text - Text string to print
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if sent successfully
 */
bool printer_print_raw(const char *text)
{
    if (!s_initialized || !text) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool success = send_string(text);
    xSemaphoreGive(s_mutex);

    return success;
}

/**
 * @brief Print text with pipe escape sequence processing
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * This is the MAIN PRINTING FUNCTION used by mobile app.
 * Processes pipe escape sequences and sends ESC/POS commands.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param text - Text with pipe escapes like:
 *               "|s|g NITARA Dairy |n Amount: Rs.100 |n|c"
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if printed successfully
 * @return false if error occurred
 *
 * ============================================================================
 * PIPE ESCAPE REFERENCE:
 * ============================================================================
 *   |s = Start/reset printer (ESC @)
 *   |n = New line (LF)
 *   |c = Cut paper (GS V)
 *   |f = Feed 4 lines
 *   |b = Bold ON
 *   |B = Bold OFF
 *   |C = Center align
 *   |l = Left align
 *   |r = Right align
 *   |d = Double height ON
 *   |D = Double height OFF (normal)
 *   |u = Underline ON
 *   |U = Underline OFF
 *   |i = Inverse ON
 *   |I = Inverse OFF
 *   |g = Font B (small)
 *   |G = Font A (large)
 *   |t = Tab
 *   || = Literal pipe character '|'
 *   |e = Form feed
 *   |p = Space
 *
 * ============================================================================
 * EXAMPLE:
 * ============================================================================
 * Input:  "|s|C|b NITARA |B|n Amount: 100 |n|n|c"
 * Output on printer:
 *         [printer reset]
 *               NITARA          (centered, bold)
 *         Amount: 100          (normal)
 *
 *         [paper cut]
 *
 * ============================================================================
 * CALLED BY:
 * ============================================================================
 * cmd_parser.c when it receives print_receipt command from mobile app
 *
 * ============================================================================
 * DATA FLOW:
 * ============================================================================
 * Mobile App → BLE → cmd_parser → THIS FUNCTION → UART → Printer
 */
bool printer_print_with_special_chars(const char *text)
{
    // Validate input
    if (!s_initialized || !text) {
        ESP_LOGW(TAG, "Print failed: %s", !s_initialized ? "not initialized" : "null text");
        return false;
    }

    // Empty string = success (nothing to print)
    if (*text == '\0') {
        return true;
    }

    ESP_LOGI(TAG, "Printing receipt (%d bytes)", strlen(text));

    // Store receipt for reprint functionality
    printer_store_receipt(text);

    // Always reset printer at start for clean state
    if (!printer_start()) {
        if (s_callback) s_callback(false);
        return false;
    }

    // Process text character by character
    const char *p = text;
    while (*p) {
        // Check for pipe escape sequence
        if (*p == '|') {
            p++;  // Skip the pipe character
            if (!*p) break;  // End of string after pipe

            // Process the escape character
            bool cmd_ok = true;
            switch (*p) {
                // ============================================
                // Basic Control
                // ============================================
                case 's':  // |s = Start/reset printer
                    cmd_ok = printer_start();
                    break;

                case 'n':  // |n = New line
                    cmd_ok = printer_new_line();
                    break;

                case 'c':  // |c = Cut paper
                    cmd_ok = printer_cut_paper();
                    break;

                case 'f':  // |f = Feed paper (4 lines)
                    cmd_ok = printer_feed_paper(4);
                    break;

                // ============================================
                // Bold
                // ============================================
                case 'b':  // |b = Bold ON
                    cmd_ok = printer_set_bold();
                    break;

                case 'B':  // |B = Bold OFF
                    cmd_ok = printer_end_bold();
                    break;

                // ============================================
                // Alignment
                // ============================================
                case 'C':  // |C = Center align
                    cmd_ok = printer_set_center_align();
                    break;

                case 'l':  // |l = Left align
                    cmd_ok = printer_set_left_align();
                    break;

                case 'r':  // |r = Right align
                    cmd_ok = printer_set_right_align();
                    break;

                // ============================================
                // Text Size
                // ============================================
                case 'd':  // |d = Double height ON
                    cmd_ok = printer_set_double_height();
                    break;

                case 'D':  // |D = Double height OFF (normal)
                    cmd_ok = printer_set_normal_height();
                    break;

                // ============================================
                // Underline
                // ============================================
                case 'u':  // |u = Underline ON
                    cmd_ok = printer_set_underline();
                    break;

                case 'U':  // |U = Underline OFF
                    cmd_ok = printer_end_underline();
                    break;

                // ============================================
                // Inverse (white on black)
                // ============================================
                case 'i':  // |i = Inverse ON
                    cmd_ok = printer_set_inverse();
                    break;

                case 'I':  // |I = Inverse OFF
                    cmd_ok = printer_end_inverse();
                    break;

                // ============================================
                // Special Characters
                // ============================================
                case 't':  // |t = Tab
                    cmd_ok = printer_print_char('\t');
                    break;

                case '|':  // || = Literal pipe character
                    cmd_ok = printer_print_char('|');
                    break;

                case 'e':  // |e = Form feed
                    cmd_ok = send_command(FF, 0, 0, 2);
                    break;

                case 'p':  // |p = Space
                    cmd_ok = printer_print_char(' ');
                    break;

                // ============================================
                // Font Selection
                // ============================================
                case 'G':  // |G = Font A (large)
                    cmd_ok = printer_set_font_a();
                    break;

                case 'g':  // |g = Font B (small) - MOST COMMON IN APP
                    cmd_ok = printer_set_font_b();
                    break;

                // ============================================
                // Unknown escape - print both characters
                // ============================================
                default:
                    printer_print_char('|');
                    cmd_ok = printer_print_char(*p);
                    break;
            }

            // Check if command succeeded
            if (!cmd_ok) {
                ESP_LOGW(TAG, "Command failed at position %d", (int)(p - text));
                if (s_callback) s_callback(false);
                return false;
            }
        } else {
            // Regular character - just print it
            if (!printer_print_char(*p)) {
                ESP_LOGW(TAG, "Print char failed at position %d", (int)(p - text));
                if (s_callback) s_callback(false);
                return false;
            }
        }

        p++;  // Move to next character
    }

    ESP_LOGI(TAG, "Receipt printed successfully");
    if (s_callback) s_callback(true);
    return true;
}

/**
 * @brief Reprint the last receipt
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Reprints the previously printed receipt.
 * Useful when paper jammed or receipt was damaged.
 *
 * ============================================================================
 * COMMAND:
 * ============================================================================
 * {"command":"reprint_last_receipt"}
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if reprinted successfully
 * @return false if no receipt stored or error
 */
bool printer_reprint_last_receipt(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Reprint failed: not initialized");
        return false;
    }

    if (!s_receipt_stored || s_receipt_buffer[0] == '\0') {
        ESP_LOGW(TAG, "Reprint failed: no receipt stored");
        return false;
    }

    ESP_LOGI(TAG, "Reprinting last receipt");
    return printer_print_with_special_chars(s_receipt_buffer);
}

/**
 * @brief Store receipt data for later reprinting
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Saves receipt text so it can be reprinted later.
 * Called automatically by printer_print_with_special_chars().
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param data - Receipt text to store
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if stored successfully
 */
bool printer_store_receipt(const char *data)
{
    if (!data) {
        return false;
    }

    size_t len = strlen(data);

    // Truncate if too large
    if (len >= PRINTER_RECEIPT_BUF_SIZE) {
        ESP_LOGW(TAG, "Receipt too large (%d bytes), truncating to %d",
                 (int)len, PRINTER_RECEIPT_BUF_SIZE - 1);
        len = PRINTER_RECEIPT_BUF_SIZE - 1;
    }

    strncpy(s_receipt_buffer, data, len);
    s_receipt_buffer[len] = '\0';
    s_receipt_stored = true;

    ESP_LOGD(TAG, "Receipt stored (%d bytes)", (int)len);
    return true;
}

/**
 * @brief Clear stored receipt data
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Clears the stored receipt. Called when user wants to clear history.
 */
void printer_clear_receipt(void)
{
    memset(s_receipt_buffer, 0, sizeof(s_receipt_buffer));
    s_receipt_stored = false;
    ESP_LOGD(TAG, "Receipt cleared");
}

// ============================================================================
// Status Functions
// ============================================================================

/**
 * @brief Check if printer is ready
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Simple check if printer module is initialized.
 * Full status (paper out, cover open) would need printer RX.
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return true if initialized and ready
 * @return false if not initialized
 */
bool printer_check_status(void)
{
    // Note: Full status check would require reading from printer RX
    // Most thermal printers can report paper status, but requires bidirectional comm
    // For now, just return initialized state
    return s_initialized;
}

/**
 * @brief Print status information to console
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Debug function to display current printer status and configuration.
 */
void printer_print_status(void)
{
    printf("\n");
    printf("=========================================\n");
    printf("   Printer Status\n");
    printf("=========================================\n");
    printf("UART%d: TX=GPIO%d, RX=GPIO%d\n",
           s_config.uart_num, s_config.tx_pin, s_config.rx_pin);
    printf("Baud: %d, Parity: %s\n", s_config.baud_rate,
           s_config.parity == 0 ? "None" : (s_config.parity == 1 ? "Even" : "Odd"));
    printf("-----------------------------------------\n");
    printf("Initialized: %s\n", s_initialized ? "YES" : "NO");
    printf("Receipt Stored: %s\n", s_receipt_stored ? "YES" : "NO");
    if (s_receipt_stored) {
        printf("Receipt Size: %d bytes\n", (int)strlen(s_receipt_buffer));
    }
    printf("=========================================\n\n");
}

// ============================================================================
// Callbacks
// ============================================================================

/**
 * @brief Set callback for print completion
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Register a function to be called when printing completes.
 * Useful for updating UI or triggering next action.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param callback - Function to call: void callback(bool success)
 *                   success = true if print succeeded
 *                   success = false if print failed
 */
void printer_set_callback(printer_callback_t callback)
{
    s_callback = callback;
}
