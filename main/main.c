/**
 * @file main.c
 * @brief NCLite ESP32-S3 - Main Application Entry Point
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * This is the main entry point for the NCLite ESP32-S3 dairy connector.
 * It initializes all hardware components and starts the system.
 *
 * ============================================================================
 * WHAT THIS FILE DOES:
 * ============================================================================
 * 1. Initializes NVS (Non-Volatile Storage) for saving configurations
 * 2. Initializes USB console for debugging and JSON commands
 * 3. Initializes BLE for mobile app communication
 * 4. Initializes WM (Weighing Machine) UART for weight data
 * 5. Initializes MA (Milk Analyzer) UART for milk quality data
 * 6. Initializes Printer UART for receipt printing
 * 7. Runs LED task for status indication
 * 8. Runs console task for processing JSON commands
 *
 * ============================================================================
 * DATA FLOW:
 * ============================================================================
 * - WM/MA data → UART → JSON → BLE → Mobile App
 * - Mobile App → BLE → JSON Command → cmd_parser → Action
 *
 * ============================================================================
 * RELATED FILES:
 * ============================================================================
 * - ble_spp.c     : BLE communication with mobile app
 * - cmd_parser.c  : JSON command processing
 * - wm_uart.c     : Weighing Machine UART driver
 * - ma_uart.c     : Milk Analyzer UART driver
 * - printer_uart.c: Thermal Printer driver
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

// Include WM module if enabled in menuconfig
#ifdef CONFIG_NCLE_WM_ENABLE
#include "wm_uart.h"
#endif

// Include MA module if enabled in menuconfig
#ifdef CONFIG_NCLE_MA_ENABLE
#include "ma_uart.h"
#endif

// Include Printer module if enabled in menuconfig
#ifdef CONFIG_NCLE_PRINTER_ENABLE
#include "printer_uart.h"
#endif

// Include command parser (always needed)
#include "cmd_parser.h"

// Include BLE SPP module if enabled in menuconfig
#ifdef CONFIG_BLE_SPP_ENABLED
#include "ble_spp.h"
#endif

// Include Battery monitor module if enabled in menuconfig
#ifdef CONFIG_NCLE_BATTERY_ENABLE
#include "battery.h"
#endif

// Include OTA module if enabled in menuconfig
#ifdef CONFIG_NCLE_OTA_ENABLE
#include "ota.h"
#endif

// Include GSM module if enabled in menuconfig
#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm.h"
#include "gsm_task.h"
#endif

#ifdef CONFIG_NCLE_SOFT_UART_LOOPBACK_TEST
#include "soft_uart_rmt.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#endif

#ifdef CONFIG_NCLE_WM_VALIDATOR_ENABLE
#include "wm_uart_validator.h"
#endif

static const char *TAG = "NCLE_MAIN";

// ============================================================================
// Soft UART Loopback Smoke Test (Phase 1 development only)
// ============================================================================
#ifdef CONFIG_NCLE_SOFT_UART_LOOPBACK_TEST
static volatile int s_loopback_rx_count = 0;
static char s_loopback_rx_buf[32];

static void loopback_byte_cb(const soft_uart_rmt_rx_t *rx, void *ctx)
{
    (void)ctx;
    if (s_loopback_rx_count < (int)sizeof(s_loopback_rx_buf) - 1) {
        s_loopback_rx_buf[s_loopback_rx_count++] = (char)rx->byte;
        s_loopback_rx_buf[s_loopback_rx_count] = '\0';
    }
}

static void bitbang_byte(int gpio, uint8_t b, int bit_us)
{
    gpio_set_level(gpio, 0); esp_rom_delay_us(bit_us);            // start
    for (int i = 0; i < 8; i++) {
        gpio_set_level(gpio, (b >> i) & 1); esp_rom_delay_us(bit_us);
    }
    gpio_set_level(gpio, 1); esp_rom_delay_us(bit_us);            // stop
}

static void run_loopback_test(void)
{
    const int tx_gpio = CONFIG_NCLE_SOFT_UART_LOOPBACK_TX_GPIO;
    const int rx_gpio = CONFIG_NCLE_SOFT_UART_LOOPBACK_RX_GPIO;
    const int baud = 9600;
    const int bit_us = 1000000 / baud; // 104

    ESP_LOGI(TAG, "LOOPBACK: jumper required from GPIO %d -> GPIO %d", tx_gpio, rx_gpio);

    // Configure TX as output
    gpio_config_t cfg_tx = {
        .pin_bit_mask = 1ULL << tx_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_tx);
    gpio_set_level(tx_gpio, 1); // idle high

    // ------- PRE-TEST WIRE CHECK (plain GPIO) -------
    // Temporarily drive TX and read RX as GPIO to verify the jumper wire.
    gpio_config_t cfg_rx_in = {
        .pin_bit_mask = 1ULL << rx_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_rx_in);

    gpio_set_level(tx_gpio, 0);
    esp_rom_delay_us(2000);
    int read_when_low  = gpio_get_level(rx_gpio);
    gpio_set_level(tx_gpio, 1);
    esp_rom_delay_us(2000);
    int read_when_high = gpio_get_level(rx_gpio);
    ESP_LOGI(TAG, "LOOPBACK: jumper check: TX=LOW -> RX=%d, TX=HIGH -> RX=%d  (want 0, 1)",
             read_when_low, read_when_high);

    if (read_when_low != 0 || read_when_high != 1) {
        ESP_LOGE(TAG, "LOOPBACK: *** WIRE-CHECK FAIL *** jumper not conducting between GPIO %d and GPIO %d. Aborting test.",
                 tx_gpio, rx_gpio);
        gpio_reset_pin(tx_gpio);
        gpio_reset_pin(rx_gpio);
        return;
    }
    ESP_LOGI(TAG, "LOOPBACK: jumper check OK");

    // Release RX pin so the RMT driver can claim it
    gpio_reset_pin(rx_gpio);

    soft_uart_rmt_config_t ucfg = {
        .gpio_num = rx_gpio, .baud_rate = baud, .data_bits = 8,
        .stop_bits = 1, .parity = 0,
    };
    soft_uart_rmt_handle_t h = NULL;
    ESP_ERROR_CHECK(soft_uart_rmt_init(&ucfg, &h));
    ESP_ERROR_CHECK(soft_uart_rmt_register_byte_cb(h, loopback_byte_cb, NULL));
    ESP_ERROR_CHECK(soft_uart_rmt_start(h));

    vTaskDelay(pdMS_TO_TICKS(100));

    const char *msg = "HELLO\n";
    ESP_LOGI(TAG, "LOOPBACK: sending '%s' on GPIO %d", msg, tx_gpio);
    for (const char *p = msg; *p; p++) {
        bitbang_byte(tx_gpio, (uint8_t)*p, bit_us);
        esp_rom_delay_us(bit_us);
    }

    /* Give plenty of time for partial-rx batches + diagnostic logs to appear */
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "LOOPBACK: received %d bytes: '%s'",
             s_loopback_rx_count, s_loopback_rx_buf);

    if (s_loopback_rx_count == (int)strlen(msg) &&
        strncmp(s_loopback_rx_buf, msg, strlen(msg)) == 0) {
        ESP_LOGI(TAG, "LOOPBACK: *** PASS ***");
    } else {
        ESP_LOGE(TAG, "LOOPBACK: *** FAIL *** (expected '%s')", msg);
    }

    soft_uart_rmt_stop(h);
    soft_uart_rmt_deinit(h);
    gpio_reset_pin(tx_gpio);
}
#endif

// ============================================================================
// Status LED Configuration (from menuconfig)
// ============================================================================
#define STATUS_LED_GPIO     CONFIG_NCLE_STATUS_LED_GPIO
#ifdef CONFIG_NCLE_STATUS_LED_ACTIVE_HIGH
#define LED_ON_LEVEL        1   // LED turns ON when GPIO is HIGH
#define LED_OFF_LEVEL       0   // LED turns OFF when GPIO is LOW
#else
#define LED_ON_LEVEL        0   // LED turns ON when GPIO is LOW (inverted)
#define LED_OFF_LEVEL       1   // LED turns OFF when GPIO is HIGH
#endif

// LED activity flag - set by WM/MA modules when data is received
static volatile bool s_led_activity_flag = false;

// ============================================================================
// BLE Data Callback Wrappers
// ============================================================================
#ifdef CONFIG_BLE_SPP_ENABLED

/**
 * @brief Callback wrapper for WM (Weighing Machine) data to BLE
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * When WM receives weight data from the weighing machine, this function
 * sends it to the mobile app via BLE.
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * WM callback has signature: void (*)(const char*, size_t)
 * BLE send function needs:   void (*)(const char*, unsigned int)
 * This wrapper converts size_t to unsigned int.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param json_data - JSON string containing weight data
 *                    Example: {"device":"wm","data":"0017.31Kg","model":9001 }
 * @param len       - Length of the JSON string in bytes
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Sends data to mobile app via BLE notify characteristic.
 * Mobile app receives JSON and displays weight to user.
 *
 * ============================================================================
 * DATA FLOW:
 * ============================================================================
 * Weighing Machine → UART → wm_uart.c → this callback → BLE → Mobile App
 */
static void wm_ble_data_callback(const char *json_data, size_t len)
{
    ble_spp_output_callback(json_data, (unsigned int)len);
}

#ifdef CONFIG_NCLE_GSM_ENABLE
/**
 * @brief Status callback wired to BLE TX
 * Fires once per gsm_task poll interval (default 10s).
 */
static void gsm_ble_status_callback(const gsm_status_t *s, void *ctx)
{
    (void)ctx;
    static char buf[512];
    /* Reports each connectivity stage separately (alive / sim / registered /
     * data) so the app can show WHICH one failed, rather than one ambiguous
     * "connected". "data" true means an IP is actually assigned.
     *
     * "problem" and "action" carry plain-language text the app can display
     * directly - the user cannot act on rssi=4 or net=3, but can act on
     * "Check the antenna" or "Call the carrier". */
    int n = snprintf(buf, sizeof(buf),
        "{\"device\":\"gsm\",\"alive\":%d,\"sim\":\"%s\",\"registered\":%d,"
        "\"data\":%d,\"rssi\":%d,\"bars\":%d,\"net\":%d,\"iccid\":\"%s\","
        "\"fault\":\"%s\",\"problem\":\"%s\",\"action\":\"%s\"}\n",
        s->alive, gsm_sim_status_str(s->sim_status), s->registered,
        s->data_up, s->rssi, s->bars, (int)s->net_status, s->iccid,
        gsm_fault_name(s->fault),
        gsm_fault_problem(s->fault),
        gsm_fault_action(s->fault));
    if (n > 0 && n < (int)sizeof(buf)) {
        ble_spp_output_callback(buf, (unsigned int)n);
    }
}
#endif

/**
 * @brief Callback wrapper for MA (Milk Analyzer) data to BLE
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * When MA receives milk analysis data (receipt with FAT, SNF, etc.),
 * this function sends it to the mobile app via BLE.
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * MA callback has signature: void (*)(const char*, int)
 * BLE send function needs:   void (*)(const char*, unsigned int)
 * This wrapper converts int to unsigned int.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param json_data - JSON string containing milk analysis receipt
 *                    Example: {"device":"ma","data":"FAT: 4.5%\nSNF: 8.6%...","model":1002 }
 * @param len       - Length of the JSON string in bytes
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Sends data to mobile app via BLE notify characteristic.
 * Mobile app receives JSON and displays milk analysis to user.
 *
 * ============================================================================
 * DATA FLOW:
 * ============================================================================
 * Milk Analyzer → UART → ma_uart.c → this callback → BLE → Mobile App
 */
static void ma_ble_data_callback(const char *json_data, int len)
{
    ble_spp_output_callback(json_data, (unsigned int)len);
}
#endif

// ============================================================================
// LED Activity Callback
// ============================================================================

/**
 * @brief Callback function triggered when any device has activity
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Sets a flag that tells the LED task to blink rapidly.
 * This provides visual feedback when WM/MA receives data.
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * User can see LED blinking and know that data is being received.
 * - If LED is doing slow heartbeat → waiting for data
 * - If LED is blinking rapidly → data was just received
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Sets s_led_activity_flag = true
 * LED task will see this flag and blink rapidly.
 *
 * ============================================================================
 * CALLED BY:
 * ============================================================================
 * - wm_uart.c : When weight data is received
 * - ma_uart.c : When milk analysis data is received
 */
static void on_activity(void)
{
    s_led_activity_flag = true;
}

// ============================================================================
// Status LED Task
// ============================================================================

/**
 * @brief FreeRTOS task that controls the status LED
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Provides visual indication of system status:
 * - IDLE: Slow heartbeat blink (system is running, waiting for data)
 * - ACTIVITY: Rapid blink (data is being received/processed)
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * User can see at a glance if the device is working and receiving data.
 * No need to check logs or connect to see status.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param arg - FreeRTOS task argument (not used, always NULL)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Controls GPIO pin connected to LED:
 * - Sets GPIO HIGH or LOW to turn LED on/off
 *
 * ============================================================================
 * LED BEHAVIOR:
 * ============================================================================
 * IDLE (waiting for data):
 *   - LED ON for 100ms
 *   - LED OFF for 900ms
 *   - Repeat forever (slow heartbeat)
 *
 * ACTIVITY (data received):
 *   - LED ON for 100ms, OFF for 100ms
 *   - Repeat 5 times (rapid blink)
 *   - Then return to idle heartbeat
 *
 * ============================================================================
 * RUNS:
 * ============================================================================
 * Forever in background as FreeRTOS task (never returns)
 */
static void led_task(void *arg)
{
    // Initialize LED GPIO pin as output
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "LED: GPIO%d (active %s)", STATUS_LED_GPIO, LED_ON_LEVEL ? "HIGH" : "LOW");

    while (1) {
        if (s_led_activity_flag) {
            // Activity detected - clear flag and blink rapidly 5 times
            s_led_activity_flag = false;

            for (int i = 0; i < 5; i++) {
                gpio_set_level(STATUS_LED_GPIO, LED_ON_LEVEL);   // LED ON
                vTaskDelay(pdMS_TO_TICKS(100));                   // Wait 100ms
                gpio_set_level(STATUS_LED_GPIO, LED_OFF_LEVEL);  // LED OFF
                vTaskDelay(pdMS_TO_TICKS(100));                   // Wait 100ms
            }
        } else {
            // Idle - slow heartbeat
            gpio_set_level(STATUS_LED_GPIO, LED_ON_LEVEL);   // LED ON
            vTaskDelay(pdMS_TO_TICKS(100));                   // Wait 100ms (short on)
            gpio_set_level(STATUS_LED_GPIO, LED_OFF_LEVEL);  // LED OFF
            vTaskDelay(pdMS_TO_TICKS(900));                   // Wait 900ms (long off)
        }
    }
}

// ============================================================================
// USB Console Functions
// ============================================================================

#define CMD_BUF_SIZE 1024  // Maximum command buffer size (bytes)

/**
 * @brief Initialize USB-CDC console for serial communication
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Sets up USB-CDC (USB Serial) for:
 * - Debug output (ESP_LOGI, printf, etc.) to PC terminal
 * - Receiving JSON commands from PC terminal for testing
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * Allows developers to:
 * - See debug logs on PC without special hardware
 * - Send test commands without needing mobile app
 * - Debug issues during development
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * USB-CDC driver installed and ready for use.
 * After this, printf() and ESP_LOGI() output will appear on PC terminal.
 *
 * ============================================================================
 * BUFFER SIZES:
 * ============================================================================
 * - RX buffer: 512 bytes (commands from PC)
 * - TX buffer: 512 bytes (logs to PC)
 */
static void init_usb_console(void)
{
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = 512,
        .tx_buffer_size = 512,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}

/**
 * @brief FreeRTOS task that processes JSON commands from USB console
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * Reads JSON commands typed by user in PC terminal and processes them.
 * Uses same interface as BLE - allows testing without mobile app.
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * - Developers can test commands from PC terminal
 * - Same JSON format as mobile app uses
 * - Useful for debugging and factory testing
 * - No need to connect mobile app for basic testing
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * @param arg - FreeRTOS task argument (not used, always NULL)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * Processes commands and sends JSON responses to USB console.
 * Responses also sent to BLE if connected.
 *
 * ============================================================================
 * COMMAND FORMAT:
 * ============================================================================
 * {"command":"command_name","param":"value"}#
 *
 * Rules:
 * - Must be valid JSON starting with '{'
 * - Terminated by '#' or newline ('\r' or '\n')
 * - Maximum length: 1024 bytes
 *
 * ============================================================================
 * EXAMPLE:
 * ============================================================================
 * User types in terminal:
 *   {"command":"get_firmware_version"}#
 *
 * System responds:
 *   {"response_message":"get_firmware_version","status_code":0,"data":"2.0.0.1000"}
 *
 * ============================================================================
 * RUNS:
 * ============================================================================
 * Forever in background as FreeRTOS task (never returns)
 */
static void console_task(void *arg)
{
    static char cmd_buf[CMD_BUF_SIZE];  // Buffer to accumulate command characters
    int cmd_idx = 0;                     // Current position in buffer
    uint8_t rx_byte;                     // Single received byte

    ESP_LOGI(TAG, "USB Console ready (JSON commands only)");
    ESP_LOGI(TAG, "Format: {\"command\":\"...\"}#");

    while (1) {
        // Read one byte from USB with 100ms timeout
        // Returns 1 if byte received, 0 if timeout
        int len = usb_serial_jtag_read_bytes(&rx_byte, 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            char ch = (char)rx_byte;

            // Check for command terminator: '#' or newline
            if (ch == '#' || ch == '\r' || ch == '\n') {
                if (cmd_idx > 0) {
                    // Null-terminate the command string
                    cmd_buf[cmd_idx] = '\0';

                    // Only process if it looks like JSON (starts with '{')
                    // Ignores garbage characters or partial commands
                    if (cmd_buf[0] == '{') {
                        parse_and_process_commands(cmd_buf, cmd_idx);
                    }
                }
                // Reset buffer for next command
                cmd_idx = 0;
                memset(cmd_buf, 0, sizeof(cmd_buf));
                continue;
            }

            // Accumulate character into buffer (if space available)
            if (cmd_idx < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_idx++] = ch;
            }
        }
    }
}

// ============================================================================
// NVS Initialization
// ============================================================================

/**
 * @brief Initialize Non-Volatile Storage (NVS)
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * NVS is flash storage that survives power cycles. Used to store:
 * - WM configuration (baud rate, model, stream mode)
 * - MA configuration (baud rate, model)
 * - Printer configuration (baud rate, parity)
 *
 * ============================================================================
 * WHY THIS FUNCTION EXISTS:
 * ============================================================================
 * User configures device once via mobile app, settings are saved to NVS.
 * On next power-on, device loads saved settings from NVS automatically.
 * User doesn't need to reconfigure after every power cycle.
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * @return ESP_OK if NVS initialized successfully
 * @return Error code if initialization failed
 *
 * ============================================================================
 * ERROR HANDLING:
 * ============================================================================
 * - First boot (no NVS data): Initializes empty NVS
 * - Corrupted NVS: Erases and reinitializes (loses saved data)
 * - NVS version mismatch: Erases and reinitializes (after firmware update)
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    // Handle corrupted or version mismatch - erase and retry
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased (corrupted or version mismatch)");
        ESP_ERROR_CHECK(nvs_flash_erase());  // Erase all NVS data
        ret = nvs_flash_init();               // Try again with clean NVS
    }
    return ret;
}

// ============================================================================
// Main Entry Point
// ============================================================================

/**
 * @brief Main application entry point - called by ESP-IDF after boot
 *
 * ============================================================================
 * PURPOSE:
 * ============================================================================
 * This is where everything starts. Initializes all components in the
 * correct order and starts all background tasks.
 *
 * ============================================================================
 * WHY INITIALIZATION ORDER MATTERS:
 * ============================================================================
 * 1. NVS first       - Other components need to load saved configs
 * 2. USB console     - For debug output during remaining init
 * 3. Command parser  - Needed before BLE can process commands
 * 4. WM/MA/Printer   - Hardware interfaces (order doesn't matter)
 * 5. BLE last        - Registers callbacks to already-initialized modules
 *
 * ============================================================================
 * INPUT:
 * ============================================================================
 * None (called by ESP-IDF, not by user code)
 *
 * ============================================================================
 * OUTPUT:
 * ============================================================================
 * None (runs forever via FreeRTOS tasks, never returns)
 *
 * ============================================================================
 * INITIALIZATION SEQUENCE:
 * ============================================================================
 * 1. Wait 2 seconds   - USB enumeration (PC needs time to detect device)
 * 2. Initialize NVS   - Persistent storage for configurations
 * 3. Initialize USB   - Debug output console
 * 4. Initialize parser- JSON command processing
 * 5. Start LED task   - Status indication (heartbeat)
 * 6. Initialize WM    - Weighing machine UART
 * 7. Initialize MA    - Milk analyzer UART
 * 8. Initialize Printer- Thermal printer UART
 * 9. Initialize BLE   - Mobile app connection
 * 10. Start console   - USB command processing
 *
 * ============================================================================
 * AFTER INIT COMPLETES:
 * ============================================================================
 * System is ready. LED shows heartbeat.
 * Waiting for:
 * - BLE connection from mobile app (NitaraCLE4)
 * - WM/MA data from connected dairy devices
 * - USB commands from PC terminal (for debugging)
 */
void app_main(void)
{
    // ========================================================================
    // Step 1: Wait for USB-CDC enumeration
    // ========================================================================
    // USB-CDC takes time for host PC to enumerate the device.
    // If we start logging immediately, first messages are lost.
    // Wait 2 seconds so user has time to open terminal and see all logs.
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   NCLite ESP32-S3 Starting...");
    ESP_LOGI(TAG, "========================================");

    // ========================================================================
    // Step 2: Initialize NVS (Non-Volatile Storage)
    // ========================================================================
    // Must be first - other components load their saved configs from NVS
    ESP_ERROR_CHECK(init_nvs());
    ESP_LOGI(TAG, "NVS initialized");

    // ========================================================================
    // Step 3: Initialize USB console
    // ========================================================================
    // Enables debug logging and command input from PC terminal
    init_usb_console();
    ESP_LOGI(TAG, "USB console initialized");

    // ========================================================================
    // Step 4: Initialize command parser
    // ========================================================================
    // JSON command processor - needed before BLE or console can work
    cmd_parser_init();
    ESP_LOGI(TAG, "Command parser initialized");

    // Brief delay before hardware initialization
    vTaskDelay(pdMS_TO_TICKS(500));

    // ========================================================================
    // Step 5: Start LED task
    // ========================================================================
    // Status LED for visual indication (heartbeat when idle, rapid blink on activity)
    xTaskCreate(led_task, "led", 4096, NULL, 1, NULL);

    // ========================================================================
    // Step 5b: Soft UART loopback smoke test (Phase 1 dev only, Kconfig-gated)
    // ========================================================================
#ifdef CONFIG_NCLE_SOFT_UART_LOOPBACK_TEST
    run_loopback_test();
#endif

    // ========================================================================
    // Step 6: Initialize WM (Weighing Machine) module
    // ========================================================================
#ifdef CONFIG_NCLE_WM_ENABLE
    ESP_LOGI(TAG, "WM Module: Enabled");
    ESP_ERROR_CHECK(wm_uart_init());             // Initialize UART driver
    wm_uart_set_activity_callback(on_activity);  // LED blinks when data received
#ifdef CONFIG_NCLE_WM_VALIDATOR_ENABLE
    /* Keep WM running continuously so the validator has a steady byte stream
     * to compare. Without STREAM mode the driver auto-stops after the first
     * stable reading and HW byte counts freeze. */
    wm_uart_set_stream_mode(WM_MODE_STREAM);
    wm_uart_set_sample_rate(1); /* log every sample to maximise HW byte flow */
    ESP_LOGI(TAG, "WM Module: STREAM mode (for validator)");
#endif
    ESP_ERROR_CHECK(wm_uart_start());            // Start receive task
#else
    ESP_LOGI(TAG, "WM Module: Disabled");
#endif

#ifdef CONFIG_NCLE_WM_VALIDATOR_ENABLE
    {
        wm_uart_validator_config_t vcfg = {
            .shared_gpio         = CONFIG_NCLE_WM_UART_RX_PIN,
            .rmt_channel         = CONFIG_NCLE_WM_VALIDATOR_RMT_CHANNEL,
            .baud_rate           = CONFIG_NCLE_WM_UART_BAUD_RATE,
            .report_interval_ms  = CONFIG_NCLE_WM_VALIDATOR_REPORT_INTERVAL_MS,
            .packet_quiet_ms     = CONFIG_NCLE_WM_VALIDATOR_PACKET_QUIET_MS,
        };
        esp_err_t verr = wm_uart_validator_init(&vcfg);
        if (verr == ESP_OK) verr = wm_uart_validator_start();
        if (verr != ESP_OK) {
            ESP_LOGE(TAG, "WM validator failed to start: %d", verr);
        } else {
            ESP_LOGI(TAG, "WM validator running (GPIO %d, RMT ch %d, baud %d)",
                     CONFIG_NCLE_WM_UART_RX_PIN,
                     CONFIG_NCLE_WM_VALIDATOR_RMT_CHANNEL,
                     CONFIG_NCLE_WM_UART_BAUD_RATE);
        }
    }
#endif

#ifdef CONFIG_NCLE_WM_VALIDATOR_RUN_SWEEP
    {
        static const int sweep_bauds[] = {9600, 19200, 38400, 57600, 115200};
        vTaskDelay(pdMS_TO_TICKS(3000)); /* settle before starting */
        wm_uart_validator_run_baud_sweep(sweep_bauds,
                                         sizeof(sweep_bauds)/sizeof(sweep_bauds[0]),
                                         CONFIG_NCLE_WM_VALIDATOR_SWEEP_SECONDS_PER_STEP);
    }
#endif

    // ========================================================================
    // Step 7: Initialize MA (Milk Analyzer) module
    // ========================================================================
#ifdef CONFIG_NCLE_MA_ENABLE
    ESP_LOGI(TAG, "MA Module: Enabled");
    ESP_ERROR_CHECK(ma_uart_init());             // Initialize UART driver
    ma_uart_set_activity_callback(on_activity);  // LED blinks when data received
    ESP_ERROR_CHECK(ma_uart_start());            // Start receive task
#else
    ESP_LOGI(TAG, "MA Module: Disabled");
#endif

    // ========================================================================
    // Step 8: Initialize Printer module
    // ========================================================================
#ifdef CONFIG_NCLE_PRINTER_ENABLE
    ESP_LOGI(TAG, "Printer Module: Enabled");
    ESP_ERROR_CHECK(printer_uart_init());        // Initialize UART driver
    // Note: Printer doesn't need start() - it sends when commanded
#else
    ESP_LOGI(TAG, "Printer Module: Disabled");
#endif

    // ========================================================================
    // Step 8.5: Initialize Battery monitor module
    // ========================================================================
#ifdef CONFIG_NCLE_BATTERY_ENABLE
    ESP_LOGI(TAG, "Battery Module: Enabled (GPIO%d)", BATTERY_ADC_GPIO);
    esp_err_t batt_ret = battery_init();
    if (batt_ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery: %lumV (%d%%)",
                 battery_get_voltage_mv(), battery_get_percentage());
    } else {
        ESP_LOGW(TAG, "Battery Module: Init failed (0x%x)", batt_ret);
    }
#else
    ESP_LOGI(TAG, "Battery Module: Disabled");
#endif

    // ========================================================================
    // Step 8.6: Initialize OTA update module
    // ========================================================================
#ifdef CONFIG_NCLE_OTA_ENABLE
    ESP_LOGI(TAG, "OTA Module: Enabled");
    esp_err_t ota_ret = ota_init();
    if (ota_ret == ESP_OK) {
        // Mark current firmware as valid (prevents rollback)
        ota_mark_valid();
        char part_label[16];
        uint32_t part_addr, part_size;
        ota_get_running_partition_info(part_label, &part_addr, &part_size);
        ESP_LOGI(TAG, "OTA: Running from %s (0x%lx, %luKB)",
                 part_label, part_addr, part_size / 1024);
    } else {
        ESP_LOGW(TAG, "OTA Module: Init failed (0x%x)", ota_ret);
    }
#else
    ESP_LOGI(TAG, "OTA Module: Disabled");
#endif

    // ========================================================================
    // Step 9: Initialize BLE SPP module
    // ========================================================================
#ifdef CONFIG_BLE_SPP_ENABLED
    ESP_LOGI(TAG, "BLE SPP Module: Initializing...");
    esp_err_t ble_ret = ble_spp_init();
    if (ble_ret == ESP_OK) {
        ESP_LOGI(TAG, "BLE SPP Module: Enabled (Device: %s)", ble_spp_get_device_name());

        // Register callback so command responses go to BLE (and USB console)
        cmd_parser_register_output_callback(ble_spp_output_callback);

        // Register WM data callback - sends weight data to mobile app via BLE
#ifdef CONFIG_NCLE_WM_ENABLE
        wm_uart_set_data_callback(wm_ble_data_callback);
        ESP_LOGI(TAG, "WM -> BLE callback registered");
#endif

        // Register MA data callback - sends milk data to mobile app via BLE
#ifdef CONFIG_NCLE_MA_ENABLE
        ma_uart_set_data_callback(ma_ble_data_callback);
        ESP_LOGI(TAG, "MA -> BLE callback registered");
#endif

        // Register GSM status callback - sends signal/registration to mobile app via BLE
#ifdef CONFIG_NCLE_GSM_ENABLE
        gsm_task_set_status_callback(gsm_ble_status_callback, NULL);
        ESP_LOGI(TAG, "GSM -> BLE callback registered");

#ifdef CONFIG_NCLE_GSM_AUTO_START
        /* Field devices have nobody to send gsm_enable. The task retries with
         * back-off if the modem is absent or slow to power up, and all of its
         * work happens on its own task - WM, MA, printer and BLE keep running
         * normally throughout, including during the ~2 min worst case when
         * every APN fails.
         *
         * A user who sent gsm_disable stays disabled: auto-start must not
         * silently undo a deliberate choice at the next power cycle. */
        if (!gsm_is_enabled_pref()) {
            ESP_LOGI(TAG, "GSM Module: disabled by user - send 'gsm_enable' to turn on");
        } else if (gsm_task_start() == ESP_OK) {
            ESP_LOGI(TAG, "GSM Module: auto-start enabled, connecting...");
        } else {
            ESP_LOGW(TAG, "GSM Module: auto-start failed to launch task");
        }
#else
        ESP_LOGI(TAG, "GSM Module: waiting for 'gsm_enable' command");
#endif
        // gsm_task_start() is NOT called here; waits for BLE command {"command":"gsm_enable"}
#endif
    } else {
        ESP_LOGE(TAG, "BLE SPP Module: Failed to initialize (0x%x)", ble_ret);
    }
#else
    ESP_LOGI(TAG, "BLE SPP Module: Disabled");
#endif

    // ========================================================================
    // Step 10: Start console task
    // ========================================================================
    // USB command processing - allows testing via PC terminal
    xTaskCreate(console_task, "console", 8192, NULL, 3, NULL);

    // ========================================================================
    // Initialization complete
    // ========================================================================
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   System Ready - JSON commands only");
    ESP_LOGI(TAG, "========================================");

    // Log memory status (useful for debugging memory leaks)
    ESP_LOGI(TAG, "Memory: Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Memory: Min free heap: %lu bytes", esp_get_minimum_free_heap_size());

    // app_main() returns here, but FreeRTOS tasks continue running forever
}
