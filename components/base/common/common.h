/**
 * @file common.h
 * @brief Common definitions and structures for NCLite ESP32-S3
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Firmware version (defined in CMakeLists.txt)
#ifndef NCLE_FIRMWARE_VERSION
#define NCLE_FIRMWARE_VERSION "2.0.0.1000"
#endif

// Buffer sizes (use menuconfig values if available, else defaults)
#ifndef WM_UART_BUF_SIZE
#define WM_UART_BUF_SIZE        256
#endif
#ifndef MA_UART_BUF_SIZE
#define MA_UART_BUF_SIZE        1024
#endif
#ifndef PRINTER_UART_BUF_SIZE
#define PRINTER_UART_BUF_SIZE   2048
#endif
#define CMD_BUF_SIZE            512
#define RESPONSE_BUF_SIZE       1024

// Device types
typedef enum {
    DEVICE_TYPE_MA = 0,     // Milk Analyzer
    DEVICE_TYPE_WM,         // Weighing Machine
    DEVICE_TYPE_PRINTER,    // Printer
    DEVICE_TYPE_MAX
} device_type_t;

// Parity settings
typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN = 1,
    PARITY_ODD = 2
} parity_t;

// Data bits
typedef enum {
    DATA_BITS_7 = 7,
    DATA_BITS_8 = 8
} data_bits_t;

// Stop bits
typedef enum {
    STOP_BITS_1 = 1,
    STOP_BITS_2 = 2
} stop_bits_t;

// UART configuration structure
typedef struct {
    uint32_t baud_rate;
    data_bits_t data_bits;
    stop_bits_t stop_bits;
    parity_t parity;
    bool stream_mode;       // Continuous streaming mode
    int model;              // Device model ID
} uart_port_config_t;

// Command message structure (received from BLE)
typedef struct {
    char data[CMD_BUF_SIZE];
    size_t len;
} cmd_message_t;

// Response message structure (sent to BLE)
typedef struct {
    char data[RESPONSE_BUF_SIZE];
    size_t len;
} response_message_t;

// Data message from UART (MA/WM)
typedef struct {
    device_type_t device;
    char data[MA_UART_BUF_SIZE];
    size_t len;
} uart_data_message_t;

// Print message structure
typedef struct {
    char data[PRINTER_UART_BUF_SIZE];
    size_t len;
} print_message_t;

// Status codes
#define STATUS_OK               0
#define STATUS_ERR              -1
#define STATUS_COMM_TIMEOUT     -2
#define STATUS_CONFIG_SUCCESS   0
#define STATUS_DATA_SUCCESS     0
#define STATUS_PRINTER_READY    0
#define STATUS_PRINTER_ERROR    -1
#define STATUS_MA_READY         0
#define STATUS_WM_READY         0
#define STATUS_NVS_FAIL         -100

// Global queues (extern declarations)
extern QueueHandle_t cmd_queue;
extern QueueHandle_t response_queue;

// Utility functions
void common_init(void);
const char* get_device_type_string(device_type_t type);
uint64_t get_unique_id(void);

// ============================================================================
// Cycle Timing Instrumentation
//
// Measures how long one collection cycle takes: MA reading -> weight ->
// print command -> paper out. There is no RTC on this board, so every
// value is elapsed time relative to TIMING_MA_RX (T=0), taken from
// esp_timer_get_time() (microsecond resolution since boot).
//
// Total time for the whole task = TIMING_TX_END - TIMING_MA_RX.
// ============================================================================

typedef enum {
    TIMING_T0_MA_FIRST = 0,  // T0: first byte of the MA frame arrived (T=0)
    TIMING_T1_MA_DONE,       // T1: MA data received completely / decoded
    TIMING_T2_APP_SENT,      // T2: reading handed to BLE for the app
    TIMING_T3_CMD_RX,        // T3: print data received back from the app
    TIMING_T4_PRINT_FIRST,   // T4: printing started - first byte to printer
    TIMING_T5_PRINT_DONE,    // T5: printing complete (last byte clocked out)

    // Not part of the T0-T5 sequence. Only stamped when a SEPARATE weighing
    // machine is used; analysers with an inbuilt WM carry the weight inside
    // the MA frame, so this stays -1 on those units.
    TIMING_WM_RX,
    TIMING_MARK_MAX
} timing_mark_t;

// What each interval tells you:
//   T1-T0  MA frame reception + framing wait (terminator detect or timeout)
//   T2-T1  decode + hand to BLE
//   T3-T2  app-side round trip - outside the connector, not ours to fix
//   T4-T3  JSON parse + printer reset
//   T5-T4  the print itself (baud-rate bound)
//   T5-T0  TOTAL task time

/**
 * @brief Stamp a timing marker with the current time
 * Stamping TIMING_T0_MA_FIRST clears the previous cycle and becomes the new T=0.
 */
void timing_mark(timing_mark_t mark);

/**
 * @brief Stamp a marker ONLY if it has not been stamped yet this cycle
 *
 * The app sends one receipt as several print_receipt commands. T3 and T4 must
 * record the FIRST of those chunks - if every chunk overwrote them, the print
 * phase would appear to last only as long as the final fragment (153ms rather
 * than the true 5164ms).
 *
 * T5 keeps using ordinary timing_mark() so the LAST chunk wins, which is what
 * "printing finished" means.
 */
void timing_mark_once(timing_mark_t mark);

/**
 * @brief Clear all markers and start a fresh cycle at this instant
 */
void timing_reset(void);

/**
 * @brief Elapsed milliseconds from T=0 to the given marker
 * @return Milliseconds, or -1 if that marker was never stamped this cycle
 */
int32_t timing_get_ms(timing_mark_t mark);

/**
 * @brief Uptime in ms at which the current cycle started (the T=0 reference)
 * Useful for telling two receipts apart when comparing paper against logs.
 */
int32_t timing_get_epoch_ms(void);

/**
 * @brief Build a compact one-line summary of the current cycle
 * Format: "T0=4471 MA0 WM2150 CMD2280 PRN7660 TOT9955"  (all values in ms)
 * @return Number of characters written
 */
int timing_format(char *buf, size_t size);

/**
 * @brief Enable/disable printing the timing lines on the receipt itself
 * Off by default - the extra characters cost real time at 1200 baud.
 * Console logging happens regardless of this setting.
 */
void timing_set_receipt_print(bool enable);
bool timing_get_receipt_print(void);

#ifdef __cplusplus
}
#endif

#endif // _COMMON_H_
