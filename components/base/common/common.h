/**
 * @file common.h
 * @brief Common definitions and structures for NCLite ESP32-S3
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdbool.h>
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

#ifdef __cplusplus
}
#endif

#endif // _COMMON_H_
